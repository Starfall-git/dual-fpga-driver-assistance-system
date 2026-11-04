`timescale 1ns/1ps

/*
统一延迟架构

主链统一到 35 拍；两级居中形态学再延迟 2×(LINE_CLOCKS+3) 拍。
720p 总延迟 3335 拍。窗口中心的空间偏移与时钟延迟分别对齐。
- 原图: RGB 直通并补齐延迟，不强制灰度化或高斯滤波
- Gaussian/Bypass: Gray 3 + Window 2 + Gaussian/Bypass 4 = 9 拍
- Median/Bypass: 再 Window 2 + Median/Bypass 3 = 14 拍
- Sobel: 再 Window 2 + Sobel 3 = 19 拍，补 16 拍
- Canny: Median/Bypass 14 + Canny 21 = 35 拍

Gaussian 开关或 Canny 使能时，执行同一次 Gaussian；二者同时开不重复。
Gaussian 在 Median 之前；Canny 优先于 Sobel。
边缘模式仅原图/滤波图、Sobel、Sobel→膨胀→侵蚀、Canny。
锐化/红色叠加使用所选模式的边缘掩码；不做边缘检测时两者无效果。
锐化内部复用 Gaussian 差分，不强制模糊普通输出。
*/
module video_processing #(
    parameter IMAGE_WIDTH   = 1280,
    parameter RUNTIME_FEATURES = 0,
    /* independent optional shape stage; legacy pipelines bypass it. */
    parameter SHAPE_SUPPORT = 0,
    parameter LANE_SUPPORT = 0,
    /* optional final compositor, no changes to edge arithmetic. */
    parameter SPLIT_SUPPORT = 0,
    parameter LINE_CLOCKS   = 1650,
    /* AR0135 monochrome=0; RGB color camera=1. */
    parameter ENABLE_GRAY = 0,
    /* reference center-luminance threshold with UART floor. */
    parameter SOBEL_ADAPTIVE = 1,
    
    parameter SOBEL_BORDER_GUARD = 8,
    
    parameter GRAYSCALE_OUTPUT = 0,
    parameter VS_ACTIVE     = 1'b0,
    parameter NEON_FRAMES_PER_COLOR = 60
)(
    input  wire         clk,
    input  wire         rst_n,
    // {edge_color[2:0],threshold[11:0],alpha[1:0],grayscale,reserved_median,overlay,sharpen}
    input wire [34:0] FEATURE_CONFIG,
    input wire [55:0] LANE_CONFIG, LANE_GUIDE, LANE_MODEL, LANE_RECT,
    input wire [8:0] FATIGUE_STATE,
    /* 新增独立中值滤波开关，复位默认关闭，由像素域邮箱驱动。 */
    input  wire         ENABLE_SOBEL, ENABLE_MEDIAN, BINARY_OUTPUT, ENABLE_CANNY, ENABLE_GAUSSIAN,
    /* 、15：保留用户改为变量的阈值接口；输入须已同步到本clk域。 */
    input  wire [2:0]   SOBEL_MORPH_MODE,
    input  wire [11:0]  SOBEL_THRESHOLD,
    input  wire [11:0]  CANNY_LOW_THRESHOLD,
    input  wire [11:0]  CANNY_HIGH_THRESHOLD,
    input  wire [23:0]  rgb_i,
    input  wire         hs_i,
    input  wire         vs_i,
    input  wire         de_i,

    output wire [23:0]  rgb_o,
    output wire         hs_o,
    output wire         vs_o,
    output wire         de_o,
    output wire [15:0] assist_status_o
);

    localparam ALIGN_DEPTH       = 35;
    localparam MEDIAN_LATENCY    = 14;
    localparam GAUSSIAN_LATENCY  = 9;
    localparam SOBEL_LATENCY     = 19;

    //==========================================================
    // 帧边界锁存 mode，并按像素流水同步到各选择点。
    //==========================================================
    reg [8:0] mode_reg;
    reg [8:0] mode_pipe [0:ALIGN_DEPTH-1];
    reg       vs_d;
    wire      frame_start;
    wire [8:0] mode_next;
    integer mode_index;

    assign frame_start = (vs_i == VS_ACTIVE) && (vs_d != VS_ACTIVE);
    assign mode_next   = (vs_i == VS_ACTIVE) ? {SOBEL_MORPH_MODE, BINARY_OUTPUT, ENABLE_CANNY, ENABLE_SOBEL,
                                        ENABLE_GAUSSIAN, ENABLE_MEDIAN, (ENABLE_GRAY != 0)}
                                 : mode_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mode_reg <= 9'b000100000;
            vs_d     <= ~VS_ACTIVE;
            for (mode_index=0; mode_index<ALIGN_DEPTH; mode_index=mode_index+1)
                mode_pipe[mode_index] <= 9'b000100000;
        end else begin
            vs_d <= vs_i;
            mode_reg <= mode_next;
            mode_pipe[0] <= mode_next;
            for (mode_index=1; mode_index<ALIGN_DEPTH; mode_index=mode_index+1)
                mode_pipe[mode_index] <= mode_pipe[mode_index-1];
        end
    end

    wire median_sel_med_stage    = mode_pipe[MEDIAN_LATENCY-1][1];
    wire gaussian_req_gauss_stage = mode_pipe[GAUSSIAN_LATENCY-1][2] |
                                    mode_pipe[GAUSSIAN_LATENCY-1][4];
    wire [8:0] mode_out = mode_pipe[ALIGN_DEPTH-1];
    reg [34:0] feature_pipe[0:ALIGN_DEPTH-1];
    wire median_en_out   = mode_out[1];
    wire sobel_en_out    = mode_out[3];
    wire binary_en_out = feature_pipe[ALIGN_DEPTH-1][34];
    wire canny_en_out    = mode_out[4] && !binary_en_out;
    wire edge_white_sobel_stage = mode_pipe[SOBEL_LATENCY-1][5];
    wire sobel_eff_out   = (sobel_en_out || binary_en_out) && !canny_en_out;

    // Features and thresholds are sampled during VS blank and travel with pixels.
    wire [34:0] feature_input=RUNTIME_FEATURES ? FEATURE_CONFIG : {3'd0,3'd0,12'd32,2'd2,(GRAYSCALE_OUTPUT!=0),3'b100};
    reg [34:0] feature_reg;
    integer feature_index;
    wire [34:0] feature_next=(vs_i==VS_ACTIVE) ? feature_input : feature_reg;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            feature_reg<={3'd0,3'd0,12'd32,2'd2,(GRAYSCALE_OUTPUT!=0),3'b100};
            for(feature_index=0;feature_index<ALIGN_DEPTH;feature_index=feature_index+1)
                feature_pipe[feature_index]<={3'd0,3'd0,12'd32,2'd2,(GRAYSCALE_OUTPUT!=0),3'b100};
        end else begin
            feature_reg<=feature_next;feature_pipe[0]<=feature_next;
            for(feature_index=1;feature_index<ALIGN_DEPTH;feature_index=feature_index+1)
                feature_pipe[feature_index]<=feature_pipe[feature_index-1];
        end
    end

    // Advance on input frame boundaries; delay the selected color with its pixels.
    localparam NEON_COUNT_BITS=$clog2(NEON_FRAMES_PER_COLOR+1);
    reg [NEON_COUNT_BITS-1:0] neon_frames;
    reg [2:0] neon_color;
    reg neon_was_on;
    wire neon_on=(feature_input[20:18]==3'd6) && feature_input[1];
    wire [2:0] neon_next_color=(!neon_on || !neon_was_on) ? 3'd0 :
        (neon_frames==NEON_FRAMES_PER_COLOR-1) ?
            (neon_color==3'd5 ? 3'd0 : neon_color+3'd1) : neon_color;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin neon_frames<=0;neon_color<=0;neon_was_on<=0;end
        else begin
            neon_was_on<=neon_on;
            if(!neon_on || !neon_was_on) begin neon_frames<=0;neon_color<=0;end
            else if(frame_start) begin
                neon_color<=neon_next_color;
                if(neon_frames==NEON_FRAMES_PER_COLOR-1) neon_frames<=0;
                else neon_frames<=neon_frames+1'b1;
            end
        end
    end
    wire [2:0] neon_aligned;
    video_stream_delay #(.WIDTH(3),.DELAY(ALIGN_DEPTH)) u_neon_align(
        .clk(clk),.rst_n(rst_n),.data_i(frame_start ? neon_next_color : neon_color),.data_o(neon_aligned));

    //==========================================================
    // Stage1: RGB → Gray (3 拍)
    //==========================================================
        
    wire [7:0] gray;
    wire gray_hs, gray_vs, gray_de;

    video_rgb2gray #(.ENABLE_GRAY(ENABLE_GRAY), .VS_ACTIVE(VS_ACTIVE)) u_gray (
        .clk(clk), .rst_n(rst_n), .rgb_i(rgb_i),
        .hs_i(hs_i), .vs_i(vs_i), .de_i(de_i),
        .gray_o(gray), .hs_o(gray_hs), .vs_o(gray_vs), .de_o(gray_de)
    );

    // Raw RGB is not passed through any filter. Keep optional RGB-to-gray support.
    wire [23:0] original_aligned;
    wire [7:0] gray_aligned;
    video_stream_delay #(.WIDTH(24),.DELAY(ALIGN_DEPTH)) u_original_align(
        .clk(clk),.rst_n(rst_n),.data_i(de_i ? rgb_i : 24'd0),.data_o(original_aligned));
    video_stream_delay #(.WIDTH(8),.DELAY(ALIGN_DEPTH-3)) u_gray_align(
        .clk(clk),.rst_n(rst_n),.data_i(gray_de ? gray : 8'd0),.data_o(gray_aligned));

    //==========================================================
    // Stage 2: Gaussian Window (始终实例化)
    //==========================================================

    wire [71:0] gauss_win_pixels;
    wire        gauss_win_hs, gauss_win_vs, gauss_win_de, gauss_win_valid;

    video_window3x3 #(.IMAGE_WIDTH(IMAGE_WIDTH), .VS_ACTIVE(VS_ACTIVE))
    u_gauss_win (
        .clk(clk), .rst_n(rst_n),
        .gray_i(gray),
        .pixel_valid_i(gray_de),
        .hs_i(gray_hs), .vs_i(gray_vs), .de_i(gray_de),
        .pixels_o(gauss_win_pixels),
        .hs_o(gauss_win_hs), .vs_o(gauss_win_vs), .de_o(gauss_win_de),
        .window_valid_o(gauss_win_valid)
    );

    wire [7:0] gauss_p11 = gauss_win_pixels[39:32];

    //==========================================================
    // Stage 3: Gaussian (4 拍) OR Bypass (p11 延迟 4 拍对齐)
    //==========================================================
    wire [7:0]  gauss_gray;
    wire        gauss_hs, gauss_vs, gauss_de, gauss_valid;

    video_gaussian3x3 #(.VS_ACTIVE(VS_ACTIVE)) u_gaussian (
        .clk(clk), .rst_n(rst_n),
        .pixels_i(gauss_win_pixels),
        .hs_i(gauss_win_hs), .vs_i(gauss_win_vs), .de_i(gauss_win_de),
        .window_valid_i(gauss_win_valid),
        .gray_o(gauss_gray),
        .hs_o(gauss_hs), .vs_o(gauss_vs), .de_o(gauss_de),
        .window_valid_o(gauss_valid)
    );

    reg [7:0] gauss_bp0, gauss_bp1, gauss_bp2, gauss_bp3;
    reg       gauss_bp_hs0, gauss_bp_hs1, gauss_bp_hs2, gauss_bp_hs3;
    reg       gauss_bp_vs0, gauss_bp_vs1, gauss_bp_vs2, gauss_bp_vs3;
    reg       gauss_bp_de0, gauss_bp_de1, gauss_bp_de2, gauss_bp_de3;
    reg       gauss_bp_vld0, gauss_bp_vld1, gauss_bp_vld2, gauss_bp_vld3;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gauss_bp0<=0; gauss_bp1<=0; gauss_bp2<=0; gauss_bp3<=0;
            gauss_bp_hs0<=1'b1; gauss_bp_hs1<=1'b1; gauss_bp_hs2<=1'b1; gauss_bp_hs3<=1'b1;
            gauss_bp_vs0<=VS_ACTIVE; gauss_bp_vs1<=VS_ACTIVE; gauss_bp_vs2<=VS_ACTIVE; gauss_bp_vs3<=VS_ACTIVE;
            gauss_bp_de0<=0; gauss_bp_de1<=0; gauss_bp_de2<=0; gauss_bp_de3<=0;
            gauss_bp_vld0<=0; gauss_bp_vld1<=0; gauss_bp_vld2<=0; gauss_bp_vld3<=0;
        end else begin
            gauss_bp0 <= gauss_p11; gauss_bp1 <= gauss_bp0;
            gauss_bp2 <= gauss_bp1; gauss_bp3 <= gauss_bp2;
            gauss_bp_hs0 <= gauss_win_hs; gauss_bp_hs1 <= gauss_bp_hs0;
            gauss_bp_hs2 <= gauss_bp_hs1; gauss_bp_hs3 <= gauss_bp_hs2;
            gauss_bp_vs0 <= gauss_win_vs; gauss_bp_vs1 <= gauss_bp_vs0;
            gauss_bp_vs2 <= gauss_bp_vs1; gauss_bp_vs3 <= gauss_bp_vs2;
            gauss_bp_de0 <= gauss_win_de; gauss_bp_de1 <= gauss_bp_de0;
            gauss_bp_de2 <= gauss_bp_de1; gauss_bp_de3 <= gauss_bp_de2;
            gauss_bp_vld0 <= gauss_win_valid; gauss_bp_vld1 <= gauss_bp_vld0;
            gauss_bp_vld2 <= gauss_bp_vld1; gauss_bp_vld3 <= gauss_bp_vld2;
        end
    end

    wire [7:0] gauss_eff_gray = gaussian_req_gauss_stage ? gauss_gray : gauss_bp3;
    wire       gauss_eff_hs   = gaussian_req_gauss_stage ? gauss_hs   : gauss_bp_hs3;
    wire       gauss_eff_vs   = gaussian_req_gauss_stage ? gauss_vs   : gauss_bp_vs3;
    wire       gauss_eff_de   = gaussian_req_gauss_stage ? gauss_de   : gauss_bp_de3;
    wire       gauss_eff_vld  = gaussian_req_gauss_stage ? gauss_valid : gauss_bp_vld3;

    // Gaussian result aligned to 35 clocks.
    reg [23:0] gaussian_delay [0:ALIGN_DEPTH-GAUSSIAN_LATENCY-1];
    integer gaussian_index;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (gaussian_index=0; gaussian_index<ALIGN_DEPTH-GAUSSIAN_LATENCY; gaussian_index=gaussian_index+1)
                gaussian_delay[gaussian_index] <= 0;
        end else begin
            gaussian_delay[0] <= (gauss_eff_de && gauss_eff_vld) ? {gauss_eff_gray, gauss_eff_gray, gauss_eff_gray} : 24'd0;
            for (gaussian_index=1; gaussian_index<ALIGN_DEPTH-GAUSSIAN_LATENCY; gaussian_index=gaussian_index+1)
                gaussian_delay[gaussian_index] <= gaussian_delay[gaussian_index-1];
        end
    end

    //==========================================================
    // Stage 4: Median Window (始终实例化)
    //==========================================================
    wire [71:0] med_win_pixels;
    wire        med_win_hs, med_win_vs, med_win_de, med_win_valid;

    video_window3x3 #(.IMAGE_WIDTH(IMAGE_WIDTH), .VS_ACTIVE(VS_ACTIVE))
    u_med_win (
        .clk(clk), .rst_n(rst_n),
        .gray_i(gauss_eff_gray),
        .pixel_valid_i(gauss_eff_vld),
        .hs_i(gauss_eff_hs), .vs_i(gauss_eff_vs), .de_i(gauss_eff_de),
        .pixels_o(med_win_pixels),
        .hs_o(med_win_hs), .vs_o(med_win_vs), .de_o(med_win_de),
        .window_valid_o(med_win_valid)
    );

    wire [7:0] med_p11 = med_win_pixels[39:32];

    //==========================================================
    // Stage 5: Median (3 拍) OR Bypass (p11 延迟 3 拍对齐)
    //==========================================================
    wire [23:0] med_rgb, med_full_rgb;
    wire        med_hs, med_vs, med_de, med_valid;

    video_median3x3 #(.VS_ACTIVE(VS_ACTIVE)) u_median (
        .clk(clk), .rst_n(rst_n),
        .pixels_i(med_win_pixels),
        .hs_i(med_win_hs), .vs_i(med_win_vs), .de_i(med_win_de),
        .window_valid_i(med_win_valid),
        .rgb_o(med_rgb), .median_rgb_o(med_full_rgb),
        .hs_o(med_hs), .vs_o(med_vs), .de_o(med_de),
        .pixel_valid_o(med_valid)
    );

    reg [7:0] med_bp0, med_bp1, med_bp2;
    reg       med_bp_hs0, med_bp_hs1, med_bp_hs2;
    reg       med_bp_vs0, med_bp_vs1, med_bp_vs2;
    reg       med_bp_de0, med_bp_de1, med_bp_de2;
    reg       med_bp_vld0, med_bp_vld1, med_bp_vld2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            med_bp0<=0; med_bp1<=0; med_bp2<=0;
            med_bp_hs0<=1'b1; med_bp_hs1<=1'b1; med_bp_hs2<=1'b1;
            med_bp_vs0<=VS_ACTIVE; med_bp_vs1<=VS_ACTIVE; med_bp_vs2<=VS_ACTIVE;
            med_bp_de0<=0; med_bp_de1<=0; med_bp_de2<=0;
            med_bp_vld0<=0; med_bp_vld1<=0; med_bp_vld2<=0;
        end else begin
            med_bp0 <= med_p11; med_bp1 <= med_bp0; med_bp2 <= med_bp1;
            med_bp_hs0 <= med_win_hs; med_bp_hs1 <= med_bp_hs0; med_bp_hs2 <= med_bp_hs1;
            med_bp_vs0 <= med_win_vs; med_bp_vs1 <= med_bp_vs0; med_bp_vs2 <= med_bp_vs1;
            med_bp_de0 <= med_win_de; med_bp_de1 <= med_bp_de0; med_bp_de2 <= med_bp_de1;
            med_bp_vld0 <= med_win_valid; med_bp_vld1 <= med_bp_vld0; med_bp_vld2 <= med_bp_vld1;
        end
    end

    wire [23:0] med_bypass = {med_bp2, med_bp2, med_bp2};
    wire [23:0] med_out    = median_sel_med_stage ? med_rgb : med_bypass;
    wire        med_out_hs  = median_sel_med_stage ? med_hs  : med_bp_hs2;
    wire        med_out_vs  = median_sel_med_stage ? med_vs  : med_bp_vs2;
    wire        med_out_de  = median_sel_med_stage ? med_de  : med_bp_de2;
    wire        med_out_vld = median_sel_med_stage ? med_valid : med_bp_vld2;

    // Median result aligned to 35 clocks.
    reg [23:0] median_delay [0:ALIGN_DEPTH-MEDIAN_LATENCY-1];
    integer median_index;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (median_index=0; median_index<ALIGN_DEPTH-MEDIAN_LATENCY; median_index=median_index+1)
                median_delay[median_index] <= 0;
        end else begin
            median_delay[0] <= med_out_de ? med_out : 24'd0;
            for (median_index=1; median_index<ALIGN_DEPTH-MEDIAN_LATENCY; median_index=median_index+1)
                median_delay[median_index] <= median_delay[median_index-1];
        end
    end

    //==========================================================
    // Stage 6: Canny (接 med_out_* 时序)
    //==========================================================
    wire [71:0] edge_win_pixels;
    wire edge_win_hs, edge_win_vs, edge_win_de, edge_win_valid;
    wire [23:0] canny_rgb;
    wire [7:0] canny_strength;
    wire        canny_hs, canny_vs, canny_de;
    wire        canny_threshold_error_unused;

    video_canny_core #(.IMAGE_WIDTH(IMAGE_WIDTH), .VS_ACTIVE(VS_ACTIVE), .SOBEL_GUARD(SOBEL_BORDER_GUARD))
    u_canny (
        .clk(clk), .rst_n(rst_n),
        .LOW_THRESHOLD(CANNY_LOW_THRESHOLD),
        .HIGH_THRESHOLD(CANNY_HIGH_THRESHOLD),
        /* CANNY-MULTI-combined Canny+Sobel mode selects new core. */
        .improved_i(mode_pipe[MEDIAN_LATENCY-1][4] && mode_pipe[MEDIAN_LATENCY-1][3]),
        .gray_i(med_out[7:0]),
        .hs_i(med_out_hs), .vs_i(med_out_vs),
        .de_i(med_out_de), .valid_i(med_out_vld),
        .rgb_o(canny_rgb), .strength_o(canny_strength),
        .hs_o(canny_hs), .vs_o(canny_vs), .de_o(canny_de),
        .threshold_error(canny_threshold_error_unused),
        .shared_window_pixels_o(edge_win_pixels),.shared_window_hs_o(edge_win_hs),
        .shared_window_vs_o(edge_win_vs),.shared_window_de_o(edge_win_de),.shared_window_valid_o(edge_win_valid)
    );

    //==========================================================
    // Stage 6b: Sobel (普通 Sobel 路径, 用 edge_win)
    //==========================================================
    wire [23:0] sobel_rgb;
    wire [7:0] sobel_strength;
    wire        sobel_hs, sobel_vs, sobel_de, sobel_edge;

    sobeledge_8d_window #(.GRAYSCALE_OUTPUT(0),
                          .ADAPTIVE_THRESHOLD(SOBEL_ADAPTIVE),
                          .VS_ACTIVE(VS_ACTIVE))
    u_sobel (
        .clk(clk), .rst_n(rst_n),
        .BINARY_OUTPUT(edge_white_sobel_stage),
        .THRESHOLD(SOBEL_THRESHOLD),
        .pixels_i(edge_win_pixels),
        .hs_i(edge_win_hs), .vs_i(edge_win_vs), .de_i(edge_win_de),
        .window_valid_i(edge_win_valid),
        .rgb_o(sobel_rgb), .strength_o(sobel_strength), .magnitude_o(),
        .hs_o(sobel_hs), .vs_o(sobel_vs), .de_o(sobel_de), .edge_o(sobel_edge)
    );

    
    wire [7:0] sobel2_strength;
    wire sobel2_edge;
    video_sobel2 u_sobel2(.clk(clk),.rst_n(rst_n),.pixels_i(edge_win_pixels),
        .THRESHOLD(SOBEL_THRESHOLD),.hs_i(edge_win_hs),.vs_i(edge_win_vs),
        .de_i(edge_win_de),.window_valid_i(edge_win_valid),
        .strength_o(sobel2_strength),.edge_o(sobel2_edge),.hs_o(),.vs_o(),.de_o());
    wire select_sobel2=feature_pipe[SOBEL_LATENCY-1][21];
    wire [7:0] selected_strength=select_sobel2 ? sobel2_strength : sobel_strength;
    wire selected_sobel_edge=select_sobel2 ? sobel2_edge : sobel_edge;
    wire [23:0] sobel_selected_rgb=select_sobel2 ?
        {3{edge_white_sobel_stage ? sobel2_strength : ~sobel2_strength}} :
        (feature_pipe[SOBEL_LATENCY-1][3] ?
        {3{edge_white_sobel_stage ? sobel_strength : ~sobel_strength}} : sobel_rgb);

    // Sobel result aligned to 35 clocks.
    reg [23:0] sobel_delay [0:ALIGN_DEPTH-SOBEL_LATENCY-1];
    integer sobel_index;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (sobel_index=0; sobel_index<ALIGN_DEPTH-SOBEL_LATENCY; sobel_index=sobel_index+1)
                sobel_delay[sobel_index] <= 0;
        end else begin
            sobel_delay[0] <= sobel_de ? sobel_selected_rgb : 24'd0;
            for (sobel_index=1; sobel_index<ALIGN_DEPTH-SOBEL_LATENCY; sobel_index=sobel_index+1)
                sobel_delay[sobel_index] <= sobel_delay[sobel_index-1];
        end
    end

    //==========================================================
    // 最终 MUX: RGB 与统一到 35 拍的同步信号同拍输出。
    //==========================================================
    // Display polarity never changes the foreground mask used by sharpening/overlay.
    wire [7:0] canny_pixel=feature_pipe[ALIGN_DEPTH-1][3] ? canny_strength : canny_rgb[7:0];
    wire [7:0] canny_display=mode_out[5] ? canny_pixel : ~canny_pixel;
    
    wire binary_bit=gray_aligned>=feature_pipe[ALIGN_DEPTH-1][33:26];
    wire [7:0] binary_display=(binary_bit==mode_out[5]) ? 8'hff : 8'h00;
    wire [23:0] base_rgb = binary_en_out ? {3{binary_display}} : canny_en_out    ? {3{canny_display}} :
                   sobel_eff_out   ? sobel_delay[ALIGN_DEPTH-SOBEL_LATENCY-1] :
                   median_en_out   ? median_delay[ALIGN_DEPTH-MEDIAN_LATENCY-1] :
                   mode_out[2]     ? gaussian_delay[ALIGN_DEPTH-GAUSSIAN_LATENCY-1] :
                                     (ENABLE_GRAY ? {3{gray_aligned}} : original_aligned);

    // Morphology is applied only AFTER Sobel. All existing paths retain
    // their pixel values and receive the same two centered line delays.
    localparam MORPH_DELAY=2*(LINE_CLOCKS+3);
    wire edge_aligned;
    wire [7:0] strength_aligned,morph_strength;
    video_stream_delay #(.WIDTH(8),.DELAY(ALIGN_DEPTH-SOBEL_LATENCY)) u_strength_align(
        .clk(clk),.rst_n(rst_n),.data_i(selected_strength),.data_o(strength_aligned));
    video_stream_delay #(.WIDTH(1),.DELAY(ALIGN_DEPTH-SOBEL_LATENCY)) u_edge_align(
        .clk(clk),.rst_n(rst_n),.data_i(selected_sobel_edge),.data_o(edge_aligned));
    wire morph_bit, morph_hs, morph_vs, morph_de;
    video_sobel_morph #(.LINE_CLOCKS(LINE_CLOCKS),.VS_ACTIVE(VS_ACTIVE)) u_sobel_morph(
        .clk(clk),.rst_n(rst_n),.bit_i(binary_en_out ? binary_bit : (edge_aligned && sobel_eff_out)),
        .strength_i(binary_en_out ? {8{binary_bit}} : canny_en_out ? canny_strength : strength_aligned),.strength_o(morph_strength),.hs_i(canny_hs),.vs_i(canny_vs),
        .de_i(canny_de),.mode_i(canny_en_out && !binary_en_out ? 3'd0 : mode_out[8:6]),.bit_o(morph_bit),
        .hs_o(morph_hs),.vs_o(morph_vs),.de_o(morph_de));
    // A separate fixed-threshold gradient on the raw window gates sharpening.
    // Reuse the optional Gaussian arithmetic, even when its normal-output switch is off.
    wire [9:0] sharp_mag_8,sharp_mag_9;
    wire [7:0] sharp_center_9;
    wire [23:0] raw_center_9;
    sobeledge_8d_window #(.ADAPTIVE_THRESHOLD(0),.VS_ACTIVE(VS_ACTIVE)) u_sharp_gradient(
        .clk(clk),.rst_n(rst_n),.BINARY_OUTPUT(1'b1),.THRESHOLD(12'd0),
        .pixels_i(gauss_win_pixels),.hs_i(gauss_win_hs),.vs_i(gauss_win_vs),
        .de_i(gauss_win_de),.window_valid_i(gauss_win_valid),
        .rgb_o(),.hs_o(),.vs_o(),.de_o(),.edge_o(),.strength_o(),.magnitude_o(sharp_mag_8));
    video_stream_delay #(.WIDTH(10),.DELAY(1)) u_sharp_mag_align(
        .clk(clk),.rst_n(rst_n),.data_i(sharp_mag_8),.data_o(sharp_mag_9));
    video_stream_delay #(.WIDTH(8),.DELAY(4)) u_sharp_center_align(
        .clk(clk),.rst_n(rst_n),.data_i(gauss_win_pixels[39:32]),.data_o(sharp_center_9));
    // A causal window's center is one line and one pixel behind its DE coordinate.
    video_stream_delay #(.WIDTH(24),.DELAY(9+LINE_CLOCKS+1)) u_raw_center_align(
        .clk(clk),.rst_n(rst_n),.data_i(de_i ? rgb_i : 24'd0),.data_o(raw_center_9));
    wire [23:0] raw_rgb_12;
    wire signed [9:0] sharp_delta_12;
    video_sharpen u_sharpen(.clk(clk),.rst_n(rst_n),.rgb_i(raw_center_9),
        .center_i(sharp_center_9),.blur_i(gauss_gray),.magnitude_i(sharp_mag_9),
        .threshold_i(feature_pipe[8][17:6]),.alpha_i(feature_pipe[8][5:4]),
        .enable_i(feature_pipe[8][0]),.valid_i(gauss_valid && gauss_de),
        .rgb_o(),.original_o(raw_rgb_12),.delta_o(sharp_delta_12));
    wire [33:0] background_base,background_median,background_sobel,background_canny;
    video_stream_delay #(.WIDTH(34),.DELAY(ALIGN_DEPTH-12)) u_background_base(
        .clk(clk),.rst_n(rst_n),.data_i({sharp_delta_12,raw_rgb_12}),.data_o(background_base));
    // Sobel has three causal windows total; Canny has five. Morphology is centered
    // and shifts data and sync together, adding no additional spatial offset.
    /* expose the middle tap without duplicating raw line storage. */
    video_stream_delay #(.WIDTH(34),.DELAY(LINE_CLOCKS+1)) u_background_median(
        .clk(clk),.rst_n(rst_n),.data_i(background_base),.data_o(background_median));
    video_stream_delay #(.WIDTH(34),.DELAY(LINE_CLOCKS+1)) u_background_sobel(
        .clk(clk),.rst_n(rst_n),.data_i(background_median),.data_o(background_sobel));
    video_stream_delay #(.WIDTH(34),.DELAY(2*(LINE_CLOCKS+1))) u_background_canny(
        .clk(clk),.rst_n(rst_n),.data_i(background_sobel),.data_o(background_canny));
    wire [23:0] raw_selected=median_en_out ? background_median[23:0] :
        mode_out[2] ? background_base[23:0] : original_aligned;
    /* Non-edge background was previously unused; carry split raw through it. */
    wire [33:0] background_selected=binary_en_out ? {10'd0,original_aligned} : canny_en_out ? background_canny : sobel_eff_out ? background_sobel :
        {background_base[33:24],raw_selected};
    // Delay both candidates: the chosen algorithm's mask arrives after morphology.
    wire any_edge=(sobel_eff_out || canny_en_out) && !binary_en_out;
    wire [2:0] edge_color=feature_pipe[ALIGN_DEPTH-1][20:18]==3'd6 ?
        neon_aligned : feature_pipe[ALIGN_DEPTH-1][20:18];
    // Every filtered or edge output is grayscale. The only RGB base path
    // is the unfiltered image already carried as the raw background.
    wire base_is_color=!binary_en_out && !canny_en_out && !sobel_eff_out &&
        !median_en_out && !mode_out[2] && !ENABLE_GRAY;
    wire [55:0] delayed;
    video_stream_delay #(.WIDTH(56),.DELAY(MORPH_DELAY),
        .RESET_VALUE({51'd0,1'b0,1'b1,1'b1,VS_ACTIVE,1'b0})) u_output_align(
        .clk(clk),.rst_n(rst_n),
        .data_i({edge_color,feature_pipe[ALIGN_DEPTH-1][3] && !binary_en_out,feature_pipe[ALIGN_DEPTH-1][1] && any_edge,
                 feature_pipe[ALIGN_DEPTH-1][0] && any_edge,background_selected,canny_en_out,canny_rgb[0],
                 base_is_color,base_rgb[7:0],sobel_eff_out && mode_out[8:6]!=0,mode_out[5],canny_hs,canny_vs,canny_de}),
        .data_o(delayed));
    
    wire [1:0] palette_delayed;
    video_stream_delay #(.WIDTH(2),.DELAY(MORPH_DELAY)) u_palette_align(
        .clk(clk),.rst_n(rst_n),
        .data_i({feature_pipe[ALIGN_DEPTH-1][22],any_edge}),
        .data_o(palette_delayed));
    wire [23:0] strength_color;
    video_edge_palette u_palette(.strength_i(morph_strength), .rgb_o(strength_color));
    wire palette_active=palette_delayed[1] && palette_delayed[0];
    wire [23:0] palette_display=delayed[3] ? strength_color : ~strength_color;
    wire [7:0] morph_pixel=(morph_bit==delayed[3]) ? 8'hff : 8'h00;
    wire [7:0] morph_gray=delayed[3] ? morph_strength : ~morph_strength;
    wire [23:0] normal_rgb=delayed[4] ? {3{delayed[52] ? morph_gray : morph_pixel}} : (delayed[13] ? delayed[39:16] : {3{delayed[12:5]}});
    wire selected_edge=delayed[15] ? delayed[14] : morph_bit;
    function [7:0] background_add_clip;
        input [7:0] pixel;
        input signed [10:0] delta;
        reg signed [11:0] sum;
        begin
            sum=$signed({1'b0,pixel})+delta;
            background_add_clip=sum<0 ? 8'd0 : sum>255 ? 8'd255 : sum[7:0];
        end
    endfunction
    wire signed [10:0] aligned_delta={delayed[49],delayed[49:40]};
    wire [23:0] background_rgb=(delayed[50] && selected_edge) ?
        {background_add_clip(delayed[39:32],aligned_delta),background_add_clip(delayed[31:24],aligned_delta),background_add_clip(delayed[23:16],aligned_delta)} : delayed[39:16];
    reg [23:0] overlay_color;
    always @* case(delayed[55:53])
        3'd1: overlay_color=24'hff8000; // orange
        3'd2: overlay_color=24'hffff00; // yellow
        3'd3: overlay_color=24'h00ff00; // green
        3'd4: overlay_color=24'h0000ff; // blue
        3'd5: overlay_color=24'h8000ff; // purple
        default: overlay_color=24'hff0000; // red, including reset/reserved
    endcase
    wire [23:0] legacy_rgb=delayed[0] ?
        (delayed[51] ? (selected_edge ? (palette_active ? strength_color : overlay_color) : background_rgb) :
         delayed[50] ? background_rgb : palette_active ? palette_display : normal_rgb) : 24'd0;
    
    wire [23:0] processed_rgb;
    wire processed_hs,processed_vs,processed_de;
    generate if(SHAPE_SUPPORT) begin: g_shape
        wire shape_enable;
        video_stream_delay #(.WIDTH(1),.DELAY(MORPH_DELAY)) u_shape_control(
            .clk(clk),.rst_n(rst_n),.data_i(feature_pipe[ALIGN_DEPTH-1][23] && !binary_en_out),.data_o(shape_enable));
        video_shape_recognizer #(.IMAGE_WIDTH(IMAGE_WIDTH),.VS_ACTIVE(VS_ACTIVE)) u_shapes(
            .clk(clk),.rst_n(rst_n),.enable_i(shape_enable),.edge_i(selected_edge),
            .rgb_i(legacy_rgb),.hs_i(delayed[2]),.vs_i(delayed[1]),.de_i(delayed[0]),
            .rgb_o(processed_rgb),.hs_o(processed_hs),.vs_o(processed_vs),.de_o(processed_de));
    end else begin: g_no_shape
        assign processed_rgb=legacy_rgb;
        assign processed_hs=delayed[2]; assign processed_vs=delayed[1]; assign processed_de=delayed[0];
    end endgenerate

    // Road overlay precedes the line-buffered split compositor. This reuses
    // the existing raw and gradient streams rather than adding line stores.
    wire [23:0] lane_raw, lane_legacy, lane_curve, lane_processed, split_raw;
    wire [7:0] lane_edge;
    wire lane_hs,lane_vs,lane_de;
    video_stream_delay #(.WIDTH(24),.DELAY(SHAPE_SUPPORT ? 1 : 0)) u_lane_raw(
        .clk(clk),.rst_n(rst_n),.data_i(delayed[39:16]),.data_o(lane_raw));
    video_stream_delay #(.WIDTH(8),.DELAY(SHAPE_SUPPORT ? 1 : 0)) u_lane_edge(
        .clk(clk),.rst_n(rst_n),.data_i(morph_strength),.data_o(lane_edge));
    generate if(LANE_SUPPORT) begin: g_lane
    video_lane_overlay #(.IMAGE_WIDTH(IMAGE_WIDTH),.IMAGE_HEIGHT(720),.VS_ACTIVE(VS_ACTIVE)) u_lane(
        .clk(clk),.rst_n(rst_n),.config_i(LANE_CONFIG),.guide_i(LANE_GUIDE),
        .original_i(lane_raw),.processed_i(processed_rgb),
        .hs_i(processed_hs),.vs_i(processed_vs),.de_i(processed_de),
        .rgb_o(lane_legacy),.hs_o(lane_hs),.vs_o(lane_vs),.de_o(lane_de));
    video_lane_curve #(.IMAGE_WIDTH(IMAGE_WIDTH),.IMAGE_HEIGHT(720),.VS_ACTIVE(VS_ACTIVE)) u_curve(
        .clk(clk),.rst_n(rst_n),.config_i(LANE_CONFIG),.guide_i(LANE_GUIDE),
        .model_i(LANE_MODEL),.rect_i(LANE_RECT),.edge_i(lane_edge),
        .original_i(lane_raw),.processed_i(processed_rgb),.fatigue_state_async_i(FATIGUE_STATE),
        .hs_i(processed_hs),.vs_i(processed_vs),.de_i(processed_de),
        .rgb_o(lane_curve),.hs_o(),.vs_o(),.de_o(),.assist_status_o(assist_status_o));
    assign lane_processed=LANE_MODEL[0] ? lane_curve : lane_legacy;
    video_stream_delay #(.WIDTH(24),.DELAY(1)) u_lane_split_raw(
        .clk(clk),.rst_n(rst_n),.data_i(lane_raw),.data_o(split_raw));
    end else begin: g_no_lane
        assign assist_status_o=16'h000e;
        assign lane_processed=processed_rgb;
        assign lane_hs=processed_hs;assign lane_vs=processed_vs;assign lane_de=processed_de;
        assign split_raw=lane_raw;
    end endgenerate

    
    generate if(SPLIT_SUPPORT) begin:g_split
        wire [23:0] raw_final=split_raw;
        wire [1:0] split_mode;
        video_stream_delay #(.WIDTH(2),.DELAY(MORPH_DELAY+(SHAPE_SUPPORT ? 1 : 0)+(LANE_SUPPORT ? 1 : 0))) u_split_mode(
            .clk(clk),.rst_n(rst_n),.data_i(feature_pipe[ALIGN_DEPTH-1][25:24]),.data_o(split_mode));
        video_split_display #(.IMAGE_WIDTH(IMAGE_WIDTH),.LINE_CLOCKS(LINE_CLOCKS),.VS_ACTIVE(VS_ACTIVE)) u_split(
            .clk(clk),.rst_n(rst_n),.mode_i(split_mode),.processed_i(lane_processed),.raw_i(split_raw),
            .hs_i(lane_hs),.vs_i(lane_vs),.de_i(lane_de),
            .rgb_o(rgb_o),.hs_o(hs_o),.vs_o(vs_o),.de_o(de_o));
    end else begin:g_no_split
        assign rgb_o=lane_processed;
        assign hs_o=lane_hs;assign vs_o=lane_vs;assign de_o=lane_de;
    end endgenerate
endmodule
