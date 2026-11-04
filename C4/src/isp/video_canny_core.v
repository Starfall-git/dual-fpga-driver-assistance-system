`timescale 1ns/1ps
/*
 * Canny edge detector for a one-pixel-per-clock video stream.
 *
 * Timing rules:
 * - The first 3x3 window is built by video_window3x3, so the input
 *   coordinate and window_valid signal have one common definition.
 * - The magnitude and hysteresis windows use the same coordinate/validity
 *   convention locally.
 * - The result is delayed by CANNY_TAIL_DELAY clocks so the complete Canny
 *   path has a fixed 21-clock latency from gray_i to rgb_o.  The enclosing
 *   video_processing module uses that fixed latency to align all paths.
 */
module video_canny_core #(
    parameter IMAGE_WIDTH = 1280,
    parameter VS_ACTIVE   = 1'b0,
    parameter SOBEL_GUARD = 8
)(
    input  wire        clk,
    input  wire        rst_n,

    input  wire [11:0] LOW_THRESHOLD,
    input  wire [11:0] HIGH_THRESHOLD,
    /* CANNY-MULTI-choose directional maximum; 0 retains legacy Canny. */
    input  wire        improved_i,

    input  wire [7:0]  gray_i,
    input  wire        hs_i,
    input  wire        vs_i,
    input  wire        de_i,
    input  wire        valid_i,

    output reg  [23:0] rgb_o,
    output reg  [7:0] strength_o,
    output reg         hs_o,
    output reg         vs_o,
    output reg         de_o,

    output wire        threshold_error,
    output wire [71:0] shared_window_pixels_o,
    output wire shared_window_hs_o, shared_window_vs_o, shared_window_de_o, shared_window_valid_o
);

    localparam ANGLE_0   = 2'd0;
    localparam ANGLE_45  = 2'd1;
    localparam ANGLE_90  = 2'd2;
    localparam ANGLE_135 = 2'd3;

    // Window(2) + gradient(1) + magnitude(1) + magnitude window(2) +
    // NMS(1) + threshold(1) + state window(2) + decision(1) + core(1)
    // + tail registers(8) + output register(1) = 21 clocks.
    localparam CANNY_TAIL_DELAY = 8;

    //==========================================================
    // Threshold latch: thresholds remain constant for one frame.
    //==========================================================
    reg [11:0] lo_latch, hi_latch;
    reg        vs_d;
    wire       vs_rise = (vs_i == VS_ACTIVE) && (vs_d != VS_ACTIVE);

    wire [11:0] s_lo = (LOW_THRESHOLD > HIGH_THRESHOLD) ? HIGH_THRESHOLD : LOW_THRESHOLD;
    wire [11:0] s_hi = (LOW_THRESHOLD > HIGH_THRESHOLD) ? LOW_THRESHOLD  : HIGH_THRESHOLD;

    assign threshold_error = (LOW_THRESHOLD > HIGH_THRESHOLD) ||
                             (LOW_THRESHOLD == 0 && HIGH_THRESHOLD == 0);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lo_latch <= 12'd20;
            hi_latch <= 12'd60;
            vs_d     <= ~VS_ACTIVE;
        end else begin
            vs_d <= vs_i;
            if (vs_i == VS_ACTIVE) begin
                lo_latch <= (s_lo == 0) ? 12'd1 : s_lo;
                hi_latch <= (s_hi == 0) ? 12'd2 : s_hi;
            end
        end
    end

    //==========================================================
    // Input 3x3 window
    //==========================================================
    wire [71:0] in_win_pixels;
    wire        in_win_hs, in_win_vs, in_win_de, in_win_valid;

    video_window3x3 #(
        .IMAGE_WIDTH (IMAGE_WIDTH),
        .VS_ACTIVE   (VS_ACTIVE),
        .GUARD_XY    (SOBEL_GUARD)
    ) u_input_window (
        .clk            (clk),
        .rst_n          (rst_n),
        .gray_i         (gray_i),
        .pixel_valid_i  (valid_i),
        .hs_i           (hs_i),
        .vs_i           (vs_i),
        .de_i           (de_i),
        .pixels_o       (in_win_pixels),
        .hs_o           (in_win_hs),
        .vs_o           (in_win_vs),
        .de_o           (in_win_de),
        .window_valid_o (in_win_valid),
        .guard_valid_o  (shared_window_valid_o)
    );

    assign shared_window_pixels_o=in_win_pixels;
    assign shared_window_hs_o=in_win_hs;
    assign shared_window_vs_o=in_win_vs;
    assign shared_window_de_o=in_win_de;

    wire signed [11:0] gx_w;
    wire signed [11:0] gy_w;
    /* Opposite signed kernels represent eight directed responses using four
       absolute magnitudes, matching the project's eight-direction Sobel. */
    wire signed [11:0] gd45_w = $signed({3'b0,in_win_pixels[31:24]}) +
        $signed({2'b0,in_win_pixels[7:0],1'b0}) + $signed({3'b0,in_win_pixels[15:8]}) -
        $signed({3'b0,in_win_pixels[63:56]}) - $signed({2'b0,in_win_pixels[71:64],1'b0}) -
        $signed({3'b0,in_win_pixels[47:40]});
    wire signed [11:0] gd135_w = $signed({3'b0,in_win_pixels[63:56]}) +
        $signed({2'b0,in_win_pixels[55:48],1'b0}) + $signed({3'b0,in_win_pixels[31:24]}) -
        $signed({3'b0,in_win_pixels[47:40]}) - $signed({2'b0,in_win_pixels[23:16],1'b0}) -
        $signed({3'b0,in_win_pixels[15:8]});

    assign gx_w = $signed({3'b000, in_win_pixels[55:48]}) +
                  $signed({2'b00,  in_win_pixels[31:24], 1'b0}) +
                  $signed({3'b000, in_win_pixels[7:0]}) -
                  $signed({3'b000, in_win_pixels[71:64]}) -
                  $signed({2'b00,  in_win_pixels[47:40], 1'b0}) -
                  $signed({3'b000, in_win_pixels[23:16]});

    assign gy_w = $signed({3'b000, in_win_pixels[23:16]}) +
                  $signed({2'b00,  in_win_pixels[15:8], 1'b0}) +
                  $signed({3'b000, in_win_pixels[7:0]}) -
                  $signed({3'b000, in_win_pixels[71:64]}) -
                  $signed({2'b00,  in_win_pixels[63:56], 1'b0}) -
                  $signed({3'b000, in_win_pixels[55:48]});

    //==========================================================
    // Sobel gradient
    //==========================================================
    reg signed [11:0] gx, gy, gd45, gd135;
    reg improved_grad;
    reg                sob_valid;
    reg                sob_hs, sob_vs, sob_de;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gx        <= 0;
            gy        <= 0;
            gd45      <= 0; gd135 <= 0; improved_grad <= 0;
            sob_valid <= 0;
            sob_hs    <= 1'b1;
            sob_vs    <= VS_ACTIVE;
            sob_de    <= 0;
        end else begin
            gx        <= gx_w;
            gy        <= gy_w;
            gd45 <= gd45_w; gd135 <= gd135_w;
            improved_grad <= improved_i;
            sob_valid <= in_win_valid;
            sob_hs    <= in_win_hs;
            sob_vs    <= in_win_vs;
            sob_de    <= in_win_de;
        end
    end

    //==========================================================
    // Magnitude and direction
    //==========================================================
    wire [11:0] abs_gx = gx[11] ? (~gx + 12'd1) : gx;
    wire [11:0] abs_gy = gy[11] ? (~gy + 12'd1) : gy;
    wire [11:0] abs_d45 = gd45[11] ? (~gd45 + 12'd1) : gd45;
    wire [11:0] abs_d135 = gd135[11] ? (~gd135 + 12'd1) : gd135;
    wire [11:0] best_axis = abs_gx >= abs_gy ? abs_gx : abs_gy;
    wire [11:0] best_diag = abs_d45 >= abs_d135 ? abs_d45 : abs_d135;

    reg [11:0] magnitude;
    reg [1:0]  angle;
    reg        mag_valid;
    reg        mag_hs, mag_vs, mag_de;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            magnitude <= 0;
            angle     <= ANGLE_0;
            mag_valid <= 0;
            mag_hs    <= 1'b1;
            mag_vs    <= VS_ACTIVE;
            mag_de    <= 0;
        end else begin
            if (improved_grad) begin
                magnitude <= best_axis >= best_diag ? best_axis : best_diag;
                if (best_axis >= best_diag) angle <= abs_gx >= abs_gy ? ANGLE_0 : ANGLE_90;
                /* m00/m02 are the newest (bottom) row; therefore +x,+y
                   compares m02/m20, and +x,-y compares m00/m22. */
                else angle <= abs_d45 >= abs_d135 ? ANGLE_135 : ANGLE_45;
            end else begin
                magnitude <= abs_gx + abs_gy;
                if      (abs_gx >= {abs_gy, 1'b0}) angle <= ANGLE_0;
                else if (abs_gy >= {abs_gx, 1'b0}) angle <= ANGLE_90;
                else if ((gx[11] ^ gy[11]) == 1'b0) angle <= ANGLE_45;
                else                              angle <= ANGLE_135;
            end
            mag_valid <= sob_valid;
            mag_hs    <= sob_hs;
            mag_vs    <= sob_vs;
            mag_de    <= sob_de;
        end
    end

    //==========================================================
    // Generic 3x3 window for magnitude + direction (14 bits)
    //==========================================================
    reg [14:0] mag_line0 [0:IMAGE_WIDTH-1];
    reg [14:0] mag_line1 [0:IMAGE_WIDTH-1];
    reg [14:0] mag_row1_packed_q, mag_row2_packed_q;
    wire [13:0] mag_row1_q=mag_row1_packed_q[13:0],mag_row2_q=mag_row2_packed_q[13:0];
    wire mag_row1_valid_q=mag_row1_packed_q[14],mag_row2_valid_q=mag_row2_packed_q[14];
    reg [10:0] mag_x, mag_y, mag_x_q, mag_y_q;
    reg [13:0] mag_data_q;
    reg        mag_data_valid_q;
    reg        mag_hs_q, mag_vs_q, mag_de_q;
    reg [13:0] m00,m01,m02, m10,m11,m12, m20,m21,m22;
    reg        nms_valid;
    reg        nms_hs, nms_vs, nms_de;

    wire [13:0] mag_data = {angle, magnitude};

    always @(posedge clk) begin
        if (rst_n && mag_de && (mag_vs != VS_ACTIVE)) begin
            mag_row1_packed_q <= mag_line0[mag_x];
            mag_row2_packed_q <= mag_line1[mag_x];
            mag_line0[mag_x] <= {mag_valid,mag_data};
        end
        if (rst_n && mag_de_q && (mag_vs_q != VS_ACTIVE)) begin
            mag_line1[mag_x_q] <= {mag_row1_valid_q,mag_row1_q};
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mag_x <= 0;
            mag_y <= 0;
            mag_x_q <= 0;
            mag_y_q <= 0;
            mag_data_q <= 0;
            mag_data_valid_q <= 0;
            mag_hs_q <= 1'b1;
            mag_vs_q <= VS_ACTIVE;
            mag_de_q <= 0;
            {m00,m01,m02,m10,m11,m12,m20,m21,m22} <= 0;
            nms_valid <= 0;
            nms_hs <= 1'b1;
            nms_vs <= VS_ACTIVE;
            nms_de <= 0;
        end else begin
            if (mag_vs == VS_ACTIVE) begin
                mag_x <= 0;
                mag_y <= 0;
            end else if (mag_de) begin
                if (mag_x == IMAGE_WIDTH-1) begin
                    mag_x <= 0;
                    mag_y <= mag_y + 1'b1;
                end else begin
                    mag_x <= mag_x + 1'b1;
                end
            end

            mag_x_q <= mag_x;
            mag_y_q <= mag_y;
            mag_data_q <= mag_data;
            mag_data_valid_q <= mag_valid;
            mag_hs_q <= mag_hs;
            mag_vs_q <= mag_vs;
            mag_de_q <= mag_de;

            nms_hs <= mag_hs_q;
            nms_vs <= mag_vs_q;
            nms_de <= mag_de_q;
            if (mag_de_q) begin
                m00 <= m01; m01 <= m02; m02 <= mag_data_q;
                m10 <= m11; m11 <= m12; m12 <= mag_row1_q;
                m20 <= m21; m21 <= m22; m22 <= mag_row2_q;
            end else begin
                m02 <= 0; m12 <= 0; m22 <= 0;
            end
            nms_valid <= mag_de_q && (mag_vs_q != VS_ACTIVE) &&
                         (mag_x_q >= 2) && (mag_y_q >= 2) &&
                         // Both asynchronous reads addressed the pixel just
                         // captured into these registered validity values.
                         // The DE/VS gate above guarantees the write occurred.
                         mag_row2_valid_q && mag_data_valid_q && mag_row1_valid_q;
        end
    end

    //==========================================================
    // Non-maximum suppression
    //==========================================================
    reg        nms_keep;
    reg [11:0] nms_mag;
    reg        nm2_valid;
    reg        nm2_hs, nm2_vs, nm2_de;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            nms_keep <= 0;
            nms_mag  <= 0;
            nm2_valid <= 0;
            nm2_hs <= 1'b1;
            nm2_vs <= VS_ACTIVE;
            nm2_de <= 0;
        end else begin
            nm2_hs <= nms_hs;
            nm2_vs <= nms_vs;
            nm2_de <= nms_de;
            case (m11[13:12])
                ANGLE_0:   nms_keep <= (m11[11:0] >= m10[11:0]) && (m11[11:0] >= m12[11:0]);
                ANGLE_90:  nms_keep <= (m11[11:0] >= m01[11:0]) && (m11[11:0] >= m21[11:0]);
                ANGLE_45:  nms_keep <= (m11[11:0] >= m00[11:0]) && (m11[11:0] >= m22[11:0]);
                ANGLE_135: nms_keep <= (m11[11:0] >= m02[11:0]) && (m11[11:0] >= m20[11:0]);
                default:   nms_keep <= 1'b0;
            endcase
            nms_mag   <= m11[11:0];
            nm2_valid <= nms_valid;
        end
    end

    //==========================================================
    // Double threshold
    //==========================================================
    reg [1:0] th_cls;
    // NMS magnitude / 2, saturated to 8 bits; preserve it through the SAME
    // spatial hysteresis window as the class. Threshold units stay unchanged.
    reg [7:0] th_strength;
    reg       th_valid;
    reg       th_hs, th_vs, th_de;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            th_cls   <= 0;
            th_strength <= 0;
            th_valid <= 0;
            th_hs    <= 1'b1;
            th_vs    <= VS_ACTIVE;
            th_de    <= 0;
        end else begin
            th_hs <= nm2_hs;
            th_vs <= nm2_vs;
            th_de <= nm2_de;
            if (nms_keep && nm2_valid) begin
                if      (nms_mag >= hi_latch) th_cls <= 2'd2;
                else if (nms_mag >= lo_latch) th_cls <= 2'd1;
                else                          th_cls <= 2'd0;
            end else begin
                th_cls <= 2'd0;
            end
            th_valid <= nm2_valid;
            th_strength <= (nms_keep && nm2_valid) ?
                           (nms_mag>12'd511 ? 8'hff : nms_mag[8:1]) : 8'd0;
        end
    end

    //==========================================================
    // Generic 3x3 window for hysteresis classes and strength (10 bits)
    //==========================================================
    // Class 3 is unused by the detector and encodes an invalid sample.
    // Restore class 0 on reads so neighborhood arithmetic is unchanged.
    reg [9:0] state_line0 [0:IMAGE_WIDTH-1];
    reg [9:0] state_line1 [0:IMAGE_WIDTH-1];
    reg [9:0] state_row1_packed_q, state_row2_packed_q;
    wire state_row1_valid_q=state_row1_packed_q[1:0]!=2'd3;
    wire state_row2_valid_q=state_row2_packed_q[1:0]!=2'd3;
    wire [9:0] state_row1_q=state_row1_valid_q ? state_row1_packed_q : {state_row1_packed_q[9:2],2'd0};
    wire [9:0] state_row2_q=state_row2_valid_q ? state_row2_packed_q : {state_row2_packed_q[9:2],2'd0};
    reg [10:0] state_x, state_y, state_x_q, state_y_q;
    reg [9:0] state_data_q;
    reg       state_data_valid_q;
    reg       state_hs_q, state_vs_q, state_de_q;
    reg [9:0] n00,n01,n02, n10,n11,n12, n20,n21,n22;
    reg       hy_valid;
    reg       hy_hs, hy_vs, hy_de;

    always @(posedge clk) begin
        if (rst_n && th_de && (th_vs != VS_ACTIVE)) begin
            state_row1_packed_q <= state_line0[state_x];
            state_row2_packed_q <= state_line1[state_x];
            state_line0[state_x] <= {th_strength,(th_valid ? th_cls : 2'd3)};
        end
        if (rst_n && state_de_q && (state_vs_q != VS_ACTIVE)) begin
            state_line1[state_x_q] <= {state_row1_q[9:2],(state_row1_valid_q ? state_row1_q[1:0] : 2'd3)};
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state_x <= 0;
            state_y <= 0;
            state_x_q <= 0;
            state_y_q <= 0;
            state_data_q <= 0;
            state_data_valid_q <= 0;
            state_hs_q <= 1'b1;
            state_vs_q <= VS_ACTIVE;
            state_de_q <= 0;
            {n00,n01,n02,n10,n11,n12,n20,n21,n22} <= 0;
            hy_valid <= 0;
            hy_hs <= 1'b1;
            hy_vs <= VS_ACTIVE;
            hy_de <= 0;
        end else begin
            if (th_vs == VS_ACTIVE) begin
                state_x <= 0;
                state_y <= 0;
            end else if (th_de) begin
                if (state_x == IMAGE_WIDTH-1) begin
                    state_x <= 0;
                    state_y <= state_y + 1'b1;
                end else begin
                    state_x <= state_x + 1'b1;
                end
            end

            state_x_q <= state_x;
            state_y_q <= state_y;
            state_data_q <= {th_strength,th_cls};
            state_data_valid_q <= th_valid;
            state_hs_q <= th_hs;
            state_vs_q <= th_vs;
            state_de_q <= th_de;

            hy_hs <= state_hs_q;
            hy_vs <= state_vs_q;
            hy_de <= state_de_q;
            if (state_de_q) begin
                n00 <= n01; n01 <= n02; n02 <= state_data_q;
                n10 <= n11; n11 <= n12; n12 <= state_row1_q;
                n20 <= n21; n21 <= n22; n22 <= state_row2_q;
            end else begin
                n02 <= 0; n12 <= 0; n22 <= 0;
            end
            hy_valid <= state_de_q && (state_vs_q != VS_ACTIVE) &&
                        (state_x_q >= 2) && (state_y_q >= 2) &&
                        // These registered validity bits already correspond
                        // to the same x/y samples guarded by DE and VS above.
                        state_row2_valid_q && state_data_valid_q && state_row1_valid_q;
        end
    end

    //==========================================================
    // Hysteresis decision and core output
    //==========================================================
    reg       final_edge;
    reg [7:0] final_strength,core_strength;
    reg       fi_valid;
    reg       fi_hs, fi_vs, fi_de;
    reg [23:0] core_rgb;
    reg        core_hs, core_vs, core_de, core_valid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            final_edge <= 0;
            final_strength <= 0;
            fi_valid   <= 0;
            fi_hs      <= 1'b1;
            fi_vs      <= VS_ACTIVE;
            fi_de      <= 0;
        end else begin
            fi_hs <= hy_hs;
            fi_vs <= hy_vs;
            fi_de <= hy_de;
            if (n11[1:0] == 2'd2) begin
                final_edge <= 1'b1;
            end else if (n11[1:0] == 2'd1) begin
                final_edge <= (n00[1:0]==2'd2)||(n01[1:0]==2'd2)||(n02[1:0]==2'd2)||
                              (n10[1:0]==2'd2)          ||(n12[1:0]==2'd2)||
                              (n20[1:0]==2'd2)||(n21[1:0]==2'd2)||(n22[1:0]==2'd2);
            end else begin
                final_edge <= 1'b0;
            end
            fi_valid <= hy_valid;
            final_strength <= n11[9:2];
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            core_rgb   <= 0;
            core_strength <= 0;
            core_hs    <= 1'b1;
            core_vs    <= VS_ACTIVE;
            core_de    <= 0;
            core_valid <= 0;
        end else begin
            core_hs    <= fi_hs;
            core_vs    <= fi_vs;
            core_de    <= fi_de;
            core_valid <= fi_valid;
            core_strength <= (fi_valid && fi_de && final_edge) ? final_strength : 8'd0;
            if (fi_valid && fi_de)
                core_rgb <= final_edge ? 24'hFFFFFF : 24'h000000;
            else
                core_rgb <= 0;
        end
    end

    //==========================================================
    // Fixed output alignment
    //==========================================================
    reg [7:0] strength_tail [0:CANNY_TAIL_DELAY-1];
    reg [23:0] rgb_tail [0:CANNY_TAIL_DELAY-1];
    reg        hs_tail  [0:CANNY_TAIL_DELAY-1];
    reg        vs_tail  [0:CANNY_TAIL_DELAY-1];
    reg        de_tail  [0:CANNY_TAIL_DELAY-1];
    integer tail_i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rgb_o <= 0;
            strength_o <= 0;
            hs_o  <= 1'b1;
            vs_o  <= VS_ACTIVE;
            de_o  <= 0;
            for (tail_i=0; tail_i<CANNY_TAIL_DELAY; tail_i=tail_i+1) begin
                rgb_tail[tail_i] <= 0;
                strength_tail[tail_i] <= 0;
                hs_tail[tail_i]  <= 1'b1;
                vs_tail[tail_i]  <= VS_ACTIVE;
                de_tail[tail_i]  <= 0;
            end
        end else begin
            rgb_tail[0] <= core_rgb;
            strength_tail[0] <= core_strength;
            hs_tail[0]  <= core_hs;
            vs_tail[0]  <= core_vs;
            de_tail[0]  <= core_de;
            for (tail_i=1; tail_i<CANNY_TAIL_DELAY; tail_i=tail_i+1) begin
                rgb_tail[tail_i] <= rgb_tail[tail_i-1];
                strength_tail[tail_i] <= strength_tail[tail_i-1];
                hs_tail[tail_i]  <= hs_tail[tail_i-1];
                vs_tail[tail_i]  <= vs_tail[tail_i-1];
                de_tail[tail_i]  <= de_tail[tail_i-1];
            end
            rgb_o <= rgb_tail[CANNY_TAIL_DELAY-1];
            strength_o <= strength_tail[CANNY_TAIL_DELAY-1];
            hs_o  <= hs_tail[CANNY_TAIL_DELAY-1];
            vs_o  <= vs_tail[CANNY_TAIL_DELAY-1];
            de_o  <= de_tail[CANNY_TAIL_DELAY-1];
        end
    end

endmodule
