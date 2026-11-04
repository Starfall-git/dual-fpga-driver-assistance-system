`timescale 1ns/1ps

module video_window3x3 #(
    parameter IMAGE_WIDTH = 1280,
    /* generic sample width for Canny magnitude/direction and labels. */
    parameter DATA_WIDTH = 8,
    parameter VS_ACTIVE = 1'b0,
    /* additional input-side halo for downstream edge filters. */
    parameter MIN_VALID_XY = 2,
    parameter GUARD_XY = 2
)(
    input wire clk,
    input wire rst_n,
    input wire [DATA_WIDTH-1:0] gray_i,
    
    input wire pixel_valid_i,
    input wire hs_i, vs_i, de_i,
    output reg [9*DATA_WIDTH-1:0] pixels_o,
    output reg hs_o, vs_o, de_o,
    output reg window_valid_o,
    output reg guard_valid_o
);
    // Pixel validity shares each line RAM word with its pixel.
    reg [DATA_WIDTH:0] line1 [0:IMAGE_WIDTH-1];
    reg [DATA_WIDTH:0] line2 [0:IMAGE_WIDTH-1];
    reg [DATA_WIDTH:0] row1_packed_q,row2_packed_q;
    wire [DATA_WIDTH-1:0] row1_q=row1_packed_q[DATA_WIDTH-1:0],row2_q=row2_packed_q[DATA_WIDTH-1:0];
    wire row1_valid_q=row1_packed_q[DATA_WIDTH],row2_valid_q=row2_packed_q[DATA_WIDTH];
    reg [11:0] x, y, x_q, y_q;
    reg [DATA_WIDTH-1:0] gray_q;
    reg gray_valid_q;
    reg hs_q, vs_q, de_q;
    reg [DATA_WIDTH-1:0] top_left2, top_left1;
    reg [DATA_WIDTH-1:0] mid_left2, mid_left1;
    reg [DATA_WIDTH-1:0] bot_left2, bot_left1;
    reg top_left2_valid, top_left1_valid;
    reg mid_left2_valid, mid_left1_valid;
    reg bot_left2_valid, bot_left1_valid;

    /* 新增：同步读 RAM；第二行缓存延迟一拍写入第一行缓存读出的旧像素。
       读、写地址分开，避免组合读 RAM，允许综合为 FPGA 块 RAM。 */
    always @(posedge clk) begin
        if (rst_n && de_i && (vs_i != VS_ACTIVE)) begin
            row1_packed_q <= line1[x];
            row2_packed_q <= line2[x];
            line1[x] <= {pixel_valid_i,gray_i};
        end
        if (rst_n && de_q && (vs_q != VS_ACTIVE)) begin
            line2[x_q] <= {row1_valid_q,row1_q};
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x <= 0; y <= 0; x_q <= 0; y_q <= 0;
            gray_q <= 0;
            gray_valid_q <= 0;
            hs_q <= 1; vs_q <= VS_ACTIVE; de_q <= 0;
            hs_o <= 1; vs_o <= VS_ACTIVE; de_o <= 0;
            pixels_o <= 0; window_valid_o <= 0; guard_valid_o <= 0;
            top_left2 <= 0; top_left1 <= 0;
            mid_left2 <= 0; mid_left1 <= 0;
            bot_left2 <= 0; bot_left1 <= 0;
            top_left2_valid <= 0; top_left1_valid <= 0;
            mid_left2_valid <= 0; mid_left1_valid <= 0;
            bot_left2_valid <= 0; bot_left1_valid <= 0;
        end else begin
            /* 新增：仅有效像素推进坐标，消隐期间保持；VS 重置帧坐标。 */
            if (vs_i == VS_ACTIVE) begin
                x <= 0; y <= 0;
            end else if (de_i) begin
                if (x == IMAGE_WIDTH-1) begin
                    x <= 0;
                    if (y != 12'hfff) y <= y + 1'b1;
                end else x <= x + 1'b1;
            end
            x_q <= x; y_q <= y; gray_q <= gray_i; gray_valid_q <= pixel_valid_i;
            hs_q <= hs_i; vs_q <= vs_i; de_q <= de_i;
            hs_o <= hs_q; vs_o <= vs_q; de_o <= de_q;
            
            window_valid_o <= de_q && (vs_q != VS_ACTIVE)
                && (x_q >= MIN_VALID_XY) && (y_q >= MIN_VALID_XY)
                && top_left2_valid && top_left1_valid && row2_valid_q
                && mid_left2_valid && mid_left1_valid && row1_valid_q
                && bot_left2_valid && bot_left1_valid && gray_valid_q;
            guard_valid_o <= de_q && (vs_q != VS_ACTIVE)
                && (x_q >= GUARD_XY) && (y_q >= GUARD_XY)
                && top_left2_valid && top_left1_valid && row2_valid_q
                && mid_left2_valid && mid_left1_valid && row1_valid_q
                && bot_left2_valid && bot_left1_valid && gray_valid_q;
            if (de_q) begin
                pixels_o <= {top_left2, top_left1, row2_q,
                             mid_left2, mid_left1, row1_q,
                             bot_left2, bot_left1, gray_q};
                top_left2 <= top_left1; top_left1 <= row2_q;
                mid_left2 <= mid_left1; mid_left1 <= row1_q;
                bot_left2 <= bot_left1; bot_left1 <= gray_q;
                top_left2_valid <= top_left1_valid; top_left1_valid <= row2_valid_q;
                mid_left2_valid <= mid_left1_valid; mid_left1_valid <= row1_valid_q;
                bot_left2_valid <= bot_left1_valid; bot_left1_valid <= gray_valid_q;
            end
        end
    end
endmodule
