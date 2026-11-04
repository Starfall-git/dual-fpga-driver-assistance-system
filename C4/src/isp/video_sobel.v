`timescale 1ns/1ps

module video_sobel #(
    /* old BINARY_OUTPUT parameter becomes polarity input below. */
    parameter GRAYSCALE_OUTPUT = 0,
    parameter ADAPTIVE_THRESHOLD = 1,
    parameter VS_ACTIVE = 1'b0
)(
    input wire clk, rst_n,
    input wire BINARY_OUTPUT, /* 1: white edge; 0: black edge (runtime input). */
    /* 、15：保留用户改为变量的阈值接口；输入须已同步到本clk域。 */
    input wire [11:0] THRESHOLD,
    input wire [71:0] pixels_i,
    input wire hs_i, vs_i, de_i, window_valid_i,
    output reg [23:0] rgb_o,
    output reg hs_o, vs_o, de_o
);
    wire [7:0] p00 = pixels_i[71:64], p01 = pixels_i[63:56], p02 = pixels_i[55:48];
    wire [7:0] p10 = pixels_i[47:40], p12 = pixels_i[31:24];
    wire [7:0] p20 = pixels_i[23:16], p21 = pixels_i[15:8], p22 = pixels_i[7:0];
    /* each directional signed result is in [-1020,1020]. */
    reg signed [10:0] gx, gy, g45, g135;
    reg [7:0] center_s1, center_s2;
    wire [10:0] abs_g45 = g45[10] ? (~g45 + 11'd1) : g45;
    wire [10:0] abs_g135 = g135[10] ? (~g135 + 11'd1) : g135;
    wire [10:0] abs_gx = gx[10] ? (~gx + 11'd1) : gx;
    wire [10:0] abs_gy = gy[10] ? (~gy + 11'd1) : gy;
    wire [10:0] max_xy = abs_gx > abs_gy ? abs_gx : abs_gy;
    wire [10:0] max_diag = abs_g45 > abs_g135 ? abs_g45 : abs_g135;
    reg [11:0] magnitude;
    reg [1:0] hs_pipe, vs_pipe, de_pipe, valid_pipe;
    /* explicit binary polarity; original grayscale option is independent. */
    wire [11:0] local_floor = center_s2 == 0 ? 12'd1 : {4'b0,center_s2};
    wire [11:0] effective_threshold = ADAPTIVE_THRESHOLD && local_floor > THRESHOLD
                                              ? local_floor : THRESHOLD;
    wire edge_hit=(magnitude>=effective_threshold);
    wire [7:0] edge_value=GRAYSCALE_OUTPUT ?
        ((magnitude>255) ? 8'hff : magnitude[7:0]) :
        ((edge_hit == BINARY_OUTPUT) ? 8'hff : 8'h00);
    wire [23:0] border_value=(!GRAYSCALE_OUTPUT && !BINARY_OUTPUT) ? 24'hffffff : 24'd0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gx <= 0; gy <= 0; g45 <= 0; g135 <= 0;
            center_s1 <= 0; center_s2 <= 0; magnitude <= 0; rgb_o <= 0;
            hs_pipe <= 2'b11; vs_pipe <= {2{VS_ACTIVE}};
            de_pipe <= 0; valid_pipe <= 0;
            hs_o <= 1; vs_o <= VS_ACTIVE; de_o <= 0;
        end else begin
            /* 新增：先扩展到 11 bit 再移位/相减，避免 8 bit 运算溢出。 */
            gx <= $signed({3'b0,p02}) + $signed({2'b0,p12,1'b0}) + $signed({3'b0,p22})
                - $signed({3'b0,p00}) - $signed({2'b0,p10,1'b0}) - $signed({3'b0,p20});
            gy <= $signed({3'b0,p20}) + $signed({2'b0,p21,1'b0}) + $signed({3'b0,p22})
                - $signed({3'b0,p00}) - $signed({2'b0,p01,1'b0}) - $signed({3'b0,p02});
            
            g45 <= $signed({3'b0,p12}) + $signed({2'b0,p22,1'b0}) + $signed({3'b0,p21})
                 - $signed({3'b0,p01}) - $signed({2'b0,p00,1'b0}) - $signed({3'b0,p10});
            g135 <= $signed({3'b0,p01}) + $signed({2'b0,p02,1'b0}) + $signed({3'b0,p12})
                  - $signed({3'b0,p10}) - $signed({2'b0,p20,1'b0}) - $signed({3'b0,p21});
            center_s1 <= pixels_i[39:32]; center_s2 <= center_s1;
            magnitude <= max_xy > max_diag ? {1'b0,max_xy} : {1'b0,max_diag};
            hs_pipe <= {hs_pipe[0],hs_i}; vs_pipe <= {vs_pipe[0],vs_i};
            de_pipe <= {de_pipe[0],de_i}; valid_pipe <= {valid_pipe[0],window_valid_i};
            hs_o <= hs_pipe[1]; vs_o <= vs_pipe[1]; de_o <= de_pipe[1];
            rgb_o <= de_pipe[1] ? (valid_pipe[1] ? {3{edge_value}} : border_value) : 24'd0;
        end
    end
endmodule

