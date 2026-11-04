`timescale 1ns/1ps

module video_rgb2gray #(
    parameter ENABLE_GRAY = 0,
    parameter VS_ACTIVE = 1'b0
)(
    input wire clk, rst_n,
    input wire [23:0] rgb_i,
    input wire hs_i, vs_i, de_i,
    output reg [7:0] gray_o,
    output reg hs_o, vs_o, de_o
);
    reg [17:0] r_product, g_product, b_product, total;
    reg [7:0] mono_s1, mono_s2;
    reg [1:0] hs_pipe, vs_pipe, de_pipe;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_product<=0; g_product<=0; b_product<=0; total<=0;
            mono_s1<=0; mono_s2<=0; gray_o<=0;
            hs_pipe<=2'b11; vs_pipe<={2{VS_ACTIVE}}; de_pipe<=0;
            hs_o<=1; vs_o<=VS_ACTIVE; de_o<=0;
        end else begin
            r_product <= rgb_i[23:16] * 10'd306;
            g_product <= rgb_i[15:8] * 10'd601;
            b_product <= rgb_i[7:0] * 10'd117;
            total <= r_product + g_product + b_product;
            mono_s1<=rgb_i[15:8]; mono_s2<=mono_s1;
            gray_o <= de_pipe[1] ? (ENABLE_GRAY ? total[17:10] : mono_s2) : 8'd0;
            hs_pipe<={hs_pipe[0],hs_i}; vs_pipe<={vs_pipe[0],vs_i}; de_pipe<={de_pipe[0],de_i};
            hs_o<=hs_pipe[1]; vs_o<=vs_pipe[1]; de_o<=de_pipe[1];
        end
    end
endmodule
