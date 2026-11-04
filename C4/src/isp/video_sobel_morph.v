`timescale 1ns/1ps
/* Two centered stages; unused operations bypass the center sample while
   keeping the same delay. Foreground always means an edge, never polarity. */
module video_sobel_morph #(
    parameter LINE_CLOCKS=1650, VS_ACTIVE=1'b0
)(
    input wire clk, rst_n, bit_i, hs_i, vs_i, de_i,
    input wire [2:0] mode_i,
    input wire [7:0] strength_i,
    output wire [7:0] strength_o,
    output wire bit_o, hs_o, vs_o, de_o
);
    wire [2:0] bits, hs, vs, de;
    wire [7:0] strength[0:2];
    assign strength[0]=strength_i;
    assign strength_o=strength[2];
    wire [2:0] mode[0:2];
    assign bits[0]=bit_i; assign hs[0]=hs_i; assign vs[0]=vs_i;
    assign de[0]=de_i; assign mode[0]=mode_i;
    genvar stage;
    generate for(stage=0;stage<2;stage=stage+1) begin:g_stage
        video_binary_morph3x3 #(.LINE_CLOCKS(LINE_CLOCKS),.STAGE(stage),.VS_ACTIVE(VS_ACTIVE)) u_morph(
            .clk(clk),.rst_n(rst_n),.bit_i(bits[stage]),.strength_i(strength[stage]),.strength_o(strength[stage+1]),.hs_i(hs[stage]),.vs_i(vs[stage]),
            .de_i(de[stage]),.mode_i(mode[stage]),.bit_o(bits[stage+1]),
            .hs_o(hs[stage+1]),.vs_o(vs[stage+1]),.de_o(de[stage+1]),.mode_o(mode[stage+1]));
    end endgenerate
    assign bit_o=bits[2]; assign hs_o=hs[2]; assign vs_o=vs[2]; assign de_o=de[2];
endmodule
