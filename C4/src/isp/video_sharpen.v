`timescale 1ns/1ps
// Three-clock signed unsharp mask. All inputs refer to the same center pixel.
// alpha: 0=1/2 (arithmetic shift), 1=1, 2=2. Saturate only after addition.
module video_sharpen(
    input wire clk, rst_n,
    input wire [23:0] rgb_i,
    input wire [7:0] center_i, blur_i,
    input wire [9:0] magnitude_i,
    input wire [11:0] threshold_i,
    input wire [1:0] alpha_i,
    input wire enable_i, valid_i,
    output reg [23:0] rgb_o,
    output reg [23:0] original_o,
    output reg signed [9:0] delta_o
);
    reg signed [10:0] difference, scaled;
    reg [23:0] rgb1, rgb2;
    reg [1:0] alpha1;
    reg gate1, gate2;
    function [7:0] add_clip;
        input [7:0] pixel;
        input signed [10:0] delta;
        reg signed [11:0] sum;
        begin
            sum=$signed({1'b0,pixel})+delta;
            add_clip=sum<0 ? 8'd0 : sum>255 ? 8'd255 : sum[7:0];
        end
    endfunction
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            difference<=0;scaled<=0;rgb1<=0;rgb2<=0;rgb_o<=0;original_o<=0;delta_o<=0;
            alpha1<=0;gate1<=0;gate2<=0;
        end else begin
            difference<=$signed({1'b0,center_i})-$signed({1'b0,blur_i});
            rgb1<=rgb_i;alpha1<=alpha_i;
            gate1<=enable_i && valid_i && ({2'b0,magnitude_i}>=threshold_i);
            case(alpha1)
                0:scaled<=difference>>>1;
                1:scaled<=difference;
                2:scaled<=difference<<<1;
                default:scaled<=0;
            endcase
            rgb2<=rgb1;gate2<=gate1;
            original_o<=rgb2;
            delta_o<=gate2 ? scaled[9:0] : 10'sd0;
            rgb_o<=gate2 ? {add_clip(rgb2[23:16],scaled),add_clip(rgb2[15:8],scaled),add_clip(rgb2[7:0],scaled)} : rgb2;
        end
    end
endmodule
