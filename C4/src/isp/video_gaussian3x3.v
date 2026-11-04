`timescale 1ns/1ps
/*
 * 3x3 Gaussian filter, one pixel per clock.
 *
 * The arithmetic path is four clocks and every control signal follows the
 * same four registers: weighted terms, sum, normalization, output.
 */
module video_gaussian3x3 #(
    parameter VS_ACTIVE = 1'b0
)(
    input  wire            clk,
    input  wire            rst_n,

    input  wire [71:0]     pixels_i,

    input  wire            hs_i,
    input  wire            vs_i,
    input  wire            de_i,
    input  wire            window_valid_i,

    output reg  [7:0]      gray_o,
    output reg             hs_o,
    output reg             vs_o,
    output reg             de_o,
    output reg             window_valid_o
);

    //==========================================================
    // Stage 1: weighted samples
    //==========================================================
    reg [11:0] w0, w1, w2, w3, w4, w5, w6, w7, w8;
    reg        s1_valid;
    reg        s1_hs, s1_vs, s1_de;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            w0<=0; w1<=0; w2<=0;
            w3<=0; w4<=0; w5<=0;
            w6<=0; w7<=0; w8<=0;
            s1_valid <= 0;
            s1_hs <= 1'b1;
            s1_vs <= VS_ACTIVE;
            s1_de <= 0;
        end else begin
            w0 <= {4'b0000, pixels_i[71:64]};
            w1 <= {3'b000,  pixels_i[63:56], 1'b0};
            w2 <= {4'b0000, pixels_i[55:48]};
            w3 <= {3'b000,  pixels_i[47:40], 1'b0};
            w4 <= {2'b00,   pixels_i[39:32], 2'b00};
            w5 <= {3'b000,  pixels_i[31:24], 1'b0};
            w6 <= {4'b0000, pixels_i[23:16]};
            w7 <= {3'b000,  pixels_i[15:8],  1'b0};
            w8 <= {4'b0000, pixels_i[7:0]};
            s1_valid <= window_valid_i;
            s1_hs <= hs_i;
            s1_vs <= vs_i;
            s1_de <= de_i;
        end
    end

    //==========================================================
    // Stage 2: weighted sum
    //==========================================================
    reg [14:0] sum;
    reg        s2_valid;
    reg        s2_hs, s2_vs, s2_de;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sum <= 0;
            s2_valid <= 0;
            s2_hs <= 1'b1;
            s2_vs <= VS_ACTIVE;
            s2_de <= 0;
        end else begin
            sum <= w0+w1+w2+w3+w4+w5+w6+w7+w8;
            s2_valid <= s1_valid;
            s2_hs <= s1_hs;
            s2_vs <= s1_vs;
            s2_de <= s1_de;
        end
    end

    //==========================================================
    // Stage 3: normalize by 16
    //==========================================================
    reg [7:0] norm;
    reg       s3_valid;
    reg       s3_hs, s3_vs, s3_de;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            norm <= 0;
            s3_valid <= 0;
            s3_hs <= 1'b1;
            s3_vs <= VS_ACTIVE;
            s3_de <= 0;
        end else begin
            norm <= sum[14:4];
            s3_valid <= s2_valid;
            s3_hs <= s2_hs;
            s3_vs <= s2_vs;
            s3_de <= s2_de;
        end
    end

    //==========================================================
    // Stage 4: registered output
    //==========================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gray_o <= 0;
            hs_o <= 1'b1;
            vs_o <= VS_ACTIVE;
            de_o <= 0;
            window_valid_o <= 0;
        end else begin
            gray_o <= norm;
            hs_o <= s3_hs;
            vs_o <= s3_vs;
            de_o <= s3_de;
            window_valid_o <= s3_valid;
        end
    end

endmodule