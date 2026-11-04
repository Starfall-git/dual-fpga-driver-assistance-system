`timescale 1ns/1ps
/*
 * Window-input adaptation of the arithmetic in the supplied
 * Sobeledge_8d/hdl/Sobeledge_proc.v. The reference's own matrix/line buffer
 * cannot be cascaded after video_window3x3 without changing the shared
 * 13-clock video_processing timing. The four directional sums and Gmax
 * selection below retain its exact coefficients and 10-bit ranges.
 */
module sobeledge_8d_window #(
    parameter GRAYSCALE_OUTPUT = 1,
    parameter ADAPTIVE_THRESHOLD = 1,
    parameter VS_ACTIVE = 1'b0
)(
    input wire clk, rst_n,
    input wire BINARY_OUTPUT,
    input wire [11:0] THRESHOLD,
    input wire [71:0] pixels_i,
    input wire hs_i, vs_i, de_i, window_valid_i,
    output reg [23:0] rgb_o,
    output reg hs_o, vs_o, de_o,
    output reg [7:0] strength_o,
    output reg [9:0] magnitude_o,
    output reg edge_o
);
    wire [7:0] p11 = pixels_i[71:64], p12 = pixels_i[63:56];
    wire [7:0] p13 = pixels_i[55:48], p21 = pixels_i[47:40];
    wire [7:0] p22 = pixels_i[39:32], p23 = pixels_i[31:24];
    wire [7:0] p31 = pixels_i[23:16], p32 = pixels_i[15:8];
    wire [7:0] p33 = pixels_i[7:0];

    reg [9:0] g0_p, g0_n, g45_p, g45_n;
    reg [9:0] g90_p, g90_n, g135_p, g135_n;
    wire [9:0] g0 = g0_p >= g0_n ? g0_p-g0_n : g0_n-g0_p;
    wire [9:0] g45 = g45_p >= g45_n ? g45_p-g45_n : g45_n-g45_p;
    wire [9:0] g90 = g90_p >= g90_n ? g90_p-g90_n : g90_n-g90_p;
    wire [9:0] g135 = g135_p >= g135_n ? g135_p-g135_n : g135_n-g135_p;
    wire [9:0] max_0_45 = g0 >= g45 ? g0 : g45;
    wire [9:0] max_90_135 = g90 >= g135 ? g90 : g135;
    reg [9:0] gmax;
    reg [7:0] center_s1, center_s2;
    reg [1:0] hs_pipe, vs_pipe, de_pipe, valid_pipe;
    wire [11:0] center_floor = center_s2 == 0 ? 12'd1 : {4'd0, center_s2};
    wire [11:0] effective_threshold = ADAPTIVE_THRESHOLD && center_floor > THRESHOLD
                                     ? center_floor : THRESHOLD;
    wire edge_hit = {2'd0, gmax} >= effective_threshold;
    /* At threshold zero this is the reference bin_en=0 output.
       Subtracting the UART threshold restores a useful continuous noise
       control: weak gradients vanish, stronger gradients fade gradually. */
    wire [11:0] excess = {2'd0, gmax} > THRESHOLD
                       ? {2'd0, gmax} - THRESHOLD : 12'd0;
    wire [7:0] strength = excess > 12'd510 ? 8'hff : excess[8:1];
    wire [7:0] binary = edge_hit == BINARY_OUTPUT ? 8'hff : 8'h00;
    wire [7:0] pixel = GRAYSCALE_OUTPUT
                     ? (BINARY_OUTPUT ? strength : ~strength) : binary;
    wire [23:0] border = !BINARY_OUTPUT ? 24'hffffff : 24'd0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            g0_p <= 0; g0_n <= 0; g45_p <= 0; g45_n <= 0;
            g90_p <= 0; g90_n <= 0; g135_p <= 0; g135_n <= 0;
            gmax <= 0; center_s1 <= 0; center_s2 <= 0;
            hs_pipe <= 2'b11; vs_pipe <= {2{VS_ACTIVE}};
            de_pipe <= 0; valid_pipe <= 0;
            hs_o <= 1; vs_o <= VS_ACTIVE; de_o <= 0; rgb_o <= 0; edge_o <= 0; strength_o<=0; magnitude_o<=0;
        end else begin
            /* Stage 1: the reference's positive and negative kernel sums. */
            g0_p <= p11 + {1'b0,p12,1'b0} + p13;
            g0_n <= p31 + {1'b0,p32,1'b0} + p33;
            g45_p <= p23 + {1'b0,p33,1'b0} + p32;
            g45_n <= p12 + {1'b0,p11,1'b0} + p21;
            g90_p <= p13 + {1'b0,p23,1'b0} + p33;
            g90_n <= p11 + {1'b0,p21,1'b0} + p31;
            g135_p <= p12 + {1'b0,p13,1'b0} + p23;
            g135_n <= p21 + {1'b0,p31,1'b0} + p32;
            center_s1 <= p22;
            /* Stage 2: absolute values and maximum, same reference formula. */
            gmax <= max_0_45 >= max_90_135 ? max_0_45 : max_90_135;
            center_s2 <= center_s1;
            hs_pipe <= {hs_pipe[0], hs_i};
            vs_pipe <= {vs_pipe[0], vs_i};
            de_pipe <= {de_pipe[0], de_i};
            valid_pipe <= {valid_pipe[0], window_valid_i};
            /* Stage 3: reference intensity output or existing binary mode. */
            hs_o <= hs_pipe[1]; vs_o <= vs_pipe[1]; de_o <= de_pipe[1];
            strength_o<=de_pipe[1] && valid_pipe[1] ? strength : 8'd0;
            magnitude_o<=gmax;
            edge_o <= de_pipe[1] && valid_pipe[1] && edge_hit;
            rgb_o <= de_pipe[1] ? (valid_pipe[1] ? {3{pixel}} : border) : 24'd0;
        end
    end
endmodule
