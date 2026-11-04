`timescale 1ns/1ps
/* 2026-09-21 新增 UART 8N1 发送；start_i 仅在 busy_o=0 时接受。 */
module uart_tx #(parameter CLOCK_HZ=96000000, parameter BAUD=115200)(
    input wire clk, rst_n,
    input wire [7:0] data_i,
    input wire start_i,
    output wire tx_o,
    output reg busy_o
);
    localparam TICKS=(CLOCK_HZ+BAUD/2)/BAUD;
    reg [9:0] shift;
    reg [3:0] bit_index;
    reg [31:0] timer;
    assign tx_o=busy_o ? shift[0] : 1'b1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin shift<=10'h3ff; bit_index<=0; timer<=0; busy_o<=0; end
        else if (!busy_o) begin
            if (start_i) begin
                shift<={1'b1,data_i,1'b0}; timer<=TICKS-1; bit_index<=0; busy_o<=1;
            end
        end else if (timer!=0) timer<=timer-1'b1;
        else if (bit_index==9) busy_o<=0;
        else begin shift<={1'b1,shift[9:1]}; timer<=TICKS-1; bit_index<=bit_index+1'b1; end
    end
endmodule
