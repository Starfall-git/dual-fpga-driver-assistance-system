`timescale 1ns/1ps
/* 2026-09-21 新增 UART 8N1 接收，双级同步、位中心采样、停止位检查。 */
module uart_rx #(parameter CLOCK_HZ=96000000, parameter BAUD=115200)(
    input wire clk, rst_n, rx_i,
    output reg [7:0] data_o,
    output reg valid_o, error_o
);
    localparam TICKS=(CLOCK_HZ+BAUD/2)/BAUD;
    reg rx_meta, rx_sync;
    reg [1:0] state;
    reg [31:0] timer;
    reg [2:0] bit_index;
    reg [7:0] shift;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin rx_meta<=1; rx_sync<=1; end
        else begin rx_meta<=rx_i; rx_sync<=rx_meta; end
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state<=0; timer<=0; bit_index<=0; shift<=0;
            data_o<=0; valid_o<=0; error_o<=0;
        end else begin
            valid_o<=0; error_o<=0;
            case(state)
                0: if (!rx_sync) begin timer<=TICKS/2-1; state<=1; end
                1: if (timer!=0) timer<=timer-1'b1;
                   else if (!rx_sync) begin timer<=TICKS-1; state<=2; bit_index<=0; end
                   else state<=0;
                2: if (timer!=0) timer<=timer-1'b1;
                   else begin
                       shift[bit_index]<=rx_sync; timer<=TICKS-1;
                       if (bit_index==7) state<=3;
                       else bit_index<=bit_index+1'b1;
                   end
                3: if (timer!=0) timer<=timer-1'b1;
                   else begin
                       state<=0;
                       if (rx_sync) begin data_o<=shift; valid_o<=1; end
                       else error_o<=1;
                   end
            endcase
        end
    end
endmodule
