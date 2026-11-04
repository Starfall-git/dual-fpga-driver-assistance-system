`timescale 1ns/1ps

module key_debounce #(parameter DEBOUNCE_CYCLES=1920000)(
    input wire clk, rst_n, key_i,
    output reg pressed_o
);
    reg meta, synced, stable;
    reg [31:0] count;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin meta<=1; synced<=1; stable<=1; count<=0; pressed_o<=0; end
        else begin
            meta<=key_i; synced<=meta; pressed_o<=0;
            if (synced==stable) count<=0;
            else if (count==DEBOUNCE_CYCLES-1) begin
                stable<=synced; count<=0; pressed_o<=!synced;
            end else count<=count+1'b1;
        end
    end
endmodule
