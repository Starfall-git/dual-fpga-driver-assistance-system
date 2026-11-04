`timescale 1ns/1ps
/* Fixed-clock FIFO. RAM is not reset; the fill flag hides old contents.
   DELAY counts registers, including the synchronous RAM read register. */
module video_stream_delay #(
    parameter WIDTH=1, DELAY=2,
    parameter [WIDTH-1:0] RESET_VALUE=0
)(
    input wire clk, rst_n,
    input wire [WIDTH-1:0] data_i,
    output wire [WIDTH-1:0] data_o
);
    generate
        if(DELAY==0) begin:g_wire
            assign data_o=data_i;
        end else if(DELAY==1) begin:g_register
            reg [WIDTH-1:0] q;
            always @(posedge clk or negedge rst_n)
                if(!rst_n) q<=RESET_VALUE; else q<=data_i;
            assign data_o=q;
        end else begin:g_ram
            localparam DEPTH=DELAY-1;
            localparam ADDR_WIDTH=(DEPTH<=1) ? 1 : $clog2(DEPTH);
            reg [WIDTH-1:0] memory[0:DEPTH-1];
            reg [WIDTH-1:0] q;
            reg [ADDR_WIDTH-1:0] address;
            reg [ADDR_WIDTH:0] filled;
            reg ready;
            always @(posedge clk) begin
                if(rst_n) begin
                    q<=memory[address];
                    memory[address]<=data_i;
                end
            end
            always @(posedge clk or negedge rst_n) begin
                if(!rst_n) begin address<=0; filled<=0; ready<=0; end
                else begin
                    address<=address==DEPTH-1 ? 0 : address+1'b1;
                    if(filled<DEPTH) filled<=filled+1'b1;
                    ready<=filled>=DEPTH;
                end
            end
            assign data_o=ready ? q : RESET_VALUE;
        end
    endgenerate
endmodule
