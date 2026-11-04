`timescale 1ns/1ps
// Independent-clock FIFO with synchronous RAM read and held output handshake.
// Both resets must be asserted together on stream restart.
module fatigue_async_fifo #(parameter WIDTH=42, parameter ADDR=10)(
 input wire wclk,wrst_n,input wire w_valid,output wire w_ready,output wire w_almost_full,input wire [WIDTH-1:0] w_data,
 input wire rclk,rrst_n,output reg r_valid,input wire r_ready,output reg [WIDTH-1:0] r_data
);
 localparam DEPTH=(1<<ADDR);
 reg [WIDTH-1:0] mem[0:DEPTH-1];
 reg [ADDR:0] wb,wg,rb,rg;
 (* async_reg="true" *) reg [ADDR:0] rgray_m,rgray_s,wgray_m,wgray_s;
 function [ADDR:0] gray_binary;
  input [ADDR:0] g;integer j;
  begin gray_binary[ADDR]=g[ADDR];for(j=ADDR-1;j>=0;j=j-1)gray_binary[j]=gray_binary[j+1]^g[j];end
 endfunction
 wire [ADDR:0] used=wb-gray_binary(rgray_s);
 assign w_almost_full=used>=DEPTH-2;
 wire [ADDR:0] wb_next=wb+1'b1;
 wire [ADDR:0] wg_next=(wb_next>>1)^wb_next;
 wire full=wg=={~rgray_s[ADDR:ADDR-1],rgray_s[ADDR-2:0]};
 wire empty=rg==wgray_s;
 wire push=w_valid&&w_ready;
 wire pop=(!r_valid||r_ready)&&!empty;
 assign w_ready=!full;
 always @(posedge wclk or negedge wrst_n)begin
  if(!wrst_n)begin wb<=0;wg<=0;rgray_m<=0;rgray_s<=0;end
  else begin
   rgray_m<=rg;rgray_s<=rgray_m;
   if(push)begin mem[wb[ADDR-1:0]]<=w_data;wb<=wb_next;wg<=wg_next;end
  end
 end
 always @(posedge rclk or negedge rrst_n)begin
  if(!rrst_n)begin rb<=0;rg<=0;wgray_m<=0;wgray_s<=0;r_valid<=0;r_data<=0;end
  else begin
   wgray_m<=wg;wgray_s<=wgray_m;
   if(pop)begin r_data<=mem[rb[ADDR-1:0]];rb<=rb+1'b1;rg<=((rb+1'b1)>>1)^(rb+1'b1);r_valid<=1;end
   else if(r_valid&&r_ready)r_valid<=0;
  end
 end
endmodule
