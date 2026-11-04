`timescale 1ns/1ps
// Double scanline RAM: eight independent lanes permit a 128-bit DDR beat per
// write cycle and one RGB565 pixel per pixel-clock cycle. Ownership/safe swap
// is the caller's responsibility; never write a bank the display is reading.
module fatigue_line_store #(parameter WIDTH=640,BEATS=(WIDTH+7)/8,ADDR=$clog2(BEATS))(
 input wire wclk,input wire wr_valid,input wire wr_bank,input wire [ADDR-1:0] wr_beat,input wire [127:0] wr_data,
 input wire pclk,input wire pr_valid,input wire pr_bank,input wire [$clog2(WIDTH)-1:0] pr_x,
 output reg pixel_valid,output reg [15:0] pixel
);
 wire [ADDR:0] wa={wr_bank,wr_beat};wire [ADDR:0] ra={pr_bank,pr_x[$clog2(WIDTH)-1:3]};
 reg [15:0] rd[0:7];reg [2:0] lane_d;reg valid_d;
 genvar i;
 generate for(i=0;i<8;i=i+1)begin:lanes
  reg [15:0] mem[0:2*(1<<ADDR)-1];
  always @(posedge wclk)if(wr_valid && wr_beat<BEATS)mem[wa]<=wr_data[i*16+:16];
  always @(posedge pclk)if(pr_valid && pr_x<WIDTH)rd[i]<=mem[ra];
 end endgenerate
 always @(posedge pclk)begin
  valid_d<=pr_valid && pr_x<WIDTH;lane_d<=pr_x[2:0];
  pixel_valid<=valid_d;
  if(valid_d)pixel<=rd[lane_d];else pixel<=0;
 end
endmodule
