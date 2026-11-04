`timescale 1ns/1ps

module video_split_display #(
 parameter IMAGE_WIDTH=1280, LINE_CLOCKS=1650, VS_ACTIVE=0
)(input wire clk,rst_n, input wire [1:0] mode_i,
 input wire [23:0] processed_i,raw_i,input wire hs_i,vs_i,de_i,
 output reg [23:0] rgb_o,output reg hs_o,vs_o,de_o);
 localparam XW=$clog2(IMAGE_WIDTH), HALF=IMAGE_WIDTH/2;
 reg [23:0] left_mem[0:IMAGE_WIDTH-1],right_mem[0:IMAGE_WIDTH-1];
 reg [XW-1:0] x,ox;
 reg bank,de_d;
 reg [1:0] mode;
 wire [XW-1:0] write_x=de_d ? x : 0;
 wire central=mode==2;
 wire center_valid=write_x>=IMAGE_WIDTH/4 && write_x<3*IMAGE_WIDTH/4;
 wire left_we=de_i && (central ? center_valid : write_x<HALF);
 wire right_we=de_i && (central ? center_valid : write_x>=HALF);
 wire [XW-1:0] left_addr=(bank ? HALF : 0)+(central ? write_x-IMAGE_WIDTH/4 : write_x);
 wire [XW-1:0] right_addr=(bank ? HALF : 0)+(central ? write_x-IMAGE_WIDTH/4 : write_x-HALF);
 always @(posedge clk) begin
  if(rst_n && left_we) left_mem[left_addr]<=processed_i;
  if(rst_n && right_we) right_mem[right_addr]<=mode==0 ? processed_i : raw_i;
 end
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin x<=0;bank<=0;de_d<=0;mode<=0;end
  else begin
   de_d<=de_i;
   if(vs_i==VS_ACTIVE) mode<=mode_i==3 ? 0 : mode_i;
   if(de_i) x<=write_x==IMAGE_WIDTH-1 ? 0 : write_x+1'b1;
   if(de_d && !de_i) bank<=~bank;
  end
 end
 wire [3:0] meta;
 video_stream_delay #(.WIDTH(4),.DELAY(LINE_CLOCKS),.RESET_VALUE(0)) u_meta(
 .clk(clk),.rst_n(rst_n),.data_i({bank,hs_i,vs_i,de_i}),.data_o(meta));
 wire right_half=ox>=HALF;
 wire [XW-1:0] read_addr=(meta[3] ? HALF : 0)+(right_half ? ox-HALF : ox);
 reg [23:0] left_q,right_q;
 reg [2:0] sync_q;
 reg right_q_sel;
 always @(posedge clk) if(rst_n) begin left_q<=left_mem[read_addr];right_q<=right_mem[read_addr];end
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin ox<=0;sync_q<=0;right_q_sel<=0;rgb_o<=0;hs_o<=0;vs_o<=VS_ACTIVE;de_o<=0;end
  else begin
   if(!meta[0] || ox==IMAGE_WIDTH-1) ox<=0;else ox<=ox+1'b1;
   sync_q<=meta[2:0];right_q_sel<=right_half;
   {hs_o,vs_o,de_o}<=sync_q;
   rgb_o<=sync_q[0] ? (right_q_sel ? right_q : left_q) : 24'd0;
  end
 end
endmodule
