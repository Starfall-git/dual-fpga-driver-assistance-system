`timescale 1ns/1ps

module video_sobel2(
 input wire clk,rst_n,
 input wire [71:0] pixels_i,
 input wire [11:0] THRESHOLD,
 input wire hs_i,vs_i,de_i,window_valid_i,
 output reg [7:0] strength_o,
 output reg edge_o,
 output reg hs_o,vs_o,de_o
);
 wire [7:0] a=pixels_i[71:64],b=pixels_i[63:56],c=pixels_i[55:48];
 wire [7:0] d=pixels_i[47:40],f=pixels_i[31:24];
 wire [7:0] g=pixels_i[23:16],h=pixels_i[15:8],i=pixels_i[7:0];
 reg signed [11:0] gx,gy;
 wire [11:0] ax=gx[11] ? -gx : gx, ay=gy[11] ? -gy : gy;
 reg [11:0] magnitude,threshold1,threshold2;
 reg [1:0] hp,vp,dp,valid;
 wire [11:0] excess=magnitude>threshold2 ? magnitude-threshold2 : 12'd0;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   gx<=0;gy<=0;magnitude<=0;threshold1<=0;threshold2<=0;
   hp<=3;vp<=0;dp<=0;valid<=0;strength_o<=0;edge_o<=0;hs_o<=1;vs_o<=0;de_o<=0;
  end else begin
   gx<=($signed({4'd0,c})-$signed({4'd0,a}))+
       (($signed({4'd0,f})-$signed({4'd0,d}))<<<1)+
       ($signed({4'd0,i})-$signed({4'd0,g}));
   gy<=($signed({4'd0,g})-$signed({4'd0,a}))+
       (($signed({4'd0,h})-$signed({4'd0,b}))<<<1)+
       ($signed({4'd0,i})-$signed({4'd0,c}));
   magnitude<=ax+ay;threshold1<=THRESHOLD;threshold2<=threshold1;
   hp<={hp[0],hs_i};vp<={vp[0],vs_i};dp<={dp[0],de_i};valid<={valid[0],window_valid_i};
   hs_o<=hp[1];vs_o<=vp[1];de_o<=dp[1];
   strength_o<=dp[1] && valid[1] ? (excess>255 ? 8'd255 : excess[7:0]) : 8'd0;
   edge_o<=dp[1] && valid[1] && excess!=0;
  end
 end
endmodule


module video_edge_palette(input wire [7:0] strength_i,output reg [23:0] rgb_o);
 always @* begin
  if(strength_i>=235) rgb_o=24'hff0000;
  else if(strength_i>=80) rgb_o=24'h0000ff;
  else if(strength_i>=30) rgb_o=24'h002d00;
  else rgb_o=24'd0;
 end
endmodule
