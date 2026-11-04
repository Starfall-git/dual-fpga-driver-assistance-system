`timescale 1ns/1ps
// 3x display timing from 640x360 RGB565 to 1920x1080p60 (148.5 MHz).
// Requests a source coordinate every active output pixel. External buffering
// supplies the pixel with its own stated latency and must align HS/VS/DE.
module fatigue_video_timing #(
 parameter H_ACTIVE=1920,H_FRONT=88,H_SYNC=44,H_BACK=148,
 parameter V_ACTIVE=1080,V_FRONT=4,V_SYNC=5,V_BACK=36,SCALE=3
)(
 input wire clk,rst_n,
 output wire de,hs,vs,frame_start,line_start,
 output wire [15:0] x,y,source_x,source_y
);
 localparam HT=H_ACTIVE+H_FRONT+H_SYNC+H_BACK,VT=V_ACTIVE+V_FRONT+V_SYNC+V_BACK;
 reg [15:0] hc,vc,sx,sy;reg [7:0] xp,yp;
 assign x=hc;assign y=vc;assign source_x=sx;assign source_y=sy;
 assign de=hc<H_ACTIVE && vc<V_ACTIVE;
 assign hs=hc>=H_ACTIVE+H_FRONT && hc<H_ACTIVE+H_FRONT+H_SYNC;
 assign vs=vc>=V_ACTIVE+V_FRONT && vc<V_ACTIVE+V_FRONT+V_SYNC;
 assign frame_start=hc==0 && vc==0;
 assign line_start=hc==0 && vc<V_ACTIVE;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin hc<=0;vc<=0;sx<=0;sy<=0;xp<=0;yp<=0;end
  else begin
   if(hc==HT-1)begin
    hc<=0;sx<=0;xp<=0;
    if(vc==VT-1)begin vc<=0;sy<=0;yp<=0;end
    else begin
     vc<=vc+1'b1;
     if(vc<V_ACTIVE)begin
      if(yp==SCALE-1)begin yp<=0;sy<=sy+1'b1;end else yp<=yp+1'b1;
     end
    end
   end else begin
    hc<=hc+1'b1;
    if(hc<H_ACTIVE)begin
     if(xp==SCALE-1)begin xp<=0;sx<=sx+1'b1;end else xp<=xp+1'b1;
    end
   end
  end
 end
endmodule
