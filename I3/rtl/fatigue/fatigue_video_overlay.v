`timescale 1ns/1ps
/* registered HDMI RGB/status overlay, one cycle for RGB and sync.
   Pixel coordinates refer to the input RGB stream. Labels are demo status, not diagnosis. */
/* FATIGUE-SC431-1080p centered inference guide default. */
module fatigue_video_overlay #(parameter ROI_X=672,ROI_Y=252,ROI_SIZE=576)(
 input wire clk,rst_n,input wire [23:0] rgb_i,input wire hs_i,vs_i,de_i,
 input wire [11:0] x,y,input wire [2:0] state,input wire face_valid,
 input wire [31:0] box0,box1,
 input wire [319:0] extra_points, input wire [31:0] visual,p0,p1,p2,p3,p4,output reg [23:0] rgb_o,output reg hs_o,vs_o,de_o);
 // Hold the last confirmed eye display briefly during uncertain inference.
 // This never changes the underlying fatigue state or warning output.
 reg [1:0] last_eye; reg [7:0] hold_age; reg vs_prev;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n)begin last_eye<=0;hold_age<=255;vs_prev<=0;end
  else begin
   vs_prev<=vs_i;
   if(face_valid && (state==1 || state==2 || state==3))begin
    last_eye<=state==1 ? 1 : 2; hold_age<=0;
   end else if(!face_valid || state==4 || state==6)begin last_eye<=0;hold_age<=255;end
   else if(vs_i&&!vs_prev && hold_age<255)hold_age<=hold_age+1'b1;
  end
 end
 reg [127:0] label;
 always @* case(state)
  1:label="AWAKE           ";
  2:label="EYES CLOSED     ";
  3:label="FATIGUE RISK    ";
  4:label="NO FACE         ";
  5:label="UNCERTAIN       ";
  6:label="STALE           ";
  default:label="STARTING        ";
 endcase
 function [34:0] glyph;
 input [7:0] ch;
 begin case(ch)
  8'd65:glyph=35'h3a31fc631;
  8'd66:glyph=35'h7a31f463e;
  8'd67:glyph=35'h3e108420f;
  8'd68:glyph=35'h7a318c63e;
  8'd69:glyph=35'h7e10f421f;
  8'd70:glyph=35'h7e10f4210;
  8'd71:glyph=35'h3e10bc62f;
  8'd72:glyph=35'h4631fc631;
  8'd73:glyph=35'h7c842109f;
  8'd74:glyph=35'h1c4210a4c;
  8'd75:glyph=35'h4654c5251;
  8'd76:glyph=35'h42108421f;
  8'd77:glyph=35'h4775ac631;
  8'd78:glyph=35'h47359c631;
  8'd79:glyph=35'h3a318c62e;
  8'd80:glyph=35'h7a31f4210;
  8'd81:glyph=35'h3a318d64d;
  8'd82:glyph=35'h7a31f5251;
  8'd83:glyph=35'h3e107043e;
  8'd84:glyph=35'h7c8421084;
  8'd85:glyph=35'h46318c62e;
  8'd86:glyph=35'h46318c544;
  8'd87:glyph=35'h4631ad771;
  8'd88:glyph=35'h462a22a31;
  8'd89:glyph=35'h462a21084;
  8'd90:glyph=35'h7c222221f;
  8'd48:glyph=35'h3a33ae62e;
  8'd49:glyph=35'h11842108e;
  8'd50:glyph=35'h3a211111f;
  8'd51:glyph=35'h78217043e;
  8'd52:glyph=35'h08ca97c42;
  8'd53:glyph=35'h7e10f043e;
  8'd54:glyph=35'h3a10f462e;
  8'd55:glyph=35'h7c2222108;
  8'd56:glyph=35'h3a317462e;
  8'd57:glyph=35'h3a317842e;
  8'd45:glyph=35'h0000f8000;
  8'd47:glyph=35'h042222210;
  8'd58:glyph=35'h008401080;
  8'd46:glyph=35'h00000018c;
  default:glyph=0;
 endcase end endfunction
 wire [23:0] color=visual[11]?24'hff3030:state==1?24'h30ff30:24'hffff30;
 wire [11:0] tx=x-28,ty=y-28;
 wire [3:0] char_index=tx[7:4];
 wire [1:0] line=ty/24;
 reg [127:0] text_line;
 always @* begin
  text_line="                ";
  case(line)
   0:if(visual[17]) begin
      if(state==6)text_line="EYE: NO DATA    ";
      else if(state==4)text_line="EYE: NO FACE    ";
      else if(state==1)text_line="EYE: OPEN       ";
      else if(state==2||state==3)text_line="EYE: CLOSED     ";
      else if(state==5 && last_eye==1 && hold_age<120)text_line="EYE: HOLD OPEN  ";
      else if(state==5 && last_eye==2 && hold_age<120)text_line="EYE: HOLD CLOSED";
      else text_line="EYE: --         ";
     end
   1:if(visual[17]&&face_valid)text_line=visual[9]?"YAWN: YES       ":"YAWN: NO        ";
   2:if(visual[17]&&face_valid)text_line=visual[10]?"HEAD: DOWN      ":"HEAD: NORMAL    ";
   3:if(visual[18])begin
      if(state==6||state==4||!face_valid)text_line="RISK: UNKNOWN   ";
      else text_line=visual[11]?"RISK: ALERT     ":"RISK: NORMAL    ";
     end
  endcase
 end
 wire in_panel=(visual[17]||visual[18])&&x>=16&&x<368&&y>=16&&y<140;
 wire panel_border=x==16||x==367||y==16||y==139;
 wire [7:0] ch=text_line>>(120-{char_index,3'b000});
 wire [34:0] bits=glyph(ch);
 wire [4:0] local_y=ty%24;
 wire text_area=x>=28&&x<284&&y>=28&&y<124;
 wire ink=text_area&&tx[3:1]<5&&local_y[4:1]<7&&bits[34-(local_y[4:1]*5+tx[3:1])];
 wire [15:0] bx0=box0[15:0],by0=box0[31:16],bx1=box1[15:0],by1=box1[31:16];
 wire face_edge=visual[16]&&face_valid&&bx0<bx1&&by0<by1&&x>=bx0&&x<=bx1&&y>=by0&&y<=by1&&
                (x==bx0||x==bx1||y==by0||y==by1);
 function automatic point_mark(input [31:0] p,input [11:0] xx,input [11:0] yy);
  reg [15:0] px,py;begin px=p[15:0];py=p[31:16];
   point_mark=(xx+3>=px&&xx<=px+3&&yy+3>=py&&yy<=py+3);
  end
 endfunction
 function automatic part_mark(input [31:0] p,input [11:0] xx,input [11:0] yy);
  reg [15:0] px,py;begin px=p[15:0];py=p[31:16];
   part_mark=xx+18>=px&&xx<=px+18&&yy+12>=py&&yy<=py+12&&
       (xx+18==px||xx==px+18||yy+12==py||yy==py+12);
  end
 endfunction
 reg extra_dots; integer dot_index;
 always @* begin
  extra_dots=1'b0;
  for(dot_index=0;dot_index<10;dot_index=dot_index+1)
   extra_dots=extra_dots|point_mark(extra_points[dot_index*32 +: 32],x,y);
 end
 wire dots=visual[19]&&face_valid&&(point_mark(p0,x,y)||point_mark(p1,x,y)||point_mark(p2,x,y)||point_mark(p3,x,y)||point_mark(p4,x,y)||(visual[21]&&extra_dots));
 wire parts=visual[20]&&face_valid&&(part_mark(p0,x,y)||part_mark(p1,x,y)||part_mark(p2,x,y)||part_mark(p3,x,y));
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin rgb_o<=0;hs_o<=0;vs_o<=0;de_o<=0;end
  else begin
   hs_o<=hs_i;vs_o<=vs_i;de_o<=de_i;
   if(!de_i)rgb_o<=0;
   else if(in_panel)rgb_o<=(panel_border||ink)?color:24'h181818;
   else if(face_edge)rgb_o<=color;
   else if(dots)rgb_o<=24'h00ffff;
   else if(parts)rgb_o<=24'hffb000;
   else rgb_o<=rgb_i;
  end
 end
endmodule
