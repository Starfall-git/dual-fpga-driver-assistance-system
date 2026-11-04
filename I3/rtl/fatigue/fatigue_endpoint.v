`timescale 1ns/1ps
/*
2026-10-26 VF board RGB snapshot and result mailbox.
Input is the unannotated 1280x720 display RGB stream. A fixed central 576x576
ROI is sampled at 3-pixel spacing into 192x192 RGB. Snapshot RAM is immutable
until the CPU requests another frame. No camera/HDMI backpressure is introduced.
Multi-bit payloads remain stable until the toggle handshake completes.
*/
/* FATIGUE-SC431-defaults adapted to the vendor 1920x1080 serialized
RGB stream; centered 576x576 ROI at (672,252), 148.5 MHz. Integration with
vendor video timings and CPU/DDR remains unverified. */
module fatigue_endpoint #(
 parameter IMAGE_WIDTH=1920,IMAGE_HEIGHT=1080,
 // Experimental larger ROI for the user's recorded 1280x720 face video.
 // It maps to x=368..879, y=0..511 in that source after 3x display scaling.
 parameter ROI_X=552,ROI_Y=0,SIDE=192,STEP=4,PIXEL_KHZ=148500
)(
 input wire cpu_clk,cpu_rst_n,pixel_clk,pixel_rst_n,
 input wire [15:0] PADDR,input wire PSEL,PENABLE,PWRITE,
 input wire [31:0] PWDATA,output reg [31:0] PRDATA,
 output wire PREADY,output wire PSLVERROR,
 input wire [23:0] rgb_i,input wire hs_i,vs_i,de_i,
 input wire [31:0] source_frame_id,
 input wire [7:0] display_frame_count,
 output wire [23:0] rgb_o,output wire hs_o,vs_o,de_o
);
 localparam TOTAL=SIDE*SIDE;
 wire reset_n=cpu_rst_n&&pixel_rst_n;
 (* async_reg="true" *) reg [1:0] pixel_run,cpu_run;
 always @(posedge pixel_clk or negedge reset_n)
  if(!reset_n)pixel_run<=0;else pixel_run<={pixel_run[0],1'b1};
 always @(posedge cpu_clk or negedge reset_n)
  if(!reset_n)cpu_run<=0;else cpu_run<={cpu_run[0],1'b1};
 reg request,done,request_seen,done_seen,pending,capturing;
 (* async_reg="true" *) reg req_meta,req_sync,done_meta,done_sync;
 reg prev_vs,prev_de;
 reg [11:0] x,y,next_x,next_y;
 reg [15:0] count,held_count,index;
 reg [31:0] frame,held_frame,milliseconds,held_ms,held_source_frame;
 reg [7:0] held_display_count,completed_display_count;
 reg [31:0] ms_div;
 reg held_error,busy,ready,error;
 reg [31:0] completed_frame,completed_ms,completed_source_frame;
 reg [15:0] completed_count;
 /* Experimental RGB332 snapshot; board resources and recognition accuracy
    must be checked before this candidate is used in a deployable build. */
 reg [7:0] mem_rgb[0:TOTAL-1];
 reg [7:0] read_rgb;
 wire [23:0] read_pixel={read_rgb[7:5],read_rgb[7:5],read_rgb[7:6],read_rgb[4:2],read_rgb[4:2],read_rgb[4:3],{4{read_rgb[1:0]}}};
 wire sample=capturing&&de_i&&x==next_x&&y==next_y&&count<TOTAL;
 always @(posedge pixel_clk) if(pixel_run[1]&&sample)
  mem_rgb[count]<={rgb_i[23:21],rgb_i[15:13],rgb_i[7:6]};
 always @(posedge cpu_clk) read_rgb<=mem_rgb[index];
 always @(posedge pixel_clk or negedge reset_n) begin
  if(!reset_n)begin
   req_meta<=0;req_sync<=0;request_seen<=0;done<=0;pending<=0;capturing<=0;
   prev_vs<=0;prev_de<=0;x<=0;y<=0;next_x<=ROI_X+STEP/2;next_y<=ROI_Y+STEP/2;
   count<=0;held_count<=0;frame<=0;held_frame<=0;held_ms<=0;held_error<=0;held_source_frame<=0;held_display_count<=0;
   milliseconds<=0;ms_div<=0;
  end else if(pixel_run[1])begin
   if(ms_div==PIXEL_KHZ-1)begin ms_div<=0;milliseconds<=milliseconds+1'b1;end
   else ms_div<=ms_div+1'b1;
   req_meta<=request;req_sync<=req_meta;prev_vs<=vs_i;prev_de<=de_i;
   if(req_sync!=request_seen)begin request_seen<=req_sync;pending<=1;end
   if(!de_i)begin x<=0;next_x<=ROI_X+STEP/2;if(prev_de)y<=y+1'b1;end
   else x<=x+1'b1;
   if(vs_i&&!prev_vs)begin
    x<=0;y<=0;frame<=frame+1'b1;
    if(capturing)begin capturing<=0;held_error<=1;held_count<=count;done<=~done;end
    else if(pending)begin
     pending<=0;capturing<=1;count<=0;next_y<=ROI_Y+STEP/2;
     held_frame<=frame+1'b1;held_ms<=milliseconds;held_error<=0;
    end
   end
   if(sample)begin
    count<=count+1'b1;next_x<=next_x+STEP;
    if(next_x==ROI_X+STEP/2+(SIDE-1)*STEP) next_y<=next_y+STEP;
    if(count==TOTAL-1)begin
     capturing<=0;held_error<=0;held_count<=TOTAL;held_source_frame<=source_frame_id;held_display_count<=display_frame_count;done<=~done;
    end
   end
  end
 end
 /* CPU staging registers are copied to held registers on COMMIT. */
 reg [31:0] stage_status,stage_closed,stage_ear,stage_box0,stage_box1;
 reg [31:0] stage_p[0:14],held_p[0:14],active_p[0:14];
 integer k;
 reg [31:0] held_status,held_closed,held_ear,held_box0,held_box1,result_ms;
 reg result_toggle,ack_toggle;
 (* async_reg="true" *) reg result_meta,result_sync,ack_meta,ack_sync;
 wire result_busy=result_toggle!=ack_sync;
 wire access=PSEL&&PENABLE;
 reg address_ok,write_ok;
 always @* begin
  PRDATA=0;address_ok=1;write_ok=0;
  case(PADDR)
   16'h0000:PRDATA=32'h46415431; // FAT1
   16'h0004:PRDATA={28'd0,result_busy,error,busy,ready};
   16'h0008:begin write_ok=!busy&&PWDATA==1;end
   16'h000c:begin PRDATA={16'd0,index};write_ok=PWDATA<TOTAL;end
   16'h0010:PRDATA={8'd0,read_pixel};
   16'h0014:PRDATA=completed_frame;
   16'h0018:PRDATA=completed_ms;
   16'h001c:PRDATA={16'd0,completed_count};
   16'h0020:begin PRDATA=stage_status;write_ok=PWDATA[2:0]<=6;end
   16'h0024:begin PRDATA=stage_closed;write_ok=1;end
   16'h0028:begin PRDATA=stage_ear;write_ok=1;end
   16'h002c:begin PRDATA=stage_box0;write_ok=1;end
   16'h0030:begin PRDATA=stage_box1;write_ok=1;end
   16'h0034:begin write_ok=!result_busy&&ready&&PWDATA==1;end
   16'h0038:PRDATA=(SIDE<<16)|STEP;
   16'h003c:PRDATA=completed_source_frame;
   16'h0040:PRDATA={24'b0,completed_display_count};
   16'h0044,16'h0048,16'h004c,16'h0050,16'h0054,16'h0058,16'h005c,16'h0060,16'h0064,16'h0068,16'h006c,16'h0070,16'h0074,16'h0078,16'h007c:begin PRDATA=stage_p[(PADDR-16'h44)>>2];write_ok=1;end
   default:address_ok=0;
  endcase
 end
 assign PREADY=1;
 assign PSLVERROR=access&&(!cpu_run[1]||!address_ok||(PWRITE&&!write_ok)||
                                (!PWRITE&&PADDR==16'h0010&&!ready));
 always @(posedge cpu_clk or negedge reset_n) begin
  if(!reset_n)begin
   request<=0;done_meta<=0;done_sync<=0;done_seen<=0;busy<=0;ready<=0;error<=0;
   completed_frame<=0;completed_ms<=0;completed_count<=0;completed_source_frame<=0;completed_display_count<=0;index<=0;
   stage_status<=0;stage_closed<=0;stage_ear<=0;stage_box0<=0;stage_box1<=0;
   for(k=0;k<15;k=k+1)begin stage_p[k]<=0;held_p[k]<=0;end
   held_status<=0;held_closed<=0;held_ear<=0;held_box0<=0;held_box1<=0;result_ms<=0;
   result_toggle<=0;ack_meta<=0;ack_sync<=0;
  end else if(cpu_run[1])begin
   done_meta<=done;done_sync<=done_meta;ack_meta<=ack_toggle;ack_sync<=ack_meta;
   if(done_sync!=done_seen)begin
    done_seen<=done_sync;busy<=0;ready<=!held_error;error<=held_error;
    completed_frame<=held_frame;completed_ms<=held_ms;completed_count<=held_count;completed_source_frame<=held_source_frame;completed_display_count<=held_display_count;
   end
   if(access&&PWRITE&&!PSLVERROR)case(PADDR)
    16'h0008:begin request<=~request;busy<=1;ready<=0;error<=0;completed_count<=0;end
    16'h000c:index<=PWDATA[15:0];
    16'h0020:stage_status<=PWDATA;
    16'h0024:stage_closed<=PWDATA;
    16'h0028:stage_ear<=PWDATA;
    16'h002c:stage_box0<=PWDATA;
    16'h0030:stage_box1<=PWDATA;
    16'h0044,16'h0048,16'h004c,16'h0050,16'h0054,16'h0058,16'h005c,16'h0060,16'h0064,16'h0068,16'h006c,16'h0070,16'h0074,16'h0078,16'h007c:stage_p[(PADDR-16'h44)>>2]<=PWDATA;
    16'h0034:begin
     for(k=0;k<15;k=k+1)held_p[k]<=stage_p[k];
     held_status<=stage_status;held_closed<=stage_closed;held_ear<=stage_ear;
     held_box0<=stage_box0;held_box1<=stage_box1;result_ms<=completed_ms;
     result_toggle<=~result_toggle;
    end
   endcase
  end
 end
 reg [31:0] active_status,active_box0,active_box1,active_ms;
 reg have_result;
 always @(posedge pixel_clk or negedge reset_n) begin
  if(!reset_n)begin
   result_meta<=0;result_sync<=0;ack_toggle<=0;active_status<=0;
   active_box0<=0;active_box1<=0;active_ms<=0;have_result<=0;
   for(integer q=0;q<15;q=q+1)active_p[q]<=0;
  end else if(pixel_run[1])begin
   result_meta<=result_toggle;result_sync<=result_meta;
   if(vs_i&&!prev_vs&&result_sync!=ack_toggle)begin
    active_status<=held_status;active_box0<=held_box0;active_box1<=held_box1;
    active_ms<=result_ms;ack_toggle<=result_sync;have_result<=1;
    for(integer q=0;q<15;q=q+1)active_p[q]<=held_p[q];
   end
  end
 end
 wire stale=have_result&&(milliseconds-active_ms)>5000;
 /* keep the physical display live while CPU/DDR is unavailable.
    Both branches have one pixel-clock latency; switch only on vertical sync. */
 reg [23:0] bypass_rgb;reg bypass_hs,bypass_vs,bypass_de,show_overlay;
 wire [23:0] overlay_rgb;wire overlay_hs,overlay_vs,overlay_de;
 always @(posedge pixel_clk or negedge pixel_rst_n) begin
  if(!pixel_rst_n)begin bypass_rgb<=0;bypass_hs<=0;bypass_vs<=0;bypass_de<=0;show_overlay<=0;end
  else begin
   bypass_rgb<=de_i?rgb_i:24'd0;bypass_hs<=hs_i;bypass_vs<=vs_i;bypass_de<=de_i;
   if(!pixel_run[1])show_overlay<=0;
   else if(vs_i)show_overlay<=1;
  end
 end
 assign rgb_o=show_overlay?overlay_rgb:bypass_rgb;
 assign hs_o=show_overlay?overlay_hs:bypass_hs;
 assign vs_o=show_overlay?overlay_vs:bypass_vs;
 assign de_o=show_overlay?overlay_de:bypass_de;
 fatigue_video_overlay #(.ROI_X(ROI_X),.ROI_Y(ROI_Y),.ROI_SIZE(SIDE*STEP)) overlay(
  .clk(pixel_clk),.rst_n(pixel_run[1]),.rgb_i(rgb_i),.hs_i(hs_i),.vs_i(vs_i),.de_i(de_i),
  .x(x),.y(y),.state(stale?3'd6:active_status[2:0]),
  .face_valid(have_result&&!stale&&active_status[8]),.box0(active_box0),.box1(active_box1),
  .extra_points({active_p[14],active_p[13],active_p[12],active_p[11],active_p[10],active_p[9],active_p[8],active_p[7],active_p[6],active_p[5]}),.visual(active_status),.p0(active_p[0]),.p1(active_p[1]),.p2(active_p[2]),.p3(active_p[3]),.p4(active_p[4]),
  .rgb_o(overlay_rgb),.hs_o(overlay_hs),.vs_o(overlay_vs),.de_o(overlay_de));
endmodule

