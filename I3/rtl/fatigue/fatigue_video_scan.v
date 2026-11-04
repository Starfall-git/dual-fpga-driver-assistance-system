`timescale 1ns/1ps
// Experimental DDR scanout. Reset both domains together; buffer ownership
// and bandwidth guarantees remain external responsibilities.
module fatigue_video_scan #(
 parameter WIDTH=640,HEIGHT=360,SCALE=3,
 parameter H_ACTIVE=WIDTH*SCALE,H_FRONT=88,H_SYNC=44,H_BACK=148,
 parameter V_ACTIVE=HEIGHT*SCALE,V_FRONT=4,V_SYNC=5,V_BACK=36,
 parameter BEATS=(WIDTH+7)/8,ADDR=$clog2(BEATS)
)(
 input wire pix_clk,pix_rst_n,ddr_clk,ddr_rst_n,
 input wire display_bank,display_valid,
 output wire vblank_start,output wire [23:0] rgb,output reg hs,vs,de,
 output wire avalid,input wire aready,output wire [31:0] addr,output wire [7:0] len,output wire [2:0] size,output wire [1:0] burst,output wire write,
 input wire rvalid,output wire rready,input wire [127:0] rdata,input wire [1:0] rresp,input wire rlast,
 output reg underrun,output reg read_error,output wire read_idle
);
 wire t_de,t_hs,t_vs,t_fs,t_ls;wire [15:0] x,y,sx,sy;
 fatigue_video_timing #(.H_ACTIVE(H_ACTIVE),.H_FRONT(H_FRONT),.H_SYNC(H_SYNC),.H_BACK(H_BACK),.V_ACTIVE(V_ACTIVE),.V_FRONT(V_FRONT),.V_SYNC(V_SYNC),.V_BACK(V_BACK),.SCALE(SCALE)) timing(
 .clk(pix_clk),.rst_n(pix_rst_n),.de(t_de),.hs(t_hs),.vs(t_vs),.frame_start(t_fs),.line_start(t_ls),.x(x),.y(y),.source_x(sx),.source_y(sy));
 assign vblank_start=(x==0 && y==V_ACTIVE);
 reg request_toggle,request_bank;reg [15:0] request_row;reg request_frame;
 reg ack_toggle,done_good;(* async_reg="true" *) reg ack_m,ack_s,req_m,req_s;
 reg ack_seen;reg [1:0] ready_valid;reg [15:0] ready_row[0:1];reg ready_frame[0:1];
 reg line_good;
 wire row_good=display_valid && ready_valid[sy[0]] && ready_row[sy[0]]==sy && ready_frame[sy[0]]==display_bank;
 wire prefetch_zero=(x==0 && y==V_ACTIVE+1);
 wire prefetch_next=(t_ls && sy+1<HEIGHT && (y%SCALE)==0);
 always @(posedge pix_clk or negedge pix_rst_n)begin
  if(!pix_rst_n)begin request_toggle<=0;request_bank<=0;request_row<=0;request_frame<=0;ack_m<=0;ack_s<=0;ack_seen<=0;ready_valid<=0;ready_row[0]<=0;ready_row[1]<=0;ready_frame[0]<=0;ready_frame[1]<=0;line_good<=0;underrun<=0;end
  else begin
   ack_m<=ack_toggle;ack_s<=ack_m;
   if(ack_s!=ack_seen)begin ack_seen<=ack_s;ready_valid[request_bank]<=done_good;ready_row[request_bank]<=request_row;ready_frame[request_bank]<=request_frame;end
   if(t_ls)begin line_good<=row_good;if(!row_good&&display_valid)underrun<=1;end
   if((prefetch_zero||prefetch_next) && display_valid && request_toggle==ack_s)begin
    request_row<=prefetch_zero?0:sy+1'b1;request_bank<=prefetch_zero?0:~sy[0];request_frame<=display_bank;request_toggle<=~request_toggle;
   end
  end
 end
 reg [15:0] held_row;reg held_bank;reg reader_request;reg busy;
 wire reader_ready,line_done,line_error,beat_valid;
 reg read_idle_q;
 assign read_idle=read_idle_q;
 always @(posedge ddr_clk or negedge ddr_rst_n)if(!ddr_rst_n)read_idle_q<=0;else read_idle_q<=!busy && reader_ready && req_s==ack_toggle;wire [7:0] beat_index;wire [127:0] beat_data;
 always @(posedge ddr_clk or negedge ddr_rst_n)begin
  if(!ddr_rst_n)begin req_m<=0;req_s<=0;ack_toggle<=0;held_row<=0;held_bank<=0;reader_request<=0;busy<=0;read_error<=0;done_good<=0;end
  else begin
   req_m<=request_toggle;req_s<=req_m;reader_request<=0;
   if(req_s!=ack_toggle && !busy && reader_ready)begin held_row<=request_row;held_bank<=request_frame;reader_request<=1;busy<=1;end
   if(line_done||line_error)begin if(line_error)read_error<=1;done_good<=line_done;busy<=0;ack_toggle<=req_s;end
  end
 end
 fatigue_line_reader #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) reader(.clk(ddr_clk),.rst_n(ddr_rst_n),
 .request(reader_request),.bank(held_bank),.row(held_row),.request_ready(reader_ready),.avalid(avalid),.aready(aready),.addr(addr),.len(len),.size(size),.burst(burst),.write(write),
 .rvalid(rvalid),.rready(rready),.rdata(rdata),.rresp(rresp),.rlast(rlast),.beat_valid(beat_valid),.beat_ready(1'b1),.beat_index(beat_index),.beat_data(beat_data),.line_done(line_done),.line_error(line_error));
 wire [15:0] pixel_raw;wire pixel_ok;
 fatigue_line_store #(.WIDTH(WIDTH)) store(.wclk(ddr_clk),.wr_valid(beat_valid),.wr_bank(held_row[0]),.wr_beat(beat_index[ADDR-1:0]),.wr_data(beat_data),
 .pclk(pix_clk),.pr_valid(t_de && (x==0?row_good:line_good)),.pr_bank(sy[0]),.pr_x(sx[$clog2(WIDTH)-1:0]),.pixel_valid(pixel_ok),.pixel(pixel_raw));
 wire [15:0] pixel565={pixel_raw[7:0],pixel_raw[15:8]};
 assign rgb=pixel_ok ? {{pixel565[15:11],pixel565[15:13]},{pixel565[10:5],pixel565[10:9]},{pixel565[4:0],pixel565[4:2]}} : 24'b0;
 reg hs_d,vs_d,de_d;
 always @(posedge pix_clk or negedge pix_rst_n)begin
  if(!pix_rst_n)begin hs_d<=0;vs_d<=0;de_d<=0;hs<=0;vs<=0;de<=0;end
  else begin hs_d<=t_hs;vs_d<=t_vs;de_d<=t_de;hs<=hs_d;vs<=vs_d;de<=de_d;
  end
 end
endmodule
