`timescale 1ns/1ps
// Experimental complete frame data-plane (UDP payload stream, not Ethernet).
// User MAC must gate/reject new frames if input_ready is low; dual resets must
// be asserted together. No integration with the existing AI top is implied.
module fatigue_network_video #(
 parameter WIDTH=640,HEIGHT=360,CHUNK=1152,FIFO_ADDR=10,SCALE=3,
 parameter H_FRONT=88,H_SYNC=44,H_BACK=148,V_FRONT=4,V_SYNC=5,V_BACK=36
)(
 input wire rx_clk,ddr_clk,pix_clk,rx_rst_n,ddr_rst_n,pix_rst_n,
 input wire in_valid,in_sop,in_eop,input wire [7:0] in_data,output wire in_ready,
 output wire [23:0] rgb,output wire hs,vs,de,display_valid,output wire [31:0] display_id,
 output wire write_error,output wire read_error,output wire underrun,output wire frame_overflow,
 output wire [7:0] diag_network,
 output wire m_avalid,input wire m_aready,output wire [31:0] m_addr,output wire [3:0] m_id,output wire [7:0] m_len,output wire [2:0] m_size,output wire [1:0] m_burst,output wire m_write,m_lock,
 output wire m_wvalid,input wire m_wready,output wire [127:0] m_wdata,output wire [15:0] m_wstrb,output wire m_wlast,output wire [3:0] m_wid,
 input wire m_bvalid,output wire m_bready,input wire [1:0] m_bresp,
 input wire m_rvalid,output wire m_rready,input wire [127:0] m_rdata,input wire [1:0] m_rresp,input wire m_rlast
);
 wire wb,db,writable,accepted,commit,commit_error;wire [31:0] cid;wire vb,scan_idle;
 wire diag_parser_error,diag_parser_complete,diag_fifo_pressure;wire [3:0] diag_errors;
 reg diag_commit,diag_commit_error,diag_overflow;
 always @(posedge ddr_clk or negedge ddr_rst_n)begin
  if(!ddr_rst_n)begin diag_commit<=0;diag_commit_error<=0;diag_overflow<=0;end
  else begin if(commit)diag_commit<=1;if(commit_error)diag_commit_error<=1;if(frame_overflow)diag_overflow<=1;end
 end
 assign diag_network={writable,diag_commit,diag_fifo_pressure,diag_errors,display_valid};
 (* async_reg="true" *) reg idle_m,idle_s,writable_m,writable_s;
 always @(posedge pix_clk or negedge pix_rst_n)if(!pix_rst_n)begin idle_m<=0;idle_s<=0;end else begin idle_m<=scan_idle;idle_s<=idle_m;end
 always @(posedge rx_clk or negedge rx_rst_n)if(!rx_rst_n)begin writable_m<=0;writable_s<=0;end else begin writable_m<=writable;writable_s<=writable_m;end
 fatigue_frame_banks banks(.ddr_clk(ddr_clk),.ddr_rst_n(ddr_rst_n),.complete(commit),.completed_id(cid),.write_bank(wb),.writable(writable),.accepted(accepted),.overflow(frame_overflow),
 .pix_clk(pix_clk),.pix_rst_n(pix_rst_n),.frame_start(vb),.old_bank_drained(idle_s),.display_bank(db),.display_valid(display_valid),.display_id(display_id));
 wire [2:0] av,ardy,wv,wrdy,wl,bv,br,rv,rr,rl,ww,lock;
 wire [95:0] a;wire [23:0] ids,lens,bids,rids;wire [8:0] sizes;wire [5:0] bursts,breps,rreps;
 wire [383:0] wd,rd;wire [47:0] strb;wire [31:0] scan_addr,net_addr;wire [7:0] scan_len,net_len;wire [2:0] scan_size,net_size;wire [1:0] scan_burst,net_burst;
 wire scan_write,net_write,scan_ready,net_ready;
 assign in_ready=net_ready&&writable_s;
 fatigue_net_ingest #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.CHUNK(CHUNK),.FIFO_ADDR(FIFO_ADDR)) ingest(
 .rx_clk(rx_clk),.rx_rst_n(rx_rst_n),.in_valid(in_valid&&writable_s),.in_sop(in_sop),.in_eop(in_eop),.in_data(in_data),.in_ready(net_ready),
 .diag_parser_error(diag_parser_error),.diag_parser_complete(diag_parser_complete),.diag_fifo_pressure(diag_fifo_pressure),
 .diag_errors(diag_errors),
 .ddr_clk(ddr_clk),.ddr_rst_n(ddr_rst_n),.bank(wb),.commit_ok(commit),.commit_error(commit_error),.committed_id(cid),
 .avalid(av[1]),.aready(ardy[1]),.addr(net_addr),.len(net_len),.size(net_size),.burst(net_burst),.write(net_write),
 .wvalid(wv[1]),.wready(wrdy[1]),.wdata(wd[255:128]),.wstrb(strb[31:16]),.wlast(wl[1]),.bvalid(bv[1]),.bready(br[1]),.bresp(breps[3:2]));
 assign write_error=commit_error;
 fatigue_video_scan #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.SCALE(SCALE),.H_FRONT(H_FRONT),.H_SYNC(H_SYNC),.H_BACK(H_BACK),.V_FRONT(V_FRONT),.V_SYNC(V_SYNC),.V_BACK(V_BACK)) scan(
 .pix_clk(pix_clk),.pix_rst_n(pix_rst_n),.ddr_clk(ddr_clk),.ddr_rst_n(ddr_rst_n),.display_bank(db),.display_valid(display_valid),
 .vblank_start(vb),.rgb(rgb),.hs(hs),.vs(vs),.de(de),.avalid(av[0]),.aready(ardy[0]),.addr(scan_addr),.len(scan_len),.size(scan_size),.burst(scan_burst),.write(scan_write),
 .rvalid(rv[0]),.rready(rr[0]),.rdata(rd[127:0]),.rresp(rreps[1:0]),.rlast(rl[0]),.underrun(underrun),.read_error(read_error),.read_idle(scan_idle));
 assign av[2]=0;assign wv[2]=0;assign wl[2]=0;assign br[2]=1;assign rr[2]=1;
 assign wv[0]=0;assign wl[0]=0;assign br[0]=1;assign rr[1]=1;
 assign a={32'b0,net_addr,scan_addr};assign ids=0;assign lens={8'b0,net_len,scan_len};assign sizes={3'b0,net_size,scan_size};assign bursts={2'b0,net_burst,scan_burst};
 assign ww={1'b0,net_write,scan_write};assign lock=0;assign wd[127:0]=0;assign wd[383:256]=0;assign strb[15:0]=0;assign strb[47:32]=0;
 cnn_ddr_arbiter arb(.clk(ddr_clk),.rst_n(ddr_rst_n),.s_avalid(av),.s_aready(ardy),.s_addr(a),.s_id(ids),.s_len(lens),.s_size(sizes),.s_burst(bursts),.s_write(ww),.s_lock(lock),
 .s_wvalid(wv),.s_wready(wrdy),.s_wdata(wd),.s_wstrb(strb),.s_wlast(wl),.s_bvalid(bv),.s_bready(br),.s_bid(bids),.s_bresp(breps),
 .s_rvalid(rv),.s_rready(rr),.s_rdata(rd),.s_rid(rids),.s_rresp(rreps),.s_rlast(rl),
 .m_avalid(m_avalid),.m_aready(m_aready),.m_addr(m_addr),.m_id(m_id),.m_len(m_len),.m_size(m_size),.m_burst(m_burst),.m_write(m_write),.m_lock(m_lock),
 .m_wvalid(m_wvalid),.m_wready(m_wready),.m_wdata(m_wdata),.m_wstrb(m_wstrb),.m_wlast(m_wlast),.m_wid(m_wid),
 .m_bvalid(m_bvalid),.m_bready(m_bready),.m_bresp(m_bresp),.m_rvalid(m_rvalid),.m_rready(m_rready),.m_rdata(m_rdata),.m_rresp(m_rresp),.m_rlast(m_rlast),.idle_o());
endmodule
