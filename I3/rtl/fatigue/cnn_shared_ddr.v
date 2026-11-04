`timescale 1ns/1ps
// Lane 0 = existing video physical addresses; lanes 1/2 = CPU/TinyML logical addresses.
// AI abort never resets this module or DDR: buffered transactions drain first.
module cnn_shared_ddr(

 input wire clk,rst_n,
 input wire [1:0] ai_abort,output wire [1:0] ai_quiescent,
 input wire [2:0] s_avalid,output wire [2:0] s_aready,
 input wire [95:0] s_addr,input wire [23:0] s_id,s_len,
 input wire [8:0] s_size,input wire [5:0] s_burst,input wire [2:0] s_write,s_lock,
 input wire [2:0] s_wvalid,output wire [2:0] s_wready,
 input wire [383:0] s_wdata,input wire [47:0] s_wstrb,input wire [2:0] s_wlast,
 output wire [2:0] s_bvalid,input wire [2:0] s_bready,
 output wire [23:0] s_bid,output wire [5:0] s_bresp,
 output wire [2:0] s_rvalid,input wire [2:0] s_rready,
 output wire [383:0] s_rdata,output wire [23:0] s_rid,
 output wire [5:0] s_rresp,output wire [2:0] s_rlast,
 output wire m_avalid,input wire m_aready,output wire [31:0] m_addr,
 output wire [3:0] m_id,output wire [7:0] m_len,
 output wire [2:0] m_size,output wire [1:0] m_burst,output wire m_write,m_lock,
 output wire m_wvalid,input wire m_wready,output wire [127:0] m_wdata,
 output wire [15:0] m_wstrb,output wire m_wlast,output wire [3:0] m_wid,
 input wire m_bvalid,output wire m_bready,input wire [1:0] m_bresp,
 input wire m_rvalid,output wire m_rready,input wire [127:0] m_rdata,
 input wire [1:0] m_rresp,input wire m_rlast,
 output wire idle_o

);
 wire [2:0] fabric_s_avalid;
 wire [2:0] fabric_s_aready;
 wire [95:0] fabric_s_addr;
 wire [23:0] fabric_s_id;
 wire [23:0] fabric_s_len;
 wire [8:0] fabric_s_size;
 wire [5:0] fabric_s_burst;
 wire [2:0] fabric_s_write;
 wire [2:0] fabric_s_lock;
 wire [2:0] fabric_s_wvalid;
 wire [2:0] fabric_s_wready;
 wire [383:0] fabric_s_wdata;
 wire [47:0] fabric_s_wstrb;
 wire [2:0] fabric_s_wlast;
 wire [2:0] fabric_s_bvalid;
 wire [2:0] fabric_s_bready;
 wire [23:0] fabric_s_bid;
 wire [5:0] fabric_s_bresp;
 wire [2:0] fabric_s_rvalid;
 wire [2:0] fabric_s_rready;
 wire [383:0] fabric_s_rdata;
 wire [23:0] fabric_s_rid;
 wire [5:0] fabric_s_rresp;
 wire [2:0] fabric_s_rlast;
 assign fabric_s_avalid[0:0]=s_avalid[0:0];
 assign s_aready[0:0]=fabric_s_aready[0:0];
 assign fabric_s_addr[31:0]=s_addr[31:0];
 assign fabric_s_id[7:0]=s_id[7:0];
 assign fabric_s_len[7:0]=s_len[7:0];
 assign fabric_s_size[2:0]=s_size[2:0];
 assign fabric_s_burst[1:0]=s_burst[1:0];
 assign fabric_s_write[0:0]=s_write[0:0];
 assign fabric_s_lock[0:0]=s_lock[0:0];
 assign fabric_s_wvalid[0:0]=s_wvalid[0:0];
 assign s_wready[0:0]=fabric_s_wready[0:0];
 assign fabric_s_wdata[127:0]=s_wdata[127:0];
 assign fabric_s_wstrb[15:0]=s_wstrb[15:0];
 assign fabric_s_wlast[0:0]=s_wlast[0:0];
 assign s_bvalid[0:0]=fabric_s_bvalid[0:0];
 assign fabric_s_bready[0:0]=s_bready[0:0];
 assign s_bid[7:0]=fabric_s_bid[7:0];
 assign s_bresp[1:0]=fabric_s_bresp[1:0];
 assign s_rvalid[0:0]=fabric_s_rvalid[0:0];
 assign fabric_s_rready[0:0]=s_rready[0:0];
 assign s_rdata[127:0]=fabric_s_rdata[127:0];
 assign s_rid[7:0]=fabric_s_rid[7:0];
 assign s_rresp[1:0]=fabric_s_rresp[1:0];
 assign s_rlast[0:0]=fabric_s_rlast[0:0];
 genvar ai;
 generate for(ai=1;ai<3;ai=ai+1)begin:ai_port
  cnn_axi_isolated_port port(
   .clk(clk),
   .rst_n(rst_n),
   .abort_i(ai_abort[ai-1]),
   .quiescent_o(ai_quiescent[ai-1]),
   .s_avalid(s_avalid[ai*1+:1]),
   .m_avalid(fabric_s_avalid[ai*1+:1]),
   .s_aready(s_aready[ai*1+:1]),
   .m_aready(fabric_s_aready[ai*1+:1]),
   .s_addr(s_addr[ai*32+:32]),
   .m_addr(fabric_s_addr[ai*32+:32]),
   .s_id(s_id[ai*8+:8]),
   .m_id(fabric_s_id[ai*8+:8]),
   .s_len(s_len[ai*8+:8]),
   .m_len(fabric_s_len[ai*8+:8]),
   .s_size(s_size[ai*3+:3]),
   .m_size(fabric_s_size[ai*3+:3]),
   .s_burst(s_burst[ai*2+:2]),
   .m_burst(fabric_s_burst[ai*2+:2]),
   .s_write(s_write[ai*1+:1]),
   .m_write(fabric_s_write[ai*1+:1]),
   .s_lock(s_lock[ai*1+:1]),
   .m_lock(fabric_s_lock[ai*1+:1]),
   .s_wvalid(s_wvalid[ai*1+:1]),
   .m_wvalid(fabric_s_wvalid[ai*1+:1]),
   .s_wready(s_wready[ai*1+:1]),
   .m_wready(fabric_s_wready[ai*1+:1]),
   .s_wdata(s_wdata[ai*128+:128]),
   .m_wdata(fabric_s_wdata[ai*128+:128]),
   .s_wstrb(s_wstrb[ai*16+:16]),
   .m_wstrb(fabric_s_wstrb[ai*16+:16]),
   .s_wlast(s_wlast[ai*1+:1]),
   .m_wlast(fabric_s_wlast[ai*1+:1]),
   .s_bvalid(s_bvalid[ai*1+:1]),
   .m_bvalid(fabric_s_bvalid[ai*1+:1]),
   .s_bready(s_bready[ai*1+:1]),
   .m_bready(fabric_s_bready[ai*1+:1]),
   .s_bid(s_bid[ai*8+:8]),
   .m_bid(fabric_s_bid[ai*8+:8]),
   .s_bresp(s_bresp[ai*2+:2]),
   .m_bresp(fabric_s_bresp[ai*2+:2]),
   .s_rvalid(s_rvalid[ai*1+:1]),
   .m_rvalid(fabric_s_rvalid[ai*1+:1]),
   .s_rready(s_rready[ai*1+:1]),
   .m_rready(fabric_s_rready[ai*1+:1]),
   .s_rdata(s_rdata[ai*128+:128]),
   .m_rdata(fabric_s_rdata[ai*128+:128]),
   .s_rid(s_rid[ai*8+:8]),
   .m_rid(fabric_s_rid[ai*8+:8]),
   .s_rresp(s_rresp[ai*2+:2]),
   .m_rresp(fabric_s_rresp[ai*2+:2]),
   .s_rlast(s_rlast[ai*1+:1]),
   .m_rlast(fabric_s_rlast[ai*1+:1]));
 end endgenerate
 cnn_ddr_arbiter arbiter(
 .clk(clk),
 .rst_n(rst_n),
 .s_avalid(fabric_s_avalid),
 .s_aready(fabric_s_aready),
 .s_addr(fabric_s_addr),
 .s_id(fabric_s_id),
 .s_len(fabric_s_len),
 .s_size(fabric_s_size),
 .s_burst(fabric_s_burst),
 .s_write(fabric_s_write),
 .s_lock(fabric_s_lock),
 .s_wvalid(fabric_s_wvalid),
 .s_wready(fabric_s_wready),
 .s_wdata(fabric_s_wdata),
 .s_wstrb(fabric_s_wstrb),
 .s_wlast(fabric_s_wlast),
 .s_bvalid(fabric_s_bvalid),
 .s_bready(fabric_s_bready),
 .s_bid(fabric_s_bid),
 .s_bresp(fabric_s_bresp),
 .s_rvalid(fabric_s_rvalid),
 .s_rready(fabric_s_rready),
 .s_rdata(fabric_s_rdata),
 .s_rid(fabric_s_rid),
 .s_rresp(fabric_s_rresp),
 .s_rlast(fabric_s_rlast),
 .m_avalid(m_avalid),
 .m_aready(m_aready),
 .m_addr(m_addr),
 .m_id(m_id),
 .m_len(m_len),
 .m_size(m_size),
 .m_burst(m_burst),
 .m_write(m_write),
 .m_lock(m_lock),
 .m_wvalid(m_wvalid),
 .m_wready(m_wready),
 .m_wdata(m_wdata),
 .m_wstrb(m_wstrb),
 .m_wlast(m_wlast),
 .m_wid(m_wid),
 .m_bvalid(m_bvalid),
 .m_bready(m_bready),
 .m_bresp(m_bresp),
 .m_rvalid(m_rvalid),
 .m_rready(m_rready),
 .m_rdata(m_rdata),
 .m_rresp(m_rresp),
 .m_rlast(m_rlast),
 .idle_o(idle_o));
endmodule
