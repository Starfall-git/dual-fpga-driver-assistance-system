`timescale 1ns/1ps
// AI master -> complete-transaction buffer -> checked address window -> DDR fabric.
// Both modules use fabric reset; independent AI reset belongs on abort_i only.
module cnn_axi_isolated_port(

 input wire clk,rst_n,
 input wire abort_i, output wire quiescent_o,
 input wire s_avalid, output wire s_aready,
 input wire [31:0] s_addr, input wire [7:0] s_id,s_len,
 input wire [2:0] s_size, input wire [1:0] s_burst, input wire s_write,s_lock,
 input wire s_wvalid,output wire s_wready,input wire [127:0] s_wdata,
 input wire [15:0] s_wstrb,input wire s_wlast,
 output wire s_bvalid,input wire s_bready,output wire [7:0] s_bid,output wire [1:0] s_bresp,
 output wire s_rvalid,input wire s_rready,output wire [127:0] s_rdata,
 output wire [7:0] s_rid,output wire [1:0] s_rresp,output wire s_rlast,
 output wire m_avalid,input wire m_aready,
 output wire [31:0] m_addr,output wire [7:0] m_id,m_len,
 output wire [2:0] m_size,output wire [1:0] m_burst,output wire m_write,m_lock,
 output wire m_wvalid,input wire m_wready,output wire [127:0] m_wdata,
 output wire [15:0] m_wstrb,output wire m_wlast,
 input wire m_bvalid,output wire m_bready,input wire [7:0] m_bid,input wire [1:0] m_bresp,
 input wire m_rvalid,output wire m_rready,input wire [127:0] m_rdata,
 input wire [7:0] m_rid,input wire [1:0] m_rresp,input wire m_rlast


);
 wire  buffered_avalid;
 wire  buffered_aready;
 wire [31:0] buffered_addr;
 wire [7:0] buffered_id;
 wire [7:0] buffered_len;
 wire [2:0] buffered_size;
 wire [1:0] buffered_burst;
 wire  buffered_write;
 wire  buffered_lock;
 wire  buffered_wvalid;
 wire  buffered_wready;
 wire [127:0] buffered_wdata;
 wire [15:0] buffered_wstrb;
 wire  buffered_wlast;
 wire  buffered_bvalid;
 wire  buffered_bready;
 wire [7:0] buffered_bid;
 wire [1:0] buffered_bresp;
 wire  buffered_rvalid;
 wire  buffered_rready;
 wire [127:0] buffered_rdata;
 wire [7:0] buffered_rid;
 wire [1:0] buffered_rresp;
 wire  buffered_rlast;
 cnn_axi_transaction_buffer buffer(
 .clk(clk),
 .rst_n(rst_n),
 .abort_i(abort_i),
 .quiescent_o(quiescent_o),
 .s_avalid(s_avalid),
 .s_aready(s_aready),
 .s_addr(s_addr),
 .s_id(s_id),
 .s_len(s_len),
 .s_size(s_size),
 .s_burst(s_burst),
 .s_write(s_write),
 .s_lock(s_lock),
 .s_wvalid(s_wvalid),
 .s_wready(s_wready),
 .s_wdata(s_wdata),
 .s_wstrb(s_wstrb),
 .s_wlast(s_wlast),
 .s_bvalid(s_bvalid),
 .s_bready(s_bready),
 .s_bid(s_bid),
 .s_bresp(s_bresp),
 .s_rvalid(s_rvalid),
 .s_rready(s_rready),
 .s_rdata(s_rdata),
 .s_rid(s_rid),
 .s_rresp(s_rresp),
 .s_rlast(s_rlast),
 .m_avalid(buffered_avalid),
 .m_aready(buffered_aready),
 .m_addr(buffered_addr),
 .m_id(buffered_id),
 .m_len(buffered_len),
 .m_size(buffered_size),
 .m_burst(buffered_burst),
 .m_write(buffered_write),
 .m_lock(buffered_lock),
 .m_wvalid(buffered_wvalid),
 .m_wready(buffered_wready),
 .m_wdata(buffered_wdata),
 .m_wstrb(buffered_wstrb),
 .m_wlast(buffered_wlast),
 .m_bvalid(buffered_bvalid),
 .m_bready(buffered_bready),
 .m_bid(buffered_bid),
 .m_bresp(buffered_bresp),
 .m_rvalid(buffered_rvalid),
 .m_rready(buffered_rready),
 .m_rdata(buffered_rdata),
 .m_rid(buffered_rid),
 .m_rresp(buffered_rresp),
 .m_rlast(buffered_rlast));
 cnn_axi_window window(
 .clk(clk),
 .rst_n(rst_n),
 .s_avalid(buffered_avalid),
 .s_aready(buffered_aready),
 .s_addr(buffered_addr),
 .s_id(buffered_id),
 .s_len(buffered_len),
 .s_size(buffered_size),
 .s_burst(buffered_burst),
 .s_write(buffered_write),
 .s_lock(buffered_lock),
 .s_wvalid(buffered_wvalid),
 .s_wready(buffered_wready),
 .s_wdata(buffered_wdata),
 .s_wstrb(buffered_wstrb),
 .s_wlast(buffered_wlast),
 .s_bvalid(buffered_bvalid),
 .s_bready(buffered_bready),
 .s_bid(buffered_bid),
 .s_bresp(buffered_bresp),
 .s_rvalid(buffered_rvalid),
 .s_rready(buffered_rready),
 .s_rdata(buffered_rdata),
 .s_rid(buffered_rid),
 .s_rresp(buffered_rresp),
 .s_rlast(buffered_rlast),
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
 .m_bvalid(m_bvalid),
 .m_bready(m_bready),
 .m_bid(m_bid),
 .m_bresp(m_bresp),
 .m_rvalid(m_rvalid),
 .m_rready(m_rready),
 .m_rdata(m_rdata),
 .m_rid(m_rid),
 .m_rresp(m_rresp),
 .m_rlast(m_rlast));
endmodule
