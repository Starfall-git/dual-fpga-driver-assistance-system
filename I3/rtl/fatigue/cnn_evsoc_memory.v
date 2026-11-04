// Real Sapphire/TinyML instance -> isolated shared DDR; all memory ports at clk_96.
// fabric_rst_n is GLOBAL; ai_reset may assert independently. Parent must wait
// for ai_quiescent before releasing a restarted CPU. No camera DMA is added here.
/* FATIGUE-SC431-forward 1080p geometry and pixel clock to the snapshot. */
module cnn_evsoc_memory #(parameter IMAGE_WIDTH=1920,IMAGE_HEIGHT=1080,PIXEL_KHZ=148500)(
input wire  clk_96,
input wire  ai_reset,
input wire  ai_online,
input wire  uart_clk,
input wire  uart_rst_n,
input wire [1:0] requested,
output wire [1:0] applied,
output wire  available,
output wire  inference_enable,
output wire  system_reset,
output wire  memory_reset,
output wire  peripheral_reset,
input wire  pixel_clk,
input wire  pixel_rst_n,
input wire [31:0] source_frame_id,
input wire [7:0] display_frame_count,
input wire  cam_clk,
input wire  cam_rst_n,
input wire  cam_frame_valid,
input wire  cam_pixel_valid,
input wire [7:0] cam_gray,
input wire [23:0] rgb_i,
input wire  hs_i,
input wire  vs_i,
input wire  de_i,
output wire [23:0] rgb_o,
output wire  hs_o,
output wire  vs_o,
output wire  de_o,
output wire [31:0] displayed_source_frame,
input wire  jtagCtrl_enable,
input wire  jtagCtrl_tdi,
input wire  jtagCtrl_capture,
input wire  jtagCtrl_shift,
input wire  jtagCtrl_update,
input wire  jtagCtrl_reset,
output wire  jtagCtrl_tdo,
input wire  jtagCtrl_tck,
input wire  system_spi_0_io_data_0_read,
output wire  system_spi_0_io_data_0_write,
output wire  system_spi_0_io_data_0_writeEnable,
input wire  system_spi_0_io_data_1_read,
output wire  system_spi_0_io_data_1_write,
output wire  system_spi_0_io_data_1_writeEnable,
input wire  system_spi_0_io_data_2_read,
output wire  system_spi_0_io_data_2_write,
output wire  system_spi_0_io_data_2_writeEnable,
input wire  system_spi_0_io_data_3_read,
output wire  system_spi_0_io_data_3_write,
output wire  system_spi_0_io_data_3_writeEnable,
output wire  system_spi_0_io_sclk_write,
output wire  system_uart_0_io_txd,
input wire  system_uart_0_io_rxd,
output wire [0:0] system_spi_0_io_ss,
input wire  fabric_rst_n,
output wire [1:0] ai_quiescent,
output reg [7:0] diag_cpu_read,
input wire  video_avalid,
output wire  video_aready,
input wire [31:0] video_addr,
input wire [7:0] video_id,
input wire [7:0] video_len,
input wire [2:0] video_size,
input wire [1:0] video_burst,
input wire  video_write,
input wire  video_lock,
input wire  video_wvalid,
output wire  video_wready,
input wire [127:0] video_wdata,
input wire [15:0] video_wstrb,
input wire  video_wlast,
output wire  video_bvalid,
input wire  video_bready,
output wire [7:0] video_bid,
output wire [1:0] video_bresp,
output wire  video_rvalid,
input wire  video_rready,
output wire [127:0] video_rdata,
output wire [7:0] video_rid,
output wire [1:0] video_rresp,
output wire  video_rlast,
output wire  m_avalid,
input wire  m_aready,
output wire [31:0] m_addr,
output wire [3:0] m_id,
output wire [7:0] m_len,
output wire [2:0] m_size,
output wire [1:0] m_burst,
output wire  m_write,
output wire  m_lock,
output wire  m_wvalid,
input wire  m_wready,
output wire [127:0] m_wdata,
output wire [15:0] m_wstrb,
output wire  m_wlast,
output wire [3:0] m_wid,
input wire  m_bvalid,
output wire  m_bready,
input wire [1:0] m_bresp,
input wire  m_rvalid,
output wire  m_rready,
input wire [127:0] m_rdata,
input wire [1:0] m_rresp,
input wire  m_rlast,
output wire  idle_o
);
wire [15:0] cpu_w_payload_strb;
wire [127:0] cpu_w_payload_data;
wire [7:0] cpu_w_payload_id;
wire  cpu_r_payload_last;
wire [1:0] cpu_r_payload_resp;
wire [7:0] cpu_r_payload_id;
wire [127:0] cpu_r_payload_data;
wire  cpu_r_ready;
wire  cpu_r_valid;
wire [1:0] cpu_b_payload_resp;
wire [7:0] cpu_b_payload_id;
wire  cpu_b_ready;
wire  cpu_b_valid;
wire  cpu_w_payload_last;
wire  cpu_w_ready;
wire  cpu_w_valid;
wire  cpu_arw_payload_write;
wire [2:0] cpu_arw_payload_prot;
wire [3:0] cpu_arw_payload_qos;
wire [3:0] cpu_arw_payload_cache;
wire  cpu_arw_payload_lock;
wire [1:0] cpu_arw_payload_burst;
wire [2:0] cpu_arw_payload_size;
wire [7:0] cpu_arw_payload_len;
wire [3:0] cpu_arw_payload_region;
wire [7:0] cpu_arw_payload_id;
wire [31:0] cpu_arw_payload_addr;
// Sticky diagnostic of CPU-side DDR reads. Clear only on a fabric reset.
always @(posedge clk_96 or negedge fabric_rst_n) begin
 if(!fabric_rst_n)diag_cpu_read<=0;
 else begin
  if(cpu_arw_valid && cpu_arw_ready && !cpu_arw_payload_write) begin
   diag_cpu_read[0]<=1;
   diag_cpu_read[1]<=cpu_arw_payload_size==4;
   diag_cpu_read[2]<=cpu_arw_payload_burst==1;
   diag_cpu_read[3]<=!cpu_arw_payload_lock;
   diag_cpu_read[6]<=cpu_arw_payload_addr==32'h1000;
  end
  if(cpu_r_valid && cpu_r_ready) begin
   diag_cpu_read[4]<=1;
   diag_cpu_read[5]<=cpu_r_payload_resp==0;
   if(cpu_r_payload_resp!=0)diag_cpu_read[7]<=1;
  end
 end
end
wire  cpu_arw_ready;
wire  cpu_arw_valid;
wire  tiny_awvalid;
wire  tiny_awready;
wire [31:0] tiny_awaddr;
wire [7:0] tiny_awlen;
wire [7:0] tiny_awid;
wire [2:0] tiny_awsize;
wire [1:0] tiny_awburst;
wire [0:0] tiny_awlock;
wire [3:0] tiny_awcache;
wire [2:0] tiny_awprot;
wire  tiny_wvalid;
wire  tiny_wready;
wire [127:0] tiny_wdata;
wire [15:0] tiny_wstrb;
wire  tiny_wlast;
wire  tiny_bvalid;
wire  tiny_bready;
wire [1:0] tiny_bresp;
wire  tiny_arvalid;
wire  tiny_arready;
wire [31:0] tiny_araddr;
wire [7:0] tiny_arlen;
wire [7:0] tiny_arid;
wire [2:0] tiny_arsize;
wire [1:0] tiny_arburst;
wire [0:0] tiny_arlock;
wire [3:0] tiny_arcache;
wire [2:0] tiny_arprot;
wire  tiny_rvalid;
wire  tiny_rready;
wire [127:0] tiny_rdata;
wire  tiny_rlast;
wire [1:0] tiny_rresp;
wire  accel_avalid;
wire [2:0] bus_avalid;
wire  accel_aready;
wire [2:0] bus_aready;
wire [31:0] accel_addr;
wire [95:0] bus_addr;
wire [7:0] accel_id;
wire [23:0] bus_id;
wire [7:0] accel_len;
wire [23:0] bus_len;
wire [2:0] accel_size;
wire [8:0] bus_size;
wire [1:0] accel_burst;
wire [5:0] bus_burst;
wire  accel_write;
wire [2:0] bus_write;
wire  accel_lock;
wire [2:0] bus_lock;
wire  accel_wvalid;
wire [2:0] bus_wvalid;
wire  accel_wready;
wire [2:0] bus_wready;
wire [127:0] accel_wdata;
wire [383:0] bus_wdata;
wire [15:0] accel_wstrb;
wire [47:0] bus_wstrb;
wire  accel_wlast;
wire [2:0] bus_wlast;
wire  accel_bvalid;
wire [2:0] bus_bvalid;
wire  accel_bready;
wire [2:0] bus_bready;
wire [7:0] accel_bid;
wire [23:0] bus_bid;
wire [1:0] accel_bresp;
wire [5:0] bus_bresp;
wire  accel_rvalid;
wire [2:0] bus_rvalid;
wire  accel_rready;
wire [2:0] bus_rready;
wire [127:0] accel_rdata;
wire [383:0] bus_rdata;
wire [7:0] accel_rid;
wire [23:0] bus_rid;
wire [1:0] accel_rresp;
wire [5:0] bus_rresp;
wire  accel_rlast;
wire [2:0] bus_rlast;
wire [1:0] accel_lock_full;
wire [7:0] unused_wid,unused_bid,unused_rid;
cnn_soc_subsystem #(.IMAGE_WIDTH(IMAGE_WIDTH),.IMAGE_HEIGHT(IMAGE_HEIGHT),.PIXEL_KHZ(PIXEL_KHZ)) soc(
.clk_96(clk_96),
.ai_reset(ai_reset),
.ai_online(ai_online),
.uart_clk(uart_clk),
.uart_rst_n(uart_rst_n),
.requested(requested),
.applied(applied),
.available(available),
.inference_enable(inference_enable),
.system_reset(system_reset),
.memory_reset(memory_reset),
.peripheral_reset(peripheral_reset),
.pixel_clk(pixel_clk),
.pixel_rst_n(pixel_rst_n),
.source_frame_id(source_frame_id),
.display_frame_count(display_frame_count),
.cam_clk(cam_clk),
.cam_rst_n(cam_rst_n),
.cam_frame_valid(cam_frame_valid),
.cam_pixel_valid(cam_pixel_valid),
.cam_gray(cam_gray),
.rgb_i(rgb_i),
.hs_i(hs_i),
.vs_i(vs_i),
.de_i(de_i),
.rgb_o(rgb_o),
.hs_o(hs_o),
.vs_o(vs_o),
.de_o(de_o),
.displayed_source_frame(displayed_source_frame),
.cpu_w_payload_strb(cpu_w_payload_strb),
.cpu_w_payload_data(cpu_w_payload_data),
.jtagCtrl_enable(jtagCtrl_enable),
.jtagCtrl_tdi(jtagCtrl_tdi),
.jtagCtrl_capture(jtagCtrl_capture),
.jtagCtrl_shift(jtagCtrl_shift),
.jtagCtrl_update(jtagCtrl_update),
.jtagCtrl_reset(jtagCtrl_reset),
.jtagCtrl_tdo(jtagCtrl_tdo),
.jtagCtrl_tck(jtagCtrl_tck),
.cpu_w_payload_id(cpu_w_payload_id),
.cpu_r_payload_last(cpu_r_payload_last),
.cpu_r_payload_resp(cpu_r_payload_resp),
.cpu_r_payload_id(cpu_r_payload_id),
.cpu_r_payload_data(cpu_r_payload_data),
.cpu_r_ready(cpu_r_ready),
.cpu_r_valid(cpu_r_valid),
.cpu_b_payload_resp(cpu_b_payload_resp),
.cpu_b_payload_id(cpu_b_payload_id),
.cpu_b_ready(cpu_b_ready),
.cpu_b_valid(cpu_b_valid),
.cpu_w_payload_last(cpu_w_payload_last),
.cpu_w_ready(cpu_w_ready),
.cpu_w_valid(cpu_w_valid),
.cpu_arw_payload_write(cpu_arw_payload_write),
.cpu_arw_payload_prot(cpu_arw_payload_prot),
.cpu_arw_payload_qos(cpu_arw_payload_qos),
.cpu_arw_payload_cache(cpu_arw_payload_cache),
.cpu_arw_payload_lock(cpu_arw_payload_lock),
.cpu_arw_payload_burst(cpu_arw_payload_burst),
.cpu_arw_payload_size(cpu_arw_payload_size),
.cpu_arw_payload_len(cpu_arw_payload_len),
.cpu_arw_payload_region(cpu_arw_payload_region),
.cpu_arw_payload_id(cpu_arw_payload_id),
.cpu_arw_payload_addr(cpu_arw_payload_addr),
.cpu_arw_ready(cpu_arw_ready),
.cpu_arw_valid(cpu_arw_valid),
.system_spi_0_io_data_0_read(system_spi_0_io_data_0_read),
.system_spi_0_io_data_0_write(system_spi_0_io_data_0_write),
.system_spi_0_io_data_0_writeEnable(system_spi_0_io_data_0_writeEnable),
.system_spi_0_io_data_1_read(system_spi_0_io_data_1_read),
.system_spi_0_io_data_1_write(system_spi_0_io_data_1_write),
.system_spi_0_io_data_1_writeEnable(system_spi_0_io_data_1_writeEnable),
.system_spi_0_io_data_2_read(system_spi_0_io_data_2_read),
.system_spi_0_io_data_2_write(system_spi_0_io_data_2_write),
.system_spi_0_io_data_2_writeEnable(system_spi_0_io_data_2_writeEnable),
.system_spi_0_io_data_3_read(system_spi_0_io_data_3_read),
.system_spi_0_io_data_3_write(system_spi_0_io_data_3_write),
.system_spi_0_io_data_3_writeEnable(system_spi_0_io_data_3_writeEnable),
.system_spi_0_io_sclk_write(system_spi_0_io_sclk_write),
.system_uart_0_io_txd(system_uart_0_io_txd),
.system_uart_0_io_rxd(system_uart_0_io_rxd),
.system_spi_0_io_ss(system_spi_0_io_ss),
.tiny_awvalid(tiny_awvalid),
.tiny_awready(tiny_awready),
.tiny_awaddr(tiny_awaddr),
.tiny_awlen(tiny_awlen),
.tiny_awid(tiny_awid),
.tiny_awsize(tiny_awsize),
.tiny_awburst(tiny_awburst),
.tiny_awlock(tiny_awlock),
.tiny_awcache(tiny_awcache),
.tiny_awprot(tiny_awprot),
.tiny_wvalid(tiny_wvalid),
.tiny_wready(tiny_wready),
.tiny_wdata(tiny_wdata),
.tiny_wstrb(tiny_wstrb),
.tiny_wlast(tiny_wlast),
.tiny_bvalid(tiny_bvalid),
.tiny_bready(tiny_bready),
.tiny_bresp(tiny_bresp),
.tiny_arvalid(tiny_arvalid),
.tiny_arready(tiny_arready),
.tiny_araddr(tiny_araddr),
.tiny_arlen(tiny_arlen),
.tiny_arid(tiny_arid),
.tiny_arsize(tiny_arsize),
.tiny_arburst(tiny_arburst),
.tiny_arlock(tiny_arlock),
.tiny_arcache(tiny_arcache),
.tiny_arprot(tiny_arprot),
.tiny_rvalid(tiny_rvalid),
.tiny_rready(tiny_rready),
.tiny_rdata(tiny_rdata),
.tiny_rlast(tiny_rlast),
.tiny_rresp(tiny_rresp)
);
cnn_axi_full_to_half_duplex #(.DATA_WIDTH(128)) accelerator_adapter(
.clk(clk_96),
.rst(ai_reset | system_reset),
.io_ddr_arw_valid(accel_avalid),
.io_ddr_arw_ready(accel_aready),
.io_ddr_arw_payload_addr(accel_addr),
.io_ddr_arw_payload_id(accel_id),
.io_ddr_arw_payload_len(accel_len),
.io_ddr_arw_payload_size(accel_size),
.io_ddr_arw_payload_burst(accel_burst),
.io_ddr_arw_payload_lock(accel_lock_full),
.io_ddr_arw_payload_write(accel_write),
.io_ddr_w_payload_id(unused_wid),
.io_ddr_w_valid(accel_wvalid),
.io_ddr_w_ready(accel_wready),
.io_ddr_w_payload_data(accel_wdata),
.io_ddr_w_payload_strb(accel_wstrb),
.io_ddr_w_payload_last(accel_wlast),
.io_ddr_b_valid(accel_bvalid),
.io_ddr_b_ready(accel_bready),
.io_ddr_b_payload_id(accel_bid),
.io_ddr_b_payload_resp(accel_bresp),
.io_ddr_r_valid(accel_rvalid),
.io_ddr_r_ready(accel_rready),
.io_ddr_r_payload_data(accel_rdata),
.io_ddr_r_payload_id(accel_rid),
.io_ddr_r_payload_resp(accel_rresp),
.io_ddr_r_payload_last(accel_rlast),
.s_axi_awid(tiny_awid),
.s_axi_awaddr(tiny_awaddr),
.s_axi_awlen(tiny_awlen),
.s_axi_awsize(tiny_awsize),
.s_axi_awburst(tiny_awburst),
.s_axi_awlock(tiny_awlock),
.s_axi_awcache(tiny_awcache),
.s_axi_awprot(tiny_awprot),
.s_axi_awqos(4'b0),
.s_axi_awregion(4'b0),
.s_axi_awvalid(tiny_awvalid),
.s_axi_awready(tiny_awready),
.s_axi_wdata(tiny_wdata),
.s_axi_wstrb(tiny_wstrb),
.s_axi_wlast(tiny_wlast),
.s_axi_wvalid(tiny_wvalid),
.s_axi_wready(tiny_wready),
.s_axi_bid(unused_bid),
.s_axi_bresp(tiny_bresp),
.s_axi_bvalid(tiny_bvalid),
.s_axi_bready(tiny_bready),
.s_axi_arid(tiny_arid),
.s_axi_araddr(tiny_araddr),
.s_axi_arlen(tiny_arlen),
.s_axi_arsize(tiny_arsize),
.s_axi_arburst(tiny_arburst),
.s_axi_arlock(tiny_arlock),
.s_axi_arcache(tiny_arcache),
.s_axi_arprot(tiny_arprot),
.s_axi_arqos(4'b0),
.s_axi_arregion(4'b0),
.s_axi_arvalid(tiny_arvalid),
.s_axi_arready(tiny_arready),
.s_axi_rid(unused_rid),
.s_axi_rdata(tiny_rdata),
.s_axi_rresp(tiny_rresp),
.s_axi_rlast(tiny_rlast),
.s_axi_rvalid(tiny_rvalid),
.s_axi_rready(tiny_rready)
);
assign accel_lock=|accel_lock_full;
assign bus_avalid={accel_avalid,cpu_arw_valid,video_avalid};
assign {accel_aready,cpu_arw_ready,video_aready}=bus_aready;
assign bus_addr={accel_addr,cpu_arw_payload_addr,video_addr};
assign bus_id={accel_id,cpu_arw_payload_id,video_id};
assign bus_len={accel_len,cpu_arw_payload_len,video_len};
assign bus_size={accel_size,cpu_arw_payload_size,video_size};
assign bus_burst={accel_burst,cpu_arw_payload_burst,video_burst};
assign bus_write={accel_write,cpu_arw_payload_write,video_write};
assign bus_lock={accel_lock,cpu_arw_payload_lock,video_lock};
assign bus_wvalid={accel_wvalid,cpu_w_valid,video_wvalid};
assign {accel_wready,cpu_w_ready,video_wready}=bus_wready;
assign bus_wdata={accel_wdata,cpu_w_payload_data,video_wdata};
assign bus_wstrb={accel_wstrb,cpu_w_payload_strb,video_wstrb};
assign bus_wlast={accel_wlast,cpu_w_payload_last,video_wlast};
assign {accel_bvalid,cpu_b_valid,video_bvalid}=bus_bvalid;
assign bus_bready={accel_bready,cpu_b_ready,video_bready};
assign {accel_bid,cpu_b_payload_id,video_bid}=bus_bid;
assign {accel_bresp,cpu_b_payload_resp,video_bresp}=bus_bresp;
assign {accel_rvalid,cpu_r_valid,video_rvalid}=bus_rvalid;
assign bus_rready={accel_rready,cpu_r_ready,video_rready};
assign {accel_rdata,cpu_r_payload_data,video_rdata}=bus_rdata;
assign {accel_rid,cpu_r_payload_id,video_rid}=bus_rid;
assign {accel_rresp,cpu_r_payload_resp,video_rresp}=bus_rresp;
assign {accel_rlast,cpu_r_payload_last,video_rlast}=bus_rlast;
cnn_shared_ddr memory_fabric(
.clk(clk_96),
.rst_n(fabric_rst_n),
.ai_abort({2{ai_reset | system_reset | memory_reset | peripheral_reset}}),
.ai_quiescent(ai_quiescent),
.s_avalid(bus_avalid),
.s_aready(bus_aready),
.s_addr(bus_addr),
.s_id(bus_id),
.s_len(bus_len),
.s_size(bus_size),
.s_burst(bus_burst),
.s_write(bus_write),
.s_lock(bus_lock),
.s_wvalid(bus_wvalid),
.s_wready(bus_wready),
.s_wdata(bus_wdata),
.s_wstrb(bus_wstrb),
.s_wlast(bus_wlast),
.s_bvalid(bus_bvalid),
.s_bready(bus_bready),
.s_bid(bus_bid),
.s_bresp(bus_bresp),
.s_rvalid(bus_rvalid),
.s_rready(bus_rready),
.s_rdata(bus_rdata),
.s_rid(bus_rid),
.s_rresp(bus_rresp),
.s_rlast(bus_rlast),
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
.idle_o(idle_o)
);
endmodule
