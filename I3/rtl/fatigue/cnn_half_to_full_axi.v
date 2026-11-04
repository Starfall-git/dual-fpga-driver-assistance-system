`timescale 1ns/1ps
/* FATIGUE-SC431-Translate the existing single-outstanding half-duplex
   arbiter into the DDR controller's separate AXI4 address channels. The
   upstream arbiter must hold address signals until handshake and must not
   issue another transaction before its response. */
module cnn_half_to_full_axi (
 input wire enable,
 input wire h_avalid, output wire h_aready,
 input wire [31:0] h_addr, input wire [3:0] h_id,
 input wire [7:0] h_len, input wire [2:0] h_size,
 input wire [1:0] h_burst, input wire h_write, h_lock,
 input wire h_wvalid, output wire h_wready, input wire [127:0] h_wdata,
 input wire [15:0] h_wstrb, input wire h_wlast,
 output wire h_bvalid, input wire h_bready, output wire [1:0] h_bresp,
 output wire h_rvalid, input wire h_rready, output wire [127:0] h_rdata,
 output wire [1:0] h_rresp, output wire h_rlast,
 output wire awvalid, input wire awready, output wire [31:0] awaddr,
 output wire [3:0] awid, output wire [7:0] awlen, output wire [2:0] awsize,
 output wire [1:0] awburst, output wire awlock,
 output wire arvalid, input wire arready, output wire [31:0] araddr,
 output wire [3:0] arid, output wire [7:0] arlen, output wire [2:0] arsize,
 output wire [1:0] arburst, output wire arlock,
 output wire wvalid, input wire wready, output wire [127:0] wdata,
 output wire [15:0] wstrb, output wire wlast,
 input wire bvalid, output wire bready, input wire [1:0] bresp,
 input wire rvalid, output wire rready, input wire [127:0] rdata,
 input wire [1:0] rresp, input wire rlast
);
 assign awvalid = enable && h_avalid && h_write;
 assign arvalid = enable && h_avalid && !h_write;
 assign h_aready = enable && (h_write ? awready : arready);
 assign awaddr=h_addr; assign araddr=h_addr;
 assign awid=h_id; assign arid=h_id;
 assign awlen=h_len; assign arlen=h_len;
 assign awsize=h_size; assign arsize=h_size;
 assign awburst=h_burst; assign arburst=h_burst;
 assign awlock=h_lock; assign arlock=h_lock;
 /* enable gates admission only. In-flight transfers must drain even when
    calibration/availability drops; resetting an active bus requires a
    system-level reset shared with both endpoints. */
 assign wvalid=h_wvalid; assign h_wready=wready;
 assign wdata=h_wdata; assign wstrb=h_wstrb; assign wlast=h_wlast;
 assign h_bvalid=bvalid; assign bready=h_bready; assign h_bresp=bresp;
 assign h_rvalid=rvalid; assign rready=h_rready; assign h_rdata=rdata;
 assign h_rresp=rresp; assign h_rlast=rlast;
endmodule
