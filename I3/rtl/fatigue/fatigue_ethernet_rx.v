`timescale 1ns/1ps
// Vendor RGMII and UDP MAC, exposing received UDP payload with backpressure.
// A 1-Gb/s full-duplex direct link is assumed. RAM may be overwritten if a
// new datagram arrives before the previous one has drained: packet CRC and
// frame CRC must reject that frame; overrun is surfaced for diagnostics.
module fatigue_ethernet_rx(
 input wire rst_n,rxc,input wire [3:0] rxd_hi_i,rxd_lo_i,input wire rx_dv_HI,rx_dv_LO,
 output wire phy_rst_n,mdc_o,mdio_o,mdio_oe,input wire mdio_i,
 output wire tx_en_o_HI,tx_en_o_LO,txc_hi_o,txc_lo_o,output wire[3:0]txd_hi_o,txd_lo_o,
 output wire stream_valid,input wire stream_ready,output wire stream_sop,stream_eop,output wire[7:0]stream_data,output wire overrun,invalid
);
 assign phy_rst_n=1'b1;assign mdc_o=0;assign mdio_o=0;assign mdio_oe=0;assign txc_hi_o=1;assign txc_lo_o=0;
 wire [7:0] rxdata,txdata;wire rxvalid,rxerror,rxclock,txclock,txvalid;wire col,crs;
 reg [3:0] hi;reg dvhi;
 always @(posedge rxc)begin hi<=rxd_hi_i;dvhi<=rx_dv_HI;end
 util_gmii_to_rgmii phy(.rst_n(rst_n),.rgmii_rxc(rxc),.rgmii_rx_hi(rxd_lo_i),.rgmii_rx_lo(hi),.rgmii_rx_dv(rx_dv_LO),.rgmii_rx_er(dvhi),
 .rgmii_tx_ctrl_hi(tx_en_o_HI),.rgmii_tx_ctrl_lo(tx_en_o_LO),.rgmii_txd_lo(txd_lo_o),.rgmii_txd_hi(txd_hi_o),
 .gmii_txd(txdata),.gmii_tx_en(txvalid),.gmii_tx_er(1'b0),.gmii_tx_clk(txclock),.gmii_crs(crs),.gmii_col(col),.gmii_rxd(rxdata),.gmii_rx_dv(rxvalid),.gmii_rx_er(rxerror),.gmii_rx_clk(rxclock),.duplex_mode(1'b1));
 reg rxvalid_d;reg [7:0]rxdata_d;
 always @(posedge rxclock or negedge rst_n)if(!rst_n)begin rxvalid_d<=0;rxdata_d<=0;end else begin rxvalid_d<=rxvalid;rxdata_d<=rxdata;end
 wire [7:0] ram_data;wire [10:0] ram_addr;wire [15:0] payload_length;wire payload_valid;
 wire ack_req,ack_wr_en,ack_ram_req,ack_send_end,ack_busy_error,peer_missing;wire [7:0] ack_wr_data;
 // Resolve the fixed host address before transmitting unicast ACKs.
 reg arp_req;reg [26:0] arp_retry;
 always @(posedge rxclock or negedge rst_n)begin
  if(!rst_n)begin arp_req<=0;arp_retry<=0;end
  else begin
   arp_req<=0;
   if(!peer_missing)arp_retry<=0;
   else if(arp_retry==0)begin arp_req<=1;arp_retry<=27'd124999999;end
   else arp_retry<=arp_retry-1'b1;
  end
 end
 fatigue_packet_ack ack(.clk(rxclock),.rst_n(rst_n),.in_valid(stream_valid),.in_ready(stream_ready),.in_sop(stream_sop),.in_eop(stream_eop),.in_data(stream_data),
 .udp_ram_data_req(ack_ram_req),.mac_send_end(ack_send_end),.udp_tx_req(ack_req),.ram_wr_en(ack_wr_en),.ram_wr_data(ack_wr_data),.ack_busy_error(ack_busy_error));
 mac_top mac(.gmii_tx_clk(txclock),.gmii_rx_clk(rxclock),.rst_n(rst_n),
 .source_mac_addr(48'h000a3501fec0),.TTL(8'h80),.source_ip_addr(32'h01000002),.destination_ip_addr(32'h01000006),
 .udp_send_source_port(16'd8080),.udp_send_destination_port(16'd8080),.ram_wr_data(ack_wr_data),.ram_wr_en(ack_wr_en),.udp_ram_data_req(ack_ram_req),.udp_send_data_length(16'd10),.udp_tx_req(ack_req&&!peer_missing),.arp_request_req(arp_req),
 .mac_data_valid(txvalid),.mac_send_end(ack_send_end),.mac_tx_data(txdata),.rx_dv(rxvalid_d),.mac_rx_datain(rxdata_d),
 .udp_rec_ram_rdata(ram_data),.udp_rec_ram_read_addr(ram_addr),.udp_rec_data_length(payload_length),.udp_rec_data_valid(payload_valid),.arp_found(),.mac_not_exist(peer_missing));
 fatigue_udp_ram_stream stream(.clk(rxclock),.rst_n(rst_n),.udp_valid(payload_valid),.udp_length(payload_length),.ram_addr(ram_addr),.ram_q(ram_data),
 .out_valid(stream_valid),.out_ready(stream_ready),.out_sop(stream_sop),.out_eop(stream_eop),.out_data(stream_data),.packet_overrun(overrun),.packet_invalid(invalid));
endmodule
