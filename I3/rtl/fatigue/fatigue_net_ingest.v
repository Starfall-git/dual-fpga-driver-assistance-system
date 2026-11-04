`timescale 1ns/1ps
// UDP payload -> CRC-validated ordered frame events -> CDC -> DDR writes.
// Input is a ready/valid UDP payload stream, not raw Ethernet. A separate MAC
// must supply packet bytes, boundaries, and buffer packets during backpressure.
module fatigue_net_ingest #(
 parameter WIDTH=640,HEIGHT=360,CHUNK=1152,FIFO_ADDR=10,FRAME_BYTES=WIDTH*HEIGHT*2
)(
 input wire rx_clk,rx_rst_n,input wire in_valid,in_sop,in_eop,input wire [7:0] in_data,output wire in_ready,
 input wire ddr_clk,ddr_rst_n,input wire bank,
 output reg diag_parser_error,diag_parser_complete,diag_fifo_pressure,
 output wire [3:0] diag_errors,
 output wire commit_ok,commit_error,output wire [31:0] committed_id,
 output wire avalid,input wire aready,output wire [31:0] addr,output wire [7:0] len,output wire [2:0] size,output wire [1:0] burst,output wire write,
 output wire wvalid,input wire wready,output wire [127:0] wdata,output wire [15:0] wstrb,output wire wlast,
 input wire bvalid,output wire bready,input wire [1:0] bresp
);
 wire pr_ready,pwv,pfv,pfe;wire [31:0] paddr,pid;wire [7:0] pdata;
 wire q_ready,q_almost,q_valid,q_pop;wire [41:0] q_out;
 wire event_pending=pfv||pfe;
 always @(posedge rx_clk or negedge rx_rst_n)begin
  if(!rx_rst_n)begin diag_parser_error<=0;diag_parser_complete<=0;diag_fifo_pressure<=0;end
  else begin
   if(pfe)diag_parser_error<=1;
   if(pfv)diag_parser_complete<=1;
   if(q_almost)diag_fifo_pressure<=1;
  end
 end
 assign in_ready=pr_ready&&!q_almost&&!event_pending;
 fatigue_udp_frame #(.WIDTH(WIDTH),.HEIGHT(HEIGHT),.CHUNK(CHUNK)) parser(
 .clk(rx_clk),.rst_n(rx_rst_n),.in_valid(in_valid&&!q_almost&&!event_pending),.in_sop(in_sop),.in_eop(in_eop),.in_data(in_data),.in_ready(pr_ready),
 .wr_valid(pwv),.wr_ready(q_ready&&!q_almost&&!event_pending),.wr_addr(paddr),.wr_data(pdata),.frame_valid(pfv),.frame_id(pid),.frame_error(pfe),.diag_error(diag_errors));
 wire [41:0] q_in=pfe ? {2'b10,32'b0,8'b0} : pfv ? {2'b01,pid,8'b0} : {2'b00,paddr,pdata};
 fatigue_async_fifo #(.WIDTH(42),.ADDR(FIFO_ADDR)) bridge(.wclk(rx_clk),.wrst_n(rx_rst_n),
 .w_valid(pwv||pfv||pfe),.w_ready(q_ready),.w_almost_full(q_almost),.w_data(q_in),
 .rclk(ddr_clk),.rrst_n(ddr_rst_n),.r_valid(q_valid),.r_ready(q_pop),.r_data(q_out));
 wire byte_ready;
 assign q_pop=q_valid&&((q_out[41:40]!=0)||byte_ready);
 fatigue_frame_writer #(.FRAME_BYTES(FRAME_BYTES)) writer(.clk(ddr_clk),.rst_n(ddr_rst_n),.bank(bank),
 .byte_valid(q_valid&&q_out[41:40]==0),.byte_ready(byte_ready),.byte_addr(q_out[39:8]),.byte_data(q_out[7:0]),
 .frame_valid(q_valid&&q_pop&&q_out[41:40]==1),.frame_error(q_valid&&q_pop&&q_out[41:40]==2),.frame_id(q_out[39:8]),
 .commit_ok(commit_ok),.commit_error(commit_error),.committed_id(committed_id),
 .avalid(avalid),.aready(aready),.addr(addr),.len(len),.size(size),.burst(burst),.write(write),
 .wvalid(wvalid),.wready(wready),.wdata(wdata),.wstrb(wstrb),.wlast(wlast),.bvalid(bvalid),.bready(bready),.bresp(bresp));
endmodule
