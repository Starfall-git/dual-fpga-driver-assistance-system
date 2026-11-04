`timescale 1ns/1ps
// One outstanding combined-address AXI transaction, with checked relocation.
// Use on each AI master BEFORE the shared fabric; video bypasses this window.
// Local decode errors never issue a downstream address/data transfer.
module cnn_axi_window #(
 parameter [31:0] LOGICAL_BASE=32'h00001000,
 parameter [31:0] PHYSICAL_BASE=32'h04000000,
 parameter [31:0] WINDOW_BYTES=32'h04000000
)(
 input wire clk,rst_n,
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
 localparam IDLE=0,WRITE=1,RESPONSE=2,READ=3,ERROR_WRITE=4,ERROR_B=5,ERROR_READ=6;
 reg [2:0] state;
 reg [7:0] id;
 reg [8:0] remaining;
 wire [32:0] byte_count=({25'd0,s_len}+33'd1)<<4;
 // Official Sapphire BmbToAxi4Bridge fixes AxSIZE=4, but retains the
 // byte address on masked CPU stores. Normalize the DDR beat address;
 // WDATA/WSTRB already identify the correct byte lanes and stay unchanged.
 wire [31:0] beat_address={s_addr[31:4],4'b0000};
 wire [32:0] end_address={1'b0,beat_address}+byte_count;
 wire [32:0] physical_end={1'b0,PHYSICAL_BASE}+{1'b0,WINDOW_BYTES};
 wire legal=LOGICAL_BASE[11:0]==0 && PHYSICAL_BASE[11:0]==0 &&
   WINDOW_BYTES[11:0]==0 && WINDOW_BYTES!=0 && s_size==4 && s_burst==1 && !s_lock &&
   s_addr>=LOGICAL_BASE && end_address<=({1'b0,LOGICAL_BASE}+{1'b0,WINDOW_BYTES}) &&
   !end_address[32] && physical_end<=33'h100000000 &&
   ({1'b0,beat_address[11:0]}+byte_count)<=4096;
 assign m_avalid=rst_n && state==IDLE && s_avalid && legal;
 assign s_aready=rst_n && state==IDLE && (legal ? m_aready : 1'b1);
 assign m_addr=beat_address-LOGICAL_BASE+PHYSICAL_BASE;
 assign m_id=s_id;assign m_len=s_len;assign m_size=s_size;
 assign m_burst=s_burst;assign m_write=s_write;assign m_lock=s_lock;
 assign m_wvalid=rst_n && state==WRITE && s_wvalid;
 assign s_wready=rst_n && (state==ERROR_WRITE || (state==WRITE && m_wready));
 assign m_wdata=s_wdata;assign m_wstrb=s_wstrb;assign m_wlast=s_wlast;
 assign s_bvalid=rst_n && (state==ERROR_B || (state==RESPONSE && m_bvalid));
 assign s_bid=state==ERROR_B ? id : m_bid;
 assign s_bresp=state==ERROR_B ? 2'b11 : m_bresp;
 assign m_bready=rst_n && state==RESPONSE && s_bready;
 assign s_rvalid=rst_n && (state==ERROR_READ || (state==READ && m_rvalid));
 assign s_rdata=state==ERROR_READ ? 128'd0 : m_rdata;
 assign s_rid=state==ERROR_READ ? id : m_rid;
 assign s_rresp=state==ERROR_READ ? 2'b11 : m_rresp;
 assign s_rlast=state==ERROR_READ ? remaining==1 : m_rlast;
 assign m_rready=rst_n && state==READ && s_rready;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin state<=IDLE;id<=0;remaining<=0;end
  else case(state)
   IDLE: if(s_avalid && s_aready) begin
    id<=s_id;remaining<={1'b0,s_len}+1'b1;
    if(legal) state<=s_write ? WRITE : READ;
    else state<=s_write ? ERROR_WRITE : ERROR_READ;
   end
   WRITE: if(s_wvalid && s_wready && s_wlast) state<=RESPONSE;
   RESPONSE: if(s_bvalid && s_bready) state<=IDLE;
   READ: if(s_rvalid && s_rready && s_rlast) state<=IDLE;
   ERROR_WRITE: if(s_wvalid && s_wready) begin
    remaining<=remaining-1'b1;if(remaining==1)state<=ERROR_B;
   end
   ERROR_B: if(s_bready)state<=IDLE;
   ERROR_READ: if(s_rready)begin remaining<=remaining-1'b1;if(remaining==1)state<=IDLE;end
   default:state<=IDLE;
  endcase
 end
endmodule
