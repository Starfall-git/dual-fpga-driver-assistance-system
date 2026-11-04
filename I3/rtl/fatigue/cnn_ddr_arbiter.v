`timescale 1ns/1ps
// Three combined-address ports: video, buffered CPU, buffered TinyML.
// Single outstanding DDR transaction; round robin advances after completion.
// AI ports MUST be store-and-forward isolated ports, all on the DDR UI clock.
module cnn_ddr_arbiter(
 input wire clk,rst_n,
 input wire [2:0] s_avalid,output reg [2:0] s_aready,
 input wire [95:0] s_addr,input wire [23:0] s_id,s_len,
 input wire [8:0] s_size,input wire [5:0] s_burst,input wire [2:0] s_write,s_lock,
 input wire [2:0] s_wvalid,output reg [2:0] s_wready,
 input wire [383:0] s_wdata,input wire [47:0] s_wstrb,input wire [2:0] s_wlast,
 output reg [2:0] s_bvalid,input wire [2:0] s_bready,
 output reg [23:0] s_bid,output reg [5:0] s_bresp,
 output reg [2:0] s_rvalid,input wire [2:0] s_rready,
 output reg [383:0] s_rdata,output reg [23:0] s_rid,
 output reg [5:0] s_rresp,output reg [2:0] s_rlast,
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
 localparam IDLE=0,ADDRESS=1,WRITE=2,BRESP=3,READ=4;
 reg [2:0] state;
 reg [1:0] owner,next_owner;
 reg [7:0] response_id;
 integer k,idx;
 reg found;
 reg [1:0] choice;
 always @* begin
  found=0;choice=0;
  for(k=0;k<3;k=k+1)begin
   idx=next_owner+k;if(idx>=3)idx=idx-3;
   if(!found && s_avalid[idx])begin found=1;choice=idx;end
  end
 end
 assign idle_o=state==IDLE;
 assign m_avalid=rst_n && state==ADDRESS && s_avalid[owner];
 assign m_addr=s_addr[owner*32+:32];assign m_len=s_len[owner*8+:8];
 assign m_size=s_size[owner*3+:3];assign m_burst=s_burst[owner*2+:2];
 assign m_write=s_write[owner];assign m_lock=s_lock[owner];
 assign m_id={2'b0,owner};assign m_wid={2'b0,owner};
 assign m_wvalid=rst_n && state==WRITE && s_wvalid[owner];
 assign m_wdata=s_wdata[owner*128+:128];assign m_wstrb=s_wstrb[owner*16+:16];
 assign m_wlast=s_wlast[owner];
 assign m_bready=rst_n && state==BRESP && s_bready[owner];
 assign m_rready=rst_n && state==READ && s_rready[owner];
 always @* begin
  s_aready=0;s_wready=0;s_bvalid=0;s_bid=0;s_bresp=0;
  s_rvalid=0;s_rid=0;s_rresp=0;s_rdata=0;s_rlast=0;
  if(rst_n)case(state)
   ADDRESS:s_aready[owner]=m_aready;
   WRITE:s_wready[owner]=m_wready;
   BRESP:begin s_bvalid[owner]=m_bvalid;s_bid[owner*8+:8]=response_id;s_bresp[owner*2+:2]=m_bresp;end
   READ:begin
    s_rvalid[owner]=m_rvalid;s_rid[owner*8+:8]=response_id;
    s_rresp[owner*2+:2]=m_rresp;s_rdata[owner*128+:128]=m_rdata;s_rlast[owner]=m_rlast;
   end
   default:begin end
  endcase
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin state<=IDLE;owner<=0;next_owner<=0;response_id<=0;end
  else case(state)
   IDLE:if(found)begin owner<=choice;state<=ADDRESS;end
   ADDRESS:if(m_avalid && m_aready)begin response_id<=s_id[owner*8+:8];state<=m_write ? WRITE : READ;end
   WRITE:if(m_wvalid && m_wready && m_wlast)state<=BRESP;
   BRESP:if(m_bvalid && m_bready)begin state<=IDLE;next_owner<=owner==2 ? 0 : owner+1'b1;end
   READ:if(m_rvalid && m_rready && m_rlast)begin state<=IDLE;next_owner<=owner==2 ? 0 : owner+1'b1;end
   default:state<=IDLE;
  endcase
 end
endmodule
