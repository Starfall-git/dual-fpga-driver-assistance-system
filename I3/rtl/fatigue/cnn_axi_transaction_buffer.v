`timescale 1ns/1ps
// Store a complete AI transaction before admitting it to shared DDR.
// rst_n is the MEMORY FABRIC reset, not the independent CPU reset.
// abort_i cancels unissued work; issued work drains without CPU participation.
module cnn_axi_transaction_buffer(
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
 localparam IDLE=0,COLLECT=1,ISSUE=2,W_FETCH=3,W_SEND=4,B_WAIT=5,B_OUT=6,R_FILL=7,R_FETCH=8,R_OUT=9;
 reg [3:0] state;
 reg [31:0] addr;
 reg [7:0] id,len,index;
 reg [2:0] size;
 reg [1:0] burst,bresp;
 reg write_op,lock,discard,bad_write;
 reg [7:0] bid;
 // Shared 256 x 144 storage: W={strb,data}; R={padding,id,resp,last,data}.
 // No RAM reset; controls never expose an unwritten entry.
 (* ram_style="block" *) reg [143:0] memory [0:255];
 reg [143:0] word;
 always @(posedge clk) begin
  if(rst_n && state==COLLECT && s_wvalid && s_wready)
   memory[index]<={s_wstrb,s_wdata};
  if(rst_n && state==R_FILL && m_rvalid && m_rready)
   memory[index]<={5'd0,m_rid,m_rresp,m_rlast,m_rdata};
  if(state==W_FETCH || state==R_FETCH)word<=memory[index];
 end
 assign quiescent_o=state==IDLE;
 assign s_aready=rst_n && state==IDLE && !abort_i;
 assign s_wready=rst_n && state==COLLECT && !abort_i;
 assign m_avalid=rst_n && state==ISSUE;
 assign m_addr=addr;assign m_id=id;assign m_len=len;
 assign m_size=size;assign m_burst=burst;assign m_write=write_op;assign m_lock=lock;
 assign m_wvalid=rst_n && state==W_SEND;
 assign m_wdata=word[127:0];assign m_wstrb=word[143:128];assign m_wlast=index==len;
 assign m_bready=rst_n && state==B_WAIT;
 assign s_bvalid=rst_n && state==B_OUT && !discard && !abort_i;
 assign s_bid=bid;assign s_bresp=bresp;
 assign m_rready=rst_n && state==R_FILL;
 assign s_rvalid=rst_n && state==R_OUT && !discard && !abort_i;
 assign s_rdata=word[127:0];assign s_rlast=word[128];
 assign s_rresp=word[130:129];assign s_rid=word[138:131];
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   state<=IDLE;addr<=0;id<=0;len<=0;index<=0;size<=0;burst<=0;write_op<=0;
   lock<=0;discard<=0;bad_write<=0;bid<=0;bresp<=0;
  end else begin
   if(abort_i)discard<=1;
   case(state)
    IDLE: begin
     discard<=0;
     if(s_avalid && s_aready) begin
      addr<=s_addr;id<=s_id;len<=s_len;size<=s_size;burst<=s_burst;
      lock<=s_lock;write_op<=s_write;index<=0;bad_write<=0;
      state<=s_write ? COLLECT : ISSUE;
     end
    end
    COLLECT: if(abort_i)state<=IDLE;
     else if(s_wvalid && s_wready) begin
      if(s_wlast!=(index==len))bad_write<=1;
      if(index==len) begin
       index<=0;
       if(bad_write || !s_wlast) begin bid<=id;bresp<=2'b10;state<=B_OUT;end
       else state<=ISSUE;
      end else index<=index+1'b1;
     end
    // Never withdraw VALID after assertion, even if abort arrives while stalled.
    ISSUE: if(m_avalid && m_aready)begin index<=0;state<=write_op ? W_FETCH : R_FILL;end
    W_FETCH:state<=W_SEND;
    W_SEND:if(m_wvalid && m_wready)begin
     if(index==len)state<=B_WAIT;
     else begin index<=index+1'b1;state<=W_FETCH;end
    end
    B_WAIT:if(m_bvalid && m_bready)begin
     bid<=m_bid;bresp<=m_bresp;state<=(discard || abort_i) ? IDLE : B_OUT;
    end
    B_OUT:if(discard || abort_i || (s_bvalid && s_bready))state<=IDLE;
    R_FILL:if(m_rvalid && m_rready)begin
     if(m_rlast)begin index<=0;state<=(discard || abort_i) ? IDLE : R_FETCH;end
     else index<=index+1'b1;
    end
    R_FETCH:state<=(discard || abort_i) ? IDLE : R_OUT;
    R_OUT:if(discard || abort_i)state<=IDLE;
     else if(s_rvalid && s_rready)begin
      if(s_rlast)state<=IDLE;
      else begin index<=index+1'b1;state<=R_FETCH;end
     end
    default:state<=IDLE;
   endcase
  end
 end
endmodule
