`timescale 1ns/1ps
// Network-byte to 128-bit shared-DDR write adapter. Run after an async FIFO
// in the shared DDR clock domain. Banks are private physical DDR addresses;
// a consumer must not read the candidate bank until commit_ok.
module fatigue_frame_writer #(
 parameter [31:0] BASE0=32'h03000000, BASE1=32'h03100000,
 parameter FRAME_BYTES=460800
)(
 input wire clk,rst_n,bank,
 input wire byte_valid,output wire byte_ready,input wire [31:0] byte_addr,input wire [7:0] byte_data,
 input wire frame_valid,frame_error,input wire [31:0] frame_id,
 output reg commit_ok,output reg commit_error,output reg [31:0] committed_id,
 output wire avalid,input wire aready,output wire [31:0] addr,output wire [7:0] len,output wire [2:0] size,output wire [1:0] burst,output wire write,
 output wire wvalid,input wire wready,output wire [127:0] wdata,output wire [15:0] wstrb,output wire wlast,
 input wire bvalid,output wire bready,input wire [1:0] bresp
);
 localparam COLLECT=0,SEND=1,WAIT_B=2;
 reg [1:0] state;
 reg [127:0] buffer;reg [15:0] mask;
 reg [31:0] beat_addr,expected_addr,held_frame;
 reg held_bank,a_done,w_done,pending_commit,discard;
 wire accepted=byte_valid&&byte_ready;
 assign byte_ready=state==COLLECT;
 assign avalid=state==SEND&&!a_done;assign addr=(held_bank?BASE1:BASE0)+beat_addr;
 assign len=0;assign size=3'd4;assign burst=2'b01;assign write=1'b1;
 assign wvalid=state==SEND&&!w_done;assign wdata=buffer;assign wstrb=mask;assign wlast=1'b1;
 assign bready=state==WAIT_B;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin state<=COLLECT;buffer<=0;mask<=0;beat_addr<=0;expected_addr<=0;held_bank<=0;a_done<=0;w_done<=0;pending_commit<=0;discard<=0;held_frame<=0;commit_ok<=0;commit_error<=0;committed_id<=0;end
  else begin
   commit_ok<=0;commit_error<=0;
   if(frame_error)begin pending_commit<=0;discard<=1;end
   if(frame_valid)begin pending_commit<=1;held_frame<=frame_id;end
   case(state)
    COLLECT:begin
     // Validation may arrive after the final DDR response. In that case
     // there will be no later WAIT_B to publish the completed frame.
     if(frame_valid)begin
      pending_commit<=0;
      if(!discard && !frame_error && expected_addr==FRAME_BYTES && mask==0)begin
       commit_ok<=1;committed_id<=frame_id;
      end else commit_error<=1;
     end
     if(accepted)begin
      if(byte_addr>=FRAME_BYTES || (byte_addr!=0 && (byte_addr!=expected_addr || bank!=held_bank)))begin
       discard<=1;commit_error<=1;
      end else begin
       if(byte_addr==0)begin expected_addr<=1;held_bank<=bank;discard<=0;buffer<=0;mask<=0;beat_addr<=0;end
       else expected_addr<=expected_addr+1'b1;
       buffer[byte_addr[3:0]*8+:8]<=byte_data;mask[byte_addr[3:0]]<=1'b1;
       if(byte_addr[3:0]==15 || byte_addr==FRAME_BYTES-1)begin
        beat_addr<={byte_addr[31:4],4'b0};a_done<=0;w_done<=0;state<=SEND;
       end
      end
     end
    end
    SEND:begin
     if(avalid&&aready)a_done<=1;
     if(wvalid&&wready)w_done<=1;
     if((a_done||(avalid&&aready))&&(w_done||(wvalid&&wready)))state<=WAIT_B;
    end
    WAIT_B:if(bvalid)begin
     if(bresp!=0)begin discard<=1;commit_error<=1;end
     if(pending_commit || frame_valid)begin
      if(bresp==0 && !discard && !frame_error && beat_addr+16>=FRAME_BYTES)begin commit_ok<=1;committed_id<=frame_valid?frame_id:held_frame;end
      else commit_error<=1;
      pending_commit<=0;
     end
     state<=COLLECT;buffer<=0;mask<=0;
    end
    default:state<=COLLECT;
   endcase
  end
 end
endmodule
