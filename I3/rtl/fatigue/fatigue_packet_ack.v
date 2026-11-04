`timescale 1ns/1ps
// Diagnostic stop-and-wait acknowledgement. Same clock as vendor MAC RX/TX.
// ACK is sent only after the full incoming UDP payload was accepted.
module fatigue_packet_ack(
 input wire clk,rst_n,
 input wire in_valid,in_ready,in_sop,in_eop,input wire [7:0] in_data,
 input wire udp_ram_data_req,mac_send_end,
 output wire udp_tx_req,output wire ram_wr_en,output reg [7:0] ram_wr_data,
 output reg ack_busy_error
);
 localparam IDLE=0,REQ=1,WRITE=2,WAIT_END=3;
 reg [1:0] state;reg [15:0] pos;reg [31:0] seq,held_seq;reg [15:0] chunk,held_chunk;
 reg [3:0] index;
 assign udp_tx_req=state==REQ;
 assign ram_wr_en=state==WRITE;
 always @* case(index)
  0:ram_wr_data="F";1:ram_wr_data="A";2:ram_wr_data="C";3:ram_wr_data="K";
  4:ram_wr_data=held_seq[31:24];5:ram_wr_data=held_seq[23:16];6:ram_wr_data=held_seq[15:8];7:ram_wr_data=held_seq[7:0];
  8:ram_wr_data=held_chunk[15:8];9:ram_wr_data=held_chunk[7:0];default:ram_wr_data=0;
 endcase
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin state<=IDLE;pos<=0;seq<=0;chunk<=0;held_seq<=0;held_chunk<=0;index<=0;ack_busy_error<=0;end
  else begin
   ack_busy_error<=0;
   if(in_valid&&in_ready)begin
    if(in_sop)begin pos<=1;seq<=0;chunk<=0;end
    else begin
     pos<=pos+1'b1;
     if(pos>=6&&pos<=9)seq<={seq[23:0],in_data};
     if(pos>=10&&pos<=11)chunk<={chunk[7:0],in_data};
     if(in_eop)begin
      if(state==IDLE)begin held_seq<=seq;held_chunk<=chunk;state<=REQ;end
      else ack_busy_error<=1;
      pos<=0;
     end
    end
   end
   case(state)
    REQ:if(udp_ram_data_req)begin index<=0;state<=WRITE;end
    WRITE:if(index==9)state<=WAIT_END;else index<=index+1'b1;
    WAIT_END:if(mac_send_end)state<=IDLE;
    default:begin end
   endcase
  end
 end
endmodule
