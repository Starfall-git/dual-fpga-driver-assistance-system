`timescale 1ns/1ps
// Adapter for the vendor udp_rx receive RAM. udp_length includes the 8-byte
// UDP header; packet RAM address zero is the first payload byte. q is read
// with the vendor dpram's one-cycle registered-address latency.
module fatigue_udp_ram_stream(
 input wire clk,rst_n,input wire udp_valid,input wire [15:0] udp_length,
 output reg [10:0] ram_addr,input wire [7:0] ram_q,
 output wire out_valid,input wire out_ready,output wire out_sop,out_eop,output wire [7:0] out_data,
 output reg packet_overrun,packet_invalid
);
 localparam IDLE=0,ISSUE=1,WAIT_RAM=2,OUTPUT=3;
 reg [1:0] state;reg last_valid;reg [11:0] position,length;
 wire new_packet=udp_valid&&!last_valid;
 assign out_valid=state==OUTPUT;assign out_sop=position==0;assign out_eop=position+1==length;assign out_data=ram_q;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin state<=IDLE;last_valid<=0;ram_addr<=0;position<=0;length<=0;packet_overrun<=0;packet_invalid<=0;end
  else begin
   last_valid<=udp_valid;packet_overrun<=0;packet_invalid<=0;
   if(new_packet && state!=IDLE)packet_overrun<=1;
   case(state)
    IDLE:if(new_packet)begin
     if(udp_length<34 || udp_length>2056)packet_invalid<=1;
     else begin length<=udp_length-8;position<=0;ram_addr<=0;state<=ISSUE;end
    end
    ISSUE:begin ram_addr<=position[10:0];state<=WAIT_RAM;end
    WAIT_RAM:state<=OUTPUT;
    OUTPUT:if(out_valid&&out_ready)begin
     if(position+1==length)state<=IDLE;
     else begin position<=position+1'b1;state<=ISSUE;end
    end
    default:state<=IDLE;
   endcase
  end
 end
endmodule
