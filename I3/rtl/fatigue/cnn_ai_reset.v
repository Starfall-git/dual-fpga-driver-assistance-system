`timescale 1ns/1ps
// Shared-memory clock domain. Global reset alone resets fabric; this only resets AI.
module cnn_ai_reset(
 input wire clk,rst_n,memory_ready,reset_request,soc_reset,
 input wire [1:0] quiescent,
 output wire ai_reset,output wire hardware_ready
);
 localparam HOLD=0,RELEASE=1,RUN=2;
 reg [1:0] state;
 reg [4:0] hold_cycles;
 assign ai_reset=!rst_n || state==HOLD || !memory_ready || reset_request;
 assign hardware_ready=state==RUN && !ai_reset && !soc_reset;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin state<=HOLD;hold_cycles<=0;end
  else if(!memory_ready || reset_request)begin state<=HOLD;hold_cycles<=0;end
  else case(state)
   HOLD:begin
    if(hold_cycles!=31)hold_cycles<=hold_cycles+1'b1;
    if(hold_cycles==31 && &quiescent)state<=RELEASE;
   end
   // Sapphire stretches its reset. Do not retrigger while waiting for that release.
   RELEASE:if(!soc_reset)state<=RUN;
   RUN:if(soc_reset)begin state<=HOLD;hold_cycles<=0;end
   default:begin state<=HOLD;hold_cycles<=0;end
  endcase
 end
endmodule
