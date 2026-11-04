`timescale 1ns/1ps
// Request a receive-side reset after the vendor's single UDP RAM has been
// overwritten. The downstream frame parser must reject incomplete frames.
module fatigue_net_recovery #(parameter HOLD_CYCLES=3125000)(
 input wire clk,rst_n,overrun,
 output wire recovering
);
 reg [23:0] remaining;
 assign recovering=remaining!=0;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) remaining<=0;
  else if(overrun && !recovering) remaining<=HOLD_CYCLES;
  else if(recovering) remaining<=remaining-1'b1;
 end
endmodule
