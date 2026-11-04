`timescale 1ns/1ps
// Independent frame-bank ownership. Publish only at a video boundary and keep
// the old bank owned by the reader until it explicitly reports read drain.
module fatigue_frame_banks(
 input wire ddr_clk,ddr_rst_n,
 input wire complete,input wire [31:0] completed_id,
 output reg write_bank,output wire writable,output reg accepted,output reg overflow,
 input wire pix_clk,pix_rst_n,input wire frame_start,input wire old_bank_drained,
 output reg display_bank,output reg display_valid,output reg [31:0] display_id
);
 reg toggle,held_bank;reg [31:0] held_id;
 reg ack;(* async_reg="true" *) reg ack_m,ack_s,req_m,req_s;
 reg changing,writable_reg;
 assign writable=writable_reg;
 always @(posedge ddr_clk or negedge ddr_rst_n)begin
  if(!ddr_rst_n)begin toggle<=0;held_bank<=0;held_id<=0;ack_m<=0;ack_s<=0;write_bank<=1;writable_reg<=1;accepted<=0;overflow<=0;end
  else begin
   ack_m<=ack;ack_s<=ack_m;accepted<=0;overflow<=0;
   if(complete)begin
    if(writable)begin held_bank<=write_bank;held_id<=completed_id;toggle<=~toggle;writable_reg<=0;accepted<=1;end
    else overflow<=1;
   end
   if(!writable_reg && toggle==ack_s && write_bank==held_bank)begin write_bank<=~held_bank;writable_reg<=1;end
  end
 end
 always @(posedge pix_clk or negedge pix_rst_n)begin
  if(!pix_rst_n)begin req_m<=0;req_s<=0;ack<=0;display_bank<=0;display_valid<=0;display_id<=0;changing<=0;end
  else begin
   req_m<=toggle;req_s<=req_m;
   if(frame_start && req_s!=ack && !changing)begin
    display_bank<=held_bank;display_id<=held_id;display_valid<=1;changing<=1;
   end
   if(changing && old_bank_drained)begin ack<=req_s;changing<=0;end
  end
 end
endmodule
