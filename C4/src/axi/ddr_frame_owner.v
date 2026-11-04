`timescale 1ns/1ps

module ddr_frame_owner #(parameter SNAPSHOT_ENABLE=0)(
 input wire clk,reset,write_done,read_boundary,freeze_request,
 output reg [1:0] write_index,read_index,
 output reg frozen
);
 reg [1:0] last_complete;
 reg freeze_meta,freeze_sync,have_written,have_read;
 wire hold_read=SNAPSHOT_ENABLE && ((freeze_sync && have_read) || frozen);
 wire [1:0] plus_one=write_index+2'd1;
 wire [1:0] next_write=plus_one==read_index ? write_index+2'd2 : plus_one;
 always @(posedge clk) begin
  if(reset) begin
   write_index<=0;read_index<=2;last_complete<=0;frozen<=0;
   freeze_meta<=0;freeze_sync<=0;have_written<=0;have_read<=0;
  end else begin
   freeze_meta<=freeze_request;freeze_sync<=freeze_meta;
   if(write_done) have_written<=1;
   if(read_boundary && have_written && !hold_read) have_read<=1;
   if(read_boundary) begin
    if(!SNAPSHOT_ENABLE || !freeze_sync) frozen<=0;
    else if(have_read) frozen<=1;
   end
   case({(read_boundary && !hold_read),write_done})
    2'b01:begin write_index<=next_write;last_complete<=write_index;end
    2'b10:read_index<=last_complete;
    2'b11:begin write_index<=next_write;read_index<=write_index;end
   endcase
  end
 end
endmodule
