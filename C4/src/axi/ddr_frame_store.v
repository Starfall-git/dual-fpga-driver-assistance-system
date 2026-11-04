`timescale 1ns/1ps

module ddr_frame_store(
 input wire clk,reset,write_done,read_boundary,
 input wire command_toggle_i,
 input wire [31:0] command_i,
 output reg command_ack_o,
 output reg [7:0] result_o,
 output reg [3:0] write_index,read_index,
 output reg [11:0] saved_o,compare_o,
 /* stable capture ordinals survive host reconnects. */
 output reg [383:0] order_o,
 output reg [1:0] mode_o
);
 reg request_meta,request_sync,have_written,have_read;
 reg [3:0] latest;
 reg [31:0] capture_ordinal;
 integer k,j,candidate,count;
 reg [3:0] next_write;
 reg found;
 always @* begin
  count=0;
  for(k=0;k<12;k=k+1) count=count+saved_o[k];
  next_write=write_index;found=0;
  for(j=1;j<=12;j=j+1) begin
   candidate=write_index+j;
   if(candidate>=12) candidate=candidate-12;
   if(!found && !saved_o[candidate] && candidate!=read_index) begin
    next_write=candidate;found=1;
   end
  end
 end
 always @(posedge clk) begin
  if(reset) begin
   write_index<=0;read_index<=2;latest<=0;have_written<=0;have_read<=0;
   request_meta<=0;request_sync<=0;command_ack_o<=0;result_o<=0;
   saved_o<=0;compare_o<=0;mode_o<=0;order_o<=0;capture_ordinal<=0;
  end else begin
   request_meta<=command_toggle_i;request_sync<=request_meta;
   if(write_done) begin latest<=write_index;write_index<=next_write;have_written<=1;end
   if(read_boundary) begin
    if(mode_o==0 && (have_written || write_done)) begin
     read_index<=write_done ? write_index : latest;have_read<=1;
    end
    if(request_sync!=command_ack_o) begin
     command_ack_o<=request_sync;result_o<=0;
     case(command_i[7:0])
      0:begin mode_o<=0;compare_o<=0;end
      1:begin
       if(!have_read || mode_o!=0) result_o<=4;
       else if(saved_o[read_index]) result_o<=4;
       else if(count>=8) result_o<=7;
       else begin saved_o[read_index]<=1;capture_ordinal<=capture_ordinal+1'b1;order_o[read_index*32+:32]<=capture_ordinal+1'b1;end
      end
      2:begin
       if(!have_read) result_o<=4;
       else begin mode_o<=1;read_index<=read_index;end
      end
      3:begin
       if(command_i[15:8]>=12 || !saved_o[command_i[11:8]]) result_o<=2;
       else begin mode_o<=2;compare_o<=0;read_index<=command_i[11:8];end
      end
      4:begin
       if(command_i[31:28]!=0 || command_i[27:16]==0 || (command_i[27:16] & ~saved_o)!=0) result_o<=2;
       else begin mode_o<=3;compare_o<=command_i[27:16];end
      end
      5:begin saved_o<=0;compare_o<=0;mode_o<=0;end
      
      6:begin
       if(command_i[15:8]>=12 || !saved_o[command_i[11:8]]) result_o<=2;
       else begin saved_o[command_i[11:8]]<=0;compare_o<=0;mode_o<=0;end
      end
      default:result_o<=2;
     endcase
    end
   end
  end
 end
endmodule
