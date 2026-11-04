`timescale 1ns/1ps
// One 640-pixel RGB565 scanline fetch. Result beats must be stored in a
// non-displayed line buffer. The consumer publishes only on line_done.
module fatigue_line_reader #(
 parameter [31:0] BASE0=32'h03000000,BASE1=32'h03100000,
 parameter WIDTH=640,HEIGHT=360,LINE_BYTES=WIDTH*2,BEATS=(LINE_BYTES+15)/16
)(
 input wire clk,rst_n,
 input wire request,input wire bank,input wire [15:0] row,output wire request_ready,
 output wire avalid,input wire aready,output wire [31:0] addr,output wire [7:0] len,output wire [2:0] size,output wire [1:0] burst,output wire write,
 input wire rvalid,output wire rready,input wire [127:0] rdata,input wire [1:0] rresp,input wire rlast,
 output wire beat_valid,input wire beat_ready,output wire [7:0] beat_index,output wire [127:0] beat_data,
 output reg line_done,output reg line_error
);
 localparam IDLE=0,ADDR=1,DATA=2,DRAIN=3;
 reg [1:0] state;reg [31:0] held_addr;reg [8:0] beat;reg bad;
 assign request_ready=state==IDLE;
 assign avalid=state==ADDR;assign addr=held_addr;assign len=BEATS-1;assign size=4;assign burst=1;assign write=0;
 assign beat_valid=state==DATA&&rvalid&&(rresp==0)&&!bad;
 assign rready=state==DRAIN ? 1'b1 : state==DATA ? ((rresp!=0||bad)?1'b1:beat_ready) : 1'b0;
 assign beat_index=beat[7:0];assign beat_data=rdata;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin state<=IDLE;held_addr<=0;beat<=0;bad<=0;line_done<=0;line_error<=0;end
  else begin
   line_done<=0;line_error<=0;
   case(state)
    IDLE:if(request)begin
     if(row>=HEIGHT)line_error<=1;
     else begin held_addr<=(bank?BASE1:BASE0)+row*LINE_BYTES;beat<=0;bad<=0;state<=ADDR;end
    end
    ADDR:if(avalid&&aready)state<=DATA;
    DATA:if(rvalid&&rready)begin
     if(rresp!=0)bad<=1;
     if(rlast)begin
      if(beat==BEATS-1 && !bad && rresp==0)line_done<=1;
      else line_error<=1;
      state<=IDLE;
     end else if(beat==BEATS-1)begin line_error<=1;state<=DRAIN;end
     else beat<=beat+1'b1;
    end
    DRAIN:if(rvalid&&rready&&rlast)state<=IDLE;
    default:state<=IDLE;
   endcase
  end
 end
endmodule
