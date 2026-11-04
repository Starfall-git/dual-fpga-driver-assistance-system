`timescale 1ns/1ps
// UDP payload parser. Writes target a non-displayed frame buffer. A frame is
// published only after every in-order packet and the entire CRC32 validate.
// The UDP/IP/MAC layer must check packet integrity and hold byte data under stall.
module fatigue_udp_frame #(
 parameter WIDTH=640, HEIGHT=360, CHUNK=1152,
 parameter FRAME_BYTES=WIDTH*HEIGHT*2, CHUNKS=(FRAME_BYTES+CHUNK-1)/CHUNK
)(
 input wire clk,rst_n,
 input wire in_valid,in_sop,in_eop,input wire [7:0] in_data,
 output wire in_ready,
 output wire wr_valid,input wire wr_ready,output wire [31:0] wr_addr,output wire [7:0] wr_data,
 output reg frame_valid,output reg [31:0] frame_id,
 output reg frame_error,output reg [3:0] diag_error
);
 reg [15:0] pos,payload_count,chunk_index,chunk_count,w,h,expected_chunk;
 reg [31:0] magic,seq,byte_count,expected_seq,expected_crc,packet_size,packet_crc,crc;
 reg [7:0] version,flags;
 reg active,packet_ok;
 wire payload=pos>=26;
 wire within_packet=payload_count<((chunk_index==CHUNKS-1)?(FRAME_BYTES-CHUNK*(CHUNKS-1)):CHUNK);
 assign wr_valid=in_valid&&!in_sop&&payload&&packet_ok&&active&&within_packet&&byte_count<FRAME_BYTES;
 assign wr_addr=byte_count;
 assign wr_data=in_data;
 assign in_ready=!(payload&&packet_ok&&active&&within_packet&&byte_count<FRAME_BYTES)||wr_ready;
 wire take=in_valid&&in_ready;
 function [31:0] crc_byte;
  input [31:0] old;input [7:0] value;
  reg [31:0] c;integer i;
  begin c=old^{24'b0,value};for(i=0;i<8;i=i+1)c=c[0] ? (c>>1)^32'hedb88320 : c>>1;crc_byte=c;end
 endfunction
 wire [31:0] crc_next=crc_byte(crc,in_data);
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n)begin
   pos<=0;payload_count<=0;chunk_index<=0;chunk_count<=0;w<=0;h<=0;expected_chunk<=0;
   magic<=0;seq<=0;byte_count<=0;expected_seq<=0;expected_crc<=0;packet_size<=0;packet_crc<=0;crc<=32'hffffffff;
   version<=0;flags<=0;active<=0;packet_ok<=0;frame_valid<=0;frame_id<=0;frame_error<=0;diag_error<=0;
  end else begin
   frame_valid<=0;frame_error<=0;
   if(take)begin
    if(in_sop)begin
     if(pos!=0)begin active<=0;frame_error<=1;diag_error[2]<=1;end
     magic<={24'b0,in_data};pos<=1;payload_count<=0;packet_ok<=0;
     seq<=0;chunk_index<=0;chunk_count<=0;w<=0;h<=0;packet_size<=0;packet_crc<=0;
    end else begin
     pos<=pos+1'b1;
     case(pos)
      1,2,3:magic<={magic[23:0],in_data};
      4:version<=in_data; 5:flags<=in_data;
      6,7,8,9:seq<={seq[23:0],in_data};
      10,11:chunk_index<={chunk_index[7:0],in_data};
      12,13:chunk_count<={chunk_count[7:0],in_data};
      14,15:w<={w[7:0],in_data};
      16,17:h<={h[7:0],in_data};
      18,19,20,21:packet_size<={packet_size[23:0],in_data};
      22,23,24:packet_crc<={packet_crc[23:0],in_data};
      25:begin
       packet_crc<={packet_crc[23:0],in_data};
       if(magic==32'h46564944 && version==1 && flags==0 && w==WIDTH && h==HEIGHT && packet_size==FRAME_BYTES && chunk_count==CHUNKS &&
          ((chunk_index==0) || (active && chunk_index==expected_chunk && seq==expected_seq && {packet_crc[23:0],in_data}==expected_crc)))begin
        packet_ok<=1;
        if(chunk_index==0)begin active<=1;expected_seq<=seq;expected_crc<={packet_crc[23:0],in_data};byte_count<=0;crc<=32'hffffffff;end
       end else begin
        if(magic!=32'h46564944 || version!=1 || flags!=0 || w!=WIDTH || h!=HEIGHT || packet_size!=FRAME_BYTES || chunk_count!=CHUNKS)diag_error[0]<=1;
        else diag_error[1]<=1;
        active<=0;packet_ok<=0;frame_error<=1;
       end
      end
      default:if(payload&&packet_ok&&active)begin
       if(within_packet && byte_count<FRAME_BYTES)begin
        byte_count<=byte_count+1'b1;payload_count<=payload_count+1'b1;crc<=crc_next;
       end else begin active<=0;packet_ok<=0;frame_error<=1;end
      end
     endcase
     if(in_eop)begin
      pos<=0;
      if(!payload || !packet_ok || !active || byte_count>=FRAME_BYTES ||
         payload_count+1!=((chunk_index==CHUNKS-1)?(FRAME_BYTES-CHUNK*(CHUNKS-1)):CHUNK))begin
       active<=0;frame_error<=1;diag_error[2]<=1;
      end else if(chunk_index==CHUNKS-1)begin
       if(byte_count+1==FRAME_BYTES && (crc_next^32'hffffffff)==expected_crc)begin frame_id<=expected_seq;frame_valid<=1;end
       else begin frame_error<=1;diag_error[3]<=1;end
       active<=0;expected_chunk<=0;
      end else expected_chunk<=chunk_index+1'b1;
     end
    end
   end
  end
 end
endmodule
