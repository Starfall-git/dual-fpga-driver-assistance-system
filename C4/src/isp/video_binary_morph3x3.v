`timescale 1ns/1ps
/* Centered 3x3 square on binary masks and 8-bit strength, with two continuous-clock line
   buffers. The output refers to the CENTER, not the bottom-right sample.
   Delay = LINE_CLOCKS+3, including RAM read and reduction registers.
   DE=0 samples are neutral: 0 for dilation, 1 for erosion. At least two
   horizontal blank clocks and two bottom blank lines are required by the
   complete two-stage chain. RAM contents from before reset are masked. */
module video_binary_morph3x3 #(
    parameter LINE_CLOCKS=1650, STAGE=0, VS_ACTIVE=1'b0
)(
    input wire clk, rst_n, bit_i, hs_i, vs_i, de_i,
    input wire [2:0] mode_i,
    input wire [7:0] strength_i,
    output reg [7:0] strength_o,
    output reg bit_o, hs_o, vs_o, de_o,
    output reg [2:0] mode_o
);
    localparam AW=$clog2(LINE_CLOCKS);
    localparam FW=$clog2(2*LINE_CLOCKS+1);
    reg [9:0] line1[0:LINE_CLOCKS-1], line2[0:LINE_CLOCKS-1];
    reg [AW-1:0] address, address_q;
    reg [FW-1:0] filled;
    reg [9:0] top_q, mid_q, bot_q;
    reg [9:0] top_l2, top_l1, mid_l2, mid_l1, bot_l2, bot_l1;
    wire [5:0] control_pre;
    video_stream_delay #(.WIDTH(6),.DELAY(LINE_CLOCKS+2),
        .RESET_VALUE({1'b1,VS_ACTIVE,1'b0,3'd0})) u_control(
        .clk(clk),.rst_n(rst_n),.data_i({hs_i,vs_i,de_i,mode_i}),.data_o(control_pre));

    
    function [1:0] operation;
        input [2:0] mode;
        begin
            case(mode)
                1: operation=(STAGE==0) ? 1 : 2;
                2: operation=(STAGE==0) ? 2 : 1;
                3: operation=(STAGE==0) ? 1 : 0;
                4: operation=(STAGE==0) ? 2 : 0;
                default: operation=0;
            endcase
        end
    endfunction
    wire [1:0] op=operation(control_pre[2:0]);
    wire erode=(op==2);
    function sample;
        input [9:0] pixel;
        input neutral;
        begin sample=pixel[9] ? pixel[0] : neutral; end
    endfunction
    wire [8:0] neighborhood={sample(top_l2,erode),sample(top_l1,erode),sample(top_q,erode),
                            sample(mid_l2,erode),sample(mid_l1,erode),sample(mid_q,erode),
                            sample(bot_l2,erode),sample(bot_l1,erode),sample(bot_q,erode)};
    function [7:0] gray_sample;
        input [9:0] pixel;
        input neutral;
        begin gray_sample=pixel[9] ? pixel[8:1] : {8{neutral}}; end
    endfunction
    function [7:0] extreme;
        input [7:0] a,b;
        input minimum;
        begin extreme=((a<b)==minimum) ? a : b; end
    endfunction
    // Balanced reduction, same register/center as the binary mask.
    wire [7:0] g0=extreme(gray_sample(top_l2,erode),gray_sample(top_l1,erode),erode);
    wire [7:0] g1=extreme(gray_sample(top_q,erode),gray_sample(mid_l2,erode),erode);
    wire [7:0] g2=extreme(gray_sample(mid_l1,erode),gray_sample(mid_q,erode),erode);
    wire [7:0] g3=extreme(gray_sample(bot_l2,erode),gray_sample(bot_l1,erode),erode);
    wire [7:0] gray_result=extreme(extreme(extreme(g0,g1,erode),extreme(g2,g3,erode),erode),gray_sample(bot_q,erode),erode);
    always @(posedge clk) begin
        if(rst_n) begin
            mid_q<=filled>=LINE_CLOCKS ? line1[address] : 10'd0;
            top_q<=filled>=2*LINE_CLOCKS ? line2[address] : 10'd0;
            line1[address]<={de_i && vs_i!=VS_ACTIVE,strength_i,bit_i};
            line2[address_q]<=mid_q;
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            address<=0; address_q<=0; filled<=0; bot_q<=0;
            top_l2<=0; top_l1<=0; mid_l2<=0; mid_l1<=0; bot_l2<=0; bot_l1<=0;
            bit_o<=0; strength_o<=0; hs_o<=1; vs_o<=VS_ACTIVE; de_o<=0; mode_o<=0;
        end else begin
            address<=address==LINE_CLOCKS-1 ? 0 : address+1'b1;
            address_q<=address;
            if(filled<2*LINE_CLOCKS) filled<=filled+1'b1;
            bot_q<={de_i && vs_i!=VS_ACTIVE,strength_i,bit_i};
            top_l2<=top_l1; top_l1<=top_q;
            mid_l2<=mid_l1; mid_l1<=mid_q;
            bot_l2<=bot_l1; bot_l1<=bot_q;
            hs_o<=control_pre[5]; vs_o<=control_pre[4]; de_o<=control_pre[3];
            mode_o<=control_pre[2:0];
            strength_o<=control_pre[3] ? ((op==0) ? mid_l1[8:1] : gray_result) : 8'd0;
            bit_o<=control_pre[3] && ((op==0) ? mid_l1[0] : (erode ? &neighborhood : |neighborhood));
        end
    end
endmodule
