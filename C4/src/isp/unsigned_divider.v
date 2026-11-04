`timescale 1ns/1ps

module unsigned_divider(
    input wire clk, rst_n, start_i,
    input wire [31:0] dividend_i, divisor_i,
    output reg busy_o, done_o,
    output reg [31:0] quotient_o, remainder_o
);
    reg [31:0] q, d, r;
    reg [5:0] count;
    wire [32:0] trial={r,q[31]};
    wire take=trial>={1'b0,d};
    wire [31:0] next_r=take ? trial-{1'b0,d} : trial;
    wire [31:0] next_q={q[30:0],take};
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin q<=0; d<=1; r<=0; count<=0; busy_o<=0; done_o<=0; quotient_o<=0; remainder_o<=0; end
        else begin
            done_o<=0;
            if(start_i && !busy_o) begin q<=dividend_i; d<=divisor_i; r<=0; count<=0; busy_o<=1; end
            else if(busy_o) begin
                q<=next_q; r<=next_r; count<=count+1'b1;
                if(count==31) begin busy_o<=0; done_o<=1; quotient_o<=next_q; remainder_o<=next_r; end
            end
        end
    end
endmodule
