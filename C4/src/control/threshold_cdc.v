`timescale 1ns/1ps

/* parameterize mailbox width/reset to reuse for runtime ISP flags. */
module threshold_cdc #(parameter WIDTH=12, parameter RESET_VALUE=128)(
    input wire src_clk, src_rst_n,
    input wire src_request_i,
    input wire [WIDTH-1:0] src_value_i,
    output wire busy_o,
    output reg [WIDTH-1:0] applied_o,
    input wire pixel_clk, pixel_rst_n, frame_blank_i,
    output reg [WIDTH-1:0] pixel_value_o
);
    reg [WIDTH-1:0] mailbox;
    reg request_toggle, acknowledge;
    reg ack_meta, ack_sync, ack_seen;
    reg req_meta, req_sync;
    /* 等 applied_o 完成更新后才释放 busy，防止源端重复发送旧快照。 */
    assign busy_o=(request_toggle!=ack_sync) || (ack_seen!=ack_sync);
    always @(posedge src_clk or negedge src_rst_n) begin
        if (!src_rst_n) begin
            mailbox<=RESET_VALUE; request_toggle<=0; ack_meta<=0; ack_sync<=0; ack_seen<=0; applied_o<=RESET_VALUE;
        end else begin
            ack_meta<=acknowledge; ack_sync<=ack_meta;
            if (ack_seen!=ack_sync) begin ack_seen<=ack_sync; applied_o<=mailbox; end
            if (src_request_i && !busy_o) begin mailbox<=src_value_i; request_toggle<=~request_toggle; end
        end
    end
    always @(posedge pixel_clk or negedge pixel_rst_n) begin
        if (!pixel_rst_n) begin req_meta<=0; req_sync<=0; acknowledge<=0; pixel_value_o<=RESET_VALUE; end
        else begin
            req_meta<=request_toggle; req_sync<=req_meta;
            if ((req_sync!=acknowledge) && frame_blank_i) begin
                pixel_value_o<=mailbox;
                acknowledge<=req_sync;
            end
        end
    end
endmodule
