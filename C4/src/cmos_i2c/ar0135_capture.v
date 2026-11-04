
module ar0135_capture #(
    parameter WIDTH=1280, HEIGHT=720, EMBEDDED_ROWS=2
) (
    input wire pclk, input wire rst_n, input wire configured,
    input wire fv, input wire lv, input wire [7:0] raw,
    output reg frame_valid, output reg pixel_valid, output reg [15:0] rgb565
);
    reg [1:0] reset_sync, config_sync;
    /* Match the reference capture's registered FV/LV/data sampling before
       deriving pixel coordinates. All three signals must have equal delay. */
    reg fv_s, lv_s;
    reg [7:0] raw_s;
    reg armed, active, fv_d, lv_d;
    reg [15:0] x, y;
    always @(posedge pclk or negedge rst_n) begin
        if (!rst_n) begin
            reset_sync<=0; config_sync<=0;
            fv_s<=0; lv_s<=0; raw_s<=0;
        end else begin
            reset_sync<={reset_sync[0],1'b1}; config_sync<={config_sync[0],configured};
            fv_s<=fv; lv_s<=lv; raw_s<=raw;
        end
    end
    always @(posedge pclk or negedge rst_n) begin
        if(!rst_n) begin
            armed<=0; active<=0; fv_d<=0; lv_d<=0; x<=0; y<=0;
            frame_valid<=0; pixel_valid<=0; rgb565<=0;
        end else if (!reset_sync[1] || !config_sync[1]) begin
            armed<=0; active<=0; fv_d<=fv_s; lv_d<=lv_s; x<=0; y<=0;
            frame_valid<=0; pixel_valid<=0; rgb565<=0;
        end else begin
            fv_d<=fv_s; lv_d<=lv_s;
            if(!fv_s) begin armed<=1; active<=0; x<=0; y<=0; end
            else begin
                if(!fv_d && armed) active<=1;
                if(lv_s) begin if(x!=16'hffff) x<=x+1'b1; end
                else begin
                    x<=0;
                    if(lv_d && y!=16'hffff) y<=y+1'b1;
                end
            end
            frame_valid<=active && fv_s;
            pixel_valid<=active && fv_s && lv_s && x<WIDTH &&
                         y>=EMBEDDED_ROWS && y<EMBEDDED_ROWS+HEIGHT;
            rgb565<={raw_s[7:3],raw_s[7:2],raw_s[7:3]};
        end
    end
endmodule
