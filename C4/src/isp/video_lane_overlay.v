`timescale 1ns/1ps
// Streaming image-space lane tracker. One clock of latency on every mode.
// cfg: flags, ROI y LE16, brightness, min stripe width, max width, min rows.
// Row RAM stores the previous frame; no full image buffer or divider required.
module video_lane_overlay #(
    parameter IMAGE_WIDTH=1280, IMAGE_HEIGHT=720, VS_ACTIVE=1'b0
)(
    input wire clk, rst_n,
    input wire [55:0] config_i,
    input wire [55:0] guide_i,
    input wire [23:0] original_i, processed_i,
    input wire hs_i, vs_i, de_i,
    output wire [23:0] rgb_o,
    output reg hs_o, vs_o, de_o
);
    reg [15:0] x,y;
    reg de_d,vs_d;
    reg [55:0] config_d,guide_d;
    reg [7:0] row_min,row_max,adaptive_threshold;
    reg row_seen,adaptive_valid;
    reg [15:0] run_start,run_length;
    reg [7:0] previous_gray,run_background,run_peak;
    reg [5:0] streak_left,streak_right;
    reg [15:0] best_left,best_right,left_cost,right_cost;
    reg found_left,found_right;
    reg [15:0] track_left,track_right;
    reg [5:0] gap_left,gap_right;
    reg [35:0] rows[0:IMAGE_HEIGHT-1];
    reg [35:0] row_q;
    reg [15:0] support_left,support_right;
    reg good_left,good_right,have_frame;
    reg [15:0] stable_left_rows,stable_right_rows;
    reg path_ready;
    reg [23:0] base_q;
    reg [15:0] x_q;
    reg overlay_q,roi_q;
    reg [15:0] y_q,left_guide_q,right_guide_q,active_top_q;
    wire [15:0] center=run_start+(run_length>>1);
    wire enabled=config_i[0], mask_mode=config_i[1], show_roi=config_i[2];
    wire [15:0] roi_y=config_i[23:8];
    wire [7:0] brightness=config_i[31:24], min_width=config_i[39:32];
    wire [7:0] max_width=config_i[47:40], min_rows=config_i[55:48];

    wire strict=(config_i[5]===1'b1);
    wire guides=config_i[6];
    wire [15:0] road_center=guide_i[15:0],horizon=guide_i[31:16];
    wire [7:0] divergence=guide_i[39:32],tolerance=guide_i[47:40],contrast=guide_i[55:48];
    wire [15:0] depth=y>horizon ? y-horizon : 0;
    wire [23:0] perspective_product=depth*divergence;
    wire [15:0] half_lane=perspective_product[22:7];
    wire [15:0] expected_left=road_center>half_lane ? road_center-half_lane : 0;
    wire [16:0] right_sum={1'b0,road_center}+{1'b0,half_lane};
    wire [15:0] expected_right=right_sum<IMAGE_WIDTH ? right_sum[15:0] : IMAGE_WIDTH-1;
    wire [23:0] tolerance_product=depth*tolerance;
    wire [15:0] corridor=4+(tolerance>>2)+(tolerance_product>>9);
    wire [15:0] strict_left=expected_left>corridor ? expected_left-corridor : 0;
    wire [16:0] strict_right_sum={1'b0,expected_right}+{1'b0,corridor};
    wire [15:0] strict_right=strict_right_sum<IMAGE_WIDTH ? strict_right_sum[15:0] : IMAGE_WIDTH-1;
    wire frame_start=(vs_i==VS_ACTIVE && vs_d!=VS_ACTIVE);
    wire changed=(config_i!=config_d) || (strict && guide_i!=guide_d);
    // Trapezoid expands by two pixels on each side for every image row.
    wire [15:0] spread=(y>=roi_y) ? ((y-roi_y)<<1) : 0;
    wire [15:0] left_limit=(spread<IMAGE_WIDTH*3/8) ? IMAGE_WIDTH*3/8-spread : 0;
    wire [15:0] right_limit=(IMAGE_WIDTH*5/8+spread<IMAGE_WIDTH-1) ? IMAGE_WIDTH*5/8+spread : IMAGE_WIDTH-1;
    wire full_frame=config_i[4];
    wire [15:0] active_top=strict ? ((roi_y>horizon+8) ? roi_y : horizon+8) : (full_frame ? 16'd0 : roi_y);
    wire [15:0] active_left=strict ? strict_left : (full_frame ? 16'd0 : left_limit);
    wire [15:0] active_right=strict ? strict_right : (full_frame ? IMAGE_WIDTH-1 : right_limit);
    wire in_roi=y>=active_top && y<IMAGE_HEIGHT && x>=active_left && x<=active_right;

    // This permits dim lane markings without turning a uniformly bright road white.
    wire adaptive=config_i[3];
    wire [8:0] row_sum={1'b0,row_min}+{1'b0,row_max};
    wire [7:0] effective_threshold=adaptive ? ((strict && brightness>adaptive_threshold) ? brightness : adaptive_threshold) : brightness;
    wire bright=de_i && in_roi && (!adaptive || adaptive_valid) &&
        original_i[15:8]>=effective_threshold;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            row_min<=255;row_max<=0;row_seen<=0;
            adaptive_threshold<=180;adaptive_valid<=0;
        end else if(changed || frame_start) begin
            row_min<=255;row_max<=0;row_seen<=0;
            adaptive_threshold<=brightness;adaptive_valid<=0;
        end else begin
            if(de_i && in_roi) begin
                row_seen<=1;
                if(original_i[15:8]<row_min) row_min<=original_i[15:8];
                if(original_i[15:8]>row_max) row_max<=original_i[15:8];
            end
            if(de_d && !de_i) begin
                adaptive_valid<=row_seen && ({1'b0,row_max}>={1'b0,row_min}+9'd12);
                adaptive_threshold<=row_sum[8:1];
                row_min<=255;row_max<=0;row_seen<=0;
            end
        end
    end
    wire [15:0] predicted_left_error=center>expected_left ? center-expected_left : expected_left-center;
    wire [15:0] predicted_right_error=center>expected_right ? center-expected_right : expected_right-center;
    wire [15:0] prior_left_error=center>row_q[15:0] ? center-row_q[15:0] : row_q[15:0]-center;
    wire [15:0] prior_right_error=center>row_q[31:16] ? center-row_q[31:16] : row_q[31:16]-center;
    wire [15:0] current_left_error=best_left>row_q[15:0] ? best_left-row_q[15:0] : row_q[15:0]-best_left;
    wire [15:0] current_right_error=best_right>row_q[31:16] ? best_right-row_q[31:16] : row_q[31:16]-best_right;
    wire [15:0] motion_limit=8+(depth>>6);
    wire local_contrast=({1'b0,run_peak}>={1'b0,run_background}+{1'b0,contrast}) &&
        ({1'b0,run_peak}>={1'b0,original_i[15:8]}+{1'b0,contrast});
    wire [15:0] perspective_width=3+(depth>>2);
    wire strict_stripe=!strict || (local_contrast && run_length<=perspective_width);
    wire left_geometry=!strict || (predicted_left_error<=corridor &&
        (gap_left==32 || (center>track_left ? center-track_left : track_left-center)<=4+(gap_left>>2)));
    wire right_geometry=!strict || (predicted_right_error<=corridor &&
        (gap_right==32 || (center>track_right ? center-track_right : track_right-center)<=4+(gap_right>>2)));
    wire [15:0] target_left=strict ? ((have_frame && row_q[32]) ? row_q[15:0] : road_center) : (gap_left<32 ? track_left : IMAGE_WIDTH/2);
    wire [15:0] target_right=strict ? ((have_frame && row_q[33]) ? row_q[31:16] : road_center) : (gap_right<32 ? track_right : IMAGE_WIDTH/2);
    wire [15:0] cost_left=center>target_left ? center-target_left : target_left-center;
    wire [15:0] cost_right=center>target_right ? center-target_right : target_right-center;
    wire stripe=run_length>=min_width && run_length<=max_width;
    // Validity is stored per row, and qualified by actual observed support per frame.
    wire valid_left=found_left && (!strict || streak_left>=7);
    wire valid_right=found_right && (!strict || streak_right>=7);
    wire [15:0] row_left=found_left ? best_left : track_left;
    wire [15:0] row_right=found_right ? best_right : track_right;
    wire confirmed_left=valid_left && (!strict || (have_frame && row_q[32] && current_left_error<=motion_limit));
    wire confirmed_right=valid_right && (!strict || (have_frame && row_q[33] && current_right_error<=motion_limit));
    // Do not reset the RAM: synthesis can infer block RAM. have_frame guards it.
    always @(posedge clk) begin
        if(y<IMAGE_HEIGHT) begin
            row_q<=rows[y];
            if(de_d && !de_i)
                rows[y]<={confirmed_right,confirmed_left,valid_right && y>=active_top,valid_left && y>=active_top,row_right,row_left};
        end
    end
    wire left_hit=good_left && (strict ? row_q[34] : row_q[32]) &&
        ({1'b0,x_q}+2 >= {1'b0,row_q[15:0]}) && ({1'b0,x_q} <= {1'b0,row_q[15:0]}+2);
    wire right_hit=good_right && (strict ? row_q[35] : row_q[33]) &&
        ({1'b0,x_q}+2 >= {1'b0,row_q[31:16]}) && ({1'b0,x_q} <= {1'b0,row_q[31:16]}+2);
    wire guide_hit=enabled && !mask_mode && guides && strict && y_q>=active_top_q &&
        ((x_q==road_center) || (x_q==left_guide_q) || (x_q==right_guide_q));
    assign rgb_o=!de_o ? 24'd0 : overlay_q && left_hit ? 24'h00ff40 :
        overlay_q && right_hit ? 24'hff4040 : guide_hit ? 24'hffc000 : roi_q ? 24'h00c0ff : base_q;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            x<=0;y<=0;de_d<=0;vs_d<=VS_ACTIVE;config_d<=0;guide_d<=0;
            previous_gray<=0;run_background<=0;run_peak<=0;streak_left<=0;streak_right<=0;
            y_q<=0;left_guide_q<=0;right_guide_q<=0;active_top_q<=0;
            run_start<=0;run_length<=0;best_left<=0;best_right<=0;
            left_cost<=16'hffff;right_cost<=16'hffff;found_left<=0;found_right<=0;
            track_left<=0;track_right<=0;gap_left<=32;gap_right<=32;
            support_left<=0;support_right<=0;good_left<=0;good_right<=0;have_frame<=0;
            stable_left_rows<=0;stable_right_rows<=0;path_ready<=0;
            hs_o<=0;vs_o<=VS_ACTIVE;de_o<=0;base_q<=0;x_q<=0;overlay_q<=0;roi_q<=0;
        end else begin
            de_d<=de_i;vs_d<=vs_i;config_d<=config_i;guide_d<=guide_i;
            previous_gray<=original_i[15:8];
            y_q<=y;left_guide_q<=expected_left;right_guide_q<=expected_right;active_top_q<=active_top;
            hs_o<=hs_i;vs_o<=vs_i;de_o<=de_i;x_q<=x;
            base_q<=enabled ? (mask_mode ? (bright ? 24'hffffff : 24'd0) : original_i) : processed_i;
            overlay_q<=enabled && have_frame && !changed && !mask_mode;
            roi_q<=enabled && show_roi && in_roi && (y==active_top || x==active_left || x==active_right || (full_frame && y==IMAGE_HEIGHT-1));
            if(changed || frame_start) begin
                x<=0;y<=0;run_length<=0;found_left<=0;found_right<=0;
                left_cost<=16'hffff;right_cost<=16'hffff;gap_left<=32;gap_right<=32;
                support_left<=0;support_right<=0;streak_left<=0;streak_right<=0;
                path_ready<=!changed && stable_left_rows>=min_rows && stable_right_rows>=min_rows;
                stable_left_rows<=0;stable_right_rows<=0;
                good_left<=!changed && support_left>=min_rows;
                good_right<=!changed && support_right>=min_rows;
                have_frame<=!changed;
            end else begin
                if(de_i) x<=x+1'b1;
                if(bright) begin
                    if(run_length==0) begin run_start<=x;run_background<=previous_gray;run_peak<=original_i[15:8];end
                    else if(original_i[15:8]>run_peak) run_peak<=original_i[15:8];
                    if(run_length<IMAGE_WIDTH) run_length<=run_length+1'b1;
                end else begin
                    run_length<=0;
                    // Reject clipped stripes at the last pixel / ROI edge.
                    if(de_i && in_roi && stripe && strict_stripe && run_length!=0) begin
                        if(center<(strict ? road_center : IMAGE_WIDTH/2) && left_geometry && cost_left<left_cost &&
                           (strict || gap_left==32 || cost_left<=IMAGE_WIDTH/16)) begin
                            found_left<=1;best_left<=center;left_cost<=cost_left;
                        end
                        if(center>=(strict ? road_center : IMAGE_WIDTH/2) && right_geometry && cost_right<right_cost &&
                           (strict || gap_right==32 || cost_right<=IMAGE_WIDTH/16)) begin
                            found_right<=1;best_right<=center;right_cost<=cost_right;
                        end
                    end
                end
                if(de_d && !de_i) begin
                    x<=0;y<=y+1'b1;found_left<=0;found_right<=0;
                    if(confirmed_left)stable_left_rows<=stable_left_rows+1'b1;
                    if(confirmed_right)stable_right_rows<=stable_right_rows+1'b1;
                    left_cost<=16'hffff;right_cost<=16'hffff;
                    if(found_left) begin track_left<=best_left;gap_left<=0;support_left<=support_left+1'b1; if(streak_left<63)streak_left<=streak_left+1'b1;end
                    else begin streak_left<=0;if(gap_left<32) gap_left<=gap_left+1'b1;end
                    if(found_right) begin track_right<=best_right;gap_right<=0;support_right<=support_right+1'b1; if(streak_right<63)streak_right<=streak_right+1'b1;end
                    else begin streak_right<=0;if(gap_right<32) gap_right<=gap_right+1'b1;end
                end
            end
        end
    end
endmodule
