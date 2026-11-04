`timescale 1ns/1ps
// Road paint tracker: paired brightness flanks, perspective width, row continuity,
// road divergence and bounded temporal innovations. Optional supported-span fit.
module video_lane_curve #(
    parameter IMAGE_WIDTH=1280, IMAGE_HEIGHT=720, BAND_SHIFT=6, VS_ACTIVE=1'b0
)(
    input wire clk,rst_n,
    input wire [55:0] config_i,guide_i,model_i,
    input wire [55:0] rect_i,
    input wire [7:0] edge_i,
    input wire [23:0] original_i,processed_i,
    /* FUSION-synchronize external fatigue state. */
    input wire [8:0] fatigue_state_async_i,
    input wire hs_i,vs_i,de_i,
    output reg [23:0] rgb_o,
    output reg hs_o,vs_o,de_o, output reg [15:0] assist_status_o
);
    localparam BANDS=(IMAGE_HEIGHT+(1<<BAND_SHIFT)-1)>>BAND_SHIFT;
    localparam SLOTS=2*BANDS;
    localparam IDLE=0,MEAN=1,MEAN_WAIT=2,QUALIFY=3,FILL=4,FILL_WAIT=5,KALMAN=6,KALMAN_WAIT=7,FINISH=8,CHECK=9,TRACE=10,STABLE_CHECK=11,PAIR_CHECK=12;
    wire enabled=config_i[0] && model_i[0];
    wire use_edges=model_i[1],use_kalman=model_i[2];
    wire edge_points=model_i[3],draw_fit=model_i[4],edge_background=model_i[5],edge_only=model_i[6];
    wire [15:0] roi_top=config_i[23:8],road_center=guide_i[15:0],horizon=guide_i[31:16];
    reg [8:0] fatigue_meta, fatigue_onehot;
    reg [3:0] fatigue_sync;
    always @(posedge clk or negedge rst_n) begin
      if(!rst_n) begin fatigue_meta<=9'b001000000; fatigue_onehot<=9'b001000000; end
      else begin fatigue_meta<=fatigue_state_async_i; fatigue_onehot<=fatigue_meta; end
    end
    always @* begin
      case(fatigue_onehot)
        9'b000000001:fatigue_sync=0; 9'b000000010:fatigue_sync=1;
        9'b000000100:fatigue_sync=2; 9'b000001000:fatigue_sync=3;
        9'b000010000:fatigue_sync=4; 9'b000100000:fatigue_sync=5;
        9'b010000000:fatigue_sync=7; 9'b100000000:fatigue_sync=8;
        default:fatigue_sync=6;
      endcase
    end
    reg [15:0] x,y;
    wire rect_enabled=(rect_i[0]===1'b1);
    /* 2026-10-26 / 100: stable demonstration mode uses the configured
       trapezoid and continuous edge fit. Legacy paint/debug modes keep their
       observed span. The ROI supplies display height, never fake observations. */
    wire stable_road=edge_points && rect_enabled && draw_fit;
    wire [15:0] rect_top_width=rect_i[55:40]==0 ? rect_i[23:8] : rect_i[55:40];
    reg [31:0] roi_width_q16,roi_step;
    reg roi_pending,roi_start;
    wire roi_busy,roi_done;
    wire [31:0] roi_quotient;
    wire [15:0] row_roi_width=y==IMAGE_HEIGHT-1 ? rect_i[23:8] : roi_width_q16[31:16];
    wire [15:0] rect_left=(IMAGE_WIDTH-row_roi_width)>>1;
    wire [15:0] rect_right=rect_left+row_roi_width-1'b1;
    wire [15:0] rect_top=IMAGE_HEIGHT-rect_i[39:24];
    wire [15:0] edge_top=rect_enabled && rect_top>roi_top ? rect_top : roi_top;
    wire [15:0] top_y=edge_points ? edge_top : (roi_top>horizon+8 ? roi_top : horizon+8);
    wire [7:0] edge_threshold=model_i[15:8],q_noise=model_i[23:16],r_noise=model_i[31:24];
    wire [7:0] hold_frames=model_i[39:32],min_bands=model_i[47:40],search_radius=model_i[55:48];
    /* LANE-restore the configured hold; a long static hold can lag bends. */
    reg [55:0] cfg_d,guide_d,model_d,rect_d;
    reg de_d,vs_d;
    wire frame_start=vs_i==VS_ACTIVE && vs_d!=VS_ACTIVE;
    wire changed=config_i!=cfg_d || (!edge_points && guide_i!=guide_d) || model_i!=model_d || ((rect_i[0]===1'b1 || rect_d[0]===1'b1) && (rect_i[55:8]!=rect_d[55:8] || rect_i[1:0]!=rect_d[1:0]));

    // One configuration-time divide; row width advances in Q16 during blanking.
    unsigned_divider u_roi_div(.clk(clk),.rst_n(rst_n),.start_i(roi_start),
        .dividend_i(({16'd0,rect_i[23:8]}-{16'd0,rect_top_width})<<16),
        .divisor_i(rect_i[39:24]>1 ? {16'd0,rect_i[39:24]}-1'b1 : 32'd1),
        .busy_o(roi_busy),.done_o(roi_done),.quotient_o(roi_quotient),.remainder_o());
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin roi_width_q16<=0;roi_step<=0;roi_pending<=0;roi_start<=0;end
        else begin
            roi_start<=0;
            if(changed)begin roi_pending<=rect_enabled;roi_width_q16<={rect_top_width,16'd0};end
            else begin
                if(roi_pending && !roi_busy && !roi_start)begin roi_start<=1;roi_pending<=0;end
                if(frame_start)roi_width_q16<={rect_top_width,16'd0};
                else if(de_d && !de_i && y>=rect_top)roi_width_q16<=roi_width_q16+roi_step;
            end
            if(roi_done)roi_step<=roi_quotient;
        end
    end

    function [15:0] expected_x;
        input [15:0] yy;
        input side;
        reg [15:0] d;
        reg [23:0] product;
        reg [16:0] half_width;
        reg signed [19:0] value;
        begin
            d=yy>horizon ? yy-horizon : 0;
            product=d*guide_i[39:32];
            half_width=product>>7;
            value=side ? $signed({1'b0,road_center})+$signed({1'b0,half_width}) :
                         $signed({1'b0,road_center})-$signed({1'b0,half_width});
            expected_x=edge_points ? 0 : value<0 ? 0 : value>=IMAGE_WIDTH ? IMAGE_WIDTH-1 : value;
        end
    endfunction
    function signed [23:0] clamp_q8;
        input signed [31:0] value;
        begin clamp_q8=value<0 ? 0 : value>((IMAGE_WIDTH-1)<<8) ? ((IMAGE_WIDTH-1)<<8) : value;end
    endfunction
    // Same-sign tangents bounded by the segment secant keep a cubic monotone
    // between its endpoints. A flat or turning segment gets zero tangent.
    function signed [31:0] limited_tangent;
        input signed [31:0] slope,delta;
        begin
            if(delta==0 || (slope<0)!=(delta<0))limited_tangent=0;
            else if(delta>0)limited_tangent=slope>delta ? delta : slope;
            else limited_tangent=slope<delta ? delta : slope;
        end
    endfunction

    reg signed [31:0] sums[0:SLOTS-1];
    // A band contains at most 2**BAND_SHIFT rows.
    reg [BAND_SHIFT:0] counts[0:SLOTS-1];
    // Fitted x values are clamped to [0,(IMAGE_WIDTH-1)*256].
    localparam FIT_BITS=$clog2(IMAGE_WIDTH*256+1)+1;
    reg signed [23:0] measured[0:SLOTS-1];
    reg signed [FIT_BITS-1:0] anchors[0:SLOTS-1],fit[0:SLOTS-1];
    reg observed[0:SLOTS-1],accepted[0:SLOTS-1],initialized[0:SLOTS-1];
    reg [BAND_SHIFT:0] support[0:SLOTS-1];
    reg [15:0] variance[0:SLOTS-1];
    reg [15:0] frame_rows[0:1],saved_rows[0:1];
    reg [7:0] band_count[0:1],misses[0:1];
    reg side_update[0:1],curve_valid[0:1];
    reg signed [31:0] first_position[0:1],last_position[0:1];
    reg [15:0] first_band[0:1],last_band[0:1];
    wire [31:0] span_left=((last_band[0]-first_band[0])<<BAND_SHIFT)*guide_i[39:32];
    wire [31:0] span_right=((last_band[1]-first_band[1])<<BAND_SHIFT)*guide_i[39:32];
    wire signed [31:0] min_span_left=((span_left>>9)<2 ? 32'd2 : (span_left>>9))<<8;
    wire signed [31:0] min_span_right=((span_right>>9)<2 ? 32'd2 : (span_right>>9))<<8;
    reg [$clog2(IMAGE_HEIGHT+1)-1:0] row_first[0:SLOTS-1],row_last[0:SLOTS-1];
    reg [15:0] first_y[0:1],last_y[0:1],display_first[0:1],display_last[0:1];
    reg [3:0] fsm;
    localparam SLOT_BITS=$clog2(SLOTS+1);
    reg [SLOT_BITS-1:0] slot;
    integer j,band,side;
    reg div_start;
    reg [31:0] dividend,divisor;
    wire div_busy,div_done;
    wire [31:0] quotient;
    unsigned_divider u_div(.clk(clk),.rst_n(rst_n),.start_i(div_start),
        .dividend_i(dividend),.divisor_i(divisor),.busy_o(div_busy),.done_o(div_done),
        .quotient_o(quotient),.remainder_o());
    reg negative;
    reg signed [31:0] interp_base,measurement_position;
    reg [15:0] predicted_variance;
    reg signed [31:0] innovation;
    reg signed [31:0] next_position;
    reg [31:0] covariance_product;
    /* 102: validate a complete proposed model before any anchor commits.
       Compare lane width at the same height, not far versus near pixel width. */
    wire signed [31:0] candidate_delta=$signed(fit[slot])-$signed(anchors[slot]);
    /* LANE-cap a local candidate bend rather than discard the entire side. */
    wire signed [31:0] bend_sum=$signed(fit[slot-2])+$signed(fit[slot+2]);
    wire signed [31:0] bend_error=bend_sum-($signed(fit[slot])<<<1);
    localparam signed [31:0] BEND_LIMIT=(IMAGE_WIDTH/128+4)<<8;
    wire signed [31:0] pair_left=side_update[0] ? $signed(fit[slot]) : $signed(anchors[slot]);
    wire signed [31:0] pair_right=side_update[1] ? $signed(fit[slot+1]) : $signed(anchors[slot+1]);
    wire signed [31:0] pair_width=pair_right-pair_left;
    wire signed [31:0] old_width=$signed(anchors[slot+1])-$signed(anchors[slot]);
    wire signed [31:0] width_margin=(old_width>>>2)>(8<<8) ? (old_width>>>2) : (8<<8);
    wire signed [31:0] left_change=pair_left-$signed(anchors[slot]);
    wire signed [31:0] right_change=pair_right-$signed(anchors[slot+1]);
    wire [31:0] left_error=left_change<0 ? -left_change : left_change;
    wire [31:0] right_error=right_change<0 ? -right_change : right_change;
    localparam STEP_PIXELS=(IMAGE_WIDTH/256+2)>8 ? 8 : IMAGE_WIDTH/256+2;
    function signed [31:0] bounded_step;
        input signed [31:0] delta;
        begin bounded_step=delta>(STEP_PIXELS<<8) ? (STEP_PIXELS<<8) : delta<-(STEP_PIXELS<<8) ? -(STEP_PIXELS<<8) : delta;end
    endfunction
    localparam BAND_INDEX_BITS=$clog2(BANDS+1)+1;
    reg signed [BAND_INDEX_BITS-1:0] lo,hi,aa,bb,idx_band;
    reg idx_side;
    integer k,anchor_y;
    reg signed [31:0] interpolation_numerator,measurement_residual;
    reg signed [31:0] neighbor_min,neighbor_max,spatial_position;
    wire [15:0] outlier_pixels=(search_radius>>2)<6 ? 16'd6 : search_radius>>2;
    wire use_neighbor_expected=(fsm==FILL && stable_road && curve_valid[slot&1] &&
        initialized[slot] && !accepted[slot] && (lo<0 || hi<0) && side_update[slot&1]);
    wire [15:0] expected_slot_row=((slot>>1)<<BAND_SHIFT)+(1<<(BAND_SHIFT-1));
    wire [15:0] expected_neighbor_row=(((lo>=0 ? lo : hi))<<BAND_SHIFT)+(1<<(BAND_SHIFT-1));
    wire [15:0] shared_expected=expected_x(use_neighbor_expected ? expected_neighbor_row : expected_slot_row,slot&1);
    wire signed [31:0] measured_position=($signed({1'b0,shared_expected})<<8)+$signed(measured[slot]);
    wire signed [31:0] temporal_error=measured_position-$signed(anchors[slot]);
    // A missed frame expands the innovation gate, independently for each side.
    wire [15:0] temporal_limit=edge_points ? 24+(misses[slot&1]>4 ? 4 : misses[slot&1])*12 : 12;
    wire signed [31:0] temporal_limit_q8=$signed({1'b0,temporal_limit,8'b0});
    // Only one endpoint exists in this branch. Share its difference and
    // bounded band-distance product rather than synthesize four wide multipliers.
    wire endpoint_has=lo>=0 || hi>=0;
    wire [BAND_INDEX_BITS-1:0] endpoint_gap=lo>=0 ? idx_band-lo : hi-idx_band;
    wire [BAND_INDEX_BITS+7:0] endpoint_product=outlier_pixels[7:0]*endpoint_gap;
    wire signed [31:0] endpoint_limit_q8=endpoint_product<<8;
    // CHECK and FILL run at different times; share the residual read port.
    wire [SLOT_BITS-1:0] neighbor_slot=2*(lo>=0 ? lo : hi)+(slot&1);
    wire [SLOT_BITS-1:0] fill_slot=accepted[slot] ? slot : lo>=0 ? 2*lo+(slot&1) : hi>=0 ? 2*hi+(slot&1) : slot;
    wire [SLOT_BITS-1:0] residual_slot=(fsm==FILL && !use_neighbor_expected) ? fill_slot : neighbor_slot;
    wire signed [23:0] residual_read=measured[residual_slot];
    wire signed [31:0] endpoint_delta=$signed(measured[slot])-$signed(residual_read);
    wire endpoint_ok=!edge_points || (lo>=0 && hi>=0) ||
        (endpoint_has && endpoint_delta<=endpoint_limit_q8 && endpoint_delta>=-endpoint_limit_q8);
    wire temporal_ok=!curve_valid[slot&1] || !initialized[slot] ||
        (temporal_error>=-temporal_limit_q8 && temporal_error<=temporal_limit_q8);
    wire band_ok=observed[slot] && (edge_points ? (!stable_road || temporal_ok) : (endpoint_ok && temporal_ok && !(lo>=0 && hi>=0 &&
        ($signed(measured[slot])<neighbor_min-$signed({1'b0,outlier_pixels,8'b0}) ||
         $signed(measured[slot])>neighbor_max+$signed({1'b0,outlier_pixels,8'b0})))));
    // CHECK compares neighboring residuals; FILL interpolates endpoints.
    // These stages are exclusive and can use the same two read selectors.
    wire [SLOT_BITS-1:0] residual_a_slot=2*((fsm==CHECK) ? lo : aa)+idx_side;
    wire [SLOT_BITS-1:0] residual_b_slot=2*((fsm==CHECK) ? hi : bb)+idx_side;
    wire signed [23:0] residual_a=measured[residual_a_slot];
    wire signed [23:0] residual_b=measured[residual_b_slot];
    // Check raw observations simultaneously; gap filling uses accepted points.
    // End gaps retain the nearest residual instead of amplifying a noisy slope.
    always @* begin
        idx_band=slot>>1;idx_side=slot&1;lo=-1;hi=-1;aa=-1;bb=-1;
        for(k=0;k<BANDS;k=k+1) begin
            if(fsm==CHECK ? observed[2*k+idx_side] : accepted[2*k+idx_side]) begin
                if(k<idx_band)lo=k;
                if(k>idx_band && hi<0)hi=k;
            end
        end
        if(lo>=0 && hi>=0)begin aa=lo;bb=hi;end
        else if(lo>=0)begin
            bb=lo;
            for(k=0;k<BANDS;k=k+1)if(k<lo && accepted[2*k+idx_side])aa=k;
        end else if(hi>=0)begin
            aa=hi;
            for(k=BANDS-1;k>=0;k=k-1)if(k>hi && accepted[2*k+idx_side])bb=k;
        end
        interpolation_numerator=0;
        if(aa>=0 && bb>=0)
            interpolation_numerator=($signed(residual_b)-$signed(residual_a))*(idx_band-aa);
        neighbor_min=0;neighbor_max=0;
        if(lo>=0 && hi>=0)begin
            neighbor_min=$signed(residual_a)<$signed(residual_b) ? residual_a : residual_b;
            neighbor_max=$signed(residual_a)>$signed(residual_b) ? residual_a : residual_b;
        end
        // In edge mode, do not pull measured anchors toward synthesized end gaps.
        // Hermite interpolation and temporal filtering provide continuity.
        spatial_position=edge_points ? $signed(fit[slot]) : ($signed(fit[slot])*2+
            $signed(fit[idx_band>0 ? slot-2 : slot])+
            $signed(fit[idx_band<BANDS-1 ? slot+2 : slot]))>>>2;
    end

    // Five-clock Horner pipeline. y changes at DE falling, well before next line.
    localparam SEG_BITS=$clog2(BANDS+1);
    reg [SEG_BITS-1:0] seg,i0,i1,i2,i3;
    reg signed [7:0] t_value;
    reg signed [23:0] p0,p1,p2,p3;
    reg signed [23:0] m1,m2,d;
    reg [15:0] curve_left,curve_right;
    reg [3:0] eval_phase;
    always @* begin
        seg=y>=(1<<(BAND_SHIFT-1)) ? ((y-(1<<(BAND_SHIFT-1)))>>BAND_SHIFT) : 0;
        if(seg>BANDS-2)seg=BANDS-2;
        t_value=($signed({1'b0,y})-(seg<<BAND_SHIFT)-(1<<(BAND_SHIFT-1)))<<(6-BAND_SHIFT);
        if(t_value<0)t_value=0;
        if(t_value>64)t_value=64;
        i0=seg>0 ? seg-1 : 0;i1=seg;i2=seg+1;i3=seg+2<BANDS ? seg+2 : BANDS-1;
        p0=anchors[2*i0+(eval_phase>=4)];p1=anchors[2*i1+(eval_phase>=4)];
        p2=anchors[2*i2+(eval_phase>=4)];p3=anchors[2*i3+(eval_phase>=4)];
        d=p2-p1;
        m1=limited_tangent((p2-p0)>>>1,d);m2=limited_tangent((p3-p1)>>>1,d);
    end
    // Time-multiplex the Horner multiplier over both sides. A complete
    // evaluation takes ten clocks, safely within 720p horizontal blanking.
    reg signed [23:0] eval_h,eval_c0,eval_c1,eval_c2;
    reg signed [7:0] eval_t;
    reg [15:0] eval_y;
    wire signed [31:0] eval_product=eval_h*eval_t;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            eval_phase<=0;eval_h<=0;eval_c0<=0;eval_c1<=0;eval_c2<=0;eval_t<=0;eval_y<=0;
            curve_left<=0;curve_right<=0;
        end else if(y!=eval_y) begin eval_y<=y;eval_phase<=0;end
        else case(eval_phase)
        0:begin eval_h<=-4*d+2*m1+2*m2;eval_c2<=6*d-4*m1-2*m2;
            eval_c1<=2*m1;eval_c0<=2*p1;eval_t<=t_value;eval_phase<=1;end
        1:begin eval_h<=(eval_product>>>6)+eval_c2;eval_phase<=2;end
        2:begin eval_h<=(eval_product>>>6)+eval_c1;eval_phase<=3;end
        3:begin eval_h<=(eval_product>>>6)+eval_c0;eval_phase<=4;end
        4:begin
            curve_left<=eval_h<0 ? 0 : eval_h>((IMAGE_WIDTH-1)<<9) ? IMAGE_WIDTH-1 : eval_h>>>9;
            eval_h<=-4*d+2*m1+2*m2;eval_c2<=6*d-4*m1-2*m2;
            eval_c1<=2*m1;eval_c0<=2*p1;eval_t<=t_value;eval_phase<=5;
        end
        5:begin eval_h<=(eval_product>>>6)+eval_c2;eval_phase<=6;end
        6:begin eval_h<=(eval_product>>>6)+eval_c1;eval_phase<=7;end
        7:begin eval_h<=(eval_product>>>6)+eval_c0;eval_phase<=8;end
        8:begin
            curve_right<=eval_h<0 ? 0 : eval_h>((IMAGE_WIDTH-1)<<9) ? IMAGE_WIDTH-1 : eval_h>>>9;
            eval_phase<=0;
        end
        default:eval_phase<=0;
        endcase
    end

    reg [15:0] run_start,run_length,best_left,best_right,cost_left,cost_right;
    reg found_left,found_right;
    wire in_roi=y>=top_y && y<IMAGE_HEIGHT && x>4 && x<IMAGE_WIDTH-5;
    wire [15:0] reference_left=curve_valid[0] ? curve_left : expected_x(y,0);
    wire [15:0] reference_right=curve_valid[1] ? curve_right : expected_x(y,1);
    wire [15:0] radius=search_radius;
    reg [7:0] row_min,row_max,paint_threshold;
    reg row_seen,paint_ready;
    wire [9:0] row_sum={2'b0,row_min}+{2'b0,row_max}+{1'b0,row_max,1'b0};
    wire [7:0] effective_threshold=paint_threshold>config_i[31:24] ? paint_threshold : config_i[31:24];
    wire bright=original_i[15:8]>=effective_threshold;
    wire in_search=x<road_center ? (x>reference_left ? x-reference_left : reference_left-x)<=radius :
                                                 (x>reference_right ? x-reference_right : reference_right-x)<=radius;
    wire edge_evidence=!use_edges || (edge_i>=edge_threshold && edge_i!=0);
    // Edge-first mode segments Sobel features; the gray floor rejects dark edges.
    // Legacy paint mode still validates an entire bright stripe.
    wire in_rect=!rect_enabled || (x>=rect_left && x<=rect_right && y>=rect_top);
    wire edge_candidate=in_roi && in_rect && edge_i>=edge_threshold && edge_i!=0 && (edge_only || original_i[15:8]>=config_i[31:24]);
    wire candidate=edge_points ? edge_candidate : in_roi && paint_ready && bright;
    wire [15:0] depth=y>horizon ? y-horizon : 0;
    wire [15:0] perspective_width=12+(depth>>2);
    wire [15:0] paint_width_limit=perspective_width<config_i[47:40] ? perspective_width : config_i[47:40];
    reg [7:0] previous_gray,run_background,run_peak;
    reg run_edge,run_clipped;
    wire paint_contrast=({1'b0,run_peak}>={1'b0,run_background}+{1'b0,guide_i[55:48]}) &&
        ({1'b0,run_peak}>={1'b0,original_i[15:8]}+{1'b0,guide_i[55:48]});
    wire legacy_paint_stripe=run_length>=config_i[39:32] && run_length<=config_i[47:40] &&
        run_length<=perspective_width && run_length!=0 && !run_clipped && paint_contrast &&
        (run_edge || edge_evidence);
    wire edge_cluster=run_length!=0 && run_length>=config_i[39:32] &&
        run_length<=config_i[47:40] &&
        run_start>5 && run_start+run_length<IMAGE_WIDTH-5 && (!rect_enabled || (run_start>rect_left && run_start+run_length<=rect_right));
    wire paint_stripe=edge_points ? edge_cluster : legacy_paint_stripe;
    // Previous-row local road contrast adapts paint extraction to gray asphalt.
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n)begin row_min<=255;row_max<=0;row_seen<=0;paint_threshold<=180;paint_ready<=0;end
        else if(changed || frame_start)begin
            row_min<=255;row_max<=0;row_seen<=0;paint_threshold<=config_i[31:24];paint_ready<=0;
        end else begin
            if(de_i && in_roi && in_search)begin
                row_seen<=1;
                if(original_i[15:8]<row_min)row_min<=original_i[15:8];
                if(original_i[15:8]>row_max)row_max<=original_i[15:8];
            end
            if(de_d && !de_i)begin
                paint_ready<=row_seen && {1'b0,row_max}>={1'b0,row_min}+{1'b0,guide_i[55:48]};
                paint_threshold<=row_sum[9:2];row_min<=255;row_max<=0;row_seen<=0;
            end
        end
    end
    reg [15:0] track_left,track_right;
    reg [2:0] streak_left,streak_right;
    wire coherent_left=found_left && streak_left>=3 &&
        (best_left>track_left ? best_left-track_left : track_left-best_left)<=4;
    wire coherent_right=found_right && streak_right>=3 &&
        (best_right>track_right ? best_right-track_right : track_right-best_right)<=4;
    reg [33:0] road_rows[0:IMAGE_HEIGHT-1];
    reg [33:0] road_row;
    reg history_ready;
    // Frame history is never reset in RAM; history_ready masks uninitialized data.
    always @(posedge clk) begin
        if(y<IMAGE_HEIGHT) begin
            road_row<=road_rows[y];
            if(de_d && !de_i)
                road_rows[y]<={coherent_right && enabled && y>=top_y,
                    coherent_left && enabled && y>=top_y,best_right,best_left};
        end
    end
    wire [15:0] run_center=run_start+(run_length>>1);
    wire [15:0] distance_left=run_center>reference_left ? run_center-reference_left : reference_left-run_center;
    wire [15:0] distance_right=run_center>reference_right ? run_center-reference_right : reference_right-run_center;
    wire [15:0] select_left=distance_left;
    wire [15:0] select_right=distance_right;
    wire live_flank=run_length==0 ?
        ({1'b0,original_i[15:8]}>={1'b0,previous_gray}+{1'b0,guide_i[55:48]}) :
        (!run_clipped && run_length<=(paint_width_limit>>1)+2 &&
         {1'b0,run_peak}>={1'b0,run_background}+{1'b0,guide_i[55:48]});
    wire live_paint=edge_points ? edge_candidate : candidate && live_flank;
    wire left_hit=edge_points ? (curve_valid[0] && live_paint && (x>curve_left ? x-curve_left : curve_left-x)<=2) : history_ready && curve_valid[0] && road_row[32] && live_paint &&
        (x>road_row[15:0] ? x-road_row[15:0] : road_row[15:0]-x)<=2 &&
        (road_row[15:0]>curve_left ? road_row[15:0]-curve_left : curve_left-road_row[15:0])<=12;
    wire right_hit=edge_points ? (curve_valid[1] && live_paint && (x>curve_right ? x-curve_right : curve_right-x)<=2) : history_ready && curve_valid[1] && road_row[33] && live_paint &&
        (x>road_row[31:16] ? x-road_row[31:16] : road_row[31:16]-x)<=2 &&
        (road_row[31:16]>curve_right ? road_row[31:16]-curve_right : curve_right-road_row[31:16])<=12;
    /* 100: both sides use identical fixed display rows in stable mode;
       display_first/last remain observation metadata. The existing trapezoid
       mask clips an out-of-range intersection rather than pinning it to a corner. */
    wire fit_left=curve_valid[0] && y>=(stable_road ? top_y : display_first[0]) &&
        y<=(stable_road ? IMAGE_HEIGHT-1 : display_last[0]) && (edge_points || curve_left+2<road_center) &&
        (x>curve_left ? x-curve_left : curve_left-x)<=2;
    wire fit_right=curve_valid[1] && y>=(stable_road ? top_y : display_first[1]) &&
        y<=(stable_road ? IMAGE_HEIGHT-1 : display_last[1]) && (edge_points || curve_right>road_center+2) &&
        (x>curve_right ? x-curve_right : curve_right-x)<=2;
    wire trace_valid,trace_done;
    wire [15:0] trace_query_y;
    reg [$clog2(BANDS+1)-1:0] prior_band;
    reg signed [BAND_SHIFT+1:0] prior_t;
    reg signed [31:0] prior_l,prior_r;
    always @* begin
        prior_band=trace_query_y>=(1<<(BAND_SHIFT-1)) ? (trace_query_y-(1<<(BAND_SHIFT-1)))>>BAND_SHIFT : 0;
        if(prior_band>BANDS-2)prior_band=BANDS-2;
        prior_t=$signed({1'b0,trace_query_y})-(prior_band<<BAND_SHIFT)-(1<<(BAND_SHIFT-1));
        prior_l=$signed(anchors[2*prior_band])+((($signed(anchors[2*prior_band+2])-$signed(anchors[2*prior_band]))*prior_t)>>>BAND_SHIFT);
        prior_r=$signed(anchors[2*prior_band+1])+((($signed(anchors[2*prior_band+3])-$signed(anchors[2*prior_band+1]))*prior_t)>>>BAND_SHIFT);
    end
    wire [15:0] trace_y,trace_left,trace_right;
    wire signed [31:0] trace_lslope,trace_rslope;
    wire signed [31:0] trace_offset=$signed({1'b0,((trace_y>>BAND_SHIFT)<<BAND_SHIFT)})+(1<<(BAND_SHIFT-1))-$signed({1'b0,trace_y});
    // Trace slope is bounded by a difference of two valid x coordinates,
    // scaled by 16; the row offset is within half a band.
    localparam TRACE_SLOPE_BITS=$clog2(IMAGE_WIDTH*16+1)+1;
    localparam TRACE_OFFSET_BITS=BAND_SHIFT+2;
    wire signed [TRACE_SLOPE_BITS+TRACE_OFFSET_BITS-1:0] trace_left_product=
        $signed(trace_lslope[TRACE_SLOPE_BITS-1:0])*$signed(trace_offset[TRACE_OFFSET_BITS-1:0]);
    wire signed [TRACE_SLOPE_BITS+TRACE_OFFSET_BITS-1:0] trace_right_product=
        $signed(trace_rslope[TRACE_SLOPE_BITS-1:0])*$signed(trace_offset[TRACE_OFFSET_BITS-1:0]);
    wire trace_lgood,trace_rgood;
    lane_inner_trace #(.W(IMAGE_WIDTH),.H(IMAGE_HEIGHT)) u_inner(
        .clk(clk),.rst_n(rst_n),.clear_i(changed),
        .cluster_i(edge_points && enabled && de_i && !candidate && in_roi && edge_cluster && fsm==IDLE),
        .cluster_first(run_start),.cluster_last(run_start+run_length-1'b1),.row_y(y),
        .row_end(de_d && !de_i),.start_i(frame_start && edge_points && enabled && y>=IMAGE_HEIGHT),
        .query_y_o(trace_query_y),.prior_left_i(prior_l>>>8),.prior_right_i(prior_r>>>8),
        .prior_valid_i({curve_valid[1],curve_valid[0]}),
        .top_i(top_y),.radius_i(search_radius),.divergence_i(guide_i[39:32]),
        .valid_o(trace_valid),.done_o(trace_done),.y_o(trace_y),.left_o(trace_left),.right_o(trace_right),
        .left_valid_o(trace_lgood),.right_valid_o(trace_rgood),.left_slope_o(trace_lslope),.right_slope_o(trace_rslope));
    wire [23:0] display_base=edge_background ? processed_i : original_i;
    
    // Select rows inside the actual visible ROI, including short trapezoids.
    wire [15:0] analysis_height=IMAGE_HEIGHT>top_y ? IMAGE_HEIGHT-top_y : 0;
    wire [15:0] analysis_inset=(analysis_height>>3)>1 ? analysis_height>>3 : 1;
    wire [15:0] analysis_y=IMAGE_HEIGHT-1-analysis_inset;
    // Two near-row samples reduce sensitivity to a single extrapolated endpoint.
    reg [15:0] near_probe_left,near_probe_right;
    reg near_probe_valid;
    wire [15:0] near_probe_y=analysis_y>top_y+16 ? analysis_y-16 : top_y+1;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n)begin near_probe_left<=0;near_probe_right<=0;near_probe_valid<=0;end
        else if(changed || !enabled || frame_start)near_probe_valid<=0;
        else if(de_i && x==16 && y==near_probe_y)begin
            near_probe_left<=curve_left;near_probe_right<=curve_right;
            near_probe_valid<=curve_valid[0] && curve_valid[1] && curve_right>curve_left;
        end
    end
    wire [15:0] position_left=near_probe_valid ? ({1'b0,curve_left}+{1'b0,near_probe_left})>>1 : curve_left;
    wire [15:0] position_right=near_probe_valid ? ({1'b0,curve_right}+{1'b0,near_probe_right})>>1 : curve_right;
    reg [1:0] lane_position;
    reg lane_position_valid; reg lane_seen;
    reg [4:0] lane_loss_count;
    reg departure_confirmed;
    reg [1:0] departure_count, departure_clear_count;
    wire lane_loss_departure = !lane_position_valid && lane_seen && departure_confirmed && lane_loss_count<15;
    
    reg [3:0] lane_bar_bin;
    
    reg [1:0] lane_pending,lane_pending_count;
    wire [16:0] sample_width={1'b0,position_right}-{1'b0,position_left};
    wire [17:0] reference4={2'b0,road_center}<<2;
    wire [17:0] left_limit={2'b0,position_left}+({2'b0,position_left}<<1)+{2'b0,position_right};
    wire [17:0] right_limit={2'b0,position_right}+({2'b0,position_right}<<1)+{2'b0,position_left};
    wire [18:0] reference8={3'b0,road_center}<<3;
    wire [18:0] left8={3'b0,position_left}<<3;
    wire [18:0] width8={2'b0,sample_width};
    wire [18:0] bar_offset=reference8-left8;
    wire [3:0] sample_bin=road_center<=position_left ? 0 : road_center>=position_right ? 8 :
        bar_offset<width8 ? 0 : bar_offset<(width8<<1) ? 1 :
        bar_offset<((width8<<1)+width8) ? 2 : bar_offset<(width8<<2) ? 3 :
        bar_offset<((width8<<2)+width8) ? 4 : bar_offset<((width8<<2)+(width8<<1)) ? 5 :
        bar_offset<((width8<<3)-width8) ? 6 : 7;
    wire [18:0] left_exit=({3'b0,position_left}<<2)+{3'b0,position_left}+({3'b0,position_right}<<1)+{3'b0,position_right};
    wire [18:0] right_exit=({3'b0,position_right}<<2)+{3'b0,position_right}+({3'b0,position_left}<<1)+{3'b0,position_left};
    wire sample_valid=stable_road && curve_valid[0] && curve_valid[1] && position_right>position_left && sample_width>=32 && road_center<IMAGE_WIDTH;
    // Warn only when the reference point enters the outer eighth of lane width.
    wire [18:0] left_warning=({3'b0,position_left}<<3)+width8;
    wire [18:0] right_warning=({3'b0,position_right}<<3)-width8;
    wire [1:0] raw_position=reference8<left_warning ? 0 : reference8>right_warning ? 2 : 1;
    wire [1:0] next_position_class=(lane_position==0 && sample_bin<2 && raw_position!=2) ? 0 :
        (lane_position==2 && sample_bin>5 && raw_position!=0) ? 2 : raw_position;
    
    reg [15:0] far_lane_center,mid_lane_center;
    reg far_lane_valid,mid_lane_valid,trend_valid;
    reg [1:0] road_trend,trend_pending;
    reg [4:0] trend_pending_count;
    reg [3:0] trend_loss;
    wire [15:0] far_sample_y=top_y+analysis_inset;
    wire [16:0] mid_sample_sum={1'b0,far_sample_y}+analysis_y;
    wire [15:0] mid_sample_y=mid_sample_sum>>1;
    wire [16:0] near_center=({1'b0,curve_left}+{1'b0,curve_right})>>1;
    wire signed [19:0] bend_delta=$signed({4'b0,far_lane_center})+
        $signed({3'b0,near_center})-($signed({4'b0,mid_lane_center})<<<1);
    // Wider hysteresis at entry, narrower at exit reduces left/right chatter.
    wire [16:0] bend_scaled=analysis_height<128 ? (sample_width>>8) : (sample_width>>7);
    wire signed [19:0] bend_margin=$signed({3'b0,(bend_scaled<1 ? 17'd1 : bend_scaled)});
    wire signed [19:0] bend_enter=bend_margin<<<1;
    wire [1:0] trend_raw=(road_trend==0 && bend_delta < -bend_margin) ? 0 :
        (road_trend==2 && bend_delta > bend_margin) ? 2 :
        (bend_delta < -bend_enter) ? 0 : (bend_delta > bend_enter) ? 2 : 1;
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin far_lane_center<=0;mid_lane_center<=0;far_lane_valid<=0;mid_lane_valid<=0;trend_valid<=0;road_trend<=1;trend_pending<=1;trend_pending_count<=0;trend_loss<=0;end
        else if(changed || !enabled)begin far_lane_valid<=0;mid_lane_valid<=0;trend_valid<=0;trend_pending_count<=0;trend_loss<=0;end
        else begin
            if(frame_start)begin far_lane_valid<=0;mid_lane_valid<=0;end
            if(de_i && x==16 && y==far_sample_y && far_sample_y<mid_sample_y)begin
                far_lane_valid<=sample_valid;
                far_lane_center<=near_center;
            end
            if(de_i && x==16 && y==mid_sample_y && mid_sample_y<analysis_y)begin
                mid_lane_valid<=sample_valid;
                mid_lane_center<=near_center;
            end
            if(de_i && x==16 && y==analysis_y)begin
                if(!sample_valid || !far_lane_valid || !mid_lane_valid)begin
                    trend_pending_count<=0;
                    if(trend_loss<8)trend_loss<=trend_loss+1'b1;
                    else trend_valid<=0;
                end else begin
                trend_valid<=1;trend_loss<=0;
                if(!trend_valid)begin road_trend<=trend_raw;trend_pending<=trend_raw;trend_pending_count<=0;end
                else if(trend_raw==road_trend)trend_pending_count<=0;
                else if(trend_raw!=trend_pending)begin trend_pending<=trend_raw;trend_pending_count<=1;end
                else if(trend_pending_count>=((road_trend!=1 && trend_raw!=1) ? 5'd19 : 5'd9))begin road_trend<=trend_raw;trend_pending_count<=0;end
                else trend_pending_count<=trend_pending_count+1'b1;
                end
            end
        end
    end
    
    reg bend_alert;
    reg [1:0] bend_alert_count;
    wire signed [19:0] bend_alert_margin=$signed({3'b0,sample_width>>3});
    wire bend_alert_raw=(bend_delta > bend_alert_margin) || (bend_delta < -bend_alert_margin);
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n)begin bend_alert<=0;bend_alert_count<=0;end
        else if(changed || !enabled)begin bend_alert<=0;bend_alert_count<=0;end
        else if(de_i && x==16 && y==analysis_y)begin
            if(!sample_valid || !far_lane_valid || !mid_lane_valid)begin bend_alert<=0;bend_alert_count<=0;end
            else if(bend_alert_raw==bend_alert)bend_alert_count<=0;
            else if(bend_alert_count>=2)begin bend_alert<=bend_alert_raw;bend_alert_count<=0;end
            else bend_alert_count<=bend_alert_count+1'b1;
        end
    end
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin lane_position<=1;lane_position_valid<=0;lane_seen<=0;lane_pending<=1;lane_pending_count<=0;lane_bar_bin<=4;lane_loss_count<=31;end
        else if(changed || !enabled)begin lane_position_valid<=0;lane_seen<=0;lane_pending_count<=0;lane_loss_count<=31;end
        else if(de_i && y==analysis_y && x==16)begin
            lane_position_valid<=sample_valid;
            if(sample_valid) lane_loss_count<=0;
            else if(lane_loss_count<31) lane_loss_count<=lane_loss_count+1'b1;
            if(sample_valid)begin lane_bar_bin<=sample_bin;lane_seen<=1;end
            if(!sample_valid)lane_pending_count<=0;
            else if(!lane_position_valid)begin lane_position<=raw_position;lane_pending<=raw_position;lane_pending_count<=0;end
            else if(next_position_class==lane_position)lane_pending_count<=0;
            else if(next_position_class!=lane_pending)begin lane_pending<=next_position_class;lane_pending_count<=1;end
            else if(lane_pending_count>=2)begin lane_position<=next_position_class;lane_pending_count<=0;end
            else lane_pending_count<=lane_pending_count+1'b1;
        end
    end
    // Require sustained boundary crossing; mere proximity or a missed dashed
    // line is not enough to assert departure. Hold only a confirmed result.
    always @(posedge clk or negedge rst_n) begin
      if(!rst_n) begin departure_confirmed<=0;departure_count<=0;departure_clear_count<=0;end
      else if(changed || !enabled) begin departure_confirmed<=0;departure_count<=0;departure_clear_count<=0;end
      else if(de_i && y==analysis_y && x==16) begin
        if(sample_valid) begin
          if(road_center<=position_left || road_center>=position_right) begin
            departure_clear_count<=0;
            if(departure_count<3) departure_count<=departure_count+1'b1;
            if(departure_count>=2) departure_confirmed<=1;
          end else begin
            departure_count<=0;
            if(departure_clear_count<3) departure_clear_count<=departure_clear_count+1'b1;
            if(departure_clear_count>=2) departure_confirmed<=0;
          end
        end else if(lane_loss_count>=15) begin departure_confirmed<=0;departure_count<=0;departure_clear_count<=0;end
      end
    end
    // The reference outside the estimated image-space boundary is only a
    // possible departure; it is not proof of crossing or intentional lane change.
    reg [3:0] advice_code;
    wire head_down=(fatigue_sync==7 || fatigue_sync==8);
    wire fatigue_risk=(fatigue_sync==3 || fatigue_sync==8);
    wire fatigue_valid=(fatigue_sync==1 || fatigue_sync==2 || fatigue_sync==3 || fatigue_sync==7 || fatigue_sync==8);
    wire lane_outside=departure_confirmed && (lane_position_valid || lane_loss_departure);
    wire lane_near=lane_position_valid && lane_position!=1;
    always @* begin
      advice_code=0;
      if(!enabled)advice_code=14;
      else if(fatigue_risk && lane_outside)advice_code=10;
      else if(fatigue_risk && lane_near)advice_code=9;
      else if(fatigue_risk && lane_position_valid && trend_valid && road_trend==0)advice_code=11;
      else if(fatigue_risk && lane_position_valid && trend_valid && road_trend==2)advice_code=12;
      else if(fatigue_risk)advice_code=8;
      else if(!lane_position_valid && !lane_loss_departure)advice_code=1;
      else if(lane_outside)advice_code=lane_loss_departure ? (lane_position==0?4:5) : (road_center<=position_left?4:5);
      else if(lane_near)advice_code=lane_position==0?2:3;
      else if(fatigue_sync==2)advice_code=15;
      else if(trend_valid && road_trend==0)advice_code=6;
      else if(trend_valid && road_trend==2)advice_code=7;
      else if(!fatigue_valid)advice_code=13;
    end
    reg [27:0] assist_frame_age;
    always @(posedge clk or negedge rst_n) begin
      if(!rst_n)begin assist_status_o<=16'h000e;assist_frame_age<=0;end
      else if(frame_start)begin
        assist_frame_age<=0;
        assist_status_o<={(fatigue_sync==2),enabled,fatigue_valid,fatigue_risk,head_down,road_trend,trend_valid,lane_position,lane_outside,lane_position_valid,advice_code};
      end else if(assist_frame_age<28'd148500000)assist_frame_age<=assist_frame_age+1'b1;
      else assist_status_o<=16'h000e;
    end
    function [34:0] position_glyph;
        input [7:0] c;
        begin case(c)
            "B":position_glyph=35'b11110100011000111110100011000111110;
            "L":position_glyph=35'b10000100001000010000100001000011111;
            "E":position_glyph=35'b11111100001000011110100001000011111;
            "F":position_glyph=35'b11111100001000011110100001000010000;
            "T":position_glyph=35'b11111001000010000100001000010000100;
            "C":position_glyph=35'b01111100001000010000100001000001111;
            "N":position_glyph=35'b10001110011010110011100011000110001;
            "R":position_glyph=35'b11110100011000111110101001001010001;
            "I":position_glyph=35'b11111001000010000100001000010011111;
            "G":position_glyph=35'b01111100001000010111100011000101111;
            "D":position_glyph=35'b11110100011000110001100011000111110;
            "A":position_glyph=35'b01110100011000111111100011000110001;
            "O":position_glyph=35'b01110100011000110001100011000101110;
            "S":position_glyph=35'b01111100001000001110000010000111110;
            "J":position_glyph=35'b00111000100001000010000101001001100;
            "V":position_glyph=35'b10001100011000110001010100101000100;
            "K":position_glyph=35'b10001100101010011000101001001010001;
            "U":position_glyph=35'b10001100011000110001100011000101110;
            "H":position_glyph=35'b10001100011000111111100011000110001;
            default:position_glyph=0;
        endcase end
    endfunction
    reg [7:0] position_char,trend_char,bend_char;
    wire [1:0] display_position=(lane_bar_bin<=1) ? 0 : (lane_bar_bin>=7) ? 2 : 1;
    wire display_very_close=(lane_bar_bin==0 || lane_bar_bin>=8);
    reg [34:0] position_bits,trend_bits,bend_bits;
    wire [7:0] panel_offset=(x>=24 && x<240) ? x-24 : 0;
    reg [3:0] position_column,position_row,position_index;
    always @* begin
        position_index=panel_offset>>4;position_column=(panel_offset & 8'h0f)>>1;
        position_row=(y>=20 && y<48) ? (y-20)>>2 : 0;
        position_char=" ";
        case(display_position)
          0:case(position_index) 0:position_char="L";1:position_char="E";2:position_char="F";3:position_char="T";5:position_char=display_very_close ? "C" : "N";6:position_char=display_very_close ? "L" : "E";7:position_char=display_very_close ? "O" : "A";8:position_char=display_very_close ? "S" : "R";9:position_char=display_very_close ? "E" : " ";endcase
          1:case(position_index) 0:position_char="C";1:position_char="E";2:position_char="N";3:position_char="T";4:position_char="E";5:position_char="R";endcase
          2:case(position_index) 0:position_char="R";1:position_char="I";2:position_char="G";3:position_char="H";4:position_char="T";6:position_char=display_very_close ? "C" : "N";7:position_char=display_very_close ? "L" : "E";8:position_char=display_very_close ? "O" : "A";9:position_char=display_very_close ? "S" : "R";10:position_char=display_very_close ? "E" : " ";endcase
        endcase
        position_bits=position_glyph(position_char);
        trend_char=" ";
        case(position_index)
          0:trend_char="R";1:trend_char="O";2:trend_char="A";3:trend_char="D";
          5:trend_char=road_trend==0 ? "L" : road_trend==2 ? "R" : "S";
          6:trend_char=road_trend==0 ? "E" : road_trend==2 ? "I" : "T";
          7:trend_char=road_trend==0 ? "F" : road_trend==2 ? "G" : "R";
          8:trend_char=road_trend==0 ? "T" : road_trend==2 ? "H" : "A";
          9:trend_char=road_trend==0 ? " " : road_trend==2 ? "T" : "I";
          10:trend_char=road_trend==1 ? "G" : " ";
          11:trend_char=road_trend==1 ? "H" : " ";
          12:trend_char=road_trend==1 ? "T" : " ";
        endcase
        trend_bits=position_glyph(trend_char);
        /* LANE-alert message aligned with the existing glyph grid. */
        bend_char=" ";
        case(position_index)
          0:bend_char="B";1:bend_char="E";2:bend_char="N";3:bend_char="D";
          5:bend_char="A";6:bend_char="L";7:bend_char="E";8:bend_char="R";9:bend_char="T";
        endcase
        bend_bits=position_glyph(bend_char);
    end
    /* Combined warning requires both a fresh I3 risk and a valid lane edge. */
    wire joint_risk=(fatigue_risk && stable_road && enabled && lane_outside);
    wire joint_border=joint_risk && (x<8 || x>=IMAGE_WIDTH-8 || y<8 || y>=IMAGE_HEIGHT-8);
    reg [7:0] fatigue_char; reg [34:0] fatigue_bits;
    wire [7:0] fatigue_offset=(x>=1000 && x<1168) ? x-1000 : 0;
    reg [3:0] fatigue_index, fatigue_col, fatigue_row;
    always @* begin
      fatigue_index=fatigue_offset/8'd12; fatigue_col=(fatigue_offset%8'd12)/2;
      fatigue_row=(y>=20 && y<34) ? (y-20)/2 : 0;
      fatigue_char=" ";
      if(joint_risk) case(fatigue_index)
        0:fatigue_char="J";1:fatigue_char="O";2:fatigue_char="I";3:fatigue_char="N";4:fatigue_char="T";
        6:fatigue_char="R";7:fatigue_char="I";8:fatigue_char="S";9:fatigue_char="K";
      endcase
      else case(fatigue_index)
        0:fatigue_char="D";1:fatigue_char="R";2:fatigue_char="I";3:fatigue_char="V";4:fatigue_char="E";5:fatigue_char="R";
        7:fatigue_char=(fatigue_sync==1 ? "O" : fatigue_sync==2 ? "C" : fatigue_sync==3 ? "R" : "N");
        8:fatigue_char=(fatigue_sync==1 ? "K" : fatigue_sync==2 ? "L" : fatigue_sync==3 ? "I" : "A");
        9:fatigue_char=(fatigue_sync==2 ? "O" : fatigue_sync==3 ? "S" : " ");
        10:fatigue_char=(fatigue_sync==2 ? "S" : fatigue_sync==3 ? "K" : " ");
        11:fatigue_char=(fatigue_sync==2 ? "E" : " "); 12:fatigue_char=(fatigue_sync==2 ? "D" : " ");
      endcase
      fatigue_bits=position_glyph(fatigue_char);
    end
    wire fatigue_area=IMAGE_WIDTH>=1200 && x>=992 && x<1176 && y>=12 && y<42;
    wire fatigue_text=fatigue_area && x>=1000 && x<1168 && y>=20 && y<34 && fatigue_col<5 && fatigue_row<7 && fatigue_bits[34-(fatigue_row*5+fatigue_col)];
    wire panel_area=IMAGE_WIDTH>=640 && stable_road && enabled && x>=16 && x<288 && y>=12 && y<112 && !rect_i[4];
    // Chinese title replaces the legacy CENTER/LEFT/RIGHT label.
    wire panel_text=1'b0;
    wire bar_track=panel_area && x>=24 && x<=168 && y>=54 && y<=64;
    wire [15:0] bar_marker_x=16'd24+(lane_bar_bin*16'd18);
    wire bar_marker=bar_track && lane_seen && x+2>=bar_marker_x && x<=bar_marker_x+2;
    // Retain the last observed indicator when tracking is lost; the explicit
    // Chinese label makes clear that the position is historical, not current.
    wire [4:0] left_message=lane_outside?5'd22:!lane_position_valid?5'd19:lane_position==0?5'd20:lane_position==2?5'd21:5'd23;
    wire [8:0] left_font_x=x-16'd24;wire [5:0] left_font_y=y-16'd20;
    wire left_area=IMAGE_WIDTH>=640 && stable_road && enabled && !rect_i[4] && x>=24 && x<280 && y>=20 && y<52;
    // Fatigue gets its own line; head, lane risk and road trend can coexist.
    wire turn_notice=lane_position_valid && trend_valid && road_trend!=1;
    wire road_risk_notice=fatigue_risk && (lane_outside || lane_near);
    wire [4:0] turn_message=road_trend==0 ? 5'd17 : 5'd18;
    wire [4:0] right_message=(y>=122 && y<154) ? turn_message :
        (y>=88 && y<120) ? (road_risk_notice ? (lane_outside ? (lane_position==0 ? 5'd4 : 5'd5) : (lane_position==0 ? 5'd2 : 5'd3)) : turn_message) :
        (y>=54 && y<86) ? 5'd16 :
        fatigue_risk ? 5'd8 :
        ((head_down && advice_code==0)?5'd16:((advice_code==4 || advice_code==5) && fatigue_sync==1)?5'd25:((advice_code==4 || advice_code==5) && fatigue_sync==2)?5'd26:{1'b0,advice_code});
    wire right_row=(y>=20 && y<52) || (head_down && advice_code!=0 && y>=54 && y<86) ||
        ((road_risk_notice || (turn_notice && (fatigue_risk || (advice_code!=6 && advice_code!=7)))) && y>=88 && y<120) ||
        (road_risk_notice && turn_notice && y>=122 && y<154);
    wire [5:0] right_y=y>=122?y-16'd122:y>=88?y-16'd88:y>=54?y-16'd54:y-16'd20;
    wire [8:0] right_x=x-16'd912;
    // The two text regions do not overlap: share one font ROM to fit the FPGA.
    wire font_pixel;
    video_assist_font u_assist_font(.clk(clk),.message(left_area?5'd24:right_message),.x(left_area?left_font_x:right_x),.y(left_area?left_font_y:right_y),.pixel(font_pixel));
    wire assist_area=IMAGE_WIDTH>=1200 && !rect_i[6] && x>=900 && x<1280 && y>=12 && y<160;
    reg left_area_q,right_area_q;
    always @(posedge clk)begin left_area_q<=left_area;right_area_q<=assist_area && right_row && x>=912 && x<1264;end
    wire left_text=left_area_q && font_pixel;
    wire assist_text=right_area_q && font_pixel;
    wire [15:0] trend_row=(y-64)>>1;
    wire trend_text=1'b0;
    /* LANE-alert row shares the lane panel and disappears on invalid data. */
    wire [15:0] bend_row=(y-80)>>2;
    wire bend_text=panel_area && trend_valid && bend_alert && x>=24 && x<200 && y>=80 && y<108 && position_column<5 && bend_bits[34-(bend_row*5+position_column)];
    // Explicit camera/vehicle reference; this is not a measured physical centre.
    wire vehicle_center_hit=stable_road && rect_i[3] && in_roi && in_rect &&
        (x==road_center || x+1==road_center);
    wire signed [17:0] arrow_dx=$signed({2'b0,x})-$signed(IMAGE_WIDTH/2);
    wire signed [17:0] arrow_dy=$signed({2'b0,y})-18'sd42;
    wire [17:0] arrow_abs_y=arrow_dy<0 ? -arrow_dy : arrow_dy;
    wire arrow_box=stable_road && enabled && trend_valid && !rect_i[5] &&
        arrow_dx>=-78 && arrow_dx<=78 && y>=10 && y<=76;
    wire arrow_straight=(arrow_dx>=-6 && arrow_dx<=6 && y>=32 && y<=68) ||
        (y>=17 && y<38 && arrow_dx>=-$signed({2'b0,y-16'd17}) && arrow_dx<=$signed({2'b0,y-16'd17}));
    wire arrow_right=(arrow_dx>=-52 && arrow_dx<=19 && arrow_abs_y<=6) ||
        (arrow_dx>19 && arrow_dx<=56 && arrow_abs_y<=((18'sd56-arrow_dx)>>>1));
    wire arrow_left=(arrow_dx>=-19 && arrow_dx<=52 && arrow_abs_y<=6) ||
        (x>=IMAGE_WIDTH/2-56 && x<IMAGE_WIDTH/2-19 && arrow_abs_y<=((x-(IMAGE_WIDTH/2-56))>>1));
    wire arrow_hit=arrow_box && (road_trend==0 ? arrow_left : road_trend==2 ? arrow_right : arrow_straight);
    wire center_hit=stable_road && curve_valid[0] && curve_valid[1] && curve_right>curve_left && in_roi && in_rect &&
        x==(({1'b0,curve_left}+{1'b0,curve_right})>>1);
    wire guide_hit=!edge_points && config_i[6] && in_roi && (x==road_center || x==expected_x(y,0) || x==expected_x(y,1));
    wire rect_hit=edge_points && rect_enabled && rect_i[1] && x>=rect_left && x<=rect_right && y>=rect_top && (x==rect_left || x==rect_right || y==rect_top || y==IMAGE_HEIGHT-1);
    wire roi_hit=config_i[2] && in_roi && (y==top_y || x==5 || x==IMAGE_WIDTH-6);
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin rgb_o<=0;hs_o<=1;vs_o<=VS_ACTIVE;de_o<=0;end
        else begin
            hs_o<=hs_i;vs_o<=vs_i;de_o<=de_i;
            rgb_o<=!de_i ? 0 : !enabled ? processed_i : config_i[1] ? (candidate ? 24'hffffff : 0) :
                joint_border ? 24'hff3030 :
                assist_text ? (fatigue_risk || head_down || lane_near || lane_outside ? 24'hffb020 : 24'h40e0ff) : assist_area ? 24'h102030 :
                left_text ? 24'hffc040 :
                arrow_hit ? 24'h40e0ff : arrow_box ? 24'h102030 :
                bend_text ? 24'hffb020 : panel_text ? (display_position==1 ? 24'h40e0ff : display_very_close ? 24'hff4040 : 24'hffc040) : trend_text ? 24'h40e0ff :
                bar_marker ? ((lane_position!=1 && (lane_bar_bin==0 || lane_bar_bin>=7)) ? 24'hff4040 : 24'hffffff) : bar_track ? ((x<42 || x>150) ? 24'h805020 : 24'h306060) : panel_area ? 24'h102030 :
                !rect_i[2] && in_roi && in_rect && (draw_fit ? fit_left : left_hit) ? (IMAGE_WIDTH>=640 && lane_position_valid && lane_position==0 ? 24'hffc040 : 24'h00ff40) :
                !rect_i[2] && in_roi && in_rect && (draw_fit ? fit_right : right_hit) ? (IMAGE_WIDTH>=640 && lane_position_valid && lane_position==2 ? 24'hffc040 : 24'hff4040) :
                center_hit ? 24'h40c0c0 :
                rect_hit ? 24'h00c0ff : guide_hit ? 24'hffc000 : roi_hit ? 24'h00c0ff : display_base;
        end
    end

    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            cfg_d<=0;guide_d<=0;model_d<=0;rect_d<=0;x<=0;y<=0;de_d<=0;vs_d<=VS_ACTIVE;
            previous_gray<=0;run_background<=0;run_peak<=0;run_edge<=0;run_clipped<=0;
            track_left<=0;track_right<=0;streak_left<=0;streak_right<=0;history_ready<=0;
            fsm<=IDLE;slot<=0;div_start<=0;dividend<=0;divisor<=1;negative<=0;
            interp_base<=0;measurement_position<=0;predicted_variance<=1;innovation<=0;next_position<=0;covariance_product<=0;
            run_start<=0;run_length<=0;best_left<=0;best_right<=0;cost_left<=65535;cost_right<=65535;found_left<=0;found_right<=0;
            for(j=0;j<SLOTS;j=j+1)begin row_first[j]<=0;row_last[j]<=0;sums[j]<=0;counts[j]<=0;measured[j]<=0;anchors[j]<=0;fit[j]<=0;variance[j]<=1;observed[j]<=0;accepted[j]<=0;support[j]<=0;initialized[j]<=0;end
            for(j=0;j<2;j=j+1)begin first_y[j]<=0;last_y[j]<=0;display_first[j]<=0;display_last[j]<=0;first_band[j]<=0;last_band[j]<=0;first_position[j]<=0;last_position[j]<=0;frame_rows[j]<=0;saved_rows[j]<=0;band_count[j]<=0;misses[j]<=0;side_update[j]<=0;curve_valid[j]<=0;end
        end else begin
            previous_gray<=original_i[15:8];
            cfg_d<=config_i;guide_d<=guide_i;model_d<=model_i;rect_d<=rect_i;de_d<=de_i;vs_d<=vs_i;div_start<=0;
            if(changed)begin
                history_ready<=0;streak_left<=0;streak_right<=0;
                fsm<=IDLE;x<=0;y<=0;run_length<=0;found_left<=0;found_right<=0;cost_left<=65535;cost_right<=65535;
                for(j=0;j<SLOTS;j=j+1)begin sums[j]<=0;counts[j]<=0;observed[j]<=0;accepted[j]<=0;initialized[j]<=0;variance[j]<=1;end
                for(j=0;j<2;j=j+1)begin frame_rows[j]<=0;curve_valid[j]<=0;misses[j]<=0;end
            end else begin
                if(frame_start)begin
                    history_ready<=enabled && y>=IMAGE_HEIGHT;streak_left<=0;streak_right<=0;
                    first_position[0]<=0;first_position[1]<=0;last_position[0]<=0;last_position[1]<=0;
                    x<=0;y<=0;run_length<=0;found_left<=0;found_right<=0;cost_left<=65535;cost_right<=65535;
                    if(fsm==IDLE && !div_busy)begin
                        saved_rows[0]<=0;saved_rows[1]<=0;frame_rows[0]<=0;frame_rows[1]<=0;
                        band_count[0]<=0;band_count[1]<=0;slot<=0;fsm<=edge_points && enabled && y>=IMAGE_HEIGHT ? TRACE : MEAN;
                    end
                end else begin
                    if(de_i)x<=x+1'b1;
                    if(candidate && enabled && fsm==IDLE)begin
                        if(run_length==0)begin
                            run_start<=x;run_background<=previous_gray;run_peak<=original_i[15:8];
                            run_edge<=edge_evidence;run_clipped<=x<=5 || previous_gray>=effective_threshold;
                        end else begin
                            if(original_i[15:8]>run_peak)run_peak<=original_i[15:8];
                            if(edge_evidence)run_edge<=1;
                        end
                        if(run_length<IMAGE_WIDTH)run_length<=run_length+1'b1;
                    end else begin
                        run_length<=0;
                        if(de_i && in_roi && paint_stripe)begin
                            if(run_center<road_center && distance_left<=radius && select_left<cost_left)begin
                                best_left<=run_center;found_left<=1;cost_left<=select_left;
                            end
                            if(run_center>=road_center && distance_right<=radius && select_right<cost_right)begin
                                best_right<=run_center;found_right<=1;cost_right<=select_right;
                            end
                        end
                    end
                    if(de_d && !de_i)begin
                        x<=0;y<=y+1'b1;found_left<=0;found_right<=0;cost_left<=65535;cost_right<=65535;
                        if(found_left)begin
                            track_left<=best_left;
                            streak_left<=streak_left==0 || (best_left>track_left ? best_left-track_left : track_left-best_left)>4 ? 1 : streak_left<7 ? streak_left+1'b1 : 7;
                        end else streak_left<=0;
                        if(found_right)begin
                            track_right<=best_right;
                            streak_right<=streak_right==0 || (best_right>track_right ? best_right-track_right : track_right-best_right)>4 ? 1 : streak_right<7 ? streak_right+1'b1 : 7;
                        end else streak_right<=0;
                        if(y<IMAGE_HEIGHT && fsm==IDLE && enabled && !edge_points)begin
                            if(coherent_left)begin
                                if(counts[2*(y>>BAND_SHIFT)]==0)row_first[2*(y>>BAND_SHIFT)]<=y;
                                row_last[2*(y>>BAND_SHIFT)]<=y;
                                sums[2*(y>>BAND_SHIFT)]<=sums[2*(y>>BAND_SHIFT)]+$signed({1'b0,best_left})-$signed({1'b0,expected_x(y,0)});
                                counts[2*(y>>BAND_SHIFT)]<=counts[2*(y>>BAND_SHIFT)]+1'b1;frame_rows[0]<=frame_rows[0]+1'b1;
                            end
                            if(coherent_right)begin
                                if(counts[2*(y>>BAND_SHIFT)+1]==0)row_first[2*(y>>BAND_SHIFT)+1]<=y;
                                row_last[2*(y>>BAND_SHIFT)+1]<=y;
                                sums[2*(y>>BAND_SHIFT)+1]<=sums[2*(y>>BAND_SHIFT)+1]+$signed({1'b0,best_right})-$signed({1'b0,expected_x(y,1)});
                                counts[2*(y>>BAND_SHIFT)+1]<=counts[2*(y>>BAND_SHIFT)+1]+1'b1;frame_rows[1]<=frame_rows[1]+1'b1;
                            end
                        end
                    end
                end
                if(fsm==TRACE && trace_valid)begin
                    if(trace_lgood)begin
                        if(counts[2*(trace_y>>BAND_SHIFT)]==0)row_last[2*(trace_y>>BAND_SHIFT)]<=trace_y;
                        row_first[2*(trace_y>>BAND_SHIFT)]<=trace_y;
                        sums[2*(trace_y>>BAND_SHIFT)]<=sums[2*(trace_y>>BAND_SHIFT)]+($signed({1'b0,trace_left})<<<8)-trace_left_product;
                        counts[2*(trace_y>>BAND_SHIFT)]<=counts[2*(trace_y>>BAND_SHIFT)]+1'b1;
                    end
                    if(trace_rgood)begin
                        if(counts[2*(trace_y>>BAND_SHIFT)+1]==0)row_last[2*(trace_y>>BAND_SHIFT)+1]<=trace_y;
                        row_first[2*(trace_y>>BAND_SHIFT)+1]<=trace_y;
                        sums[2*(trace_y>>BAND_SHIFT)+1]<=sums[2*(trace_y>>BAND_SHIFT)+1]+($signed({1'b0,trace_right})<<<8)-trace_right_product;
                        counts[2*(trace_y>>BAND_SHIFT)+1]<=counts[2*(trace_y>>BAND_SHIFT)+1]+1'b1;
                    end
                end
                case(fsm)
                    TRACE:if(trace_done)begin slot<=0;fsm<=MEAN;end
                    MEAN:begin
                        observed[slot]<=counts[slot]>=4;support[slot]<=counts[slot];
                        if(counts[slot]>=4)begin
                            dividend<=edge_points ? (sums[slot]<0 ? -sums[slot] : sums[slot]) : (sums[slot]<0 ? (-sums[slot])<<8 : sums[slot]<<8);divisor<=counts[slot];
                            negative<=sums[slot]<0;div_start<=1;fsm<=MEAN_WAIT;
                        end else begin
                            measured[slot]<=0;sums[slot]<=0;counts[slot]<=0;
                            if(slot==SLOTS-1)begin slot<=0;fsm<=CHECK;end else slot<=slot+1;
                        end
                    end
                    MEAN_WAIT:if(div_done)begin
                        measured[slot]<=negative ? -$signed(quotient) : $signed(quotient);sums[slot]<=0;counts[slot]<=0;
                        if(slot==SLOTS-1)begin slot<=0;fsm<=CHECK;end else begin slot<=slot+1;fsm<=MEAN;end
                    end
                    CHECK:begin
                        accepted[slot]<=band_ok;
                        if(band_ok)begin
                            if(band_count[slot&1]==0)begin first_y[slot&1]<=row_first[slot];first_position[slot&1]<=measured_position;first_band[slot&1]<=slot>>1;end
                            last_y[slot&1]<=row_last[slot];
                            last_band[slot&1]<=slot>>1;
                            last_position[slot&1]<=measured_position;
                            band_count[slot&1]<=band_count[slot&1]+1'b1;
                            saved_rows[slot&1]<=saved_rows[slot&1]+support[slot];
                        end
                        if(slot==SLOTS-1)fsm<=QUALIFY;else slot<=slot+1;
                    end
                    QUALIFY:begin
                        side_update[0]<=band_count[0]>=(edge_points && curve_valid[0] ? 2 : min_bands) && saved_rows[0]>=config_i[55:48] && (edge_points ? (first_position[0]-last_position[0]>=512 || last_position[0]-first_position[0]>=512) : first_position[0]-last_position[0]>=min_span_left);
                        side_update[1]<=band_count[1]>=(edge_points && curve_valid[1] ? 2 : min_bands) && saved_rows[1]>=config_i[55:48] && (edge_points ? (last_position[1]-first_position[1]>=512 || first_position[1]-last_position[1]>=512) : last_position[1]-first_position[1]>=min_span_right);slot<=0;fsm<=FILL;
                    end
                    FILL:begin
                        if(!side_update[slot&1])begin
                            if(slot==SLOTS-1)begin slot<=0;fsm<=stable_road ? STABLE_CHECK : KALMAN;end else slot<=slot+1;
                        end else if(stable_road && curve_valid[slot&1] && initialized[slot] && !accepted[slot] && (lo<0 || hi<0))begin
                            
                            fit[slot]<=clamp_q8($signed(anchors[slot]) +
                                (($signed({1'b0,shared_expected})<<<8)
                                + $signed(residual_read)
                                - $signed(anchors[2*(lo>=0 ? lo : hi)+(slot&1)])));
                            if(slot==SLOTS-1)begin slot<=0;fsm<=STABLE_CHECK;end else slot<=slot+1;
                        end else if(accepted[slot] || (!edge_points && (lo<0 || hi<0)))begin
                            fit[slot]<=clamp_q8(($signed({1'b0,shared_expected})<<8)+
                                $signed(residual_read));
                            if(slot==SLOTS-1)begin slot<=0;fsm<=stable_road ? STABLE_CHECK : KALMAN;end else slot<=slot+1;
                        end else if(aa>=0 && bb>=0 && bb>aa)begin
                            dividend<=interpolation_numerator<0 ? -interpolation_numerator : interpolation_numerator;
                            divisor<=bb-aa;negative<=interpolation_numerator<0;interp_base<=residual_a;div_start<=1;fsm<=FILL_WAIT;
                        end else begin
                            fit[slot]<=shared_expected<<8;
                            if(slot==SLOTS-1)begin slot<=0;fsm<=stable_road ? STABLE_CHECK : KALMAN;end else slot<=slot+1;
                        end
                    end
                    FILL_WAIT:if(div_done)begin
                        fit[slot]<=clamp_q8(($signed({1'b0,shared_expected})<<8)+interp_base+(negative ? -$signed(quotient) : $signed(quotient)));
                        if(slot==SLOTS-1)begin slot<=0;fsm<=stable_road ? STABLE_CHECK : KALMAN;end else begin slot<=slot+1;fsm<=FILL;end
                    end
                    STABLE_CHECK:begin
                        
                        /* Ignore hidden bands above the displayed ROI when
                           deciding whether the visible model is plausible. */
                        if((slot>>1)>0 && (slot>>1)<BANDS-1 &&
                           (((slot>>1)<<BAND_SHIFT)+(1<<(BAND_SHIFT-1))+(1<<BAND_SHIFT)>=top_y) &&
                           side_update[slot&1] &&
                           (bend_error>BEND_LIMIT || bend_error< -BEND_LIMIT))begin
                            fit[slot]<=clamp_q8((bend_sum+(bend_error>0 ? -BEND_LIMIT : BEND_LIMIT))>>>1);
                        end else if((((slot>>1)<<BAND_SHIFT)+(1<<(BAND_SHIFT-1))+(1<<BAND_SHIFT)>=top_y) &&
                           side_update[slot&1] && curve_valid[slot&1] && initialized[slot] &&
                           (candidate_delta>temporal_limit_q8 || candidate_delta<-temporal_limit_q8))side_update[slot&1]<=0;
                        if(slot==SLOTS-1)begin slot<=0;fsm<=PAIR_CHECK;end else slot<=slot+1;
                    end
                    PAIR_CHECK:begin
                        if(((slot>>1)<<BAND_SHIFT)+(1<<(BAND_SHIFT-1))>=top_y &&
                           (side_update[0] || curve_valid[0]) && (side_update[1] || curve_valid[1]) &&
                           (pair_width<(4<<8) || (curve_valid[0] && curve_valid[1] &&
                             (pair_width>old_width+width_margin || pair_width<old_width-width_margin))))begin
                            /* Reject the less consistent side; keep independent
                               valid observations on the other side. On acquisition
                               an impossible pair rejects both, then reacquires. */
                            if(!curve_valid[0] || !curve_valid[1])begin side_update[0]<=0;side_update[1]<=0;end
                            else if(left_error>right_error)side_update[0]<=0;
                            else if(right_error>left_error)side_update[1]<=0;
                            else begin side_update[0]<=0;side_update[1]<=0;end
                        end
                        if(slot>=SLOTS-2)begin slot<=0;fsm<=KALMAN;end else slot<=slot+2;
                    end
                    KALMAN:begin
                        if(!side_update[slot&1])begin
                            variance[slot]<=variance[slot]>60000 ? 60000 : variance[slot]+q_noise;
                            if(slot==SLOTS-1)fsm<=FINISH;else slot<=slot+1;
                        end else if(!initialized[slot] || !use_kalman)begin
                            anchors[slot]<=stable_road && initialized[slot] ? clamp_q8($signed(anchors[slot])+bounded_step(spatial_position-$signed(anchors[slot]))) : spatial_position;variance[slot]<=r_noise;initialized[slot]<=1;
                            if(slot==SLOTS-1)fsm<=FINISH;else slot<=slot+1;
                        end else begin
                            predicted_variance<=variance[slot]>60000 ? 60000 : variance[slot]+q_noise;
                            dividend<=(variance[slot]>60000 ? 60000 : variance[slot]+q_noise)<<8;
                            divisor<=(variance[slot]>60000 ? 60000 : variance[slot]+q_noise)+r_noise;
                            innovation<=spatial_position-$signed(anchors[slot]);div_start<=1;fsm<=KALMAN_WAIT;
                        end
                    end
                    KALMAN_WAIT:if(div_done)begin
                        anchors[slot]<=clamp_q8($signed(anchors[slot])+(stable_road ? bounded_step(($signed(innovation)*$signed({1'b0,quotient[8:0]}))>>>8) : (($signed(innovation)*$signed({1'b0,quotient[8:0]}))>>>8)));
                        covariance_product=predicted_variance*quotient[8:0];
                        variance[slot]<=predicted_variance>(covariance_product>>8) ? predicted_variance-(covariance_product>>8) : 1;
                        if(slot==SLOTS-1)fsm<=FINISH;else begin slot<=slot+1;fsm<=KALMAN;end
                    end
                    FINISH:begin
                        for(j=0;j<2;j=j+1)begin
                            if(side_update[j])begin curve_valid[j]<=1;misses[j]<=0;display_first[j]<=first_y[j];display_last[j]<=last_y[j];end
                            else if(curve_valid[j] && misses[j]<hold_frames)misses[j]<=misses[j]+1'b1;
                            else begin
                                curve_valid[j]<=0;
                                for(band=0;band<BANDS;band=band+1)initialized[2*band+j]<=0;
                            end
                        end
                        fsm<=IDLE;
                    end
                endcase
            end
        end
    end
endmodule

// Sparse row feature RAM, read in reverse during vertical blanking. Each cell_index
// keeps both extrema; left tracking uses its right edge, right uses its left.
module lane_inner_trace #(parameter W=1280,H=720)(
    input wire clk,rst_n,clear_i,cluster_i,row_end,start_i,
    input wire [15:0] cluster_first,cluster_last,row_y,top_i,
    input wire [7:0] radius_i,divergence_i,
    input wire signed [31:0] prior_left_i,prior_right_i,
    input wire [1:0] prior_valid_i,
    output wire [15:0] query_y_o,
    output reg valid_o,done_o,left_valid_o,right_valid_o,
    output reg signed [31:0] left_slope_o,right_slope_o,
    output reg [15:0] y_o,left_o,right_o
);
    localparam SHIFT=W>256 ? 6 : 4;
    localparam CELLS=(W+(1<<SHIFT)-1)>>SHIFT;
    // The caller only emits clusters with length <= 255 pixels. Store the
    // first coordinate relative to its cell and the farthest last coordinate
    // relative to the same cell. This is lossless under that interface contract.
    localparam LAST_BITS=$clog2((1<<SHIFT)+256);
    localparam CELL_BITS=1+SHIFT+LAST_BITS;
    localparam BITS=CELLS*CELL_BITS;
    localparam MAX_GAP=H/3;
    reg [BITS-1:0] rows[0:H-1];
    wire [BITS-1:0] collecting;
    reg [BITS-1:0] row_data;
    reg [15:0] read_y;
    reg [2:0] state;
    integer cell_index,ci,j;
    reg [15:0] best[0:1],cost[0:1],previous[0:1],gaps[0:1];
    reg [4:0] streak[0:1];
    reg [255:0] recent[0:1];
    reg selected_ok[0:1];
    reg seeded[0:1],dead[0:1];
    reg signed [31:0] prediction[0:1],slope[0:1];
    reg [15:0] cfirst,clast;
    reg [15:0] cx[0:1];
    reg [31:0] distance[0:1];
    reg eligible[0:1];
    assign query_y_o=read_y;
    integer refx,limit,s,prior_x;
    always @* begin
        ci=cluster_first>>SHIFT;
        if(row_data[cell_index*CELL_BITS+CELL_BITS-1]) begin
            cfirst=(cell_index<<SHIFT)+row_data[cell_index*CELL_BITS+:SHIFT];
            clast=(cell_index<<SHIFT)+row_data[cell_index*CELL_BITS+SHIFT+:LAST_BITS];
        end else begin cfirst=16'hffff;clast=16'hffff;end
        cx[0]=clast;cx[1]=cfirst;
        limit=radius_i<(W/32>8 ? W/32 : 8) ? radius_i : (W/32>8 ? W/32 : 8);
        for(s=0;s<2;s=s+1)begin
            prior_x=s==0 ? prior_left_i : prior_right_i;
            refx=seeded[s] ? prediction[s]>>>8 : prior_valid_i[s] ? prior_x : W/2;
            distance[s]=cx[s]>refx ? cx[s]-refx : refx-cx[s];
            eligible[s]=cfirst!=16'hffff && !dead[s] && cx[s]>4 && cx[s]<W-5 &&
                (seeded[s] ? distance[s]<=((limit+(gaps[s]>>2))<radius_i ? limit+(gaps[s]>>2) : radius_i) : read_y>=H/2 &&
                    (s==0 ? cx[s]<W/2-W/8 : cx[s]>W/2+W/8));
            // Preserve lane identity across dashed gaps and frame reacquisition.
            // An established curve may cross the image centre on a bend.
            if(!seeded[s] && prior_valid_i[s])
                eligible[s]=cfirst!=16'hffff && !dead[s] && cx[s]>4 && cx[s]<W-5 && distance[s]<=radius_i;
            if(prior_valid_i[s] && (cx[s]>prior_x ? cx[s]-prior_x : prior_x-cx[s])>radius_i)eligible[s]=0;
            selected_ok[s]=cost[s]!=65535 && !(seeded[s] && gaps[s]==0 && (best[s]>previous[s] ? best[s]-previous[s] : previous[s]-best[s])>4);
            if(seeded[0] && seeded[1])begin
                if(s==0 && cx[s]+4>=(prediction[1]>>>8))eligible[s]=0;
                if(s==1 && cx[s]<=(prediction[0]>>>8)+4)eligible[s]=0;
            end
        end
    end
    // Constant cell slices avoid variable-width write barrel shifters.
    genvar gc;
    generate for(gc=0;gc<CELLS;gc=gc+1) begin:g_collect
        reg valid_q;
        reg [SHIFT-1:0] first_q;
        reg [LAST_BITS-1:0] last_q;
        assign collecting[gc*CELL_BITS+:CELL_BITS]={valid_q,last_q,first_q};
        always @(posedge clk or negedge rst_n) begin
            if(!rst_n) begin valid_q<=0;first_q<=0;last_q<=0;end
            else if(clear_i || row_end) begin valid_q<=0;first_q<=0;last_q<=0;end
            else if(cluster_i && ci==gc) begin
                if(!valid_q || cluster_first[SHIFT-1:0]<first_q) first_q<=cluster_first[SHIFT-1:0];
                if(!valid_q || cluster_last>((gc<<SHIFT)+last_q)) last_q<=cluster_last-(gc<<SHIFT);
                valid_q<=1;
            end
        end
    end endgenerate
    // No reset on frame RAM: start is allowed only after a complete frame.
    always @(posedge clk)begin
        if(row_end && row_y<H)rows[row_y]<=collecting;
        row_data<=rows[read_y];
    end
    always @(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            state<=0;read_y<=0;cell_index<=0;
            left_slope_o<=0;right_slope_o<=0;valid_o<=0;done_o<=0;left_valid_o<=0;right_valid_o<=0;y_o<=0;left_o<=0;right_o<=0;
            for(j=0;j<2;j=j+1)begin recent[j]<=0;seeded[j]<=0;dead[j]<=0;prediction[j]<=0;slope[j]<=0;streak[j]<=0;gaps[j]<=0;previous[j]<=0;cost[j]<=65535;best[j]<=0;end
        end else begin
            valid_o<=0;done_o<=0;
            if(clear_i)begin state<=0;end
            else begin
                if(start_i)begin
                    
                    read_y<=H-1;state<=1;
                    for(j=0;j<2;j=j+1)begin
                        seeded[j]<=0;dead[j]<=0;streak[j]<=0;gaps[j]<=0;
                        prediction[j]<=(W/2)<<8;slope[j]<=0;
                    end
                end else case(state)
                    1:begin state<=2;cell_index<=0;cost[0]<=65535;cost[1]<=65535;end
                    2:begin
                        for(j=0;j<2;j=j+1)if(eligible[j] && distance[j]<cost[j])begin best[j]<=cx[j];cost[j]<=distance[j];end
                        if(cell_index==CELLS-1)state<=3;else cell_index<=cell_index+1;
                    end
                    3:begin
                        valid_o<=1;y_o<=read_y;left_o<=best[0];right_o<=best[1];left_slope_o<=slope[0];right_slope_o<=slope[1];
                        left_valid_o<=selected_ok[0] && (seeded[0] || streak[0]>=7) && streak[0]>=3 && (best[0]>previous[0] ? best[0]-previous[0] : previous[0]-best[0])<=4;
                        right_valid_o<=selected_ok[1] && (seeded[1] || streak[1]>=7) && streak[1]>=3 && (best[1]>previous[1] ? best[1]-previous[1] : previous[1]-best[1])<=4;
                        for(j=0;j<2;j=j+1)begin
                            if(selected_ok[j])begin
                                previous[j]<=best[j];recent[j]<={recent[j][239:0],best[j]};
                                if(!seeded[j] || (streak[j]>=3 && (best[j]>previous[j] ? best[j]-previous[j] : previous[j]-best[j])<=4))begin
                                    prediction[j]<=($signed({1'b0,best[j]})<<<8)+slope[j];gaps[j]<=0;
                                end else begin
                                    prediction[j]<=prediction[j]+slope[j];
                                    if(gaps[j]>=MAX_GAP)dead[j]<=1;else gaps[j]<=gaps[j]+1'b1;
                                end
                                if(streak[j]!=0 && (best[j]>previous[j] ? best[j]-previous[j] : previous[j]-best[j])<=4)begin
                                    streak[j]<=streak[j]<31 ? streak[j]+1'b1 : 31;
                                    if(streak[j]>=7)seeded[j]<=1;
                                    if(gaps[j]==0 && (!seeded[j] || streak[j]>=3))begin
                                        if(streak[j]>=16)slope[j]<=($signed({1'b0,best[j]})-$signed({1'b0,recent[j][255:240]}))<<<4;
                                        else slope[j]<=($signed(slope[j])*3+(($signed({1'b0,best[j]})-$signed({1'b0,previous[j]}))<<<8))>>>2;
                                    end
                                end else streak[j]<=1;
                            end else begin
                                streak[j]<=0;
                                if(seeded[j])begin
                                    prediction[j]<=prediction[j]+slope[j];
                                    if(gaps[j]>=MAX_GAP)dead[j]<=1;else gaps[j]<=gaps[j]+1'b1;
                                end
                            end
                        end
                        if(read_y<=top_i || read_y==0)state<=4;else begin read_y<=read_y-1'b1;state<=1;end
                    end
                    4:begin done_o<=1;state<=0;end
                endcase
            end
        end
    end
endmodule
