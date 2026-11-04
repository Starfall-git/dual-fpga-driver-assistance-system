`timescale 1ns/1ps

module video_shape_recognizer #(
    parameter IMAGE_WIDTH=1280, IMAGE_HEIGHT=720, SCALE_LOG2=3,
    parameter MAX_OBJECTS=8, VS_ACTIVE=0
)(
    input wire clk,rst_n,enable_i,edge_i,
    input wire [23:0] rgb_i,
    input wire hs_i,vs_i,de_i,
    output reg [23:0] rgb_o,
    output reg hs_o,vs_o,de_o
);
    localparam W=IMAGE_WIDTH>>SCALE_LOG2,H=IMAGE_HEIGHT>>SCALE_LOG2,N=W*H;
    localparam AB=$clog2(N),XB=$clog2(W),YB=$clog2(H),QB=XB+YB;
    localparam S_IDLE=0,S_SCAN=1,S_WAIT=2,S_CHECK=3,S_CLEAR=4,
        S_POP=5,S_POP_WAIT=6,S_POINT=7,S_NEIGHBOR=8,S_NWAIT=9,S_NCHECK=10,
        S_ROWS=11,S_CLASSIFY=12,S_FINISH=13,S_ROW_WIDTH=14,
        S_ROW_PRODUCT=15,S_ROW_ACC=16,S_METRIC1=17,S_METRIC2=18,
        S_SAVE_WAIT=19,S_SAVE_WRITE=20,S_TRACK_START=21,S_TRACK_SCAN=22,S_TRACK_DONE=23, S_HOLD_SCAN=24,S_HOLD_READ=25,S_HOLD_WAIT=26,S_HOLD_WRITE=27;
    reg [4:0] state;
    reg vs_d,de_d,enabled,bank,proc_bank,captured,pending;
    reg [11:0] x,y;
    wire frame_start=(vs_i==VS_ACTIVE)&&(vs_d!=VS_ACTIVE);
    wire [XB-1:0] bx=x>>SCALE_LOG2;
    wire [YB-1:0] by=y>>SCALE_LOG2;
    reg [W-1:0] pool;
    reg hpool;
    wire cell_end=(x & ((1<<SCALE_LOG2)-1))==((1<<SCALE_LOG2)-1);
    wire row_end=(y & ((1<<SCALE_LOG2)-1))==((1<<SCALE_LOG2)-1);
    wire cell_bit=pool[bx]|hpool|edge_i;
    wire capture_we=de_i && enabled && cell_end && row_end && x<IMAGE_WIDTH && y<IMAGE_HEIGHT;
    wire [AB-1:0] capture_addr=by*W+bx;
    reg bitmap0[0:N-1],bitmap1[0:N-1];
    reg [AB-1:0] read_addr;
    reg bit0,bit1;
    wire read_bit=proc_bank ? bit1 : bit0;
    
    wire visit=!frame_start && (state==S_CHECK || state==S_NCHECK) && read_bit;
    /* Two physical RAMs: one port captures a complete frame, the other scans
       the opposite bank. No RAM reset loops; captured gates first use. */
    
    wire cap0=capture_we && !bank,cap1=capture_we && bank;
    wire we0=cap0 || (visit && !proc_bank),we1=cap1 || (visit && proc_bank);
    wire [AB-1:0] wa0=cap0 ? capture_addr : read_addr;
    wire [AB-1:0] wa1=cap1 ? capture_addr : read_addr;
    always @(posedge clk) begin
        bit0<=bitmap0[read_addr];bit1<=bitmap1[read_addr];
        if(we0) bitmap0[wa0]<=cap0 ? cell_bit : 1'b0;
        if(we1) bitmap1[wa1]<=cap1 ? cell_bit : 1'b0;
    end
    reg [QB-1:0] queue[0:N-1];
    reg [AB:0] head,tail;
    reg [QB-1:0] queue_read;
    reg [XB-1:0] sx,px;
    reg [YB-1:0] sy,py;
    reg [3:0] neighbor;
    reg signed [XB+1:0] nx;
    reg signed [YB+1:0] ny;
    always @* begin
        nx=$signed({1'b0,px});ny=$signed({1'b0,py});
        case(neighbor)
          0:begin nx=nx-1;ny=ny-1;end
          1:ny=ny-1;
          2:begin nx=nx+1;ny=ny-1;end
          3:nx=nx-1;
          4:nx=nx+1;
          5:begin nx=nx-1;ny=ny+1;end
          6:ny=ny+1;
          7:begin nx=nx+1;ny=ny+1;end
        endcase
    end
    wire neighbor_valid=nx>=0 && nx<W && ny>=0 && ny<H;
    wire queue_seed=!frame_start && state==S_CHECK && read_bit;
    wire queue_push=queue_seed || (!frame_start && state==S_NCHECK && read_bit && tail<N);
    wire [AB-1:0] queue_wa=queue_seed ? {AB{1'b0}} : tail[AB-1:0];
    wire [QB-1:0] queue_wd=queue_seed ? {sy,sx} : {ny[YB-1:0],nx[XB-1:0]};
    always @(posedge clk) begin
        queue_read<=queue[head[AB-1:0]];
        if(queue_push) queue[queue_wa]<=queue_wd;
    end
    reg [XB-1:0] row_min[0:H-1],row_max[0:H-1];
    reg [H-1:0] row_valid;
    reg [XB-1:0] xmin,xmax;
    reg [YB-1:0] ymin,ymax;
    reg [YB:0] row;
    
    reg [AB:0] area;
    reg [AB+XB+1:0] moment_x2;
    reg [AB+YB+1:0] moment_y2;
    wire rows_phase=state==S_ROWS || state==S_ROW_WIDTH || state==S_ROW_PRODUCT || state==S_ROW_ACC || state==S_SAVE_WAIT || state==S_SAVE_WRITE;
    wire [YB-1:0] row_address=rows_phase ? row[YB-1:0] : queue_read[QB-1:XB];
    reg [XB-1:0] row_left_q,row_right_q;
    always @(posedge clk) begin
        row_left_q<=row_min[row_address];row_right_q<=row_max[row_address];
        if(state==S_POINT && !frame_start) begin
            row_min[queue_read[QB-1:XB]]<=!row_valid[queue_read[QB-1:XB]] || queue_read[XB-1:0]<row_left_q ? queue_read[XB-1:0] : row_left_q;
            row_max[queue_read[QB-1:XB]]<=!row_valid[queue_read[QB-1:XB]] || queue_read[XB-1:0]>row_right_q ? queue_read[XB-1:0] : row_right_q;
        end
    end
    reg [XB:0] row_width,row_sum;
    reg row_present;
    reg [2*XB+1:0] row_mx;
    reg [XB+YB+1:0] row_my;
    wire [XB:0] width={1'b0,xmax}-{1'b0,xmin}+1'b1;
    wire [YB:0] height={1'b0,ymax}-{1'b0,ymin}+1'b1;
    reg [AB:0] box_area;
    reg [AB+XB+1:0] center_x2,dx,area_width;
    reg [AB+YB+1:0] center_y2,dy,area_height;
    reg [2:0] kind;
    
    reg [XB:0] width_top,width_q1,width_mid,width_q3,width_bottom;
    reg [XB:0] center_top,center_bottom;
    wire [XB:0] center_shift = center_top>center_bottom ? center_top-center_bottom : center_bottom-center_top;
    wire plausible=width>=6 && height>=6 && area>=36 && tail>=width+height &&
        tail<=4*(width+height) && xmin>0 && ymin>0 && xmax<W-1 && ymax<H-1;
    always @* begin
        kind=0;
        if(plausible) begin
            
            if((area*100>=box_area*85 && width_q1*100>=width_mid*92 && width_q3*100>=width_mid*92 &&
               width_top*100>=width_mid*80 && width_bottom*100>=width_mid*80) ||
               (area*100>=box_area*60 && center_shift>=3 &&
                dx*8<area_width && dy*8<area_height))
                kind=2;
            else if(area*100>=box_area*65 && area*100<=box_area*88 &&
                    width*5>=height*3 && height*5>=width*3 &&
                    width_top*10<width_mid*9 && width_bottom*10<width_mid*9 &&
                    width_q1*10>=width_mid*7 && width_q3*10>=width_mid*7 &&
                    dx*10<area_width && dy*10<area_height) kind=3;
            else if(area*100>=box_area*38 && area*100<=box_area*67) begin
                if(dx*8>area_width || dy*8>area_height) kind=1;
                else if(dx*12<area_width && dy*12<area_height &&
                    width_q1*10<width_mid*8 && width_q3*10<width_mid*8) kind=4;
            end
        end
    end
    reg [3:0] work_count,display_count;
    reg [XB-1:0] work_x0[0:MAX_OBJECTS-1],work_x1[0:MAX_OBJECTS-1];
    reg [YB-1:0] work_y0[0:MAX_OBJECTS-1],work_y1[0:MAX_OBJECTS-1];
    reg [2:0] work_kind[0:MAX_OBJECTS-1];
    reg [XB-1:0] display_x0[0:MAX_OBJECTS-1],display_x1[0:MAX_OBJECTS-1];
    reg [YB-1:0] display_y0[0:MAX_OBJECTS-1],display_y1[0:MAX_OBJECTS-1];
    reg [2:0] display_kind[0:MAX_OBJECTS-1];
    
    localparam PROFILE_DEPTH=2*MAX_OBJECTS*H, PA=$clog2(PROFILE_DEPTH);
    reg [2*XB-1:0] profiles[0:PROFILE_DEPTH-1];
    reg work_bank,display_bank;
    reg [2*XB-1:0] profile_q;
    reg [PA-1:0] profile_addr;
    reg [2*XB-1:0] profile_cache[0:MAX_OBJECTS-1];
    
    reg [2*XB-1:0] profile_above[0:MAX_OBJECTS-1],profile_below[0:MAX_OBJECTS-1];
    reg [6:0] prefetch;
    reg [MAX_OBJECTS-1:0] matched;
    reg [3:0] best_index,hold_index;
    reg [1:0] work_misses[0:MAX_OBJECTS-1],display_misses[0:MAX_OBJECTS-1];
    wire holding=state==S_HOLD_READ || state==S_HOLD_WAIT || state==S_HOLD_WRITE;
    wire [PA-1:0] hold_addr=(display_bank ? MAX_OBJECTS*H : 0)+hold_index*H+row;
    wire hold_we=state==S_HOLD_WRITE && de_i && !frame_start;
    reg [YB-1:0] cache_y;
    reg cache_bank;
    wire [YB-1:0] next_profile_y=(vs_i==VS_ACTIVE) ? 0 : (y>>SCALE_LOG2);
    wire profile_we=!frame_start && (state==S_SAVE_WRITE || hold_we) && row<H;
    wire [PA-1:0] profile_wa=(work_bank ? MAX_OBJECTS*H : 0)+work_count*H+row;
    always @(posedge clk) begin
        profile_q<=profiles[holding && de_i ? hold_addr : profile_addr];
        if(profile_we) profiles[profile_wa]<=hold_we ? profile_q : row_valid[row] ? {row_left_q,row_right_q} : {{XB{1'b1}},{XB{1'b0}}};
    end
    integer ci;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            prefetch<=0;cache_y<=0;cache_bank<=0;profile_addr<=0;
            for(ci=0;ci<MAX_OBJECTS;ci=ci+1)begin
                profile_cache[ci]<={{XB{1'b1}},{XB{1'b0}}};
                profile_above[ci]<={{XB{1'b1}},{XB{1'b0}}};
                profile_below[ci]<={{XB{1'b1}},{XB{1'b0}}};
            end
        end else if(!de_i) begin
            if(de_d || cache_y!=next_profile_y || cache_bank!=display_bank) begin
                prefetch<=0;cache_y<=next_profile_y;cache_bank<=display_bank;
            end else if(prefetch<MAX_OBJECTS*9) begin
                case(prefetch%3)
                    0: profile_addr<=(display_bank ? MAX_OBJECTS*H : 0)+(prefetch/9)*H+
                        ((prefetch%9)==0 ? cache_y : (prefetch%9)==3 ? (cache_y==0 ? 0 : cache_y-1) : (cache_y==H-1 ? cache_y : cache_y+1));
                    2: case(prefetch%9)
                        2:profile_cache[prefetch/9]<=profile_q;
                        5:profile_above[prefetch/9]<=cache_y==0 ? {{XB{1'b1}},{XB{1'b0}}} : profile_q;
                        8:profile_below[prefetch/9]<=cache_y==H-1 ? {{XB{1'b1}},{XB{1'b0}}} : profile_q;
                    endcase
                endcase
                prefetch<=prefetch+1'b1;
            end
        end
    end
    
    reg [2:0] work_stable[0:MAX_OBJECTS-1],display_candidate[0:MAX_OBJECTS-1];
    reg [1:0] work_hits[0:MAX_OBJECTS-1],display_hits[0:MAX_OBJECTS-1];
    reg [3:0] track_i,track_j;
    reg [2:0] best_kind,best_candidate;
    reg [1:0] best_hits;
    reg [11:0] best_distance;
    reg container;
    wire [11:0] distance_x0=work_x0[track_i]>display_x0[track_j] ? work_x0[track_i]-display_x0[track_j] : display_x0[track_j]-work_x0[track_i];
    wire [11:0] distance_x1=work_x1[track_i]>display_x1[track_j] ? work_x1[track_i]-display_x1[track_j] : display_x1[track_j]-work_x1[track_i];
    wire [11:0] distance_y0=work_y0[track_i]>display_y0[track_j] ? work_y0[track_i]-display_y0[track_j] : display_y0[track_j]-work_y0[track_i];
    wire [11:0] distance_y1=work_y1[track_i]>display_y1[track_j] ? work_y1[track_i]-display_y1[track_j] : display_y1[track_j]-work_y1[track_i];
    wire [11:0] track_distance=distance_x0+distance_x1+distance_y0+distance_y1;
    wire [1:0] next_hits=best_candidate==work_kind[track_i] ? (best_hits==3 ? 2'd3 : best_hits+1'b1) : 2'd1;
    integer k;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            work_bank<=1;display_bank<=0;matched<=0;best_index<=MAX_OBJECTS;hold_index<=0;
            width_top<=0;width_q1<=0;width_mid<=0;width_q3<=0;width_bottom<=0;center_top<=0;center_bottom<=0;
            track_i<=0;track_j<=0;best_kind<=0;best_candidate<=0;best_hits<=0;best_distance<=4095;container<=0;
            state<=S_IDLE;bank<=0;proc_bank<=1;captured<=0;pending<=0;
            vs_d<=VS_ACTIVE;de_d<=0;enabled<=0;x<=0;y<=0;pool<=0;hpool<=0;
            head<=0;tail<=0;read_addr<=0;sx<=0;sy<=0;px<=0;py<=0;neighbor<=0;
            row_valid<=0;row<=0;xmin<=0;xmax<=0;ymin<=0;ymax<=0;
            row_width<=0;row_sum<=0;row_present<=0;row_mx<=0;row_my<=0;
            box_area<=0;center_x2<=0;center_y2<=0;dx<=0;dy<=0;area_width<=0;area_height<=0;
            area<=0;moment_x2<=0;moment_y2<=0;work_count<=0;display_count<=0;
            for(k=0;k<MAX_OBJECTS;k=k+1) begin
                work_misses[k]<=0;display_misses[k]<=0;work_stable[k]<=0;display_candidate[k]<=0;work_hits[k]<=0;display_hits[k]<=0;
                work_x0[k]<=0;work_x1[k]<=0;work_y0[k]<=0;work_y1[k]<=0;work_kind[k]<=0;
                display_x0[k]<=0;display_x1[k]<=0;display_y0[k]<=0;display_y1[k]<=0;display_kind[k]<=0;
            end
        end else begin
            vs_d<=vs_i;de_d<=de_i;
            if(de_i) begin
                x<=x+1'b1;
                if(enabled && x<IMAGE_WIDTH && y<IMAGE_HEIGHT) begin
                    if(cell_end) begin
                        pool[bx]<=row_end ? 1'b0 : cell_bit;hpool<=0;
                    end else hpool<=hpool|edge_i;
                end
            end else begin x<=0;hpool<=0;if(de_d) y<=y+1'b1;end
            if(frame_start) begin
                x<=0;y<=0;pool<=0;hpool<=0;enabled<=enable_i;
                if(!enable_i || !enabled) begin
                    captured<=0;display_count<=0;pending<=0;state<=S_IDLE;
                end else begin
                    captured<=1;
                    if(holding)state<=S_HOLD_READ;
                    if(pending) begin
                        display_count<=work_count;pending<=0;display_bank<=work_bank;work_bank<=~work_bank;
                        for(k=0;k<MAX_OBJECTS;k=k+1) begin
                            display_x0[k]<=work_x0[k];display_x1[k]<=work_x1[k];
                            display_y0[k]<=work_y0[k];display_y1[k]<=work_y1[k];display_kind[k]<=work_stable[k];
                            display_misses[k]<=work_misses[k];display_candidate[k]<=work_kind[k];display_hits[k]<=work_hits[k];
                        end
                    end
                    if(state==S_IDLE && captured) begin
                        proc_bank<=bank;bank<=~bank;work_count<=0;sx<=0;sy<=0;state<=S_SCAN;
                    end
                end
            end else case(state)
                S_IDLE:begin end
                S_SCAN:begin read_addr<=sy*W+sx;state<=S_WAIT;end
                S_WAIT:state<=S_CHECK;
                S_CHECK:begin
                    if(read_bit) begin
                        head<=0;tail<=1;row_valid<=0;
                        width_top<=0;width_q1<=0;width_mid<=0;width_q3<=0;width_bottom<=0;center_top<=0;center_bottom<=0;
                        xmin<=sx;xmax<=sx;ymin<=sy;ymax<=sy;
                        area<=0;moment_x2<=0;moment_y2<=0;state<=S_POP;
                    end else if(sx==W-1) begin
                        sx<=0;
                        if(sy==H-1) state<=S_FINISH;else begin sy<=sy+1'b1;state<=S_SCAN;end
                    end else begin sx<=sx+1'b1;state<=S_SCAN;end
                end
                S_POP:begin
                    if(head==tail) begin row<=0;state<=S_ROWS;end
                    else state<=S_POP_WAIT;
                end
                S_POP_WAIT:state<=S_POINT;
                S_POINT:begin
                    px<=queue_read[XB-1:0];py<=queue_read[QB-1:XB];head<=head+1'b1;
                    if(queue_read[XB-1:0]<xmin)xmin<=queue_read[XB-1:0];
                    if(queue_read[XB-1:0]>xmax)xmax<=queue_read[XB-1:0];
                    if(queue_read[QB-1:XB]<ymin)ymin<=queue_read[QB-1:XB];
                    if(queue_read[QB-1:XB]>ymax)ymax<=queue_read[QB-1:XB];
                    row_valid[queue_read[QB-1:XB]]<=1;
                    neighbor<=0;state<=S_NEIGHBOR;
                end
                S_NEIGHBOR:begin
                    if(neighbor==8)state<=S_POP;
                    else if(neighbor_valid)begin read_addr<=ny*W+nx;state<=S_NWAIT;end
                    else neighbor<=neighbor+1'b1;
                end
                S_NWAIT:state<=S_NCHECK;
                S_NCHECK:begin
                    if(read_bit && tail<N)tail<=tail+1'b1;
                    neighbor<=neighbor+1'b1;state<=S_NEIGHBOR;
                end
                S_ROWS:begin
                    if(row==H)state<=S_METRIC1;
                    else state<=S_ROW_WIDTH;
                end
                S_ROW_WIDTH:begin
                    row_width<={1'b0,row_right_q}-{1'b0,row_left_q}+1'b1;
                    row_sum<={1'b0,row_left_q}+{1'b0,row_right_q};
                    row_present<=row_valid[row];state<=S_ROW_PRODUCT;
                end
                S_ROW_PRODUCT:begin
                    row_mx<=row_width*row_sum;row_my<=row_width*{row,1'b0};
                    state<=S_ROW_ACC;
                end
                S_ROW_ACC:begin
                    if(row_present)begin
                        if(row==ymin+(height>>3))begin width_top<=row_width;center_top<=row_sum;end
                        if(row==ymin+(height>>2))width_q1<=row_width;
                        if(row==ymin+(height>>1))width_mid<=row_width;
                        if(row==ymax-(height>>2))width_q3<=row_width;
                        if(row==ymax-(height>>3))begin width_bottom<=row_width;center_bottom<=row_sum;end
                        area<=area+row_width;moment_x2<=moment_x2+row_mx;moment_y2<=moment_y2+row_my;
                    end
                    row<=row+1'b1;state<=S_ROWS;
                end
                S_METRIC1:begin
                    box_area<=width*height;
                    center_x2<=area*({1'b0,xmin}+{1'b0,xmax});
                    center_y2<=area*({1'b0,ymin}+{1'b0,ymax});
                    area_width<=area*width;area_height<=area*height;
                    state<=S_METRIC2;
                end
                S_METRIC2:begin
                    dx<=moment_x2>center_x2 ? moment_x2-center_x2 : center_x2-moment_x2;
                    dy<=moment_y2>center_y2 ? moment_y2-center_y2 : center_y2-moment_y2;
                    state<=S_CLASSIFY;
                end
                S_CLASSIFY:begin
                    if(kind!=0 && work_count<MAX_OBJECTS)begin
                        work_x0[work_count]<=xmin;work_x1[work_count]<=xmax;
                        work_y0[work_count]<=ymin;work_y1[work_count]<=ymax;work_kind[work_count]<=kind;
                        row<=0;state<=S_SAVE_WAIT;
                    end else state<=S_SCAN;
                end
                S_SAVE_WAIT:state<=S_SAVE_WRITE;
                S_SAVE_WRITE:begin
                    if(row==H-1) begin work_count<=work_count+1'b1;state<=S_SCAN;end
                    else begin row<=row+1'b1;state<=S_SAVE_WAIT;end
                end
                S_FINISH:begin matched<=0;track_i<=0;state<=S_TRACK_START;end
                S_TRACK_START:begin
                    if(track_i==work_count)begin hold_index<=0;state<=S_HOLD_SCAN;end
                    else begin
                        track_j<=0;best_index<=MAX_OBJECTS;best_distance<=4095;best_kind<=0;best_candidate<=0;best_hits<=0;container<=0;
                        state<=S_TRACK_SCAN;
                    end
                end
                S_TRACK_SCAN:begin
                    if(track_j==MAX_OBJECTS)state<=S_TRACK_DONE;
                    else begin
                        if(track_j<display_count && !matched[track_j] && track_distance<=12 && track_distance<best_distance)begin
                            best_index<=track_j;best_distance<=track_distance;best_kind<=display_kind[track_j];
                            best_candidate<=display_candidate[track_j];best_hits<=display_hits[track_j];
                        end
                        if(track_j<work_count && track_j!=track_i &&
                           work_x0[track_i]+2<work_x0[track_j] && work_x1[track_i]>work_x1[track_j]+2 &&
                           work_y0[track_i]+2<work_y0[track_j] && work_y1[track_i]>work_y1[track_j]+2)container<=1;
                        track_j<=track_j+1'b1;
                    end
                end
                S_TRACK_DONE:begin
                    work_misses[track_i]<=0;
                    if(best_index<MAX_OBJECTS)matched[best_index]<=1;
                    work_hits[track_i]<=next_hits;
                    work_stable[track_i]<=container ? 3'd0 : next_hits>=3 ? work_kind[track_i] : best_kind;
                    track_i<=track_i+1'b1;state<=S_TRACK_START;
                end
                
                S_HOLD_SCAN:begin
                    if(hold_index==display_count || work_count==MAX_OBJECTS)begin pending<=1;state<=S_IDLE;end
                    else if(!matched[hold_index] && display_kind[hold_index]!=0 && display_misses[hold_index]<2)begin
                        work_x0[work_count]<=display_x0[hold_index];work_x1[work_count]<=display_x1[hold_index];
                        work_y0[work_count]<=display_y0[hold_index];work_y1[work_count]<=display_y1[hold_index];
                        work_kind[work_count]<=display_candidate[hold_index];work_stable[work_count]<=display_kind[hold_index];
                        work_hits[work_count]<=display_hits[hold_index];work_misses[work_count]<=display_misses[hold_index]+1'b1;
                        row<=0;state<=S_HOLD_READ;
                    end else hold_index<=hold_index+1'b1;
                end
                S_HOLD_READ:if(de_i)state<=S_HOLD_WAIT;
                S_HOLD_WAIT:state<=de_i ? S_HOLD_WRITE : S_HOLD_READ;
                S_HOLD_WRITE:begin
                    if(!de_i)state<=S_HOLD_READ;
                    else if(row==H-1)begin work_count<=work_count+1'b1;hold_index<=hold_index+1'b1;state<=S_HOLD_SCAN;end
                    else begin row<=row+1'b1;state<=S_HOLD_READ;end
                end
                default:state<=S_IDLE;
            endcase
        end
    end
    function [23:0] color;
        input [2:0] c;
        begin case(c)
            1:color=24'hff5050;2:color=24'h40ff60;3:color=24'h40a0ff;4:color=24'hff60ff;
            default:color=0;
        endcase end
    endfunction
    // Font ROM is generated as literal constants; no external memory file dependency.
    wire [23:0] text_rgb;
    wire text_on;
    reg [2:0] text_kind;
    reg [6:0] text_x;
    reg [4:0] text_y;
    video_shape_font u_font(.kind_i(text_kind),.x_i(text_x),.y_i(text_y),.on_o(text_on));
    assign text_rgb=color(text_kind);
    // Add one-pixel horizontal support to the associated visible contour.
    reg edge_previous;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) edge_previous<=0;
        else if(!de_i) edge_previous<=0;
        else edge_previous<=edge_i;
    end
    wire edge_stroke=edge_i | edge_previous;
    reg [23:0] rendered;
    integer j,lx,ly;
    reg [XB-1:0] outline_left,outline_right,above_left,above_right,below_left,below_right;
    always @* begin
        rendered=rgb_i;text_kind=0;text_x=0;text_y=0;lx=0;ly=0;outline_left=0;outline_right=0;above_left=0;above_right=0;below_left=0;below_right=0;
        for(j=0;j<MAX_OBJECTS;j=j+1) begin
            if(enabled && j<display_count && display_kind[j]!=0)begin
                outline_left=profile_cache[j][2*XB-1:XB];outline_right=profile_cache[j][XB-1:0];
                above_left=profile_above[j][2*XB-1:XB];above_right=profile_above[j][XB-1:0];
                below_left=profile_below[j][2*XB-1:XB];below_right=profile_below[j][XB-1:0];
                /* Recolor input edges and one adjacent pixel within the observed
                   envelope and near its boundary. */
                if(edge_stroke && (x>>SCALE_LOG2)+1>=display_x0[j] && (x>>SCALE_LOG2)<=display_x1[j]+1 &&
                   (y>>SCALE_LOG2)>=display_y0[j] && (y>>SCALE_LOG2)<=display_y1[j] && outline_left<=outline_right &&
                   (x>>SCALE_LOG2)+1>=outline_left && (x>>SCALE_LOG2)<=outline_right+1 &&
                   ((y>>SCALE_LOG2)<=display_y0[j]+1 || (y>>SCALE_LOG2)+1>=display_y1[j] ||
                    (x>>SCALE_LOG2)<=outline_left+2 || (x>>SCALE_LOG2)+2>=outline_right ||
                    above_left>above_right || below_left>below_right ||
                    (x>>SCALE_LOG2)<=above_left+2 || (x>>SCALE_LOG2)+2>=above_right ||
                    (x>>SCALE_LOG2)<=below_left+2 || (x>>SCALE_LOG2)+2>=below_right)) rendered=color(display_kind[j]);
                lx=display_x0[j]<<SCALE_LOG2;
                if(lx>IMAGE_WIDTH-96)lx=IMAGE_WIDTH-96;
                ly=((display_y1[j]+1)<<SCALE_LOG2)+4;
                if(ly>IMAGE_HEIGHT-32)ly=IMAGE_HEIGHT-32;
                if(x>=lx && x<lx+96 && y>=ly && y<ly+32) begin
                    text_kind=display_kind[j];text_x=x-lx;text_y=y-ly;
                end
            end
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n)begin rgb_o<=0;hs_o<=1;vs_o<=VS_ACTIVE;de_o<=0;end
        else begin
            hs_o<=hs_i;vs_o<=vs_i;de_o<=de_i;
            rgb_o<=de_i ? ((enabled && text_on) ? text_rgb : rendered) : 24'd0;
        end
    end
endmodule
