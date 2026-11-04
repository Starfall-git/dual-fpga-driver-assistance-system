module video_median3x3 #(
    parameter VS_ACTIVE = 1'b0
)(
    input  wire        clk,
    input  wire        rst_n,

    input  wire [71:0] pixels_i,

    input  wire        hs_i,
    input  wire        vs_i,
    input  wire        de_i,
    input  wire        window_valid_i,

    output reg  [23:0] rgb_o,
    output reg [23:0] median_rgb_o,
    output reg         hs_o,
    output reg         vs_o,
    output reg         de_o,
    /* distinguish padded black border from a real black pixel. */
    output reg         pixel_valid_o
);


    /*==========================================================
     * 3x3 Window
     *=========================================================*/

    wire [7:0] p00 = pixels_i[71:64];
    wire [7:0] p01 = pixels_i[63:56];
    wire [7:0] p02 = pixels_i[55:48];

    wire [7:0] p10 = pixels_i[47:40];
    wire [7:0] p11 = pixels_i[39:32];
    wire [7:0] p12 = pixels_i[31:24];

    wire [7:0] p20 = pixels_i[23:16];
    wire [7:0] p21 = pixels_i[15:8];
    wire [7:0] p22 = pixels_i[7:0];

    /*==========================================================
     * Stage 1
     *
     * 分别对三行的三个像素进行排序
     *
     * L = Low
     * M = Middle
     * H = High
     *=========================================================*/

    wire [7:0] l0_w;
    wire [7:0] m0_w;
    wire [7:0] h0_w;

    wire [7:0] l1_w;
    wire [7:0] m1_w;
    wire [7:0] h1_w;

    wire [7:0] l2_w;
    wire [7:0] m2_w;
    wire [7:0] h2_w;


    sort3_8bit u_sort_row0 (
        .a     (p00),
        .b     (p01),
        .c     (p02),

        .min_o (l0_w),
        .mid_o (m0_w),
        .max_o (h0_w)
    );


    sort3_8bit u_sort_row1 (
        .a     (p10),
        .b     (p11),
        .c     (p12),

        .min_o (l1_w),
        .mid_o (m1_w),
        .max_o (h1_w)
    );


    sort3_8bit u_sort_row2 (
        .a     (p20),
        .b     (p21),
        .c     (p22),

        .min_o (l2_w),
        .mid_o (m2_w),
        .max_o (h2_w)
    );


    /*
     * Stage-1 registers
     */

    reg [7:0] l0_s1, m0_s1, h0_s1;
    reg [7:0] l1_s1, m1_s1, h1_s1;
    reg [7:0] l2_s1, m2_s1, h2_s1;

    reg hs_s1;
    reg vs_s1;
    reg de_s1;
    reg valid_s1;


    /*==========================================================
     * Stage 2
     *
     * A = max(L0,L1,L2)
     *
     * B = median(M0,M1,M2)
     *
     * C = min(H0,H1,H2)
     *=========================================================*/


    /*
     * 求三个 Low 中的最大值
     */

    wire [7:0] low_max_01;

    assign low_max_01 =
        (l0_s1 > l1_s1) ? l0_s1 : l1_s1;

    wire [7:0] low_max;

    assign low_max =
        (low_max_01 > l2_s1) ? low_max_01 : l2_s1;


    /*
     * 求三个 Middle 中的中值
     */

    wire [7:0] middle_min_unused;
    wire [7:0] middle_median;
    wire [7:0] middle_max_unused;

    sort3_8bit u_sort_middle (
        .a     (m0_s1),
        .b     (m1_s1),
        .c     (m2_s1),

        .min_o (middle_min_unused),
        .mid_o (middle_median),
        .max_o (middle_max_unused)
    );


    /*
     * 求三个 High 中的最小值
     */

    wire [7:0] high_min_01;

    assign high_min_01 =
        (h0_s1 < h1_s1) ? h0_s1 : h1_s1;

    wire [7:0] high_min;

    assign high_min =
        (high_min_01 < h2_s1) ? high_min_01 : h2_s1;


    /*
     * Stage-2 registers
     */

    reg [7:0] candidate_a_s2;
    reg [7:0] candidate_b_s2;
    reg [7:0] candidate_c_s2;

    reg hs_s2;
    reg vs_s2;
    reg de_s2;
    reg valid_s2;


    /*==========================================================
     * Stage 3
     *
     * 对三个候选值再次求中值
     *
     * median9 = median(A,B,C)
     *=========================================================*/

    wire [7:0] final_min_unused;
    wire [7:0] median_value;
    wire [7:0] final_max_unused;


    sort3_8bit u_sort_final (
        .a     (candidate_a_s2),
        .b     (candidate_b_s2),
        .c     (candidate_c_s2),

        .min_o (final_min_unused),
        .mid_o (median_value),
        .max_o (final_max_unused)
    );
    /*==========================================================
     * Pipeline registers
     *=========================================================*/

    always @(posedge clk or negedge rst_n) begin

        if (!rst_n) begin

            /*---------------- Stage 1 ----------------*/

            l0_s1 <= 8'd0;
            m0_s1 <= 8'd0;
            h0_s1 <= 8'd0;

            l1_s1 <= 8'd0;
            m1_s1 <= 8'd0;
            h1_s1 <= 8'd0;

            l2_s1 <= 8'd0;
            m2_s1 <= 8'd0;
            h2_s1 <= 8'd0;

            hs_s1    <= 1'b1;
            vs_s1    <= ~VS_ACTIVE;
            de_s1    <= 1'b0;
            valid_s1 <= 1'b0;


            /*---------------- Stage 2 ----------------*/

            candidate_a_s2 <= 8'd0;
            candidate_b_s2 <= 8'd0;
            candidate_c_s2 <= 8'd0;

            hs_s2    <= 1'b1;
            vs_s2    <= ~VS_ACTIVE;
            de_s2    <= 1'b0;
            valid_s2 <= 1'b0;


            /*---------------- Stage 3 ----------------*/

            rgb_o <= 24'd0; median_rgb_o <= 24'd0;

            hs_o <= 1'b1;
            vs_o <= ~VS_ACTIVE;
            de_o <= 1'b0;
            pixel_valid_o <= 1'b0;

        end
        else begin

            /*==================================================
             * Stage 1
             *=================================================*/

            l0_s1 <= l0_w;
            m0_s1 <= m0_w;
            h0_s1 <= h0_w;

            l1_s1 <= l1_w;
            m1_s1 <= m1_w;
            h1_s1 <= h1_w;

            l2_s1 <= l2_w;
            m2_s1 <= m2_w;
            h2_s1 <= h2_w;

            hs_s1    <= hs_i;
            vs_s1    <= vs_i;
            de_s1    <= de_i;
            valid_s1 <= window_valid_i;


            /*==================================================
             * Stage 2
             *=================================================*/

            candidate_a_s2 <= low_max;
            candidate_b_s2 <= middle_median;
            candidate_c_s2 <= high_min;

            hs_s2    <= hs_s1;
            vs_s2    <= vs_s1;
            de_s2    <= de_s1;
            valid_s2 <= valid_s1;


            /*==================================================
             * Stage 3
             *=================================================*/

            hs_o <= hs_s2;
            vs_o <= vs_s2;
            de_o <= de_s2;
            pixel_valid_o <= de_s2 && valid_s2;

            if (de_s2 && valid_s2) begin
                median_rgb_o <= {3{median_value}};

                /*
                 * 灰度结果复制到 RGB 三通道
                 */

                rgb_o <= {
                    median_value,
                    median_value,
                    median_value
                };

            end
            else begin

                rgb_o <= 24'd0; median_rgb_o <= 24'd0;

            end

        end

    end


endmodule
