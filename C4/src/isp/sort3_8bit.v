module sort3_8bit (
    input  wire [7:0] a,
    input  wire [7:0] b,
    input  wire [7:0] c,

    output wire [7:0] min_o,
    output wire [7:0] mid_o,
    output wire [7:0] max_o
);


    /*
     * Comparator 1
     *
     * 比较 a 和 b
     */

    wire [7:0] ab_min;
    wire [7:0] ab_max;

    assign ab_min = (a < b) ? a : b;
    assign ab_max = (a < b) ? b : a;


    /*
     * Comparator 2
     *
     * 比较较大的 ab_max 和 c
     *
     * 得到三个数中的最大值
     */

    wire [7:0] temp_mid;

    assign temp_mid = (ab_max < c) ? ab_max : c;
    assign max_o    = (ab_max < c) ? c      : ab_max;


    /*
     * Comparator 3
     *
     * 比较 ab_min 和 temp_mid
     *
     * 得到最小值和中间值
     */

    assign min_o =
        (ab_min < temp_mid) ? ab_min : temp_mid;

    assign mid_o =
        (ab_min < temp_mid) ? temp_mid : ab_min;


endmodule