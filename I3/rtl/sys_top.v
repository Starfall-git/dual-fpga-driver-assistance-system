/* FATIGUE-SC431-integrated top: shared DDR, 1080p input, CPU JTAG. */

/////////////////////////////////////////////////////////////////////////////
//
// Copyright (C) 2013-2021 Efinix Inc. All rights reserved.
//
// Description:
// Example top file for ti60f225 dev kit OOB design
//
// Language:  Verilog 2001
//Ti60f225_sc431hai2hdmi_v6：
//使用了最新版本的framebuffer_v3;
// ------------------------------------------------------------------------------

/////////////////////////////////////////////////////////////////////////////////
//`define SOFT_TAP 1
`include "ddr3_controller/ddr3_parameter.vh"
// `include "define.v"

module sys_top #(

	parameter                       RANK_RATIO         = 1,       // # of unique CS outputs per rank
	parameter                       ASYN_AXI_CLK       = `ASYN_AXI_CLK, 
	parameter                       RANKS              = `RANKS,
	parameter                       CK_WIDTH           = `CK_WIDTH,       // # of CK/CK# outputs to memory   
	parameter                       CKE_WIDTH          = `CKE_WIDTH,       // # of cke outputs
	parameter                       CS_WIDTH           = `CS_WIDTH,       // # of unique CS outputs
	parameter                       BANK_WIDTH         = `BANK_WIDTH,       // # of bank bits
	parameter                       ROW_WIDTH          = `ROW_WIDTH,       // DRAM address bus width
	parameter                       COL_WIDTH          = `COL_WIDTH,      // column address width
	parameter                       DM_WIDTH           = `DM_WIDTH,       // # of DM (data mask)
	parameter                       DQS_WIDTH          = `DQS_WIDTH,       // # of DQS (strobe)
	parameter                       DQ_WIDTH           = `DQ_WIDTH,      // # of DQ (data)
	parameter                       ODT_WIDTH          = `ODT_WIDTH,
	parameter                       DQ_CNT_WIDTH       = `DQ_CNT_WIDTH,       // = ceil(log2(DQ_WIDTH))
	parameter                       DQS_CNT_WIDTH      = `DQS_CNT_WIDTH,       // = ceil(log2(DQS_WIDTH))  
	parameter                       DRAM_WIDTH         = `DRAM_WIDTH,       // # of DQ per DQS   
	parameter                       DATA_WIDTH         = `DATA_WIDTH,
	parameter                       ADDR_WIDTH         = `ADDR_WIDTH,    
	parameter                       AXI_ID_WIDTH       = `AXI_ID_WIDTH,
	parameter                       AXI_ADDR_WIDTH     = `AXI_ADDR_WIDTH,
	parameter                       AXI_DATA_WIDTH     = `AXI_DATA_WIDTH
)
(

  (* syn_peri_port = 0 *) input ge_0_gclk_pll_LOCKED,
  (* syn_peri_port = 0 *) input rxc,rx_dv_HI,rx_dv_LO,
  (* syn_peri_port = 0 *) input [3:0] rxd_hi_i,rxd_lo_i,
  (* syn_peri_port = 0 *) output phy_rst_n,ge_rx_pll_RSTN,mdc_o,mdio_o,mdio_oe,
  (* syn_peri_port = 0 *) input mdio_i,
  (* syn_peri_port = 0 *) output tx_en_o_HI,tx_en_o_LO,txc_hi_o,txc_lo_o,
  (* syn_peri_port = 0 *) output [3:0] txd_hi_o,txd_lo_o,
  /* UART channel to board USB serial. */
  output wire fatigue_uart_tx,
  input wire fatigue_uart_rx,
  (* syn_peri_port = 0 *) input jtag_inst1_CAPTURE,
  (* syn_peri_port = 0 *) input jtag_inst1_DRCK,
  (* syn_peri_port = 0 *) input jtag_inst1_RESET,
  (* syn_peri_port = 0 *) input jtag_inst1_RUNTEST,
  (* syn_peri_port = 0 *) input jtag_inst1_SEL,
  (* syn_peri_port = 0 *) input jtag_inst1_SHIFT,
  (* syn_peri_port = 0 *) input jtag_inst1_TCK,
  (* syn_peri_port = 0 *) input jtag_inst1_TDI,
  (* syn_peri_port = 0 *) input jtag_inst1_TMS,
  (* syn_peri_port = 0 *) input jtag_inst1_UPDATE,
  (* syn_peri_port = 0 *) output jtag_inst1_TDO,

   //Clocks 
	input	wire	i_arstn,

    input wire  vid_clk_dvi2,
    input wire hdmi_tx_slow_clk,

    input  wire CLK_25M,
	output  wire    pll_inst1_RSTN,
	input	wire	i_pll_locked,
	input 	wire  	pll_locked,
	output  wire    pll_inst4_RSTN,
	input   wire    pll_inst4_LOCKED,
	output  wire  	USER_PLL_RSTN,
    output 	wire 	DDR3_PLL_RSTN,
	input 	wire  	user_pll_locked,
    //CSI Interface
    input	 wire		io_cam_sda_IN,
    output wire   io_cam_sda_OUT,
    output wire		io_cam_sda_OE,

    output io_cam_scl_OUT,
    input io_cam_scl_IN,
    output io_cam_scl_OE,

    output	wire		o_cam_rst_p,
    
    input	wire		i_cam_ck_LP_P_IN,
    input	wire		i_cam_ck_LP_N_IN,
    output	wire		o_cam_ck_HS_TERM,
    output	wire		o_cam_ck_HS_ENA,
    input	wire		i_cam_ck_CLKOUT,
    
    input	wire	[7:0]			cam_d0_HS_IN,
    input	  wire		      cam_d0_LP_P_IN,
    input	  wire		      cam_d0_LP_N_IN,
    output	wire		      cam_d0_HS_TERM,
    output	wire		      cam_d0_HS_ENA,
    output	wire		      cam_d0_RST,
    output	wire		      cam_d0_FIFO_RD,
    input	  wire		      cam_d0_FIFO_EMPTY,
    
    input 	wire	[7:0]			cam_d1_HS_IN,
    input	  wire		        cam_d1_LP_P_IN,
    input	  wire		        cam_d1_LP_N_IN,
    output	wire		        cam_d1_HS_TERM,
    output	wire		        cam_d1_HS_ENA,
    output	wire		        cam_d1_RST,
    output	wire		        cam_d1_FIFO_RD,
    input	  wire		        cam_d1_FIFO_EMPTY,
    
    input 	wire	[7:0]			cam_d2_HS_IN,
    input	  wire		        cam_d2_LP_P_IN,
    input	  wire		        cam_d2_LP_N_IN,
    output	wire		        cam_d2_HS_TERM,
    output	wire		        cam_d2_HS_ENA,
    output	wire		        cam_d2_RST,
    output	wire		        cam_d2_FIFO_RD,
    input	  wire		        cam_d2_FIFO_EMPTY,  
   
    input 	wire	[7:0]			cam_d3_HS_IN,
    input	  wire		        cam_d3_LP_P_IN,
    input	  wire		        cam_d3_LP_N_IN,
    output	wire		        cam_d3_HS_TERM,
    output	wire		        cam_d3_HS_ENA,
    output	wire		        cam_d3_RST,
    output	wire		        cam_d3_FIFO_RD,
    input	  wire		        cam_d3_FIFO_EMPTY,

	
	input 					tx_cal_clk_90edge,
  	input 					rx_cal_clk,
  	input 					tx_cal_clk,
	input                              core_clk,     // CORE CLK @ 100MHz
	input                              sdram_clk,    // SDRAM CK @ 400MHz
	// PLL status flags  
	output [2:0]                       pll_shift,  
	output [4:0]                       pll_shift_sel,
	output                             pll_shift_ena,  
	// memory interface ports
	output                             ddr_ck_hi,
	output                             ddr_ck_lo,
	output                             ddr_reset_n,
	output [CKE_WIDTH-1:0]             ddr_cke,     
	output [ROW_WIDTH-1:0]             ddr_addr,
	output [BANK_WIDTH-1:0]            ddr_ba,
	output                             ddr_cas_n,
 
	output [CS_WIDTH*RANK_RATIO-1:0]   ddr_cs_n,
	output                             ddr_ras_n,
	output                             ddr_we_n,
	
	input  [DQS_WIDTH-1:0]             ddr_dqs_in_hi,
	input  [DQS_WIDTH-1:0]             ddr_dqs_in_lo,
	input  [DQ_WIDTH-1:0]              ddr_dq_in_hi,
	input  [DQ_WIDTH-1:0]              ddr_dq_in_lo,
	
	output [DQS_WIDTH-1:0]             ddr_dqs_oe,
	output [DQS_WIDTH-1:0]             ddr_dqs_oe_n,
	output [DQ_WIDTH-1:0]              ddr_dq_oe,  
	output [DQS_WIDTH-1:0]             ddr_dqs_out_hi,
	output [DQS_WIDTH-1:0]             ddr_dqs_out_lo,
	output [DQ_WIDTH-1:0]              ddr_dq_out_hi,
	output [DQ_WIDTH-1:0]              ddr_dq_out_lo,
	output [DM_WIDTH-1:0]              ddr_dm_hi,
	output [DM_WIDTH-1:0]              ddr_dm_lo,
	output [ODT_WIDTH-1:0]             ddr_odt,

       //LED
       output [7:0] led,

       // MIPI DSI
       input	wire	                     i_mipi_tx_pclk		,
       output	wire	                     mipi_dp_clk_LP_P_OUT		,
       output	wire	                     mipi_dp_clk_LP_N_OUT		,
       output	wire	[7:0] 	              mipi_dp_clk_HS_OUT		,
       output	wire	                     mipi_dp_clk_HS_OE		,
       output	wire	                     mipi_dp_data3_LP_P_OUT	,
       output	wire	                     mipi_dp_data2_LP_P_OUT	,
       output	wire	                     mipi_dp_data1_LP_P_OUT	,
       output	wire	                     mipi_dp_data0_LP_P_OUT	,
       output	wire	                     mipi_dp_data3_LP_N_OUT	,
       output	wire	                     mipi_dp_data2_LP_N_OUT	,
       output	wire	                     mipi_dp_data1_LP_N_OUT	,
       output	wire	                     mipi_dp_data0_LP_N_OUT	,
       output	wire	[7:0] 	              mipi_dp_data0_HS_OUT	       ,
       output	wire	[7:0] 	              mipi_dp_data1_HS_OUT	       ,
       output	wire	[7:0] 	              mipi_dp_data2_HS_OUT	       ,
       output	wire	[7:0] 	              mipi_dp_data3_HS_OUT	       ,
       output	wire	                     mipi_dp_data3_HS_OE		,
       output	wire	                     mipi_dp_data2_HS_OE		,
       output	wire	                     mipi_dp_data1_HS_OE		,
       output	wire	                     mipi_dp_data0_HS_OE		,

       output	wire	                     mipi_dp_clk_RST		,
       output	wire	                     mipi_dp_data0_RST		,
       output	wire	                     mipi_dp_data1_RST		,
       output	wire	                     mipi_dp_data2_RST		,
       output	wire	                     mipi_dp_data3_RST		,
       output	wire	                     mipi_dp_clk_LP_P_OE		,
       output	wire	                     mipi_dp_clk_LP_N_OE		,
       output	wire	                     mipi_dp_data3_LP_P_OE	,
       output	wire	                     mipi_dp_data3_LP_N_OE	,
       output	wire	                     mipi_dp_data2_LP_P_OE	,
       output	wire	                     mipi_dp_data2_LP_N_OE	,
       output	wire	                     mipi_dp_data1_LP_P_OE	,
       output	wire	                     mipi_dp_data1_LP_N_OE	,
       output	wire	                     mipi_dp_data0_LP_P_OE	,
       output	wire	                     mipi_dp_data0_LP_N_OE	,

       input  wire	                     mipi_dp_data0_LP_P_IN	,
       input  wire	                     mipi_dp_data0_LP_N_IN	,
       output	wire	                     LCD_RST_P			,
       output wire                        LCD_POWER			,

       // hdmi interface

    output tmds_tx_clk_TX_OE,
    output [9:0] tmds_tx_clk_TX_DATA,
    output tmds_tx_clk_TX_RST,
    output tmds_tx_data0_TX_OE,
    output [9:0] tmds_tx_data0_TX_DATA,
    output tmds_tx_data0_TX_RST,
    output tmds_tx_data1_TX_OE,
    output [9:0] tmds_tx_data1_TX_DATA,
    output tmds_tx_data1_TX_RST,
    output tmds_tx_data2_TX_OE,
    output [9:0] tmds_tx_data2_TX_DATA,
    output tmds_tx_data2_TX_RST


);

function integer log2;
	input	integer	val;
	integer	i;
	begin
		log2 = 0;
		for (i=0; 2**i<val; i=i+1)
			log2 = i+1;
	end
endfunction


//===============================================================================
//localparam
//===============================================================================
localparam WR_FIFO_DEPTH  = 1024;
localparam RD_FIFO_DEPTH  = 1024;


localparam HACT     = 1920;
localparam HFP      = 88;
localparam HSP      = 44;
localparam HBP      = 148;

localparam VACT     = 1080;
localparam VFP      = 4;
localparam VSP      = 5;
localparam VBP      = 36;
//===============================================================================
//signal
//===============================================================================
   
wire                              app_sr_active;
wire                              app_ref_ack;
wire                              app_zq_ack;
wire                              cal_done;

// Slave Interface Write Address Ports
wire [AXI_ID_WIDTH-1:0]           s_axi_awid;
wire [AXI_ADDR_WIDTH-1:0]         s_axi_awaddr;
wire [7:0]                        s_axi_awlen;
wire [2:0]                        s_axi_awsize;
wire [1:0]                        s_axi_awburst;
wire [0:0]                        s_axi_awlock;
wire [3:0]                        s_axi_awcache;
wire [2:0]                        s_axi_awprot;
wire                              s_axi_awvalid;
wire                              s_axi_awready;
// Slave Interface Write Data Ports
wire [AXI_DATA_WIDTH-1:0]         s_axi_wdata;
wire [(AXI_DATA_WIDTH/8)-1:0]     s_axi_wstrb;
wire                              s_axi_wlast;
wire                              s_axi_wvalid;
wire                              s_axi_wready;
// Slave Interface Write Response Ports
wire                              s_axi_bready;
wire [AXI_ID_WIDTH-1:0]           s_axi_bid;
wire [1:0]                        s_axi_bresp;
wire                              s_axi_bvalid;
// Slave Interface Read Address Ports
wire [AXI_ID_WIDTH-1:0]           s_axi_arid;
wire [AXI_ADDR_WIDTH-1:0]         s_axi_araddr;
wire [7:0]                        s_axi_arlen;
wire [2:0]                        s_axi_arsize;
wire [1:0]                        s_axi_arburst;
wire [0:0]                        s_axi_arlock;
wire [3:0]                        s_axi_arcache;
wire [2:0]                        s_axi_arprot;
wire                              s_axi_arvalid;
wire                              s_axi_arready;
// Slave Interface Read Data Ports
wire                              s_axi_rready;
wire [AXI_ID_WIDTH-1:0]           s_axi_rid;
wire [AXI_DATA_WIDTH-1:0]         s_axi_rdata;
wire [1:0]                        s_axi_rresp;
wire                              s_axi_rlast;
wire                              s_axi_rvalid;
/* FATIGUE-SC431-DDR-side bus after shared arbiter. */
wire [AXI_ID_WIDTH-1:0]           d_axi_awid;
wire [AXI_ADDR_WIDTH-1:0]         d_axi_awaddr;
wire [7:0]                        d_axi_awlen;
wire [2:0]                        d_axi_awsize;
wire [1:0]                        d_axi_awburst;
wire [0:0]                        d_axi_awlock;
wire [3:0]                        d_axi_awcache;
wire [2:0]                        d_axi_awprot;
wire                              d_axi_awvalid;
wire                              d_axi_awready;
// Slave Interface Write Data Ports
wire [AXI_DATA_WIDTH-1:0]         d_axi_wdata;
wire [(AXI_DATA_WIDTH/8)-1:0]     d_axi_wstrb;
wire                              d_axi_wlast;
wire                              d_axi_wvalid;
wire                              d_axi_wready;
// Slave Interface Write Response Ports
wire                              d_axi_bready;
wire [AXI_ID_WIDTH-1:0]           d_axi_bid;
wire [1:0]                        d_axi_bresp;
wire                              d_axi_bvalid;
// Slave Interface Read Address Ports
wire [AXI_ID_WIDTH-1:0]           d_axi_arid;
wire [AXI_ADDR_WIDTH-1:0]         d_axi_araddr;
wire [7:0]                        d_axi_arlen;
wire [2:0]                        d_axi_arsize;
wire [1:0]                        d_axi_arburst;
wire [0:0]                        d_axi_arlock;
wire [3:0]                        d_axi_arcache;
wire [2:0]                        d_axi_arprot;
wire                              d_axi_arvalid;
wire                              d_axi_arready;
// Slave Interface Read Data Ports
wire                              d_axi_rready;
wire [AXI_ID_WIDTH-1:0]           d_axi_rid;
wire [AXI_DATA_WIDTH-1:0]         d_axi_rdata;
wire [1:0]                        d_axi_rresp;
wire                              d_axi_rlast;
wire                              d_axi_rvalid;




////////////////////////////////////////////////////////////////
// System & Debugger
wire    w_arstn;
// wire	w_sysclk_mcu_arstn;
// wire	w_sysclk_mcu_arst;
wire	w_mipi_rx_pclk_arstn;
wire	w_mipi_rx_pclk_arst;

wire		       w_mipi_rx_vs;
wire		       w_mipi_rx_hs;
wire	       w_mipi_rx_de;
wire	[63:0]			w_mipi_rx_data	;

wire					io_systemReset;



//===================================================================================================
//reset module
//===================================================================================================

assign w_arstn  		       = i_pll_locked & pll_locked & user_pll_locked;
assign pll_inst1_RSTN  		= i_arstn;
assign USER_PLL_RSTN      		= i_arstn;
assign mipi_dsi_tx_pll_RSTN 	= i_arstn;
assign DDR3_PLL_RSTN 		= i_arstn;
reset_ctrl
#(
	.NUM_RST		(2),
	.CYCLE			(1),
	.IN_RST_ACTIVE	(4'b00),
	.OUT_RST_ACTIVE	(4'b10)
)
inst_reset_ctrl
(
	.i_arst		({2{w_arstn}}),//({{4{i_pll_locked}},                  {2{i_arstn}}}),
	.i_clk			({2{i_mipi_rx_pclk}}),
	.o_srst		({w_mipi_rx_pclk_arst, w_mipi_rx_pclk_arstn})
);



reg [25:0] cnt = 'd0;
always @( posedge core_clk)
begin 
       cnt <= cnt + 1'b1;
end
// Diagnostic build: unconditional core heartbeat separates missing core clock
// from DDR calibration failure. LED1 reports DDR calibration, subject to the
// development board's LED polarity.
assign led[0] = cnt[24];
assign led[1] = cal_done;

//========================================================================================================
//MIPI RX
//========================================================================================================
wire i_mipi_clk ;
assign i_mipi_clk = core_clk;

		assign	cam_d0_RST		= 1'b0;
		assign	cam_d1_RST		= 1'b0;
		assign  cam_d2_RST = 1'b0;
    assign  cam_d3_RST = 1'b0;
	
wire [15:0] mipi_debug_out;
		

		csi_rx_controller inst_efx_csi2_rx
		(
              .reset_n			(i_arstn),
              .clk				(i_mipi_clk),
              .reset_byte_HS_n	(i_arstn),
              .clk_byte_HS		  (i_cam_ck_CLKOUT),
              .reset_pixel_n		(w_mipi_rx_pclk_arstn),
              .clk_pixel			  (i_mipi_rx_pclk),
              
              .Rx_LP_CLK_P		  (i_cam_ck_LP_P_IN),
              .Rx_LP_CLK_N		  (i_cam_ck_LP_N_IN),
              .Rx_HS_enable_C		(o_cam_ck_HS_ENA),
              .LVDS_termen_C		(o_cam_ck_HS_TERM),
              
              .Rx_LP_D_P			({cam_d3_LP_P_IN, cam_d2_LP_P_IN, cam_d1_LP_P_IN, cam_d0_LP_P_IN}),//(r_mipi_rx_data_LP_P_IN_2P),
              .Rx_LP_D_N			({cam_d3_LP_N_IN, cam_d2_LP_N_IN, cam_d1_LP_N_IN, cam_d0_LP_N_IN}),//(r_mipi_rx_data_LP_N_IN_2P),
              .Rx_HS_D_0			(cam_d0_HS_IN),//(r_mipi_rx_data_HS_IN_2P[0*8+:8]),
              .Rx_HS_D_1			(cam_d1_HS_IN),//(r_mipi_rx_data_HS_IN_2P[1*8+:8]),
              .Rx_HS_D_2			(cam_d2_HS_IN),
              .Rx_HS_D_3			(cam_d3_HS_IN),
              .Rx_HS_D_4			(),
              .Rx_HS_D_5			(),
              .Rx_HS_D_6			(),
              .Rx_HS_D_7			(),
              .Rx_HS_enable_D		({cam_d3_HS_ENA    ,cam_d2_HS_ENA    ,cam_d1_HS_ENA    ,cam_d0_HS_ENA    }),//( w_cam_d_HS_ENA),
              .LVDS_termen_D		({cam_d3_HS_TERM   ,cam_d2_HS_TERM   ,cam_d1_HS_TERM   ,cam_d0_HS_TERM   }),
              .fifo_rd_enable		({cam_d3_FIFO_RD   ,cam_d2_FIFO_RD   ,cam_d1_FIFO_RD   ,cam_d0_FIFO_RD   }),
              .fifo_rd_empty		({cam_d3_FIFO_EMPTY,cam_d2_FIFO_EMPTY,cam_d1_FIFO_EMPTY,cam_d0_FIFO_EMPTY}),
              .DLY_enable_D		       (),
              .DLY_inc_D			(),
              .u_dly_enable_D		(),
              .u_dly_inc_D		       (),
              
              .axi_clk			(1'b0),
              .axi_reset_n		       (1'b0),
              .axi_awaddr			(6'b0),
              .axi_awvalid		       (1'b0),
              .axi_awready		       (),
              .axi_wdata			(32'b0),
              .axi_wvalid			(1'b0),
              .axi_wready			(),
              
              .axi_bvalid			(),
              .axi_bready			(1'b0),
              .axi_araddr			(6'b0),
              .axi_arvalid		       (1'b0),
              .axi_arready		       (),
              .axi_rdata			(),
              .axi_rvalid			(),
              .axi_rready			(1'b1),
              
              .hsync_vc0			(w_mipi_rx_hs),
              .hsync_vc1			(),
              .hsync_vc2			(),
              .hsync_vc3			(),
              .vsync_vc0			(w_mipi_rx_vs),
              .vsync_vc1			(),
              .vsync_vc2			(),
              .vsync_vc3			(),
              .vc				(),
              .word_count			(),
              .shortpkt_data_field        (),
              .datatype			(),
              .pixel_per_clk		(),
              .pixel_data			(w_mipi_rx_data),
              .pixel_data_valid	       (w_mipi_rx_de),
              .irq				(),
              .mipi_debug_out(mipi_debug_out)
		);


//     .o_hs(w1_mipi_rx_hs),
//     .o_vs(w1_mipi_rx_vs),
//     .o_de(w1_mipi_rx_de),
//     .o_dat(w1_mipi_rx_data)
   
//   );


wire mdebug_pixel_fifo_full     = mipi_debug_out[0] ;
wire mdebug_pixel_fifo_empty    = mipi_debug_out[1] ;
wire mdebug_crc_error           = mipi_debug_out[2] ;
wire mdebug_ecc_1bit_error      = mipi_debug_out[3] ;
wire mdebug_ecc_2bit_error      = mipi_debug_out[4] ;
wire mdebug_undersize_pkt_error = mipi_debug_out[5] ;
wire mdebug_line_vc0_error      = mipi_debug_out[6] ;
wire mdebug_frame_vc0_error    = mipi_debug_out[10];   
wire mdebug_receive_error      = mipi_debug_out[14];


//========================================================================================================
//
//========================================================================================================
		

										

wire scl_padoen_o;
wire sda_padoen_o;


reg [12:0] i2c_rst_cnt = '0;

always @( posedge CLK_5M or negedge w_arstn )
begin
    if( !w_arstn )
       i2c_rst_cnt <= 'd0;
    else 
       i2c_rst_cnt <= i2c_rst_cnt[12] ? i2c_rst_cnt : i2c_rst_cnt + 1'b1;
end
wire i2c_rst_n = i2c_rst_cnt[12];

i2c_master_ctrl_top u2_i2c_master_ctrl_top(
  /*i*/.clk			(CLK_5M		),
  /*i*/.rst_n			(i2c_rst_n		),
  /*i*/.scl_pad_i     (io_cam_scl_IN),//(1'b1),
  /*o*/.scl_pad_o     (io_cam_scl_OUT),
  /*o*/.scl_padoen_o  (scl_padoen_o),
  /*i*/.sda_pad_i     (io_cam_sda_IN),
  /*o*/.sda_pad_o     (io_cam_sda_OUT),
  /*o*/.sda_padoen_o  (sda_padoen_o)
  
  );
assign o_cam_rst_p = ~w_arstn;
assign io_cam_scl_OE = ~scl_padoen_o;
assign io_cam_sda_OE = ~sda_padoen_o;

//=====================================================================================
//frame buffer
//=====================================================================================
wire [7:0] 	ch0_r;
wire [15:0]    rawdata_2pix;
wire [7:0]    ch0_b;
wire ch0_vs;
wire ch0_hs;
wire ch0_de;

wire  [16-1:0] m_axis_tdata;
wire           m_axis_tvalid;
wire           m_axis_tready;
wire           m_axis_tlast;
wire           m_axis_tuser;
wire  fifo_rd_period;
// IMX219
  frame_buffer #(
.I_VID_WIDTH         (32),
.O_VID_WIDTH         (16),
.START_ADDR          (32'h00000        ),
.AXI_DATA_WIDTH      ( AXI_DATA_WIDTH	),
.AXI_ADDR_WIDTH      ( AXI_ADDR_WIDTH	),
.WR_FIFO_DEPTH	      ( 512		),    
.RD_FIFO_DEPTH 	      ( 512 	),
.BURST_LEN  	        (127),
.FB_NUM	              (3),
.MAX_VID_WIDTH	      (960) ,
.MAX_VID_HIGHT	      (1080),
.O_FRAME_WIDTH      (960),
.O_FRAME_HEIGHT     (1080)

)checker0(
       .axi_clk		(core_clk 	        ),
       .rst_n			(vid_rst_n   ),

/*i*/.i_clk			(i_mipi_rx_pclk      ),
/*i*/.i_vs			(w_mipi_rx_vs	),
/*i*/.i_de			(w_mipi_rx_de & w_mipi_rx_hs	),
/*i*/.vin 			({w_mipi_rx_data[9:2],w_mipi_rx_data[19:12],w_mipi_rx_data[29:22],w_mipi_rx_data[39:32]}	),
                     
/*i*/.o_clk			(vid_clk_dvi2),//(i_mipi_rx_pclk	),                  
/*i*/.fifo_rd_period(fifo_rd_period),//(fifo_rd_period  ),

    .m_axis_tdata  (m_axis_tdata),
    .m_axis_tvalid (m_axis_tvalid),
    .m_axis_tready (m_axis_tready),
    .m_axis_tuser  (m_axis_tuser),
    .m_axis_tlast  (m_axis_tlast),
	

       .awid			(s_axi_awid	),      
       .awaddr		(s_axi_awaddr	),
       .awlen			(s_axi_awlen	),
       .awsize		(s_axi_awsize	),
       .awburst		(s_axi_awburst),
       .awprot    (s_axi_awprot ),
       .awcache   (s_axi_awcache),
       .awlock		(s_axi_awlock	),
       .awvalid		(s_axi_awvalid),
       .awready		(s_axi_awready),

       .arid			(s_axi_arid	  ),
       .araddr		(s_axi_araddr	),
       .arlen			(s_axi_arlen	),
       .arsize		(s_axi_arsize	),
       .arburst 	(s_axi_arburst),

       .arprot    (s_axi_arprot ),
       .arcache   (s_axi_arcache),
       .arlock 		(s_axi_arlock	),
       .arvalid		(s_axi_arvalid),
       .arready		(s_axi_arready),

       .wdata			(s_axi_wdata	),
       .wstrb			(s_axi_wstrb	),
       .wlast			(s_axi_wlast	),
       .wvalid		(s_axi_wvalid	),
       .wready		(s_axi_wready	),


       .rid			  (s_axi_rid	),
       .rdata			(s_axi_rdata	),
       .rlast			(s_axi_rlast	),
       .rvalid 		(s_axi_rvalid	),
       .rready		(s_axi_rready	),
       .rresp			(s_axi_rresp	),

       .bid			  (s_axi_bid	),
       .bvalid		(s_axi_bvalid	),
       .bready	  (s_axi_bready	)
);

		    
		    
cvo_axi # (
    .SYMBOL_WIDTH(8),
    .SYMBOL_NUM(1),
    .PIXEL_NUM(2),
    .HSYNC_POL(1),
    .VSYNC_POL(1),
    .FIFO_DEPTH(1024),
    .FIFO_ALMOST_FULL(1000),
    .MAX_H_VALID(1920),
    .MAX_V_VALID(1080)
  )
  cvo_axi_inst (
    .o_clk          (vid_clk_dvi2   ),
    .rst_n          (vid_rst_n      ),
    .fifo_rd_period (fifo_rd_period ),
    .s_axi_clk      (vid_clk_dvi2   ),
    .s_axis_tdata   (m_axis_tdata   ),
    .s_axis_tvalid  (m_axis_tvalid  ),
    .s_axis_tready  (m_axis_tready  ),
    .s_axis_tlast   (m_axis_tlast   ),
    .s_axis_tuser   (m_axis_tuser   ),
    .H_FRONT_PORCH  (HFP            ),
    .H_SYNC         (HSP            ),
    .H_VALID        (HACT           ),
    .H_BACK_PORCH   (HBP            ),
    .V_FRONT_PORCH  (VFP            ),
    .V_SYNC         (VSP            ),
    .V_VALID        (VACT           ),
    .V_BACK_PORCH   (VBP            ),
     .vout          ({rawdata_2pix}  ),
    .o_hs           (ch0_hs         ),
    .o_vs           (ch0_vs         ),
    .o_de           (ch0_de         )
  );		       


ddr3_top                 u_ddr3_top
(

.axi_clk                (core_clk            ),
.core_clk               (core_clk            ),
.sdram_clk              (sdram_clk           ),  
.rx_cal_clk             (rx_cal_clk          ),
.tx_cal_clk             (tx_cal_clk          ),
.tx_cal_clk_90edge      (tx_cal_clk_90edge   ),
.rstn                   (w_arstn             ),      
.pll_shift              (pll_shift           ),
.pll_shift_sel          (pll_shift_sel       ),
.pll_shift_ena          (pll_shift_ena       ),       
///////////////DDR BUS
.ddr_ck_hi              (ddr_ck_hi           ),
.ddr_ck_lo              (ddr_ck_lo           ),
.ddr_cke                (ddr_cke             ),    
.ddr_reset_n            (ddr_reset_n         ),
.ddr_cs_n               (ddr_cs_n            ),
.ddr_ras_n              (ddr_ras_n           ),
.ddr_cas_n              (ddr_cas_n           ),
.ddr_we_n               (ddr_we_n            ),     
.ddr_addr               (ddr_addr            ),
.ddr_ba                 (ddr_ba              ),

.ddr_dqs_oe             (ddr_dqs_oe          ),
.ddr_dqs_oe_n           (ddr_dqs_oe_n        ),
.ddr_dq_oe              (ddr_dq_oe           ),
.ddr_dqs_in_hi          (ddr_dqs_in_hi       ),
.ddr_dqs_in_lo          (ddr_dqs_in_lo       ),
.ddr_dq_in_hi           (ddr_dq_in_hi        ),
.ddr_dq_in_lo           (ddr_dq_in_lo        ),

.ddr_dqs_out_hi         (ddr_dqs_out_hi      ),
.ddr_dqs_out_lo         (ddr_dqs_out_lo      ),
.ddr_dq_out_hi          (ddr_dq_out_hi       ),
.ddr_dq_out_lo          (ddr_dq_out_lo       ),

.ddr_dm_hi              (ddr_dm_hi           ),
.ddr_dm_lo              (ddr_dm_lo           ),
.ddr_odt                (ddr_odt             ),

// Application interface ports
.app_sr_req                     (1'b0),
.app_ref_req                    (1'b0),
.app_zq_req                     (1'b0),
.app_sr_active                  (app_sr_active),
.app_ref_ack                    (app_ref_ack),
.app_zq_ack                     (app_zq_ack),

// Slave Interface Write Address Ports
.s_axi_awid                     (d_axi_awid        ),
.s_axi_awaddr                   (d_axi_awaddr      ),
.s_axi_awlen                    (d_axi_awlen       ),
.s_axi_awsize                   (d_axi_awsize      ),
.s_axi_awburst                  (d_axi_awburst     ),
.s_axi_awlock                   (d_axi_awlock      ),
.s_axi_awcache                  (d_axi_awcache     ),
.s_axi_awprot                   (d_axi_awprot      ),
.s_axi_awqos                    (4'h0              ),
.s_axi_awvalid                  (d_axi_awvalid     ),
.s_axi_awready                  (d_axi_awready     ),
// Slave Interface Write Data Ports
.s_axi_wdata                    (d_axi_wdata       ),
.s_axi_wstrb                    (d_axi_wstrb       ),
.s_axi_wlast                    (d_axi_wlast       ),
.s_axi_wvalid                   (d_axi_wvalid      ),
.s_axi_wready                   (d_axi_wready      ),
// Slave Interface Write Response Ports
.s_axi_bid                      (d_axi_bid         ),
.s_axi_bresp                    (d_axi_bresp       ),
.s_axi_bvalid                   (d_axi_bvalid      ),
.s_axi_bready                   (d_axi_bready      ),
// Slave Interface Read Address Ports
.s_axi_arid                     (d_axi_arid        ),
.s_axi_araddr                   (d_axi_araddr      ),
.s_axi_arlen                    (d_axi_arlen       ),
.s_axi_arsize                   (d_axi_arsize      ),
.s_axi_arburst                  (d_axi_arburst     ),
.s_axi_arlock                   (d_axi_arlock      ),
.s_axi_arcache                  (d_axi_arcache     ),
.s_axi_arprot                   (d_axi_arprot      ),
.s_axi_arqos                    (4'h0              ),
.s_axi_arvalid                  (d_axi_arvalid     ),
.s_axi_arready                  (d_axi_arready     ),
// Slave Interface Read Data Ports
.s_axi_rid                      (d_axi_rid         ),
.s_axi_rdata                    (d_axi_rdata       ),
.s_axi_rresp                    (d_axi_rresp       ),
.s_axi_rlast                    (d_axi_rlast       ),
.s_axi_rvalid                   (d_axi_rvalid      ),
.s_axi_rready                   (d_axi_rready      ),
//DEBUG       
.wrlvl_dq_check                 (wrlvl_dq_check    ) ,
.rd_level_dqs_check             (rd_level_dqs_check) ,
.init_cur_state                 (init_cur_state    ) ,
.idelay_ld                      (idelay_ld         ) ,
.mpr_rdlvl_dly                  (mpr_rdlvl_dly     ) ,
.cal_done                       (cal_done          ) 
);

//***************************************************************************
// debayer
//***************************************************************************


wire        rgb_vs;
wire        rgb_hs;
wire        rgb_de;
wire        rgb_valid;
wire [47:0] rgb_datax2;

// rgb_sharpen -> rgb_ccm -> rgb_saturate 之间的中间信号。
// 拆成 shp_* / ccm_* / rgb_* 三级是为了让下游的 hdmi_tx_* 消费块一行都不用改:
// 消费块看到的输入名还是 rgb_*, 只是驱动者从 rgb_sharpen 换成了 rgb_saturate。
wire        shp_vs;
wire        shp_hs;
wire        shp_de;
wire        shp_valid;
wire [47:0] shp_datax2;

// rgb_ccm 的输出 —— 只在 rgb_ccm 与 rgb_saturate 之间走线
wire        ccm_vs;
wire        ccm_hs;
wire        ccm_de;
wire        ccm_valid;
wire [47:0] ccm_datax2;

wire [15:0] r_gain;
wire [15:0] g_gain;
wire [15:0] b_gain;

wire        blc_vs;
wire        blc_hs;
wire        blc_de;
wire        blc_valid;
wire [15:0] blc_rawdata;

raw_data_blc # (
    .P_DEPTH(8),
    .P_BLACK(8'd17)
  )
  raw_data_blc_inst (
    .i_vs     (ch0_vs                                ),
    .i_hs     (ch0_hs                                ),
    .i_de     (ch0_de                                ),
    .i_valid  (ch0_de                                ),  // 与 .raw_valid_i 同源
    .i_data   ({rawdata_2pix[7:0],rawdata_2pix[15:8]}),
    .o_vs     (blc_vs                                ),
    .o_hs     (blc_hs                                ),
    .o_de     (blc_de                                ),
    .o_valid  (blc_valid                             ),
    .o_data   (blc_rawdata                           )
  );

wire        nr_vs;
wire        nr_hs;
wire        nr_de;
wire        nr_valid;
wire [15:0] nr_rawdata;
wire        nr_bypass;
/* FATIGUE-SC431-bypass RAW denoise to reduce BRAM; retain matched pixel/control signals. */
assign nr_vs=blc_vs; assign nr_hs=blc_hs; assign nr_de=blc_de;
assign nr_valid=blc_valid; assign nr_rawdata=blc_rawdata;

// debayer 的原始输出 -> 锐化模块的输入 (必须在下面用之前声明, 否则会被当成
// 隐式线网再和显式声明撞车)
wire        db_vs;
wire        db_hs;
wire        db_de;
wire        db_valid;
wire [47:0] db_datax2;

debayer_top_wbg_2to1 # (
    .GAMMA_BYPASS(0)
  )
  debayer_top_wbg_2to1_inst (
    .in_pclk(vid_clk_dvi2),
    .in_rstn(vid_rst_n),
    .raw_vs_i     (nr_vs       ),
    .raw_hs_i     (nr_hs       ),
    .raw_de_i     (nr_de       ),
    .raw_valid_i  (nr_valid    ),
    .raw_datax4_i (nr_rawdata  ),
    .wb_r_gain      (|r_gain? r_gain : 16'd17826),   // 1.0880
    .wb_g_gain      (|g_gain? g_gain : 16'd24718),   // 1.5087
    .wb_b_gain      (|b_gain? b_gain : 16'd17778),   // 1.0851

    .rgb_vs_o     (db_vs       ),
    .rgb_hs_o     (db_hs       ),
    .rgb_de_o     (db_de       ),
    .rgb_valid_o  (db_valid    ),
    .rgb_datax2_o (db_datax2   )
  );

  wire sharpen_bypass;
rgb_sharpen # (
    .P_SHARP_GAIN (16'd95535    ),
    // .P_BYPASS     (1'b0        ),
    .X_CNT_WIDTH  (10          )
  )
  rgb_sharpen_inst (
    .i_pclk   (vid_clk_dvi2),
    .i_arstn  (vid_rst_n   ),
    .P_BYPASS (sharpen_bypass ),
    .i_vs     (db_vs       ),
    .i_hs     (db_hs       ),
    .i_de     (db_de       ),
    .i_valid  (db_valid    ),
    .i_datax2 (db_datax2   ),
    .o_vs     (shp_vs      ),
    .o_hs     (shp_hs      ),
    .o_de     (shp_de      ),
    .o_valid  (shp_valid   ),
    .o_datax2 (shp_datax2  )
  );

  wire ccm_bypass;
rgb_ccm # (
    .M_RR      (16'sd16885),   // Q2.14 恒等矩阵: 16384 = 1.0, 其余为 0
    .M_RG      (16'sd3851   ),   // *** 恒等矩阵逐位等于直通 (已穷举 2^24 验证) ***
    .M_RB      (-16'sd4352   ),
    .M_GR      (-16'sd1126   ),
    .M_GG      (16'sd15054),
    .M_GB      (16'sd2456    ),
    .M_BR      (16'sd4491    ),
    .M_BG      (-16'sd3344   ),
    .M_BB      (16'sd15237)//,
    // .P_BYPASS  (1'b1      )    // 1 = 只 mux 数据, 延迟链不变
  )
  rgb_ccm_inst (
    .i_pclk   (vid_clk_dvi2),
    .i_arstn  (vid_rst_n   ),
    .P_BYPASS (ccm_bypass  ),
    .i_vs     (shp_vs      ),
    .i_hs     (shp_hs      ),
    .i_de     (shp_de      ),
    .i_valid  (shp_valid   ),
    .i_datax2 (shp_datax2  ),
    .o_vs     (ccm_vs      ),
    .o_hs     (ccm_hs      ),
    .o_de     (ccm_de      ),
    .o_valid  (ccm_valid   ),
    .o_datax2 (ccm_datax2  )
  );

//***************************************************************************
// RGB 域饱和度 (色度增益) + 暗部色度门控
//   详见 rtl/debayer/rgb_saturate.v 文件头。要点:
//     - 病因 (红不够红): 白平衡正常 (灰/白物体是中性色), 但红色"不够红、发灰发橙"
//       —— 这不是增益问题, 是**色度幅度不足** (主要是双线性 demosaic 的过度平滑)。
//     - 算法: Y = (77R+150G+29B+128)>>8, out_c = clamp(Y + round((c-Y)*S/2^14)),
//       只拉伸色度、保持 Y, 所以 *** 中性色恒不受影响 ***, 不会动已经调好的白平衡。
//     - 病因 (黑处红色噪点, 本轮新增的用法): gamma 底部斜率约 15 级/线性级,
//       暗部 R 与 G 各自的噪声在编码域是 ±15 级, 于是 R-G 抖动能到 ±20 —— 屏幕上
//       就是暗处的彩色颗粒。色度 ×S 等于把这份抖动也 ×S, 所以暗部必须让 S < 1。
//       处方: S 随亮度走 —— Y<=P_Y_LO 用 P_SAT_GAIN_LO, Y>=P_Y_HI 用 P_SAT_GAIN,
//       中间线性插值。标定实测 (rawdata.hex 整帧, 暗部 R-G 高频分量 std):
//         无门控 3.48  ->  S_LO=0.50/Y_LO=32/Y_HI=80 得到 1.61 (暗部色度噪声减半)
//       而 Y >= P_Y_HI 的像素被改动的个数是 **0** —— 门控完全不碰中间调,
//       这是 tb_rgb_saturate.v 检查 4 断言的机器可验红线。
//
//     - 位置在 gamma 之后。*** 注意理由不是"避免暗部被放大" *** —— 上一版是这么
//       写的, 那是错的 (gamma 前 d_lin*S*slope 与 gamma 后 d_lin*slope*S 恒等,
//       暗部放大倍数就是同一个 S)。真正的理由: (1) 与 rgb_sharpen 共用同一个
//       编码域 Y 定义; (2) 编码域近似感知均匀, 保 Y 拉伸对应视觉亮度不变。
//       暗部那个真问题由上面的亮度门控解决, 不是靠挪位置。
//     - *** 1 拍流水 (2026-10-06 改) ***。上一版是零延迟组合逻辑, 上板时序翻车:
//       综合报 10 条失败路径, 最差 slack -5.881 ns。实测根因是这个模块的组合深度
//       —— 数据路径 12.038 ns 里它占 8.26 ns (48 级逻辑), 而这里的下游
//       hdmi_tx_* 那一块用 148.5MHz 的 hdmi_tx_slow_clk 采 rgb_datax2,
//       所以整条 Y->门控->delta->adj->钳位 链都得塞进 6.734 ns。
//       资源从来不是问题 (Memory Block 149/256、XLR 37%、DSP 22/160)。
//       现在把寄存器切在 sat_delta 与 adj_ch 之间, 输入侧回到本模块自己的
//       13.468 ns 周期里, 输出侧只剩加/钳位。详见 rgb_saturate.v 第六节。
//       *** 四个标志与数据同步延迟 1 拍 ***, 相对错拍不变 (红线说的正是"相对"),
//       而 rgb_vs/hs/de/valid/datax2 的唯一消费者就是下面那个 hdmi_tx_* 块,
//       同源同拍, 所以画面只是晚 1 个 vid_clk_dvi2 周期出现, 相对关系一个没变。
//     - 16384 = 1.0 = 逐位直通 (恒等式, 已穷举 2^24 验证)。太灰就往上调 P_SAT_GAIN,
//       高饱和红色发闷就往下调; 暗部还有彩色颗粒就把 P_SAT_GAIN_LO 再调小。
//       P_BYPASS=1 可整体摘掉这一级 (走同一级寄存器, 拍数不变)。
//     - 参数, 不是 VIO: 现有 VIO 核的 4 个输出 (r/g/b_gain + gamma_sel) 已全部占用,
//       加探针必须在 Efinity 的 Debugger wizard 里重新生成核, 改 RTL 改不出来。
//
// *** 2026-10-06 排障记录: 这一级确实把画面搞坏过, 病因是时序不是算法 ***
//   现象: 用户上板反馈"添加了饱和处理之后, 画面完全变成了彩色噪声"; 把本模块
//         P_BYPASS 置 1 之后"现在没有噪点了" —— 至此确认坏在**本模块算出来的
//         那一路数据**上。
//
//   >>> 决定性推论: 旁路走的**不是**另一条路, 是同一级寄存器的另一个抽头 <<<
//   末尾那行是   o_datax2 = P_BYPASS ? r_dat : {w_pix_o, w_pix_e};
//   r_dat 和 w_pix_e/w_pix_o 都是**同一个 always 块、同一拍**打出来的寄存器。
//   所以旁路与直通的**延迟严格相同** —— 换成旁路能变的东西只有数据的值, 不可能是
//   相位、也不可能是拍数。这就把"下游 vid_cnt 偶/奇像素选错相位"那一类猜测
//   (ti60f225_oob_top.v 里 148.5MHz 采 rgb_datax2 的那块) 整个排除掉了。
//
//   已排除的:
//     - 不是数值削波。按 wb_gain_3ch.v:104 的 Q2.14 与 gamma_lut_v1.v:8 的表把
//       rawdata.hex 整帧跑一遍: CFA 采样点被 WB 顶到 511 的只有 0.14%, 至少一个
//       平面到 255 的像素 0.33%。把 G 增益从 1.509 降到 1.0 也只把这 0.217%
//       降到 0 —— 削波与画面好坏无关。
//     - 不是资源: Memory Block 149/256、XLR 37%、DSP 22/160。
//     - 不是算法/字节序。逐位核过: sat_front 的 44 位打包与 sat_back 的解包对齐,
//       adj_ch 的借位判溢正确, {B,G,R} 且 R 在最低字节与顶层
//       {hdmi_tx_bdata,hdmi_tx_gdata,hdmi_tx_rdata} <= rgb_datax2[23:0] 一致。
//       拿文件头第六节那个 R=230/G=120/B=100 的例子手算: Y=151, S=1.30,
//       out = 254/111/85, 与算法定义相符。
//     *** 注意 check_rgb_saturate.py 是照抄 RTL 写的, 两者一起错时对拍全绿 ——
//         它证明不了算术对, 只能证明两处写法一致。所以上面那条是**手算**结论,
//         不是对拍结论。这条坑见 isp_matlab 的既有教训。***
//
//   算法对 + 数据错 + 延迟无关 => 只剩一个解释: **计算路径没收敛, r_f_e/r_f_o
//   被采到了错的值。** 这条路径的预算只有本模块自己一个 13.468 ns 周期
//   (vid_clk_dvi2), 因为下游 hdmi_tx_* 那块是 148.5MHz 采样, 更紧。
//   *** 注意: 从改 1 拍流水那次到现在, outflow/ 下**从来没生成过** place/route
//   报告, 用户几次跑的都是只有 Synthesis。所以"修复后时序到底过没过"这件事
//   我一次都没有数据, 全靠推算。 ***
//
//   >>> 本轮的处置: 把这条路径上最重的一块删掉 <<<
//   sat_gain_at 原来写的是   sat_gain_at = P_SAT_GAIN_LO + num / (P_Y_HI-P_Y_LO);
//   —— 那是在**赌**综合器认出除数是常数、把它优化成移位。赌输了就是这条路径上
//   最重的组合块。现在改成写死的 >> P_RAMP_SHIFT (见 rgb_saturate.v 的
//   P_RAMP_SHIFT / P_RAMP_HALF), 综合器没有别的选择, 只能是移位。
//   这是**删减**不是新增: 除法的行为与移位逐位相同 (P_Y_HI-P_Y_LO = 64 = 2^6),
//   只是不再依赖综合器的优化能力。
//
//   >>> 如果彩色噪声又回来了 <<<
//   那就说明瓶颈不在那个除法, 而是路径整体还是太长。下一步**不要**再猜:
//   请跑**完整流程 (Place & Route / Bitstream), 不要只跑 Synthesis** —— 只有完整
//   流程才会生成 outflow/ti60f225_oob.timing.rpt, 那份报告里能直接看到是哪条
//   路径、多少级逻辑、差多少 ns。拿到它之前任何 RTL 改动都是盲改。
//
//   补充 (与本次故障无关, 但下次重新标定时要记住): 按 rawdata.hex 整帧统计
//   gamma 后的 Y (每 2x2 一个采样, 共 518400 点): 均值 78.9 / p50 67 / p99 170,
//   其中 Y<=32 只占 2.62%、32<Y<96 占 77.04%、Y>=96 占 20.33%。也就是门控真正
//   卖力的区域只有 2.6% 的像素, 而 77% 的画面落在斜坡上 (Y=48->S=0.70,
//   Y=64->0.90, Y=80->1.10)。这帧 raw p50 只有 13/11/5 (BLC 后中位数贴到 0),
//   非常暗 —— 想重新标 P_Y_LO/P_Y_HI 的话, 请按上面这组**实测分布**来, 不要再
//   拍脑袋定"暗部阈值"。
//***************************************************************************
  wire sat_bypass;
rgb_saturate # (
    .P_SAT_GAIN     (16'd21299  ),   // Q2.14, 1.30  中间调及以上的色度增益
    .P_SAT_GAIN_LO  (16'd8192   ),   // Q2.14, 0.50  暗部 (Y<=P_Y_LO) 的色度增益
    .P_Y_LO         (8'd32      ),   // 暗部门控下限
    .P_Y_HI         (8'd96      ),   // 暗部门控上限; 与 P_Y_LO 之差 = 64 = 2^6
    .P_RED_SHIFT    (0          )//,
    // 0 = 打开饱和度; 上板若再看到彩色噪声, 把它改成 1'b1 就能整级摘掉
    // (旁路走同一级寄存器, 所以摘掉不改延迟, 只换数据来源)
    // .P_BYPASS       (1'b0       )
  )
  rgb_saturate_inst (
    .i_pclk   (vid_clk_dvi2),
    .i_arstn  (vid_rst_n   ),
    .P_BYPASS (sat_bypass  ),
    .i_vs     (ccm_vs      ),
    .i_hs     (ccm_hs      ),
    .i_de     (ccm_de      ),
    .i_valid  (ccm_valid   ),
    .i_datax2 (ccm_datax2  ),
    .o_vs     (rgb_vs      ),
    .o_hs     (rgb_hs      ),
    .o_de     (rgb_de      ),
    .o_valid  (rgb_valid   ),
    .o_datax2 (rgb_datax2  )
  );


// debayer_top_2to1 debayer_top
// (
// 	.in_pclk		  (vid_clk_dvi2),//(i_mipi_rx_pclk ),
// 	.in_rstn		  (vid_rst_n	),
	
reg rgb_vs_r = 'd0;
always @( posedge hdmi_tx_slow_clk )
begin
  rgb_vs_r <= rgb_vs;
end
assign pos_vs = {rgb_vs_r,rgb_vs} == 2'b01;
reg vid_cnt = 'd0;
reg [7:0] hdmi_tx_rdata ;
reg [7:0] hdmi_tx_gdata ;
reg [7:0] hdmi_tx_bdata ;
reg hdmi_tx_vs;
reg hdmi_tx_hs;
reg hdmi_tx_de;
always @( posedge hdmi_tx_slow_clk )
begin
    if( pos_vs )
      vid_cnt <= 1'b1;
    else if( rgb_de )
      vid_cnt <= ~vid_cnt ;

    if( vid_cnt )
      {hdmi_tx_bdata,hdmi_tx_gdata,hdmi_tx_rdata} <= rgb_datax2[23:0];
    else 
      {hdmi_tx_bdata,hdmi_tx_gdata,hdmi_tx_rdata} <= rgb_datax2[47:24];

  hdmi_tx_vs <= rgb_vs;
  hdmi_tx_hs <= rgb_hs;
  hdmi_tx_de <= rgb_de;
end
//==============================================================================
// MIPI DSI
//==============================================================================
// signal 

reg		[25:0]	r_rst_cnt;

wire	[31:0]	w_axi_rdata;
wire			w_axi_awready;
wire			w_axi_wready;
wire			w_axi_arready;
wire			w_axi_rvalid;
wire			w_axi_bvalid;

wire	[6:0]	w_axi_awaddr;
wire			w_axi_awvalid;
wire	[31:0]	w_axi_wdata;
wire			w_axi_wvalid;
wire			w_axi_bready;
wire	[6:0]	w_axi_araddr;
wire			w_axi_arvalid;
wire			w_axi_rready;

wire			w_confdone;
assign w_confdone = 1'b1;

assign  LCD_POWER 			= i_arstn;
assign	LCD_RST_P	      = ~w_arstn;//~r_rst_cnt[23]; LCD复位先释放，然后再释放MIPI 的复位
assign	mipi_dp_clk_RST		= ~i_arstn;
assign	mipi_dp_data0_RST	= ~i_arstn;
assign	mipi_dp_data1_RST	= ~i_arstn;
assign	mipi_dp_data2_RST	= ~i_arstn;
assign	mipi_dp_data3_RST	= ~i_arstn;
////////////////////////////////////////////////////////////////
always@(posedge i_mipi_clk or  negedge w_arstn )
begin
  if ( !w_arstn ) begin
    r_rst_cnt	<= 'd0;
  end else 		
    r_rst_cnt	<= r_rst_cnt[25] ? r_rst_cnt :r_rst_cnt + 1'b1;
end

wire axi_rst_n = r_rst_cnt[25];

reg [26:0] dly_cnt = 'd0;

always @( posedge vid_clk_dvi2 or negedge w_arstn )
begin
  if( !w_arstn )
    dly_cnt <= 'd0;
  else if( w_confdone )
    dly_cnt <= dly_cnt[26] ? dly_cnt :(dly_cnt + 1'b1);
  else 
    dly_cnt <= 'd0;
end 

assign vid_rst_n = dly_cnt[26];




wire hs;
wire vs;
wire de;
wire [7:0] rgb_r;
wire [7:0] rgb_g;
wire [7:0] rgb_b;
  
  
  // );

//=================================================================================
//hdmi tx ctrl 
//=================================================================================
wire                            video_hs;
wire                            video_vs;
wire                            video_de;
wire[7:0]                       video_r;
wire[7:0]                       video_g;
wire[7:0]                       video_b;

assign tmds_tx_data0_TX_OE = 1'b1;
assign tmds_tx_data1_TX_OE = 1'b1;
assign tmds_tx_data2_TX_OE = 1'b1;
assign tmds_tx_clk_TX_OE   = 1'b1;

assign tmds_tx_data0_TX_RST = 1'b0;
assign tmds_tx_data1_TX_RST = 1'b0;
assign tmds_tx_data2_TX_RST = 1'b0;
assign tmds_tx_clk_TX_RST   = 1'b0;


wire [9:0] tmds_data0;
wire [9:0] tmds_data1;
wire [9:0] tmds_data2;
wire [9:0] tmds_clk ;
wire [7:0] gamma_rdata ;
wire [7:0] gamma_gdata ;
wire [7:0] gamma_bdata ;
wire gamma_valid;
wire gamma_hs;
wire gamma_vs;
wire gamma_sel;

wire [7:0] con_rdata ;
wire [7:0] con_gdata ;
wire [7:0] con_bdata ;
wire con_de;
wire con_hs;
wire con_vs;
wire con_sel;

wire [7:0] BRIGHT_SIG    ;
wire CONTRAST_SIG_G;
wire CONTRAST_SIG_R;
wire [7:0] CONTRAST_G    ;
wire CONTRAST_SIG_B;
wire [7:0] BRIGHT        ;
wire [7:0] CONTRAST_R    ;


wire CONTRAST_B    ;

// Diagnostic-only screen indicators, left to right: core reset released,
// DDR calibrated, UDP payload observed, complete frame displayed, AI ready.
// Red means not yet observed; green means observed/ready.
reg [11:0] diag_x,diag_y;reg diag_de_d,diag_vs_d;
reg udp_seen;
(* async_reg="true" *) reg [3:0] diag_meta,diag_sync;
wire [7:0] diag_cpu_read;
wire [7:0] diag_network;
reg diag_adapter_overrun,diag_adapter_invalid;
(* async_reg="true" *) reg [9:0] diag_cpu_meta,diag_cpu_sync;
always @(posedge rxc or negedge w_arstn)
 if(!w_arstn)udp_seen<=0;else if(net_valid&&net_ready&&net_sop)udp_seen<=1;
always @(posedge rxc or negedge w_arstn)
 if(!w_arstn)begin diag_adapter_overrun<=0;diag_adapter_invalid<=0;end
 else begin if(net_overrun)diag_adapter_overrun<=1;if(net_invalid)diag_adapter_invalid<=1;end
always @(posedge hdmi_tx_slow_clk or negedge vid_rst_n) begin
 if(!vid_rst_n)begin diag_x<=0;diag_y<=0;diag_de_d<=0;diag_vs_d<=0;diag_meta<=0;diag_sync<=0;diag_cpu_meta<=0;diag_cpu_sync<=0;end
 else begin
  diag_meta<={ai_ready,udp_seen,cal_done,axi_rst_n};diag_sync<=diag_meta;
  diag_cpu_meta<={diag_adapter_invalid,diag_adapter_overrun,diag_network};diag_cpu_sync<=diag_cpu_meta;
  diag_de_d<=ai_de;diag_vs_d<=ai_vs;
  if(ai_vs&&!diag_vs_d)diag_y<=0;
  else if(!ai_de&&diag_de_d)diag_y<=diag_y+1'b1;
  if(!ai_de)diag_x<=0;else diag_x<=diag_x+1'b1;
 end
end
wire diag_area=ai_de&&diag_y>=20&&diag_y<52&&((diag_x>=400&&diag_x<428)||(diag_x>=436&&diag_x<464)||(diag_x>=472&&diag_x<500)||(diag_x>=508&&diag_x<536)||(diag_x>=544&&diag_x<572));
wire diag_value=diag_x<432?diag_sync[0]:diag_x<468?diag_sync[1]:diag_x<504?diag_sync[2]:diag_x<540?net_display_valid:diag_sync[3];
wire diag_cpu_area=ai_de&&diag_y>=60&&diag_y<88&&((diag_x>=400&&diag_x<428)||(diag_x>=436&&diag_x<464)||(diag_x>=472&&diag_x<500)||(diag_x>=508&&diag_x<536)||(diag_x>=544&&diag_x<572)||(diag_x>=580&&diag_x<608)||(diag_x>=616&&diag_x<644)||(diag_x>=652&&diag_x<680)||(diag_x>=688&&diag_x<716)||(diag_x>=724&&diag_x<752));
wire [3:0] diag_cpu_index=diag_x<432?0:diag_x<468?1:diag_x<504?2:diag_x<540?3:diag_x<576?4:diag_x<612?5:diag_x<648?6:diag_x<684?7:diag_x<720?8:9;
// Diagnostic tiles hidden in the presentation output; counters remain available.
wire [23:0] diag_rgb=ai_rgb;



dvi_encoder dvi_encoder_m0
(
	.pixelclk      		(hdmi_tx_slow_clk          ),// system clock
	.rst_p         		(~vid_rst_n       ),// reset
	.i_bdata               (diag_rgb[7:0]),
	.i_gdata               (diag_rgb[15:8]),
	.i_rdata               (diag_rgb[23:16]),
  .i_de            	(ai_de       ),//(hdmi_tx_de    ),//(video_de           ),//(rx_de		  ),   //
	.i_hs         		(ai_hs       ),//(video_hs           ),//(rx_hsync	),   //   
	.i_vs         		(ai_vs       ),//(video_vs           ),//(rx_vsync	),   //  
	
  .video_format     (2'b00), //// 00 = RGB, 01 = YCbCr 4:2:2, 10 = YCbCr 4:4:4
  .video_VIC        (0),
	.audio_L			    (audio_L),
  .audio_R			    (audio_R),
  .audio_valid		  (audio_valid),
  .audio_N                (6144),     //(20'h01880),//     
  .audio_CTS              (148500),      //(20'h28488),//      
  .audio_sample_frequency (audio_samp_freq),    //(3'b000),// 
  .audio_word_length      ({audio_samp_word_len,audio_max_word_length}),//(4'b1011),//

	.tmds_data0    		(tmds_data0),
  .tmds_data1    		(tmds_data1),
  .tmds_data2    		(tmds_data2),
  .tmds_clk      	  (tmds_clk  )
);
assign tmds_tx_clk_TX_DATA   = ~tmds_clk;
assign tmds_tx_data0_TX_DATA = ~tmds_data0;
assign tmds_tx_data1_TX_DATA = ~tmds_data1;
assign tmds_tx_data2_TX_DATA = ~tmds_data2;

/* FATIGUE-SC431-video AXI -> half duplex -> shared DDR. */
wire v_av,v_ar,v_wr,v_lock,v_wv,v_wrdy,v_wlast,v_bv,v_br,v_rv,v_rr,v_rlast;
wire [31:0] v_addr; wire [7:0] v_len; wire [2:0] v_size; wire [1:0] v_burst,v_lock2,v_bresp,v_rresp;
wire [3:0] v_id,v_wid,v_bid,v_rid; wire [127:0] v_wdata,v_rdata; wire [15:0] v_wstrb;

wire [1:0] ai_quiescent;wire ai_reset,ai_ready,ai_sys_reset,ai_mem_reset,ai_per_reset;
wire [23:0] ai_rgb;wire ai_hs,ai_vs,ai_de;wire [7:0] ai_video_bid,ai_video_rid;
assign v_bid=ai_video_bid[3:0];assign v_rid=ai_video_rid[3:0];
wire [3:0] ai_mem_unused_bid,ai_mem_unused_rid;wire [1:0] ai_mem_lock2;
assign ai_mem_lock2={1'b0,ai_mem_lock};
wire  ai_mem_avalid;
/* old DDR raw AREADY was returned while AVALID was masked.
   Both sides must observe the same handshake before calibration completes. */
wire ddr_raw_aready;
wire ai_mem_aready = ddr_raw_aready && cal_done;
wire [31:0] ai_mem_addr;
wire [3:0] ai_mem_id;
wire [7:0] ai_mem_len;
wire [2:0] ai_mem_size;
wire [1:0] ai_mem_burst;
wire  ai_mem_write;
wire  ai_mem_lock;
wire  ai_mem_wvalid;
wire  ai_mem_wready;
wire [127:0] ai_mem_wdata;
wire [15:0] ai_mem_wstrb;
wire  ai_mem_wlast;
wire [3:0] ai_mem_wid;
wire  ai_mem_bvalid;
wire  ai_mem_bready;
wire [1:0] ai_mem_bresp;
wire  ai_mem_rvalid;
wire  ai_mem_rready;
wire [127:0] ai_mem_rdata;
wire [1:0] ai_mem_rresp;
wire  ai_mem_rlast;

/* Temporary fabric clocks for the inactive legacy camera chain in this variant. */
reg [3:0] rx_clk_div=0;reg CLK_5M=0;reg i_mipi_rx_pclk=0;
always @(posedge core_clk) begin
 i_mipi_rx_pclk<=~i_mipi_rx_pclk;
 if(rx_clk_div==9)begin rx_clk_div<=0;CLK_5M<=~CLK_5M;end else rx_clk_div<=rx_clk_div+1'b1;
end
// Network-only experiment: input over RGMII, output via HDMI TX.
assign ge_rx_pll_RSTN=1'b1;
wire net_valid,net_ready,net_sop,net_eop;wire [7:0] net_data;wire net_overrun,net_invalid;
wire net_recovering;
// Recover the MAC/packet RAM only. Do not reset the network AXI master while
// a DDR transaction may be outstanding; the frame parser resynchronizes on SOP.
fatigue_net_recovery u_net_recovery(.clk(rxc),.rst_n(w_arstn),.overrun(net_overrun),.recovering(net_recovering));
wire [23:0] net_rgb;wire net_hs,net_vs,net_de,net_display_valid;wire [31:0] net_frame_id;
wire net_write_error,net_read_error,net_underrun,net_overflow;
reg [31:0] count_last_id;
reg [7:0] display_frame_count;
reg count_last_valid;
always @(posedge hdmi_tx_slow_clk or negedge vid_rst_n) begin
 if(!vid_rst_n)begin count_last_id<=0;display_frame_count<=0;count_last_valid<=0;end
 else begin
  if(net_display_valid && (!count_last_valid || net_frame_id!=count_last_id))begin
   display_frame_count<=display_frame_count+1'b1;count_last_id<=net_frame_id;count_last_valid<=1;
  end else if(!net_display_valid) count_last_valid<=0;
 end
end
fatigue_ethernet_rx u_net_rx(.rst_n(ge_0_gclk_pll_LOCKED && w_arstn && !net_recovering),.rxc(rxc),.rxd_hi_i(rxd_hi_i),.rxd_lo_i(rxd_lo_i),.rx_dv_HI(rx_dv_HI),.rx_dv_LO(rx_dv_LO),
 .phy_rst_n(phy_rst_n),.mdc_o(mdc_o),.mdio_o(mdio_o),.mdio_oe(mdio_oe),.mdio_i(mdio_i),.tx_en_o_HI(tx_en_o_HI),.tx_en_o_LO(tx_en_o_LO),.txc_hi_o(txc_hi_o),.txc_lo_o(txc_lo_o),.txd_hi_o(txd_hi_o),.txd_lo_o(txd_lo_o),
 .stream_valid(net_valid),.stream_ready(net_ready),.stream_sop(net_sop),.stream_eop(net_eop),.stream_data(net_data),.overrun(net_overrun),.invalid(net_invalid));
// Common asynchronous assertion; deassert separately in each network domain.
wire net_global_rst_n=axi_rst_n && vid_rst_n;
(* async_reg="true" *) reg [1:0] net_rx_reset_sync,net_ddr_reset_sync,net_pix_reset_sync;
always @(posedge rxc or negedge net_global_rst_n)if(!net_global_rst_n)net_rx_reset_sync<=0;else net_rx_reset_sync<={net_rx_reset_sync[0],1'b1};
always @(posedge core_clk or negedge net_global_rst_n)if(!net_global_rst_n)net_ddr_reset_sync<=0;else net_ddr_reset_sync<={net_ddr_reset_sync[0],1'b1};
always @(posedge hdmi_tx_slow_clk or negedge net_global_rst_n)if(!net_global_rst_n)net_pix_reset_sync<=0;else net_pix_reset_sync<={net_pix_reset_sync[0],1'b1};
// Experimental lower-bandwidth stream; 320x180 is scaled to 1920x1080.
fatigue_network_video #(.WIDTH(320),.HEIGHT(180),.SCALE(6)) u_network_video(.rx_clk(rxc),.ddr_clk(core_clk),.pix_clk(hdmi_tx_slow_clk),.rx_rst_n(net_rx_reset_sync[1]),.ddr_rst_n(net_ddr_reset_sync[1]),.pix_rst_n(net_pix_reset_sync[1]),
 .in_valid(net_valid),.in_sop(net_sop),.in_eop(net_eop),.in_data(net_data),.in_ready(net_ready),
 .rgb(net_rgb),.hs(net_hs),.vs(net_vs),.de(net_de),.display_valid(net_display_valid),.display_id(net_frame_id),.write_error(net_write_error),.read_error(net_read_error),.underrun(net_underrun),.frame_overflow(net_overflow),
 .diag_network(diag_network),
 .m_avalid(v_av),.m_aready(v_ar),.m_addr(v_addr),.m_id(v_id),.m_len(v_len),.m_size(v_size),.m_burst(v_burst),.m_write(v_wr),.m_lock(v_lock),
 .m_wvalid(v_wv),.m_wready(v_wrdy),.m_wdata(v_wdata),.m_wstrb(v_wstrb),.m_wlast(v_wlast),.m_wid(v_wid),
 .m_bvalid(v_bv),.m_bready(v_br),.m_bresp(v_bresp),.m_rvalid(v_rv),.m_rready(v_rr),.m_rdata(v_rdata),.m_rresp(v_rresp),.m_rlast(v_rlast));

cnn_ai_reset u_fatigue_reset(.clk(core_clk),.rst_n(axi_rst_n),
 .memory_ready(cal_done&&cal_done),.reset_request(1'b0),
 .soc_reset(ai_sys_reset|ai_mem_reset|ai_per_reset),.quiescent(ai_quiescent),.ai_reset(ai_reset),.hardware_ready(ai_ready));
cnn_evsoc_memory u_fatigue_system(
 .clk_96(core_clk),
 .ai_reset(ai_reset),
 .ai_online(ai_ready),
 .uart_clk(core_clk),
 .uart_rst_n(axi_rst_n),
 .requested(2'b11),
 .applied(),
 .available(),
 .inference_enable(),
 .system_reset(ai_sys_reset),
 .memory_reset(ai_mem_reset),
 .peripheral_reset(ai_per_reset),
 .pixel_clk(hdmi_tx_slow_clk),
 .pixel_rst_n(vid_rst_n),
 .source_frame_id(net_frame_id),
 .display_frame_count(display_frame_count),
 .cam_clk(hdmi_tx_slow_clk),
 .cam_rst_n(vid_rst_n),
 .cam_frame_valid(1'b0),
 .cam_pixel_valid(1'b0),
 .cam_gray(8'b0),
 .rgb_i(net_rgb),
 .hs_i(net_hs),
 .vs_i(net_vs),
 .de_i(net_de),
 .rgb_o(ai_rgb),
 .hs_o(ai_hs),
 .vs_o(ai_vs),
 .de_o(ai_de),
 .displayed_source_frame(),
 .jtagCtrl_enable(jtag_inst1_SEL),
 .jtagCtrl_tdi(jtag_inst1_TDI),
 .jtagCtrl_capture(jtag_inst1_CAPTURE),
 .jtagCtrl_shift(jtag_inst1_SHIFT),
 .jtagCtrl_update(jtag_inst1_UPDATE),
 .jtagCtrl_reset(jtag_inst1_RESET),
 .jtagCtrl_tdo(jtag_inst1_TDO),
 .jtagCtrl_tck(jtag_inst1_TCK),
 .system_spi_0_io_data_0_read(1'b1),
 .system_spi_0_io_data_0_write(),
 .system_spi_0_io_data_0_writeEnable(),
 .system_spi_0_io_data_1_read(1'b1),
 .system_spi_0_io_data_1_write(),
 .system_spi_0_io_data_1_writeEnable(),
 .system_spi_0_io_data_2_read(1'b1),
 .system_spi_0_io_data_2_write(),
 .system_spi_0_io_data_2_writeEnable(),
 .system_spi_0_io_data_3_read(1'b1),
 .system_spi_0_io_data_3_write(),
 .system_spi_0_io_data_3_writeEnable(),
 .system_spi_0_io_sclk_write(),
 .system_uart_0_io_txd(fatigue_uart_tx),
 .system_uart_0_io_rxd(fatigue_uart_rx),
 .system_spi_0_io_ss(),
 .fabric_rst_n(axi_rst_n),
 .ai_quiescent(ai_quiescent),
 .diag_cpu_read(diag_cpu_read),
 .video_avalid(v_av),
 .video_aready(v_ar),
 .video_addr(v_addr),
 .video_id({4'b0,v_id}),
 .video_len(v_len),
 .video_size(v_size),
 .video_burst(v_burst),
 .video_write(v_wr),
 .video_lock(|v_lock),
 .video_wvalid(v_wv),
 .video_wready(v_wrdy),
 .video_wdata(v_wdata),
 .video_wstrb(v_wstrb),
 .video_wlast(v_wlast),
 .video_bvalid(v_bv),
 .video_bready(v_br),
 .video_bid(ai_video_bid),
 .video_bresp(v_bresp),
 .video_rvalid(v_rv),
 .video_rready(v_rr),
 .video_rdata(v_rdata),
 .video_rid(ai_video_rid),
 .video_rresp(v_rresp),
 .video_rlast(v_rlast),
 .m_avalid(ai_mem_avalid),
 .m_aready(ai_mem_aready),
 .m_addr(ai_mem_addr),
 .m_id(ai_mem_id),
 .m_len(ai_mem_len),
 .m_size(ai_mem_size),
 .m_burst(ai_mem_burst),
 .m_write(ai_mem_write),
 .m_lock(ai_mem_lock),
 .m_wvalid(ai_mem_wvalid),
 .m_wready(ai_mem_wready),
 .m_wdata(ai_mem_wdata),
 .m_wstrb(ai_mem_wstrb),
 .m_wlast(ai_mem_wlast),
 .m_wid(ai_mem_wid),
 .m_bvalid(ai_mem_bvalid),
 .m_bready(ai_mem_bready),
 .m_bresp(ai_mem_bresp),
 .m_rvalid(ai_mem_rvalid),
 .m_rready(ai_mem_rready),
 .m_rdata(ai_mem_rdata),
 .m_rresp(ai_mem_rresp),
 .m_rlast(ai_mem_rlast),
 .idle_o()
);

cnn_half_to_full_axi u_ddr_adapter(.enable(cal_done),
.h_avalid(ai_mem_avalid),
.h_aready(ddr_raw_aready),
.h_addr(ai_mem_addr),
.h_id(ai_mem_id),
.h_len(ai_mem_len),
.h_size(ai_mem_size),
.h_burst(ai_mem_burst),
.h_write(ai_mem_write),
.h_lock(ai_mem_lock),
.h_wvalid(ai_mem_wvalid),
.h_wready(ai_mem_wready),
.h_wdata(ai_mem_wdata),
.h_wstrb(ai_mem_wstrb),
.h_wlast(ai_mem_wlast),
.h_bvalid(ai_mem_bvalid),
.h_bready(ai_mem_bready),
.h_bresp(ai_mem_bresp),
.h_rvalid(ai_mem_rvalid),
.h_rready(ai_mem_rready),
.h_rdata(ai_mem_rdata),
.h_rresp(ai_mem_rresp),
.h_rlast(ai_mem_rlast),
.awvalid(d_axi_awvalid),
.awready(d_axi_awready),
.awaddr(d_axi_awaddr),
.awid(d_axi_awid),
.awlen(d_axi_awlen),
.awsize(d_axi_awsize),
.awburst(d_axi_awburst),
.awlock(d_axi_awlock),
.arvalid(d_axi_arvalid),
.arready(d_axi_arready),
.araddr(d_axi_araddr),
.arid(d_axi_arid),
.arlen(d_axi_arlen),
.arsize(d_axi_arsize),
.arburst(d_axi_arburst),
.arlock(d_axi_arlock),
.wvalid(d_axi_wvalid),
.wready(d_axi_wready),
.wdata(d_axi_wdata),
.wstrb(d_axi_wstrb),
.wlast(d_axi_wlast),
.bvalid(d_axi_bvalid),
.bready(d_axi_bready),
.bresp(d_axi_bresp),
.rvalid(d_axi_rvalid),
.rready(d_axi_rready),
.rdata(d_axi_rdata),
.rresp(d_axi_rresp),
.rlast(d_axi_rlast));
assign d_axi_awcache=4'b0; assign d_axi_awprot=3'b0; assign d_axi_arcache=4'b0; assign d_axi_arprot=3'b0;

debug_edb_top debug_edb_top_inst (
    .bscan_CAPTURE      ( jtag_inst1_CAPTURE  ),
    .bscan_DRCK         ( jtag_inst1_DRCK     ),
    .bscan_RESET        ( jtag_inst1_RESET    ),
    .bscan_RUNTEST      ( jtag_inst1_RUNTEST  ),
    .bscan_SEL          ( 1'b0                ),
    .bscan_SHIFT        ( jtag_inst1_SHIFT    ),
    .bscan_TCK          ( jtag_inst1_TCK      ),
    .bscan_TDI          ( jtag_inst1_TDI      ),
    .bscan_TMS          ( jtag_inst1_TMS      ),
    .bscan_UPDATE       ( jtag_inst1_UPDATE   ),
    .bscan_TDO          (                    ),
    .vio0_clk           ( vid_clk_dvi2        ),
    .vio0_r_gain        ( r_gain              ),
    .vio0_g_gain        ( g_gain              ),
    .vio0_b_gain        ( b_gain              ),

    .vio0_ccm_bypass    (ccm_bypass ),
    .vio0_sat_bypass    (sat_bypass    ),
    .vio0_sharpen_bypass(sharpen_bypass),
    .vio0_nr_bypass     (nr_bypass),
    .la0_clk            ( vid_clk_dvi2        ),
    .la0_ch0_g          ( rawdata_2pix               ),
    .la0_ch0_b          ( ch0_b               ),
    .la0_ch0_de         ( ch0_de              ),
    .la0_cal_done       ( cal_done            )
);

endmodule
