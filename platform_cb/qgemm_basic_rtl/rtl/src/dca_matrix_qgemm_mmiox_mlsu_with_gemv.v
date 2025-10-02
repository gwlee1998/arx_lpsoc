`timescale 1ns/1ps
// ============================================================================
// DCA_MATRIX_QDQ_MMIOX_MLSU
// [X, W matrix]
// - LOAD: MMIO → LOAD2MREG → MREG(TYPE3) → qdq controller
// - STORE: qdq controller → store → BRAM(512bit wide) → AXI
// [O matrix]
// - LOAD: AXI → BRAM(512bit wide) → loader(needed implement) → qdq controller
// - STORE: qdq controller → MREG(TYPE3) → MREG2STORE → MMIO
// ============================================================================

`include "ervp_global.vh"
`include "ervp_endian.vh"
`include "dca_module_memorymap_offset.vh"
`include "dca_matrix_qgemm_defines.vh"
`include "dca_matrix_info.vh"
`include "ervp_axi_define.vh"
`include "munoc_network_include.vh"

module DCA_MATRIX_QGEMM_MMIOX_MLSU
(
	clk,
	rstnn,
		
	control_rmx_core_config,
	control_rmx_core_status,
	control_rmx_clear_request,
	control_rmx_clear_finish,
	control_rmx_log_fifo_wready,
	control_rmx_log_fifo_wrequest,
	control_rmx_log_fifo_wdata,
	control_rmx_inst_fifo_rready,
	control_rmx_inst_fifo_rdata,
	control_rmx_inst_fifo_rrequest,
	control_rmx_operation_finish,
	control_rmx_input_fifo_rready,
	control_rmx_input_fifo_rdata,
	control_rmx_input_fifo_rrequest,
	control_rmx_output_fifo_wready,
	control_rmx_output_fifo_wrequest,
	control_rmx_output_fifo_wdata,

	mx_sinst_wvalid,
	mx_sinst_wdata,
	mx_sinst_wready,
	mx_sinst_decode_finish,
	mx_sinst_execute_finish,
	mx_sinst_busy,
	mx_sload_tensor_row_wvalid,
	mx_sload_tensor_row_wlast,
	mx_sload_tensor_row_wdata,
	mx_sload_tensor_row_wready,
	mx_sstore_tensor_row_rvalid,
	mx_sstore_tensor_row_rlast,
	mx_sstore_tensor_row_rready,
	mx_sstore_tensor_row_rdata,

	mw_sinst_wvalid,
	mw_sinst_wdata,
	mw_sinst_wready,
	mw_sinst_decode_finish,
	mw_sinst_execute_finish,
	mw_sinst_busy,
	mw_sload_tensor_row_wvalid,
	mw_sload_tensor_row_wlast,
	mw_sload_tensor_row_wdata,
	mw_sload_tensor_row_wready,
	mw_sstore_tensor_row_rvalid,
	mw_sstore_tensor_row_rlast,
	mw_sstore_tensor_row_rready,
	mw_sstore_tensor_row_rdata,

	mo_sinst_wvalid,
	mo_sinst_wdata,
	mo_sinst_wready,
	mo_sinst_decode_finish,
	mo_sinst_execute_finish,
	mo_sinst_busy,
	mo_sload_tensor_row_wvalid,
	mo_sload_tensor_row_wlast,
	mo_sload_tensor_row_wdata,
	mo_sload_tensor_row_wready,
	mo_sstore_tensor_row_rvalid,
	mo_sstore_tensor_row_rlast,
	mo_sstore_tensor_row_rready,
	mo_sstore_tensor_row_rdata,

	// mq2vta_rxawid,
	// mq2vta_rxawaddr,
	// mq2vta_rxawlen,
	// mq2vta_rxawsize,
	// mq2vta_rxawburst,
	// mq2vta_rxawvalid,	
	// mq2vta_rxawready,

	// mq2vta_rxwid,
	// mq2vta_rxwdata,
	// mq2vta_rxwstrb,
	// mq2vta_rxwlast,
	// mq2vta_rxwvalid,
	// mq2vta_rxwready,

	// mq2vta_rxbid,
	// mq2vta_rxbresp,
	// mq2vta_rxbvalid,
	// mq2vta_rxbready,

	// mq2vta_rxarid,
	// mq2vta_rxaraddr,
	// mq2vta_rxarlen,
	// mq2vta_rxarsize,
	// mq2vta_rxarburst,
	// mq2vta_rxarvalid,
	// mq2vta_rxarready,

	// mq2vta_rxrid,
	// mq2vta_rxrdata,
	// mq2vta_rxrresp,
	// mq2vta_rxrlast,
	// mq2vta_rxrvalid,
  	// mq2vta_rxrready
);

	parameter BIT_NUM          = 8;
	parameter FP_EXP_W         = 8;
	parameter FP_MANT_W        = 23;
	parameter FP_EXP_BIAS      = 127;
	parameter FP_DATA_W		= 32;
	parameter SCALE_FIFO_DEPTH = 4;
	parameter BW_AXI_DATA = 128;
	parameter BW_AXI_TID  = 4;
	parameter BW_ADDR = 32;
	parameter SIZE_OF_MEMORYMAP = 1;

	parameter INPUT_MATRIX_SIZE  = 16;
	parameter WEIGHT_MATRIX_SIZE = 16;
	parameter OUTPUT_MATRIX_SIZE = 16;
	parameter TENSOR_PARA = 0;
	parameter LSU_PARA = 0;
	parameter MATRIX_SIZE_PARA = 160016;

// dca_matrix_dim_lpara.vb
// dca_tensor_scalar_lpara.vb or BW_TENSOR_SCALAR
	`include "dca_matrix_lsu_inst.vh"
	// `include "ervp_axi_lpara.vb"
	`include "dca_lsu_util.vb"
	`include "dca_matrix_dim_util.vb"
	`include "dca_matrix_dim_lpara.vb"
	`include "dca_tensor_scalar_lpara.vb"

	localparam BW_CONFIG = 1;
	localparam BW_STATUS = `BW_DCA_MATRIX_QDQ_STATUS;
	localparam BW_LOG    = `BW_DCA_MATRIX_QDQ_LOG;
	localparam BW_INST   = `BW_DCA_MATRIX_QDQ_INST;
	localparam BW_INPUT  = 32;
	localparam BW_OUTPUT = 32;
	localparam VTA_DATA_WIDTH = 32;
	localparam SRAM_INDEX_WIDTH = 8;
// dca_matrix_lsu_inst.vh
// ervp_axi_lpara.vb
// dca_matrix_dim_lpara.vb
// dca_lsu_util.vb
// dca_matrix_dim_util.vb
	// ----------------------------------------------------------------------------
	// ----------------------------------------------------------------------------

	// localparam MATRIX_NUM_COL = `DCA_MATRIX_QDQ_NUM_COL;
	// localparam MATRIX_NUM_ROW = `DCA_MATRIX_QDQ_NUM_ROW;
	localparam INPUT_MATRIX_NUM_COL  = GET_MATRIX_NUM_COL(INPUT_MATRIX_SIZE);
	localparam WEIGHT_MATRIX_NUM_COL = GET_MATRIX_NUM_COL(WEIGHT_MATRIX_SIZE);
	localparam OUTPUT_MATRIX_NUM_COL = GET_MATRIX_NUM_COL(OUTPUT_MATRIX_SIZE);
	localparam SRAM_DATA_WIDTH = VTA_DATA_WIDTH * MATRIX_NUM_COL;

	localparam BW_INPUT_TENSOR_ROW   = BW_TENSOR_SCALAR*INPUT_MATRIX_NUM_COL;
	localparam BW_WEIGHT_TENSOR_ROW  = BW_TENSOR_SCALAR*WEIGHT_MATRIX_NUM_COL;
	localparam BW_OUTPUT_TENSOR_ROW  = BW_TENSOR_SCALAR*OUTPUT_MATRIX_NUM_COL;
	localparam LOG_MATRIX_NUM_ROW = $clog2(MATRIX_NUM_ROW);
	localparam BW_INPUT_TENSOR_MATRIX = BW_TENSOR_SCALAR*MATRIX_NUM_COL*MATRIX_NUM_ROW;
	localparam BW_WEIGHT_TENSOR_MATRIX = BW_TENSOR_SCALAR*MATRIX_NUM_COL*MATRIX_NUM_ROW;
	localparam BW_OUTPUT_TENSOR_MATRIX = BW_TENSOR_SCALAR*MATRIX_NUM_COL*MATRIX_NUM_ROW;

	localparam [`BW_DCA_MATRIX_LSU_INST_OPCODE-1:0] OPC_READ  = `DCA_MATRIX_LSU_INST_OPCODE_READ;
	localparam [`BW_DCA_MATRIX_LSU_INST_OPCODE-1:0] OPC_WRITE = `DCA_MATRIX_LSU_INST_OPCODE_WRITE;

	localparam MREG_RESET_VALUE = TENSOR_ZERO;
	
	`include "axi_sram128_params.vb"

	input  wire clk;
	input  wire rstnn;

	// ---------------- Control ties ----------------
	input  wire [(BW_CONFIG)-1:0] control_rmx_core_config;
	output wire [(BW_STATUS)-1:0] control_rmx_core_status;
	input  wire control_rmx_clear_request;
	output wire control_rmx_clear_finish;
	input  wire control_rmx_log_fifo_wready;
	output wire control_rmx_log_fifo_wrequest;
	output wire [(BW_LOG)-1:0] control_rmx_log_fifo_wdata;
	input  wire control_rmx_inst_fifo_rready;
	input  wire [(BW_INST)-1:0] control_rmx_inst_fifo_rdata;
	output wire control_rmx_inst_fifo_rrequest;
	output wire control_rmx_operation_finish;
	input  wire control_rmx_input_fifo_rready;
	input  wire [(BW_INPUT)-1:0] control_rmx_input_fifo_rdata;
	output wire control_rmx_input_fifo_rrequest;
	input  wire control_rmx_output_fifo_wready;
	output wire control_rmx_output_fifo_wrequest;
	output wire [(BW_OUTPUT)-1:0] control_rmx_output_fifo_wdata;

	output wire mx_sinst_wvalid;
	output wire [(`BW_DCA_MATRIX_LSU_INST)-1:0] mx_sinst_wdata;
	input  wire mx_sinst_wready;
	input  wire mx_sinst_decode_finish;
	input  wire mx_sinst_execute_finish;
	input  wire mx_sinst_busy;
	input  wire mx_sload_tensor_row_wvalid;
	input  wire mx_sload_tensor_row_wlast;
	input  wire [BW_INPUT_TENSOR_ROW-1:0] mx_sload_tensor_row_wdata;
	output wire mx_sload_tensor_row_wready;
	input  wire mx_sstore_tensor_row_rvalid;
	input  wire mx_sstore_tensor_row_rlast;
	output wire mx_sstore_tensor_row_rready;
	output wire [BW_INPUT_TENSOR_ROW-1:0] mx_sstore_tensor_row_rdata;

	output wire mw_sinst_wvalid;
	output wire [(`BW_DCA_MATRIX_LSU_INST)-1:0] mw_sinst_wdata;
	input  wire mw_sinst_wready;
	input  wire mw_sinst_decode_finish;
	input  wire mw_sinst_execute_finish;
	input  wire mw_sinst_busy;
	input  wire mw_sload_tensor_row_wvalid;
	input  wire mw_sload_tensor_row_wlast;
	input  wire [BW_WEIGHT_TENSOR_ROW-1:0] mw_sload_tensor_row_wdata;
	output wire mw_sload_tensor_row_wready;
	input  wire mw_sstore_tensor_row_rvalid;
	input  wire mw_sstore_tensor_row_rlast;
	output wire mw_sstore_tensor_row_rready;
	output wire [BW_WEIGHT_TENSOR_ROW-1:0] mw_sstore_tensor_row_rdata;

	output wire mo_sinst_wvalid;
	output wire [(`BW_DCA_MATRIX_LSU_INST)-1:0] mo_sinst_wdata;
	input  wire mo_sinst_wready;
	input  wire mo_sinst_decode_finish;
	input  wire mo_sinst_execute_finish;
	input  wire mo_sinst_busy;
	input  wire mo_sload_tensor_row_wvalid;
	input  wire mo_sload_tensor_row_wlast;
	input  wire [BW_OUTPUT_TENSOR_ROW-1:0] mo_sload_tensor_row_wdata;
	output wire mo_sload_tensor_row_wready;
	input  wire mo_sstore_tensor_row_rvalid;
	input  wire mo_sstore_tensor_row_rlast;
	output wire mo_sstore_tensor_row_rready;
	output wire [BW_OUTPUT_TENSOR_ROW-1:0] mo_sstore_tensor_row_rdata;

	// ---------------- AXI (tied-off) ----------------
	// AW Channel
	// input  wire [BW_AXI_TID-1:0]   mq2vta_rxawid;
	// input  wire [BW_ADDR-1:0]      mq2vta_rxawaddr;
	// input  wire [`BW_AXI_ALEN-1:0] mq2vta_rxawlen;
	// input  wire [`BW_AXI_ASIZE-1:0] mq2vta_rxawsize;
	// input  wire [`BW_AXI_ABURST-1:0] mq2vta_rxawburst;
	// input  wire                    mq2vta_rxawvalid;
	// output wire                    mq2vta_rxawready;

	// // W Channel
	// input  wire [BW_AXI_TID-1:0]   mq2vta_rxwid;
	// input  wire [BW_AXI_DATA-1:0]  mq2vta_rxwdata;
	// input  wire [`BW_AXI_WSTRB(BW_AXI_DATA)-1:0] mq2vta_rxwstrb;
	// input  wire                    mq2vta_rxwlast;
	// input  wire                    mq2vta_rxwvalid;
	// output wire                    mq2vta_rxwready;

	// // B Channel
	// output wire [BW_AXI_TID-1:0]   mq2vta_rxbid;
	// output wire [`BW_AXI_BRESP-1:0] mq2vta_rxbresp;
	// output wire                    mq2vta_rxbvalid;
	// input  wire                    mq2vta_rxbready;

	// // AR Channel
	// input  wire [BW_AXI_TID-1:0]   mq2vta_rxarid;
	// input  wire [BW_ADDR-1:0]      mq2vta_rxaraddr;
	// input  wire [`BW_AXI_ALEN-1:0] mq2vta_rxarlen;
	// input  wire [`BW_AXI_ASIZE-1:0] mq2vta_rxarsize;
	// input  wire [`BW_AXI_ABURST-1:0] mq2vta_rxarburst;
	// input  wire                    mq2vta_rxarvalid;
	// output wire                    mq2vta_rxarready;

	// // R Channel
	// output wire [BW_AXI_TID-1:0]   mq2vta_rxrid;
	// output wire [BW_AXI_DATA-1:0]  mq2vta_rxrdata;
	// output wire [`BW_AXI_RRESP-1:0] mq2vta_rxrresp;
	// output wire                    mq2vta_rxrlast;
	// output wire                    mq2vta_rxrvalid;
	// input  wire                    mq2vta_rxrready;


	wire [`BW_DCA_MATRIX_INFO_ALIGNED-1:0] mx_info, mw_info, mo_info;
	wire [32-1:0] qgemm_sinst_option;
	wire [`BW_DCA_MATRIX_INFO_NUM_COL_M1-1:0] mx_num_col_m1, mw_num_col_m1, mo_num_col_m1;
	wire [`BW_DCA_MATRIX_INFO_NUM_ROW_M1-1:0] mx_num_row_m1, mw_num_row_m1, mo_num_row_m1;
	wire [`BW_DCA_MATRIX_INFO_ADDR-1:0] mx_base_addr, mw_base_addr, mo_base_addr;
	wire [`BW_DCA_MATRIX_INFO_STRIDE_LS3-1:0] mx_stride, mw_stride, mo_stride;
	wire transpose_x_inst, transpose_w_inst;
	reg  transpose_x_q, transpose_w_q;

	wire i_load2mxreg_busy;
	wire i_load2mxreg_move_wenable;
	wire [BW_INPUT_TENSOR_ROW-1:0] i_load2mxreg_move_wdata_list1d;
	wire i_load2mxreg_loadreg_rready;
	wire i_load2mxreg_loadreg_rrequest;

	wire mxreg_move_wenable;
	wire [BW_INPUT_TENSOR_ROW-1:0] mxreg_move_wdata_list;
	wire mxreg_move_renable;
	wire [BW_INPUT_TENSOR_ROW-1:0] mxreg_move_rdata_list;
	wire mxreg_shift_up;
	wire mxreg_shift_left;
	wire mxreg_transpose;
	wire [BW_INPUT_TENSOR_MATRIX-1:0] mxreg_all_rdata_list2d;
	wire [BW_INPUT_TENSOR_ROW-1:0] mxreg_upmost_rdata_list1d;

	wire i_load2mwreg_busy;
	wire i_load2mwreg_move_wenable;
	wire [BW_WEIGHT_TENSOR_ROW-1:0] i_load2mwreg_move_wdata_list1d;
	wire i_load2mwreg_loadreg_rready;
	wire i_load2mwreg_loadreg_rrequest;

	wire mwreg_move_wenable;
	wire [BW_WEIGHT_TENSOR_ROW-1:0] mwreg_move_wdata_list;
	wire mwreg_move_renable;
	wire [BW_WEIGHT_TENSOR_ROW-1:0] mwreg_move_rdata_list;
	wire mwreg_shift_up;
	wire mwreg_shift_left;
	wire mwreg_transpose;
	wire [BW_WEIGHT_TENSOR_MATRIX-1:0] mwreg_all_rdata_list2d;
	wire [BW_WEIGHT_TENSOR_ROW-1:0] mwreg_upmost_rdata_list1d;

	wire i_moreg2store_busy;
	wire i_moreg2store_storereg_wready;
	wire i_moreg2store_storereg_wrequest;
	wire i_moreg2store_mreg_move_renable;
	wire [BW_OUTPUT_TENSOR_ROW-1:0] i_moreg2store_mreg_move_rdata_list1d;

	wire moreg_move_wenable;
	wire [BW_OUTPUT_TENSOR_ROW-1:0] moreg_move_wdata_list;
	wire moreg_move_renable;
	wire [BW_OUTPUT_TENSOR_ROW-1:0] moreg_move_rdata_list;
	wire moreg_shift_up;
	wire moreg_shift_left;
	wire moreg_transpose;
	wire [BW_OUTPUT_TENSOR_MATRIX-1:0] moreg_all_rdata_list2d;
	wire [BW_OUTPUT_TENSOR_ROW-1:0] moreg_upmost_rdata_list1d;

	wire i_qdq_controller_qx_start;
	wire i_qdq_controller_qw_start;
	wire i_qdq_controller_dq_start;
	wire i_qdq_controller_qx_busy;
	wire i_qdq_controller_qw_busy;
	wire i_qdq_controller_dq_busy;
	wire i_qdq_controller_scl_busy;
	wire i_qdq_controller_mx_valid;
	wire i_qdq_controller_mx_ready;
	wire [MATRIX_NUM_COL*FP_DATA_W-1:0] i_qdq_controller_mx_data;
	wire i_qdq_controller_qx_valid;
	wire i_qdq_controller_qx_ready;
	wire [MATRIX_NUM_COL*BIT_NUM-1:0] i_qdq_controller_qx_data;
	wire [LOG_MATRIX_NUM_ROW-1:0] i_qdq_controller_qx_index;
	wire i_qdq_controller_mw_valid;
	wire i_qdq_controller_mw_ready;
	wire [MATRIX_NUM_COL*FP_DATA_W-1:0] i_qdq_controller_mw_data;
	wire i_qdq_controller_qw_valid;
	wire i_qdq_controller_qw_ready;
	wire [MATRIX_NUM_COL*BIT_NUM-1:0] i_qdq_controller_qw_data;
	wire [LOG_MATRIX_NUM_ROW-1:0] i_qdq_controller_qw_index;
	wire i_qdq_controller_qo_valid;
	wire i_qdq_controller_qo_ready;
	wire [MATRIX_NUM_COL*VTA_DATA_WIDTH-1:0] i_qdq_controller_qo_data;
	wire [LOG_MATRIX_NUM_ROW-1:0] i_qdq_controller_qo_index;
	wire i_qdq_controller_mo_valid;
	wire i_qdq_controller_mo_ready;
	wire [MATRIX_NUM_COL*FP_DATA_W-1:0] i_qdq_controller_mo_data;

	localparam DP_LATENCY = 1;
	localparam DP_IN_DATA_WIDTH = VTA_DATA_WIDTH;
	localparam DP_OUT_DATA_WIDTH = 2*VTA_DATA_WIDTH + 5;  // 69

  	wire i_gemv_start;
  	wire i_gemv_busy;
  	wire i_gemv_qx_valid;
  	wire i_gemv_qx_ready;
  	wire [MATRIX_NUM_COL*DP_IN_DATA_WIDTH-1:0] i_gemv_qx_data;
  	wire [LOG_MATRIX_NUM_ROW-1:0] i_gemv_qx_index;
  	wire i_gemv_qw_valid;
  	wire i_gemv_qw_ready;
  	wire [MATRIX_NUM_COL*DP_IN_DATA_WIDTH-1:0] i_gemv_qw_data;
  	wire [LOG_MATRIX_NUM_ROW-1:0] i_gemv_qw_index;
  	wire i_gemv_qo_valid;
  	wire i_gemv_qo_ready;
  	wire [MATRIX_NUM_COL*DP_IN_DATA_WIDTH-1:0] i_gemv_qo_data;
	wire [LOG_MATRIX_NUM_ROW-1:0] i_gemv_qo_index;

	wire [MATRIX_NUM_COL*DP_IN_DATA_WIDTH-1:0] qx_data_repack_w;
	wire [MATRIX_NUM_COL*DP_IN_DATA_WIDTH-1:0] qw_data_repack_w;

	// wire qx2wsram_enable;
	// wire qx2wsram_init;
	// wire [MATRIX_NUM_COL*BIT_NUM-1:0] qx2wsram_quant_1row_data;
	// wire qx2wsram_quant_1row_valid;
	// wire qx2wsram_quant_1row_ready;
	// wire [LOG_MATRIX_NUM_ROW-1:0] qx2wsram_quant_row_index;
	// wire qx2wsram_wgrant;
	// wire qx2wsram_has_entry;
	// wire qx2wsram_ram_wen;
	// wire [SRAM_INDEX_WIDTH-1:0] qx2wsram_ram_windex;
	// wire [MATRIX_NUM_COL*VTA_DATA_WIDTH-1:0] qx2wsram_ram_wdata;
	// wire qx2wsram_wlast;

	// wire qw2wsram_enable;
	// wire qw2wsram_init;
	// wire [MATRIX_NUM_COL*BIT_NUM-1:0] qw2wsram_quant_1row_data;
	// wire qw2wsram_quant_1row_valid;
	// wire qw2wsram_quant_1row_ready;
	// wire [LOG_MATRIX_NUM_ROW-1:0] qw2wsram_quant_row_index;
	// wire qw2wsram_wgrant;
	// wire qw2wsram_has_entry;
	// wire qw2wsram_ram_wen;
	// wire [SRAM_INDEX_WIDTH-1:0] qw2wsram_ram_windex;
	// wire [MATRIX_NUM_COL*VTA_DATA_WIDTH-1:0] qw2wsram_ram_wdata;
	// wire qw2wsram_wlast;

	// localparam NUM_SRAM_PORT = 2;
	// wire	[NUM_SRAM_PORT-1:0] 					sram_renable_list;
	// wire 	[NUM_SRAM_PORT-1:0] 					sram_rvalid_list;
	// wire	[NUM_SRAM_PORT*SRAM_DATA_WIDTH-1:0] 	sram_rdata_list;
	// wire	[NUM_SRAM_PORT-1:0] 					sram_wenable_list;
	// wire	[NUM_SRAM_PORT*(SRAM_DATA_WIDTH/8)-1:0] sram_byte_enable_list;
	// wire	[NUM_SRAM_PORT*SRAM_INDEX_WIDTH-1:0] 	sram_index_list;
	// wire	[NUM_SRAM_PORT*SRAM_DATA_WIDTH-1:0] 	sram_wdata_list;

	// reg [1:0]has_grant;
	// localparam GRANT_NONE  = 2'b00;
	// localparam X_HAS_GRANT = 2'b01;
	// localparam W_HAS_GRANT = 2'b10;
	
	// wire quant2sram_wenable_d;
	// wire [SRAM_DATA_WIDTH-1:0] quant2sram_wdata_d;
	// wire [SRAM_INDEX_WIDTH-1:0] quant2sram_windex_d;
	
	// reg quant2sram_wenable_q;
	// reg [SRAM_DATA_WIDTH-1:0] quant2sram_wdata_q;
	// reg [SRAM_INDEX_WIDTH-1:0] quant2sram_windex_q;

	// wire sram2dq_renable;
	// wire [SRAM_INDEX_WIDTH-1:0] sram2dq_rindex;
	// wire [SRAM_DATA_WIDTH-1:0] sram2dq_rdata;
	// wire sram2dq_rvalid;
	// wire sram2dq_dq_1row_valid;
	// wire [LOG_MATRIX_NUM_ROW-1:0] sram2dq_dq_row_index;
	// wire [SRAM_DATA_WIDTH-1:0] sram2dq_dq_1row_data;
	// wire sram2dq_dq_1row_ready;
	// wire sram2dq_start;
	// wire sram2dq_busy;
	// wire sram2dq_read_last;

	reg [1:0] q_state, dq_state;
	localparam [1:0]  	S_IDLE=2'd0,
						S_LOAD=2'd1,
						S_EXEC=2'd2,
						S_STORE=2'd3;

	wire q_go_load,  q_go_exec,  q_go_store,  q_go_idle;
	wire dq_go_load, dq_go_exec, dq_go_store, dq_go_idle;

	wire [32-1:0] log_data;
	reg  [32-1:0] log_data_q;
	wire log_valid;

	always@(posedge clk, negedge rstnn) begin
		if(!rstnn) begin
			log_data_q <=0;
		end
		else begin
			log_data_q <= log_data;
		end
	end

	assign log_data = {q_state, dq_state, i_qdq_controller_qx_busy, i_qdq_controller_qw_busy, i_qdq_controller_dq_busy, i_qdq_controller_scl_busy,
						i_load2mxreg_busy, i_load2mwreg_busy, i_moreg2store_busy/*, has_grant, qx2wsram_has_entry, qw2wsram_has_entry, sram2dq_busy*/};
		
	assign log_valid = log_data != log_data_q;

	//unused mmiox signals
	assign control_rmx_core_status          = 0;
	assign control_rmx_log_fifo_wrequest    = log_valid & control_rmx_log_fifo_wready;
	assign control_rmx_log_fifo_wdata       = log_data;
	// assign control_rmx_input_fifo_rrequest  = 1'b0;
	assign control_rmx_output_fifo_wrequest = 1'b0;
	assign control_rmx_output_fifo_wdata    = 0;
	assign control_rmx_inst_fifo_rrequest = q_go_store;
	assign control_rmx_operation_finish = q_go_idle;
	assign control_rmx_clear_finish = 1'b0;
	assign control_rmx_output_fifo_wrequest = 0;
	assign control_rmx_output_fifo_wdata = 0;
	assign control_rmx_input_fifo_rrequest = dq_go_idle;

	//decode mmiox inst
	assign {qgemm_sinst_option, mo_info, mw_info, mx_info} = control_rmx_inst_fifo_rdata;
	assign transpose_x_inst = qgemm_sinst_option[0];
	assign transpose_w_inst = qgemm_sinst_option[1];
	assign {mo_num_col_m1, mo_num_row_m1, mo_stride, mo_base_addr} = mo_info;
	assign {mw_num_col_m1, mw_num_row_m1, mw_stride, mw_base_addr} = mw_info;
	assign {mx_num_col_m1, mx_num_row_m1, mx_stride, mx_base_addr} = mx_info;
	assign mo_sload_tensor_row_wready = 1'b0;

	always @(posedge clk or negedge rstnn) begin
		if(!rstnn) begin
			transpose_x_q <= 0;
			transpose_w_q <= 0;
		end
		else if(q_go_load) begin
			transpose_x_q <= transpose_x_inst;
			transpose_w_q <= ~transpose_w_inst; //for vta support :: default transpose
		end
	end

	assign mxreg_transpose = q_go_exec & transpose_x_q;
	assign mwreg_transpose = q_go_exec & transpose_w_q;
	assign moreg_transpose = 0;
	assign mxreg_shift_up  = 0;
	assign mxreg_shift_left= 0;
	assign mwreg_shift_up  = 0;
	assign mwreg_shift_left= 0;
	assign moreg_shift_up  = 0;
	assign moreg_shift_left= 0;
	assign mqxreg_shift_up  = 0;
	assign mqxreg_shift_left= 0;
	assign mqwreg_shift_up  = 0;
	assign mqwreg_shift_left= 0;

	always @(posedge clk or negedge rstnn) begin
		if(!rstnn) q_state <= S_IDLE;
		else
			case(q_state)
				S_IDLE : if(q_go_load)  q_state <= S_LOAD;
				S_LOAD : if(q_go_exec)  q_state <= S_EXEC;
				S_EXEC : if(q_go_store) q_state <= S_STORE;
				S_STORE: if(q_go_idle)  q_state <= S_IDLE;
			endcase
	end

	always @(posedge clk or negedge rstnn) begin
		if(!rstnn) dq_state <= S_IDLE;
		else 
			case(dq_state)
				S_IDLE : if(dq_go_load) dq_state <= S_LOAD;
				S_LOAD : if(dq_go_exec) dq_state <= S_EXEC;
				S_EXEC : if(dq_go_store) dq_state <= S_STORE;
				S_STORE: if(dq_go_idle) dq_state <= S_IDLE;
			endcase
	end

	reg dq_start_pending;
	always @(posedge clk or negedge rstnn) begin
	  if (!rstnn) dq_start_pending <= 1'b0;
	  else if (q_go_idle) dq_start_pending <= 1'b1;
	  else if (dq_go_load) dq_start_pending <= 1'b0;
	end

	assign q_go_load = (q_state==S_IDLE) & (control_rmx_inst_fifo_rready) & mx_sinst_wready & mw_sinst_wready & mo_sinst_wready;
	assign q_go_exec = (q_state == S_LOAD) & i_load2mxreg_loadreg_rready & i_load2mwreg_loadreg_rready;
	// assign q_go_store= (q_state == S_EXEC) & (~i_qdq_controller_qx_busy) & (~i_qdq_controller_qw_busy);
	// assign q_go_idle = (q_state == S_STORE) & (~qx2wsram_has_entry) & (~qw2wsram_has_entry);
	assign q_go_store= (q_state == S_EXEC) & (~i_qdq_controller_qw_busy);
	assign q_go_idle = (q_state == S_STORE) & (~i_qdq_controller_qx_busy);

	assign dq_go_load = (dq_state==S_IDLE) & (dq_start_pending) && (~i_qdq_controller_scl_busy);
	// assign dq_go_exec = (dq_state == S_LOAD) & (sram2dq_read_last);
	assign dq_go_exec = (dq_state == S_LOAD) & (~i_gemv_busy);
	assign dq_go_store= (dq_state == S_EXEC) & (~i_qdq_controller_dq_busy);
	assign dq_go_idle = (dq_state == S_STORE) /*& control_rmx_output_fifo_wready*/;

	assign mx_sinst_wvalid = q_go_load;
	assign mw_sinst_wvalid = q_go_load;
	assign mo_sinst_wvalid = q_go_load;

	assign mx_sinst_wdata  = {mx_info, OPC_READ};
	assign mw_sinst_wdata  = {mw_info, OPC_READ};
	assign mo_sinst_wdata  = {mo_info, OPC_WRITE};

	DCA_MATRIX_LOAD2MREG #(
		.MATRIX_SIZE_PARA(INPUT_MATRIX_SIZE),
		.TENSOR_PARA(TENSOR_PARA)
	) i_load2mxreg (
		.clk(clk),
		.rstnn(rstnn),
		.clear(1'b0),
		.enable(1'b1),
		.busy(i_load2mxreg_busy),

		.load_tensor_row_wvalid (mx_sload_tensor_row_wvalid),
		.load_tensor_row_wlast  (mx_sload_tensor_row_wlast),
		.load_tensor_row_wdata  (mx_sload_tensor_row_wdata),
		.load_tensor_row_wready (mx_sload_tensor_row_wready),

		.mreg_move_wenable      (i_load2mxreg_move_wenable),
		.mreg_move_wdata_list1d (i_load2mxreg_move_wdata_list1d),

		.loadreg_rready         (i_load2mxreg_loadreg_rready),
		.loadreg_rrequest       (i_load2mxreg_loadreg_rrequest)
	);

	DCA_MATRIX_REGISTER_TYPE3 #(
		.MATRIX_SIZE_PARA(INPUT_MATRIX_SIZE),
		.BW_TENSOR_SCALAR(BW_TENSOR_SCALAR),
		.BW_MOVE_DATA(BW_INPUT_TENSOR_ROW),
		.RESET_VALUE(MREG_RESET_VALUE)
	) i_mregx (
		.clk(clk),
		.rstnn(rstnn),

		.move_wenable (mxreg_move_wenable),
		.move_wdata_list(mxreg_move_wdata_list),
		.move_renable (mxreg_move_renable),
		.move_rdata_list(mxreg_move_rdata_list),

		.shift_up(mxreg_shift_up),
		.shift_left(mxreg_shift_left),
		.transpose(mxreg_transpose),

		.all_rdata_list2d(mxreg_all_rdata_list2d),
		.upmost_rdata_list1d(mxreg_upmost_rdata_list1d)
	);

	DCA_MATRIX_LOAD2MREG #(
		.MATRIX_SIZE_PARA(WEIGHT_MATRIX_SIZE),
		.TENSOR_PARA(TENSOR_PARA)
	) i_load2mwreg (
		.clk(clk),
		.rstnn(rstnn),
		.clear(1'b0),
		.enable(1'b1),
		.busy(i_load2mwreg_busy),

		.load_tensor_row_wvalid (mw_sload_tensor_row_wvalid),
		.load_tensor_row_wlast  (mw_sload_tensor_row_wlast),
		.load_tensor_row_wdata  (mw_sload_tensor_row_wdata),
		.load_tensor_row_wready (mw_sload_tensor_row_wready),

		.mreg_move_wenable      (i_load2mwreg_move_wenable),
		.mreg_move_wdata_list1d (i_load2mwreg_move_wdata_list1d),

		.loadreg_rready         (i_load2mwreg_loadreg_rready),
		.loadreg_rrequest       (i_load2mwreg_loadreg_rrequest)
	);

	DCA_MATRIX_REGISTER_TYPE3 #(
		.MATRIX_SIZE_PARA(WEIGHT_MATRIX_SIZE),
		.BW_TENSOR_SCALAR(BW_TENSOR_SCALAR),
		.BW_MOVE_DATA(BW_WEIGHT_TENSOR_ROW),
		.RESET_VALUE(MREG_RESET_VALUE)
	) i_mregw (
		.clk(clk),
		.rstnn(rstnn),

		.move_wenable (mwreg_move_wenable),
		.move_wdata_list(mwreg_move_wdata_list),
		.move_renable (mwreg_move_renable),
		.move_rdata_list(mwreg_move_rdata_list),

		.shift_up(mwreg_shift_up),
		.shift_left(mwreg_shift_left),
		.transpose(mwreg_transpose),

		.all_rdata_list2d(mwreg_all_rdata_list2d),
		.upmost_rdata_list1d(mwreg_upmost_rdata_list1d)
	);

	assign i_load2mxreg_loadreg_rrequest = q_go_store;
	assign mxreg_move_wenable = i_load2mxreg_move_wenable;
	assign mxreg_move_wdata_list = i_load2mxreg_move_wdata_list1d;
	assign mxreg_move_renable = i_qdq_controller_mx_valid & i_qdq_controller_mx_ready;  // Advance when data is consumed
	assign mx_sstore_tensor_row_rready = 1'b0;
	assign mx_sstore_tensor_row_rdata = {BW_INPUT_TENSOR_ROW{1'b0}};
	
	assign i_load2mwreg_loadreg_rrequest = q_go_store;
	assign mwreg_move_wenable = i_load2mwreg_move_wenable;
	assign mwreg_move_wdata_list = i_load2mwreg_move_wdata_list1d;
	assign mwreg_move_renable = i_qdq_controller_mw_valid & i_qdq_controller_mw_ready;  // Advance when data is consumed
	assign mw_sstore_tensor_row_rready = 1'b0;
	assign mw_sstore_tensor_row_rdata = {BW_WEIGHT_TENSOR_ROW{1'b0}};

	qdq_controller_1row #(
		.BIT_NUM(BIT_NUM),
		.FP_DATA_W(FP_DATA_W),
		.FP_EXP_W(FP_EXP_W),
		.FP_MANT_W(FP_MANT_W),
		.FP_EXP_BIAS(FP_EXP_BIAS),
		.VTA_DATA_WIDTH(VTA_DATA_WIDTH),
		.MATRIX_NUM_COL(MATRIX_NUM_COL),
		.MATRIX_NUM_ROW(MATRIX_NUM_ROW),
		.SCALE_FIFO_DEPTH(SCALE_FIFO_DEPTH)
	) i_qdq_controller (
		.clk(clk),
		.rstnn(rstnn),
		.qx_start_i(i_qdq_controller_qx_start),
		.qw_start_i(i_qdq_controller_qw_start),
		.dq_start_i(i_qdq_controller_dq_start),
		.qx_busy_o(i_qdq_controller_qx_busy),
		.qw_busy_o(i_qdq_controller_qw_busy),
		.dq_busy_o(i_qdq_controller_dq_busy),
		.scl_busy_o(i_qdq_controller_scl_busy),
		.mx_valid_i(i_qdq_controller_mx_valid),
		.mx_ready_o(i_qdq_controller_mx_ready),
		.mx_data_i(i_qdq_controller_mx_data),
		.qx_valid_o(i_qdq_controller_qx_valid),
		.qx_ready_i(i_qdq_controller_qx_ready),
		.qx_data_o(i_qdq_controller_qx_data),
		.qx_index_o(i_qdq_controller_qx_index),
		.mw_valid_i(i_qdq_controller_mw_valid),
		.mw_ready_o(i_qdq_controller_mw_ready),
		.mw_data_i(i_qdq_controller_mw_data),
		.qw_valid_o(i_qdq_controller_qw_valid),
		.qw_ready_i(i_qdq_controller_qw_ready),
		.qw_data_o(i_qdq_controller_qw_data),
		.qw_index_o(i_qdq_controller_qw_index),
		.qo_valid_i(i_qdq_controller_qo_valid),
		.qo_ready_o(i_qdq_controller_qo_ready),
		.qo_data_i(i_qdq_controller_qo_data),
		.qo_index_i(i_qdq_controller_qo_index),
		.mo_valid_o(i_qdq_controller_mo_valid),
		.mo_ready_i(i_qdq_controller_mo_ready),
		.mo_data_o(i_qdq_controller_mo_data)
	);

	assign i_qdq_controller_qx_start = q_go_store & (~i_qdq_controller_qx_busy);
	assign i_qdq_controller_qw_start = q_go_exec & (~i_qdq_controller_qw_busy);
	assign i_qdq_controller_dq_start = dq_go_exec;

	assign i_qdq_controller_mx_valid = i_qdq_controller_qx_busy;
	assign i_qdq_controller_mx_data = mxreg_move_rdata_list;
	assign i_qdq_controller_mw_valid = i_qdq_controller_qw_busy;
	assign i_qdq_controller_mw_data = mwreg_move_rdata_list;

	assign i_qdq_controller_qo_valid = i_gemv_qo_valid;
	assign i_qdq_controller_qo_data = i_gemv_qo_data;
	assign i_qdq_controller_qo_index = i_gemv_qo_index;
	assign i_gemv_qo_ready = i_qdq_controller_qo_ready;

	// dequant 결과 → moreg
	assign moreg_move_wenable = i_qdq_controller_mo_valid;
	assign moreg_move_wdata_list = i_qdq_controller_mo_data;
	assign i_qdq_controller_mo_ready = 1'b1;

	// moreg → store 경로

	DCA_MATRIX_REGISTER_TYPE3 #(
		.MATRIX_SIZE_PARA(OUTPUT_MATRIX_SIZE),
		.BW_TENSOR_SCALAR(BW_TENSOR_SCALAR),
		.BW_MOVE_DATA(BW_OUTPUT_TENSOR_ROW),
		.RESET_VALUE(MREG_RESET_VALUE)
	) i_moreg (
		.clk(clk),
		.rstnn(rstnn),

		.move_wenable (moreg_move_wenable),
		.move_wdata_list(moreg_move_wdata_list),
		.move_renable (moreg_move_renable),
		.move_rdata_list(moreg_move_rdata_list),

		.shift_up(moreg_shift_up),
		.shift_left(moreg_shift_left),
		.transpose(moreg_transpose),

		.all_rdata_list2d(moreg_all_rdata_list2d),
		.upmost_rdata_list1d(moreg_upmost_rdata_list1d)
	);

	DCA_MATRIX_MREG2STORE #(
		.MATRIX_SIZE_PARA(OUTPUT_MATRIX_SIZE),
  		.BW_TENSOR_SCALAR(BW_TENSOR_SCALAR)
	) i_moreg2store (
		.clk(clk),
		.rstnn(rstnn),
		.clear(1'b0),
		.enable(1'b1),
		.busy(i_moreg2store_busy),

		.storereg_wready (i_moreg2store_storereg_wready),
		.storereg_wrequest(i_moreg2store_storereg_wrequest),

		.mreg_move_renable      (i_moreg2store_mreg_move_renable),
		.mreg_move_rdata_list1d (i_moreg2store_mreg_move_rdata_list1d),

		.store_tensor_row_rvalid (mo_sstore_tensor_row_rvalid),
		.store_tensor_row_rlast  (mo_sstore_tensor_row_rlast),
		.store_tensor_row_rready (mo_sstore_tensor_row_rready),
		.store_tensor_row_rdata  (mo_sstore_tensor_row_rdata)
	);

	assign moreg_move_renable = i_moreg2store_mreg_move_renable;
	assign i_moreg2store_mreg_move_rdata_list1d = moreg_upmost_rdata_list1d;
	assign i_moreg2store_storereg_wrequest = dq_go_store; // DQ completion triggers store operation

  	GEMV #(
  	  .MATRIX_SIZE      (INPUT_MATRIX_SIZE),
  	  .BW_IN_DATA       (DP_IN_DATA_WIDTH), 
  	  .BW_OUT_DATA      (DP_OUT_DATA_WIDTH),
  	  .DP_LATENCY       (DP_LATENCY),
	  .INDEX_W          (LOG_MATRIX_NUM_ROW),
  	  .MREG_RESET_VALUE (MREG_RESET_VALUE),
  	  .BW_TENSOR_SCALAR (BW_TENSOR_SCALAR)
	) i_gemv (
  	  .clk(clk),
  	  .rstnn(rstnn),

  	  .start_i(i_gemv_start),
  	  .busy_o(i_gemv_busy),

  	  .qx_valid_i(i_gemv_qx_valid),
  	  .qx_ready_o(i_gemv_qx_ready),
  	  .qx_data_i(i_gemv_qx_data),
  	  .qx_index_i(i_gemv_qx_index),

  	  .qw_valid_i(i_gemv_qw_valid),
  	  .qw_ready_o(i_gemv_qw_ready),
  	  .qw_data_i(i_gemv_qw_data),
  	  .qw_index_i(i_gemv_qw_index),

  	  .qo_valid_o(i_gemv_qo_valid),
  	  .qo_ready_i(i_gemv_qo_ready),
  	  .qo_data_o(i_gemv_qo_data),
	  .qo_index_o(i_gemv_qo_index)
  	);

	// 비트 확장( Quant(BIT_NUM) -> GEMV(DP_IN_DATA_WIDTH) )
	genvar c;
	generate
	  for (c = 0; c < MATRIX_NUM_COL; c++) begin
	    wire [BIT_NUM-1:0] qx_data_w = i_qdq_controller_qx_data[BIT_NUM*(c+1)-1 -: BIT_NUM];
	    wire [BIT_NUM-1:0] qw_data_w = i_qdq_controller_qw_data[BIT_NUM*(c+1)-1 -: BIT_NUM];

	    assign qx_data_repack_w[DP_IN_DATA_WIDTH*(c+1)-1 -: DP_IN_DATA_WIDTH] = {{(DP_IN_DATA_WIDTH-BIT_NUM){qx_data_w[BIT_NUM-1]}}, qx_data_w};
	    assign qw_data_repack_w[DP_IN_DATA_WIDTH*(c+1)-1 -: DP_IN_DATA_WIDTH] = {{(DP_IN_DATA_WIDTH-BIT_NUM){qw_data_w[BIT_NUM-1]}}, qw_data_w};
	  end
	endgenerate


	assign i_gemv_start = q_go_exec;
	assign i_gemv_qx_valid = i_qdq_controller_qx_valid;
	assign i_gemv_qx_data = qx_data_repack_w;
	assign i_gemv_qx_index = i_qdq_controller_qx_index;
	assign i_qdq_controller_qx_ready = i_gemv_qx_ready;

	assign i_gemv_qw_valid = i_qdq_controller_qw_valid;
	assign i_gemv_qw_data = qw_data_repack_w;
	assign i_gemv_qw_index = i_qdq_controller_qw_index;
	assign i_qdq_controller_qw_ready = i_gemv_qw_ready;

	// quant2wsram_store #(
	// 	.RAM_INDEX_BASE(0),
	// 	.QUANT_DATA_1ELM_WIDTH(BIT_NUM),
	// 	.RAM_DATA_1ELM_WIDTH(VTA_DATA_WIDTH),
	// 	.MATRIX_NUM_COL(MATRIX_NUM_COL),
	// 	.MATRIX_NUM_ROW(MATRIX_NUM_ROW),
	// 	.RAM_INDEX_WIDTH(SRAM_INDEX_WIDTH)
	// ) i_qx2wsram_store(
	// 	.clk(clk),
	// 	.rstnn(rstnn),
	// 	.enable_i(qx2wsram_enable),
	// 	.init_i(qx2wsram_init),
	// 	.quant_1row_data_i(qx2wsram_quant_1row_data),
	// 	.quant_1row_valid_i(qx2wsram_quant_1row_valid),
	// 	.quant_1row_ready_o(qx2wsram_quant_1row_ready),
	// 	.quant_row_index_i(qx2wsram_quant_row_index),
	// 	.wgrant_i(qx2wsram_wgrant),
	// 	.has_entry_o(qx2wsram_has_entry),
	// 	.ram_wen_o(qx2wsram_ram_wen),
	// 	.ram_windex_o(qx2wsram_ram_windex),
	// 	.ram_wdata_o(qx2wsram_ram_wdata),
	// 	.ram_wlast_o(qx2wsram_wlast)
	// );

	// quant2wsram_store #(
	// 	.RAM_INDEX_BASE(32'h40),
	// 	.QUANT_DATA_1ELM_WIDTH(BIT_NUM),
	// 	.RAM_DATA_1ELM_WIDTH(VTA_DATA_WIDTH),
	// 	.MATRIX_NUM_COL(MATRIX_NUM_COL),
	// 	.MATRIX_NUM_ROW(MATRIX_NUM_ROW),
	// 	.RAM_INDEX_WIDTH(SRAM_INDEX_WIDTH)
	// ) i_qw2wsram_store(
	// 	.clk(clk),
	// 	.rstnn(rstnn),
	// 	.enable_i(qw2wsram_enable),
	// 	.init_i(qw2wsram_init),
	// 	.quant_1row_data_i(qw2wsram_quant_1row_data),
	// 	.quant_1row_valid_i(qw2wsram_quant_1row_valid),
	// 	.quant_1row_ready_o(qw2wsram_quant_1row_ready),
	// 	.quant_row_index_i(qw2wsram_quant_row_index),
	// 	.wgrant_i(qw2wsram_wgrant),
	// 	.has_entry_o(qw2wsram_has_entry),
	// 	.ram_wen_o(qw2wsram_ram_wen),
	// 	.ram_windex_o(qw2wsram_ram_windex),
	// 	.ram_wdata_o(qw2wsram_ram_wdata),
	// 	.ram_wlast_o(qw2wsram_wlast)
	// );

	// assign qx2wsram_enable = 1'b1;
	// assign qx2wsram_init   = q_go_load;
	// assign qx2wsram_quant_1row_data = i_qdq_controller_qx_data;
	// assign qx2wsram_quant_1row_valid = i_qdq_controller_qx_valid;
	// assign i_qdq_controller_qx_ready = qx2wsram_quant_1row_ready;
	// assign qx2wsram_quant_row_index = i_qdq_controller_qx_index;

	// assign qw2wsram_enable = 1'b1;
	// assign qw2wsram_init   = q_go_load;
	// assign qw2wsram_quant_1row_data = i_qdq_controller_qw_data;
	// assign qw2wsram_quant_1row_valid = i_qdq_controller_qw_valid;
	// assign i_qdq_controller_qw_ready = qw2wsram_quant_1row_ready;
	// assign qw2wsram_quant_row_index = i_qdq_controller_qw_index;

	// wsram2dq_load #(
	// 	.RAM_INDEX_BASE(0),
	// 	.RAM_DATA_1ELM_WIDTH(VTA_DATA_WIDTH),  // 32
	// 	.MATRIX_NUM_COL(MATRIX_NUM_COL),
	// 	.MATRIX_NUM_ROW(MATRIX_NUM_ROW),
	// 	.RAM_INDEX_WIDTH(SRAM_INDEX_WIDTH)
	// ) u_wsram2dq_load (
	// 	.clk            (clk),
	// 	.rstnn          (rstnn),
	// 	.enable_i       (1'b1),
	// 	.start_i        (sram2dq_start),
	// 	.busy_o         (sram2dq_busy),
	// 	.wgrant_i       (1'b1),
	// 	.read_last_o    (sram2dq_read_last),
	// 	.ram_ren_o      (sram2dq_renable),
	// 	.ram_rindex_o   (sram2dq_rindex),
	// 	.ram_rdata_i    (sram2dq_rdata),
	// 	.ram_rvalid_i   (sram2dq_rvalid),
	// 	.dq_1row_data_o (sram2dq_dq_1row_data),
	// 	.dq_1row_valid_o(sram2dq_dq_1row_valid),
	// 	.dq_1row_ready_i(sram2dq_dq_1row_ready),
	// 	.dq_row_index_o (sram2dq_dq_row_index)
	// );
	
	// axi_dual_port_sram128#(
	// 	.BW_AXI_DATA(BW_AXI_DATA),
	// 	.BW_ADDR(BW_ADDR),
	// 	.BW_AXI_TID(BW_AXI_TID)
	// )
	// i_axi_dual_port_sram128(
	// 	.clk(clk),
	// 	.rstnn(rstnn),
	// 	.rxawid(mq2vta_rxawid),
	// 	.rxawaddr(mq2vta_rxawaddr),
	// 	.rxawlen(mq2vta_rxawlen),
	// 	.rxawsize(mq2vta_rxawsize),
	// 	.rxawburst(mq2vta_rxawburst),
	// 	.rxawvalid(mq2vta_rxawvalid),
	// 	.rxawready(mq2vta_rxawready),
	// 	.rxwid(mq2vta_rxwid),
	// 	.rxwdata(mq2vta_rxwdata),
	// 	.rxwstrb(mq2vta_rxwstrb),
	// 	.rxwlast(mq2vta_rxwlast),
	// 	.rxwvalid(mq2vta_rxwvalid),
	// 	.rxwready(mq2vta_rxwready),
	// 	.rxbid(mq2vta_rxbid),
	// 	.rxbresp(mq2vta_rxbresp),
	// 	.rxbvalid(mq2vta_rxbvalid),
	// 	.rxbready(mq2vta_rxbready),
	// 	.rxarid(mq2vta_rxarid),
	// 	.rxaraddr(mq2vta_rxaraddr),
	// 	.rxarlen(mq2vta_rxarlen),
	// 	.rxarsize(mq2vta_rxarsize),
	// 	.rxarburst(mq2vta_rxarburst),
	// 	.rxarvalid(mq2vta_rxarvalid),
	// 	.rxarready(mq2vta_rxarready),
	// 	.rxrid(mq2vta_rxrid),
	// 	.rxrdata(mq2vta_rxrdata),
	// 	.rxrresp(mq2vta_rxrresp),
	// 	.rxrlast(mq2vta_rxrlast),
	// 	.rxrvalid(mq2vta_rxrvalid),
	// 	.rxrready(mq2vta_rxrready),
	// 	.int_renable_list(sram_renable_list),
	// 	.int_rdata_list(sram_rdata_list),
	// 	.int_rvalid_list(sram_rvalid_list),
	// 	.int_index_list(sram_index_list),
	// 	.int_wenable_list(sram_wenable_list),
	// 	.int_byte_enable_list(sram_byte_enable_list),
	// 	.int_wdata_list(sram_wdata_list)
	// );

	// always @(posedge clk, negedge rstnn) begin
	// 	if(!rstnn) begin
	// 		has_grant <= GRANT_NONE;
	// 	end
	// 	else begin
	// 		case(has_grant)
	// 			GRANT_NONE: begin
	// 				if(qx2wsram_quant_1row_valid)
	// 					has_grant <= X_HAS_GRANT;
	// 				else if(qw2wsram_quant_1row_valid)
	// 					has_grant <= W_HAS_GRANT;
	// 			end
	// 			X_HAS_GRANT: begin
	// 				if(~qx2wsram_has_entry & ~i_qdq_controller_qx_busy) begin //after finishing all qx store
	// 					if(qw2wsram_has_entry)
	// 						has_grant <= W_HAS_GRANT;
	// 					else
	// 						has_grant <= GRANT_NONE;
	// 				end
	// 			end
	// 			W_HAS_GRANT: begin
	// 				if(~qw2wsram_has_entry & ~i_qdq_controller_qw_busy) begin //after finishing all qw store
	// 					if(qx2wsram_has_entry)
	// 						has_grant <= X_HAS_GRANT;
	// 					else
	// 						has_grant <= GRANT_NONE;
	// 				end
	// 			end
	// 		endcase
	// 	end
	// end

	// assign qx2wsram_wgrant = (has_grant == X_HAS_GRANT) || (qx2wsram_has_entry & ~qw2wsram_has_entry);
	// assign qw2wsram_wgrant = (has_grant == W_HAS_GRANT) || (qw2wsram_has_entry & ~qx2wsram_has_entry);
	// assign quant2sram_wenable_d = qx2wsram_ram_wen | qw2wsram_ram_wen;
	// assign quant2sram_wdata_d   = qx2wsram_wgrant ? qx2wsram_ram_wdata : qw2wsram_ram_wdata;
	// assign quant2sram_windex_d  = qx2wsram_wgrant ? qx2wsram_ram_windex : qw2wsram_ram_windex;
	
	// always @(posedge clk, negedge rstnn) begin
	// 	if(!rstnn) begin
	// 		quant2sram_wenable_q <= 0;
	// 		quant2sram_wdata_q   <= 0;
	// 		quant2sram_windex_q  <= 0;
	// 	end
	// 	else begin
	// 		quant2sram_wenable_q <= quant2sram_wenable_d;
	// 		quant2sram_wdata_q   <= quant2sram_wdata_d;
	// 		quant2sram_windex_q  <= quant2sram_windex_d;
	// 	end
	// end

	// assign sram_byte_enable_list = {NUM_SRAM_PORT{ {SRAM_DATA_WIDTH/8{1'b1}} }};
	// assign sram_wenable_list = {1'b0,quant2sram_wenable_q};
	// assign sram_wdata_list   = {{SRAM_DATA_WIDTH{1'b0}},quant2sram_wdata_q};
	// assign sram_index_list  = {sram2dq_rindex, quant2sram_windex_q};
	// assign sram_renable_list = {sram2dq_renable, 1'b0};
	// assign sram2dq_rdata = sram_rdata_list[2*SRAM_DATA_WIDTH-1:SRAM_DATA_WIDTH];
	// assign sram2dq_rvalid = sram_rvalid_list[1];

endmodule

