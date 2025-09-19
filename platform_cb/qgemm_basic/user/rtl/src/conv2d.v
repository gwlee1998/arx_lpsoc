// ****************************************************************************
// ****************************************************************************
// Copyright SoC Design Research Group, All rights reserved.
// Electronics and Telecommunications Research Institute (ETRI)
// 
// THESE DOCUMENTS CONTAIN CONFIDENTIAL INFORMATION AND KNOWLEDGE
// WHICH IS THE PROPERTY OF ETRI. NO PART OF THIS PUBLICATION IS
// TO BE USED FOR ANY OTHER PURPOSE, AND THESE ARE NOT TO BE
// REPRODUCED, COPIED, DISCLOSED, TRANSMITTED, STORED IN A RETRIEVAL
// SYSTEM OR TRANSLATED INTO ANY OTHER HUMAN OR COMPUTER LANGUAGE,
// IN ANY FORM, BY ANY MEANS, IN WHOLE OR IN PART, WITHOUT THE
// COMPLETE PRIOR WRITTEN PERMISSION OF ETRI.
// ****************************************************************************
// 2024-06-28
// Kyuseung Han (han@etri.re.kr)
// ****************************************************************************
// ****************************************************************************

`include "ervp_global.vh"
`include "ervp_endian.vh"
`include "dca_module_memorymap_offset.vh"

`include "dca_matrix_info.vh"
`include "dca_matrix_lsu_inst.vh"

module DCA_MATRIX_CONV2D_MMIOX_MLSU
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

  mi_sinst_wvalid,
	mi_sinst_wdata,
	mi_sinst_wready,
	mi_sinst_decode_finish,
	mi_sinst_execute_finish,
	mi_sinst_busy,
	mi_sload_tensor_row_wvalid,
	mi_sload_tensor_row_wlast,
	mi_sload_tensor_row_wdata,
	mi_sload_tensor_row_wready,
	mi_sstore_tensor_row_rvalid,
	mi_sstore_tensor_row_rlast,
	mi_sstore_tensor_row_rready,
  mi_sstore_tensor_row_rdata,

  mk_sinst_wvalid,
	mk_sinst_wdata,
	mk_sinst_wready,
	mk_sinst_decode_finish,
	mk_sinst_execute_finish,
	mk_sinst_busy,
	mk_sload_tensor_row_wvalid,
	mk_sload_tensor_row_wlast,
	mk_sload_tensor_row_wdata,
	mk_sload_tensor_row_wready,
	mk_sstore_tensor_row_rvalid,
	mk_sstore_tensor_row_rlast,
	mk_sstore_tensor_row_rready,
	mk_sstore_tensor_row_rdata,

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
  mo_sstore_tensor_row_rdata
);

////////////////////////////
/* parameter input output */
////////////////////////////

parameter BW_ADDR = 32;
parameter MA_BW_DATA = 128;
parameter MB_BW_DATA = 128;
parameter MC_BW_DATA = 128;

parameter INPUT_MATRIX_SIZE = 18;
parameter KERNEL_MATRIX_SIZE = 7;
parameter OUTPUT_MATRIX_SIZE =16;
parameter TENSOR_PARA = 0;

localparam BW_CONFIG = 1;
localparam BW_STATUS = `BW_DCA_MATRIX_CONV2D_STATUS;
localparam BW_LOG = `BW_DCA_MATRIX_CONV2D_LOG;
localparam BW_INST = `BW_DCA_MATRIX_CONV2D_INST;
localparam BW_INPUT = 32;
localparam BW_OUTPUT = 32;

`include "dca_matrix_dim_util.vb"
`include "dca_tensor_scalar_lpara.vb"

localparam INPUT_MATRIX_NUM_COL = GET_MATRIX_NUM_COL(INPUT_MATRIX_SIZE);
localparam BW_INPUT_TENSOR_ROW = BW_TENSOR_SCALAR*INPUT_MATRIX_NUM_COL;
localparam BW_INPUT_TENSOR_MATRIX = BW_INPUT_TENSOR_ROW*INPUT_MATRIX_NUM_COL;

localparam KERNEL_MATRIX_NUM_COL = GET_MATRIX_NUM_COL(KERNEL_MATRIX_SIZE);
localparam BW_KERNEL_TENSOR_ROW = BW_TENSOR_SCALAR*KERNEL_MATRIX_NUM_COL;
localparam BW_KERNEL_TENSOR_MATRIX = BW_KERNEL_TENSOR_ROW*KERNEL_MATRIX_NUM_COL;

localparam OUTPUT_MATRIX_NUM_COL = GET_MATRIX_NUM_COL(OUTPUT_MATRIX_SIZE);
localparam BW_OUTPUT_TENSOR_ROW = BW_TENSOR_SCALAR*OUTPUT_MATRIX_NUM_COL;
localparam BW_OUTPUT_TENSOR_MATRIX = BW_OUTPUT_TENSOR_ROW*OUTPUT_MATRIX_NUM_COL;

input wire clk;
input wire rstnn;

input wire [(BW_CONFIG)-1:0] control_rmx_core_config;
output wire [(BW_STATUS)-1:0] control_rmx_core_status;
input wire control_rmx_clear_request;
output wire control_rmx_clear_finish;
input wire control_rmx_log_fifo_wready;
output wire control_rmx_log_fifo_wrequest;
output wire [(BW_LOG)-1:0] control_rmx_log_fifo_wdata;
input wire control_rmx_inst_fifo_rready;
input wire [(BW_INST)-1:0] control_rmx_inst_fifo_rdata;
output wire control_rmx_inst_fifo_rrequest;
output wire control_rmx_operation_finish;
input wire control_rmx_input_fifo_rready;
input wire [(BW_INPUT)-1:0] control_rmx_input_fifo_rdata;
output wire control_rmx_input_fifo_rrequest;
input wire control_rmx_output_fifo_wready;
output wire control_rmx_output_fifo_wrequest;
output wire [(BW_OUTPUT)-1:0] control_rmx_output_fifo_wdata;

output wire mi_sinst_wvalid;
output wire [(`BW_DCA_MATRIX_LSU_INST)-1:0] mi_sinst_wdata;
input wire mi_sinst_wready;
input wire mi_sinst_decode_finish;
input wire mi_sinst_execute_finish;
input wire mi_sinst_busy;
input wire mi_sload_tensor_row_wvalid;
input wire mi_sload_tensor_row_wlast;
input wire [BW_INPUT_TENSOR_ROW-1:0] mi_sload_tensor_row_wdata;
output wire mi_sload_tensor_row_wready;
input wire mi_sstore_tensor_row_rvalid;
input wire mi_sstore_tensor_row_rlast;
output wire mi_sstore_tensor_row_rready;
output wire [BW_INPUT_TENSOR_ROW-1:0] mi_sstore_tensor_row_rdata;

output wire mk_sinst_wvalid;
output wire [(`BW_DCA_MATRIX_LSU_INST)-1:0] mk_sinst_wdata;
input wire mk_sinst_wready;
input wire mk_sinst_decode_finish;
input wire mk_sinst_execute_finish;
input wire mk_sinst_busy;
input wire mk_sload_tensor_row_wvalid;
input wire mk_sload_tensor_row_wlast;
input wire [BW_KERNEL_TENSOR_ROW-1:0] mk_sload_tensor_row_wdata;
output wire mk_sload_tensor_row_wready;
input wire mk_sstore_tensor_row_rvalid;
input wire mk_sstore_tensor_row_rlast;
output wire mk_sstore_tensor_row_rready;
output wire [BW_KERNEL_TENSOR_ROW-1:0] mk_sstore_tensor_row_rdata;

output wire mo_sinst_wvalid;
output wire [(`BW_DCA_MATRIX_LSU_INST)-1:0] mo_sinst_wdata;
input wire mo_sinst_wready;
input wire mo_sinst_decode_finish;
input wire mo_sinst_execute_finish;
input wire mo_sinst_busy;
input wire mo_sload_tensor_row_wvalid;
input wire mo_sload_tensor_row_wlast;
input wire [BW_OUTPUT_TENSOR_ROW-1:0] mo_sload_tensor_row_wdata;
output wire mo_sload_tensor_row_wready;
input wire mo_sstore_tensor_row_rvalid;
input wire mo_sstore_tensor_row_rlast;
output wire mo_sstore_tensor_row_rready;
output wire [BW_OUTPUT_TENSOR_ROW-1:0] mo_sstore_tensor_row_rdata;

/////////////
/* signals */
/////////////

genvar i, j;

wire [`BW_DCA_MATRIX_INFO_ALIGNED-1:0] mi_info;
wire [`BW_DCA_MATRIX_INFO_ALIGNED-1:0] mk_info;
wire [`BW_DCA_MATRIX_INFO_ALIGNED-1:0] mo_info;
wire [`BW_DCA_MATRIX_CONV2D_INST_STRIDE_M1-1:0] inst_stride_m1;
wire [`BW_DCA_MATRIX_CONV2D_INST_PAD-1:0] inst_pad;

wire [`BW_DCA_MATRIX_LSU_INST_OPCODE-1:0] lsu0_inst_opcode;
wire [`BW_DCA_MATRIX_LSU_INST_OPCODE-1:0] lsu1_inst_opcode;
wire [`BW_DCA_MATRIX_LSU_INST_OPCODE-1:0] lsu2_inst_opcode;

wire [`BW_DCA_MATRIX_INFO_ADDR-1:0] mi_inst_addr;
wire [`BW_DCA_MATRIX_INFO_STRIDE_LS3-1:0] mi_inst_stride_ls3;
wire [`BW_DCA_MATRIX_INFO_NUM_ROW_M1-1:0] mi_inst_num_row_m1;

wire [`BW_DCA_MATRIX_INFO_ADDR-1:0] mk_inst_addr;
wire [`BW_DCA_MATRIX_INFO_STRIDE_LS3-1:0] mk_inst_stride_ls3;
wire [`BW_DCA_MATRIX_INFO_NUM_ROW_M1-1:0] mk_inst_num_row_m1;

wire [`BW_DCA_MATRIX_INFO_ADDR-1:0] mo_inst_addr;
wire [`BW_DCA_MATRIX_INFO_STRIDE_LS3-1:0] mo_inst_stride_ls3;
wire [`BW_DCA_MATRIX_INFO_NUM_ROW_M1-1:0] mo_inst_num_row_m1;

localparam BW_MATRIX_SIZE = 8;

wire [BW_MATRIX_SIZE-1:0] input_size_m1, kernel_size_m1, output_size_m1;

localparam BW_STATE = 2;
localparam IDLE = 0;
localparam LOAD = 1;
localparam EXECUTE = 2;
localparam STORE = 3;

reg [BW_STATE-1:0] state;
wire go_load;
wire go_execute;
wire go_store;
wire go_idle;

wire execute_finish;

localparam MREG_RESET_VALUE = TENSOR_ZERO;

wire i_load02mreg_busy;
wire i_load02mreg_load_tensor_row_wvalid;
wire i_load02mreg_load_tensor_row_wlast;
wire [BW_INPUT_TENSOR_ROW-1:0] i_load02mreg_load_tensor_row_wdata;
wire i_load02mreg_load_tensor_row_wready;
wire i_load02mreg_mreg_move_wenable;
wire [BW_INPUT_TENSOR_ROW-1:0] i_load02mreg_mreg_move_wdata_list1d;
wire i_load02mreg_load_rready;
wire i_load02mreg_load_rrequest;

wire i_mreg0_move_wenable;
wire [BW_INPUT_TENSOR_ROW-1:0] i_mreg0_move_wdata_list;
wire i_mreg0_move_renable;
wire [BW_INPUT_TENSOR_ROW-1:0] i_mreg0_move_rdata_list;
wire i_mreg0_shift_up;
wire i_mreg0_shift_left;
wire i_mreg0_transpose;
wire [BW_INPUT_TENSOR_MATRIX-1:0] i_mreg0_all_rdata_list2d;
wire [BW_INPUT_TENSOR_ROW-1:0] i_mreg0_upmost_rdata_list1d;

wire i_load12mreg_busy;
wire i_load12mreg_load_tensor_row_wvalid;
wire i_load12mreg_load_tensor_row_wlast;
wire [BW_KERNEL_TENSOR_ROW-1:0] i_load12mreg_load_tensor_row_wdata;
wire i_load12mreg_load_tensor_row_wready;
wire i_load12mreg_mreg_move_wenable;
wire [BW_KERNEL_TENSOR_ROW-1:0] i_load12mreg_mreg_move_wdata_list1d;
wire i_load12mreg_load_rready;
wire i_load12mreg_load_rrequest;

wire i_mreg1_move_wenable;
wire [BW_KERNEL_TENSOR_ROW-1:0] i_mreg1_move_wdata_list;
wire i_mreg1_move_renable;
wire [BW_KERNEL_TENSOR_ROW-1:0] i_mreg1_move_rdata_list;
wire i_mreg1_shift_up;
wire i_mreg1_shift_left;
wire i_mreg1_transpose;
wire [BW_KERNEL_TENSOR_MATRIX-1:0] i_mreg1_all_rdata_list2d;
wire [BW_KERNEL_TENSOR_ROW-1:0] i_mreg1_upmost_rdata_list1d;

wire i_mreg2store_busy;
wire i_mreg2store_store_wready;
wire i_mreg2store_store_wrequest;
wire i_mreg2store_mreg_move_renable;
wire [BW_OUTPUT_TENSOR_ROW-1:0] i_mreg2store_mreg_move_rdata_list1d;
wire i_mreg2store_store_tensor_row_rvalid;
wire i_mreg2store_store_tensor_row_rlast;
wire i_mreg2store_store_tensor_row_rready;
wire [BW_OUTPUT_TENSOR_ROW-1:0] i_mreg2store_store_tensor_row_rdata;

wire i_mreg2_move_wenable;
wire [BW_OUTPUT_TENSOR_ROW-1:0] i_mreg2_move_wdata_list;
wire i_mreg2_move_renable;
wire [BW_OUTPUT_TENSOR_ROW-1:0] i_mreg2_move_rdata_list;
wire i_mreg2_shift_up;
wire i_mreg2_shift_left;
wire i_mreg2_transpose;
wire [BW_OUTPUT_TENSOR_MATRIX-1:0] i_mreg2_all_rdata_list2d;
wire [BW_OUTPUT_TENSOR_ROW-1:0] i_mreg2_upmost_rdata_list1d;

// by hkim
wire end_of_2d_conv;
wire conv_core_valid;
wire [BW_OUTPUT_TENSOR_ROW-1:0] conv_core_out;


////////////
/* logics */
////////////

// not used
assign control_rmx_core_status = 0;
assign control_rmx_clear_finish = 0;
assign control_rmx_log_fifo_wrequest = 0;
assign control_rmx_log_fifo_wdata = 0;
assign control_rmx_input_fifo_rrequest = 0;
assign control_rmx_output_fifo_wrequest = 0;
assign control_rmx_output_fifo_wdata = 0;

assign mi_sstore_tensor_row_rready = 0;
assign mi_sstore_tensor_row_rlast = 0;
assign mi_sstore_tensor_row_rdata = 0;

assign mk_sstore_tensor_row_rready = 0;
assign mk_sstore_tensor_row_rlast = 0;
assign mk_sstore_tensor_row_rdata = 0;

assign mo_sload_tensor_row_wready = 0;

// inst
assign {inst_pad,inst_stride_m1,mo_info,mk_info,mi_info} = control_rmx_inst_fifo_rdata;
assign {mi_inst_num_row_m1, mi_inst_stride_ls3, mi_inst_addr} = mi_info;
assign {mk_inst_num_row_m1, mk_inst_stride_ls3, mk_inst_addr} = mk_info;
assign {mo_inst_num_row_m1, mo_inst_stride_ls3, mo_inst_addr} = mo_info;

assign input_size_m1 = mi_inst_num_row_m1;
assign kernel_size_m1 = mk_inst_num_row_m1;
assign output_size_m1 = mo_inst_num_row_m1;

// state
always@(posedge clk or negedge rstnn)
begin
  if(~rstnn)
    state <= IDLE;
  else
    case(state)
      IDLE:
        if(go_load)
          state <= LOAD;
      LOAD:
        if(go_execute)
          state <= EXECUTE;
      EXECUTE:
        if(go_store)
          state <= STORE;
      STORE:
        if(go_idle)
          state <= IDLE;
    endcase
end

assign go_load = (state==IDLE) & control_rmx_inst_fifo_rready & mi_sinst_wready & mk_sinst_wready & mo_sinst_wready;
assign go_execute = (state==LOAD) & i_load02mreg_load_rready & i_load12mreg_load_rready & i_mreg2store_store_wready;
assign go_store = (state==EXECUTE) & execute_finish;
assign go_idle = (state==STORE) & (~mo_sinst_busy);

// mreg0
DCA_MATRIX_LOAD2MREG
#(
  .MATRIX_SIZE_PARA(INPUT_MATRIX_SIZE),
  .TENSOR_PARA(TENSOR_PARA)
)
i_load02mreg
(
  .clk(clk),
  .rstnn(rstnn),
  .clear(1'b 0),
  .enable(1'b 1),
  .busy(i_load02mreg_busy),

  .load_tensor_row_wvalid(i_load02mreg_load_tensor_row_wvalid),
  .load_tensor_row_wlast(i_load02mreg_load_tensor_row_wlast),
  .load_tensor_row_wdata(i_load02mreg_load_tensor_row_wdata),
  .load_tensor_row_wready(i_load02mreg_load_tensor_row_wready),

  .mreg_move_wenable(i_load02mreg_mreg_move_wenable),
  .mreg_move_wdata_list1d(i_load02mreg_mreg_move_wdata_list1d),

  .loadreg_rready(i_load02mreg_load_rready),        //V250811.hkim
  .loadreg_rrequest(i_load02mreg_load_rrequest)     //V250811.hkim
);
  //V250811.hkim : port name matching : .load_rready(i_load02mreg_load_rready),
  //V250811.hkim : port name matching : .load_rrequest(i_load02mreg_load_rrequest)

assign i_load02mreg_load_tensor_row_wvalid = mi_sload_tensor_row_wvalid;
assign i_load02mreg_load_tensor_row_wlast = mi_sload_tensor_row_wlast;
assign i_load02mreg_load_tensor_row_wdata = mi_sload_tensor_row_wdata;
assign i_load02mreg_load_rrequest = go_store;
assign mi_sload_tensor_row_wready = i_load02mreg_load_tensor_row_wready;

DCA_MATRIX_REGISTER_TYPE3
#(
  .MATRIX_SIZE_PARA(INPUT_MATRIX_SIZE),
  .BW_TENSOR_SCALAR(BW_TENSOR_SCALAR),
  .BW_MOVE_DATA(BW_INPUT_TENSOR_ROW),
  .RESET_VALUE(MREG_RESET_VALUE)
)
i_mreg0
(
  .clk(clk),
  .rstnn(rstnn),

  .move_wenable(i_mreg0_move_wenable),
  .move_wdata_list(i_mreg0_move_wdata_list),
  .move_renable(i_mreg0_move_renable),
  .move_rdata_list(i_mreg0_move_rdata_list),
  
  .shift_up(i_mreg0_shift_up),
  .shift_left(i_mreg0_shift_left),
  .transpose(i_mreg0_transpose),
  
  .all_rdata_list2d(i_mreg0_all_rdata_list2d),
  .upmost_rdata_list1d(i_mreg0_upmost_rdata_list1d)
);

assign i_mreg0_move_wenable = i_load02mreg_mreg_move_wenable;
assign i_mreg0_move_wdata_list = i_load02mreg_mreg_move_wdata_list1d;
assign i_mreg0_move_renable = 0;
assign i_mreg0_shift_up = 0;
assign i_mreg0_shift_left = 0;
assign i_mreg0_transpose = 0;

// mreg1
DCA_MATRIX_LOAD2MREG
#(
  .MATRIX_SIZE_PARA(KERNEL_MATRIX_SIZE),
  .TENSOR_PARA(TENSOR_PARA)
)
i_load12mreg
(
  .clk(clk),
  .rstnn(rstnn),
  .clear(1'b 0),
  .enable(1'b 1),
  .busy(i_load12mreg_busy),

  .load_tensor_row_wvalid(i_load12mreg_load_tensor_row_wvalid),
  .load_tensor_row_wlast(i_load12mreg_load_tensor_row_wlast),
  .load_tensor_row_wdata(i_load12mreg_load_tensor_row_wdata),
  .load_tensor_row_wready(i_load12mreg_load_tensor_row_wready),

  .mreg_move_wenable(i_load12mreg_mreg_move_wenable),
  .mreg_move_wdata_list1d(i_load12mreg_mreg_move_wdata_list1d),

  .loadreg_rready(i_load12mreg_load_rready),        //V250811.hkim
  .loadreg_rrequest(i_load12mreg_load_rrequest)     //V250811.hkim
);
  //V250811.hkim : port name matching : .load_rready(i_load12mreg_load_rready),
  //V250811.hkim : port name matching : .load_rrequest(i_load12mreg_load_rrequest)

assign i_load12mreg_load_tensor_row_wvalid = mk_sload_tensor_row_wvalid;
assign i_load12mreg_load_tensor_row_wlast = mk_sload_tensor_row_wlast;
assign i_load12mreg_load_tensor_row_wdata = mk_sload_tensor_row_wdata;
assign i_load12mreg_load_rrequest = go_store;
assign mk_sload_tensor_row_wready = i_load12mreg_load_tensor_row_wready;

DCA_MATRIX_REGISTER_TYPE3
#(
  .MATRIX_SIZE_PARA(KERNEL_MATRIX_SIZE),
  .BW_TENSOR_SCALAR(BW_TENSOR_SCALAR),
  .BW_MOVE_DATA(BW_KERNEL_TENSOR_ROW),
  .RESET_VALUE(MREG_RESET_VALUE)
)
i_mreg1
(
  .clk(clk),
  .rstnn(rstnn),

  .move_wenable(i_mreg1_move_wenable),
  .move_wdata_list(i_mreg1_move_wdata_list),
  .move_renable(i_mreg1_move_renable),
  .move_rdata_list(i_mreg1_move_rdata_list),
  
  .shift_up(i_mreg1_shift_up),
  .shift_left(i_mreg1_shift_left),
  .transpose(i_mreg1_transpose),
  
  .all_rdata_list2d(i_mreg1_all_rdata_list2d),
  .upmost_rdata_list1d(i_mreg1_upmost_rdata_list1d)
);

assign i_mreg1_move_wenable = i_load12mreg_mreg_move_wenable;
assign i_mreg1_move_wdata_list = i_load12mreg_mreg_move_wdata_list1d;
assign i_mreg1_move_renable = 0;
assign i_mreg1_shift_up = 0;
assign i_mreg1_shift_left = 0;
assign i_mreg1_transpose = 0;

// mreg2
DCA_MATRIX_MREG2STORE
#(
  .MATRIX_SIZE_PARA(OUTPUT_MATRIX_SIZE),
  .BW_TENSOR_SCALAR(BW_TENSOR_SCALAR)
)
i_mreg2store
(
  .clk(clk),
  .rstnn(rstnn),
  .clear(1'b 0),
  .enable(1'b 1),
  .busy(i_mreg2store_busy),

  .storereg_wready(i_mreg2store_store_wready),      // V250811.hkim
  .storereg_wrequest(i_mreg2store_store_wrequest),  // V250811.hkim

  .mreg_move_renable(i_mreg2store_mreg_move_renable),
  .mreg_move_rdata_list1d(i_mreg2store_mreg_move_rdata_list1d),

  .store_tensor_row_rvalid(i_mreg2store_store_tensor_row_rvalid),
  .store_tensor_row_rlast(i_mreg2store_store_tensor_row_rlast),
  .store_tensor_row_rready(i_mreg2store_store_tensor_row_rready),
  .store_tensor_row_rdata(i_mreg2store_store_tensor_row_rdata)
);
  // V250811.hkim : port name matching : .store_wready(i_mreg2store_store_wready),
  // V250811.hkim : port name matching : .store_wrequest(i_mreg2store_store_wrequest),

assign i_mreg2store_store_wrequest = go_store;
assign i_mreg2store_store_tensor_row_rvalid = mo_sstore_tensor_row_rvalid;
assign i_mreg2store_store_tensor_row_rlast = mo_sstore_tensor_row_rlast;
assign mo_sstore_tensor_row_rready = i_mreg2store_store_tensor_row_rready;
assign mo_sstore_tensor_row_rdata = i_mreg2store_store_tensor_row_rdata;

DCA_MATRIX_REGISTER_TYPE3
#(
  .MATRIX_SIZE_PARA(OUTPUT_MATRIX_SIZE),
  .BW_TENSOR_SCALAR(BW_TENSOR_SCALAR),
  .BW_MOVE_DATA(BW_OUTPUT_TENSOR_ROW),
  .RESET_VALUE(MREG_RESET_VALUE)
)
i_mreg2
(
  .clk(clk),
  .rstnn(rstnn),

  .move_wenable(i_mreg2_move_wenable),
  .move_wdata_list(i_mreg2_move_wdata_list),
  .move_renable(i_mreg2_move_renable),
  .move_rdata_list(i_mreg2_move_rdata_list),
  
  .shift_up(i_mreg2_shift_up),
  .shift_left(i_mreg2_shift_left),
  .transpose(i_mreg2_transpose),
  
  .all_rdata_list2d(i_mreg2_all_rdata_list2d),
  .upmost_rdata_list1d(i_mreg2_upmost_rdata_list1d)
);

assign i_mreg2_move_renable = i_mreg2store_mreg_move_renable;
assign i_mreg2store_mreg_move_rdata_list1d = i_mreg2_move_rdata_list;
assign i_mreg2_shift_up = 0;
assign i_mreg2_shift_left = 0;
assign i_mreg2_transpose = 0;

// outputs
assign mi_sinst_wvalid = go_load;
assign mk_sinst_wvalid = go_load;
assign mo_sinst_wvalid = go_load;

assign lsu0_inst_opcode = `DCA_MATRIX_LSU_INST_OPCODE_READ;
assign lsu1_inst_opcode = `DCA_MATRIX_LSU_INST_OPCODE_READ;
assign lsu2_inst_opcode = `DCA_MATRIX_LSU_INST_OPCODE_WRITE;

assign mi_sinst_wdata = {mi_info, lsu0_inst_opcode};
assign mk_sinst_wdata = {mk_info, lsu1_inst_opcode};
assign mo_sinst_wdata = {mo_info, lsu2_inst_opcode};

assign control_rmx_inst_fifo_rrequest = go_store;
assign control_rmx_operation_finish = go_idle;

// execute
// hkim
c2dConvTop
#(
  .IMAGE_BUF_WIDTH          (INPUT_MATRIX_SIZE ),
  .IMAGE_BUF_HEIGHT         (INPUT_MATRIX_SIZE ),
  .IMAGE_BUF_BITSIZE        (BW_TENSOR_SCALAR  ),
  .IMAGE_BUF_WIDTH_BITSIZE  (BW_MATRIX_SIZE    ),
  .IMAGE_BUF_HEIGHT_BITSIZE (BW_MATRIX_SIZE    ),
  .KERNEL_BUF_WIDTH         (KERNEL_MATRIX_SIZE),
  .KERNEL_BUF_HEIGHT        (KERNEL_MATRIX_SIZE),
  .KERNEL_BUF_BITSIZE       (BW_TENSOR_SCALAR  ),
  .KERNEL_BUF_WIDTH_BITSIZE (BW_MATRIX_SIZE    ),
  .KERNEL_BUF_HEIGHT_BITSIZE(BW_MATRIX_SIZE    ),
  .OUTPUT_BUF_WIDTH         (OUTPUT_MATRIX_SIZE),
  .OUTPUT_BUF_BITSIZE       (BW_TENSOR_SCALAR  ),
  .MAX_OUTPUT_NUM           (OUTPUT_MATRIX_SIZE)
)
i_c2dConvTop
(
  .endOfConv2D     ( end_of_2d_conv                       ),
  .convCoreValid   ( conv_core_valid                      ),
  .convCoreOut     ( conv_core_out                        ),
  .kerInBufLdInit  ( go_load                              ),
  .kerInBufLdEn    ( mk_sload_tensor_row_wvalid           ),  // input
  .kerInBufLdEnd   ( mk_sload_tensor_row_wlast            ),  // input
  .kerInBufDataIn  ( mk_sload_tensor_row_wdata            ),  // input
  .imgInBufLdInit  ( go_load                              ),
  .imgInBufLdEn    ( mi_sload_tensor_row_wvalid           ),
  .imgInBufLdEnd   ( mi_sload_tensor_row_wlast            ),
  .imgInBufDataIn  ( mi_sload_tensor_row_wdata            ),
  .numKernelWidth  ( kernel_size_m1 + 1'b1                ),
  .numKernelHeight ( kernel_size_m1 + 1'b1                ),
  .numImageWidth   ( input_size_m1 + 1'b1                 ),
  .numImageHeight  ( input_size_m1 + 1'b1                 ),
  .numOutWidth     ( output_size_m1 + 1'b1                ),
  .numOutHeight    ( output_size_m1 + 1'b1                ),
  .npuStart        ( go_execute                           ),
  .clk             ( clk                                  ),
  .resetB          ( rstnn                                )
);


// debug
`ifdef SIM_ENV

wire [BW_TENSOR_SCALAR-1:0] input_value [INPUT_MATRIX_NUM_COL-1:0][INPUT_MATRIX_NUM_COL-1:0];

generate
for(i=0; i<INPUT_MATRIX_NUM_COL; i=i+1)
begin : i_gen_row
  for(j=0; j<INPUT_MATRIX_NUM_COL; j=j+1)
  begin : i_gen_col
    assign input_value[i][j] = i_mreg0_all_rdata_list2d[(BW_INPUT_TENSOR_ROW*i)+BW_TENSOR_SCALAR*(j+1)-1-:BW_TENSOR_SCALAR];
  end
end
endgenerate

integer p_i, p_j;

always@(state)
begin
  if(state==EXECUTE)
  begin
    $display("[%s]", "dca_matrix_conv2d_mmiox_mlsu.v" );
    $display("%2d %2d %2d", (input_size_m1+1), (kernel_size_m1+1), (output_size_m1+1));
    for(p_i=0; p_i<=input_size_m1; p_i=p_i+1)
    begin
      $display("");
      for(p_j=0; p_j<=input_size_m1; p_j=p_j+1)
        $display(" %d", $signed(input_value[p_i][p_j]));
    end
  end
end

`endif

// i_mreg0_all_rdata_list2d
// i_mreg1_all_rdata_list2d
// by hkim
assign i_mreg2_move_wenable = conv_core_valid;
assign i_mreg2_move_wdata_list = conv_core_out;
assign execute_finish = (state==EXECUTE) & end_of_2d_conv;
//assign i_mreg2_move_wenable = 0; // update
//assign i_mreg2_move_wdata_list = 0; // update
//assign execute_finish = (state==EXECUTE); // update

endmodule
