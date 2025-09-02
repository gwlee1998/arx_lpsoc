// ****************************************************************************
// ****************************************************************************
// Copyright SoC Design Research Group, All rights reservxd.
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
// 2025-08-12
// Kyuseung Han (han@etri.re.kr)
// ****************************************************************************
// ****************************************************************************

`include "ervp_global.vh"

`include "dca_matrix_info.vh"
`include "dca_matrix_lsu_inst.vh"
`include "dca_module_memorymap_offset.vh"




module DCA_MODULE_27
(
  dca_port_20,
  dca_port_18,
  dca_port_07,
  dca_port_06,
  dca_port_23,

  dca_port_02,
  dca_port_35,
  dca_port_04,
  dca_port_00,
  dca_port_30,
  dca_port_05,
  dca_port_09,
  dca_port_32,
  dca_port_15,
  dca_port_11,
  dca_port_10,

  dca_port_13,
  dca_port_19,
  dca_port_31,
  dca_port_14,
  dca_port_28,
  dca_port_27,
  dca_port_16,
  dca_port_25,
  dca_port_21,
  dca_port_24,

  dca_port_33,
  dca_port_34,
  dca_port_08,
  dca_port_12,
  dca_port_17,
  dca_port_03,
  dca_port_26,
  dca_port_22,
  dca_port_29,
  dca_port_01
);




parameter MATRIX_SIZE_PARA = 8;
parameter TENSOR_PARA = 0;

`include "dca_matrix_dim_util.vb"
`include "dca_matrix_dim_lpara.vb"
`include "dca_include_0.vh"
`include "dca_tensor_scalar_lpara.vb"
`include "dca_tensor_dim_lpara.vb"

input wire dca_port_20;
input wire dca_port_18;
input wire dca_port_07;
input wire dca_port_06;
output wire dca_port_23;

localparam  DCA_LPARA_1 = `BW_DCA_MRU_STATUS;
localparam  DCA_LPARA_3 = `BW_DCA_MRU_LOG;
localparam  DCA_LPARA_0 = `BW_DCA_MRU_INST;
localparam  DCA_LPARA_4 = `BW_DCA_MRU_INPUT;

input wire dca_port_02;
input wire [DCA_LPARA_0-1:0] dca_port_35;
output wire dca_port_04;
output wire dca_port_00;
input wire dca_port_30;
output wire dca_port_05;
output wire [DCA_LPARA_3-1:0] dca_port_09;
input wire dca_port_32;
input wire [DCA_LPARA_4-1:0] dca_port_15;
output wire dca_port_11;
output wire [DCA_LPARA_1-1:0] dca_port_10;

input wire dca_port_13;
output wire dca_port_19;
output wire [`BW_DCA_MATRIX_LSU_INST-1:0] dca_port_31;
input wire dca_port_14;
input wire dca_port_28;
input wire dca_port_27;

output wire dca_port_16;
input wire dca_port_25;
input wire dca_port_21;
input wire [BW_TENSOR_ROW-1:0] dca_port_24;

input wire dca_port_33;
output wire dca_port_34;
output wire [`BW_DCA_MATRIX_LSU_INST-1:0] dca_port_08;
input wire dca_port_12;
input wire dca_port_17;
input wire dca_port_03;

input wire dca_port_26;
output wire dca_port_22;
input wire dca_port_29;
output wire [BW_TENSOR_ROW-1:0] dca_port_01;

localparam  DCA_LPARA_6 = BW_BLOCKED_STEP_INST;
localparam  DCA_LPARA_7 = 4;

wire dca_signal_01;
wire dca_signal_16;
wire [DCA_LPARA_6-1:0] dca_signal_06;
wire dca_signal_22;
wire dca_signal_19;
wire [DCA_LPARA_6-1:0] dca_signal_05;

wire dca_signal_28;

localparam  DCA_LPARA_5 = BW_TENSOR_ROW;
localparam  DCA_LPARA_2 = TENSOR_ZERO;

wire dca_signal_25;
wire [DCA_LPARA_5-1:0] dca_signal_13;
wire dca_signal_03;
wire [DCA_LPARA_5-1:0] dca_signal_04;
wire dca_signal_00;
wire dca_signal_23;
wire dca_signal_11;
wire [BW_TENSOR_MATRIX-1:0] dca_signal_09;
wire [BW_TENSOR_ROW-1:0] dca_signal_08;

wire dca_signal_17;
wire dca_signal_20;
wire dca_signal_29;

wire dca_signal_27;
wire dca_signal_02;
wire [DCA_LPARA_5-1:0] dca_signal_15;
wire dca_signal_30;
wire [DCA_LPARA_5-1:0] dca_signal_07;
wire [MATRIX_NUM_ELEMENT-1:0] dca_signal_21;
wire [BW_TENSOR_MATRIX-1:0] dca_signal_18;
wire [BW_TENSOR_MATRIX-1:0] dca_signal_26;

wire dca_signal_12;
wire dca_signal_24;
wire dca_signal_10;

wire dca_signal_14;

ERVP_FIFO
#(
  .BW_DATA(DCA_LPARA_6),
  .DEPTH(DCA_LPARA_7)
)
i_dca_instance_0
(
	.clk(dca_port_20),
  .rstnn(dca_port_18),
	.enable(dca_port_06),
  .clear(dca_port_07),
	.wready(dca_signal_01),
	.wfull(),
	.wrequest(dca_signal_16),
	.wdata(dca_signal_06),
	.wnum(),
	.rready(dca_signal_22),
	.rempty(),
	.rrequest(dca_signal_19),
	.rdata(dca_signal_05),
	.rnum()
);

DCA_MODULE_14
#(
  .MATRIX_SIZE_PARA(MATRIX_SIZE_PARA)
)
i_dca_instance_3
(
	.dca_port_12(dca_port_20),
	.dca_port_13(dca_port_18),
  .dca_port_05(dca_port_07),
  .dca_port_08(dca_port_06),
  .dca_port_14(),

  .dca_port_01(dca_port_02),
  .dca_port_16(dca_port_35),
  .dca_port_10(dca_port_04),
  .dca_port_03(dca_signal_28),

  .dca_port_11(dca_signal_01),
  .dca_port_15(dca_signal_16),
  .dca_port_04(dca_signal_06),
  .dca_port_09(dca_port_14),
  .dca_port_06(dca_port_19),
  .dca_port_17(dca_port_31),
  .dca_port_02(dca_port_12),
  .dca_port_07(dca_port_34),
  .dca_port_00(dca_port_08)
);

DCA_MATRIX_REGISTER_TYPE3
#(
  .MATRIX_SIZE_PARA(MATRIX_SIZE_PARA),
  .BW_TENSOR_SCALAR(BW_TENSOR_SCALAR),
  .BW_MOVE_DATA(DCA_LPARA_5),
  .RESET_VALUE(DCA_LPARA_2)
)
i_dca_instance_1
(
  .clk(dca_port_20),
  .rstnn(dca_port_18),

  .move_wenable(dca_signal_25),
  .move_wdata_list(dca_signal_13),
  .move_renable(dca_signal_03),
  .move_rdata_list(dca_signal_04),
  
  .shift_up(dca_signal_00),
  .shift_left(dca_signal_23),
  .transpose(dca_signal_11),
  
  .all_rdata_list2d(dca_signal_09),
  .upmost_rdata_list1d(dca_signal_08)
);

assign dca_signal_03 = 0;
assign dca_signal_00 = 0;
assign dca_signal_23 = 0;
assign dca_signal_11 = 0;

DCA_MATRIX_LOAD2MREG
#(
  .MATRIX_SIZE_PARA(MATRIX_SIZE_PARA),
  .TENSOR_PARA(TENSOR_PARA)
)
i_dca_instance_6
(
  .clk(dca_port_20),
  .rstnn(dca_port_18),
  .clear(dca_port_07),
  .enable(dca_port_06),
  .busy(dca_signal_17),

  .load_tensor_row_wready(dca_port_16),
  .load_tensor_row_wvalid(dca_port_25),
  .load_tensor_row_wlast(dca_port_21),
  .load_tensor_row_wdata(dca_port_24),

  .mreg_move_wenable(dca_signal_25),
  .mreg_move_wdata_list1d(dca_signal_13),

  .loadreg_rready(dca_signal_20),
  .loadreg_rrequest(dca_signal_29)
);

DCA_MATRIX_REGISTER_TYPE5
#(
  .MATRIX_SIZE_PARA(MATRIX_SIZE_PARA),
  .BW_TENSOR_SCALAR(BW_TENSOR_SCALAR),
  .RESET_VALUE(DCA_LPARA_2)
)
i_dca_instance_4
(
  .clk(dca_port_20),
  .rstnn(dca_port_18),

  .init(dca_signal_27),

  .move_wenable(dca_signal_02),
  .move_wdata_list(dca_signal_15),
  .move_renable(dca_signal_30),
  .move_rdata_list(dca_signal_07),

  .all_wenable_list2d(dca_signal_21),
  .all_wdata_list2d(dca_signal_18),
  .all_rdata_list2d(dca_signal_26)
);

assign dca_signal_27 = 0;
assign dca_signal_02 = 0;
assign dca_signal_15 = 0;

DCA_MATRIX_MREG2STORE
#(
  .MATRIX_SIZE_PARA(MATRIX_SIZE_PARA),
  .BW_TENSOR_SCALAR(BW_TENSOR_SCALAR)
)
i_dca_instance_5
(
  .clk(dca_port_20),
  .rstnn(dca_port_18),
  .clear(dca_port_07),
  .enable(dca_port_06),
  .busy(dca_signal_12),

  .storereg_wready(dca_signal_24),
  .storereg_wrequest(dca_signal_10),

  .mreg_move_renable(dca_signal_30),
  .mreg_move_rdata_list1d(dca_signal_07),

  .store_tensor_row_rvalid(dca_port_26),
  .store_tensor_row_rready(dca_port_22),
  .store_tensor_row_rlast(dca_port_29),
  .store_tensor_row_rdata(dca_port_01)
);

DCA_MODULE_15
#(
  .MATRIX_SIZE_PARA(MATRIX_SIZE_PARA),
  .TENSOR_PARA(TENSOR_PARA)
)
i_dca_instance_2
(
  .dca_port_11(dca_port_20),
  .dca_port_00(dca_port_18),
  .dca_port_20(dca_port_07),
  .dca_port_19(dca_port_06),
  .dca_port_13(dca_signal_14),

  .dca_port_12(dca_port_32),
  .dca_port_03(dca_port_15),
  .dca_port_05(dca_port_11),

  .dca_port_17(dca_signal_22),
  .dca_port_18(dca_signal_05),
  .dca_port_01(dca_signal_19),

  .dca_port_10(dca_signal_20),
  .dca_port_04(dca_signal_29),
  .dca_port_15(dca_signal_24),
  .dca_port_06(dca_signal_10),
  .dca_port_07(dca_port_03),

  .dca_port_14(dca_signal_09),
  .dca_port_02(dca_signal_21),
  .dca_port_09(dca_signal_18),

  .dca_port_16(dca_signal_28),
  .dca_port_08(dca_port_00)
);

assign dca_port_05 = 0;
assign dca_port_09 = 0;
assign dca_port_10 = 0;
assign dca_port_23 = dca_port_02 | dca_signal_22 | dca_signal_14;

endmodule
