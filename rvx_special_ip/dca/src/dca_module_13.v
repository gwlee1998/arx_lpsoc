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




module DCA_MODULE_13
(
  dca_port_24,
  dca_port_21,
  dca_port_43,
  dca_port_36,
  dca_port_22,

  dca_port_03,
  dca_port_25,
  dca_port_40,
  dca_port_46,
  dca_port_15,
  dca_port_44,
  dca_port_19,
  dca_port_07,

  dca_port_10,
  dca_port_08,
  dca_port_29,
  dca_port_37,
  dca_port_38,
  dca_port_12,
  dca_port_11,
  dca_port_04,
  dca_port_13,
  dca_port_31,

  dca_port_02,
  dca_port_20,
  dca_port_05,
  dca_port_34,
  dca_port_30,
  dca_port_41,
  dca_port_32,
  dca_port_42,
  dca_port_39,
  dca_port_16,

  dca_port_18,
  dca_port_35,
  dca_port_45,
  dca_port_09,
  dca_port_23,
  dca_port_17,
  dca_port_28,
  dca_port_26,
  dca_port_00,
  dca_port_33,
  dca_port_06,
  dca_port_27,
  dca_port_14,
  dca_port_01
);




parameter MATRIX_SIZE_PARA = 8;
parameter TENSOR_PARA = 0;

`include "dca_matrix_dim_util.vb"
`include "dca_matrix_dim_lpara.vb"
`include "dca_include_1.vh"
`include "dca_tensor_scalar_lpara.vb"
`include "dca_tensor_dim_lpara.vb"

input wire dca_port_24;
input wire dca_port_21;
input wire dca_port_43;
input wire dca_port_36;
output wire dca_port_22;

localparam  DCA_LPARA_5 = `BW_DCA_NEUGEMM_INST;
localparam  DCA_LPARA_6 = `BW_DCA_NEUGEMM_LOG;
localparam  DCA_LPARA_4 = `BW_DCA_NEUGEMM_STATUS;

input wire dca_port_03;
input wire [DCA_LPARA_5-1:0] dca_port_25;
output wire dca_port_40;
output wire dca_port_46;
input wire dca_port_15;
output wire dca_port_44;
output wire [DCA_LPARA_6-1:0] dca_port_19;
output wire [DCA_LPARA_4-1:0] dca_port_07;

input wire dca_port_10;
output wire dca_port_08;
output wire [`BW_DCA_MATRIX_LSU_INST-1:0] dca_port_29;
input wire dca_port_37;
input wire dca_port_38;
input wire dca_port_12;

output wire dca_port_11;
input wire dca_port_04;
input wire dca_port_13;
input wire [BW_TENSOR_ROW-1:0] dca_port_31;

input wire dca_port_02;
output wire dca_port_20;
output wire [`BW_DCA_MATRIX_LSU_INST-1:0] dca_port_05;
input wire dca_port_34;
input wire dca_port_30;
input wire dca_port_41;

output wire dca_port_32;
input wire dca_port_42;
input wire dca_port_39;
input wire [BW_TENSOR_ROW-1:0] dca_port_16;

input wire dca_port_18;
output wire dca_port_35;
output wire [`BW_DCA_MATRIX_LSU_INST-1:0] dca_port_45;
input wire dca_port_09;
input wire dca_port_23;
input wire dca_port_17;

output wire dca_port_28;
input wire dca_port_26;
input wire dca_port_00;
input wire [BW_TENSOR_ROW-1:0] dca_port_33;
input wire dca_port_06;
output wire dca_port_27;
input wire dca_port_14;
output wire [BW_TENSOR_ROW-1:0] dca_port_01;

localparam  DCA_LPARA_2 = BW_BLOCKED_STEP_INST;
localparam  DCA_LPARA_0 = 4;

wire dca_signal_39;
wire dca_signal_09;
wire [DCA_LPARA_2-1:0] dca_signal_52;
wire dca_signal_49;
wire dca_signal_42;
wire [DCA_LPARA_2-1:0] dca_signal_34;

localparam  DCA_LPARA_3 = BW_TENSOR_ROW;
localparam  DCA_LPARA_1 = TENSOR_ZERO;

wire dca_signal_38;
wire [DCA_LPARA_3-1:0] dca_signal_43;
wire dca_signal_35;
wire [DCA_LPARA_3-1:0] dca_signal_05;
wire dca_signal_55;
wire dca_signal_23;
wire dca_signal_61;
wire [BW_TENSOR_MATRIX-1:0] dca_signal_24;
wire [BW_TENSOR_ROW-1:0] dca_signal_18;

wire dca_signal_15;
wire dca_signal_46;
wire dca_signal_13;

wire dca_signal_57;
wire [DCA_LPARA_3-1:0] dca_signal_50;
wire dca_signal_53;
wire [DCA_LPARA_3-1:0] dca_signal_40;
wire dca_signal_01;
wire dca_signal_02;
wire dca_signal_21;
wire [BW_TENSOR_MATRIX-1:0] dca_signal_36;
wire [BW_TENSOR_ROW-1:0] dca_signal_44;

wire dca_signal_30;
wire dca_signal_14;
wire dca_signal_07;

wire dca_signal_11;
wire dca_signal_33;
wire [DCA_LPARA_3-1:0] dca_signal_06;
wire dca_signal_00;
wire [DCA_LPARA_3-1:0] dca_signal_10;
wire [MATRIX_NUM_ELEMENT-1:0] dca_signal_22;
wire [BW_TENSOR_MATRIX-1:0] dca_signal_32;
wire [BW_TENSOR_MATRIX-1:0] dca_signal_31;

wire dca_signal_63;
wire dca_signal_59;
wire dca_signal_56;

wire dca_signal_08;
wire dca_signal_48;
wire dca_signal_16;

wire dca_signal_03;
wire dca_signal_12;
wire [DCA_LPARA_3-1:0] dca_signal_41;
wire dca_signal_45;
wire [DCA_LPARA_3-1:0] dca_signal_29;
wire [MATRIX_NUM_ELEMENT-1:0] dca_signal_47;
wire [BW_TENSOR_MATRIX-1:0] dca_signal_26;
wire [BW_TENSOR_MATRIX-1:0] dca_signal_51;

wire dca_signal_25;
wire [BW_TENSOR_SCALAR-1:0] dca_signal_62;
wire [BW_TENSOR_MATRIX-1:0] dca_signal_37;
wire [BW_TENSOR_ROW-1:0] dca_signal_04;

wire dca_signal_54;
wire dca_signal_27;
wire [BW_BLOCKED_STEP_INST-1:0] dca_signal_28;
wire dca_signal_17;
wire dca_signal_19;
wire dca_signal_60;
wire dca_signal_58;

wire dca_signal_20;

ERVP_FIFO
#(
  .BW_DATA(DCA_LPARA_2),
  .DEPTH(DCA_LPARA_0)
)
i_dca_instance_11
(
	.clk(dca_port_24),
  .rstnn(dca_port_21),
	.enable(dca_port_36),
  .clear(dca_port_43),
	.wready(dca_signal_39),
	.wfull(),
	.wrequest(dca_signal_09),
	.wdata(dca_signal_52),
	.wnum(),
	.rready(dca_signal_49),
	.rempty(),
	.rrequest(dca_signal_42),
	.rdata(dca_signal_34),
	.rnum()
);

DCA_MODULE_10
#(
  .MATRIX_SIZE_PARA(MATRIX_SIZE_PARA)
)
i_dca_instance_00
(
	.dca_port_06(dca_port_24),
	.dca_port_13(dca_port_21),
  .dca_port_16(dca_port_43),
  .dca_port_18(dca_port_36),
  .dca_port_09(),

  .dca_port_15(dca_port_03),
  .dca_port_05(dca_port_25),
  .dca_port_17(dca_port_40),
  .dca_port_07(dca_signal_19),
  .dca_port_03(dca_signal_60),
  .dca_port_14(dca_signal_58),

  .dca_port_08(dca_signal_39),
  .dca_port_19(dca_signal_09),
  .dca_port_01(dca_signal_52),
  .dca_port_21(dca_port_37),
  .dca_port_10(dca_port_08),
  .dca_port_00(dca_port_29),
  .dca_port_11(dca_port_34),
  .dca_port_02(dca_port_20),
  .dca_port_20(dca_port_05),
  .dca_port_12(dca_port_09),
  .dca_port_04(dca_port_35),
  .dca_port_22(dca_port_45)
);

DCA_MATRIX_REGISTER_TYPE3
#(
  .MATRIX_SIZE_PARA(MATRIX_SIZE_PARA),
  .BW_TENSOR_SCALAR(BW_TENSOR_SCALAR),
  .BW_MOVE_DATA(DCA_LPARA_3),
  .RESET_VALUE(DCA_LPARA_1)
)
i_dca_instance_07
(
  .clk(dca_port_24),
  .rstnn(dca_port_21),

  .move_wenable(dca_signal_38),
  .move_wdata_list(dca_signal_43),
  .move_renable(dca_signal_35),
  .move_rdata_list(dca_signal_05),
  
  .shift_up(dca_signal_55),
  .shift_left(dca_signal_23),
  .transpose(dca_signal_61),
  
  .all_rdata_list2d(dca_signal_24),
  .upmost_rdata_list1d(dca_signal_18)
);

assign dca_signal_35 = 0;
assign dca_signal_61 = 0;

DCA_MATRIX_LOAD2MREG
#(
  .MATRIX_SIZE_PARA(MATRIX_SIZE_PARA),
  .TENSOR_PARA(TENSOR_PARA)
)
i_dca_instance_03
(
  .clk(dca_port_24),
  .rstnn(dca_port_21),
  .clear(dca_port_43),
  .enable(dca_port_36),
  .busy(dca_signal_15),

  .load_tensor_row_wready(dca_port_11),
  .load_tensor_row_wvalid(dca_port_04),
  .load_tensor_row_wlast(dca_port_13),
  .load_tensor_row_wdata(dca_port_31),

  .mreg_move_wenable(dca_signal_38),
  .mreg_move_wdata_list1d(dca_signal_43),

  .loadreg_rready(dca_signal_46),
  .loadreg_rrequest(dca_signal_13)
);

DCA_MATRIX_REGISTER_TYPE3
#(
  .MATRIX_SIZE_PARA(MATRIX_SIZE_PARA),
  .BW_TENSOR_SCALAR(BW_TENSOR_SCALAR),
  .BW_MOVE_DATA(DCA_LPARA_3),
  .RESET_VALUE(DCA_LPARA_1)
)
i_dca_instance_06
(
  .clk(dca_port_24),
  .rstnn(dca_port_21),

  .move_wenable(dca_signal_57),
  .move_wdata_list(dca_signal_50),
  .move_renable(dca_signal_53),
  .move_rdata_list(dca_signal_40),
  
  .shift_up(dca_signal_01),
  .shift_left(dca_signal_02),
  .transpose(dca_signal_21),
  
  .all_rdata_list2d(dca_signal_36),
  .upmost_rdata_list1d(dca_signal_44)
);

assign dca_signal_53 =0;
assign dca_signal_21 = 0;

DCA_MATRIX_LOAD2MREG
#(
  .MATRIX_SIZE_PARA(MATRIX_SIZE_PARA),
  .TENSOR_PARA(TENSOR_PARA)
)
i_dca_instance_12
(
  .clk(dca_port_24),
  .rstnn(dca_port_21),
  .clear(dca_port_43),
  .enable(dca_port_36),
  .busy(dca_signal_30),

  .load_tensor_row_wready(dca_port_32),
  .load_tensor_row_wvalid(dca_port_42),
  .load_tensor_row_wlast(dca_port_39),
  .load_tensor_row_wdata(dca_port_16),

  .mreg_move_wenable(dca_signal_57),
  .mreg_move_wdata_list1d(dca_signal_50),

  .loadreg_rready(dca_signal_14),
  .loadreg_rrequest(dca_signal_07)
);

DCA_MATRIX_REGISTER_TYPE5
#(
  .MATRIX_SIZE_PARA(MATRIX_SIZE_PARA),
  .BW_TENSOR_SCALAR(BW_TENSOR_SCALAR),
  .RESET_VALUE(DCA_LPARA_1)
)
i_dca_instance_02
(
  .clk(dca_port_24),
  .rstnn(dca_port_21),

  .init(dca_signal_11),

  .move_wenable(dca_signal_33),
  .move_wdata_list(dca_signal_06),
  .move_renable(dca_signal_00),
  .move_rdata_list(dca_signal_10),

  .all_wenable_list2d(dca_signal_22),
  .all_wdata_list2d(dca_signal_32),
  .all_rdata_list2d(dca_signal_31)
);

DCA_MATRIX_LOAD2MREG
#(
  .MATRIX_SIZE_PARA(MATRIX_SIZE_PARA),
  .TENSOR_PARA(TENSOR_PARA)
)
i_dca_instance_05
(
  .clk(dca_port_24),
  .rstnn(dca_port_21),
  .clear(dca_port_43),
  .enable(dca_port_36),
  .busy(dca_signal_63),

  .load_tensor_row_wready(dca_port_28),
  .load_tensor_row_wvalid(dca_port_26),
  .load_tensor_row_wlast(dca_port_00),
  .load_tensor_row_wdata(dca_port_33),

  .mreg_move_wenable(dca_signal_33),
  .mreg_move_wdata_list1d(dca_signal_06),

  .loadreg_rready(dca_signal_59),
  .loadreg_rrequest(dca_signal_56)
);

DCA_MATRIX_MREG2STORE
#(
  .MATRIX_SIZE_PARA(MATRIX_SIZE_PARA),
  .BW_TENSOR_SCALAR(BW_TENSOR_SCALAR)
)
i_dca_instance_01
(
  .clk(dca_port_24),
  .rstnn(dca_port_21),
  .clear(dca_port_43),
  .enable(dca_port_36),
  .busy(dca_signal_08),

  .storereg_wready(dca_signal_48),
  .storereg_wrequest(dca_signal_16),

  .mreg_move_renable(dca_signal_00),
  .mreg_move_rdata_list1d(dca_signal_10),

  .store_tensor_row_rvalid(dca_port_06),
  .store_tensor_row_rready(dca_port_27),
  .store_tensor_row_rlast(dca_port_14),
  .store_tensor_row_rdata(dca_port_01)
);

DCA_MATRIX_REGISTER_TYPE5
#(
  .MATRIX_SIZE_PARA(MATRIX_SIZE_PARA),
  .BW_TENSOR_SCALAR(BW_TENSOR_SCALAR),
  .RESET_VALUE(DCA_LPARA_1)
)
i_dca_instance_09
(
  .clk(dca_port_24),
  .rstnn(dca_port_21),

  .init(dca_signal_03),

  .move_wenable(dca_signal_12),
  .move_wdata_list(dca_signal_41),
  .move_renable(dca_signal_45),
  .move_rdata_list(dca_signal_29),

  .all_wenable_list2d(dca_signal_47),
  .all_wdata_list2d(dca_signal_26),
  .all_rdata_list2d(dca_signal_51)
);

assign dca_signal_12 = 0;
assign dca_signal_41 = 0;
assign dca_signal_45 = 0;

DCA_MATRIX_REGISTER_CONSTANT
#(
  .MATRIX_SIZE_PARA(MATRIX_SIZE_PARA),
  .BW_TENSOR_SCALAR(BW_TENSOR_SCALAR)
)
i_dca_instance_04
(
  .clk(dca_port_24),
  .rstnn(dca_port_21),

  .scalar_wenable(dca_signal_25),
  .scalar_wdata(dca_signal_62),
  
  .all_rdata_list2d(dca_signal_37),
  .upmost_rdata_list1d(dca_signal_04)
);

assign dca_signal_62 = dca_signal_44[BW_TENSOR_SCALAR-1-:BW_TENSOR_SCALAR];

DCA_MODULE_12
#(
  .MATRIX_SIZE_PARA(MATRIX_SIZE_PARA),
  .TENSOR_PARA(TENSOR_PARA)
)
i_dca_instance_08
(
  .dca_port_17(dca_port_24),
  .dca_port_00(dca_port_21),
  .dca_port_21(dca_port_43),
  .dca_port_07(dca_port_36),
  .dca_port_15(dca_signal_54),

  .dca_port_10(dca_signal_27),
  .dca_port_05(dca_signal_28),
  .dca_port_11(dca_signal_17),

  .dca_port_20(dca_signal_24),
  .dca_port_04(dca_signal_55),
  .dca_port_13(dca_signal_23),

  .dca_port_18(dca_signal_36),
  .dca_port_16(dca_signal_01),
  .dca_port_19(dca_signal_02),

  .dca_port_01(dca_signal_25),
  .dca_port_06(dca_signal_37),

  .dca_port_22(dca_signal_31),
  .dca_port_09(dca_signal_11),
  .dca_port_02(dca_signal_22),
  .dca_port_03(dca_signal_32),

  .dca_port_12(dca_signal_51),
  .dca_port_08(dca_signal_47),
  .dca_port_14(dca_signal_26)
);

DCA_MODULE_20
#(
  .MATRIX_SIZE_PARA(MATRIX_SIZE_PARA)
)
i_dca_instance_10
(
  .dca_port_24(dca_port_24),
  .dca_port_23(dca_port_21),
  .dca_port_21(dca_port_43),
  .dca_port_10(dca_port_36),
  .dca_port_09(dca_signal_20),

  .dca_port_11(dca_signal_49),
  .dca_port_05(dca_signal_34),
  .dca_port_04(dca_signal_42),

  .dca_port_19(dca_signal_46),
  .dca_port_13(dca_signal_13),
  .dca_port_14(dca_signal_14),
  .dca_port_07(dca_signal_07),
  .dca_port_16(dca_signal_59),
  .dca_port_00(dca_signal_56),
  .dca_port_17(dca_signal_48),
  .dca_port_15(dca_signal_48),
  .dca_port_18(dca_signal_16),
  .dca_port_03(dca_port_17),

  .dca_port_02(dca_signal_27),
  .dca_port_22(dca_signal_28),
  .dca_port_20(dca_signal_17),

  .dca_port_12(dca_signal_19),
  .dca_port_08(dca_signal_60),
  .dca_port_01(dca_signal_58),
  .dca_port_06(dca_port_46)
);

assign dca_port_44 = 0;
assign dca_port_19 = 0;
assign dca_port_07 = 0;
assign dca_port_22 = dca_port_03 | dca_signal_49 | dca_signal_20;

endmodule
