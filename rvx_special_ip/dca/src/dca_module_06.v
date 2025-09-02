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




module DCA_MODULE_06
(
  dca_port_20,
  dca_port_41,
  dca_port_42,
  dca_port_33,
  dca_port_23,

  dca_port_14,
  dca_port_00,
  dca_port_30,
  dca_port_06,
  dca_port_18,
  dca_port_19,
  dca_port_16,
  dca_port_36,

  dca_port_24,
  dca_port_04,
  dca_port_21,
  dca_port_22,
  dca_port_15,
  dca_port_32,
  dca_port_09,
  dca_port_08,
  dca_port_07,
  dca_port_43,

  dca_port_02,
  dca_port_05,
  dca_port_38,
  dca_port_25,
  dca_port_40,
  dca_port_12,
  dca_port_17,
  dca_port_37,
  dca_port_10,
  dca_port_39,

  dca_port_46,
  dca_port_01,
  dca_port_29,
  dca_port_31,
  dca_port_27,
  dca_port_45,
  dca_port_26,
  dca_port_03,
  dca_port_28,
  dca_port_35,
  dca_port_34,
  dca_port_11,
  dca_port_13,
  dca_port_44
);




parameter MATRIX_SIZE_PARA = 8;
parameter TENSOR_PARA = 0;

`include "dca_matrix_dim_util.vb"
`include "dca_matrix_dim_lpara.vb"
`include "dca_include_2.vh"
`include "dca_tensor_scalar_lpara.vb"
`include "dca_tensor_dim_lpara.vb"

input wire dca_port_20;
input wire dca_port_41;
input wire dca_port_42;
input wire dca_port_33;
output wire dca_port_23;

localparam  DCA_LPARA_2 = `BW_DCA_MATRIX_MAC_INST;
localparam  DCA_LPARA_3 = `BW_DCA_MATRIX_MAC_LOG;
localparam  DCA_LPARA_6 = `BW_DCA_MATRIX_MAC_STATUS;

input wire dca_port_14;
input wire [DCA_LPARA_2-1:0] dca_port_00;
output wire dca_port_30;
output wire dca_port_06;
input wire dca_port_18;
output wire dca_port_19;
output wire [DCA_LPARA_3-1:0] dca_port_16;
output wire [DCA_LPARA_6-1:0] dca_port_36;

input wire dca_port_24;
output wire dca_port_04;
output wire [`BW_DCA_MATRIX_LSU_INST-1:0] dca_port_21;
input wire dca_port_22;
input wire dca_port_15;
input wire dca_port_32;

output wire dca_port_09;
input wire dca_port_08;
input wire dca_port_07;
input wire [BW_TENSOR_ROW-1:0] dca_port_43;

input wire dca_port_02;
output wire dca_port_05;
output wire [`BW_DCA_MATRIX_LSU_INST-1:0] dca_port_38;
input wire dca_port_25;
input wire dca_port_40;
input wire dca_port_12;

output wire dca_port_17;
input wire dca_port_37;
input wire dca_port_10;
input wire [BW_TENSOR_ROW-1:0] dca_port_39;

input wire dca_port_46;
output wire dca_port_01;
output wire [`BW_DCA_MATRIX_LSU_INST-1:0] dca_port_29;
input wire dca_port_31;
input wire dca_port_27;
input wire dca_port_45;

output wire dca_port_26;
input wire dca_port_03;
input wire dca_port_28;
input wire [BW_TENSOR_ROW-1:0] dca_port_35;
input wire dca_port_34;
output wire dca_port_11;
input wire dca_port_13;
output wire [BW_TENSOR_ROW-1:0] dca_port_44;

localparam  DCA_LPARA_1 = BW_BLOCKED_STEP_INST;
localparam  DCA_LPARA_4 = 4;

wire dca_signal_24;
wire dca_signal_08;
wire [DCA_LPARA_1-1:0] dca_signal_39;
wire dca_signal_09;
wire dca_signal_13;
wire [DCA_LPARA_1-1:0] dca_signal_46;

localparam  DCA_LPARA_0 = BW_TENSOR_ROW;
localparam  DCA_LPARA_5 = TENSOR_ZERO;

wire dca_signal_19;
wire [DCA_LPARA_0-1:0] dca_signal_55;
wire dca_signal_57;
wire [DCA_LPARA_0-1:0] dca_signal_29;
wire dca_signal_02;
wire dca_signal_50;
wire dca_signal_11;
wire [BW_TENSOR_MATRIX-1:0] dca_signal_30;
wire [BW_TENSOR_ROW-1:0] dca_signal_53;

wire dca_signal_10;
wire dca_signal_35;
wire dca_signal_22;

wire dca_signal_59;
wire [DCA_LPARA_0-1:0] dca_signal_07;
wire dca_signal_25;
wire [DCA_LPARA_0-1:0] dca_signal_47;
wire dca_signal_20;
wire dca_signal_52;
wire dca_signal_06;
wire [BW_TENSOR_MATRIX-1:0] dca_signal_54;
wire [BW_TENSOR_ROW-1:0] dca_signal_31;

wire dca_signal_04;
wire dca_signal_44;
wire dca_signal_26;

wire dca_signal_05;
wire dca_signal_63;
wire [DCA_LPARA_0-1:0] dca_signal_14;
wire dca_signal_00;
wire [DCA_LPARA_0-1:0] dca_signal_48;
wire [MATRIX_NUM_ELEMENT-1:0] dca_signal_01;
wire [BW_TENSOR_MATRIX-1:0] dca_signal_15;
wire [BW_TENSOR_MATRIX-1:0] dca_signal_33;

wire dca_signal_18;
wire dca_signal_49;
wire dca_signal_28;

wire dca_signal_23;
wire dca_signal_34;
wire dca_signal_32;

wire dca_signal_42;
wire dca_signal_38;
wire [DCA_LPARA_0-1:0] dca_signal_03;
wire dca_signal_60;
wire [DCA_LPARA_0-1:0] dca_signal_45;
wire [MATRIX_NUM_ELEMENT-1:0] dca_signal_17;
wire [BW_TENSOR_MATRIX-1:0] dca_signal_40;
wire [BW_TENSOR_MATRIX-1:0] dca_signal_56;

wire dca_signal_27;
wire [BW_TENSOR_SCALAR-1:0] dca_signal_12;
wire [BW_TENSOR_MATRIX-1:0] dca_signal_16;
wire [BW_TENSOR_ROW-1:0] dca_signal_36;

wire dca_signal_21;
wire dca_signal_62;
wire [BW_BLOCKED_STEP_INST-1:0] dca_signal_37;
wire dca_signal_41;
wire dca_signal_61;
wire dca_signal_51;
wire dca_signal_58;

wire dca_signal_43;

ERVP_FIFO
#(
  .BW_DATA(DCA_LPARA_1),
  .DEPTH(DCA_LPARA_4)
)
i_dca_instance_00
(
	.clk(dca_port_20),
  .rstnn(dca_port_41),
	.enable(dca_port_33),
  .clear(dca_port_42),
	.wready(dca_signal_24),
	.wfull(),
	.wrequest(dca_signal_08),
	.wdata(dca_signal_39),
	.wnum(),
	.rready(dca_signal_09),
	.rempty(),
	.rrequest(dca_signal_13),
	.rdata(dca_signal_46),
	.rnum()
);

DCA_MODULE_26
#(
  .MATRIX_SIZE_PARA(MATRIX_SIZE_PARA)
)
i_dca_instance_06
(
	.dca_port_05(dca_port_20),
	.dca_port_17(dca_port_41),
  .dca_port_02(dca_port_42),
  .dca_port_03(dca_port_33),
  .dca_port_04(),

  .dca_port_19(dca_port_14),
  .dca_port_10(dca_port_00),
  .dca_port_09(dca_port_30),
  .dca_port_00(dca_signal_61),
  .dca_port_21(dca_signal_51),
  .dca_port_14(dca_signal_58),

  .dca_port_13(dca_signal_24),
  .dca_port_15(dca_signal_08),
  .dca_port_12(dca_signal_39),
  .dca_port_06(dca_port_22),
  .dca_port_18(dca_port_04),
  .dca_port_16(dca_port_21),
  .dca_port_08(dca_port_25),
  .dca_port_11(dca_port_05),
  .dca_port_22(dca_port_38),
  .dca_port_20(dca_port_31),
  .dca_port_01(dca_port_01),
  .dca_port_07(dca_port_29)
);

DCA_MATRIX_REGISTER_TYPE3
#(
  .MATRIX_SIZE_PARA(MATRIX_SIZE_PARA),
  .BW_TENSOR_SCALAR(BW_TENSOR_SCALAR),
  .BW_MOVE_DATA(DCA_LPARA_0),
  .RESET_VALUE(DCA_LPARA_5)
)
i_dca_instance_07
(
  .clk(dca_port_20),
  .rstnn(dca_port_41),

  .move_wenable(dca_signal_19),
  .move_wdata_list(dca_signal_55),
  .move_renable(dca_signal_57),
  .move_rdata_list(dca_signal_29),
  
  .shift_up(dca_signal_02),
  .shift_left(dca_signal_50),
  .transpose(dca_signal_11),
  
  .all_rdata_list2d(dca_signal_30),
  .upmost_rdata_list1d(dca_signal_53)
);

assign dca_signal_57 = 0;
assign dca_signal_11 = 0;

DCA_MATRIX_LOAD2MREG
#(
  .MATRIX_SIZE_PARA(MATRIX_SIZE_PARA),
  .TENSOR_PARA(TENSOR_PARA)
)
i_dca_instance_10
(
  .clk(dca_port_20),
  .rstnn(dca_port_41),
  .clear(dca_port_42),
  .enable(dca_port_33),
  .busy(dca_signal_10),

  .load_tensor_row_wready(dca_port_09),
  .load_tensor_row_wvalid(dca_port_08),
  .load_tensor_row_wlast(dca_port_07),
  .load_tensor_row_wdata(dca_port_43),

  .mreg_move_wenable(dca_signal_19),
  .mreg_move_wdata_list1d(dca_signal_55),

  .loadreg_rready(dca_signal_35),
  .loadreg_rrequest(dca_signal_22)
);

DCA_MATRIX_REGISTER_TYPE3
#(
  .MATRIX_SIZE_PARA(MATRIX_SIZE_PARA),
  .BW_TENSOR_SCALAR(BW_TENSOR_SCALAR),
  .BW_MOVE_DATA(DCA_LPARA_0),
  .RESET_VALUE(DCA_LPARA_5)
)
i_dca_instance_02
(
  .clk(dca_port_20),
  .rstnn(dca_port_41),

  .move_wenable(dca_signal_59),
  .move_wdata_list(dca_signal_07),
  .move_renable(dca_signal_25),
  .move_rdata_list(dca_signal_47),
  
  .shift_up(dca_signal_20),
  .shift_left(dca_signal_52),
  .transpose(dca_signal_06),
  
  .all_rdata_list2d(dca_signal_54),
  .upmost_rdata_list1d(dca_signal_31)
);

assign dca_signal_25 =0;
assign dca_signal_06 = 0;

DCA_MATRIX_LOAD2MREG
#(
  .MATRIX_SIZE_PARA(MATRIX_SIZE_PARA),
  .TENSOR_PARA(TENSOR_PARA)
)
i_dca_instance_01
(
  .clk(dca_port_20),
  .rstnn(dca_port_41),
  .clear(dca_port_42),
  .enable(dca_port_33),
  .busy(dca_signal_04),

  .load_tensor_row_wready(dca_port_17),
  .load_tensor_row_wvalid(dca_port_37),
  .load_tensor_row_wlast(dca_port_10),
  .load_tensor_row_wdata(dca_port_39),

  .mreg_move_wenable(dca_signal_59),
  .mreg_move_wdata_list1d(dca_signal_07),

  .loadreg_rready(dca_signal_44),
  .loadreg_rrequest(dca_signal_26)
);

DCA_MATRIX_REGISTER_TYPE5
#(
  .MATRIX_SIZE_PARA(MATRIX_SIZE_PARA),
  .BW_TENSOR_SCALAR(BW_TENSOR_SCALAR),
  .RESET_VALUE(DCA_LPARA_5)
)
i_dca_instance_08
(
  .clk(dca_port_20),
  .rstnn(dca_port_41),

  .init(dca_signal_05),

  .move_wenable(dca_signal_63),
  .move_wdata_list(dca_signal_14),
  .move_renable(dca_signal_00),
  .move_rdata_list(dca_signal_48),

  .all_wenable_list2d(dca_signal_01),
  .all_wdata_list2d(dca_signal_15),
  .all_rdata_list2d(dca_signal_33)
);

DCA_MATRIX_LOAD2MREG
#(
  .MATRIX_SIZE_PARA(MATRIX_SIZE_PARA),
  .TENSOR_PARA(TENSOR_PARA)
)
i_dca_instance_04
(
  .clk(dca_port_20),
  .rstnn(dca_port_41),
  .clear(dca_port_42),
  .enable(dca_port_33),
  .busy(dca_signal_18),

  .load_tensor_row_wready(dca_port_26),
  .load_tensor_row_wvalid(dca_port_03),
  .load_tensor_row_wlast(dca_port_28),
  .load_tensor_row_wdata(dca_port_35),

  .mreg_move_wenable(dca_signal_63),
  .mreg_move_wdata_list1d(dca_signal_14),

  .loadreg_rready(dca_signal_49),
  .loadreg_rrequest(dca_signal_28)
);

DCA_MATRIX_MREG2STORE
#(
  .MATRIX_SIZE_PARA(MATRIX_SIZE_PARA),
  .BW_TENSOR_SCALAR(BW_TENSOR_SCALAR)
)
i_dca_instance_12
(
  .clk(dca_port_20),
  .rstnn(dca_port_41),
  .clear(dca_port_42),
  .enable(dca_port_33),
  .busy(dca_signal_23),

  .storereg_wready(dca_signal_34),
  .storereg_wrequest(dca_signal_32),

  .mreg_move_renable(dca_signal_00),
  .mreg_move_rdata_list1d(dca_signal_48),

  .store_tensor_row_rvalid(dca_port_34),
  .store_tensor_row_rready(dca_port_11),
  .store_tensor_row_rlast(dca_port_13),
  .store_tensor_row_rdata(dca_port_44)
);

DCA_MATRIX_REGISTER_TYPE5
#(
  .MATRIX_SIZE_PARA(MATRIX_SIZE_PARA),
  .BW_TENSOR_SCALAR(BW_TENSOR_SCALAR),
  .RESET_VALUE(DCA_LPARA_5)
)
i_dca_instance_05
(
  .clk(dca_port_20),
  .rstnn(dca_port_41),

  .init(dca_signal_42),

  .move_wenable(dca_signal_38),
  .move_wdata_list(dca_signal_03),
  .move_renable(dca_signal_60),
  .move_rdata_list(dca_signal_45),

  .all_wenable_list2d(dca_signal_17),
  .all_wdata_list2d(dca_signal_40),
  .all_rdata_list2d(dca_signal_56)
);

assign dca_signal_38 = 0;
assign dca_signal_03 = 0;
assign dca_signal_60 = 0;

DCA_MATRIX_REGISTER_CONSTANT
#(
  .MATRIX_SIZE_PARA(MATRIX_SIZE_PARA),
  .BW_TENSOR_SCALAR(BW_TENSOR_SCALAR)
)
i_dca_instance_03
(
  .clk(dca_port_20),
  .rstnn(dca_port_41),

  .scalar_wenable(dca_signal_27),
  .scalar_wdata(dca_signal_12),
  
  .all_rdata_list2d(dca_signal_16),
  .upmost_rdata_list1d(dca_signal_36)
);

assign dca_signal_12 = dca_signal_31[BW_TENSOR_SCALAR-1-:BW_TENSOR_SCALAR];

DCA_MODULE_29
#(
  .MATRIX_SIZE_PARA(MATRIX_SIZE_PARA),
  .TENSOR_PARA(TENSOR_PARA)
)
i_dca_instance_09
(
  .dca_port_16(dca_port_20),
  .dca_port_09(dca_port_41),
  .dca_port_10(dca_port_42),
  .dca_port_18(dca_port_33),
  .dca_port_12(dca_signal_21),

  .dca_port_06(dca_signal_62),
  .dca_port_03(dca_signal_37),
  .dca_port_00(dca_signal_41),

  .dca_port_17(dca_signal_30),
  .dca_port_15(dca_signal_02),
  .dca_port_11(dca_signal_50),

  .dca_port_01(dca_signal_54),
  .dca_port_04(dca_signal_20),
  .dca_port_02(dca_signal_52),

  .dca_port_21(dca_signal_27),
  .dca_port_20(dca_signal_16),

  .dca_port_13(dca_signal_33),
  .dca_port_05(dca_signal_05),
  .dca_port_19(dca_signal_01),
  .dca_port_22(dca_signal_15),

  .dca_port_08(dca_signal_56),
  .dca_port_14(dca_signal_17),
  .dca_port_07(dca_signal_40)
);

DCA_MODULE_09
#(
  .MATRIX_SIZE_PARA(MATRIX_SIZE_PARA)
)
i_dca_instance_11
(
  .dca_port_23(dca_port_20),
  .dca_port_08(dca_port_41),
  .dca_port_14(dca_port_42),
  .dca_port_17(dca_port_33),
  .dca_port_22(dca_signal_43),

  .dca_port_19(dca_signal_09),
  .dca_port_01(dca_signal_46),
  .dca_port_24(dca_signal_13),

  .dca_port_15(dca_signal_35),
  .dca_port_07(dca_signal_22),
  .dca_port_09(dca_signal_44),
  .dca_port_13(dca_signal_26),
  .dca_port_10(dca_signal_49),
  .dca_port_03(dca_signal_28),
  .dca_port_12(dca_signal_34),
  .dca_port_00(dca_signal_34),
  .dca_port_21(dca_signal_32),
  .dca_port_05(dca_port_45),

  .dca_port_06(dca_signal_62),
  .dca_port_16(dca_signal_37),
  .dca_port_02(dca_signal_41),

  .dca_port_18(dca_signal_61),
  .dca_port_04(dca_signal_51),
  .dca_port_11(dca_signal_58),
  .dca_port_20(dca_port_06)
);

assign dca_port_19 = 0;
assign dca_port_16 = 0;
assign dca_port_36 = 0;
assign dca_port_23 = dca_port_14 | dca_signal_09 | dca_signal_43;

endmodule
