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
`include "ervp_axi_define.vh"
`include "dca_matrix_lsu_inst.vh"



module DCA_MODULE_11
(
  dca_port_05,
  dca_port_09,
  dca_port_08,
  dca_port_07,
  dca_port_03,

  dca_port_06,

  dca_port_01,
  dca_port_11,
  dca_port_12,

  dca_port_00,
  dca_port_13,
  dca_port_04,
  dca_port_14,
  dca_port_02,
  dca_port_10
);



parameter LSU_PARA = 0;
parameter AXI_PARA = 32;
parameter BW_LPI_BURDEN = 0;
parameter MATRIX_SIZE_PARA = 4;

`include "ervp_axi_lpara.vb"
`include "dca_matrix_dim_util.vb"
`include "dca_matrix_dim_lpara.vb"

input wire dca_port_05;
input wire dca_port_09;
input wire dca_port_08;
input wire dca_port_07;
output wire dca_port_03;

localparam  DCA_LPARA_0 = `BW_DCA_MATRIX_LSU_INST;

input wire [DCA_LPARA_0-1:0] dca_port_06;

`include "dca_lsu_util.vb"
`include "dca_lsu_lpara.vb"

input wire dca_port_01;
input wire [BW_TXN_INFO-1:0] dca_port_11;
output wire dca_port_12;

`include "lpit_function.vb"
`include "lpixm_function.vb"

localparam  BW_LPIXM_ADDR = BW_AXI_ADDR;
localparam  BW_LPIXM_DATA = BW_AXI_DATA;

`include "lpixm_lpara.vb"

input wire [2-1:0] dca_port_00;
output wire dca_port_13;
output wire dca_port_04;
output wire dca_port_14;
output wire dca_port_02;
output wire [BW_LPI_QDATA-1:0] dca_port_10;

wire [`BW_DCA_MATRIX_LSU_INST_OPCODE-1:0] dca_signal_18;
wire [`BW_DCA_MATRIX_INFO_ADDR-1:0] dca_signal_20;
wire [`BW_DCA_MATRIX_INFO_STRIDE_LS3-1:0] dca_signal_16;
wire [`BW_DCA_MATRIX_INFO_NUM_ROW_M1-1:0] dca_signal_08;
wire [`BW_DCA_MATRIX_INFO_NUM_COL_M1-1:0] dca_signal_23;
wire [`BW_DCA_MATRIX_INFO_IS_SIGNED-1:0] dca_signal_06;
wire [`BW_DCA_MATRIX_INFO_IS_FLOAT-1:0] dca_signal_19;
wire [`BW_DCA_MATRIX_INFO_ADDR_LSA_P3-1:0] dca_signal_21;

wire [BW_BITADDR-1:0] dca_signal_09;
wire [`BW_AXI_ALEN-1:0] dca_signal_02;
wire dca_signal_11;
wire dca_signal_17;

wire [BW_AXI_ADDR-1:0] dca_signal_15;
wire [BW_AXI_ADDR_OFFSET-1:0] dca_signal_12;
wire [BW_AXI_ADDR_OFFSET-1:0] dca_signal_01;
wire [BW_AXI_ADDR-1:0] dca_signal_22;

wire dca_signal_07;

wire dca_signal_05;
wire [`BW_AXI_ALEN-1:0] dca_signal_04;
wire [`BW_AXI_ASIZE-1:0] dca_signal_14;
wire [`BW_AXI_ABURST-1:0] dca_signal_00;
wire [BW_AXI_ADDR-1:0] dca_signal_13;
wire [`NUM_BYTE(BW_AXI_DATA)-1:0] dca_signal_10;
wire [BW_AXI_DATA-1:0] dca_signal_03;

assign {dca_signal_21, dca_signal_19, dca_signal_06, dca_signal_23, dca_signal_08, dca_signal_16, dca_signal_20, dca_signal_18} = dca_port_06;
assign {dca_signal_17, dca_signal_11, dca_signal_02, dca_signal_09} = dca_port_11;

assign dca_signal_15 = dca_signal_09>>3;
assign dca_signal_12 = 0;
assign dca_signal_01 = dca_signal_15;
assign dca_signal_22 = {dca_signal_15[BW_AXI_ADDR-1:BW_AXI_ADDR_OFFSET], dca_signal_12};

assign dca_signal_07 = dca_port_07 & (dca_signal_18==`DCA_MATRIX_LSU_INST_OPCODE_READ);

assign dca_port_13 = dca_signal_07 & dca_port_01;
assign dca_port_04 = 0;
assign dca_port_14 = 1;
assign dca_port_02 = 1;
assign dca_port_10 = {dca_signal_05,dca_signal_04,dca_signal_14,dca_signal_00,dca_signal_10,dca_signal_03,dca_signal_13};

assign dca_signal_05 = 0;
assign dca_signal_04 = $unsigned(dca_signal_02);
assign dca_signal_14 = AXI_SIZE;
assign dca_signal_00 = `AXI_BURST_INCR;
assign dca_signal_10 = 0;
assign dca_signal_03 = 0;
assign dca_signal_13 = dca_signal_22;

assign dca_port_12 = dca_signal_07 & dca_port_00[0];

assign dca_port_03 = 0;

endmodule
