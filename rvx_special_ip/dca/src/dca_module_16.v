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




module DCA_MODULE_16
(
  dca_port_05,
  dca_port_00,
  dca_port_13,
  dca_port_19,
  dca_port_20,

  dca_port_07,

  dca_port_10,
  dca_port_02,
  dca_port_03,

  dca_port_04,
  dca_port_06,
  dca_port_12,

  dca_port_16,
  dca_port_08,
  dca_port_18,
  dca_port_17,
  dca_port_15,

  dca_port_01,
  dca_port_09,
  dca_port_14,
  dca_port_11
);




parameter LSU_PARA = 0;
parameter AXI_PARA = 32;
parameter BW_LPI_BURDEN = 1;
parameter MATRIX_SIZE_PARA = 4;
parameter TENSOR_PARA = 0;

`include "ervp_axi_lpara.vb"
`include "dca_matrix_dim_util.vb"
`include "dca_matrix_dim_lpara.vb"

input wire dca_port_05;
input wire dca_port_00;
input wire dca_port_13;
input wire dca_port_19;
output wire dca_port_20;

output wire dca_port_07;
`include "dca_lsu_util.vb"
`include "dca_lsu_lpara.vb"

input wire dca_port_10;
input wire [BW_RRESP_INFO-1:0] dca_port_02;
output wire dca_port_03;

input wire dca_port_04;
input wire [BW_TXN_INFO-1:0] dca_port_06;
output wire dca_port_12;

`include "lpit_function.vb"
`include "lpixm_function.vb"

localparam BW_LPIXM_ADDR = BW_AXI_ADDR;
localparam BW_LPIXM_DATA = BW_AXI_DATA;

`include "lpixm_lpara.vb"

output wire [2-1:0] dca_port_16;
input wire dca_port_08;
input wire dca_port_18;
input wire dca_port_17;
input wire [BW_LPI_YDATA-1:0] dca_port_15;

`include "dca_tensor_scalar_lpara.vb"
`include "dca_tensor_dim_lpara.vb"

output wire dca_port_01;
output wire dca_port_09;
output wire [BW_TENSOR_ROW-1:0] dca_port_14;
input wire dca_port_11;

wire dca_signal_03;
wire dca_signal_07;
wire dca_signal_10;
wire [BW_AXI_DATA-1:0] dca_signal_06;

wire dca_signal_05;
wire dca_signal_15;
wire [BW_MEMORY_ROW_BUFFER-1:0] dca_signal_00;
wire [BW_TXN_INFO-1:0] dca_signal_12;

wire dca_signal_14;
wire dca_signal_13;
wire [BW_MEMORY_ROW_BUFFER-1:0] dca_signal_02;
wire [BW_TXN_INFO-1:0] dca_signal_04;

wire dca_signal_01;
wire dca_signal_09;
wire [BW_LSU_ELEMENT_ROW-1:0] dca_signal_11;
wire [BW_TXN_INFO-1:0] dca_signal_08;

DCA_MODULE_04
#(
  .LSU_PARA(LSU_PARA),
  .AXI_PARA(AXI_PARA),
  .MATRIX_SIZE_PARA(MATRIX_SIZE_PARA)
)
i_dca_instance_1
(
  .dca_port_07(dca_port_05),
  .dca_port_02(dca_port_00),
  .dca_port_09(dca_port_13),
  .dca_port_06(dca_port_19),

  .dca_port_03(dca_port_04),
  .dca_port_12(dca_port_06),
  .dca_port_00(dca_port_12),

  .dca_port_13(dca_signal_03),
  .dca_port_10(dca_signal_07),
  .dca_port_11(dca_signal_10),
  .dca_port_01(dca_signal_06),

  .dca_port_04(dca_signal_05),
  .dca_port_08(dca_signal_15),
  .dca_port_14(dca_signal_00),
  .dca_port_05(dca_signal_12)
);

assign dca_port_16 = dca_signal_03;
assign dca_signal_07 = dca_port_08;
assign dca_signal_10 = dca_port_17;
assign dca_signal_06 = dca_port_15;

DCA_MODULE_17
#(
  .LSU_PARA(LSU_PARA),
  .AXI_PARA(AXI_PARA),
  .MATRIX_SIZE_PARA(MATRIX_SIZE_PARA)
)
i_dca_instance_2
(
  .dca_port_06(dca_port_05),
  .dca_port_03(dca_port_00),
  .dca_port_01(dca_port_13),
  .dca_port_10(dca_port_19),

  .dca_port_00(dca_port_02),

  .dca_port_02(dca_signal_05),
  .dca_port_04(dca_signal_15),
  .dca_port_08(dca_signal_00),
  .dca_port_09(dca_signal_12),

  .dca_port_05(dca_signal_14),
  .dca_port_11(dca_signal_13),
  .dca_port_07(dca_signal_02),
  .dca_port_12(dca_signal_04)
);

DCA_MODULE_05
#(
  .LSU_PARA(LSU_PARA),
  .AXI_PARA(AXI_PARA),
  .MATRIX_SIZE_PARA(MATRIX_SIZE_PARA)
)
i_dca_instance_0
(
  .dca_port_00(dca_port_05),
  .dca_port_12(dca_port_00),
  .dca_port_07(dca_port_13),
  .dca_port_11(dca_port_19),

  .dca_port_02(dca_port_02),

  .dca_port_09(dca_signal_14),
  .dca_port_05(dca_signal_13),
  .dca_port_01(dca_signal_02),
  .dca_port_10(dca_signal_04),

  .dca_port_08(dca_signal_01),
  .dca_port_06(dca_signal_09),
  .dca_port_03(dca_signal_11),
  .dca_port_04(dca_signal_08)
);

DCA_MODULE_18
#(
  .LSU_PARA(LSU_PARA),
  .AXI_PARA(AXI_PARA),
  .MATRIX_SIZE_PARA(MATRIX_SIZE_PARA),
  .TENSOR_PARA(TENSOR_PARA)
)
i_dca_instance_3
(
  .dca_port_00(dca_port_05),
  .dca_port_04(dca_port_00),
  .dca_port_06(dca_port_13),
  .dca_port_08(dca_port_19),

  .dca_port_03(dca_port_02),

  .dca_port_05(dca_signal_01),
  .dca_port_09(dca_signal_09),
  .dca_port_02(dca_signal_11),
  .dca_port_01(dca_signal_08),

  .dca_port_12(dca_port_01),
  .dca_port_10(dca_port_09),
  .dca_port_07(dca_port_14),
  .dca_port_11(dca_port_11)
);

assign dca_port_03 = dca_port_01 & dca_port_11 & dca_port_09;
assign dca_port_07 = dca_port_03;
assign dca_port_20 = dca_port_10;

endmodule
