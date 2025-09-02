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



module DCA_MODULE_03
(
  dca_port_05,
  dca_port_14,
  dca_port_04,
  dca_port_00,

  dca_port_09,

  dca_port_10,
  dca_port_06,
  dca_port_13,
  dca_port_11,
  dca_port_08,

  dca_port_07,
  dca_port_01,
  dca_port_03,
  dca_port_12,
  dca_port_02
);



parameter LSU_PARA = 0;
parameter AXI_PARA = 32;
parameter MATRIX_SIZE_PARA = 4;

`include "ervp_axi_lpara.vb"
`include "dca_matrix_dim_util.vb"
`include "dca_matrix_dim_lpara.vb"

input wire dca_port_05;
input wire dca_port_14;
input wire dca_port_04;
input wire dca_port_00;

`include "dca_lsu_util.vb"
`include "dca_lsu_lpara.vb"

input wire [BW_RRESP_INFO-1:0] dca_port_09;

output wire dca_port_10;
input wire dca_port_06;
input wire [BW_MEMORY_ROW_BUFFER-1:0] dca_port_13;
input wire [BW_MEMORY_ROW_BUFFER-1:0] dca_port_11;
input wire [BW_TXN_INFO-1:0] dca_port_08;

input wire dca_port_07;
output reg dca_port_01;
output reg [BW_MEMORY_ROW_BUFFER-1:0] dca_port_03;
output reg [(BW_MEMORY_ROW_BUFFER/`BW_BYTE)-1:0] dca_port_12;
output reg [BW_TXN_INFO-1:0] dca_port_02;

genvar i;

wire [`BW_DCA_MATRIX_LSU_INST_OPCODE-1:0] dca_signal_02;
wire [`BW_DCA_MATRIX_INFO_ADDR-1:0] dca_signal_15;
wire [`BW_DCA_MATRIX_INFO_STRIDE_LS3-1:0] dca_signal_07;
wire [`BW_DCA_MATRIX_INFO_NUM_ROW_M1-1:0] dca_signal_09;
wire [`BW_DCA_MATRIX_INFO_NUM_COL_M1-1:0] dca_signal_13;
wire [`BW_DCA_MATRIX_INFO_IS_SIGNED-1:0] dca_signal_04;
wire [`BW_DCA_MATRIX_INFO_IS_FLOAT-1:0] dca_signal_10;
wire [`BW_DCA_MATRIX_INFO_ADDR_LSA_P3-1:0] dca_signal_14;

wire [MATRIX_NUM_COL-1:0] dca_signal_12;

wire [BW_BITADDR-1:0] dca_signal_06;
wire [`BW_AXI_ALEN-1:0] dca_signal_05;
wire dca_signal_08;
wire dca_signal_01;

wire dca_signal_11;
wire dca_signal_00;

wire [(BW_MEMORY_ROW_BUFFER/`BW_BYTE)-1:0] dca_signal_03;

assign {dca_signal_12, dca_signal_14, dca_signal_10, dca_signal_04, dca_signal_13, dca_signal_09, dca_signal_07, dca_signal_15, dca_signal_02} = dca_port_09;
assign {dca_signal_01, dca_signal_08, dca_signal_05, dca_signal_06} = dca_port_08;

assign dca_signal_11 = dca_port_06 & dca_port_10;
assign dca_signal_00 = dca_port_01 & dca_port_07;

assign dca_port_10 = dca_port_01? dca_port_07 : 1;

always@(posedge dca_port_05 or negedge dca_port_14)
begin
  if(dca_port_14==0)
    dca_port_01 <= 0;
  else if(dca_signal_11)
    dca_port_01 <= ~dca_signal_01;
  else if(dca_signal_00)
    dca_port_01 <= 0;
end

always@(posedge dca_port_05 or negedge dca_port_14)
begin
  if(dca_port_14==0)
  begin
    dca_port_03 <= 0;
    dca_port_02 <= 0;
  end
  else if(dca_port_00 && dca_signal_11)
  begin
    dca_port_03 <= ((~dca_port_11)&dca_port_03) | (dca_port_11&dca_port_13);
    dca_port_02 <= dca_port_08;
  end
end

generate
for(i=0; i<(BW_MEMORY_ROW_BUFFER/`BW_BYTE); i=i+1)
begin : i_gen_aligned_row_wstrb
  assign dca_signal_03[i] = dca_port_11[i<<3];
end
endgenerate

always@(posedge dca_port_05 or negedge dca_port_14)
begin
  if(dca_port_14==0)
    dca_port_12 <= 0;
  else if(dca_port_00)
  begin
    if(dca_signal_11 && dca_signal_00)
      dca_port_12 <= dca_signal_03;
    else if(dca_signal_00)
      dca_port_12 <= 0;
    else if(dca_signal_11)
      dca_port_12 <= dca_port_12 | dca_signal_03;
  end
end

endmodule
