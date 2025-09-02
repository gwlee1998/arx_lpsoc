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



module DCA_MODULE_05
(
  dca_port_00,
  dca_port_12,
  dca_port_07,
  dca_port_11,

  dca_port_02,

  dca_port_09,
  dca_port_05,
  dca_port_01,
  dca_port_10,

  dca_port_08,
  dca_port_06,
  dca_port_03,
  dca_port_04
);



parameter LSU_PARA = 0;
parameter AXI_PARA = 32;
parameter MATRIX_SIZE_PARA = 4;

`include "ervp_axi_lpara.vb"
`include "dca_matrix_dim_util.vb"
`include "dca_matrix_dim_lpara.vb"

input wire dca_port_00;
input wire dca_port_12;
input wire dca_port_07;
input wire dca_port_11;

`include "dca_lsu_util.vb"
`include "dca_lsu_lpara.vb"

input wire [BW_RRESP_INFO-1:0] dca_port_02;

output wire dca_port_09;
input wire dca_port_05;
input wire [BW_MEMORY_ROW_BUFFER-1:0] dca_port_01;
input wire [BW_TXN_INFO-1:0] dca_port_10;

input wire dca_port_08;
output wire dca_port_06;
output wire [BW_LSU_ELEMENT_ROW-1:0] dca_port_03;
output wire [BW_TXN_INFO-1:0] dca_port_04;

integer i;

wire [`BW_DCA_MATRIX_LSU_INST_OPCODE-1:0] dca_signal_01;
wire [`BW_DCA_MATRIX_INFO_ADDR-1:0] dca_signal_11;
wire [`BW_DCA_MATRIX_INFO_STRIDE_LS3-1:0] dca_signal_06;
wire [`BW_DCA_MATRIX_INFO_NUM_ROW_M1-1:0] dca_signal_02;
wire [`BW_DCA_MATRIX_INFO_NUM_COL_M1-1:0] dca_signal_08;
wire [`BW_DCA_MATRIX_INFO_IS_SIGNED-1:0] dca_signal_03;
wire [`BW_DCA_MATRIX_INFO_IS_FLOAT-1:0] dca_signal_07;
wire [`BW_DCA_MATRIX_INFO_ADDR_LSA_P3-1:0] dca_signal_05;

wire [MATRIX_NUM_COL-1:0] dca_signal_09;

wire [MAX_BW_MEMORY_SINGLE_DEFAULT*MATRIX_NUM_COL-1:0] dca_signal_10;
wire dca_signal_04;
reg [BW_LSU_ELEMENT_ROW-1:0] dca_signal_00;

assign {dca_signal_09, dca_signal_05, dca_signal_07, dca_signal_03, dca_signal_08, dca_signal_02, dca_signal_06, dca_signal_11, dca_signal_01} = dca_port_02;

assign dca_signal_10 = dca_port_01;

always@(*)
begin
  dca_signal_00 = 0;
  for(i=0; i<MATRIX_NUM_COL; i=i+1)
  begin
    case(dca_signal_05)
      0:
      begin
        dca_signal_00[BW_LSU_ELEMENT*i+:BW_LSU_ELEMENT] = $unsigned(dca_signal_10[1*i+:1]);
      end
      1:
      begin
        if(dca_signal_03)
          dca_signal_00[BW_LSU_ELEMENT*i+:BW_LSU_ELEMENT] = $signed(dca_signal_10[2*i+:2]);
        else
          dca_signal_00[BW_LSU_ELEMENT*i+:BW_LSU_ELEMENT] = $unsigned(dca_signal_10[2*i+:2]);
      end
      2:
      begin
        if(dca_signal_03)
          dca_signal_00[BW_LSU_ELEMENT*i+:BW_LSU_ELEMENT] = $signed(dca_signal_10[4*i+:4]);
        else
          dca_signal_00[BW_LSU_ELEMENT*i+:BW_LSU_ELEMENT] = $unsigned(dca_signal_10[4*i+:4]);
      end
      3:
      begin
        if(dca_signal_03)
          dca_signal_00[BW_LSU_ELEMENT*i+:BW_LSU_ELEMENT] = $signed(dca_signal_10[8*i+:8]);
        else
          dca_signal_00[BW_LSU_ELEMENT*i+:BW_LSU_ELEMENT] = $unsigned(dca_signal_10[8*i+:8]);
      end
      4:
      begin
        if(dca_signal_03)
          dca_signal_00[BW_LSU_ELEMENT*i+:BW_LSU_ELEMENT] = $signed(dca_signal_10[16*i+:16]);
        else
          dca_signal_00[BW_LSU_ELEMENT*i+:BW_LSU_ELEMENT] = $unsigned(dca_signal_10[16*i+:16]);
      end
      5:
      begin
        if(dca_signal_03)
          dca_signal_00[BW_LSU_ELEMENT*i+:BW_LSU_ELEMENT] = $signed(dca_signal_10[32*i+:32]);
        else
          dca_signal_00[BW_LSU_ELEMENT*i+:BW_LSU_ELEMENT] = $unsigned(dca_signal_10[32*i+:32]);
      end
      default:
      begin
        if(dca_signal_03)
          dca_signal_00[BW_LSU_ELEMENT*i+:BW_LSU_ELEMENT] = $signed(dca_signal_10[32*i+:32]);
        else
          dca_signal_00[BW_LSU_ELEMENT*i+:BW_LSU_ELEMENT] = $unsigned(dca_signal_10[32*i+:32]);
      end
    endcase
  end
end

assign dca_signal_04 = dca_port_05;

assign dca_port_09 = dca_port_08;
assign dca_port_06 = dca_signal_04;
assign dca_port_03 = dca_signal_00;
assign dca_port_04 = dca_port_10;

endmodule
