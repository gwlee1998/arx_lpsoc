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



module DCA_MODULE_02
(
  dca_port_01,
  dca_port_11,
  dca_port_07,
  dca_port_00,

  dca_port_12,

  dca_port_08,
  dca_port_02,
  dca_port_03,
  dca_port_04,

  dca_port_05,
  dca_port_06,
  dca_port_09,
  dca_port_10  
);



parameter LSU_PARA = 0;
parameter AXI_PARA = 32;
parameter MATRIX_SIZE_PARA = 4;

`include "ervp_axi_lpara.vb"
`include "dca_matrix_dim_util.vb"
`include "dca_matrix_dim_lpara.vb"

input wire dca_port_01;
input wire dca_port_11;
input wire dca_port_07;
input wire dca_port_00;

`include "dca_lsu_util.vb"
`include "dca_lsu_lpara.vb"

input wire [BW_RRESP_INFO-1:0] dca_port_12;

output wire dca_port_08;
input wire dca_port_02;
input wire [BW_LSU_ELEMENT_ROW-1:0] dca_port_03;
input wire [BW_TXN_INFO-1:0] dca_port_04;

input wire dca_port_05;
output wire dca_port_06;
output wire [BW_MEMORY_ROW_BUFFER-1:0] dca_port_09;
output wire [BW_TXN_INFO-1:0] dca_port_10;

integer i;

wire [`BW_DCA_MATRIX_LSU_INST_OPCODE-1:0] dca_signal_02;
wire [`BW_DCA_MATRIX_INFO_ADDR-1:0] dca_signal_10;
wire [`BW_DCA_MATRIX_INFO_STRIDE_LS3-1:0] dca_signal_04;
wire [`BW_DCA_MATRIX_INFO_NUM_ROW_M1-1:0] dca_signal_06;
wire [`BW_DCA_MATRIX_INFO_NUM_COL_M1-1:0] dca_signal_08;
wire [`BW_DCA_MATRIX_INFO_IS_SIGNED-1:0] dca_signal_01;
wire [`BW_DCA_MATRIX_INFO_IS_FLOAT-1:0] dca_signal_09;
wire [`BW_DCA_MATRIX_INFO_ADDR_LSA_P3-1:0] dca_signal_00;

wire [MATRIX_NUM_COL-1:0] dca_signal_03;

wire dca_signal_05;
reg [MAX_BW_MEMORY_SINGLE_DEFAULT*MATRIX_NUM_COL-1:0] dca_signal_07;

assign {dca_signal_03, dca_signal_00, dca_signal_09, dca_signal_01, dca_signal_08, dca_signal_06, dca_signal_04, dca_signal_10, dca_signal_02} = dca_port_12;

always@(*)
begin
  dca_signal_07 = 0;
  for(i=0; i<MATRIX_NUM_COL; i=i+1)
  begin
    case(dca_signal_00)
      0:
      begin
        dca_signal_07[1*i+:1] = $unsigned(dca_port_03[BW_LSU_ELEMENT*i+:BW_LSU_ELEMENT]);
      end
      1:
      begin
        if(dca_signal_01)
          dca_signal_07[2*i+:2] = $signed(dca_port_03[BW_LSU_ELEMENT*i+:BW_LSU_ELEMENT]);
        else
          dca_signal_07[2*i+:2] = $unsigned(dca_port_03[BW_LSU_ELEMENT*i+:BW_LSU_ELEMENT]);
      end
      2:
      begin
        if(dca_signal_01)
          dca_signal_07[4*i+:4] = $signed(dca_port_03[BW_LSU_ELEMENT*i+:BW_LSU_ELEMENT]);
        else
          dca_signal_07[4*i+:4] = $unsigned(dca_port_03[BW_LSU_ELEMENT*i+:BW_LSU_ELEMENT]);
      end
      3:
      begin
        if(dca_signal_01)
          dca_signal_07[8*i+:8] = $signed(dca_port_03[BW_LSU_ELEMENT*i+:BW_LSU_ELEMENT]);
        else
          dca_signal_07[8*i+:8] = $unsigned(dca_port_03[BW_LSU_ELEMENT*i+:BW_LSU_ELEMENT]);
      end
      4:
      begin
        if(dca_signal_01)
          dca_signal_07[16*i+:16] = $signed(dca_port_03[BW_LSU_ELEMENT*i+:BW_LSU_ELEMENT]);
        else
          dca_signal_07[16*i+:16] = $unsigned(dca_port_03[BW_LSU_ELEMENT*i+:BW_LSU_ELEMENT]);
      end
      5:
      begin
        if(dca_signal_01)
          dca_signal_07[32*i+:32] = $signed(dca_port_03[BW_LSU_ELEMENT*i+:BW_LSU_ELEMENT]);
        else
          dca_signal_07[32*i+:32] = $unsigned(dca_port_03[BW_LSU_ELEMENT*i+:BW_LSU_ELEMENT]);
      end
      default:
      begin
        if(dca_signal_01)
          dca_signal_07[32*i+:32] = $signed(dca_port_03[BW_LSU_ELEMENT*i+:BW_LSU_ELEMENT]);
        else
          dca_signal_07[32*i+:32] = $unsigned(dca_port_03[BW_LSU_ELEMENT*i+:BW_LSU_ELEMENT]);
      end
    endcase
  end
end

assign dca_signal_05 = dca_port_02;

assign dca_port_08 = dca_port_05;
assign dca_port_06 = dca_signal_05;
assign dca_port_09 = dca_signal_07;
assign dca_port_10 = dca_port_04;

endmodule
