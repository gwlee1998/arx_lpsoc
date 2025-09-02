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



module DCA_MATRIX_REGISTER_TYPE5
(
  clk,
  rstnn,
  
  init,

  move_wenable,
  move_wdata_list,
  move_renable,
  move_rdata_list,
  
  all_wenable_list2d,
  all_wdata_list2d,
  all_rdata_list2d
);



parameter MATRIX_SIZE_PARA = 8;
parameter BW_TENSOR_SCALAR = 32;
parameter RESET_VALUE = 0;
parameter INIT_VALUE = RESET_VALUE;

`include "dca_matrix_dim_util.vb"
`include "dca_matrix_dim_lpara.vb"
`include "dca_tensor_dim_lpara.vb"

parameter BW_MOVE_DATA = BW_TENSOR_ROW;

input wire clk, rstnn;
input wire init;

input wire move_wenable;
input wire [BW_MOVE_DATA-1:0] move_wdata_list;
input wire move_renable;
output wire [BW_MOVE_DATA-1:0] move_rdata_list;

input wire [MATRIX_NUM_ELEMENT-1:0] all_wenable_list2d;
input wire [BW_TENSOR_MATRIX-1:0] all_wdata_list2d;
output wire [BW_TENSOR_MATRIX-1:0] all_rdata_list2d;

genvar i;

wire dca_signal_18;
wire dca_signal_11;

wire [MATRIX_NUM_ELEMENT-1:0] dca_signal_10;

wire [MATRIX_NUM_ELEMENT-1:0] dca_signal_08;
wire [BW_TENSOR_MATRIX-1:0] dca_signal_00;

wire [MATRIX_NUM_COL-1:0] dca_signal_07;
wire [BW_TENSOR_ROW-1:0] dca_signal_20;
wire [MATRIX_NUM_COL-1:0] dca_signal_04;
wire [BW_TENSOR_ROW-1:0] dca_signal_17;
wire [MATRIX_NUM_ROW-1:0] dca_signal_02;
wire [BW_TENSOR_COL-1:0] dca_signal_09;
wire [MATRIX_NUM_ROW-1:0] dca_signal_19;
wire [BW_TENSOR_COL-1:0] dca_signal_01;
wire [MATRIX_NUM_ELEMENT-1:0] dca_signal_14;
wire [MATRIX_NUM_ELEMENT-1:0] dca_signal_12;
wire [MATRIX_NUM_ELEMENT-1:0] dca_signal_13;
wire [MATRIX_NUM_ELEMENT-1:0] dca_signal_21;
wire [MATRIX_NUM_ELEMENT-1:0] dca_signal_16;

wire [BW_TENSOR_ROW-1:0] dca_signal_06;
wire [BW_TENSOR_ROW-1:0] dca_signal_03;
wire [BW_TENSOR_COL-1:0] dca_signal_15;
wire [BW_TENSOR_COL-1:0] dca_signal_05;

DCA_MATRIX_REGISTER_TEMPLATE
#(
  .MATRIX_SIZE_PARA(MATRIX_SIZE_PARA),
  .BW_TENSOR_SCALAR(BW_TENSOR_SCALAR),
  .RESET_VALUE(RESET_VALUE),
  .INIT_VALUE(INIT_VALUE)
)
i_dca_instance_0
(
  .clk(clk),
  .rstnn(rstnn),
  
  .init_list2d(dca_signal_10),
  
  .each_wenable_list2d(dca_signal_08),
  .each_wdata_list2d(dca_signal_00),
  
  .downmost_wenable_list1d(dca_signal_07),
  .downmost_wdata_list1d(dca_signal_20),
  
  .upmost_wenable_list1d(dca_signal_04),
  .upmost_wdata_list1d(dca_signal_17),
  
  .rightmost_wenable_list1d(dca_signal_02),
  .rightmost_wdata_list1d(dca_signal_09),
  
  .leftmost_wenable_list1d(dca_signal_19),
  .leftmost_wdata_list1d(dca_signal_01),
  
  .shift_up_list2d(dca_signal_14),
  .shift_down_list2d(dca_signal_12),
  .shift_left_list2d(dca_signal_13),
  .shift_right_list2d(dca_signal_21),
  
  .transpose_list2d(dca_signal_16),
  
  .all_rdata_list2d(all_rdata_list2d),
  .upmost_rdata_list1d(dca_signal_06),
  .downmost_data_list1d(dca_signal_03),
  .leftmost_data_list1d(dca_signal_15),
  .rightmost_data_list1d(dca_signal_05)
);

assign dca_signal_18 = (BW_MOVE_DATA==BW_TENSOR_ROW);
assign dca_signal_11 = (BW_MOVE_DATA==BW_TENSOR_MATRIX);

assign dca_signal_00 = (dca_signal_11 && move_wenable)? move_wdata_list : all_wdata_list2d;

generate
  for(i=0; i<MATRIX_NUM_ELEMENT; i=i+1)
  begin : i_duplicate_2d
    assign dca_signal_10[i] = init;
    assign dca_signal_08[i] = all_wenable_list2d[i] | (dca_signal_11 & move_wenable);
    assign dca_signal_14[i] = dca_signal_18 & (move_wenable|move_renable);
  end
endgenerate

generate
  for(i=0; i<MATRIX_NUM_COL; i=i+1)
  begin : i_duplicate_col
    assign dca_signal_07[i] = dca_signal_18 & move_wenable;
  end
endgenerate

assign dca_signal_20 = move_wdata_list;

assign dca_signal_04 = 0;
assign dca_signal_17 = 0;
assign dca_signal_02 = 0;
assign dca_signal_09 = 0;
assign dca_signal_19 = 0;
assign dca_signal_01 = 0;
assign dca_signal_12 = 0;
assign dca_signal_13 = 0;
assign dca_signal_21 = 0;
assign dca_signal_16 = 0;

assign move_rdata_list = dca_signal_18? dca_signal_06 : all_rdata_list2d;

endmodule
