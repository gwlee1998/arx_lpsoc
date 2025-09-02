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



module DCA_MATRIX_REGISTER_TYPE1
(
  clk,
  rstnn,
  
  init,

  move_wenable,
  move_wdata_list,
  move_renable,
  move_rdata_list,
  
  all_wenable,
  all_wdata_list2d,
  
  downmost_wenable,
  downmost_wdata_list1d,
  
  rightmost_wenable,
  rightmost_wdata_list1d,
  
  shift_up,
  shift_left,
  transpose,
  
  all_rdata_list2d,
  upmost_rdata_list1d
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

input wire all_wenable;
input wire [BW_TENSOR_MATRIX-1:0] all_wdata_list2d;

input wire downmost_wenable;
input wire [BW_TENSOR_ROW-1:0] downmost_wdata_list1d;

input wire rightmost_wenable;
input wire [BW_TENSOR_ROW-1:0] rightmost_wdata_list1d;

input wire shift_up;
input wire shift_left;
input wire transpose;

output wire [BW_TENSOR_MATRIX-1:0] all_rdata_list2d;
output wire [BW_TENSOR_ROW-1:0] upmost_rdata_list1d;

genvar i, i_row, i_col;

wire dca_signal_07;
wire dca_signal_16;

wire [MATRIX_NUM_ELEMENT-1:0] dca_signal_03;

wire [MATRIX_NUM_ELEMENT-1:0] dca_signal_11;
wire [BW_TENSOR_MATRIX-1:0] dca_signal_04;

wire [MATRIX_NUM_COL-1:0] dca_signal_14;
wire [BW_TENSOR_ROW-1:0] dca_signal_09;

wire [MATRIX_NUM_COL-1:0] dca_signal_06;
wire [BW_TENSOR_ROW-1:0] dca_signal_19;

wire [MATRIX_NUM_ROW-1:0] dca_signal_10;

wire [MATRIX_NUM_ROW-1:0] dca_signal_01;
wire [BW_TENSOR_ROW-1:0] dca_signal_00;

wire [MATRIX_NUM_ELEMENT-1:0] dca_signal_13;
wire [MATRIX_NUM_ELEMENT-1:0] dca_signal_18;
wire [MATRIX_NUM_ELEMENT-1:0] dca_signal_05;
wire [MATRIX_NUM_ELEMENT-1:0] dca_signal_17;
wire [MATRIX_NUM_ELEMENT-1:0] dca_signal_08;

wire [BW_TENSOR_ROW-1:0] dca_signal_12;
wire [BW_TENSOR_ROW-1:0] dca_signal_02;
wire [BW_TENSOR_ROW-1:0] dca_signal_15;

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
  
  .init_list2d(dca_signal_03),
  
  .each_wenable_list2d(dca_signal_11),
  .each_wdata_list2d(dca_signal_04),
  
  .downmost_wenable_list1d(dca_signal_14),
  .downmost_wdata_list1d(dca_signal_09),
  
  .upmost_wenable_list1d(dca_signal_06),
  .upmost_wdata_list1d(dca_signal_19),
  
  .rightmost_wenable_list1d(dca_signal_10),
  .rightmost_wdata_list1d(rightmost_wdata_list1d),
  
  .leftmost_wenable_list1d(dca_signal_01),
  .leftmost_wdata_list1d(dca_signal_00),
  
  .shift_up_list2d(dca_signal_13),
  .shift_down_list2d(dca_signal_18),
  .shift_left_list2d(dca_signal_05),
  .shift_right_list2d(dca_signal_17),
  
  .transpose_list2d(dca_signal_08),
  
  .all_rdata_list2d(all_rdata_list2d),
  .upmost_rdata_list1d(upmost_rdata_list1d),
  .downmost_data_list1d(dca_signal_12),
  .leftmost_data_list1d(dca_signal_02),
  .rightmost_data_list1d(dca_signal_15)
);

assign dca_signal_07 = (BW_MOVE_DATA==BW_TENSOR_ROW);
assign dca_signal_16 = (BW_MOVE_DATA==BW_TENSOR_MATRIX);

assign dca_signal_04 = (dca_signal_16 && (~all_wenable))? move_wdata_list : all_wdata_list2d;
assign dca_signal_09 = (dca_signal_07 && (~downmost_wenable))? move_wdata_list : downmost_wdata_list1d;

for(i=0; i<MATRIX_NUM_ELEMENT; i=i+1)
begin : i_duplicate_2d
  assign dca_signal_03[i] = init;
  assign dca_signal_11[i] = all_wenable | (dca_signal_16 & move_wenable);
  assign dca_signal_13[i] = shift_up | ( dca_signal_07 & (move_wenable|move_renable) );
  assign dca_signal_05[i] = shift_left;
  assign dca_signal_08[i] = transpose;
end

for(i=0; i<MATRIX_NUM_COL; i=i+1)
begin : i_duplicate_col
  assign dca_signal_14[i] = downmost_wenable | (dca_signal_07 & move_wenable);
end

for(i=0; i<MATRIX_NUM_ROW; i=i+1)
begin : i_duplicate_row
  assign dca_signal_10[i] = rightmost_wenable;
end

assign dca_signal_18 = 0;
assign dca_signal_17 = 0;
assign dca_signal_06 = 0;
assign dca_signal_01 = 0;
assign dca_signal_19 = 0;
assign dca_signal_00 = 0;

assign move_rdata_list = dca_signal_07? upmost_rdata_list1d : all_rdata_list2d;

endmodule
