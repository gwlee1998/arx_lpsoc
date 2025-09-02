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



module DCA_MATRIX_REGISTER_TYPE2
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

input wire shift_up;
input wire shift_left;
input wire transpose;

output wire [BW_TENSOR_MATRIX-1:0] all_rdata_list2d;
output wire [BW_TENSOR_ROW-1:0] upmost_rdata_list1d;

wire [BW_TENSOR_ROW-1:0] dca_signal_0;
wire [BW_TENSOR_ROW-1:0] dca_signal_1;

DCA_MATRIX_REGISTER_TYPE1
#(
  .MATRIX_SIZE_PARA(MATRIX_SIZE_PARA),
  .BW_TENSOR_SCALAR(BW_TENSOR_SCALAR),
  .RESET_VALUE(RESET_VALUE),
  .BW_MOVE_DATA(BW_MOVE_DATA)
)
i_dca_instance_0
(
  .clk(clk),
  .rstnn(rstnn),
  
  .init(init),

  .move_wenable(move_wenable),
  .move_wdata_list(move_wdata_list),
  .move_renable(move_renable),
  .move_rdata_list(move_rdata_list),
  
  .all_wenable(all_wenable),
  .all_wdata_list2d(all_wdata_list2d),
  
  .downmost_wenable(1'b 0),
  .downmost_wdata_list1d(dca_signal_0),
  
  .rightmost_wenable(1'b 0),
  .rightmost_wdata_list1d(dca_signal_1),
  
  .shift_up(shift_up),
  .shift_left(shift_left),
  .transpose(transpose),
  
  .all_rdata_list2d(all_rdata_list2d),
  .upmost_rdata_list1d(upmost_rdata_list1d)
);

assign dca_signal_0 = 0;
assign dca_signal_1 = 0;

endmodule
