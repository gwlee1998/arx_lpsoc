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



module DCA_MATRIX_REGISTER_TYPE4
(
  clk,
  rstnn,

  init,
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

input wire clk, rstnn;

input wire init;
input wire all_wenable;
input wire [BW_TENSOR_MATRIX-1:0] all_wdata_list2d;

input wire shift_up;
input wire shift_left;
input wire transpose;

output wire [BW_TENSOR_MATRIX-1:0] all_rdata_list2d;
output wire [BW_TENSOR_ROW-1:0] upmost_rdata_list1d;

localparam  DCA_LPARA_0 = BW_TENSOR_ROW;

wire [DCA_LPARA_0-1:0] dca_signal_0;

DCA_MATRIX_REGISTER_TYPE2
#(
  .MATRIX_SIZE_PARA(MATRIX_SIZE_PARA),
  .BW_TENSOR_SCALAR(BW_TENSOR_SCALAR),
  .RESET_VALUE(RESET_VALUE),
  .INIT_VALUE(INIT_VALUE),
  .BW_MOVE_DATA(DCA_LPARA_0)
)
i_dca_instance_0
(
  .clk(clk),
  .rstnn(rstnn),
  
  .init(init),
  
  .move_wenable(1'b 0),
  .move_wdata_list(dca_signal_0),
  .move_renable(1'b 0),
  .move_rdata_list(),
  
  .all_wenable(all_wenable),
  .all_wdata_list2d(all_wdata_list2d),
  
  .shift_up(shift_up),
  .shift_left(shift_left),
  .transpose(transpose),
  
  .all_rdata_list2d(all_rdata_list2d),
  .upmost_rdata_list1d(upmost_rdata_list1d)
);

assign dca_signal_0 = 0;

endmodule
