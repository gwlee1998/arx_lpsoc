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


module DCA_MATRIX_REGISTER_CONSTANT
(
  clk,
  rstnn,

  scalar_wenable,
  scalar_wdata,
  
  all_rdata_list2d,
  upmost_rdata_list1d
);


parameter MATRIX_SIZE_PARA = 8;
parameter BW_TENSOR_SCALAR = 32;

`include "dca_matrix_dim_util.vb"
`include "dca_matrix_dim_lpara.vb"
`include "dca_tensor_dim_lpara.vb"

input wire clk, rstnn;

input wire scalar_wenable;
input wire [BW_TENSOR_SCALAR-1:0] scalar_wdata;

output wire [BW_TENSOR_MATRIX-1:0] all_rdata_list2d;
output wire [BW_TENSOR_ROW-1:0] upmost_rdata_list1d;

genvar i;
genvar i_row, i_col;

reg [BW_TENSOR_SCALAR-1:0] dca_signal_0 [0:MATRIX_NUM_COL-1];

generate
	for(i_col=0; i_col<MATRIX_NUM_COL; i_col=i_col+1)
	begin : i_write_row_reg
		always@(posedge clk, negedge rstnn)
		begin
			if(rstnn==0)
				dca_signal_0[i_col] <= 0;
			else if(scalar_wenable)
				dca_signal_0[i_col] <= scalar_wdata;
		end
		assign upmost_rdata_list1d[(i_col+1)*BW_TENSOR_SCALAR-1 -:BW_TENSOR_SCALAR] = dca_signal_0[i_col];
	end
endgenerate

generate
	for(i_row=0; i_row<MATRIX_NUM_ROW; i_row=i_row+1)
	begin : i_iter_row
	  assign all_rdata_list2d[(i_row+1)*BW_TENSOR_ROW-1 -:BW_TENSOR_ROW] = upmost_rdata_list1d;
  end
endgenerate

endmodule
