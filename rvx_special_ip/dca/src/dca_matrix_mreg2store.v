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



module DCA_MATRIX_MREG2STORE
(
  clk,
  rstnn,
  clear,
  enable,
  busy,

  storereg_wready,
  storereg_wrequest,

  mreg_move_renable,
  mreg_move_rdata_list1d,

  store_tensor_row_rvalid,
  store_tensor_row_rlast,
  store_tensor_row_rready,
  store_tensor_row_rdata
);



parameter MATRIX_SIZE_PARA = 4;
parameter BW_TENSOR_SCALAR = 32;

`include "dca_matrix_dim_util.vb"
`include "dca_matrix_dim_lpara.vb"
`include "dca_tensor_dim_lpara.vb"

input wire clk;
input wire rstnn;
input wire clear;
input wire enable;
output wire busy;

localparam  DCA_LPARA_3 = BW_TENSOR_ROW;

output wire storereg_wready;
input wire storereg_wrequest;

output wire mreg_move_renable;
input wire [DCA_LPARA_3-1:0] mreg_move_rdata_list1d;

input wire store_tensor_row_rvalid;
input wire store_tensor_row_rlast;
output wire store_tensor_row_rready;
output wire [BW_TENSOR_ROW-1:0] store_tensor_row_rdata;

wire dca_signal_7;
wire dca_signal_1;
wire dca_signal_8;

localparam  DCA_LPARA_5 = MATRIX_NUM_ROW;

wire dca_signal_3;
wire dca_signal_6;
wire [DCA_LPARA_5-1:0] dca_signal_0;
wire dca_signal_4;
wire dca_signal_2;

localparam  DCA_LPARA_2 = 2;
localparam  DCA_LPARA_1 = 0;
localparam  DCA_LPARA_0 = 1;
localparam  DCA_LPARA_4 = 2;

reg [DCA_LPARA_2-1:0] dca_signal_5;

assign dca_signal_7 = store_tensor_row_rready & store_tensor_row_rvalid;
assign dca_signal_1 = dca_signal_7 | (dca_signal_5==DCA_LPARA_4);
assign dca_signal_8 = dca_signal_1 & dca_signal_2;

ERVP_COUNTER_WITH_ONEHOT_ENCODING
#(
  .COUNT_LENGTH(DCA_LPARA_5)
)
i_dca_instance_0
(
  .clk(clk),
  .rstnn(rstnn),
  .enable(enable),
  .init(dca_signal_3),
  .count(dca_signal_6),
  .value(dca_signal_0),
  .is_first_count(dca_signal_4),
  .is_last_count(dca_signal_2)
);

assign dca_signal_3 = clear | dca_signal_8;
assign dca_signal_6 = dca_signal_1;

always@(posedge clk, negedge rstnn)
begin
  if(rstnn==0)
    dca_signal_5 <= DCA_LPARA_1;
  else if(clear)
    dca_signal_5 <= DCA_LPARA_1;
  else if(enable)
  begin
    case(dca_signal_5)
      DCA_LPARA_1:
        if(storereg_wrequest)
          dca_signal_5 <= DCA_LPARA_0;
      DCA_LPARA_0:
        if(dca_signal_7 & store_tensor_row_rlast)
        begin
          if(dca_signal_2)
            dca_signal_5 <= DCA_LPARA_1;
          else
            dca_signal_5 <= DCA_LPARA_4;
        end
      DCA_LPARA_4:
        if(dca_signal_8)
          dca_signal_5 <= DCA_LPARA_1;
    endcase
  end
end

assign mreg_move_renable = dca_signal_1;

assign store_tensor_row_rready = (dca_signal_5==DCA_LPARA_0);
assign store_tensor_row_rdata = mreg_move_rdata_list1d;

assign storereg_wready = (dca_signal_5==DCA_LPARA_1);
assign busy = (dca_signal_5!=DCA_LPARA_1);

endmodule
