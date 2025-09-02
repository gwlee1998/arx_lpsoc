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



module DCA_MATRIX_LOAD2MREG
(
  clk,
  rstnn,
  clear,
  enable,
  busy,
  
  load_tensor_row_wvalid,
  load_tensor_row_wlast,
  load_tensor_row_wdata,
  load_tensor_row_wready,

  mreg_move_wenable,
  mreg_move_wdata_list1d,

  loadreg_rready,
  loadreg_rrequest
);



parameter MATRIX_SIZE_PARA = 4;
parameter TENSOR_PARA = 0;

`include "dca_matrix_dim_util.vb"
`include "dca_matrix_dim_lpara.vb"
`include "dca_tensor_scalar_lpara.vb"
`include "dca_tensor_dim_lpara.vb"

input wire clk;
input wire rstnn;
input wire clear;
input wire enable;
output wire busy;

input wire load_tensor_row_wvalid;
input wire load_tensor_row_wlast;
input wire [BW_TENSOR_ROW-1:0] load_tensor_row_wdata;
output wire load_tensor_row_wready;

localparam  DCA_LPARA_1 = BW_TENSOR_ROW;

output wire mreg_move_wenable;
output wire [DCA_LPARA_1-1:0] mreg_move_wdata_list1d;

output wire loadreg_rready;
input wire loadreg_rrequest;

genvar i;

wire dca_signal_8;
wire dca_signal_1;
wire dca_signal_3;

localparam  DCA_LPARA_4 = MATRIX_NUM_ROW;

wire dca_signal_5;
wire dca_signal_7;
wire [DCA_LPARA_4-1:0] dca_signal_0;
wire dca_signal_4;
wire dca_signal_2;

localparam  DCA_LPARA_5 = 2;
localparam  DCA_LPARA_0 = 0;
localparam  DCA_LPARA_2 = 1;
localparam  DCA_LPARA_3 = 2;

reg [DCA_LPARA_5-1:0] dca_signal_6;

assign dca_signal_8 = load_tensor_row_wvalid & load_tensor_row_wready;
assign dca_signal_1 = dca_signal_8 | (dca_signal_6==DCA_LPARA_2);
assign dca_signal_3 = dca_signal_1 & dca_signal_2;

ERVP_COUNTER_WITH_ONEHOT_ENCODING
#(
  .COUNT_LENGTH(DCA_LPARA_4)
)
i_dca_instance_0
(
  .clk(clk),
  .rstnn(rstnn),
  .enable(enable),
  .init(dca_signal_5),
  .count(dca_signal_7),
  .value(dca_signal_0),
  .is_first_count(dca_signal_4),
  .is_last_count(dca_signal_2)
);

assign dca_signal_5 = clear | dca_signal_3;
assign dca_signal_7 = dca_signal_1;

always@(posedge clk or negedge rstnn)
begin
  if(~rstnn)
    dca_signal_6 <= DCA_LPARA_0;
  else if(clear)
    dca_signal_6 <= DCA_LPARA_0;
  else if(enable)
    case(dca_signal_6)
      DCA_LPARA_0:
        if(dca_signal_8 & load_tensor_row_wlast)
        begin
          if(dca_signal_2)
            dca_signal_6 <= DCA_LPARA_3;
          else
            dca_signal_6 <= DCA_LPARA_2;
        end
      DCA_LPARA_2:
        if(dca_signal_3)
          dca_signal_6 <= DCA_LPARA_3;
      DCA_LPARA_3:
        if(loadreg_rrequest)
          dca_signal_6 <= DCA_LPARA_0;
    endcase
end

assign load_tensor_row_wready = (dca_signal_6==DCA_LPARA_0);

assign mreg_move_wenable = dca_signal_1;
generate
for(i=0; i<MATRIX_NUM_COL; i=i+1)
begin : i_wdata
  assign mreg_move_wdata_list1d[BW_TENSOR_SCALAR*i+:BW_TENSOR_SCALAR] = (dca_signal_6==DCA_LPARA_2)? TENSOR_ZERO : load_tensor_row_wdata[BW_TENSOR_SCALAR*i+:BW_TENSOR_SCALAR];
end
endgenerate

assign loadreg_rready = (dca_signal_6==DCA_LPARA_3);
assign busy = (dca_signal_6!=DCA_LPARA_3);

endmodule
