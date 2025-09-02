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
`include "fpir_define.vh"




module DCA_MODULE_21
(
  dca_port_3,
  dca_port_0,
  dca_port_4,
  dca_port_7,

  dca_port_5,
  dca_port_6,
  dca_port_1,
  dca_port_8,
  dca_port_2
);




parameter TENSOR_PARA = 0;

`include "dca_tensor_scalar_lpara.vb"

input wire dca_port_3, dca_port_0;
input wire dca_port_4;
input wire dca_port_7;

input wire dca_port_5;
input wire [BW_TENSOR_SCALAR-1:0] dca_port_6;
input wire [BW_TENSOR_SCALAR-1:0] dca_port_1;
output wire dca_port_8;
output wire [BW_TENSOR_SCALAR-1:0] dca_port_2;

localparam  DCA_LPARA_2 = TENSOR_BW_EXPONENT + 1;
localparam  DCA_LPARA_4 = TENSOR_BW_SIGNIFICAND; 
localparam  DCA_LPARA_1 = 2*DCA_LPARA_4-1;

`ifdef PACT_SUPPORT_FLOAT32
localparam  DCA_LPARA_0 = `MAX(TENSOR_BW_INTEGER, (DCA_LPARA_4+1));
`else
localparam  DCA_LPARA_0 = TENSOR_BW_INTEGER;
`endif
localparam  DCA_LPARA_5 = 2*DCA_LPARA_0;

wire dca_signal_21;
reg  [DCA_LPARA_0-1:0] dca_signal_24;
reg  [DCA_LPARA_0-1:0] dca_signal_11;
wire dca_signal_20;
wire dca_signal_22;
wire [DCA_LPARA_5-1:0] dca_signal_04;

wire signed [TENSOR_BW_INTEGER-1:0] dca_signal_03;
wire signed [TENSOR_BW_INTEGER-1:0] dca_signal_25;
wire signed [TENSOR_BW_INTEGER-1:0] dca_signal_12;
wire dca_signal_01;

wire dca_signal_05;
wire dca_signal_16;
wire [`BW_FPIR_TYPE-1:0] dca_signal_19;
wire [TENSOR_BW_SIGNIFICAND-1:0] dca_signal_15;
wire [TENSOR_BW_SIGNIFICAND-1:0] dca_signal_09;
wire [DCA_LPARA_2-1:0] dca_signal_14;

wire dca_signal_13;
wire dca_signal_18;
wire [`BW_FPIR_TYPE-1:0] dca_signal_17;
wire [DCA_LPARA_2-1:0] dca_signal_10;
wire dca_signal_23;

wire [DCA_LPARA_4-1:0] dca_signal_00;
wire [DCA_LPARA_4-1:0] dca_signal_07;
wire [DCA_LPARA_1-1:0] dca_signal_08;

wire dca_signal_06;
wire [BW_TENSOR_SCALAR-1:0] dca_signal_02;

ERVP_PIPELINED_MULTIPLIER
#(
  .BW_INPUT(DCA_LPARA_0),
  .BW_OUTPUT(DCA_LPARA_5),
  .USE_LIBRARY(0)
)
i_dca_instance_3
(
  .clk(dca_port_3),
  .rstnn(dca_port_0),
  .enable(dca_port_4),
  .stall(1'b 0),

  .input_wvalid(dca_signal_21),
  .input_wready(),
  .input_left(dca_signal_24),
  .input_right(dca_signal_11),

  .output_rvalid(dca_signal_20),
  .output_rready(dca_signal_22),
  .output_result(dca_signal_04),
  .output_upper(),
  .output_lower()
);

assign dca_signal_21 = dca_port_7? dca_signal_16 : dca_port_5;
assign dca_signal_22 = 1;

always@(*)
begin
  dca_signal_24 = $signed(dca_signal_03);
  dca_signal_11 = $signed(dca_signal_25);
  if(dca_port_7)
  begin
    dca_signal_24 = $unsigned(dca_signal_00);
    dca_signal_11 = $unsigned(dca_signal_07);
  end
  else
  begin
    dca_signal_24 = $signed(dca_signal_03);
    dca_signal_11 = $signed(dca_signal_25);
  end
end

assign dca_signal_03 = dca_port_6;
assign dca_signal_25 = dca_port_1;
assign dca_signal_01 = dca_signal_20;
assign dca_signal_12 = dca_signal_04;

`ifdef PACT_SUPPORT_FLOAT32

PACT_FMULTIPLIER_PIPELINE1
i_dca_instance_0
(
  .clk(dca_port_3),
  .rstnn(dca_port_0),
  .enable(dca_port_4),

  .in_valid(dca_port_5),
  .in_input0_float(dca_port_6),
  .in_input1_float(dca_port_1),

  .out_valid_early_case(dca_signal_05),
  .out_valid_normal_case(dca_signal_16),

  .out_early_special_case(dca_signal_19),
  .out_input0_significand(dca_signal_15),
  .out_input1_significand(dca_signal_09),
  .out_added_exponent(dca_signal_14)
);

PACT_FMULTIPLIER_PIPELINE2
i_dca_instance_2
(
  .clk(dca_port_3),
  .rstnn(dca_port_0),  
  .enable(dca_port_4),

  .in_valid_early_case(dca_signal_05),
  .in_valid_normal_case(dca_signal_16),
  .in_early_special_case(dca_signal_19),
  .in_input0_significand(dca_signal_15),
  .in_input1_significand(dca_signal_09),
  .in_added_exponent(dca_signal_14),

  .out_multiplier_input0(dca_signal_00),
  .out_multiplier_input1(dca_signal_07),

  .out_valid_early_case(dca_signal_13),
  .out_valid_normal_case(dca_signal_18),
  .out_early_special_case(dca_signal_17),
  .out_added_exponent(dca_signal_10),
  .out_multiplied_sign(dca_signal_23)
);

PACT_FMULTIPLIER_PIPELINE3
i_dca_instance_1
(
  .clk(dca_port_3),
  .rstnn(dca_port_0),
  .enable(dca_port_4),

  .in_valid_early_case(dca_signal_13),
  .in_valid_normal_case(dca_signal_18),
  .in_early_special_case(dca_signal_17),
  .in_added_exponent(dca_signal_10),
  .in_multiplied_sign(dca_signal_23),

  .in_multiplier_output0(dca_signal_08),

  .out_valid(dca_signal_06),
  .out_result(dca_signal_02)
);

assign dca_signal_08 = dca_signal_04;

`endif 

assign dca_port_8 = (dca_port_7)? dca_signal_06 : dca_signal_01;
assign dca_port_2 = (dca_port_7)? dca_signal_02 : $unsigned(dca_signal_12);

endmodule
