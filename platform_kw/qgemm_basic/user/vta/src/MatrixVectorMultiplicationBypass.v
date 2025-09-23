module MatrixVectorMultiplicationBypass(
  input         clock,
  input         io_valid_reset,
  input  [31:0] io_inp_data_bits_0_0,
  input  [31:0] io_inp_data_bits_0_1,
  input  [31:0] io_inp_data_bits_0_2,
  input  [31:0] io_inp_data_bits_0_3,
  input  [31:0] io_inp_data_bits_0_4,
  input  [31:0] io_inp_data_bits_0_5,
  input  [31:0] io_inp_data_bits_0_6,
  input  [31:0] io_inp_data_bits_0_7,
  input  [31:0] io_inp_data_bits_0_8,
  input  [31:0] io_inp_data_bits_0_9,
  input  [31:0] io_inp_data_bits_0_10,
  input  [31:0] io_inp_data_bits_0_11,
  input  [31:0] io_inp_data_bits_0_12,
  input  [31:0] io_inp_data_bits_0_13,
  input  [31:0] io_inp_data_bits_0_14,
  input  [31:0] io_inp_data_bits_0_15,
  input  [31:0] io_wgt_data_bits_0_0,
  input  [31:0] io_wgt_data_bits_0_1,
  input  [31:0] io_wgt_data_bits_0_2,
  input  [31:0] io_wgt_data_bits_0_3,
  input  [31:0] io_wgt_data_bits_0_4,
  input  [31:0] io_wgt_data_bits_0_5,
  input  [31:0] io_wgt_data_bits_0_6,
  input  [31:0] io_wgt_data_bits_0_7,
  input  [31:0] io_wgt_data_bits_0_8,
  input  [31:0] io_wgt_data_bits_0_9,
  input  [31:0] io_wgt_data_bits_0_10,
  input  [31:0] io_wgt_data_bits_0_11,
  input  [31:0] io_wgt_data_bits_0_12,
  input  [31:0] io_wgt_data_bits_0_13,
  input  [31:0] io_wgt_data_bits_0_14,
  input  [31:0] io_wgt_data_bits_0_15,
  input  [31:0] io_wgt_data_bits_1_0,
  input  [31:0] io_wgt_data_bits_1_1,
  input  [31:0] io_wgt_data_bits_1_2,
  input  [31:0] io_wgt_data_bits_1_3,
  input  [31:0] io_wgt_data_bits_1_4,
  input  [31:0] io_wgt_data_bits_1_5,
  input  [31:0] io_wgt_data_bits_1_6,
  input  [31:0] io_wgt_data_bits_1_7,
  input  [31:0] io_wgt_data_bits_1_8,
  input  [31:0] io_wgt_data_bits_1_9,
  input  [31:0] io_wgt_data_bits_1_10,
  input  [31:0] io_wgt_data_bits_1_11,
  input  [31:0] io_wgt_data_bits_1_12,
  input  [31:0] io_wgt_data_bits_1_13,
  input  [31:0] io_wgt_data_bits_1_14,
  input  [31:0] io_wgt_data_bits_1_15,
  input  [31:0] io_wgt_data_bits_2_0,
  input  [31:0] io_wgt_data_bits_2_1,
  input  [31:0] io_wgt_data_bits_2_2,
  input  [31:0] io_wgt_data_bits_2_3,
  input  [31:0] io_wgt_data_bits_2_4,
  input  [31:0] io_wgt_data_bits_2_5,
  input  [31:0] io_wgt_data_bits_2_6,
  input  [31:0] io_wgt_data_bits_2_7,
  input  [31:0] io_wgt_data_bits_2_8,
  input  [31:0] io_wgt_data_bits_2_9,
  input  [31:0] io_wgt_data_bits_2_10,
  input  [31:0] io_wgt_data_bits_2_11,
  input  [31:0] io_wgt_data_bits_2_12,
  input  [31:0] io_wgt_data_bits_2_13,
  input  [31:0] io_wgt_data_bits_2_14,
  input  [31:0] io_wgt_data_bits_2_15,
  input  [31:0] io_wgt_data_bits_3_0,
  input  [31:0] io_wgt_data_bits_3_1,
  input  [31:0] io_wgt_data_bits_3_2,
  input  [31:0] io_wgt_data_bits_3_3,
  input  [31:0] io_wgt_data_bits_3_4,
  input  [31:0] io_wgt_data_bits_3_5,
  input  [31:0] io_wgt_data_bits_3_6,
  input  [31:0] io_wgt_data_bits_3_7,
  input  [31:0] io_wgt_data_bits_3_8,
  input  [31:0] io_wgt_data_bits_3_9,
  input  [31:0] io_wgt_data_bits_3_10,
  input  [31:0] io_wgt_data_bits_3_11,
  input  [31:0] io_wgt_data_bits_3_12,
  input  [31:0] io_wgt_data_bits_3_13,
  input  [31:0] io_wgt_data_bits_3_14,
  input  [31:0] io_wgt_data_bits_3_15,
  input  [31:0] io_wgt_data_bits_4_0,
  input  [31:0] io_wgt_data_bits_4_1,
  input  [31:0] io_wgt_data_bits_4_2,
  input  [31:0] io_wgt_data_bits_4_3,
  input  [31:0] io_wgt_data_bits_4_4,
  input  [31:0] io_wgt_data_bits_4_5,
  input  [31:0] io_wgt_data_bits_4_6,
  input  [31:0] io_wgt_data_bits_4_7,
  input  [31:0] io_wgt_data_bits_4_8,
  input  [31:0] io_wgt_data_bits_4_9,
  input  [31:0] io_wgt_data_bits_4_10,
  input  [31:0] io_wgt_data_bits_4_11,
  input  [31:0] io_wgt_data_bits_4_12,
  input  [31:0] io_wgt_data_bits_4_13,
  input  [31:0] io_wgt_data_bits_4_14,
  input  [31:0] io_wgt_data_bits_4_15,
  input  [31:0] io_wgt_data_bits_5_0,
  input  [31:0] io_wgt_data_bits_5_1,
  input  [31:0] io_wgt_data_bits_5_2,
  input  [31:0] io_wgt_data_bits_5_3,
  input  [31:0] io_wgt_data_bits_5_4,
  input  [31:0] io_wgt_data_bits_5_5,
  input  [31:0] io_wgt_data_bits_5_6,
  input  [31:0] io_wgt_data_bits_5_7,
  input  [31:0] io_wgt_data_bits_5_8,
  input  [31:0] io_wgt_data_bits_5_9,
  input  [31:0] io_wgt_data_bits_5_10,
  input  [31:0] io_wgt_data_bits_5_11,
  input  [31:0] io_wgt_data_bits_5_12,
  input  [31:0] io_wgt_data_bits_5_13,
  input  [31:0] io_wgt_data_bits_5_14,
  input  [31:0] io_wgt_data_bits_5_15,
  input  [31:0] io_wgt_data_bits_6_0,
  input  [31:0] io_wgt_data_bits_6_1,
  input  [31:0] io_wgt_data_bits_6_2,
  input  [31:0] io_wgt_data_bits_6_3,
  input  [31:0] io_wgt_data_bits_6_4,
  input  [31:0] io_wgt_data_bits_6_5,
  input  [31:0] io_wgt_data_bits_6_6,
  input  [31:0] io_wgt_data_bits_6_7,
  input  [31:0] io_wgt_data_bits_6_8,
  input  [31:0] io_wgt_data_bits_6_9,
  input  [31:0] io_wgt_data_bits_6_10,
  input  [31:0] io_wgt_data_bits_6_11,
  input  [31:0] io_wgt_data_bits_6_12,
  input  [31:0] io_wgt_data_bits_6_13,
  input  [31:0] io_wgt_data_bits_6_14,
  input  [31:0] io_wgt_data_bits_6_15,
  input  [31:0] io_wgt_data_bits_7_0,
  input  [31:0] io_wgt_data_bits_7_1,
  input  [31:0] io_wgt_data_bits_7_2,
  input  [31:0] io_wgt_data_bits_7_3,
  input  [31:0] io_wgt_data_bits_7_4,
  input  [31:0] io_wgt_data_bits_7_5,
  input  [31:0] io_wgt_data_bits_7_6,
  input  [31:0] io_wgt_data_bits_7_7,
  input  [31:0] io_wgt_data_bits_7_8,
  input  [31:0] io_wgt_data_bits_7_9,
  input  [31:0] io_wgt_data_bits_7_10,
  input  [31:0] io_wgt_data_bits_7_11,
  input  [31:0] io_wgt_data_bits_7_12,
  input  [31:0] io_wgt_data_bits_7_13,
  input  [31:0] io_wgt_data_bits_7_14,
  input  [31:0] io_wgt_data_bits_7_15,
  input  [31:0] io_wgt_data_bits_8_0,
  input  [31:0] io_wgt_data_bits_8_1,
  input  [31:0] io_wgt_data_bits_8_2,
  input  [31:0] io_wgt_data_bits_8_3,
  input  [31:0] io_wgt_data_bits_8_4,
  input  [31:0] io_wgt_data_bits_8_5,
  input  [31:0] io_wgt_data_bits_8_6,
  input  [31:0] io_wgt_data_bits_8_7,
  input  [31:0] io_wgt_data_bits_8_8,
  input  [31:0] io_wgt_data_bits_8_9,
  input  [31:0] io_wgt_data_bits_8_10,
  input  [31:0] io_wgt_data_bits_8_11,
  input  [31:0] io_wgt_data_bits_8_12,
  input  [31:0] io_wgt_data_bits_8_13,
  input  [31:0] io_wgt_data_bits_8_14,
  input  [31:0] io_wgt_data_bits_8_15,
  input  [31:0] io_wgt_data_bits_9_0,
  input  [31:0] io_wgt_data_bits_9_1,
  input  [31:0] io_wgt_data_bits_9_2,
  input  [31:0] io_wgt_data_bits_9_3,
  input  [31:0] io_wgt_data_bits_9_4,
  input  [31:0] io_wgt_data_bits_9_5,
  input  [31:0] io_wgt_data_bits_9_6,
  input  [31:0] io_wgt_data_bits_9_7,
  input  [31:0] io_wgt_data_bits_9_8,
  input  [31:0] io_wgt_data_bits_9_9,
  input  [31:0] io_wgt_data_bits_9_10,
  input  [31:0] io_wgt_data_bits_9_11,
  input  [31:0] io_wgt_data_bits_9_12,
  input  [31:0] io_wgt_data_bits_9_13,
  input  [31:0] io_wgt_data_bits_9_14,
  input  [31:0] io_wgt_data_bits_9_15,
  input  [31:0] io_wgt_data_bits_10_0,
  input  [31:0] io_wgt_data_bits_10_1,
  input  [31:0] io_wgt_data_bits_10_2,
  input  [31:0] io_wgt_data_bits_10_3,
  input  [31:0] io_wgt_data_bits_10_4,
  input  [31:0] io_wgt_data_bits_10_5,
  input  [31:0] io_wgt_data_bits_10_6,
  input  [31:0] io_wgt_data_bits_10_7,
  input  [31:0] io_wgt_data_bits_10_8,
  input  [31:0] io_wgt_data_bits_10_9,
  input  [31:0] io_wgt_data_bits_10_10,
  input  [31:0] io_wgt_data_bits_10_11,
  input  [31:0] io_wgt_data_bits_10_12,
  input  [31:0] io_wgt_data_bits_10_13,
  input  [31:0] io_wgt_data_bits_10_14,
  input  [31:0] io_wgt_data_bits_10_15,
  input  [31:0] io_wgt_data_bits_11_0,
  input  [31:0] io_wgt_data_bits_11_1,
  input  [31:0] io_wgt_data_bits_11_2,
  input  [31:0] io_wgt_data_bits_11_3,
  input  [31:0] io_wgt_data_bits_11_4,
  input  [31:0] io_wgt_data_bits_11_5,
  input  [31:0] io_wgt_data_bits_11_6,
  input  [31:0] io_wgt_data_bits_11_7,
  input  [31:0] io_wgt_data_bits_11_8,
  input  [31:0] io_wgt_data_bits_11_9,
  input  [31:0] io_wgt_data_bits_11_10,
  input  [31:0] io_wgt_data_bits_11_11,
  input  [31:0] io_wgt_data_bits_11_12,
  input  [31:0] io_wgt_data_bits_11_13,
  input  [31:0] io_wgt_data_bits_11_14,
  input  [31:0] io_wgt_data_bits_11_15,
  input  [31:0] io_wgt_data_bits_12_0,
  input  [31:0] io_wgt_data_bits_12_1,
  input  [31:0] io_wgt_data_bits_12_2,
  input  [31:0] io_wgt_data_bits_12_3,
  input  [31:0] io_wgt_data_bits_12_4,
  input  [31:0] io_wgt_data_bits_12_5,
  input  [31:0] io_wgt_data_bits_12_6,
  input  [31:0] io_wgt_data_bits_12_7,
  input  [31:0] io_wgt_data_bits_12_8,
  input  [31:0] io_wgt_data_bits_12_9,
  input  [31:0] io_wgt_data_bits_12_10,
  input  [31:0] io_wgt_data_bits_12_11,
  input  [31:0] io_wgt_data_bits_12_12,
  input  [31:0] io_wgt_data_bits_12_13,
  input  [31:0] io_wgt_data_bits_12_14,
  input  [31:0] io_wgt_data_bits_12_15,
  input  [31:0] io_wgt_data_bits_13_0,
  input  [31:0] io_wgt_data_bits_13_1,
  input  [31:0] io_wgt_data_bits_13_2,
  input  [31:0] io_wgt_data_bits_13_3,
  input  [31:0] io_wgt_data_bits_13_4,
  input  [31:0] io_wgt_data_bits_13_5,
  input  [31:0] io_wgt_data_bits_13_6,
  input  [31:0] io_wgt_data_bits_13_7,
  input  [31:0] io_wgt_data_bits_13_8,
  input  [31:0] io_wgt_data_bits_13_9,
  input  [31:0] io_wgt_data_bits_13_10,
  input  [31:0] io_wgt_data_bits_13_11,
  input  [31:0] io_wgt_data_bits_13_12,
  input  [31:0] io_wgt_data_bits_13_13,
  input  [31:0] io_wgt_data_bits_13_14,
  input  [31:0] io_wgt_data_bits_13_15,
  input  [31:0] io_wgt_data_bits_14_0,
  input  [31:0] io_wgt_data_bits_14_1,
  input  [31:0] io_wgt_data_bits_14_2,
  input  [31:0] io_wgt_data_bits_14_3,
  input  [31:0] io_wgt_data_bits_14_4,
  input  [31:0] io_wgt_data_bits_14_5,
  input  [31:0] io_wgt_data_bits_14_6,
  input  [31:0] io_wgt_data_bits_14_7,
  input  [31:0] io_wgt_data_bits_14_8,
  input  [31:0] io_wgt_data_bits_14_9,
  input  [31:0] io_wgt_data_bits_14_10,
  input  [31:0] io_wgt_data_bits_14_11,
  input  [31:0] io_wgt_data_bits_14_12,
  input  [31:0] io_wgt_data_bits_14_13,
  input  [31:0] io_wgt_data_bits_14_14,
  input  [31:0] io_wgt_data_bits_14_15,
  input  [31:0] io_wgt_data_bits_15_0,
  input  [31:0] io_wgt_data_bits_15_1,
  input  [31:0] io_wgt_data_bits_15_2,
  input  [31:0] io_wgt_data_bits_15_3,
  input  [31:0] io_wgt_data_bits_15_4,
  input  [31:0] io_wgt_data_bits_15_5,
  input  [31:0] io_wgt_data_bits_15_6,
  input  [31:0] io_wgt_data_bits_15_7,
  input  [31:0] io_wgt_data_bits_15_8,
  input  [31:0] io_wgt_data_bits_15_9,
  input  [31:0] io_wgt_data_bits_15_10,
  input  [31:0] io_wgt_data_bits_15_11,
  input  [31:0] io_wgt_data_bits_15_12,
  input  [31:0] io_wgt_data_bits_15_13,
  input  [31:0] io_wgt_data_bits_15_14,
  input  [31:0] io_wgt_data_bits_15_15,
  input         io_acc_i_data_valid,
  input  [31:0] io_acc_i_data_bits_0_0,
  input  [31:0] io_acc_i_data_bits_0_1,
  input  [31:0] io_acc_i_data_bits_0_2,
  input  [31:0] io_acc_i_data_bits_0_3,
  input  [31:0] io_acc_i_data_bits_0_4,
  input  [31:0] io_acc_i_data_bits_0_5,
  input  [31:0] io_acc_i_data_bits_0_6,
  input  [31:0] io_acc_i_data_bits_0_7,
  input  [31:0] io_acc_i_data_bits_0_8,
  input  [31:0] io_acc_i_data_bits_0_9,
  input  [31:0] io_acc_i_data_bits_0_10,
  input  [31:0] io_acc_i_data_bits_0_11,
  input  [31:0] io_acc_i_data_bits_0_12,
  input  [31:0] io_acc_i_data_bits_0_13,
  input  [31:0] io_acc_i_data_bits_0_14,
  input  [31:0] io_acc_i_data_bits_0_15,
  output        io_acc_o_data_valid,
  output [31:0] io_acc_o_data_bits_0_0,
  output [31:0] io_acc_o_data_bits_0_1,
  output [31:0] io_acc_o_data_bits_0_2,
  output [31:0] io_acc_o_data_bits_0_3,
  output [31:0] io_acc_o_data_bits_0_4,
  output [31:0] io_acc_o_data_bits_0_5,
  output [31:0] io_acc_o_data_bits_0_6,
  output [31:0] io_acc_o_data_bits_0_7,
  output [31:0] io_acc_o_data_bits_0_8,
  output [31:0] io_acc_o_data_bits_0_9,
  output [31:0] io_acc_o_data_bits_0_10,
  output [31:0] io_acc_o_data_bits_0_11,
  output [31:0] io_acc_o_data_bits_0_12,
  output [31:0] io_acc_o_data_bits_0_13,
  output [31:0] io_acc_o_data_bits_0_14,
  output [31:0] io_acc_o_data_bits_0_15,
  output        io_out_data_valid,
  output [31:0] io_out_data_bits_0_0,
  output [31:0] io_out_data_bits_0_1,
  output [31:0] io_out_data_bits_0_2,
  output [31:0] io_out_data_bits_0_3,
  output [31:0] io_out_data_bits_0_4,
  output [31:0] io_out_data_bits_0_5,
  output [31:0] io_out_data_bits_0_6,
  output [31:0] io_out_data_bits_0_7,
  output [31:0] io_out_data_bits_0_8,
  output [31:0] io_out_data_bits_0_9,
  output [31:0] io_out_data_bits_0_10,
  output [31:0] io_out_data_bits_0_11,
  output [31:0] io_out_data_bits_0_12,
  output [31:0] io_out_data_bits_0_13,
  output [31:0] io_out_data_bits_0_14,
  output [31:0] io_out_data_bits_0_15,
  input         io_bypass_cond
);
`ifdef RANDOMIZE_REG_INIT
  reg [31:0] _RAND_0;
  reg [31:0] _RAND_1;
  reg [31:0] _RAND_2;
  reg [31:0] _RAND_3;
  reg [31:0] _RAND_4;
  reg [31:0] _RAND_5;
  reg [31:0] _RAND_6;
  reg [31:0] _RAND_7;
  reg [31:0] _RAND_8;
  reg [31:0] _RAND_9;
  reg [31:0] _RAND_10;
  reg [31:0] _RAND_11;
  reg [31:0] _RAND_12;
  reg [31:0] _RAND_13;
  reg [31:0] _RAND_14;
  reg [31:0] _RAND_15;
`endif // RANDOMIZE_REG_INIT
  wire  dot_0_0_clock; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_0_io_a_0; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_0_io_a_1; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_0_io_a_2; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_0_io_a_3; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_0_io_a_4; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_0_io_a_5; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_0_io_a_6; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_0_io_a_7; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_0_io_a_8; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_0_io_a_9; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_0_io_a_10; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_0_io_a_11; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_0_io_a_12; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_0_io_a_13; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_0_io_a_14; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_0_io_a_15; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_0_io_b_0; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_0_io_b_1; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_0_io_b_2; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_0_io_b_3; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_0_io_b_4; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_0_io_b_5; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_0_io_b_6; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_0_io_b_7; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_0_io_b_8; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_0_io_b_9; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_0_io_b_10; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_0_io_b_11; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_0_io_b_12; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_0_io_b_13; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_0_io_b_14; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_0_io_b_15; // @[TensorGemm.scala 198:11]
  wire [68:0] dot_0_0_io_y; // @[TensorGemm.scala 198:11]
  wire  dot_0_1_clock; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_1_io_a_0; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_1_io_a_1; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_1_io_a_2; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_1_io_a_3; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_1_io_a_4; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_1_io_a_5; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_1_io_a_6; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_1_io_a_7; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_1_io_a_8; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_1_io_a_9; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_1_io_a_10; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_1_io_a_11; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_1_io_a_12; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_1_io_a_13; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_1_io_a_14; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_1_io_a_15; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_1_io_b_0; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_1_io_b_1; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_1_io_b_2; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_1_io_b_3; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_1_io_b_4; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_1_io_b_5; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_1_io_b_6; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_1_io_b_7; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_1_io_b_8; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_1_io_b_9; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_1_io_b_10; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_1_io_b_11; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_1_io_b_12; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_1_io_b_13; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_1_io_b_14; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_1_io_b_15; // @[TensorGemm.scala 198:11]
  wire [68:0] dot_0_1_io_y; // @[TensorGemm.scala 198:11]
  wire  dot_0_2_clock; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_2_io_a_0; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_2_io_a_1; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_2_io_a_2; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_2_io_a_3; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_2_io_a_4; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_2_io_a_5; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_2_io_a_6; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_2_io_a_7; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_2_io_a_8; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_2_io_a_9; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_2_io_a_10; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_2_io_a_11; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_2_io_a_12; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_2_io_a_13; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_2_io_a_14; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_2_io_a_15; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_2_io_b_0; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_2_io_b_1; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_2_io_b_2; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_2_io_b_3; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_2_io_b_4; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_2_io_b_5; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_2_io_b_6; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_2_io_b_7; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_2_io_b_8; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_2_io_b_9; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_2_io_b_10; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_2_io_b_11; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_2_io_b_12; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_2_io_b_13; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_2_io_b_14; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_2_io_b_15; // @[TensorGemm.scala 198:11]
  wire [68:0] dot_0_2_io_y; // @[TensorGemm.scala 198:11]
  wire  dot_0_3_clock; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_3_io_a_0; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_3_io_a_1; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_3_io_a_2; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_3_io_a_3; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_3_io_a_4; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_3_io_a_5; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_3_io_a_6; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_3_io_a_7; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_3_io_a_8; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_3_io_a_9; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_3_io_a_10; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_3_io_a_11; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_3_io_a_12; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_3_io_a_13; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_3_io_a_14; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_3_io_a_15; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_3_io_b_0; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_3_io_b_1; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_3_io_b_2; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_3_io_b_3; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_3_io_b_4; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_3_io_b_5; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_3_io_b_6; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_3_io_b_7; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_3_io_b_8; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_3_io_b_9; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_3_io_b_10; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_3_io_b_11; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_3_io_b_12; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_3_io_b_13; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_3_io_b_14; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_3_io_b_15; // @[TensorGemm.scala 198:11]
  wire [68:0] dot_0_3_io_y; // @[TensorGemm.scala 198:11]
  wire  dot_0_4_clock; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_4_io_a_0; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_4_io_a_1; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_4_io_a_2; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_4_io_a_3; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_4_io_a_4; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_4_io_a_5; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_4_io_a_6; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_4_io_a_7; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_4_io_a_8; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_4_io_a_9; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_4_io_a_10; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_4_io_a_11; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_4_io_a_12; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_4_io_a_13; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_4_io_a_14; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_4_io_a_15; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_4_io_b_0; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_4_io_b_1; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_4_io_b_2; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_4_io_b_3; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_4_io_b_4; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_4_io_b_5; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_4_io_b_6; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_4_io_b_7; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_4_io_b_8; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_4_io_b_9; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_4_io_b_10; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_4_io_b_11; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_4_io_b_12; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_4_io_b_13; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_4_io_b_14; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_4_io_b_15; // @[TensorGemm.scala 198:11]
  wire [68:0] dot_0_4_io_y; // @[TensorGemm.scala 198:11]
  wire  dot_0_5_clock; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_5_io_a_0; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_5_io_a_1; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_5_io_a_2; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_5_io_a_3; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_5_io_a_4; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_5_io_a_5; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_5_io_a_6; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_5_io_a_7; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_5_io_a_8; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_5_io_a_9; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_5_io_a_10; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_5_io_a_11; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_5_io_a_12; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_5_io_a_13; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_5_io_a_14; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_5_io_a_15; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_5_io_b_0; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_5_io_b_1; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_5_io_b_2; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_5_io_b_3; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_5_io_b_4; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_5_io_b_5; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_5_io_b_6; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_5_io_b_7; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_5_io_b_8; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_5_io_b_9; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_5_io_b_10; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_5_io_b_11; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_5_io_b_12; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_5_io_b_13; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_5_io_b_14; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_5_io_b_15; // @[TensorGemm.scala 198:11]
  wire [68:0] dot_0_5_io_y; // @[TensorGemm.scala 198:11]
  wire  dot_0_6_clock; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_6_io_a_0; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_6_io_a_1; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_6_io_a_2; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_6_io_a_3; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_6_io_a_4; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_6_io_a_5; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_6_io_a_6; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_6_io_a_7; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_6_io_a_8; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_6_io_a_9; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_6_io_a_10; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_6_io_a_11; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_6_io_a_12; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_6_io_a_13; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_6_io_a_14; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_6_io_a_15; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_6_io_b_0; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_6_io_b_1; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_6_io_b_2; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_6_io_b_3; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_6_io_b_4; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_6_io_b_5; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_6_io_b_6; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_6_io_b_7; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_6_io_b_8; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_6_io_b_9; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_6_io_b_10; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_6_io_b_11; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_6_io_b_12; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_6_io_b_13; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_6_io_b_14; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_6_io_b_15; // @[TensorGemm.scala 198:11]
  wire [68:0] dot_0_6_io_y; // @[TensorGemm.scala 198:11]
  wire  dot_0_7_clock; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_7_io_a_0; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_7_io_a_1; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_7_io_a_2; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_7_io_a_3; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_7_io_a_4; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_7_io_a_5; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_7_io_a_6; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_7_io_a_7; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_7_io_a_8; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_7_io_a_9; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_7_io_a_10; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_7_io_a_11; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_7_io_a_12; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_7_io_a_13; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_7_io_a_14; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_7_io_a_15; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_7_io_b_0; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_7_io_b_1; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_7_io_b_2; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_7_io_b_3; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_7_io_b_4; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_7_io_b_5; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_7_io_b_6; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_7_io_b_7; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_7_io_b_8; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_7_io_b_9; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_7_io_b_10; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_7_io_b_11; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_7_io_b_12; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_7_io_b_13; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_7_io_b_14; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_7_io_b_15; // @[TensorGemm.scala 198:11]
  wire [68:0] dot_0_7_io_y; // @[TensorGemm.scala 198:11]
  wire  dot_0_8_clock; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_8_io_a_0; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_8_io_a_1; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_8_io_a_2; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_8_io_a_3; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_8_io_a_4; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_8_io_a_5; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_8_io_a_6; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_8_io_a_7; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_8_io_a_8; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_8_io_a_9; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_8_io_a_10; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_8_io_a_11; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_8_io_a_12; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_8_io_a_13; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_8_io_a_14; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_8_io_a_15; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_8_io_b_0; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_8_io_b_1; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_8_io_b_2; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_8_io_b_3; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_8_io_b_4; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_8_io_b_5; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_8_io_b_6; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_8_io_b_7; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_8_io_b_8; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_8_io_b_9; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_8_io_b_10; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_8_io_b_11; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_8_io_b_12; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_8_io_b_13; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_8_io_b_14; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_8_io_b_15; // @[TensorGemm.scala 198:11]
  wire [68:0] dot_0_8_io_y; // @[TensorGemm.scala 198:11]
  wire  dot_0_9_clock; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_9_io_a_0; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_9_io_a_1; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_9_io_a_2; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_9_io_a_3; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_9_io_a_4; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_9_io_a_5; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_9_io_a_6; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_9_io_a_7; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_9_io_a_8; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_9_io_a_9; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_9_io_a_10; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_9_io_a_11; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_9_io_a_12; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_9_io_a_13; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_9_io_a_14; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_9_io_a_15; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_9_io_b_0; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_9_io_b_1; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_9_io_b_2; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_9_io_b_3; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_9_io_b_4; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_9_io_b_5; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_9_io_b_6; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_9_io_b_7; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_9_io_b_8; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_9_io_b_9; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_9_io_b_10; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_9_io_b_11; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_9_io_b_12; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_9_io_b_13; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_9_io_b_14; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_9_io_b_15; // @[TensorGemm.scala 198:11]
  wire [68:0] dot_0_9_io_y; // @[TensorGemm.scala 198:11]
  wire  dot_0_10_clock; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_10_io_a_0; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_10_io_a_1; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_10_io_a_2; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_10_io_a_3; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_10_io_a_4; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_10_io_a_5; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_10_io_a_6; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_10_io_a_7; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_10_io_a_8; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_10_io_a_9; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_10_io_a_10; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_10_io_a_11; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_10_io_a_12; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_10_io_a_13; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_10_io_a_14; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_10_io_a_15; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_10_io_b_0; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_10_io_b_1; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_10_io_b_2; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_10_io_b_3; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_10_io_b_4; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_10_io_b_5; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_10_io_b_6; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_10_io_b_7; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_10_io_b_8; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_10_io_b_9; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_10_io_b_10; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_10_io_b_11; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_10_io_b_12; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_10_io_b_13; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_10_io_b_14; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_10_io_b_15; // @[TensorGemm.scala 198:11]
  wire [68:0] dot_0_10_io_y; // @[TensorGemm.scala 198:11]
  wire  dot_0_11_clock; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_11_io_a_0; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_11_io_a_1; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_11_io_a_2; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_11_io_a_3; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_11_io_a_4; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_11_io_a_5; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_11_io_a_6; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_11_io_a_7; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_11_io_a_8; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_11_io_a_9; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_11_io_a_10; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_11_io_a_11; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_11_io_a_12; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_11_io_a_13; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_11_io_a_14; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_11_io_a_15; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_11_io_b_0; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_11_io_b_1; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_11_io_b_2; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_11_io_b_3; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_11_io_b_4; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_11_io_b_5; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_11_io_b_6; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_11_io_b_7; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_11_io_b_8; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_11_io_b_9; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_11_io_b_10; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_11_io_b_11; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_11_io_b_12; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_11_io_b_13; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_11_io_b_14; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_11_io_b_15; // @[TensorGemm.scala 198:11]
  wire [68:0] dot_0_11_io_y; // @[TensorGemm.scala 198:11]
  wire  dot_0_12_clock; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_12_io_a_0; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_12_io_a_1; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_12_io_a_2; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_12_io_a_3; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_12_io_a_4; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_12_io_a_5; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_12_io_a_6; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_12_io_a_7; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_12_io_a_8; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_12_io_a_9; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_12_io_a_10; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_12_io_a_11; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_12_io_a_12; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_12_io_a_13; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_12_io_a_14; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_12_io_a_15; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_12_io_b_0; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_12_io_b_1; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_12_io_b_2; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_12_io_b_3; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_12_io_b_4; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_12_io_b_5; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_12_io_b_6; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_12_io_b_7; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_12_io_b_8; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_12_io_b_9; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_12_io_b_10; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_12_io_b_11; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_12_io_b_12; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_12_io_b_13; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_12_io_b_14; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_12_io_b_15; // @[TensorGemm.scala 198:11]
  wire [68:0] dot_0_12_io_y; // @[TensorGemm.scala 198:11]
  wire  dot_0_13_clock; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_13_io_a_0; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_13_io_a_1; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_13_io_a_2; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_13_io_a_3; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_13_io_a_4; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_13_io_a_5; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_13_io_a_6; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_13_io_a_7; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_13_io_a_8; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_13_io_a_9; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_13_io_a_10; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_13_io_a_11; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_13_io_a_12; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_13_io_a_13; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_13_io_a_14; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_13_io_a_15; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_13_io_b_0; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_13_io_b_1; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_13_io_b_2; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_13_io_b_3; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_13_io_b_4; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_13_io_b_5; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_13_io_b_6; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_13_io_b_7; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_13_io_b_8; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_13_io_b_9; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_13_io_b_10; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_13_io_b_11; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_13_io_b_12; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_13_io_b_13; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_13_io_b_14; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_13_io_b_15; // @[TensorGemm.scala 198:11]
  wire [68:0] dot_0_13_io_y; // @[TensorGemm.scala 198:11]
  wire  dot_0_14_clock; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_14_io_a_0; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_14_io_a_1; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_14_io_a_2; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_14_io_a_3; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_14_io_a_4; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_14_io_a_5; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_14_io_a_6; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_14_io_a_7; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_14_io_a_8; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_14_io_a_9; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_14_io_a_10; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_14_io_a_11; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_14_io_a_12; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_14_io_a_13; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_14_io_a_14; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_14_io_a_15; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_14_io_b_0; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_14_io_b_1; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_14_io_b_2; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_14_io_b_3; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_14_io_b_4; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_14_io_b_5; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_14_io_b_6; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_14_io_b_7; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_14_io_b_8; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_14_io_b_9; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_14_io_b_10; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_14_io_b_11; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_14_io_b_12; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_14_io_b_13; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_14_io_b_14; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_14_io_b_15; // @[TensorGemm.scala 198:11]
  wire [68:0] dot_0_14_io_y; // @[TensorGemm.scala 198:11]
  wire  dot_0_15_clock; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_15_io_a_0; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_15_io_a_1; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_15_io_a_2; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_15_io_a_3; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_15_io_a_4; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_15_io_a_5; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_15_io_a_6; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_15_io_a_7; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_15_io_a_8; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_15_io_a_9; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_15_io_a_10; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_15_io_a_11; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_15_io_a_12; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_15_io_a_13; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_15_io_a_14; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_15_io_a_15; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_15_io_b_0; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_15_io_b_1; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_15_io_b_2; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_15_io_b_3; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_15_io_b_4; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_15_io_b_5; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_15_io_b_6; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_15_io_b_7; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_15_io_b_8; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_15_io_b_9; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_15_io_b_10; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_15_io_b_11; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_15_io_b_12; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_15_io_b_13; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_15_io_b_14; // @[TensorGemm.scala 198:11]
  wire [31:0] dot_0_15_io_b_15; // @[TensorGemm.scala 198:11]
  wire [68:0] dot_0_15_io_y; // @[TensorGemm.scala 198:11]
  reg [31:0] last_acc_write_0_0; // @[TensorGemm.scala 200:62]
  reg [31:0] last_acc_write_0_1; // @[TensorGemm.scala 200:62]
  reg [31:0] last_acc_write_0_2; // @[TensorGemm.scala 200:62]
  reg [31:0] last_acc_write_0_3; // @[TensorGemm.scala 200:62]
  reg [31:0] last_acc_write_0_4; // @[TensorGemm.scala 200:62]
  reg [31:0] last_acc_write_0_5; // @[TensorGemm.scala 200:62]
  reg [31:0] last_acc_write_0_6; // @[TensorGemm.scala 200:62]
  reg [31:0] last_acc_write_0_7; // @[TensorGemm.scala 200:62]
  reg [31:0] last_acc_write_0_8; // @[TensorGemm.scala 200:62]
  reg [31:0] last_acc_write_0_9; // @[TensorGemm.scala 200:62]
  reg [31:0] last_acc_write_0_10; // @[TensorGemm.scala 200:62]
  reg [31:0] last_acc_write_0_11; // @[TensorGemm.scala 200:62]
  reg [31:0] last_acc_write_0_12; // @[TensorGemm.scala 200:62]
  reg [31:0] last_acc_write_0_13; // @[TensorGemm.scala 200:62]
  reg [31:0] last_acc_write_0_14; // @[TensorGemm.scala 200:62]
  reg [31:0] last_acc_write_0_15; // @[TensorGemm.scala 200:62]
  wire [31:0] byp = io_bypass_cond ? $signed(last_acc_write_0_0) : $signed(io_acc_i_data_bits_0_0); // @[TensorGemm.scala 208:20]
  wire [68:0] _GEN_0 = {{37{byp[31]}},byp}; // @[TensorGemm.scala 209:24]
  wire [68:0] _add_0_0_T_2 = $signed(_GEN_0) + $signed(dot_0_0_io_y); // @[TensorGemm.scala 209:24]
  wire [31:0] add_0_0 = _add_0_0_T_2[31:0]; // @[TensorGemm.scala 199:52 TensorGemm.scala 209:17]
  wire [31:0] byp_1 = io_bypass_cond ? $signed(last_acc_write_0_1) : $signed(io_acc_i_data_bits_0_1); // @[TensorGemm.scala 208:20]
  wire [68:0] _GEN_2 = {{37{byp_1[31]}},byp_1}; // @[TensorGemm.scala 209:24]
  wire [68:0] _add_0_1_T_2 = $signed(_GEN_2) + $signed(dot_0_1_io_y); // @[TensorGemm.scala 209:24]
  wire [31:0] add_0_1 = _add_0_1_T_2[31:0]; // @[TensorGemm.scala 199:52 TensorGemm.scala 209:17]
  wire [31:0] byp_2 = io_bypass_cond ? $signed(last_acc_write_0_2) : $signed(io_acc_i_data_bits_0_2); // @[TensorGemm.scala 208:20]
  wire [68:0] _GEN_4 = {{37{byp_2[31]}},byp_2}; // @[TensorGemm.scala 209:24]
  wire [68:0] _add_0_2_T_2 = $signed(_GEN_4) + $signed(dot_0_2_io_y); // @[TensorGemm.scala 209:24]
  wire [31:0] add_0_2 = _add_0_2_T_2[31:0]; // @[TensorGemm.scala 199:52 TensorGemm.scala 209:17]
  wire [31:0] byp_3 = io_bypass_cond ? $signed(last_acc_write_0_3) : $signed(io_acc_i_data_bits_0_3); // @[TensorGemm.scala 208:20]
  wire [68:0] _GEN_6 = {{37{byp_3[31]}},byp_3}; // @[TensorGemm.scala 209:24]
  wire [68:0] _add_0_3_T_2 = $signed(_GEN_6) + $signed(dot_0_3_io_y); // @[TensorGemm.scala 209:24]
  wire [31:0] add_0_3 = _add_0_3_T_2[31:0]; // @[TensorGemm.scala 199:52 TensorGemm.scala 209:17]
  wire [31:0] byp_4 = io_bypass_cond ? $signed(last_acc_write_0_4) : $signed(io_acc_i_data_bits_0_4); // @[TensorGemm.scala 208:20]
  wire [68:0] _GEN_8 = {{37{byp_4[31]}},byp_4}; // @[TensorGemm.scala 209:24]
  wire [68:0] _add_0_4_T_2 = $signed(_GEN_8) + $signed(dot_0_4_io_y); // @[TensorGemm.scala 209:24]
  wire [31:0] add_0_4 = _add_0_4_T_2[31:0]; // @[TensorGemm.scala 199:52 TensorGemm.scala 209:17]
  wire [31:0] byp_5 = io_bypass_cond ? $signed(last_acc_write_0_5) : $signed(io_acc_i_data_bits_0_5); // @[TensorGemm.scala 208:20]
  wire [68:0] _GEN_10 = {{37{byp_5[31]}},byp_5}; // @[TensorGemm.scala 209:24]
  wire [68:0] _add_0_5_T_2 = $signed(_GEN_10) + $signed(dot_0_5_io_y); // @[TensorGemm.scala 209:24]
  wire [31:0] add_0_5 = _add_0_5_T_2[31:0]; // @[TensorGemm.scala 199:52 TensorGemm.scala 209:17]
  wire [31:0] byp_6 = io_bypass_cond ? $signed(last_acc_write_0_6) : $signed(io_acc_i_data_bits_0_6); // @[TensorGemm.scala 208:20]
  wire [68:0] _GEN_12 = {{37{byp_6[31]}},byp_6}; // @[TensorGemm.scala 209:24]
  wire [68:0] _add_0_6_T_2 = $signed(_GEN_12) + $signed(dot_0_6_io_y); // @[TensorGemm.scala 209:24]
  wire [31:0] add_0_6 = _add_0_6_T_2[31:0]; // @[TensorGemm.scala 199:52 TensorGemm.scala 209:17]
  wire [31:0] byp_7 = io_bypass_cond ? $signed(last_acc_write_0_7) : $signed(io_acc_i_data_bits_0_7); // @[TensorGemm.scala 208:20]
  wire [68:0] _GEN_14 = {{37{byp_7[31]}},byp_7}; // @[TensorGemm.scala 209:24]
  wire [68:0] _add_0_7_T_2 = $signed(_GEN_14) + $signed(dot_0_7_io_y); // @[TensorGemm.scala 209:24]
  wire [31:0] add_0_7 = _add_0_7_T_2[31:0]; // @[TensorGemm.scala 199:52 TensorGemm.scala 209:17]
  wire [31:0] byp_8 = io_bypass_cond ? $signed(last_acc_write_0_8) : $signed(io_acc_i_data_bits_0_8); // @[TensorGemm.scala 208:20]
  wire [68:0] _GEN_16 = {{37{byp_8[31]}},byp_8}; // @[TensorGemm.scala 209:24]
  wire [68:0] _add_0_8_T_2 = $signed(_GEN_16) + $signed(dot_0_8_io_y); // @[TensorGemm.scala 209:24]
  wire [31:0] add_0_8 = _add_0_8_T_2[31:0]; // @[TensorGemm.scala 199:52 TensorGemm.scala 209:17]
  wire [31:0] byp_9 = io_bypass_cond ? $signed(last_acc_write_0_9) : $signed(io_acc_i_data_bits_0_9); // @[TensorGemm.scala 208:20]
  wire [68:0] _GEN_18 = {{37{byp_9[31]}},byp_9}; // @[TensorGemm.scala 209:24]
  wire [68:0] _add_0_9_T_2 = $signed(_GEN_18) + $signed(dot_0_9_io_y); // @[TensorGemm.scala 209:24]
  wire [31:0] add_0_9 = _add_0_9_T_2[31:0]; // @[TensorGemm.scala 199:52 TensorGemm.scala 209:17]
  wire [31:0] byp_10 = io_bypass_cond ? $signed(last_acc_write_0_10) : $signed(io_acc_i_data_bits_0_10); // @[TensorGemm.scala 208:20]
  wire [68:0] _GEN_20 = {{37{byp_10[31]}},byp_10}; // @[TensorGemm.scala 209:24]
  wire [68:0] _add_0_10_T_2 = $signed(_GEN_20) + $signed(dot_0_10_io_y); // @[TensorGemm.scala 209:24]
  wire [31:0] add_0_10 = _add_0_10_T_2[31:0]; // @[TensorGemm.scala 199:52 TensorGemm.scala 209:17]
  wire [31:0] byp_11 = io_bypass_cond ? $signed(last_acc_write_0_11) : $signed(io_acc_i_data_bits_0_11); // @[TensorGemm.scala 208:20]
  wire [68:0] _GEN_22 = {{37{byp_11[31]}},byp_11}; // @[TensorGemm.scala 209:24]
  wire [68:0] _add_0_11_T_2 = $signed(_GEN_22) + $signed(dot_0_11_io_y); // @[TensorGemm.scala 209:24]
  wire [31:0] add_0_11 = _add_0_11_T_2[31:0]; // @[TensorGemm.scala 199:52 TensorGemm.scala 209:17]
  wire [31:0] byp_12 = io_bypass_cond ? $signed(last_acc_write_0_12) : $signed(io_acc_i_data_bits_0_12); // @[TensorGemm.scala 208:20]
  wire [68:0] _GEN_24 = {{37{byp_12[31]}},byp_12}; // @[TensorGemm.scala 209:24]
  wire [68:0] _add_0_12_T_2 = $signed(_GEN_24) + $signed(dot_0_12_io_y); // @[TensorGemm.scala 209:24]
  wire [31:0] add_0_12 = _add_0_12_T_2[31:0]; // @[TensorGemm.scala 199:52 TensorGemm.scala 209:17]
  wire [31:0] byp_13 = io_bypass_cond ? $signed(last_acc_write_0_13) : $signed(io_acc_i_data_bits_0_13); // @[TensorGemm.scala 208:20]
  wire [68:0] _GEN_26 = {{37{byp_13[31]}},byp_13}; // @[TensorGemm.scala 209:24]
  wire [68:0] _add_0_13_T_2 = $signed(_GEN_26) + $signed(dot_0_13_io_y); // @[TensorGemm.scala 209:24]
  wire [31:0] add_0_13 = _add_0_13_T_2[31:0]; // @[TensorGemm.scala 199:52 TensorGemm.scala 209:17]
  wire [31:0] byp_14 = io_bypass_cond ? $signed(last_acc_write_0_14) : $signed(io_acc_i_data_bits_0_14); // @[TensorGemm.scala 208:20]
  wire [68:0] _GEN_28 = {{37{byp_14[31]}},byp_14}; // @[TensorGemm.scala 209:24]
  wire [68:0] _add_0_14_T_2 = $signed(_GEN_28) + $signed(dot_0_14_io_y); // @[TensorGemm.scala 209:24]
  wire [31:0] add_0_14 = _add_0_14_T_2[31:0]; // @[TensorGemm.scala 199:52 TensorGemm.scala 209:17]
  wire [31:0] byp_15 = io_bypass_cond ? $signed(last_acc_write_0_15) : $signed(io_acc_i_data_bits_0_15); // @[TensorGemm.scala 208:20]
  wire [68:0] _GEN_30 = {{37{byp_15[31]}},byp_15}; // @[TensorGemm.scala 209:24]
  wire [68:0] _add_0_15_T_2 = $signed(_GEN_30) + $signed(dot_0_15_io_y); // @[TensorGemm.scala 209:24]
  wire [31:0] add_0_15 = _add_0_15_T_2[31:0]; // @[TensorGemm.scala 199:52 TensorGemm.scala 209:17]
  DotProduct dot_0_0 ( // @[TensorGemm.scala 198:11]
    .clock(dot_0_0_clock),
    .io_a_0(dot_0_0_io_a_0),
    .io_a_1(dot_0_0_io_a_1),
    .io_a_2(dot_0_0_io_a_2),
    .io_a_3(dot_0_0_io_a_3),
    .io_a_4(dot_0_0_io_a_4),
    .io_a_5(dot_0_0_io_a_5),
    .io_a_6(dot_0_0_io_a_6),
    .io_a_7(dot_0_0_io_a_7),
    .io_a_8(dot_0_0_io_a_8),
    .io_a_9(dot_0_0_io_a_9),
    .io_a_10(dot_0_0_io_a_10),
    .io_a_11(dot_0_0_io_a_11),
    .io_a_12(dot_0_0_io_a_12),
    .io_a_13(dot_0_0_io_a_13),
    .io_a_14(dot_0_0_io_a_14),
    .io_a_15(dot_0_0_io_a_15),
    .io_b_0(dot_0_0_io_b_0),
    .io_b_1(dot_0_0_io_b_1),
    .io_b_2(dot_0_0_io_b_2),
    .io_b_3(dot_0_0_io_b_3),
    .io_b_4(dot_0_0_io_b_4),
    .io_b_5(dot_0_0_io_b_5),
    .io_b_6(dot_0_0_io_b_6),
    .io_b_7(dot_0_0_io_b_7),
    .io_b_8(dot_0_0_io_b_8),
    .io_b_9(dot_0_0_io_b_9),
    .io_b_10(dot_0_0_io_b_10),
    .io_b_11(dot_0_0_io_b_11),
    .io_b_12(dot_0_0_io_b_12),
    .io_b_13(dot_0_0_io_b_13),
    .io_b_14(dot_0_0_io_b_14),
    .io_b_15(dot_0_0_io_b_15),
    .io_y(dot_0_0_io_y)
  );
  DotProduct dot_0_1 ( // @[TensorGemm.scala 198:11]
    .clock(dot_0_1_clock),
    .io_a_0(dot_0_1_io_a_0),
    .io_a_1(dot_0_1_io_a_1),
    .io_a_2(dot_0_1_io_a_2),
    .io_a_3(dot_0_1_io_a_3),
    .io_a_4(dot_0_1_io_a_4),
    .io_a_5(dot_0_1_io_a_5),
    .io_a_6(dot_0_1_io_a_6),
    .io_a_7(dot_0_1_io_a_7),
    .io_a_8(dot_0_1_io_a_8),
    .io_a_9(dot_0_1_io_a_9),
    .io_a_10(dot_0_1_io_a_10),
    .io_a_11(dot_0_1_io_a_11),
    .io_a_12(dot_0_1_io_a_12),
    .io_a_13(dot_0_1_io_a_13),
    .io_a_14(dot_0_1_io_a_14),
    .io_a_15(dot_0_1_io_a_15),
    .io_b_0(dot_0_1_io_b_0),
    .io_b_1(dot_0_1_io_b_1),
    .io_b_2(dot_0_1_io_b_2),
    .io_b_3(dot_0_1_io_b_3),
    .io_b_4(dot_0_1_io_b_4),
    .io_b_5(dot_0_1_io_b_5),
    .io_b_6(dot_0_1_io_b_6),
    .io_b_7(dot_0_1_io_b_7),
    .io_b_8(dot_0_1_io_b_8),
    .io_b_9(dot_0_1_io_b_9),
    .io_b_10(dot_0_1_io_b_10),
    .io_b_11(dot_0_1_io_b_11),
    .io_b_12(dot_0_1_io_b_12),
    .io_b_13(dot_0_1_io_b_13),
    .io_b_14(dot_0_1_io_b_14),
    .io_b_15(dot_0_1_io_b_15),
    .io_y(dot_0_1_io_y)
  );
  DotProduct dot_0_2 ( // @[TensorGemm.scala 198:11]
    .clock(dot_0_2_clock),
    .io_a_0(dot_0_2_io_a_0),
    .io_a_1(dot_0_2_io_a_1),
    .io_a_2(dot_0_2_io_a_2),
    .io_a_3(dot_0_2_io_a_3),
    .io_a_4(dot_0_2_io_a_4),
    .io_a_5(dot_0_2_io_a_5),
    .io_a_6(dot_0_2_io_a_6),
    .io_a_7(dot_0_2_io_a_7),
    .io_a_8(dot_0_2_io_a_8),
    .io_a_9(dot_0_2_io_a_9),
    .io_a_10(dot_0_2_io_a_10),
    .io_a_11(dot_0_2_io_a_11),
    .io_a_12(dot_0_2_io_a_12),
    .io_a_13(dot_0_2_io_a_13),
    .io_a_14(dot_0_2_io_a_14),
    .io_a_15(dot_0_2_io_a_15),
    .io_b_0(dot_0_2_io_b_0),
    .io_b_1(dot_0_2_io_b_1),
    .io_b_2(dot_0_2_io_b_2),
    .io_b_3(dot_0_2_io_b_3),
    .io_b_4(dot_0_2_io_b_4),
    .io_b_5(dot_0_2_io_b_5),
    .io_b_6(dot_0_2_io_b_6),
    .io_b_7(dot_0_2_io_b_7),
    .io_b_8(dot_0_2_io_b_8),
    .io_b_9(dot_0_2_io_b_9),
    .io_b_10(dot_0_2_io_b_10),
    .io_b_11(dot_0_2_io_b_11),
    .io_b_12(dot_0_2_io_b_12),
    .io_b_13(dot_0_2_io_b_13),
    .io_b_14(dot_0_2_io_b_14),
    .io_b_15(dot_0_2_io_b_15),
    .io_y(dot_0_2_io_y)
  );
  DotProduct dot_0_3 ( // @[TensorGemm.scala 198:11]
    .clock(dot_0_3_clock),
    .io_a_0(dot_0_3_io_a_0),
    .io_a_1(dot_0_3_io_a_1),
    .io_a_2(dot_0_3_io_a_2),
    .io_a_3(dot_0_3_io_a_3),
    .io_a_4(dot_0_3_io_a_4),
    .io_a_5(dot_0_3_io_a_5),
    .io_a_6(dot_0_3_io_a_6),
    .io_a_7(dot_0_3_io_a_7),
    .io_a_8(dot_0_3_io_a_8),
    .io_a_9(dot_0_3_io_a_9),
    .io_a_10(dot_0_3_io_a_10),
    .io_a_11(dot_0_3_io_a_11),
    .io_a_12(dot_0_3_io_a_12),
    .io_a_13(dot_0_3_io_a_13),
    .io_a_14(dot_0_3_io_a_14),
    .io_a_15(dot_0_3_io_a_15),
    .io_b_0(dot_0_3_io_b_0),
    .io_b_1(dot_0_3_io_b_1),
    .io_b_2(dot_0_3_io_b_2),
    .io_b_3(dot_0_3_io_b_3),
    .io_b_4(dot_0_3_io_b_4),
    .io_b_5(dot_0_3_io_b_5),
    .io_b_6(dot_0_3_io_b_6),
    .io_b_7(dot_0_3_io_b_7),
    .io_b_8(dot_0_3_io_b_8),
    .io_b_9(dot_0_3_io_b_9),
    .io_b_10(dot_0_3_io_b_10),
    .io_b_11(dot_0_3_io_b_11),
    .io_b_12(dot_0_3_io_b_12),
    .io_b_13(dot_0_3_io_b_13),
    .io_b_14(dot_0_3_io_b_14),
    .io_b_15(dot_0_3_io_b_15),
    .io_y(dot_0_3_io_y)
  );
  DotProduct dot_0_4 ( // @[TensorGemm.scala 198:11]
    .clock(dot_0_4_clock),
    .io_a_0(dot_0_4_io_a_0),
    .io_a_1(dot_0_4_io_a_1),
    .io_a_2(dot_0_4_io_a_2),
    .io_a_3(dot_0_4_io_a_3),
    .io_a_4(dot_0_4_io_a_4),
    .io_a_5(dot_0_4_io_a_5),
    .io_a_6(dot_0_4_io_a_6),
    .io_a_7(dot_0_4_io_a_7),
    .io_a_8(dot_0_4_io_a_8),
    .io_a_9(dot_0_4_io_a_9),
    .io_a_10(dot_0_4_io_a_10),
    .io_a_11(dot_0_4_io_a_11),
    .io_a_12(dot_0_4_io_a_12),
    .io_a_13(dot_0_4_io_a_13),
    .io_a_14(dot_0_4_io_a_14),
    .io_a_15(dot_0_4_io_a_15),
    .io_b_0(dot_0_4_io_b_0),
    .io_b_1(dot_0_4_io_b_1),
    .io_b_2(dot_0_4_io_b_2),
    .io_b_3(dot_0_4_io_b_3),
    .io_b_4(dot_0_4_io_b_4),
    .io_b_5(dot_0_4_io_b_5),
    .io_b_6(dot_0_4_io_b_6),
    .io_b_7(dot_0_4_io_b_7),
    .io_b_8(dot_0_4_io_b_8),
    .io_b_9(dot_0_4_io_b_9),
    .io_b_10(dot_0_4_io_b_10),
    .io_b_11(dot_0_4_io_b_11),
    .io_b_12(dot_0_4_io_b_12),
    .io_b_13(dot_0_4_io_b_13),
    .io_b_14(dot_0_4_io_b_14),
    .io_b_15(dot_0_4_io_b_15),
    .io_y(dot_0_4_io_y)
  );
  DotProduct dot_0_5 ( // @[TensorGemm.scala 198:11]
    .clock(dot_0_5_clock),
    .io_a_0(dot_0_5_io_a_0),
    .io_a_1(dot_0_5_io_a_1),
    .io_a_2(dot_0_5_io_a_2),
    .io_a_3(dot_0_5_io_a_3),
    .io_a_4(dot_0_5_io_a_4),
    .io_a_5(dot_0_5_io_a_5),
    .io_a_6(dot_0_5_io_a_6),
    .io_a_7(dot_0_5_io_a_7),
    .io_a_8(dot_0_5_io_a_8),
    .io_a_9(dot_0_5_io_a_9),
    .io_a_10(dot_0_5_io_a_10),
    .io_a_11(dot_0_5_io_a_11),
    .io_a_12(dot_0_5_io_a_12),
    .io_a_13(dot_0_5_io_a_13),
    .io_a_14(dot_0_5_io_a_14),
    .io_a_15(dot_0_5_io_a_15),
    .io_b_0(dot_0_5_io_b_0),
    .io_b_1(dot_0_5_io_b_1),
    .io_b_2(dot_0_5_io_b_2),
    .io_b_3(dot_0_5_io_b_3),
    .io_b_4(dot_0_5_io_b_4),
    .io_b_5(dot_0_5_io_b_5),
    .io_b_6(dot_0_5_io_b_6),
    .io_b_7(dot_0_5_io_b_7),
    .io_b_8(dot_0_5_io_b_8),
    .io_b_9(dot_0_5_io_b_9),
    .io_b_10(dot_0_5_io_b_10),
    .io_b_11(dot_0_5_io_b_11),
    .io_b_12(dot_0_5_io_b_12),
    .io_b_13(dot_0_5_io_b_13),
    .io_b_14(dot_0_5_io_b_14),
    .io_b_15(dot_0_5_io_b_15),
    .io_y(dot_0_5_io_y)
  );
  DotProduct dot_0_6 ( // @[TensorGemm.scala 198:11]
    .clock(dot_0_6_clock),
    .io_a_0(dot_0_6_io_a_0),
    .io_a_1(dot_0_6_io_a_1),
    .io_a_2(dot_0_6_io_a_2),
    .io_a_3(dot_0_6_io_a_3),
    .io_a_4(dot_0_6_io_a_4),
    .io_a_5(dot_0_6_io_a_5),
    .io_a_6(dot_0_6_io_a_6),
    .io_a_7(dot_0_6_io_a_7),
    .io_a_8(dot_0_6_io_a_8),
    .io_a_9(dot_0_6_io_a_9),
    .io_a_10(dot_0_6_io_a_10),
    .io_a_11(dot_0_6_io_a_11),
    .io_a_12(dot_0_6_io_a_12),
    .io_a_13(dot_0_6_io_a_13),
    .io_a_14(dot_0_6_io_a_14),
    .io_a_15(dot_0_6_io_a_15),
    .io_b_0(dot_0_6_io_b_0),
    .io_b_1(dot_0_6_io_b_1),
    .io_b_2(dot_0_6_io_b_2),
    .io_b_3(dot_0_6_io_b_3),
    .io_b_4(dot_0_6_io_b_4),
    .io_b_5(dot_0_6_io_b_5),
    .io_b_6(dot_0_6_io_b_6),
    .io_b_7(dot_0_6_io_b_7),
    .io_b_8(dot_0_6_io_b_8),
    .io_b_9(dot_0_6_io_b_9),
    .io_b_10(dot_0_6_io_b_10),
    .io_b_11(dot_0_6_io_b_11),
    .io_b_12(dot_0_6_io_b_12),
    .io_b_13(dot_0_6_io_b_13),
    .io_b_14(dot_0_6_io_b_14),
    .io_b_15(dot_0_6_io_b_15),
    .io_y(dot_0_6_io_y)
  );
  DotProduct dot_0_7 ( // @[TensorGemm.scala 198:11]
    .clock(dot_0_7_clock),
    .io_a_0(dot_0_7_io_a_0),
    .io_a_1(dot_0_7_io_a_1),
    .io_a_2(dot_0_7_io_a_2),
    .io_a_3(dot_0_7_io_a_3),
    .io_a_4(dot_0_7_io_a_4),
    .io_a_5(dot_0_7_io_a_5),
    .io_a_6(dot_0_7_io_a_6),
    .io_a_7(dot_0_7_io_a_7),
    .io_a_8(dot_0_7_io_a_8),
    .io_a_9(dot_0_7_io_a_9),
    .io_a_10(dot_0_7_io_a_10),
    .io_a_11(dot_0_7_io_a_11),
    .io_a_12(dot_0_7_io_a_12),
    .io_a_13(dot_0_7_io_a_13),
    .io_a_14(dot_0_7_io_a_14),
    .io_a_15(dot_0_7_io_a_15),
    .io_b_0(dot_0_7_io_b_0),
    .io_b_1(dot_0_7_io_b_1),
    .io_b_2(dot_0_7_io_b_2),
    .io_b_3(dot_0_7_io_b_3),
    .io_b_4(dot_0_7_io_b_4),
    .io_b_5(dot_0_7_io_b_5),
    .io_b_6(dot_0_7_io_b_6),
    .io_b_7(dot_0_7_io_b_7),
    .io_b_8(dot_0_7_io_b_8),
    .io_b_9(dot_0_7_io_b_9),
    .io_b_10(dot_0_7_io_b_10),
    .io_b_11(dot_0_7_io_b_11),
    .io_b_12(dot_0_7_io_b_12),
    .io_b_13(dot_0_7_io_b_13),
    .io_b_14(dot_0_7_io_b_14),
    .io_b_15(dot_0_7_io_b_15),
    .io_y(dot_0_7_io_y)
  );
  DotProduct dot_0_8 ( // @[TensorGemm.scala 198:11]
    .clock(dot_0_8_clock),
    .io_a_0(dot_0_8_io_a_0),
    .io_a_1(dot_0_8_io_a_1),
    .io_a_2(dot_0_8_io_a_2),
    .io_a_3(dot_0_8_io_a_3),
    .io_a_4(dot_0_8_io_a_4),
    .io_a_5(dot_0_8_io_a_5),
    .io_a_6(dot_0_8_io_a_6),
    .io_a_7(dot_0_8_io_a_7),
    .io_a_8(dot_0_8_io_a_8),
    .io_a_9(dot_0_8_io_a_9),
    .io_a_10(dot_0_8_io_a_10),
    .io_a_11(dot_0_8_io_a_11),
    .io_a_12(dot_0_8_io_a_12),
    .io_a_13(dot_0_8_io_a_13),
    .io_a_14(dot_0_8_io_a_14),
    .io_a_15(dot_0_8_io_a_15),
    .io_b_0(dot_0_8_io_b_0),
    .io_b_1(dot_0_8_io_b_1),
    .io_b_2(dot_0_8_io_b_2),
    .io_b_3(dot_0_8_io_b_3),
    .io_b_4(dot_0_8_io_b_4),
    .io_b_5(dot_0_8_io_b_5),
    .io_b_6(dot_0_8_io_b_6),
    .io_b_7(dot_0_8_io_b_7),
    .io_b_8(dot_0_8_io_b_8),
    .io_b_9(dot_0_8_io_b_9),
    .io_b_10(dot_0_8_io_b_10),
    .io_b_11(dot_0_8_io_b_11),
    .io_b_12(dot_0_8_io_b_12),
    .io_b_13(dot_0_8_io_b_13),
    .io_b_14(dot_0_8_io_b_14),
    .io_b_15(dot_0_8_io_b_15),
    .io_y(dot_0_8_io_y)
  );
  DotProduct dot_0_9 ( // @[TensorGemm.scala 198:11]
    .clock(dot_0_9_clock),
    .io_a_0(dot_0_9_io_a_0),
    .io_a_1(dot_0_9_io_a_1),
    .io_a_2(dot_0_9_io_a_2),
    .io_a_3(dot_0_9_io_a_3),
    .io_a_4(dot_0_9_io_a_4),
    .io_a_5(dot_0_9_io_a_5),
    .io_a_6(dot_0_9_io_a_6),
    .io_a_7(dot_0_9_io_a_7),
    .io_a_8(dot_0_9_io_a_8),
    .io_a_9(dot_0_9_io_a_9),
    .io_a_10(dot_0_9_io_a_10),
    .io_a_11(dot_0_9_io_a_11),
    .io_a_12(dot_0_9_io_a_12),
    .io_a_13(dot_0_9_io_a_13),
    .io_a_14(dot_0_9_io_a_14),
    .io_a_15(dot_0_9_io_a_15),
    .io_b_0(dot_0_9_io_b_0),
    .io_b_1(dot_0_9_io_b_1),
    .io_b_2(dot_0_9_io_b_2),
    .io_b_3(dot_0_9_io_b_3),
    .io_b_4(dot_0_9_io_b_4),
    .io_b_5(dot_0_9_io_b_5),
    .io_b_6(dot_0_9_io_b_6),
    .io_b_7(dot_0_9_io_b_7),
    .io_b_8(dot_0_9_io_b_8),
    .io_b_9(dot_0_9_io_b_9),
    .io_b_10(dot_0_9_io_b_10),
    .io_b_11(dot_0_9_io_b_11),
    .io_b_12(dot_0_9_io_b_12),
    .io_b_13(dot_0_9_io_b_13),
    .io_b_14(dot_0_9_io_b_14),
    .io_b_15(dot_0_9_io_b_15),
    .io_y(dot_0_9_io_y)
  );
  DotProduct dot_0_10 ( // @[TensorGemm.scala 198:11]
    .clock(dot_0_10_clock),
    .io_a_0(dot_0_10_io_a_0),
    .io_a_1(dot_0_10_io_a_1),
    .io_a_2(dot_0_10_io_a_2),
    .io_a_3(dot_0_10_io_a_3),
    .io_a_4(dot_0_10_io_a_4),
    .io_a_5(dot_0_10_io_a_5),
    .io_a_6(dot_0_10_io_a_6),
    .io_a_7(dot_0_10_io_a_7),
    .io_a_8(dot_0_10_io_a_8),
    .io_a_9(dot_0_10_io_a_9),
    .io_a_10(dot_0_10_io_a_10),
    .io_a_11(dot_0_10_io_a_11),
    .io_a_12(dot_0_10_io_a_12),
    .io_a_13(dot_0_10_io_a_13),
    .io_a_14(dot_0_10_io_a_14),
    .io_a_15(dot_0_10_io_a_15),
    .io_b_0(dot_0_10_io_b_0),
    .io_b_1(dot_0_10_io_b_1),
    .io_b_2(dot_0_10_io_b_2),
    .io_b_3(dot_0_10_io_b_3),
    .io_b_4(dot_0_10_io_b_4),
    .io_b_5(dot_0_10_io_b_5),
    .io_b_6(dot_0_10_io_b_6),
    .io_b_7(dot_0_10_io_b_7),
    .io_b_8(dot_0_10_io_b_8),
    .io_b_9(dot_0_10_io_b_9),
    .io_b_10(dot_0_10_io_b_10),
    .io_b_11(dot_0_10_io_b_11),
    .io_b_12(dot_0_10_io_b_12),
    .io_b_13(dot_0_10_io_b_13),
    .io_b_14(dot_0_10_io_b_14),
    .io_b_15(dot_0_10_io_b_15),
    .io_y(dot_0_10_io_y)
  );
  DotProduct dot_0_11 ( // @[TensorGemm.scala 198:11]
    .clock(dot_0_11_clock),
    .io_a_0(dot_0_11_io_a_0),
    .io_a_1(dot_0_11_io_a_1),
    .io_a_2(dot_0_11_io_a_2),
    .io_a_3(dot_0_11_io_a_3),
    .io_a_4(dot_0_11_io_a_4),
    .io_a_5(dot_0_11_io_a_5),
    .io_a_6(dot_0_11_io_a_6),
    .io_a_7(dot_0_11_io_a_7),
    .io_a_8(dot_0_11_io_a_8),
    .io_a_9(dot_0_11_io_a_9),
    .io_a_10(dot_0_11_io_a_10),
    .io_a_11(dot_0_11_io_a_11),
    .io_a_12(dot_0_11_io_a_12),
    .io_a_13(dot_0_11_io_a_13),
    .io_a_14(dot_0_11_io_a_14),
    .io_a_15(dot_0_11_io_a_15),
    .io_b_0(dot_0_11_io_b_0),
    .io_b_1(dot_0_11_io_b_1),
    .io_b_2(dot_0_11_io_b_2),
    .io_b_3(dot_0_11_io_b_3),
    .io_b_4(dot_0_11_io_b_4),
    .io_b_5(dot_0_11_io_b_5),
    .io_b_6(dot_0_11_io_b_6),
    .io_b_7(dot_0_11_io_b_7),
    .io_b_8(dot_0_11_io_b_8),
    .io_b_9(dot_0_11_io_b_9),
    .io_b_10(dot_0_11_io_b_10),
    .io_b_11(dot_0_11_io_b_11),
    .io_b_12(dot_0_11_io_b_12),
    .io_b_13(dot_0_11_io_b_13),
    .io_b_14(dot_0_11_io_b_14),
    .io_b_15(dot_0_11_io_b_15),
    .io_y(dot_0_11_io_y)
  );
  DotProduct dot_0_12 ( // @[TensorGemm.scala 198:11]
    .clock(dot_0_12_clock),
    .io_a_0(dot_0_12_io_a_0),
    .io_a_1(dot_0_12_io_a_1),
    .io_a_2(dot_0_12_io_a_2),
    .io_a_3(dot_0_12_io_a_3),
    .io_a_4(dot_0_12_io_a_4),
    .io_a_5(dot_0_12_io_a_5),
    .io_a_6(dot_0_12_io_a_6),
    .io_a_7(dot_0_12_io_a_7),
    .io_a_8(dot_0_12_io_a_8),
    .io_a_9(dot_0_12_io_a_9),
    .io_a_10(dot_0_12_io_a_10),
    .io_a_11(dot_0_12_io_a_11),
    .io_a_12(dot_0_12_io_a_12),
    .io_a_13(dot_0_12_io_a_13),
    .io_a_14(dot_0_12_io_a_14),
    .io_a_15(dot_0_12_io_a_15),
    .io_b_0(dot_0_12_io_b_0),
    .io_b_1(dot_0_12_io_b_1),
    .io_b_2(dot_0_12_io_b_2),
    .io_b_3(dot_0_12_io_b_3),
    .io_b_4(dot_0_12_io_b_4),
    .io_b_5(dot_0_12_io_b_5),
    .io_b_6(dot_0_12_io_b_6),
    .io_b_7(dot_0_12_io_b_7),
    .io_b_8(dot_0_12_io_b_8),
    .io_b_9(dot_0_12_io_b_9),
    .io_b_10(dot_0_12_io_b_10),
    .io_b_11(dot_0_12_io_b_11),
    .io_b_12(dot_0_12_io_b_12),
    .io_b_13(dot_0_12_io_b_13),
    .io_b_14(dot_0_12_io_b_14),
    .io_b_15(dot_0_12_io_b_15),
    .io_y(dot_0_12_io_y)
  );
  DotProduct dot_0_13 ( // @[TensorGemm.scala 198:11]
    .clock(dot_0_13_clock),
    .io_a_0(dot_0_13_io_a_0),
    .io_a_1(dot_0_13_io_a_1),
    .io_a_2(dot_0_13_io_a_2),
    .io_a_3(dot_0_13_io_a_3),
    .io_a_4(dot_0_13_io_a_4),
    .io_a_5(dot_0_13_io_a_5),
    .io_a_6(dot_0_13_io_a_6),
    .io_a_7(dot_0_13_io_a_7),
    .io_a_8(dot_0_13_io_a_8),
    .io_a_9(dot_0_13_io_a_9),
    .io_a_10(dot_0_13_io_a_10),
    .io_a_11(dot_0_13_io_a_11),
    .io_a_12(dot_0_13_io_a_12),
    .io_a_13(dot_0_13_io_a_13),
    .io_a_14(dot_0_13_io_a_14),
    .io_a_15(dot_0_13_io_a_15),
    .io_b_0(dot_0_13_io_b_0),
    .io_b_1(dot_0_13_io_b_1),
    .io_b_2(dot_0_13_io_b_2),
    .io_b_3(dot_0_13_io_b_3),
    .io_b_4(dot_0_13_io_b_4),
    .io_b_5(dot_0_13_io_b_5),
    .io_b_6(dot_0_13_io_b_6),
    .io_b_7(dot_0_13_io_b_7),
    .io_b_8(dot_0_13_io_b_8),
    .io_b_9(dot_0_13_io_b_9),
    .io_b_10(dot_0_13_io_b_10),
    .io_b_11(dot_0_13_io_b_11),
    .io_b_12(dot_0_13_io_b_12),
    .io_b_13(dot_0_13_io_b_13),
    .io_b_14(dot_0_13_io_b_14),
    .io_b_15(dot_0_13_io_b_15),
    .io_y(dot_0_13_io_y)
  );
  DotProduct dot_0_14 ( // @[TensorGemm.scala 198:11]
    .clock(dot_0_14_clock),
    .io_a_0(dot_0_14_io_a_0),
    .io_a_1(dot_0_14_io_a_1),
    .io_a_2(dot_0_14_io_a_2),
    .io_a_3(dot_0_14_io_a_3),
    .io_a_4(dot_0_14_io_a_4),
    .io_a_5(dot_0_14_io_a_5),
    .io_a_6(dot_0_14_io_a_6),
    .io_a_7(dot_0_14_io_a_7),
    .io_a_8(dot_0_14_io_a_8),
    .io_a_9(dot_0_14_io_a_9),
    .io_a_10(dot_0_14_io_a_10),
    .io_a_11(dot_0_14_io_a_11),
    .io_a_12(dot_0_14_io_a_12),
    .io_a_13(dot_0_14_io_a_13),
    .io_a_14(dot_0_14_io_a_14),
    .io_a_15(dot_0_14_io_a_15),
    .io_b_0(dot_0_14_io_b_0),
    .io_b_1(dot_0_14_io_b_1),
    .io_b_2(dot_0_14_io_b_2),
    .io_b_3(dot_0_14_io_b_3),
    .io_b_4(dot_0_14_io_b_4),
    .io_b_5(dot_0_14_io_b_5),
    .io_b_6(dot_0_14_io_b_6),
    .io_b_7(dot_0_14_io_b_7),
    .io_b_8(dot_0_14_io_b_8),
    .io_b_9(dot_0_14_io_b_9),
    .io_b_10(dot_0_14_io_b_10),
    .io_b_11(dot_0_14_io_b_11),
    .io_b_12(dot_0_14_io_b_12),
    .io_b_13(dot_0_14_io_b_13),
    .io_b_14(dot_0_14_io_b_14),
    .io_b_15(dot_0_14_io_b_15),
    .io_y(dot_0_14_io_y)
  );
  DotProduct dot_0_15 ( // @[TensorGemm.scala 198:11]
    .clock(dot_0_15_clock),
    .io_a_0(dot_0_15_io_a_0),
    .io_a_1(dot_0_15_io_a_1),
    .io_a_2(dot_0_15_io_a_2),
    .io_a_3(dot_0_15_io_a_3),
    .io_a_4(dot_0_15_io_a_4),
    .io_a_5(dot_0_15_io_a_5),
    .io_a_6(dot_0_15_io_a_6),
    .io_a_7(dot_0_15_io_a_7),
    .io_a_8(dot_0_15_io_a_8),
    .io_a_9(dot_0_15_io_a_9),
    .io_a_10(dot_0_15_io_a_10),
    .io_a_11(dot_0_15_io_a_11),
    .io_a_12(dot_0_15_io_a_12),
    .io_a_13(dot_0_15_io_a_13),
    .io_a_14(dot_0_15_io_a_14),
    .io_a_15(dot_0_15_io_a_15),
    .io_b_0(dot_0_15_io_b_0),
    .io_b_1(dot_0_15_io_b_1),
    .io_b_2(dot_0_15_io_b_2),
    .io_b_3(dot_0_15_io_b_3),
    .io_b_4(dot_0_15_io_b_4),
    .io_b_5(dot_0_15_io_b_5),
    .io_b_6(dot_0_15_io_b_6),
    .io_b_7(dot_0_15_io_b_7),
    .io_b_8(dot_0_15_io_b_8),
    .io_b_9(dot_0_15_io_b_9),
    .io_b_10(dot_0_15_io_b_10),
    .io_b_11(dot_0_15_io_b_11),
    .io_b_12(dot_0_15_io_b_12),
    .io_b_13(dot_0_15_io_b_13),
    .io_b_14(dot_0_15_io_b_14),
    .io_b_15(dot_0_15_io_b_15),
    .io_y(dot_0_15_io_y)
  );
  assign io_acc_o_data_valid = io_acc_i_data_valid | io_valid_reset; // @[TensorGemm.scala 216:46]
  assign io_acc_o_data_bits_0_0 = io_valid_reset ? $signed(32'sh0) : $signed(add_0_0); // @[TensorGemm.scala 211:39]
  assign io_acc_o_data_bits_0_1 = io_valid_reset ? $signed(32'sh0) : $signed(add_0_1); // @[TensorGemm.scala 211:39]
  assign io_acc_o_data_bits_0_2 = io_valid_reset ? $signed(32'sh0) : $signed(add_0_2); // @[TensorGemm.scala 211:39]
  assign io_acc_o_data_bits_0_3 = io_valid_reset ? $signed(32'sh0) : $signed(add_0_3); // @[TensorGemm.scala 211:39]
  assign io_acc_o_data_bits_0_4 = io_valid_reset ? $signed(32'sh0) : $signed(add_0_4); // @[TensorGemm.scala 211:39]
  assign io_acc_o_data_bits_0_5 = io_valid_reset ? $signed(32'sh0) : $signed(add_0_5); // @[TensorGemm.scala 211:39]
  assign io_acc_o_data_bits_0_6 = io_valid_reset ? $signed(32'sh0) : $signed(add_0_6); // @[TensorGemm.scala 211:39]
  assign io_acc_o_data_bits_0_7 = io_valid_reset ? $signed(32'sh0) : $signed(add_0_7); // @[TensorGemm.scala 211:39]
  assign io_acc_o_data_bits_0_8 = io_valid_reset ? $signed(32'sh0) : $signed(add_0_8); // @[TensorGemm.scala 211:39]
  assign io_acc_o_data_bits_0_9 = io_valid_reset ? $signed(32'sh0) : $signed(add_0_9); // @[TensorGemm.scala 211:39]
  assign io_acc_o_data_bits_0_10 = io_valid_reset ? $signed(32'sh0) : $signed(add_0_10); // @[TensorGemm.scala 211:39]
  assign io_acc_o_data_bits_0_11 = io_valid_reset ? $signed(32'sh0) : $signed(add_0_11); // @[TensorGemm.scala 211:39]
  assign io_acc_o_data_bits_0_12 = io_valid_reset ? $signed(32'sh0) : $signed(add_0_12); // @[TensorGemm.scala 211:39]
  assign io_acc_o_data_bits_0_13 = io_valid_reset ? $signed(32'sh0) : $signed(add_0_13); // @[TensorGemm.scala 211:39]
  assign io_acc_o_data_bits_0_14 = io_valid_reset ? $signed(32'sh0) : $signed(add_0_14); // @[TensorGemm.scala 211:39]
  assign io_acc_o_data_bits_0_15 = io_valid_reset ? $signed(32'sh0) : $signed(add_0_15); // @[TensorGemm.scala 211:39]
  assign io_out_data_valid = io_acc_i_data_valid & ~io_valid_reset; // @[TensorGemm.scala 217:44]
  assign io_out_data_bits_0_0 = _add_0_0_T_2[31:0]; // @[TensorGemm.scala 213:43]
  assign io_out_data_bits_0_1 = _add_0_1_T_2[31:0]; // @[TensorGemm.scala 213:43]
  assign io_out_data_bits_0_2 = _add_0_2_T_2[31:0]; // @[TensorGemm.scala 213:43]
  assign io_out_data_bits_0_3 = _add_0_3_T_2[31:0]; // @[TensorGemm.scala 213:43]
  assign io_out_data_bits_0_4 = _add_0_4_T_2[31:0]; // @[TensorGemm.scala 213:43]
  assign io_out_data_bits_0_5 = _add_0_5_T_2[31:0]; // @[TensorGemm.scala 213:43]
  assign io_out_data_bits_0_6 = _add_0_6_T_2[31:0]; // @[TensorGemm.scala 213:43]
  assign io_out_data_bits_0_7 = _add_0_7_T_2[31:0]; // @[TensorGemm.scala 213:43]
  assign io_out_data_bits_0_8 = _add_0_8_T_2[31:0]; // @[TensorGemm.scala 213:43]
  assign io_out_data_bits_0_9 = _add_0_9_T_2[31:0]; // @[TensorGemm.scala 213:43]
  assign io_out_data_bits_0_10 = _add_0_10_T_2[31:0]; // @[TensorGemm.scala 213:43]
  assign io_out_data_bits_0_11 = _add_0_11_T_2[31:0]; // @[TensorGemm.scala 213:43]
  assign io_out_data_bits_0_12 = _add_0_12_T_2[31:0]; // @[TensorGemm.scala 213:43]
  assign io_out_data_bits_0_13 = _add_0_13_T_2[31:0]; // @[TensorGemm.scala 213:43]
  assign io_out_data_bits_0_14 = _add_0_14_T_2[31:0]; // @[TensorGemm.scala 213:43]
  assign io_out_data_bits_0_15 = _add_0_15_T_2[31:0]; // @[TensorGemm.scala 213:43]
  assign dot_0_0_clock = clock;
  assign dot_0_0_io_a_0 = io_inp_data_bits_0_0; // @[TensorGemm.scala 205:53]
  assign dot_0_0_io_a_1 = io_inp_data_bits_0_1; // @[TensorGemm.scala 205:53]
  assign dot_0_0_io_a_2 = io_inp_data_bits_0_2; // @[TensorGemm.scala 205:53]
  assign dot_0_0_io_a_3 = io_inp_data_bits_0_3; // @[TensorGemm.scala 205:53]
  assign dot_0_0_io_a_4 = io_inp_data_bits_0_4; // @[TensorGemm.scala 205:53]
  assign dot_0_0_io_a_5 = io_inp_data_bits_0_5; // @[TensorGemm.scala 205:53]
  assign dot_0_0_io_a_6 = io_inp_data_bits_0_6; // @[TensorGemm.scala 205:53]
  assign dot_0_0_io_a_7 = io_inp_data_bits_0_7; // @[TensorGemm.scala 205:53]
  assign dot_0_0_io_a_8 = io_inp_data_bits_0_8; // @[TensorGemm.scala 205:53]
  assign dot_0_0_io_a_9 = io_inp_data_bits_0_9; // @[TensorGemm.scala 205:53]
  assign dot_0_0_io_a_10 = io_inp_data_bits_0_10; // @[TensorGemm.scala 205:53]
  assign dot_0_0_io_a_11 = io_inp_data_bits_0_11; // @[TensorGemm.scala 205:53]
  assign dot_0_0_io_a_12 = io_inp_data_bits_0_12; // @[TensorGemm.scala 205:53]
  assign dot_0_0_io_a_13 = io_inp_data_bits_0_13; // @[TensorGemm.scala 205:53]
  assign dot_0_0_io_a_14 = io_inp_data_bits_0_14; // @[TensorGemm.scala 205:53]
  assign dot_0_0_io_a_15 = io_inp_data_bits_0_15; // @[TensorGemm.scala 205:53]
  assign dot_0_0_io_b_0 = io_wgt_data_bits_0_0; // @[TensorGemm.scala 206:53]
  assign dot_0_0_io_b_1 = io_wgt_data_bits_0_1; // @[TensorGemm.scala 206:53]
  assign dot_0_0_io_b_2 = io_wgt_data_bits_0_2; // @[TensorGemm.scala 206:53]
  assign dot_0_0_io_b_3 = io_wgt_data_bits_0_3; // @[TensorGemm.scala 206:53]
  assign dot_0_0_io_b_4 = io_wgt_data_bits_0_4; // @[TensorGemm.scala 206:53]
  assign dot_0_0_io_b_5 = io_wgt_data_bits_0_5; // @[TensorGemm.scala 206:53]
  assign dot_0_0_io_b_6 = io_wgt_data_bits_0_6; // @[TensorGemm.scala 206:53]
  assign dot_0_0_io_b_7 = io_wgt_data_bits_0_7; // @[TensorGemm.scala 206:53]
  assign dot_0_0_io_b_8 = io_wgt_data_bits_0_8; // @[TensorGemm.scala 206:53]
  assign dot_0_0_io_b_9 = io_wgt_data_bits_0_9; // @[TensorGemm.scala 206:53]
  assign dot_0_0_io_b_10 = io_wgt_data_bits_0_10; // @[TensorGemm.scala 206:53]
  assign dot_0_0_io_b_11 = io_wgt_data_bits_0_11; // @[TensorGemm.scala 206:53]
  assign dot_0_0_io_b_12 = io_wgt_data_bits_0_12; // @[TensorGemm.scala 206:53]
  assign dot_0_0_io_b_13 = io_wgt_data_bits_0_13; // @[TensorGemm.scala 206:53]
  assign dot_0_0_io_b_14 = io_wgt_data_bits_0_14; // @[TensorGemm.scala 206:53]
  assign dot_0_0_io_b_15 = io_wgt_data_bits_0_15; // @[TensorGemm.scala 206:53]
  assign dot_0_1_clock = clock;
  assign dot_0_1_io_a_0 = io_inp_data_bits_0_0; // @[TensorGemm.scala 205:53]
  assign dot_0_1_io_a_1 = io_inp_data_bits_0_1; // @[TensorGemm.scala 205:53]
  assign dot_0_1_io_a_2 = io_inp_data_bits_0_2; // @[TensorGemm.scala 205:53]
  assign dot_0_1_io_a_3 = io_inp_data_bits_0_3; // @[TensorGemm.scala 205:53]
  assign dot_0_1_io_a_4 = io_inp_data_bits_0_4; // @[TensorGemm.scala 205:53]
  assign dot_0_1_io_a_5 = io_inp_data_bits_0_5; // @[TensorGemm.scala 205:53]
  assign dot_0_1_io_a_6 = io_inp_data_bits_0_6; // @[TensorGemm.scala 205:53]
  assign dot_0_1_io_a_7 = io_inp_data_bits_0_7; // @[TensorGemm.scala 205:53]
  assign dot_0_1_io_a_8 = io_inp_data_bits_0_8; // @[TensorGemm.scala 205:53]
  assign dot_0_1_io_a_9 = io_inp_data_bits_0_9; // @[TensorGemm.scala 205:53]
  assign dot_0_1_io_a_10 = io_inp_data_bits_0_10; // @[TensorGemm.scala 205:53]
  assign dot_0_1_io_a_11 = io_inp_data_bits_0_11; // @[TensorGemm.scala 205:53]
  assign dot_0_1_io_a_12 = io_inp_data_bits_0_12; // @[TensorGemm.scala 205:53]
  assign dot_0_1_io_a_13 = io_inp_data_bits_0_13; // @[TensorGemm.scala 205:53]
  assign dot_0_1_io_a_14 = io_inp_data_bits_0_14; // @[TensorGemm.scala 205:53]
  assign dot_0_1_io_a_15 = io_inp_data_bits_0_15; // @[TensorGemm.scala 205:53]
  assign dot_0_1_io_b_0 = io_wgt_data_bits_1_0; // @[TensorGemm.scala 206:53]
  assign dot_0_1_io_b_1 = io_wgt_data_bits_1_1; // @[TensorGemm.scala 206:53]
  assign dot_0_1_io_b_2 = io_wgt_data_bits_1_2; // @[TensorGemm.scala 206:53]
  assign dot_0_1_io_b_3 = io_wgt_data_bits_1_3; // @[TensorGemm.scala 206:53]
  assign dot_0_1_io_b_4 = io_wgt_data_bits_1_4; // @[TensorGemm.scala 206:53]
  assign dot_0_1_io_b_5 = io_wgt_data_bits_1_5; // @[TensorGemm.scala 206:53]
  assign dot_0_1_io_b_6 = io_wgt_data_bits_1_6; // @[TensorGemm.scala 206:53]
  assign dot_0_1_io_b_7 = io_wgt_data_bits_1_7; // @[TensorGemm.scala 206:53]
  assign dot_0_1_io_b_8 = io_wgt_data_bits_1_8; // @[TensorGemm.scala 206:53]
  assign dot_0_1_io_b_9 = io_wgt_data_bits_1_9; // @[TensorGemm.scala 206:53]
  assign dot_0_1_io_b_10 = io_wgt_data_bits_1_10; // @[TensorGemm.scala 206:53]
  assign dot_0_1_io_b_11 = io_wgt_data_bits_1_11; // @[TensorGemm.scala 206:53]
  assign dot_0_1_io_b_12 = io_wgt_data_bits_1_12; // @[TensorGemm.scala 206:53]
  assign dot_0_1_io_b_13 = io_wgt_data_bits_1_13; // @[TensorGemm.scala 206:53]
  assign dot_0_1_io_b_14 = io_wgt_data_bits_1_14; // @[TensorGemm.scala 206:53]
  assign dot_0_1_io_b_15 = io_wgt_data_bits_1_15; // @[TensorGemm.scala 206:53]
  assign dot_0_2_clock = clock;
  assign dot_0_2_io_a_0 = io_inp_data_bits_0_0; // @[TensorGemm.scala 205:53]
  assign dot_0_2_io_a_1 = io_inp_data_bits_0_1; // @[TensorGemm.scala 205:53]
  assign dot_0_2_io_a_2 = io_inp_data_bits_0_2; // @[TensorGemm.scala 205:53]
  assign dot_0_2_io_a_3 = io_inp_data_bits_0_3; // @[TensorGemm.scala 205:53]
  assign dot_0_2_io_a_4 = io_inp_data_bits_0_4; // @[TensorGemm.scala 205:53]
  assign dot_0_2_io_a_5 = io_inp_data_bits_0_5; // @[TensorGemm.scala 205:53]
  assign dot_0_2_io_a_6 = io_inp_data_bits_0_6; // @[TensorGemm.scala 205:53]
  assign dot_0_2_io_a_7 = io_inp_data_bits_0_7; // @[TensorGemm.scala 205:53]
  assign dot_0_2_io_a_8 = io_inp_data_bits_0_8; // @[TensorGemm.scala 205:53]
  assign dot_0_2_io_a_9 = io_inp_data_bits_0_9; // @[TensorGemm.scala 205:53]
  assign dot_0_2_io_a_10 = io_inp_data_bits_0_10; // @[TensorGemm.scala 205:53]
  assign dot_0_2_io_a_11 = io_inp_data_bits_0_11; // @[TensorGemm.scala 205:53]
  assign dot_0_2_io_a_12 = io_inp_data_bits_0_12; // @[TensorGemm.scala 205:53]
  assign dot_0_2_io_a_13 = io_inp_data_bits_0_13; // @[TensorGemm.scala 205:53]
  assign dot_0_2_io_a_14 = io_inp_data_bits_0_14; // @[TensorGemm.scala 205:53]
  assign dot_0_2_io_a_15 = io_inp_data_bits_0_15; // @[TensorGemm.scala 205:53]
  assign dot_0_2_io_b_0 = io_wgt_data_bits_2_0; // @[TensorGemm.scala 206:53]
  assign dot_0_2_io_b_1 = io_wgt_data_bits_2_1; // @[TensorGemm.scala 206:53]
  assign dot_0_2_io_b_2 = io_wgt_data_bits_2_2; // @[TensorGemm.scala 206:53]
  assign dot_0_2_io_b_3 = io_wgt_data_bits_2_3; // @[TensorGemm.scala 206:53]
  assign dot_0_2_io_b_4 = io_wgt_data_bits_2_4; // @[TensorGemm.scala 206:53]
  assign dot_0_2_io_b_5 = io_wgt_data_bits_2_5; // @[TensorGemm.scala 206:53]
  assign dot_0_2_io_b_6 = io_wgt_data_bits_2_6; // @[TensorGemm.scala 206:53]
  assign dot_0_2_io_b_7 = io_wgt_data_bits_2_7; // @[TensorGemm.scala 206:53]
  assign dot_0_2_io_b_8 = io_wgt_data_bits_2_8; // @[TensorGemm.scala 206:53]
  assign dot_0_2_io_b_9 = io_wgt_data_bits_2_9; // @[TensorGemm.scala 206:53]
  assign dot_0_2_io_b_10 = io_wgt_data_bits_2_10; // @[TensorGemm.scala 206:53]
  assign dot_0_2_io_b_11 = io_wgt_data_bits_2_11; // @[TensorGemm.scala 206:53]
  assign dot_0_2_io_b_12 = io_wgt_data_bits_2_12; // @[TensorGemm.scala 206:53]
  assign dot_0_2_io_b_13 = io_wgt_data_bits_2_13; // @[TensorGemm.scala 206:53]
  assign dot_0_2_io_b_14 = io_wgt_data_bits_2_14; // @[TensorGemm.scala 206:53]
  assign dot_0_2_io_b_15 = io_wgt_data_bits_2_15; // @[TensorGemm.scala 206:53]
  assign dot_0_3_clock = clock;
  assign dot_0_3_io_a_0 = io_inp_data_bits_0_0; // @[TensorGemm.scala 205:53]
  assign dot_0_3_io_a_1 = io_inp_data_bits_0_1; // @[TensorGemm.scala 205:53]
  assign dot_0_3_io_a_2 = io_inp_data_bits_0_2; // @[TensorGemm.scala 205:53]
  assign dot_0_3_io_a_3 = io_inp_data_bits_0_3; // @[TensorGemm.scala 205:53]
  assign dot_0_3_io_a_4 = io_inp_data_bits_0_4; // @[TensorGemm.scala 205:53]
  assign dot_0_3_io_a_5 = io_inp_data_bits_0_5; // @[TensorGemm.scala 205:53]
  assign dot_0_3_io_a_6 = io_inp_data_bits_0_6; // @[TensorGemm.scala 205:53]
  assign dot_0_3_io_a_7 = io_inp_data_bits_0_7; // @[TensorGemm.scala 205:53]
  assign dot_0_3_io_a_8 = io_inp_data_bits_0_8; // @[TensorGemm.scala 205:53]
  assign dot_0_3_io_a_9 = io_inp_data_bits_0_9; // @[TensorGemm.scala 205:53]
  assign dot_0_3_io_a_10 = io_inp_data_bits_0_10; // @[TensorGemm.scala 205:53]
  assign dot_0_3_io_a_11 = io_inp_data_bits_0_11; // @[TensorGemm.scala 205:53]
  assign dot_0_3_io_a_12 = io_inp_data_bits_0_12; // @[TensorGemm.scala 205:53]
  assign dot_0_3_io_a_13 = io_inp_data_bits_0_13; // @[TensorGemm.scala 205:53]
  assign dot_0_3_io_a_14 = io_inp_data_bits_0_14; // @[TensorGemm.scala 205:53]
  assign dot_0_3_io_a_15 = io_inp_data_bits_0_15; // @[TensorGemm.scala 205:53]
  assign dot_0_3_io_b_0 = io_wgt_data_bits_3_0; // @[TensorGemm.scala 206:53]
  assign dot_0_3_io_b_1 = io_wgt_data_bits_3_1; // @[TensorGemm.scala 206:53]
  assign dot_0_3_io_b_2 = io_wgt_data_bits_3_2; // @[TensorGemm.scala 206:53]
  assign dot_0_3_io_b_3 = io_wgt_data_bits_3_3; // @[TensorGemm.scala 206:53]
  assign dot_0_3_io_b_4 = io_wgt_data_bits_3_4; // @[TensorGemm.scala 206:53]
  assign dot_0_3_io_b_5 = io_wgt_data_bits_3_5; // @[TensorGemm.scala 206:53]
  assign dot_0_3_io_b_6 = io_wgt_data_bits_3_6; // @[TensorGemm.scala 206:53]
  assign dot_0_3_io_b_7 = io_wgt_data_bits_3_7; // @[TensorGemm.scala 206:53]
  assign dot_0_3_io_b_8 = io_wgt_data_bits_3_8; // @[TensorGemm.scala 206:53]
  assign dot_0_3_io_b_9 = io_wgt_data_bits_3_9; // @[TensorGemm.scala 206:53]
  assign dot_0_3_io_b_10 = io_wgt_data_bits_3_10; // @[TensorGemm.scala 206:53]
  assign dot_0_3_io_b_11 = io_wgt_data_bits_3_11; // @[TensorGemm.scala 206:53]
  assign dot_0_3_io_b_12 = io_wgt_data_bits_3_12; // @[TensorGemm.scala 206:53]
  assign dot_0_3_io_b_13 = io_wgt_data_bits_3_13; // @[TensorGemm.scala 206:53]
  assign dot_0_3_io_b_14 = io_wgt_data_bits_3_14; // @[TensorGemm.scala 206:53]
  assign dot_0_3_io_b_15 = io_wgt_data_bits_3_15; // @[TensorGemm.scala 206:53]
  assign dot_0_4_clock = clock;
  assign dot_0_4_io_a_0 = io_inp_data_bits_0_0; // @[TensorGemm.scala 205:53]
  assign dot_0_4_io_a_1 = io_inp_data_bits_0_1; // @[TensorGemm.scala 205:53]
  assign dot_0_4_io_a_2 = io_inp_data_bits_0_2; // @[TensorGemm.scala 205:53]
  assign dot_0_4_io_a_3 = io_inp_data_bits_0_3; // @[TensorGemm.scala 205:53]
  assign dot_0_4_io_a_4 = io_inp_data_bits_0_4; // @[TensorGemm.scala 205:53]
  assign dot_0_4_io_a_5 = io_inp_data_bits_0_5; // @[TensorGemm.scala 205:53]
  assign dot_0_4_io_a_6 = io_inp_data_bits_0_6; // @[TensorGemm.scala 205:53]
  assign dot_0_4_io_a_7 = io_inp_data_bits_0_7; // @[TensorGemm.scala 205:53]
  assign dot_0_4_io_a_8 = io_inp_data_bits_0_8; // @[TensorGemm.scala 205:53]
  assign dot_0_4_io_a_9 = io_inp_data_bits_0_9; // @[TensorGemm.scala 205:53]
  assign dot_0_4_io_a_10 = io_inp_data_bits_0_10; // @[TensorGemm.scala 205:53]
  assign dot_0_4_io_a_11 = io_inp_data_bits_0_11; // @[TensorGemm.scala 205:53]
  assign dot_0_4_io_a_12 = io_inp_data_bits_0_12; // @[TensorGemm.scala 205:53]
  assign dot_0_4_io_a_13 = io_inp_data_bits_0_13; // @[TensorGemm.scala 205:53]
  assign dot_0_4_io_a_14 = io_inp_data_bits_0_14; // @[TensorGemm.scala 205:53]
  assign dot_0_4_io_a_15 = io_inp_data_bits_0_15; // @[TensorGemm.scala 205:53]
  assign dot_0_4_io_b_0 = io_wgt_data_bits_4_0; // @[TensorGemm.scala 206:53]
  assign dot_0_4_io_b_1 = io_wgt_data_bits_4_1; // @[TensorGemm.scala 206:53]
  assign dot_0_4_io_b_2 = io_wgt_data_bits_4_2; // @[TensorGemm.scala 206:53]
  assign dot_0_4_io_b_3 = io_wgt_data_bits_4_3; // @[TensorGemm.scala 206:53]
  assign dot_0_4_io_b_4 = io_wgt_data_bits_4_4; // @[TensorGemm.scala 206:53]
  assign dot_0_4_io_b_5 = io_wgt_data_bits_4_5; // @[TensorGemm.scala 206:53]
  assign dot_0_4_io_b_6 = io_wgt_data_bits_4_6; // @[TensorGemm.scala 206:53]
  assign dot_0_4_io_b_7 = io_wgt_data_bits_4_7; // @[TensorGemm.scala 206:53]
  assign dot_0_4_io_b_8 = io_wgt_data_bits_4_8; // @[TensorGemm.scala 206:53]
  assign dot_0_4_io_b_9 = io_wgt_data_bits_4_9; // @[TensorGemm.scala 206:53]
  assign dot_0_4_io_b_10 = io_wgt_data_bits_4_10; // @[TensorGemm.scala 206:53]
  assign dot_0_4_io_b_11 = io_wgt_data_bits_4_11; // @[TensorGemm.scala 206:53]
  assign dot_0_4_io_b_12 = io_wgt_data_bits_4_12; // @[TensorGemm.scala 206:53]
  assign dot_0_4_io_b_13 = io_wgt_data_bits_4_13; // @[TensorGemm.scala 206:53]
  assign dot_0_4_io_b_14 = io_wgt_data_bits_4_14; // @[TensorGemm.scala 206:53]
  assign dot_0_4_io_b_15 = io_wgt_data_bits_4_15; // @[TensorGemm.scala 206:53]
  assign dot_0_5_clock = clock;
  assign dot_0_5_io_a_0 = io_inp_data_bits_0_0; // @[TensorGemm.scala 205:53]
  assign dot_0_5_io_a_1 = io_inp_data_bits_0_1; // @[TensorGemm.scala 205:53]
  assign dot_0_5_io_a_2 = io_inp_data_bits_0_2; // @[TensorGemm.scala 205:53]
  assign dot_0_5_io_a_3 = io_inp_data_bits_0_3; // @[TensorGemm.scala 205:53]
  assign dot_0_5_io_a_4 = io_inp_data_bits_0_4; // @[TensorGemm.scala 205:53]
  assign dot_0_5_io_a_5 = io_inp_data_bits_0_5; // @[TensorGemm.scala 205:53]
  assign dot_0_5_io_a_6 = io_inp_data_bits_0_6; // @[TensorGemm.scala 205:53]
  assign dot_0_5_io_a_7 = io_inp_data_bits_0_7; // @[TensorGemm.scala 205:53]
  assign dot_0_5_io_a_8 = io_inp_data_bits_0_8; // @[TensorGemm.scala 205:53]
  assign dot_0_5_io_a_9 = io_inp_data_bits_0_9; // @[TensorGemm.scala 205:53]
  assign dot_0_5_io_a_10 = io_inp_data_bits_0_10; // @[TensorGemm.scala 205:53]
  assign dot_0_5_io_a_11 = io_inp_data_bits_0_11; // @[TensorGemm.scala 205:53]
  assign dot_0_5_io_a_12 = io_inp_data_bits_0_12; // @[TensorGemm.scala 205:53]
  assign dot_0_5_io_a_13 = io_inp_data_bits_0_13; // @[TensorGemm.scala 205:53]
  assign dot_0_5_io_a_14 = io_inp_data_bits_0_14; // @[TensorGemm.scala 205:53]
  assign dot_0_5_io_a_15 = io_inp_data_bits_0_15; // @[TensorGemm.scala 205:53]
  assign dot_0_5_io_b_0 = io_wgt_data_bits_5_0; // @[TensorGemm.scala 206:53]
  assign dot_0_5_io_b_1 = io_wgt_data_bits_5_1; // @[TensorGemm.scala 206:53]
  assign dot_0_5_io_b_2 = io_wgt_data_bits_5_2; // @[TensorGemm.scala 206:53]
  assign dot_0_5_io_b_3 = io_wgt_data_bits_5_3; // @[TensorGemm.scala 206:53]
  assign dot_0_5_io_b_4 = io_wgt_data_bits_5_4; // @[TensorGemm.scala 206:53]
  assign dot_0_5_io_b_5 = io_wgt_data_bits_5_5; // @[TensorGemm.scala 206:53]
  assign dot_0_5_io_b_6 = io_wgt_data_bits_5_6; // @[TensorGemm.scala 206:53]
  assign dot_0_5_io_b_7 = io_wgt_data_bits_5_7; // @[TensorGemm.scala 206:53]
  assign dot_0_5_io_b_8 = io_wgt_data_bits_5_8; // @[TensorGemm.scala 206:53]
  assign dot_0_5_io_b_9 = io_wgt_data_bits_5_9; // @[TensorGemm.scala 206:53]
  assign dot_0_5_io_b_10 = io_wgt_data_bits_5_10; // @[TensorGemm.scala 206:53]
  assign dot_0_5_io_b_11 = io_wgt_data_bits_5_11; // @[TensorGemm.scala 206:53]
  assign dot_0_5_io_b_12 = io_wgt_data_bits_5_12; // @[TensorGemm.scala 206:53]
  assign dot_0_5_io_b_13 = io_wgt_data_bits_5_13; // @[TensorGemm.scala 206:53]
  assign dot_0_5_io_b_14 = io_wgt_data_bits_5_14; // @[TensorGemm.scala 206:53]
  assign dot_0_5_io_b_15 = io_wgt_data_bits_5_15; // @[TensorGemm.scala 206:53]
  assign dot_0_6_clock = clock;
  assign dot_0_6_io_a_0 = io_inp_data_bits_0_0; // @[TensorGemm.scala 205:53]
  assign dot_0_6_io_a_1 = io_inp_data_bits_0_1; // @[TensorGemm.scala 205:53]
  assign dot_0_6_io_a_2 = io_inp_data_bits_0_2; // @[TensorGemm.scala 205:53]
  assign dot_0_6_io_a_3 = io_inp_data_bits_0_3; // @[TensorGemm.scala 205:53]
  assign dot_0_6_io_a_4 = io_inp_data_bits_0_4; // @[TensorGemm.scala 205:53]
  assign dot_0_6_io_a_5 = io_inp_data_bits_0_5; // @[TensorGemm.scala 205:53]
  assign dot_0_6_io_a_6 = io_inp_data_bits_0_6; // @[TensorGemm.scala 205:53]
  assign dot_0_6_io_a_7 = io_inp_data_bits_0_7; // @[TensorGemm.scala 205:53]
  assign dot_0_6_io_a_8 = io_inp_data_bits_0_8; // @[TensorGemm.scala 205:53]
  assign dot_0_6_io_a_9 = io_inp_data_bits_0_9; // @[TensorGemm.scala 205:53]
  assign dot_0_6_io_a_10 = io_inp_data_bits_0_10; // @[TensorGemm.scala 205:53]
  assign dot_0_6_io_a_11 = io_inp_data_bits_0_11; // @[TensorGemm.scala 205:53]
  assign dot_0_6_io_a_12 = io_inp_data_bits_0_12; // @[TensorGemm.scala 205:53]
  assign dot_0_6_io_a_13 = io_inp_data_bits_0_13; // @[TensorGemm.scala 205:53]
  assign dot_0_6_io_a_14 = io_inp_data_bits_0_14; // @[TensorGemm.scala 205:53]
  assign dot_0_6_io_a_15 = io_inp_data_bits_0_15; // @[TensorGemm.scala 205:53]
  assign dot_0_6_io_b_0 = io_wgt_data_bits_6_0; // @[TensorGemm.scala 206:53]
  assign dot_0_6_io_b_1 = io_wgt_data_bits_6_1; // @[TensorGemm.scala 206:53]
  assign dot_0_6_io_b_2 = io_wgt_data_bits_6_2; // @[TensorGemm.scala 206:53]
  assign dot_0_6_io_b_3 = io_wgt_data_bits_6_3; // @[TensorGemm.scala 206:53]
  assign dot_0_6_io_b_4 = io_wgt_data_bits_6_4; // @[TensorGemm.scala 206:53]
  assign dot_0_6_io_b_5 = io_wgt_data_bits_6_5; // @[TensorGemm.scala 206:53]
  assign dot_0_6_io_b_6 = io_wgt_data_bits_6_6; // @[TensorGemm.scala 206:53]
  assign dot_0_6_io_b_7 = io_wgt_data_bits_6_7; // @[TensorGemm.scala 206:53]
  assign dot_0_6_io_b_8 = io_wgt_data_bits_6_8; // @[TensorGemm.scala 206:53]
  assign dot_0_6_io_b_9 = io_wgt_data_bits_6_9; // @[TensorGemm.scala 206:53]
  assign dot_0_6_io_b_10 = io_wgt_data_bits_6_10; // @[TensorGemm.scala 206:53]
  assign dot_0_6_io_b_11 = io_wgt_data_bits_6_11; // @[TensorGemm.scala 206:53]
  assign dot_0_6_io_b_12 = io_wgt_data_bits_6_12; // @[TensorGemm.scala 206:53]
  assign dot_0_6_io_b_13 = io_wgt_data_bits_6_13; // @[TensorGemm.scala 206:53]
  assign dot_0_6_io_b_14 = io_wgt_data_bits_6_14; // @[TensorGemm.scala 206:53]
  assign dot_0_6_io_b_15 = io_wgt_data_bits_6_15; // @[TensorGemm.scala 206:53]
  assign dot_0_7_clock = clock;
  assign dot_0_7_io_a_0 = io_inp_data_bits_0_0; // @[TensorGemm.scala 205:53]
  assign dot_0_7_io_a_1 = io_inp_data_bits_0_1; // @[TensorGemm.scala 205:53]
  assign dot_0_7_io_a_2 = io_inp_data_bits_0_2; // @[TensorGemm.scala 205:53]
  assign dot_0_7_io_a_3 = io_inp_data_bits_0_3; // @[TensorGemm.scala 205:53]
  assign dot_0_7_io_a_4 = io_inp_data_bits_0_4; // @[TensorGemm.scala 205:53]
  assign dot_0_7_io_a_5 = io_inp_data_bits_0_5; // @[TensorGemm.scala 205:53]
  assign dot_0_7_io_a_6 = io_inp_data_bits_0_6; // @[TensorGemm.scala 205:53]
  assign dot_0_7_io_a_7 = io_inp_data_bits_0_7; // @[TensorGemm.scala 205:53]
  assign dot_0_7_io_a_8 = io_inp_data_bits_0_8; // @[TensorGemm.scala 205:53]
  assign dot_0_7_io_a_9 = io_inp_data_bits_0_9; // @[TensorGemm.scala 205:53]
  assign dot_0_7_io_a_10 = io_inp_data_bits_0_10; // @[TensorGemm.scala 205:53]
  assign dot_0_7_io_a_11 = io_inp_data_bits_0_11; // @[TensorGemm.scala 205:53]
  assign dot_0_7_io_a_12 = io_inp_data_bits_0_12; // @[TensorGemm.scala 205:53]
  assign dot_0_7_io_a_13 = io_inp_data_bits_0_13; // @[TensorGemm.scala 205:53]
  assign dot_0_7_io_a_14 = io_inp_data_bits_0_14; // @[TensorGemm.scala 205:53]
  assign dot_0_7_io_a_15 = io_inp_data_bits_0_15; // @[TensorGemm.scala 205:53]
  assign dot_0_7_io_b_0 = io_wgt_data_bits_7_0; // @[TensorGemm.scala 206:53]
  assign dot_0_7_io_b_1 = io_wgt_data_bits_7_1; // @[TensorGemm.scala 206:53]
  assign dot_0_7_io_b_2 = io_wgt_data_bits_7_2; // @[TensorGemm.scala 206:53]
  assign dot_0_7_io_b_3 = io_wgt_data_bits_7_3; // @[TensorGemm.scala 206:53]
  assign dot_0_7_io_b_4 = io_wgt_data_bits_7_4; // @[TensorGemm.scala 206:53]
  assign dot_0_7_io_b_5 = io_wgt_data_bits_7_5; // @[TensorGemm.scala 206:53]
  assign dot_0_7_io_b_6 = io_wgt_data_bits_7_6; // @[TensorGemm.scala 206:53]
  assign dot_0_7_io_b_7 = io_wgt_data_bits_7_7; // @[TensorGemm.scala 206:53]
  assign dot_0_7_io_b_8 = io_wgt_data_bits_7_8; // @[TensorGemm.scala 206:53]
  assign dot_0_7_io_b_9 = io_wgt_data_bits_7_9; // @[TensorGemm.scala 206:53]
  assign dot_0_7_io_b_10 = io_wgt_data_bits_7_10; // @[TensorGemm.scala 206:53]
  assign dot_0_7_io_b_11 = io_wgt_data_bits_7_11; // @[TensorGemm.scala 206:53]
  assign dot_0_7_io_b_12 = io_wgt_data_bits_7_12; // @[TensorGemm.scala 206:53]
  assign dot_0_7_io_b_13 = io_wgt_data_bits_7_13; // @[TensorGemm.scala 206:53]
  assign dot_0_7_io_b_14 = io_wgt_data_bits_7_14; // @[TensorGemm.scala 206:53]
  assign dot_0_7_io_b_15 = io_wgt_data_bits_7_15; // @[TensorGemm.scala 206:53]
  assign dot_0_8_clock = clock;
  assign dot_0_8_io_a_0 = io_inp_data_bits_0_0; // @[TensorGemm.scala 205:53]
  assign dot_0_8_io_a_1 = io_inp_data_bits_0_1; // @[TensorGemm.scala 205:53]
  assign dot_0_8_io_a_2 = io_inp_data_bits_0_2; // @[TensorGemm.scala 205:53]
  assign dot_0_8_io_a_3 = io_inp_data_bits_0_3; // @[TensorGemm.scala 205:53]
  assign dot_0_8_io_a_4 = io_inp_data_bits_0_4; // @[TensorGemm.scala 205:53]
  assign dot_0_8_io_a_5 = io_inp_data_bits_0_5; // @[TensorGemm.scala 205:53]
  assign dot_0_8_io_a_6 = io_inp_data_bits_0_6; // @[TensorGemm.scala 205:53]
  assign dot_0_8_io_a_7 = io_inp_data_bits_0_7; // @[TensorGemm.scala 205:53]
  assign dot_0_8_io_a_8 = io_inp_data_bits_0_8; // @[TensorGemm.scala 205:53]
  assign dot_0_8_io_a_9 = io_inp_data_bits_0_9; // @[TensorGemm.scala 205:53]
  assign dot_0_8_io_a_10 = io_inp_data_bits_0_10; // @[TensorGemm.scala 205:53]
  assign dot_0_8_io_a_11 = io_inp_data_bits_0_11; // @[TensorGemm.scala 205:53]
  assign dot_0_8_io_a_12 = io_inp_data_bits_0_12; // @[TensorGemm.scala 205:53]
  assign dot_0_8_io_a_13 = io_inp_data_bits_0_13; // @[TensorGemm.scala 205:53]
  assign dot_0_8_io_a_14 = io_inp_data_bits_0_14; // @[TensorGemm.scala 205:53]
  assign dot_0_8_io_a_15 = io_inp_data_bits_0_15; // @[TensorGemm.scala 205:53]
  assign dot_0_8_io_b_0 = io_wgt_data_bits_8_0; // @[TensorGemm.scala 206:53]
  assign dot_0_8_io_b_1 = io_wgt_data_bits_8_1; // @[TensorGemm.scala 206:53]
  assign dot_0_8_io_b_2 = io_wgt_data_bits_8_2; // @[TensorGemm.scala 206:53]
  assign dot_0_8_io_b_3 = io_wgt_data_bits_8_3; // @[TensorGemm.scala 206:53]
  assign dot_0_8_io_b_4 = io_wgt_data_bits_8_4; // @[TensorGemm.scala 206:53]
  assign dot_0_8_io_b_5 = io_wgt_data_bits_8_5; // @[TensorGemm.scala 206:53]
  assign dot_0_8_io_b_6 = io_wgt_data_bits_8_6; // @[TensorGemm.scala 206:53]
  assign dot_0_8_io_b_7 = io_wgt_data_bits_8_7; // @[TensorGemm.scala 206:53]
  assign dot_0_8_io_b_8 = io_wgt_data_bits_8_8; // @[TensorGemm.scala 206:53]
  assign dot_0_8_io_b_9 = io_wgt_data_bits_8_9; // @[TensorGemm.scala 206:53]
  assign dot_0_8_io_b_10 = io_wgt_data_bits_8_10; // @[TensorGemm.scala 206:53]
  assign dot_0_8_io_b_11 = io_wgt_data_bits_8_11; // @[TensorGemm.scala 206:53]
  assign dot_0_8_io_b_12 = io_wgt_data_bits_8_12; // @[TensorGemm.scala 206:53]
  assign dot_0_8_io_b_13 = io_wgt_data_bits_8_13; // @[TensorGemm.scala 206:53]
  assign dot_0_8_io_b_14 = io_wgt_data_bits_8_14; // @[TensorGemm.scala 206:53]
  assign dot_0_8_io_b_15 = io_wgt_data_bits_8_15; // @[TensorGemm.scala 206:53]
  assign dot_0_9_clock = clock;
  assign dot_0_9_io_a_0 = io_inp_data_bits_0_0; // @[TensorGemm.scala 205:53]
  assign dot_0_9_io_a_1 = io_inp_data_bits_0_1; // @[TensorGemm.scala 205:53]
  assign dot_0_9_io_a_2 = io_inp_data_bits_0_2; // @[TensorGemm.scala 205:53]
  assign dot_0_9_io_a_3 = io_inp_data_bits_0_3; // @[TensorGemm.scala 205:53]
  assign dot_0_9_io_a_4 = io_inp_data_bits_0_4; // @[TensorGemm.scala 205:53]
  assign dot_0_9_io_a_5 = io_inp_data_bits_0_5; // @[TensorGemm.scala 205:53]
  assign dot_0_9_io_a_6 = io_inp_data_bits_0_6; // @[TensorGemm.scala 205:53]
  assign dot_0_9_io_a_7 = io_inp_data_bits_0_7; // @[TensorGemm.scala 205:53]
  assign dot_0_9_io_a_8 = io_inp_data_bits_0_8; // @[TensorGemm.scala 205:53]
  assign dot_0_9_io_a_9 = io_inp_data_bits_0_9; // @[TensorGemm.scala 205:53]
  assign dot_0_9_io_a_10 = io_inp_data_bits_0_10; // @[TensorGemm.scala 205:53]
  assign dot_0_9_io_a_11 = io_inp_data_bits_0_11; // @[TensorGemm.scala 205:53]
  assign dot_0_9_io_a_12 = io_inp_data_bits_0_12; // @[TensorGemm.scala 205:53]
  assign dot_0_9_io_a_13 = io_inp_data_bits_0_13; // @[TensorGemm.scala 205:53]
  assign dot_0_9_io_a_14 = io_inp_data_bits_0_14; // @[TensorGemm.scala 205:53]
  assign dot_0_9_io_a_15 = io_inp_data_bits_0_15; // @[TensorGemm.scala 205:53]
  assign dot_0_9_io_b_0 = io_wgt_data_bits_9_0; // @[TensorGemm.scala 206:53]
  assign dot_0_9_io_b_1 = io_wgt_data_bits_9_1; // @[TensorGemm.scala 206:53]
  assign dot_0_9_io_b_2 = io_wgt_data_bits_9_2; // @[TensorGemm.scala 206:53]
  assign dot_0_9_io_b_3 = io_wgt_data_bits_9_3; // @[TensorGemm.scala 206:53]
  assign dot_0_9_io_b_4 = io_wgt_data_bits_9_4; // @[TensorGemm.scala 206:53]
  assign dot_0_9_io_b_5 = io_wgt_data_bits_9_5; // @[TensorGemm.scala 206:53]
  assign dot_0_9_io_b_6 = io_wgt_data_bits_9_6; // @[TensorGemm.scala 206:53]
  assign dot_0_9_io_b_7 = io_wgt_data_bits_9_7; // @[TensorGemm.scala 206:53]
  assign dot_0_9_io_b_8 = io_wgt_data_bits_9_8; // @[TensorGemm.scala 206:53]
  assign dot_0_9_io_b_9 = io_wgt_data_bits_9_9; // @[TensorGemm.scala 206:53]
  assign dot_0_9_io_b_10 = io_wgt_data_bits_9_10; // @[TensorGemm.scala 206:53]
  assign dot_0_9_io_b_11 = io_wgt_data_bits_9_11; // @[TensorGemm.scala 206:53]
  assign dot_0_9_io_b_12 = io_wgt_data_bits_9_12; // @[TensorGemm.scala 206:53]
  assign dot_0_9_io_b_13 = io_wgt_data_bits_9_13; // @[TensorGemm.scala 206:53]
  assign dot_0_9_io_b_14 = io_wgt_data_bits_9_14; // @[TensorGemm.scala 206:53]
  assign dot_0_9_io_b_15 = io_wgt_data_bits_9_15; // @[TensorGemm.scala 206:53]
  assign dot_0_10_clock = clock;
  assign dot_0_10_io_a_0 = io_inp_data_bits_0_0; // @[TensorGemm.scala 205:53]
  assign dot_0_10_io_a_1 = io_inp_data_bits_0_1; // @[TensorGemm.scala 205:53]
  assign dot_0_10_io_a_2 = io_inp_data_bits_0_2; // @[TensorGemm.scala 205:53]
  assign dot_0_10_io_a_3 = io_inp_data_bits_0_3; // @[TensorGemm.scala 205:53]
  assign dot_0_10_io_a_4 = io_inp_data_bits_0_4; // @[TensorGemm.scala 205:53]
  assign dot_0_10_io_a_5 = io_inp_data_bits_0_5; // @[TensorGemm.scala 205:53]
  assign dot_0_10_io_a_6 = io_inp_data_bits_0_6; // @[TensorGemm.scala 205:53]
  assign dot_0_10_io_a_7 = io_inp_data_bits_0_7; // @[TensorGemm.scala 205:53]
  assign dot_0_10_io_a_8 = io_inp_data_bits_0_8; // @[TensorGemm.scala 205:53]
  assign dot_0_10_io_a_9 = io_inp_data_bits_0_9; // @[TensorGemm.scala 205:53]
  assign dot_0_10_io_a_10 = io_inp_data_bits_0_10; // @[TensorGemm.scala 205:53]
  assign dot_0_10_io_a_11 = io_inp_data_bits_0_11; // @[TensorGemm.scala 205:53]
  assign dot_0_10_io_a_12 = io_inp_data_bits_0_12; // @[TensorGemm.scala 205:53]
  assign dot_0_10_io_a_13 = io_inp_data_bits_0_13; // @[TensorGemm.scala 205:53]
  assign dot_0_10_io_a_14 = io_inp_data_bits_0_14; // @[TensorGemm.scala 205:53]
  assign dot_0_10_io_a_15 = io_inp_data_bits_0_15; // @[TensorGemm.scala 205:53]
  assign dot_0_10_io_b_0 = io_wgt_data_bits_10_0; // @[TensorGemm.scala 206:53]
  assign dot_0_10_io_b_1 = io_wgt_data_bits_10_1; // @[TensorGemm.scala 206:53]
  assign dot_0_10_io_b_2 = io_wgt_data_bits_10_2; // @[TensorGemm.scala 206:53]
  assign dot_0_10_io_b_3 = io_wgt_data_bits_10_3; // @[TensorGemm.scala 206:53]
  assign dot_0_10_io_b_4 = io_wgt_data_bits_10_4; // @[TensorGemm.scala 206:53]
  assign dot_0_10_io_b_5 = io_wgt_data_bits_10_5; // @[TensorGemm.scala 206:53]
  assign dot_0_10_io_b_6 = io_wgt_data_bits_10_6; // @[TensorGemm.scala 206:53]
  assign dot_0_10_io_b_7 = io_wgt_data_bits_10_7; // @[TensorGemm.scala 206:53]
  assign dot_0_10_io_b_8 = io_wgt_data_bits_10_8; // @[TensorGemm.scala 206:53]
  assign dot_0_10_io_b_9 = io_wgt_data_bits_10_9; // @[TensorGemm.scala 206:53]
  assign dot_0_10_io_b_10 = io_wgt_data_bits_10_10; // @[TensorGemm.scala 206:53]
  assign dot_0_10_io_b_11 = io_wgt_data_bits_10_11; // @[TensorGemm.scala 206:53]
  assign dot_0_10_io_b_12 = io_wgt_data_bits_10_12; // @[TensorGemm.scala 206:53]
  assign dot_0_10_io_b_13 = io_wgt_data_bits_10_13; // @[TensorGemm.scala 206:53]
  assign dot_0_10_io_b_14 = io_wgt_data_bits_10_14; // @[TensorGemm.scala 206:53]
  assign dot_0_10_io_b_15 = io_wgt_data_bits_10_15; // @[TensorGemm.scala 206:53]
  assign dot_0_11_clock = clock;
  assign dot_0_11_io_a_0 = io_inp_data_bits_0_0; // @[TensorGemm.scala 205:53]
  assign dot_0_11_io_a_1 = io_inp_data_bits_0_1; // @[TensorGemm.scala 205:53]
  assign dot_0_11_io_a_2 = io_inp_data_bits_0_2; // @[TensorGemm.scala 205:53]
  assign dot_0_11_io_a_3 = io_inp_data_bits_0_3; // @[TensorGemm.scala 205:53]
  assign dot_0_11_io_a_4 = io_inp_data_bits_0_4; // @[TensorGemm.scala 205:53]
  assign dot_0_11_io_a_5 = io_inp_data_bits_0_5; // @[TensorGemm.scala 205:53]
  assign dot_0_11_io_a_6 = io_inp_data_bits_0_6; // @[TensorGemm.scala 205:53]
  assign dot_0_11_io_a_7 = io_inp_data_bits_0_7; // @[TensorGemm.scala 205:53]
  assign dot_0_11_io_a_8 = io_inp_data_bits_0_8; // @[TensorGemm.scala 205:53]
  assign dot_0_11_io_a_9 = io_inp_data_bits_0_9; // @[TensorGemm.scala 205:53]
  assign dot_0_11_io_a_10 = io_inp_data_bits_0_10; // @[TensorGemm.scala 205:53]
  assign dot_0_11_io_a_11 = io_inp_data_bits_0_11; // @[TensorGemm.scala 205:53]
  assign dot_0_11_io_a_12 = io_inp_data_bits_0_12; // @[TensorGemm.scala 205:53]
  assign dot_0_11_io_a_13 = io_inp_data_bits_0_13; // @[TensorGemm.scala 205:53]
  assign dot_0_11_io_a_14 = io_inp_data_bits_0_14; // @[TensorGemm.scala 205:53]
  assign dot_0_11_io_a_15 = io_inp_data_bits_0_15; // @[TensorGemm.scala 205:53]
  assign dot_0_11_io_b_0 = io_wgt_data_bits_11_0; // @[TensorGemm.scala 206:53]
  assign dot_0_11_io_b_1 = io_wgt_data_bits_11_1; // @[TensorGemm.scala 206:53]
  assign dot_0_11_io_b_2 = io_wgt_data_bits_11_2; // @[TensorGemm.scala 206:53]
  assign dot_0_11_io_b_3 = io_wgt_data_bits_11_3; // @[TensorGemm.scala 206:53]
  assign dot_0_11_io_b_4 = io_wgt_data_bits_11_4; // @[TensorGemm.scala 206:53]
  assign dot_0_11_io_b_5 = io_wgt_data_bits_11_5; // @[TensorGemm.scala 206:53]
  assign dot_0_11_io_b_6 = io_wgt_data_bits_11_6; // @[TensorGemm.scala 206:53]
  assign dot_0_11_io_b_7 = io_wgt_data_bits_11_7; // @[TensorGemm.scala 206:53]
  assign dot_0_11_io_b_8 = io_wgt_data_bits_11_8; // @[TensorGemm.scala 206:53]
  assign dot_0_11_io_b_9 = io_wgt_data_bits_11_9; // @[TensorGemm.scala 206:53]
  assign dot_0_11_io_b_10 = io_wgt_data_bits_11_10; // @[TensorGemm.scala 206:53]
  assign dot_0_11_io_b_11 = io_wgt_data_bits_11_11; // @[TensorGemm.scala 206:53]
  assign dot_0_11_io_b_12 = io_wgt_data_bits_11_12; // @[TensorGemm.scala 206:53]
  assign dot_0_11_io_b_13 = io_wgt_data_bits_11_13; // @[TensorGemm.scala 206:53]
  assign dot_0_11_io_b_14 = io_wgt_data_bits_11_14; // @[TensorGemm.scala 206:53]
  assign dot_0_11_io_b_15 = io_wgt_data_bits_11_15; // @[TensorGemm.scala 206:53]
  assign dot_0_12_clock = clock;
  assign dot_0_12_io_a_0 = io_inp_data_bits_0_0; // @[TensorGemm.scala 205:53]
  assign dot_0_12_io_a_1 = io_inp_data_bits_0_1; // @[TensorGemm.scala 205:53]
  assign dot_0_12_io_a_2 = io_inp_data_bits_0_2; // @[TensorGemm.scala 205:53]
  assign dot_0_12_io_a_3 = io_inp_data_bits_0_3; // @[TensorGemm.scala 205:53]
  assign dot_0_12_io_a_4 = io_inp_data_bits_0_4; // @[TensorGemm.scala 205:53]
  assign dot_0_12_io_a_5 = io_inp_data_bits_0_5; // @[TensorGemm.scala 205:53]
  assign dot_0_12_io_a_6 = io_inp_data_bits_0_6; // @[TensorGemm.scala 205:53]
  assign dot_0_12_io_a_7 = io_inp_data_bits_0_7; // @[TensorGemm.scala 205:53]
  assign dot_0_12_io_a_8 = io_inp_data_bits_0_8; // @[TensorGemm.scala 205:53]
  assign dot_0_12_io_a_9 = io_inp_data_bits_0_9; // @[TensorGemm.scala 205:53]
  assign dot_0_12_io_a_10 = io_inp_data_bits_0_10; // @[TensorGemm.scala 205:53]
  assign dot_0_12_io_a_11 = io_inp_data_bits_0_11; // @[TensorGemm.scala 205:53]
  assign dot_0_12_io_a_12 = io_inp_data_bits_0_12; // @[TensorGemm.scala 205:53]
  assign dot_0_12_io_a_13 = io_inp_data_bits_0_13; // @[TensorGemm.scala 205:53]
  assign dot_0_12_io_a_14 = io_inp_data_bits_0_14; // @[TensorGemm.scala 205:53]
  assign dot_0_12_io_a_15 = io_inp_data_bits_0_15; // @[TensorGemm.scala 205:53]
  assign dot_0_12_io_b_0 = io_wgt_data_bits_12_0; // @[TensorGemm.scala 206:53]
  assign dot_0_12_io_b_1 = io_wgt_data_bits_12_1; // @[TensorGemm.scala 206:53]
  assign dot_0_12_io_b_2 = io_wgt_data_bits_12_2; // @[TensorGemm.scala 206:53]
  assign dot_0_12_io_b_3 = io_wgt_data_bits_12_3; // @[TensorGemm.scala 206:53]
  assign dot_0_12_io_b_4 = io_wgt_data_bits_12_4; // @[TensorGemm.scala 206:53]
  assign dot_0_12_io_b_5 = io_wgt_data_bits_12_5; // @[TensorGemm.scala 206:53]
  assign dot_0_12_io_b_6 = io_wgt_data_bits_12_6; // @[TensorGemm.scala 206:53]
  assign dot_0_12_io_b_7 = io_wgt_data_bits_12_7; // @[TensorGemm.scala 206:53]
  assign dot_0_12_io_b_8 = io_wgt_data_bits_12_8; // @[TensorGemm.scala 206:53]
  assign dot_0_12_io_b_9 = io_wgt_data_bits_12_9; // @[TensorGemm.scala 206:53]
  assign dot_0_12_io_b_10 = io_wgt_data_bits_12_10; // @[TensorGemm.scala 206:53]
  assign dot_0_12_io_b_11 = io_wgt_data_bits_12_11; // @[TensorGemm.scala 206:53]
  assign dot_0_12_io_b_12 = io_wgt_data_bits_12_12; // @[TensorGemm.scala 206:53]
  assign dot_0_12_io_b_13 = io_wgt_data_bits_12_13; // @[TensorGemm.scala 206:53]
  assign dot_0_12_io_b_14 = io_wgt_data_bits_12_14; // @[TensorGemm.scala 206:53]
  assign dot_0_12_io_b_15 = io_wgt_data_bits_12_15; // @[TensorGemm.scala 206:53]
  assign dot_0_13_clock = clock;
  assign dot_0_13_io_a_0 = io_inp_data_bits_0_0; // @[TensorGemm.scala 205:53]
  assign dot_0_13_io_a_1 = io_inp_data_bits_0_1; // @[TensorGemm.scala 205:53]
  assign dot_0_13_io_a_2 = io_inp_data_bits_0_2; // @[TensorGemm.scala 205:53]
  assign dot_0_13_io_a_3 = io_inp_data_bits_0_3; // @[TensorGemm.scala 205:53]
  assign dot_0_13_io_a_4 = io_inp_data_bits_0_4; // @[TensorGemm.scala 205:53]
  assign dot_0_13_io_a_5 = io_inp_data_bits_0_5; // @[TensorGemm.scala 205:53]
  assign dot_0_13_io_a_6 = io_inp_data_bits_0_6; // @[TensorGemm.scala 205:53]
  assign dot_0_13_io_a_7 = io_inp_data_bits_0_7; // @[TensorGemm.scala 205:53]
  assign dot_0_13_io_a_8 = io_inp_data_bits_0_8; // @[TensorGemm.scala 205:53]
  assign dot_0_13_io_a_9 = io_inp_data_bits_0_9; // @[TensorGemm.scala 205:53]
  assign dot_0_13_io_a_10 = io_inp_data_bits_0_10; // @[TensorGemm.scala 205:53]
  assign dot_0_13_io_a_11 = io_inp_data_bits_0_11; // @[TensorGemm.scala 205:53]
  assign dot_0_13_io_a_12 = io_inp_data_bits_0_12; // @[TensorGemm.scala 205:53]
  assign dot_0_13_io_a_13 = io_inp_data_bits_0_13; // @[TensorGemm.scala 205:53]
  assign dot_0_13_io_a_14 = io_inp_data_bits_0_14; // @[TensorGemm.scala 205:53]
  assign dot_0_13_io_a_15 = io_inp_data_bits_0_15; // @[TensorGemm.scala 205:53]
  assign dot_0_13_io_b_0 = io_wgt_data_bits_13_0; // @[TensorGemm.scala 206:53]
  assign dot_0_13_io_b_1 = io_wgt_data_bits_13_1; // @[TensorGemm.scala 206:53]
  assign dot_0_13_io_b_2 = io_wgt_data_bits_13_2; // @[TensorGemm.scala 206:53]
  assign dot_0_13_io_b_3 = io_wgt_data_bits_13_3; // @[TensorGemm.scala 206:53]
  assign dot_0_13_io_b_4 = io_wgt_data_bits_13_4; // @[TensorGemm.scala 206:53]
  assign dot_0_13_io_b_5 = io_wgt_data_bits_13_5; // @[TensorGemm.scala 206:53]
  assign dot_0_13_io_b_6 = io_wgt_data_bits_13_6; // @[TensorGemm.scala 206:53]
  assign dot_0_13_io_b_7 = io_wgt_data_bits_13_7; // @[TensorGemm.scala 206:53]
  assign dot_0_13_io_b_8 = io_wgt_data_bits_13_8; // @[TensorGemm.scala 206:53]
  assign dot_0_13_io_b_9 = io_wgt_data_bits_13_9; // @[TensorGemm.scala 206:53]
  assign dot_0_13_io_b_10 = io_wgt_data_bits_13_10; // @[TensorGemm.scala 206:53]
  assign dot_0_13_io_b_11 = io_wgt_data_bits_13_11; // @[TensorGemm.scala 206:53]
  assign dot_0_13_io_b_12 = io_wgt_data_bits_13_12; // @[TensorGemm.scala 206:53]
  assign dot_0_13_io_b_13 = io_wgt_data_bits_13_13; // @[TensorGemm.scala 206:53]
  assign dot_0_13_io_b_14 = io_wgt_data_bits_13_14; // @[TensorGemm.scala 206:53]
  assign dot_0_13_io_b_15 = io_wgt_data_bits_13_15; // @[TensorGemm.scala 206:53]
  assign dot_0_14_clock = clock;
  assign dot_0_14_io_a_0 = io_inp_data_bits_0_0; // @[TensorGemm.scala 205:53]
  assign dot_0_14_io_a_1 = io_inp_data_bits_0_1; // @[TensorGemm.scala 205:53]
  assign dot_0_14_io_a_2 = io_inp_data_bits_0_2; // @[TensorGemm.scala 205:53]
  assign dot_0_14_io_a_3 = io_inp_data_bits_0_3; // @[TensorGemm.scala 205:53]
  assign dot_0_14_io_a_4 = io_inp_data_bits_0_4; // @[TensorGemm.scala 205:53]
  assign dot_0_14_io_a_5 = io_inp_data_bits_0_5; // @[TensorGemm.scala 205:53]
  assign dot_0_14_io_a_6 = io_inp_data_bits_0_6; // @[TensorGemm.scala 205:53]
  assign dot_0_14_io_a_7 = io_inp_data_bits_0_7; // @[TensorGemm.scala 205:53]
  assign dot_0_14_io_a_8 = io_inp_data_bits_0_8; // @[TensorGemm.scala 205:53]
  assign dot_0_14_io_a_9 = io_inp_data_bits_0_9; // @[TensorGemm.scala 205:53]
  assign dot_0_14_io_a_10 = io_inp_data_bits_0_10; // @[TensorGemm.scala 205:53]
  assign dot_0_14_io_a_11 = io_inp_data_bits_0_11; // @[TensorGemm.scala 205:53]
  assign dot_0_14_io_a_12 = io_inp_data_bits_0_12; // @[TensorGemm.scala 205:53]
  assign dot_0_14_io_a_13 = io_inp_data_bits_0_13; // @[TensorGemm.scala 205:53]
  assign dot_0_14_io_a_14 = io_inp_data_bits_0_14; // @[TensorGemm.scala 205:53]
  assign dot_0_14_io_a_15 = io_inp_data_bits_0_15; // @[TensorGemm.scala 205:53]
  assign dot_0_14_io_b_0 = io_wgt_data_bits_14_0; // @[TensorGemm.scala 206:53]
  assign dot_0_14_io_b_1 = io_wgt_data_bits_14_1; // @[TensorGemm.scala 206:53]
  assign dot_0_14_io_b_2 = io_wgt_data_bits_14_2; // @[TensorGemm.scala 206:53]
  assign dot_0_14_io_b_3 = io_wgt_data_bits_14_3; // @[TensorGemm.scala 206:53]
  assign dot_0_14_io_b_4 = io_wgt_data_bits_14_4; // @[TensorGemm.scala 206:53]
  assign dot_0_14_io_b_5 = io_wgt_data_bits_14_5; // @[TensorGemm.scala 206:53]
  assign dot_0_14_io_b_6 = io_wgt_data_bits_14_6; // @[TensorGemm.scala 206:53]
  assign dot_0_14_io_b_7 = io_wgt_data_bits_14_7; // @[TensorGemm.scala 206:53]
  assign dot_0_14_io_b_8 = io_wgt_data_bits_14_8; // @[TensorGemm.scala 206:53]
  assign dot_0_14_io_b_9 = io_wgt_data_bits_14_9; // @[TensorGemm.scala 206:53]
  assign dot_0_14_io_b_10 = io_wgt_data_bits_14_10; // @[TensorGemm.scala 206:53]
  assign dot_0_14_io_b_11 = io_wgt_data_bits_14_11; // @[TensorGemm.scala 206:53]
  assign dot_0_14_io_b_12 = io_wgt_data_bits_14_12; // @[TensorGemm.scala 206:53]
  assign dot_0_14_io_b_13 = io_wgt_data_bits_14_13; // @[TensorGemm.scala 206:53]
  assign dot_0_14_io_b_14 = io_wgt_data_bits_14_14; // @[TensorGemm.scala 206:53]
  assign dot_0_14_io_b_15 = io_wgt_data_bits_14_15; // @[TensorGemm.scala 206:53]
  assign dot_0_15_clock = clock;
  assign dot_0_15_io_a_0 = io_inp_data_bits_0_0; // @[TensorGemm.scala 205:53]
  assign dot_0_15_io_a_1 = io_inp_data_bits_0_1; // @[TensorGemm.scala 205:53]
  assign dot_0_15_io_a_2 = io_inp_data_bits_0_2; // @[TensorGemm.scala 205:53]
  assign dot_0_15_io_a_3 = io_inp_data_bits_0_3; // @[TensorGemm.scala 205:53]
  assign dot_0_15_io_a_4 = io_inp_data_bits_0_4; // @[TensorGemm.scala 205:53]
  assign dot_0_15_io_a_5 = io_inp_data_bits_0_5; // @[TensorGemm.scala 205:53]
  assign dot_0_15_io_a_6 = io_inp_data_bits_0_6; // @[TensorGemm.scala 205:53]
  assign dot_0_15_io_a_7 = io_inp_data_bits_0_7; // @[TensorGemm.scala 205:53]
  assign dot_0_15_io_a_8 = io_inp_data_bits_0_8; // @[TensorGemm.scala 205:53]
  assign dot_0_15_io_a_9 = io_inp_data_bits_0_9; // @[TensorGemm.scala 205:53]
  assign dot_0_15_io_a_10 = io_inp_data_bits_0_10; // @[TensorGemm.scala 205:53]
  assign dot_0_15_io_a_11 = io_inp_data_bits_0_11; // @[TensorGemm.scala 205:53]
  assign dot_0_15_io_a_12 = io_inp_data_bits_0_12; // @[TensorGemm.scala 205:53]
  assign dot_0_15_io_a_13 = io_inp_data_bits_0_13; // @[TensorGemm.scala 205:53]
  assign dot_0_15_io_a_14 = io_inp_data_bits_0_14; // @[TensorGemm.scala 205:53]
  assign dot_0_15_io_a_15 = io_inp_data_bits_0_15; // @[TensorGemm.scala 205:53]
  assign dot_0_15_io_b_0 = io_wgt_data_bits_15_0; // @[TensorGemm.scala 206:53]
  assign dot_0_15_io_b_1 = io_wgt_data_bits_15_1; // @[TensorGemm.scala 206:53]
  assign dot_0_15_io_b_2 = io_wgt_data_bits_15_2; // @[TensorGemm.scala 206:53]
  assign dot_0_15_io_b_3 = io_wgt_data_bits_15_3; // @[TensorGemm.scala 206:53]
  assign dot_0_15_io_b_4 = io_wgt_data_bits_15_4; // @[TensorGemm.scala 206:53]
  assign dot_0_15_io_b_5 = io_wgt_data_bits_15_5; // @[TensorGemm.scala 206:53]
  assign dot_0_15_io_b_6 = io_wgt_data_bits_15_6; // @[TensorGemm.scala 206:53]
  assign dot_0_15_io_b_7 = io_wgt_data_bits_15_7; // @[TensorGemm.scala 206:53]
  assign dot_0_15_io_b_8 = io_wgt_data_bits_15_8; // @[TensorGemm.scala 206:53]
  assign dot_0_15_io_b_9 = io_wgt_data_bits_15_9; // @[TensorGemm.scala 206:53]
  assign dot_0_15_io_b_10 = io_wgt_data_bits_15_10; // @[TensorGemm.scala 206:53]
  assign dot_0_15_io_b_11 = io_wgt_data_bits_15_11; // @[TensorGemm.scala 206:53]
  assign dot_0_15_io_b_12 = io_wgt_data_bits_15_12; // @[TensorGemm.scala 206:53]
  assign dot_0_15_io_b_13 = io_wgt_data_bits_15_13; // @[TensorGemm.scala 206:53]
  assign dot_0_15_io_b_14 = io_wgt_data_bits_15_14; // @[TensorGemm.scala 206:53]
  assign dot_0_15_io_b_15 = io_wgt_data_bits_15_15; // @[TensorGemm.scala 206:53]
  always @(posedge clock) begin
    if (io_valid_reset) begin // @[TensorGemm.scala 210:20]
      last_acc_write_0_0 <= 32'sh0;
    end else begin
      last_acc_write_0_0 <= add_0_0;
    end
    if (io_valid_reset) begin // @[TensorGemm.scala 210:20]
      last_acc_write_0_1 <= 32'sh0;
    end else begin
      last_acc_write_0_1 <= add_0_1;
    end
    if (io_valid_reset) begin // @[TensorGemm.scala 210:20]
      last_acc_write_0_2 <= 32'sh0;
    end else begin
      last_acc_write_0_2 <= add_0_2;
    end
    if (io_valid_reset) begin // @[TensorGemm.scala 210:20]
      last_acc_write_0_3 <= 32'sh0;
    end else begin
      last_acc_write_0_3 <= add_0_3;
    end
    if (io_valid_reset) begin // @[TensorGemm.scala 210:20]
      last_acc_write_0_4 <= 32'sh0;
    end else begin
      last_acc_write_0_4 <= add_0_4;
    end
    if (io_valid_reset) begin // @[TensorGemm.scala 210:20]
      last_acc_write_0_5 <= 32'sh0;
    end else begin
      last_acc_write_0_5 <= add_0_5;
    end
    if (io_valid_reset) begin // @[TensorGemm.scala 210:20]
      last_acc_write_0_6 <= 32'sh0;
    end else begin
      last_acc_write_0_6 <= add_0_6;
    end
    if (io_valid_reset) begin // @[TensorGemm.scala 210:20]
      last_acc_write_0_7 <= 32'sh0;
    end else begin
      last_acc_write_0_7 <= add_0_7;
    end
    if (io_valid_reset) begin // @[TensorGemm.scala 210:20]
      last_acc_write_0_8 <= 32'sh0;
    end else begin
      last_acc_write_0_8 <= add_0_8;
    end
    if (io_valid_reset) begin // @[TensorGemm.scala 210:20]
      last_acc_write_0_9 <= 32'sh0;
    end else begin
      last_acc_write_0_9 <= add_0_9;
    end
    if (io_valid_reset) begin // @[TensorGemm.scala 210:20]
      last_acc_write_0_10 <= 32'sh0;
    end else begin
      last_acc_write_0_10 <= add_0_10;
    end
    if (io_valid_reset) begin // @[TensorGemm.scala 210:20]
      last_acc_write_0_11 <= 32'sh0;
    end else begin
      last_acc_write_0_11 <= add_0_11;
    end
    if (io_valid_reset) begin // @[TensorGemm.scala 210:20]
      last_acc_write_0_12 <= 32'sh0;
    end else begin
      last_acc_write_0_12 <= add_0_12;
    end
    if (io_valid_reset) begin // @[TensorGemm.scala 210:20]
      last_acc_write_0_13 <= 32'sh0;
    end else begin
      last_acc_write_0_13 <= add_0_13;
    end
    if (io_valid_reset) begin // @[TensorGemm.scala 210:20]
      last_acc_write_0_14 <= 32'sh0;
    end else begin
      last_acc_write_0_14 <= add_0_14;
    end
    if (io_valid_reset) begin // @[TensorGemm.scala 210:20]
      last_acc_write_0_15 <= 32'sh0;
    end else begin
      last_acc_write_0_15 <= add_0_15;
    end
  end
// Register and memory initialization
`ifdef RANDOMIZE_GARBAGE_ASSIGN
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_INVALID_ASSIGN
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_REG_INIT
`define RANDOMIZE
`endif
`ifdef RANDOMIZE_MEM_INIT
`define RANDOMIZE
`endif
`ifndef RANDOM
`define RANDOM $random
`endif
`ifdef RANDOMIZE_MEM_INIT
  integer initvar;
`endif
`ifndef SYNTHESIS
`ifdef FIRRTL_BEFORE_INITIAL
`FIRRTL_BEFORE_INITIAL
`endif
initial begin
  `ifdef RANDOMIZE
    `ifdef INIT_RANDOM
      `INIT_RANDOM
    `endif
    `ifndef VERILATOR
      `ifdef RANDOMIZE_DELAY
        #`RANDOMIZE_DELAY begin end
      `else
        #0.002 begin end
      `endif
    `endif
`ifdef RANDOMIZE_REG_INIT
  _RAND_0 = {1{`RANDOM}};
  last_acc_write_0_0 = _RAND_0[31:0];
  _RAND_1 = {1{`RANDOM}};
  last_acc_write_0_1 = _RAND_1[31:0];
  _RAND_2 = {1{`RANDOM}};
  last_acc_write_0_2 = _RAND_2[31:0];
  _RAND_3 = {1{`RANDOM}};
  last_acc_write_0_3 = _RAND_3[31:0];
  _RAND_4 = {1{`RANDOM}};
  last_acc_write_0_4 = _RAND_4[31:0];
  _RAND_5 = {1{`RANDOM}};
  last_acc_write_0_5 = _RAND_5[31:0];
  _RAND_6 = {1{`RANDOM}};
  last_acc_write_0_6 = _RAND_6[31:0];
  _RAND_7 = {1{`RANDOM}};
  last_acc_write_0_7 = _RAND_7[31:0];
  _RAND_8 = {1{`RANDOM}};
  last_acc_write_0_8 = _RAND_8[31:0];
  _RAND_9 = {1{`RANDOM}};
  last_acc_write_0_9 = _RAND_9[31:0];
  _RAND_10 = {1{`RANDOM}};
  last_acc_write_0_10 = _RAND_10[31:0];
  _RAND_11 = {1{`RANDOM}};
  last_acc_write_0_11 = _RAND_11[31:0];
  _RAND_12 = {1{`RANDOM}};
  last_acc_write_0_12 = _RAND_12[31:0];
  _RAND_13 = {1{`RANDOM}};
  last_acc_write_0_13 = _RAND_13[31:0];
  _RAND_14 = {1{`RANDOM}};
  last_acc_write_0_14 = _RAND_14[31:0];
  _RAND_15 = {1{`RANDOM}};
  last_acc_write_0_15 = _RAND_15[31:0];
`endif // RANDOMIZE_REG_INIT
  `endif // RANDOMIZE
end // initial
`ifdef FIRRTL_AFTER_INITIAL
`FIRRTL_AFTER_INITIAL
`endif
`endif // SYNTHESIS
endmodule