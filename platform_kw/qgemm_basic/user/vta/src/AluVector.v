module AluVector(
  input         clock,
  input  [2:0]  io_opcode,
  input         io_acc_a_data_valid,
  input  [31:0] io_acc_a_data_bits_0_0,
  input  [31:0] io_acc_a_data_bits_0_1,
  input  [31:0] io_acc_a_data_bits_0_2,
  input  [31:0] io_acc_a_data_bits_0_3,
  input  [31:0] io_acc_a_data_bits_0_4,
  input  [31:0] io_acc_a_data_bits_0_5,
  input  [31:0] io_acc_a_data_bits_0_6,
  input  [31:0] io_acc_a_data_bits_0_7,
  input  [31:0] io_acc_a_data_bits_0_8,
  input  [31:0] io_acc_a_data_bits_0_9,
  input  [31:0] io_acc_a_data_bits_0_10,
  input  [31:0] io_acc_a_data_bits_0_11,
  input  [31:0] io_acc_a_data_bits_0_12,
  input  [31:0] io_acc_a_data_bits_0_13,
  input  [31:0] io_acc_a_data_bits_0_14,
  input  [31:0] io_acc_a_data_bits_0_15,
  input         io_acc_b_data_valid,
  input  [31:0] io_acc_b_data_bits_0_0,
  input  [31:0] io_acc_b_data_bits_0_1,
  input  [31:0] io_acc_b_data_bits_0_2,
  input  [31:0] io_acc_b_data_bits_0_3,
  input  [31:0] io_acc_b_data_bits_0_4,
  input  [31:0] io_acc_b_data_bits_0_5,
  input  [31:0] io_acc_b_data_bits_0_6,
  input  [31:0] io_acc_b_data_bits_0_7,
  input  [31:0] io_acc_b_data_bits_0_8,
  input  [31:0] io_acc_b_data_bits_0_9,
  input  [31:0] io_acc_b_data_bits_0_10,
  input  [31:0] io_acc_b_data_bits_0_11,
  input  [31:0] io_acc_b_data_bits_0_12,
  input  [31:0] io_acc_b_data_bits_0_13,
  input  [31:0] io_acc_b_data_bits_0_14,
  input  [31:0] io_acc_b_data_bits_0_15,
  output        io_acc_y_data_valid,
  output [31:0] io_acc_y_data_bits_0_0,
  output [31:0] io_acc_y_data_bits_0_1,
  output [31:0] io_acc_y_data_bits_0_2,
  output [31:0] io_acc_y_data_bits_0_3,
  output [31:0] io_acc_y_data_bits_0_4,
  output [31:0] io_acc_y_data_bits_0_5,
  output [31:0] io_acc_y_data_bits_0_6,
  output [31:0] io_acc_y_data_bits_0_7,
  output [31:0] io_acc_y_data_bits_0_8,
  output [31:0] io_acc_y_data_bits_0_9,
  output [31:0] io_acc_y_data_bits_0_10,
  output [31:0] io_acc_y_data_bits_0_11,
  output [31:0] io_acc_y_data_bits_0_12,
  output [31:0] io_acc_y_data_bits_0_13,
  output [31:0] io_acc_y_data_bits_0_14,
  output [31:0] io_acc_y_data_bits_0_15,
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
  output [31:0] io_out_data_bits_0_15
);
  wire  f_0_clock; // @[TensorAlu.scala 84:36]
  wire [2:0] f_0_io_opcode; // @[TensorAlu.scala 84:36]
  wire  f_0_io_a_valid; // @[TensorAlu.scala 84:36]
  wire [31:0] f_0_io_a_bits; // @[TensorAlu.scala 84:36]
  wire  f_0_io_b_valid; // @[TensorAlu.scala 84:36]
  wire [31:0] f_0_io_b_bits; // @[TensorAlu.scala 84:36]
  wire  f_0_io_y_valid; // @[TensorAlu.scala 84:36]
  wire [31:0] f_0_io_y_bits; // @[TensorAlu.scala 84:36]
  wire  f_1_clock; // @[TensorAlu.scala 84:36]
  wire [2:0] f_1_io_opcode; // @[TensorAlu.scala 84:36]
  wire  f_1_io_a_valid; // @[TensorAlu.scala 84:36]
  wire [31:0] f_1_io_a_bits; // @[TensorAlu.scala 84:36]
  wire  f_1_io_b_valid; // @[TensorAlu.scala 84:36]
  wire [31:0] f_1_io_b_bits; // @[TensorAlu.scala 84:36]
  wire  f_1_io_y_valid; // @[TensorAlu.scala 84:36]
  wire [31:0] f_1_io_y_bits; // @[TensorAlu.scala 84:36]
  wire  f_2_clock; // @[TensorAlu.scala 84:36]
  wire [2:0] f_2_io_opcode; // @[TensorAlu.scala 84:36]
  wire  f_2_io_a_valid; // @[TensorAlu.scala 84:36]
  wire [31:0] f_2_io_a_bits; // @[TensorAlu.scala 84:36]
  wire  f_2_io_b_valid; // @[TensorAlu.scala 84:36]
  wire [31:0] f_2_io_b_bits; // @[TensorAlu.scala 84:36]
  wire  f_2_io_y_valid; // @[TensorAlu.scala 84:36]
  wire [31:0] f_2_io_y_bits; // @[TensorAlu.scala 84:36]
  wire  f_3_clock; // @[TensorAlu.scala 84:36]
  wire [2:0] f_3_io_opcode; // @[TensorAlu.scala 84:36]
  wire  f_3_io_a_valid; // @[TensorAlu.scala 84:36]
  wire [31:0] f_3_io_a_bits; // @[TensorAlu.scala 84:36]
  wire  f_3_io_b_valid; // @[TensorAlu.scala 84:36]
  wire [31:0] f_3_io_b_bits; // @[TensorAlu.scala 84:36]
  wire  f_3_io_y_valid; // @[TensorAlu.scala 84:36]
  wire [31:0] f_3_io_y_bits; // @[TensorAlu.scala 84:36]
  wire  f_4_clock; // @[TensorAlu.scala 84:36]
  wire [2:0] f_4_io_opcode; // @[TensorAlu.scala 84:36]
  wire  f_4_io_a_valid; // @[TensorAlu.scala 84:36]
  wire [31:0] f_4_io_a_bits; // @[TensorAlu.scala 84:36]
  wire  f_4_io_b_valid; // @[TensorAlu.scala 84:36]
  wire [31:0] f_4_io_b_bits; // @[TensorAlu.scala 84:36]
  wire  f_4_io_y_valid; // @[TensorAlu.scala 84:36]
  wire [31:0] f_4_io_y_bits; // @[TensorAlu.scala 84:36]
  wire  f_5_clock; // @[TensorAlu.scala 84:36]
  wire [2:0] f_5_io_opcode; // @[TensorAlu.scala 84:36]
  wire  f_5_io_a_valid; // @[TensorAlu.scala 84:36]
  wire [31:0] f_5_io_a_bits; // @[TensorAlu.scala 84:36]
  wire  f_5_io_b_valid; // @[TensorAlu.scala 84:36]
  wire [31:0] f_5_io_b_bits; // @[TensorAlu.scala 84:36]
  wire  f_5_io_y_valid; // @[TensorAlu.scala 84:36]
  wire [31:0] f_5_io_y_bits; // @[TensorAlu.scala 84:36]
  wire  f_6_clock; // @[TensorAlu.scala 84:36]
  wire [2:0] f_6_io_opcode; // @[TensorAlu.scala 84:36]
  wire  f_6_io_a_valid; // @[TensorAlu.scala 84:36]
  wire [31:0] f_6_io_a_bits; // @[TensorAlu.scala 84:36]
  wire  f_6_io_b_valid; // @[TensorAlu.scala 84:36]
  wire [31:0] f_6_io_b_bits; // @[TensorAlu.scala 84:36]
  wire  f_6_io_y_valid; // @[TensorAlu.scala 84:36]
  wire [31:0] f_6_io_y_bits; // @[TensorAlu.scala 84:36]
  wire  f_7_clock; // @[TensorAlu.scala 84:36]
  wire [2:0] f_7_io_opcode; // @[TensorAlu.scala 84:36]
  wire  f_7_io_a_valid; // @[TensorAlu.scala 84:36]
  wire [31:0] f_7_io_a_bits; // @[TensorAlu.scala 84:36]
  wire  f_7_io_b_valid; // @[TensorAlu.scala 84:36]
  wire [31:0] f_7_io_b_bits; // @[TensorAlu.scala 84:36]
  wire  f_7_io_y_valid; // @[TensorAlu.scala 84:36]
  wire [31:0] f_7_io_y_bits; // @[TensorAlu.scala 84:36]
  wire  f_8_clock; // @[TensorAlu.scala 84:36]
  wire [2:0] f_8_io_opcode; // @[TensorAlu.scala 84:36]
  wire  f_8_io_a_valid; // @[TensorAlu.scala 84:36]
  wire [31:0] f_8_io_a_bits; // @[TensorAlu.scala 84:36]
  wire  f_8_io_b_valid; // @[TensorAlu.scala 84:36]
  wire [31:0] f_8_io_b_bits; // @[TensorAlu.scala 84:36]
  wire  f_8_io_y_valid; // @[TensorAlu.scala 84:36]
  wire [31:0] f_8_io_y_bits; // @[TensorAlu.scala 84:36]
  wire  f_9_clock; // @[TensorAlu.scala 84:36]
  wire [2:0] f_9_io_opcode; // @[TensorAlu.scala 84:36]
  wire  f_9_io_a_valid; // @[TensorAlu.scala 84:36]
  wire [31:0] f_9_io_a_bits; // @[TensorAlu.scala 84:36]
  wire  f_9_io_b_valid; // @[TensorAlu.scala 84:36]
  wire [31:0] f_9_io_b_bits; // @[TensorAlu.scala 84:36]
  wire  f_9_io_y_valid; // @[TensorAlu.scala 84:36]
  wire [31:0] f_9_io_y_bits; // @[TensorAlu.scala 84:36]
  wire  f_10_clock; // @[TensorAlu.scala 84:36]
  wire [2:0] f_10_io_opcode; // @[TensorAlu.scala 84:36]
  wire  f_10_io_a_valid; // @[TensorAlu.scala 84:36]
  wire [31:0] f_10_io_a_bits; // @[TensorAlu.scala 84:36]
  wire  f_10_io_b_valid; // @[TensorAlu.scala 84:36]
  wire [31:0] f_10_io_b_bits; // @[TensorAlu.scala 84:36]
  wire  f_10_io_y_valid; // @[TensorAlu.scala 84:36]
  wire [31:0] f_10_io_y_bits; // @[TensorAlu.scala 84:36]
  wire  f_11_clock; // @[TensorAlu.scala 84:36]
  wire [2:0] f_11_io_opcode; // @[TensorAlu.scala 84:36]
  wire  f_11_io_a_valid; // @[TensorAlu.scala 84:36]
  wire [31:0] f_11_io_a_bits; // @[TensorAlu.scala 84:36]
  wire  f_11_io_b_valid; // @[TensorAlu.scala 84:36]
  wire [31:0] f_11_io_b_bits; // @[TensorAlu.scala 84:36]
  wire  f_11_io_y_valid; // @[TensorAlu.scala 84:36]
  wire [31:0] f_11_io_y_bits; // @[TensorAlu.scala 84:36]
  wire  f_12_clock; // @[TensorAlu.scala 84:36]
  wire [2:0] f_12_io_opcode; // @[TensorAlu.scala 84:36]
  wire  f_12_io_a_valid; // @[TensorAlu.scala 84:36]
  wire [31:0] f_12_io_a_bits; // @[TensorAlu.scala 84:36]
  wire  f_12_io_b_valid; // @[TensorAlu.scala 84:36]
  wire [31:0] f_12_io_b_bits; // @[TensorAlu.scala 84:36]
  wire  f_12_io_y_valid; // @[TensorAlu.scala 84:36]
  wire [31:0] f_12_io_y_bits; // @[TensorAlu.scala 84:36]
  wire  f_13_clock; // @[TensorAlu.scala 84:36]
  wire [2:0] f_13_io_opcode; // @[TensorAlu.scala 84:36]
  wire  f_13_io_a_valid; // @[TensorAlu.scala 84:36]
  wire [31:0] f_13_io_a_bits; // @[TensorAlu.scala 84:36]
  wire  f_13_io_b_valid; // @[TensorAlu.scala 84:36]
  wire [31:0] f_13_io_b_bits; // @[TensorAlu.scala 84:36]
  wire  f_13_io_y_valid; // @[TensorAlu.scala 84:36]
  wire [31:0] f_13_io_y_bits; // @[TensorAlu.scala 84:36]
  wire  f_14_clock; // @[TensorAlu.scala 84:36]
  wire [2:0] f_14_io_opcode; // @[TensorAlu.scala 84:36]
  wire  f_14_io_a_valid; // @[TensorAlu.scala 84:36]
  wire [31:0] f_14_io_a_bits; // @[TensorAlu.scala 84:36]
  wire  f_14_io_b_valid; // @[TensorAlu.scala 84:36]
  wire [31:0] f_14_io_b_bits; // @[TensorAlu.scala 84:36]
  wire  f_14_io_y_valid; // @[TensorAlu.scala 84:36]
  wire [31:0] f_14_io_y_bits; // @[TensorAlu.scala 84:36]
  wire  f_15_clock; // @[TensorAlu.scala 84:36]
  wire [2:0] f_15_io_opcode; // @[TensorAlu.scala 84:36]
  wire  f_15_io_a_valid; // @[TensorAlu.scala 84:36]
  wire [31:0] f_15_io_a_bits; // @[TensorAlu.scala 84:36]
  wire  f_15_io_b_valid; // @[TensorAlu.scala 84:36]
  wire [31:0] f_15_io_b_bits; // @[TensorAlu.scala 84:36]
  wire  f_15_io_y_valid; // @[TensorAlu.scala 84:36]
  wire [31:0] f_15_io_y_bits; // @[TensorAlu.scala 84:36]
  wire  valid_1 = f_1_io_y_valid; // @[TensorAlu.scala 85:19 TensorAlu.scala 92:14]
  wire  valid_0 = f_0_io_y_valid; // @[TensorAlu.scala 85:19 TensorAlu.scala 92:14]
  wire  valid_3 = f_3_io_y_valid; // @[TensorAlu.scala 85:19 TensorAlu.scala 92:14]
  wire  valid_2 = f_2_io_y_valid; // @[TensorAlu.scala 85:19 TensorAlu.scala 92:14]
  wire  valid_5 = f_5_io_y_valid; // @[TensorAlu.scala 85:19 TensorAlu.scala 92:14]
  wire  valid_4 = f_4_io_y_valid; // @[TensorAlu.scala 85:19 TensorAlu.scala 92:14]
  wire  valid_7 = f_7_io_y_valid; // @[TensorAlu.scala 85:19 TensorAlu.scala 92:14]
  wire  valid_6 = f_6_io_y_valid; // @[TensorAlu.scala 85:19 TensorAlu.scala 92:14]
  wire [7:0] io_acc_y_data_valid_lo = {valid_7,valid_6,valid_5,valid_4,valid_3,valid_2,valid_1,valid_0}; // @[TensorAlu.scala 96:32]
  wire  valid_9 = f_9_io_y_valid; // @[TensorAlu.scala 85:19 TensorAlu.scala 92:14]
  wire  valid_8 = f_8_io_y_valid; // @[TensorAlu.scala 85:19 TensorAlu.scala 92:14]
  wire  valid_11 = f_11_io_y_valid; // @[TensorAlu.scala 85:19 TensorAlu.scala 92:14]
  wire  valid_10 = f_10_io_y_valid; // @[TensorAlu.scala 85:19 TensorAlu.scala 92:14]
  wire  valid_13 = f_13_io_y_valid; // @[TensorAlu.scala 85:19 TensorAlu.scala 92:14]
  wire  valid_12 = f_12_io_y_valid; // @[TensorAlu.scala 85:19 TensorAlu.scala 92:14]
  wire  valid_15 = f_15_io_y_valid; // @[TensorAlu.scala 85:19 TensorAlu.scala 92:14]
  wire  valid_14 = f_14_io_y_valid; // @[TensorAlu.scala 85:19 TensorAlu.scala 92:14]
  wire [15:0] _io_acc_y_data_valid_T = {valid_15,valid_14,valid_13,valid_12,valid_11,valid_10,valid_9,valid_8,
    io_acc_y_data_valid_lo}; // @[TensorAlu.scala 96:32]
  AluReg f_0 ( // @[TensorAlu.scala 84:36]
    .clock(f_0_clock),
    .io_opcode(f_0_io_opcode),
    .io_a_valid(f_0_io_a_valid),
    .io_a_bits(f_0_io_a_bits),
    .io_b_valid(f_0_io_b_valid),
    .io_b_bits(f_0_io_b_bits),
    .io_y_valid(f_0_io_y_valid),
    .io_y_bits(f_0_io_y_bits)
  );
  AluReg f_1 ( // @[TensorAlu.scala 84:36]
    .clock(f_1_clock),
    .io_opcode(f_1_io_opcode),
    .io_a_valid(f_1_io_a_valid),
    .io_a_bits(f_1_io_a_bits),
    .io_b_valid(f_1_io_b_valid),
    .io_b_bits(f_1_io_b_bits),
    .io_y_valid(f_1_io_y_valid),
    .io_y_bits(f_1_io_y_bits)
  );
  AluReg f_2 ( // @[TensorAlu.scala 84:36]
    .clock(f_2_clock),
    .io_opcode(f_2_io_opcode),
    .io_a_valid(f_2_io_a_valid),
    .io_a_bits(f_2_io_a_bits),
    .io_b_valid(f_2_io_b_valid),
    .io_b_bits(f_2_io_b_bits),
    .io_y_valid(f_2_io_y_valid),
    .io_y_bits(f_2_io_y_bits)
  );
  AluReg f_3 ( // @[TensorAlu.scala 84:36]
    .clock(f_3_clock),
    .io_opcode(f_3_io_opcode),
    .io_a_valid(f_3_io_a_valid),
    .io_a_bits(f_3_io_a_bits),
    .io_b_valid(f_3_io_b_valid),
    .io_b_bits(f_3_io_b_bits),
    .io_y_valid(f_3_io_y_valid),
    .io_y_bits(f_3_io_y_bits)
  );
  AluReg f_4 ( // @[TensorAlu.scala 84:36]
    .clock(f_4_clock),
    .io_opcode(f_4_io_opcode),
    .io_a_valid(f_4_io_a_valid),
    .io_a_bits(f_4_io_a_bits),
    .io_b_valid(f_4_io_b_valid),
    .io_b_bits(f_4_io_b_bits),
    .io_y_valid(f_4_io_y_valid),
    .io_y_bits(f_4_io_y_bits)
  );
  AluReg f_5 ( // @[TensorAlu.scala 84:36]
    .clock(f_5_clock),
    .io_opcode(f_5_io_opcode),
    .io_a_valid(f_5_io_a_valid),
    .io_a_bits(f_5_io_a_bits),
    .io_b_valid(f_5_io_b_valid),
    .io_b_bits(f_5_io_b_bits),
    .io_y_valid(f_5_io_y_valid),
    .io_y_bits(f_5_io_y_bits)
  );
  AluReg f_6 ( // @[TensorAlu.scala 84:36]
    .clock(f_6_clock),
    .io_opcode(f_6_io_opcode),
    .io_a_valid(f_6_io_a_valid),
    .io_a_bits(f_6_io_a_bits),
    .io_b_valid(f_6_io_b_valid),
    .io_b_bits(f_6_io_b_bits),
    .io_y_valid(f_6_io_y_valid),
    .io_y_bits(f_6_io_y_bits)
  );
  AluReg f_7 ( // @[TensorAlu.scala 84:36]
    .clock(f_7_clock),
    .io_opcode(f_7_io_opcode),
    .io_a_valid(f_7_io_a_valid),
    .io_a_bits(f_7_io_a_bits),
    .io_b_valid(f_7_io_b_valid),
    .io_b_bits(f_7_io_b_bits),
    .io_y_valid(f_7_io_y_valid),
    .io_y_bits(f_7_io_y_bits)
  );
  AluReg f_8 ( // @[TensorAlu.scala 84:36]
    .clock(f_8_clock),
    .io_opcode(f_8_io_opcode),
    .io_a_valid(f_8_io_a_valid),
    .io_a_bits(f_8_io_a_bits),
    .io_b_valid(f_8_io_b_valid),
    .io_b_bits(f_8_io_b_bits),
    .io_y_valid(f_8_io_y_valid),
    .io_y_bits(f_8_io_y_bits)
  );
  AluReg f_9 ( // @[TensorAlu.scala 84:36]
    .clock(f_9_clock),
    .io_opcode(f_9_io_opcode),
    .io_a_valid(f_9_io_a_valid),
    .io_a_bits(f_9_io_a_bits),
    .io_b_valid(f_9_io_b_valid),
    .io_b_bits(f_9_io_b_bits),
    .io_y_valid(f_9_io_y_valid),
    .io_y_bits(f_9_io_y_bits)
  );
  AluReg f_10 ( // @[TensorAlu.scala 84:36]
    .clock(f_10_clock),
    .io_opcode(f_10_io_opcode),
    .io_a_valid(f_10_io_a_valid),
    .io_a_bits(f_10_io_a_bits),
    .io_b_valid(f_10_io_b_valid),
    .io_b_bits(f_10_io_b_bits),
    .io_y_valid(f_10_io_y_valid),
    .io_y_bits(f_10_io_y_bits)
  );
  AluReg f_11 ( // @[TensorAlu.scala 84:36]
    .clock(f_11_clock),
    .io_opcode(f_11_io_opcode),
    .io_a_valid(f_11_io_a_valid),
    .io_a_bits(f_11_io_a_bits),
    .io_b_valid(f_11_io_b_valid),
    .io_b_bits(f_11_io_b_bits),
    .io_y_valid(f_11_io_y_valid),
    .io_y_bits(f_11_io_y_bits)
  );
  AluReg f_12 ( // @[TensorAlu.scala 84:36]
    .clock(f_12_clock),
    .io_opcode(f_12_io_opcode),
    .io_a_valid(f_12_io_a_valid),
    .io_a_bits(f_12_io_a_bits),
    .io_b_valid(f_12_io_b_valid),
    .io_b_bits(f_12_io_b_bits),
    .io_y_valid(f_12_io_y_valid),
    .io_y_bits(f_12_io_y_bits)
  );
  AluReg f_13 ( // @[TensorAlu.scala 84:36]
    .clock(f_13_clock),
    .io_opcode(f_13_io_opcode),
    .io_a_valid(f_13_io_a_valid),
    .io_a_bits(f_13_io_a_bits),
    .io_b_valid(f_13_io_b_valid),
    .io_b_bits(f_13_io_b_bits),
    .io_y_valid(f_13_io_y_valid),
    .io_y_bits(f_13_io_y_bits)
  );
  AluReg f_14 ( // @[TensorAlu.scala 84:36]
    .clock(f_14_clock),
    .io_opcode(f_14_io_opcode),
    .io_a_valid(f_14_io_a_valid),
    .io_a_bits(f_14_io_a_bits),
    .io_b_valid(f_14_io_b_valid),
    .io_b_bits(f_14_io_b_bits),
    .io_y_valid(f_14_io_y_valid),
    .io_y_bits(f_14_io_y_bits)
  );
  AluReg f_15 ( // @[TensorAlu.scala 84:36]
    .clock(f_15_clock),
    .io_opcode(f_15_io_opcode),
    .io_a_valid(f_15_io_a_valid),
    .io_a_bits(f_15_io_a_bits),
    .io_b_valid(f_15_io_b_valid),
    .io_b_bits(f_15_io_b_bits),
    .io_y_valid(f_15_io_y_valid),
    .io_y_bits(f_15_io_y_bits)
  );
  assign io_acc_y_data_valid = &_io_acc_y_data_valid_T; // @[TensorAlu.scala 96:39]
  assign io_acc_y_data_bits_0_0 = f_0_io_y_bits; // @[TensorAlu.scala 93:30]
  assign io_acc_y_data_bits_0_1 = f_1_io_y_bits; // @[TensorAlu.scala 93:30]
  assign io_acc_y_data_bits_0_2 = f_2_io_y_bits; // @[TensorAlu.scala 93:30]
  assign io_acc_y_data_bits_0_3 = f_3_io_y_bits; // @[TensorAlu.scala 93:30]
  assign io_acc_y_data_bits_0_4 = f_4_io_y_bits; // @[TensorAlu.scala 93:30]
  assign io_acc_y_data_bits_0_5 = f_5_io_y_bits; // @[TensorAlu.scala 93:30]
  assign io_acc_y_data_bits_0_6 = f_6_io_y_bits; // @[TensorAlu.scala 93:30]
  assign io_acc_y_data_bits_0_7 = f_7_io_y_bits; // @[TensorAlu.scala 93:30]
  assign io_acc_y_data_bits_0_8 = f_8_io_y_bits; // @[TensorAlu.scala 93:30]
  assign io_acc_y_data_bits_0_9 = f_9_io_y_bits; // @[TensorAlu.scala 93:30]
  assign io_acc_y_data_bits_0_10 = f_10_io_y_bits; // @[TensorAlu.scala 93:30]
  assign io_acc_y_data_bits_0_11 = f_11_io_y_bits; // @[TensorAlu.scala 93:30]
  assign io_acc_y_data_bits_0_12 = f_12_io_y_bits; // @[TensorAlu.scala 93:30]
  assign io_acc_y_data_bits_0_13 = f_13_io_y_bits; // @[TensorAlu.scala 93:30]
  assign io_acc_y_data_bits_0_14 = f_14_io_y_bits; // @[TensorAlu.scala 93:30]
  assign io_acc_y_data_bits_0_15 = f_15_io_y_bits; // @[TensorAlu.scala 93:30]
  assign io_out_data_valid = &_io_acc_y_data_valid_T; // @[TensorAlu.scala 97:37]
  assign io_out_data_bits_0_0 = f_0_io_y_bits; // @[TensorAlu.scala 94:28]
  assign io_out_data_bits_0_1 = f_1_io_y_bits; // @[TensorAlu.scala 94:28]
  assign io_out_data_bits_0_2 = f_2_io_y_bits; // @[TensorAlu.scala 94:28]
  assign io_out_data_bits_0_3 = f_3_io_y_bits; // @[TensorAlu.scala 94:28]
  assign io_out_data_bits_0_4 = f_4_io_y_bits; // @[TensorAlu.scala 94:28]
  assign io_out_data_bits_0_5 = f_5_io_y_bits; // @[TensorAlu.scala 94:28]
  assign io_out_data_bits_0_6 = f_6_io_y_bits; // @[TensorAlu.scala 94:28]
  assign io_out_data_bits_0_7 = f_7_io_y_bits; // @[TensorAlu.scala 94:28]
  assign io_out_data_bits_0_8 = f_8_io_y_bits; // @[TensorAlu.scala 94:28]
  assign io_out_data_bits_0_9 = f_9_io_y_bits; // @[TensorAlu.scala 94:28]
  assign io_out_data_bits_0_10 = f_10_io_y_bits; // @[TensorAlu.scala 94:28]
  assign io_out_data_bits_0_11 = f_11_io_y_bits; // @[TensorAlu.scala 94:28]
  assign io_out_data_bits_0_12 = f_12_io_y_bits; // @[TensorAlu.scala 94:28]
  assign io_out_data_bits_0_13 = f_13_io_y_bits; // @[TensorAlu.scala 94:28]
  assign io_out_data_bits_0_14 = f_14_io_y_bits; // @[TensorAlu.scala 94:28]
  assign io_out_data_bits_0_15 = f_15_io_y_bits; // @[TensorAlu.scala 94:28]
  assign f_0_clock = clock;
  assign f_0_io_opcode = io_opcode; // @[TensorAlu.scala 87:20]
  assign f_0_io_a_valid = io_acc_a_data_valid; // @[TensorAlu.scala 88:21]
  assign f_0_io_a_bits = io_acc_a_data_bits_0_0; // @[TensorAlu.scala 89:20]
  assign f_0_io_b_valid = io_acc_b_data_valid; // @[TensorAlu.scala 90:21]
  assign f_0_io_b_bits = io_acc_b_data_bits_0_0; // @[TensorAlu.scala 91:20]
  assign f_1_clock = clock;
  assign f_1_io_opcode = io_opcode; // @[TensorAlu.scala 87:20]
  assign f_1_io_a_valid = io_acc_a_data_valid; // @[TensorAlu.scala 88:21]
  assign f_1_io_a_bits = io_acc_a_data_bits_0_1; // @[TensorAlu.scala 89:20]
  assign f_1_io_b_valid = io_acc_b_data_valid; // @[TensorAlu.scala 90:21]
  assign f_1_io_b_bits = io_acc_b_data_bits_0_1; // @[TensorAlu.scala 91:20]
  assign f_2_clock = clock;
  assign f_2_io_opcode = io_opcode; // @[TensorAlu.scala 87:20]
  assign f_2_io_a_valid = io_acc_a_data_valid; // @[TensorAlu.scala 88:21]
  assign f_2_io_a_bits = io_acc_a_data_bits_0_2; // @[TensorAlu.scala 89:20]
  assign f_2_io_b_valid = io_acc_b_data_valid; // @[TensorAlu.scala 90:21]
  assign f_2_io_b_bits = io_acc_b_data_bits_0_2; // @[TensorAlu.scala 91:20]
  assign f_3_clock = clock;
  assign f_3_io_opcode = io_opcode; // @[TensorAlu.scala 87:20]
  assign f_3_io_a_valid = io_acc_a_data_valid; // @[TensorAlu.scala 88:21]
  assign f_3_io_a_bits = io_acc_a_data_bits_0_3; // @[TensorAlu.scala 89:20]
  assign f_3_io_b_valid = io_acc_b_data_valid; // @[TensorAlu.scala 90:21]
  assign f_3_io_b_bits = io_acc_b_data_bits_0_3; // @[TensorAlu.scala 91:20]
  assign f_4_clock = clock;
  assign f_4_io_opcode = io_opcode; // @[TensorAlu.scala 87:20]
  assign f_4_io_a_valid = io_acc_a_data_valid; // @[TensorAlu.scala 88:21]
  assign f_4_io_a_bits = io_acc_a_data_bits_0_4; // @[TensorAlu.scala 89:20]
  assign f_4_io_b_valid = io_acc_b_data_valid; // @[TensorAlu.scala 90:21]
  assign f_4_io_b_bits = io_acc_b_data_bits_0_4; // @[TensorAlu.scala 91:20]
  assign f_5_clock = clock;
  assign f_5_io_opcode = io_opcode; // @[TensorAlu.scala 87:20]
  assign f_5_io_a_valid = io_acc_a_data_valid; // @[TensorAlu.scala 88:21]
  assign f_5_io_a_bits = io_acc_a_data_bits_0_5; // @[TensorAlu.scala 89:20]
  assign f_5_io_b_valid = io_acc_b_data_valid; // @[TensorAlu.scala 90:21]
  assign f_5_io_b_bits = io_acc_b_data_bits_0_5; // @[TensorAlu.scala 91:20]
  assign f_6_clock = clock;
  assign f_6_io_opcode = io_opcode; // @[TensorAlu.scala 87:20]
  assign f_6_io_a_valid = io_acc_a_data_valid; // @[TensorAlu.scala 88:21]
  assign f_6_io_a_bits = io_acc_a_data_bits_0_6; // @[TensorAlu.scala 89:20]
  assign f_6_io_b_valid = io_acc_b_data_valid; // @[TensorAlu.scala 90:21]
  assign f_6_io_b_bits = io_acc_b_data_bits_0_6; // @[TensorAlu.scala 91:20]
  assign f_7_clock = clock;
  assign f_7_io_opcode = io_opcode; // @[TensorAlu.scala 87:20]
  assign f_7_io_a_valid = io_acc_a_data_valid; // @[TensorAlu.scala 88:21]
  assign f_7_io_a_bits = io_acc_a_data_bits_0_7; // @[TensorAlu.scala 89:20]
  assign f_7_io_b_valid = io_acc_b_data_valid; // @[TensorAlu.scala 90:21]
  assign f_7_io_b_bits = io_acc_b_data_bits_0_7; // @[TensorAlu.scala 91:20]
  assign f_8_clock = clock;
  assign f_8_io_opcode = io_opcode; // @[TensorAlu.scala 87:20]
  assign f_8_io_a_valid = io_acc_a_data_valid; // @[TensorAlu.scala 88:21]
  assign f_8_io_a_bits = io_acc_a_data_bits_0_8; // @[TensorAlu.scala 89:20]
  assign f_8_io_b_valid = io_acc_b_data_valid; // @[TensorAlu.scala 90:21]
  assign f_8_io_b_bits = io_acc_b_data_bits_0_8; // @[TensorAlu.scala 91:20]
  assign f_9_clock = clock;
  assign f_9_io_opcode = io_opcode; // @[TensorAlu.scala 87:20]
  assign f_9_io_a_valid = io_acc_a_data_valid; // @[TensorAlu.scala 88:21]
  assign f_9_io_a_bits = io_acc_a_data_bits_0_9; // @[TensorAlu.scala 89:20]
  assign f_9_io_b_valid = io_acc_b_data_valid; // @[TensorAlu.scala 90:21]
  assign f_9_io_b_bits = io_acc_b_data_bits_0_9; // @[TensorAlu.scala 91:20]
  assign f_10_clock = clock;
  assign f_10_io_opcode = io_opcode; // @[TensorAlu.scala 87:20]
  assign f_10_io_a_valid = io_acc_a_data_valid; // @[TensorAlu.scala 88:21]
  assign f_10_io_a_bits = io_acc_a_data_bits_0_10; // @[TensorAlu.scala 89:20]
  assign f_10_io_b_valid = io_acc_b_data_valid; // @[TensorAlu.scala 90:21]
  assign f_10_io_b_bits = io_acc_b_data_bits_0_10; // @[TensorAlu.scala 91:20]
  assign f_11_clock = clock;
  assign f_11_io_opcode = io_opcode; // @[TensorAlu.scala 87:20]
  assign f_11_io_a_valid = io_acc_a_data_valid; // @[TensorAlu.scala 88:21]
  assign f_11_io_a_bits = io_acc_a_data_bits_0_11; // @[TensorAlu.scala 89:20]
  assign f_11_io_b_valid = io_acc_b_data_valid; // @[TensorAlu.scala 90:21]
  assign f_11_io_b_bits = io_acc_b_data_bits_0_11; // @[TensorAlu.scala 91:20]
  assign f_12_clock = clock;
  assign f_12_io_opcode = io_opcode; // @[TensorAlu.scala 87:20]
  assign f_12_io_a_valid = io_acc_a_data_valid; // @[TensorAlu.scala 88:21]
  assign f_12_io_a_bits = io_acc_a_data_bits_0_12; // @[TensorAlu.scala 89:20]
  assign f_12_io_b_valid = io_acc_b_data_valid; // @[TensorAlu.scala 90:21]
  assign f_12_io_b_bits = io_acc_b_data_bits_0_12; // @[TensorAlu.scala 91:20]
  assign f_13_clock = clock;
  assign f_13_io_opcode = io_opcode; // @[TensorAlu.scala 87:20]
  assign f_13_io_a_valid = io_acc_a_data_valid; // @[TensorAlu.scala 88:21]
  assign f_13_io_a_bits = io_acc_a_data_bits_0_13; // @[TensorAlu.scala 89:20]
  assign f_13_io_b_valid = io_acc_b_data_valid; // @[TensorAlu.scala 90:21]
  assign f_13_io_b_bits = io_acc_b_data_bits_0_13; // @[TensorAlu.scala 91:20]
  assign f_14_clock = clock;
  assign f_14_io_opcode = io_opcode; // @[TensorAlu.scala 87:20]
  assign f_14_io_a_valid = io_acc_a_data_valid; // @[TensorAlu.scala 88:21]
  assign f_14_io_a_bits = io_acc_a_data_bits_0_14; // @[TensorAlu.scala 89:20]
  assign f_14_io_b_valid = io_acc_b_data_valid; // @[TensorAlu.scala 90:21]
  assign f_14_io_b_bits = io_acc_b_data_bits_0_14; // @[TensorAlu.scala 91:20]
  assign f_15_clock = clock;
  assign f_15_io_opcode = io_opcode; // @[TensorAlu.scala 87:20]
  assign f_15_io_a_valid = io_acc_a_data_valid; // @[TensorAlu.scala 88:21]
  assign f_15_io_a_bits = io_acc_a_data_bits_0_15; // @[TensorAlu.scala 89:20]
  assign f_15_io_b_valid = io_acc_b_data_valid; // @[TensorAlu.scala 90:21]
  assign f_15_io_b_bits = io_acc_b_data_bits_0_15; // @[TensorAlu.scala 91:20]
endmodule