module TensorAlu(
  input         clock,
  input         reset,
  input         io_start,
  output        io_done,
  input  [15:0] io_dec_alu_imm,
  input         io_dec_alu_use_imm,
  input  [2:0]  io_dec_alu_op,
  input  [10:0] io_dec_src_1,
  input  [10:0] io_dec_src_0,
  input  [10:0] io_dec_dst_1,
  input  [10:0] io_dec_dst_0,
  input  [13:0] io_dec_lp_1,
  input  [13:0] io_dec_lp_0,
  input  [13:0] io_dec_uop_end,
  input  [12:0] io_dec_uop_begin,
  output        io_uop_idx_valid,
  output [10:0] io_uop_idx_bits,
  input  [9:0]  io_uop_data_bits_u2,
  input  [10:0] io_uop_data_bits_u1,
  input  [10:0] io_uop_data_bits_u0,
  output        io_acc_rd_0_idx_valid,
  output [10:0] io_acc_rd_0_idx_bits,
  input         io_acc_rd_0_data_valid,
  input  [31:0] io_acc_rd_0_data_bits_0_0,
  input  [31:0] io_acc_rd_0_data_bits_0_1,
  input  [31:0] io_acc_rd_0_data_bits_0_2,
  input  [31:0] io_acc_rd_0_data_bits_0_3,
  input  [31:0] io_acc_rd_0_data_bits_0_4,
  input  [31:0] io_acc_rd_0_data_bits_0_5,
  input  [31:0] io_acc_rd_0_data_bits_0_6,
  input  [31:0] io_acc_rd_0_data_bits_0_7,
  input  [31:0] io_acc_rd_0_data_bits_0_8,
  input  [31:0] io_acc_rd_0_data_bits_0_9,
  input  [31:0] io_acc_rd_0_data_bits_0_10,
  input  [31:0] io_acc_rd_0_data_bits_0_11,
  input  [31:0] io_acc_rd_0_data_bits_0_12,
  input  [31:0] io_acc_rd_0_data_bits_0_13,
  input  [31:0] io_acc_rd_0_data_bits_0_14,
  input  [31:0] io_acc_rd_0_data_bits_0_15,
  output        io_acc_wr_0_valid,
  output [10:0] io_acc_wr_0_bits_idx,
  output [31:0] io_acc_wr_0_bits_data_0_0,
  output [31:0] io_acc_wr_0_bits_data_0_1,
  output [31:0] io_acc_wr_0_bits_data_0_2,
  output [31:0] io_acc_wr_0_bits_data_0_3,
  output [31:0] io_acc_wr_0_bits_data_0_4,
  output [31:0] io_acc_wr_0_bits_data_0_5,
  output [31:0] io_acc_wr_0_bits_data_0_6,
  output [31:0] io_acc_wr_0_bits_data_0_7,
  output [31:0] io_acc_wr_0_bits_data_0_8,
  output [31:0] io_acc_wr_0_bits_data_0_9,
  output [31:0] io_acc_wr_0_bits_data_0_10,
  output [31:0] io_acc_wr_0_bits_data_0_11,
  output [31:0] io_acc_wr_0_bits_data_0_12,
  output [31:0] io_acc_wr_0_bits_data_0_13,
  output [31:0] io_acc_wr_0_bits_data_0_14,
  output [31:0] io_acc_wr_0_bits_data_0_15,
  input         io_out_rd_0_data_valid,
  output        io_out_wr_0_valid,
  output [10:0] io_out_wr_0_bits_idx,
  output [31:0] io_out_wr_0_bits_data_0_0,
  output [31:0] io_out_wr_0_bits_data_0_1,
  output [31:0] io_out_wr_0_bits_data_0_2,
  output [31:0] io_out_wr_0_bits_data_0_3,
  output [31:0] io_out_wr_0_bits_data_0_4,
  output [31:0] io_out_wr_0_bits_data_0_5,
  output [31:0] io_out_wr_0_bits_data_0_6,
  output [31:0] io_out_wr_0_bits_data_0_7,
  output [31:0] io_out_wr_0_bits_data_0_8,
  output [31:0] io_out_wr_0_bits_data_0_9,
  output [31:0] io_out_wr_0_bits_data_0_10,
  output [31:0] io_out_wr_0_bits_data_0_11,
  output [31:0] io_out_wr_0_bits_data_0_12,
  output [31:0] io_out_wr_0_bits_data_0_13,
  output [31:0] io_out_wr_0_bits_data_0_14,
  output [31:0] io_out_wr_0_bits_data_0_15
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
  reg [31:0] _RAND_16;
  reg [31:0] _RAND_17;
  reg [31:0] _RAND_18;
  reg [31:0] _RAND_19;
  reg [31:0] _RAND_20;
  reg [31:0] _RAND_21;
  reg [31:0] _RAND_22;
  reg [31:0] _RAND_23;
  reg [31:0] _RAND_24;
  reg [31:0] _RAND_25;
  reg [31:0] _RAND_26;
  reg [31:0] _RAND_27;
  reg [31:0] _RAND_28;
  reg [31:0] _RAND_29;
  reg [31:0] _RAND_30;
  reg [31:0] _RAND_31;
  reg [31:0] _RAND_32;
  reg [31:0] _RAND_33;
  reg [31:0] _RAND_34;
`endif // RANDOMIZE_REG_INIT
  wire  index_generator_clock; // @[TensorAlu.scala 205:31]
  wire  index_generator_reset; // @[TensorAlu.scala 205:31]
  wire  index_generator_io_start; // @[TensorAlu.scala 205:31]
  wire  index_generator_io_last; // @[TensorAlu.scala 205:31]
  wire  index_generator_io_dec_alu_use_imm; // @[TensorAlu.scala 205:31]
  wire [10:0] index_generator_io_dec_src_1; // @[TensorAlu.scala 205:31]
  wire [10:0] index_generator_io_dec_src_0; // @[TensorAlu.scala 205:31]
  wire [10:0] index_generator_io_dec_dst_1; // @[TensorAlu.scala 205:31]
  wire [10:0] index_generator_io_dec_dst_0; // @[TensorAlu.scala 205:31]
  wire [13:0] index_generator_io_dec_lp_1; // @[TensorAlu.scala 205:31]
  wire [13:0] index_generator_io_dec_lp_0; // @[TensorAlu.scala 205:31]
  wire [13:0] index_generator_io_dec_uop_end; // @[TensorAlu.scala 205:31]
  wire [12:0] index_generator_io_dec_uop_begin; // @[TensorAlu.scala 205:31]
  wire  index_generator_io_valid; // @[TensorAlu.scala 205:31]
  wire  index_generator_io_src_valid; // @[TensorAlu.scala 205:31]
  wire [10:0] index_generator_io_dst_idx; // @[TensorAlu.scala 205:31]
  wire [10:0] index_generator_io_src_idx; // @[TensorAlu.scala 205:31]
  wire [10:0] index_generator_io_uop_idx; // @[TensorAlu.scala 205:31]
  wire  alu_clock; // @[TensorAlu.scala 301:21]
  wire [2:0] alu_io_opcode; // @[TensorAlu.scala 301:21]
  wire  alu_io_acc_a_data_valid; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_acc_a_data_bits_0_0; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_acc_a_data_bits_0_1; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_acc_a_data_bits_0_2; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_acc_a_data_bits_0_3; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_acc_a_data_bits_0_4; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_acc_a_data_bits_0_5; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_acc_a_data_bits_0_6; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_acc_a_data_bits_0_7; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_acc_a_data_bits_0_8; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_acc_a_data_bits_0_9; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_acc_a_data_bits_0_10; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_acc_a_data_bits_0_11; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_acc_a_data_bits_0_12; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_acc_a_data_bits_0_13; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_acc_a_data_bits_0_14; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_acc_a_data_bits_0_15; // @[TensorAlu.scala 301:21]
  wire  alu_io_acc_b_data_valid; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_acc_b_data_bits_0_0; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_acc_b_data_bits_0_1; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_acc_b_data_bits_0_2; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_acc_b_data_bits_0_3; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_acc_b_data_bits_0_4; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_acc_b_data_bits_0_5; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_acc_b_data_bits_0_6; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_acc_b_data_bits_0_7; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_acc_b_data_bits_0_8; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_acc_b_data_bits_0_9; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_acc_b_data_bits_0_10; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_acc_b_data_bits_0_11; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_acc_b_data_bits_0_12; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_acc_b_data_bits_0_13; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_acc_b_data_bits_0_14; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_acc_b_data_bits_0_15; // @[TensorAlu.scala 301:21]
  wire  alu_io_acc_y_data_valid; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_acc_y_data_bits_0_0; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_acc_y_data_bits_0_1; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_acc_y_data_bits_0_2; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_acc_y_data_bits_0_3; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_acc_y_data_bits_0_4; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_acc_y_data_bits_0_5; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_acc_y_data_bits_0_6; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_acc_y_data_bits_0_7; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_acc_y_data_bits_0_8; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_acc_y_data_bits_0_9; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_acc_y_data_bits_0_10; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_acc_y_data_bits_0_11; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_acc_y_data_bits_0_12; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_acc_y_data_bits_0_13; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_acc_y_data_bits_0_14; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_acc_y_data_bits_0_15; // @[TensorAlu.scala 301:21]
  wire  alu_io_out_data_valid; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_out_data_bits_0_0; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_out_data_bits_0_1; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_out_data_bits_0_2; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_out_data_bits_0_3; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_out_data_bits_0_4; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_out_data_bits_0_5; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_out_data_bits_0_6; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_out_data_bits_0_7; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_out_data_bits_0_8; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_out_data_bits_0_9; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_out_data_bits_0_10; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_out_data_bits_0_11; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_out_data_bits_0_12; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_out_data_bits_0_13; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_out_data_bits_0_14; // @[TensorAlu.scala 301:21]
  wire [31:0] alu_io_out_data_bits_0_15; // @[TensorAlu.scala 301:21]
  reg [1:0] state; // @[TensorAlu.scala 202:22]
  reg [3:0] inflight; // @[TensorAlu.scala 203:25]
  wire  _T = state == 2'h0; // @[TensorAlu.scala 210:14]
  wire  _T_5 = inflight == 4'h0; // @[TensorAlu.scala 214:42]
  wire  _T_6 = state == 2'h2 & inflight == 4'h0; // @[TensorAlu.scala 214:30]
  wire  _GEN_3 = state == 2'h1 & index_generator_io_last ? 1'h0 : _T_6; // @[TensorAlu.scala 212:57 TensorAlu.scala 209:11]
  reg  valid_r1; // @[Reg.scala 27:20]
  wire  _GEN_6 = index_generator_io_valid; // @[Reg.scala 28:19 Reg.scala 28:23 Reg.scala 27:20]
  reg  valid_r2; // @[TensorAlu.scala 227:25]
  reg  valid_r3; // @[TensorAlu.scala 228:25]
  reg  valid_r4; // @[TensorAlu.scala 229:25]
  wire  _T_7 = index_generator_io_valid & valid_r4; // @[TensorAlu.scala 231:33]
  wire [3:0] _inflight_T_1 = inflight + 4'h1; // @[TensorAlu.scala 234:26]
  wire [3:0] _inflight_T_3 = inflight - 4'h1; // @[TensorAlu.scala 237:26]
  wire [3:0] _GEN_7 = valid_r4 ? _inflight_T_3 : inflight; // @[TensorAlu.scala 235:24 TensorAlu.scala 237:14 TensorAlu.scala 203:25]
  reg  src_valid_r1; // @[Reg.scala 27:20]
  wire  _GEN_11 = index_generator_io_src_valid; // @[Reg.scala 28:19 Reg.scala 28:23 Reg.scala 27:20]
  reg  src_valid_r2; // @[TensorAlu.scala 248:29]
  reg  src_valid_r3; // @[TensorAlu.scala 249:29]
  reg [10:0] dst_idx_r1; // @[Reg.scala 15:16]
  reg [10:0] src_idx_r1; // @[Reg.scala 15:16]
  wire [10:0] u2 = {{1'd0}, io_uop_data_bits_u2}; // @[TensorAlu.scala 260:40 TensorAlu.scala 260:40]
  wire [21:0] _src_offset_T = {u2, 11'h0}; // @[TensorAlu.scala 263:24]
  wire [21:0] _GEN_14 = {{11'd0}, io_uop_data_bits_u1}; // @[TensorAlu.scala 263:30]
  wire [21:0] src_offset = _src_offset_T | _GEN_14; // @[TensorAlu.scala 263:30]
  reg  io_acc_rd_0_idx_valid_REG; // @[TensorAlu.scala 268:40]
  wire [21:0] _GEN_15 = {{11'd0}, src_idx_r1}; // @[TensorAlu.scala 271:35]
  wire [21:0] new_src_idx_r1 = _GEN_15 + src_offset; // @[TensorAlu.scala 271:35]
  reg [21:0] src_idx_r2; // @[TensorAlu.scala 272:27]
  reg [21:0] src_idx_r3; // @[TensorAlu.scala 273:27]
  wire [10:0] new_dst_idx_r1 = dst_idx_r1 + io_uop_data_bits_u0; // @[TensorAlu.scala 275:35]
  reg [10:0] dst_idx_r2; // @[TensorAlu.scala 276:27]
  reg [10:0] dst_idx_r3; // @[TensorAlu.scala 277:27]
  reg [10:0] dst_idx_r4; // @[TensorAlu.scala 278:27]
  reg [21:0] io_acc_rd_0_idx_bits_REG; // @[TensorAlu.scala 283:39]
  reg [31:0] save_src_0_0; // @[TensorAlu.scala 311:27]
  reg [31:0] save_src_0_1; // @[TensorAlu.scala 311:27]
  reg [31:0] save_src_0_2; // @[TensorAlu.scala 311:27]
  reg [31:0] save_src_0_3; // @[TensorAlu.scala 311:27]
  reg [31:0] save_src_0_4; // @[TensorAlu.scala 311:27]
  reg [31:0] save_src_0_5; // @[TensorAlu.scala 311:27]
  reg [31:0] save_src_0_6; // @[TensorAlu.scala 311:27]
  reg [31:0] save_src_0_7; // @[TensorAlu.scala 311:27]
  reg [31:0] save_src_0_8; // @[TensorAlu.scala 311:27]
  reg [31:0] save_src_0_9; // @[TensorAlu.scala 311:27]
  reg [31:0] save_src_0_10; // @[TensorAlu.scala 311:27]
  reg [31:0] save_src_0_11; // @[TensorAlu.scala 311:27]
  reg [31:0] save_src_0_12; // @[TensorAlu.scala 311:27]
  reg [31:0] save_src_0_13; // @[TensorAlu.scala 311:27]
  reg [31:0] save_src_0_14; // @[TensorAlu.scala 311:27]
  reg [31:0] save_src_0_15; // @[TensorAlu.scala 311:27]
  wire [31:0] _tensorImm_data_bits_0_0_T_1 = {16'hffff,io_dec_alu_imm}; // @[Cat.scala 30:58]
  wire [31:0] tensorImm_data_bits_0_0 = io_dec_alu_imm[15] ? _tensorImm_data_bits_0_0_T_1 : {{16'd0}, io_dec_alu_imm}; // @[TensorAlu.scala 319:17]
  wire  isSHR = io_dec_alu_op == 3'h3; // @[TensorAlu.scala 328:37]
  wire  neg_shift = isSHR & io_dec_alu_imm[15]; // @[TensorAlu.scala 329:27]
  reg  alu_io_acc_a_data_valid_REG; // @[TensorAlu.scala 338:39]
  wire  bypass_dst = valid_r3 & valid_r4 & dst_idx_r4 == dst_idx_r3; // @[TensorAlu.scala 386:41]
  wire [21:0] _GEN_16 = {{11'd0}, dst_idx_r4}; // @[TensorAlu.scala 387:60]
  wire  bypass_src = src_valid_r3 & valid_r4 & _GEN_16 == src_idx_r3; // @[TensorAlu.scala 387:45]
  wire  _GEN_17 = ~_T_7; // @[TensorAlu.scala 233:11]
  TensorAluIndexGenerator index_generator ( // @[TensorAlu.scala 205:31]
    .clock(index_generator_clock),
    .reset(index_generator_reset),
    .io_start(index_generator_io_start),
    .io_last(index_generator_io_last),
    .io_dec_alu_use_imm(index_generator_io_dec_alu_use_imm),
    .io_dec_src_1(index_generator_io_dec_src_1),
    .io_dec_src_0(index_generator_io_dec_src_0),
    .io_dec_dst_1(index_generator_io_dec_dst_1),
    .io_dec_dst_0(index_generator_io_dec_dst_0),
    .io_dec_lp_1(index_generator_io_dec_lp_1),
    .io_dec_lp_0(index_generator_io_dec_lp_0),
    .io_dec_uop_end(index_generator_io_dec_uop_end),
    .io_dec_uop_begin(index_generator_io_dec_uop_begin),
    .io_valid(index_generator_io_valid),
    .io_src_valid(index_generator_io_src_valid),
    .io_dst_idx(index_generator_io_dst_idx),
    .io_src_idx(index_generator_io_src_idx),
    .io_uop_idx(index_generator_io_uop_idx)
  );
  AluVector alu ( // @[TensorAlu.scala 301:21]
    .clock(alu_clock),
    .io_opcode(alu_io_opcode),
    .io_acc_a_data_valid(alu_io_acc_a_data_valid),
    .io_acc_a_data_bits_0_0(alu_io_acc_a_data_bits_0_0),
    .io_acc_a_data_bits_0_1(alu_io_acc_a_data_bits_0_1),
    .io_acc_a_data_bits_0_2(alu_io_acc_a_data_bits_0_2),
    .io_acc_a_data_bits_0_3(alu_io_acc_a_data_bits_0_3),
    .io_acc_a_data_bits_0_4(alu_io_acc_a_data_bits_0_4),
    .io_acc_a_data_bits_0_5(alu_io_acc_a_data_bits_0_5),
    .io_acc_a_data_bits_0_6(alu_io_acc_a_data_bits_0_6),
    .io_acc_a_data_bits_0_7(alu_io_acc_a_data_bits_0_7),
    .io_acc_a_data_bits_0_8(alu_io_acc_a_data_bits_0_8),
    .io_acc_a_data_bits_0_9(alu_io_acc_a_data_bits_0_9),
    .io_acc_a_data_bits_0_10(alu_io_acc_a_data_bits_0_10),
    .io_acc_a_data_bits_0_11(alu_io_acc_a_data_bits_0_11),
    .io_acc_a_data_bits_0_12(alu_io_acc_a_data_bits_0_12),
    .io_acc_a_data_bits_0_13(alu_io_acc_a_data_bits_0_13),
    .io_acc_a_data_bits_0_14(alu_io_acc_a_data_bits_0_14),
    .io_acc_a_data_bits_0_15(alu_io_acc_a_data_bits_0_15),
    .io_acc_b_data_valid(alu_io_acc_b_data_valid),
    .io_acc_b_data_bits_0_0(alu_io_acc_b_data_bits_0_0),
    .io_acc_b_data_bits_0_1(alu_io_acc_b_data_bits_0_1),
    .io_acc_b_data_bits_0_2(alu_io_acc_b_data_bits_0_2),
    .io_acc_b_data_bits_0_3(alu_io_acc_b_data_bits_0_3),
    .io_acc_b_data_bits_0_4(alu_io_acc_b_data_bits_0_4),
    .io_acc_b_data_bits_0_5(alu_io_acc_b_data_bits_0_5),
    .io_acc_b_data_bits_0_6(alu_io_acc_b_data_bits_0_6),
    .io_acc_b_data_bits_0_7(alu_io_acc_b_data_bits_0_7),
    .io_acc_b_data_bits_0_8(alu_io_acc_b_data_bits_0_8),
    .io_acc_b_data_bits_0_9(alu_io_acc_b_data_bits_0_9),
    .io_acc_b_data_bits_0_10(alu_io_acc_b_data_bits_0_10),
    .io_acc_b_data_bits_0_11(alu_io_acc_b_data_bits_0_11),
    .io_acc_b_data_bits_0_12(alu_io_acc_b_data_bits_0_12),
    .io_acc_b_data_bits_0_13(alu_io_acc_b_data_bits_0_13),
    .io_acc_b_data_bits_0_14(alu_io_acc_b_data_bits_0_14),
    .io_acc_b_data_bits_0_15(alu_io_acc_b_data_bits_0_15),
    .io_acc_y_data_valid(alu_io_acc_y_data_valid),
    .io_acc_y_data_bits_0_0(alu_io_acc_y_data_bits_0_0),
    .io_acc_y_data_bits_0_1(alu_io_acc_y_data_bits_0_1),
    .io_acc_y_data_bits_0_2(alu_io_acc_y_data_bits_0_2),
    .io_acc_y_data_bits_0_3(alu_io_acc_y_data_bits_0_3),
    .io_acc_y_data_bits_0_4(alu_io_acc_y_data_bits_0_4),
    .io_acc_y_data_bits_0_5(alu_io_acc_y_data_bits_0_5),
    .io_acc_y_data_bits_0_6(alu_io_acc_y_data_bits_0_6),
    .io_acc_y_data_bits_0_7(alu_io_acc_y_data_bits_0_7),
    .io_acc_y_data_bits_0_8(alu_io_acc_y_data_bits_0_8),
    .io_acc_y_data_bits_0_9(alu_io_acc_y_data_bits_0_9),
    .io_acc_y_data_bits_0_10(alu_io_acc_y_data_bits_0_10),
    .io_acc_y_data_bits_0_11(alu_io_acc_y_data_bits_0_11),
    .io_acc_y_data_bits_0_12(alu_io_acc_y_data_bits_0_12),
    .io_acc_y_data_bits_0_13(alu_io_acc_y_data_bits_0_13),
    .io_acc_y_data_bits_0_14(alu_io_acc_y_data_bits_0_14),
    .io_acc_y_data_bits_0_15(alu_io_acc_y_data_bits_0_15),
    .io_out_data_valid(alu_io_out_data_valid),
    .io_out_data_bits_0_0(alu_io_out_data_bits_0_0),
    .io_out_data_bits_0_1(alu_io_out_data_bits_0_1),
    .io_out_data_bits_0_2(alu_io_out_data_bits_0_2),
    .io_out_data_bits_0_3(alu_io_out_data_bits_0_3),
    .io_out_data_bits_0_4(alu_io_out_data_bits_0_4),
    .io_out_data_bits_0_5(alu_io_out_data_bits_0_5),
    .io_out_data_bits_0_6(alu_io_out_data_bits_0_6),
    .io_out_data_bits_0_7(alu_io_out_data_bits_0_7),
    .io_out_data_bits_0_8(alu_io_out_data_bits_0_8),
    .io_out_data_bits_0_9(alu_io_out_data_bits_0_9),
    .io_out_data_bits_0_10(alu_io_out_data_bits_0_10),
    .io_out_data_bits_0_11(alu_io_out_data_bits_0_11),
    .io_out_data_bits_0_12(alu_io_out_data_bits_0_12),
    .io_out_data_bits_0_13(alu_io_out_data_bits_0_13),
    .io_out_data_bits_0_14(alu_io_out_data_bits_0_14),
    .io_out_data_bits_0_15(alu_io_out_data_bits_0_15)
  );
  assign io_done = state == 2'h0 & io_start ? 1'h0 : _GEN_3; // @[TensorAlu.scala 210:37 TensorAlu.scala 209:11]
  assign io_uop_idx_valid = index_generator_io_valid | index_generator_io_src_valid; // @[TensorAlu.scala 223:48]
  assign io_uop_idx_bits = index_generator_io_uop_idx; // @[TensorAlu.scala 224:19]
  assign io_acc_rd_0_idx_valid = io_acc_rd_0_idx_valid_REG; // @[TensorAlu.scala 268:30]
  assign io_acc_rd_0_idx_bits = io_acc_rd_0_idx_bits_REG[10:0]; // @[TensorAlu.scala 283:29]
  assign io_acc_wr_0_valid = valid_r4; // @[TensorAlu.scala 360:26]
  assign io_acc_wr_0_bits_idx = dst_idx_r4; // @[TensorAlu.scala 361:29]
  assign io_acc_wr_0_bits_data_0_0 = alu_io_acc_y_data_bits_0_0; // @[TensorAlu.scala 367:62]
  assign io_acc_wr_0_bits_data_0_1 = alu_io_acc_y_data_bits_0_1; // @[TensorAlu.scala 367:62]
  assign io_acc_wr_0_bits_data_0_2 = alu_io_acc_y_data_bits_0_2; // @[TensorAlu.scala 367:62]
  assign io_acc_wr_0_bits_data_0_3 = alu_io_acc_y_data_bits_0_3; // @[TensorAlu.scala 367:62]
  assign io_acc_wr_0_bits_data_0_4 = alu_io_acc_y_data_bits_0_4; // @[TensorAlu.scala 367:62]
  assign io_acc_wr_0_bits_data_0_5 = alu_io_acc_y_data_bits_0_5; // @[TensorAlu.scala 367:62]
  assign io_acc_wr_0_bits_data_0_6 = alu_io_acc_y_data_bits_0_6; // @[TensorAlu.scala 367:62]
  assign io_acc_wr_0_bits_data_0_7 = alu_io_acc_y_data_bits_0_7; // @[TensorAlu.scala 367:62]
  assign io_acc_wr_0_bits_data_0_8 = alu_io_acc_y_data_bits_0_8; // @[TensorAlu.scala 367:62]
  assign io_acc_wr_0_bits_data_0_9 = alu_io_acc_y_data_bits_0_9; // @[TensorAlu.scala 367:62]
  assign io_acc_wr_0_bits_data_0_10 = alu_io_acc_y_data_bits_0_10; // @[TensorAlu.scala 367:62]
  assign io_acc_wr_0_bits_data_0_11 = alu_io_acc_y_data_bits_0_11; // @[TensorAlu.scala 367:62]
  assign io_acc_wr_0_bits_data_0_12 = alu_io_acc_y_data_bits_0_12; // @[TensorAlu.scala 367:62]
  assign io_acc_wr_0_bits_data_0_13 = alu_io_acc_y_data_bits_0_13; // @[TensorAlu.scala 367:62]
  assign io_acc_wr_0_bits_data_0_14 = alu_io_acc_y_data_bits_0_14; // @[TensorAlu.scala 367:62]
  assign io_acc_wr_0_bits_data_0_15 = alu_io_acc_y_data_bits_0_15; // @[TensorAlu.scala 367:62]
  assign io_out_wr_0_valid = valid_r4; // @[TensorAlu.scala 381:22]
  assign io_out_wr_0_bits_idx = dst_idx_r4; // @[TensorAlu.scala 382:25]
  assign io_out_wr_0_bits_data_0_0 = alu_io_out_data_bits_0_0; // @[TensorAlu.scala 289:21 TensorAlu.scala 375:66]
  assign io_out_wr_0_bits_data_0_1 = alu_io_out_data_bits_0_1; // @[TensorAlu.scala 289:21 TensorAlu.scala 375:66]
  assign io_out_wr_0_bits_data_0_2 = alu_io_out_data_bits_0_2; // @[TensorAlu.scala 289:21 TensorAlu.scala 375:66]
  assign io_out_wr_0_bits_data_0_3 = alu_io_out_data_bits_0_3; // @[TensorAlu.scala 289:21 TensorAlu.scala 375:66]
  assign io_out_wr_0_bits_data_0_4 = alu_io_out_data_bits_0_4; // @[TensorAlu.scala 289:21 TensorAlu.scala 375:66]
  assign io_out_wr_0_bits_data_0_5 = alu_io_out_data_bits_0_5; // @[TensorAlu.scala 289:21 TensorAlu.scala 375:66]
  assign io_out_wr_0_bits_data_0_6 = alu_io_out_data_bits_0_6; // @[TensorAlu.scala 289:21 TensorAlu.scala 375:66]
  assign io_out_wr_0_bits_data_0_7 = alu_io_out_data_bits_0_7; // @[TensorAlu.scala 289:21 TensorAlu.scala 375:66]
  assign io_out_wr_0_bits_data_0_8 = alu_io_out_data_bits_0_8; // @[TensorAlu.scala 289:21 TensorAlu.scala 375:66]
  assign io_out_wr_0_bits_data_0_9 = alu_io_out_data_bits_0_9; // @[TensorAlu.scala 289:21 TensorAlu.scala 375:66]
  assign io_out_wr_0_bits_data_0_10 = alu_io_out_data_bits_0_10; // @[TensorAlu.scala 289:21 TensorAlu.scala 375:66]
  assign io_out_wr_0_bits_data_0_11 = alu_io_out_data_bits_0_11; // @[TensorAlu.scala 289:21 TensorAlu.scala 375:66]
  assign io_out_wr_0_bits_data_0_12 = alu_io_out_data_bits_0_12; // @[TensorAlu.scala 289:21 TensorAlu.scala 375:66]
  assign io_out_wr_0_bits_data_0_13 = alu_io_out_data_bits_0_13; // @[TensorAlu.scala 289:21 TensorAlu.scala 375:66]
  assign io_out_wr_0_bits_data_0_14 = alu_io_out_data_bits_0_14; // @[TensorAlu.scala 289:21 TensorAlu.scala 375:66]
  assign io_out_wr_0_bits_data_0_15 = alu_io_out_data_bits_0_15; // @[TensorAlu.scala 289:21 TensorAlu.scala 375:66]
  assign index_generator_clock = clock;
  assign index_generator_reset = reset;
  assign index_generator_io_start = io_start; // @[TensorAlu.scala 219:28]
  assign index_generator_io_dec_alu_use_imm = io_dec_alu_use_imm; // @[TensorAlu.scala 220:26]
  assign index_generator_io_dec_src_1 = io_dec_src_1; // @[TensorAlu.scala 220:26]
  assign index_generator_io_dec_src_0 = io_dec_src_0; // @[TensorAlu.scala 220:26]
  assign index_generator_io_dec_dst_1 = io_dec_dst_1; // @[TensorAlu.scala 220:26]
  assign index_generator_io_dec_dst_0 = io_dec_dst_0; // @[TensorAlu.scala 220:26]
  assign index_generator_io_dec_lp_1 = io_dec_lp_1; // @[TensorAlu.scala 220:26]
  assign index_generator_io_dec_lp_0 = io_dec_lp_0; // @[TensorAlu.scala 220:26]
  assign index_generator_io_dec_uop_end = io_dec_uop_end; // @[TensorAlu.scala 220:26]
  assign index_generator_io_dec_uop_begin = io_dec_uop_begin; // @[TensorAlu.scala 220:26]
  assign alu_clock = clock;
  assign alu_io_opcode = neg_shift ? 3'h4 : io_dec_alu_op; // @[TensorAlu.scala 330:27]
  assign alu_io_acc_a_data_valid = alu_io_acc_a_data_valid_REG; // @[TensorAlu.scala 338:29]
  assign alu_io_acc_a_data_bits_0_0 = io_acc_rd_0_data_bits_0_0; // @[TensorAlu.scala 291:24 TensorAlu.scala 344:47]
  assign alu_io_acc_a_data_bits_0_1 = io_acc_rd_0_data_bits_0_1; // @[TensorAlu.scala 291:24 TensorAlu.scala 344:47]
  assign alu_io_acc_a_data_bits_0_2 = io_acc_rd_0_data_bits_0_2; // @[TensorAlu.scala 291:24 TensorAlu.scala 344:47]
  assign alu_io_acc_a_data_bits_0_3 = io_acc_rd_0_data_bits_0_3; // @[TensorAlu.scala 291:24 TensorAlu.scala 344:47]
  assign alu_io_acc_a_data_bits_0_4 = io_acc_rd_0_data_bits_0_4; // @[TensorAlu.scala 291:24 TensorAlu.scala 344:47]
  assign alu_io_acc_a_data_bits_0_5 = io_acc_rd_0_data_bits_0_5; // @[TensorAlu.scala 291:24 TensorAlu.scala 344:47]
  assign alu_io_acc_a_data_bits_0_6 = io_acc_rd_0_data_bits_0_6; // @[TensorAlu.scala 291:24 TensorAlu.scala 344:47]
  assign alu_io_acc_a_data_bits_0_7 = io_acc_rd_0_data_bits_0_7; // @[TensorAlu.scala 291:24 TensorAlu.scala 344:47]
  assign alu_io_acc_a_data_bits_0_8 = io_acc_rd_0_data_bits_0_8; // @[TensorAlu.scala 291:24 TensorAlu.scala 344:47]
  assign alu_io_acc_a_data_bits_0_9 = io_acc_rd_0_data_bits_0_9; // @[TensorAlu.scala 291:24 TensorAlu.scala 344:47]
  assign alu_io_acc_a_data_bits_0_10 = io_acc_rd_0_data_bits_0_10; // @[TensorAlu.scala 291:24 TensorAlu.scala 344:47]
  assign alu_io_acc_a_data_bits_0_11 = io_acc_rd_0_data_bits_0_11; // @[TensorAlu.scala 291:24 TensorAlu.scala 344:47]
  assign alu_io_acc_a_data_bits_0_12 = io_acc_rd_0_data_bits_0_12; // @[TensorAlu.scala 291:24 TensorAlu.scala 344:47]
  assign alu_io_acc_a_data_bits_0_13 = io_acc_rd_0_data_bits_0_13; // @[TensorAlu.scala 291:24 TensorAlu.scala 344:47]
  assign alu_io_acc_a_data_bits_0_14 = io_acc_rd_0_data_bits_0_14; // @[TensorAlu.scala 291:24 TensorAlu.scala 344:47]
  assign alu_io_acc_a_data_bits_0_15 = io_acc_rd_0_data_bits_0_15; // @[TensorAlu.scala 291:24 TensorAlu.scala 344:47]
  assign alu_io_acc_b_data_valid = valid_r3; // @[TensorAlu.scala 352:35]
  assign alu_io_acc_b_data_bits_0_0 = io_dec_alu_use_imm ? tensorImm_data_bits_0_0 : save_src_0_0; // @[TensorAlu.scala 355:34]
  assign alu_io_acc_b_data_bits_0_1 = io_dec_alu_use_imm ? tensorImm_data_bits_0_0 : save_src_0_1; // @[TensorAlu.scala 355:34]
  assign alu_io_acc_b_data_bits_0_2 = io_dec_alu_use_imm ? tensorImm_data_bits_0_0 : save_src_0_2; // @[TensorAlu.scala 355:34]
  assign alu_io_acc_b_data_bits_0_3 = io_dec_alu_use_imm ? tensorImm_data_bits_0_0 : save_src_0_3; // @[TensorAlu.scala 355:34]
  assign alu_io_acc_b_data_bits_0_4 = io_dec_alu_use_imm ? tensorImm_data_bits_0_0 : save_src_0_4; // @[TensorAlu.scala 355:34]
  assign alu_io_acc_b_data_bits_0_5 = io_dec_alu_use_imm ? tensorImm_data_bits_0_0 : save_src_0_5; // @[TensorAlu.scala 355:34]
  assign alu_io_acc_b_data_bits_0_6 = io_dec_alu_use_imm ? tensorImm_data_bits_0_0 : save_src_0_6; // @[TensorAlu.scala 355:34]
  assign alu_io_acc_b_data_bits_0_7 = io_dec_alu_use_imm ? tensorImm_data_bits_0_0 : save_src_0_7; // @[TensorAlu.scala 355:34]
  assign alu_io_acc_b_data_bits_0_8 = io_dec_alu_use_imm ? tensorImm_data_bits_0_0 : save_src_0_8; // @[TensorAlu.scala 355:34]
  assign alu_io_acc_b_data_bits_0_9 = io_dec_alu_use_imm ? tensorImm_data_bits_0_0 : save_src_0_9; // @[TensorAlu.scala 355:34]
  assign alu_io_acc_b_data_bits_0_10 = io_dec_alu_use_imm ? tensorImm_data_bits_0_0 : save_src_0_10; // @[TensorAlu.scala 355:34]
  assign alu_io_acc_b_data_bits_0_11 = io_dec_alu_use_imm ? tensorImm_data_bits_0_0 : save_src_0_11; // @[TensorAlu.scala 355:34]
  assign alu_io_acc_b_data_bits_0_12 = io_dec_alu_use_imm ? tensorImm_data_bits_0_0 : save_src_0_12; // @[TensorAlu.scala 355:34]
  assign alu_io_acc_b_data_bits_0_13 = io_dec_alu_use_imm ? tensorImm_data_bits_0_0 : save_src_0_13; // @[TensorAlu.scala 355:34]
  assign alu_io_acc_b_data_bits_0_14 = io_dec_alu_use_imm ? tensorImm_data_bits_0_0 : save_src_0_14; // @[TensorAlu.scala 355:34]
  assign alu_io_acc_b_data_bits_0_15 = io_dec_alu_use_imm ? tensorImm_data_bits_0_0 : save_src_0_15; // @[TensorAlu.scala 355:34]
  always @(posedge clock) begin
    if (reset) begin // @[TensorAlu.scala 202:22]
      state <= 2'h0; // @[TensorAlu.scala 202:22]
    end else if (state == 2'h0 & io_start) begin // @[TensorAlu.scala 210:37]
      state <= 2'h1; // @[TensorAlu.scala 211:11]
    end else if (state == 2'h1 & index_generator_io_last) begin // @[TensorAlu.scala 212:57]
      state <= 2'h2; // @[TensorAlu.scala 213:11]
    end else if (state == 2'h2 & inflight == 4'h0) begin // @[TensorAlu.scala 214:51]
      state <= 2'h0; // @[TensorAlu.scala 215:11]
    end
    if (reset) begin // @[TensorAlu.scala 203:25]
      inflight <= 4'h0; // @[TensorAlu.scala 203:25]
    end else if (_T) begin // @[TensorAlu.scala 239:25]
      inflight <= 4'h0; // @[TensorAlu.scala 241:14]
    end else if (!(index_generator_io_valid & valid_r4)) begin // @[TensorAlu.scala 231:46]
      if (index_generator_io_valid) begin // @[TensorAlu.scala 232:40]
        inflight <= _inflight_T_1; // @[TensorAlu.scala 234:14]
      end else begin
        inflight <= _GEN_7;
      end
    end
    if (reset) begin // @[Reg.scala 27:20]
      valid_r1 <= 1'h0; // @[Reg.scala 27:20]
    end else begin
      valid_r1 <= _GEN_6;
    end
    if (reset) begin // @[TensorAlu.scala 227:25]
      valid_r2 <= 1'h0; // @[TensorAlu.scala 227:25]
    end else begin
      valid_r2 <= valid_r1; // @[TensorAlu.scala 227:25]
    end
    if (reset) begin // @[TensorAlu.scala 228:25]
      valid_r3 <= 1'h0; // @[TensorAlu.scala 228:25]
    end else begin
      valid_r3 <= valid_r2; // @[TensorAlu.scala 228:25]
    end
    if (reset) begin // @[TensorAlu.scala 229:25]
      valid_r4 <= 1'h0; // @[TensorAlu.scala 229:25]
    end else begin
      valid_r4 <= valid_r3; // @[TensorAlu.scala 229:25]
    end
    if (reset) begin // @[Reg.scala 27:20]
      src_valid_r1 <= 1'h0; // @[Reg.scala 27:20]
    end else begin
      src_valid_r1 <= _GEN_11;
    end
    if (reset) begin // @[TensorAlu.scala 248:29]
      src_valid_r2 <= 1'h0; // @[TensorAlu.scala 248:29]
    end else begin
      src_valid_r2 <= src_valid_r1; // @[TensorAlu.scala 248:29]
    end
    if (reset) begin // @[TensorAlu.scala 249:29]
      src_valid_r3 <= 1'h0; // @[TensorAlu.scala 249:29]
    end else begin
      src_valid_r3 <= src_valid_r2; // @[TensorAlu.scala 249:29]
    end
    dst_idx_r1 <= index_generator_io_dst_idx; // @[Reg.scala 16:19 Reg.scala 16:23 Reg.scala 15:16]
    src_idx_r1 <= index_generator_io_src_idx; // @[Reg.scala 16:19 Reg.scala 16:23 Reg.scala 15:16]
    io_acc_rd_0_idx_valid_REG <= valid_r1 | src_valid_r1; // @[TensorAlu.scala 266:32]
    src_idx_r2 <= _GEN_15 + src_offset; // @[TensorAlu.scala 271:35]
    src_idx_r3 <= src_idx_r2; // @[TensorAlu.scala 273:27]
    dst_idx_r2 <= dst_idx_r1 + io_uop_data_bits_u0; // @[TensorAlu.scala 275:35]
    dst_idx_r3 <= dst_idx_r2; // @[TensorAlu.scala 277:27]
    dst_idx_r4 <= dst_idx_r3; // @[TensorAlu.scala 278:27]
    if (src_valid_r1 | io_dec_alu_use_imm) begin // @[TensorAlu.scala 281:25]
      io_acc_rd_0_idx_bits_REG <= new_src_idx_r1;
    end else begin
      io_acc_rd_0_idx_bits_REG <= {{11'd0}, new_dst_idx_r1};
    end
    save_src_0_0 <= io_acc_rd_0_data_bits_0_0; // @[TensorAlu.scala 290:24 TensorAlu.scala 307:47]
    save_src_0_1 <= io_acc_rd_0_data_bits_0_1; // @[TensorAlu.scala 290:24 TensorAlu.scala 307:47]
    save_src_0_2 <= io_acc_rd_0_data_bits_0_2; // @[TensorAlu.scala 290:24 TensorAlu.scala 307:47]
    save_src_0_3 <= io_acc_rd_0_data_bits_0_3; // @[TensorAlu.scala 290:24 TensorAlu.scala 307:47]
    save_src_0_4 <= io_acc_rd_0_data_bits_0_4; // @[TensorAlu.scala 290:24 TensorAlu.scala 307:47]
    save_src_0_5 <= io_acc_rd_0_data_bits_0_5; // @[TensorAlu.scala 290:24 TensorAlu.scala 307:47]
    save_src_0_6 <= io_acc_rd_0_data_bits_0_6; // @[TensorAlu.scala 290:24 TensorAlu.scala 307:47]
    save_src_0_7 <= io_acc_rd_0_data_bits_0_7; // @[TensorAlu.scala 290:24 TensorAlu.scala 307:47]
    save_src_0_8 <= io_acc_rd_0_data_bits_0_8; // @[TensorAlu.scala 290:24 TensorAlu.scala 307:47]
    save_src_0_9 <= io_acc_rd_0_data_bits_0_9; // @[TensorAlu.scala 290:24 TensorAlu.scala 307:47]
    save_src_0_10 <= io_acc_rd_0_data_bits_0_10; // @[TensorAlu.scala 290:24 TensorAlu.scala 307:47]
    save_src_0_11 <= io_acc_rd_0_data_bits_0_11; // @[TensorAlu.scala 290:24 TensorAlu.scala 307:47]
    save_src_0_12 <= io_acc_rd_0_data_bits_0_12; // @[TensorAlu.scala 290:24 TensorAlu.scala 307:47]
    save_src_0_13 <= io_acc_rd_0_data_bits_0_13; // @[TensorAlu.scala 290:24 TensorAlu.scala 307:47]
    save_src_0_14 <= io_acc_rd_0_data_bits_0_14; // @[TensorAlu.scala 290:24 TensorAlu.scala 307:47]
    save_src_0_15 <= io_acc_rd_0_data_bits_0_15; // @[TensorAlu.scala 290:24 TensorAlu.scala 307:47]
    alu_io_acc_a_data_valid_REG <= valid_r2; // @[TensorAlu.scala 338:39]
    `ifndef SYNTHESIS
    `ifdef PRINTF_COND
      if (`PRINTF_COND) begin
    `endif
        if (~_T_7 & index_generator_io_valid & ~(inflight != 4'hf | reset)) begin
          $fwrite(32'h80000002,
            "Assertion failed\n    at TensorAlu.scala:233 assert(inflight =/= ((1<<inflightBits)-1).U)\n"); // @[TensorAlu.scala 233:11]
        end
    `ifdef PRINTF_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef STOP_COND
      if (`STOP_COND) begin
    `endif
        if (~_T_7 & index_generator_io_valid & ~(inflight != 4'hf | reset)) begin
          $fatal; // @[TensorAlu.scala 233:11]
        end
    `ifdef STOP_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef PRINTF_COND
      if (`PRINTF_COND) begin
    `endif
        if (_GEN_17 & ~index_generator_io_valid & valid_r4 & ~(inflight != 4'h0 | reset)) begin
          $fwrite(32'h80000002,"Assertion failed\n    at TensorAlu.scala:236 assert(inflight =/= 0.U)\n"); // @[TensorAlu.scala 236:11]
        end
    `ifdef PRINTF_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef STOP_COND
      if (`STOP_COND) begin
    `endif
        if (_GEN_17 & ~index_generator_io_valid & valid_r4 & ~(inflight != 4'h0 | reset)) begin
          $fatal; // @[TensorAlu.scala 236:11]
        end
    `ifdef STOP_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef PRINTF_COND
      if (`PRINTF_COND) begin
    `endif
        if (_T & ~(_T_5 | reset)) begin
          $fwrite(32'h80000002,"Assertion failed\n    at TensorAlu.scala:240 assert(inflight === 0.U)\n"); // @[TensorAlu.scala 240:11]
        end
    `ifdef PRINTF_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef STOP_COND
      if (`STOP_COND) begin
    `endif
        if (_T & ~(_T_5 | reset)) begin
          $fatal; // @[TensorAlu.scala 240:11]
        end
    `ifdef STOP_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef PRINTF_COND
      if (`PRINTF_COND) begin
    `endif
        if (~(io_acc_rd_0_data_valid == (valid_r3 | src_valid_r3) | reset)) begin
          $fwrite(32'h80000002,
            "Assertion failed\n    at TensorAlu.scala:284 assert(io.acc.rd(idx).data.valid === (valid_r3 || src_valid_r3))\n"
            ); // @[TensorAlu.scala 284:11]
        end
    `ifdef PRINTF_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef STOP_COND
      if (`STOP_COND) begin
    `endif
        if (~(io_acc_rd_0_data_valid == (valid_r3 | src_valid_r3) | reset)) begin
          $fatal; // @[TensorAlu.scala 284:11]
        end
    `ifdef STOP_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef PRINTF_COND
      if (`PRINTF_COND) begin
    `endif
        if (~(~valid_r3 | io_acc_rd_0_data_valid | reset)) begin
          $fwrite(32'h80000002,
            "Assertion failed\n    at TensorAlu.scala:336 assert(!valid_r3 || io.acc.rd(idx).data.valid)\n"); // @[TensorAlu.scala 336:11]
        end
    `ifdef PRINTF_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef STOP_COND
      if (`STOP_COND) begin
    `endif
        if (~(~valid_r3 | io_acc_rd_0_data_valid | reset)) begin
          $fatal; // @[TensorAlu.scala 336:11]
        end
    `ifdef STOP_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef PRINTF_COND
      if (`PRINTF_COND) begin
    `endif
        if (~(alu_io_acc_y_data_valid == valid_r4 | reset)) begin
          $fwrite(32'h80000002,
            "Assertion failed\n    at TensorAlu.scala:359 assert(alu.io.acc_y.data.valid === valid_r4)\n"); // @[TensorAlu.scala 359:11]
        end
    `ifdef PRINTF_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef STOP_COND
      if (`STOP_COND) begin
    `endif
        if (~(alu_io_acc_y_data_valid == valid_r4 | reset)) begin
          $fatal; // @[TensorAlu.scala 359:11]
        end
    `ifdef STOP_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef PRINTF_COND
      if (`PRINTF_COND) begin
    `endif
        if (~(alu_io_out_data_valid == valid_r4 | reset)) begin
          $fwrite(32'h80000002,
            "Assertion failed\n    at TensorAlu.scala:372 assert(alu.io.out.data.valid === valid_r4)\n"); // @[TensorAlu.scala 372:11]
        end
    `ifdef PRINTF_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef STOP_COND
      if (`STOP_COND) begin
    `endif
        if (~(alu_io_out_data_valid == valid_r4 | reset)) begin
          $fatal; // @[TensorAlu.scala 372:11]
        end
    `ifdef STOP_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef PRINTF_COND
      if (`PRINTF_COND) begin
    `endif
        if (~(~bypass_dst | reset)) begin
          $fwrite(32'h80000002,
            "Assertion failed: Bypass required on dst_idx read UInt(Reg in TensorAlu) RAW with write UInt(Reg in TensorAlu)\n\n    at TensorAlu.scala:390 assert(!bypass_dst, s\"Bypass required on dst_idx read $dst_idx_r3 RAW with write $dst_idx_r4\\n\")\n"
            ); // @[TensorAlu.scala 390:9]
        end
    `ifdef PRINTF_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef STOP_COND
      if (`STOP_COND) begin
    `endif
        if (~(~bypass_dst | reset)) begin
          $fatal; // @[TensorAlu.scala 390:9]
        end
    `ifdef STOP_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef PRINTF_COND
      if (`PRINTF_COND) begin
    `endif
        if (~(~bypass_src | reset)) begin
          $fwrite(32'h80000002,
            "Assertion failed: Bypass required on src_idx read UInt(Reg in TensorAlu) RAW with write UInt(Reg in TensorAlu)\n\n    at TensorAlu.scala:391 assert(!bypass_src, s\"Bypass required on src_idx read $src_idx_r3 RAW with write $dst_idx_r4\\n\")\n"
            ); // @[TensorAlu.scala 391:9]
        end
    `ifdef PRINTF_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef STOP_COND
      if (`STOP_COND) begin
    `endif
        if (~(~bypass_src | reset)) begin
          $fatal; // @[TensorAlu.scala 391:9]
        end
    `ifdef STOP_COND
      end
    `endif
    `endif // SYNTHESIS
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
  state = _RAND_0[1:0];
  _RAND_1 = {1{`RANDOM}};
  inflight = _RAND_1[3:0];
  _RAND_2 = {1{`RANDOM}};
  valid_r1 = _RAND_2[0:0];
  _RAND_3 = {1{`RANDOM}};
  valid_r2 = _RAND_3[0:0];
  _RAND_4 = {1{`RANDOM}};
  valid_r3 = _RAND_4[0:0];
  _RAND_5 = {1{`RANDOM}};
  valid_r4 = _RAND_5[0:0];
  _RAND_6 = {1{`RANDOM}};
  src_valid_r1 = _RAND_6[0:0];
  _RAND_7 = {1{`RANDOM}};
  src_valid_r2 = _RAND_7[0:0];
  _RAND_8 = {1{`RANDOM}};
  src_valid_r3 = _RAND_8[0:0];
  _RAND_9 = {1{`RANDOM}};
  dst_idx_r1 = _RAND_9[10:0];
  _RAND_10 = {1{`RANDOM}};
  src_idx_r1 = _RAND_10[10:0];
  _RAND_11 = {1{`RANDOM}};
  io_acc_rd_0_idx_valid_REG = _RAND_11[0:0];
  _RAND_12 = {1{`RANDOM}};
  src_idx_r2 = _RAND_12[21:0];
  _RAND_13 = {1{`RANDOM}};
  src_idx_r3 = _RAND_13[21:0];
  _RAND_14 = {1{`RANDOM}};
  dst_idx_r2 = _RAND_14[10:0];
  _RAND_15 = {1{`RANDOM}};
  dst_idx_r3 = _RAND_15[10:0];
  _RAND_16 = {1{`RANDOM}};
  dst_idx_r4 = _RAND_16[10:0];
  _RAND_17 = {1{`RANDOM}};
  io_acc_rd_0_idx_bits_REG = _RAND_17[21:0];
  _RAND_18 = {1{`RANDOM}};
  save_src_0_0 = _RAND_18[31:0];
  _RAND_19 = {1{`RANDOM}};
  save_src_0_1 = _RAND_19[31:0];
  _RAND_20 = {1{`RANDOM}};
  save_src_0_2 = _RAND_20[31:0];
  _RAND_21 = {1{`RANDOM}};
  save_src_0_3 = _RAND_21[31:0];
  _RAND_22 = {1{`RANDOM}};
  save_src_0_4 = _RAND_22[31:0];
  _RAND_23 = {1{`RANDOM}};
  save_src_0_5 = _RAND_23[31:0];
  _RAND_24 = {1{`RANDOM}};
  save_src_0_6 = _RAND_24[31:0];
  _RAND_25 = {1{`RANDOM}};
  save_src_0_7 = _RAND_25[31:0];
  _RAND_26 = {1{`RANDOM}};
  save_src_0_8 = _RAND_26[31:0];
  _RAND_27 = {1{`RANDOM}};
  save_src_0_9 = _RAND_27[31:0];
  _RAND_28 = {1{`RANDOM}};
  save_src_0_10 = _RAND_28[31:0];
  _RAND_29 = {1{`RANDOM}};
  save_src_0_11 = _RAND_29[31:0];
  _RAND_30 = {1{`RANDOM}};
  save_src_0_12 = _RAND_30[31:0];
  _RAND_31 = {1{`RANDOM}};
  save_src_0_13 = _RAND_31[31:0];
  _RAND_32 = {1{`RANDOM}};
  save_src_0_14 = _RAND_32[31:0];
  _RAND_33 = {1{`RANDOM}};
  save_src_0_15 = _RAND_33[31:0];
  _RAND_34 = {1{`RANDOM}};
  alu_io_acc_a_data_valid_REG = _RAND_34[0:0];
`endif // RANDOMIZE_REG_INIT
  `endif // RANDOMIZE
end // initial
`ifdef FIRRTL_AFTER_INITIAL
`FIRRTL_AFTER_INITIAL
`endif
`endif // SYNTHESIS
endmodule