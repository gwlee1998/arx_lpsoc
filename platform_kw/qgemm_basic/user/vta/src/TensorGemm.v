module TensorGemm(
  input         clock,
  input         reset,
  input         io_start,
  output        io_done,
  input  [9:0]  io_dec_wgt_1,
  input  [9:0]  io_dec_wgt_0,
  input  [10:0] io_dec_inp_1,
  input  [10:0] io_dec_inp_0,
  input  [10:0] io_dec_acc_1,
  input  [10:0] io_dec_acc_0,
  input         io_dec_empty_0,
  input  [13:0] io_dec_lp_1,
  input  [13:0] io_dec_lp_0,
  input  [13:0] io_dec_uop_end,
  input  [12:0] io_dec_uop_begin,
  input         io_dec_reset,
  input         io_dec_push_next,
  input         io_dec_push_prev,
  input         io_dec_pop_next,
  input         io_dec_pop_prev,
  input  [2:0]  io_dec_op,
  output        io_uop_idx_valid,
  output [10:0] io_uop_idx_bits,
  input         io_uop_data_valid,
  input  [9:0]  io_uop_data_bits_u2,
  input  [10:0] io_uop_data_bits_u1,
  input  [10:0] io_uop_data_bits_u0,
  output        io_inp_rd_0_idx_valid,
  output [10:0] io_inp_rd_0_idx_bits,
  input         io_inp_rd_0_data_valid,
  input  [31:0] io_inp_rd_0_data_bits_0_0,
  input  [31:0] io_inp_rd_0_data_bits_0_1,
  input  [31:0] io_inp_rd_0_data_bits_0_2,
  input  [31:0] io_inp_rd_0_data_bits_0_3,
  input  [31:0] io_inp_rd_0_data_bits_0_4,
  input  [31:0] io_inp_rd_0_data_bits_0_5,
  input  [31:0] io_inp_rd_0_data_bits_0_6,
  input  [31:0] io_inp_rd_0_data_bits_0_7,
  input  [31:0] io_inp_rd_0_data_bits_0_8,
  input  [31:0] io_inp_rd_0_data_bits_0_9,
  input  [31:0] io_inp_rd_0_data_bits_0_10,
  input  [31:0] io_inp_rd_0_data_bits_0_11,
  input  [31:0] io_inp_rd_0_data_bits_0_12,
  input  [31:0] io_inp_rd_0_data_bits_0_13,
  input  [31:0] io_inp_rd_0_data_bits_0_14,
  input  [31:0] io_inp_rd_0_data_bits_0_15,
  output        io_wgt_rd_0_idx_valid,
  output [9:0]  io_wgt_rd_0_idx_bits,
  input         io_wgt_rd_0_data_valid,
  input  [31:0] io_wgt_rd_0_data_bits_0_0,
  input  [31:0] io_wgt_rd_0_data_bits_0_1,
  input  [31:0] io_wgt_rd_0_data_bits_0_2,
  input  [31:0] io_wgt_rd_0_data_bits_0_3,
  input  [31:0] io_wgt_rd_0_data_bits_0_4,
  input  [31:0] io_wgt_rd_0_data_bits_0_5,
  input  [31:0] io_wgt_rd_0_data_bits_0_6,
  input  [31:0] io_wgt_rd_0_data_bits_0_7,
  input  [31:0] io_wgt_rd_0_data_bits_0_8,
  input  [31:0] io_wgt_rd_0_data_bits_0_9,
  input  [31:0] io_wgt_rd_0_data_bits_0_10,
  input  [31:0] io_wgt_rd_0_data_bits_0_11,
  input  [31:0] io_wgt_rd_0_data_bits_0_12,
  input  [31:0] io_wgt_rd_0_data_bits_0_13,
  input  [31:0] io_wgt_rd_0_data_bits_0_14,
  input  [31:0] io_wgt_rd_0_data_bits_0_15,
  input  [31:0] io_wgt_rd_0_data_bits_1_0,
  input  [31:0] io_wgt_rd_0_data_bits_1_1,
  input  [31:0] io_wgt_rd_0_data_bits_1_2,
  input  [31:0] io_wgt_rd_0_data_bits_1_3,
  input  [31:0] io_wgt_rd_0_data_bits_1_4,
  input  [31:0] io_wgt_rd_0_data_bits_1_5,
  input  [31:0] io_wgt_rd_0_data_bits_1_6,
  input  [31:0] io_wgt_rd_0_data_bits_1_7,
  input  [31:0] io_wgt_rd_0_data_bits_1_8,
  input  [31:0] io_wgt_rd_0_data_bits_1_9,
  input  [31:0] io_wgt_rd_0_data_bits_1_10,
  input  [31:0] io_wgt_rd_0_data_bits_1_11,
  input  [31:0] io_wgt_rd_0_data_bits_1_12,
  input  [31:0] io_wgt_rd_0_data_bits_1_13,
  input  [31:0] io_wgt_rd_0_data_bits_1_14,
  input  [31:0] io_wgt_rd_0_data_bits_1_15,
  input  [31:0] io_wgt_rd_0_data_bits_2_0,
  input  [31:0] io_wgt_rd_0_data_bits_2_1,
  input  [31:0] io_wgt_rd_0_data_bits_2_2,
  input  [31:0] io_wgt_rd_0_data_bits_2_3,
  input  [31:0] io_wgt_rd_0_data_bits_2_4,
  input  [31:0] io_wgt_rd_0_data_bits_2_5,
  input  [31:0] io_wgt_rd_0_data_bits_2_6,
  input  [31:0] io_wgt_rd_0_data_bits_2_7,
  input  [31:0] io_wgt_rd_0_data_bits_2_8,
  input  [31:0] io_wgt_rd_0_data_bits_2_9,
  input  [31:0] io_wgt_rd_0_data_bits_2_10,
  input  [31:0] io_wgt_rd_0_data_bits_2_11,
  input  [31:0] io_wgt_rd_0_data_bits_2_12,
  input  [31:0] io_wgt_rd_0_data_bits_2_13,
  input  [31:0] io_wgt_rd_0_data_bits_2_14,
  input  [31:0] io_wgt_rd_0_data_bits_2_15,
  input  [31:0] io_wgt_rd_0_data_bits_3_0,
  input  [31:0] io_wgt_rd_0_data_bits_3_1,
  input  [31:0] io_wgt_rd_0_data_bits_3_2,
  input  [31:0] io_wgt_rd_0_data_bits_3_3,
  input  [31:0] io_wgt_rd_0_data_bits_3_4,
  input  [31:0] io_wgt_rd_0_data_bits_3_5,
  input  [31:0] io_wgt_rd_0_data_bits_3_6,
  input  [31:0] io_wgt_rd_0_data_bits_3_7,
  input  [31:0] io_wgt_rd_0_data_bits_3_8,
  input  [31:0] io_wgt_rd_0_data_bits_3_9,
  input  [31:0] io_wgt_rd_0_data_bits_3_10,
  input  [31:0] io_wgt_rd_0_data_bits_3_11,
  input  [31:0] io_wgt_rd_0_data_bits_3_12,
  input  [31:0] io_wgt_rd_0_data_bits_3_13,
  input  [31:0] io_wgt_rd_0_data_bits_3_14,
  input  [31:0] io_wgt_rd_0_data_bits_3_15,
  input  [31:0] io_wgt_rd_0_data_bits_4_0,
  input  [31:0] io_wgt_rd_0_data_bits_4_1,
  input  [31:0] io_wgt_rd_0_data_bits_4_2,
  input  [31:0] io_wgt_rd_0_data_bits_4_3,
  input  [31:0] io_wgt_rd_0_data_bits_4_4,
  input  [31:0] io_wgt_rd_0_data_bits_4_5,
  input  [31:0] io_wgt_rd_0_data_bits_4_6,
  input  [31:0] io_wgt_rd_0_data_bits_4_7,
  input  [31:0] io_wgt_rd_0_data_bits_4_8,
  input  [31:0] io_wgt_rd_0_data_bits_4_9,
  input  [31:0] io_wgt_rd_0_data_bits_4_10,
  input  [31:0] io_wgt_rd_0_data_bits_4_11,
  input  [31:0] io_wgt_rd_0_data_bits_4_12,
  input  [31:0] io_wgt_rd_0_data_bits_4_13,
  input  [31:0] io_wgt_rd_0_data_bits_4_14,
  input  [31:0] io_wgt_rd_0_data_bits_4_15,
  input  [31:0] io_wgt_rd_0_data_bits_5_0,
  input  [31:0] io_wgt_rd_0_data_bits_5_1,
  input  [31:0] io_wgt_rd_0_data_bits_5_2,
  input  [31:0] io_wgt_rd_0_data_bits_5_3,
  input  [31:0] io_wgt_rd_0_data_bits_5_4,
  input  [31:0] io_wgt_rd_0_data_bits_5_5,
  input  [31:0] io_wgt_rd_0_data_bits_5_6,
  input  [31:0] io_wgt_rd_0_data_bits_5_7,
  input  [31:0] io_wgt_rd_0_data_bits_5_8,
  input  [31:0] io_wgt_rd_0_data_bits_5_9,
  input  [31:0] io_wgt_rd_0_data_bits_5_10,
  input  [31:0] io_wgt_rd_0_data_bits_5_11,
  input  [31:0] io_wgt_rd_0_data_bits_5_12,
  input  [31:0] io_wgt_rd_0_data_bits_5_13,
  input  [31:0] io_wgt_rd_0_data_bits_5_14,
  input  [31:0] io_wgt_rd_0_data_bits_5_15,
  input  [31:0] io_wgt_rd_0_data_bits_6_0,
  input  [31:0] io_wgt_rd_0_data_bits_6_1,
  input  [31:0] io_wgt_rd_0_data_bits_6_2,
  input  [31:0] io_wgt_rd_0_data_bits_6_3,
  input  [31:0] io_wgt_rd_0_data_bits_6_4,
  input  [31:0] io_wgt_rd_0_data_bits_6_5,
  input  [31:0] io_wgt_rd_0_data_bits_6_6,
  input  [31:0] io_wgt_rd_0_data_bits_6_7,
  input  [31:0] io_wgt_rd_0_data_bits_6_8,
  input  [31:0] io_wgt_rd_0_data_bits_6_9,
  input  [31:0] io_wgt_rd_0_data_bits_6_10,
  input  [31:0] io_wgt_rd_0_data_bits_6_11,
  input  [31:0] io_wgt_rd_0_data_bits_6_12,
  input  [31:0] io_wgt_rd_0_data_bits_6_13,
  input  [31:0] io_wgt_rd_0_data_bits_6_14,
  input  [31:0] io_wgt_rd_0_data_bits_6_15,
  input  [31:0] io_wgt_rd_0_data_bits_7_0,
  input  [31:0] io_wgt_rd_0_data_bits_7_1,
  input  [31:0] io_wgt_rd_0_data_bits_7_2,
  input  [31:0] io_wgt_rd_0_data_bits_7_3,
  input  [31:0] io_wgt_rd_0_data_bits_7_4,
  input  [31:0] io_wgt_rd_0_data_bits_7_5,
  input  [31:0] io_wgt_rd_0_data_bits_7_6,
  input  [31:0] io_wgt_rd_0_data_bits_7_7,
  input  [31:0] io_wgt_rd_0_data_bits_7_8,
  input  [31:0] io_wgt_rd_0_data_bits_7_9,
  input  [31:0] io_wgt_rd_0_data_bits_7_10,
  input  [31:0] io_wgt_rd_0_data_bits_7_11,
  input  [31:0] io_wgt_rd_0_data_bits_7_12,
  input  [31:0] io_wgt_rd_0_data_bits_7_13,
  input  [31:0] io_wgt_rd_0_data_bits_7_14,
  input  [31:0] io_wgt_rd_0_data_bits_7_15,
  input  [31:0] io_wgt_rd_0_data_bits_8_0,
  input  [31:0] io_wgt_rd_0_data_bits_8_1,
  input  [31:0] io_wgt_rd_0_data_bits_8_2,
  input  [31:0] io_wgt_rd_0_data_bits_8_3,
  input  [31:0] io_wgt_rd_0_data_bits_8_4,
  input  [31:0] io_wgt_rd_0_data_bits_8_5,
  input  [31:0] io_wgt_rd_0_data_bits_8_6,
  input  [31:0] io_wgt_rd_0_data_bits_8_7,
  input  [31:0] io_wgt_rd_0_data_bits_8_8,
  input  [31:0] io_wgt_rd_0_data_bits_8_9,
  input  [31:0] io_wgt_rd_0_data_bits_8_10,
  input  [31:0] io_wgt_rd_0_data_bits_8_11,
  input  [31:0] io_wgt_rd_0_data_bits_8_12,
  input  [31:0] io_wgt_rd_0_data_bits_8_13,
  input  [31:0] io_wgt_rd_0_data_bits_8_14,
  input  [31:0] io_wgt_rd_0_data_bits_8_15,
  input  [31:0] io_wgt_rd_0_data_bits_9_0,
  input  [31:0] io_wgt_rd_0_data_bits_9_1,
  input  [31:0] io_wgt_rd_0_data_bits_9_2,
  input  [31:0] io_wgt_rd_0_data_bits_9_3,
  input  [31:0] io_wgt_rd_0_data_bits_9_4,
  input  [31:0] io_wgt_rd_0_data_bits_9_5,
  input  [31:0] io_wgt_rd_0_data_bits_9_6,
  input  [31:0] io_wgt_rd_0_data_bits_9_7,
  input  [31:0] io_wgt_rd_0_data_bits_9_8,
  input  [31:0] io_wgt_rd_0_data_bits_9_9,
  input  [31:0] io_wgt_rd_0_data_bits_9_10,
  input  [31:0] io_wgt_rd_0_data_bits_9_11,
  input  [31:0] io_wgt_rd_0_data_bits_9_12,
  input  [31:0] io_wgt_rd_0_data_bits_9_13,
  input  [31:0] io_wgt_rd_0_data_bits_9_14,
  input  [31:0] io_wgt_rd_0_data_bits_9_15,
  input  [31:0] io_wgt_rd_0_data_bits_10_0,
  input  [31:0] io_wgt_rd_0_data_bits_10_1,
  input  [31:0] io_wgt_rd_0_data_bits_10_2,
  input  [31:0] io_wgt_rd_0_data_bits_10_3,
  input  [31:0] io_wgt_rd_0_data_bits_10_4,
  input  [31:0] io_wgt_rd_0_data_bits_10_5,
  input  [31:0] io_wgt_rd_0_data_bits_10_6,
  input  [31:0] io_wgt_rd_0_data_bits_10_7,
  input  [31:0] io_wgt_rd_0_data_bits_10_8,
  input  [31:0] io_wgt_rd_0_data_bits_10_9,
  input  [31:0] io_wgt_rd_0_data_bits_10_10,
  input  [31:0] io_wgt_rd_0_data_bits_10_11,
  input  [31:0] io_wgt_rd_0_data_bits_10_12,
  input  [31:0] io_wgt_rd_0_data_bits_10_13,
  input  [31:0] io_wgt_rd_0_data_bits_10_14,
  input  [31:0] io_wgt_rd_0_data_bits_10_15,
  input  [31:0] io_wgt_rd_0_data_bits_11_0,
  input  [31:0] io_wgt_rd_0_data_bits_11_1,
  input  [31:0] io_wgt_rd_0_data_bits_11_2,
  input  [31:0] io_wgt_rd_0_data_bits_11_3,
  input  [31:0] io_wgt_rd_0_data_bits_11_4,
  input  [31:0] io_wgt_rd_0_data_bits_11_5,
  input  [31:0] io_wgt_rd_0_data_bits_11_6,
  input  [31:0] io_wgt_rd_0_data_bits_11_7,
  input  [31:0] io_wgt_rd_0_data_bits_11_8,
  input  [31:0] io_wgt_rd_0_data_bits_11_9,
  input  [31:0] io_wgt_rd_0_data_bits_11_10,
  input  [31:0] io_wgt_rd_0_data_bits_11_11,
  input  [31:0] io_wgt_rd_0_data_bits_11_12,
  input  [31:0] io_wgt_rd_0_data_bits_11_13,
  input  [31:0] io_wgt_rd_0_data_bits_11_14,
  input  [31:0] io_wgt_rd_0_data_bits_11_15,
  input  [31:0] io_wgt_rd_0_data_bits_12_0,
  input  [31:0] io_wgt_rd_0_data_bits_12_1,
  input  [31:0] io_wgt_rd_0_data_bits_12_2,
  input  [31:0] io_wgt_rd_0_data_bits_12_3,
  input  [31:0] io_wgt_rd_0_data_bits_12_4,
  input  [31:0] io_wgt_rd_0_data_bits_12_5,
  input  [31:0] io_wgt_rd_0_data_bits_12_6,
  input  [31:0] io_wgt_rd_0_data_bits_12_7,
  input  [31:0] io_wgt_rd_0_data_bits_12_8,
  input  [31:0] io_wgt_rd_0_data_bits_12_9,
  input  [31:0] io_wgt_rd_0_data_bits_12_10,
  input  [31:0] io_wgt_rd_0_data_bits_12_11,
  input  [31:0] io_wgt_rd_0_data_bits_12_12,
  input  [31:0] io_wgt_rd_0_data_bits_12_13,
  input  [31:0] io_wgt_rd_0_data_bits_12_14,
  input  [31:0] io_wgt_rd_0_data_bits_12_15,
  input  [31:0] io_wgt_rd_0_data_bits_13_0,
  input  [31:0] io_wgt_rd_0_data_bits_13_1,
  input  [31:0] io_wgt_rd_0_data_bits_13_2,
  input  [31:0] io_wgt_rd_0_data_bits_13_3,
  input  [31:0] io_wgt_rd_0_data_bits_13_4,
  input  [31:0] io_wgt_rd_0_data_bits_13_5,
  input  [31:0] io_wgt_rd_0_data_bits_13_6,
  input  [31:0] io_wgt_rd_0_data_bits_13_7,
  input  [31:0] io_wgt_rd_0_data_bits_13_8,
  input  [31:0] io_wgt_rd_0_data_bits_13_9,
  input  [31:0] io_wgt_rd_0_data_bits_13_10,
  input  [31:0] io_wgt_rd_0_data_bits_13_11,
  input  [31:0] io_wgt_rd_0_data_bits_13_12,
  input  [31:0] io_wgt_rd_0_data_bits_13_13,
  input  [31:0] io_wgt_rd_0_data_bits_13_14,
  input  [31:0] io_wgt_rd_0_data_bits_13_15,
  input  [31:0] io_wgt_rd_0_data_bits_14_0,
  input  [31:0] io_wgt_rd_0_data_bits_14_1,
  input  [31:0] io_wgt_rd_0_data_bits_14_2,
  input  [31:0] io_wgt_rd_0_data_bits_14_3,
  input  [31:0] io_wgt_rd_0_data_bits_14_4,
  input  [31:0] io_wgt_rd_0_data_bits_14_5,
  input  [31:0] io_wgt_rd_0_data_bits_14_6,
  input  [31:0] io_wgt_rd_0_data_bits_14_7,
  input  [31:0] io_wgt_rd_0_data_bits_14_8,
  input  [31:0] io_wgt_rd_0_data_bits_14_9,
  input  [31:0] io_wgt_rd_0_data_bits_14_10,
  input  [31:0] io_wgt_rd_0_data_bits_14_11,
  input  [31:0] io_wgt_rd_0_data_bits_14_12,
  input  [31:0] io_wgt_rd_0_data_bits_14_13,
  input  [31:0] io_wgt_rd_0_data_bits_14_14,
  input  [31:0] io_wgt_rd_0_data_bits_14_15,
  input  [31:0] io_wgt_rd_0_data_bits_15_0,
  input  [31:0] io_wgt_rd_0_data_bits_15_1,
  input  [31:0] io_wgt_rd_0_data_bits_15_2,
  input  [31:0] io_wgt_rd_0_data_bits_15_3,
  input  [31:0] io_wgt_rd_0_data_bits_15_4,
  input  [31:0] io_wgt_rd_0_data_bits_15_5,
  input  [31:0] io_wgt_rd_0_data_bits_15_6,
  input  [31:0] io_wgt_rd_0_data_bits_15_7,
  input  [31:0] io_wgt_rd_0_data_bits_15_8,
  input  [31:0] io_wgt_rd_0_data_bits_15_9,
  input  [31:0] io_wgt_rd_0_data_bits_15_10,
  input  [31:0] io_wgt_rd_0_data_bits_15_11,
  input  [31:0] io_wgt_rd_0_data_bits_15_12,
  input  [31:0] io_wgt_rd_0_data_bits_15_13,
  input  [31:0] io_wgt_rd_0_data_bits_15_14,
  input  [31:0] io_wgt_rd_0_data_bits_15_15,
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
`endif // RANDOMIZE_REG_INIT
  wire  m_clock; // @[TensorGemm.scala 554:17]
  wire  m_reset; // @[TensorGemm.scala 554:17]
  wire  m_io_start; // @[TensorGemm.scala 554:17]
  wire  m_io_last; // @[TensorGemm.scala 554:17]
  wire [9:0] m_io_dec_wgt_1; // @[TensorGemm.scala 554:17]
  wire [9:0] m_io_dec_wgt_0; // @[TensorGemm.scala 554:17]
  wire [10:0] m_io_dec_inp_1; // @[TensorGemm.scala 554:17]
  wire [10:0] m_io_dec_inp_0; // @[TensorGemm.scala 554:17]
  wire [10:0] m_io_dec_acc_1; // @[TensorGemm.scala 554:17]
  wire [10:0] m_io_dec_acc_0; // @[TensorGemm.scala 554:17]
  wire [13:0] m_io_dec_lp_1; // @[TensorGemm.scala 554:17]
  wire [13:0] m_io_dec_lp_0; // @[TensorGemm.scala 554:17]
  wire [13:0] m_io_dec_uop_end; // @[TensorGemm.scala 554:17]
  wire [12:0] m_io_dec_uop_begin; // @[TensorGemm.scala 554:17]
  wire [10:0] m_io_acc_i; // @[TensorGemm.scala 554:17]
  wire [10:0] m_io_inp_i; // @[TensorGemm.scala 554:17]
  wire [9:0] m_io_wgt_i; // @[TensorGemm.scala 554:17]
  wire [10:0] m_io_uop_idx; // @[TensorGemm.scala 554:17]
  wire  m_io_valid; // @[TensorGemm.scala 554:17]
  wire  reset_pipe_clock; // @[TensorGemm.scala 603:26]
  wire  reset_pipe_reset; // @[TensorGemm.scala 603:26]
  wire  reset_pipe_io_enq_valid; // @[TensorGemm.scala 603:26]
  wire  reset_pipe_io_enq_bits; // @[TensorGemm.scala 603:26]
  wire  reset_pipe_io_deq_valid; // @[TensorGemm.scala 603:26]
  wire  reset_pipe_io_deq_bits; // @[TensorGemm.scala 603:26]
  wire  acc_idx_pipe_clock; // @[TensorGemm.scala 610:28]
  wire  acc_idx_pipe_reset; // @[TensorGemm.scala 610:28]
  wire  acc_idx_pipe_io_enq_valid; // @[TensorGemm.scala 610:28]
  wire [10:0] acc_idx_pipe_io_enq_bits; // @[TensorGemm.scala 610:28]
  wire  acc_idx_pipe_io_deq_valid; // @[TensorGemm.scala 610:28]
  wire [10:0] acc_idx_pipe_io_deq_bits; // @[TensorGemm.scala 610:28]
  wire  wrpipe0_clock; // @[TensorGemm.scala 637:23]
  wire  wrpipe0_reset; // @[TensorGemm.scala 637:23]
  wire  wrpipe0_io_enq_valid; // @[TensorGemm.scala 637:23]
  wire [10:0] wrpipe0_io_enq_bits; // @[TensorGemm.scala 637:23]
  wire  wrpipe0_io_deq_valid; // @[TensorGemm.scala 637:23]
  wire [10:0] wrpipe0_io_deq_bits; // @[TensorGemm.scala 637:23]
  wire  wrpipeNs_clock; // @[TensorGemm.scala 641:24]
  wire  wrpipeNs_reset; // @[TensorGemm.scala 641:24]
  wire  wrpipeNs_io_enq_valid; // @[TensorGemm.scala 641:24]
  wire [10:0] wrpipeNs_io_enq_bits; // @[TensorGemm.scala 641:24]
  wire  wrpipeNs_io_deq_valid; // @[TensorGemm.scala 641:24]
  wire [10:0] wrpipeNs_io_deq_bits; // @[TensorGemm.scala 641:24]
  wire  wrpipe_0_clock; // @[TensorGemm.scala 645:22]
  wire  wrpipe_0_reset; // @[TensorGemm.scala 645:22]
  wire  wrpipe_0_io_enq_valid; // @[TensorGemm.scala 645:22]
  wire [10:0] wrpipe_0_io_enq_bits; // @[TensorGemm.scala 645:22]
  wire  wrpipe_0_io_deq_valid; // @[TensorGemm.scala 645:22]
  wire [10:0] wrpipe_0_io_deq_bits; // @[TensorGemm.scala 645:22]
  wire  mvc_0_clock; // @[TensorGemm.scala 686:55]
  wire  mvc_0_io_valid_reset; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_inp_data_bits_0_0; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_inp_data_bits_0_1; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_inp_data_bits_0_2; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_inp_data_bits_0_3; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_inp_data_bits_0_4; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_inp_data_bits_0_5; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_inp_data_bits_0_6; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_inp_data_bits_0_7; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_inp_data_bits_0_8; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_inp_data_bits_0_9; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_inp_data_bits_0_10; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_inp_data_bits_0_11; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_inp_data_bits_0_12; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_inp_data_bits_0_13; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_inp_data_bits_0_14; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_inp_data_bits_0_15; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_0_0; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_0_1; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_0_2; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_0_3; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_0_4; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_0_5; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_0_6; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_0_7; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_0_8; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_0_9; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_0_10; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_0_11; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_0_12; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_0_13; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_0_14; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_0_15; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_1_0; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_1_1; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_1_2; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_1_3; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_1_4; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_1_5; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_1_6; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_1_7; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_1_8; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_1_9; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_1_10; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_1_11; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_1_12; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_1_13; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_1_14; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_1_15; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_2_0; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_2_1; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_2_2; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_2_3; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_2_4; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_2_5; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_2_6; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_2_7; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_2_8; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_2_9; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_2_10; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_2_11; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_2_12; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_2_13; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_2_14; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_2_15; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_3_0; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_3_1; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_3_2; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_3_3; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_3_4; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_3_5; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_3_6; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_3_7; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_3_8; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_3_9; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_3_10; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_3_11; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_3_12; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_3_13; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_3_14; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_3_15; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_4_0; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_4_1; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_4_2; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_4_3; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_4_4; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_4_5; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_4_6; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_4_7; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_4_8; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_4_9; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_4_10; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_4_11; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_4_12; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_4_13; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_4_14; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_4_15; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_5_0; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_5_1; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_5_2; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_5_3; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_5_4; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_5_5; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_5_6; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_5_7; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_5_8; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_5_9; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_5_10; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_5_11; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_5_12; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_5_13; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_5_14; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_5_15; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_6_0; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_6_1; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_6_2; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_6_3; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_6_4; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_6_5; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_6_6; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_6_7; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_6_8; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_6_9; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_6_10; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_6_11; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_6_12; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_6_13; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_6_14; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_6_15; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_7_0; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_7_1; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_7_2; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_7_3; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_7_4; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_7_5; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_7_6; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_7_7; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_7_8; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_7_9; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_7_10; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_7_11; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_7_12; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_7_13; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_7_14; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_7_15; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_8_0; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_8_1; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_8_2; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_8_3; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_8_4; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_8_5; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_8_6; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_8_7; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_8_8; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_8_9; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_8_10; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_8_11; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_8_12; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_8_13; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_8_14; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_8_15; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_9_0; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_9_1; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_9_2; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_9_3; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_9_4; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_9_5; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_9_6; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_9_7; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_9_8; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_9_9; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_9_10; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_9_11; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_9_12; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_9_13; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_9_14; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_9_15; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_10_0; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_10_1; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_10_2; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_10_3; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_10_4; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_10_5; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_10_6; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_10_7; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_10_8; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_10_9; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_10_10; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_10_11; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_10_12; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_10_13; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_10_14; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_10_15; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_11_0; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_11_1; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_11_2; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_11_3; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_11_4; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_11_5; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_11_6; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_11_7; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_11_8; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_11_9; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_11_10; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_11_11; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_11_12; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_11_13; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_11_14; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_11_15; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_12_0; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_12_1; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_12_2; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_12_3; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_12_4; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_12_5; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_12_6; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_12_7; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_12_8; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_12_9; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_12_10; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_12_11; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_12_12; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_12_13; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_12_14; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_12_15; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_13_0; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_13_1; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_13_2; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_13_3; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_13_4; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_13_5; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_13_6; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_13_7; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_13_8; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_13_9; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_13_10; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_13_11; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_13_12; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_13_13; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_13_14; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_13_15; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_14_0; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_14_1; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_14_2; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_14_3; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_14_4; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_14_5; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_14_6; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_14_7; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_14_8; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_14_9; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_14_10; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_14_11; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_14_12; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_14_13; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_14_14; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_14_15; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_15_0; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_15_1; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_15_2; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_15_3; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_15_4; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_15_5; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_15_6; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_15_7; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_15_8; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_15_9; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_15_10; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_15_11; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_15_12; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_15_13; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_15_14; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_wgt_data_bits_15_15; // @[TensorGemm.scala 686:55]
  wire  mvc_0_io_acc_i_data_valid; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_acc_i_data_bits_0_0; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_acc_i_data_bits_0_1; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_acc_i_data_bits_0_2; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_acc_i_data_bits_0_3; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_acc_i_data_bits_0_4; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_acc_i_data_bits_0_5; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_acc_i_data_bits_0_6; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_acc_i_data_bits_0_7; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_acc_i_data_bits_0_8; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_acc_i_data_bits_0_9; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_acc_i_data_bits_0_10; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_acc_i_data_bits_0_11; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_acc_i_data_bits_0_12; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_acc_i_data_bits_0_13; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_acc_i_data_bits_0_14; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_acc_i_data_bits_0_15; // @[TensorGemm.scala 686:55]
  wire  mvc_0_io_acc_o_data_valid; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_acc_o_data_bits_0_0; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_acc_o_data_bits_0_1; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_acc_o_data_bits_0_2; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_acc_o_data_bits_0_3; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_acc_o_data_bits_0_4; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_acc_o_data_bits_0_5; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_acc_o_data_bits_0_6; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_acc_o_data_bits_0_7; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_acc_o_data_bits_0_8; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_acc_o_data_bits_0_9; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_acc_o_data_bits_0_10; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_acc_o_data_bits_0_11; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_acc_o_data_bits_0_12; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_acc_o_data_bits_0_13; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_acc_o_data_bits_0_14; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_acc_o_data_bits_0_15; // @[TensorGemm.scala 686:55]
  wire  mvc_0_io_out_data_valid; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_out_data_bits_0_0; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_out_data_bits_0_1; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_out_data_bits_0_2; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_out_data_bits_0_3; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_out_data_bits_0_4; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_out_data_bits_0_5; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_out_data_bits_0_6; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_out_data_bits_0_7; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_out_data_bits_0_8; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_out_data_bits_0_9; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_out_data_bits_0_10; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_out_data_bits_0_11; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_out_data_bits_0_12; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_out_data_bits_0_13; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_out_data_bits_0_14; // @[TensorGemm.scala 686:55]
  wire [31:0] mvc_0_io_out_data_bits_0_15; // @[TensorGemm.scala 686:55]
  wire  mvc_0_io_bypass_cond; // @[TensorGemm.scala 686:55]
  wire  wrpipe2_clock; // @[TensorGemm.scala 691:25]
  wire  wrpipe2_reset; // @[TensorGemm.scala 691:25]
  wire  wrpipe2_io_enq_valid; // @[TensorGemm.scala 691:25]
  wire [10:0] wrpipe2_io_enq_bits; // @[TensorGemm.scala 691:25]
  wire  wrpipe2_io_deq_valid; // @[TensorGemm.scala 691:25]
  wire [10:0] wrpipe2_io_deq_bits; // @[TensorGemm.scala 691:25]
  reg  delayed_valid; // @[Reg.scala 27:20]
  wire  _GEN_0 = m_io_valid; // @[Reg.scala 28:19 Reg.scala 28:23 Reg.scala 27:20]
  reg [10:0] delayed_acc_i; // @[Reg.scala 15:16]
  reg [10:0] delayed_inp_i; // @[Reg.scala 15:16]
  reg [9:0] delayed_wgt_i; // @[Reg.scala 15:16]
  reg [1:0] state; // @[TensorGemm.scala 566:22]
  reg [3:0] inflight; // @[TensorGemm.scala 567:25]
  reg [9:0] capture_dec_wgt_1; // @[TensorGemm.scala 569:24]
  reg [9:0] capture_dec_wgt_0; // @[TensorGemm.scala 569:24]
  reg [10:0] capture_dec_inp_1; // @[TensorGemm.scala 569:24]
  reg [10:0] capture_dec_inp_0; // @[TensorGemm.scala 569:24]
  reg [10:0] capture_dec_acc_1; // @[TensorGemm.scala 569:24]
  reg [10:0] capture_dec_acc_0; // @[TensorGemm.scala 569:24]
  reg  capture_dec_empty_0; // @[TensorGemm.scala 569:24]
  reg [13:0] capture_dec_lp_1; // @[TensorGemm.scala 569:24]
  reg [13:0] capture_dec_lp_0; // @[TensorGemm.scala 569:24]
  reg [13:0] capture_dec_uop_end; // @[TensorGemm.scala 569:24]
  reg [12:0] capture_dec_uop_begin; // @[TensorGemm.scala 569:24]
  reg  capture_dec_reset; // @[TensorGemm.scala 569:24]
  reg  capture_dec_push_next; // @[TensorGemm.scala 569:24]
  reg  capture_dec_push_prev; // @[TensorGemm.scala 569:24]
  reg  capture_dec_pop_next; // @[TensorGemm.scala 569:24]
  reg  capture_dec_pop_prev; // @[TensorGemm.scala 569:24]
  reg [2:0] capture_dec_op; // @[TensorGemm.scala 569:24]
  wire  _T = state == 2'h0; // @[TensorGemm.scala 572:14]
  wire  _T_5 = inflight == 4'h0; // @[TensorGemm.scala 579:42]
  wire  _T_6 = state == 2'h2 & inflight == 4'h0; // @[TensorGemm.scala 579:30]
  wire  _GEN_7 = state == 2'h1 & m_io_last ? 1'h0 : _T_6; // @[TensorGemm.scala 577:43 TensorGemm.scala 571:11]
  wire [34:0] lo = {capture_dec_uop_end,capture_dec_uop_begin,capture_dec_reset,capture_dec_push_next,
    capture_dec_push_prev,capture_dec_pop_next,capture_dec_pop_prev,capture_dec_op}; // @[TensorGemm.scala 585:41]
  wire [127:0] _T_8 = {capture_dec_wgt_1,capture_dec_wgt_0,capture_dec_inp_1,capture_dec_inp_0,capture_dec_acc_1,
    capture_dec_acc_0,capture_dec_empty_0,capture_dec_lp_1,capture_dec_lp_0,lo}; // @[TensorGemm.scala 585:41]
  wire [34:0] lo_1 = {io_dec_uop_end,io_dec_uop_begin,io_dec_reset,io_dec_push_next,io_dec_push_prev,io_dec_pop_next,
    io_dec_pop_prev,io_dec_op}; // @[TensorGemm.scala 585:59]
  wire [127:0] _T_9 = {io_dec_wgt_1,io_dec_wgt_0,io_dec_inp_1,io_dec_inp_0,io_dec_acc_1,io_dec_acc_0,io_dec_empty_0,
    io_dec_lp_1,io_dec_lp_0,lo_1}; // @[TensorGemm.scala 585:59]
  wire  _T_10 = _T_8 == _T_9; // @[TensorGemm.scala 585:48]
  reg  delayed_uop_valid; // @[TensorGemm.scala 618:34]
  reg  io_acc_rd_0_idx_valid_REG; // @[TensorGemm.scala 623:40]
  reg [10:0] io_acc_rd_0_idx_bits_REG; // @[TensorGemm.scala 624:39]
  wire  _T_39 = m_io_valid & wrpipeNs_io_deq_valid; // @[TensorGemm.scala 654:19]
  wire [3:0] _inflight_T_1 = inflight + 4'h1; // @[TensorGemm.scala 657:26]
  wire [3:0] _inflight_T_3 = inflight - 4'h1; // @[TensorGemm.scala 660:26]
  wire [3:0] _GEN_27 = wrpipeNs_io_deq_valid ? _inflight_T_3 : inflight; // @[TensorGemm.scala 658:37 TensorGemm.scala 660:14 TensorGemm.scala 567:25]
  reg  mvc_0_io_valid_reset_REG; // @[TensorGemm.scala 698:40]
  wire  _GEN_31 = ~_T_39; // @[TensorGemm.scala 656:11]
  TensorGemmIndexGenerator m ( // @[TensorGemm.scala 554:17]
    .clock(m_clock),
    .reset(m_reset),
    .io_start(m_io_start),
    .io_last(m_io_last),
    .io_dec_wgt_1(m_io_dec_wgt_1),
    .io_dec_wgt_0(m_io_dec_wgt_0),
    .io_dec_inp_1(m_io_dec_inp_1),
    .io_dec_inp_0(m_io_dec_inp_0),
    .io_dec_acc_1(m_io_dec_acc_1),
    .io_dec_acc_0(m_io_dec_acc_0),
    .io_dec_lp_1(m_io_dec_lp_1),
    .io_dec_lp_0(m_io_dec_lp_0),
    .io_dec_uop_end(m_io_dec_uop_end),
    .io_dec_uop_begin(m_io_dec_uop_begin),
    .io_acc_i(m_io_acc_i),
    .io_inp_i(m_io_inp_i),
    .io_wgt_i(m_io_wgt_i),
    .io_uop_idx(m_io_uop_idx),
    .io_valid(m_io_valid)
  );
  Pipe reset_pipe ( // @[TensorGemm.scala 603:26]
    .clock(reset_pipe_clock),
    .reset(reset_pipe_reset),
    .io_enq_valid(reset_pipe_io_enq_valid),
    .io_enq_bits(reset_pipe_io_enq_bits),
    .io_deq_valid(reset_pipe_io_deq_valid),
    .io_deq_bits(reset_pipe_io_deq_bits)
  );
  Pipe_1 acc_idx_pipe ( // @[TensorGemm.scala 610:28]
    .clock(acc_idx_pipe_clock),
    .reset(acc_idx_pipe_reset),
    .io_enq_valid(acc_idx_pipe_io_enq_valid),
    .io_enq_bits(acc_idx_pipe_io_enq_bits),
    .io_deq_valid(acc_idx_pipe_io_deq_valid),
    .io_deq_bits(acc_idx_pipe_io_deq_bits)
  );
  Pipe_2 wrpipe0 ( // @[TensorGemm.scala 637:23]
    .clock(wrpipe0_clock),
    .reset(wrpipe0_reset),
    .io_enq_valid(wrpipe0_io_enq_valid),
    .io_enq_bits(wrpipe0_io_enq_bits),
    .io_deq_valid(wrpipe0_io_deq_valid),
    .io_deq_bits(wrpipe0_io_deq_bits)
  );
  Pipe_1 wrpipeNs ( // @[TensorGemm.scala 641:24]
    .clock(wrpipeNs_clock),
    .reset(wrpipeNs_reset),
    .io_enq_valid(wrpipeNs_io_enq_valid),
    .io_enq_bits(wrpipeNs_io_enq_bits),
    .io_deq_valid(wrpipeNs_io_deq_valid),
    .io_deq_bits(wrpipeNs_io_deq_bits)
  );
  Pipe_1 wrpipe_0 ( // @[TensorGemm.scala 645:22]
    .clock(wrpipe_0_clock),
    .reset(wrpipe_0_reset),
    .io_enq_valid(wrpipe_0_io_enq_valid),
    .io_enq_bits(wrpipe_0_io_enq_bits),
    .io_deq_valid(wrpipe_0_io_deq_valid),
    .io_deq_bits(wrpipe_0_io_deq_bits)
  );
  MatrixVectorMultiplicationBypass mvc_0 ( // @[TensorGemm.scala 686:55]
    .clock(mvc_0_clock),
    .io_valid_reset(mvc_0_io_valid_reset),
    .io_inp_data_bits_0_0(mvc_0_io_inp_data_bits_0_0),
    .io_inp_data_bits_0_1(mvc_0_io_inp_data_bits_0_1),
    .io_inp_data_bits_0_2(mvc_0_io_inp_data_bits_0_2),
    .io_inp_data_bits_0_3(mvc_0_io_inp_data_bits_0_3),
    .io_inp_data_bits_0_4(mvc_0_io_inp_data_bits_0_4),
    .io_inp_data_bits_0_5(mvc_0_io_inp_data_bits_0_5),
    .io_inp_data_bits_0_6(mvc_0_io_inp_data_bits_0_6),
    .io_inp_data_bits_0_7(mvc_0_io_inp_data_bits_0_7),
    .io_inp_data_bits_0_8(mvc_0_io_inp_data_bits_0_8),
    .io_inp_data_bits_0_9(mvc_0_io_inp_data_bits_0_9),
    .io_inp_data_bits_0_10(mvc_0_io_inp_data_bits_0_10),
    .io_inp_data_bits_0_11(mvc_0_io_inp_data_bits_0_11),
    .io_inp_data_bits_0_12(mvc_0_io_inp_data_bits_0_12),
    .io_inp_data_bits_0_13(mvc_0_io_inp_data_bits_0_13),
    .io_inp_data_bits_0_14(mvc_0_io_inp_data_bits_0_14),
    .io_inp_data_bits_0_15(mvc_0_io_inp_data_bits_0_15),
    .io_wgt_data_bits_0_0(mvc_0_io_wgt_data_bits_0_0),
    .io_wgt_data_bits_0_1(mvc_0_io_wgt_data_bits_0_1),
    .io_wgt_data_bits_0_2(mvc_0_io_wgt_data_bits_0_2),
    .io_wgt_data_bits_0_3(mvc_0_io_wgt_data_bits_0_3),
    .io_wgt_data_bits_0_4(mvc_0_io_wgt_data_bits_0_4),
    .io_wgt_data_bits_0_5(mvc_0_io_wgt_data_bits_0_5),
    .io_wgt_data_bits_0_6(mvc_0_io_wgt_data_bits_0_6),
    .io_wgt_data_bits_0_7(mvc_0_io_wgt_data_bits_0_7),
    .io_wgt_data_bits_0_8(mvc_0_io_wgt_data_bits_0_8),
    .io_wgt_data_bits_0_9(mvc_0_io_wgt_data_bits_0_9),
    .io_wgt_data_bits_0_10(mvc_0_io_wgt_data_bits_0_10),
    .io_wgt_data_bits_0_11(mvc_0_io_wgt_data_bits_0_11),
    .io_wgt_data_bits_0_12(mvc_0_io_wgt_data_bits_0_12),
    .io_wgt_data_bits_0_13(mvc_0_io_wgt_data_bits_0_13),
    .io_wgt_data_bits_0_14(mvc_0_io_wgt_data_bits_0_14),
    .io_wgt_data_bits_0_15(mvc_0_io_wgt_data_bits_0_15),
    .io_wgt_data_bits_1_0(mvc_0_io_wgt_data_bits_1_0),
    .io_wgt_data_bits_1_1(mvc_0_io_wgt_data_bits_1_1),
    .io_wgt_data_bits_1_2(mvc_0_io_wgt_data_bits_1_2),
    .io_wgt_data_bits_1_3(mvc_0_io_wgt_data_bits_1_3),
    .io_wgt_data_bits_1_4(mvc_0_io_wgt_data_bits_1_4),
    .io_wgt_data_bits_1_5(mvc_0_io_wgt_data_bits_1_5),
    .io_wgt_data_bits_1_6(mvc_0_io_wgt_data_bits_1_6),
    .io_wgt_data_bits_1_7(mvc_0_io_wgt_data_bits_1_7),
    .io_wgt_data_bits_1_8(mvc_0_io_wgt_data_bits_1_8),
    .io_wgt_data_bits_1_9(mvc_0_io_wgt_data_bits_1_9),
    .io_wgt_data_bits_1_10(mvc_0_io_wgt_data_bits_1_10),
    .io_wgt_data_bits_1_11(mvc_0_io_wgt_data_bits_1_11),
    .io_wgt_data_bits_1_12(mvc_0_io_wgt_data_bits_1_12),
    .io_wgt_data_bits_1_13(mvc_0_io_wgt_data_bits_1_13),
    .io_wgt_data_bits_1_14(mvc_0_io_wgt_data_bits_1_14),
    .io_wgt_data_bits_1_15(mvc_0_io_wgt_data_bits_1_15),
    .io_wgt_data_bits_2_0(mvc_0_io_wgt_data_bits_2_0),
    .io_wgt_data_bits_2_1(mvc_0_io_wgt_data_bits_2_1),
    .io_wgt_data_bits_2_2(mvc_0_io_wgt_data_bits_2_2),
    .io_wgt_data_bits_2_3(mvc_0_io_wgt_data_bits_2_3),
    .io_wgt_data_bits_2_4(mvc_0_io_wgt_data_bits_2_4),
    .io_wgt_data_bits_2_5(mvc_0_io_wgt_data_bits_2_5),
    .io_wgt_data_bits_2_6(mvc_0_io_wgt_data_bits_2_6),
    .io_wgt_data_bits_2_7(mvc_0_io_wgt_data_bits_2_7),
    .io_wgt_data_bits_2_8(mvc_0_io_wgt_data_bits_2_8),
    .io_wgt_data_bits_2_9(mvc_0_io_wgt_data_bits_2_9),
    .io_wgt_data_bits_2_10(mvc_0_io_wgt_data_bits_2_10),
    .io_wgt_data_bits_2_11(mvc_0_io_wgt_data_bits_2_11),
    .io_wgt_data_bits_2_12(mvc_0_io_wgt_data_bits_2_12),
    .io_wgt_data_bits_2_13(mvc_0_io_wgt_data_bits_2_13),
    .io_wgt_data_bits_2_14(mvc_0_io_wgt_data_bits_2_14),
    .io_wgt_data_bits_2_15(mvc_0_io_wgt_data_bits_2_15),
    .io_wgt_data_bits_3_0(mvc_0_io_wgt_data_bits_3_0),
    .io_wgt_data_bits_3_1(mvc_0_io_wgt_data_bits_3_1),
    .io_wgt_data_bits_3_2(mvc_0_io_wgt_data_bits_3_2),
    .io_wgt_data_bits_3_3(mvc_0_io_wgt_data_bits_3_3),
    .io_wgt_data_bits_3_4(mvc_0_io_wgt_data_bits_3_4),
    .io_wgt_data_bits_3_5(mvc_0_io_wgt_data_bits_3_5),
    .io_wgt_data_bits_3_6(mvc_0_io_wgt_data_bits_3_6),
    .io_wgt_data_bits_3_7(mvc_0_io_wgt_data_bits_3_7),
    .io_wgt_data_bits_3_8(mvc_0_io_wgt_data_bits_3_8),
    .io_wgt_data_bits_3_9(mvc_0_io_wgt_data_bits_3_9),
    .io_wgt_data_bits_3_10(mvc_0_io_wgt_data_bits_3_10),
    .io_wgt_data_bits_3_11(mvc_0_io_wgt_data_bits_3_11),
    .io_wgt_data_bits_3_12(mvc_0_io_wgt_data_bits_3_12),
    .io_wgt_data_bits_3_13(mvc_0_io_wgt_data_bits_3_13),
    .io_wgt_data_bits_3_14(mvc_0_io_wgt_data_bits_3_14),
    .io_wgt_data_bits_3_15(mvc_0_io_wgt_data_bits_3_15),
    .io_wgt_data_bits_4_0(mvc_0_io_wgt_data_bits_4_0),
    .io_wgt_data_bits_4_1(mvc_0_io_wgt_data_bits_4_1),
    .io_wgt_data_bits_4_2(mvc_0_io_wgt_data_bits_4_2),
    .io_wgt_data_bits_4_3(mvc_0_io_wgt_data_bits_4_3),
    .io_wgt_data_bits_4_4(mvc_0_io_wgt_data_bits_4_4),
    .io_wgt_data_bits_4_5(mvc_0_io_wgt_data_bits_4_5),
    .io_wgt_data_bits_4_6(mvc_0_io_wgt_data_bits_4_6),
    .io_wgt_data_bits_4_7(mvc_0_io_wgt_data_bits_4_7),
    .io_wgt_data_bits_4_8(mvc_0_io_wgt_data_bits_4_8),
    .io_wgt_data_bits_4_9(mvc_0_io_wgt_data_bits_4_9),
    .io_wgt_data_bits_4_10(mvc_0_io_wgt_data_bits_4_10),
    .io_wgt_data_bits_4_11(mvc_0_io_wgt_data_bits_4_11),
    .io_wgt_data_bits_4_12(mvc_0_io_wgt_data_bits_4_12),
    .io_wgt_data_bits_4_13(mvc_0_io_wgt_data_bits_4_13),
    .io_wgt_data_bits_4_14(mvc_0_io_wgt_data_bits_4_14),
    .io_wgt_data_bits_4_15(mvc_0_io_wgt_data_bits_4_15),
    .io_wgt_data_bits_5_0(mvc_0_io_wgt_data_bits_5_0),
    .io_wgt_data_bits_5_1(mvc_0_io_wgt_data_bits_5_1),
    .io_wgt_data_bits_5_2(mvc_0_io_wgt_data_bits_5_2),
    .io_wgt_data_bits_5_3(mvc_0_io_wgt_data_bits_5_3),
    .io_wgt_data_bits_5_4(mvc_0_io_wgt_data_bits_5_4),
    .io_wgt_data_bits_5_5(mvc_0_io_wgt_data_bits_5_5),
    .io_wgt_data_bits_5_6(mvc_0_io_wgt_data_bits_5_6),
    .io_wgt_data_bits_5_7(mvc_0_io_wgt_data_bits_5_7),
    .io_wgt_data_bits_5_8(mvc_0_io_wgt_data_bits_5_8),
    .io_wgt_data_bits_5_9(mvc_0_io_wgt_data_bits_5_9),
    .io_wgt_data_bits_5_10(mvc_0_io_wgt_data_bits_5_10),
    .io_wgt_data_bits_5_11(mvc_0_io_wgt_data_bits_5_11),
    .io_wgt_data_bits_5_12(mvc_0_io_wgt_data_bits_5_12),
    .io_wgt_data_bits_5_13(mvc_0_io_wgt_data_bits_5_13),
    .io_wgt_data_bits_5_14(mvc_0_io_wgt_data_bits_5_14),
    .io_wgt_data_bits_5_15(mvc_0_io_wgt_data_bits_5_15),
    .io_wgt_data_bits_6_0(mvc_0_io_wgt_data_bits_6_0),
    .io_wgt_data_bits_6_1(mvc_0_io_wgt_data_bits_6_1),
    .io_wgt_data_bits_6_2(mvc_0_io_wgt_data_bits_6_2),
    .io_wgt_data_bits_6_3(mvc_0_io_wgt_data_bits_6_3),
    .io_wgt_data_bits_6_4(mvc_0_io_wgt_data_bits_6_4),
    .io_wgt_data_bits_6_5(mvc_0_io_wgt_data_bits_6_5),
    .io_wgt_data_bits_6_6(mvc_0_io_wgt_data_bits_6_6),
    .io_wgt_data_bits_6_7(mvc_0_io_wgt_data_bits_6_7),
    .io_wgt_data_bits_6_8(mvc_0_io_wgt_data_bits_6_8),
    .io_wgt_data_bits_6_9(mvc_0_io_wgt_data_bits_6_9),
    .io_wgt_data_bits_6_10(mvc_0_io_wgt_data_bits_6_10),
    .io_wgt_data_bits_6_11(mvc_0_io_wgt_data_bits_6_11),
    .io_wgt_data_bits_6_12(mvc_0_io_wgt_data_bits_6_12),
    .io_wgt_data_bits_6_13(mvc_0_io_wgt_data_bits_6_13),
    .io_wgt_data_bits_6_14(mvc_0_io_wgt_data_bits_6_14),
    .io_wgt_data_bits_6_15(mvc_0_io_wgt_data_bits_6_15),
    .io_wgt_data_bits_7_0(mvc_0_io_wgt_data_bits_7_0),
    .io_wgt_data_bits_7_1(mvc_0_io_wgt_data_bits_7_1),
    .io_wgt_data_bits_7_2(mvc_0_io_wgt_data_bits_7_2),
    .io_wgt_data_bits_7_3(mvc_0_io_wgt_data_bits_7_3),
    .io_wgt_data_bits_7_4(mvc_0_io_wgt_data_bits_7_4),
    .io_wgt_data_bits_7_5(mvc_0_io_wgt_data_bits_7_5),
    .io_wgt_data_bits_7_6(mvc_0_io_wgt_data_bits_7_6),
    .io_wgt_data_bits_7_7(mvc_0_io_wgt_data_bits_7_7),
    .io_wgt_data_bits_7_8(mvc_0_io_wgt_data_bits_7_8),
    .io_wgt_data_bits_7_9(mvc_0_io_wgt_data_bits_7_9),
    .io_wgt_data_bits_7_10(mvc_0_io_wgt_data_bits_7_10),
    .io_wgt_data_bits_7_11(mvc_0_io_wgt_data_bits_7_11),
    .io_wgt_data_bits_7_12(mvc_0_io_wgt_data_bits_7_12),
    .io_wgt_data_bits_7_13(mvc_0_io_wgt_data_bits_7_13),
    .io_wgt_data_bits_7_14(mvc_0_io_wgt_data_bits_7_14),
    .io_wgt_data_bits_7_15(mvc_0_io_wgt_data_bits_7_15),
    .io_wgt_data_bits_8_0(mvc_0_io_wgt_data_bits_8_0),
    .io_wgt_data_bits_8_1(mvc_0_io_wgt_data_bits_8_1),
    .io_wgt_data_bits_8_2(mvc_0_io_wgt_data_bits_8_2),
    .io_wgt_data_bits_8_3(mvc_0_io_wgt_data_bits_8_3),
    .io_wgt_data_bits_8_4(mvc_0_io_wgt_data_bits_8_4),
    .io_wgt_data_bits_8_5(mvc_0_io_wgt_data_bits_8_5),
    .io_wgt_data_bits_8_6(mvc_0_io_wgt_data_bits_8_6),
    .io_wgt_data_bits_8_7(mvc_0_io_wgt_data_bits_8_7),
    .io_wgt_data_bits_8_8(mvc_0_io_wgt_data_bits_8_8),
    .io_wgt_data_bits_8_9(mvc_0_io_wgt_data_bits_8_9),
    .io_wgt_data_bits_8_10(mvc_0_io_wgt_data_bits_8_10),
    .io_wgt_data_bits_8_11(mvc_0_io_wgt_data_bits_8_11),
    .io_wgt_data_bits_8_12(mvc_0_io_wgt_data_bits_8_12),
    .io_wgt_data_bits_8_13(mvc_0_io_wgt_data_bits_8_13),
    .io_wgt_data_bits_8_14(mvc_0_io_wgt_data_bits_8_14),
    .io_wgt_data_bits_8_15(mvc_0_io_wgt_data_bits_8_15),
    .io_wgt_data_bits_9_0(mvc_0_io_wgt_data_bits_9_0),
    .io_wgt_data_bits_9_1(mvc_0_io_wgt_data_bits_9_1),
    .io_wgt_data_bits_9_2(mvc_0_io_wgt_data_bits_9_2),
    .io_wgt_data_bits_9_3(mvc_0_io_wgt_data_bits_9_3),
    .io_wgt_data_bits_9_4(mvc_0_io_wgt_data_bits_9_4),
    .io_wgt_data_bits_9_5(mvc_0_io_wgt_data_bits_9_5),
    .io_wgt_data_bits_9_6(mvc_0_io_wgt_data_bits_9_6),
    .io_wgt_data_bits_9_7(mvc_0_io_wgt_data_bits_9_7),
    .io_wgt_data_bits_9_8(mvc_0_io_wgt_data_bits_9_8),
    .io_wgt_data_bits_9_9(mvc_0_io_wgt_data_bits_9_9),
    .io_wgt_data_bits_9_10(mvc_0_io_wgt_data_bits_9_10),
    .io_wgt_data_bits_9_11(mvc_0_io_wgt_data_bits_9_11),
    .io_wgt_data_bits_9_12(mvc_0_io_wgt_data_bits_9_12),
    .io_wgt_data_bits_9_13(mvc_0_io_wgt_data_bits_9_13),
    .io_wgt_data_bits_9_14(mvc_0_io_wgt_data_bits_9_14),
    .io_wgt_data_bits_9_15(mvc_0_io_wgt_data_bits_9_15),
    .io_wgt_data_bits_10_0(mvc_0_io_wgt_data_bits_10_0),
    .io_wgt_data_bits_10_1(mvc_0_io_wgt_data_bits_10_1),
    .io_wgt_data_bits_10_2(mvc_0_io_wgt_data_bits_10_2),
    .io_wgt_data_bits_10_3(mvc_0_io_wgt_data_bits_10_3),
    .io_wgt_data_bits_10_4(mvc_0_io_wgt_data_bits_10_4),
    .io_wgt_data_bits_10_5(mvc_0_io_wgt_data_bits_10_5),
    .io_wgt_data_bits_10_6(mvc_0_io_wgt_data_bits_10_6),
    .io_wgt_data_bits_10_7(mvc_0_io_wgt_data_bits_10_7),
    .io_wgt_data_bits_10_8(mvc_0_io_wgt_data_bits_10_8),
    .io_wgt_data_bits_10_9(mvc_0_io_wgt_data_bits_10_9),
    .io_wgt_data_bits_10_10(mvc_0_io_wgt_data_bits_10_10),
    .io_wgt_data_bits_10_11(mvc_0_io_wgt_data_bits_10_11),
    .io_wgt_data_bits_10_12(mvc_0_io_wgt_data_bits_10_12),
    .io_wgt_data_bits_10_13(mvc_0_io_wgt_data_bits_10_13),
    .io_wgt_data_bits_10_14(mvc_0_io_wgt_data_bits_10_14),
    .io_wgt_data_bits_10_15(mvc_0_io_wgt_data_bits_10_15),
    .io_wgt_data_bits_11_0(mvc_0_io_wgt_data_bits_11_0),
    .io_wgt_data_bits_11_1(mvc_0_io_wgt_data_bits_11_1),
    .io_wgt_data_bits_11_2(mvc_0_io_wgt_data_bits_11_2),
    .io_wgt_data_bits_11_3(mvc_0_io_wgt_data_bits_11_3),
    .io_wgt_data_bits_11_4(mvc_0_io_wgt_data_bits_11_4),
    .io_wgt_data_bits_11_5(mvc_0_io_wgt_data_bits_11_5),
    .io_wgt_data_bits_11_6(mvc_0_io_wgt_data_bits_11_6),
    .io_wgt_data_bits_11_7(mvc_0_io_wgt_data_bits_11_7),
    .io_wgt_data_bits_11_8(mvc_0_io_wgt_data_bits_11_8),
    .io_wgt_data_bits_11_9(mvc_0_io_wgt_data_bits_11_9),
    .io_wgt_data_bits_11_10(mvc_0_io_wgt_data_bits_11_10),
    .io_wgt_data_bits_11_11(mvc_0_io_wgt_data_bits_11_11),
    .io_wgt_data_bits_11_12(mvc_0_io_wgt_data_bits_11_12),
    .io_wgt_data_bits_11_13(mvc_0_io_wgt_data_bits_11_13),
    .io_wgt_data_bits_11_14(mvc_0_io_wgt_data_bits_11_14),
    .io_wgt_data_bits_11_15(mvc_0_io_wgt_data_bits_11_15),
    .io_wgt_data_bits_12_0(mvc_0_io_wgt_data_bits_12_0),
    .io_wgt_data_bits_12_1(mvc_0_io_wgt_data_bits_12_1),
    .io_wgt_data_bits_12_2(mvc_0_io_wgt_data_bits_12_2),
    .io_wgt_data_bits_12_3(mvc_0_io_wgt_data_bits_12_3),
    .io_wgt_data_bits_12_4(mvc_0_io_wgt_data_bits_12_4),
    .io_wgt_data_bits_12_5(mvc_0_io_wgt_data_bits_12_5),
    .io_wgt_data_bits_12_6(mvc_0_io_wgt_data_bits_12_6),
    .io_wgt_data_bits_12_7(mvc_0_io_wgt_data_bits_12_7),
    .io_wgt_data_bits_12_8(mvc_0_io_wgt_data_bits_12_8),
    .io_wgt_data_bits_12_9(mvc_0_io_wgt_data_bits_12_9),
    .io_wgt_data_bits_12_10(mvc_0_io_wgt_data_bits_12_10),
    .io_wgt_data_bits_12_11(mvc_0_io_wgt_data_bits_12_11),
    .io_wgt_data_bits_12_12(mvc_0_io_wgt_data_bits_12_12),
    .io_wgt_data_bits_12_13(mvc_0_io_wgt_data_bits_12_13),
    .io_wgt_data_bits_12_14(mvc_0_io_wgt_data_bits_12_14),
    .io_wgt_data_bits_12_15(mvc_0_io_wgt_data_bits_12_15),
    .io_wgt_data_bits_13_0(mvc_0_io_wgt_data_bits_13_0),
    .io_wgt_data_bits_13_1(mvc_0_io_wgt_data_bits_13_1),
    .io_wgt_data_bits_13_2(mvc_0_io_wgt_data_bits_13_2),
    .io_wgt_data_bits_13_3(mvc_0_io_wgt_data_bits_13_3),
    .io_wgt_data_bits_13_4(mvc_0_io_wgt_data_bits_13_4),
    .io_wgt_data_bits_13_5(mvc_0_io_wgt_data_bits_13_5),
    .io_wgt_data_bits_13_6(mvc_0_io_wgt_data_bits_13_6),
    .io_wgt_data_bits_13_7(mvc_0_io_wgt_data_bits_13_7),
    .io_wgt_data_bits_13_8(mvc_0_io_wgt_data_bits_13_8),
    .io_wgt_data_bits_13_9(mvc_0_io_wgt_data_bits_13_9),
    .io_wgt_data_bits_13_10(mvc_0_io_wgt_data_bits_13_10),
    .io_wgt_data_bits_13_11(mvc_0_io_wgt_data_bits_13_11),
    .io_wgt_data_bits_13_12(mvc_0_io_wgt_data_bits_13_12),
    .io_wgt_data_bits_13_13(mvc_0_io_wgt_data_bits_13_13),
    .io_wgt_data_bits_13_14(mvc_0_io_wgt_data_bits_13_14),
    .io_wgt_data_bits_13_15(mvc_0_io_wgt_data_bits_13_15),
    .io_wgt_data_bits_14_0(mvc_0_io_wgt_data_bits_14_0),
    .io_wgt_data_bits_14_1(mvc_0_io_wgt_data_bits_14_1),
    .io_wgt_data_bits_14_2(mvc_0_io_wgt_data_bits_14_2),
    .io_wgt_data_bits_14_3(mvc_0_io_wgt_data_bits_14_3),
    .io_wgt_data_bits_14_4(mvc_0_io_wgt_data_bits_14_4),
    .io_wgt_data_bits_14_5(mvc_0_io_wgt_data_bits_14_5),
    .io_wgt_data_bits_14_6(mvc_0_io_wgt_data_bits_14_6),
    .io_wgt_data_bits_14_7(mvc_0_io_wgt_data_bits_14_7),
    .io_wgt_data_bits_14_8(mvc_0_io_wgt_data_bits_14_8),
    .io_wgt_data_bits_14_9(mvc_0_io_wgt_data_bits_14_9),
    .io_wgt_data_bits_14_10(mvc_0_io_wgt_data_bits_14_10),
    .io_wgt_data_bits_14_11(mvc_0_io_wgt_data_bits_14_11),
    .io_wgt_data_bits_14_12(mvc_0_io_wgt_data_bits_14_12),
    .io_wgt_data_bits_14_13(mvc_0_io_wgt_data_bits_14_13),
    .io_wgt_data_bits_14_14(mvc_0_io_wgt_data_bits_14_14),
    .io_wgt_data_bits_14_15(mvc_0_io_wgt_data_bits_14_15),
    .io_wgt_data_bits_15_0(mvc_0_io_wgt_data_bits_15_0),
    .io_wgt_data_bits_15_1(mvc_0_io_wgt_data_bits_15_1),
    .io_wgt_data_bits_15_2(mvc_0_io_wgt_data_bits_15_2),
    .io_wgt_data_bits_15_3(mvc_0_io_wgt_data_bits_15_3),
    .io_wgt_data_bits_15_4(mvc_0_io_wgt_data_bits_15_4),
    .io_wgt_data_bits_15_5(mvc_0_io_wgt_data_bits_15_5),
    .io_wgt_data_bits_15_6(mvc_0_io_wgt_data_bits_15_6),
    .io_wgt_data_bits_15_7(mvc_0_io_wgt_data_bits_15_7),
    .io_wgt_data_bits_15_8(mvc_0_io_wgt_data_bits_15_8),
    .io_wgt_data_bits_15_9(mvc_0_io_wgt_data_bits_15_9),
    .io_wgt_data_bits_15_10(mvc_0_io_wgt_data_bits_15_10),
    .io_wgt_data_bits_15_11(mvc_0_io_wgt_data_bits_15_11),
    .io_wgt_data_bits_15_12(mvc_0_io_wgt_data_bits_15_12),
    .io_wgt_data_bits_15_13(mvc_0_io_wgt_data_bits_15_13),
    .io_wgt_data_bits_15_14(mvc_0_io_wgt_data_bits_15_14),
    .io_wgt_data_bits_15_15(mvc_0_io_wgt_data_bits_15_15),
    .io_acc_i_data_valid(mvc_0_io_acc_i_data_valid),
    .io_acc_i_data_bits_0_0(mvc_0_io_acc_i_data_bits_0_0),
    .io_acc_i_data_bits_0_1(mvc_0_io_acc_i_data_bits_0_1),
    .io_acc_i_data_bits_0_2(mvc_0_io_acc_i_data_bits_0_2),
    .io_acc_i_data_bits_0_3(mvc_0_io_acc_i_data_bits_0_3),
    .io_acc_i_data_bits_0_4(mvc_0_io_acc_i_data_bits_0_4),
    .io_acc_i_data_bits_0_5(mvc_0_io_acc_i_data_bits_0_5),
    .io_acc_i_data_bits_0_6(mvc_0_io_acc_i_data_bits_0_6),
    .io_acc_i_data_bits_0_7(mvc_0_io_acc_i_data_bits_0_7),
    .io_acc_i_data_bits_0_8(mvc_0_io_acc_i_data_bits_0_8),
    .io_acc_i_data_bits_0_9(mvc_0_io_acc_i_data_bits_0_9),
    .io_acc_i_data_bits_0_10(mvc_0_io_acc_i_data_bits_0_10),
    .io_acc_i_data_bits_0_11(mvc_0_io_acc_i_data_bits_0_11),
    .io_acc_i_data_bits_0_12(mvc_0_io_acc_i_data_bits_0_12),
    .io_acc_i_data_bits_0_13(mvc_0_io_acc_i_data_bits_0_13),
    .io_acc_i_data_bits_0_14(mvc_0_io_acc_i_data_bits_0_14),
    .io_acc_i_data_bits_0_15(mvc_0_io_acc_i_data_bits_0_15),
    .io_acc_o_data_valid(mvc_0_io_acc_o_data_valid),
    .io_acc_o_data_bits_0_0(mvc_0_io_acc_o_data_bits_0_0),
    .io_acc_o_data_bits_0_1(mvc_0_io_acc_o_data_bits_0_1),
    .io_acc_o_data_bits_0_2(mvc_0_io_acc_o_data_bits_0_2),
    .io_acc_o_data_bits_0_3(mvc_0_io_acc_o_data_bits_0_3),
    .io_acc_o_data_bits_0_4(mvc_0_io_acc_o_data_bits_0_4),
    .io_acc_o_data_bits_0_5(mvc_0_io_acc_o_data_bits_0_5),
    .io_acc_o_data_bits_0_6(mvc_0_io_acc_o_data_bits_0_6),
    .io_acc_o_data_bits_0_7(mvc_0_io_acc_o_data_bits_0_7),
    .io_acc_o_data_bits_0_8(mvc_0_io_acc_o_data_bits_0_8),
    .io_acc_o_data_bits_0_9(mvc_0_io_acc_o_data_bits_0_9),
    .io_acc_o_data_bits_0_10(mvc_0_io_acc_o_data_bits_0_10),
    .io_acc_o_data_bits_0_11(mvc_0_io_acc_o_data_bits_0_11),
    .io_acc_o_data_bits_0_12(mvc_0_io_acc_o_data_bits_0_12),
    .io_acc_o_data_bits_0_13(mvc_0_io_acc_o_data_bits_0_13),
    .io_acc_o_data_bits_0_14(mvc_0_io_acc_o_data_bits_0_14),
    .io_acc_o_data_bits_0_15(mvc_0_io_acc_o_data_bits_0_15),
    .io_out_data_valid(mvc_0_io_out_data_valid),
    .io_out_data_bits_0_0(mvc_0_io_out_data_bits_0_0),
    .io_out_data_bits_0_1(mvc_0_io_out_data_bits_0_1),
    .io_out_data_bits_0_2(mvc_0_io_out_data_bits_0_2),
    .io_out_data_bits_0_3(mvc_0_io_out_data_bits_0_3),
    .io_out_data_bits_0_4(mvc_0_io_out_data_bits_0_4),
    .io_out_data_bits_0_5(mvc_0_io_out_data_bits_0_5),
    .io_out_data_bits_0_6(mvc_0_io_out_data_bits_0_6),
    .io_out_data_bits_0_7(mvc_0_io_out_data_bits_0_7),
    .io_out_data_bits_0_8(mvc_0_io_out_data_bits_0_8),
    .io_out_data_bits_0_9(mvc_0_io_out_data_bits_0_9),
    .io_out_data_bits_0_10(mvc_0_io_out_data_bits_0_10),
    .io_out_data_bits_0_11(mvc_0_io_out_data_bits_0_11),
    .io_out_data_bits_0_12(mvc_0_io_out_data_bits_0_12),
    .io_out_data_bits_0_13(mvc_0_io_out_data_bits_0_13),
    .io_out_data_bits_0_14(mvc_0_io_out_data_bits_0_14),
    .io_out_data_bits_0_15(mvc_0_io_out_data_bits_0_15),
    .io_bypass_cond(mvc_0_io_bypass_cond)
  );
  Pipe_1 wrpipe2 ( // @[TensorGemm.scala 691:25]
    .clock(wrpipe2_clock),
    .reset(wrpipe2_reset),
    .io_enq_valid(wrpipe2_io_enq_valid),
    .io_enq_bits(wrpipe2_io_enq_bits),
    .io_deq_valid(wrpipe2_io_deq_valid),
    .io_deq_bits(wrpipe2_io_deq_bits)
  );
  assign io_done = state == 2'h0 & io_start ? 1'h0 : _GEN_7; // @[TensorGemm.scala 572:37 TensorGemm.scala 571:11]
  assign io_uop_idx_valid = m_io_valid; // @[TensorGemm.scala 592:20]
  assign io_uop_idx_bits = m_io_uop_idx; // @[TensorGemm.scala 591:19]
  assign io_inp_rd_0_idx_valid = delayed_valid; // @[TensorGemm.scala 616:26]
  assign io_inp_rd_0_idx_bits = io_uop_data_bits_u1 + delayed_inp_i; // @[TensorGemm.scala 600:41]
  assign io_wgt_rd_0_idx_valid = delayed_valid; // @[TensorGemm.scala 627:30]
  assign io_wgt_rd_0_idx_bits = io_uop_data_bits_u2 + delayed_wgt_i; // @[TensorGemm.scala 601:54]
  assign io_acc_rd_0_idx_valid = io_acc_rd_0_idx_valid_REG; // @[TensorGemm.scala 623:30]
  assign io_acc_rd_0_idx_bits = io_acc_rd_0_idx_bits_REG; // @[TensorGemm.scala 624:29]
  assign io_acc_wr_0_valid = wrpipe_0_io_deq_valid; // @[TensorGemm.scala 723:27]
  assign io_acc_wr_0_bits_idx = wrpipe_0_io_deq_bits; // @[TensorGemm.scala 724:30]
  assign io_acc_wr_0_bits_data_0_0 = mvc_0_io_acc_o_data_bits_0_0; // @[TensorGemm.scala 718:62]
  assign io_acc_wr_0_bits_data_0_1 = mvc_0_io_acc_o_data_bits_0_1; // @[TensorGemm.scala 718:62]
  assign io_acc_wr_0_bits_data_0_2 = mvc_0_io_acc_o_data_bits_0_2; // @[TensorGemm.scala 718:62]
  assign io_acc_wr_0_bits_data_0_3 = mvc_0_io_acc_o_data_bits_0_3; // @[TensorGemm.scala 718:62]
  assign io_acc_wr_0_bits_data_0_4 = mvc_0_io_acc_o_data_bits_0_4; // @[TensorGemm.scala 718:62]
  assign io_acc_wr_0_bits_data_0_5 = mvc_0_io_acc_o_data_bits_0_5; // @[TensorGemm.scala 718:62]
  assign io_acc_wr_0_bits_data_0_6 = mvc_0_io_acc_o_data_bits_0_6; // @[TensorGemm.scala 718:62]
  assign io_acc_wr_0_bits_data_0_7 = mvc_0_io_acc_o_data_bits_0_7; // @[TensorGemm.scala 718:62]
  assign io_acc_wr_0_bits_data_0_8 = mvc_0_io_acc_o_data_bits_0_8; // @[TensorGemm.scala 718:62]
  assign io_acc_wr_0_bits_data_0_9 = mvc_0_io_acc_o_data_bits_0_9; // @[TensorGemm.scala 718:62]
  assign io_acc_wr_0_bits_data_0_10 = mvc_0_io_acc_o_data_bits_0_10; // @[TensorGemm.scala 718:62]
  assign io_acc_wr_0_bits_data_0_11 = mvc_0_io_acc_o_data_bits_0_11; // @[TensorGemm.scala 718:62]
  assign io_acc_wr_0_bits_data_0_12 = mvc_0_io_acc_o_data_bits_0_12; // @[TensorGemm.scala 718:62]
  assign io_acc_wr_0_bits_data_0_13 = mvc_0_io_acc_o_data_bits_0_13; // @[TensorGemm.scala 718:62]
  assign io_acc_wr_0_bits_data_0_14 = mvc_0_io_acc_o_data_bits_0_14; // @[TensorGemm.scala 718:62]
  assign io_acc_wr_0_bits_data_0_15 = mvc_0_io_acc_o_data_bits_0_15; // @[TensorGemm.scala 718:62]
  assign io_out_wr_0_valid = wrpipeNs_io_deq_valid & mvc_0_io_out_data_valid; // @[TensorGemm.scala 742:47]
  assign io_out_wr_0_bits_idx = wrpipeNs_io_deq_bits; // @[TensorGemm.scala 743:25]
  assign io_out_wr_0_bits_data_0_0 = mvc_0_io_out_data_bits_0_0; // @[TensorGemm.scala 733:21 TensorGemm.scala 737:63]
  assign io_out_wr_0_bits_data_0_1 = mvc_0_io_out_data_bits_0_1; // @[TensorGemm.scala 733:21 TensorGemm.scala 737:63]
  assign io_out_wr_0_bits_data_0_2 = mvc_0_io_out_data_bits_0_2; // @[TensorGemm.scala 733:21 TensorGemm.scala 737:63]
  assign io_out_wr_0_bits_data_0_3 = mvc_0_io_out_data_bits_0_3; // @[TensorGemm.scala 733:21 TensorGemm.scala 737:63]
  assign io_out_wr_0_bits_data_0_4 = mvc_0_io_out_data_bits_0_4; // @[TensorGemm.scala 733:21 TensorGemm.scala 737:63]
  assign io_out_wr_0_bits_data_0_5 = mvc_0_io_out_data_bits_0_5; // @[TensorGemm.scala 733:21 TensorGemm.scala 737:63]
  assign io_out_wr_0_bits_data_0_6 = mvc_0_io_out_data_bits_0_6; // @[TensorGemm.scala 733:21 TensorGemm.scala 737:63]
  assign io_out_wr_0_bits_data_0_7 = mvc_0_io_out_data_bits_0_7; // @[TensorGemm.scala 733:21 TensorGemm.scala 737:63]
  assign io_out_wr_0_bits_data_0_8 = mvc_0_io_out_data_bits_0_8; // @[TensorGemm.scala 733:21 TensorGemm.scala 737:63]
  assign io_out_wr_0_bits_data_0_9 = mvc_0_io_out_data_bits_0_9; // @[TensorGemm.scala 733:21 TensorGemm.scala 737:63]
  assign io_out_wr_0_bits_data_0_10 = mvc_0_io_out_data_bits_0_10; // @[TensorGemm.scala 733:21 TensorGemm.scala 737:63]
  assign io_out_wr_0_bits_data_0_11 = mvc_0_io_out_data_bits_0_11; // @[TensorGemm.scala 733:21 TensorGemm.scala 737:63]
  assign io_out_wr_0_bits_data_0_12 = mvc_0_io_out_data_bits_0_12; // @[TensorGemm.scala 733:21 TensorGemm.scala 737:63]
  assign io_out_wr_0_bits_data_0_13 = mvc_0_io_out_data_bits_0_13; // @[TensorGemm.scala 733:21 TensorGemm.scala 737:63]
  assign io_out_wr_0_bits_data_0_14 = mvc_0_io_out_data_bits_0_14; // @[TensorGemm.scala 733:21 TensorGemm.scala 737:63]
  assign io_out_wr_0_bits_data_0_15 = mvc_0_io_out_data_bits_0_15; // @[TensorGemm.scala 733:21 TensorGemm.scala 737:63]
  assign m_clock = clock;
  assign m_reset = reset;
  assign m_io_start = io_start; // @[TensorGemm.scala 588:14]
  assign m_io_dec_wgt_1 = io_dec_wgt_1; // @[TensorGemm.scala 590:12]
  assign m_io_dec_wgt_0 = io_dec_wgt_0; // @[TensorGemm.scala 590:12]
  assign m_io_dec_inp_1 = io_dec_inp_1; // @[TensorGemm.scala 590:12]
  assign m_io_dec_inp_0 = io_dec_inp_0; // @[TensorGemm.scala 590:12]
  assign m_io_dec_acc_1 = io_dec_acc_1; // @[TensorGemm.scala 590:12]
  assign m_io_dec_acc_0 = io_dec_acc_0; // @[TensorGemm.scala 590:12]
  assign m_io_dec_lp_1 = io_dec_lp_1; // @[TensorGemm.scala 590:12]
  assign m_io_dec_lp_0 = io_dec_lp_0; // @[TensorGemm.scala 590:12]
  assign m_io_dec_uop_end = io_dec_uop_end; // @[TensorGemm.scala 590:12]
  assign m_io_dec_uop_begin = io_dec_uop_begin; // @[TensorGemm.scala 590:12]
  assign reset_pipe_clock = clock;
  assign reset_pipe_reset = reset;
  assign reset_pipe_io_enq_valid = m_io_valid; // @[TensorGemm.scala 607:27]
  assign reset_pipe_io_enq_bits = capture_dec_reset; // @[TensorGemm.scala 608:26]
  assign acc_idx_pipe_clock = clock;
  assign acc_idx_pipe_reset = reset;
  assign acc_idx_pipe_io_enq_valid = delayed_valid; // @[TensorGemm.scala 612:29]
  assign acc_idx_pipe_io_enq_bits = io_uop_data_bits_u0 + delayed_acc_i; // @[TensorGemm.scala 599:54]
  assign wrpipe0_clock = clock;
  assign wrpipe0_reset = reset;
  assign wrpipe0_io_enq_valid = delayed_valid; // @[TensorGemm.scala 638:24]
  assign wrpipe0_io_enq_bits = io_uop_data_bits_u0 + delayed_acc_i; // @[TensorGemm.scala 599:54]
  assign wrpipeNs_clock = clock;
  assign wrpipeNs_reset = reset;
  assign wrpipeNs_io_enq_valid = wrpipe0_io_deq_valid; // @[TensorGemm.scala 642:19]
  assign wrpipeNs_io_enq_bits = wrpipe0_io_deq_bits; // @[TensorGemm.scala 642:19]
  assign wrpipe_0_clock = clock;
  assign wrpipe_0_reset = reset;
  assign wrpipe_0_io_enq_valid = wrpipe0_io_deq_valid; // @[TensorGemm.scala 646:17]
  assign wrpipe_0_io_enq_bits = wrpipe0_io_deq_bits; // @[TensorGemm.scala 646:17]
  assign mvc_0_clock = clock;
  assign mvc_0_io_valid_reset = mvc_0_io_valid_reset_REG; // @[TensorGemm.scala 698:30]
  assign mvc_0_io_inp_data_bits_0_0 = io_inp_rd_0_data_bits_0_0; // @[TensorGemm.scala 700:27]
  assign mvc_0_io_inp_data_bits_0_1 = io_inp_rd_0_data_bits_0_1; // @[TensorGemm.scala 700:27]
  assign mvc_0_io_inp_data_bits_0_2 = io_inp_rd_0_data_bits_0_2; // @[TensorGemm.scala 700:27]
  assign mvc_0_io_inp_data_bits_0_3 = io_inp_rd_0_data_bits_0_3; // @[TensorGemm.scala 700:27]
  assign mvc_0_io_inp_data_bits_0_4 = io_inp_rd_0_data_bits_0_4; // @[TensorGemm.scala 700:27]
  assign mvc_0_io_inp_data_bits_0_5 = io_inp_rd_0_data_bits_0_5; // @[TensorGemm.scala 700:27]
  assign mvc_0_io_inp_data_bits_0_6 = io_inp_rd_0_data_bits_0_6; // @[TensorGemm.scala 700:27]
  assign mvc_0_io_inp_data_bits_0_7 = io_inp_rd_0_data_bits_0_7; // @[TensorGemm.scala 700:27]
  assign mvc_0_io_inp_data_bits_0_8 = io_inp_rd_0_data_bits_0_8; // @[TensorGemm.scala 700:27]
  assign mvc_0_io_inp_data_bits_0_9 = io_inp_rd_0_data_bits_0_9; // @[TensorGemm.scala 700:27]
  assign mvc_0_io_inp_data_bits_0_10 = io_inp_rd_0_data_bits_0_10; // @[TensorGemm.scala 700:27]
  assign mvc_0_io_inp_data_bits_0_11 = io_inp_rd_0_data_bits_0_11; // @[TensorGemm.scala 700:27]
  assign mvc_0_io_inp_data_bits_0_12 = io_inp_rd_0_data_bits_0_12; // @[TensorGemm.scala 700:27]
  assign mvc_0_io_inp_data_bits_0_13 = io_inp_rd_0_data_bits_0_13; // @[TensorGemm.scala 700:27]
  assign mvc_0_io_inp_data_bits_0_14 = io_inp_rd_0_data_bits_0_14; // @[TensorGemm.scala 700:27]
  assign mvc_0_io_inp_data_bits_0_15 = io_inp_rd_0_data_bits_0_15; // @[TensorGemm.scala 700:27]
  assign mvc_0_io_wgt_data_bits_0_0 = io_wgt_rd_0_data_bits_0_0; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_0_1 = io_wgt_rd_0_data_bits_0_1; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_0_2 = io_wgt_rd_0_data_bits_0_2; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_0_3 = io_wgt_rd_0_data_bits_0_3; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_0_4 = io_wgt_rd_0_data_bits_0_4; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_0_5 = io_wgt_rd_0_data_bits_0_5; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_0_6 = io_wgt_rd_0_data_bits_0_6; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_0_7 = io_wgt_rd_0_data_bits_0_7; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_0_8 = io_wgt_rd_0_data_bits_0_8; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_0_9 = io_wgt_rd_0_data_bits_0_9; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_0_10 = io_wgt_rd_0_data_bits_0_10; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_0_11 = io_wgt_rd_0_data_bits_0_11; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_0_12 = io_wgt_rd_0_data_bits_0_12; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_0_13 = io_wgt_rd_0_data_bits_0_13; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_0_14 = io_wgt_rd_0_data_bits_0_14; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_0_15 = io_wgt_rd_0_data_bits_0_15; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_1_0 = io_wgt_rd_0_data_bits_1_0; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_1_1 = io_wgt_rd_0_data_bits_1_1; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_1_2 = io_wgt_rd_0_data_bits_1_2; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_1_3 = io_wgt_rd_0_data_bits_1_3; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_1_4 = io_wgt_rd_0_data_bits_1_4; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_1_5 = io_wgt_rd_0_data_bits_1_5; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_1_6 = io_wgt_rd_0_data_bits_1_6; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_1_7 = io_wgt_rd_0_data_bits_1_7; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_1_8 = io_wgt_rd_0_data_bits_1_8; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_1_9 = io_wgt_rd_0_data_bits_1_9; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_1_10 = io_wgt_rd_0_data_bits_1_10; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_1_11 = io_wgt_rd_0_data_bits_1_11; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_1_12 = io_wgt_rd_0_data_bits_1_12; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_1_13 = io_wgt_rd_0_data_bits_1_13; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_1_14 = io_wgt_rd_0_data_bits_1_14; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_1_15 = io_wgt_rd_0_data_bits_1_15; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_2_0 = io_wgt_rd_0_data_bits_2_0; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_2_1 = io_wgt_rd_0_data_bits_2_1; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_2_2 = io_wgt_rd_0_data_bits_2_2; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_2_3 = io_wgt_rd_0_data_bits_2_3; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_2_4 = io_wgt_rd_0_data_bits_2_4; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_2_5 = io_wgt_rd_0_data_bits_2_5; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_2_6 = io_wgt_rd_0_data_bits_2_6; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_2_7 = io_wgt_rd_0_data_bits_2_7; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_2_8 = io_wgt_rd_0_data_bits_2_8; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_2_9 = io_wgt_rd_0_data_bits_2_9; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_2_10 = io_wgt_rd_0_data_bits_2_10; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_2_11 = io_wgt_rd_0_data_bits_2_11; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_2_12 = io_wgt_rd_0_data_bits_2_12; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_2_13 = io_wgt_rd_0_data_bits_2_13; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_2_14 = io_wgt_rd_0_data_bits_2_14; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_2_15 = io_wgt_rd_0_data_bits_2_15; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_3_0 = io_wgt_rd_0_data_bits_3_0; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_3_1 = io_wgt_rd_0_data_bits_3_1; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_3_2 = io_wgt_rd_0_data_bits_3_2; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_3_3 = io_wgt_rd_0_data_bits_3_3; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_3_4 = io_wgt_rd_0_data_bits_3_4; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_3_5 = io_wgt_rd_0_data_bits_3_5; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_3_6 = io_wgt_rd_0_data_bits_3_6; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_3_7 = io_wgt_rd_0_data_bits_3_7; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_3_8 = io_wgt_rd_0_data_bits_3_8; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_3_9 = io_wgt_rd_0_data_bits_3_9; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_3_10 = io_wgt_rd_0_data_bits_3_10; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_3_11 = io_wgt_rd_0_data_bits_3_11; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_3_12 = io_wgt_rd_0_data_bits_3_12; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_3_13 = io_wgt_rd_0_data_bits_3_13; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_3_14 = io_wgt_rd_0_data_bits_3_14; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_3_15 = io_wgt_rd_0_data_bits_3_15; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_4_0 = io_wgt_rd_0_data_bits_4_0; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_4_1 = io_wgt_rd_0_data_bits_4_1; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_4_2 = io_wgt_rd_0_data_bits_4_2; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_4_3 = io_wgt_rd_0_data_bits_4_3; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_4_4 = io_wgt_rd_0_data_bits_4_4; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_4_5 = io_wgt_rd_0_data_bits_4_5; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_4_6 = io_wgt_rd_0_data_bits_4_6; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_4_7 = io_wgt_rd_0_data_bits_4_7; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_4_8 = io_wgt_rd_0_data_bits_4_8; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_4_9 = io_wgt_rd_0_data_bits_4_9; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_4_10 = io_wgt_rd_0_data_bits_4_10; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_4_11 = io_wgt_rd_0_data_bits_4_11; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_4_12 = io_wgt_rd_0_data_bits_4_12; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_4_13 = io_wgt_rd_0_data_bits_4_13; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_4_14 = io_wgt_rd_0_data_bits_4_14; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_4_15 = io_wgt_rd_0_data_bits_4_15; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_5_0 = io_wgt_rd_0_data_bits_5_0; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_5_1 = io_wgt_rd_0_data_bits_5_1; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_5_2 = io_wgt_rd_0_data_bits_5_2; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_5_3 = io_wgt_rd_0_data_bits_5_3; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_5_4 = io_wgt_rd_0_data_bits_5_4; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_5_5 = io_wgt_rd_0_data_bits_5_5; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_5_6 = io_wgt_rd_0_data_bits_5_6; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_5_7 = io_wgt_rd_0_data_bits_5_7; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_5_8 = io_wgt_rd_0_data_bits_5_8; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_5_9 = io_wgt_rd_0_data_bits_5_9; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_5_10 = io_wgt_rd_0_data_bits_5_10; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_5_11 = io_wgt_rd_0_data_bits_5_11; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_5_12 = io_wgt_rd_0_data_bits_5_12; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_5_13 = io_wgt_rd_0_data_bits_5_13; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_5_14 = io_wgt_rd_0_data_bits_5_14; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_5_15 = io_wgt_rd_0_data_bits_5_15; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_6_0 = io_wgt_rd_0_data_bits_6_0; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_6_1 = io_wgt_rd_0_data_bits_6_1; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_6_2 = io_wgt_rd_0_data_bits_6_2; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_6_3 = io_wgt_rd_0_data_bits_6_3; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_6_4 = io_wgt_rd_0_data_bits_6_4; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_6_5 = io_wgt_rd_0_data_bits_6_5; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_6_6 = io_wgt_rd_0_data_bits_6_6; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_6_7 = io_wgt_rd_0_data_bits_6_7; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_6_8 = io_wgt_rd_0_data_bits_6_8; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_6_9 = io_wgt_rd_0_data_bits_6_9; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_6_10 = io_wgt_rd_0_data_bits_6_10; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_6_11 = io_wgt_rd_0_data_bits_6_11; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_6_12 = io_wgt_rd_0_data_bits_6_12; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_6_13 = io_wgt_rd_0_data_bits_6_13; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_6_14 = io_wgt_rd_0_data_bits_6_14; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_6_15 = io_wgt_rd_0_data_bits_6_15; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_7_0 = io_wgt_rd_0_data_bits_7_0; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_7_1 = io_wgt_rd_0_data_bits_7_1; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_7_2 = io_wgt_rd_0_data_bits_7_2; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_7_3 = io_wgt_rd_0_data_bits_7_3; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_7_4 = io_wgt_rd_0_data_bits_7_4; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_7_5 = io_wgt_rd_0_data_bits_7_5; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_7_6 = io_wgt_rd_0_data_bits_7_6; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_7_7 = io_wgt_rd_0_data_bits_7_7; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_7_8 = io_wgt_rd_0_data_bits_7_8; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_7_9 = io_wgt_rd_0_data_bits_7_9; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_7_10 = io_wgt_rd_0_data_bits_7_10; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_7_11 = io_wgt_rd_0_data_bits_7_11; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_7_12 = io_wgt_rd_0_data_bits_7_12; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_7_13 = io_wgt_rd_0_data_bits_7_13; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_7_14 = io_wgt_rd_0_data_bits_7_14; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_7_15 = io_wgt_rd_0_data_bits_7_15; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_8_0 = io_wgt_rd_0_data_bits_8_0; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_8_1 = io_wgt_rd_0_data_bits_8_1; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_8_2 = io_wgt_rd_0_data_bits_8_2; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_8_3 = io_wgt_rd_0_data_bits_8_3; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_8_4 = io_wgt_rd_0_data_bits_8_4; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_8_5 = io_wgt_rd_0_data_bits_8_5; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_8_6 = io_wgt_rd_0_data_bits_8_6; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_8_7 = io_wgt_rd_0_data_bits_8_7; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_8_8 = io_wgt_rd_0_data_bits_8_8; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_8_9 = io_wgt_rd_0_data_bits_8_9; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_8_10 = io_wgt_rd_0_data_bits_8_10; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_8_11 = io_wgt_rd_0_data_bits_8_11; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_8_12 = io_wgt_rd_0_data_bits_8_12; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_8_13 = io_wgt_rd_0_data_bits_8_13; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_8_14 = io_wgt_rd_0_data_bits_8_14; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_8_15 = io_wgt_rd_0_data_bits_8_15; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_9_0 = io_wgt_rd_0_data_bits_9_0; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_9_1 = io_wgt_rd_0_data_bits_9_1; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_9_2 = io_wgt_rd_0_data_bits_9_2; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_9_3 = io_wgt_rd_0_data_bits_9_3; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_9_4 = io_wgt_rd_0_data_bits_9_4; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_9_5 = io_wgt_rd_0_data_bits_9_5; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_9_6 = io_wgt_rd_0_data_bits_9_6; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_9_7 = io_wgt_rd_0_data_bits_9_7; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_9_8 = io_wgt_rd_0_data_bits_9_8; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_9_9 = io_wgt_rd_0_data_bits_9_9; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_9_10 = io_wgt_rd_0_data_bits_9_10; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_9_11 = io_wgt_rd_0_data_bits_9_11; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_9_12 = io_wgt_rd_0_data_bits_9_12; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_9_13 = io_wgt_rd_0_data_bits_9_13; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_9_14 = io_wgt_rd_0_data_bits_9_14; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_9_15 = io_wgt_rd_0_data_bits_9_15; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_10_0 = io_wgt_rd_0_data_bits_10_0; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_10_1 = io_wgt_rd_0_data_bits_10_1; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_10_2 = io_wgt_rd_0_data_bits_10_2; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_10_3 = io_wgt_rd_0_data_bits_10_3; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_10_4 = io_wgt_rd_0_data_bits_10_4; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_10_5 = io_wgt_rd_0_data_bits_10_5; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_10_6 = io_wgt_rd_0_data_bits_10_6; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_10_7 = io_wgt_rd_0_data_bits_10_7; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_10_8 = io_wgt_rd_0_data_bits_10_8; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_10_9 = io_wgt_rd_0_data_bits_10_9; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_10_10 = io_wgt_rd_0_data_bits_10_10; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_10_11 = io_wgt_rd_0_data_bits_10_11; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_10_12 = io_wgt_rd_0_data_bits_10_12; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_10_13 = io_wgt_rd_0_data_bits_10_13; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_10_14 = io_wgt_rd_0_data_bits_10_14; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_10_15 = io_wgt_rd_0_data_bits_10_15; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_11_0 = io_wgt_rd_0_data_bits_11_0; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_11_1 = io_wgt_rd_0_data_bits_11_1; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_11_2 = io_wgt_rd_0_data_bits_11_2; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_11_3 = io_wgt_rd_0_data_bits_11_3; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_11_4 = io_wgt_rd_0_data_bits_11_4; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_11_5 = io_wgt_rd_0_data_bits_11_5; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_11_6 = io_wgt_rd_0_data_bits_11_6; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_11_7 = io_wgt_rd_0_data_bits_11_7; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_11_8 = io_wgt_rd_0_data_bits_11_8; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_11_9 = io_wgt_rd_0_data_bits_11_9; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_11_10 = io_wgt_rd_0_data_bits_11_10; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_11_11 = io_wgt_rd_0_data_bits_11_11; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_11_12 = io_wgt_rd_0_data_bits_11_12; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_11_13 = io_wgt_rd_0_data_bits_11_13; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_11_14 = io_wgt_rd_0_data_bits_11_14; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_11_15 = io_wgt_rd_0_data_bits_11_15; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_12_0 = io_wgt_rd_0_data_bits_12_0; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_12_1 = io_wgt_rd_0_data_bits_12_1; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_12_2 = io_wgt_rd_0_data_bits_12_2; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_12_3 = io_wgt_rd_0_data_bits_12_3; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_12_4 = io_wgt_rd_0_data_bits_12_4; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_12_5 = io_wgt_rd_0_data_bits_12_5; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_12_6 = io_wgt_rd_0_data_bits_12_6; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_12_7 = io_wgt_rd_0_data_bits_12_7; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_12_8 = io_wgt_rd_0_data_bits_12_8; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_12_9 = io_wgt_rd_0_data_bits_12_9; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_12_10 = io_wgt_rd_0_data_bits_12_10; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_12_11 = io_wgt_rd_0_data_bits_12_11; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_12_12 = io_wgt_rd_0_data_bits_12_12; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_12_13 = io_wgt_rd_0_data_bits_12_13; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_12_14 = io_wgt_rd_0_data_bits_12_14; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_12_15 = io_wgt_rd_0_data_bits_12_15; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_13_0 = io_wgt_rd_0_data_bits_13_0; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_13_1 = io_wgt_rd_0_data_bits_13_1; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_13_2 = io_wgt_rd_0_data_bits_13_2; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_13_3 = io_wgt_rd_0_data_bits_13_3; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_13_4 = io_wgt_rd_0_data_bits_13_4; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_13_5 = io_wgt_rd_0_data_bits_13_5; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_13_6 = io_wgt_rd_0_data_bits_13_6; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_13_7 = io_wgt_rd_0_data_bits_13_7; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_13_8 = io_wgt_rd_0_data_bits_13_8; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_13_9 = io_wgt_rd_0_data_bits_13_9; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_13_10 = io_wgt_rd_0_data_bits_13_10; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_13_11 = io_wgt_rd_0_data_bits_13_11; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_13_12 = io_wgt_rd_0_data_bits_13_12; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_13_13 = io_wgt_rd_0_data_bits_13_13; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_13_14 = io_wgt_rd_0_data_bits_13_14; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_13_15 = io_wgt_rd_0_data_bits_13_15; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_14_0 = io_wgt_rd_0_data_bits_14_0; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_14_1 = io_wgt_rd_0_data_bits_14_1; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_14_2 = io_wgt_rd_0_data_bits_14_2; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_14_3 = io_wgt_rd_0_data_bits_14_3; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_14_4 = io_wgt_rd_0_data_bits_14_4; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_14_5 = io_wgt_rd_0_data_bits_14_5; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_14_6 = io_wgt_rd_0_data_bits_14_6; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_14_7 = io_wgt_rd_0_data_bits_14_7; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_14_8 = io_wgt_rd_0_data_bits_14_8; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_14_9 = io_wgt_rd_0_data_bits_14_9; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_14_10 = io_wgt_rd_0_data_bits_14_10; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_14_11 = io_wgt_rd_0_data_bits_14_11; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_14_12 = io_wgt_rd_0_data_bits_14_12; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_14_13 = io_wgt_rd_0_data_bits_14_13; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_14_14 = io_wgt_rd_0_data_bits_14_14; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_14_15 = io_wgt_rd_0_data_bits_14_15; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_15_0 = io_wgt_rd_0_data_bits_15_0; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_15_1 = io_wgt_rd_0_data_bits_15_1; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_15_2 = io_wgt_rd_0_data_bits_15_2; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_15_3 = io_wgt_rd_0_data_bits_15_3; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_15_4 = io_wgt_rd_0_data_bits_15_4; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_15_5 = io_wgt_rd_0_data_bits_15_5; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_15_6 = io_wgt_rd_0_data_bits_15_6; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_15_7 = io_wgt_rd_0_data_bits_15_7; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_15_8 = io_wgt_rd_0_data_bits_15_8; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_15_9 = io_wgt_rd_0_data_bits_15_9; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_15_10 = io_wgt_rd_0_data_bits_15_10; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_15_11 = io_wgt_rd_0_data_bits_15_11; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_15_12 = io_wgt_rd_0_data_bits_15_12; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_15_13 = io_wgt_rd_0_data_bits_15_13; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_15_14 = io_wgt_rd_0_data_bits_15_14; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_wgt_data_bits_15_15 = io_wgt_rd_0_data_bits_15_15; // @[TensorGemm.scala 702:27]
  assign mvc_0_io_acc_i_data_valid = io_acc_rd_0_data_valid; // @[TensorGemm.scala 703:35]
  assign mvc_0_io_acc_i_data_bits_0_0 = io_acc_rd_0_data_bits_0_0; // @[TensorGemm.scala 709:68]
  assign mvc_0_io_acc_i_data_bits_0_1 = io_acc_rd_0_data_bits_0_1; // @[TensorGemm.scala 709:68]
  assign mvc_0_io_acc_i_data_bits_0_2 = io_acc_rd_0_data_bits_0_2; // @[TensorGemm.scala 709:68]
  assign mvc_0_io_acc_i_data_bits_0_3 = io_acc_rd_0_data_bits_0_3; // @[TensorGemm.scala 709:68]
  assign mvc_0_io_acc_i_data_bits_0_4 = io_acc_rd_0_data_bits_0_4; // @[TensorGemm.scala 709:68]
  assign mvc_0_io_acc_i_data_bits_0_5 = io_acc_rd_0_data_bits_0_5; // @[TensorGemm.scala 709:68]
  assign mvc_0_io_acc_i_data_bits_0_6 = io_acc_rd_0_data_bits_0_6; // @[TensorGemm.scala 709:68]
  assign mvc_0_io_acc_i_data_bits_0_7 = io_acc_rd_0_data_bits_0_7; // @[TensorGemm.scala 709:68]
  assign mvc_0_io_acc_i_data_bits_0_8 = io_acc_rd_0_data_bits_0_8; // @[TensorGemm.scala 709:68]
  assign mvc_0_io_acc_i_data_bits_0_9 = io_acc_rd_0_data_bits_0_9; // @[TensorGemm.scala 709:68]
  assign mvc_0_io_acc_i_data_bits_0_10 = io_acc_rd_0_data_bits_0_10; // @[TensorGemm.scala 709:68]
  assign mvc_0_io_acc_i_data_bits_0_11 = io_acc_rd_0_data_bits_0_11; // @[TensorGemm.scala 709:68]
  assign mvc_0_io_acc_i_data_bits_0_12 = io_acc_rd_0_data_bits_0_12; // @[TensorGemm.scala 709:68]
  assign mvc_0_io_acc_i_data_bits_0_13 = io_acc_rd_0_data_bits_0_13; // @[TensorGemm.scala 709:68]
  assign mvc_0_io_acc_i_data_bits_0_14 = io_acc_rd_0_data_bits_0_14; // @[TensorGemm.scala 709:68]
  assign mvc_0_io_acc_i_data_bits_0_15 = io_acc_rd_0_data_bits_0_15; // @[TensorGemm.scala 709:68]
  assign mvc_0_io_bypass_cond = wrpipe_0_io_deq_bits == wrpipe2_io_deq_bits & wrpipe_0_io_deq_valid &
    wrpipe2_io_deq_valid; // @[TensorGemm.scala 695:85]
  assign wrpipe2_clock = clock;
  assign wrpipe2_reset = reset;
  assign wrpipe2_io_enq_valid = wrpipe_0_io_deq_valid; // @[TensorGemm.scala 692:20]
  assign wrpipe2_io_enq_bits = wrpipe_0_io_deq_bits; // @[TensorGemm.scala 692:20]
  always @(posedge clock) begin
    if (reset) begin // @[Reg.scala 27:20]
      delayed_valid <= 1'h0; // @[Reg.scala 27:20]
    end else begin
      delayed_valid <= _GEN_0;
    end
    delayed_acc_i <= m_io_acc_i; // @[Reg.scala 16:19 Reg.scala 16:23 Reg.scala 15:16]
    delayed_inp_i <= m_io_inp_i; // @[Reg.scala 16:19 Reg.scala 16:23 Reg.scala 15:16]
    delayed_wgt_i <= m_io_wgt_i; // @[Reg.scala 16:19 Reg.scala 16:23 Reg.scala 15:16]
    if (reset) begin // @[TensorGemm.scala 566:22]
      state <= 2'h0; // @[TensorGemm.scala 566:22]
    end else if (state == 2'h0 & io_start) begin // @[TensorGemm.scala 572:37]
      state <= 2'h1; // @[TensorGemm.scala 573:11]
    end else if (state == 2'h1 & m_io_last) begin // @[TensorGemm.scala 577:43]
      state <= 2'h2; // @[TensorGemm.scala 578:11]
    end else if (state == 2'h2 & inflight == 4'h0) begin // @[TensorGemm.scala 579:51]
      state <= 2'h0; // @[TensorGemm.scala 580:11]
    end
    if (reset) begin // @[TensorGemm.scala 567:25]
      inflight <= 4'h0; // @[TensorGemm.scala 567:25]
    end else if (_T) begin // @[TensorGemm.scala 662:25]
      inflight <= 4'h0; // @[TensorGemm.scala 664:14]
    end else if (!(m_io_valid & wrpipeNs_io_deq_valid)) begin // @[TensorGemm.scala 654:45]
      if (m_io_valid) begin // @[TensorGemm.scala 655:26]
        inflight <= _inflight_T_1; // @[TensorGemm.scala 657:14]
      end else begin
        inflight <= _GEN_27;
      end
    end
    if (state == 2'h0 & io_start) begin // @[TensorGemm.scala 572:37]
      capture_dec_wgt_1 <= io_dec_wgt_1; // @[TensorGemm.scala 574:17]
    end
    if (state == 2'h0 & io_start) begin // @[TensorGemm.scala 572:37]
      capture_dec_wgt_0 <= io_dec_wgt_0; // @[TensorGemm.scala 574:17]
    end
    if (state == 2'h0 & io_start) begin // @[TensorGemm.scala 572:37]
      capture_dec_inp_1 <= io_dec_inp_1; // @[TensorGemm.scala 574:17]
    end
    if (state == 2'h0 & io_start) begin // @[TensorGemm.scala 572:37]
      capture_dec_inp_0 <= io_dec_inp_0; // @[TensorGemm.scala 574:17]
    end
    if (state == 2'h0 & io_start) begin // @[TensorGemm.scala 572:37]
      capture_dec_acc_1 <= io_dec_acc_1; // @[TensorGemm.scala 574:17]
    end
    if (state == 2'h0 & io_start) begin // @[TensorGemm.scala 572:37]
      capture_dec_acc_0 <= io_dec_acc_0; // @[TensorGemm.scala 574:17]
    end
    if (state == 2'h0 & io_start) begin // @[TensorGemm.scala 572:37]
      capture_dec_empty_0 <= io_dec_empty_0; // @[TensorGemm.scala 574:17]
    end
    if (state == 2'h0 & io_start) begin // @[TensorGemm.scala 572:37]
      capture_dec_lp_1 <= io_dec_lp_1; // @[TensorGemm.scala 574:17]
    end
    if (state == 2'h0 & io_start) begin // @[TensorGemm.scala 572:37]
      capture_dec_lp_0 <= io_dec_lp_0; // @[TensorGemm.scala 574:17]
    end
    if (state == 2'h0 & io_start) begin // @[TensorGemm.scala 572:37]
      capture_dec_uop_end <= io_dec_uop_end; // @[TensorGemm.scala 574:17]
    end
    if (state == 2'h0 & io_start) begin // @[TensorGemm.scala 572:37]
      capture_dec_uop_begin <= io_dec_uop_begin; // @[TensorGemm.scala 574:17]
    end
    if (state == 2'h0 & io_start) begin // @[TensorGemm.scala 572:37]
      capture_dec_reset <= io_dec_reset; // @[TensorGemm.scala 574:17]
    end
    if (state == 2'h0 & io_start) begin // @[TensorGemm.scala 572:37]
      capture_dec_push_next <= io_dec_push_next; // @[TensorGemm.scala 574:17]
    end
    if (state == 2'h0 & io_start) begin // @[TensorGemm.scala 572:37]
      capture_dec_push_prev <= io_dec_push_prev; // @[TensorGemm.scala 574:17]
    end
    if (state == 2'h0 & io_start) begin // @[TensorGemm.scala 572:37]
      capture_dec_pop_next <= io_dec_pop_next; // @[TensorGemm.scala 574:17]
    end
    if (state == 2'h0 & io_start) begin // @[TensorGemm.scala 572:37]
      capture_dec_pop_prev <= io_dec_pop_prev; // @[TensorGemm.scala 574:17]
    end
    if (state == 2'h0 & io_start) begin // @[TensorGemm.scala 572:37]
      capture_dec_op <= io_dec_op; // @[TensorGemm.scala 574:17]
    end
    if (reset) begin // @[TensorGemm.scala 618:34]
      delayed_uop_valid <= 1'h0; // @[TensorGemm.scala 618:34]
    end else begin
      delayed_uop_valid <= delayed_valid; // @[TensorGemm.scala 618:34]
    end
    if (reset) begin // @[TensorGemm.scala 623:40]
      io_acc_rd_0_idx_valid_REG <= 1'h0; // @[TensorGemm.scala 623:40]
    end else begin
      io_acc_rd_0_idx_valid_REG <= acc_idx_pipe_io_deq_valid; // @[TensorGemm.scala 623:40]
    end
    io_acc_rd_0_idx_bits_REG <= acc_idx_pipe_io_deq_bits; // @[TensorGemm.scala 624:39]
    if (reset) begin // @[TensorGemm.scala 698:40]
      mvc_0_io_valid_reset_REG <= 1'h0; // @[TensorGemm.scala 698:40]
    end else begin
      mvc_0_io_valid_reset_REG <= reset_pipe_io_deq_bits & reset_pipe_io_deq_valid; // @[TensorGemm.scala 698:40]
    end
    `ifndef SYNTHESIS
    `ifdef PRINTF_COND
      if (`PRINTF_COND) begin
    `endif
        if (~(state != 2'h1 | _T_8 == _T_9 | reset)) begin
          $fwrite(32'h80000002,
            "Assertion failed\n    at TensorGemm.scala:585 assert(state =/= sRun  || capture_dec.asUInt === io.dec.asUInt)\n"
            ); // @[TensorGemm.scala 585:9]
        end
    `ifdef PRINTF_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef STOP_COND
      if (`STOP_COND) begin
    `endif
        if (~(state != 2'h1 | _T_8 == _T_9 | reset)) begin
          $fatal; // @[TensorGemm.scala 585:9]
        end
    `ifdef STOP_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef PRINTF_COND
      if (`PRINTF_COND) begin
    `endif
        if (~(state != 2'h2 | _T_10 | reset)) begin
          $fwrite(32'h80000002,
            "Assertion failed\n    at TensorGemm.scala:586 assert(state =/= sWait || capture_dec.asUInt === io.dec.asUInt)\n"
            ); // @[TensorGemm.scala 586:9]
        end
    `ifdef PRINTF_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef STOP_COND
      if (`STOP_COND) begin
    `endif
        if (~(state != 2'h2 | _T_10 | reset)) begin
          $fatal; // @[TensorGemm.scala 586:9]
        end
    `ifdef STOP_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef PRINTF_COND
      if (`PRINTF_COND) begin
    `endif
        if (~(io_uop_data_valid == delayed_valid | reset)) begin
          $fwrite(32'h80000002,
            "Assertion failed\n    at TensorGemm.scala:596 assert(delayedUopData.valid === delayed_valid)\n"); // @[TensorGemm.scala 596:9]
        end
    `ifdef PRINTF_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef STOP_COND
      if (`STOP_COND) begin
    `endif
        if (~(io_uop_data_valid == delayed_valid | reset)) begin
          $fatal; // @[TensorGemm.scala 596:9]
        end
    `ifdef STOP_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef PRINTF_COND
      if (`PRINTF_COND) begin
    `endif
        if (~(io_inp_rd_0_data_valid == delayed_uop_valid | reset)) begin
          $fwrite(32'h80000002,
            "Assertion failed\n    at TensorGemm.scala:621 assert(io.inp.rd(0).data.valid === delayed_uop_valid)\n"); // @[TensorGemm.scala 621:9]
        end
    `ifdef PRINTF_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef STOP_COND
      if (`STOP_COND) begin
    `endif
        if (~(io_inp_rd_0_data_valid == delayed_uop_valid | reset)) begin
          $fatal; // @[TensorGemm.scala 621:9]
        end
    `ifdef STOP_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef PRINTF_COND
      if (`PRINTF_COND) begin
    `endif
        if (~(io_wgt_rd_0_data_valid == delayed_uop_valid | reset)) begin
          $fwrite(32'h80000002,
            "Assertion failed\n    at TensorGemm.scala:630 assert(io.wgt.rd(idx).data.valid === ShiftRegister(delayed_uop_valid, scratchpadReadLatency))\n"
            ); // @[TensorGemm.scala 630:11]
        end
    `ifdef PRINTF_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef STOP_COND
      if (`STOP_COND) begin
    `endif
        if (~(io_wgt_rd_0_data_valid == delayed_uop_valid | reset)) begin
          $fatal; // @[TensorGemm.scala 630:11]
        end
    `ifdef STOP_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef PRINTF_COND
      if (`PRINTF_COND) begin
    `endif
        if (~(io_acc_rd_0_data_valid == wrpipe_0_io_deq_valid | reset)) begin
          $fwrite(32'h80000002,
            "Assertion failed\n    at TensorGemm.scala:651 assert(io.acc.rd(idx).data.valid === wrpipe(idx).io.deq.valid)\n"
            ); // @[TensorGemm.scala 651:11]
        end
    `ifdef PRINTF_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef STOP_COND
      if (`STOP_COND) begin
    `endif
        if (~(io_acc_rd_0_data_valid == wrpipe_0_io_deq_valid | reset)) begin
          $fatal; // @[TensorGemm.scala 651:11]
        end
    `ifdef STOP_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef PRINTF_COND
      if (`PRINTF_COND) begin
    `endif
        if (~_T_39 & m_io_valid & ~(inflight != 4'hf | reset)) begin
          $fwrite(32'h80000002,
            "Assertion failed\n    at TensorGemm.scala:656 assert(inflight =/= ((1<<inflightBits)-1).U)\n"); // @[TensorGemm.scala 656:11]
        end
    `ifdef PRINTF_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef STOP_COND
      if (`STOP_COND) begin
    `endif
        if (~_T_39 & m_io_valid & ~(inflight != 4'hf | reset)) begin
          $fatal; // @[TensorGemm.scala 656:11]
        end
    `ifdef STOP_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef PRINTF_COND
      if (`PRINTF_COND) begin
    `endif
        if (_GEN_31 & ~m_io_valid & wrpipeNs_io_deq_valid & ~(inflight != 4'h0 | reset)) begin
          $fwrite(32'h80000002,"Assertion failed\n    at TensorGemm.scala:659 assert(inflight =/= 0.U)\n"); // @[TensorGemm.scala 659:11]
        end
    `ifdef PRINTF_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef STOP_COND
      if (`STOP_COND) begin
    `endif
        if (_GEN_31 & ~m_io_valid & wrpipeNs_io_deq_valid & ~(inflight != 4'h0 | reset)) begin
          $fatal; // @[TensorGemm.scala 659:11]
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
          $fwrite(32'h80000002,"Assertion failed\n    at TensorGemm.scala:663 assert(inflight === 0.U)\n"); // @[TensorGemm.scala 663:11]
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
          $fatal; // @[TensorGemm.scala 663:11]
        end
    `ifdef STOP_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef PRINTF_COND
      if (`PRINTF_COND) begin
    `endif
        if (~(mvc_0_io_acc_o_data_valid == (wrpipe_0_io_deq_valid | mvc_0_io_valid_reset) | reset)) begin
          $fwrite(32'h80000002,
            "Assertion failed\n    at TensorGemm.scala:704 assert(mvc(idx1).io.acc_o.data.valid === (wrpipe(idx1).io.deq.valid | mvc(idx1).io.valid_reset))\n"
            ); // @[TensorGemm.scala 704:11]
        end
    `ifdef PRINTF_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef STOP_COND
      if (`STOP_COND) begin
    `endif
        if (~(mvc_0_io_acc_o_data_valid == (wrpipe_0_io_deq_valid | mvc_0_io_valid_reset) | reset)) begin
          $fatal; // @[TensorGemm.scala 704:11]
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
  delayed_valid = _RAND_0[0:0];
  _RAND_1 = {1{`RANDOM}};
  delayed_acc_i = _RAND_1[10:0];
  _RAND_2 = {1{`RANDOM}};
  delayed_inp_i = _RAND_2[10:0];
  _RAND_3 = {1{`RANDOM}};
  delayed_wgt_i = _RAND_3[9:0];
  _RAND_4 = {1{`RANDOM}};
  state = _RAND_4[1:0];
  _RAND_5 = {1{`RANDOM}};
  inflight = _RAND_5[3:0];
  _RAND_6 = {1{`RANDOM}};
  capture_dec_wgt_1 = _RAND_6[9:0];
  _RAND_7 = {1{`RANDOM}};
  capture_dec_wgt_0 = _RAND_7[9:0];
  _RAND_8 = {1{`RANDOM}};
  capture_dec_inp_1 = _RAND_8[10:0];
  _RAND_9 = {1{`RANDOM}};
  capture_dec_inp_0 = _RAND_9[10:0];
  _RAND_10 = {1{`RANDOM}};
  capture_dec_acc_1 = _RAND_10[10:0];
  _RAND_11 = {1{`RANDOM}};
  capture_dec_acc_0 = _RAND_11[10:0];
  _RAND_12 = {1{`RANDOM}};
  capture_dec_empty_0 = _RAND_12[0:0];
  _RAND_13 = {1{`RANDOM}};
  capture_dec_lp_1 = _RAND_13[13:0];
  _RAND_14 = {1{`RANDOM}};
  capture_dec_lp_0 = _RAND_14[13:0];
  _RAND_15 = {1{`RANDOM}};
  capture_dec_uop_end = _RAND_15[13:0];
  _RAND_16 = {1{`RANDOM}};
  capture_dec_uop_begin = _RAND_16[12:0];
  _RAND_17 = {1{`RANDOM}};
  capture_dec_reset = _RAND_17[0:0];
  _RAND_18 = {1{`RANDOM}};
  capture_dec_push_next = _RAND_18[0:0];
  _RAND_19 = {1{`RANDOM}};
  capture_dec_push_prev = _RAND_19[0:0];
  _RAND_20 = {1{`RANDOM}};
  capture_dec_pop_next = _RAND_20[0:0];
  _RAND_21 = {1{`RANDOM}};
  capture_dec_pop_prev = _RAND_21[0:0];
  _RAND_22 = {1{`RANDOM}};
  capture_dec_op = _RAND_22[2:0];
  _RAND_23 = {1{`RANDOM}};
  delayed_uop_valid = _RAND_23[0:0];
  _RAND_24 = {1{`RANDOM}};
  io_acc_rd_0_idx_valid_REG = _RAND_24[0:0];
  _RAND_25 = {1{`RANDOM}};
  io_acc_rd_0_idx_bits_REG = _RAND_25[10:0];
  _RAND_26 = {1{`RANDOM}};
  mvc_0_io_valid_reset_REG = _RAND_26[0:0];
`endif // RANDOMIZE_REG_INIT
  `endif // RANDOMIZE
end // initial
`ifdef FIRRTL_AFTER_INITIAL
`FIRRTL_AFTER_INITIAL
`endif
`endif // SYNTHESIS
endmodule