module Compute(
  input          clock,
  input          reset,
  input          io_i_post_0,
  input          io_i_post_1,
  output         io_o_post_0,
  output         io_o_post_1,
  output         io_inst_ready,
  input          io_inst_valid,
  input  [127:0] io_inst_bits,
  input  [31:0]  io_uop_baddr,
  input  [31:0]  io_acc_baddr,
  input          io_vme_rd_0_cmd_ready,
  output         io_vme_rd_0_cmd_valid,
  output [31:0]  io_vme_rd_0_cmd_bits_addr,
  output [7:0]   io_vme_rd_0_cmd_bits_len,
  output [20:0]  io_vme_rd_0_cmd_bits_tag,
  input          io_vme_rd_0_data_valid,
  input  [63:0]  io_vme_rd_0_data_bits_data,
  input  [20:0]  io_vme_rd_0_data_bits_tag,
  input          io_vme_rd_0_data_bits_last,
  input          io_vme_rd_1_cmd_ready,
  output         io_vme_rd_1_cmd_valid,
  output [31:0]  io_vme_rd_1_cmd_bits_addr,
  output [7:0]   io_vme_rd_1_cmd_bits_len,
  output [20:0]  io_vme_rd_1_cmd_bits_tag,
  input          io_vme_rd_1_data_valid,
  input  [63:0]  io_vme_rd_1_data_bits_data,
  input  [20:0]  io_vme_rd_1_data_bits_tag,
  output         io_inp_rd_0_idx_valid,
  output [10:0]  io_inp_rd_0_idx_bits,
  input          io_inp_rd_0_data_valid,
  input  [31:0]  io_inp_rd_0_data_bits_0_0,
  input  [31:0]  io_inp_rd_0_data_bits_0_1,
  input  [31:0]  io_inp_rd_0_data_bits_0_2,
  input  [31:0]  io_inp_rd_0_data_bits_0_3,
  input  [31:0]  io_inp_rd_0_data_bits_0_4,
  input  [31:0]  io_inp_rd_0_data_bits_0_5,
  input  [31:0]  io_inp_rd_0_data_bits_0_6,
  input  [31:0]  io_inp_rd_0_data_bits_0_7,
  input  [31:0]  io_inp_rd_0_data_bits_0_8,
  input  [31:0]  io_inp_rd_0_data_bits_0_9,
  input  [31:0]  io_inp_rd_0_data_bits_0_10,
  input  [31:0]  io_inp_rd_0_data_bits_0_11,
  input  [31:0]  io_inp_rd_0_data_bits_0_12,
  input  [31:0]  io_inp_rd_0_data_bits_0_13,
  input  [31:0]  io_inp_rd_0_data_bits_0_14,
  input  [31:0]  io_inp_rd_0_data_bits_0_15,
  output         io_wgt_rd_0_idx_valid,
  output [9:0]   io_wgt_rd_0_idx_bits,
  input          io_wgt_rd_0_data_valid,
  input  [31:0]  io_wgt_rd_0_data_bits_0_0,
  input  [31:0]  io_wgt_rd_0_data_bits_0_1,
  input  [31:0]  io_wgt_rd_0_data_bits_0_2,
  input  [31:0]  io_wgt_rd_0_data_bits_0_3,
  input  [31:0]  io_wgt_rd_0_data_bits_0_4,
  input  [31:0]  io_wgt_rd_0_data_bits_0_5,
  input  [31:0]  io_wgt_rd_0_data_bits_0_6,
  input  [31:0]  io_wgt_rd_0_data_bits_0_7,
  input  [31:0]  io_wgt_rd_0_data_bits_0_8,
  input  [31:0]  io_wgt_rd_0_data_bits_0_9,
  input  [31:0]  io_wgt_rd_0_data_bits_0_10,
  input  [31:0]  io_wgt_rd_0_data_bits_0_11,
  input  [31:0]  io_wgt_rd_0_data_bits_0_12,
  input  [31:0]  io_wgt_rd_0_data_bits_0_13,
  input  [31:0]  io_wgt_rd_0_data_bits_0_14,
  input  [31:0]  io_wgt_rd_0_data_bits_0_15,
  input  [31:0]  io_wgt_rd_0_data_bits_1_0,
  input  [31:0]  io_wgt_rd_0_data_bits_1_1,
  input  [31:0]  io_wgt_rd_0_data_bits_1_2,
  input  [31:0]  io_wgt_rd_0_data_bits_1_3,
  input  [31:0]  io_wgt_rd_0_data_bits_1_4,
  input  [31:0]  io_wgt_rd_0_data_bits_1_5,
  input  [31:0]  io_wgt_rd_0_data_bits_1_6,
  input  [31:0]  io_wgt_rd_0_data_bits_1_7,
  input  [31:0]  io_wgt_rd_0_data_bits_1_8,
  input  [31:0]  io_wgt_rd_0_data_bits_1_9,
  input  [31:0]  io_wgt_rd_0_data_bits_1_10,
  input  [31:0]  io_wgt_rd_0_data_bits_1_11,
  input  [31:0]  io_wgt_rd_0_data_bits_1_12,
  input  [31:0]  io_wgt_rd_0_data_bits_1_13,
  input  [31:0]  io_wgt_rd_0_data_bits_1_14,
  input  [31:0]  io_wgt_rd_0_data_bits_1_15,
  input  [31:0]  io_wgt_rd_0_data_bits_2_0,
  input  [31:0]  io_wgt_rd_0_data_bits_2_1,
  input  [31:0]  io_wgt_rd_0_data_bits_2_2,
  input  [31:0]  io_wgt_rd_0_data_bits_2_3,
  input  [31:0]  io_wgt_rd_0_data_bits_2_4,
  input  [31:0]  io_wgt_rd_0_data_bits_2_5,
  input  [31:0]  io_wgt_rd_0_data_bits_2_6,
  input  [31:0]  io_wgt_rd_0_data_bits_2_7,
  input  [31:0]  io_wgt_rd_0_data_bits_2_8,
  input  [31:0]  io_wgt_rd_0_data_bits_2_9,
  input  [31:0]  io_wgt_rd_0_data_bits_2_10,
  input  [31:0]  io_wgt_rd_0_data_bits_2_11,
  input  [31:0]  io_wgt_rd_0_data_bits_2_12,
  input  [31:0]  io_wgt_rd_0_data_bits_2_13,
  input  [31:0]  io_wgt_rd_0_data_bits_2_14,
  input  [31:0]  io_wgt_rd_0_data_bits_2_15,
  input  [31:0]  io_wgt_rd_0_data_bits_3_0,
  input  [31:0]  io_wgt_rd_0_data_bits_3_1,
  input  [31:0]  io_wgt_rd_0_data_bits_3_2,
  input  [31:0]  io_wgt_rd_0_data_bits_3_3,
  input  [31:0]  io_wgt_rd_0_data_bits_3_4,
  input  [31:0]  io_wgt_rd_0_data_bits_3_5,
  input  [31:0]  io_wgt_rd_0_data_bits_3_6,
  input  [31:0]  io_wgt_rd_0_data_bits_3_7,
  input  [31:0]  io_wgt_rd_0_data_bits_3_8,
  input  [31:0]  io_wgt_rd_0_data_bits_3_9,
  input  [31:0]  io_wgt_rd_0_data_bits_3_10,
  input  [31:0]  io_wgt_rd_0_data_bits_3_11,
  input  [31:0]  io_wgt_rd_0_data_bits_3_12,
  input  [31:0]  io_wgt_rd_0_data_bits_3_13,
  input  [31:0]  io_wgt_rd_0_data_bits_3_14,
  input  [31:0]  io_wgt_rd_0_data_bits_3_15,
  input  [31:0]  io_wgt_rd_0_data_bits_4_0,
  input  [31:0]  io_wgt_rd_0_data_bits_4_1,
  input  [31:0]  io_wgt_rd_0_data_bits_4_2,
  input  [31:0]  io_wgt_rd_0_data_bits_4_3,
  input  [31:0]  io_wgt_rd_0_data_bits_4_4,
  input  [31:0]  io_wgt_rd_0_data_bits_4_5,
  input  [31:0]  io_wgt_rd_0_data_bits_4_6,
  input  [31:0]  io_wgt_rd_0_data_bits_4_7,
  input  [31:0]  io_wgt_rd_0_data_bits_4_8,
  input  [31:0]  io_wgt_rd_0_data_bits_4_9,
  input  [31:0]  io_wgt_rd_0_data_bits_4_10,
  input  [31:0]  io_wgt_rd_0_data_bits_4_11,
  input  [31:0]  io_wgt_rd_0_data_bits_4_12,
  input  [31:0]  io_wgt_rd_0_data_bits_4_13,
  input  [31:0]  io_wgt_rd_0_data_bits_4_14,
  input  [31:0]  io_wgt_rd_0_data_bits_4_15,
  input  [31:0]  io_wgt_rd_0_data_bits_5_0,
  input  [31:0]  io_wgt_rd_0_data_bits_5_1,
  input  [31:0]  io_wgt_rd_0_data_bits_5_2,
  input  [31:0]  io_wgt_rd_0_data_bits_5_3,
  input  [31:0]  io_wgt_rd_0_data_bits_5_4,
  input  [31:0]  io_wgt_rd_0_data_bits_5_5,
  input  [31:0]  io_wgt_rd_0_data_bits_5_6,
  input  [31:0]  io_wgt_rd_0_data_bits_5_7,
  input  [31:0]  io_wgt_rd_0_data_bits_5_8,
  input  [31:0]  io_wgt_rd_0_data_bits_5_9,
  input  [31:0]  io_wgt_rd_0_data_bits_5_10,
  input  [31:0]  io_wgt_rd_0_data_bits_5_11,
  input  [31:0]  io_wgt_rd_0_data_bits_5_12,
  input  [31:0]  io_wgt_rd_0_data_bits_5_13,
  input  [31:0]  io_wgt_rd_0_data_bits_5_14,
  input  [31:0]  io_wgt_rd_0_data_bits_5_15,
  input  [31:0]  io_wgt_rd_0_data_bits_6_0,
  input  [31:0]  io_wgt_rd_0_data_bits_6_1,
  input  [31:0]  io_wgt_rd_0_data_bits_6_2,
  input  [31:0]  io_wgt_rd_0_data_bits_6_3,
  input  [31:0]  io_wgt_rd_0_data_bits_6_4,
  input  [31:0]  io_wgt_rd_0_data_bits_6_5,
  input  [31:0]  io_wgt_rd_0_data_bits_6_6,
  input  [31:0]  io_wgt_rd_0_data_bits_6_7,
  input  [31:0]  io_wgt_rd_0_data_bits_6_8,
  input  [31:0]  io_wgt_rd_0_data_bits_6_9,
  input  [31:0]  io_wgt_rd_0_data_bits_6_10,
  input  [31:0]  io_wgt_rd_0_data_bits_6_11,
  input  [31:0]  io_wgt_rd_0_data_bits_6_12,
  input  [31:0]  io_wgt_rd_0_data_bits_6_13,
  input  [31:0]  io_wgt_rd_0_data_bits_6_14,
  input  [31:0]  io_wgt_rd_0_data_bits_6_15,
  input  [31:0]  io_wgt_rd_0_data_bits_7_0,
  input  [31:0]  io_wgt_rd_0_data_bits_7_1,
  input  [31:0]  io_wgt_rd_0_data_bits_7_2,
  input  [31:0]  io_wgt_rd_0_data_bits_7_3,
  input  [31:0]  io_wgt_rd_0_data_bits_7_4,
  input  [31:0]  io_wgt_rd_0_data_bits_7_5,
  input  [31:0]  io_wgt_rd_0_data_bits_7_6,
  input  [31:0]  io_wgt_rd_0_data_bits_7_7,
  input  [31:0]  io_wgt_rd_0_data_bits_7_8,
  input  [31:0]  io_wgt_rd_0_data_bits_7_9,
  input  [31:0]  io_wgt_rd_0_data_bits_7_10,
  input  [31:0]  io_wgt_rd_0_data_bits_7_11,
  input  [31:0]  io_wgt_rd_0_data_bits_7_12,
  input  [31:0]  io_wgt_rd_0_data_bits_7_13,
  input  [31:0]  io_wgt_rd_0_data_bits_7_14,
  input  [31:0]  io_wgt_rd_0_data_bits_7_15,
  input  [31:0]  io_wgt_rd_0_data_bits_8_0,
  input  [31:0]  io_wgt_rd_0_data_bits_8_1,
  input  [31:0]  io_wgt_rd_0_data_bits_8_2,
  input  [31:0]  io_wgt_rd_0_data_bits_8_3,
  input  [31:0]  io_wgt_rd_0_data_bits_8_4,
  input  [31:0]  io_wgt_rd_0_data_bits_8_5,
  input  [31:0]  io_wgt_rd_0_data_bits_8_6,
  input  [31:0]  io_wgt_rd_0_data_bits_8_7,
  input  [31:0]  io_wgt_rd_0_data_bits_8_8,
  input  [31:0]  io_wgt_rd_0_data_bits_8_9,
  input  [31:0]  io_wgt_rd_0_data_bits_8_10,
  input  [31:0]  io_wgt_rd_0_data_bits_8_11,
  input  [31:0]  io_wgt_rd_0_data_bits_8_12,
  input  [31:0]  io_wgt_rd_0_data_bits_8_13,
  input  [31:0]  io_wgt_rd_0_data_bits_8_14,
  input  [31:0]  io_wgt_rd_0_data_bits_8_15,
  input  [31:0]  io_wgt_rd_0_data_bits_9_0,
  input  [31:0]  io_wgt_rd_0_data_bits_9_1,
  input  [31:0]  io_wgt_rd_0_data_bits_9_2,
  input  [31:0]  io_wgt_rd_0_data_bits_9_3,
  input  [31:0]  io_wgt_rd_0_data_bits_9_4,
  input  [31:0]  io_wgt_rd_0_data_bits_9_5,
  input  [31:0]  io_wgt_rd_0_data_bits_9_6,
  input  [31:0]  io_wgt_rd_0_data_bits_9_7,
  input  [31:0]  io_wgt_rd_0_data_bits_9_8,
  input  [31:0]  io_wgt_rd_0_data_bits_9_9,
  input  [31:0]  io_wgt_rd_0_data_bits_9_10,
  input  [31:0]  io_wgt_rd_0_data_bits_9_11,
  input  [31:0]  io_wgt_rd_0_data_bits_9_12,
  input  [31:0]  io_wgt_rd_0_data_bits_9_13,
  input  [31:0]  io_wgt_rd_0_data_bits_9_14,
  input  [31:0]  io_wgt_rd_0_data_bits_9_15,
  input  [31:0]  io_wgt_rd_0_data_bits_10_0,
  input  [31:0]  io_wgt_rd_0_data_bits_10_1,
  input  [31:0]  io_wgt_rd_0_data_bits_10_2,
  input  [31:0]  io_wgt_rd_0_data_bits_10_3,
  input  [31:0]  io_wgt_rd_0_data_bits_10_4,
  input  [31:0]  io_wgt_rd_0_data_bits_10_5,
  input  [31:0]  io_wgt_rd_0_data_bits_10_6,
  input  [31:0]  io_wgt_rd_0_data_bits_10_7,
  input  [31:0]  io_wgt_rd_0_data_bits_10_8,
  input  [31:0]  io_wgt_rd_0_data_bits_10_9,
  input  [31:0]  io_wgt_rd_0_data_bits_10_10,
  input  [31:0]  io_wgt_rd_0_data_bits_10_11,
  input  [31:0]  io_wgt_rd_0_data_bits_10_12,
  input  [31:0]  io_wgt_rd_0_data_bits_10_13,
  input  [31:0]  io_wgt_rd_0_data_bits_10_14,
  input  [31:0]  io_wgt_rd_0_data_bits_10_15,
  input  [31:0]  io_wgt_rd_0_data_bits_11_0,
  input  [31:0]  io_wgt_rd_0_data_bits_11_1,
  input  [31:0]  io_wgt_rd_0_data_bits_11_2,
  input  [31:0]  io_wgt_rd_0_data_bits_11_3,
  input  [31:0]  io_wgt_rd_0_data_bits_11_4,
  input  [31:0]  io_wgt_rd_0_data_bits_11_5,
  input  [31:0]  io_wgt_rd_0_data_bits_11_6,
  input  [31:0]  io_wgt_rd_0_data_bits_11_7,
  input  [31:0]  io_wgt_rd_0_data_bits_11_8,
  input  [31:0]  io_wgt_rd_0_data_bits_11_9,
  input  [31:0]  io_wgt_rd_0_data_bits_11_10,
  input  [31:0]  io_wgt_rd_0_data_bits_11_11,
  input  [31:0]  io_wgt_rd_0_data_bits_11_12,
  input  [31:0]  io_wgt_rd_0_data_bits_11_13,
  input  [31:0]  io_wgt_rd_0_data_bits_11_14,
  input  [31:0]  io_wgt_rd_0_data_bits_11_15,
  input  [31:0]  io_wgt_rd_0_data_bits_12_0,
  input  [31:0]  io_wgt_rd_0_data_bits_12_1,
  input  [31:0]  io_wgt_rd_0_data_bits_12_2,
  input  [31:0]  io_wgt_rd_0_data_bits_12_3,
  input  [31:0]  io_wgt_rd_0_data_bits_12_4,
  input  [31:0]  io_wgt_rd_0_data_bits_12_5,
  input  [31:0]  io_wgt_rd_0_data_bits_12_6,
  input  [31:0]  io_wgt_rd_0_data_bits_12_7,
  input  [31:0]  io_wgt_rd_0_data_bits_12_8,
  input  [31:0]  io_wgt_rd_0_data_bits_12_9,
  input  [31:0]  io_wgt_rd_0_data_bits_12_10,
  input  [31:0]  io_wgt_rd_0_data_bits_12_11,
  input  [31:0]  io_wgt_rd_0_data_bits_12_12,
  input  [31:0]  io_wgt_rd_0_data_bits_12_13,
  input  [31:0]  io_wgt_rd_0_data_bits_12_14,
  input  [31:0]  io_wgt_rd_0_data_bits_12_15,
  input  [31:0]  io_wgt_rd_0_data_bits_13_0,
  input  [31:0]  io_wgt_rd_0_data_bits_13_1,
  input  [31:0]  io_wgt_rd_0_data_bits_13_2,
  input  [31:0]  io_wgt_rd_0_data_bits_13_3,
  input  [31:0]  io_wgt_rd_0_data_bits_13_4,
  input  [31:0]  io_wgt_rd_0_data_bits_13_5,
  input  [31:0]  io_wgt_rd_0_data_bits_13_6,
  input  [31:0]  io_wgt_rd_0_data_bits_13_7,
  input  [31:0]  io_wgt_rd_0_data_bits_13_8,
  input  [31:0]  io_wgt_rd_0_data_bits_13_9,
  input  [31:0]  io_wgt_rd_0_data_bits_13_10,
  input  [31:0]  io_wgt_rd_0_data_bits_13_11,
  input  [31:0]  io_wgt_rd_0_data_bits_13_12,
  input  [31:0]  io_wgt_rd_0_data_bits_13_13,
  input  [31:0]  io_wgt_rd_0_data_bits_13_14,
  input  [31:0]  io_wgt_rd_0_data_bits_13_15,
  input  [31:0]  io_wgt_rd_0_data_bits_14_0,
  input  [31:0]  io_wgt_rd_0_data_bits_14_1,
  input  [31:0]  io_wgt_rd_0_data_bits_14_2,
  input  [31:0]  io_wgt_rd_0_data_bits_14_3,
  input  [31:0]  io_wgt_rd_0_data_bits_14_4,
  input  [31:0]  io_wgt_rd_0_data_bits_14_5,
  input  [31:0]  io_wgt_rd_0_data_bits_14_6,
  input  [31:0]  io_wgt_rd_0_data_bits_14_7,
  input  [31:0]  io_wgt_rd_0_data_bits_14_8,
  input  [31:0]  io_wgt_rd_0_data_bits_14_9,
  input  [31:0]  io_wgt_rd_0_data_bits_14_10,
  input  [31:0]  io_wgt_rd_0_data_bits_14_11,
  input  [31:0]  io_wgt_rd_0_data_bits_14_12,
  input  [31:0]  io_wgt_rd_0_data_bits_14_13,
  input  [31:0]  io_wgt_rd_0_data_bits_14_14,
  input  [31:0]  io_wgt_rd_0_data_bits_14_15,
  input  [31:0]  io_wgt_rd_0_data_bits_15_0,
  input  [31:0]  io_wgt_rd_0_data_bits_15_1,
  input  [31:0]  io_wgt_rd_0_data_bits_15_2,
  input  [31:0]  io_wgt_rd_0_data_bits_15_3,
  input  [31:0]  io_wgt_rd_0_data_bits_15_4,
  input  [31:0]  io_wgt_rd_0_data_bits_15_5,
  input  [31:0]  io_wgt_rd_0_data_bits_15_6,
  input  [31:0]  io_wgt_rd_0_data_bits_15_7,
  input  [31:0]  io_wgt_rd_0_data_bits_15_8,
  input  [31:0]  io_wgt_rd_0_data_bits_15_9,
  input  [31:0]  io_wgt_rd_0_data_bits_15_10,
  input  [31:0]  io_wgt_rd_0_data_bits_15_11,
  input  [31:0]  io_wgt_rd_0_data_bits_15_12,
  input  [31:0]  io_wgt_rd_0_data_bits_15_13,
  input  [31:0]  io_wgt_rd_0_data_bits_15_14,
  input  [31:0]  io_wgt_rd_0_data_bits_15_15,
  output         io_out_wr_0_valid,
  output [10:0]  io_out_wr_0_bits_idx,
  output [31:0]  io_out_wr_0_bits_data_0_0,
  output [31:0]  io_out_wr_0_bits_data_0_1,
  output [31:0]  io_out_wr_0_bits_data_0_2,
  output [31:0]  io_out_wr_0_bits_data_0_3,
  output [31:0]  io_out_wr_0_bits_data_0_4,
  output [31:0]  io_out_wr_0_bits_data_0_5,
  output [31:0]  io_out_wr_0_bits_data_0_6,
  output [31:0]  io_out_wr_0_bits_data_0_7,
  output [31:0]  io_out_wr_0_bits_data_0_8,
  output [31:0]  io_out_wr_0_bits_data_0_9,
  output [31:0]  io_out_wr_0_bits_data_0_10,
  output [31:0]  io_out_wr_0_bits_data_0_11,
  output [31:0]  io_out_wr_0_bits_data_0_12,
  output [31:0]  io_out_wr_0_bits_data_0_13,
  output [31:0]  io_out_wr_0_bits_data_0_14,
  output [31:0]  io_out_wr_0_bits_data_0_15,
  output         io_finish,
  output         io_acc_wr_event
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
`endif // RANDOMIZE_REG_INIT
  wire  s_0_clock; // @[Compute.scala 58:11]
  wire  s_0_reset; // @[Compute.scala 58:11]
  wire  s_0_io_spost; // @[Compute.scala 58:11]
  wire  s_0_io_swait; // @[Compute.scala 58:11]
  wire  s_0_io_sready; // @[Compute.scala 58:11]
  wire  s_1_clock; // @[Compute.scala 58:11]
  wire  s_1_reset; // @[Compute.scala 58:11]
  wire  s_1_io_spost; // @[Compute.scala 58:11]
  wire  s_1_io_swait; // @[Compute.scala 58:11]
  wire  s_1_io_sready; // @[Compute.scala 58:11]
  wire  loadUop_clock; // @[Compute.scala 61:23]
  wire  loadUop_reset; // @[Compute.scala 61:23]
  wire  loadUop_io_start; // @[Compute.scala 61:23]
  wire  loadUop_io_done; // @[Compute.scala 61:23]
  wire [127:0] loadUop_io_inst; // @[Compute.scala 61:23]
  wire [31:0] loadUop_io_baddr; // @[Compute.scala 61:23]
  wire  loadUop_io_vme_rd_cmd_ready; // @[Compute.scala 61:23]
  wire  loadUop_io_vme_rd_cmd_valid; // @[Compute.scala 61:23]
  wire [31:0] loadUop_io_vme_rd_cmd_bits_addr; // @[Compute.scala 61:23]
  wire [7:0] loadUop_io_vme_rd_cmd_bits_len; // @[Compute.scala 61:23]
  wire [20:0] loadUop_io_vme_rd_cmd_bits_tag; // @[Compute.scala 61:23]
  wire  loadUop_io_vme_rd_data_valid; // @[Compute.scala 61:23]
  wire [63:0] loadUop_io_vme_rd_data_bits_data; // @[Compute.scala 61:23]
  wire [20:0] loadUop_io_vme_rd_data_bits_tag; // @[Compute.scala 61:23]
  wire  loadUop_io_vme_rd_data_bits_last; // @[Compute.scala 61:23]
  wire  loadUop_io_uop_idx_valid; // @[Compute.scala 61:23]
  wire [10:0] loadUop_io_uop_idx_bits; // @[Compute.scala 61:23]
  wire  loadUop_io_uop_data_valid; // @[Compute.scala 61:23]
  wire [9:0] loadUop_io_uop_data_bits_u2; // @[Compute.scala 61:23]
  wire [10:0] loadUop_io_uop_data_bits_u1; // @[Compute.scala 61:23]
  wire [10:0] loadUop_io_uop_data_bits_u0; // @[Compute.scala 61:23]
  wire  tensorAcc_clock; // @[Compute.scala 62:25]
  wire  tensorAcc_reset; // @[Compute.scala 62:25]
  wire  tensorAcc_io_start; // @[Compute.scala 62:25]
  wire  tensorAcc_io_done; // @[Compute.scala 62:25]
  wire [127:0] tensorAcc_io_inst; // @[Compute.scala 62:25]
  wire [31:0] tensorAcc_io_baddr; // @[Compute.scala 62:25]
  wire  tensorAcc_io_vme_rd_cmd_ready; // @[Compute.scala 62:25]
  wire  tensorAcc_io_vme_rd_cmd_valid; // @[Compute.scala 62:25]
  wire [31:0] tensorAcc_io_vme_rd_cmd_bits_addr; // @[Compute.scala 62:25]
  wire [7:0] tensorAcc_io_vme_rd_cmd_bits_len; // @[Compute.scala 62:25]
  wire [20:0] tensorAcc_io_vme_rd_cmd_bits_tag; // @[Compute.scala 62:25]
  wire  tensorAcc_io_vme_rd_data_valid; // @[Compute.scala 62:25]
  wire [63:0] tensorAcc_io_vme_rd_data_bits_data; // @[Compute.scala 62:25]
  wire [20:0] tensorAcc_io_vme_rd_data_bits_tag; // @[Compute.scala 62:25]
  wire  tensorAcc_io_tensor_rd_0_idx_valid; // @[Compute.scala 62:25]
  wire [10:0] tensorAcc_io_tensor_rd_0_idx_bits; // @[Compute.scala 62:25]
  wire  tensorAcc_io_tensor_rd_0_data_valid; // @[Compute.scala 62:25]
  wire [31:0] tensorAcc_io_tensor_rd_0_data_bits_0_0; // @[Compute.scala 62:25]
  wire [31:0] tensorAcc_io_tensor_rd_0_data_bits_0_1; // @[Compute.scala 62:25]
  wire [31:0] tensorAcc_io_tensor_rd_0_data_bits_0_2; // @[Compute.scala 62:25]
  wire [31:0] tensorAcc_io_tensor_rd_0_data_bits_0_3; // @[Compute.scala 62:25]
  wire [31:0] tensorAcc_io_tensor_rd_0_data_bits_0_4; // @[Compute.scala 62:25]
  wire [31:0] tensorAcc_io_tensor_rd_0_data_bits_0_5; // @[Compute.scala 62:25]
  wire [31:0] tensorAcc_io_tensor_rd_0_data_bits_0_6; // @[Compute.scala 62:25]
  wire [31:0] tensorAcc_io_tensor_rd_0_data_bits_0_7; // @[Compute.scala 62:25]
  wire [31:0] tensorAcc_io_tensor_rd_0_data_bits_0_8; // @[Compute.scala 62:25]
  wire [31:0] tensorAcc_io_tensor_rd_0_data_bits_0_9; // @[Compute.scala 62:25]
  wire [31:0] tensorAcc_io_tensor_rd_0_data_bits_0_10; // @[Compute.scala 62:25]
  wire [31:0] tensorAcc_io_tensor_rd_0_data_bits_0_11; // @[Compute.scala 62:25]
  wire [31:0] tensorAcc_io_tensor_rd_0_data_bits_0_12; // @[Compute.scala 62:25]
  wire [31:0] tensorAcc_io_tensor_rd_0_data_bits_0_13; // @[Compute.scala 62:25]
  wire [31:0] tensorAcc_io_tensor_rd_0_data_bits_0_14; // @[Compute.scala 62:25]
  wire [31:0] tensorAcc_io_tensor_rd_0_data_bits_0_15; // @[Compute.scala 62:25]
  wire  tensorAcc_io_tensor_wr_0_valid; // @[Compute.scala 62:25]
  wire [10:0] tensorAcc_io_tensor_wr_0_bits_idx; // @[Compute.scala 62:25]
  wire [31:0] tensorAcc_io_tensor_wr_0_bits_data_0_0; // @[Compute.scala 62:25]
  wire [31:0] tensorAcc_io_tensor_wr_0_bits_data_0_1; // @[Compute.scala 62:25]
  wire [31:0] tensorAcc_io_tensor_wr_0_bits_data_0_2; // @[Compute.scala 62:25]
  wire [31:0] tensorAcc_io_tensor_wr_0_bits_data_0_3; // @[Compute.scala 62:25]
  wire [31:0] tensorAcc_io_tensor_wr_0_bits_data_0_4; // @[Compute.scala 62:25]
  wire [31:0] tensorAcc_io_tensor_wr_0_bits_data_0_5; // @[Compute.scala 62:25]
  wire [31:0] tensorAcc_io_tensor_wr_0_bits_data_0_6; // @[Compute.scala 62:25]
  wire [31:0] tensorAcc_io_tensor_wr_0_bits_data_0_7; // @[Compute.scala 62:25]
  wire [31:0] tensorAcc_io_tensor_wr_0_bits_data_0_8; // @[Compute.scala 62:25]
  wire [31:0] tensorAcc_io_tensor_wr_0_bits_data_0_9; // @[Compute.scala 62:25]
  wire [31:0] tensorAcc_io_tensor_wr_0_bits_data_0_10; // @[Compute.scala 62:25]
  wire [31:0] tensorAcc_io_tensor_wr_0_bits_data_0_11; // @[Compute.scala 62:25]
  wire [31:0] tensorAcc_io_tensor_wr_0_bits_data_0_12; // @[Compute.scala 62:25]
  wire [31:0] tensorAcc_io_tensor_wr_0_bits_data_0_13; // @[Compute.scala 62:25]
  wire [31:0] tensorAcc_io_tensor_wr_0_bits_data_0_14; // @[Compute.scala 62:25]
  wire [31:0] tensorAcc_io_tensor_wr_0_bits_data_0_15; // @[Compute.scala 62:25]
  wire  tensorGemm_clock; // @[Compute.scala 63:26]
  wire  tensorGemm_reset; // @[Compute.scala 63:26]
  wire  tensorGemm_io_start; // @[Compute.scala 63:26]
  wire  tensorGemm_io_done; // @[Compute.scala 63:26]
  wire [9:0] tensorGemm_io_dec_wgt_1; // @[Compute.scala 63:26]
  wire [9:0] tensorGemm_io_dec_wgt_0; // @[Compute.scala 63:26]
  wire [10:0] tensorGemm_io_dec_inp_1; // @[Compute.scala 63:26]
  wire [10:0] tensorGemm_io_dec_inp_0; // @[Compute.scala 63:26]
  wire [10:0] tensorGemm_io_dec_acc_1; // @[Compute.scala 63:26]
  wire [10:0] tensorGemm_io_dec_acc_0; // @[Compute.scala 63:26]
  wire  tensorGemm_io_dec_empty_0; // @[Compute.scala 63:26]
  wire [13:0] tensorGemm_io_dec_lp_1; // @[Compute.scala 63:26]
  wire [13:0] tensorGemm_io_dec_lp_0; // @[Compute.scala 63:26]
  wire [13:0] tensorGemm_io_dec_uop_end; // @[Compute.scala 63:26]
  wire [12:0] tensorGemm_io_dec_uop_begin; // @[Compute.scala 63:26]
  wire  tensorGemm_io_dec_reset; // @[Compute.scala 63:26]
  wire  tensorGemm_io_dec_push_next; // @[Compute.scala 63:26]
  wire  tensorGemm_io_dec_push_prev; // @[Compute.scala 63:26]
  wire  tensorGemm_io_dec_pop_next; // @[Compute.scala 63:26]
  wire  tensorGemm_io_dec_pop_prev; // @[Compute.scala 63:26]
  wire [2:0] tensorGemm_io_dec_op; // @[Compute.scala 63:26]
  wire  tensorGemm_io_uop_idx_valid; // @[Compute.scala 63:26]
  wire [10:0] tensorGemm_io_uop_idx_bits; // @[Compute.scala 63:26]
  wire  tensorGemm_io_uop_data_valid; // @[Compute.scala 63:26]
  wire [9:0] tensorGemm_io_uop_data_bits_u2; // @[Compute.scala 63:26]
  wire [10:0] tensorGemm_io_uop_data_bits_u1; // @[Compute.scala 63:26]
  wire [10:0] tensorGemm_io_uop_data_bits_u0; // @[Compute.scala 63:26]
  wire  tensorGemm_io_inp_rd_0_idx_valid; // @[Compute.scala 63:26]
  wire [10:0] tensorGemm_io_inp_rd_0_idx_bits; // @[Compute.scala 63:26]
  wire  tensorGemm_io_inp_rd_0_data_valid; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_inp_rd_0_data_bits_0_0; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_inp_rd_0_data_bits_0_1; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_inp_rd_0_data_bits_0_2; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_inp_rd_0_data_bits_0_3; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_inp_rd_0_data_bits_0_4; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_inp_rd_0_data_bits_0_5; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_inp_rd_0_data_bits_0_6; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_inp_rd_0_data_bits_0_7; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_inp_rd_0_data_bits_0_8; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_inp_rd_0_data_bits_0_9; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_inp_rd_0_data_bits_0_10; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_inp_rd_0_data_bits_0_11; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_inp_rd_0_data_bits_0_12; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_inp_rd_0_data_bits_0_13; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_inp_rd_0_data_bits_0_14; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_inp_rd_0_data_bits_0_15; // @[Compute.scala 63:26]
  wire  tensorGemm_io_wgt_rd_0_idx_valid; // @[Compute.scala 63:26]
  wire [9:0] tensorGemm_io_wgt_rd_0_idx_bits; // @[Compute.scala 63:26]
  wire  tensorGemm_io_wgt_rd_0_data_valid; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_0_0; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_0_1; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_0_2; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_0_3; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_0_4; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_0_5; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_0_6; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_0_7; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_0_8; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_0_9; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_0_10; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_0_11; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_0_12; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_0_13; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_0_14; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_0_15; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_1_0; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_1_1; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_1_2; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_1_3; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_1_4; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_1_5; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_1_6; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_1_7; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_1_8; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_1_9; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_1_10; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_1_11; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_1_12; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_1_13; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_1_14; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_1_15; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_2_0; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_2_1; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_2_2; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_2_3; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_2_4; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_2_5; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_2_6; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_2_7; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_2_8; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_2_9; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_2_10; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_2_11; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_2_12; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_2_13; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_2_14; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_2_15; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_3_0; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_3_1; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_3_2; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_3_3; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_3_4; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_3_5; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_3_6; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_3_7; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_3_8; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_3_9; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_3_10; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_3_11; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_3_12; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_3_13; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_3_14; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_3_15; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_4_0; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_4_1; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_4_2; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_4_3; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_4_4; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_4_5; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_4_6; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_4_7; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_4_8; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_4_9; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_4_10; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_4_11; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_4_12; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_4_13; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_4_14; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_4_15; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_5_0; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_5_1; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_5_2; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_5_3; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_5_4; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_5_5; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_5_6; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_5_7; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_5_8; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_5_9; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_5_10; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_5_11; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_5_12; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_5_13; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_5_14; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_5_15; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_6_0; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_6_1; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_6_2; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_6_3; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_6_4; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_6_5; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_6_6; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_6_7; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_6_8; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_6_9; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_6_10; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_6_11; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_6_12; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_6_13; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_6_14; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_6_15; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_7_0; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_7_1; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_7_2; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_7_3; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_7_4; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_7_5; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_7_6; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_7_7; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_7_8; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_7_9; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_7_10; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_7_11; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_7_12; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_7_13; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_7_14; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_7_15; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_8_0; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_8_1; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_8_2; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_8_3; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_8_4; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_8_5; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_8_6; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_8_7; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_8_8; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_8_9; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_8_10; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_8_11; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_8_12; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_8_13; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_8_14; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_8_15; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_9_0; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_9_1; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_9_2; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_9_3; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_9_4; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_9_5; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_9_6; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_9_7; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_9_8; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_9_9; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_9_10; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_9_11; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_9_12; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_9_13; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_9_14; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_9_15; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_10_0; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_10_1; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_10_2; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_10_3; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_10_4; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_10_5; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_10_6; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_10_7; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_10_8; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_10_9; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_10_10; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_10_11; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_10_12; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_10_13; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_10_14; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_10_15; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_11_0; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_11_1; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_11_2; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_11_3; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_11_4; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_11_5; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_11_6; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_11_7; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_11_8; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_11_9; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_11_10; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_11_11; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_11_12; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_11_13; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_11_14; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_11_15; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_12_0; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_12_1; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_12_2; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_12_3; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_12_4; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_12_5; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_12_6; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_12_7; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_12_8; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_12_9; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_12_10; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_12_11; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_12_12; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_12_13; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_12_14; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_12_15; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_13_0; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_13_1; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_13_2; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_13_3; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_13_4; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_13_5; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_13_6; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_13_7; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_13_8; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_13_9; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_13_10; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_13_11; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_13_12; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_13_13; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_13_14; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_13_15; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_14_0; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_14_1; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_14_2; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_14_3; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_14_4; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_14_5; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_14_6; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_14_7; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_14_8; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_14_9; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_14_10; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_14_11; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_14_12; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_14_13; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_14_14; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_14_15; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_15_0; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_15_1; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_15_2; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_15_3; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_15_4; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_15_5; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_15_6; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_15_7; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_15_8; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_15_9; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_15_10; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_15_11; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_15_12; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_15_13; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_15_14; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_wgt_rd_0_data_bits_15_15; // @[Compute.scala 63:26]
  wire  tensorGemm_io_acc_rd_0_idx_valid; // @[Compute.scala 63:26]
  wire [10:0] tensorGemm_io_acc_rd_0_idx_bits; // @[Compute.scala 63:26]
  wire  tensorGemm_io_acc_rd_0_data_valid; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_acc_rd_0_data_bits_0_0; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_acc_rd_0_data_bits_0_1; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_acc_rd_0_data_bits_0_2; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_acc_rd_0_data_bits_0_3; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_acc_rd_0_data_bits_0_4; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_acc_rd_0_data_bits_0_5; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_acc_rd_0_data_bits_0_6; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_acc_rd_0_data_bits_0_7; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_acc_rd_0_data_bits_0_8; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_acc_rd_0_data_bits_0_9; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_acc_rd_0_data_bits_0_10; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_acc_rd_0_data_bits_0_11; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_acc_rd_0_data_bits_0_12; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_acc_rd_0_data_bits_0_13; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_acc_rd_0_data_bits_0_14; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_acc_rd_0_data_bits_0_15; // @[Compute.scala 63:26]
  wire  tensorGemm_io_acc_wr_0_valid; // @[Compute.scala 63:26]
  wire [10:0] tensorGemm_io_acc_wr_0_bits_idx; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_acc_wr_0_bits_data_0_0; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_acc_wr_0_bits_data_0_1; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_acc_wr_0_bits_data_0_2; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_acc_wr_0_bits_data_0_3; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_acc_wr_0_bits_data_0_4; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_acc_wr_0_bits_data_0_5; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_acc_wr_0_bits_data_0_6; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_acc_wr_0_bits_data_0_7; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_acc_wr_0_bits_data_0_8; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_acc_wr_0_bits_data_0_9; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_acc_wr_0_bits_data_0_10; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_acc_wr_0_bits_data_0_11; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_acc_wr_0_bits_data_0_12; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_acc_wr_0_bits_data_0_13; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_acc_wr_0_bits_data_0_14; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_acc_wr_0_bits_data_0_15; // @[Compute.scala 63:26]
  wire  tensorGemm_io_out_rd_0_data_valid; // @[Compute.scala 63:26]
  wire  tensorGemm_io_out_wr_0_valid; // @[Compute.scala 63:26]
  wire [10:0] tensorGemm_io_out_wr_0_bits_idx; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_out_wr_0_bits_data_0_0; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_out_wr_0_bits_data_0_1; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_out_wr_0_bits_data_0_2; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_out_wr_0_bits_data_0_3; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_out_wr_0_bits_data_0_4; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_out_wr_0_bits_data_0_5; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_out_wr_0_bits_data_0_6; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_out_wr_0_bits_data_0_7; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_out_wr_0_bits_data_0_8; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_out_wr_0_bits_data_0_9; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_out_wr_0_bits_data_0_10; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_out_wr_0_bits_data_0_11; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_out_wr_0_bits_data_0_12; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_out_wr_0_bits_data_0_13; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_out_wr_0_bits_data_0_14; // @[Compute.scala 63:26]
  wire [31:0] tensorGemm_io_out_wr_0_bits_data_0_15; // @[Compute.scala 63:26]
  wire  tensorAlu_clock; // @[Compute.scala 64:25]
  wire  tensorAlu_reset; // @[Compute.scala 64:25]
  wire  tensorAlu_io_start; // @[Compute.scala 64:25]
  wire  tensorAlu_io_done; // @[Compute.scala 64:25]
  wire [15:0] tensorAlu_io_dec_alu_imm; // @[Compute.scala 64:25]
  wire  tensorAlu_io_dec_alu_use_imm; // @[Compute.scala 64:25]
  wire [2:0] tensorAlu_io_dec_alu_op; // @[Compute.scala 64:25]
  wire [10:0] tensorAlu_io_dec_src_1; // @[Compute.scala 64:25]
  wire [10:0] tensorAlu_io_dec_src_0; // @[Compute.scala 64:25]
  wire [10:0] tensorAlu_io_dec_dst_1; // @[Compute.scala 64:25]
  wire [10:0] tensorAlu_io_dec_dst_0; // @[Compute.scala 64:25]
  wire [13:0] tensorAlu_io_dec_lp_1; // @[Compute.scala 64:25]
  wire [13:0] tensorAlu_io_dec_lp_0; // @[Compute.scala 64:25]
  wire [13:0] tensorAlu_io_dec_uop_end; // @[Compute.scala 64:25]
  wire [12:0] tensorAlu_io_dec_uop_begin; // @[Compute.scala 64:25]
  wire  tensorAlu_io_uop_idx_valid; // @[Compute.scala 64:25]
  wire [10:0] tensorAlu_io_uop_idx_bits; // @[Compute.scala 64:25]
  wire [9:0] tensorAlu_io_uop_data_bits_u2; // @[Compute.scala 64:25]
  wire [10:0] tensorAlu_io_uop_data_bits_u1; // @[Compute.scala 64:25]
  wire [10:0] tensorAlu_io_uop_data_bits_u0; // @[Compute.scala 64:25]
  wire  tensorAlu_io_acc_rd_0_idx_valid; // @[Compute.scala 64:25]
  wire [10:0] tensorAlu_io_acc_rd_0_idx_bits; // @[Compute.scala 64:25]
  wire  tensorAlu_io_acc_rd_0_data_valid; // @[Compute.scala 64:25]
  wire [31:0] tensorAlu_io_acc_rd_0_data_bits_0_0; // @[Compute.scala 64:25]
  wire [31:0] tensorAlu_io_acc_rd_0_data_bits_0_1; // @[Compute.scala 64:25]
  wire [31:0] tensorAlu_io_acc_rd_0_data_bits_0_2; // @[Compute.scala 64:25]
  wire [31:0] tensorAlu_io_acc_rd_0_data_bits_0_3; // @[Compute.scala 64:25]
  wire [31:0] tensorAlu_io_acc_rd_0_data_bits_0_4; // @[Compute.scala 64:25]
  wire [31:0] tensorAlu_io_acc_rd_0_data_bits_0_5; // @[Compute.scala 64:25]
  wire [31:0] tensorAlu_io_acc_rd_0_data_bits_0_6; // @[Compute.scala 64:25]
  wire [31:0] tensorAlu_io_acc_rd_0_data_bits_0_7; // @[Compute.scala 64:25]
  wire [31:0] tensorAlu_io_acc_rd_0_data_bits_0_8; // @[Compute.scala 64:25]
  wire [31:0] tensorAlu_io_acc_rd_0_data_bits_0_9; // @[Compute.scala 64:25]
  wire [31:0] tensorAlu_io_acc_rd_0_data_bits_0_10; // @[Compute.scala 64:25]
  wire [31:0] tensorAlu_io_acc_rd_0_data_bits_0_11; // @[Compute.scala 64:25]
  wire [31:0] tensorAlu_io_acc_rd_0_data_bits_0_12; // @[Compute.scala 64:25]
  wire [31:0] tensorAlu_io_acc_rd_0_data_bits_0_13; // @[Compute.scala 64:25]
  wire [31:0] tensorAlu_io_acc_rd_0_data_bits_0_14; // @[Compute.scala 64:25]
  wire [31:0] tensorAlu_io_acc_rd_0_data_bits_0_15; // @[Compute.scala 64:25]
  wire  tensorAlu_io_acc_wr_0_valid; // @[Compute.scala 64:25]
  wire [10:0] tensorAlu_io_acc_wr_0_bits_idx; // @[Compute.scala 64:25]
  wire [31:0] tensorAlu_io_acc_wr_0_bits_data_0_0; // @[Compute.scala 64:25]
  wire [31:0] tensorAlu_io_acc_wr_0_bits_data_0_1; // @[Compute.scala 64:25]
  wire [31:0] tensorAlu_io_acc_wr_0_bits_data_0_2; // @[Compute.scala 64:25]
  wire [31:0] tensorAlu_io_acc_wr_0_bits_data_0_3; // @[Compute.scala 64:25]
  wire [31:0] tensorAlu_io_acc_wr_0_bits_data_0_4; // @[Compute.scala 64:25]
  wire [31:0] tensorAlu_io_acc_wr_0_bits_data_0_5; // @[Compute.scala 64:25]
  wire [31:0] tensorAlu_io_acc_wr_0_bits_data_0_6; // @[Compute.scala 64:25]
  wire [31:0] tensorAlu_io_acc_wr_0_bits_data_0_7; // @[Compute.scala 64:25]
  wire [31:0] tensorAlu_io_acc_wr_0_bits_data_0_8; // @[Compute.scala 64:25]
  wire [31:0] tensorAlu_io_acc_wr_0_bits_data_0_9; // @[Compute.scala 64:25]
  wire [31:0] tensorAlu_io_acc_wr_0_bits_data_0_10; // @[Compute.scala 64:25]
  wire [31:0] tensorAlu_io_acc_wr_0_bits_data_0_11; // @[Compute.scala 64:25]
  wire [31:0] tensorAlu_io_acc_wr_0_bits_data_0_12; // @[Compute.scala 64:25]
  wire [31:0] tensorAlu_io_acc_wr_0_bits_data_0_13; // @[Compute.scala 64:25]
  wire [31:0] tensorAlu_io_acc_wr_0_bits_data_0_14; // @[Compute.scala 64:25]
  wire [31:0] tensorAlu_io_acc_wr_0_bits_data_0_15; // @[Compute.scala 64:25]
  wire  tensorAlu_io_out_rd_0_data_valid; // @[Compute.scala 64:25]
  wire  tensorAlu_io_out_wr_0_valid; // @[Compute.scala 64:25]
  wire [10:0] tensorAlu_io_out_wr_0_bits_idx; // @[Compute.scala 64:25]
  wire [31:0] tensorAlu_io_out_wr_0_bits_data_0_0; // @[Compute.scala 64:25]
  wire [31:0] tensorAlu_io_out_wr_0_bits_data_0_1; // @[Compute.scala 64:25]
  wire [31:0] tensorAlu_io_out_wr_0_bits_data_0_2; // @[Compute.scala 64:25]
  wire [31:0] tensorAlu_io_out_wr_0_bits_data_0_3; // @[Compute.scala 64:25]
  wire [31:0] tensorAlu_io_out_wr_0_bits_data_0_4; // @[Compute.scala 64:25]
  wire [31:0] tensorAlu_io_out_wr_0_bits_data_0_5; // @[Compute.scala 64:25]
  wire [31:0] tensorAlu_io_out_wr_0_bits_data_0_6; // @[Compute.scala 64:25]
  wire [31:0] tensorAlu_io_out_wr_0_bits_data_0_7; // @[Compute.scala 64:25]
  wire [31:0] tensorAlu_io_out_wr_0_bits_data_0_8; // @[Compute.scala 64:25]
  wire [31:0] tensorAlu_io_out_wr_0_bits_data_0_9; // @[Compute.scala 64:25]
  wire [31:0] tensorAlu_io_out_wr_0_bits_data_0_10; // @[Compute.scala 64:25]
  wire [31:0] tensorAlu_io_out_wr_0_bits_data_0_11; // @[Compute.scala 64:25]
  wire [31:0] tensorAlu_io_out_wr_0_bits_data_0_12; // @[Compute.scala 64:25]
  wire [31:0] tensorAlu_io_out_wr_0_bits_data_0_13; // @[Compute.scala 64:25]
  wire [31:0] tensorAlu_io_out_wr_0_bits_data_0_14; // @[Compute.scala 64:25]
  wire [31:0] tensorAlu_io_out_wr_0_bits_data_0_15; // @[Compute.scala 64:25]
  wire  inst_q_clock; // @[Compute.scala 69:22]
  wire  inst_q_reset; // @[Compute.scala 69:22]
  wire  inst_q_io_enq_ready; // @[Compute.scala 69:22]
  wire  inst_q_io_enq_valid; // @[Compute.scala 69:22]
  wire [127:0] inst_q_io_enq_bits; // @[Compute.scala 69:22]
  wire  inst_q_io_deq_ready; // @[Compute.scala 69:22]
  wire  inst_q_io_deq_valid; // @[Compute.scala 69:22]
  wire [127:0] inst_q_io_deq_bits; // @[Compute.scala 69:22]
  wire [127:0] dec_io_inst; // @[Compute.scala 72:19]
  wire  dec_io_push_next; // @[Compute.scala 72:19]
  wire  dec_io_push_prev; // @[Compute.scala 72:19]
  wire  dec_io_pop_next; // @[Compute.scala 72:19]
  wire  dec_io_pop_prev; // @[Compute.scala 72:19]
  wire  dec_io_isLoadAcc; // @[Compute.scala 72:19]
  wire  dec_io_isLoadUop; // @[Compute.scala 72:19]
  wire  dec_io_isSync; // @[Compute.scala 72:19]
  wire  dec_io_isAlu; // @[Compute.scala 72:19]
  wire  dec_io_isGemm; // @[Compute.scala 72:19]
  wire  dec_io_isFinish; // @[Compute.scala 72:19]
  reg [1:0] state; // @[Compute.scala 55:22]
  wire [4:0] inst_type = {dec_io_isFinish,dec_io_isAlu,dec_io_isGemm,dec_io_isLoadAcc,dec_io_isLoadUop}; // @[Cat.scala 30:58]
  wire  _sprev_T = dec_io_pop_prev ? s_0_io_sready : 1'h1; // @[Compute.scala 82:40]
  wire  sprev = inst_q_io_deq_valid & _sprev_T; // @[Compute.scala 82:35]
  wire  _snext_T = dec_io_pop_next ? s_1_io_sready : 1'h1; // @[Compute.scala 83:40]
  wire  snext = inst_q_io_deq_valid & _snext_T; // @[Compute.scala 83:35]
  wire  start = snext & sprev; // @[Compute.scala 84:21]
  wire  _done_T_3 = 5'h2 == inst_type ? tensorAcc_io_done : 5'h1 == inst_type & loadUop_io_done; // @[Mux.scala 80:57]
  wire  _done_T_5 = 5'h4 == inst_type ? tensorGemm_io_done : _done_T_3; // @[Mux.scala 80:57]
  wire  _done_T_7 = 5'h8 == inst_type ? tensorAlu_io_done : _done_T_5; // @[Mux.scala 80:57]
  wire  done = 5'h10 == inst_type | _done_T_7; // @[Mux.scala 80:57]
  wire  _T = 2'h0 == state; // @[Conditional.scala 37:30]
  wire [1:0] _GEN_0 = |inst_type ? 2'h2 : state; // @[Compute.scala 104:35 Compute.scala 105:17 Compute.scala 55:22]
  wire  _T_2 = 2'h1 == state; // @[Conditional.scala 37:30]
  wire  _T_3 = 2'h2 == state; // @[Conditional.scala 37:30]
  wire [1:0] _GEN_3 = done ? 2'h0 : state; // @[Compute.scala 113:18 Compute.scala 114:15 Compute.scala 55:22]
  wire  _inst_q_io_deq_ready_T_1 = state == 2'h2 & done; // @[Compute.scala 121:42]
  wire  _inst_q_io_deq_ready_T_3 = state == 2'h2 & done | state == 2'h1; // @[Compute.scala 121:50]
  wire  _loadUop_io_start_T_1 = state == 2'h0 & start; // @[Compute.scala 124:39]
  reg  tensorAcc_io_tensor_rd_0_idx_REG; // @[Compute.scala 150:14]
  reg  tensorAcc_io_tensor_wr_0_REG; // @[Compute.scala 154:14]
  reg  tensorGemm_io_start_REG; // @[Compute.scala 162:33]
  wire [127:0] _tensorGemm_io_dec_WIRE_1 = inst_q_io_deq_bits;
  reg  tensorGemm_io_acc_rd_0_data_valid_REG; // @[Compute.scala 170:55]
  reg  tensorAlu_io_start_REG; // @[Compute.scala 180:32]
  reg  tensorAlu_io_acc_rd_0_data_valid_REG; // @[Compute.scala 186:55]
  reg  io_out_wr_0_valid_REG; // @[Compute.scala 208:12]
  reg  io_out_wr_0_bits_idx_REG; // @[Compute.scala 210:12]
  reg  outDataBits_0_REG; // @[Compute.scala 221:14]
  wire [255:0] srcGemFlat_lo = {tensorGemm_io_out_wr_0_bits_data_0_7,tensorGemm_io_out_wr_0_bits_data_0_6,
    tensorGemm_io_out_wr_0_bits_data_0_5,tensorGemm_io_out_wr_0_bits_data_0_4,tensorGemm_io_out_wr_0_bits_data_0_3,
    tensorGemm_io_out_wr_0_bits_data_0_2,tensorGemm_io_out_wr_0_bits_data_0_1,tensorGemm_io_out_wr_0_bits_data_0_0}; // @[Compute.scala 219:56]
  wire [511:0] srcGemFlat = {tensorGemm_io_out_wr_0_bits_data_0_15,tensorGemm_io_out_wr_0_bits_data_0_14,
    tensorGemm_io_out_wr_0_bits_data_0_13,tensorGemm_io_out_wr_0_bits_data_0_12,tensorGemm_io_out_wr_0_bits_data_0_11,
    tensorGemm_io_out_wr_0_bits_data_0_10,tensorGemm_io_out_wr_0_bits_data_0_9,tensorGemm_io_out_wr_0_bits_data_0_8,
    srcGemFlat_lo}; // @[Compute.scala 219:56]
  wire [255:0] srcAluFlat_lo = {tensorAlu_io_out_wr_0_bits_data_0_7,tensorAlu_io_out_wr_0_bits_data_0_6,
    tensorAlu_io_out_wr_0_bits_data_0_5,tensorAlu_io_out_wr_0_bits_data_0_4,tensorAlu_io_out_wr_0_bits_data_0_3,
    tensorAlu_io_out_wr_0_bits_data_0_2,tensorAlu_io_out_wr_0_bits_data_0_1,tensorAlu_io_out_wr_0_bits_data_0_0}; // @[Compute.scala 218:55]
  wire [511:0] srcAluFlat = {tensorAlu_io_out_wr_0_bits_data_0_15,tensorAlu_io_out_wr_0_bits_data_0_14,
    tensorAlu_io_out_wr_0_bits_data_0_13,tensorAlu_io_out_wr_0_bits_data_0_12,tensorAlu_io_out_wr_0_bits_data_0_11,
    tensorAlu_io_out_wr_0_bits_data_0_10,tensorAlu_io_out_wr_0_bits_data_0_9,tensorAlu_io_out_wr_0_bits_data_0_8,
    srcAluFlat_lo}; // @[Compute.scala 218:55]
  wire [511:0] outDataBits_0 = outDataBits_0_REG ? srcGemFlat : srcAluFlat; // @[Compute.scala 220:28]
  Semaphore s_0 ( // @[Compute.scala 58:11]
    .clock(s_0_clock),
    .reset(s_0_reset),
    .io_spost(s_0_io_spost),
    .io_swait(s_0_io_swait),
    .io_sready(s_0_io_sready)
  );
  Semaphore s_1 ( // @[Compute.scala 58:11]
    .clock(s_1_clock),
    .reset(s_1_reset),
    .io_spost(s_1_io_spost),
    .io_swait(s_1_io_swait),
    .io_sready(s_1_io_sready)
  );
  LoadUopTop loadUop ( // @[Compute.scala 61:23]
    .clock(loadUop_clock),
    .reset(loadUop_reset),
    .io_start(loadUop_io_start),
    .io_done(loadUop_io_done),
    .io_inst(loadUop_io_inst),
    .io_baddr(loadUop_io_baddr),
    .io_vme_rd_cmd_ready(loadUop_io_vme_rd_cmd_ready),
    .io_vme_rd_cmd_valid(loadUop_io_vme_rd_cmd_valid),
    .io_vme_rd_cmd_bits_addr(loadUop_io_vme_rd_cmd_bits_addr),
    .io_vme_rd_cmd_bits_len(loadUop_io_vme_rd_cmd_bits_len),
    .io_vme_rd_cmd_bits_tag(loadUop_io_vme_rd_cmd_bits_tag),
    .io_vme_rd_data_valid(loadUop_io_vme_rd_data_valid),
    .io_vme_rd_data_bits_data(loadUop_io_vme_rd_data_bits_data),
    .io_vme_rd_data_bits_tag(loadUop_io_vme_rd_data_bits_tag),
    .io_vme_rd_data_bits_last(loadUop_io_vme_rd_data_bits_last),
    .io_uop_idx_valid(loadUop_io_uop_idx_valid),
    .io_uop_idx_bits(loadUop_io_uop_idx_bits),
    .io_uop_data_valid(loadUop_io_uop_data_valid),
    .io_uop_data_bits_u2(loadUop_io_uop_data_bits_u2),
    .io_uop_data_bits_u1(loadUop_io_uop_data_bits_u1),
    .io_uop_data_bits_u0(loadUop_io_uop_data_bits_u0)
  );
  TensorLoadAcc tensorAcc ( // @[Compute.scala 62:25]
    .clock(tensorAcc_clock),
    .reset(tensorAcc_reset),
    .io_start(tensorAcc_io_start),
    .io_done(tensorAcc_io_done),
    .io_inst(tensorAcc_io_inst),
    .io_baddr(tensorAcc_io_baddr),
    .io_vme_rd_cmd_ready(tensorAcc_io_vme_rd_cmd_ready),
    .io_vme_rd_cmd_valid(tensorAcc_io_vme_rd_cmd_valid),
    .io_vme_rd_cmd_bits_addr(tensorAcc_io_vme_rd_cmd_bits_addr),
    .io_vme_rd_cmd_bits_len(tensorAcc_io_vme_rd_cmd_bits_len),
    .io_vme_rd_cmd_bits_tag(tensorAcc_io_vme_rd_cmd_bits_tag),
    .io_vme_rd_data_valid(tensorAcc_io_vme_rd_data_valid),
    .io_vme_rd_data_bits_data(tensorAcc_io_vme_rd_data_bits_data),
    .io_vme_rd_data_bits_tag(tensorAcc_io_vme_rd_data_bits_tag),
    .io_tensor_rd_0_idx_valid(tensorAcc_io_tensor_rd_0_idx_valid),
    .io_tensor_rd_0_idx_bits(tensorAcc_io_tensor_rd_0_idx_bits),
    .io_tensor_rd_0_data_valid(tensorAcc_io_tensor_rd_0_data_valid),
    .io_tensor_rd_0_data_bits_0_0(tensorAcc_io_tensor_rd_0_data_bits_0_0),
    .io_tensor_rd_0_data_bits_0_1(tensorAcc_io_tensor_rd_0_data_bits_0_1),
    .io_tensor_rd_0_data_bits_0_2(tensorAcc_io_tensor_rd_0_data_bits_0_2),
    .io_tensor_rd_0_data_bits_0_3(tensorAcc_io_tensor_rd_0_data_bits_0_3),
    .io_tensor_rd_0_data_bits_0_4(tensorAcc_io_tensor_rd_0_data_bits_0_4),
    .io_tensor_rd_0_data_bits_0_5(tensorAcc_io_tensor_rd_0_data_bits_0_5),
    .io_tensor_rd_0_data_bits_0_6(tensorAcc_io_tensor_rd_0_data_bits_0_6),
    .io_tensor_rd_0_data_bits_0_7(tensorAcc_io_tensor_rd_0_data_bits_0_7),
    .io_tensor_rd_0_data_bits_0_8(tensorAcc_io_tensor_rd_0_data_bits_0_8),
    .io_tensor_rd_0_data_bits_0_9(tensorAcc_io_tensor_rd_0_data_bits_0_9),
    .io_tensor_rd_0_data_bits_0_10(tensorAcc_io_tensor_rd_0_data_bits_0_10),
    .io_tensor_rd_0_data_bits_0_11(tensorAcc_io_tensor_rd_0_data_bits_0_11),
    .io_tensor_rd_0_data_bits_0_12(tensorAcc_io_tensor_rd_0_data_bits_0_12),
    .io_tensor_rd_0_data_bits_0_13(tensorAcc_io_tensor_rd_0_data_bits_0_13),
    .io_tensor_rd_0_data_bits_0_14(tensorAcc_io_tensor_rd_0_data_bits_0_14),
    .io_tensor_rd_0_data_bits_0_15(tensorAcc_io_tensor_rd_0_data_bits_0_15),
    .io_tensor_wr_0_valid(tensorAcc_io_tensor_wr_0_valid),
    .io_tensor_wr_0_bits_idx(tensorAcc_io_tensor_wr_0_bits_idx),
    .io_tensor_wr_0_bits_data_0_0(tensorAcc_io_tensor_wr_0_bits_data_0_0),
    .io_tensor_wr_0_bits_data_0_1(tensorAcc_io_tensor_wr_0_bits_data_0_1),
    .io_tensor_wr_0_bits_data_0_2(tensorAcc_io_tensor_wr_0_bits_data_0_2),
    .io_tensor_wr_0_bits_data_0_3(tensorAcc_io_tensor_wr_0_bits_data_0_3),
    .io_tensor_wr_0_bits_data_0_4(tensorAcc_io_tensor_wr_0_bits_data_0_4),
    .io_tensor_wr_0_bits_data_0_5(tensorAcc_io_tensor_wr_0_bits_data_0_5),
    .io_tensor_wr_0_bits_data_0_6(tensorAcc_io_tensor_wr_0_bits_data_0_6),
    .io_tensor_wr_0_bits_data_0_7(tensorAcc_io_tensor_wr_0_bits_data_0_7),
    .io_tensor_wr_0_bits_data_0_8(tensorAcc_io_tensor_wr_0_bits_data_0_8),
    .io_tensor_wr_0_bits_data_0_9(tensorAcc_io_tensor_wr_0_bits_data_0_9),
    .io_tensor_wr_0_bits_data_0_10(tensorAcc_io_tensor_wr_0_bits_data_0_10),
    .io_tensor_wr_0_bits_data_0_11(tensorAcc_io_tensor_wr_0_bits_data_0_11),
    .io_tensor_wr_0_bits_data_0_12(tensorAcc_io_tensor_wr_0_bits_data_0_12),
    .io_tensor_wr_0_bits_data_0_13(tensorAcc_io_tensor_wr_0_bits_data_0_13),
    .io_tensor_wr_0_bits_data_0_14(tensorAcc_io_tensor_wr_0_bits_data_0_14),
    .io_tensor_wr_0_bits_data_0_15(tensorAcc_io_tensor_wr_0_bits_data_0_15)
  );
  TensorGemm tensorGemm ( // @[Compute.scala 63:26]
    .clock(tensorGemm_clock),
    .reset(tensorGemm_reset),
    .io_start(tensorGemm_io_start),
    .io_done(tensorGemm_io_done),
    .io_dec_wgt_1(tensorGemm_io_dec_wgt_1),
    .io_dec_wgt_0(tensorGemm_io_dec_wgt_0),
    .io_dec_inp_1(tensorGemm_io_dec_inp_1),
    .io_dec_inp_0(tensorGemm_io_dec_inp_0),
    .io_dec_acc_1(tensorGemm_io_dec_acc_1),
    .io_dec_acc_0(tensorGemm_io_dec_acc_0),
    .io_dec_empty_0(tensorGemm_io_dec_empty_0),
    .io_dec_lp_1(tensorGemm_io_dec_lp_1),
    .io_dec_lp_0(tensorGemm_io_dec_lp_0),
    .io_dec_uop_end(tensorGemm_io_dec_uop_end),
    .io_dec_uop_begin(tensorGemm_io_dec_uop_begin),
    .io_dec_reset(tensorGemm_io_dec_reset),
    .io_dec_push_next(tensorGemm_io_dec_push_next),
    .io_dec_push_prev(tensorGemm_io_dec_push_prev),
    .io_dec_pop_next(tensorGemm_io_dec_pop_next),
    .io_dec_pop_prev(tensorGemm_io_dec_pop_prev),
    .io_dec_op(tensorGemm_io_dec_op),
    .io_uop_idx_valid(tensorGemm_io_uop_idx_valid),
    .io_uop_idx_bits(tensorGemm_io_uop_idx_bits),
    .io_uop_data_valid(tensorGemm_io_uop_data_valid),
    .io_uop_data_bits_u2(tensorGemm_io_uop_data_bits_u2),
    .io_uop_data_bits_u1(tensorGemm_io_uop_data_bits_u1),
    .io_uop_data_bits_u0(tensorGemm_io_uop_data_bits_u0),
    .io_inp_rd_0_idx_valid(tensorGemm_io_inp_rd_0_idx_valid),
    .io_inp_rd_0_idx_bits(tensorGemm_io_inp_rd_0_idx_bits),
    .io_inp_rd_0_data_valid(tensorGemm_io_inp_rd_0_data_valid),
    .io_inp_rd_0_data_bits_0_0(tensorGemm_io_inp_rd_0_data_bits_0_0),
    .io_inp_rd_0_data_bits_0_1(tensorGemm_io_inp_rd_0_data_bits_0_1),
    .io_inp_rd_0_data_bits_0_2(tensorGemm_io_inp_rd_0_data_bits_0_2),
    .io_inp_rd_0_data_bits_0_3(tensorGemm_io_inp_rd_0_data_bits_0_3),
    .io_inp_rd_0_data_bits_0_4(tensorGemm_io_inp_rd_0_data_bits_0_4),
    .io_inp_rd_0_data_bits_0_5(tensorGemm_io_inp_rd_0_data_bits_0_5),
    .io_inp_rd_0_data_bits_0_6(tensorGemm_io_inp_rd_0_data_bits_0_6),
    .io_inp_rd_0_data_bits_0_7(tensorGemm_io_inp_rd_0_data_bits_0_7),
    .io_inp_rd_0_data_bits_0_8(tensorGemm_io_inp_rd_0_data_bits_0_8),
    .io_inp_rd_0_data_bits_0_9(tensorGemm_io_inp_rd_0_data_bits_0_9),
    .io_inp_rd_0_data_bits_0_10(tensorGemm_io_inp_rd_0_data_bits_0_10),
    .io_inp_rd_0_data_bits_0_11(tensorGemm_io_inp_rd_0_data_bits_0_11),
    .io_inp_rd_0_data_bits_0_12(tensorGemm_io_inp_rd_0_data_bits_0_12),
    .io_inp_rd_0_data_bits_0_13(tensorGemm_io_inp_rd_0_data_bits_0_13),
    .io_inp_rd_0_data_bits_0_14(tensorGemm_io_inp_rd_0_data_bits_0_14),
    .io_inp_rd_0_data_bits_0_15(tensorGemm_io_inp_rd_0_data_bits_0_15),
    .io_wgt_rd_0_idx_valid(tensorGemm_io_wgt_rd_0_idx_valid),
    .io_wgt_rd_0_idx_bits(tensorGemm_io_wgt_rd_0_idx_bits),
    .io_wgt_rd_0_data_valid(tensorGemm_io_wgt_rd_0_data_valid),
    .io_wgt_rd_0_data_bits_0_0(tensorGemm_io_wgt_rd_0_data_bits_0_0),
    .io_wgt_rd_0_data_bits_0_1(tensorGemm_io_wgt_rd_0_data_bits_0_1),
    .io_wgt_rd_0_data_bits_0_2(tensorGemm_io_wgt_rd_0_data_bits_0_2),
    .io_wgt_rd_0_data_bits_0_3(tensorGemm_io_wgt_rd_0_data_bits_0_3),
    .io_wgt_rd_0_data_bits_0_4(tensorGemm_io_wgt_rd_0_data_bits_0_4),
    .io_wgt_rd_0_data_bits_0_5(tensorGemm_io_wgt_rd_0_data_bits_0_5),
    .io_wgt_rd_0_data_bits_0_6(tensorGemm_io_wgt_rd_0_data_bits_0_6),
    .io_wgt_rd_0_data_bits_0_7(tensorGemm_io_wgt_rd_0_data_bits_0_7),
    .io_wgt_rd_0_data_bits_0_8(tensorGemm_io_wgt_rd_0_data_bits_0_8),
    .io_wgt_rd_0_data_bits_0_9(tensorGemm_io_wgt_rd_0_data_bits_0_9),
    .io_wgt_rd_0_data_bits_0_10(tensorGemm_io_wgt_rd_0_data_bits_0_10),
    .io_wgt_rd_0_data_bits_0_11(tensorGemm_io_wgt_rd_0_data_bits_0_11),
    .io_wgt_rd_0_data_bits_0_12(tensorGemm_io_wgt_rd_0_data_bits_0_12),
    .io_wgt_rd_0_data_bits_0_13(tensorGemm_io_wgt_rd_0_data_bits_0_13),
    .io_wgt_rd_0_data_bits_0_14(tensorGemm_io_wgt_rd_0_data_bits_0_14),
    .io_wgt_rd_0_data_bits_0_15(tensorGemm_io_wgt_rd_0_data_bits_0_15),
    .io_wgt_rd_0_data_bits_1_0(tensorGemm_io_wgt_rd_0_data_bits_1_0),
    .io_wgt_rd_0_data_bits_1_1(tensorGemm_io_wgt_rd_0_data_bits_1_1),
    .io_wgt_rd_0_data_bits_1_2(tensorGemm_io_wgt_rd_0_data_bits_1_2),
    .io_wgt_rd_0_data_bits_1_3(tensorGemm_io_wgt_rd_0_data_bits_1_3),
    .io_wgt_rd_0_data_bits_1_4(tensorGemm_io_wgt_rd_0_data_bits_1_4),
    .io_wgt_rd_0_data_bits_1_5(tensorGemm_io_wgt_rd_0_data_bits_1_5),
    .io_wgt_rd_0_data_bits_1_6(tensorGemm_io_wgt_rd_0_data_bits_1_6),
    .io_wgt_rd_0_data_bits_1_7(tensorGemm_io_wgt_rd_0_data_bits_1_7),
    .io_wgt_rd_0_data_bits_1_8(tensorGemm_io_wgt_rd_0_data_bits_1_8),
    .io_wgt_rd_0_data_bits_1_9(tensorGemm_io_wgt_rd_0_data_bits_1_9),
    .io_wgt_rd_0_data_bits_1_10(tensorGemm_io_wgt_rd_0_data_bits_1_10),
    .io_wgt_rd_0_data_bits_1_11(tensorGemm_io_wgt_rd_0_data_bits_1_11),
    .io_wgt_rd_0_data_bits_1_12(tensorGemm_io_wgt_rd_0_data_bits_1_12),
    .io_wgt_rd_0_data_bits_1_13(tensorGemm_io_wgt_rd_0_data_bits_1_13),
    .io_wgt_rd_0_data_bits_1_14(tensorGemm_io_wgt_rd_0_data_bits_1_14),
    .io_wgt_rd_0_data_bits_1_15(tensorGemm_io_wgt_rd_0_data_bits_1_15),
    .io_wgt_rd_0_data_bits_2_0(tensorGemm_io_wgt_rd_0_data_bits_2_0),
    .io_wgt_rd_0_data_bits_2_1(tensorGemm_io_wgt_rd_0_data_bits_2_1),
    .io_wgt_rd_0_data_bits_2_2(tensorGemm_io_wgt_rd_0_data_bits_2_2),
    .io_wgt_rd_0_data_bits_2_3(tensorGemm_io_wgt_rd_0_data_bits_2_3),
    .io_wgt_rd_0_data_bits_2_4(tensorGemm_io_wgt_rd_0_data_bits_2_4),
    .io_wgt_rd_0_data_bits_2_5(tensorGemm_io_wgt_rd_0_data_bits_2_5),
    .io_wgt_rd_0_data_bits_2_6(tensorGemm_io_wgt_rd_0_data_bits_2_6),
    .io_wgt_rd_0_data_bits_2_7(tensorGemm_io_wgt_rd_0_data_bits_2_7),
    .io_wgt_rd_0_data_bits_2_8(tensorGemm_io_wgt_rd_0_data_bits_2_8),
    .io_wgt_rd_0_data_bits_2_9(tensorGemm_io_wgt_rd_0_data_bits_2_9),
    .io_wgt_rd_0_data_bits_2_10(tensorGemm_io_wgt_rd_0_data_bits_2_10),
    .io_wgt_rd_0_data_bits_2_11(tensorGemm_io_wgt_rd_0_data_bits_2_11),
    .io_wgt_rd_0_data_bits_2_12(tensorGemm_io_wgt_rd_0_data_bits_2_12),
    .io_wgt_rd_0_data_bits_2_13(tensorGemm_io_wgt_rd_0_data_bits_2_13),
    .io_wgt_rd_0_data_bits_2_14(tensorGemm_io_wgt_rd_0_data_bits_2_14),
    .io_wgt_rd_0_data_bits_2_15(tensorGemm_io_wgt_rd_0_data_bits_2_15),
    .io_wgt_rd_0_data_bits_3_0(tensorGemm_io_wgt_rd_0_data_bits_3_0),
    .io_wgt_rd_0_data_bits_3_1(tensorGemm_io_wgt_rd_0_data_bits_3_1),
    .io_wgt_rd_0_data_bits_3_2(tensorGemm_io_wgt_rd_0_data_bits_3_2),
    .io_wgt_rd_0_data_bits_3_3(tensorGemm_io_wgt_rd_0_data_bits_3_3),
    .io_wgt_rd_0_data_bits_3_4(tensorGemm_io_wgt_rd_0_data_bits_3_4),
    .io_wgt_rd_0_data_bits_3_5(tensorGemm_io_wgt_rd_0_data_bits_3_5),
    .io_wgt_rd_0_data_bits_3_6(tensorGemm_io_wgt_rd_0_data_bits_3_6),
    .io_wgt_rd_0_data_bits_3_7(tensorGemm_io_wgt_rd_0_data_bits_3_7),
    .io_wgt_rd_0_data_bits_3_8(tensorGemm_io_wgt_rd_0_data_bits_3_8),
    .io_wgt_rd_0_data_bits_3_9(tensorGemm_io_wgt_rd_0_data_bits_3_9),
    .io_wgt_rd_0_data_bits_3_10(tensorGemm_io_wgt_rd_0_data_bits_3_10),
    .io_wgt_rd_0_data_bits_3_11(tensorGemm_io_wgt_rd_0_data_bits_3_11),
    .io_wgt_rd_0_data_bits_3_12(tensorGemm_io_wgt_rd_0_data_bits_3_12),
    .io_wgt_rd_0_data_bits_3_13(tensorGemm_io_wgt_rd_0_data_bits_3_13),
    .io_wgt_rd_0_data_bits_3_14(tensorGemm_io_wgt_rd_0_data_bits_3_14),
    .io_wgt_rd_0_data_bits_3_15(tensorGemm_io_wgt_rd_0_data_bits_3_15),
    .io_wgt_rd_0_data_bits_4_0(tensorGemm_io_wgt_rd_0_data_bits_4_0),
    .io_wgt_rd_0_data_bits_4_1(tensorGemm_io_wgt_rd_0_data_bits_4_1),
    .io_wgt_rd_0_data_bits_4_2(tensorGemm_io_wgt_rd_0_data_bits_4_2),
    .io_wgt_rd_0_data_bits_4_3(tensorGemm_io_wgt_rd_0_data_bits_4_3),
    .io_wgt_rd_0_data_bits_4_4(tensorGemm_io_wgt_rd_0_data_bits_4_4),
    .io_wgt_rd_0_data_bits_4_5(tensorGemm_io_wgt_rd_0_data_bits_4_5),
    .io_wgt_rd_0_data_bits_4_6(tensorGemm_io_wgt_rd_0_data_bits_4_6),
    .io_wgt_rd_0_data_bits_4_7(tensorGemm_io_wgt_rd_0_data_bits_4_7),
    .io_wgt_rd_0_data_bits_4_8(tensorGemm_io_wgt_rd_0_data_bits_4_8),
    .io_wgt_rd_0_data_bits_4_9(tensorGemm_io_wgt_rd_0_data_bits_4_9),
    .io_wgt_rd_0_data_bits_4_10(tensorGemm_io_wgt_rd_0_data_bits_4_10),
    .io_wgt_rd_0_data_bits_4_11(tensorGemm_io_wgt_rd_0_data_bits_4_11),
    .io_wgt_rd_0_data_bits_4_12(tensorGemm_io_wgt_rd_0_data_bits_4_12),
    .io_wgt_rd_0_data_bits_4_13(tensorGemm_io_wgt_rd_0_data_bits_4_13),
    .io_wgt_rd_0_data_bits_4_14(tensorGemm_io_wgt_rd_0_data_bits_4_14),
    .io_wgt_rd_0_data_bits_4_15(tensorGemm_io_wgt_rd_0_data_bits_4_15),
    .io_wgt_rd_0_data_bits_5_0(tensorGemm_io_wgt_rd_0_data_bits_5_0),
    .io_wgt_rd_0_data_bits_5_1(tensorGemm_io_wgt_rd_0_data_bits_5_1),
    .io_wgt_rd_0_data_bits_5_2(tensorGemm_io_wgt_rd_0_data_bits_5_2),
    .io_wgt_rd_0_data_bits_5_3(tensorGemm_io_wgt_rd_0_data_bits_5_3),
    .io_wgt_rd_0_data_bits_5_4(tensorGemm_io_wgt_rd_0_data_bits_5_4),
    .io_wgt_rd_0_data_bits_5_5(tensorGemm_io_wgt_rd_0_data_bits_5_5),
    .io_wgt_rd_0_data_bits_5_6(tensorGemm_io_wgt_rd_0_data_bits_5_6),
    .io_wgt_rd_0_data_bits_5_7(tensorGemm_io_wgt_rd_0_data_bits_5_7),
    .io_wgt_rd_0_data_bits_5_8(tensorGemm_io_wgt_rd_0_data_bits_5_8),
    .io_wgt_rd_0_data_bits_5_9(tensorGemm_io_wgt_rd_0_data_bits_5_9),
    .io_wgt_rd_0_data_bits_5_10(tensorGemm_io_wgt_rd_0_data_bits_5_10),
    .io_wgt_rd_0_data_bits_5_11(tensorGemm_io_wgt_rd_0_data_bits_5_11),
    .io_wgt_rd_0_data_bits_5_12(tensorGemm_io_wgt_rd_0_data_bits_5_12),
    .io_wgt_rd_0_data_bits_5_13(tensorGemm_io_wgt_rd_0_data_bits_5_13),
    .io_wgt_rd_0_data_bits_5_14(tensorGemm_io_wgt_rd_0_data_bits_5_14),
    .io_wgt_rd_0_data_bits_5_15(tensorGemm_io_wgt_rd_0_data_bits_5_15),
    .io_wgt_rd_0_data_bits_6_0(tensorGemm_io_wgt_rd_0_data_bits_6_0),
    .io_wgt_rd_0_data_bits_6_1(tensorGemm_io_wgt_rd_0_data_bits_6_1),
    .io_wgt_rd_0_data_bits_6_2(tensorGemm_io_wgt_rd_0_data_bits_6_2),
    .io_wgt_rd_0_data_bits_6_3(tensorGemm_io_wgt_rd_0_data_bits_6_3),
    .io_wgt_rd_0_data_bits_6_4(tensorGemm_io_wgt_rd_0_data_bits_6_4),
    .io_wgt_rd_0_data_bits_6_5(tensorGemm_io_wgt_rd_0_data_bits_6_5),
    .io_wgt_rd_0_data_bits_6_6(tensorGemm_io_wgt_rd_0_data_bits_6_6),
    .io_wgt_rd_0_data_bits_6_7(tensorGemm_io_wgt_rd_0_data_bits_6_7),
    .io_wgt_rd_0_data_bits_6_8(tensorGemm_io_wgt_rd_0_data_bits_6_8),
    .io_wgt_rd_0_data_bits_6_9(tensorGemm_io_wgt_rd_0_data_bits_6_9),
    .io_wgt_rd_0_data_bits_6_10(tensorGemm_io_wgt_rd_0_data_bits_6_10),
    .io_wgt_rd_0_data_bits_6_11(tensorGemm_io_wgt_rd_0_data_bits_6_11),
    .io_wgt_rd_0_data_bits_6_12(tensorGemm_io_wgt_rd_0_data_bits_6_12),
    .io_wgt_rd_0_data_bits_6_13(tensorGemm_io_wgt_rd_0_data_bits_6_13),
    .io_wgt_rd_0_data_bits_6_14(tensorGemm_io_wgt_rd_0_data_bits_6_14),
    .io_wgt_rd_0_data_bits_6_15(tensorGemm_io_wgt_rd_0_data_bits_6_15),
    .io_wgt_rd_0_data_bits_7_0(tensorGemm_io_wgt_rd_0_data_bits_7_0),
    .io_wgt_rd_0_data_bits_7_1(tensorGemm_io_wgt_rd_0_data_bits_7_1),
    .io_wgt_rd_0_data_bits_7_2(tensorGemm_io_wgt_rd_0_data_bits_7_2),
    .io_wgt_rd_0_data_bits_7_3(tensorGemm_io_wgt_rd_0_data_bits_7_3),
    .io_wgt_rd_0_data_bits_7_4(tensorGemm_io_wgt_rd_0_data_bits_7_4),
    .io_wgt_rd_0_data_bits_7_5(tensorGemm_io_wgt_rd_0_data_bits_7_5),
    .io_wgt_rd_0_data_bits_7_6(tensorGemm_io_wgt_rd_0_data_bits_7_6),
    .io_wgt_rd_0_data_bits_7_7(tensorGemm_io_wgt_rd_0_data_bits_7_7),
    .io_wgt_rd_0_data_bits_7_8(tensorGemm_io_wgt_rd_0_data_bits_7_8),
    .io_wgt_rd_0_data_bits_7_9(tensorGemm_io_wgt_rd_0_data_bits_7_9),
    .io_wgt_rd_0_data_bits_7_10(tensorGemm_io_wgt_rd_0_data_bits_7_10),
    .io_wgt_rd_0_data_bits_7_11(tensorGemm_io_wgt_rd_0_data_bits_7_11),
    .io_wgt_rd_0_data_bits_7_12(tensorGemm_io_wgt_rd_0_data_bits_7_12),
    .io_wgt_rd_0_data_bits_7_13(tensorGemm_io_wgt_rd_0_data_bits_7_13),
    .io_wgt_rd_0_data_bits_7_14(tensorGemm_io_wgt_rd_0_data_bits_7_14),
    .io_wgt_rd_0_data_bits_7_15(tensorGemm_io_wgt_rd_0_data_bits_7_15),
    .io_wgt_rd_0_data_bits_8_0(tensorGemm_io_wgt_rd_0_data_bits_8_0),
    .io_wgt_rd_0_data_bits_8_1(tensorGemm_io_wgt_rd_0_data_bits_8_1),
    .io_wgt_rd_0_data_bits_8_2(tensorGemm_io_wgt_rd_0_data_bits_8_2),
    .io_wgt_rd_0_data_bits_8_3(tensorGemm_io_wgt_rd_0_data_bits_8_3),
    .io_wgt_rd_0_data_bits_8_4(tensorGemm_io_wgt_rd_0_data_bits_8_4),
    .io_wgt_rd_0_data_bits_8_5(tensorGemm_io_wgt_rd_0_data_bits_8_5),
    .io_wgt_rd_0_data_bits_8_6(tensorGemm_io_wgt_rd_0_data_bits_8_6),
    .io_wgt_rd_0_data_bits_8_7(tensorGemm_io_wgt_rd_0_data_bits_8_7),
    .io_wgt_rd_0_data_bits_8_8(tensorGemm_io_wgt_rd_0_data_bits_8_8),
    .io_wgt_rd_0_data_bits_8_9(tensorGemm_io_wgt_rd_0_data_bits_8_9),
    .io_wgt_rd_0_data_bits_8_10(tensorGemm_io_wgt_rd_0_data_bits_8_10),
    .io_wgt_rd_0_data_bits_8_11(tensorGemm_io_wgt_rd_0_data_bits_8_11),
    .io_wgt_rd_0_data_bits_8_12(tensorGemm_io_wgt_rd_0_data_bits_8_12),
    .io_wgt_rd_0_data_bits_8_13(tensorGemm_io_wgt_rd_0_data_bits_8_13),
    .io_wgt_rd_0_data_bits_8_14(tensorGemm_io_wgt_rd_0_data_bits_8_14),
    .io_wgt_rd_0_data_bits_8_15(tensorGemm_io_wgt_rd_0_data_bits_8_15),
    .io_wgt_rd_0_data_bits_9_0(tensorGemm_io_wgt_rd_0_data_bits_9_0),
    .io_wgt_rd_0_data_bits_9_1(tensorGemm_io_wgt_rd_0_data_bits_9_1),
    .io_wgt_rd_0_data_bits_9_2(tensorGemm_io_wgt_rd_0_data_bits_9_2),
    .io_wgt_rd_0_data_bits_9_3(tensorGemm_io_wgt_rd_0_data_bits_9_3),
    .io_wgt_rd_0_data_bits_9_4(tensorGemm_io_wgt_rd_0_data_bits_9_4),
    .io_wgt_rd_0_data_bits_9_5(tensorGemm_io_wgt_rd_0_data_bits_9_5),
    .io_wgt_rd_0_data_bits_9_6(tensorGemm_io_wgt_rd_0_data_bits_9_6),
    .io_wgt_rd_0_data_bits_9_7(tensorGemm_io_wgt_rd_0_data_bits_9_7),
    .io_wgt_rd_0_data_bits_9_8(tensorGemm_io_wgt_rd_0_data_bits_9_8),
    .io_wgt_rd_0_data_bits_9_9(tensorGemm_io_wgt_rd_0_data_bits_9_9),
    .io_wgt_rd_0_data_bits_9_10(tensorGemm_io_wgt_rd_0_data_bits_9_10),
    .io_wgt_rd_0_data_bits_9_11(tensorGemm_io_wgt_rd_0_data_bits_9_11),
    .io_wgt_rd_0_data_bits_9_12(tensorGemm_io_wgt_rd_0_data_bits_9_12),
    .io_wgt_rd_0_data_bits_9_13(tensorGemm_io_wgt_rd_0_data_bits_9_13),
    .io_wgt_rd_0_data_bits_9_14(tensorGemm_io_wgt_rd_0_data_bits_9_14),
    .io_wgt_rd_0_data_bits_9_15(tensorGemm_io_wgt_rd_0_data_bits_9_15),
    .io_wgt_rd_0_data_bits_10_0(tensorGemm_io_wgt_rd_0_data_bits_10_0),
    .io_wgt_rd_0_data_bits_10_1(tensorGemm_io_wgt_rd_0_data_bits_10_1),
    .io_wgt_rd_0_data_bits_10_2(tensorGemm_io_wgt_rd_0_data_bits_10_2),
    .io_wgt_rd_0_data_bits_10_3(tensorGemm_io_wgt_rd_0_data_bits_10_3),
    .io_wgt_rd_0_data_bits_10_4(tensorGemm_io_wgt_rd_0_data_bits_10_4),
    .io_wgt_rd_0_data_bits_10_5(tensorGemm_io_wgt_rd_0_data_bits_10_5),
    .io_wgt_rd_0_data_bits_10_6(tensorGemm_io_wgt_rd_0_data_bits_10_6),
    .io_wgt_rd_0_data_bits_10_7(tensorGemm_io_wgt_rd_0_data_bits_10_7),
    .io_wgt_rd_0_data_bits_10_8(tensorGemm_io_wgt_rd_0_data_bits_10_8),
    .io_wgt_rd_0_data_bits_10_9(tensorGemm_io_wgt_rd_0_data_bits_10_9),
    .io_wgt_rd_0_data_bits_10_10(tensorGemm_io_wgt_rd_0_data_bits_10_10),
    .io_wgt_rd_0_data_bits_10_11(tensorGemm_io_wgt_rd_0_data_bits_10_11),
    .io_wgt_rd_0_data_bits_10_12(tensorGemm_io_wgt_rd_0_data_bits_10_12),
    .io_wgt_rd_0_data_bits_10_13(tensorGemm_io_wgt_rd_0_data_bits_10_13),
    .io_wgt_rd_0_data_bits_10_14(tensorGemm_io_wgt_rd_0_data_bits_10_14),
    .io_wgt_rd_0_data_bits_10_15(tensorGemm_io_wgt_rd_0_data_bits_10_15),
    .io_wgt_rd_0_data_bits_11_0(tensorGemm_io_wgt_rd_0_data_bits_11_0),
    .io_wgt_rd_0_data_bits_11_1(tensorGemm_io_wgt_rd_0_data_bits_11_1),
    .io_wgt_rd_0_data_bits_11_2(tensorGemm_io_wgt_rd_0_data_bits_11_2),
    .io_wgt_rd_0_data_bits_11_3(tensorGemm_io_wgt_rd_0_data_bits_11_3),
    .io_wgt_rd_0_data_bits_11_4(tensorGemm_io_wgt_rd_0_data_bits_11_4),
    .io_wgt_rd_0_data_bits_11_5(tensorGemm_io_wgt_rd_0_data_bits_11_5),
    .io_wgt_rd_0_data_bits_11_6(tensorGemm_io_wgt_rd_0_data_bits_11_6),
    .io_wgt_rd_0_data_bits_11_7(tensorGemm_io_wgt_rd_0_data_bits_11_7),
    .io_wgt_rd_0_data_bits_11_8(tensorGemm_io_wgt_rd_0_data_bits_11_8),
    .io_wgt_rd_0_data_bits_11_9(tensorGemm_io_wgt_rd_0_data_bits_11_9),
    .io_wgt_rd_0_data_bits_11_10(tensorGemm_io_wgt_rd_0_data_bits_11_10),
    .io_wgt_rd_0_data_bits_11_11(tensorGemm_io_wgt_rd_0_data_bits_11_11),
    .io_wgt_rd_0_data_bits_11_12(tensorGemm_io_wgt_rd_0_data_bits_11_12),
    .io_wgt_rd_0_data_bits_11_13(tensorGemm_io_wgt_rd_0_data_bits_11_13),
    .io_wgt_rd_0_data_bits_11_14(tensorGemm_io_wgt_rd_0_data_bits_11_14),
    .io_wgt_rd_0_data_bits_11_15(tensorGemm_io_wgt_rd_0_data_bits_11_15),
    .io_wgt_rd_0_data_bits_12_0(tensorGemm_io_wgt_rd_0_data_bits_12_0),
    .io_wgt_rd_0_data_bits_12_1(tensorGemm_io_wgt_rd_0_data_bits_12_1),
    .io_wgt_rd_0_data_bits_12_2(tensorGemm_io_wgt_rd_0_data_bits_12_2),
    .io_wgt_rd_0_data_bits_12_3(tensorGemm_io_wgt_rd_0_data_bits_12_3),
    .io_wgt_rd_0_data_bits_12_4(tensorGemm_io_wgt_rd_0_data_bits_12_4),
    .io_wgt_rd_0_data_bits_12_5(tensorGemm_io_wgt_rd_0_data_bits_12_5),
    .io_wgt_rd_0_data_bits_12_6(tensorGemm_io_wgt_rd_0_data_bits_12_6),
    .io_wgt_rd_0_data_bits_12_7(tensorGemm_io_wgt_rd_0_data_bits_12_7),
    .io_wgt_rd_0_data_bits_12_8(tensorGemm_io_wgt_rd_0_data_bits_12_8),
    .io_wgt_rd_0_data_bits_12_9(tensorGemm_io_wgt_rd_0_data_bits_12_9),
    .io_wgt_rd_0_data_bits_12_10(tensorGemm_io_wgt_rd_0_data_bits_12_10),
    .io_wgt_rd_0_data_bits_12_11(tensorGemm_io_wgt_rd_0_data_bits_12_11),
    .io_wgt_rd_0_data_bits_12_12(tensorGemm_io_wgt_rd_0_data_bits_12_12),
    .io_wgt_rd_0_data_bits_12_13(tensorGemm_io_wgt_rd_0_data_bits_12_13),
    .io_wgt_rd_0_data_bits_12_14(tensorGemm_io_wgt_rd_0_data_bits_12_14),
    .io_wgt_rd_0_data_bits_12_15(tensorGemm_io_wgt_rd_0_data_bits_12_15),
    .io_wgt_rd_0_data_bits_13_0(tensorGemm_io_wgt_rd_0_data_bits_13_0),
    .io_wgt_rd_0_data_bits_13_1(tensorGemm_io_wgt_rd_0_data_bits_13_1),
    .io_wgt_rd_0_data_bits_13_2(tensorGemm_io_wgt_rd_0_data_bits_13_2),
    .io_wgt_rd_0_data_bits_13_3(tensorGemm_io_wgt_rd_0_data_bits_13_3),
    .io_wgt_rd_0_data_bits_13_4(tensorGemm_io_wgt_rd_0_data_bits_13_4),
    .io_wgt_rd_0_data_bits_13_5(tensorGemm_io_wgt_rd_0_data_bits_13_5),
    .io_wgt_rd_0_data_bits_13_6(tensorGemm_io_wgt_rd_0_data_bits_13_6),
    .io_wgt_rd_0_data_bits_13_7(tensorGemm_io_wgt_rd_0_data_bits_13_7),
    .io_wgt_rd_0_data_bits_13_8(tensorGemm_io_wgt_rd_0_data_bits_13_8),
    .io_wgt_rd_0_data_bits_13_9(tensorGemm_io_wgt_rd_0_data_bits_13_9),
    .io_wgt_rd_0_data_bits_13_10(tensorGemm_io_wgt_rd_0_data_bits_13_10),
    .io_wgt_rd_0_data_bits_13_11(tensorGemm_io_wgt_rd_0_data_bits_13_11),
    .io_wgt_rd_0_data_bits_13_12(tensorGemm_io_wgt_rd_0_data_bits_13_12),
    .io_wgt_rd_0_data_bits_13_13(tensorGemm_io_wgt_rd_0_data_bits_13_13),
    .io_wgt_rd_0_data_bits_13_14(tensorGemm_io_wgt_rd_0_data_bits_13_14),
    .io_wgt_rd_0_data_bits_13_15(tensorGemm_io_wgt_rd_0_data_bits_13_15),
    .io_wgt_rd_0_data_bits_14_0(tensorGemm_io_wgt_rd_0_data_bits_14_0),
    .io_wgt_rd_0_data_bits_14_1(tensorGemm_io_wgt_rd_0_data_bits_14_1),
    .io_wgt_rd_0_data_bits_14_2(tensorGemm_io_wgt_rd_0_data_bits_14_2),
    .io_wgt_rd_0_data_bits_14_3(tensorGemm_io_wgt_rd_0_data_bits_14_3),
    .io_wgt_rd_0_data_bits_14_4(tensorGemm_io_wgt_rd_0_data_bits_14_4),
    .io_wgt_rd_0_data_bits_14_5(tensorGemm_io_wgt_rd_0_data_bits_14_5),
    .io_wgt_rd_0_data_bits_14_6(tensorGemm_io_wgt_rd_0_data_bits_14_6),
    .io_wgt_rd_0_data_bits_14_7(tensorGemm_io_wgt_rd_0_data_bits_14_7),
    .io_wgt_rd_0_data_bits_14_8(tensorGemm_io_wgt_rd_0_data_bits_14_8),
    .io_wgt_rd_0_data_bits_14_9(tensorGemm_io_wgt_rd_0_data_bits_14_9),
    .io_wgt_rd_0_data_bits_14_10(tensorGemm_io_wgt_rd_0_data_bits_14_10),
    .io_wgt_rd_0_data_bits_14_11(tensorGemm_io_wgt_rd_0_data_bits_14_11),
    .io_wgt_rd_0_data_bits_14_12(tensorGemm_io_wgt_rd_0_data_bits_14_12),
    .io_wgt_rd_0_data_bits_14_13(tensorGemm_io_wgt_rd_0_data_bits_14_13),
    .io_wgt_rd_0_data_bits_14_14(tensorGemm_io_wgt_rd_0_data_bits_14_14),
    .io_wgt_rd_0_data_bits_14_15(tensorGemm_io_wgt_rd_0_data_bits_14_15),
    .io_wgt_rd_0_data_bits_15_0(tensorGemm_io_wgt_rd_0_data_bits_15_0),
    .io_wgt_rd_0_data_bits_15_1(tensorGemm_io_wgt_rd_0_data_bits_15_1),
    .io_wgt_rd_0_data_bits_15_2(tensorGemm_io_wgt_rd_0_data_bits_15_2),
    .io_wgt_rd_0_data_bits_15_3(tensorGemm_io_wgt_rd_0_data_bits_15_3),
    .io_wgt_rd_0_data_bits_15_4(tensorGemm_io_wgt_rd_0_data_bits_15_4),
    .io_wgt_rd_0_data_bits_15_5(tensorGemm_io_wgt_rd_0_data_bits_15_5),
    .io_wgt_rd_0_data_bits_15_6(tensorGemm_io_wgt_rd_0_data_bits_15_6),
    .io_wgt_rd_0_data_bits_15_7(tensorGemm_io_wgt_rd_0_data_bits_15_7),
    .io_wgt_rd_0_data_bits_15_8(tensorGemm_io_wgt_rd_0_data_bits_15_8),
    .io_wgt_rd_0_data_bits_15_9(tensorGemm_io_wgt_rd_0_data_bits_15_9),
    .io_wgt_rd_0_data_bits_15_10(tensorGemm_io_wgt_rd_0_data_bits_15_10),
    .io_wgt_rd_0_data_bits_15_11(tensorGemm_io_wgt_rd_0_data_bits_15_11),
    .io_wgt_rd_0_data_bits_15_12(tensorGemm_io_wgt_rd_0_data_bits_15_12),
    .io_wgt_rd_0_data_bits_15_13(tensorGemm_io_wgt_rd_0_data_bits_15_13),
    .io_wgt_rd_0_data_bits_15_14(tensorGemm_io_wgt_rd_0_data_bits_15_14),
    .io_wgt_rd_0_data_bits_15_15(tensorGemm_io_wgt_rd_0_data_bits_15_15),
    .io_acc_rd_0_idx_valid(tensorGemm_io_acc_rd_0_idx_valid),
    .io_acc_rd_0_idx_bits(tensorGemm_io_acc_rd_0_idx_bits),
    .io_acc_rd_0_data_valid(tensorGemm_io_acc_rd_0_data_valid),
    .io_acc_rd_0_data_bits_0_0(tensorGemm_io_acc_rd_0_data_bits_0_0),
    .io_acc_rd_0_data_bits_0_1(tensorGemm_io_acc_rd_0_data_bits_0_1),
    .io_acc_rd_0_data_bits_0_2(tensorGemm_io_acc_rd_0_data_bits_0_2),
    .io_acc_rd_0_data_bits_0_3(tensorGemm_io_acc_rd_0_data_bits_0_3),
    .io_acc_rd_0_data_bits_0_4(tensorGemm_io_acc_rd_0_data_bits_0_4),
    .io_acc_rd_0_data_bits_0_5(tensorGemm_io_acc_rd_0_data_bits_0_5),
    .io_acc_rd_0_data_bits_0_6(tensorGemm_io_acc_rd_0_data_bits_0_6),
    .io_acc_rd_0_data_bits_0_7(tensorGemm_io_acc_rd_0_data_bits_0_7),
    .io_acc_rd_0_data_bits_0_8(tensorGemm_io_acc_rd_0_data_bits_0_8),
    .io_acc_rd_0_data_bits_0_9(tensorGemm_io_acc_rd_0_data_bits_0_9),
    .io_acc_rd_0_data_bits_0_10(tensorGemm_io_acc_rd_0_data_bits_0_10),
    .io_acc_rd_0_data_bits_0_11(tensorGemm_io_acc_rd_0_data_bits_0_11),
    .io_acc_rd_0_data_bits_0_12(tensorGemm_io_acc_rd_0_data_bits_0_12),
    .io_acc_rd_0_data_bits_0_13(tensorGemm_io_acc_rd_0_data_bits_0_13),
    .io_acc_rd_0_data_bits_0_14(tensorGemm_io_acc_rd_0_data_bits_0_14),
    .io_acc_rd_0_data_bits_0_15(tensorGemm_io_acc_rd_0_data_bits_0_15),
    .io_acc_wr_0_valid(tensorGemm_io_acc_wr_0_valid),
    .io_acc_wr_0_bits_idx(tensorGemm_io_acc_wr_0_bits_idx),
    .io_acc_wr_0_bits_data_0_0(tensorGemm_io_acc_wr_0_bits_data_0_0),
    .io_acc_wr_0_bits_data_0_1(tensorGemm_io_acc_wr_0_bits_data_0_1),
    .io_acc_wr_0_bits_data_0_2(tensorGemm_io_acc_wr_0_bits_data_0_2),
    .io_acc_wr_0_bits_data_0_3(tensorGemm_io_acc_wr_0_bits_data_0_3),
    .io_acc_wr_0_bits_data_0_4(tensorGemm_io_acc_wr_0_bits_data_0_4),
    .io_acc_wr_0_bits_data_0_5(tensorGemm_io_acc_wr_0_bits_data_0_5),
    .io_acc_wr_0_bits_data_0_6(tensorGemm_io_acc_wr_0_bits_data_0_6),
    .io_acc_wr_0_bits_data_0_7(tensorGemm_io_acc_wr_0_bits_data_0_7),
    .io_acc_wr_0_bits_data_0_8(tensorGemm_io_acc_wr_0_bits_data_0_8),
    .io_acc_wr_0_bits_data_0_9(tensorGemm_io_acc_wr_0_bits_data_0_9),
    .io_acc_wr_0_bits_data_0_10(tensorGemm_io_acc_wr_0_bits_data_0_10),
    .io_acc_wr_0_bits_data_0_11(tensorGemm_io_acc_wr_0_bits_data_0_11),
    .io_acc_wr_0_bits_data_0_12(tensorGemm_io_acc_wr_0_bits_data_0_12),
    .io_acc_wr_0_bits_data_0_13(tensorGemm_io_acc_wr_0_bits_data_0_13),
    .io_acc_wr_0_bits_data_0_14(tensorGemm_io_acc_wr_0_bits_data_0_14),
    .io_acc_wr_0_bits_data_0_15(tensorGemm_io_acc_wr_0_bits_data_0_15),
    .io_out_rd_0_data_valid(tensorGemm_io_out_rd_0_data_valid),
    .io_out_wr_0_valid(tensorGemm_io_out_wr_0_valid),
    .io_out_wr_0_bits_idx(tensorGemm_io_out_wr_0_bits_idx),
    .io_out_wr_0_bits_data_0_0(tensorGemm_io_out_wr_0_bits_data_0_0),
    .io_out_wr_0_bits_data_0_1(tensorGemm_io_out_wr_0_bits_data_0_1),
    .io_out_wr_0_bits_data_0_2(tensorGemm_io_out_wr_0_bits_data_0_2),
    .io_out_wr_0_bits_data_0_3(tensorGemm_io_out_wr_0_bits_data_0_3),
    .io_out_wr_0_bits_data_0_4(tensorGemm_io_out_wr_0_bits_data_0_4),
    .io_out_wr_0_bits_data_0_5(tensorGemm_io_out_wr_0_bits_data_0_5),
    .io_out_wr_0_bits_data_0_6(tensorGemm_io_out_wr_0_bits_data_0_6),
    .io_out_wr_0_bits_data_0_7(tensorGemm_io_out_wr_0_bits_data_0_7),
    .io_out_wr_0_bits_data_0_8(tensorGemm_io_out_wr_0_bits_data_0_8),
    .io_out_wr_0_bits_data_0_9(tensorGemm_io_out_wr_0_bits_data_0_9),
    .io_out_wr_0_bits_data_0_10(tensorGemm_io_out_wr_0_bits_data_0_10),
    .io_out_wr_0_bits_data_0_11(tensorGemm_io_out_wr_0_bits_data_0_11),
    .io_out_wr_0_bits_data_0_12(tensorGemm_io_out_wr_0_bits_data_0_12),
    .io_out_wr_0_bits_data_0_13(tensorGemm_io_out_wr_0_bits_data_0_13),
    .io_out_wr_0_bits_data_0_14(tensorGemm_io_out_wr_0_bits_data_0_14),
    .io_out_wr_0_bits_data_0_15(tensorGemm_io_out_wr_0_bits_data_0_15)
  );
  TensorAlu tensorAlu ( // @[Compute.scala 64:25]
    .clock(tensorAlu_clock),
    .reset(tensorAlu_reset),
    .io_start(tensorAlu_io_start),
    .io_done(tensorAlu_io_done),
    .io_dec_alu_imm(tensorAlu_io_dec_alu_imm),
    .io_dec_alu_use_imm(tensorAlu_io_dec_alu_use_imm),
    .io_dec_alu_op(tensorAlu_io_dec_alu_op),
    .io_dec_src_1(tensorAlu_io_dec_src_1),
    .io_dec_src_0(tensorAlu_io_dec_src_0),
    .io_dec_dst_1(tensorAlu_io_dec_dst_1),
    .io_dec_dst_0(tensorAlu_io_dec_dst_0),
    .io_dec_lp_1(tensorAlu_io_dec_lp_1),
    .io_dec_lp_0(tensorAlu_io_dec_lp_0),
    .io_dec_uop_end(tensorAlu_io_dec_uop_end),
    .io_dec_uop_begin(tensorAlu_io_dec_uop_begin),
    .io_uop_idx_valid(tensorAlu_io_uop_idx_valid),
    .io_uop_idx_bits(tensorAlu_io_uop_idx_bits),
    .io_uop_data_bits_u2(tensorAlu_io_uop_data_bits_u2),
    .io_uop_data_bits_u1(tensorAlu_io_uop_data_bits_u1),
    .io_uop_data_bits_u0(tensorAlu_io_uop_data_bits_u0),
    .io_acc_rd_0_idx_valid(tensorAlu_io_acc_rd_0_idx_valid),
    .io_acc_rd_0_idx_bits(tensorAlu_io_acc_rd_0_idx_bits),
    .io_acc_rd_0_data_valid(tensorAlu_io_acc_rd_0_data_valid),
    .io_acc_rd_0_data_bits_0_0(tensorAlu_io_acc_rd_0_data_bits_0_0),
    .io_acc_rd_0_data_bits_0_1(tensorAlu_io_acc_rd_0_data_bits_0_1),
    .io_acc_rd_0_data_bits_0_2(tensorAlu_io_acc_rd_0_data_bits_0_2),
    .io_acc_rd_0_data_bits_0_3(tensorAlu_io_acc_rd_0_data_bits_0_3),
    .io_acc_rd_0_data_bits_0_4(tensorAlu_io_acc_rd_0_data_bits_0_4),
    .io_acc_rd_0_data_bits_0_5(tensorAlu_io_acc_rd_0_data_bits_0_5),
    .io_acc_rd_0_data_bits_0_6(tensorAlu_io_acc_rd_0_data_bits_0_6),
    .io_acc_rd_0_data_bits_0_7(tensorAlu_io_acc_rd_0_data_bits_0_7),
    .io_acc_rd_0_data_bits_0_8(tensorAlu_io_acc_rd_0_data_bits_0_8),
    .io_acc_rd_0_data_bits_0_9(tensorAlu_io_acc_rd_0_data_bits_0_9),
    .io_acc_rd_0_data_bits_0_10(tensorAlu_io_acc_rd_0_data_bits_0_10),
    .io_acc_rd_0_data_bits_0_11(tensorAlu_io_acc_rd_0_data_bits_0_11),
    .io_acc_rd_0_data_bits_0_12(tensorAlu_io_acc_rd_0_data_bits_0_12),
    .io_acc_rd_0_data_bits_0_13(tensorAlu_io_acc_rd_0_data_bits_0_13),
    .io_acc_rd_0_data_bits_0_14(tensorAlu_io_acc_rd_0_data_bits_0_14),
    .io_acc_rd_0_data_bits_0_15(tensorAlu_io_acc_rd_0_data_bits_0_15),
    .io_acc_wr_0_valid(tensorAlu_io_acc_wr_0_valid),
    .io_acc_wr_0_bits_idx(tensorAlu_io_acc_wr_0_bits_idx),
    .io_acc_wr_0_bits_data_0_0(tensorAlu_io_acc_wr_0_bits_data_0_0),
    .io_acc_wr_0_bits_data_0_1(tensorAlu_io_acc_wr_0_bits_data_0_1),
    .io_acc_wr_0_bits_data_0_2(tensorAlu_io_acc_wr_0_bits_data_0_2),
    .io_acc_wr_0_bits_data_0_3(tensorAlu_io_acc_wr_0_bits_data_0_3),
    .io_acc_wr_0_bits_data_0_4(tensorAlu_io_acc_wr_0_bits_data_0_4),
    .io_acc_wr_0_bits_data_0_5(tensorAlu_io_acc_wr_0_bits_data_0_5),
    .io_acc_wr_0_bits_data_0_6(tensorAlu_io_acc_wr_0_bits_data_0_6),
    .io_acc_wr_0_bits_data_0_7(tensorAlu_io_acc_wr_0_bits_data_0_7),
    .io_acc_wr_0_bits_data_0_8(tensorAlu_io_acc_wr_0_bits_data_0_8),
    .io_acc_wr_0_bits_data_0_9(tensorAlu_io_acc_wr_0_bits_data_0_9),
    .io_acc_wr_0_bits_data_0_10(tensorAlu_io_acc_wr_0_bits_data_0_10),
    .io_acc_wr_0_bits_data_0_11(tensorAlu_io_acc_wr_0_bits_data_0_11),
    .io_acc_wr_0_bits_data_0_12(tensorAlu_io_acc_wr_0_bits_data_0_12),
    .io_acc_wr_0_bits_data_0_13(tensorAlu_io_acc_wr_0_bits_data_0_13),
    .io_acc_wr_0_bits_data_0_14(tensorAlu_io_acc_wr_0_bits_data_0_14),
    .io_acc_wr_0_bits_data_0_15(tensorAlu_io_acc_wr_0_bits_data_0_15),
    .io_out_rd_0_data_valid(tensorAlu_io_out_rd_0_data_valid),
    .io_out_wr_0_valid(tensorAlu_io_out_wr_0_valid),
    .io_out_wr_0_bits_idx(tensorAlu_io_out_wr_0_bits_idx),
    .io_out_wr_0_bits_data_0_0(tensorAlu_io_out_wr_0_bits_data_0_0),
    .io_out_wr_0_bits_data_0_1(tensorAlu_io_out_wr_0_bits_data_0_1),
    .io_out_wr_0_bits_data_0_2(tensorAlu_io_out_wr_0_bits_data_0_2),
    .io_out_wr_0_bits_data_0_3(tensorAlu_io_out_wr_0_bits_data_0_3),
    .io_out_wr_0_bits_data_0_4(tensorAlu_io_out_wr_0_bits_data_0_4),
    .io_out_wr_0_bits_data_0_5(tensorAlu_io_out_wr_0_bits_data_0_5),
    .io_out_wr_0_bits_data_0_6(tensorAlu_io_out_wr_0_bits_data_0_6),
    .io_out_wr_0_bits_data_0_7(tensorAlu_io_out_wr_0_bits_data_0_7),
    .io_out_wr_0_bits_data_0_8(tensorAlu_io_out_wr_0_bits_data_0_8),
    .io_out_wr_0_bits_data_0_9(tensorAlu_io_out_wr_0_bits_data_0_9),
    .io_out_wr_0_bits_data_0_10(tensorAlu_io_out_wr_0_bits_data_0_10),
    .io_out_wr_0_bits_data_0_11(tensorAlu_io_out_wr_0_bits_data_0_11),
    .io_out_wr_0_bits_data_0_12(tensorAlu_io_out_wr_0_bits_data_0_12),
    .io_out_wr_0_bits_data_0_13(tensorAlu_io_out_wr_0_bits_data_0_13),
    .io_out_wr_0_bits_data_0_14(tensorAlu_io_out_wr_0_bits_data_0_14),
    .io_out_wr_0_bits_data_0_15(tensorAlu_io_out_wr_0_bits_data_0_15)
  );
  SyncQueue_1 inst_q ( // @[Compute.scala 69:22]
    .clock(inst_q_clock),
    .reset(inst_q_reset),
    .io_enq_ready(inst_q_io_enq_ready),
    .io_enq_valid(inst_q_io_enq_valid),
    .io_enq_bits(inst_q_io_enq_bits),
    .io_deq_ready(inst_q_io_deq_ready),
    .io_deq_valid(inst_q_io_deq_valid),
    .io_deq_bits(inst_q_io_deq_bits)
  );
  ComputeDecode dec ( // @[Compute.scala 72:19]
    .io_inst(dec_io_inst),
    .io_push_next(dec_io_push_next),
    .io_push_prev(dec_io_push_prev),
    .io_pop_next(dec_io_pop_next),
    .io_pop_prev(dec_io_pop_prev),
    .io_isLoadAcc(dec_io_isLoadAcc),
    .io_isLoadUop(dec_io_isLoadUop),
    .io_isSync(dec_io_isSync),
    .io_isAlu(dec_io_isAlu),
    .io_isGemm(dec_io_isGemm),
    .io_isFinish(dec_io_isFinish)
  );
  assign io_o_post_0 = dec_io_push_prev & _inst_q_io_deq_ready_T_3; // @[Compute.scala 230:36]
  assign io_o_post_1 = dec_io_push_next & _inst_q_io_deq_ready_T_3; // @[Compute.scala 231:36]
  assign io_inst_ready = inst_q_io_enq_ready; // @[Compute.scala 120:17]
  assign io_vme_rd_0_cmd_valid = loadUop_io_vme_rd_cmd_valid; // @[Compute.scala 127:16]
  assign io_vme_rd_0_cmd_bits_addr = loadUop_io_vme_rd_cmd_bits_addr; // @[Compute.scala 127:16]
  assign io_vme_rd_0_cmd_bits_len = loadUop_io_vme_rd_cmd_bits_len; // @[Compute.scala 127:16]
  assign io_vme_rd_0_cmd_bits_tag = loadUop_io_vme_rd_cmd_bits_tag; // @[Compute.scala 127:16]
  assign io_vme_rd_1_cmd_valid = tensorAcc_io_vme_rd_cmd_valid; // @[Compute.scala 158:16]
  assign io_vme_rd_1_cmd_bits_addr = tensorAcc_io_vme_rd_cmd_bits_addr; // @[Compute.scala 158:16]
  assign io_vme_rd_1_cmd_bits_len = tensorAcc_io_vme_rd_cmd_bits_len; // @[Compute.scala 158:16]
  assign io_vme_rd_1_cmd_bits_tag = tensorAcc_io_vme_rd_cmd_bits_tag; // @[Compute.scala 158:16]
  assign io_inp_rd_0_idx_valid = tensorGemm_io_inp_rd_0_idx_valid; // @[Compute.scala 166:21]
  assign io_inp_rd_0_idx_bits = tensorGemm_io_inp_rd_0_idx_bits; // @[Compute.scala 166:21]
  assign io_wgt_rd_0_idx_valid = tensorGemm_io_wgt_rd_0_idx_valid; // @[Compute.scala 167:21]
  assign io_wgt_rd_0_idx_bits = tensorGemm_io_wgt_rd_0_idx_bits; // @[Compute.scala 167:21]
  assign io_out_wr_0_valid = io_out_wr_0_valid_REG ? tensorGemm_io_out_wr_0_valid : tensorAlu_io_out_wr_0_valid; // @[Compute.scala 207:28]
  assign io_out_wr_0_bits_idx = io_out_wr_0_bits_idx_REG ? tensorGemm_io_out_wr_0_bits_idx :
    tensorAlu_io_out_wr_0_bits_idx; // @[Compute.scala 209:31]
  assign io_out_wr_0_bits_data_0_0 = outDataBits_0[31:0]; // @[Compute.scala 214:49]
  assign io_out_wr_0_bits_data_0_1 = outDataBits_0[63:32]; // @[Compute.scala 214:49]
  assign io_out_wr_0_bits_data_0_2 = outDataBits_0[95:64]; // @[Compute.scala 214:49]
  assign io_out_wr_0_bits_data_0_3 = outDataBits_0[127:96]; // @[Compute.scala 214:49]
  assign io_out_wr_0_bits_data_0_4 = outDataBits_0[159:128]; // @[Compute.scala 214:49]
  assign io_out_wr_0_bits_data_0_5 = outDataBits_0[191:160]; // @[Compute.scala 214:49]
  assign io_out_wr_0_bits_data_0_6 = outDataBits_0[223:192]; // @[Compute.scala 214:49]
  assign io_out_wr_0_bits_data_0_7 = outDataBits_0[255:224]; // @[Compute.scala 214:49]
  assign io_out_wr_0_bits_data_0_8 = outDataBits_0[287:256]; // @[Compute.scala 214:49]
  assign io_out_wr_0_bits_data_0_9 = outDataBits_0[319:288]; // @[Compute.scala 214:49]
  assign io_out_wr_0_bits_data_0_10 = outDataBits_0[351:320]; // @[Compute.scala 214:49]
  assign io_out_wr_0_bits_data_0_11 = outDataBits_0[383:352]; // @[Compute.scala 214:49]
  assign io_out_wr_0_bits_data_0_12 = outDataBits_0[415:384]; // @[Compute.scala 214:49]
  assign io_out_wr_0_bits_data_0_13 = outDataBits_0[447:416]; // @[Compute.scala 214:49]
  assign io_out_wr_0_bits_data_0_14 = outDataBits_0[479:448]; // @[Compute.scala 214:49]
  assign io_out_wr_0_bits_data_0_15 = outDataBits_0[511:480]; // @[Compute.scala 214:49]
  assign io_finish = _inst_q_io_deq_ready_T_1 & dec_io_isFinish; // @[Compute.scala 234:38]
  assign io_acc_wr_event = tensorAcc_io_tensor_wr_0_valid; // @[Compute.scala 159:19]
  assign s_0_clock = clock;
  assign s_0_reset = reset;
  assign s_0_io_spost = io_i_post_0; // @[Compute.scala 226:17]
  assign s_0_io_swait = dec_io_pop_prev & _loadUop_io_start_T_1; // @[Compute.scala 228:36]
  assign s_1_clock = clock;
  assign s_1_reset = reset;
  assign s_1_io_spost = io_i_post_1; // @[Compute.scala 227:17]
  assign s_1_io_swait = dec_io_pop_next & _loadUop_io_start_T_1; // @[Compute.scala 229:36]
  assign loadUop_clock = clock;
  assign loadUop_reset = reset;
  assign loadUop_io_start = state == 2'h0 & start & dec_io_isLoadUop; // @[Compute.scala 124:47]
  assign loadUop_io_inst = inst_q_io_deq_bits; // @[Compute.scala 125:19]
  assign loadUop_io_baddr = io_uop_baddr; // @[Compute.scala 126:20]
  assign loadUop_io_vme_rd_cmd_ready = io_vme_rd_0_cmd_ready; // @[Compute.scala 127:16]
  assign loadUop_io_vme_rd_data_valid = io_vme_rd_0_data_valid; // @[Compute.scala 127:16]
  assign loadUop_io_vme_rd_data_bits_data = io_vme_rd_0_data_bits_data; // @[Compute.scala 127:16]
  assign loadUop_io_vme_rd_data_bits_tag = io_vme_rd_0_data_bits_tag; // @[Compute.scala 127:16]
  assign loadUop_io_vme_rd_data_bits_last = io_vme_rd_0_data_bits_last; // @[Compute.scala 127:16]
  assign loadUop_io_uop_idx_valid = dec_io_isGemm ? tensorGemm_io_uop_idx_valid : tensorAlu_io_uop_idx_valid; // @[Compute.scala 128:28]
  assign loadUop_io_uop_idx_bits = dec_io_isGemm ? tensorGemm_io_uop_idx_bits : tensorAlu_io_uop_idx_bits; // @[Compute.scala 128:28]
  assign tensorAcc_clock = clock;
  assign tensorAcc_reset = reset;
  assign tensorAcc_io_start = _loadUop_io_start_T_1 & dec_io_isLoadAcc; // @[Compute.scala 132:49]
  assign tensorAcc_io_inst = inst_q_io_deq_bits; // @[Compute.scala 133:21]
  assign tensorAcc_io_baddr = io_acc_baddr; // @[Compute.scala 134:22]
  assign tensorAcc_io_vme_rd_cmd_ready = io_vme_rd_1_cmd_ready; // @[Compute.scala 158:16]
  assign tensorAcc_io_vme_rd_data_valid = io_vme_rd_1_data_valid; // @[Compute.scala 158:16]
  assign tensorAcc_io_vme_rd_data_bits_data = io_vme_rd_1_data_bits_data; // @[Compute.scala 158:16]
  assign tensorAcc_io_vme_rd_data_bits_tag = io_vme_rd_1_data_bits_tag; // @[Compute.scala 158:16]
  assign tensorAcc_io_tensor_rd_0_idx_valid = tensorAcc_io_tensor_rd_0_idx_REG ? tensorGemm_io_acc_rd_0_idx_valid :
    tensorAlu_io_acc_rd_0_idx_valid; // @[Compute.scala 149:43]
  assign tensorAcc_io_tensor_rd_0_idx_bits = tensorAcc_io_tensor_rd_0_idx_REG ? tensorGemm_io_acc_rd_0_idx_bits :
    tensorAlu_io_acc_rd_0_idx_bits; // @[Compute.scala 149:43]
  assign tensorAcc_io_tensor_wr_0_valid = tensorAcc_io_tensor_wr_0_REG ? tensorGemm_io_acc_wr_0_valid :
    tensorAlu_io_acc_wr_0_valid; // @[Compute.scala 153:39]
  assign tensorAcc_io_tensor_wr_0_bits_idx = tensorAcc_io_tensor_wr_0_REG ? tensorGemm_io_acc_wr_0_bits_idx :
    tensorAlu_io_acc_wr_0_bits_idx; // @[Compute.scala 153:39]
  assign tensorAcc_io_tensor_wr_0_bits_data_0_0 = tensorAcc_io_tensor_wr_0_REG ? tensorGemm_io_acc_wr_0_bits_data_0_0 :
    tensorAlu_io_acc_wr_0_bits_data_0_0; // @[Compute.scala 153:39]
  assign tensorAcc_io_tensor_wr_0_bits_data_0_1 = tensorAcc_io_tensor_wr_0_REG ? tensorGemm_io_acc_wr_0_bits_data_0_1 :
    tensorAlu_io_acc_wr_0_bits_data_0_1; // @[Compute.scala 153:39]
  assign tensorAcc_io_tensor_wr_0_bits_data_0_2 = tensorAcc_io_tensor_wr_0_REG ? tensorGemm_io_acc_wr_0_bits_data_0_2 :
    tensorAlu_io_acc_wr_0_bits_data_0_2; // @[Compute.scala 153:39]
  assign tensorAcc_io_tensor_wr_0_bits_data_0_3 = tensorAcc_io_tensor_wr_0_REG ? tensorGemm_io_acc_wr_0_bits_data_0_3 :
    tensorAlu_io_acc_wr_0_bits_data_0_3; // @[Compute.scala 153:39]
  assign tensorAcc_io_tensor_wr_0_bits_data_0_4 = tensorAcc_io_tensor_wr_0_REG ? tensorGemm_io_acc_wr_0_bits_data_0_4 :
    tensorAlu_io_acc_wr_0_bits_data_0_4; // @[Compute.scala 153:39]
  assign tensorAcc_io_tensor_wr_0_bits_data_0_5 = tensorAcc_io_tensor_wr_0_REG ? tensorGemm_io_acc_wr_0_bits_data_0_5 :
    tensorAlu_io_acc_wr_0_bits_data_0_5; // @[Compute.scala 153:39]
  assign tensorAcc_io_tensor_wr_0_bits_data_0_6 = tensorAcc_io_tensor_wr_0_REG ? tensorGemm_io_acc_wr_0_bits_data_0_6 :
    tensorAlu_io_acc_wr_0_bits_data_0_6; // @[Compute.scala 153:39]
  assign tensorAcc_io_tensor_wr_0_bits_data_0_7 = tensorAcc_io_tensor_wr_0_REG ? tensorGemm_io_acc_wr_0_bits_data_0_7 :
    tensorAlu_io_acc_wr_0_bits_data_0_7; // @[Compute.scala 153:39]
  assign tensorAcc_io_tensor_wr_0_bits_data_0_8 = tensorAcc_io_tensor_wr_0_REG ? tensorGemm_io_acc_wr_0_bits_data_0_8 :
    tensorAlu_io_acc_wr_0_bits_data_0_8; // @[Compute.scala 153:39]
  assign tensorAcc_io_tensor_wr_0_bits_data_0_9 = tensorAcc_io_tensor_wr_0_REG ? tensorGemm_io_acc_wr_0_bits_data_0_9 :
    tensorAlu_io_acc_wr_0_bits_data_0_9; // @[Compute.scala 153:39]
  assign tensorAcc_io_tensor_wr_0_bits_data_0_10 = tensorAcc_io_tensor_wr_0_REG ? tensorGemm_io_acc_wr_0_bits_data_0_10
     : tensorAlu_io_acc_wr_0_bits_data_0_10; // @[Compute.scala 153:39]
  assign tensorAcc_io_tensor_wr_0_bits_data_0_11 = tensorAcc_io_tensor_wr_0_REG ? tensorGemm_io_acc_wr_0_bits_data_0_11
     : tensorAlu_io_acc_wr_0_bits_data_0_11; // @[Compute.scala 153:39]
  assign tensorAcc_io_tensor_wr_0_bits_data_0_12 = tensorAcc_io_tensor_wr_0_REG ? tensorGemm_io_acc_wr_0_bits_data_0_12
     : tensorAlu_io_acc_wr_0_bits_data_0_12; // @[Compute.scala 153:39]
  assign tensorAcc_io_tensor_wr_0_bits_data_0_13 = tensorAcc_io_tensor_wr_0_REG ? tensorGemm_io_acc_wr_0_bits_data_0_13
     : tensorAlu_io_acc_wr_0_bits_data_0_13; // @[Compute.scala 153:39]
  assign tensorAcc_io_tensor_wr_0_bits_data_0_14 = tensorAcc_io_tensor_wr_0_REG ? tensorGemm_io_acc_wr_0_bits_data_0_14
     : tensorAlu_io_acc_wr_0_bits_data_0_14; // @[Compute.scala 153:39]
  assign tensorAcc_io_tensor_wr_0_bits_data_0_15 = tensorAcc_io_tensor_wr_0_REG ? tensorGemm_io_acc_wr_0_bits_data_0_15
     : tensorAlu_io_acc_wr_0_bits_data_0_15; // @[Compute.scala 153:39]
  assign tensorGemm_clock = clock;
  assign tensorGemm_reset = reset;
  assign tensorGemm_io_start = tensorGemm_io_start_REG; // @[Compute.scala 162:23]
  assign tensorGemm_io_dec_wgt_1 = _tensorGemm_io_dec_WIRE_1[127:118]; // @[Compute.scala 163:51]
  assign tensorGemm_io_dec_wgt_0 = _tensorGemm_io_dec_WIRE_1[117:108]; // @[Compute.scala 163:51]
  assign tensorGemm_io_dec_inp_1 = _tensorGemm_io_dec_WIRE_1[107:97]; // @[Compute.scala 163:51]
  assign tensorGemm_io_dec_inp_0 = _tensorGemm_io_dec_WIRE_1[96:86]; // @[Compute.scala 163:51]
  assign tensorGemm_io_dec_acc_1 = _tensorGemm_io_dec_WIRE_1[85:75]; // @[Compute.scala 163:51]
  assign tensorGemm_io_dec_acc_0 = _tensorGemm_io_dec_WIRE_1[74:64]; // @[Compute.scala 163:51]
  assign tensorGemm_io_dec_empty_0 = _tensorGemm_io_dec_WIRE_1[63]; // @[Compute.scala 163:51]
  assign tensorGemm_io_dec_lp_1 = _tensorGemm_io_dec_WIRE_1[62:49]; // @[Compute.scala 163:51]
  assign tensorGemm_io_dec_lp_0 = _tensorGemm_io_dec_WIRE_1[48:35]; // @[Compute.scala 163:51]
  assign tensorGemm_io_dec_uop_end = _tensorGemm_io_dec_WIRE_1[34:21]; // @[Compute.scala 163:51]
  assign tensorGemm_io_dec_uop_begin = _tensorGemm_io_dec_WIRE_1[20:8]; // @[Compute.scala 163:51]
  assign tensorGemm_io_dec_reset = _tensorGemm_io_dec_WIRE_1[7]; // @[Compute.scala 163:51]
  assign tensorGemm_io_dec_push_next = _tensorGemm_io_dec_WIRE_1[6]; // @[Compute.scala 163:51]
  assign tensorGemm_io_dec_push_prev = _tensorGemm_io_dec_WIRE_1[5]; // @[Compute.scala 163:51]
  assign tensorGemm_io_dec_pop_next = _tensorGemm_io_dec_WIRE_1[4]; // @[Compute.scala 163:51]
  assign tensorGemm_io_dec_pop_prev = _tensorGemm_io_dec_WIRE_1[3]; // @[Compute.scala 163:51]
  assign tensorGemm_io_dec_op = _tensorGemm_io_dec_WIRE_1[2:0]; // @[Compute.scala 163:51]
  assign tensorGemm_io_uop_data_valid = loadUop_io_uop_data_valid & dec_io_isGemm; // @[Compute.scala 164:61]
  assign tensorGemm_io_uop_data_bits_u2 = loadUop_io_uop_data_bits_u2; // @[Compute.scala 165:31]
  assign tensorGemm_io_uop_data_bits_u1 = loadUop_io_uop_data_bits_u1; // @[Compute.scala 165:31]
  assign tensorGemm_io_uop_data_bits_u0 = loadUop_io_uop_data_bits_u0; // @[Compute.scala 165:31]
  assign tensorGemm_io_inp_rd_0_data_valid = io_inp_rd_0_data_valid; // @[Compute.scala 166:21]
  assign tensorGemm_io_inp_rd_0_data_bits_0_0 = io_inp_rd_0_data_bits_0_0; // @[Compute.scala 166:21]
  assign tensorGemm_io_inp_rd_0_data_bits_0_1 = io_inp_rd_0_data_bits_0_1; // @[Compute.scala 166:21]
  assign tensorGemm_io_inp_rd_0_data_bits_0_2 = io_inp_rd_0_data_bits_0_2; // @[Compute.scala 166:21]
  assign tensorGemm_io_inp_rd_0_data_bits_0_3 = io_inp_rd_0_data_bits_0_3; // @[Compute.scala 166:21]
  assign tensorGemm_io_inp_rd_0_data_bits_0_4 = io_inp_rd_0_data_bits_0_4; // @[Compute.scala 166:21]
  assign tensorGemm_io_inp_rd_0_data_bits_0_5 = io_inp_rd_0_data_bits_0_5; // @[Compute.scala 166:21]
  assign tensorGemm_io_inp_rd_0_data_bits_0_6 = io_inp_rd_0_data_bits_0_6; // @[Compute.scala 166:21]
  assign tensorGemm_io_inp_rd_0_data_bits_0_7 = io_inp_rd_0_data_bits_0_7; // @[Compute.scala 166:21]
  assign tensorGemm_io_inp_rd_0_data_bits_0_8 = io_inp_rd_0_data_bits_0_8; // @[Compute.scala 166:21]
  assign tensorGemm_io_inp_rd_0_data_bits_0_9 = io_inp_rd_0_data_bits_0_9; // @[Compute.scala 166:21]
  assign tensorGemm_io_inp_rd_0_data_bits_0_10 = io_inp_rd_0_data_bits_0_10; // @[Compute.scala 166:21]
  assign tensorGemm_io_inp_rd_0_data_bits_0_11 = io_inp_rd_0_data_bits_0_11; // @[Compute.scala 166:21]
  assign tensorGemm_io_inp_rd_0_data_bits_0_12 = io_inp_rd_0_data_bits_0_12; // @[Compute.scala 166:21]
  assign tensorGemm_io_inp_rd_0_data_bits_0_13 = io_inp_rd_0_data_bits_0_13; // @[Compute.scala 166:21]
  assign tensorGemm_io_inp_rd_0_data_bits_0_14 = io_inp_rd_0_data_bits_0_14; // @[Compute.scala 166:21]
  assign tensorGemm_io_inp_rd_0_data_bits_0_15 = io_inp_rd_0_data_bits_0_15; // @[Compute.scala 166:21]
  assign tensorGemm_io_wgt_rd_0_data_valid = io_wgt_rd_0_data_valid; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_0_0 = io_wgt_rd_0_data_bits_0_0; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_0_1 = io_wgt_rd_0_data_bits_0_1; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_0_2 = io_wgt_rd_0_data_bits_0_2; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_0_3 = io_wgt_rd_0_data_bits_0_3; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_0_4 = io_wgt_rd_0_data_bits_0_4; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_0_5 = io_wgt_rd_0_data_bits_0_5; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_0_6 = io_wgt_rd_0_data_bits_0_6; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_0_7 = io_wgt_rd_0_data_bits_0_7; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_0_8 = io_wgt_rd_0_data_bits_0_8; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_0_9 = io_wgt_rd_0_data_bits_0_9; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_0_10 = io_wgt_rd_0_data_bits_0_10; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_0_11 = io_wgt_rd_0_data_bits_0_11; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_0_12 = io_wgt_rd_0_data_bits_0_12; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_0_13 = io_wgt_rd_0_data_bits_0_13; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_0_14 = io_wgt_rd_0_data_bits_0_14; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_0_15 = io_wgt_rd_0_data_bits_0_15; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_1_0 = io_wgt_rd_0_data_bits_1_0; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_1_1 = io_wgt_rd_0_data_bits_1_1; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_1_2 = io_wgt_rd_0_data_bits_1_2; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_1_3 = io_wgt_rd_0_data_bits_1_3; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_1_4 = io_wgt_rd_0_data_bits_1_4; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_1_5 = io_wgt_rd_0_data_bits_1_5; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_1_6 = io_wgt_rd_0_data_bits_1_6; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_1_7 = io_wgt_rd_0_data_bits_1_7; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_1_8 = io_wgt_rd_0_data_bits_1_8; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_1_9 = io_wgt_rd_0_data_bits_1_9; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_1_10 = io_wgt_rd_0_data_bits_1_10; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_1_11 = io_wgt_rd_0_data_bits_1_11; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_1_12 = io_wgt_rd_0_data_bits_1_12; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_1_13 = io_wgt_rd_0_data_bits_1_13; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_1_14 = io_wgt_rd_0_data_bits_1_14; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_1_15 = io_wgt_rd_0_data_bits_1_15; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_2_0 = io_wgt_rd_0_data_bits_2_0; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_2_1 = io_wgt_rd_0_data_bits_2_1; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_2_2 = io_wgt_rd_0_data_bits_2_2; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_2_3 = io_wgt_rd_0_data_bits_2_3; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_2_4 = io_wgt_rd_0_data_bits_2_4; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_2_5 = io_wgt_rd_0_data_bits_2_5; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_2_6 = io_wgt_rd_0_data_bits_2_6; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_2_7 = io_wgt_rd_0_data_bits_2_7; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_2_8 = io_wgt_rd_0_data_bits_2_8; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_2_9 = io_wgt_rd_0_data_bits_2_9; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_2_10 = io_wgt_rd_0_data_bits_2_10; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_2_11 = io_wgt_rd_0_data_bits_2_11; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_2_12 = io_wgt_rd_0_data_bits_2_12; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_2_13 = io_wgt_rd_0_data_bits_2_13; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_2_14 = io_wgt_rd_0_data_bits_2_14; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_2_15 = io_wgt_rd_0_data_bits_2_15; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_3_0 = io_wgt_rd_0_data_bits_3_0; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_3_1 = io_wgt_rd_0_data_bits_3_1; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_3_2 = io_wgt_rd_0_data_bits_3_2; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_3_3 = io_wgt_rd_0_data_bits_3_3; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_3_4 = io_wgt_rd_0_data_bits_3_4; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_3_5 = io_wgt_rd_0_data_bits_3_5; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_3_6 = io_wgt_rd_0_data_bits_3_6; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_3_7 = io_wgt_rd_0_data_bits_3_7; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_3_8 = io_wgt_rd_0_data_bits_3_8; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_3_9 = io_wgt_rd_0_data_bits_3_9; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_3_10 = io_wgt_rd_0_data_bits_3_10; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_3_11 = io_wgt_rd_0_data_bits_3_11; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_3_12 = io_wgt_rd_0_data_bits_3_12; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_3_13 = io_wgt_rd_0_data_bits_3_13; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_3_14 = io_wgt_rd_0_data_bits_3_14; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_3_15 = io_wgt_rd_0_data_bits_3_15; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_4_0 = io_wgt_rd_0_data_bits_4_0; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_4_1 = io_wgt_rd_0_data_bits_4_1; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_4_2 = io_wgt_rd_0_data_bits_4_2; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_4_3 = io_wgt_rd_0_data_bits_4_3; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_4_4 = io_wgt_rd_0_data_bits_4_4; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_4_5 = io_wgt_rd_0_data_bits_4_5; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_4_6 = io_wgt_rd_0_data_bits_4_6; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_4_7 = io_wgt_rd_0_data_bits_4_7; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_4_8 = io_wgt_rd_0_data_bits_4_8; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_4_9 = io_wgt_rd_0_data_bits_4_9; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_4_10 = io_wgt_rd_0_data_bits_4_10; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_4_11 = io_wgt_rd_0_data_bits_4_11; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_4_12 = io_wgt_rd_0_data_bits_4_12; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_4_13 = io_wgt_rd_0_data_bits_4_13; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_4_14 = io_wgt_rd_0_data_bits_4_14; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_4_15 = io_wgt_rd_0_data_bits_4_15; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_5_0 = io_wgt_rd_0_data_bits_5_0; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_5_1 = io_wgt_rd_0_data_bits_5_1; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_5_2 = io_wgt_rd_0_data_bits_5_2; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_5_3 = io_wgt_rd_0_data_bits_5_3; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_5_4 = io_wgt_rd_0_data_bits_5_4; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_5_5 = io_wgt_rd_0_data_bits_5_5; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_5_6 = io_wgt_rd_0_data_bits_5_6; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_5_7 = io_wgt_rd_0_data_bits_5_7; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_5_8 = io_wgt_rd_0_data_bits_5_8; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_5_9 = io_wgt_rd_0_data_bits_5_9; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_5_10 = io_wgt_rd_0_data_bits_5_10; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_5_11 = io_wgt_rd_0_data_bits_5_11; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_5_12 = io_wgt_rd_0_data_bits_5_12; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_5_13 = io_wgt_rd_0_data_bits_5_13; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_5_14 = io_wgt_rd_0_data_bits_5_14; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_5_15 = io_wgt_rd_0_data_bits_5_15; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_6_0 = io_wgt_rd_0_data_bits_6_0; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_6_1 = io_wgt_rd_0_data_bits_6_1; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_6_2 = io_wgt_rd_0_data_bits_6_2; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_6_3 = io_wgt_rd_0_data_bits_6_3; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_6_4 = io_wgt_rd_0_data_bits_6_4; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_6_5 = io_wgt_rd_0_data_bits_6_5; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_6_6 = io_wgt_rd_0_data_bits_6_6; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_6_7 = io_wgt_rd_0_data_bits_6_7; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_6_8 = io_wgt_rd_0_data_bits_6_8; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_6_9 = io_wgt_rd_0_data_bits_6_9; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_6_10 = io_wgt_rd_0_data_bits_6_10; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_6_11 = io_wgt_rd_0_data_bits_6_11; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_6_12 = io_wgt_rd_0_data_bits_6_12; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_6_13 = io_wgt_rd_0_data_bits_6_13; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_6_14 = io_wgt_rd_0_data_bits_6_14; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_6_15 = io_wgt_rd_0_data_bits_6_15; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_7_0 = io_wgt_rd_0_data_bits_7_0; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_7_1 = io_wgt_rd_0_data_bits_7_1; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_7_2 = io_wgt_rd_0_data_bits_7_2; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_7_3 = io_wgt_rd_0_data_bits_7_3; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_7_4 = io_wgt_rd_0_data_bits_7_4; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_7_5 = io_wgt_rd_0_data_bits_7_5; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_7_6 = io_wgt_rd_0_data_bits_7_6; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_7_7 = io_wgt_rd_0_data_bits_7_7; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_7_8 = io_wgt_rd_0_data_bits_7_8; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_7_9 = io_wgt_rd_0_data_bits_7_9; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_7_10 = io_wgt_rd_0_data_bits_7_10; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_7_11 = io_wgt_rd_0_data_bits_7_11; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_7_12 = io_wgt_rd_0_data_bits_7_12; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_7_13 = io_wgt_rd_0_data_bits_7_13; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_7_14 = io_wgt_rd_0_data_bits_7_14; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_7_15 = io_wgt_rd_0_data_bits_7_15; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_8_0 = io_wgt_rd_0_data_bits_8_0; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_8_1 = io_wgt_rd_0_data_bits_8_1; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_8_2 = io_wgt_rd_0_data_bits_8_2; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_8_3 = io_wgt_rd_0_data_bits_8_3; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_8_4 = io_wgt_rd_0_data_bits_8_4; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_8_5 = io_wgt_rd_0_data_bits_8_5; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_8_6 = io_wgt_rd_0_data_bits_8_6; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_8_7 = io_wgt_rd_0_data_bits_8_7; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_8_8 = io_wgt_rd_0_data_bits_8_8; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_8_9 = io_wgt_rd_0_data_bits_8_9; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_8_10 = io_wgt_rd_0_data_bits_8_10; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_8_11 = io_wgt_rd_0_data_bits_8_11; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_8_12 = io_wgt_rd_0_data_bits_8_12; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_8_13 = io_wgt_rd_0_data_bits_8_13; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_8_14 = io_wgt_rd_0_data_bits_8_14; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_8_15 = io_wgt_rd_0_data_bits_8_15; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_9_0 = io_wgt_rd_0_data_bits_9_0; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_9_1 = io_wgt_rd_0_data_bits_9_1; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_9_2 = io_wgt_rd_0_data_bits_9_2; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_9_3 = io_wgt_rd_0_data_bits_9_3; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_9_4 = io_wgt_rd_0_data_bits_9_4; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_9_5 = io_wgt_rd_0_data_bits_9_5; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_9_6 = io_wgt_rd_0_data_bits_9_6; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_9_7 = io_wgt_rd_0_data_bits_9_7; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_9_8 = io_wgt_rd_0_data_bits_9_8; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_9_9 = io_wgt_rd_0_data_bits_9_9; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_9_10 = io_wgt_rd_0_data_bits_9_10; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_9_11 = io_wgt_rd_0_data_bits_9_11; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_9_12 = io_wgt_rd_0_data_bits_9_12; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_9_13 = io_wgt_rd_0_data_bits_9_13; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_9_14 = io_wgt_rd_0_data_bits_9_14; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_9_15 = io_wgt_rd_0_data_bits_9_15; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_10_0 = io_wgt_rd_0_data_bits_10_0; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_10_1 = io_wgt_rd_0_data_bits_10_1; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_10_2 = io_wgt_rd_0_data_bits_10_2; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_10_3 = io_wgt_rd_0_data_bits_10_3; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_10_4 = io_wgt_rd_0_data_bits_10_4; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_10_5 = io_wgt_rd_0_data_bits_10_5; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_10_6 = io_wgt_rd_0_data_bits_10_6; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_10_7 = io_wgt_rd_0_data_bits_10_7; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_10_8 = io_wgt_rd_0_data_bits_10_8; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_10_9 = io_wgt_rd_0_data_bits_10_9; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_10_10 = io_wgt_rd_0_data_bits_10_10; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_10_11 = io_wgt_rd_0_data_bits_10_11; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_10_12 = io_wgt_rd_0_data_bits_10_12; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_10_13 = io_wgt_rd_0_data_bits_10_13; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_10_14 = io_wgt_rd_0_data_bits_10_14; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_10_15 = io_wgt_rd_0_data_bits_10_15; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_11_0 = io_wgt_rd_0_data_bits_11_0; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_11_1 = io_wgt_rd_0_data_bits_11_1; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_11_2 = io_wgt_rd_0_data_bits_11_2; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_11_3 = io_wgt_rd_0_data_bits_11_3; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_11_4 = io_wgt_rd_0_data_bits_11_4; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_11_5 = io_wgt_rd_0_data_bits_11_5; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_11_6 = io_wgt_rd_0_data_bits_11_6; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_11_7 = io_wgt_rd_0_data_bits_11_7; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_11_8 = io_wgt_rd_0_data_bits_11_8; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_11_9 = io_wgt_rd_0_data_bits_11_9; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_11_10 = io_wgt_rd_0_data_bits_11_10; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_11_11 = io_wgt_rd_0_data_bits_11_11; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_11_12 = io_wgt_rd_0_data_bits_11_12; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_11_13 = io_wgt_rd_0_data_bits_11_13; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_11_14 = io_wgt_rd_0_data_bits_11_14; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_11_15 = io_wgt_rd_0_data_bits_11_15; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_12_0 = io_wgt_rd_0_data_bits_12_0; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_12_1 = io_wgt_rd_0_data_bits_12_1; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_12_2 = io_wgt_rd_0_data_bits_12_2; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_12_3 = io_wgt_rd_0_data_bits_12_3; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_12_4 = io_wgt_rd_0_data_bits_12_4; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_12_5 = io_wgt_rd_0_data_bits_12_5; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_12_6 = io_wgt_rd_0_data_bits_12_6; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_12_7 = io_wgt_rd_0_data_bits_12_7; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_12_8 = io_wgt_rd_0_data_bits_12_8; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_12_9 = io_wgt_rd_0_data_bits_12_9; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_12_10 = io_wgt_rd_0_data_bits_12_10; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_12_11 = io_wgt_rd_0_data_bits_12_11; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_12_12 = io_wgt_rd_0_data_bits_12_12; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_12_13 = io_wgt_rd_0_data_bits_12_13; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_12_14 = io_wgt_rd_0_data_bits_12_14; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_12_15 = io_wgt_rd_0_data_bits_12_15; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_13_0 = io_wgt_rd_0_data_bits_13_0; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_13_1 = io_wgt_rd_0_data_bits_13_1; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_13_2 = io_wgt_rd_0_data_bits_13_2; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_13_3 = io_wgt_rd_0_data_bits_13_3; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_13_4 = io_wgt_rd_0_data_bits_13_4; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_13_5 = io_wgt_rd_0_data_bits_13_5; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_13_6 = io_wgt_rd_0_data_bits_13_6; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_13_7 = io_wgt_rd_0_data_bits_13_7; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_13_8 = io_wgt_rd_0_data_bits_13_8; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_13_9 = io_wgt_rd_0_data_bits_13_9; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_13_10 = io_wgt_rd_0_data_bits_13_10; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_13_11 = io_wgt_rd_0_data_bits_13_11; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_13_12 = io_wgt_rd_0_data_bits_13_12; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_13_13 = io_wgt_rd_0_data_bits_13_13; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_13_14 = io_wgt_rd_0_data_bits_13_14; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_13_15 = io_wgt_rd_0_data_bits_13_15; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_14_0 = io_wgt_rd_0_data_bits_14_0; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_14_1 = io_wgt_rd_0_data_bits_14_1; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_14_2 = io_wgt_rd_0_data_bits_14_2; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_14_3 = io_wgt_rd_0_data_bits_14_3; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_14_4 = io_wgt_rd_0_data_bits_14_4; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_14_5 = io_wgt_rd_0_data_bits_14_5; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_14_6 = io_wgt_rd_0_data_bits_14_6; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_14_7 = io_wgt_rd_0_data_bits_14_7; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_14_8 = io_wgt_rd_0_data_bits_14_8; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_14_9 = io_wgt_rd_0_data_bits_14_9; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_14_10 = io_wgt_rd_0_data_bits_14_10; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_14_11 = io_wgt_rd_0_data_bits_14_11; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_14_12 = io_wgt_rd_0_data_bits_14_12; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_14_13 = io_wgt_rd_0_data_bits_14_13; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_14_14 = io_wgt_rd_0_data_bits_14_14; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_14_15 = io_wgt_rd_0_data_bits_14_15; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_15_0 = io_wgt_rd_0_data_bits_15_0; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_15_1 = io_wgt_rd_0_data_bits_15_1; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_15_2 = io_wgt_rd_0_data_bits_15_2; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_15_3 = io_wgt_rd_0_data_bits_15_3; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_15_4 = io_wgt_rd_0_data_bits_15_4; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_15_5 = io_wgt_rd_0_data_bits_15_5; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_15_6 = io_wgt_rd_0_data_bits_15_6; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_15_7 = io_wgt_rd_0_data_bits_15_7; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_15_8 = io_wgt_rd_0_data_bits_15_8; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_15_9 = io_wgt_rd_0_data_bits_15_9; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_15_10 = io_wgt_rd_0_data_bits_15_10; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_15_11 = io_wgt_rd_0_data_bits_15_11; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_15_12 = io_wgt_rd_0_data_bits_15_12; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_15_13 = io_wgt_rd_0_data_bits_15_13; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_15_14 = io_wgt_rd_0_data_bits_15_14; // @[Compute.scala 167:21]
  assign tensorGemm_io_wgt_rd_0_data_bits_15_15 = io_wgt_rd_0_data_bits_15_15; // @[Compute.scala 167:21]
  assign tensorGemm_io_acc_rd_0_data_valid = tensorAcc_io_tensor_rd_0_data_valid & tensorGemm_io_acc_rd_0_data_valid_REG
    ; // @[Compute.scala 170:46]
  assign tensorGemm_io_acc_rd_0_data_bits_0_0 = tensorAcc_io_tensor_rd_0_data_bits_0_0; // @[Compute.scala 171:41]
  assign tensorGemm_io_acc_rd_0_data_bits_0_1 = tensorAcc_io_tensor_rd_0_data_bits_0_1; // @[Compute.scala 171:41]
  assign tensorGemm_io_acc_rd_0_data_bits_0_2 = tensorAcc_io_tensor_rd_0_data_bits_0_2; // @[Compute.scala 171:41]
  assign tensorGemm_io_acc_rd_0_data_bits_0_3 = tensorAcc_io_tensor_rd_0_data_bits_0_3; // @[Compute.scala 171:41]
  assign tensorGemm_io_acc_rd_0_data_bits_0_4 = tensorAcc_io_tensor_rd_0_data_bits_0_4; // @[Compute.scala 171:41]
  assign tensorGemm_io_acc_rd_0_data_bits_0_5 = tensorAcc_io_tensor_rd_0_data_bits_0_5; // @[Compute.scala 171:41]
  assign tensorGemm_io_acc_rd_0_data_bits_0_6 = tensorAcc_io_tensor_rd_0_data_bits_0_6; // @[Compute.scala 171:41]
  assign tensorGemm_io_acc_rd_0_data_bits_0_7 = tensorAcc_io_tensor_rd_0_data_bits_0_7; // @[Compute.scala 171:41]
  assign tensorGemm_io_acc_rd_0_data_bits_0_8 = tensorAcc_io_tensor_rd_0_data_bits_0_8; // @[Compute.scala 171:41]
  assign tensorGemm_io_acc_rd_0_data_bits_0_9 = tensorAcc_io_tensor_rd_0_data_bits_0_9; // @[Compute.scala 171:41]
  assign tensorGemm_io_acc_rd_0_data_bits_0_10 = tensorAcc_io_tensor_rd_0_data_bits_0_10; // @[Compute.scala 171:41]
  assign tensorGemm_io_acc_rd_0_data_bits_0_11 = tensorAcc_io_tensor_rd_0_data_bits_0_11; // @[Compute.scala 171:41]
  assign tensorGemm_io_acc_rd_0_data_bits_0_12 = tensorAcc_io_tensor_rd_0_data_bits_0_12; // @[Compute.scala 171:41]
  assign tensorGemm_io_acc_rd_0_data_bits_0_13 = tensorAcc_io_tensor_rd_0_data_bits_0_13; // @[Compute.scala 171:41]
  assign tensorGemm_io_acc_rd_0_data_bits_0_14 = tensorAcc_io_tensor_rd_0_data_bits_0_14; // @[Compute.scala 171:41]
  assign tensorGemm_io_acc_rd_0_data_bits_0_15 = tensorAcc_io_tensor_rd_0_data_bits_0_15; // @[Compute.scala 171:41]
  assign tensorGemm_io_out_rd_0_data_valid = 1'h0; // @[Compute.scala 175:33]
  assign tensorAlu_clock = clock;
  assign tensorAlu_reset = reset;
  assign tensorAlu_io_start = tensorAlu_io_start_REG; // @[Compute.scala 180:22]
  assign tensorAlu_io_dec_alu_imm = _tensorGemm_io_dec_WIRE_1[127:112]; // @[Compute.scala 181:50]
  assign tensorAlu_io_dec_alu_use_imm = _tensorGemm_io_dec_WIRE_1[111]; // @[Compute.scala 181:50]
  assign tensorAlu_io_dec_alu_op = _tensorGemm_io_dec_WIRE_1[110:108]; // @[Compute.scala 181:50]
  assign tensorAlu_io_dec_src_1 = _tensorGemm_io_dec_WIRE_1[107:97]; // @[Compute.scala 181:50]
  assign tensorAlu_io_dec_src_0 = _tensorGemm_io_dec_WIRE_1[96:86]; // @[Compute.scala 181:50]
  assign tensorAlu_io_dec_dst_1 = _tensorGemm_io_dec_WIRE_1[85:75]; // @[Compute.scala 181:50]
  assign tensorAlu_io_dec_dst_0 = _tensorGemm_io_dec_WIRE_1[74:64]; // @[Compute.scala 181:50]
  assign tensorAlu_io_dec_lp_1 = _tensorGemm_io_dec_WIRE_1[62:49]; // @[Compute.scala 181:50]
  assign tensorAlu_io_dec_lp_0 = _tensorGemm_io_dec_WIRE_1[48:35]; // @[Compute.scala 181:50]
  assign tensorAlu_io_dec_uop_end = _tensorGemm_io_dec_WIRE_1[34:21]; // @[Compute.scala 181:50]
  assign tensorAlu_io_dec_uop_begin = _tensorGemm_io_dec_WIRE_1[20:8]; // @[Compute.scala 181:50]
  assign tensorAlu_io_uop_data_bits_u2 = loadUop_io_uop_data_bits_u2; // @[Compute.scala 183:30]
  assign tensorAlu_io_uop_data_bits_u1 = loadUop_io_uop_data_bits_u1; // @[Compute.scala 183:30]
  assign tensorAlu_io_uop_data_bits_u0 = loadUop_io_uop_data_bits_u0; // @[Compute.scala 183:30]
  assign tensorAlu_io_acc_rd_0_data_valid = tensorAcc_io_tensor_rd_0_data_valid & tensorAlu_io_acc_rd_0_data_valid_REG; // @[Compute.scala 186:46]
  assign tensorAlu_io_acc_rd_0_data_bits_0_0 = tensorAcc_io_tensor_rd_0_data_bits_0_0; // @[Compute.scala 187:40]
  assign tensorAlu_io_acc_rd_0_data_bits_0_1 = tensorAcc_io_tensor_rd_0_data_bits_0_1; // @[Compute.scala 187:40]
  assign tensorAlu_io_acc_rd_0_data_bits_0_2 = tensorAcc_io_tensor_rd_0_data_bits_0_2; // @[Compute.scala 187:40]
  assign tensorAlu_io_acc_rd_0_data_bits_0_3 = tensorAcc_io_tensor_rd_0_data_bits_0_3; // @[Compute.scala 187:40]
  assign tensorAlu_io_acc_rd_0_data_bits_0_4 = tensorAcc_io_tensor_rd_0_data_bits_0_4; // @[Compute.scala 187:40]
  assign tensorAlu_io_acc_rd_0_data_bits_0_5 = tensorAcc_io_tensor_rd_0_data_bits_0_5; // @[Compute.scala 187:40]
  assign tensorAlu_io_acc_rd_0_data_bits_0_6 = tensorAcc_io_tensor_rd_0_data_bits_0_6; // @[Compute.scala 187:40]
  assign tensorAlu_io_acc_rd_0_data_bits_0_7 = tensorAcc_io_tensor_rd_0_data_bits_0_7; // @[Compute.scala 187:40]
  assign tensorAlu_io_acc_rd_0_data_bits_0_8 = tensorAcc_io_tensor_rd_0_data_bits_0_8; // @[Compute.scala 187:40]
  assign tensorAlu_io_acc_rd_0_data_bits_0_9 = tensorAcc_io_tensor_rd_0_data_bits_0_9; // @[Compute.scala 187:40]
  assign tensorAlu_io_acc_rd_0_data_bits_0_10 = tensorAcc_io_tensor_rd_0_data_bits_0_10; // @[Compute.scala 187:40]
  assign tensorAlu_io_acc_rd_0_data_bits_0_11 = tensorAcc_io_tensor_rd_0_data_bits_0_11; // @[Compute.scala 187:40]
  assign tensorAlu_io_acc_rd_0_data_bits_0_12 = tensorAcc_io_tensor_rd_0_data_bits_0_12; // @[Compute.scala 187:40]
  assign tensorAlu_io_acc_rd_0_data_bits_0_13 = tensorAcc_io_tensor_rd_0_data_bits_0_13; // @[Compute.scala 187:40]
  assign tensorAlu_io_acc_rd_0_data_bits_0_14 = tensorAcc_io_tensor_rd_0_data_bits_0_14; // @[Compute.scala 187:40]
  assign tensorAlu_io_acc_rd_0_data_bits_0_15 = tensorAcc_io_tensor_rd_0_data_bits_0_15; // @[Compute.scala 187:40]
  assign tensorAlu_io_out_rd_0_data_valid = 1'h0; // @[Compute.scala 191:33]
  assign inst_q_clock = clock;
  assign inst_q_reset = reset;
  assign inst_q_io_enq_valid = io_inst_valid; // @[Compute.scala 120:17]
  assign inst_q_io_enq_bits = io_inst_bits; // @[Compute.scala 120:17]
  assign inst_q_io_deq_ready = state == 2'h2 & done | state == 2'h1; // @[Compute.scala 121:50]
  assign dec_io_inst = inst_q_io_deq_bits; // @[Compute.scala 73:15]
  always @(posedge clock) begin
    if (reset) begin // @[Compute.scala 55:22]
      state <= 2'h0; // @[Compute.scala 55:22]
    end else if (_T) begin // @[Conditional.scala 40:58]
      if (start) begin // @[Compute.scala 101:19]
        if (dec_io_isSync) begin // @[Compute.scala 102:29]
          state <= 2'h1; // @[Compute.scala 103:17]
        end else begin
          state <= _GEN_0;
        end
      end
    end else if (_T_2) begin // @[Conditional.scala 39:67]
      state <= 2'h0; // @[Compute.scala 110:13]
    end else if (_T_3) begin // @[Conditional.scala 39:67]
      state <= _GEN_3;
    end
    if (reset) begin // @[Compute.scala 150:14]
      tensorAcc_io_tensor_rd_0_idx_REG <= 1'h0; // @[Compute.scala 150:14]
    end else begin
      tensorAcc_io_tensor_rd_0_idx_REG <= dec_io_isGemm; // @[Compute.scala 150:14]
    end
    if (reset) begin // @[Compute.scala 154:14]
      tensorAcc_io_tensor_wr_0_REG <= 1'h0; // @[Compute.scala 154:14]
    end else begin
      tensorAcc_io_tensor_wr_0_REG <= dec_io_isGemm; // @[Compute.scala 154:14]
    end
    if (reset) begin // @[Compute.scala 162:33]
      tensorGemm_io_start_REG <= 1'h0; // @[Compute.scala 162:33]
    end else begin
      tensorGemm_io_start_REG <= _loadUop_io_start_T_1 & dec_io_isGemm; // @[Compute.scala 162:33]
    end
    if (reset) begin // @[Compute.scala 170:55]
      tensorGemm_io_acc_rd_0_data_valid_REG <= 1'h0; // @[Compute.scala 170:55]
    end else begin
      tensorGemm_io_acc_rd_0_data_valid_REG <= dec_io_isGemm; // @[Compute.scala 170:55]
    end
    if (reset) begin // @[Compute.scala 180:32]
      tensorAlu_io_start_REG <= 1'h0; // @[Compute.scala 180:32]
    end else begin
      tensorAlu_io_start_REG <= _loadUop_io_start_T_1 & dec_io_isAlu; // @[Compute.scala 180:32]
    end
    if (reset) begin // @[Compute.scala 186:55]
      tensorAlu_io_acc_rd_0_data_valid_REG <= 1'h0; // @[Compute.scala 186:55]
    end else begin
      tensorAlu_io_acc_rd_0_data_valid_REG <= dec_io_isAlu; // @[Compute.scala 186:55]
    end
    if (reset) begin // @[Compute.scala 208:12]
      io_out_wr_0_valid_REG <= 1'h0; // @[Compute.scala 208:12]
    end else begin
      io_out_wr_0_valid_REG <= dec_io_isGemm; // @[Compute.scala 208:12]
    end
    if (reset) begin // @[Compute.scala 210:12]
      io_out_wr_0_bits_idx_REG <= 1'h0; // @[Compute.scala 210:12]
    end else begin
      io_out_wr_0_bits_idx_REG <= dec_io_isGemm; // @[Compute.scala 210:12]
    end
    if (reset) begin // @[Compute.scala 221:14]
      outDataBits_0_REG <= 1'h0; // @[Compute.scala 221:14]
    end else begin
      outDataBits_0_REG <= dec_io_isGemm; // @[Compute.scala 221:14]
    end
    `ifndef SYNTHESIS
    `ifdef PRINTF_COND
      if (`PRINTF_COND) begin
    `endif
        if (~(~tensorGemm_io_uop_idx_valid | ~tensorAlu_io_uop_idx_valid | reset)) begin
          $fwrite(32'h80000002,
            "Assertion failed\n    at Compute.scala:129 assert(!tensorGemm.io.uop.idx.valid || !tensorAlu.io.uop.idx.valid)\n"
            ); // @[Compute.scala 129:9]
        end
    `ifdef PRINTF_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef STOP_COND
      if (`STOP_COND) begin
    `endif
        if (~(~tensorGemm_io_uop_idx_valid | ~tensorAlu_io_uop_idx_valid | reset)) begin
          $fatal; // @[Compute.scala 129:9]
        end
    `ifdef STOP_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef PRINTF_COND
      if (`PRINTF_COND) begin
    `endif
        if (~(~tensorGemm_io_out_rd_0_data_valid | ~tensorAlu_io_out_rd_0_data_valid | reset)) begin
          $fwrite(32'h80000002,
            "Assertion failed\n    at Compute.scala:201 assert(!tensorGemm.io.out.rd(idx).data.valid || !tensorAlu.io.out.rd(idx).data.valid)\n"
            ); // @[Compute.scala 201:11]
        end
    `ifdef PRINTF_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef STOP_COND
      if (`STOP_COND) begin
    `endif
        if (~(~tensorGemm_io_out_rd_0_data_valid | ~tensorAlu_io_out_rd_0_data_valid | reset)) begin
          $fatal; // @[Compute.scala 201:11]
        end
    `ifdef STOP_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef PRINTF_COND
      if (`PRINTF_COND) begin
    `endif
        if (~(~tensorGemm_io_out_wr_0_valid | ~tensorAlu_io_out_wr_0_valid | reset)) begin
          $fwrite(32'h80000002,
            "Assertion failed\n    at Compute.scala:203 assert(!tensorGemm.io.out.wr(idx).valid || !tensorAlu.io.out.wr(idx).valid)\n"
            ); // @[Compute.scala 203:11]
        end
    `ifdef PRINTF_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef STOP_COND
      if (`STOP_COND) begin
    `endif
        if (~(~tensorGemm_io_out_wr_0_valid | ~tensorAlu_io_out_wr_0_valid | reset)) begin
          $fatal; // @[Compute.scala 203:11]
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
  tensorAcc_io_tensor_rd_0_idx_REG = _RAND_1[0:0];
  _RAND_2 = {1{`RANDOM}};
  tensorAcc_io_tensor_wr_0_REG = _RAND_2[0:0];
  _RAND_3 = {1{`RANDOM}};
  tensorGemm_io_start_REG = _RAND_3[0:0];
  _RAND_4 = {1{`RANDOM}};
  tensorGemm_io_acc_rd_0_data_valid_REG = _RAND_4[0:0];
  _RAND_5 = {1{`RANDOM}};
  tensorAlu_io_start_REG = _RAND_5[0:0];
  _RAND_6 = {1{`RANDOM}};
  tensorAlu_io_acc_rd_0_data_valid_REG = _RAND_6[0:0];
  _RAND_7 = {1{`RANDOM}};
  io_out_wr_0_valid_REG = _RAND_7[0:0];
  _RAND_8 = {1{`RANDOM}};
  io_out_wr_0_bits_idx_REG = _RAND_8[0:0];
  _RAND_9 = {1{`RANDOM}};
  outDataBits_0_REG = _RAND_9[0:0];
`endif // RANDOMIZE_REG_INIT
  `endif // RANDOMIZE
end // initial
`ifdef FIRRTL_AFTER_INITIAL
`FIRRTL_AFTER_INITIAL
`endif
`endif // SYNTHESIS
endmodule