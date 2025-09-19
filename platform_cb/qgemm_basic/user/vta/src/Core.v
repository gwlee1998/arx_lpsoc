module Core(
  input         clock,
  input         reset,
  input         io_vcr_launch,
  output        io_vcr_finish,
  output        io_vcr_ecnt_0_valid,
  output [31:0] io_vcr_ecnt_0_bits,
  input  [31:0] io_vcr_vals_0,
  input  [31:0] io_vcr_ptrs_0,
  input  [31:0] io_vcr_ptrs_1,
  input  [31:0] io_vcr_ptrs_2,
  input  [31:0] io_vcr_ptrs_3,
  input  [31:0] io_vcr_ptrs_4,
  input  [31:0] io_vcr_ptrs_5,
  output        io_vcr_ucnt_0_valid,
  output [31:0] io_vcr_ucnt_0_bits,
  input         io_vme_rd_0_cmd_ready,
  output        io_vme_rd_0_cmd_valid,
  output [31:0] io_vme_rd_0_cmd_bits_addr,
  output [7:0]  io_vme_rd_0_cmd_bits_len,
  output        io_vme_rd_0_data_ready,
  input         io_vme_rd_0_data_valid,
  input  [63:0] io_vme_rd_0_data_bits_data,
  input         io_vme_rd_1_cmd_ready,
  output        io_vme_rd_1_cmd_valid,
  output [31:0] io_vme_rd_1_cmd_bits_addr,
  output [7:0]  io_vme_rd_1_cmd_bits_len,
  output [20:0] io_vme_rd_1_cmd_bits_tag,
  input         io_vme_rd_1_data_valid,
  input  [63:0] io_vme_rd_1_data_bits_data,
  input  [20:0] io_vme_rd_1_data_bits_tag,
  input         io_vme_rd_1_data_bits_last,
  input         io_vme_rd_2_cmd_ready,
  output        io_vme_rd_2_cmd_valid,
  output [31:0] io_vme_rd_2_cmd_bits_addr,
  output [7:0]  io_vme_rd_2_cmd_bits_len,
  output [20:0] io_vme_rd_2_cmd_bits_tag,
  input         io_vme_rd_2_data_valid,
  input  [63:0] io_vme_rd_2_data_bits_data,
  input  [20:0] io_vme_rd_2_data_bits_tag,
  input         io_vme_rd_3_cmd_ready,
  output        io_vme_rd_3_cmd_valid,
  output [31:0] io_vme_rd_3_cmd_bits_addr,
  output [7:0]  io_vme_rd_3_cmd_bits_len,
  output [20:0] io_vme_rd_3_cmd_bits_tag,
  input         io_vme_rd_3_data_valid,
  input  [63:0] io_vme_rd_3_data_bits_data,
  input  [20:0] io_vme_rd_3_data_bits_tag,
  input         io_vme_rd_4_cmd_ready,
  output        io_vme_rd_4_cmd_valid,
  output [31:0] io_vme_rd_4_cmd_bits_addr,
  output [7:0]  io_vme_rd_4_cmd_bits_len,
  output [20:0] io_vme_rd_4_cmd_bits_tag,
  input         io_vme_rd_4_data_valid,
  input  [63:0] io_vme_rd_4_data_bits_data,
  input  [20:0] io_vme_rd_4_data_bits_tag,
  input         io_vme_wr_0_cmd_ready,
  output        io_vme_wr_0_cmd_valid,
  output [31:0] io_vme_wr_0_cmd_bits_addr,
  output [7:0]  io_vme_wr_0_cmd_bits_len,
  input         io_vme_wr_0_data_ready,
  output        io_vme_wr_0_data_valid,
  output [63:0] io_vme_wr_0_data_bits_data,
  input         io_vme_wr_0_ack
);
`ifdef RANDOMIZE_REG_INIT
  reg [31:0] _RAND_0;
`endif // RANDOMIZE_REG_INIT
  wire  fetch_clock; // @[Core.scala 67:21]
  wire  fetch_reset; // @[Core.scala 67:21]
  wire  fetch_io_launch; // @[Core.scala 67:21]
  wire [31:0] fetch_io_ins_baddr; // @[Core.scala 67:21]
  wire [31:0] fetch_io_ins_count; // @[Core.scala 67:21]
  wire  fetch_io_vme_rd_cmd_ready; // @[Core.scala 67:21]
  wire  fetch_io_vme_rd_cmd_valid; // @[Core.scala 67:21]
  wire [31:0] fetch_io_vme_rd_cmd_bits_addr; // @[Core.scala 67:21]
  wire [7:0] fetch_io_vme_rd_cmd_bits_len; // @[Core.scala 67:21]
  wire  fetch_io_vme_rd_data_ready; // @[Core.scala 67:21]
  wire  fetch_io_vme_rd_data_valid; // @[Core.scala 67:21]
  wire [63:0] fetch_io_vme_rd_data_bits_data; // @[Core.scala 67:21]
  wire  fetch_io_inst_ld_ready; // @[Core.scala 67:21]
  wire  fetch_io_inst_ld_valid; // @[Core.scala 67:21]
  wire [127:0] fetch_io_inst_ld_bits; // @[Core.scala 67:21]
  wire  fetch_io_inst_co_ready; // @[Core.scala 67:21]
  wire  fetch_io_inst_co_valid; // @[Core.scala 67:21]
  wire [127:0] fetch_io_inst_co_bits; // @[Core.scala 67:21]
  wire  fetch_io_inst_st_ready; // @[Core.scala 67:21]
  wire  fetch_io_inst_st_valid; // @[Core.scala 67:21]
  wire [127:0] fetch_io_inst_st_bits; // @[Core.scala 67:21]
  wire  load_clock; // @[Core.scala 68:20]
  wire  load_reset; // @[Core.scala 68:20]
  wire  load_io_i_post; // @[Core.scala 68:20]
  wire  load_io_o_post; // @[Core.scala 68:20]
  wire  load_io_inst_ready; // @[Core.scala 68:20]
  wire  load_io_inst_valid; // @[Core.scala 68:20]
  wire [127:0] load_io_inst_bits; // @[Core.scala 68:20]
  wire [31:0] load_io_inp_baddr; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_baddr; // @[Core.scala 68:20]
  wire  load_io_vme_rd_0_cmd_ready; // @[Core.scala 68:20]
  wire  load_io_vme_rd_0_cmd_valid; // @[Core.scala 68:20]
  wire [31:0] load_io_vme_rd_0_cmd_bits_addr; // @[Core.scala 68:20]
  wire [7:0] load_io_vme_rd_0_cmd_bits_len; // @[Core.scala 68:20]
  wire [20:0] load_io_vme_rd_0_cmd_bits_tag; // @[Core.scala 68:20]
  wire  load_io_vme_rd_0_data_valid; // @[Core.scala 68:20]
  wire [63:0] load_io_vme_rd_0_data_bits_data; // @[Core.scala 68:20]
  wire [20:0] load_io_vme_rd_0_data_bits_tag; // @[Core.scala 68:20]
  wire  load_io_vme_rd_1_cmd_ready; // @[Core.scala 68:20]
  wire  load_io_vme_rd_1_cmd_valid; // @[Core.scala 68:20]
  wire [31:0] load_io_vme_rd_1_cmd_bits_addr; // @[Core.scala 68:20]
  wire [7:0] load_io_vme_rd_1_cmd_bits_len; // @[Core.scala 68:20]
  wire [20:0] load_io_vme_rd_1_cmd_bits_tag; // @[Core.scala 68:20]
  wire  load_io_vme_rd_1_data_valid; // @[Core.scala 68:20]
  wire [63:0] load_io_vme_rd_1_data_bits_data; // @[Core.scala 68:20]
  wire [20:0] load_io_vme_rd_1_data_bits_tag; // @[Core.scala 68:20]
  wire  load_io_inp_rd_0_idx_valid; // @[Core.scala 68:20]
  wire [10:0] load_io_inp_rd_0_idx_bits; // @[Core.scala 68:20]
  wire  load_io_inp_rd_0_data_valid; // @[Core.scala 68:20]
  wire [31:0] load_io_inp_rd_0_data_bits_0_0; // @[Core.scala 68:20]
  wire [31:0] load_io_inp_rd_0_data_bits_0_1; // @[Core.scala 68:20]
  wire [31:0] load_io_inp_rd_0_data_bits_0_2; // @[Core.scala 68:20]
  wire [31:0] load_io_inp_rd_0_data_bits_0_3; // @[Core.scala 68:20]
  wire [31:0] load_io_inp_rd_0_data_bits_0_4; // @[Core.scala 68:20]
  wire [31:0] load_io_inp_rd_0_data_bits_0_5; // @[Core.scala 68:20]
  wire [31:0] load_io_inp_rd_0_data_bits_0_6; // @[Core.scala 68:20]
  wire [31:0] load_io_inp_rd_0_data_bits_0_7; // @[Core.scala 68:20]
  wire [31:0] load_io_inp_rd_0_data_bits_0_8; // @[Core.scala 68:20]
  wire [31:0] load_io_inp_rd_0_data_bits_0_9; // @[Core.scala 68:20]
  wire [31:0] load_io_inp_rd_0_data_bits_0_10; // @[Core.scala 68:20]
  wire [31:0] load_io_inp_rd_0_data_bits_0_11; // @[Core.scala 68:20]
  wire [31:0] load_io_inp_rd_0_data_bits_0_12; // @[Core.scala 68:20]
  wire [31:0] load_io_inp_rd_0_data_bits_0_13; // @[Core.scala 68:20]
  wire [31:0] load_io_inp_rd_0_data_bits_0_14; // @[Core.scala 68:20]
  wire [31:0] load_io_inp_rd_0_data_bits_0_15; // @[Core.scala 68:20]
  wire  load_io_wgt_rd_0_idx_valid; // @[Core.scala 68:20]
  wire [9:0] load_io_wgt_rd_0_idx_bits; // @[Core.scala 68:20]
  wire  load_io_wgt_rd_0_data_valid; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_0_0; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_0_1; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_0_2; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_0_3; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_0_4; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_0_5; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_0_6; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_0_7; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_0_8; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_0_9; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_0_10; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_0_11; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_0_12; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_0_13; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_0_14; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_0_15; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_1_0; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_1_1; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_1_2; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_1_3; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_1_4; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_1_5; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_1_6; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_1_7; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_1_8; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_1_9; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_1_10; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_1_11; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_1_12; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_1_13; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_1_14; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_1_15; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_2_0; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_2_1; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_2_2; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_2_3; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_2_4; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_2_5; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_2_6; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_2_7; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_2_8; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_2_9; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_2_10; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_2_11; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_2_12; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_2_13; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_2_14; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_2_15; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_3_0; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_3_1; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_3_2; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_3_3; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_3_4; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_3_5; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_3_6; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_3_7; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_3_8; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_3_9; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_3_10; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_3_11; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_3_12; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_3_13; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_3_14; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_3_15; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_4_0; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_4_1; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_4_2; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_4_3; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_4_4; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_4_5; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_4_6; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_4_7; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_4_8; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_4_9; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_4_10; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_4_11; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_4_12; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_4_13; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_4_14; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_4_15; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_5_0; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_5_1; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_5_2; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_5_3; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_5_4; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_5_5; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_5_6; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_5_7; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_5_8; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_5_9; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_5_10; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_5_11; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_5_12; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_5_13; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_5_14; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_5_15; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_6_0; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_6_1; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_6_2; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_6_3; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_6_4; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_6_5; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_6_6; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_6_7; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_6_8; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_6_9; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_6_10; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_6_11; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_6_12; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_6_13; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_6_14; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_6_15; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_7_0; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_7_1; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_7_2; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_7_3; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_7_4; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_7_5; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_7_6; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_7_7; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_7_8; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_7_9; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_7_10; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_7_11; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_7_12; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_7_13; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_7_14; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_7_15; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_8_0; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_8_1; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_8_2; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_8_3; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_8_4; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_8_5; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_8_6; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_8_7; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_8_8; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_8_9; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_8_10; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_8_11; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_8_12; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_8_13; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_8_14; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_8_15; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_9_0; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_9_1; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_9_2; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_9_3; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_9_4; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_9_5; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_9_6; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_9_7; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_9_8; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_9_9; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_9_10; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_9_11; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_9_12; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_9_13; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_9_14; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_9_15; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_10_0; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_10_1; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_10_2; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_10_3; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_10_4; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_10_5; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_10_6; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_10_7; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_10_8; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_10_9; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_10_10; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_10_11; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_10_12; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_10_13; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_10_14; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_10_15; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_11_0; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_11_1; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_11_2; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_11_3; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_11_4; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_11_5; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_11_6; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_11_7; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_11_8; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_11_9; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_11_10; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_11_11; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_11_12; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_11_13; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_11_14; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_11_15; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_12_0; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_12_1; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_12_2; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_12_3; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_12_4; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_12_5; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_12_6; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_12_7; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_12_8; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_12_9; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_12_10; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_12_11; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_12_12; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_12_13; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_12_14; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_12_15; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_13_0; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_13_1; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_13_2; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_13_3; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_13_4; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_13_5; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_13_6; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_13_7; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_13_8; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_13_9; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_13_10; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_13_11; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_13_12; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_13_13; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_13_14; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_13_15; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_14_0; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_14_1; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_14_2; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_14_3; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_14_4; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_14_5; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_14_6; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_14_7; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_14_8; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_14_9; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_14_10; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_14_11; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_14_12; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_14_13; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_14_14; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_14_15; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_15_0; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_15_1; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_15_2; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_15_3; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_15_4; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_15_5; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_15_6; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_15_7; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_15_8; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_15_9; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_15_10; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_15_11; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_15_12; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_15_13; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_15_14; // @[Core.scala 68:20]
  wire [31:0] load_io_wgt_rd_0_data_bits_15_15; // @[Core.scala 68:20]
  wire  compute_clock; // @[Core.scala 69:23]
  wire  compute_reset; // @[Core.scala 69:23]
  wire  compute_io_i_post_0; // @[Core.scala 69:23]
  wire  compute_io_i_post_1; // @[Core.scala 69:23]
  wire  compute_io_o_post_0; // @[Core.scala 69:23]
  wire  compute_io_o_post_1; // @[Core.scala 69:23]
  wire  compute_io_inst_ready; // @[Core.scala 69:23]
  wire  compute_io_inst_valid; // @[Core.scala 69:23]
  wire [127:0] compute_io_inst_bits; // @[Core.scala 69:23]
  wire [31:0] compute_io_uop_baddr; // @[Core.scala 69:23]
  wire [31:0] compute_io_acc_baddr; // @[Core.scala 69:23]
  wire  compute_io_vme_rd_0_cmd_ready; // @[Core.scala 69:23]
  wire  compute_io_vme_rd_0_cmd_valid; // @[Core.scala 69:23]
  wire [31:0] compute_io_vme_rd_0_cmd_bits_addr; // @[Core.scala 69:23]
  wire [7:0] compute_io_vme_rd_0_cmd_bits_len; // @[Core.scala 69:23]
  wire [20:0] compute_io_vme_rd_0_cmd_bits_tag; // @[Core.scala 69:23]
  wire  compute_io_vme_rd_0_data_valid; // @[Core.scala 69:23]
  wire [63:0] compute_io_vme_rd_0_data_bits_data; // @[Core.scala 69:23]
  wire [20:0] compute_io_vme_rd_0_data_bits_tag; // @[Core.scala 69:23]
  wire  compute_io_vme_rd_0_data_bits_last; // @[Core.scala 69:23]
  wire  compute_io_vme_rd_1_cmd_ready; // @[Core.scala 69:23]
  wire  compute_io_vme_rd_1_cmd_valid; // @[Core.scala 69:23]
  wire [31:0] compute_io_vme_rd_1_cmd_bits_addr; // @[Core.scala 69:23]
  wire [7:0] compute_io_vme_rd_1_cmd_bits_len; // @[Core.scala 69:23]
  wire [20:0] compute_io_vme_rd_1_cmd_bits_tag; // @[Core.scala 69:23]
  wire  compute_io_vme_rd_1_data_valid; // @[Core.scala 69:23]
  wire [63:0] compute_io_vme_rd_1_data_bits_data; // @[Core.scala 69:23]
  wire [20:0] compute_io_vme_rd_1_data_bits_tag; // @[Core.scala 69:23]
  wire  compute_io_inp_rd_0_idx_valid; // @[Core.scala 69:23]
  wire [10:0] compute_io_inp_rd_0_idx_bits; // @[Core.scala 69:23]
  wire  compute_io_inp_rd_0_data_valid; // @[Core.scala 69:23]
  wire [31:0] compute_io_inp_rd_0_data_bits_0_0; // @[Core.scala 69:23]
  wire [31:0] compute_io_inp_rd_0_data_bits_0_1; // @[Core.scala 69:23]
  wire [31:0] compute_io_inp_rd_0_data_bits_0_2; // @[Core.scala 69:23]
  wire [31:0] compute_io_inp_rd_0_data_bits_0_3; // @[Core.scala 69:23]
  wire [31:0] compute_io_inp_rd_0_data_bits_0_4; // @[Core.scala 69:23]
  wire [31:0] compute_io_inp_rd_0_data_bits_0_5; // @[Core.scala 69:23]
  wire [31:0] compute_io_inp_rd_0_data_bits_0_6; // @[Core.scala 69:23]
  wire [31:0] compute_io_inp_rd_0_data_bits_0_7; // @[Core.scala 69:23]
  wire [31:0] compute_io_inp_rd_0_data_bits_0_8; // @[Core.scala 69:23]
  wire [31:0] compute_io_inp_rd_0_data_bits_0_9; // @[Core.scala 69:23]
  wire [31:0] compute_io_inp_rd_0_data_bits_0_10; // @[Core.scala 69:23]
  wire [31:0] compute_io_inp_rd_0_data_bits_0_11; // @[Core.scala 69:23]
  wire [31:0] compute_io_inp_rd_0_data_bits_0_12; // @[Core.scala 69:23]
  wire [31:0] compute_io_inp_rd_0_data_bits_0_13; // @[Core.scala 69:23]
  wire [31:0] compute_io_inp_rd_0_data_bits_0_14; // @[Core.scala 69:23]
  wire [31:0] compute_io_inp_rd_0_data_bits_0_15; // @[Core.scala 69:23]
  wire  compute_io_wgt_rd_0_idx_valid; // @[Core.scala 69:23]
  wire [9:0] compute_io_wgt_rd_0_idx_bits; // @[Core.scala 69:23]
  wire  compute_io_wgt_rd_0_data_valid; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_0_0; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_0_1; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_0_2; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_0_3; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_0_4; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_0_5; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_0_6; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_0_7; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_0_8; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_0_9; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_0_10; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_0_11; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_0_12; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_0_13; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_0_14; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_0_15; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_1_0; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_1_1; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_1_2; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_1_3; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_1_4; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_1_5; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_1_6; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_1_7; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_1_8; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_1_9; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_1_10; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_1_11; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_1_12; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_1_13; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_1_14; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_1_15; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_2_0; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_2_1; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_2_2; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_2_3; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_2_4; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_2_5; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_2_6; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_2_7; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_2_8; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_2_9; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_2_10; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_2_11; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_2_12; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_2_13; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_2_14; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_2_15; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_3_0; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_3_1; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_3_2; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_3_3; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_3_4; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_3_5; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_3_6; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_3_7; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_3_8; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_3_9; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_3_10; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_3_11; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_3_12; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_3_13; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_3_14; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_3_15; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_4_0; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_4_1; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_4_2; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_4_3; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_4_4; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_4_5; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_4_6; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_4_7; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_4_8; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_4_9; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_4_10; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_4_11; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_4_12; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_4_13; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_4_14; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_4_15; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_5_0; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_5_1; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_5_2; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_5_3; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_5_4; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_5_5; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_5_6; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_5_7; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_5_8; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_5_9; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_5_10; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_5_11; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_5_12; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_5_13; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_5_14; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_5_15; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_6_0; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_6_1; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_6_2; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_6_3; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_6_4; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_6_5; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_6_6; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_6_7; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_6_8; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_6_9; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_6_10; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_6_11; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_6_12; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_6_13; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_6_14; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_6_15; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_7_0; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_7_1; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_7_2; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_7_3; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_7_4; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_7_5; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_7_6; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_7_7; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_7_8; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_7_9; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_7_10; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_7_11; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_7_12; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_7_13; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_7_14; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_7_15; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_8_0; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_8_1; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_8_2; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_8_3; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_8_4; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_8_5; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_8_6; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_8_7; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_8_8; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_8_9; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_8_10; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_8_11; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_8_12; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_8_13; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_8_14; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_8_15; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_9_0; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_9_1; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_9_2; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_9_3; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_9_4; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_9_5; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_9_6; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_9_7; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_9_8; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_9_9; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_9_10; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_9_11; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_9_12; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_9_13; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_9_14; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_9_15; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_10_0; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_10_1; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_10_2; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_10_3; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_10_4; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_10_5; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_10_6; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_10_7; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_10_8; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_10_9; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_10_10; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_10_11; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_10_12; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_10_13; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_10_14; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_10_15; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_11_0; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_11_1; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_11_2; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_11_3; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_11_4; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_11_5; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_11_6; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_11_7; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_11_8; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_11_9; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_11_10; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_11_11; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_11_12; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_11_13; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_11_14; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_11_15; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_12_0; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_12_1; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_12_2; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_12_3; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_12_4; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_12_5; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_12_6; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_12_7; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_12_8; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_12_9; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_12_10; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_12_11; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_12_12; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_12_13; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_12_14; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_12_15; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_13_0; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_13_1; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_13_2; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_13_3; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_13_4; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_13_5; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_13_6; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_13_7; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_13_8; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_13_9; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_13_10; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_13_11; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_13_12; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_13_13; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_13_14; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_13_15; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_14_0; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_14_1; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_14_2; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_14_3; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_14_4; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_14_5; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_14_6; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_14_7; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_14_8; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_14_9; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_14_10; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_14_11; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_14_12; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_14_13; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_14_14; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_14_15; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_15_0; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_15_1; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_15_2; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_15_3; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_15_4; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_15_5; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_15_6; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_15_7; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_15_8; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_15_9; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_15_10; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_15_11; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_15_12; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_15_13; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_15_14; // @[Core.scala 69:23]
  wire [31:0] compute_io_wgt_rd_0_data_bits_15_15; // @[Core.scala 69:23]
  wire  compute_io_out_wr_0_valid; // @[Core.scala 69:23]
  wire [10:0] compute_io_out_wr_0_bits_idx; // @[Core.scala 69:23]
  wire [31:0] compute_io_out_wr_0_bits_data_0_0; // @[Core.scala 69:23]
  wire [31:0] compute_io_out_wr_0_bits_data_0_1; // @[Core.scala 69:23]
  wire [31:0] compute_io_out_wr_0_bits_data_0_2; // @[Core.scala 69:23]
  wire [31:0] compute_io_out_wr_0_bits_data_0_3; // @[Core.scala 69:23]
  wire [31:0] compute_io_out_wr_0_bits_data_0_4; // @[Core.scala 69:23]
  wire [31:0] compute_io_out_wr_0_bits_data_0_5; // @[Core.scala 69:23]
  wire [31:0] compute_io_out_wr_0_bits_data_0_6; // @[Core.scala 69:23]
  wire [31:0] compute_io_out_wr_0_bits_data_0_7; // @[Core.scala 69:23]
  wire [31:0] compute_io_out_wr_0_bits_data_0_8; // @[Core.scala 69:23]
  wire [31:0] compute_io_out_wr_0_bits_data_0_9; // @[Core.scala 69:23]
  wire [31:0] compute_io_out_wr_0_bits_data_0_10; // @[Core.scala 69:23]
  wire [31:0] compute_io_out_wr_0_bits_data_0_11; // @[Core.scala 69:23]
  wire [31:0] compute_io_out_wr_0_bits_data_0_12; // @[Core.scala 69:23]
  wire [31:0] compute_io_out_wr_0_bits_data_0_13; // @[Core.scala 69:23]
  wire [31:0] compute_io_out_wr_0_bits_data_0_14; // @[Core.scala 69:23]
  wire [31:0] compute_io_out_wr_0_bits_data_0_15; // @[Core.scala 69:23]
  wire  compute_io_finish; // @[Core.scala 69:23]
  wire  compute_io_acc_wr_event; // @[Core.scala 69:23]
  wire  store_clock; // @[Core.scala 70:21]
  wire  store_reset; // @[Core.scala 70:21]
  wire  store_io_i_post; // @[Core.scala 70:21]
  wire  store_io_o_post; // @[Core.scala 70:21]
  wire  store_io_inst_ready; // @[Core.scala 70:21]
  wire  store_io_inst_valid; // @[Core.scala 70:21]
  wire [127:0] store_io_inst_bits; // @[Core.scala 70:21]
  wire [31:0] store_io_out_baddr; // @[Core.scala 70:21]
  wire  store_io_vme_wr_cmd_ready; // @[Core.scala 70:21]
  wire  store_io_vme_wr_cmd_valid; // @[Core.scala 70:21]
  wire [31:0] store_io_vme_wr_cmd_bits_addr; // @[Core.scala 70:21]
  wire [7:0] store_io_vme_wr_cmd_bits_len; // @[Core.scala 70:21]
  wire  store_io_vme_wr_data_ready; // @[Core.scala 70:21]
  wire  store_io_vme_wr_data_valid; // @[Core.scala 70:21]
  wire [63:0] store_io_vme_wr_data_bits_data; // @[Core.scala 70:21]
  wire  store_io_vme_wr_ack; // @[Core.scala 70:21]
  wire  store_io_out_wr_0_valid; // @[Core.scala 70:21]
  wire [10:0] store_io_out_wr_0_bits_idx; // @[Core.scala 70:21]
  wire [31:0] store_io_out_wr_0_bits_data_0_0; // @[Core.scala 70:21]
  wire [31:0] store_io_out_wr_0_bits_data_0_1; // @[Core.scala 70:21]
  wire [31:0] store_io_out_wr_0_bits_data_0_2; // @[Core.scala 70:21]
  wire [31:0] store_io_out_wr_0_bits_data_0_3; // @[Core.scala 70:21]
  wire [31:0] store_io_out_wr_0_bits_data_0_4; // @[Core.scala 70:21]
  wire [31:0] store_io_out_wr_0_bits_data_0_5; // @[Core.scala 70:21]
  wire [31:0] store_io_out_wr_0_bits_data_0_6; // @[Core.scala 70:21]
  wire [31:0] store_io_out_wr_0_bits_data_0_7; // @[Core.scala 70:21]
  wire [31:0] store_io_out_wr_0_bits_data_0_8; // @[Core.scala 70:21]
  wire [31:0] store_io_out_wr_0_bits_data_0_9; // @[Core.scala 70:21]
  wire [31:0] store_io_out_wr_0_bits_data_0_10; // @[Core.scala 70:21]
  wire [31:0] store_io_out_wr_0_bits_data_0_11; // @[Core.scala 70:21]
  wire [31:0] store_io_out_wr_0_bits_data_0_12; // @[Core.scala 70:21]
  wire [31:0] store_io_out_wr_0_bits_data_0_13; // @[Core.scala 70:21]
  wire [31:0] store_io_out_wr_0_bits_data_0_14; // @[Core.scala 70:21]
  wire [31:0] store_io_out_wr_0_bits_data_0_15; // @[Core.scala 70:21]
  wire  ecounters_clock; // @[Core.scala 71:25]
  wire  ecounters_reset; // @[Core.scala 71:25]
  wire  ecounters_io_launch; // @[Core.scala 71:25]
  wire  ecounters_io_finish; // @[Core.scala 71:25]
  wire  ecounters_io_ecnt_0_valid; // @[Core.scala 71:25]
  wire [31:0] ecounters_io_ecnt_0_bits; // @[Core.scala 71:25]
  wire  ecounters_io_ucnt_0_valid; // @[Core.scala 71:25]
  wire [31:0] ecounters_io_ucnt_0_bits; // @[Core.scala 71:25]
  wire  ecounters_io_acc_wr_event; // @[Core.scala 71:25]
  reg  finish; // @[Core.scala 119:23]
  Fetch fetch ( // @[Core.scala 67:21]
    .clock(fetch_clock),
    .reset(fetch_reset),
    .io_launch(fetch_io_launch),
    .io_ins_baddr(fetch_io_ins_baddr),
    .io_ins_count(fetch_io_ins_count),
    .io_vme_rd_cmd_ready(fetch_io_vme_rd_cmd_ready),
    .io_vme_rd_cmd_valid(fetch_io_vme_rd_cmd_valid),
    .io_vme_rd_cmd_bits_addr(fetch_io_vme_rd_cmd_bits_addr),
    .io_vme_rd_cmd_bits_len(fetch_io_vme_rd_cmd_bits_len),
    .io_vme_rd_data_ready(fetch_io_vme_rd_data_ready),
    .io_vme_rd_data_valid(fetch_io_vme_rd_data_valid),
    .io_vme_rd_data_bits_data(fetch_io_vme_rd_data_bits_data),
    .io_inst_ld_ready(fetch_io_inst_ld_ready),
    .io_inst_ld_valid(fetch_io_inst_ld_valid),
    .io_inst_ld_bits(fetch_io_inst_ld_bits),
    .io_inst_co_ready(fetch_io_inst_co_ready),
    .io_inst_co_valid(fetch_io_inst_co_valid),
    .io_inst_co_bits(fetch_io_inst_co_bits),
    .io_inst_st_ready(fetch_io_inst_st_ready),
    .io_inst_st_valid(fetch_io_inst_st_valid),
    .io_inst_st_bits(fetch_io_inst_st_bits)
  );
  Load load ( // @[Core.scala 68:20]
    .clock(load_clock),
    .reset(load_reset),
    .io_i_post(load_io_i_post),
    .io_o_post(load_io_o_post),
    .io_inst_ready(load_io_inst_ready),
    .io_inst_valid(load_io_inst_valid),
    .io_inst_bits(load_io_inst_bits),
    .io_inp_baddr(load_io_inp_baddr),
    .io_wgt_baddr(load_io_wgt_baddr),
    .io_vme_rd_0_cmd_ready(load_io_vme_rd_0_cmd_ready),
    .io_vme_rd_0_cmd_valid(load_io_vme_rd_0_cmd_valid),
    .io_vme_rd_0_cmd_bits_addr(load_io_vme_rd_0_cmd_bits_addr),
    .io_vme_rd_0_cmd_bits_len(load_io_vme_rd_0_cmd_bits_len),
    .io_vme_rd_0_cmd_bits_tag(load_io_vme_rd_0_cmd_bits_tag),
    .io_vme_rd_0_data_valid(load_io_vme_rd_0_data_valid),
    .io_vme_rd_0_data_bits_data(load_io_vme_rd_0_data_bits_data),
    .io_vme_rd_0_data_bits_tag(load_io_vme_rd_0_data_bits_tag),
    .io_vme_rd_1_cmd_ready(load_io_vme_rd_1_cmd_ready),
    .io_vme_rd_1_cmd_valid(load_io_vme_rd_1_cmd_valid),
    .io_vme_rd_1_cmd_bits_addr(load_io_vme_rd_1_cmd_bits_addr),
    .io_vme_rd_1_cmd_bits_len(load_io_vme_rd_1_cmd_bits_len),
    .io_vme_rd_1_cmd_bits_tag(load_io_vme_rd_1_cmd_bits_tag),
    .io_vme_rd_1_data_valid(load_io_vme_rd_1_data_valid),
    .io_vme_rd_1_data_bits_data(load_io_vme_rd_1_data_bits_data),
    .io_vme_rd_1_data_bits_tag(load_io_vme_rd_1_data_bits_tag),
    .io_inp_rd_0_idx_valid(load_io_inp_rd_0_idx_valid),
    .io_inp_rd_0_idx_bits(load_io_inp_rd_0_idx_bits),
    .io_inp_rd_0_data_valid(load_io_inp_rd_0_data_valid),
    .io_inp_rd_0_data_bits_0_0(load_io_inp_rd_0_data_bits_0_0),
    .io_inp_rd_0_data_bits_0_1(load_io_inp_rd_0_data_bits_0_1),
    .io_inp_rd_0_data_bits_0_2(load_io_inp_rd_0_data_bits_0_2),
    .io_inp_rd_0_data_bits_0_3(load_io_inp_rd_0_data_bits_0_3),
    .io_inp_rd_0_data_bits_0_4(load_io_inp_rd_0_data_bits_0_4),
    .io_inp_rd_0_data_bits_0_5(load_io_inp_rd_0_data_bits_0_5),
    .io_inp_rd_0_data_bits_0_6(load_io_inp_rd_0_data_bits_0_6),
    .io_inp_rd_0_data_bits_0_7(load_io_inp_rd_0_data_bits_0_7),
    .io_inp_rd_0_data_bits_0_8(load_io_inp_rd_0_data_bits_0_8),
    .io_inp_rd_0_data_bits_0_9(load_io_inp_rd_0_data_bits_0_9),
    .io_inp_rd_0_data_bits_0_10(load_io_inp_rd_0_data_bits_0_10),
    .io_inp_rd_0_data_bits_0_11(load_io_inp_rd_0_data_bits_0_11),
    .io_inp_rd_0_data_bits_0_12(load_io_inp_rd_0_data_bits_0_12),
    .io_inp_rd_0_data_bits_0_13(load_io_inp_rd_0_data_bits_0_13),
    .io_inp_rd_0_data_bits_0_14(load_io_inp_rd_0_data_bits_0_14),
    .io_inp_rd_0_data_bits_0_15(load_io_inp_rd_0_data_bits_0_15),
    .io_wgt_rd_0_idx_valid(load_io_wgt_rd_0_idx_valid),
    .io_wgt_rd_0_idx_bits(load_io_wgt_rd_0_idx_bits),
    .io_wgt_rd_0_data_valid(load_io_wgt_rd_0_data_valid),
    .io_wgt_rd_0_data_bits_0_0(load_io_wgt_rd_0_data_bits_0_0),
    .io_wgt_rd_0_data_bits_0_1(load_io_wgt_rd_0_data_bits_0_1),
    .io_wgt_rd_0_data_bits_0_2(load_io_wgt_rd_0_data_bits_0_2),
    .io_wgt_rd_0_data_bits_0_3(load_io_wgt_rd_0_data_bits_0_3),
    .io_wgt_rd_0_data_bits_0_4(load_io_wgt_rd_0_data_bits_0_4),
    .io_wgt_rd_0_data_bits_0_5(load_io_wgt_rd_0_data_bits_0_5),
    .io_wgt_rd_0_data_bits_0_6(load_io_wgt_rd_0_data_bits_0_6),
    .io_wgt_rd_0_data_bits_0_7(load_io_wgt_rd_0_data_bits_0_7),
    .io_wgt_rd_0_data_bits_0_8(load_io_wgt_rd_0_data_bits_0_8),
    .io_wgt_rd_0_data_bits_0_9(load_io_wgt_rd_0_data_bits_0_9),
    .io_wgt_rd_0_data_bits_0_10(load_io_wgt_rd_0_data_bits_0_10),
    .io_wgt_rd_0_data_bits_0_11(load_io_wgt_rd_0_data_bits_0_11),
    .io_wgt_rd_0_data_bits_0_12(load_io_wgt_rd_0_data_bits_0_12),
    .io_wgt_rd_0_data_bits_0_13(load_io_wgt_rd_0_data_bits_0_13),
    .io_wgt_rd_0_data_bits_0_14(load_io_wgt_rd_0_data_bits_0_14),
    .io_wgt_rd_0_data_bits_0_15(load_io_wgt_rd_0_data_bits_0_15),
    .io_wgt_rd_0_data_bits_1_0(load_io_wgt_rd_0_data_bits_1_0),
    .io_wgt_rd_0_data_bits_1_1(load_io_wgt_rd_0_data_bits_1_1),
    .io_wgt_rd_0_data_bits_1_2(load_io_wgt_rd_0_data_bits_1_2),
    .io_wgt_rd_0_data_bits_1_3(load_io_wgt_rd_0_data_bits_1_3),
    .io_wgt_rd_0_data_bits_1_4(load_io_wgt_rd_0_data_bits_1_4),
    .io_wgt_rd_0_data_bits_1_5(load_io_wgt_rd_0_data_bits_1_5),
    .io_wgt_rd_0_data_bits_1_6(load_io_wgt_rd_0_data_bits_1_6),
    .io_wgt_rd_0_data_bits_1_7(load_io_wgt_rd_0_data_bits_1_7),
    .io_wgt_rd_0_data_bits_1_8(load_io_wgt_rd_0_data_bits_1_8),
    .io_wgt_rd_0_data_bits_1_9(load_io_wgt_rd_0_data_bits_1_9),
    .io_wgt_rd_0_data_bits_1_10(load_io_wgt_rd_0_data_bits_1_10),
    .io_wgt_rd_0_data_bits_1_11(load_io_wgt_rd_0_data_bits_1_11),
    .io_wgt_rd_0_data_bits_1_12(load_io_wgt_rd_0_data_bits_1_12),
    .io_wgt_rd_0_data_bits_1_13(load_io_wgt_rd_0_data_bits_1_13),
    .io_wgt_rd_0_data_bits_1_14(load_io_wgt_rd_0_data_bits_1_14),
    .io_wgt_rd_0_data_bits_1_15(load_io_wgt_rd_0_data_bits_1_15),
    .io_wgt_rd_0_data_bits_2_0(load_io_wgt_rd_0_data_bits_2_0),
    .io_wgt_rd_0_data_bits_2_1(load_io_wgt_rd_0_data_bits_2_1),
    .io_wgt_rd_0_data_bits_2_2(load_io_wgt_rd_0_data_bits_2_2),
    .io_wgt_rd_0_data_bits_2_3(load_io_wgt_rd_0_data_bits_2_3),
    .io_wgt_rd_0_data_bits_2_4(load_io_wgt_rd_0_data_bits_2_4),
    .io_wgt_rd_0_data_bits_2_5(load_io_wgt_rd_0_data_bits_2_5),
    .io_wgt_rd_0_data_bits_2_6(load_io_wgt_rd_0_data_bits_2_6),
    .io_wgt_rd_0_data_bits_2_7(load_io_wgt_rd_0_data_bits_2_7),
    .io_wgt_rd_0_data_bits_2_8(load_io_wgt_rd_0_data_bits_2_8),
    .io_wgt_rd_0_data_bits_2_9(load_io_wgt_rd_0_data_bits_2_9),
    .io_wgt_rd_0_data_bits_2_10(load_io_wgt_rd_0_data_bits_2_10),
    .io_wgt_rd_0_data_bits_2_11(load_io_wgt_rd_0_data_bits_2_11),
    .io_wgt_rd_0_data_bits_2_12(load_io_wgt_rd_0_data_bits_2_12),
    .io_wgt_rd_0_data_bits_2_13(load_io_wgt_rd_0_data_bits_2_13),
    .io_wgt_rd_0_data_bits_2_14(load_io_wgt_rd_0_data_bits_2_14),
    .io_wgt_rd_0_data_bits_2_15(load_io_wgt_rd_0_data_bits_2_15),
    .io_wgt_rd_0_data_bits_3_0(load_io_wgt_rd_0_data_bits_3_0),
    .io_wgt_rd_0_data_bits_3_1(load_io_wgt_rd_0_data_bits_3_1),
    .io_wgt_rd_0_data_bits_3_2(load_io_wgt_rd_0_data_bits_3_2),
    .io_wgt_rd_0_data_bits_3_3(load_io_wgt_rd_0_data_bits_3_3),
    .io_wgt_rd_0_data_bits_3_4(load_io_wgt_rd_0_data_bits_3_4),
    .io_wgt_rd_0_data_bits_3_5(load_io_wgt_rd_0_data_bits_3_5),
    .io_wgt_rd_0_data_bits_3_6(load_io_wgt_rd_0_data_bits_3_6),
    .io_wgt_rd_0_data_bits_3_7(load_io_wgt_rd_0_data_bits_3_7),
    .io_wgt_rd_0_data_bits_3_8(load_io_wgt_rd_0_data_bits_3_8),
    .io_wgt_rd_0_data_bits_3_9(load_io_wgt_rd_0_data_bits_3_9),
    .io_wgt_rd_0_data_bits_3_10(load_io_wgt_rd_0_data_bits_3_10),
    .io_wgt_rd_0_data_bits_3_11(load_io_wgt_rd_0_data_bits_3_11),
    .io_wgt_rd_0_data_bits_3_12(load_io_wgt_rd_0_data_bits_3_12),
    .io_wgt_rd_0_data_bits_3_13(load_io_wgt_rd_0_data_bits_3_13),
    .io_wgt_rd_0_data_bits_3_14(load_io_wgt_rd_0_data_bits_3_14),
    .io_wgt_rd_0_data_bits_3_15(load_io_wgt_rd_0_data_bits_3_15),
    .io_wgt_rd_0_data_bits_4_0(load_io_wgt_rd_0_data_bits_4_0),
    .io_wgt_rd_0_data_bits_4_1(load_io_wgt_rd_0_data_bits_4_1),
    .io_wgt_rd_0_data_bits_4_2(load_io_wgt_rd_0_data_bits_4_2),
    .io_wgt_rd_0_data_bits_4_3(load_io_wgt_rd_0_data_bits_4_3),
    .io_wgt_rd_0_data_bits_4_4(load_io_wgt_rd_0_data_bits_4_4),
    .io_wgt_rd_0_data_bits_4_5(load_io_wgt_rd_0_data_bits_4_5),
    .io_wgt_rd_0_data_bits_4_6(load_io_wgt_rd_0_data_bits_4_6),
    .io_wgt_rd_0_data_bits_4_7(load_io_wgt_rd_0_data_bits_4_7),
    .io_wgt_rd_0_data_bits_4_8(load_io_wgt_rd_0_data_bits_4_8),
    .io_wgt_rd_0_data_bits_4_9(load_io_wgt_rd_0_data_bits_4_9),
    .io_wgt_rd_0_data_bits_4_10(load_io_wgt_rd_0_data_bits_4_10),
    .io_wgt_rd_0_data_bits_4_11(load_io_wgt_rd_0_data_bits_4_11),
    .io_wgt_rd_0_data_bits_4_12(load_io_wgt_rd_0_data_bits_4_12),
    .io_wgt_rd_0_data_bits_4_13(load_io_wgt_rd_0_data_bits_4_13),
    .io_wgt_rd_0_data_bits_4_14(load_io_wgt_rd_0_data_bits_4_14),
    .io_wgt_rd_0_data_bits_4_15(load_io_wgt_rd_0_data_bits_4_15),
    .io_wgt_rd_0_data_bits_5_0(load_io_wgt_rd_0_data_bits_5_0),
    .io_wgt_rd_0_data_bits_5_1(load_io_wgt_rd_0_data_bits_5_1),
    .io_wgt_rd_0_data_bits_5_2(load_io_wgt_rd_0_data_bits_5_2),
    .io_wgt_rd_0_data_bits_5_3(load_io_wgt_rd_0_data_bits_5_3),
    .io_wgt_rd_0_data_bits_5_4(load_io_wgt_rd_0_data_bits_5_4),
    .io_wgt_rd_0_data_bits_5_5(load_io_wgt_rd_0_data_bits_5_5),
    .io_wgt_rd_0_data_bits_5_6(load_io_wgt_rd_0_data_bits_5_6),
    .io_wgt_rd_0_data_bits_5_7(load_io_wgt_rd_0_data_bits_5_7),
    .io_wgt_rd_0_data_bits_5_8(load_io_wgt_rd_0_data_bits_5_8),
    .io_wgt_rd_0_data_bits_5_9(load_io_wgt_rd_0_data_bits_5_9),
    .io_wgt_rd_0_data_bits_5_10(load_io_wgt_rd_0_data_bits_5_10),
    .io_wgt_rd_0_data_bits_5_11(load_io_wgt_rd_0_data_bits_5_11),
    .io_wgt_rd_0_data_bits_5_12(load_io_wgt_rd_0_data_bits_5_12),
    .io_wgt_rd_0_data_bits_5_13(load_io_wgt_rd_0_data_bits_5_13),
    .io_wgt_rd_0_data_bits_5_14(load_io_wgt_rd_0_data_bits_5_14),
    .io_wgt_rd_0_data_bits_5_15(load_io_wgt_rd_0_data_bits_5_15),
    .io_wgt_rd_0_data_bits_6_0(load_io_wgt_rd_0_data_bits_6_0),
    .io_wgt_rd_0_data_bits_6_1(load_io_wgt_rd_0_data_bits_6_1),
    .io_wgt_rd_0_data_bits_6_2(load_io_wgt_rd_0_data_bits_6_2),
    .io_wgt_rd_0_data_bits_6_3(load_io_wgt_rd_0_data_bits_6_3),
    .io_wgt_rd_0_data_bits_6_4(load_io_wgt_rd_0_data_bits_6_4),
    .io_wgt_rd_0_data_bits_6_5(load_io_wgt_rd_0_data_bits_6_5),
    .io_wgt_rd_0_data_bits_6_6(load_io_wgt_rd_0_data_bits_6_6),
    .io_wgt_rd_0_data_bits_6_7(load_io_wgt_rd_0_data_bits_6_7),
    .io_wgt_rd_0_data_bits_6_8(load_io_wgt_rd_0_data_bits_6_8),
    .io_wgt_rd_0_data_bits_6_9(load_io_wgt_rd_0_data_bits_6_9),
    .io_wgt_rd_0_data_bits_6_10(load_io_wgt_rd_0_data_bits_6_10),
    .io_wgt_rd_0_data_bits_6_11(load_io_wgt_rd_0_data_bits_6_11),
    .io_wgt_rd_0_data_bits_6_12(load_io_wgt_rd_0_data_bits_6_12),
    .io_wgt_rd_0_data_bits_6_13(load_io_wgt_rd_0_data_bits_6_13),
    .io_wgt_rd_0_data_bits_6_14(load_io_wgt_rd_0_data_bits_6_14),
    .io_wgt_rd_0_data_bits_6_15(load_io_wgt_rd_0_data_bits_6_15),
    .io_wgt_rd_0_data_bits_7_0(load_io_wgt_rd_0_data_bits_7_0),
    .io_wgt_rd_0_data_bits_7_1(load_io_wgt_rd_0_data_bits_7_1),
    .io_wgt_rd_0_data_bits_7_2(load_io_wgt_rd_0_data_bits_7_2),
    .io_wgt_rd_0_data_bits_7_3(load_io_wgt_rd_0_data_bits_7_3),
    .io_wgt_rd_0_data_bits_7_4(load_io_wgt_rd_0_data_bits_7_4),
    .io_wgt_rd_0_data_bits_7_5(load_io_wgt_rd_0_data_bits_7_5),
    .io_wgt_rd_0_data_bits_7_6(load_io_wgt_rd_0_data_bits_7_6),
    .io_wgt_rd_0_data_bits_7_7(load_io_wgt_rd_0_data_bits_7_7),
    .io_wgt_rd_0_data_bits_7_8(load_io_wgt_rd_0_data_bits_7_8),
    .io_wgt_rd_0_data_bits_7_9(load_io_wgt_rd_0_data_bits_7_9),
    .io_wgt_rd_0_data_bits_7_10(load_io_wgt_rd_0_data_bits_7_10),
    .io_wgt_rd_0_data_bits_7_11(load_io_wgt_rd_0_data_bits_7_11),
    .io_wgt_rd_0_data_bits_7_12(load_io_wgt_rd_0_data_bits_7_12),
    .io_wgt_rd_0_data_bits_7_13(load_io_wgt_rd_0_data_bits_7_13),
    .io_wgt_rd_0_data_bits_7_14(load_io_wgt_rd_0_data_bits_7_14),
    .io_wgt_rd_0_data_bits_7_15(load_io_wgt_rd_0_data_bits_7_15),
    .io_wgt_rd_0_data_bits_8_0(load_io_wgt_rd_0_data_bits_8_0),
    .io_wgt_rd_0_data_bits_8_1(load_io_wgt_rd_0_data_bits_8_1),
    .io_wgt_rd_0_data_bits_8_2(load_io_wgt_rd_0_data_bits_8_2),
    .io_wgt_rd_0_data_bits_8_3(load_io_wgt_rd_0_data_bits_8_3),
    .io_wgt_rd_0_data_bits_8_4(load_io_wgt_rd_0_data_bits_8_4),
    .io_wgt_rd_0_data_bits_8_5(load_io_wgt_rd_0_data_bits_8_5),
    .io_wgt_rd_0_data_bits_8_6(load_io_wgt_rd_0_data_bits_8_6),
    .io_wgt_rd_0_data_bits_8_7(load_io_wgt_rd_0_data_bits_8_7),
    .io_wgt_rd_0_data_bits_8_8(load_io_wgt_rd_0_data_bits_8_8),
    .io_wgt_rd_0_data_bits_8_9(load_io_wgt_rd_0_data_bits_8_9),
    .io_wgt_rd_0_data_bits_8_10(load_io_wgt_rd_0_data_bits_8_10),
    .io_wgt_rd_0_data_bits_8_11(load_io_wgt_rd_0_data_bits_8_11),
    .io_wgt_rd_0_data_bits_8_12(load_io_wgt_rd_0_data_bits_8_12),
    .io_wgt_rd_0_data_bits_8_13(load_io_wgt_rd_0_data_bits_8_13),
    .io_wgt_rd_0_data_bits_8_14(load_io_wgt_rd_0_data_bits_8_14),
    .io_wgt_rd_0_data_bits_8_15(load_io_wgt_rd_0_data_bits_8_15),
    .io_wgt_rd_0_data_bits_9_0(load_io_wgt_rd_0_data_bits_9_0),
    .io_wgt_rd_0_data_bits_9_1(load_io_wgt_rd_0_data_bits_9_1),
    .io_wgt_rd_0_data_bits_9_2(load_io_wgt_rd_0_data_bits_9_2),
    .io_wgt_rd_0_data_bits_9_3(load_io_wgt_rd_0_data_bits_9_3),
    .io_wgt_rd_0_data_bits_9_4(load_io_wgt_rd_0_data_bits_9_4),
    .io_wgt_rd_0_data_bits_9_5(load_io_wgt_rd_0_data_bits_9_5),
    .io_wgt_rd_0_data_bits_9_6(load_io_wgt_rd_0_data_bits_9_6),
    .io_wgt_rd_0_data_bits_9_7(load_io_wgt_rd_0_data_bits_9_7),
    .io_wgt_rd_0_data_bits_9_8(load_io_wgt_rd_0_data_bits_9_8),
    .io_wgt_rd_0_data_bits_9_9(load_io_wgt_rd_0_data_bits_9_9),
    .io_wgt_rd_0_data_bits_9_10(load_io_wgt_rd_0_data_bits_9_10),
    .io_wgt_rd_0_data_bits_9_11(load_io_wgt_rd_0_data_bits_9_11),
    .io_wgt_rd_0_data_bits_9_12(load_io_wgt_rd_0_data_bits_9_12),
    .io_wgt_rd_0_data_bits_9_13(load_io_wgt_rd_0_data_bits_9_13),
    .io_wgt_rd_0_data_bits_9_14(load_io_wgt_rd_0_data_bits_9_14),
    .io_wgt_rd_0_data_bits_9_15(load_io_wgt_rd_0_data_bits_9_15),
    .io_wgt_rd_0_data_bits_10_0(load_io_wgt_rd_0_data_bits_10_0),
    .io_wgt_rd_0_data_bits_10_1(load_io_wgt_rd_0_data_bits_10_1),
    .io_wgt_rd_0_data_bits_10_2(load_io_wgt_rd_0_data_bits_10_2),
    .io_wgt_rd_0_data_bits_10_3(load_io_wgt_rd_0_data_bits_10_3),
    .io_wgt_rd_0_data_bits_10_4(load_io_wgt_rd_0_data_bits_10_4),
    .io_wgt_rd_0_data_bits_10_5(load_io_wgt_rd_0_data_bits_10_5),
    .io_wgt_rd_0_data_bits_10_6(load_io_wgt_rd_0_data_bits_10_6),
    .io_wgt_rd_0_data_bits_10_7(load_io_wgt_rd_0_data_bits_10_7),
    .io_wgt_rd_0_data_bits_10_8(load_io_wgt_rd_0_data_bits_10_8),
    .io_wgt_rd_0_data_bits_10_9(load_io_wgt_rd_0_data_bits_10_9),
    .io_wgt_rd_0_data_bits_10_10(load_io_wgt_rd_0_data_bits_10_10),
    .io_wgt_rd_0_data_bits_10_11(load_io_wgt_rd_0_data_bits_10_11),
    .io_wgt_rd_0_data_bits_10_12(load_io_wgt_rd_0_data_bits_10_12),
    .io_wgt_rd_0_data_bits_10_13(load_io_wgt_rd_0_data_bits_10_13),
    .io_wgt_rd_0_data_bits_10_14(load_io_wgt_rd_0_data_bits_10_14),
    .io_wgt_rd_0_data_bits_10_15(load_io_wgt_rd_0_data_bits_10_15),
    .io_wgt_rd_0_data_bits_11_0(load_io_wgt_rd_0_data_bits_11_0),
    .io_wgt_rd_0_data_bits_11_1(load_io_wgt_rd_0_data_bits_11_1),
    .io_wgt_rd_0_data_bits_11_2(load_io_wgt_rd_0_data_bits_11_2),
    .io_wgt_rd_0_data_bits_11_3(load_io_wgt_rd_0_data_bits_11_3),
    .io_wgt_rd_0_data_bits_11_4(load_io_wgt_rd_0_data_bits_11_4),
    .io_wgt_rd_0_data_bits_11_5(load_io_wgt_rd_0_data_bits_11_5),
    .io_wgt_rd_0_data_bits_11_6(load_io_wgt_rd_0_data_bits_11_6),
    .io_wgt_rd_0_data_bits_11_7(load_io_wgt_rd_0_data_bits_11_7),
    .io_wgt_rd_0_data_bits_11_8(load_io_wgt_rd_0_data_bits_11_8),
    .io_wgt_rd_0_data_bits_11_9(load_io_wgt_rd_0_data_bits_11_9),
    .io_wgt_rd_0_data_bits_11_10(load_io_wgt_rd_0_data_bits_11_10),
    .io_wgt_rd_0_data_bits_11_11(load_io_wgt_rd_0_data_bits_11_11),
    .io_wgt_rd_0_data_bits_11_12(load_io_wgt_rd_0_data_bits_11_12),
    .io_wgt_rd_0_data_bits_11_13(load_io_wgt_rd_0_data_bits_11_13),
    .io_wgt_rd_0_data_bits_11_14(load_io_wgt_rd_0_data_bits_11_14),
    .io_wgt_rd_0_data_bits_11_15(load_io_wgt_rd_0_data_bits_11_15),
    .io_wgt_rd_0_data_bits_12_0(load_io_wgt_rd_0_data_bits_12_0),
    .io_wgt_rd_0_data_bits_12_1(load_io_wgt_rd_0_data_bits_12_1),
    .io_wgt_rd_0_data_bits_12_2(load_io_wgt_rd_0_data_bits_12_2),
    .io_wgt_rd_0_data_bits_12_3(load_io_wgt_rd_0_data_bits_12_3),
    .io_wgt_rd_0_data_bits_12_4(load_io_wgt_rd_0_data_bits_12_4),
    .io_wgt_rd_0_data_bits_12_5(load_io_wgt_rd_0_data_bits_12_5),
    .io_wgt_rd_0_data_bits_12_6(load_io_wgt_rd_0_data_bits_12_6),
    .io_wgt_rd_0_data_bits_12_7(load_io_wgt_rd_0_data_bits_12_7),
    .io_wgt_rd_0_data_bits_12_8(load_io_wgt_rd_0_data_bits_12_8),
    .io_wgt_rd_0_data_bits_12_9(load_io_wgt_rd_0_data_bits_12_9),
    .io_wgt_rd_0_data_bits_12_10(load_io_wgt_rd_0_data_bits_12_10),
    .io_wgt_rd_0_data_bits_12_11(load_io_wgt_rd_0_data_bits_12_11),
    .io_wgt_rd_0_data_bits_12_12(load_io_wgt_rd_0_data_bits_12_12),
    .io_wgt_rd_0_data_bits_12_13(load_io_wgt_rd_0_data_bits_12_13),
    .io_wgt_rd_0_data_bits_12_14(load_io_wgt_rd_0_data_bits_12_14),
    .io_wgt_rd_0_data_bits_12_15(load_io_wgt_rd_0_data_bits_12_15),
    .io_wgt_rd_0_data_bits_13_0(load_io_wgt_rd_0_data_bits_13_0),
    .io_wgt_rd_0_data_bits_13_1(load_io_wgt_rd_0_data_bits_13_1),
    .io_wgt_rd_0_data_bits_13_2(load_io_wgt_rd_0_data_bits_13_2),
    .io_wgt_rd_0_data_bits_13_3(load_io_wgt_rd_0_data_bits_13_3),
    .io_wgt_rd_0_data_bits_13_4(load_io_wgt_rd_0_data_bits_13_4),
    .io_wgt_rd_0_data_bits_13_5(load_io_wgt_rd_0_data_bits_13_5),
    .io_wgt_rd_0_data_bits_13_6(load_io_wgt_rd_0_data_bits_13_6),
    .io_wgt_rd_0_data_bits_13_7(load_io_wgt_rd_0_data_bits_13_7),
    .io_wgt_rd_0_data_bits_13_8(load_io_wgt_rd_0_data_bits_13_8),
    .io_wgt_rd_0_data_bits_13_9(load_io_wgt_rd_0_data_bits_13_9),
    .io_wgt_rd_0_data_bits_13_10(load_io_wgt_rd_0_data_bits_13_10),
    .io_wgt_rd_0_data_bits_13_11(load_io_wgt_rd_0_data_bits_13_11),
    .io_wgt_rd_0_data_bits_13_12(load_io_wgt_rd_0_data_bits_13_12),
    .io_wgt_rd_0_data_bits_13_13(load_io_wgt_rd_0_data_bits_13_13),
    .io_wgt_rd_0_data_bits_13_14(load_io_wgt_rd_0_data_bits_13_14),
    .io_wgt_rd_0_data_bits_13_15(load_io_wgt_rd_0_data_bits_13_15),
    .io_wgt_rd_0_data_bits_14_0(load_io_wgt_rd_0_data_bits_14_0),
    .io_wgt_rd_0_data_bits_14_1(load_io_wgt_rd_0_data_bits_14_1),
    .io_wgt_rd_0_data_bits_14_2(load_io_wgt_rd_0_data_bits_14_2),
    .io_wgt_rd_0_data_bits_14_3(load_io_wgt_rd_0_data_bits_14_3),
    .io_wgt_rd_0_data_bits_14_4(load_io_wgt_rd_0_data_bits_14_4),
    .io_wgt_rd_0_data_bits_14_5(load_io_wgt_rd_0_data_bits_14_5),
    .io_wgt_rd_0_data_bits_14_6(load_io_wgt_rd_0_data_bits_14_6),
    .io_wgt_rd_0_data_bits_14_7(load_io_wgt_rd_0_data_bits_14_7),
    .io_wgt_rd_0_data_bits_14_8(load_io_wgt_rd_0_data_bits_14_8),
    .io_wgt_rd_0_data_bits_14_9(load_io_wgt_rd_0_data_bits_14_9),
    .io_wgt_rd_0_data_bits_14_10(load_io_wgt_rd_0_data_bits_14_10),
    .io_wgt_rd_0_data_bits_14_11(load_io_wgt_rd_0_data_bits_14_11),
    .io_wgt_rd_0_data_bits_14_12(load_io_wgt_rd_0_data_bits_14_12),
    .io_wgt_rd_0_data_bits_14_13(load_io_wgt_rd_0_data_bits_14_13),
    .io_wgt_rd_0_data_bits_14_14(load_io_wgt_rd_0_data_bits_14_14),
    .io_wgt_rd_0_data_bits_14_15(load_io_wgt_rd_0_data_bits_14_15),
    .io_wgt_rd_0_data_bits_15_0(load_io_wgt_rd_0_data_bits_15_0),
    .io_wgt_rd_0_data_bits_15_1(load_io_wgt_rd_0_data_bits_15_1),
    .io_wgt_rd_0_data_bits_15_2(load_io_wgt_rd_0_data_bits_15_2),
    .io_wgt_rd_0_data_bits_15_3(load_io_wgt_rd_0_data_bits_15_3),
    .io_wgt_rd_0_data_bits_15_4(load_io_wgt_rd_0_data_bits_15_4),
    .io_wgt_rd_0_data_bits_15_5(load_io_wgt_rd_0_data_bits_15_5),
    .io_wgt_rd_0_data_bits_15_6(load_io_wgt_rd_0_data_bits_15_6),
    .io_wgt_rd_0_data_bits_15_7(load_io_wgt_rd_0_data_bits_15_7),
    .io_wgt_rd_0_data_bits_15_8(load_io_wgt_rd_0_data_bits_15_8),
    .io_wgt_rd_0_data_bits_15_9(load_io_wgt_rd_0_data_bits_15_9),
    .io_wgt_rd_0_data_bits_15_10(load_io_wgt_rd_0_data_bits_15_10),
    .io_wgt_rd_0_data_bits_15_11(load_io_wgt_rd_0_data_bits_15_11),
    .io_wgt_rd_0_data_bits_15_12(load_io_wgt_rd_0_data_bits_15_12),
    .io_wgt_rd_0_data_bits_15_13(load_io_wgt_rd_0_data_bits_15_13),
    .io_wgt_rd_0_data_bits_15_14(load_io_wgt_rd_0_data_bits_15_14),
    .io_wgt_rd_0_data_bits_15_15(load_io_wgt_rd_0_data_bits_15_15)
  );
  Compute compute ( // @[Core.scala 69:23]
    .clock(compute_clock),
    .reset(compute_reset),
    .io_i_post_0(compute_io_i_post_0),
    .io_i_post_1(compute_io_i_post_1),
    .io_o_post_0(compute_io_o_post_0),
    .io_o_post_1(compute_io_o_post_1),
    .io_inst_ready(compute_io_inst_ready),
    .io_inst_valid(compute_io_inst_valid),
    .io_inst_bits(compute_io_inst_bits),
    .io_uop_baddr(compute_io_uop_baddr),
    .io_acc_baddr(compute_io_acc_baddr),
    .io_vme_rd_0_cmd_ready(compute_io_vme_rd_0_cmd_ready),
    .io_vme_rd_0_cmd_valid(compute_io_vme_rd_0_cmd_valid),
    .io_vme_rd_0_cmd_bits_addr(compute_io_vme_rd_0_cmd_bits_addr),
    .io_vme_rd_0_cmd_bits_len(compute_io_vme_rd_0_cmd_bits_len),
    .io_vme_rd_0_cmd_bits_tag(compute_io_vme_rd_0_cmd_bits_tag),
    .io_vme_rd_0_data_valid(compute_io_vme_rd_0_data_valid),
    .io_vme_rd_0_data_bits_data(compute_io_vme_rd_0_data_bits_data),
    .io_vme_rd_0_data_bits_tag(compute_io_vme_rd_0_data_bits_tag),
    .io_vme_rd_0_data_bits_last(compute_io_vme_rd_0_data_bits_last),
    .io_vme_rd_1_cmd_ready(compute_io_vme_rd_1_cmd_ready),
    .io_vme_rd_1_cmd_valid(compute_io_vme_rd_1_cmd_valid),
    .io_vme_rd_1_cmd_bits_addr(compute_io_vme_rd_1_cmd_bits_addr),
    .io_vme_rd_1_cmd_bits_len(compute_io_vme_rd_1_cmd_bits_len),
    .io_vme_rd_1_cmd_bits_tag(compute_io_vme_rd_1_cmd_bits_tag),
    .io_vme_rd_1_data_valid(compute_io_vme_rd_1_data_valid),
    .io_vme_rd_1_data_bits_data(compute_io_vme_rd_1_data_bits_data),
    .io_vme_rd_1_data_bits_tag(compute_io_vme_rd_1_data_bits_tag),
    .io_inp_rd_0_idx_valid(compute_io_inp_rd_0_idx_valid),
    .io_inp_rd_0_idx_bits(compute_io_inp_rd_0_idx_bits),
    .io_inp_rd_0_data_valid(compute_io_inp_rd_0_data_valid),
    .io_inp_rd_0_data_bits_0_0(compute_io_inp_rd_0_data_bits_0_0),
    .io_inp_rd_0_data_bits_0_1(compute_io_inp_rd_0_data_bits_0_1),
    .io_inp_rd_0_data_bits_0_2(compute_io_inp_rd_0_data_bits_0_2),
    .io_inp_rd_0_data_bits_0_3(compute_io_inp_rd_0_data_bits_0_3),
    .io_inp_rd_0_data_bits_0_4(compute_io_inp_rd_0_data_bits_0_4),
    .io_inp_rd_0_data_bits_0_5(compute_io_inp_rd_0_data_bits_0_5),
    .io_inp_rd_0_data_bits_0_6(compute_io_inp_rd_0_data_bits_0_6),
    .io_inp_rd_0_data_bits_0_7(compute_io_inp_rd_0_data_bits_0_7),
    .io_inp_rd_0_data_bits_0_8(compute_io_inp_rd_0_data_bits_0_8),
    .io_inp_rd_0_data_bits_0_9(compute_io_inp_rd_0_data_bits_0_9),
    .io_inp_rd_0_data_bits_0_10(compute_io_inp_rd_0_data_bits_0_10),
    .io_inp_rd_0_data_bits_0_11(compute_io_inp_rd_0_data_bits_0_11),
    .io_inp_rd_0_data_bits_0_12(compute_io_inp_rd_0_data_bits_0_12),
    .io_inp_rd_0_data_bits_0_13(compute_io_inp_rd_0_data_bits_0_13),
    .io_inp_rd_0_data_bits_0_14(compute_io_inp_rd_0_data_bits_0_14),
    .io_inp_rd_0_data_bits_0_15(compute_io_inp_rd_0_data_bits_0_15),
    .io_wgt_rd_0_idx_valid(compute_io_wgt_rd_0_idx_valid),
    .io_wgt_rd_0_idx_bits(compute_io_wgt_rd_0_idx_bits),
    .io_wgt_rd_0_data_valid(compute_io_wgt_rd_0_data_valid),
    .io_wgt_rd_0_data_bits_0_0(compute_io_wgt_rd_0_data_bits_0_0),
    .io_wgt_rd_0_data_bits_0_1(compute_io_wgt_rd_0_data_bits_0_1),
    .io_wgt_rd_0_data_bits_0_2(compute_io_wgt_rd_0_data_bits_0_2),
    .io_wgt_rd_0_data_bits_0_3(compute_io_wgt_rd_0_data_bits_0_3),
    .io_wgt_rd_0_data_bits_0_4(compute_io_wgt_rd_0_data_bits_0_4),
    .io_wgt_rd_0_data_bits_0_5(compute_io_wgt_rd_0_data_bits_0_5),
    .io_wgt_rd_0_data_bits_0_6(compute_io_wgt_rd_0_data_bits_0_6),
    .io_wgt_rd_0_data_bits_0_7(compute_io_wgt_rd_0_data_bits_0_7),
    .io_wgt_rd_0_data_bits_0_8(compute_io_wgt_rd_0_data_bits_0_8),
    .io_wgt_rd_0_data_bits_0_9(compute_io_wgt_rd_0_data_bits_0_9),
    .io_wgt_rd_0_data_bits_0_10(compute_io_wgt_rd_0_data_bits_0_10),
    .io_wgt_rd_0_data_bits_0_11(compute_io_wgt_rd_0_data_bits_0_11),
    .io_wgt_rd_0_data_bits_0_12(compute_io_wgt_rd_0_data_bits_0_12),
    .io_wgt_rd_0_data_bits_0_13(compute_io_wgt_rd_0_data_bits_0_13),
    .io_wgt_rd_0_data_bits_0_14(compute_io_wgt_rd_0_data_bits_0_14),
    .io_wgt_rd_0_data_bits_0_15(compute_io_wgt_rd_0_data_bits_0_15),
    .io_wgt_rd_0_data_bits_1_0(compute_io_wgt_rd_0_data_bits_1_0),
    .io_wgt_rd_0_data_bits_1_1(compute_io_wgt_rd_0_data_bits_1_1),
    .io_wgt_rd_0_data_bits_1_2(compute_io_wgt_rd_0_data_bits_1_2),
    .io_wgt_rd_0_data_bits_1_3(compute_io_wgt_rd_0_data_bits_1_3),
    .io_wgt_rd_0_data_bits_1_4(compute_io_wgt_rd_0_data_bits_1_4),
    .io_wgt_rd_0_data_bits_1_5(compute_io_wgt_rd_0_data_bits_1_5),
    .io_wgt_rd_0_data_bits_1_6(compute_io_wgt_rd_0_data_bits_1_6),
    .io_wgt_rd_0_data_bits_1_7(compute_io_wgt_rd_0_data_bits_1_7),
    .io_wgt_rd_0_data_bits_1_8(compute_io_wgt_rd_0_data_bits_1_8),
    .io_wgt_rd_0_data_bits_1_9(compute_io_wgt_rd_0_data_bits_1_9),
    .io_wgt_rd_0_data_bits_1_10(compute_io_wgt_rd_0_data_bits_1_10),
    .io_wgt_rd_0_data_bits_1_11(compute_io_wgt_rd_0_data_bits_1_11),
    .io_wgt_rd_0_data_bits_1_12(compute_io_wgt_rd_0_data_bits_1_12),
    .io_wgt_rd_0_data_bits_1_13(compute_io_wgt_rd_0_data_bits_1_13),
    .io_wgt_rd_0_data_bits_1_14(compute_io_wgt_rd_0_data_bits_1_14),
    .io_wgt_rd_0_data_bits_1_15(compute_io_wgt_rd_0_data_bits_1_15),
    .io_wgt_rd_0_data_bits_2_0(compute_io_wgt_rd_0_data_bits_2_0),
    .io_wgt_rd_0_data_bits_2_1(compute_io_wgt_rd_0_data_bits_2_1),
    .io_wgt_rd_0_data_bits_2_2(compute_io_wgt_rd_0_data_bits_2_2),
    .io_wgt_rd_0_data_bits_2_3(compute_io_wgt_rd_0_data_bits_2_3),
    .io_wgt_rd_0_data_bits_2_4(compute_io_wgt_rd_0_data_bits_2_4),
    .io_wgt_rd_0_data_bits_2_5(compute_io_wgt_rd_0_data_bits_2_5),
    .io_wgt_rd_0_data_bits_2_6(compute_io_wgt_rd_0_data_bits_2_6),
    .io_wgt_rd_0_data_bits_2_7(compute_io_wgt_rd_0_data_bits_2_7),
    .io_wgt_rd_0_data_bits_2_8(compute_io_wgt_rd_0_data_bits_2_8),
    .io_wgt_rd_0_data_bits_2_9(compute_io_wgt_rd_0_data_bits_2_9),
    .io_wgt_rd_0_data_bits_2_10(compute_io_wgt_rd_0_data_bits_2_10),
    .io_wgt_rd_0_data_bits_2_11(compute_io_wgt_rd_0_data_bits_2_11),
    .io_wgt_rd_0_data_bits_2_12(compute_io_wgt_rd_0_data_bits_2_12),
    .io_wgt_rd_0_data_bits_2_13(compute_io_wgt_rd_0_data_bits_2_13),
    .io_wgt_rd_0_data_bits_2_14(compute_io_wgt_rd_0_data_bits_2_14),
    .io_wgt_rd_0_data_bits_2_15(compute_io_wgt_rd_0_data_bits_2_15),
    .io_wgt_rd_0_data_bits_3_0(compute_io_wgt_rd_0_data_bits_3_0),
    .io_wgt_rd_0_data_bits_3_1(compute_io_wgt_rd_0_data_bits_3_1),
    .io_wgt_rd_0_data_bits_3_2(compute_io_wgt_rd_0_data_bits_3_2),
    .io_wgt_rd_0_data_bits_3_3(compute_io_wgt_rd_0_data_bits_3_3),
    .io_wgt_rd_0_data_bits_3_4(compute_io_wgt_rd_0_data_bits_3_4),
    .io_wgt_rd_0_data_bits_3_5(compute_io_wgt_rd_0_data_bits_3_5),
    .io_wgt_rd_0_data_bits_3_6(compute_io_wgt_rd_0_data_bits_3_6),
    .io_wgt_rd_0_data_bits_3_7(compute_io_wgt_rd_0_data_bits_3_7),
    .io_wgt_rd_0_data_bits_3_8(compute_io_wgt_rd_0_data_bits_3_8),
    .io_wgt_rd_0_data_bits_3_9(compute_io_wgt_rd_0_data_bits_3_9),
    .io_wgt_rd_0_data_bits_3_10(compute_io_wgt_rd_0_data_bits_3_10),
    .io_wgt_rd_0_data_bits_3_11(compute_io_wgt_rd_0_data_bits_3_11),
    .io_wgt_rd_0_data_bits_3_12(compute_io_wgt_rd_0_data_bits_3_12),
    .io_wgt_rd_0_data_bits_3_13(compute_io_wgt_rd_0_data_bits_3_13),
    .io_wgt_rd_0_data_bits_3_14(compute_io_wgt_rd_0_data_bits_3_14),
    .io_wgt_rd_0_data_bits_3_15(compute_io_wgt_rd_0_data_bits_3_15),
    .io_wgt_rd_0_data_bits_4_0(compute_io_wgt_rd_0_data_bits_4_0),
    .io_wgt_rd_0_data_bits_4_1(compute_io_wgt_rd_0_data_bits_4_1),
    .io_wgt_rd_0_data_bits_4_2(compute_io_wgt_rd_0_data_bits_4_2),
    .io_wgt_rd_0_data_bits_4_3(compute_io_wgt_rd_0_data_bits_4_3),
    .io_wgt_rd_0_data_bits_4_4(compute_io_wgt_rd_0_data_bits_4_4),
    .io_wgt_rd_0_data_bits_4_5(compute_io_wgt_rd_0_data_bits_4_5),
    .io_wgt_rd_0_data_bits_4_6(compute_io_wgt_rd_0_data_bits_4_6),
    .io_wgt_rd_0_data_bits_4_7(compute_io_wgt_rd_0_data_bits_4_7),
    .io_wgt_rd_0_data_bits_4_8(compute_io_wgt_rd_0_data_bits_4_8),
    .io_wgt_rd_0_data_bits_4_9(compute_io_wgt_rd_0_data_bits_4_9),
    .io_wgt_rd_0_data_bits_4_10(compute_io_wgt_rd_0_data_bits_4_10),
    .io_wgt_rd_0_data_bits_4_11(compute_io_wgt_rd_0_data_bits_4_11),
    .io_wgt_rd_0_data_bits_4_12(compute_io_wgt_rd_0_data_bits_4_12),
    .io_wgt_rd_0_data_bits_4_13(compute_io_wgt_rd_0_data_bits_4_13),
    .io_wgt_rd_0_data_bits_4_14(compute_io_wgt_rd_0_data_bits_4_14),
    .io_wgt_rd_0_data_bits_4_15(compute_io_wgt_rd_0_data_bits_4_15),
    .io_wgt_rd_0_data_bits_5_0(compute_io_wgt_rd_0_data_bits_5_0),
    .io_wgt_rd_0_data_bits_5_1(compute_io_wgt_rd_0_data_bits_5_1),
    .io_wgt_rd_0_data_bits_5_2(compute_io_wgt_rd_0_data_bits_5_2),
    .io_wgt_rd_0_data_bits_5_3(compute_io_wgt_rd_0_data_bits_5_3),
    .io_wgt_rd_0_data_bits_5_4(compute_io_wgt_rd_0_data_bits_5_4),
    .io_wgt_rd_0_data_bits_5_5(compute_io_wgt_rd_0_data_bits_5_5),
    .io_wgt_rd_0_data_bits_5_6(compute_io_wgt_rd_0_data_bits_5_6),
    .io_wgt_rd_0_data_bits_5_7(compute_io_wgt_rd_0_data_bits_5_7),
    .io_wgt_rd_0_data_bits_5_8(compute_io_wgt_rd_0_data_bits_5_8),
    .io_wgt_rd_0_data_bits_5_9(compute_io_wgt_rd_0_data_bits_5_9),
    .io_wgt_rd_0_data_bits_5_10(compute_io_wgt_rd_0_data_bits_5_10),
    .io_wgt_rd_0_data_bits_5_11(compute_io_wgt_rd_0_data_bits_5_11),
    .io_wgt_rd_0_data_bits_5_12(compute_io_wgt_rd_0_data_bits_5_12),
    .io_wgt_rd_0_data_bits_5_13(compute_io_wgt_rd_0_data_bits_5_13),
    .io_wgt_rd_0_data_bits_5_14(compute_io_wgt_rd_0_data_bits_5_14),
    .io_wgt_rd_0_data_bits_5_15(compute_io_wgt_rd_0_data_bits_5_15),
    .io_wgt_rd_0_data_bits_6_0(compute_io_wgt_rd_0_data_bits_6_0),
    .io_wgt_rd_0_data_bits_6_1(compute_io_wgt_rd_0_data_bits_6_1),
    .io_wgt_rd_0_data_bits_6_2(compute_io_wgt_rd_0_data_bits_6_2),
    .io_wgt_rd_0_data_bits_6_3(compute_io_wgt_rd_0_data_bits_6_3),
    .io_wgt_rd_0_data_bits_6_4(compute_io_wgt_rd_0_data_bits_6_4),
    .io_wgt_rd_0_data_bits_6_5(compute_io_wgt_rd_0_data_bits_6_5),
    .io_wgt_rd_0_data_bits_6_6(compute_io_wgt_rd_0_data_bits_6_6),
    .io_wgt_rd_0_data_bits_6_7(compute_io_wgt_rd_0_data_bits_6_7),
    .io_wgt_rd_0_data_bits_6_8(compute_io_wgt_rd_0_data_bits_6_8),
    .io_wgt_rd_0_data_bits_6_9(compute_io_wgt_rd_0_data_bits_6_9),
    .io_wgt_rd_0_data_bits_6_10(compute_io_wgt_rd_0_data_bits_6_10),
    .io_wgt_rd_0_data_bits_6_11(compute_io_wgt_rd_0_data_bits_6_11),
    .io_wgt_rd_0_data_bits_6_12(compute_io_wgt_rd_0_data_bits_6_12),
    .io_wgt_rd_0_data_bits_6_13(compute_io_wgt_rd_0_data_bits_6_13),
    .io_wgt_rd_0_data_bits_6_14(compute_io_wgt_rd_0_data_bits_6_14),
    .io_wgt_rd_0_data_bits_6_15(compute_io_wgt_rd_0_data_bits_6_15),
    .io_wgt_rd_0_data_bits_7_0(compute_io_wgt_rd_0_data_bits_7_0),
    .io_wgt_rd_0_data_bits_7_1(compute_io_wgt_rd_0_data_bits_7_1),
    .io_wgt_rd_0_data_bits_7_2(compute_io_wgt_rd_0_data_bits_7_2),
    .io_wgt_rd_0_data_bits_7_3(compute_io_wgt_rd_0_data_bits_7_3),
    .io_wgt_rd_0_data_bits_7_4(compute_io_wgt_rd_0_data_bits_7_4),
    .io_wgt_rd_0_data_bits_7_5(compute_io_wgt_rd_0_data_bits_7_5),
    .io_wgt_rd_0_data_bits_7_6(compute_io_wgt_rd_0_data_bits_7_6),
    .io_wgt_rd_0_data_bits_7_7(compute_io_wgt_rd_0_data_bits_7_7),
    .io_wgt_rd_0_data_bits_7_8(compute_io_wgt_rd_0_data_bits_7_8),
    .io_wgt_rd_0_data_bits_7_9(compute_io_wgt_rd_0_data_bits_7_9),
    .io_wgt_rd_0_data_bits_7_10(compute_io_wgt_rd_0_data_bits_7_10),
    .io_wgt_rd_0_data_bits_7_11(compute_io_wgt_rd_0_data_bits_7_11),
    .io_wgt_rd_0_data_bits_7_12(compute_io_wgt_rd_0_data_bits_7_12),
    .io_wgt_rd_0_data_bits_7_13(compute_io_wgt_rd_0_data_bits_7_13),
    .io_wgt_rd_0_data_bits_7_14(compute_io_wgt_rd_0_data_bits_7_14),
    .io_wgt_rd_0_data_bits_7_15(compute_io_wgt_rd_0_data_bits_7_15),
    .io_wgt_rd_0_data_bits_8_0(compute_io_wgt_rd_0_data_bits_8_0),
    .io_wgt_rd_0_data_bits_8_1(compute_io_wgt_rd_0_data_bits_8_1),
    .io_wgt_rd_0_data_bits_8_2(compute_io_wgt_rd_0_data_bits_8_2),
    .io_wgt_rd_0_data_bits_8_3(compute_io_wgt_rd_0_data_bits_8_3),
    .io_wgt_rd_0_data_bits_8_4(compute_io_wgt_rd_0_data_bits_8_4),
    .io_wgt_rd_0_data_bits_8_5(compute_io_wgt_rd_0_data_bits_8_5),
    .io_wgt_rd_0_data_bits_8_6(compute_io_wgt_rd_0_data_bits_8_6),
    .io_wgt_rd_0_data_bits_8_7(compute_io_wgt_rd_0_data_bits_8_7),
    .io_wgt_rd_0_data_bits_8_8(compute_io_wgt_rd_0_data_bits_8_8),
    .io_wgt_rd_0_data_bits_8_9(compute_io_wgt_rd_0_data_bits_8_9),
    .io_wgt_rd_0_data_bits_8_10(compute_io_wgt_rd_0_data_bits_8_10),
    .io_wgt_rd_0_data_bits_8_11(compute_io_wgt_rd_0_data_bits_8_11),
    .io_wgt_rd_0_data_bits_8_12(compute_io_wgt_rd_0_data_bits_8_12),
    .io_wgt_rd_0_data_bits_8_13(compute_io_wgt_rd_0_data_bits_8_13),
    .io_wgt_rd_0_data_bits_8_14(compute_io_wgt_rd_0_data_bits_8_14),
    .io_wgt_rd_0_data_bits_8_15(compute_io_wgt_rd_0_data_bits_8_15),
    .io_wgt_rd_0_data_bits_9_0(compute_io_wgt_rd_0_data_bits_9_0),
    .io_wgt_rd_0_data_bits_9_1(compute_io_wgt_rd_0_data_bits_9_1),
    .io_wgt_rd_0_data_bits_9_2(compute_io_wgt_rd_0_data_bits_9_2),
    .io_wgt_rd_0_data_bits_9_3(compute_io_wgt_rd_0_data_bits_9_3),
    .io_wgt_rd_0_data_bits_9_4(compute_io_wgt_rd_0_data_bits_9_4),
    .io_wgt_rd_0_data_bits_9_5(compute_io_wgt_rd_0_data_bits_9_5),
    .io_wgt_rd_0_data_bits_9_6(compute_io_wgt_rd_0_data_bits_9_6),
    .io_wgt_rd_0_data_bits_9_7(compute_io_wgt_rd_0_data_bits_9_7),
    .io_wgt_rd_0_data_bits_9_8(compute_io_wgt_rd_0_data_bits_9_8),
    .io_wgt_rd_0_data_bits_9_9(compute_io_wgt_rd_0_data_bits_9_9),
    .io_wgt_rd_0_data_bits_9_10(compute_io_wgt_rd_0_data_bits_9_10),
    .io_wgt_rd_0_data_bits_9_11(compute_io_wgt_rd_0_data_bits_9_11),
    .io_wgt_rd_0_data_bits_9_12(compute_io_wgt_rd_0_data_bits_9_12),
    .io_wgt_rd_0_data_bits_9_13(compute_io_wgt_rd_0_data_bits_9_13),
    .io_wgt_rd_0_data_bits_9_14(compute_io_wgt_rd_0_data_bits_9_14),
    .io_wgt_rd_0_data_bits_9_15(compute_io_wgt_rd_0_data_bits_9_15),
    .io_wgt_rd_0_data_bits_10_0(compute_io_wgt_rd_0_data_bits_10_0),
    .io_wgt_rd_0_data_bits_10_1(compute_io_wgt_rd_0_data_bits_10_1),
    .io_wgt_rd_0_data_bits_10_2(compute_io_wgt_rd_0_data_bits_10_2),
    .io_wgt_rd_0_data_bits_10_3(compute_io_wgt_rd_0_data_bits_10_3),
    .io_wgt_rd_0_data_bits_10_4(compute_io_wgt_rd_0_data_bits_10_4),
    .io_wgt_rd_0_data_bits_10_5(compute_io_wgt_rd_0_data_bits_10_5),
    .io_wgt_rd_0_data_bits_10_6(compute_io_wgt_rd_0_data_bits_10_6),
    .io_wgt_rd_0_data_bits_10_7(compute_io_wgt_rd_0_data_bits_10_7),
    .io_wgt_rd_0_data_bits_10_8(compute_io_wgt_rd_0_data_bits_10_8),
    .io_wgt_rd_0_data_bits_10_9(compute_io_wgt_rd_0_data_bits_10_9),
    .io_wgt_rd_0_data_bits_10_10(compute_io_wgt_rd_0_data_bits_10_10),
    .io_wgt_rd_0_data_bits_10_11(compute_io_wgt_rd_0_data_bits_10_11),
    .io_wgt_rd_0_data_bits_10_12(compute_io_wgt_rd_0_data_bits_10_12),
    .io_wgt_rd_0_data_bits_10_13(compute_io_wgt_rd_0_data_bits_10_13),
    .io_wgt_rd_0_data_bits_10_14(compute_io_wgt_rd_0_data_bits_10_14),
    .io_wgt_rd_0_data_bits_10_15(compute_io_wgt_rd_0_data_bits_10_15),
    .io_wgt_rd_0_data_bits_11_0(compute_io_wgt_rd_0_data_bits_11_0),
    .io_wgt_rd_0_data_bits_11_1(compute_io_wgt_rd_0_data_bits_11_1),
    .io_wgt_rd_0_data_bits_11_2(compute_io_wgt_rd_0_data_bits_11_2),
    .io_wgt_rd_0_data_bits_11_3(compute_io_wgt_rd_0_data_bits_11_3),
    .io_wgt_rd_0_data_bits_11_4(compute_io_wgt_rd_0_data_bits_11_4),
    .io_wgt_rd_0_data_bits_11_5(compute_io_wgt_rd_0_data_bits_11_5),
    .io_wgt_rd_0_data_bits_11_6(compute_io_wgt_rd_0_data_bits_11_6),
    .io_wgt_rd_0_data_bits_11_7(compute_io_wgt_rd_0_data_bits_11_7),
    .io_wgt_rd_0_data_bits_11_8(compute_io_wgt_rd_0_data_bits_11_8),
    .io_wgt_rd_0_data_bits_11_9(compute_io_wgt_rd_0_data_bits_11_9),
    .io_wgt_rd_0_data_bits_11_10(compute_io_wgt_rd_0_data_bits_11_10),
    .io_wgt_rd_0_data_bits_11_11(compute_io_wgt_rd_0_data_bits_11_11),
    .io_wgt_rd_0_data_bits_11_12(compute_io_wgt_rd_0_data_bits_11_12),
    .io_wgt_rd_0_data_bits_11_13(compute_io_wgt_rd_0_data_bits_11_13),
    .io_wgt_rd_0_data_bits_11_14(compute_io_wgt_rd_0_data_bits_11_14),
    .io_wgt_rd_0_data_bits_11_15(compute_io_wgt_rd_0_data_bits_11_15),
    .io_wgt_rd_0_data_bits_12_0(compute_io_wgt_rd_0_data_bits_12_0),
    .io_wgt_rd_0_data_bits_12_1(compute_io_wgt_rd_0_data_bits_12_1),
    .io_wgt_rd_0_data_bits_12_2(compute_io_wgt_rd_0_data_bits_12_2),
    .io_wgt_rd_0_data_bits_12_3(compute_io_wgt_rd_0_data_bits_12_3),
    .io_wgt_rd_0_data_bits_12_4(compute_io_wgt_rd_0_data_bits_12_4),
    .io_wgt_rd_0_data_bits_12_5(compute_io_wgt_rd_0_data_bits_12_5),
    .io_wgt_rd_0_data_bits_12_6(compute_io_wgt_rd_0_data_bits_12_6),
    .io_wgt_rd_0_data_bits_12_7(compute_io_wgt_rd_0_data_bits_12_7),
    .io_wgt_rd_0_data_bits_12_8(compute_io_wgt_rd_0_data_bits_12_8),
    .io_wgt_rd_0_data_bits_12_9(compute_io_wgt_rd_0_data_bits_12_9),
    .io_wgt_rd_0_data_bits_12_10(compute_io_wgt_rd_0_data_bits_12_10),
    .io_wgt_rd_0_data_bits_12_11(compute_io_wgt_rd_0_data_bits_12_11),
    .io_wgt_rd_0_data_bits_12_12(compute_io_wgt_rd_0_data_bits_12_12),
    .io_wgt_rd_0_data_bits_12_13(compute_io_wgt_rd_0_data_bits_12_13),
    .io_wgt_rd_0_data_bits_12_14(compute_io_wgt_rd_0_data_bits_12_14),
    .io_wgt_rd_0_data_bits_12_15(compute_io_wgt_rd_0_data_bits_12_15),
    .io_wgt_rd_0_data_bits_13_0(compute_io_wgt_rd_0_data_bits_13_0),
    .io_wgt_rd_0_data_bits_13_1(compute_io_wgt_rd_0_data_bits_13_1),
    .io_wgt_rd_0_data_bits_13_2(compute_io_wgt_rd_0_data_bits_13_2),
    .io_wgt_rd_0_data_bits_13_3(compute_io_wgt_rd_0_data_bits_13_3),
    .io_wgt_rd_0_data_bits_13_4(compute_io_wgt_rd_0_data_bits_13_4),
    .io_wgt_rd_0_data_bits_13_5(compute_io_wgt_rd_0_data_bits_13_5),
    .io_wgt_rd_0_data_bits_13_6(compute_io_wgt_rd_0_data_bits_13_6),
    .io_wgt_rd_0_data_bits_13_7(compute_io_wgt_rd_0_data_bits_13_7),
    .io_wgt_rd_0_data_bits_13_8(compute_io_wgt_rd_0_data_bits_13_8),
    .io_wgt_rd_0_data_bits_13_9(compute_io_wgt_rd_0_data_bits_13_9),
    .io_wgt_rd_0_data_bits_13_10(compute_io_wgt_rd_0_data_bits_13_10),
    .io_wgt_rd_0_data_bits_13_11(compute_io_wgt_rd_0_data_bits_13_11),
    .io_wgt_rd_0_data_bits_13_12(compute_io_wgt_rd_0_data_bits_13_12),
    .io_wgt_rd_0_data_bits_13_13(compute_io_wgt_rd_0_data_bits_13_13),
    .io_wgt_rd_0_data_bits_13_14(compute_io_wgt_rd_0_data_bits_13_14),
    .io_wgt_rd_0_data_bits_13_15(compute_io_wgt_rd_0_data_bits_13_15),
    .io_wgt_rd_0_data_bits_14_0(compute_io_wgt_rd_0_data_bits_14_0),
    .io_wgt_rd_0_data_bits_14_1(compute_io_wgt_rd_0_data_bits_14_1),
    .io_wgt_rd_0_data_bits_14_2(compute_io_wgt_rd_0_data_bits_14_2),
    .io_wgt_rd_0_data_bits_14_3(compute_io_wgt_rd_0_data_bits_14_3),
    .io_wgt_rd_0_data_bits_14_4(compute_io_wgt_rd_0_data_bits_14_4),
    .io_wgt_rd_0_data_bits_14_5(compute_io_wgt_rd_0_data_bits_14_5),
    .io_wgt_rd_0_data_bits_14_6(compute_io_wgt_rd_0_data_bits_14_6),
    .io_wgt_rd_0_data_bits_14_7(compute_io_wgt_rd_0_data_bits_14_7),
    .io_wgt_rd_0_data_bits_14_8(compute_io_wgt_rd_0_data_bits_14_8),
    .io_wgt_rd_0_data_bits_14_9(compute_io_wgt_rd_0_data_bits_14_9),
    .io_wgt_rd_0_data_bits_14_10(compute_io_wgt_rd_0_data_bits_14_10),
    .io_wgt_rd_0_data_bits_14_11(compute_io_wgt_rd_0_data_bits_14_11),
    .io_wgt_rd_0_data_bits_14_12(compute_io_wgt_rd_0_data_bits_14_12),
    .io_wgt_rd_0_data_bits_14_13(compute_io_wgt_rd_0_data_bits_14_13),
    .io_wgt_rd_0_data_bits_14_14(compute_io_wgt_rd_0_data_bits_14_14),
    .io_wgt_rd_0_data_bits_14_15(compute_io_wgt_rd_0_data_bits_14_15),
    .io_wgt_rd_0_data_bits_15_0(compute_io_wgt_rd_0_data_bits_15_0),
    .io_wgt_rd_0_data_bits_15_1(compute_io_wgt_rd_0_data_bits_15_1),
    .io_wgt_rd_0_data_bits_15_2(compute_io_wgt_rd_0_data_bits_15_2),
    .io_wgt_rd_0_data_bits_15_3(compute_io_wgt_rd_0_data_bits_15_3),
    .io_wgt_rd_0_data_bits_15_4(compute_io_wgt_rd_0_data_bits_15_4),
    .io_wgt_rd_0_data_bits_15_5(compute_io_wgt_rd_0_data_bits_15_5),
    .io_wgt_rd_0_data_bits_15_6(compute_io_wgt_rd_0_data_bits_15_6),
    .io_wgt_rd_0_data_bits_15_7(compute_io_wgt_rd_0_data_bits_15_7),
    .io_wgt_rd_0_data_bits_15_8(compute_io_wgt_rd_0_data_bits_15_8),
    .io_wgt_rd_0_data_bits_15_9(compute_io_wgt_rd_0_data_bits_15_9),
    .io_wgt_rd_0_data_bits_15_10(compute_io_wgt_rd_0_data_bits_15_10),
    .io_wgt_rd_0_data_bits_15_11(compute_io_wgt_rd_0_data_bits_15_11),
    .io_wgt_rd_0_data_bits_15_12(compute_io_wgt_rd_0_data_bits_15_12),
    .io_wgt_rd_0_data_bits_15_13(compute_io_wgt_rd_0_data_bits_15_13),
    .io_wgt_rd_0_data_bits_15_14(compute_io_wgt_rd_0_data_bits_15_14),
    .io_wgt_rd_0_data_bits_15_15(compute_io_wgt_rd_0_data_bits_15_15),
    .io_out_wr_0_valid(compute_io_out_wr_0_valid),
    .io_out_wr_0_bits_idx(compute_io_out_wr_0_bits_idx),
    .io_out_wr_0_bits_data_0_0(compute_io_out_wr_0_bits_data_0_0),
    .io_out_wr_0_bits_data_0_1(compute_io_out_wr_0_bits_data_0_1),
    .io_out_wr_0_bits_data_0_2(compute_io_out_wr_0_bits_data_0_2),
    .io_out_wr_0_bits_data_0_3(compute_io_out_wr_0_bits_data_0_3),
    .io_out_wr_0_bits_data_0_4(compute_io_out_wr_0_bits_data_0_4),
    .io_out_wr_0_bits_data_0_5(compute_io_out_wr_0_bits_data_0_5),
    .io_out_wr_0_bits_data_0_6(compute_io_out_wr_0_bits_data_0_6),
    .io_out_wr_0_bits_data_0_7(compute_io_out_wr_0_bits_data_0_7),
    .io_out_wr_0_bits_data_0_8(compute_io_out_wr_0_bits_data_0_8),
    .io_out_wr_0_bits_data_0_9(compute_io_out_wr_0_bits_data_0_9),
    .io_out_wr_0_bits_data_0_10(compute_io_out_wr_0_bits_data_0_10),
    .io_out_wr_0_bits_data_0_11(compute_io_out_wr_0_bits_data_0_11),
    .io_out_wr_0_bits_data_0_12(compute_io_out_wr_0_bits_data_0_12),
    .io_out_wr_0_bits_data_0_13(compute_io_out_wr_0_bits_data_0_13),
    .io_out_wr_0_bits_data_0_14(compute_io_out_wr_0_bits_data_0_14),
    .io_out_wr_0_bits_data_0_15(compute_io_out_wr_0_bits_data_0_15),
    .io_finish(compute_io_finish),
    .io_acc_wr_event(compute_io_acc_wr_event)
  );
  Store store ( // @[Core.scala 70:21]
    .clock(store_clock),
    .reset(store_reset),
    .io_i_post(store_io_i_post),
    .io_o_post(store_io_o_post),
    .io_inst_ready(store_io_inst_ready),
    .io_inst_valid(store_io_inst_valid),
    .io_inst_bits(store_io_inst_bits),
    .io_out_baddr(store_io_out_baddr),
    .io_vme_wr_cmd_ready(store_io_vme_wr_cmd_ready),
    .io_vme_wr_cmd_valid(store_io_vme_wr_cmd_valid),
    .io_vme_wr_cmd_bits_addr(store_io_vme_wr_cmd_bits_addr),
    .io_vme_wr_cmd_bits_len(store_io_vme_wr_cmd_bits_len),
    .io_vme_wr_data_ready(store_io_vme_wr_data_ready),
    .io_vme_wr_data_valid(store_io_vme_wr_data_valid),
    .io_vme_wr_data_bits_data(store_io_vme_wr_data_bits_data),
    .io_vme_wr_ack(store_io_vme_wr_ack),
    .io_out_wr_0_valid(store_io_out_wr_0_valid),
    .io_out_wr_0_bits_idx(store_io_out_wr_0_bits_idx),
    .io_out_wr_0_bits_data_0_0(store_io_out_wr_0_bits_data_0_0),
    .io_out_wr_0_bits_data_0_1(store_io_out_wr_0_bits_data_0_1),
    .io_out_wr_0_bits_data_0_2(store_io_out_wr_0_bits_data_0_2),
    .io_out_wr_0_bits_data_0_3(store_io_out_wr_0_bits_data_0_3),
    .io_out_wr_0_bits_data_0_4(store_io_out_wr_0_bits_data_0_4),
    .io_out_wr_0_bits_data_0_5(store_io_out_wr_0_bits_data_0_5),
    .io_out_wr_0_bits_data_0_6(store_io_out_wr_0_bits_data_0_6),
    .io_out_wr_0_bits_data_0_7(store_io_out_wr_0_bits_data_0_7),
    .io_out_wr_0_bits_data_0_8(store_io_out_wr_0_bits_data_0_8),
    .io_out_wr_0_bits_data_0_9(store_io_out_wr_0_bits_data_0_9),
    .io_out_wr_0_bits_data_0_10(store_io_out_wr_0_bits_data_0_10),
    .io_out_wr_0_bits_data_0_11(store_io_out_wr_0_bits_data_0_11),
    .io_out_wr_0_bits_data_0_12(store_io_out_wr_0_bits_data_0_12),
    .io_out_wr_0_bits_data_0_13(store_io_out_wr_0_bits_data_0_13),
    .io_out_wr_0_bits_data_0_14(store_io_out_wr_0_bits_data_0_14),
    .io_out_wr_0_bits_data_0_15(store_io_out_wr_0_bits_data_0_15)
  );
  EventCounters ecounters ( // @[Core.scala 71:25]
    .clock(ecounters_clock),
    .reset(ecounters_reset),
    .io_launch(ecounters_io_launch),
    .io_finish(ecounters_io_finish),
    .io_ecnt_0_valid(ecounters_io_ecnt_0_valid),
    .io_ecnt_0_bits(ecounters_io_ecnt_0_bits),
    .io_ucnt_0_valid(ecounters_io_ucnt_0_valid),
    .io_ucnt_0_bits(ecounters_io_ucnt_0_bits),
    .io_acc_wr_event(ecounters_io_acc_wr_event)
  );
  assign io_vcr_finish = finish; // @[Core.scala 120:17]
  assign io_vcr_ecnt_0_valid = ecounters_io_ecnt_0_valid; // @[Core.scala 114:15]
  assign io_vcr_ecnt_0_bits = ecounters_io_ecnt_0_bits; // @[Core.scala 114:15]
  assign io_vcr_ucnt_0_valid = ecounters_io_ucnt_0_valid; // @[Core.scala 115:15]
  assign io_vcr_ucnt_0_bits = ecounters_io_ucnt_0_bits; // @[Core.scala 115:15]
  assign io_vme_rd_0_cmd_valid = fetch_io_vme_rd_cmd_valid; // @[Core.scala 74:16]
  assign io_vme_rd_0_cmd_bits_addr = fetch_io_vme_rd_cmd_bits_addr; // @[Core.scala 74:16]
  assign io_vme_rd_0_cmd_bits_len = fetch_io_vme_rd_cmd_bits_len; // @[Core.scala 74:16]
  assign io_vme_rd_0_data_ready = fetch_io_vme_rd_data_ready; // @[Core.scala 74:16]
  assign io_vme_rd_1_cmd_valid = compute_io_vme_rd_0_cmd_valid; // @[Core.scala 75:16]
  assign io_vme_rd_1_cmd_bits_addr = compute_io_vme_rd_0_cmd_bits_addr; // @[Core.scala 75:16]
  assign io_vme_rd_1_cmd_bits_len = compute_io_vme_rd_0_cmd_bits_len; // @[Core.scala 75:16]
  assign io_vme_rd_1_cmd_bits_tag = compute_io_vme_rd_0_cmd_bits_tag; // @[Core.scala 75:16]
  assign io_vme_rd_2_cmd_valid = load_io_vme_rd_0_cmd_valid; // @[Core.scala 76:16]
  assign io_vme_rd_2_cmd_bits_addr = load_io_vme_rd_0_cmd_bits_addr; // @[Core.scala 76:16]
  assign io_vme_rd_2_cmd_bits_len = load_io_vme_rd_0_cmd_bits_len; // @[Core.scala 76:16]
  assign io_vme_rd_2_cmd_bits_tag = load_io_vme_rd_0_cmd_bits_tag; // @[Core.scala 76:16]
  assign io_vme_rd_3_cmd_valid = load_io_vme_rd_1_cmd_valid; // @[Core.scala 77:16]
  assign io_vme_rd_3_cmd_bits_addr = load_io_vme_rd_1_cmd_bits_addr; // @[Core.scala 77:16]
  assign io_vme_rd_3_cmd_bits_len = load_io_vme_rd_1_cmd_bits_len; // @[Core.scala 77:16]
  assign io_vme_rd_3_cmd_bits_tag = load_io_vme_rd_1_cmd_bits_tag; // @[Core.scala 77:16]
  assign io_vme_rd_4_cmd_valid = compute_io_vme_rd_1_cmd_valid; // @[Core.scala 78:16]
  assign io_vme_rd_4_cmd_bits_addr = compute_io_vme_rd_1_cmd_bits_addr; // @[Core.scala 78:16]
  assign io_vme_rd_4_cmd_bits_len = compute_io_vme_rd_1_cmd_bits_len; // @[Core.scala 78:16]
  assign io_vme_rd_4_cmd_bits_tag = compute_io_vme_rd_1_cmd_bits_tag; // @[Core.scala 78:16]
  assign io_vme_wr_0_cmd_valid = store_io_vme_wr_cmd_valid; // @[Core.scala 79:16]
  assign io_vme_wr_0_cmd_bits_addr = store_io_vme_wr_cmd_bits_addr; // @[Core.scala 79:16]
  assign io_vme_wr_0_cmd_bits_len = store_io_vme_wr_cmd_bits_len; // @[Core.scala 79:16]
  assign io_vme_wr_0_data_valid = store_io_vme_wr_data_valid; // @[Core.scala 79:16]
  assign io_vme_wr_0_data_bits_data = store_io_vme_wr_data_bits_data; // @[Core.scala 79:16]
  assign fetch_clock = clock;
  assign fetch_reset = reset;
  assign fetch_io_launch = io_vcr_launch; // @[Core.scala 82:19]
  assign fetch_io_ins_baddr = io_vcr_ptrs_0; // @[Core.scala 83:22]
  assign fetch_io_ins_count = io_vcr_vals_0; // @[Core.scala 84:22]
  assign fetch_io_vme_rd_cmd_ready = io_vme_rd_0_cmd_ready; // @[Core.scala 74:16]
  assign fetch_io_vme_rd_data_valid = io_vme_rd_0_data_valid; // @[Core.scala 74:16]
  assign fetch_io_vme_rd_data_bits_data = io_vme_rd_0_data_bits_data; // @[Core.scala 74:16]
  assign fetch_io_inst_ld_ready = load_io_inst_ready; // @[Core.scala 88:16]
  assign fetch_io_inst_co_ready = compute_io_inst_ready; // @[Core.scala 97:19]
  assign fetch_io_inst_st_ready = store_io_inst_ready; // @[Core.scala 107:17]
  assign load_clock = clock;
  assign load_reset = reset;
  assign load_io_i_post = compute_io_o_post_0; // @[Core.scala 87:18]
  assign load_io_inst_valid = fetch_io_inst_ld_valid; // @[Core.scala 88:16]
  assign load_io_inst_bits = fetch_io_inst_ld_bits; // @[Core.scala 88:16]
  assign load_io_inp_baddr = io_vcr_ptrs_2; // @[Core.scala 89:21]
  assign load_io_wgt_baddr = io_vcr_ptrs_3; // @[Core.scala 90:21]
  assign load_io_vme_rd_0_cmd_ready = io_vme_rd_2_cmd_ready; // @[Core.scala 76:16]
  assign load_io_vme_rd_0_data_valid = io_vme_rd_2_data_valid; // @[Core.scala 76:16]
  assign load_io_vme_rd_0_data_bits_data = io_vme_rd_2_data_bits_data; // @[Core.scala 76:16]
  assign load_io_vme_rd_0_data_bits_tag = io_vme_rd_2_data_bits_tag; // @[Core.scala 76:16]
  assign load_io_vme_rd_1_cmd_ready = io_vme_rd_3_cmd_ready; // @[Core.scala 77:16]
  assign load_io_vme_rd_1_data_valid = io_vme_rd_3_data_valid; // @[Core.scala 77:16]
  assign load_io_vme_rd_1_data_bits_data = io_vme_rd_3_data_bits_data; // @[Core.scala 77:16]
  assign load_io_vme_rd_1_data_bits_tag = io_vme_rd_3_data_bits_tag; // @[Core.scala 77:16]
  assign load_io_inp_rd_0_idx_valid = compute_io_inp_rd_0_idx_valid; // @[Core.scala 100:18]
  assign load_io_inp_rd_0_idx_bits = compute_io_inp_rd_0_idx_bits; // @[Core.scala 100:18]
  assign load_io_wgt_rd_0_idx_valid = compute_io_wgt_rd_0_idx_valid; // @[Core.scala 101:18]
  assign load_io_wgt_rd_0_idx_bits = compute_io_wgt_rd_0_idx_bits; // @[Core.scala 101:18]
  assign compute_clock = clock;
  assign compute_reset = reset;
  assign compute_io_i_post_0 = load_io_o_post; // @[Core.scala 95:24]
  assign compute_io_i_post_1 = store_io_o_post; // @[Core.scala 96:24]
  assign compute_io_inst_valid = fetch_io_inst_co_valid; // @[Core.scala 97:19]
  assign compute_io_inst_bits = fetch_io_inst_co_bits; // @[Core.scala 97:19]
  assign compute_io_uop_baddr = io_vcr_ptrs_1; // @[Core.scala 98:24]
  assign compute_io_acc_baddr = io_vcr_ptrs_4; // @[Core.scala 99:24]
  assign compute_io_vme_rd_0_cmd_ready = io_vme_rd_1_cmd_ready; // @[Core.scala 75:16]
  assign compute_io_vme_rd_0_data_valid = io_vme_rd_1_data_valid; // @[Core.scala 75:16]
  assign compute_io_vme_rd_0_data_bits_data = io_vme_rd_1_data_bits_data; // @[Core.scala 75:16]
  assign compute_io_vme_rd_0_data_bits_tag = io_vme_rd_1_data_bits_tag; // @[Core.scala 75:16]
  assign compute_io_vme_rd_0_data_bits_last = io_vme_rd_1_data_bits_last; // @[Core.scala 75:16]
  assign compute_io_vme_rd_1_cmd_ready = io_vme_rd_4_cmd_ready; // @[Core.scala 78:16]
  assign compute_io_vme_rd_1_data_valid = io_vme_rd_4_data_valid; // @[Core.scala 78:16]
  assign compute_io_vme_rd_1_data_bits_data = io_vme_rd_4_data_bits_data; // @[Core.scala 78:16]
  assign compute_io_vme_rd_1_data_bits_tag = io_vme_rd_4_data_bits_tag; // @[Core.scala 78:16]
  assign compute_io_inp_rd_0_data_valid = load_io_inp_rd_0_data_valid; // @[Core.scala 100:18]
  assign compute_io_inp_rd_0_data_bits_0_0 = load_io_inp_rd_0_data_bits_0_0; // @[Core.scala 100:18]
  assign compute_io_inp_rd_0_data_bits_0_1 = load_io_inp_rd_0_data_bits_0_1; // @[Core.scala 100:18]
  assign compute_io_inp_rd_0_data_bits_0_2 = load_io_inp_rd_0_data_bits_0_2; // @[Core.scala 100:18]
  assign compute_io_inp_rd_0_data_bits_0_3 = load_io_inp_rd_0_data_bits_0_3; // @[Core.scala 100:18]
  assign compute_io_inp_rd_0_data_bits_0_4 = load_io_inp_rd_0_data_bits_0_4; // @[Core.scala 100:18]
  assign compute_io_inp_rd_0_data_bits_0_5 = load_io_inp_rd_0_data_bits_0_5; // @[Core.scala 100:18]
  assign compute_io_inp_rd_0_data_bits_0_6 = load_io_inp_rd_0_data_bits_0_6; // @[Core.scala 100:18]
  assign compute_io_inp_rd_0_data_bits_0_7 = load_io_inp_rd_0_data_bits_0_7; // @[Core.scala 100:18]
  assign compute_io_inp_rd_0_data_bits_0_8 = load_io_inp_rd_0_data_bits_0_8; // @[Core.scala 100:18]
  assign compute_io_inp_rd_0_data_bits_0_9 = load_io_inp_rd_0_data_bits_0_9; // @[Core.scala 100:18]
  assign compute_io_inp_rd_0_data_bits_0_10 = load_io_inp_rd_0_data_bits_0_10; // @[Core.scala 100:18]
  assign compute_io_inp_rd_0_data_bits_0_11 = load_io_inp_rd_0_data_bits_0_11; // @[Core.scala 100:18]
  assign compute_io_inp_rd_0_data_bits_0_12 = load_io_inp_rd_0_data_bits_0_12; // @[Core.scala 100:18]
  assign compute_io_inp_rd_0_data_bits_0_13 = load_io_inp_rd_0_data_bits_0_13; // @[Core.scala 100:18]
  assign compute_io_inp_rd_0_data_bits_0_14 = load_io_inp_rd_0_data_bits_0_14; // @[Core.scala 100:18]
  assign compute_io_inp_rd_0_data_bits_0_15 = load_io_inp_rd_0_data_bits_0_15; // @[Core.scala 100:18]
  assign compute_io_wgt_rd_0_data_valid = load_io_wgt_rd_0_data_valid; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_0_0 = load_io_wgt_rd_0_data_bits_0_0; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_0_1 = load_io_wgt_rd_0_data_bits_0_1; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_0_2 = load_io_wgt_rd_0_data_bits_0_2; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_0_3 = load_io_wgt_rd_0_data_bits_0_3; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_0_4 = load_io_wgt_rd_0_data_bits_0_4; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_0_5 = load_io_wgt_rd_0_data_bits_0_5; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_0_6 = load_io_wgt_rd_0_data_bits_0_6; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_0_7 = load_io_wgt_rd_0_data_bits_0_7; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_0_8 = load_io_wgt_rd_0_data_bits_0_8; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_0_9 = load_io_wgt_rd_0_data_bits_0_9; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_0_10 = load_io_wgt_rd_0_data_bits_0_10; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_0_11 = load_io_wgt_rd_0_data_bits_0_11; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_0_12 = load_io_wgt_rd_0_data_bits_0_12; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_0_13 = load_io_wgt_rd_0_data_bits_0_13; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_0_14 = load_io_wgt_rd_0_data_bits_0_14; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_0_15 = load_io_wgt_rd_0_data_bits_0_15; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_1_0 = load_io_wgt_rd_0_data_bits_1_0; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_1_1 = load_io_wgt_rd_0_data_bits_1_1; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_1_2 = load_io_wgt_rd_0_data_bits_1_2; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_1_3 = load_io_wgt_rd_0_data_bits_1_3; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_1_4 = load_io_wgt_rd_0_data_bits_1_4; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_1_5 = load_io_wgt_rd_0_data_bits_1_5; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_1_6 = load_io_wgt_rd_0_data_bits_1_6; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_1_7 = load_io_wgt_rd_0_data_bits_1_7; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_1_8 = load_io_wgt_rd_0_data_bits_1_8; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_1_9 = load_io_wgt_rd_0_data_bits_1_9; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_1_10 = load_io_wgt_rd_0_data_bits_1_10; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_1_11 = load_io_wgt_rd_0_data_bits_1_11; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_1_12 = load_io_wgt_rd_0_data_bits_1_12; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_1_13 = load_io_wgt_rd_0_data_bits_1_13; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_1_14 = load_io_wgt_rd_0_data_bits_1_14; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_1_15 = load_io_wgt_rd_0_data_bits_1_15; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_2_0 = load_io_wgt_rd_0_data_bits_2_0; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_2_1 = load_io_wgt_rd_0_data_bits_2_1; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_2_2 = load_io_wgt_rd_0_data_bits_2_2; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_2_3 = load_io_wgt_rd_0_data_bits_2_3; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_2_4 = load_io_wgt_rd_0_data_bits_2_4; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_2_5 = load_io_wgt_rd_0_data_bits_2_5; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_2_6 = load_io_wgt_rd_0_data_bits_2_6; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_2_7 = load_io_wgt_rd_0_data_bits_2_7; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_2_8 = load_io_wgt_rd_0_data_bits_2_8; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_2_9 = load_io_wgt_rd_0_data_bits_2_9; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_2_10 = load_io_wgt_rd_0_data_bits_2_10; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_2_11 = load_io_wgt_rd_0_data_bits_2_11; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_2_12 = load_io_wgt_rd_0_data_bits_2_12; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_2_13 = load_io_wgt_rd_0_data_bits_2_13; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_2_14 = load_io_wgt_rd_0_data_bits_2_14; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_2_15 = load_io_wgt_rd_0_data_bits_2_15; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_3_0 = load_io_wgt_rd_0_data_bits_3_0; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_3_1 = load_io_wgt_rd_0_data_bits_3_1; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_3_2 = load_io_wgt_rd_0_data_bits_3_2; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_3_3 = load_io_wgt_rd_0_data_bits_3_3; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_3_4 = load_io_wgt_rd_0_data_bits_3_4; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_3_5 = load_io_wgt_rd_0_data_bits_3_5; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_3_6 = load_io_wgt_rd_0_data_bits_3_6; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_3_7 = load_io_wgt_rd_0_data_bits_3_7; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_3_8 = load_io_wgt_rd_0_data_bits_3_8; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_3_9 = load_io_wgt_rd_0_data_bits_3_9; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_3_10 = load_io_wgt_rd_0_data_bits_3_10; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_3_11 = load_io_wgt_rd_0_data_bits_3_11; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_3_12 = load_io_wgt_rd_0_data_bits_3_12; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_3_13 = load_io_wgt_rd_0_data_bits_3_13; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_3_14 = load_io_wgt_rd_0_data_bits_3_14; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_3_15 = load_io_wgt_rd_0_data_bits_3_15; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_4_0 = load_io_wgt_rd_0_data_bits_4_0; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_4_1 = load_io_wgt_rd_0_data_bits_4_1; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_4_2 = load_io_wgt_rd_0_data_bits_4_2; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_4_3 = load_io_wgt_rd_0_data_bits_4_3; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_4_4 = load_io_wgt_rd_0_data_bits_4_4; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_4_5 = load_io_wgt_rd_0_data_bits_4_5; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_4_6 = load_io_wgt_rd_0_data_bits_4_6; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_4_7 = load_io_wgt_rd_0_data_bits_4_7; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_4_8 = load_io_wgt_rd_0_data_bits_4_8; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_4_9 = load_io_wgt_rd_0_data_bits_4_9; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_4_10 = load_io_wgt_rd_0_data_bits_4_10; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_4_11 = load_io_wgt_rd_0_data_bits_4_11; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_4_12 = load_io_wgt_rd_0_data_bits_4_12; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_4_13 = load_io_wgt_rd_0_data_bits_4_13; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_4_14 = load_io_wgt_rd_0_data_bits_4_14; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_4_15 = load_io_wgt_rd_0_data_bits_4_15; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_5_0 = load_io_wgt_rd_0_data_bits_5_0; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_5_1 = load_io_wgt_rd_0_data_bits_5_1; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_5_2 = load_io_wgt_rd_0_data_bits_5_2; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_5_3 = load_io_wgt_rd_0_data_bits_5_3; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_5_4 = load_io_wgt_rd_0_data_bits_5_4; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_5_5 = load_io_wgt_rd_0_data_bits_5_5; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_5_6 = load_io_wgt_rd_0_data_bits_5_6; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_5_7 = load_io_wgt_rd_0_data_bits_5_7; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_5_8 = load_io_wgt_rd_0_data_bits_5_8; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_5_9 = load_io_wgt_rd_0_data_bits_5_9; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_5_10 = load_io_wgt_rd_0_data_bits_5_10; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_5_11 = load_io_wgt_rd_0_data_bits_5_11; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_5_12 = load_io_wgt_rd_0_data_bits_5_12; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_5_13 = load_io_wgt_rd_0_data_bits_5_13; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_5_14 = load_io_wgt_rd_0_data_bits_5_14; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_5_15 = load_io_wgt_rd_0_data_bits_5_15; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_6_0 = load_io_wgt_rd_0_data_bits_6_0; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_6_1 = load_io_wgt_rd_0_data_bits_6_1; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_6_2 = load_io_wgt_rd_0_data_bits_6_2; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_6_3 = load_io_wgt_rd_0_data_bits_6_3; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_6_4 = load_io_wgt_rd_0_data_bits_6_4; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_6_5 = load_io_wgt_rd_0_data_bits_6_5; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_6_6 = load_io_wgt_rd_0_data_bits_6_6; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_6_7 = load_io_wgt_rd_0_data_bits_6_7; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_6_8 = load_io_wgt_rd_0_data_bits_6_8; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_6_9 = load_io_wgt_rd_0_data_bits_6_9; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_6_10 = load_io_wgt_rd_0_data_bits_6_10; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_6_11 = load_io_wgt_rd_0_data_bits_6_11; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_6_12 = load_io_wgt_rd_0_data_bits_6_12; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_6_13 = load_io_wgt_rd_0_data_bits_6_13; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_6_14 = load_io_wgt_rd_0_data_bits_6_14; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_6_15 = load_io_wgt_rd_0_data_bits_6_15; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_7_0 = load_io_wgt_rd_0_data_bits_7_0; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_7_1 = load_io_wgt_rd_0_data_bits_7_1; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_7_2 = load_io_wgt_rd_0_data_bits_7_2; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_7_3 = load_io_wgt_rd_0_data_bits_7_3; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_7_4 = load_io_wgt_rd_0_data_bits_7_4; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_7_5 = load_io_wgt_rd_0_data_bits_7_5; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_7_6 = load_io_wgt_rd_0_data_bits_7_6; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_7_7 = load_io_wgt_rd_0_data_bits_7_7; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_7_8 = load_io_wgt_rd_0_data_bits_7_8; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_7_9 = load_io_wgt_rd_0_data_bits_7_9; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_7_10 = load_io_wgt_rd_0_data_bits_7_10; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_7_11 = load_io_wgt_rd_0_data_bits_7_11; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_7_12 = load_io_wgt_rd_0_data_bits_7_12; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_7_13 = load_io_wgt_rd_0_data_bits_7_13; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_7_14 = load_io_wgt_rd_0_data_bits_7_14; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_7_15 = load_io_wgt_rd_0_data_bits_7_15; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_8_0 = load_io_wgt_rd_0_data_bits_8_0; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_8_1 = load_io_wgt_rd_0_data_bits_8_1; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_8_2 = load_io_wgt_rd_0_data_bits_8_2; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_8_3 = load_io_wgt_rd_0_data_bits_8_3; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_8_4 = load_io_wgt_rd_0_data_bits_8_4; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_8_5 = load_io_wgt_rd_0_data_bits_8_5; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_8_6 = load_io_wgt_rd_0_data_bits_8_6; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_8_7 = load_io_wgt_rd_0_data_bits_8_7; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_8_8 = load_io_wgt_rd_0_data_bits_8_8; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_8_9 = load_io_wgt_rd_0_data_bits_8_9; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_8_10 = load_io_wgt_rd_0_data_bits_8_10; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_8_11 = load_io_wgt_rd_0_data_bits_8_11; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_8_12 = load_io_wgt_rd_0_data_bits_8_12; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_8_13 = load_io_wgt_rd_0_data_bits_8_13; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_8_14 = load_io_wgt_rd_0_data_bits_8_14; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_8_15 = load_io_wgt_rd_0_data_bits_8_15; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_9_0 = load_io_wgt_rd_0_data_bits_9_0; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_9_1 = load_io_wgt_rd_0_data_bits_9_1; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_9_2 = load_io_wgt_rd_0_data_bits_9_2; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_9_3 = load_io_wgt_rd_0_data_bits_9_3; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_9_4 = load_io_wgt_rd_0_data_bits_9_4; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_9_5 = load_io_wgt_rd_0_data_bits_9_5; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_9_6 = load_io_wgt_rd_0_data_bits_9_6; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_9_7 = load_io_wgt_rd_0_data_bits_9_7; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_9_8 = load_io_wgt_rd_0_data_bits_9_8; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_9_9 = load_io_wgt_rd_0_data_bits_9_9; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_9_10 = load_io_wgt_rd_0_data_bits_9_10; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_9_11 = load_io_wgt_rd_0_data_bits_9_11; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_9_12 = load_io_wgt_rd_0_data_bits_9_12; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_9_13 = load_io_wgt_rd_0_data_bits_9_13; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_9_14 = load_io_wgt_rd_0_data_bits_9_14; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_9_15 = load_io_wgt_rd_0_data_bits_9_15; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_10_0 = load_io_wgt_rd_0_data_bits_10_0; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_10_1 = load_io_wgt_rd_0_data_bits_10_1; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_10_2 = load_io_wgt_rd_0_data_bits_10_2; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_10_3 = load_io_wgt_rd_0_data_bits_10_3; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_10_4 = load_io_wgt_rd_0_data_bits_10_4; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_10_5 = load_io_wgt_rd_0_data_bits_10_5; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_10_6 = load_io_wgt_rd_0_data_bits_10_6; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_10_7 = load_io_wgt_rd_0_data_bits_10_7; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_10_8 = load_io_wgt_rd_0_data_bits_10_8; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_10_9 = load_io_wgt_rd_0_data_bits_10_9; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_10_10 = load_io_wgt_rd_0_data_bits_10_10; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_10_11 = load_io_wgt_rd_0_data_bits_10_11; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_10_12 = load_io_wgt_rd_0_data_bits_10_12; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_10_13 = load_io_wgt_rd_0_data_bits_10_13; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_10_14 = load_io_wgt_rd_0_data_bits_10_14; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_10_15 = load_io_wgt_rd_0_data_bits_10_15; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_11_0 = load_io_wgt_rd_0_data_bits_11_0; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_11_1 = load_io_wgt_rd_0_data_bits_11_1; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_11_2 = load_io_wgt_rd_0_data_bits_11_2; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_11_3 = load_io_wgt_rd_0_data_bits_11_3; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_11_4 = load_io_wgt_rd_0_data_bits_11_4; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_11_5 = load_io_wgt_rd_0_data_bits_11_5; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_11_6 = load_io_wgt_rd_0_data_bits_11_6; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_11_7 = load_io_wgt_rd_0_data_bits_11_7; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_11_8 = load_io_wgt_rd_0_data_bits_11_8; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_11_9 = load_io_wgt_rd_0_data_bits_11_9; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_11_10 = load_io_wgt_rd_0_data_bits_11_10; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_11_11 = load_io_wgt_rd_0_data_bits_11_11; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_11_12 = load_io_wgt_rd_0_data_bits_11_12; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_11_13 = load_io_wgt_rd_0_data_bits_11_13; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_11_14 = load_io_wgt_rd_0_data_bits_11_14; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_11_15 = load_io_wgt_rd_0_data_bits_11_15; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_12_0 = load_io_wgt_rd_0_data_bits_12_0; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_12_1 = load_io_wgt_rd_0_data_bits_12_1; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_12_2 = load_io_wgt_rd_0_data_bits_12_2; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_12_3 = load_io_wgt_rd_0_data_bits_12_3; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_12_4 = load_io_wgt_rd_0_data_bits_12_4; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_12_5 = load_io_wgt_rd_0_data_bits_12_5; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_12_6 = load_io_wgt_rd_0_data_bits_12_6; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_12_7 = load_io_wgt_rd_0_data_bits_12_7; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_12_8 = load_io_wgt_rd_0_data_bits_12_8; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_12_9 = load_io_wgt_rd_0_data_bits_12_9; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_12_10 = load_io_wgt_rd_0_data_bits_12_10; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_12_11 = load_io_wgt_rd_0_data_bits_12_11; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_12_12 = load_io_wgt_rd_0_data_bits_12_12; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_12_13 = load_io_wgt_rd_0_data_bits_12_13; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_12_14 = load_io_wgt_rd_0_data_bits_12_14; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_12_15 = load_io_wgt_rd_0_data_bits_12_15; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_13_0 = load_io_wgt_rd_0_data_bits_13_0; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_13_1 = load_io_wgt_rd_0_data_bits_13_1; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_13_2 = load_io_wgt_rd_0_data_bits_13_2; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_13_3 = load_io_wgt_rd_0_data_bits_13_3; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_13_4 = load_io_wgt_rd_0_data_bits_13_4; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_13_5 = load_io_wgt_rd_0_data_bits_13_5; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_13_6 = load_io_wgt_rd_0_data_bits_13_6; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_13_7 = load_io_wgt_rd_0_data_bits_13_7; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_13_8 = load_io_wgt_rd_0_data_bits_13_8; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_13_9 = load_io_wgt_rd_0_data_bits_13_9; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_13_10 = load_io_wgt_rd_0_data_bits_13_10; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_13_11 = load_io_wgt_rd_0_data_bits_13_11; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_13_12 = load_io_wgt_rd_0_data_bits_13_12; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_13_13 = load_io_wgt_rd_0_data_bits_13_13; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_13_14 = load_io_wgt_rd_0_data_bits_13_14; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_13_15 = load_io_wgt_rd_0_data_bits_13_15; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_14_0 = load_io_wgt_rd_0_data_bits_14_0; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_14_1 = load_io_wgt_rd_0_data_bits_14_1; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_14_2 = load_io_wgt_rd_0_data_bits_14_2; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_14_3 = load_io_wgt_rd_0_data_bits_14_3; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_14_4 = load_io_wgt_rd_0_data_bits_14_4; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_14_5 = load_io_wgt_rd_0_data_bits_14_5; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_14_6 = load_io_wgt_rd_0_data_bits_14_6; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_14_7 = load_io_wgt_rd_0_data_bits_14_7; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_14_8 = load_io_wgt_rd_0_data_bits_14_8; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_14_9 = load_io_wgt_rd_0_data_bits_14_9; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_14_10 = load_io_wgt_rd_0_data_bits_14_10; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_14_11 = load_io_wgt_rd_0_data_bits_14_11; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_14_12 = load_io_wgt_rd_0_data_bits_14_12; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_14_13 = load_io_wgt_rd_0_data_bits_14_13; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_14_14 = load_io_wgt_rd_0_data_bits_14_14; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_14_15 = load_io_wgt_rd_0_data_bits_14_15; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_15_0 = load_io_wgt_rd_0_data_bits_15_0; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_15_1 = load_io_wgt_rd_0_data_bits_15_1; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_15_2 = load_io_wgt_rd_0_data_bits_15_2; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_15_3 = load_io_wgt_rd_0_data_bits_15_3; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_15_4 = load_io_wgt_rd_0_data_bits_15_4; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_15_5 = load_io_wgt_rd_0_data_bits_15_5; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_15_6 = load_io_wgt_rd_0_data_bits_15_6; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_15_7 = load_io_wgt_rd_0_data_bits_15_7; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_15_8 = load_io_wgt_rd_0_data_bits_15_8; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_15_9 = load_io_wgt_rd_0_data_bits_15_9; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_15_10 = load_io_wgt_rd_0_data_bits_15_10; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_15_11 = load_io_wgt_rd_0_data_bits_15_11; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_15_12 = load_io_wgt_rd_0_data_bits_15_12; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_15_13 = load_io_wgt_rd_0_data_bits_15_13; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_15_14 = load_io_wgt_rd_0_data_bits_15_14; // @[Core.scala 101:18]
  assign compute_io_wgt_rd_0_data_bits_15_15 = load_io_wgt_rd_0_data_bits_15_15; // @[Core.scala 101:18]
  assign store_clock = clock;
  assign store_reset = reset;
  assign store_io_i_post = compute_io_o_post_1; // @[Core.scala 106:19]
  assign store_io_inst_valid = fetch_io_inst_st_valid; // @[Core.scala 107:17]
  assign store_io_inst_bits = fetch_io_inst_st_bits; // @[Core.scala 107:17]
  assign store_io_out_baddr = io_vcr_ptrs_5; // @[Core.scala 108:22]
  assign store_io_vme_wr_cmd_ready = io_vme_wr_0_cmd_ready; // @[Core.scala 79:16]
  assign store_io_vme_wr_data_ready = io_vme_wr_0_data_ready; // @[Core.scala 79:16]
  assign store_io_vme_wr_ack = io_vme_wr_0_ack; // @[Core.scala 79:16]
  assign store_io_out_wr_0_valid = compute_io_out_wr_0_valid; // @[Core.scala 109:16]
  assign store_io_out_wr_0_bits_idx = compute_io_out_wr_0_bits_idx; // @[Core.scala 109:16]
  assign store_io_out_wr_0_bits_data_0_0 = compute_io_out_wr_0_bits_data_0_0; // @[Core.scala 109:16]
  assign store_io_out_wr_0_bits_data_0_1 = compute_io_out_wr_0_bits_data_0_1; // @[Core.scala 109:16]
  assign store_io_out_wr_0_bits_data_0_2 = compute_io_out_wr_0_bits_data_0_2; // @[Core.scala 109:16]
  assign store_io_out_wr_0_bits_data_0_3 = compute_io_out_wr_0_bits_data_0_3; // @[Core.scala 109:16]
  assign store_io_out_wr_0_bits_data_0_4 = compute_io_out_wr_0_bits_data_0_4; // @[Core.scala 109:16]
  assign store_io_out_wr_0_bits_data_0_5 = compute_io_out_wr_0_bits_data_0_5; // @[Core.scala 109:16]
  assign store_io_out_wr_0_bits_data_0_6 = compute_io_out_wr_0_bits_data_0_6; // @[Core.scala 109:16]
  assign store_io_out_wr_0_bits_data_0_7 = compute_io_out_wr_0_bits_data_0_7; // @[Core.scala 109:16]
  assign store_io_out_wr_0_bits_data_0_8 = compute_io_out_wr_0_bits_data_0_8; // @[Core.scala 109:16]
  assign store_io_out_wr_0_bits_data_0_9 = compute_io_out_wr_0_bits_data_0_9; // @[Core.scala 109:16]
  assign store_io_out_wr_0_bits_data_0_10 = compute_io_out_wr_0_bits_data_0_10; // @[Core.scala 109:16]
  assign store_io_out_wr_0_bits_data_0_11 = compute_io_out_wr_0_bits_data_0_11; // @[Core.scala 109:16]
  assign store_io_out_wr_0_bits_data_0_12 = compute_io_out_wr_0_bits_data_0_12; // @[Core.scala 109:16]
  assign store_io_out_wr_0_bits_data_0_13 = compute_io_out_wr_0_bits_data_0_13; // @[Core.scala 109:16]
  assign store_io_out_wr_0_bits_data_0_14 = compute_io_out_wr_0_bits_data_0_14; // @[Core.scala 109:16]
  assign store_io_out_wr_0_bits_data_0_15 = compute_io_out_wr_0_bits_data_0_15; // @[Core.scala 109:16]
  assign ecounters_clock = clock;
  assign ecounters_reset = reset;
  assign ecounters_io_launch = io_vcr_launch; // @[Core.scala 112:23]
  assign ecounters_io_finish = compute_io_finish; // @[Core.scala 113:23]
  assign ecounters_io_acc_wr_event = compute_io_acc_wr_event; // @[Core.scala 116:29]
  always @(posedge clock) begin
    finish <= compute_io_finish; // @[Core.scala 119:23]
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
  finish = _RAND_0[0:0];
`endif // RANDOMIZE_REG_INIT
  `endif // RANDOMIZE
end // initial
`ifdef FIRRTL_AFTER_INITIAL
`FIRRTL_AFTER_INITIAL
`endif
`endif // SYNTHESIS
endmodule