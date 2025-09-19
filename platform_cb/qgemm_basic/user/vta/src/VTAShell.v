module VTAShell(
  input         clock,
  input         reset,
  output        io_host_aw_ready,
  input         io_host_aw_valid,
  input  [15:0] io_host_aw_bits_addr,
  output        io_host_w_ready,
  input         io_host_w_valid,
  input  [31:0] io_host_w_bits_data,
  input         io_host_b_ready,
  output        io_host_b_valid,
  output        io_host_ar_ready,
  input         io_host_ar_valid,
  input  [15:0] io_host_ar_bits_addr,
  input         io_host_r_ready,
  output        io_host_r_valid,
  output [31:0] io_host_r_bits_data,
  input         io_mem_aw_ready,
  output        io_mem_aw_valid,
  output [31:0] io_mem_aw_bits_addr,
  output [7:0]  io_mem_aw_bits_len,
  input         io_mem_w_ready,
  output        io_mem_w_valid,
  output [63:0] io_mem_w_bits_data,
  output        io_mem_w_bits_last,
  output        io_mem_b_ready,
  input         io_mem_b_valid,
  input         io_mem_ar_ready,
  output        io_mem_ar_valid,
  output [31:0] io_mem_ar_bits_addr,
  output [7:0]  io_mem_ar_bits_id,
  output [7:0]  io_mem_ar_bits_len,
  input         io_mem_r_valid,
  input  [63:0] io_mem_r_bits_data,
  input         io_mem_r_bits_last,
  input  [7:0]  io_mem_r_bits_id
);
  wire  vcr_clock; // @[VTAShell.scala 48:19]
  wire  vcr_reset; // @[VTAShell.scala 48:19]
  wire  vcr_io_host_aw_ready; // @[VTAShell.scala 48:19]
  wire  vcr_io_host_aw_valid; // @[VTAShell.scala 48:19]
  wire [15:0] vcr_io_host_aw_bits_addr; // @[VTAShell.scala 48:19]
  wire  vcr_io_host_w_ready; // @[VTAShell.scala 48:19]
  wire  vcr_io_host_w_valid; // @[VTAShell.scala 48:19]
  wire [31:0] vcr_io_host_w_bits_data; // @[VTAShell.scala 48:19]
  wire  vcr_io_host_b_ready; // @[VTAShell.scala 48:19]
  wire  vcr_io_host_b_valid; // @[VTAShell.scala 48:19]
  wire  vcr_io_host_ar_ready; // @[VTAShell.scala 48:19]
  wire  vcr_io_host_ar_valid; // @[VTAShell.scala 48:19]
  wire [15:0] vcr_io_host_ar_bits_addr; // @[VTAShell.scala 48:19]
  wire  vcr_io_host_r_ready; // @[VTAShell.scala 48:19]
  wire  vcr_io_host_r_valid; // @[VTAShell.scala 48:19]
  wire [31:0] vcr_io_host_r_bits_data; // @[VTAShell.scala 48:19]
  wire  vcr_io_vcr_launch; // @[VTAShell.scala 48:19]
  wire  vcr_io_vcr_finish; // @[VTAShell.scala 48:19]
  wire  vcr_io_vcr_ecnt_0_valid; // @[VTAShell.scala 48:19]
  wire [31:0] vcr_io_vcr_ecnt_0_bits; // @[VTAShell.scala 48:19]
  wire [31:0] vcr_io_vcr_vals_0; // @[VTAShell.scala 48:19]
  wire [31:0] vcr_io_vcr_ptrs_0; // @[VTAShell.scala 48:19]
  wire [31:0] vcr_io_vcr_ptrs_1; // @[VTAShell.scala 48:19]
  wire [31:0] vcr_io_vcr_ptrs_2; // @[VTAShell.scala 48:19]
  wire [31:0] vcr_io_vcr_ptrs_3; // @[VTAShell.scala 48:19]
  wire [31:0] vcr_io_vcr_ptrs_4; // @[VTAShell.scala 48:19]
  wire [31:0] vcr_io_vcr_ptrs_5; // @[VTAShell.scala 48:19]
  wire  vcr_io_vcr_ucnt_0_valid; // @[VTAShell.scala 48:19]
  wire [31:0] vcr_io_vcr_ucnt_0_bits; // @[VTAShell.scala 48:19]
  wire  vme_clock; // @[VTAShell.scala 49:19]
  wire  vme_reset; // @[VTAShell.scala 49:19]
  wire  vme_io_mem_aw_ready; // @[VTAShell.scala 49:19]
  wire  vme_io_mem_aw_valid; // @[VTAShell.scala 49:19]
  wire [31:0] vme_io_mem_aw_bits_addr; // @[VTAShell.scala 49:19]
  wire [7:0] vme_io_mem_aw_bits_len; // @[VTAShell.scala 49:19]
  wire  vme_io_mem_w_ready; // @[VTAShell.scala 49:19]
  wire  vme_io_mem_w_valid; // @[VTAShell.scala 49:19]
  wire [63:0] vme_io_mem_w_bits_data; // @[VTAShell.scala 49:19]
  wire  vme_io_mem_w_bits_last; // @[VTAShell.scala 49:19]
  wire  vme_io_mem_b_ready; // @[VTAShell.scala 49:19]
  wire  vme_io_mem_b_valid; // @[VTAShell.scala 49:19]
  wire  vme_io_mem_ar_ready; // @[VTAShell.scala 49:19]
  wire  vme_io_mem_ar_valid; // @[VTAShell.scala 49:19]
  wire [31:0] vme_io_mem_ar_bits_addr; // @[VTAShell.scala 49:19]
  wire [7:0] vme_io_mem_ar_bits_id; // @[VTAShell.scala 49:19]
  wire [7:0] vme_io_mem_ar_bits_len; // @[VTAShell.scala 49:19]
  wire  vme_io_mem_r_valid; // @[VTAShell.scala 49:19]
  wire [63:0] vme_io_mem_r_bits_data; // @[VTAShell.scala 49:19]
  wire  vme_io_mem_r_bits_last; // @[VTAShell.scala 49:19]
  wire [7:0] vme_io_mem_r_bits_id; // @[VTAShell.scala 49:19]
  wire  vme_io_vme_rd_0_cmd_ready; // @[VTAShell.scala 49:19]
  wire  vme_io_vme_rd_0_cmd_valid; // @[VTAShell.scala 49:19]
  wire [31:0] vme_io_vme_rd_0_cmd_bits_addr; // @[VTAShell.scala 49:19]
  wire [7:0] vme_io_vme_rd_0_cmd_bits_len; // @[VTAShell.scala 49:19]
  wire  vme_io_vme_rd_0_data_ready; // @[VTAShell.scala 49:19]
  wire  vme_io_vme_rd_0_data_valid; // @[VTAShell.scala 49:19]
  wire [63:0] vme_io_vme_rd_0_data_bits_data; // @[VTAShell.scala 49:19]
  wire  vme_io_vme_rd_1_cmd_ready; // @[VTAShell.scala 49:19]
  wire  vme_io_vme_rd_1_cmd_valid; // @[VTAShell.scala 49:19]
  wire [31:0] vme_io_vme_rd_1_cmd_bits_addr; // @[VTAShell.scala 49:19]
  wire [7:0] vme_io_vme_rd_1_cmd_bits_len; // @[VTAShell.scala 49:19]
  wire [20:0] vme_io_vme_rd_1_cmd_bits_tag; // @[VTAShell.scala 49:19]
  wire  vme_io_vme_rd_1_data_valid; // @[VTAShell.scala 49:19]
  wire [63:0] vme_io_vme_rd_1_data_bits_data; // @[VTAShell.scala 49:19]
  wire [20:0] vme_io_vme_rd_1_data_bits_tag; // @[VTAShell.scala 49:19]
  wire  vme_io_vme_rd_1_data_bits_last; // @[VTAShell.scala 49:19]
  wire  vme_io_vme_rd_2_cmd_ready; // @[VTAShell.scala 49:19]
  wire  vme_io_vme_rd_2_cmd_valid; // @[VTAShell.scala 49:19]
  wire [31:0] vme_io_vme_rd_2_cmd_bits_addr; // @[VTAShell.scala 49:19]
  wire [7:0] vme_io_vme_rd_2_cmd_bits_len; // @[VTAShell.scala 49:19]
  wire [20:0] vme_io_vme_rd_2_cmd_bits_tag; // @[VTAShell.scala 49:19]
  wire  vme_io_vme_rd_2_data_valid; // @[VTAShell.scala 49:19]
  wire [63:0] vme_io_vme_rd_2_data_bits_data; // @[VTAShell.scala 49:19]
  wire [20:0] vme_io_vme_rd_2_data_bits_tag; // @[VTAShell.scala 49:19]
  wire  vme_io_vme_rd_3_cmd_ready; // @[VTAShell.scala 49:19]
  wire  vme_io_vme_rd_3_cmd_valid; // @[VTAShell.scala 49:19]
  wire [31:0] vme_io_vme_rd_3_cmd_bits_addr; // @[VTAShell.scala 49:19]
  wire [7:0] vme_io_vme_rd_3_cmd_bits_len; // @[VTAShell.scala 49:19]
  wire [20:0] vme_io_vme_rd_3_cmd_bits_tag; // @[VTAShell.scala 49:19]
  wire  vme_io_vme_rd_3_data_valid; // @[VTAShell.scala 49:19]
  wire [63:0] vme_io_vme_rd_3_data_bits_data; // @[VTAShell.scala 49:19]
  wire [20:0] vme_io_vme_rd_3_data_bits_tag; // @[VTAShell.scala 49:19]
  wire  vme_io_vme_rd_4_cmd_ready; // @[VTAShell.scala 49:19]
  wire  vme_io_vme_rd_4_cmd_valid; // @[VTAShell.scala 49:19]
  wire [31:0] vme_io_vme_rd_4_cmd_bits_addr; // @[VTAShell.scala 49:19]
  wire [7:0] vme_io_vme_rd_4_cmd_bits_len; // @[VTAShell.scala 49:19]
  wire [20:0] vme_io_vme_rd_4_cmd_bits_tag; // @[VTAShell.scala 49:19]
  wire  vme_io_vme_rd_4_data_valid; // @[VTAShell.scala 49:19]
  wire [63:0] vme_io_vme_rd_4_data_bits_data; // @[VTAShell.scala 49:19]
  wire [20:0] vme_io_vme_rd_4_data_bits_tag; // @[VTAShell.scala 49:19]
  wire  vme_io_vme_wr_0_cmd_ready; // @[VTAShell.scala 49:19]
  wire  vme_io_vme_wr_0_cmd_valid; // @[VTAShell.scala 49:19]
  wire [31:0] vme_io_vme_wr_0_cmd_bits_addr; // @[VTAShell.scala 49:19]
  wire [7:0] vme_io_vme_wr_0_cmd_bits_len; // @[VTAShell.scala 49:19]
  wire  vme_io_vme_wr_0_data_ready; // @[VTAShell.scala 49:19]
  wire  vme_io_vme_wr_0_data_valid; // @[VTAShell.scala 49:19]
  wire [63:0] vme_io_vme_wr_0_data_bits_data; // @[VTAShell.scala 49:19]
  wire  vme_io_vme_wr_0_ack; // @[VTAShell.scala 49:19]
  wire  core_clock; // @[VTAShell.scala 50:20]
  wire  core_reset; // @[VTAShell.scala 50:20]
  wire  core_io_vcr_launch; // @[VTAShell.scala 50:20]
  wire  core_io_vcr_finish; // @[VTAShell.scala 50:20]
  wire  core_io_vcr_ecnt_0_valid; // @[VTAShell.scala 50:20]
  wire [31:0] core_io_vcr_ecnt_0_bits; // @[VTAShell.scala 50:20]
  wire [31:0] core_io_vcr_vals_0; // @[VTAShell.scala 50:20]
  wire [31:0] core_io_vcr_ptrs_0; // @[VTAShell.scala 50:20]
  wire [31:0] core_io_vcr_ptrs_1; // @[VTAShell.scala 50:20]
  wire [31:0] core_io_vcr_ptrs_2; // @[VTAShell.scala 50:20]
  wire [31:0] core_io_vcr_ptrs_3; // @[VTAShell.scala 50:20]
  wire [31:0] core_io_vcr_ptrs_4; // @[VTAShell.scala 50:20]
  wire [31:0] core_io_vcr_ptrs_5; // @[VTAShell.scala 50:20]
  wire  core_io_vcr_ucnt_0_valid; // @[VTAShell.scala 50:20]
  wire [31:0] core_io_vcr_ucnt_0_bits; // @[VTAShell.scala 50:20]
  wire  core_io_vme_rd_0_cmd_ready; // @[VTAShell.scala 50:20]
  wire  core_io_vme_rd_0_cmd_valid; // @[VTAShell.scala 50:20]
  wire [31:0] core_io_vme_rd_0_cmd_bits_addr; // @[VTAShell.scala 50:20]
  wire [7:0] core_io_vme_rd_0_cmd_bits_len; // @[VTAShell.scala 50:20]
  wire  core_io_vme_rd_0_data_ready; // @[VTAShell.scala 50:20]
  wire  core_io_vme_rd_0_data_valid; // @[VTAShell.scala 50:20]
  wire [63:0] core_io_vme_rd_0_data_bits_data; // @[VTAShell.scala 50:20]
  wire  core_io_vme_rd_1_cmd_ready; // @[VTAShell.scala 50:20]
  wire  core_io_vme_rd_1_cmd_valid; // @[VTAShell.scala 50:20]
  wire [31:0] core_io_vme_rd_1_cmd_bits_addr; // @[VTAShell.scala 50:20]
  wire [7:0] core_io_vme_rd_1_cmd_bits_len; // @[VTAShell.scala 50:20]
  wire [20:0] core_io_vme_rd_1_cmd_bits_tag; // @[VTAShell.scala 50:20]
  wire  core_io_vme_rd_1_data_valid; // @[VTAShell.scala 50:20]
  wire [63:0] core_io_vme_rd_1_data_bits_data; // @[VTAShell.scala 50:20]
  wire [20:0] core_io_vme_rd_1_data_bits_tag; // @[VTAShell.scala 50:20]
  wire  core_io_vme_rd_1_data_bits_last; // @[VTAShell.scala 50:20]
  wire  core_io_vme_rd_2_cmd_ready; // @[VTAShell.scala 50:20]
  wire  core_io_vme_rd_2_cmd_valid; // @[VTAShell.scala 50:20]
  wire [31:0] core_io_vme_rd_2_cmd_bits_addr; // @[VTAShell.scala 50:20]
  wire [7:0] core_io_vme_rd_2_cmd_bits_len; // @[VTAShell.scala 50:20]
  wire [20:0] core_io_vme_rd_2_cmd_bits_tag; // @[VTAShell.scala 50:20]
  wire  core_io_vme_rd_2_data_valid; // @[VTAShell.scala 50:20]
  wire [63:0] core_io_vme_rd_2_data_bits_data; // @[VTAShell.scala 50:20]
  wire [20:0] core_io_vme_rd_2_data_bits_tag; // @[VTAShell.scala 50:20]
  wire  core_io_vme_rd_3_cmd_ready; // @[VTAShell.scala 50:20]
  wire  core_io_vme_rd_3_cmd_valid; // @[VTAShell.scala 50:20]
  wire [31:0] core_io_vme_rd_3_cmd_bits_addr; // @[VTAShell.scala 50:20]
  wire [7:0] core_io_vme_rd_3_cmd_bits_len; // @[VTAShell.scala 50:20]
  wire [20:0] core_io_vme_rd_3_cmd_bits_tag; // @[VTAShell.scala 50:20]
  wire  core_io_vme_rd_3_data_valid; // @[VTAShell.scala 50:20]
  wire [63:0] core_io_vme_rd_3_data_bits_data; // @[VTAShell.scala 50:20]
  wire [20:0] core_io_vme_rd_3_data_bits_tag; // @[VTAShell.scala 50:20]
  wire  core_io_vme_rd_4_cmd_ready; // @[VTAShell.scala 50:20]
  wire  core_io_vme_rd_4_cmd_valid; // @[VTAShell.scala 50:20]
  wire [31:0] core_io_vme_rd_4_cmd_bits_addr; // @[VTAShell.scala 50:20]
  wire [7:0] core_io_vme_rd_4_cmd_bits_len; // @[VTAShell.scala 50:20]
  wire [20:0] core_io_vme_rd_4_cmd_bits_tag; // @[VTAShell.scala 50:20]
  wire  core_io_vme_rd_4_data_valid; // @[VTAShell.scala 50:20]
  wire [63:0] core_io_vme_rd_4_data_bits_data; // @[VTAShell.scala 50:20]
  wire [20:0] core_io_vme_rd_4_data_bits_tag; // @[VTAShell.scala 50:20]
  wire  core_io_vme_wr_0_cmd_ready; // @[VTAShell.scala 50:20]
  wire  core_io_vme_wr_0_cmd_valid; // @[VTAShell.scala 50:20]
  wire [31:0] core_io_vme_wr_0_cmd_bits_addr; // @[VTAShell.scala 50:20]
  wire [7:0] core_io_vme_wr_0_cmd_bits_len; // @[VTAShell.scala 50:20]
  wire  core_io_vme_wr_0_data_ready; // @[VTAShell.scala 50:20]
  wire  core_io_vme_wr_0_data_valid; // @[VTAShell.scala 50:20]
  wire [63:0] core_io_vme_wr_0_data_bits_data; // @[VTAShell.scala 50:20]
  wire  core_io_vme_wr_0_ack; // @[VTAShell.scala 50:20]
  VCR vcr ( // @[VTAShell.scala 48:19]
    .clock(vcr_clock),
    .reset(vcr_reset),
    .io_host_aw_ready(vcr_io_host_aw_ready),
    .io_host_aw_valid(vcr_io_host_aw_valid),
    .io_host_aw_bits_addr(vcr_io_host_aw_bits_addr),
    .io_host_w_ready(vcr_io_host_w_ready),
    .io_host_w_valid(vcr_io_host_w_valid),
    .io_host_w_bits_data(vcr_io_host_w_bits_data),
    .io_host_b_ready(vcr_io_host_b_ready),
    .io_host_b_valid(vcr_io_host_b_valid),
    .io_host_ar_ready(vcr_io_host_ar_ready),
    .io_host_ar_valid(vcr_io_host_ar_valid),
    .io_host_ar_bits_addr(vcr_io_host_ar_bits_addr),
    .io_host_r_ready(vcr_io_host_r_ready),
    .io_host_r_valid(vcr_io_host_r_valid),
    .io_host_r_bits_data(vcr_io_host_r_bits_data),
    .io_vcr_launch(vcr_io_vcr_launch),
    .io_vcr_finish(vcr_io_vcr_finish),
    .io_vcr_ecnt_0_valid(vcr_io_vcr_ecnt_0_valid),
    .io_vcr_ecnt_0_bits(vcr_io_vcr_ecnt_0_bits),
    .io_vcr_vals_0(vcr_io_vcr_vals_0),
    .io_vcr_ptrs_0(vcr_io_vcr_ptrs_0),
    .io_vcr_ptrs_1(vcr_io_vcr_ptrs_1),
    .io_vcr_ptrs_2(vcr_io_vcr_ptrs_2),
    .io_vcr_ptrs_3(vcr_io_vcr_ptrs_3),
    .io_vcr_ptrs_4(vcr_io_vcr_ptrs_4),
    .io_vcr_ptrs_5(vcr_io_vcr_ptrs_5),
    .io_vcr_ucnt_0_valid(vcr_io_vcr_ucnt_0_valid),
    .io_vcr_ucnt_0_bits(vcr_io_vcr_ucnt_0_bits)
  );
  VME vme ( // @[VTAShell.scala 49:19]
    .clock(vme_clock),
    .reset(vme_reset),
    .io_mem_aw_ready(vme_io_mem_aw_ready),
    .io_mem_aw_valid(vme_io_mem_aw_valid),
    .io_mem_aw_bits_addr(vme_io_mem_aw_bits_addr),
    .io_mem_aw_bits_len(vme_io_mem_aw_bits_len),
    .io_mem_w_ready(vme_io_mem_w_ready),
    .io_mem_w_valid(vme_io_mem_w_valid),
    .io_mem_w_bits_data(vme_io_mem_w_bits_data),
    .io_mem_w_bits_last(vme_io_mem_w_bits_last),
    .io_mem_b_ready(vme_io_mem_b_ready),
    .io_mem_b_valid(vme_io_mem_b_valid),
    .io_mem_ar_ready(vme_io_mem_ar_ready),
    .io_mem_ar_valid(vme_io_mem_ar_valid),
    .io_mem_ar_bits_addr(vme_io_mem_ar_bits_addr),
    .io_mem_ar_bits_id(vme_io_mem_ar_bits_id),
    .io_mem_ar_bits_len(vme_io_mem_ar_bits_len),
    .io_mem_r_valid(vme_io_mem_r_valid),
    .io_mem_r_bits_data(vme_io_mem_r_bits_data),
    .io_mem_r_bits_last(vme_io_mem_r_bits_last),
    .io_mem_r_bits_id(vme_io_mem_r_bits_id),
    .io_vme_rd_0_cmd_ready(vme_io_vme_rd_0_cmd_ready),
    .io_vme_rd_0_cmd_valid(vme_io_vme_rd_0_cmd_valid),
    .io_vme_rd_0_cmd_bits_addr(vme_io_vme_rd_0_cmd_bits_addr),
    .io_vme_rd_0_cmd_bits_len(vme_io_vme_rd_0_cmd_bits_len),
    .io_vme_rd_0_data_ready(vme_io_vme_rd_0_data_ready),
    .io_vme_rd_0_data_valid(vme_io_vme_rd_0_data_valid),
    .io_vme_rd_0_data_bits_data(vme_io_vme_rd_0_data_bits_data),
    .io_vme_rd_1_cmd_ready(vme_io_vme_rd_1_cmd_ready),
    .io_vme_rd_1_cmd_valid(vme_io_vme_rd_1_cmd_valid),
    .io_vme_rd_1_cmd_bits_addr(vme_io_vme_rd_1_cmd_bits_addr),
    .io_vme_rd_1_cmd_bits_len(vme_io_vme_rd_1_cmd_bits_len),
    .io_vme_rd_1_cmd_bits_tag(vme_io_vme_rd_1_cmd_bits_tag),
    .io_vme_rd_1_data_valid(vme_io_vme_rd_1_data_valid),
    .io_vme_rd_1_data_bits_data(vme_io_vme_rd_1_data_bits_data),
    .io_vme_rd_1_data_bits_tag(vme_io_vme_rd_1_data_bits_tag),
    .io_vme_rd_1_data_bits_last(vme_io_vme_rd_1_data_bits_last),
    .io_vme_rd_2_cmd_ready(vme_io_vme_rd_2_cmd_ready),
    .io_vme_rd_2_cmd_valid(vme_io_vme_rd_2_cmd_valid),
    .io_vme_rd_2_cmd_bits_addr(vme_io_vme_rd_2_cmd_bits_addr),
    .io_vme_rd_2_cmd_bits_len(vme_io_vme_rd_2_cmd_bits_len),
    .io_vme_rd_2_cmd_bits_tag(vme_io_vme_rd_2_cmd_bits_tag),
    .io_vme_rd_2_data_valid(vme_io_vme_rd_2_data_valid),
    .io_vme_rd_2_data_bits_data(vme_io_vme_rd_2_data_bits_data),
    .io_vme_rd_2_data_bits_tag(vme_io_vme_rd_2_data_bits_tag),
    .io_vme_rd_3_cmd_ready(vme_io_vme_rd_3_cmd_ready),
    .io_vme_rd_3_cmd_valid(vme_io_vme_rd_3_cmd_valid),
    .io_vme_rd_3_cmd_bits_addr(vme_io_vme_rd_3_cmd_bits_addr),
    .io_vme_rd_3_cmd_bits_len(vme_io_vme_rd_3_cmd_bits_len),
    .io_vme_rd_3_cmd_bits_tag(vme_io_vme_rd_3_cmd_bits_tag),
    .io_vme_rd_3_data_valid(vme_io_vme_rd_3_data_valid),
    .io_vme_rd_3_data_bits_data(vme_io_vme_rd_3_data_bits_data),
    .io_vme_rd_3_data_bits_tag(vme_io_vme_rd_3_data_bits_tag),
    .io_vme_rd_4_cmd_ready(vme_io_vme_rd_4_cmd_ready),
    .io_vme_rd_4_cmd_valid(vme_io_vme_rd_4_cmd_valid),
    .io_vme_rd_4_cmd_bits_addr(vme_io_vme_rd_4_cmd_bits_addr),
    .io_vme_rd_4_cmd_bits_len(vme_io_vme_rd_4_cmd_bits_len),
    .io_vme_rd_4_cmd_bits_tag(vme_io_vme_rd_4_cmd_bits_tag),
    .io_vme_rd_4_data_valid(vme_io_vme_rd_4_data_valid),
    .io_vme_rd_4_data_bits_data(vme_io_vme_rd_4_data_bits_data),
    .io_vme_rd_4_data_bits_tag(vme_io_vme_rd_4_data_bits_tag),
    .io_vme_wr_0_cmd_ready(vme_io_vme_wr_0_cmd_ready),
    .io_vme_wr_0_cmd_valid(vme_io_vme_wr_0_cmd_valid),
    .io_vme_wr_0_cmd_bits_addr(vme_io_vme_wr_0_cmd_bits_addr),
    .io_vme_wr_0_cmd_bits_len(vme_io_vme_wr_0_cmd_bits_len),
    .io_vme_wr_0_data_ready(vme_io_vme_wr_0_data_ready),
    .io_vme_wr_0_data_valid(vme_io_vme_wr_0_data_valid),
    .io_vme_wr_0_data_bits_data(vme_io_vme_wr_0_data_bits_data),
    .io_vme_wr_0_ack(vme_io_vme_wr_0_ack)
  );
  Core core ( // @[VTAShell.scala 50:20]
    .clock(core_clock),
    .reset(core_reset),
    .io_vcr_launch(core_io_vcr_launch),
    .io_vcr_finish(core_io_vcr_finish),
    .io_vcr_ecnt_0_valid(core_io_vcr_ecnt_0_valid),
    .io_vcr_ecnt_0_bits(core_io_vcr_ecnt_0_bits),
    .io_vcr_vals_0(core_io_vcr_vals_0),
    .io_vcr_ptrs_0(core_io_vcr_ptrs_0),
    .io_vcr_ptrs_1(core_io_vcr_ptrs_1),
    .io_vcr_ptrs_2(core_io_vcr_ptrs_2),
    .io_vcr_ptrs_3(core_io_vcr_ptrs_3),
    .io_vcr_ptrs_4(core_io_vcr_ptrs_4),
    .io_vcr_ptrs_5(core_io_vcr_ptrs_5),
    .io_vcr_ucnt_0_valid(core_io_vcr_ucnt_0_valid),
    .io_vcr_ucnt_0_bits(core_io_vcr_ucnt_0_bits),
    .io_vme_rd_0_cmd_ready(core_io_vme_rd_0_cmd_ready),
    .io_vme_rd_0_cmd_valid(core_io_vme_rd_0_cmd_valid),
    .io_vme_rd_0_cmd_bits_addr(core_io_vme_rd_0_cmd_bits_addr),
    .io_vme_rd_0_cmd_bits_len(core_io_vme_rd_0_cmd_bits_len),
    .io_vme_rd_0_data_ready(core_io_vme_rd_0_data_ready),
    .io_vme_rd_0_data_valid(core_io_vme_rd_0_data_valid),
    .io_vme_rd_0_data_bits_data(core_io_vme_rd_0_data_bits_data),
    .io_vme_rd_1_cmd_ready(core_io_vme_rd_1_cmd_ready),
    .io_vme_rd_1_cmd_valid(core_io_vme_rd_1_cmd_valid),
    .io_vme_rd_1_cmd_bits_addr(core_io_vme_rd_1_cmd_bits_addr),
    .io_vme_rd_1_cmd_bits_len(core_io_vme_rd_1_cmd_bits_len),
    .io_vme_rd_1_cmd_bits_tag(core_io_vme_rd_1_cmd_bits_tag),
    .io_vme_rd_1_data_valid(core_io_vme_rd_1_data_valid),
    .io_vme_rd_1_data_bits_data(core_io_vme_rd_1_data_bits_data),
    .io_vme_rd_1_data_bits_tag(core_io_vme_rd_1_data_bits_tag),
    .io_vme_rd_1_data_bits_last(core_io_vme_rd_1_data_bits_last),
    .io_vme_rd_2_cmd_ready(core_io_vme_rd_2_cmd_ready),
    .io_vme_rd_2_cmd_valid(core_io_vme_rd_2_cmd_valid),
    .io_vme_rd_2_cmd_bits_addr(core_io_vme_rd_2_cmd_bits_addr),
    .io_vme_rd_2_cmd_bits_len(core_io_vme_rd_2_cmd_bits_len),
    .io_vme_rd_2_cmd_bits_tag(core_io_vme_rd_2_cmd_bits_tag),
    .io_vme_rd_2_data_valid(core_io_vme_rd_2_data_valid),
    .io_vme_rd_2_data_bits_data(core_io_vme_rd_2_data_bits_data),
    .io_vme_rd_2_data_bits_tag(core_io_vme_rd_2_data_bits_tag),
    .io_vme_rd_3_cmd_ready(core_io_vme_rd_3_cmd_ready),
    .io_vme_rd_3_cmd_valid(core_io_vme_rd_3_cmd_valid),
    .io_vme_rd_3_cmd_bits_addr(core_io_vme_rd_3_cmd_bits_addr),
    .io_vme_rd_3_cmd_bits_len(core_io_vme_rd_3_cmd_bits_len),
    .io_vme_rd_3_cmd_bits_tag(core_io_vme_rd_3_cmd_bits_tag),
    .io_vme_rd_3_data_valid(core_io_vme_rd_3_data_valid),
    .io_vme_rd_3_data_bits_data(core_io_vme_rd_3_data_bits_data),
    .io_vme_rd_3_data_bits_tag(core_io_vme_rd_3_data_bits_tag),
    .io_vme_rd_4_cmd_ready(core_io_vme_rd_4_cmd_ready),
    .io_vme_rd_4_cmd_valid(core_io_vme_rd_4_cmd_valid),
    .io_vme_rd_4_cmd_bits_addr(core_io_vme_rd_4_cmd_bits_addr),
    .io_vme_rd_4_cmd_bits_len(core_io_vme_rd_4_cmd_bits_len),
    .io_vme_rd_4_cmd_bits_tag(core_io_vme_rd_4_cmd_bits_tag),
    .io_vme_rd_4_data_valid(core_io_vme_rd_4_data_valid),
    .io_vme_rd_4_data_bits_data(core_io_vme_rd_4_data_bits_data),
    .io_vme_rd_4_data_bits_tag(core_io_vme_rd_4_data_bits_tag),
    .io_vme_wr_0_cmd_ready(core_io_vme_wr_0_cmd_ready),
    .io_vme_wr_0_cmd_valid(core_io_vme_wr_0_cmd_valid),
    .io_vme_wr_0_cmd_bits_addr(core_io_vme_wr_0_cmd_bits_addr),
    .io_vme_wr_0_cmd_bits_len(core_io_vme_wr_0_cmd_bits_len),
    .io_vme_wr_0_data_ready(core_io_vme_wr_0_data_ready),
    .io_vme_wr_0_data_valid(core_io_vme_wr_0_data_valid),
    .io_vme_wr_0_data_bits_data(core_io_vme_wr_0_data_bits_data),
    .io_vme_wr_0_ack(core_io_vme_wr_0_ack)
  );
  assign io_host_aw_ready = vcr_io_host_aw_ready; // @[VTAShell.scala 55:15]
  assign io_host_w_ready = vcr_io_host_w_ready; // @[VTAShell.scala 55:15]
  assign io_host_b_valid = vcr_io_host_b_valid; // @[VTAShell.scala 55:15]
  assign io_host_ar_ready = vcr_io_host_ar_ready; // @[VTAShell.scala 55:15]
  assign io_host_r_valid = vcr_io_host_r_valid; // @[VTAShell.scala 55:15]
  assign io_host_r_bits_data = vcr_io_host_r_bits_data; // @[VTAShell.scala 55:15]
  assign io_mem_aw_valid = vme_io_mem_aw_valid; // @[VTAShell.scala 56:10]
  assign io_mem_aw_bits_addr = vme_io_mem_aw_bits_addr; // @[VTAShell.scala 56:10]
  assign io_mem_aw_bits_len = vme_io_mem_aw_bits_len; // @[VTAShell.scala 56:10]
  assign io_mem_w_valid = vme_io_mem_w_valid; // @[VTAShell.scala 56:10]
  assign io_mem_w_bits_data = vme_io_mem_w_bits_data; // @[VTAShell.scala 56:10]
  assign io_mem_w_bits_last = vme_io_mem_w_bits_last; // @[VTAShell.scala 56:10]
  assign io_mem_b_ready = vme_io_mem_b_ready; // @[VTAShell.scala 56:10]
  assign io_mem_ar_valid = vme_io_mem_ar_valid; // @[VTAShell.scala 56:10]
  assign io_mem_ar_bits_addr = vme_io_mem_ar_bits_addr; // @[VTAShell.scala 56:10]
  assign io_mem_ar_bits_id = vme_io_mem_ar_bits_id; // @[VTAShell.scala 56:10]
  assign io_mem_ar_bits_len = vme_io_mem_ar_bits_len; // @[VTAShell.scala 56:10]
  assign vcr_clock = clock;
  assign vcr_reset = reset;
  assign vcr_io_host_aw_valid = io_host_aw_valid; // @[VTAShell.scala 55:15]
  assign vcr_io_host_aw_bits_addr = io_host_aw_bits_addr; // @[VTAShell.scala 55:15]
  assign vcr_io_host_w_valid = io_host_w_valid; // @[VTAShell.scala 55:15]
  assign vcr_io_host_w_bits_data = io_host_w_bits_data; // @[VTAShell.scala 55:15]
  assign vcr_io_host_b_ready = io_host_b_ready; // @[VTAShell.scala 55:15]
  assign vcr_io_host_ar_valid = io_host_ar_valid; // @[VTAShell.scala 55:15]
  assign vcr_io_host_ar_bits_addr = io_host_ar_bits_addr; // @[VTAShell.scala 55:15]
  assign vcr_io_host_r_ready = io_host_r_ready; // @[VTAShell.scala 55:15]
  assign vcr_io_vcr_finish = core_io_vcr_finish; // @[VTAShell.scala 52:15]
  assign vcr_io_vcr_ecnt_0_valid = core_io_vcr_ecnt_0_valid; // @[VTAShell.scala 52:15]
  assign vcr_io_vcr_ecnt_0_bits = core_io_vcr_ecnt_0_bits; // @[VTAShell.scala 52:15]
  assign vcr_io_vcr_ucnt_0_valid = core_io_vcr_ucnt_0_valid; // @[VTAShell.scala 52:15]
  assign vcr_io_vcr_ucnt_0_bits = core_io_vcr_ucnt_0_bits; // @[VTAShell.scala 52:15]
  assign vme_clock = clock;
  assign vme_reset = reset;
  assign vme_io_mem_aw_ready = io_mem_aw_ready; // @[VTAShell.scala 56:10]
  assign vme_io_mem_w_ready = io_mem_w_ready; // @[VTAShell.scala 56:10]
  assign vme_io_mem_b_valid = io_mem_b_valid; // @[VTAShell.scala 56:10]
  assign vme_io_mem_ar_ready = io_mem_ar_ready; // @[VTAShell.scala 56:10]
  assign vme_io_mem_r_valid = io_mem_r_valid; // @[VTAShell.scala 56:10]
  assign vme_io_mem_r_bits_data = io_mem_r_bits_data; // @[VTAShell.scala 56:10]
  assign vme_io_mem_r_bits_last = io_mem_r_bits_last; // @[VTAShell.scala 56:10]
  assign vme_io_mem_r_bits_id = io_mem_r_bits_id; // @[VTAShell.scala 56:10]
  assign vme_io_vme_rd_0_cmd_valid = core_io_vme_rd_0_cmd_valid; // @[VTAShell.scala 53:14]
  assign vme_io_vme_rd_0_cmd_bits_addr = core_io_vme_rd_0_cmd_bits_addr; // @[VTAShell.scala 53:14]
  assign vme_io_vme_rd_0_cmd_bits_len = core_io_vme_rd_0_cmd_bits_len; // @[VTAShell.scala 53:14]
  assign vme_io_vme_rd_0_data_ready = core_io_vme_rd_0_data_ready; // @[VTAShell.scala 53:14]
  assign vme_io_vme_rd_1_cmd_valid = core_io_vme_rd_1_cmd_valid; // @[VTAShell.scala 53:14]
  assign vme_io_vme_rd_1_cmd_bits_addr = core_io_vme_rd_1_cmd_bits_addr; // @[VTAShell.scala 53:14]
  assign vme_io_vme_rd_1_cmd_bits_len = core_io_vme_rd_1_cmd_bits_len; // @[VTAShell.scala 53:14]
  assign vme_io_vme_rd_1_cmd_bits_tag = core_io_vme_rd_1_cmd_bits_tag; // @[VTAShell.scala 53:14]
  assign vme_io_vme_rd_2_cmd_valid = core_io_vme_rd_2_cmd_valid; // @[VTAShell.scala 53:14]
  assign vme_io_vme_rd_2_cmd_bits_addr = core_io_vme_rd_2_cmd_bits_addr; // @[VTAShell.scala 53:14]
  assign vme_io_vme_rd_2_cmd_bits_len = core_io_vme_rd_2_cmd_bits_len; // @[VTAShell.scala 53:14]
  assign vme_io_vme_rd_2_cmd_bits_tag = core_io_vme_rd_2_cmd_bits_tag; // @[VTAShell.scala 53:14]
  assign vme_io_vme_rd_3_cmd_valid = core_io_vme_rd_3_cmd_valid; // @[VTAShell.scala 53:14]
  assign vme_io_vme_rd_3_cmd_bits_addr = core_io_vme_rd_3_cmd_bits_addr; // @[VTAShell.scala 53:14]
  assign vme_io_vme_rd_3_cmd_bits_len = core_io_vme_rd_3_cmd_bits_len; // @[VTAShell.scala 53:14]
  assign vme_io_vme_rd_3_cmd_bits_tag = core_io_vme_rd_3_cmd_bits_tag; // @[VTAShell.scala 53:14]
  assign vme_io_vme_rd_4_cmd_valid = core_io_vme_rd_4_cmd_valid; // @[VTAShell.scala 53:14]
  assign vme_io_vme_rd_4_cmd_bits_addr = core_io_vme_rd_4_cmd_bits_addr; // @[VTAShell.scala 53:14]
  assign vme_io_vme_rd_4_cmd_bits_len = core_io_vme_rd_4_cmd_bits_len; // @[VTAShell.scala 53:14]
  assign vme_io_vme_rd_4_cmd_bits_tag = core_io_vme_rd_4_cmd_bits_tag; // @[VTAShell.scala 53:14]
  assign vme_io_vme_wr_0_cmd_valid = core_io_vme_wr_0_cmd_valid; // @[VTAShell.scala 53:14]
  assign vme_io_vme_wr_0_cmd_bits_addr = core_io_vme_wr_0_cmd_bits_addr; // @[VTAShell.scala 53:14]
  assign vme_io_vme_wr_0_cmd_bits_len = core_io_vme_wr_0_cmd_bits_len; // @[VTAShell.scala 53:14]
  assign vme_io_vme_wr_0_data_valid = core_io_vme_wr_0_data_valid; // @[VTAShell.scala 53:14]
  assign vme_io_vme_wr_0_data_bits_data = core_io_vme_wr_0_data_bits_data; // @[VTAShell.scala 53:14]
  assign core_clock = clock;
  assign core_reset = reset;
  assign core_io_vcr_launch = vcr_io_vcr_launch; // @[VTAShell.scala 52:15]
  assign core_io_vcr_vals_0 = vcr_io_vcr_vals_0; // @[VTAShell.scala 52:15]
  assign core_io_vcr_ptrs_0 = vcr_io_vcr_ptrs_0; // @[VTAShell.scala 52:15]
  assign core_io_vcr_ptrs_1 = vcr_io_vcr_ptrs_1; // @[VTAShell.scala 52:15]
  assign core_io_vcr_ptrs_2 = vcr_io_vcr_ptrs_2; // @[VTAShell.scala 52:15]
  assign core_io_vcr_ptrs_3 = vcr_io_vcr_ptrs_3; // @[VTAShell.scala 52:15]
  assign core_io_vcr_ptrs_4 = vcr_io_vcr_ptrs_4; // @[VTAShell.scala 52:15]
  assign core_io_vcr_ptrs_5 = vcr_io_vcr_ptrs_5; // @[VTAShell.scala 52:15]
  assign core_io_vme_rd_0_cmd_ready = vme_io_vme_rd_0_cmd_ready; // @[VTAShell.scala 53:14]
  assign core_io_vme_rd_0_data_valid = vme_io_vme_rd_0_data_valid; // @[VTAShell.scala 53:14]
  assign core_io_vme_rd_0_data_bits_data = vme_io_vme_rd_0_data_bits_data; // @[VTAShell.scala 53:14]
  assign core_io_vme_rd_1_cmd_ready = vme_io_vme_rd_1_cmd_ready; // @[VTAShell.scala 53:14]
  assign core_io_vme_rd_1_data_valid = vme_io_vme_rd_1_data_valid; // @[VTAShell.scala 53:14]
  assign core_io_vme_rd_1_data_bits_data = vme_io_vme_rd_1_data_bits_data; // @[VTAShell.scala 53:14]
  assign core_io_vme_rd_1_data_bits_tag = vme_io_vme_rd_1_data_bits_tag; // @[VTAShell.scala 53:14]
  assign core_io_vme_rd_1_data_bits_last = vme_io_vme_rd_1_data_bits_last; // @[VTAShell.scala 53:14]
  assign core_io_vme_rd_2_cmd_ready = vme_io_vme_rd_2_cmd_ready; // @[VTAShell.scala 53:14]
  assign core_io_vme_rd_2_data_valid = vme_io_vme_rd_2_data_valid; // @[VTAShell.scala 53:14]
  assign core_io_vme_rd_2_data_bits_data = vme_io_vme_rd_2_data_bits_data; // @[VTAShell.scala 53:14]
  assign core_io_vme_rd_2_data_bits_tag = vme_io_vme_rd_2_data_bits_tag; // @[VTAShell.scala 53:14]
  assign core_io_vme_rd_3_cmd_ready = vme_io_vme_rd_3_cmd_ready; // @[VTAShell.scala 53:14]
  assign core_io_vme_rd_3_data_valid = vme_io_vme_rd_3_data_valid; // @[VTAShell.scala 53:14]
  assign core_io_vme_rd_3_data_bits_data = vme_io_vme_rd_3_data_bits_data; // @[VTAShell.scala 53:14]
  assign core_io_vme_rd_3_data_bits_tag = vme_io_vme_rd_3_data_bits_tag; // @[VTAShell.scala 53:14]
  assign core_io_vme_rd_4_cmd_ready = vme_io_vme_rd_4_cmd_ready; // @[VTAShell.scala 53:14]
  assign core_io_vme_rd_4_data_valid = vme_io_vme_rd_4_data_valid; // @[VTAShell.scala 53:14]
  assign core_io_vme_rd_4_data_bits_data = vme_io_vme_rd_4_data_bits_data; // @[VTAShell.scala 53:14]
  assign core_io_vme_rd_4_data_bits_tag = vme_io_vme_rd_4_data_bits_tag; // @[VTAShell.scala 53:14]
  assign core_io_vme_wr_0_cmd_ready = vme_io_vme_wr_0_cmd_ready; // @[VTAShell.scala 53:14]
  assign core_io_vme_wr_0_data_ready = vme_io_vme_wr_0_data_ready; // @[VTAShell.scala 53:14]
  assign core_io_vme_wr_0_ack = vme_io_vme_wr_0_ack; // @[VTAShell.scala 53:14]
endmodule