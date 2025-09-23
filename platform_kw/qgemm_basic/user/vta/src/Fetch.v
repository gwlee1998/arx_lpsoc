module Fetch(
  input          clock,
  input          reset,
  input          io_launch,
  input  [31:0]  io_ins_baddr,
  input  [31:0]  io_ins_count,
  input          io_vme_rd_cmd_ready,
  output         io_vme_rd_cmd_valid,
  output [31:0]  io_vme_rd_cmd_bits_addr,
  output [7:0]   io_vme_rd_cmd_bits_len,
  output         io_vme_rd_data_ready,
  input          io_vme_rd_data_valid,
  input  [63:0]  io_vme_rd_data_bits_data,
  input          io_inst_ld_ready,
  output         io_inst_ld_valid,
  output [127:0] io_inst_ld_bits,
  input          io_inst_co_ready,
  output         io_inst_co_valid,
  output [127:0] io_inst_co_bits,
  input          io_inst_st_ready,
  output         io_inst_st_valid,
  output [127:0] io_inst_st_bits
);
  wire  fetch_clock; // @[Fetch.scala 70:23]
  wire  fetch_reset; // @[Fetch.scala 70:23]
  wire  fetch_io_launch; // @[Fetch.scala 70:23]
  wire [31:0] fetch_io_ins_baddr; // @[Fetch.scala 70:23]
  wire [31:0] fetch_io_ins_count; // @[Fetch.scala 70:23]
  wire  fetch_io_vme_rd_cmd_ready; // @[Fetch.scala 70:23]
  wire  fetch_io_vme_rd_cmd_valid; // @[Fetch.scala 70:23]
  wire [31:0] fetch_io_vme_rd_cmd_bits_addr; // @[Fetch.scala 70:23]
  wire [7:0] fetch_io_vme_rd_cmd_bits_len; // @[Fetch.scala 70:23]
  wire  fetch_io_vme_rd_data_ready; // @[Fetch.scala 70:23]
  wire  fetch_io_vme_rd_data_valid; // @[Fetch.scala 70:23]
  wire [63:0] fetch_io_vme_rd_data_bits_data; // @[Fetch.scala 70:23]
  wire  fetch_io_inst_ld_ready; // @[Fetch.scala 70:23]
  wire  fetch_io_inst_ld_valid; // @[Fetch.scala 70:23]
  wire [127:0] fetch_io_inst_ld_bits; // @[Fetch.scala 70:23]
  wire  fetch_io_inst_co_ready; // @[Fetch.scala 70:23]
  wire  fetch_io_inst_co_valid; // @[Fetch.scala 70:23]
  wire [127:0] fetch_io_inst_co_bits; // @[Fetch.scala 70:23]
  wire  fetch_io_inst_st_ready; // @[Fetch.scala 70:23]
  wire  fetch_io_inst_st_valid; // @[Fetch.scala 70:23]
  wire [127:0] fetch_io_inst_st_bits; // @[Fetch.scala 70:23]
  Fetch64Bit fetch ( // @[Fetch.scala 70:23]
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
  assign io_vme_rd_cmd_valid = fetch_io_vme_rd_cmd_valid; // @[Fetch.scala 71:8]
  assign io_vme_rd_cmd_bits_addr = fetch_io_vme_rd_cmd_bits_addr; // @[Fetch.scala 71:8]
  assign io_vme_rd_cmd_bits_len = fetch_io_vme_rd_cmd_bits_len; // @[Fetch.scala 71:8]
  assign io_vme_rd_data_ready = fetch_io_vme_rd_data_ready; // @[Fetch.scala 71:8]
  assign io_inst_ld_valid = fetch_io_inst_ld_valid; // @[Fetch.scala 71:8]
  assign io_inst_ld_bits = fetch_io_inst_ld_bits; // @[Fetch.scala 71:8]
  assign io_inst_co_valid = fetch_io_inst_co_valid; // @[Fetch.scala 71:8]
  assign io_inst_co_bits = fetch_io_inst_co_bits; // @[Fetch.scala 71:8]
  assign io_inst_st_valid = fetch_io_inst_st_valid; // @[Fetch.scala 71:8]
  assign io_inst_st_bits = fetch_io_inst_st_bits; // @[Fetch.scala 71:8]
  assign fetch_clock = clock;
  assign fetch_reset = reset;
  assign fetch_io_launch = io_launch; // @[Fetch.scala 71:8]
  assign fetch_io_ins_baddr = io_ins_baddr; // @[Fetch.scala 71:8]
  assign fetch_io_ins_count = io_ins_count; // @[Fetch.scala 71:8]
  assign fetch_io_vme_rd_cmd_ready = io_vme_rd_cmd_ready; // @[Fetch.scala 71:8]
  assign fetch_io_vme_rd_data_valid = io_vme_rd_data_valid; // @[Fetch.scala 71:8]
  assign fetch_io_vme_rd_data_bits_data = io_vme_rd_data_bits_data; // @[Fetch.scala 71:8]
  assign fetch_io_inst_ld_ready = io_inst_ld_ready; // @[Fetch.scala 71:8]
  assign fetch_io_inst_co_ready = io_inst_co_ready; // @[Fetch.scala 71:8]
  assign fetch_io_inst_st_ready = io_inst_st_ready; // @[Fetch.scala 71:8]
endmodule