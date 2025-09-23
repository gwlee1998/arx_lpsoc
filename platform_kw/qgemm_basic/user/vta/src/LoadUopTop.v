module LoadUopTop(
  input          clock,
  input          reset,
  input          io_start,
  output         io_done,
  input  [127:0] io_inst,
  input  [31:0]  io_baddr,
  input          io_vme_rd_cmd_ready,
  output         io_vme_rd_cmd_valid,
  output [31:0]  io_vme_rd_cmd_bits_addr,
  output [7:0]   io_vme_rd_cmd_bits_len,
  output [20:0]  io_vme_rd_cmd_bits_tag,
  input          io_vme_rd_data_valid,
  input  [63:0]  io_vme_rd_data_bits_data,
  input  [20:0]  io_vme_rd_data_bits_tag,
  input          io_vme_rd_data_bits_last,
  input          io_uop_idx_valid,
  input  [10:0]  io_uop_idx_bits,
  output         io_uop_data_valid,
  output [9:0]   io_uop_data_bits_u2,
  output [10:0]  io_uop_data_bits_u1,
  output [10:0]  io_uop_data_bits_u0
);
  wire  loadUop_clock; // @[LoadUop.scala 87:25]
  wire  loadUop_reset; // @[LoadUop.scala 87:25]
  wire  loadUop_io_start; // @[LoadUop.scala 87:25]
  wire  loadUop_io_done; // @[LoadUop.scala 87:25]
  wire [127:0] loadUop_io_inst; // @[LoadUop.scala 87:25]
  wire [31:0] loadUop_io_baddr; // @[LoadUop.scala 87:25]
  wire  loadUop_io_vme_rd_cmd_ready; // @[LoadUop.scala 87:25]
  wire  loadUop_io_vme_rd_cmd_valid; // @[LoadUop.scala 87:25]
  wire [31:0] loadUop_io_vme_rd_cmd_bits_addr; // @[LoadUop.scala 87:25]
  wire [7:0] loadUop_io_vme_rd_cmd_bits_len; // @[LoadUop.scala 87:25]
  wire [20:0] loadUop_io_vme_rd_cmd_bits_tag; // @[LoadUop.scala 87:25]
  wire  loadUop_io_vme_rd_data_valid; // @[LoadUop.scala 87:25]
  wire [63:0] loadUop_io_vme_rd_data_bits_data; // @[LoadUop.scala 87:25]
  wire [20:0] loadUop_io_vme_rd_data_bits_tag; // @[LoadUop.scala 87:25]
  wire  loadUop_io_vme_rd_data_bits_last; // @[LoadUop.scala 87:25]
  wire  loadUop_io_tensor_rd_0_idx_valid; // @[LoadUop.scala 87:25]
  wire [10:0] loadUop_io_tensor_rd_0_idx_bits; // @[LoadUop.scala 87:25]
  wire  loadUop_io_tensor_rd_0_data_valid; // @[LoadUop.scala 87:25]
  wire [31:0] loadUop_io_tensor_rd_0_data_bits_0_0; // @[LoadUop.scala 87:25]
  wire [31:0] _io_uop_data_bits_WIRE_1 = loadUop_io_tensor_rd_0_data_bits_0_0;
  TensorLoadUop loadUop ( // @[LoadUop.scala 87:25]
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
    .io_tensor_rd_0_idx_valid(loadUop_io_tensor_rd_0_idx_valid),
    .io_tensor_rd_0_idx_bits(loadUop_io_tensor_rd_0_idx_bits),
    .io_tensor_rd_0_data_valid(loadUop_io_tensor_rd_0_data_valid),
    .io_tensor_rd_0_data_bits_0_0(loadUop_io_tensor_rd_0_data_bits_0_0)
  );
  assign io_done = loadUop_io_done; // @[LoadUop.scala 91:13]
  assign io_vme_rd_cmd_valid = loadUop_io_vme_rd_cmd_valid; // @[LoadUop.scala 93:23]
  assign io_vme_rd_cmd_bits_addr = loadUop_io_vme_rd_cmd_bits_addr; // @[LoadUop.scala 93:23]
  assign io_vme_rd_cmd_bits_len = loadUop_io_vme_rd_cmd_bits_len; // @[LoadUop.scala 93:23]
  assign io_vme_rd_cmd_bits_tag = loadUop_io_vme_rd_cmd_bits_tag; // @[LoadUop.scala 93:23]
  assign io_uop_data_valid = loadUop_io_tensor_rd_0_data_valid; // @[LoadUop.scala 98:23]
  assign io_uop_data_bits_u2 = _io_uop_data_bits_WIRE_1[31:22]; // @[LoadUop.scala 99:67]
  assign io_uop_data_bits_u1 = _io_uop_data_bits_WIRE_1[21:11]; // @[LoadUop.scala 99:67]
  assign io_uop_data_bits_u0 = _io_uop_data_bits_WIRE_1[10:0]; // @[LoadUop.scala 99:67]
  assign loadUop_clock = clock;
  assign loadUop_reset = reset;
  assign loadUop_io_start = io_start; // @[LoadUop.scala 90:22]
  assign loadUop_io_inst = io_inst; // @[LoadUop.scala 95:21]
  assign loadUop_io_baddr = io_baddr; // @[LoadUop.scala 92:22]
  assign loadUop_io_vme_rd_cmd_ready = io_vme_rd_cmd_ready; // @[LoadUop.scala 93:23]
  assign loadUop_io_vme_rd_data_valid = io_vme_rd_data_valid; // @[LoadUop.scala 93:23]
  assign loadUop_io_vme_rd_data_bits_data = io_vme_rd_data_bits_data; // @[LoadUop.scala 93:23]
  assign loadUop_io_vme_rd_data_bits_tag = io_vme_rd_data_bits_tag; // @[LoadUop.scala 93:23]
  assign loadUop_io_vme_rd_data_bits_last = io_vme_rd_data_bits_last; // @[LoadUop.scala 93:23]
  assign loadUop_io_tensor_rd_0_idx_valid = io_uop_idx_valid; // @[LoadUop.scala 97:33]
  assign loadUop_io_tensor_rd_0_idx_bits = io_uop_idx_bits; // @[LoadUop.scala 97:33]
endmodule