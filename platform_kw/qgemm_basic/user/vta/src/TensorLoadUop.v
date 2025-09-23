module TensorLoadUop(
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
  input          io_tensor_rd_0_idx_valid,
  input  [10:0]  io_tensor_rd_0_idx_bits,
  output         io_tensor_rd_0_data_valid,
  output [31:0]  io_tensor_rd_0_data_bits_0_0
);
  wire  tensorLoad_clock; // @[TensorLoad.scala 65:28]
  wire  tensorLoad_reset; // @[TensorLoad.scala 65:28]
  wire  tensorLoad_io_start; // @[TensorLoad.scala 65:28]
  wire  tensorLoad_io_done; // @[TensorLoad.scala 65:28]
  wire [127:0] tensorLoad_io_inst; // @[TensorLoad.scala 65:28]
  wire [31:0] tensorLoad_io_baddr; // @[TensorLoad.scala 65:28]
  wire  tensorLoad_io_vme_rd_cmd_ready; // @[TensorLoad.scala 65:28]
  wire  tensorLoad_io_vme_rd_cmd_valid; // @[TensorLoad.scala 65:28]
  wire [31:0] tensorLoad_io_vme_rd_cmd_bits_addr; // @[TensorLoad.scala 65:28]
  wire [7:0] tensorLoad_io_vme_rd_cmd_bits_len; // @[TensorLoad.scala 65:28]
  wire [20:0] tensorLoad_io_vme_rd_cmd_bits_tag; // @[TensorLoad.scala 65:28]
  wire  tensorLoad_io_vme_rd_data_ready; // @[TensorLoad.scala 65:28]
  wire  tensorLoad_io_vme_rd_data_valid; // @[TensorLoad.scala 65:28]
  wire [63:0] tensorLoad_io_vme_rd_data_bits_data; // @[TensorLoad.scala 65:28]
  wire [20:0] tensorLoad_io_vme_rd_data_bits_tag; // @[TensorLoad.scala 65:28]
  wire  tensorLoad_io_vme_rd_data_bits_last; // @[TensorLoad.scala 65:28]
  wire  tensorLoad_io_tensor_rd_0_idx_valid; // @[TensorLoad.scala 65:28]
  wire [10:0] tensorLoad_io_tensor_rd_0_idx_bits; // @[TensorLoad.scala 65:28]
  wire  tensorLoad_io_tensor_rd_0_data_valid; // @[TensorLoad.scala 65:28]
  wire [31:0] tensorLoad_io_tensor_rd_0_data_bits_0_0; // @[TensorLoad.scala 65:28]
  TensorLoadWideVME tensorLoad ( // @[TensorLoad.scala 65:28]
    .clock(tensorLoad_clock),
    .reset(tensorLoad_reset),
    .io_start(tensorLoad_io_start),
    .io_done(tensorLoad_io_done),
    .io_inst(tensorLoad_io_inst),
    .io_baddr(tensorLoad_io_baddr),
    .io_vme_rd_cmd_ready(tensorLoad_io_vme_rd_cmd_ready),
    .io_vme_rd_cmd_valid(tensorLoad_io_vme_rd_cmd_valid),
    .io_vme_rd_cmd_bits_addr(tensorLoad_io_vme_rd_cmd_bits_addr),
    .io_vme_rd_cmd_bits_len(tensorLoad_io_vme_rd_cmd_bits_len),
    .io_vme_rd_cmd_bits_tag(tensorLoad_io_vme_rd_cmd_bits_tag),
    .io_vme_rd_data_ready(tensorLoad_io_vme_rd_data_ready),
    .io_vme_rd_data_valid(tensorLoad_io_vme_rd_data_valid),
    .io_vme_rd_data_bits_data(tensorLoad_io_vme_rd_data_bits_data),
    .io_vme_rd_data_bits_tag(tensorLoad_io_vme_rd_data_bits_tag),
    .io_vme_rd_data_bits_last(tensorLoad_io_vme_rd_data_bits_last),
    .io_tensor_rd_0_idx_valid(tensorLoad_io_tensor_rd_0_idx_valid),
    .io_tensor_rd_0_idx_bits(tensorLoad_io_tensor_rd_0_idx_bits),
    .io_tensor_rd_0_data_valid(tensorLoad_io_tensor_rd_0_data_valid),
    .io_tensor_rd_0_data_bits_0_0(tensorLoad_io_tensor_rd_0_data_bits_0_0)
  );
  assign io_done = tensorLoad_io_done; // @[TensorLoad.scala 66:8]
  assign io_vme_rd_cmd_valid = tensorLoad_io_vme_rd_cmd_valid; // @[TensorLoad.scala 66:8]
  assign io_vme_rd_cmd_bits_addr = tensorLoad_io_vme_rd_cmd_bits_addr; // @[TensorLoad.scala 66:8]
  assign io_vme_rd_cmd_bits_len = tensorLoad_io_vme_rd_cmd_bits_len; // @[TensorLoad.scala 66:8]
  assign io_vme_rd_cmd_bits_tag = tensorLoad_io_vme_rd_cmd_bits_tag; // @[TensorLoad.scala 66:8]
  assign io_tensor_rd_0_data_valid = tensorLoad_io_tensor_rd_0_data_valid; // @[TensorLoad.scala 66:8]
  assign io_tensor_rd_0_data_bits_0_0 = tensorLoad_io_tensor_rd_0_data_bits_0_0; // @[TensorLoad.scala 66:8]
  assign tensorLoad_clock = clock;
  assign tensorLoad_reset = reset;
  assign tensorLoad_io_start = io_start; // @[TensorLoad.scala 66:8]
  assign tensorLoad_io_inst = io_inst; // @[TensorLoad.scala 66:8]
  assign tensorLoad_io_baddr = io_baddr; // @[TensorLoad.scala 66:8]
  assign tensorLoad_io_vme_rd_cmd_ready = io_vme_rd_cmd_ready; // @[TensorLoad.scala 66:8]
  assign tensorLoad_io_vme_rd_data_valid = io_vme_rd_data_valid; // @[TensorLoad.scala 66:8]
  assign tensorLoad_io_vme_rd_data_bits_data = io_vme_rd_data_bits_data; // @[TensorLoad.scala 66:8]
  assign tensorLoad_io_vme_rd_data_bits_tag = io_vme_rd_data_bits_tag; // @[TensorLoad.scala 66:8]
  assign tensorLoad_io_vme_rd_data_bits_last = io_vme_rd_data_bits_last; // @[TensorLoad.scala 66:8]
  assign tensorLoad_io_tensor_rd_0_idx_valid = io_tensor_rd_0_idx_valid; // @[TensorLoad.scala 66:8]
  assign tensorLoad_io_tensor_rd_0_idx_bits = io_tensor_rd_0_idx_bits; // @[TensorLoad.scala 66:8]
endmodule