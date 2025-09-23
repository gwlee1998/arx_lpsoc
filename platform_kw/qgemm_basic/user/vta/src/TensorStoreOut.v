module TensorStoreOut(
  input          clock,
  input          reset,
  input          io_start,
  output         io_done,
  input  [127:0] io_inst,
  input  [31:0]  io_baddr,
  input          io_vme_wr_cmd_ready,
  output         io_vme_wr_cmd_valid,
  output [31:0]  io_vme_wr_cmd_bits_addr,
  output [7:0]   io_vme_wr_cmd_bits_len,
  input          io_vme_wr_data_ready,
  output         io_vme_wr_data_valid,
  output [63:0]  io_vme_wr_data_bits_data,
  input          io_vme_wr_ack,
  input          io_tensor_wr_0_valid,
  input  [10:0]  io_tensor_wr_0_bits_idx,
  input  [31:0]  io_tensor_wr_0_bits_data_0_0,
  input  [31:0]  io_tensor_wr_0_bits_data_0_1,
  input  [31:0]  io_tensor_wr_0_bits_data_0_2,
  input  [31:0]  io_tensor_wr_0_bits_data_0_3,
  input  [31:0]  io_tensor_wr_0_bits_data_0_4,
  input  [31:0]  io_tensor_wr_0_bits_data_0_5,
  input  [31:0]  io_tensor_wr_0_bits_data_0_6,
  input  [31:0]  io_tensor_wr_0_bits_data_0_7,
  input  [31:0]  io_tensor_wr_0_bits_data_0_8,
  input  [31:0]  io_tensor_wr_0_bits_data_0_9,
  input  [31:0]  io_tensor_wr_0_bits_data_0_10,
  input  [31:0]  io_tensor_wr_0_bits_data_0_11,
  input  [31:0]  io_tensor_wr_0_bits_data_0_12,
  input  [31:0]  io_tensor_wr_0_bits_data_0_13,
  input  [31:0]  io_tensor_wr_0_bits_data_0_14,
  input  [31:0]  io_tensor_wr_0_bits_data_0_15
);
  wire  tensorStore_clock; // @[TensorStore.scala 59:29]
  wire  tensorStore_reset; // @[TensorStore.scala 59:29]
  wire  tensorStore_io_start; // @[TensorStore.scala 59:29]
  wire  tensorStore_io_done; // @[TensorStore.scala 59:29]
  wire [127:0] tensorStore_io_inst; // @[TensorStore.scala 59:29]
  wire [31:0] tensorStore_io_baddr; // @[TensorStore.scala 59:29]
  wire  tensorStore_io_vme_wr_cmd_ready; // @[TensorStore.scala 59:29]
  wire  tensorStore_io_vme_wr_cmd_valid; // @[TensorStore.scala 59:29]
  wire [31:0] tensorStore_io_vme_wr_cmd_bits_addr; // @[TensorStore.scala 59:29]
  wire [7:0] tensorStore_io_vme_wr_cmd_bits_len; // @[TensorStore.scala 59:29]
  wire  tensorStore_io_vme_wr_data_ready; // @[TensorStore.scala 59:29]
  wire  tensorStore_io_vme_wr_data_valid; // @[TensorStore.scala 59:29]
  wire [63:0] tensorStore_io_vme_wr_data_bits_data; // @[TensorStore.scala 59:29]
  wire  tensorStore_io_vme_wr_ack; // @[TensorStore.scala 59:29]
  wire  tensorStore_io_tensor_wr_0_valid; // @[TensorStore.scala 59:29]
  wire [10:0] tensorStore_io_tensor_wr_0_bits_idx; // @[TensorStore.scala 59:29]
  wire [31:0] tensorStore_io_tensor_wr_0_bits_data_0_0; // @[TensorStore.scala 59:29]
  wire [31:0] tensorStore_io_tensor_wr_0_bits_data_0_1; // @[TensorStore.scala 59:29]
  wire [31:0] tensorStore_io_tensor_wr_0_bits_data_0_2; // @[TensorStore.scala 59:29]
  wire [31:0] tensorStore_io_tensor_wr_0_bits_data_0_3; // @[TensorStore.scala 59:29]
  wire [31:0] tensorStore_io_tensor_wr_0_bits_data_0_4; // @[TensorStore.scala 59:29]
  wire [31:0] tensorStore_io_tensor_wr_0_bits_data_0_5; // @[TensorStore.scala 59:29]
  wire [31:0] tensorStore_io_tensor_wr_0_bits_data_0_6; // @[TensorStore.scala 59:29]
  wire [31:0] tensorStore_io_tensor_wr_0_bits_data_0_7; // @[TensorStore.scala 59:29]
  wire [31:0] tensorStore_io_tensor_wr_0_bits_data_0_8; // @[TensorStore.scala 59:29]
  wire [31:0] tensorStore_io_tensor_wr_0_bits_data_0_9; // @[TensorStore.scala 59:29]
  wire [31:0] tensorStore_io_tensor_wr_0_bits_data_0_10; // @[TensorStore.scala 59:29]
  wire [31:0] tensorStore_io_tensor_wr_0_bits_data_0_11; // @[TensorStore.scala 59:29]
  wire [31:0] tensorStore_io_tensor_wr_0_bits_data_0_12; // @[TensorStore.scala 59:29]
  wire [31:0] tensorStore_io_tensor_wr_0_bits_data_0_13; // @[TensorStore.scala 59:29]
  wire [31:0] tensorStore_io_tensor_wr_0_bits_data_0_14; // @[TensorStore.scala 59:29]
  wire [31:0] tensorStore_io_tensor_wr_0_bits_data_0_15; // @[TensorStore.scala 59:29]
  TensorStoreNarrowVME tensorStore ( // @[TensorStore.scala 59:29]
    .clock(tensorStore_clock),
    .reset(tensorStore_reset),
    .io_start(tensorStore_io_start),
    .io_done(tensorStore_io_done),
    .io_inst(tensorStore_io_inst),
    .io_baddr(tensorStore_io_baddr),
    .io_vme_wr_cmd_ready(tensorStore_io_vme_wr_cmd_ready),
    .io_vme_wr_cmd_valid(tensorStore_io_vme_wr_cmd_valid),
    .io_vme_wr_cmd_bits_addr(tensorStore_io_vme_wr_cmd_bits_addr),
    .io_vme_wr_cmd_bits_len(tensorStore_io_vme_wr_cmd_bits_len),
    .io_vme_wr_data_ready(tensorStore_io_vme_wr_data_ready),
    .io_vme_wr_data_valid(tensorStore_io_vme_wr_data_valid),
    .io_vme_wr_data_bits_data(tensorStore_io_vme_wr_data_bits_data),
    .io_vme_wr_ack(tensorStore_io_vme_wr_ack),
    .io_tensor_wr_0_valid(tensorStore_io_tensor_wr_0_valid),
    .io_tensor_wr_0_bits_idx(tensorStore_io_tensor_wr_0_bits_idx),
    .io_tensor_wr_0_bits_data_0_0(tensorStore_io_tensor_wr_0_bits_data_0_0),
    .io_tensor_wr_0_bits_data_0_1(tensorStore_io_tensor_wr_0_bits_data_0_1),
    .io_tensor_wr_0_bits_data_0_2(tensorStore_io_tensor_wr_0_bits_data_0_2),
    .io_tensor_wr_0_bits_data_0_3(tensorStore_io_tensor_wr_0_bits_data_0_3),
    .io_tensor_wr_0_bits_data_0_4(tensorStore_io_tensor_wr_0_bits_data_0_4),
    .io_tensor_wr_0_bits_data_0_5(tensorStore_io_tensor_wr_0_bits_data_0_5),
    .io_tensor_wr_0_bits_data_0_6(tensorStore_io_tensor_wr_0_bits_data_0_6),
    .io_tensor_wr_0_bits_data_0_7(tensorStore_io_tensor_wr_0_bits_data_0_7),
    .io_tensor_wr_0_bits_data_0_8(tensorStore_io_tensor_wr_0_bits_data_0_8),
    .io_tensor_wr_0_bits_data_0_9(tensorStore_io_tensor_wr_0_bits_data_0_9),
    .io_tensor_wr_0_bits_data_0_10(tensorStore_io_tensor_wr_0_bits_data_0_10),
    .io_tensor_wr_0_bits_data_0_11(tensorStore_io_tensor_wr_0_bits_data_0_11),
    .io_tensor_wr_0_bits_data_0_12(tensorStore_io_tensor_wr_0_bits_data_0_12),
    .io_tensor_wr_0_bits_data_0_13(tensorStore_io_tensor_wr_0_bits_data_0_13),
    .io_tensor_wr_0_bits_data_0_14(tensorStore_io_tensor_wr_0_bits_data_0_14),
    .io_tensor_wr_0_bits_data_0_15(tensorStore_io_tensor_wr_0_bits_data_0_15)
  );
  assign io_done = tensorStore_io_done; // @[TensorStore.scala 60:8]
  assign io_vme_wr_cmd_valid = tensorStore_io_vme_wr_cmd_valid; // @[TensorStore.scala 60:8]
  assign io_vme_wr_cmd_bits_addr = tensorStore_io_vme_wr_cmd_bits_addr; // @[TensorStore.scala 60:8]
  assign io_vme_wr_cmd_bits_len = tensorStore_io_vme_wr_cmd_bits_len; // @[TensorStore.scala 60:8]
  assign io_vme_wr_data_valid = tensorStore_io_vme_wr_data_valid; // @[TensorStore.scala 60:8]
  assign io_vme_wr_data_bits_data = tensorStore_io_vme_wr_data_bits_data; // @[TensorStore.scala 60:8]
  assign tensorStore_clock = clock;
  assign tensorStore_reset = reset;
  assign tensorStore_io_start = io_start; // @[TensorStore.scala 60:8]
  assign tensorStore_io_inst = io_inst; // @[TensorStore.scala 60:8]
  assign tensorStore_io_baddr = io_baddr; // @[TensorStore.scala 60:8]
  assign tensorStore_io_vme_wr_cmd_ready = io_vme_wr_cmd_ready; // @[TensorStore.scala 60:8]
  assign tensorStore_io_vme_wr_data_ready = io_vme_wr_data_ready; // @[TensorStore.scala 60:8]
  assign tensorStore_io_vme_wr_ack = io_vme_wr_ack; // @[TensorStore.scala 60:8]
  assign tensorStore_io_tensor_wr_0_valid = io_tensor_wr_0_valid; // @[TensorStore.scala 60:8]
  assign tensorStore_io_tensor_wr_0_bits_idx = io_tensor_wr_0_bits_idx; // @[TensorStore.scala 60:8]
  assign tensorStore_io_tensor_wr_0_bits_data_0_0 = io_tensor_wr_0_bits_data_0_0; // @[TensorStore.scala 60:8]
  assign tensorStore_io_tensor_wr_0_bits_data_0_1 = io_tensor_wr_0_bits_data_0_1; // @[TensorStore.scala 60:8]
  assign tensorStore_io_tensor_wr_0_bits_data_0_2 = io_tensor_wr_0_bits_data_0_2; // @[TensorStore.scala 60:8]
  assign tensorStore_io_tensor_wr_0_bits_data_0_3 = io_tensor_wr_0_bits_data_0_3; // @[TensorStore.scala 60:8]
  assign tensorStore_io_tensor_wr_0_bits_data_0_4 = io_tensor_wr_0_bits_data_0_4; // @[TensorStore.scala 60:8]
  assign tensorStore_io_tensor_wr_0_bits_data_0_5 = io_tensor_wr_0_bits_data_0_5; // @[TensorStore.scala 60:8]
  assign tensorStore_io_tensor_wr_0_bits_data_0_6 = io_tensor_wr_0_bits_data_0_6; // @[TensorStore.scala 60:8]
  assign tensorStore_io_tensor_wr_0_bits_data_0_7 = io_tensor_wr_0_bits_data_0_7; // @[TensorStore.scala 60:8]
  assign tensorStore_io_tensor_wr_0_bits_data_0_8 = io_tensor_wr_0_bits_data_0_8; // @[TensorStore.scala 60:8]
  assign tensorStore_io_tensor_wr_0_bits_data_0_9 = io_tensor_wr_0_bits_data_0_9; // @[TensorStore.scala 60:8]
  assign tensorStore_io_tensor_wr_0_bits_data_0_10 = io_tensor_wr_0_bits_data_0_10; // @[TensorStore.scala 60:8]
  assign tensorStore_io_tensor_wr_0_bits_data_0_11 = io_tensor_wr_0_bits_data_0_11; // @[TensorStore.scala 60:8]
  assign tensorStore_io_tensor_wr_0_bits_data_0_12 = io_tensor_wr_0_bits_data_0_12; // @[TensorStore.scala 60:8]
  assign tensorStore_io_tensor_wr_0_bits_data_0_13 = io_tensor_wr_0_bits_data_0_13; // @[TensorStore.scala 60:8]
  assign tensorStore_io_tensor_wr_0_bits_data_0_14 = io_tensor_wr_0_bits_data_0_14; // @[TensorStore.scala 60:8]
  assign tensorStore_io_tensor_wr_0_bits_data_0_15 = io_tensor_wr_0_bits_data_0_15; // @[TensorStore.scala 60:8]
endmodule