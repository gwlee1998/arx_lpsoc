module TensorLoadAcc(
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
  input          io_tensor_rd_0_idx_valid,
  input  [10:0]  io_tensor_rd_0_idx_bits,
  output         io_tensor_rd_0_data_valid,
  output [31:0]  io_tensor_rd_0_data_bits_0_0,
  output [31:0]  io_tensor_rd_0_data_bits_0_1,
  output [31:0]  io_tensor_rd_0_data_bits_0_2,
  output [31:0]  io_tensor_rd_0_data_bits_0_3,
  output [31:0]  io_tensor_rd_0_data_bits_0_4,
  output [31:0]  io_tensor_rd_0_data_bits_0_5,
  output [31:0]  io_tensor_rd_0_data_bits_0_6,
  output [31:0]  io_tensor_rd_0_data_bits_0_7,
  output [31:0]  io_tensor_rd_0_data_bits_0_8,
  output [31:0]  io_tensor_rd_0_data_bits_0_9,
  output [31:0]  io_tensor_rd_0_data_bits_0_10,
  output [31:0]  io_tensor_rd_0_data_bits_0_11,
  output [31:0]  io_tensor_rd_0_data_bits_0_12,
  output [31:0]  io_tensor_rd_0_data_bits_0_13,
  output [31:0]  io_tensor_rd_0_data_bits_0_14,
  output [31:0]  io_tensor_rd_0_data_bits_0_15,
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
  wire  tensorLoad_clock; // @[TensorLoad.scala 71:28]
  wire  tensorLoad_reset; // @[TensorLoad.scala 71:28]
  wire  tensorLoad_io_start; // @[TensorLoad.scala 71:28]
  wire  tensorLoad_io_done; // @[TensorLoad.scala 71:28]
  wire [127:0] tensorLoad_io_inst; // @[TensorLoad.scala 71:28]
  wire [31:0] tensorLoad_io_baddr; // @[TensorLoad.scala 71:28]
  wire  tensorLoad_io_vme_rd_cmd_ready; // @[TensorLoad.scala 71:28]
  wire  tensorLoad_io_vme_rd_cmd_valid; // @[TensorLoad.scala 71:28]
  wire [31:0] tensorLoad_io_vme_rd_cmd_bits_addr; // @[TensorLoad.scala 71:28]
  wire [7:0] tensorLoad_io_vme_rd_cmd_bits_len; // @[TensorLoad.scala 71:28]
  wire [20:0] tensorLoad_io_vme_rd_cmd_bits_tag; // @[TensorLoad.scala 71:28]
  wire  tensorLoad_io_vme_rd_data_ready; // @[TensorLoad.scala 71:28]
  wire  tensorLoad_io_vme_rd_data_valid; // @[TensorLoad.scala 71:28]
  wire [63:0] tensorLoad_io_vme_rd_data_bits_data; // @[TensorLoad.scala 71:28]
  wire [20:0] tensorLoad_io_vme_rd_data_bits_tag; // @[TensorLoad.scala 71:28]
  wire  tensorLoad_io_tensor_rd_0_idx_valid; // @[TensorLoad.scala 71:28]
  wire [10:0] tensorLoad_io_tensor_rd_0_idx_bits; // @[TensorLoad.scala 71:28]
  wire  tensorLoad_io_tensor_rd_0_data_valid; // @[TensorLoad.scala 71:28]
  wire [31:0] tensorLoad_io_tensor_rd_0_data_bits_0_0; // @[TensorLoad.scala 71:28]
  wire [31:0] tensorLoad_io_tensor_rd_0_data_bits_0_1; // @[TensorLoad.scala 71:28]
  wire [31:0] tensorLoad_io_tensor_rd_0_data_bits_0_2; // @[TensorLoad.scala 71:28]
  wire [31:0] tensorLoad_io_tensor_rd_0_data_bits_0_3; // @[TensorLoad.scala 71:28]
  wire [31:0] tensorLoad_io_tensor_rd_0_data_bits_0_4; // @[TensorLoad.scala 71:28]
  wire [31:0] tensorLoad_io_tensor_rd_0_data_bits_0_5; // @[TensorLoad.scala 71:28]
  wire [31:0] tensorLoad_io_tensor_rd_0_data_bits_0_6; // @[TensorLoad.scala 71:28]
  wire [31:0] tensorLoad_io_tensor_rd_0_data_bits_0_7; // @[TensorLoad.scala 71:28]
  wire [31:0] tensorLoad_io_tensor_rd_0_data_bits_0_8; // @[TensorLoad.scala 71:28]
  wire [31:0] tensorLoad_io_tensor_rd_0_data_bits_0_9; // @[TensorLoad.scala 71:28]
  wire [31:0] tensorLoad_io_tensor_rd_0_data_bits_0_10; // @[TensorLoad.scala 71:28]
  wire [31:0] tensorLoad_io_tensor_rd_0_data_bits_0_11; // @[TensorLoad.scala 71:28]
  wire [31:0] tensorLoad_io_tensor_rd_0_data_bits_0_12; // @[TensorLoad.scala 71:28]
  wire [31:0] tensorLoad_io_tensor_rd_0_data_bits_0_13; // @[TensorLoad.scala 71:28]
  wire [31:0] tensorLoad_io_tensor_rd_0_data_bits_0_14; // @[TensorLoad.scala 71:28]
  wire [31:0] tensorLoad_io_tensor_rd_0_data_bits_0_15; // @[TensorLoad.scala 71:28]
  wire  tensorLoad_io_tensor_wr_0_valid; // @[TensorLoad.scala 71:28]
  wire [10:0] tensorLoad_io_tensor_wr_0_bits_idx; // @[TensorLoad.scala 71:28]
  wire [31:0] tensorLoad_io_tensor_wr_0_bits_data_0_0; // @[TensorLoad.scala 71:28]
  wire [31:0] tensorLoad_io_tensor_wr_0_bits_data_0_1; // @[TensorLoad.scala 71:28]
  wire [31:0] tensorLoad_io_tensor_wr_0_bits_data_0_2; // @[TensorLoad.scala 71:28]
  wire [31:0] tensorLoad_io_tensor_wr_0_bits_data_0_3; // @[TensorLoad.scala 71:28]
  wire [31:0] tensorLoad_io_tensor_wr_0_bits_data_0_4; // @[TensorLoad.scala 71:28]
  wire [31:0] tensorLoad_io_tensor_wr_0_bits_data_0_5; // @[TensorLoad.scala 71:28]
  wire [31:0] tensorLoad_io_tensor_wr_0_bits_data_0_6; // @[TensorLoad.scala 71:28]
  wire [31:0] tensorLoad_io_tensor_wr_0_bits_data_0_7; // @[TensorLoad.scala 71:28]
  wire [31:0] tensorLoad_io_tensor_wr_0_bits_data_0_8; // @[TensorLoad.scala 71:28]
  wire [31:0] tensorLoad_io_tensor_wr_0_bits_data_0_9; // @[TensorLoad.scala 71:28]
  wire [31:0] tensorLoad_io_tensor_wr_0_bits_data_0_10; // @[TensorLoad.scala 71:28]
  wire [31:0] tensorLoad_io_tensor_wr_0_bits_data_0_11; // @[TensorLoad.scala 71:28]
  wire [31:0] tensorLoad_io_tensor_wr_0_bits_data_0_12; // @[TensorLoad.scala 71:28]
  wire [31:0] tensorLoad_io_tensor_wr_0_bits_data_0_13; // @[TensorLoad.scala 71:28]
  wire [31:0] tensorLoad_io_tensor_wr_0_bits_data_0_14; // @[TensorLoad.scala 71:28]
  wire [31:0] tensorLoad_io_tensor_wr_0_bits_data_0_15; // @[TensorLoad.scala 71:28]
  TensorLoadNarrowVME_2 tensorLoad ( // @[TensorLoad.scala 71:28]
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
    .io_tensor_rd_0_idx_valid(tensorLoad_io_tensor_rd_0_idx_valid),
    .io_tensor_rd_0_idx_bits(tensorLoad_io_tensor_rd_0_idx_bits),
    .io_tensor_rd_0_data_valid(tensorLoad_io_tensor_rd_0_data_valid),
    .io_tensor_rd_0_data_bits_0_0(tensorLoad_io_tensor_rd_0_data_bits_0_0),
    .io_tensor_rd_0_data_bits_0_1(tensorLoad_io_tensor_rd_0_data_bits_0_1),
    .io_tensor_rd_0_data_bits_0_2(tensorLoad_io_tensor_rd_0_data_bits_0_2),
    .io_tensor_rd_0_data_bits_0_3(tensorLoad_io_tensor_rd_0_data_bits_0_3),
    .io_tensor_rd_0_data_bits_0_4(tensorLoad_io_tensor_rd_0_data_bits_0_4),
    .io_tensor_rd_0_data_bits_0_5(tensorLoad_io_tensor_rd_0_data_bits_0_5),
    .io_tensor_rd_0_data_bits_0_6(tensorLoad_io_tensor_rd_0_data_bits_0_6),
    .io_tensor_rd_0_data_bits_0_7(tensorLoad_io_tensor_rd_0_data_bits_0_7),
    .io_tensor_rd_0_data_bits_0_8(tensorLoad_io_tensor_rd_0_data_bits_0_8),
    .io_tensor_rd_0_data_bits_0_9(tensorLoad_io_tensor_rd_0_data_bits_0_9),
    .io_tensor_rd_0_data_bits_0_10(tensorLoad_io_tensor_rd_0_data_bits_0_10),
    .io_tensor_rd_0_data_bits_0_11(tensorLoad_io_tensor_rd_0_data_bits_0_11),
    .io_tensor_rd_0_data_bits_0_12(tensorLoad_io_tensor_rd_0_data_bits_0_12),
    .io_tensor_rd_0_data_bits_0_13(tensorLoad_io_tensor_rd_0_data_bits_0_13),
    .io_tensor_rd_0_data_bits_0_14(tensorLoad_io_tensor_rd_0_data_bits_0_14),
    .io_tensor_rd_0_data_bits_0_15(tensorLoad_io_tensor_rd_0_data_bits_0_15),
    .io_tensor_wr_0_valid(tensorLoad_io_tensor_wr_0_valid),
    .io_tensor_wr_0_bits_idx(tensorLoad_io_tensor_wr_0_bits_idx),
    .io_tensor_wr_0_bits_data_0_0(tensorLoad_io_tensor_wr_0_bits_data_0_0),
    .io_tensor_wr_0_bits_data_0_1(tensorLoad_io_tensor_wr_0_bits_data_0_1),
    .io_tensor_wr_0_bits_data_0_2(tensorLoad_io_tensor_wr_0_bits_data_0_2),
    .io_tensor_wr_0_bits_data_0_3(tensorLoad_io_tensor_wr_0_bits_data_0_3),
    .io_tensor_wr_0_bits_data_0_4(tensorLoad_io_tensor_wr_0_bits_data_0_4),
    .io_tensor_wr_0_bits_data_0_5(tensorLoad_io_tensor_wr_0_bits_data_0_5),
    .io_tensor_wr_0_bits_data_0_6(tensorLoad_io_tensor_wr_0_bits_data_0_6),
    .io_tensor_wr_0_bits_data_0_7(tensorLoad_io_tensor_wr_0_bits_data_0_7),
    .io_tensor_wr_0_bits_data_0_8(tensorLoad_io_tensor_wr_0_bits_data_0_8),
    .io_tensor_wr_0_bits_data_0_9(tensorLoad_io_tensor_wr_0_bits_data_0_9),
    .io_tensor_wr_0_bits_data_0_10(tensorLoad_io_tensor_wr_0_bits_data_0_10),
    .io_tensor_wr_0_bits_data_0_11(tensorLoad_io_tensor_wr_0_bits_data_0_11),
    .io_tensor_wr_0_bits_data_0_12(tensorLoad_io_tensor_wr_0_bits_data_0_12),
    .io_tensor_wr_0_bits_data_0_13(tensorLoad_io_tensor_wr_0_bits_data_0_13),
    .io_tensor_wr_0_bits_data_0_14(tensorLoad_io_tensor_wr_0_bits_data_0_14),
    .io_tensor_wr_0_bits_data_0_15(tensorLoad_io_tensor_wr_0_bits_data_0_15)
  );
  assign io_done = tensorLoad_io_done; // @[TensorLoad.scala 72:8]
  assign io_vme_rd_cmd_valid = tensorLoad_io_vme_rd_cmd_valid; // @[TensorLoad.scala 72:8]
  assign io_vme_rd_cmd_bits_addr = tensorLoad_io_vme_rd_cmd_bits_addr; // @[TensorLoad.scala 72:8]
  assign io_vme_rd_cmd_bits_len = tensorLoad_io_vme_rd_cmd_bits_len; // @[TensorLoad.scala 72:8]
  assign io_vme_rd_cmd_bits_tag = tensorLoad_io_vme_rd_cmd_bits_tag; // @[TensorLoad.scala 72:8]
  assign io_tensor_rd_0_data_valid = tensorLoad_io_tensor_rd_0_data_valid; // @[TensorLoad.scala 72:8]
  assign io_tensor_rd_0_data_bits_0_0 = tensorLoad_io_tensor_rd_0_data_bits_0_0; // @[TensorLoad.scala 72:8]
  assign io_tensor_rd_0_data_bits_0_1 = tensorLoad_io_tensor_rd_0_data_bits_0_1; // @[TensorLoad.scala 72:8]
  assign io_tensor_rd_0_data_bits_0_2 = tensorLoad_io_tensor_rd_0_data_bits_0_2; // @[TensorLoad.scala 72:8]
  assign io_tensor_rd_0_data_bits_0_3 = tensorLoad_io_tensor_rd_0_data_bits_0_3; // @[TensorLoad.scala 72:8]
  assign io_tensor_rd_0_data_bits_0_4 = tensorLoad_io_tensor_rd_0_data_bits_0_4; // @[TensorLoad.scala 72:8]
  assign io_tensor_rd_0_data_bits_0_5 = tensorLoad_io_tensor_rd_0_data_bits_0_5; // @[TensorLoad.scala 72:8]
  assign io_tensor_rd_0_data_bits_0_6 = tensorLoad_io_tensor_rd_0_data_bits_0_6; // @[TensorLoad.scala 72:8]
  assign io_tensor_rd_0_data_bits_0_7 = tensorLoad_io_tensor_rd_0_data_bits_0_7; // @[TensorLoad.scala 72:8]
  assign io_tensor_rd_0_data_bits_0_8 = tensorLoad_io_tensor_rd_0_data_bits_0_8; // @[TensorLoad.scala 72:8]
  assign io_tensor_rd_0_data_bits_0_9 = tensorLoad_io_tensor_rd_0_data_bits_0_9; // @[TensorLoad.scala 72:8]
  assign io_tensor_rd_0_data_bits_0_10 = tensorLoad_io_tensor_rd_0_data_bits_0_10; // @[TensorLoad.scala 72:8]
  assign io_tensor_rd_0_data_bits_0_11 = tensorLoad_io_tensor_rd_0_data_bits_0_11; // @[TensorLoad.scala 72:8]
  assign io_tensor_rd_0_data_bits_0_12 = tensorLoad_io_tensor_rd_0_data_bits_0_12; // @[TensorLoad.scala 72:8]
  assign io_tensor_rd_0_data_bits_0_13 = tensorLoad_io_tensor_rd_0_data_bits_0_13; // @[TensorLoad.scala 72:8]
  assign io_tensor_rd_0_data_bits_0_14 = tensorLoad_io_tensor_rd_0_data_bits_0_14; // @[TensorLoad.scala 72:8]
  assign io_tensor_rd_0_data_bits_0_15 = tensorLoad_io_tensor_rd_0_data_bits_0_15; // @[TensorLoad.scala 72:8]
  assign tensorLoad_clock = clock;
  assign tensorLoad_reset = reset;
  assign tensorLoad_io_start = io_start; // @[TensorLoad.scala 72:8]
  assign tensorLoad_io_inst = io_inst; // @[TensorLoad.scala 72:8]
  assign tensorLoad_io_baddr = io_baddr; // @[TensorLoad.scala 72:8]
  assign tensorLoad_io_vme_rd_cmd_ready = io_vme_rd_cmd_ready; // @[TensorLoad.scala 72:8]
  assign tensorLoad_io_vme_rd_data_valid = io_vme_rd_data_valid; // @[TensorLoad.scala 72:8]
  assign tensorLoad_io_vme_rd_data_bits_data = io_vme_rd_data_bits_data; // @[TensorLoad.scala 72:8]
  assign tensorLoad_io_vme_rd_data_bits_tag = io_vme_rd_data_bits_tag; // @[TensorLoad.scala 72:8]
  assign tensorLoad_io_tensor_rd_0_idx_valid = io_tensor_rd_0_idx_valid; // @[TensorLoad.scala 72:8]
  assign tensorLoad_io_tensor_rd_0_idx_bits = io_tensor_rd_0_idx_bits; // @[TensorLoad.scala 72:8]
  assign tensorLoad_io_tensor_wr_0_valid = io_tensor_wr_0_valid; // @[TensorLoad.scala 72:8]
  assign tensorLoad_io_tensor_wr_0_bits_idx = io_tensor_wr_0_bits_idx; // @[TensorLoad.scala 72:8]
  assign tensorLoad_io_tensor_wr_0_bits_data_0_0 = io_tensor_wr_0_bits_data_0_0; // @[TensorLoad.scala 72:8]
  assign tensorLoad_io_tensor_wr_0_bits_data_0_1 = io_tensor_wr_0_bits_data_0_1; // @[TensorLoad.scala 72:8]
  assign tensorLoad_io_tensor_wr_0_bits_data_0_2 = io_tensor_wr_0_bits_data_0_2; // @[TensorLoad.scala 72:8]
  assign tensorLoad_io_tensor_wr_0_bits_data_0_3 = io_tensor_wr_0_bits_data_0_3; // @[TensorLoad.scala 72:8]
  assign tensorLoad_io_tensor_wr_0_bits_data_0_4 = io_tensor_wr_0_bits_data_0_4; // @[TensorLoad.scala 72:8]
  assign tensorLoad_io_tensor_wr_0_bits_data_0_5 = io_tensor_wr_0_bits_data_0_5; // @[TensorLoad.scala 72:8]
  assign tensorLoad_io_tensor_wr_0_bits_data_0_6 = io_tensor_wr_0_bits_data_0_6; // @[TensorLoad.scala 72:8]
  assign tensorLoad_io_tensor_wr_0_bits_data_0_7 = io_tensor_wr_0_bits_data_0_7; // @[TensorLoad.scala 72:8]
  assign tensorLoad_io_tensor_wr_0_bits_data_0_8 = io_tensor_wr_0_bits_data_0_8; // @[TensorLoad.scala 72:8]
  assign tensorLoad_io_tensor_wr_0_bits_data_0_9 = io_tensor_wr_0_bits_data_0_9; // @[TensorLoad.scala 72:8]
  assign tensorLoad_io_tensor_wr_0_bits_data_0_10 = io_tensor_wr_0_bits_data_0_10; // @[TensorLoad.scala 72:8]
  assign tensorLoad_io_tensor_wr_0_bits_data_0_11 = io_tensor_wr_0_bits_data_0_11; // @[TensorLoad.scala 72:8]
  assign tensorLoad_io_tensor_wr_0_bits_data_0_12 = io_tensor_wr_0_bits_data_0_12; // @[TensorLoad.scala 72:8]
  assign tensorLoad_io_tensor_wr_0_bits_data_0_13 = io_tensor_wr_0_bits_data_0_13; // @[TensorLoad.scala 72:8]
  assign tensorLoad_io_tensor_wr_0_bits_data_0_14 = io_tensor_wr_0_bits_data_0_14; // @[TensorLoad.scala 72:8]
  assign tensorLoad_io_tensor_wr_0_bits_data_0_15 = io_tensor_wr_0_bits_data_0_15; // @[TensorLoad.scala 72:8]
endmodule