module Store(
  input          clock,
  input          reset,
  input          io_i_post,
  output         io_o_post,
  output         io_inst_ready,
  input          io_inst_valid,
  input  [127:0] io_inst_bits,
  input  [31:0]  io_out_baddr,
  input          io_vme_wr_cmd_ready,
  output         io_vme_wr_cmd_valid,
  output [31:0]  io_vme_wr_cmd_bits_addr,
  output [7:0]   io_vme_wr_cmd_bits_len,
  input          io_vme_wr_data_ready,
  output         io_vme_wr_data_valid,
  output [63:0]  io_vme_wr_data_bits_data,
  input          io_vme_wr_ack,
  input          io_out_wr_0_valid,
  input  [10:0]  io_out_wr_0_bits_idx,
  input  [31:0]  io_out_wr_0_bits_data_0_0,
  input  [31:0]  io_out_wr_0_bits_data_0_1,
  input  [31:0]  io_out_wr_0_bits_data_0_2,
  input  [31:0]  io_out_wr_0_bits_data_0_3,
  input  [31:0]  io_out_wr_0_bits_data_0_4,
  input  [31:0]  io_out_wr_0_bits_data_0_5,
  input  [31:0]  io_out_wr_0_bits_data_0_6,
  input  [31:0]  io_out_wr_0_bits_data_0_7,
  input  [31:0]  io_out_wr_0_bits_data_0_8,
  input  [31:0]  io_out_wr_0_bits_data_0_9,
  input  [31:0]  io_out_wr_0_bits_data_0_10,
  input  [31:0]  io_out_wr_0_bits_data_0_11,
  input  [31:0]  io_out_wr_0_bits_data_0_12,
  input  [31:0]  io_out_wr_0_bits_data_0_13,
  input  [31:0]  io_out_wr_0_bits_data_0_14,
  input  [31:0]  io_out_wr_0_bits_data_0_15
);
`ifdef RANDOMIZE_REG_INIT
  reg [31:0] _RAND_0;
`endif // RANDOMIZE_REG_INIT
  wire  s_clock; // @[Store.scala 46:17]
  wire  s_reset; // @[Store.scala 46:17]
  wire  s_io_spost; // @[Store.scala 46:17]
  wire  s_io_swait; // @[Store.scala 46:17]
  wire  s_io_sready; // @[Store.scala 46:17]
  wire  inst_q_clock; // @[Store.scala 47:22]
  wire  inst_q_reset; // @[Store.scala 47:22]
  wire  inst_q_io_enq_ready; // @[Store.scala 47:22]
  wire  inst_q_io_enq_valid; // @[Store.scala 47:22]
  wire [127:0] inst_q_io_enq_bits; // @[Store.scala 47:22]
  wire  inst_q_io_deq_ready; // @[Store.scala 47:22]
  wire  inst_q_io_deq_valid; // @[Store.scala 47:22]
  wire [127:0] inst_q_io_deq_bits; // @[Store.scala 47:22]
  wire [127:0] dec_io_inst; // @[Store.scala 49:19]
  wire  dec_io_push_prev; // @[Store.scala 49:19]
  wire  dec_io_pop_prev; // @[Store.scala 49:19]
  wire  dec_io_isStore; // @[Store.scala 49:19]
  wire  dec_io_isSync; // @[Store.scala 49:19]
  wire  tensorStore_clock; // @[Store.scala 52:27]
  wire  tensorStore_reset; // @[Store.scala 52:27]
  wire  tensorStore_io_start; // @[Store.scala 52:27]
  wire  tensorStore_io_done; // @[Store.scala 52:27]
  wire [127:0] tensorStore_io_inst; // @[Store.scala 52:27]
  wire [31:0] tensorStore_io_baddr; // @[Store.scala 52:27]
  wire  tensorStore_io_vme_wr_cmd_ready; // @[Store.scala 52:27]
  wire  tensorStore_io_vme_wr_cmd_valid; // @[Store.scala 52:27]
  wire [31:0] tensorStore_io_vme_wr_cmd_bits_addr; // @[Store.scala 52:27]
  wire [7:0] tensorStore_io_vme_wr_cmd_bits_len; // @[Store.scala 52:27]
  wire  tensorStore_io_vme_wr_data_ready; // @[Store.scala 52:27]
  wire  tensorStore_io_vme_wr_data_valid; // @[Store.scala 52:27]
  wire [63:0] tensorStore_io_vme_wr_data_bits_data; // @[Store.scala 52:27]
  wire  tensorStore_io_vme_wr_ack; // @[Store.scala 52:27]
  wire  tensorStore_io_tensor_wr_0_valid; // @[Store.scala 52:27]
  wire [10:0] tensorStore_io_tensor_wr_0_bits_idx; // @[Store.scala 52:27]
  wire [31:0] tensorStore_io_tensor_wr_0_bits_data_0_0; // @[Store.scala 52:27]
  wire [31:0] tensorStore_io_tensor_wr_0_bits_data_0_1; // @[Store.scala 52:27]
  wire [31:0] tensorStore_io_tensor_wr_0_bits_data_0_2; // @[Store.scala 52:27]
  wire [31:0] tensorStore_io_tensor_wr_0_bits_data_0_3; // @[Store.scala 52:27]
  wire [31:0] tensorStore_io_tensor_wr_0_bits_data_0_4; // @[Store.scala 52:27]
  wire [31:0] tensorStore_io_tensor_wr_0_bits_data_0_5; // @[Store.scala 52:27]
  wire [31:0] tensorStore_io_tensor_wr_0_bits_data_0_6; // @[Store.scala 52:27]
  wire [31:0] tensorStore_io_tensor_wr_0_bits_data_0_7; // @[Store.scala 52:27]
  wire [31:0] tensorStore_io_tensor_wr_0_bits_data_0_8; // @[Store.scala 52:27]
  wire [31:0] tensorStore_io_tensor_wr_0_bits_data_0_9; // @[Store.scala 52:27]
  wire [31:0] tensorStore_io_tensor_wr_0_bits_data_0_10; // @[Store.scala 52:27]
  wire [31:0] tensorStore_io_tensor_wr_0_bits_data_0_11; // @[Store.scala 52:27]
  wire [31:0] tensorStore_io_tensor_wr_0_bits_data_0_12; // @[Store.scala 52:27]
  wire [31:0] tensorStore_io_tensor_wr_0_bits_data_0_13; // @[Store.scala 52:27]
  wire [31:0] tensorStore_io_tensor_wr_0_bits_data_0_14; // @[Store.scala 52:27]
  wire [31:0] tensorStore_io_tensor_wr_0_bits_data_0_15; // @[Store.scala 52:27]
  reg [1:0] state; // @[Store.scala 44:22]
  wire  _start_T = dec_io_pop_prev ? s_io_sready : 1'h1; // @[Store.scala 54:40]
  wire  start = inst_q_io_deq_valid & _start_T; // @[Store.scala 54:35]
  wire  _T = 2'h0 == state; // @[Conditional.scala 37:30]
  wire [1:0] _GEN_0 = dec_io_isStore ? 2'h2 : state; // @[Store.scala 63:36 Store.scala 64:17 Store.scala 44:22]
  wire  _T_1 = 2'h1 == state; // @[Conditional.scala 37:30]
  wire  _T_2 = 2'h2 == state; // @[Conditional.scala 37:30]
  wire [1:0] _GEN_3 = tensorStore_io_done ? 2'h0 : state; // @[Store.scala 72:18 Store.scala 73:15 Store.scala 44:22]
  wire  _inst_q_io_deq_ready_T_3 = state == 2'h2 & tensorStore_io_done | state == 2'h1; // @[Store.scala 80:50]
  wire  _tensorStore_io_start_T_1 = state == 2'h0 & start; // @[Store.scala 83:43]
  Semaphore s ( // @[Store.scala 46:17]
    .clock(s_clock),
    .reset(s_reset),
    .io_spost(s_io_spost),
    .io_swait(s_io_swait),
    .io_sready(s_io_sready)
  );
  Queue_7 inst_q ( // @[Store.scala 47:22]
    .clock(inst_q_clock),
    .reset(inst_q_reset),
    .io_enq_ready(inst_q_io_enq_ready),
    .io_enq_valid(inst_q_io_enq_valid),
    .io_enq_bits(inst_q_io_enq_bits),
    .io_deq_ready(inst_q_io_deq_ready),
    .io_deq_valid(inst_q_io_deq_valid),
    .io_deq_bits(inst_q_io_deq_bits)
  );
  StoreDecode dec ( // @[Store.scala 49:19]
    .io_inst(dec_io_inst),
    .io_push_prev(dec_io_push_prev),
    .io_pop_prev(dec_io_pop_prev),
    .io_isStore(dec_io_isStore),
    .io_isSync(dec_io_isSync)
  );
  TensorStoreOut tensorStore ( // @[Store.scala 52:27]
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
  assign io_o_post = dec_io_push_prev & _inst_q_io_deq_ready_T_3; // @[Store.scala 92:33]
  assign io_inst_ready = inst_q_io_enq_ready; // @[Store.scala 79:17]
  assign io_vme_wr_cmd_valid = tensorStore_io_vme_wr_cmd_valid; // @[Store.scala 86:13]
  assign io_vme_wr_cmd_bits_addr = tensorStore_io_vme_wr_cmd_bits_addr; // @[Store.scala 86:13]
  assign io_vme_wr_cmd_bits_len = tensorStore_io_vme_wr_cmd_bits_len; // @[Store.scala 86:13]
  assign io_vme_wr_data_valid = tensorStore_io_vme_wr_data_valid; // @[Store.scala 86:13]
  assign io_vme_wr_data_bits_data = tensorStore_io_vme_wr_data_bits_data; // @[Store.scala 86:13]
  assign s_clock = clock;
  assign s_reset = reset;
  assign s_io_spost = io_i_post; // @[Store.scala 90:14]
  assign s_io_swait = dec_io_pop_prev & _tensorStore_io_start_T_1; // @[Store.scala 91:33]
  assign inst_q_clock = clock;
  assign inst_q_reset = reset;
  assign inst_q_io_enq_valid = io_inst_valid; // @[Store.scala 79:17]
  assign inst_q_io_enq_bits = io_inst_bits; // @[Store.scala 79:17]
  assign inst_q_io_deq_ready = state == 2'h2 & tensorStore_io_done | state == 2'h1; // @[Store.scala 80:50]
  assign dec_io_inst = inst_q_io_deq_bits; // @[Store.scala 50:15]
  assign tensorStore_clock = clock;
  assign tensorStore_reset = reset;
  assign tensorStore_io_start = state == 2'h0 & start & dec_io_isStore; // @[Store.scala 83:51]
  assign tensorStore_io_inst = inst_q_io_deq_bits; // @[Store.scala 84:23]
  assign tensorStore_io_baddr = io_out_baddr; // @[Store.scala 85:24]
  assign tensorStore_io_vme_wr_cmd_ready = io_vme_wr_cmd_ready; // @[Store.scala 86:13]
  assign tensorStore_io_vme_wr_data_ready = io_vme_wr_data_ready; // @[Store.scala 86:13]
  assign tensorStore_io_vme_wr_ack = io_vme_wr_ack; // @[Store.scala 86:13]
  assign tensorStore_io_tensor_wr_0_valid = io_out_wr_0_valid; // @[Store.scala 87:25]
  assign tensorStore_io_tensor_wr_0_bits_idx = io_out_wr_0_bits_idx; // @[Store.scala 87:25]
  assign tensorStore_io_tensor_wr_0_bits_data_0_0 = io_out_wr_0_bits_data_0_0; // @[Store.scala 87:25]
  assign tensorStore_io_tensor_wr_0_bits_data_0_1 = io_out_wr_0_bits_data_0_1; // @[Store.scala 87:25]
  assign tensorStore_io_tensor_wr_0_bits_data_0_2 = io_out_wr_0_bits_data_0_2; // @[Store.scala 87:25]
  assign tensorStore_io_tensor_wr_0_bits_data_0_3 = io_out_wr_0_bits_data_0_3; // @[Store.scala 87:25]
  assign tensorStore_io_tensor_wr_0_bits_data_0_4 = io_out_wr_0_bits_data_0_4; // @[Store.scala 87:25]
  assign tensorStore_io_tensor_wr_0_bits_data_0_5 = io_out_wr_0_bits_data_0_5; // @[Store.scala 87:25]
  assign tensorStore_io_tensor_wr_0_bits_data_0_6 = io_out_wr_0_bits_data_0_6; // @[Store.scala 87:25]
  assign tensorStore_io_tensor_wr_0_bits_data_0_7 = io_out_wr_0_bits_data_0_7; // @[Store.scala 87:25]
  assign tensorStore_io_tensor_wr_0_bits_data_0_8 = io_out_wr_0_bits_data_0_8; // @[Store.scala 87:25]
  assign tensorStore_io_tensor_wr_0_bits_data_0_9 = io_out_wr_0_bits_data_0_9; // @[Store.scala 87:25]
  assign tensorStore_io_tensor_wr_0_bits_data_0_10 = io_out_wr_0_bits_data_0_10; // @[Store.scala 87:25]
  assign tensorStore_io_tensor_wr_0_bits_data_0_11 = io_out_wr_0_bits_data_0_11; // @[Store.scala 87:25]
  assign tensorStore_io_tensor_wr_0_bits_data_0_12 = io_out_wr_0_bits_data_0_12; // @[Store.scala 87:25]
  assign tensorStore_io_tensor_wr_0_bits_data_0_13 = io_out_wr_0_bits_data_0_13; // @[Store.scala 87:25]
  assign tensorStore_io_tensor_wr_0_bits_data_0_14 = io_out_wr_0_bits_data_0_14; // @[Store.scala 87:25]
  assign tensorStore_io_tensor_wr_0_bits_data_0_15 = io_out_wr_0_bits_data_0_15; // @[Store.scala 87:25]
  always @(posedge clock) begin
    if (reset) begin // @[Store.scala 44:22]
      state <= 2'h0; // @[Store.scala 44:22]
    end else if (_T) begin // @[Conditional.scala 40:58]
      if (start) begin // @[Store.scala 60:19]
        if (dec_io_isSync) begin // @[Store.scala 61:29]
          state <= 2'h1; // @[Store.scala 62:17]
        end else begin
          state <= _GEN_0;
        end
      end
    end else if (_T_1) begin // @[Conditional.scala 39:67]
      state <= 2'h0; // @[Store.scala 69:13]
    end else if (_T_2) begin // @[Conditional.scala 39:67]
      state <= _GEN_3;
    end
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
`endif // RANDOMIZE_REG_INIT
  `endif // RANDOMIZE
end // initial
`ifdef FIRRTL_AFTER_INITIAL
`FIRRTL_AFTER_INITIAL
`endif
`endif // SYNTHESIS
endmodule