module EventCounters(
  input         clock,
  input         reset,
  input         io_launch,
  input         io_finish,
  output        io_ecnt_0_valid,
  output [31:0] io_ecnt_0_bits,
  output        io_ucnt_0_valid,
  output [31:0] io_ucnt_0_bits,
  input         io_acc_wr_event
);
`ifdef RANDOMIZE_REG_INIT
  reg [31:0] _RAND_0;
  reg [31:0] _RAND_1;
`endif // RANDOMIZE_REG_INIT
  reg [31:0] cycle_cnt; // @[EventCounters.scala 50:26]
  wire [31:0] _cycle_cnt_T_1 = cycle_cnt + 32'h1; // @[EventCounters.scala 52:28]
  reg [31:0] acc_wr_count; // @[EventCounters.scala 59:25]
  wire [31:0] _acc_wr_count_T_1 = acc_wr_count + 32'h1; // @[EventCounters.scala 63:34]
  assign io_ecnt_0_valid = io_finish; // @[EventCounters.scala 56:20]
  assign io_ecnt_0_bits = cycle_cnt; // @[EventCounters.scala 57:19]
  assign io_ucnt_0_valid = io_finish; // @[EventCounters.scala 65:20]
  assign io_ucnt_0_bits = acc_wr_count; // @[EventCounters.scala 66:19]
  always @(posedge clock) begin
    if (reset) begin // @[EventCounters.scala 50:26]
      cycle_cnt <= 32'h0; // @[EventCounters.scala 50:26]
    end else if (io_launch & ~io_finish) begin // @[EventCounters.scala 51:33]
      cycle_cnt <= _cycle_cnt_T_1; // @[EventCounters.scala 52:15]
    end else begin
      cycle_cnt <= 32'h0; // @[EventCounters.scala 54:15]
    end
    if (~io_launch | io_finish) begin // @[EventCounters.scala 60:34]
      acc_wr_count <= 32'h0; // @[EventCounters.scala 61:18]
    end else if (io_acc_wr_event) begin // @[EventCounters.scala 62:32]
      acc_wr_count <= _acc_wr_count_T_1; // @[EventCounters.scala 63:18]
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
  cycle_cnt = _RAND_0[31:0];
  _RAND_1 = {1{`RANDOM}};
  acc_wr_count = _RAND_1[31:0];
`endif // RANDOMIZE_REG_INIT
  `endif // RANDOMIZE
end // initial
`ifdef FIRRTL_AFTER_INITIAL
`FIRRTL_AFTER_INITIAL
`endif
`endif // SYNTHESIS
endmodule