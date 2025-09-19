module OneCycleQueue(
  input          clock,
  input          reset,
  output         io_enq_ready,
  input          io_enq_valid,
  input  [127:0] io_enq_bits,
  input          io_deq_ready,
  output         io_deq_valid,
  output [127:0] io_deq_bits,
  output [7:0]   io_count
);
`ifdef RANDOMIZE_REG_INIT
  reg [31:0] _RAND_0;
  reg [31:0] _RAND_1;
  reg [31:0] _RAND_2;
  reg [31:0] _RAND_3;
`endif // RANDOMIZE_REG_INIT
  wire  ram0_clock; // @[SyncQueue.scala 377:20]
  wire  ram0_io_wr_en; // @[SyncQueue.scala 377:20]
  wire [15:0] ram0_io_wr_addr; // @[SyncQueue.scala 377:20]
  wire [127:0] ram0_io_wr_data; // @[SyncQueue.scala 377:20]
  wire  ram0_io_rd_en; // @[SyncQueue.scala 377:20]
  wire [15:0] ram0_io_rd_addr; // @[SyncQueue.scala 377:20]
  wire [127:0] ram0_io_rd_data; // @[SyncQueue.scala 377:20]
  reg [6:0] value; // @[Counter.scala 60:40]
  reg [6:0] value_1; // @[Counter.scala 60:40]
  reg  maybe_full; // @[SyncQueue.scala 380:27]
  wire  ptr_match = value == value_1; // @[SyncQueue.scala 383:33]
  wire  empty = ptr_match & ~maybe_full; // @[SyncQueue.scala 384:25]
  wire  full = ptr_match & maybe_full; // @[SyncQueue.scala 385:24]
  wire  do_enq = io_enq_ready & io_enq_valid; // @[Decoupled.scala 40:37]
  wire  do_deq = io_deq_ready & io_deq_valid; // @[Decoupled.scala 40:37]
  wire  wrap = value_1 == 7'h7f; // @[Counter.scala 72:24]
  wire [6:0] _value_T_1 = value_1 + 7'h1; // @[Counter.scala 76:24]
  wire [6:0] _value_T_3 = value + 7'h1; // @[Counter.scala 76:24]
  wire  _firstRead_T_1 = do_enq & io_count == 8'h0; // @[SyncQueue.scala 403:43]
  reg  firstRead; // @[Reg.scala 27:20]
  wire  _io_deq_valid_T_1 = ~firstRead; // @[SyncQueue.scala 404:29]
  wire [6:0] _GEN_4 = wrap ? 7'h0 : _value_T_1; // @[SyncQueue.scala 413:17 SyncQueue.scala 414:14 SyncQueue.scala 416:14]
  wire [6:0] _GEN_5 = do_deq ? _GEN_4 : value_1; // @[SyncQueue.scala 411:23 SyncQueue.scala 419:12]
  wire [6:0] rdAddr = firstRead ? value_1 : _GEN_5; // @[SyncQueue.scala 409:19 SyncQueue.scala 410:12]
  wire [6:0] ptr_diff = value - value_1; // @[SyncQueue.scala 430:32]
  wire [7:0] _io_count_T_1 = maybe_full & ptr_match ? 8'h80 : 8'h0; // @[SyncQueue.scala 432:20]
  wire [7:0] _GEN_7 = {{1'd0}, ptr_diff}; // @[SyncQueue.scala 432:62]
  TwoPortMem ram0 ( // @[SyncQueue.scala 377:20]
    .clock(ram0_clock),
    .io_wr_en(ram0_io_wr_en),
    .io_wr_addr(ram0_io_wr_addr),
    .io_wr_data(ram0_io_wr_data),
    .io_rd_en(ram0_io_rd_en),
    .io_rd_addr(ram0_io_rd_addr),
    .io_rd_data(ram0_io_rd_data)
  );
  assign io_enq_ready = ~full; // @[SyncQueue.scala 405:19]
  assign io_deq_valid = ~empty & ~firstRead; // @[SyncQueue.scala 404:26]
  assign io_deq_bits = ram0_io_rd_data; // @[SyncQueue.scala 426:15]
  assign io_count = _io_count_T_1 | _GEN_7; // @[SyncQueue.scala 432:62]
  assign ram0_clock = clock;
  assign ram0_io_wr_en = io_enq_ready & io_enq_valid; // @[Decoupled.scala 40:37]
  assign ram0_io_wr_addr = {{9'd0}, value}; // @[SyncQueue.scala 423:19]
  assign ram0_io_wr_data = io_enq_bits; // @[SyncQueue.scala 422:19]
  assign ram0_io_rd_en = do_deq | firstRead; // @[SyncQueue.scala 424:27]
  assign ram0_io_rd_addr = {{9'd0}, rdAddr}; // @[SyncQueue.scala 409:19 SyncQueue.scala 410:12]
  always @(posedge clock) begin
    if (reset) begin // @[Counter.scala 60:40]
      value <= 7'h0; // @[Counter.scala 60:40]
    end else if (do_enq) begin // @[SyncQueue.scala 399:17]
      value <= _value_T_3; // @[Counter.scala 76:15]
    end
    if (reset) begin // @[Counter.scala 60:40]
      value_1 <= 7'h0; // @[Counter.scala 60:40]
    end else if (do_deq) begin // @[SyncQueue.scala 391:16]
      value_1 <= _value_T_1; // @[Counter.scala 76:15]
    end
    if (reset) begin // @[SyncQueue.scala 380:27]
      maybe_full <= 1'h0; // @[SyncQueue.scala 380:27]
    end else if (do_enq != do_deq) begin // @[SyncQueue.scala 395:27]
      maybe_full <= do_enq; // @[SyncQueue.scala 396:16]
    end
    if (reset) begin // @[Reg.scala 27:20]
      firstRead <= 1'h0; // @[Reg.scala 27:20]
    end else begin
      firstRead <= _firstRead_T_1;
    end
    `ifndef SYNTHESIS
    `ifdef PRINTF_COND
      if (`PRINTF_COND) begin
    `endif
        if (~(_io_deq_valid_T_1 | ~do_deq | reset)) begin
          $fwrite(32'h80000002,
            "Assertion failed: -F- Cannot have deq with first read as queue output is not valid yet\n    at SyncQueue.scala:406 assert(!firstRead || !do_deq, \"-F- Cannot have deq with first read as queue output is not valid yet\")\n"
            ); // @[SyncQueue.scala 406:9]
        end
    `ifdef PRINTF_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef STOP_COND
      if (`STOP_COND) begin
    `endif
        if (~(_io_deq_valid_T_1 | ~do_deq | reset)) begin
          $fatal; // @[SyncQueue.scala 406:9]
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
  value = _RAND_0[6:0];
  _RAND_1 = {1{`RANDOM}};
  value_1 = _RAND_1[6:0];
  _RAND_2 = {1{`RANDOM}};
  maybe_full = _RAND_2[0:0];
  _RAND_3 = {1{`RANDOM}};
  firstRead = _RAND_3[0:0];
`endif // RANDOMIZE_REG_INIT
  `endif // RANDOMIZE
end // initial
`ifdef FIRRTL_AFTER_INITIAL
`FIRRTL_AFTER_INITIAL
`endif
`endif // SYNTHESIS
endmodule