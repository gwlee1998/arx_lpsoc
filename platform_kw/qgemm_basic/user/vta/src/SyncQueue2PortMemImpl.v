module SyncQueue2PortMemImpl(
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
`endif // RANDOMIZE_REG_INIT
  wire  memoryQueue_clock; // @[SyncQueue.scala 172:27]
  wire  memoryQueue_reset; // @[SyncQueue.scala 172:27]
  wire  memoryQueue_io_enq_ready; // @[SyncQueue.scala 172:27]
  wire  memoryQueue_io_enq_valid; // @[SyncQueue.scala 172:27]
  wire [127:0] memoryQueue_io_enq_bits; // @[SyncQueue.scala 172:27]
  wire  memoryQueue_io_deq_ready; // @[SyncQueue.scala 172:27]
  wire  memoryQueue_io_deq_valid; // @[SyncQueue.scala 172:27]
  wire [127:0] memoryQueue_io_deq_bits; // @[SyncQueue.scala 172:27]
  wire [7:0] memoryQueue_io_count; // @[SyncQueue.scala 172:27]
  wire  buffer_clock; // @[SyncQueue.scala 173:22]
  wire  buffer_reset; // @[SyncQueue.scala 173:22]
  wire  buffer_io_enq_ready; // @[SyncQueue.scala 173:22]
  wire  buffer_io_enq_valid; // @[SyncQueue.scala 173:22]
  wire [127:0] buffer_io_enq_bits; // @[SyncQueue.scala 173:22]
  wire  buffer_io_deq_ready; // @[SyncQueue.scala 173:22]
  wire  buffer_io_deq_valid; // @[SyncQueue.scala 173:22]
  wire [127:0] buffer_io_deq_bits; // @[SyncQueue.scala 173:22]
  wire  memoryQueueHasValues = memoryQueue_io_count != 8'h0; // @[SyncQueue.scala 175:51]
  wire  _memoryQueue_io_enq_valid_T = io_enq_ready & io_enq_valid; // @[Decoupled.scala 40:37]
  wire  _countNext_T_1 = io_deq_ready & io_deq_valid; // @[Decoupled.scala 40:37]
  wire  _countNext_T_2 = _memoryQueue_io_enq_valid_T | _countNext_T_1; // @[SyncQueue.scala 190:28]
  reg [7:0] countNext; // @[Reg.scala 27:20]
  wire  _T_3 = _memoryQueue_io_enq_valid_T & ~_countNext_T_1; // @[SyncQueue.scala 191:23]
  wire [7:0] _count_T_1 = countNext + 8'h1; // @[SyncQueue.scala 193:24]
  wire  _T_11 = ~_memoryQueue_io_enq_valid_T & _countNext_T_1; // @[SyncQueue.scala 194:30]
  wire [7:0] _count_T_3 = countNext - 8'h1; // @[SyncQueue.scala 196:24]
  OneCycleQueue memoryQueue ( // @[SyncQueue.scala 172:27]
    .clock(memoryQueue_clock),
    .reset(memoryQueue_reset),
    .io_enq_ready(memoryQueue_io_enq_ready),
    .io_enq_valid(memoryQueue_io_enq_valid),
    .io_enq_bits(memoryQueue_io_enq_bits),
    .io_deq_ready(memoryQueue_io_deq_ready),
    .io_deq_valid(memoryQueue_io_deq_valid),
    .io_deq_bits(memoryQueue_io_deq_bits),
    .io_count(memoryQueue_io_count)
  );
  Queue_6 buffer ( // @[SyncQueue.scala 173:22]
    .clock(buffer_clock),
    .reset(buffer_reset),
    .io_enq_ready(buffer_io_enq_ready),
    .io_enq_valid(buffer_io_enq_valid),
    .io_enq_bits(buffer_io_enq_bits),
    .io_deq_ready(buffer_io_deq_ready),
    .io_deq_valid(buffer_io_deq_valid),
    .io_deq_bits(buffer_io_deq_bits)
  );
  assign io_enq_ready = countNext != 8'h80; // @[SyncQueue.scala 202:30]
  assign io_deq_valid = countNext != 8'h0; // @[SyncQueue.scala 203:30]
  assign io_deq_bits = buffer_io_deq_bits; // @[SyncQueue.scala 181:10]
  assign io_count = countNext; // @[SyncQueue.scala 201:12]
  assign memoryQueue_clock = clock;
  assign memoryQueue_reset = reset;
  assign memoryQueue_io_enq_valid = _memoryQueue_io_enq_valid_T & (~buffer_io_enq_ready | memoryQueueHasValues); // @[SyncQueue.scala 183:45]
  assign memoryQueue_io_enq_bits = io_enq_bits; // @[SyncQueue.scala 182:27]
  assign memoryQueue_io_deq_ready = buffer_io_enq_ready; // @[SyncQueue.scala 184:28]
  assign buffer_clock = clock;
  assign buffer_reset = reset;
  assign buffer_io_enq_valid = memoryQueueHasValues ? memoryQueue_io_deq_valid : io_enq_valid; // @[SyncQueue.scala 176:26]
  assign buffer_io_enq_bits = memoryQueueHasValues ? memoryQueue_io_deq_bits : io_enq_bits; // @[SyncQueue.scala 177:25]
  assign buffer_io_deq_ready = io_deq_ready; // @[SyncQueue.scala 181:10]
  always @(posedge clock) begin
    if (reset) begin // @[Reg.scala 27:20]
      countNext <= 8'h0; // @[Reg.scala 27:20]
    end else if (_countNext_T_2) begin // @[Reg.scala 28:19]
      if (_memoryQueue_io_enq_valid_T & ~_countNext_T_1) begin // @[SyncQueue.scala 191:42]
        countNext <= _count_T_1; // @[SyncQueue.scala 193:11]
      end else if (~_memoryQueue_io_enq_valid_T & _countNext_T_1) begin // @[SyncQueue.scala 194:48]
        countNext <= _count_T_3; // @[SyncQueue.scala 196:11]
      end
    end
    `ifndef SYNTHESIS
    `ifdef PRINTF_COND
      if (`PRINTF_COND) begin
    `endif
        if (_T_3 & ~(countNext < 8'h80 | reset)) begin
          $fwrite(32'h80000002,"Assertion failed\n    at SyncQueue.scala:192 assert(countNext < entries.U)\n"); // @[SyncQueue.scala 192:11]
        end
    `ifdef PRINTF_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef STOP_COND
      if (`STOP_COND) begin
    `endif
        if (_T_3 & ~(countNext < 8'h80 | reset)) begin
          $fatal; // @[SyncQueue.scala 192:11]
        end
    `ifdef STOP_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef PRINTF_COND
      if (`PRINTF_COND) begin
    `endif
        if (~_T_3 & _T_11 & ~(countNext > 8'h0 | reset)) begin
          $fwrite(32'h80000002,"Assertion failed\n    at SyncQueue.scala:195 assert(countNext > 0.U)\n"); // @[SyncQueue.scala 195:11]
        end
    `ifdef PRINTF_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef STOP_COND
      if (`STOP_COND) begin
    `endif
        if (~_T_3 & _T_11 & ~(countNext > 8'h0 | reset)) begin
          $fatal; // @[SyncQueue.scala 195:11]
        end
    `ifdef STOP_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef PRINTF_COND
      if (`PRINTF_COND) begin
    `endif
        if (~(io_deq_valid == buffer_io_deq_valid | reset)) begin
          $fwrite(32'h80000002,
            "Assertion failed\n    at SyncQueue.scala:204 assert(io.deq.valid === buffer.io.deq.valid)\n"); // @[SyncQueue.scala 204:9]
        end
    `ifdef PRINTF_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef STOP_COND
      if (`STOP_COND) begin
    `endif
        if (~(io_deq_valid == buffer_io_deq_valid | reset)) begin
          $fatal; // @[SyncQueue.scala 204:9]
        end
    `ifdef STOP_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef PRINTF_COND
      if (`PRINTF_COND) begin
    `endif
        if (~(io_enq_ready == buffer_io_enq_ready | memoryQueue_io_enq_ready | reset)) begin
          $fwrite(32'h80000002,
            "Assertion failed\n    at SyncQueue.scala:205 assert(io.enq.ready === buffer.io.enq.ready || memoryQueue.io.enq.ready)\n"
            ); // @[SyncQueue.scala 205:9]
        end
    `ifdef PRINTF_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef STOP_COND
      if (`STOP_COND) begin
    `endif
        if (~(io_enq_ready == buffer_io_enq_ready | memoryQueue_io_enq_ready | reset)) begin
          $fatal; // @[SyncQueue.scala 205:9]
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
  countNext = _RAND_0[7:0];
`endif // RANDOMIZE_REG_INIT
  `endif // RANDOMIZE
end // initial
`ifdef FIRRTL_AFTER_INITIAL
`FIRRTL_AFTER_INITIAL
`endif
`endif // SYNTHESIS
endmodule