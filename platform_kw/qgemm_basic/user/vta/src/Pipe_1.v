module Pipe_1(
  input         clock,
  input         reset,
  input         io_enq_valid,
  input  [10:0] io_enq_bits,
  output        io_deq_valid,
  output [10:0] io_deq_bits
);
`ifdef RANDOMIZE_REG_INIT
  reg [31:0] _RAND_0;
  reg [31:0] _RAND_1;
`endif // RANDOMIZE_REG_INIT
  reg  io_deq_v; // @[Valid.scala 117:22]
  reg [10:0] io_deq_b; // @[Reg.scala 15:16]
  assign io_deq_valid = io_deq_v; // @[Valid.scala 112:21 Valid.scala 113:17]
  assign io_deq_bits = io_deq_b; // @[Valid.scala 112:21 Valid.scala 114:16]
  always @(posedge clock) begin
    if (reset) begin // @[Valid.scala 117:22]
      io_deq_v <= 1'h0; // @[Valid.scala 117:22]
    end else begin
      io_deq_v <= io_enq_valid; // @[Valid.scala 117:22]
    end
    if (io_enq_valid) begin // @[Reg.scala 16:19]
      io_deq_b <= io_enq_bits; // @[Reg.scala 16:23]
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
  io_deq_v = _RAND_0[0:0];
  _RAND_1 = {1{`RANDOM}};
  io_deq_b = _RAND_1[10:0];
`endif // RANDOMIZE_REG_INIT
  `endif // RANDOMIZE
end // initial
`ifdef FIRRTL_AFTER_INITIAL
`FIRRTL_AFTER_INITIAL
`endif
`endif // SYNTHESIS
endmodule