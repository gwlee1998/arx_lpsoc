module Semaphore(
  input   clock,
  input   reset,
  input   io_spost,
  input   io_swait,
  output  io_sready
);
`ifdef RANDOMIZE_REG_INIT
  reg [31:0] _RAND_0;
`endif // RANDOMIZE_REG_INIT
  reg [7:0] cnt; // @[Semaphore.scala 38:20]
  wire [7:0] _cnt_T_1 = cnt + 8'h1; // @[Semaphore.scala 40:16]
  wire [7:0] _cnt_T_3 = cnt - 8'h1; // @[Semaphore.scala 42:59]
  assign io_sready = cnt != 8'h0; // @[Semaphore.scala 43:20]
  always @(posedge clock) begin
    if (reset) begin // @[Semaphore.scala 38:20]
      cnt <= 8'h0; // @[Semaphore.scala 38:20]
    end else if (~io_spost & io_swait & cnt != 8'h0) begin // @[Semaphore.scala 42:46]
      cnt <= _cnt_T_3; // @[Semaphore.scala 42:52]
    end else if (io_spost & ~io_swait & cnt != 8'hff) begin // @[Semaphore.scala 39:74]
      cnt <= _cnt_T_1; // @[Semaphore.scala 40:9]
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
  cnt = _RAND_0[7:0];
`endif // RANDOMIZE_REG_INIT
  `endif // RANDOMIZE
end // initial
`ifdef FIRRTL_AFTER_INITIAL
`FIRRTL_AFTER_INITIAL
`endif
`endif // SYNTHESIS
endmodule