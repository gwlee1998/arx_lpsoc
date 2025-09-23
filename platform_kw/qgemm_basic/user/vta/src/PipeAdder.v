module PipeAdder(
  input         clock,
  input  [66:0] io_a,
  input  [66:0] io_b,
  output [67:0] io_y
);
`ifdef RANDOMIZE_REG_INIT
  reg [95:0] _RAND_0;
  reg [95:0] _RAND_1;
`endif // RANDOMIZE_REG_INIT
  reg [66:0] rA; // @[TensorGemm.scala 63:19]
  reg [66:0] rB; // @[TensorGemm.scala 64:19]
  assign io_y = $signed(rA) + $signed(rB); // @[TensorGemm.scala 65:13]
  always @(posedge clock) begin
    rA <= io_a; // @[TensorGemm.scala 63:19]
    rB <= io_b; // @[TensorGemm.scala 64:19]
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
  _RAND_0 = {3{`RANDOM}};
  rA = _RAND_0[66:0];
  _RAND_1 = {3{`RANDOM}};
  rB = _RAND_1[66:0];
`endif // RANDOMIZE_REG_INIT
  `endif // RANDOMIZE
end // initial
`ifdef FIRRTL_AFTER_INITIAL
`FIRRTL_AFTER_INITIAL
`endif
`endif // SYNTHESIS
endmodule