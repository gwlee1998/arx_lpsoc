module Adder_12(
  input  [67:0] io_a,
  input  [67:0] io_b,
  output [68:0] io_y
);
  assign io_y = $signed(io_a) + $signed(io_b); // @[TensorGemm.scala 81:13]
endmodule