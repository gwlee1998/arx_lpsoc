module Adder(
  input  [64:0] io_a,
  input  [64:0] io_b,
  output [65:0] io_y
);
  assign io_y = $signed(io_a) + $signed(io_b); // @[TensorGemm.scala 81:13]
endmodule