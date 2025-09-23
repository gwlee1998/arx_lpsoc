module Alu(
  input  [2:0]  io_opcode,
  input  [31:0] io_a,
  input  [31:0] io_b,
  output [31:0] io_y
);
  wire [31:0] ub = io_b; // @[TensorAlu.scala 37:17]
  wire [4:0] _m_T_1 = ~ub[4:0]; // @[TensorAlu.scala 39:11]
  wire [4:0] m = _m_T_1 + 5'h1; // @[TensorAlu.scala 39:29]
  wire [31:0] fop_0 = $signed(io_a) < $signed(io_b) ? $signed(io_a) : $signed(io_b); // @[TensorAlu.scala 43:20]
  wire [31:0] fop_1 = $signed(io_a) < $signed(io_b) ? $signed(io_b) : $signed(io_a); // @[TensorAlu.scala 43:50]
  wire [31:0] fop_2 = $signed(io_a) + $signed(io_b); // @[TensorAlu.scala 44:10]
  wire [31:0] fop_3 = $signed(io_a) >>> ub[4:0]; // @[TensorAlu.scala 44:23]
  wire [62:0] _GEN_0 = {{31{io_a[31]}},io_a}; // @[TensorAlu.scala 44:34]
  wire [62:0] fop_4 = $signed(_GEN_0) << m; // @[TensorAlu.scala 44:34]
  wire [31:0] _io_y_T_1 = 3'h0 == io_opcode ? $signed(fop_0) : $signed(io_a); // @[Mux.scala 80:57]
  wire [31:0] _io_y_T_3 = 3'h1 == io_opcode ? $signed(fop_1) : $signed(_io_y_T_1); // @[Mux.scala 80:57]
  wire [31:0] _io_y_T_5 = 3'h2 == io_opcode ? $signed(fop_2) : $signed(_io_y_T_3); // @[Mux.scala 80:57]
  wire [31:0] _io_y_T_7 = 3'h3 == io_opcode ? $signed(fop_3) : $signed(_io_y_T_5); // @[Mux.scala 80:57]
  wire [62:0] _io_y_T_9 = 3'h4 == io_opcode ? $signed(fop_4) : $signed({{31{_io_y_T_7[31]}},_io_y_T_7}); // @[Mux.scala 80:57]
  assign io_y = _io_y_T_9[31:0]; // @[TensorAlu.scala 47:8]
endmodule