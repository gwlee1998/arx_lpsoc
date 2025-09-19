module LoadDecode(
  input  [127:0] io_inst,
  output         io_push_next,
  output         io_pop_next,
  output         io_isInput,
  output         io_isWeight,
  output         io_isSync
);
  wire [15:0] dec_xsize = io_inst[95:80]; // @[Decode.scala 173:29]
  wire [127:0] _io_isInput_T = io_inst & 128'h387; // @[Decode.scala 176:25]
  wire  _io_isInput_T_1 = 128'h100 == _io_isInput_T; // @[Decode.scala 176:25]
  wire  _io_isInput_T_2 = dec_xsize != 16'h0; // @[Decode.scala 176:46]
  wire  _io_isWeight_T_1 = 128'h80 == _io_isInput_T; // @[Decode.scala 177:26]
  assign io_push_next = io_inst[6]; // @[Decode.scala 173:29]
  assign io_pop_next = io_inst[4]; // @[Decode.scala 173:29]
  assign io_isInput = 128'h100 == _io_isInput_T & dec_xsize != 16'h0; // @[Decode.scala 176:34]
  assign io_isWeight = 128'h80 == _io_isInput_T & _io_isInput_T_2; // @[Decode.scala 177:35]
  assign io_isSync = (_io_isInput_T_1 | _io_isWeight_T_1) & dec_xsize == 16'h0; // @[Decode.scala 178:54]
endmodule