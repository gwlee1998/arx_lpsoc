module StoreDecode(
  input  [127:0] io_inst,
  output         io_push_prev,
  output         io_pop_prev,
  output         io_isStore,
  output         io_isSync
);
  wire [15:0] dec_xsize = io_inst[95:80]; // @[Decode.scala 224:29]
  wire [127:0] _io_isStore_T = io_inst & 128'h7; // @[Decode.scala 227:25]
  wire  _io_isStore_T_1 = 128'h1 == _io_isStore_T; // @[Decode.scala 227:25]
  assign io_push_prev = io_inst[5]; // @[Decode.scala 224:29]
  assign io_pop_prev = io_inst[3]; // @[Decode.scala 224:29]
  assign io_isStore = 128'h1 == _io_isStore_T & dec_xsize != 16'h0; // @[Decode.scala 227:34]
  assign io_isSync = _io_isStore_T_1 & dec_xsize == 16'h0; // @[Decode.scala 228:33]
endmodule