module ComputeDecode(
  input  [127:0] io_inst,
  output         io_push_next,
  output         io_push_prev,
  output         io_pop_next,
  output         io_pop_prev,
  output         io_isLoadAcc,
  output         io_isLoadUop,
  output         io_isSync,
  output         io_isAlu,
  output         io_isGemm,
  output         io_isFinish
);
  wire [15:0] dec_xsize = io_inst[95:80]; // @[Decode.scala 199:29]
  wire [127:0] _io_isLoadAcc_T = io_inst & 128'h387; // @[Decode.scala 204:27]
  wire  _io_isLoadAcc_T_1 = 128'h180 == _io_isLoadAcc_T; // @[Decode.scala 204:27]
  wire  _io_isLoadAcc_T_2 = dec_xsize != 16'h0; // @[Decode.scala 204:48]
  wire  _io_isLoadUop_T_1 = 128'h0 == _io_isLoadAcc_T; // @[Decode.scala 205:27]
  wire [127:0] _io_isAlu_T = io_inst & 128'h7000000000000000000000000007; // @[Decode.scala 207:23]
  wire [127:0] _io_isGemm_T = io_inst & 128'h7; // @[Decode.scala 208:24]
  assign io_push_next = io_inst[6]; // @[Decode.scala 199:29]
  assign io_push_prev = io_inst[5]; // @[Decode.scala 199:29]
  assign io_pop_next = io_inst[4]; // @[Decode.scala 199:29]
  assign io_pop_prev = io_inst[3]; // @[Decode.scala 199:29]
  assign io_isLoadAcc = 128'h180 == _io_isLoadAcc_T & dec_xsize != 16'h0; // @[Decode.scala 204:36]
  assign io_isLoadUop = 128'h0 == _io_isLoadAcc_T & _io_isLoadAcc_T_2; // @[Decode.scala 205:36]
  assign io_isSync = (_io_isLoadAcc_T_1 | _io_isLoadUop_T_1) & dec_xsize == 16'h0; // @[Decode.scala 206:54]
  assign io_isAlu = 128'h4 == _io_isAlu_T | 128'h1000000000000000000000000004 == _io_isAlu_T | 128'h2000000000000000000000000004
     == _io_isAlu_T | 128'h3000000000000000000000000004 == _io_isAlu_T; // @[Decode.scala 207:70]
  assign io_isGemm = 128'h2 == _io_isGemm_T; // @[Decode.scala 208:24]
  assign io_isFinish = 128'h3 == _io_isGemm_T; // @[Decode.scala 209:26]
endmodule