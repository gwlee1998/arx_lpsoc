module FetchDecode(
  input  [127:0] io_inst,
  output         io_isLoad,
  output         io_isCompute,
  output         io_isStore
);
  wire [127:0] _csignals_T = io_inst & 128'h387; // @[Lookup.scala 31:38]
  wire  _csignals_T_1 = 128'h0 == _csignals_T; // @[Lookup.scala 31:38]
  wire  _csignals_T_3 = 128'h80 == _csignals_T; // @[Lookup.scala 31:38]
  wire  _csignals_T_5 = 128'h100 == _csignals_T; // @[Lookup.scala 31:38]
  wire  _csignals_T_7 = 128'h180 == _csignals_T; // @[Lookup.scala 31:38]
  wire [127:0] _csignals_T_8 = io_inst & 128'h7; // @[Lookup.scala 31:38]
  wire  _csignals_T_9 = 128'h1 == _csignals_T_8; // @[Lookup.scala 31:38]
  wire  _csignals_T_11 = 128'h2 == _csignals_T_8; // @[Lookup.scala 31:38]
  wire  _csignals_T_13 = 128'h3 == _csignals_T_8; // @[Lookup.scala 31:38]
  wire [127:0] _csignals_T_14 = io_inst & 128'h7000000000000000000000000007; // @[Lookup.scala 31:38]
  wire  _csignals_T_15 = 128'h4 == _csignals_T_14; // @[Lookup.scala 31:38]
  wire  _csignals_T_17 = 128'h1000000000000000000000000004 == _csignals_T_14; // @[Lookup.scala 31:38]
  wire  _csignals_T_19 = 128'h2000000000000000000000000004 == _csignals_T_14; // @[Lookup.scala 31:38]
  wire  _csignals_T_21 = 128'h3000000000000000000000000004 == _csignals_T_14; // @[Lookup.scala 31:38]
  wire  cs_val_inst = _csignals_T_1 | (_csignals_T_3 | (_csignals_T_5 | (_csignals_T_7 | (_csignals_T_9 | (
    _csignals_T_11 | (_csignals_T_13 | (_csignals_T_15 | (_csignals_T_17 | (_csignals_T_19 | _csignals_T_21))))))))); // @[Lookup.scala 33:37]
  wire [2:0] _csignals_T_32 = _csignals_T_21 ? 3'h2 : 3'h5; // @[Lookup.scala 33:37]
  wire [2:0] _csignals_T_33 = _csignals_T_19 ? 3'h2 : _csignals_T_32; // @[Lookup.scala 33:37]
  wire [2:0] _csignals_T_34 = _csignals_T_17 ? 3'h2 : _csignals_T_33; // @[Lookup.scala 33:37]
  wire [2:0] _csignals_T_35 = _csignals_T_15 ? 3'h2 : _csignals_T_34; // @[Lookup.scala 33:37]
  wire [2:0] _csignals_T_36 = _csignals_T_13 ? 3'h2 : _csignals_T_35; // @[Lookup.scala 33:37]
  wire [2:0] _csignals_T_37 = _csignals_T_11 ? 3'h2 : _csignals_T_36; // @[Lookup.scala 33:37]
  wire [2:0] _csignals_T_38 = _csignals_T_9 ? 3'h1 : _csignals_T_37; // @[Lookup.scala 33:37]
  wire [2:0] _csignals_T_39 = _csignals_T_7 ? 3'h2 : _csignals_T_38; // @[Lookup.scala 33:37]
  wire [2:0] _csignals_T_40 = _csignals_T_5 ? 3'h0 : _csignals_T_39; // @[Lookup.scala 33:37]
  wire [2:0] _csignals_T_41 = _csignals_T_3 ? 3'h0 : _csignals_T_40; // @[Lookup.scala 33:37]
  wire [2:0] cs_op_type = _csignals_T_1 ? 3'h2 : _csignals_T_41; // @[Lookup.scala 33:37]
  assign io_isLoad = cs_val_inst & cs_op_type == 3'h0; // @[Decode.scala 155:28]
  assign io_isCompute = cs_val_inst & cs_op_type == 3'h2; // @[Decode.scala 156:31]
  assign io_isStore = cs_val_inst & cs_op_type == 3'h1; // @[Decode.scala 157:29]
endmodule