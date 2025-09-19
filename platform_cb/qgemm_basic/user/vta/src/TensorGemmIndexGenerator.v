module TensorGemmIndexGenerator(
  input         clock,
  input         reset,
  input         io_start,
  output        io_last,
  input  [9:0]  io_dec_wgt_1,
  input  [9:0]  io_dec_wgt_0,
  input  [10:0] io_dec_inp_1,
  input  [10:0] io_dec_inp_0,
  input  [10:0] io_dec_acc_1,
  input  [10:0] io_dec_acc_0,
  input  [13:0] io_dec_lp_1,
  input  [13:0] io_dec_lp_0,
  input  [13:0] io_dec_uop_end,
  input  [12:0] io_dec_uop_begin,
  output [10:0] io_acc_i,
  output [10:0] io_inp_i,
  output [9:0]  io_wgt_i,
  output [10:0] io_uop_idx,
  output        io_valid
);
`ifdef RANDOMIZE_REG_INIT
  reg [31:0] _RAND_0;
  reg [31:0] _RAND_1;
  reg [31:0] _RAND_2;
  reg [31:0] _RAND_3;
  reg [31:0] _RAND_4;
  reg [31:0] _RAND_5;
  reg [31:0] _RAND_6;
  reg [31:0] _RAND_7;
  reg [31:0] _RAND_8;
  reg [31:0] _RAND_9;
`endif // RANDOMIZE_REG_INIT
  reg  running; // @[TensorGemm.scala 238:24]
  wire  _T = ~running; // @[TensorGemm.scala 239:8]
  wire  _GEN_0 = io_last ? 1'h0 : running; // @[TensorGemm.scala 241:23 TensorGemm.scala 242:13 TensorGemm.scala 238:24]
  wire  _GEN_1 = ~running & io_start | _GEN_0; // @[TensorGemm.scala 239:30 TensorGemm.scala 240:13]
  reg [13:0] cnt_i; // @[TensorGemm.scala 245:18]
  reg [10:0] acc_i; // @[TensorGemm.scala 246:18]
  reg [10:0] inp_i; // @[TensorGemm.scala 247:18]
  reg [9:0] wgt_i; // @[TensorGemm.scala 248:18]
  reg [13:0] cnt_o; // @[TensorGemm.scala 250:18]
  reg [10:0] acc_o; // @[TensorGemm.scala 251:18]
  reg [10:0] inp_o; // @[TensorGemm.scala 252:18]
  reg [9:0] wgt_o; // @[TensorGemm.scala 253:18]
  reg [13:0] uop_idx; // @[TensorGemm.scala 255:20]
  wire [13:0] _T_4 = io_dec_uop_end - 14'h1; // @[TensorGemm.scala 268:38]
  wire [13:0] _uop_idx_T_1 = uop_idx + 14'h1; // @[TensorGemm.scala 269:26]
  wire [13:0] _T_7 = io_dec_lp_1 - 14'h1; // @[TensorGemm.scala 272:35]
  wire [13:0] _cnt_i_T_1 = cnt_i + 14'h1; // @[TensorGemm.scala 273:24]
  wire [10:0] _acc_i_T_1 = acc_i + io_dec_acc_1; // @[TensorGemm.scala 274:24]
  wire [10:0] _inp_i_T_1 = inp_i + io_dec_inp_1; // @[TensorGemm.scala 275:24]
  wire [9:0] _wgt_i_T_1 = wgt_i + io_dec_wgt_1; // @[TensorGemm.scala 276:24]
  wire [13:0] _T_10 = io_dec_lp_0 - 14'h1; // @[TensorGemm.scala 278:37]
  wire [10:0] acc_tmp = acc_o + io_dec_acc_0; // @[TensorGemm.scala 279:31]
  wire [10:0] inp_tmp = inp_o + io_dec_inp_0; // @[TensorGemm.scala 280:31]
  wire [9:0] wgt_tmp = wgt_o + io_dec_wgt_0; // @[TensorGemm.scala 281:31]
  wire [13:0] _cnt_o_T_1 = cnt_o + 14'h1; // @[TensorGemm.scala 282:26]
  wire  _GEN_10 = cnt_o != _T_10 ? 1'h0 : 1'h1; // @[TensorGemm.scala 278:44 TensorGemm.scala 236:11 TensorGemm.scala 291:19]
  wire  _GEN_19 = cnt_i != _T_7 ? 1'h0 : _GEN_10; // @[TensorGemm.scala 272:42 TensorGemm.scala 236:11]
  wire  _GEN_29 = uop_idx != _T_4 ? 1'h0 : _GEN_19; // @[TensorGemm.scala 268:45 TensorGemm.scala 236:11]
  assign io_last = _T ? 1'h0 : _GEN_29; // @[TensorGemm.scala 263:18 TensorGemm.scala 236:11]
  assign io_acc_i = acc_i; // @[TensorGemm.scala 258:12]
  assign io_inp_i = inp_i; // @[TensorGemm.scala 259:12]
  assign io_wgt_i = wgt_i; // @[TensorGemm.scala 260:12]
  assign io_uop_idx = uop_idx[10:0]; // @[TensorGemm.scala 261:14]
  assign io_valid = running; // @[TensorGemm.scala 257:12]
  always @(posedge clock) begin
    if (reset) begin // @[TensorGemm.scala 238:24]
      running <= 1'h0; // @[TensorGemm.scala 238:24]
    end else begin
      running <= _GEN_1;
    end
    if (_T) begin // @[TensorGemm.scala 263:18]
      cnt_i <= 14'h0; // @[TensorGemm.scala 264:11]
    end else if (!(uop_idx != _T_4)) begin // @[TensorGemm.scala 268:45]
      if (cnt_i != _T_7) begin // @[TensorGemm.scala 272:42]
        cnt_i <= _cnt_i_T_1; // @[TensorGemm.scala 273:15]
      end else if (cnt_o != _T_10) begin // @[TensorGemm.scala 278:44]
        cnt_i <= 14'h0; // @[TensorGemm.scala 286:17]
      end
    end
    if (_T) begin // @[TensorGemm.scala 263:18]
      acc_i <= 11'h0; // @[TensorGemm.scala 264:25]
    end else if (!(uop_idx != _T_4)) begin // @[TensorGemm.scala 268:45]
      if (cnt_i != _T_7) begin // @[TensorGemm.scala 272:42]
        acc_i <= _acc_i_T_1; // @[TensorGemm.scala 274:15]
      end else if (cnt_o != _T_10) begin // @[TensorGemm.scala 278:44]
        acc_i <= acc_tmp; // @[TensorGemm.scala 287:17]
      end
    end
    if (_T) begin // @[TensorGemm.scala 263:18]
      inp_i <= 11'h0; // @[TensorGemm.scala 264:39]
    end else if (!(uop_idx != _T_4)) begin // @[TensorGemm.scala 268:45]
      if (cnt_i != _T_7) begin // @[TensorGemm.scala 272:42]
        inp_i <= _inp_i_T_1; // @[TensorGemm.scala 275:15]
      end else if (cnt_o != _T_10) begin // @[TensorGemm.scala 278:44]
        inp_i <= inp_tmp; // @[TensorGemm.scala 288:17]
      end
    end
    if (_T) begin // @[TensorGemm.scala 263:18]
      wgt_i <= 10'h0; // @[TensorGemm.scala 264:53]
    end else if (!(uop_idx != _T_4)) begin // @[TensorGemm.scala 268:45]
      if (cnt_i != _T_7) begin // @[TensorGemm.scala 272:42]
        wgt_i <= _wgt_i_T_1; // @[TensorGemm.scala 276:15]
      end else if (cnt_o != _T_10) begin // @[TensorGemm.scala 278:44]
        wgt_i <= wgt_tmp; // @[TensorGemm.scala 289:17]
      end
    end
    if (_T) begin // @[TensorGemm.scala 263:18]
      cnt_o <= 14'h0; // @[TensorGemm.scala 265:11]
    end else if (!(uop_idx != _T_4)) begin // @[TensorGemm.scala 268:45]
      if (!(cnt_i != _T_7)) begin // @[TensorGemm.scala 272:42]
        if (cnt_o != _T_10) begin // @[TensorGemm.scala 278:44]
          cnt_o <= _cnt_o_T_1; // @[TensorGemm.scala 282:17]
        end
      end
    end
    if (_T) begin // @[TensorGemm.scala 263:18]
      acc_o <= 11'h0; // @[TensorGemm.scala 265:25]
    end else if (!(uop_idx != _T_4)) begin // @[TensorGemm.scala 268:45]
      if (!(cnt_i != _T_7)) begin // @[TensorGemm.scala 272:42]
        if (cnt_o != _T_10) begin // @[TensorGemm.scala 278:44]
          acc_o <= acc_tmp; // @[TensorGemm.scala 283:17]
        end
      end
    end
    if (_T) begin // @[TensorGemm.scala 263:18]
      inp_o <= 11'h0; // @[TensorGemm.scala 265:39]
    end else if (!(uop_idx != _T_4)) begin // @[TensorGemm.scala 268:45]
      if (!(cnt_i != _T_7)) begin // @[TensorGemm.scala 272:42]
        if (cnt_o != _T_10) begin // @[TensorGemm.scala 278:44]
          inp_o <= inp_tmp; // @[TensorGemm.scala 284:17]
        end
      end
    end
    if (_T) begin // @[TensorGemm.scala 263:18]
      wgt_o <= 10'h0; // @[TensorGemm.scala 265:53]
    end else if (!(uop_idx != _T_4)) begin // @[TensorGemm.scala 268:45]
      if (!(cnt_i != _T_7)) begin // @[TensorGemm.scala 272:42]
        if (cnt_o != _T_10) begin // @[TensorGemm.scala 278:44]
          wgt_o <= wgt_tmp; // @[TensorGemm.scala 285:17]
        end
      end
    end
    if (_T) begin // @[TensorGemm.scala 263:18]
      uop_idx <= {{1'd0}, io_dec_uop_begin}; // @[TensorGemm.scala 266:13]
    end else if (uop_idx != _T_4) begin // @[TensorGemm.scala 268:45]
      uop_idx <= _uop_idx_T_1; // @[TensorGemm.scala 269:15]
    end else begin
      uop_idx <= {{1'd0}, io_dec_uop_begin}; // @[TensorGemm.scala 271:15]
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
  running = _RAND_0[0:0];
  _RAND_1 = {1{`RANDOM}};
  cnt_i = _RAND_1[13:0];
  _RAND_2 = {1{`RANDOM}};
  acc_i = _RAND_2[10:0];
  _RAND_3 = {1{`RANDOM}};
  inp_i = _RAND_3[10:0];
  _RAND_4 = {1{`RANDOM}};
  wgt_i = _RAND_4[9:0];
  _RAND_5 = {1{`RANDOM}};
  cnt_o = _RAND_5[13:0];
  _RAND_6 = {1{`RANDOM}};
  acc_o = _RAND_6[10:0];
  _RAND_7 = {1{`RANDOM}};
  inp_o = _RAND_7[10:0];
  _RAND_8 = {1{`RANDOM}};
  wgt_o = _RAND_8[9:0];
  _RAND_9 = {1{`RANDOM}};
  uop_idx = _RAND_9[13:0];
`endif // RANDOMIZE_REG_INIT
  `endif // RANDOMIZE
end // initial
`ifdef FIRRTL_AFTER_INITIAL
`FIRRTL_AFTER_INITIAL
`endif
`endif // SYNTHESIS
endmodule