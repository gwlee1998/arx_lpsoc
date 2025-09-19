module TensorAluIndexGenerator(
  input         clock,
  input         reset,
  input         io_start,
  output        io_last,
  input         io_dec_alu_use_imm,
  input  [10:0] io_dec_src_1,
  input  [10:0] io_dec_src_0,
  input  [10:0] io_dec_dst_1,
  input  [10:0] io_dec_dst_0,
  input  [13:0] io_dec_lp_1,
  input  [13:0] io_dec_lp_0,
  input  [13:0] io_dec_uop_end,
  input  [12:0] io_dec_uop_begin,
  output        io_valid,
  output        io_src_valid,
  output [10:0] io_dst_idx,
  output [10:0] io_src_idx,
  output [10:0] io_uop_idx
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
`endif // RANDOMIZE_REG_INIT
  reg  running; // @[TensorAlu.scala 119:24]
  reg  stutter; // @[TensorAlu.scala 120:24]
  wire  advance = io_dec_alu_use_imm | stutter; // @[TensorAlu.scala 122:36]
  wire  _T = ~running; // @[TensorAlu.scala 124:8]
  wire  _T_2 = ~advance; // @[TensorAlu.scala 126:26]
  wire  _GEN_0 = io_last ? 1'h0 : running; // @[TensorAlu.scala 129:20 TensorAlu.scala 130:15 TensorAlu.scala 119:24]
  wire  _GEN_1 = running & advance ? _GEN_0 : running; // @[TensorAlu.scala 128:35 TensorAlu.scala 119:24]
  wire  _GEN_2 = running & advance ? 1'h0 : stutter; // @[TensorAlu.scala 128:35 TensorAlu.scala 132:13 TensorAlu.scala 120:24]
  wire  _GEN_3 = running & ~advance | _GEN_2; // @[TensorAlu.scala 126:36 TensorAlu.scala 127:13]
  wire  _GEN_4 = running & ~advance ? running : _GEN_1; // @[TensorAlu.scala 126:36 TensorAlu.scala 119:24]
  wire  _GEN_5 = ~running & io_start | _GEN_4; // @[TensorAlu.scala 124:30 TensorAlu.scala 125:13]
  reg [13:0] cnt_i; // @[TensorAlu.scala 135:18]
  reg [10:0] dst_i; // @[TensorAlu.scala 136:18]
  reg [10:0] src_i; // @[TensorAlu.scala 137:18]
  reg [13:0] cnt_o; // @[TensorAlu.scala 139:18]
  reg [10:0] dst_o; // @[TensorAlu.scala 140:18]
  reg [10:0] src_o; // @[TensorAlu.scala 141:18]
  reg [13:0] uop_idx; // @[TensorAlu.scala 143:20]
  wire [13:0] _T_7 = io_dec_uop_end - 14'h1; // @[TensorAlu.scala 158:38]
  wire [13:0] _uop_idx_T_1 = uop_idx + 14'h1; // @[TensorAlu.scala 159:26]
  wire [13:0] _T_10 = io_dec_lp_1 - 14'h1; // @[TensorAlu.scala 162:35]
  wire [13:0] _cnt_i_T_1 = cnt_i + 14'h1; // @[TensorAlu.scala 163:24]
  wire [10:0] _dst_i_T_1 = dst_i + io_dec_dst_1; // @[TensorAlu.scala 164:24]
  wire [10:0] _src_i_T_1 = src_i + io_dec_src_1; // @[TensorAlu.scala 165:24]
  wire [13:0] _T_13 = io_dec_lp_0 - 14'h1; // @[TensorAlu.scala 167:37]
  wire [10:0] dst_tmp = dst_o + io_dec_dst_0; // @[TensorAlu.scala 168:31]
  wire [10:0] src_tmp = src_o + io_dec_src_0; // @[TensorAlu.scala 169:31]
  wire [13:0] _cnt_o_T_1 = cnt_o + 14'h1; // @[TensorAlu.scala 170:26]
  wire [13:0] _GEN_7 = cnt_o != _T_13 ? _cnt_o_T_1 : cnt_o; // @[TensorAlu.scala 167:44 TensorAlu.scala 170:17 TensorAlu.scala 139:18]
  wire [10:0] _GEN_8 = cnt_o != _T_13 ? dst_tmp : dst_o; // @[TensorAlu.scala 167:44 TensorAlu.scala 171:17 TensorAlu.scala 140:18]
  wire [10:0] _GEN_9 = cnt_o != _T_13 ? src_tmp : src_o; // @[TensorAlu.scala 167:44 TensorAlu.scala 172:17 TensorAlu.scala 141:18]
  wire [13:0] _GEN_10 = cnt_o != _T_13 ? 14'h0 : cnt_i; // @[TensorAlu.scala 167:44 TensorAlu.scala 173:17 TensorAlu.scala 135:18]
  wire [10:0] _GEN_11 = cnt_o != _T_13 ? dst_tmp : dst_i; // @[TensorAlu.scala 167:44 TensorAlu.scala 174:17 TensorAlu.scala 136:18]
  wire [10:0] _GEN_12 = cnt_o != _T_13 ? src_tmp : src_i; // @[TensorAlu.scala 167:44 TensorAlu.scala 175:17 TensorAlu.scala 137:18]
  wire  _GEN_13 = cnt_o != _T_13 ? 1'h0 : 1'h1; // @[TensorAlu.scala 167:44 TensorAlu.scala 117:11 TensorAlu.scala 177:19]
  wire  _GEN_20 = cnt_i != _T_10 ? 1'h0 : _GEN_13; // @[TensorAlu.scala 162:42 TensorAlu.scala 117:11]
  wire  _GEN_28 = uop_idx != _T_7 ? 1'h0 : _GEN_20; // @[TensorAlu.scala 158:45 TensorAlu.scala 117:11]
  wire  _GEN_36 = advance & _GEN_28; // @[TensorAlu.scala 157:25 TensorAlu.scala 117:11]
  assign io_last = _T ? 1'h0 : _GEN_36; // @[TensorAlu.scala 153:18 TensorAlu.scala 117:11]
  assign io_valid = running & advance; // @[TensorAlu.scala 145:23]
  assign io_src_valid = running & _T_2; // @[TensorAlu.scala 146:27]
  assign io_dst_idx = dst_i; // @[TensorAlu.scala 147:14]
  assign io_src_idx = src_i; // @[TensorAlu.scala 148:14]
  assign io_uop_idx = uop_idx[10:0]; // @[TensorAlu.scala 149:14]
  always @(posedge clock) begin
    if (reset) begin // @[TensorAlu.scala 119:24]
      running <= 1'h0; // @[TensorAlu.scala 119:24]
    end else begin
      running <= _GEN_5;
    end
    if (reset) begin // @[TensorAlu.scala 120:24]
      stutter <= 1'h0; // @[TensorAlu.scala 120:24]
    end else if (!(~running & io_start)) begin // @[TensorAlu.scala 124:30]
      stutter <= _GEN_3;
    end
    if (_T) begin // @[TensorAlu.scala 153:18]
      cnt_i <= 14'h0; // @[TensorAlu.scala 154:11]
    end else if (advance) begin // @[TensorAlu.scala 157:25]
      if (!(uop_idx != _T_7)) begin // @[TensorAlu.scala 158:45]
        if (cnt_i != _T_10) begin // @[TensorAlu.scala 162:42]
          cnt_i <= _cnt_i_T_1; // @[TensorAlu.scala 163:15]
        end else begin
          cnt_i <= _GEN_10;
        end
      end
    end
    if (_T) begin // @[TensorAlu.scala 153:18]
      dst_i <= 11'h0; // @[TensorAlu.scala 154:25]
    end else if (advance) begin // @[TensorAlu.scala 157:25]
      if (!(uop_idx != _T_7)) begin // @[TensorAlu.scala 158:45]
        if (cnt_i != _T_10) begin // @[TensorAlu.scala 162:42]
          dst_i <= _dst_i_T_1; // @[TensorAlu.scala 164:15]
        end else begin
          dst_i <= _GEN_11;
        end
      end
    end
    if (_T) begin // @[TensorAlu.scala 153:18]
      src_i <= 11'h0; // @[TensorAlu.scala 154:39]
    end else if (advance) begin // @[TensorAlu.scala 157:25]
      if (!(uop_idx != _T_7)) begin // @[TensorAlu.scala 158:45]
        if (cnt_i != _T_10) begin // @[TensorAlu.scala 162:42]
          src_i <= _src_i_T_1; // @[TensorAlu.scala 165:15]
        end else begin
          src_i <= _GEN_12;
        end
      end
    end
    if (_T) begin // @[TensorAlu.scala 153:18]
      cnt_o <= 14'h0; // @[TensorAlu.scala 155:11]
    end else if (advance) begin // @[TensorAlu.scala 157:25]
      if (!(uop_idx != _T_7)) begin // @[TensorAlu.scala 158:45]
        if (!(cnt_i != _T_10)) begin // @[TensorAlu.scala 162:42]
          cnt_o <= _GEN_7;
        end
      end
    end
    if (_T) begin // @[TensorAlu.scala 153:18]
      dst_o <= 11'h0; // @[TensorAlu.scala 155:25]
    end else if (advance) begin // @[TensorAlu.scala 157:25]
      if (!(uop_idx != _T_7)) begin // @[TensorAlu.scala 158:45]
        if (!(cnt_i != _T_10)) begin // @[TensorAlu.scala 162:42]
          dst_o <= _GEN_8;
        end
      end
    end
    if (_T) begin // @[TensorAlu.scala 153:18]
      src_o <= 11'h0; // @[TensorAlu.scala 155:39]
    end else if (advance) begin // @[TensorAlu.scala 157:25]
      if (!(uop_idx != _T_7)) begin // @[TensorAlu.scala 158:45]
        if (!(cnt_i != _T_10)) begin // @[TensorAlu.scala 162:42]
          src_o <= _GEN_9;
        end
      end
    end
    if (_T) begin // @[TensorAlu.scala 153:18]
      uop_idx <= {{1'd0}, io_dec_uop_begin}; // @[TensorAlu.scala 156:13]
    end else if (advance) begin // @[TensorAlu.scala 157:25]
      if (uop_idx != _T_7) begin // @[TensorAlu.scala 158:45]
        uop_idx <= _uop_idx_T_1; // @[TensorAlu.scala 159:15]
      end else begin
        uop_idx <= {{1'd0}, io_dec_uop_begin}; // @[TensorAlu.scala 161:15]
      end
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
  stutter = _RAND_1[0:0];
  _RAND_2 = {1{`RANDOM}};
  cnt_i = _RAND_2[13:0];
  _RAND_3 = {1{`RANDOM}};
  dst_i = _RAND_3[10:0];
  _RAND_4 = {1{`RANDOM}};
  src_i = _RAND_4[10:0];
  _RAND_5 = {1{`RANDOM}};
  cnt_o = _RAND_5[13:0];
  _RAND_6 = {1{`RANDOM}};
  dst_o = _RAND_6[10:0];
  _RAND_7 = {1{`RANDOM}};
  src_o = _RAND_7[10:0];
  _RAND_8 = {1{`RANDOM}};
  uop_idx = _RAND_8[13:0];
`endif // RANDOMIZE_REG_INIT
  `endif // RANDOMIZE
end // initial
`ifdef FIRRTL_AFTER_INITIAL
`FIRRTL_AFTER_INITIAL
`endif
`endif // SYNTHESIS
endmodule