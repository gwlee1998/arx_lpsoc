module Fetch64Bit(
  input          clock,
  input          reset,
  input          io_launch,
  input  [31:0]  io_ins_baddr,
  input  [31:0]  io_ins_count,
  input          io_vme_rd_cmd_ready,
  output         io_vme_rd_cmd_valid,
  output [31:0]  io_vme_rd_cmd_bits_addr,
  output [7:0]   io_vme_rd_cmd_bits_len,
  output         io_vme_rd_data_ready,
  input          io_vme_rd_data_valid,
  input  [63:0]  io_vme_rd_data_bits_data,
  input          io_inst_ld_ready,
  output         io_inst_ld_valid,
  output [127:0] io_inst_ld_bits,
  input          io_inst_co_ready,
  output         io_inst_co_valid,
  output [127:0] io_inst_co_bits,
  input          io_inst_st_ready,
  output         io_inst_st_valid,
  output [127:0] io_inst_st_bits
);
`ifdef RANDOMIZE_REG_INIT
  reg [31:0] _RAND_0;
  reg [31:0] _RAND_1;
  reg [31:0] _RAND_2;
  reg [31:0] _RAND_3;
  reg [31:0] _RAND_4;
  reg [31:0] _RAND_5;
  reg [63:0] _RAND_6;
`endif // RANDOMIZE_REG_INIT
  wire  inst_q_clock; // @[FetchVME64.scala 58:22]
  wire  inst_q_reset; // @[FetchVME64.scala 58:22]
  wire  inst_q_io_enq_ready; // @[FetchVME64.scala 58:22]
  wire  inst_q_io_enq_valid; // @[FetchVME64.scala 58:22]
  wire [127:0] inst_q_io_enq_bits; // @[FetchVME64.scala 58:22]
  wire  inst_q_io_deq_ready; // @[FetchVME64.scala 58:22]
  wire  inst_q_io_deq_valid; // @[FetchVME64.scala 58:22]
  wire [127:0] inst_q_io_deq_bits; // @[FetchVME64.scala 58:22]
  wire [7:0] inst_q_io_count; // @[FetchVME64.scala 58:22]
  wire [127:0] dec_io_inst; // @[FetchVME64.scala 59:19]
  wire  dec_io_isLoad; // @[FetchVME64.scala 59:19]
  wire  dec_io_isCompute; // @[FetchVME64.scala 59:19]
  wire  dec_io_isStore; // @[FetchVME64.scala 59:19]
  reg  s1_launch; // @[FetchVME64.scala 61:26]
  wire  pulse = io_launch & ~s1_launch; // @[FetchVME64.scala 62:25]
  reg [31:0] raddr; // @[FetchVME64.scala 64:18]
  reg [7:0] rlen; // @[FetchVME64.scala 65:17]
  reg [7:0] ilen; // @[FetchVME64.scala 66:17]
  reg [31:0] xrem; // @[FetchVME64.scala 68:17]
  wire [32:0] _xsize_T = {io_ins_count, 1'h0}; // @[FetchVME64.scala 69:29]
  wire [32:0] xsize = _xsize_T - 33'h1; // @[FetchVME64.scala 69:37]
  reg [2:0] state; // @[FetchVME64.scala 74:22]
  wire  _T = 3'h0 == state; // @[Conditional.scala 37:30]
  wire [32:0] _ilen_T = {{1'd0}, xsize[32:1]}; // @[FetchVME64.scala 83:25]
  wire [8:0] _rlen_T_1 = 9'h100 - 9'h1; // @[FetchVME64.scala 86:24]
  wire [8:0] _ilen_T_3 = 9'h80 - 9'h1; // @[FetchVME64.scala 87:33]
  wire [32:0] _xrem_T_1 = xsize - 33'h100; // @[FetchVME64.scala 88:25]
  wire [32:0] _GEN_0 = xsize < 33'h100 ? xsize : {{24'd0}, _rlen_T_1}; // @[FetchVME64.scala 81:28 FetchVME64.scala 82:16 FetchVME64.scala 86:16]
  wire [32:0] _GEN_1 = xsize < 33'h100 ? _ilen_T : {{24'd0}, _ilen_T_3}; // @[FetchVME64.scala 81:28 FetchVME64.scala 83:16 FetchVME64.scala 87:16]
  wire [32:0] _GEN_2 = xsize < 33'h100 ? 33'h0 : _xrem_T_1; // @[FetchVME64.scala 81:28 FetchVME64.scala 84:16 FetchVME64.scala 88:16]
  wire [32:0] _GEN_4 = pulse ? _GEN_0 : {{25'd0}, rlen}; // @[FetchVME64.scala 79:19 FetchVME64.scala 65:17]
  wire [32:0] _GEN_5 = pulse ? _GEN_1 : {{25'd0}, ilen}; // @[FetchVME64.scala 79:19 FetchVME64.scala 66:17]
  wire [32:0] _GEN_6 = pulse ? _GEN_2 : {{1'd0}, xrem}; // @[FetchVME64.scala 79:19 FetchVME64.scala 68:17]
  wire  _T_2 = 3'h1 == state; // @[Conditional.scala 37:30]
  wire  _T_3 = 3'h2 == state; // @[Conditional.scala 37:30]
  wire [2:0] _GEN_8 = io_vme_rd_data_valid ? 3'h3 : state; // @[FetchVME64.scala 98:34 FetchVME64.scala 99:15 FetchVME64.scala 74:22]
  wire  _T_4 = 3'h3 == state; // @[Conditional.scala 37:30]
  wire [2:0] _GEN_9 = inst_q_io_count == ilen ? 3'h4 : 3'h2; // @[FetchVME64.scala 104:40 FetchVME64.scala 105:17 FetchVME64.scala 107:17]
  wire [2:0] _GEN_10 = io_vme_rd_data_valid ? _GEN_9 : state; // @[FetchVME64.scala 103:34 FetchVME64.scala 74:22]
  wire  _T_6 = 3'h4 == state; // @[Conditional.scala 37:30]
  wire  _T_7 = inst_q_io_count == 8'h0; // @[FetchVME64.scala 112:28]
  wire [31:0] _ilen_T_4 = {{1'd0}, xrem[31:1]}; // @[FetchVME64.scala 118:24]
  wire [31:0] _xrem_T_3 = xrem - 32'h100; // @[FetchVME64.scala 124:24]
  wire [31:0] _GEN_12 = xrem < 32'h100 ? xrem : {{23'd0}, _rlen_T_1}; // @[FetchVME64.scala 115:33 FetchVME64.scala 117:16 FetchVME64.scala 122:16]
  wire [31:0] _GEN_13 = xrem < 32'h100 ? _ilen_T_4 : {{23'd0}, _ilen_T_3}; // @[FetchVME64.scala 115:33 FetchVME64.scala 118:16 FetchVME64.scala 123:16]
  wire [31:0] _GEN_14 = xrem < 32'h100 ? 32'h0 : _xrem_T_3; // @[FetchVME64.scala 115:33 FetchVME64.scala 119:16 FetchVME64.scala 124:16]
  wire [2:0] _GEN_15 = xrem == 32'h0 ? 3'h0 : 3'h1; // @[FetchVME64.scala 113:28 FetchVME64.scala 114:17]
  wire [31:0] _GEN_16 = xrem == 32'h0 ? {{24'd0}, rlen} : _GEN_12; // @[FetchVME64.scala 113:28 FetchVME64.scala 65:17]
  wire [31:0] _GEN_17 = xrem == 32'h0 ? {{24'd0}, ilen} : _GEN_13; // @[FetchVME64.scala 113:28 FetchVME64.scala 66:17]
  wire [31:0] _GEN_18 = xrem == 32'h0 ? xrem : _GEN_14; // @[FetchVME64.scala 113:28 FetchVME64.scala 68:17]
  wire [2:0] _GEN_19 = inst_q_io_count == 8'h0 ? _GEN_15 : state; // @[FetchVME64.scala 112:37 FetchVME64.scala 74:22]
  wire [31:0] _GEN_20 = inst_q_io_count == 8'h0 ? _GEN_16 : {{24'd0}, rlen}; // @[FetchVME64.scala 112:37 FetchVME64.scala 65:17]
  wire [31:0] _GEN_21 = inst_q_io_count == 8'h0 ? _GEN_17 : {{24'd0}, ilen}; // @[FetchVME64.scala 112:37 FetchVME64.scala 66:17]
  wire [31:0] _GEN_22 = inst_q_io_count == 8'h0 ? _GEN_18 : xrem; // @[FetchVME64.scala 112:37 FetchVME64.scala 68:17]
  wire [2:0] _GEN_23 = _T_6 ? _GEN_19 : state; // @[Conditional.scala 39:67 FetchVME64.scala 74:22]
  wire [31:0] _GEN_24 = _T_6 ? _GEN_20 : {{24'd0}, rlen}; // @[Conditional.scala 39:67 FetchVME64.scala 65:17]
  wire [31:0] _GEN_25 = _T_6 ? _GEN_21 : {{24'd0}, ilen}; // @[Conditional.scala 39:67 FetchVME64.scala 66:17]
  wire [31:0] _GEN_26 = _T_6 ? _GEN_22 : xrem; // @[Conditional.scala 39:67 FetchVME64.scala 68:17]
  wire [2:0] _GEN_27 = _T_4 ? _GEN_10 : _GEN_23; // @[Conditional.scala 39:67]
  wire [31:0] _GEN_28 = _T_4 ? {{24'd0}, rlen} : _GEN_24; // @[Conditional.scala 39:67 FetchVME64.scala 65:17]
  wire [31:0] _GEN_29 = _T_4 ? {{24'd0}, ilen} : _GEN_25; // @[Conditional.scala 39:67 FetchVME64.scala 66:17]
  wire [31:0] _GEN_30 = _T_4 ? xrem : _GEN_26; // @[Conditional.scala 39:67 FetchVME64.scala 68:17]
  wire [31:0] _GEN_32 = _T_3 ? {{24'd0}, rlen} : _GEN_28; // @[Conditional.scala 39:67 FetchVME64.scala 65:17]
  wire [31:0] _GEN_33 = _T_3 ? {{24'd0}, ilen} : _GEN_29; // @[Conditional.scala 39:67 FetchVME64.scala 66:17]
  wire [31:0] _GEN_34 = _T_3 ? xrem : _GEN_30; // @[Conditional.scala 39:67 FetchVME64.scala 68:17]
  wire [31:0] _GEN_36 = _T_2 ? {{24'd0}, rlen} : _GEN_32; // @[Conditional.scala 39:67 FetchVME64.scala 65:17]
  wire [31:0] _GEN_37 = _T_2 ? {{24'd0}, ilen} : _GEN_33; // @[Conditional.scala 39:67 FetchVME64.scala 66:17]
  wire [31:0] _GEN_38 = _T_2 ? xrem : _GEN_34; // @[Conditional.scala 39:67 FetchVME64.scala 68:17]
  wire [32:0] _GEN_40 = _T ? _GEN_4 : {{1'd0}, _GEN_36}; // @[Conditional.scala 40:58]
  wire [32:0] _GEN_41 = _T ? _GEN_5 : {{1'd0}, _GEN_37}; // @[Conditional.scala 40:58]
  wire [32:0] _GEN_42 = _T ? _GEN_6 : {{1'd0}, _GEN_38}; // @[Conditional.scala 40:58]
  wire  _T_11 = state == 3'h4; // @[FetchVME64.scala 133:20]
  wire [31:0] _raddr_T_1 = raddr + 32'h800; // @[FetchVME64.scala 134:20]
  reg [63:0] lsb; // @[FetchVME64.scala 144:16]
  wire [2:0] deq_sel = {dec_io_isCompute,dec_io_isStore,dec_io_isLoad}; // @[Cat.scala 30:58]
  wire  _deq_ready_T_3 = 3'h2 == deq_sel ? io_inst_st_ready : 3'h1 == deq_sel & io_inst_ld_ready; // @[Mux.scala 80:57]
  wire  deq_ready = 3'h4 == deq_sel ? io_inst_co_ready : _deq_ready_T_3; // @[Mux.scala 80:57]
  SyncQueue inst_q ( // @[FetchVME64.scala 58:22]
    .clock(inst_q_clock),
    .reset(inst_q_reset),
    .io_enq_ready(inst_q_io_enq_ready),
    .io_enq_valid(inst_q_io_enq_valid),
    .io_enq_bits(inst_q_io_enq_bits),
    .io_deq_ready(inst_q_io_deq_ready),
    .io_deq_valid(inst_q_io_deq_valid),
    .io_deq_bits(inst_q_io_deq_bits),
    .io_count(inst_q_io_count)
  );
  FetchDecode dec ( // @[FetchVME64.scala 59:19]
    .io_inst(dec_io_inst),
    .io_isLoad(dec_io_isLoad),
    .io_isCompute(dec_io_isCompute),
    .io_isStore(dec_io_isStore)
  );
  assign io_vme_rd_cmd_valid = state == 3'h1; // @[FetchVME64.scala 137:32]
  assign io_vme_rd_cmd_bits_addr = raddr; // @[FetchVME64.scala 138:27]
  assign io_vme_rd_cmd_bits_len = rlen; // @[FetchVME64.scala 139:26]
  assign io_vme_rd_data_ready = inst_q_io_enq_ready; // @[FetchVME64.scala 142:24]
  assign io_inst_ld_valid = dec_io_isLoad & inst_q_io_deq_valid & _T_11; // @[FetchVME64.scala 157:59]
  assign io_inst_ld_bits = inst_q_io_deq_bits; // @[FetchVME64.scala 164:19]
  assign io_inst_co_valid = dec_io_isCompute & inst_q_io_deq_valid & _T_11; // @[FetchVME64.scala 158:62]
  assign io_inst_co_bits = inst_q_io_deq_bits; // @[FetchVME64.scala 165:19]
  assign io_inst_st_valid = dec_io_isStore & inst_q_io_deq_valid & _T_11; // @[FetchVME64.scala 159:60]
  assign io_inst_st_bits = inst_q_io_deq_bits; // @[FetchVME64.scala 166:19]
  assign inst_q_clock = clock;
  assign inst_q_reset = reset;
  assign inst_q_io_enq_valid = io_vme_rd_data_valid & state == 3'h3; // @[FetchVME64.scala 150:47]
  assign inst_q_io_enq_bits = {io_vme_rd_data_bits_data,lsb}; // @[Cat.scala 30:58]
  assign inst_q_io_deq_ready = deq_ready & inst_q_io_deq_valid & _T_11; // @[FetchVME64.scala 180:58]
  assign dec_io_inst = inst_q_io_deq_bits; // @[FetchVME64.scala 154:15]
  always @(posedge clock) begin
    if (reset) begin // @[FetchVME64.scala 61:26]
      s1_launch <= 1'h0; // @[FetchVME64.scala 61:26]
    end else begin
      s1_launch <= io_launch; // @[FetchVME64.scala 61:26]
    end
    if (state == 3'h0) begin // @[FetchVME64.scala 131:25]
      raddr <= io_ins_baddr; // @[FetchVME64.scala 132:11]
    end else if (state == 3'h4 & _T_7 & xrem != 32'h0) begin // @[FetchVME64.scala 133:75]
      raddr <= _raddr_T_1; // @[FetchVME64.scala 134:11]
    end
    rlen <= _GEN_40[7:0];
    ilen <= _GEN_41[7:0];
    xrem <= _GEN_42[31:0];
    if (reset) begin // @[FetchVME64.scala 74:22]
      state <= 3'h0; // @[FetchVME64.scala 74:22]
    end else if (_T) begin // @[Conditional.scala 40:58]
      if (pulse) begin // @[FetchVME64.scala 79:19]
        state <= 3'h1; // @[FetchVME64.scala 80:15]
      end
    end else if (_T_2) begin // @[Conditional.scala 39:67]
      if (io_vme_rd_cmd_ready) begin // @[FetchVME64.scala 93:33]
        state <= 3'h2; // @[FetchVME64.scala 94:15]
      end
    end else if (_T_3) begin // @[Conditional.scala 39:67]
      state <= _GEN_8;
    end else begin
      state <= _GEN_27;
    end
    if (state == 3'h2) begin // @[FetchVME64.scala 148:28]
      lsb <= io_vme_rd_data_bits_data; // @[FetchVME64.scala 148:34]
    end
    `ifndef SYNTHESIS
    `ifdef PRINTF_COND
      if (`PRINTF_COND) begin
    `endif
        if (~(~(inst_q_io_deq_valid & _T_11) | dec_io_isLoad | dec_io_isCompute | dec_io_isStore | reset)) begin
          $fwrite(32'h80000002,
            "Assertion failed: -F- Fetch: Unknown instruction type\n    at FetchVME64.scala:161 assert(!(inst_q.io.deq.valid & state === sDrain) || dec.io.isLoad || dec.io.isCompute || dec.io.isStore,\n"
            ); // @[FetchVME64.scala 161:9]
        end
    `ifdef PRINTF_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef STOP_COND
      if (`STOP_COND) begin
    `endif
        if (~(~(inst_q_io_deq_valid & _T_11) | dec_io_isLoad | dec_io_isCompute | dec_io_isStore | reset)) begin
          $fatal; // @[FetchVME64.scala 161:9]
        end
    `ifdef STOP_COND
      end
    `endif
    `endif // SYNTHESIS
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
  s1_launch = _RAND_0[0:0];
  _RAND_1 = {1{`RANDOM}};
  raddr = _RAND_1[31:0];
  _RAND_2 = {1{`RANDOM}};
  rlen = _RAND_2[7:0];
  _RAND_3 = {1{`RANDOM}};
  ilen = _RAND_3[7:0];
  _RAND_4 = {1{`RANDOM}};
  xrem = _RAND_4[31:0];
  _RAND_5 = {1{`RANDOM}};
  state = _RAND_5[2:0];
  _RAND_6 = {2{`RANDOM}};
  lsb = _RAND_6[63:0];
`endif // RANDOMIZE_REG_INIT
  `endif // RANDOMIZE
end // initial
`ifdef FIRRTL_AFTER_INITIAL
`FIRRTL_AFTER_INITIAL
`endif
`endif // SYNTHESIS
endmodule