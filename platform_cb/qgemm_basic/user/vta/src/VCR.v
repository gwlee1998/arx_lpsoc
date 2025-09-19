module VCR(
  input         clock,
  input         reset,
  output        io_host_aw_ready,
  input         io_host_aw_valid,
  input  [15:0] io_host_aw_bits_addr,
  output        io_host_w_ready,
  input         io_host_w_valid,
  input  [31:0] io_host_w_bits_data,
  input         io_host_b_ready,
  output        io_host_b_valid,
  output        io_host_ar_ready,
  input         io_host_ar_valid,
  input  [15:0] io_host_ar_bits_addr,
  input         io_host_r_ready,
  output        io_host_r_valid,
  output [31:0] io_host_r_bits_data,
  output        io_vcr_launch,
  input         io_vcr_finish,
  input         io_vcr_ecnt_0_valid,
  input  [31:0] io_vcr_ecnt_0_bits,
  output [31:0] io_vcr_vals_0,
  output [31:0] io_vcr_ptrs_0,
  output [31:0] io_vcr_ptrs_1,
  output [31:0] io_vcr_ptrs_2,
  output [31:0] io_vcr_ptrs_3,
  output [31:0] io_vcr_ptrs_4,
  output [31:0] io_vcr_ptrs_5,
  input         io_vcr_ucnt_0_valid,
  input  [31:0] io_vcr_ucnt_0_bits
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
  reg [31:0] _RAND_10;
  reg [31:0] _RAND_11;
  reg [31:0] _RAND_12;
  reg [31:0] _RAND_13;
`endif // RANDOMIZE_REG_INIT
  reg [15:0] waddr; // @[VCR.scala 94:22]
  reg [1:0] wstate; // @[VCR.scala 97:23]
  reg  rstate; // @[VCR.scala 101:23]
  reg [31:0] rdata; // @[VCR.scala 102:22]
  reg [31:0] reg_0; // @[VCR.scala 108:37]
  reg [31:0] reg_1; // @[VCR.scala 108:37]
  reg [31:0] reg_2; // @[VCR.scala 108:37]
  reg [31:0] reg_3; // @[VCR.scala 108:37]
  reg [31:0] reg_4; // @[VCR.scala 108:37]
  reg [31:0] reg_5; // @[VCR.scala 108:37]
  reg [31:0] reg_6; // @[VCR.scala 108:37]
  reg [31:0] reg_7; // @[VCR.scala 108:37]
  reg [31:0] reg_8; // @[VCR.scala 108:37]
  reg [31:0] reg_9; // @[VCR.scala 108:37]
  wire  _T = 2'h0 == wstate; // @[Conditional.scala 37:30]
  wire  _T_1 = 2'h1 == wstate; // @[Conditional.scala 37:30]
  wire  _T_2 = 2'h2 == wstate; // @[Conditional.scala 37:30]
  wire [1:0] _GEN_2 = io_host_b_ready ? 2'h0 : wstate; // @[VCR.scala 128:29 VCR.scala 129:16 VCR.scala 97:23]
  wire  _T_3 = io_host_aw_ready & io_host_aw_valid; // @[Decoupled.scala 40:37]
  wire  _T_4 = ~rstate; // @[Conditional.scala 37:30]
  wire  _GEN_7 = io_host_ar_valid | rstate; // @[VCR.scala 143:30 VCR.scala 144:16 VCR.scala 101:23]
  wire  _T_6 = io_host_w_ready & io_host_w_valid; // @[Decoupled.scala 40:37]
  wire  _T_33 = io_host_ar_ready & io_host_ar_valid; // @[Decoupled.scala 40:37]
  wire [31:0] _rdata_T_1 = 16'h0 == io_host_ar_bits_addr ? reg_0 : 32'h0; // @[Mux.scala 80:57]
  wire [31:0] _rdata_T_3 = 16'h4 == io_host_ar_bits_addr ? reg_1 : _rdata_T_1; // @[Mux.scala 80:57]
  wire [31:0] _rdata_T_5 = 16'h8 == io_host_ar_bits_addr ? reg_2 : _rdata_T_3; // @[Mux.scala 80:57]
  wire [31:0] _rdata_T_7 = 16'hc == io_host_ar_bits_addr ? reg_3 : _rdata_T_5; // @[Mux.scala 80:57]
  wire [31:0] _rdata_T_9 = 16'h10 == io_host_ar_bits_addr ? reg_4 : _rdata_T_7; // @[Mux.scala 80:57]
  wire [31:0] _rdata_T_11 = 16'h14 == io_host_ar_bits_addr ? reg_5 : _rdata_T_9; // @[Mux.scala 80:57]
  wire [31:0] _rdata_T_13 = 16'h18 == io_host_ar_bits_addr ? reg_6 : _rdata_T_11; // @[Mux.scala 80:57]
  wire [31:0] _rdata_T_15 = 16'h1c == io_host_ar_bits_addr ? reg_7 : _rdata_T_13; // @[Mux.scala 80:57]
  assign io_host_aw_ready = wstate == 2'h0; // @[VCR.scala 136:30]
  assign io_host_w_ready = wstate == 2'h1; // @[VCR.scala 137:29]
  assign io_host_b_valid = wstate == 2'h2; // @[VCR.scala 138:29]
  assign io_host_ar_ready = ~rstate; // @[VCR.scala 154:30]
  assign io_host_r_valid = rstate; // @[VCR.scala 155:29]
  assign io_host_r_bits_data = rdata; // @[VCR.scala 156:23]
  assign io_vcr_launch = reg_0[0]; // @[VCR.scala 183:26]
  assign io_vcr_vals_0 = reg_2; // @[VCR.scala 186:20]
  assign io_vcr_ptrs_0 = reg_3; // @[VCR.scala 191:22]
  assign io_vcr_ptrs_1 = reg_4; // @[VCR.scala 191:22]
  assign io_vcr_ptrs_2 = reg_5; // @[VCR.scala 191:22]
  assign io_vcr_ptrs_3 = reg_6; // @[VCR.scala 191:22]
  assign io_vcr_ptrs_4 = reg_7; // @[VCR.scala 191:22]
  assign io_vcr_ptrs_5 = reg_8; // @[VCR.scala 191:22]
  always @(posedge clock) begin
    if (reset) begin // @[VCR.scala 94:22]
      waddr <= 16'hffff; // @[VCR.scala 94:22]
    end else if (_T_3) begin // @[VCR.scala 134:27]
      waddr <= io_host_aw_bits_addr; // @[VCR.scala 134:35]
    end
    if (reset) begin // @[VCR.scala 97:23]
      wstate <= 2'h0; // @[VCR.scala 97:23]
    end else if (_T) begin // @[Conditional.scala 40:58]
      if (io_host_aw_valid) begin // @[VCR.scala 118:30]
        wstate <= 2'h1; // @[VCR.scala 119:16]
      end
    end else if (_T_1) begin // @[Conditional.scala 39:67]
      if (io_host_w_valid) begin // @[VCR.scala 123:29]
        wstate <= 2'h2; // @[VCR.scala 124:16]
      end
    end else if (_T_2) begin // @[Conditional.scala 39:67]
      wstate <= _GEN_2;
    end
    if (reset) begin // @[VCR.scala 101:23]
      rstate <= 1'h0; // @[VCR.scala 101:23]
    end else if (_T_4) begin // @[Conditional.scala 40:58]
      rstate <= _GEN_7;
    end else if (rstate) begin // @[Conditional.scala 39:67]
      if (io_host_r_ready) begin // @[VCR.scala 148:29]
        rstate <= 1'h0; // @[VCR.scala 149:16]
      end
    end
    if (reset) begin // @[VCR.scala 102:22]
      rdata <= 32'h0; // @[VCR.scala 102:22]
    end else if (_T_33) begin // @[VCR.scala 179:27]
      if (16'h24 == io_host_ar_bits_addr) begin // @[Mux.scala 80:57]
        rdata <= reg_9;
      end else if (16'h20 == io_host_ar_bits_addr) begin // @[Mux.scala 80:57]
        rdata <= reg_8;
      end else begin
        rdata <= _rdata_T_15;
      end
    end
    if (reset) begin // @[VCR.scala 108:37]
      reg_0 <= 32'h0; // @[VCR.scala 108:37]
    end else if (io_vcr_finish) begin // @[VCR.scala 159:23]
      reg_0 <= 32'h2; // @[VCR.scala 160:12]
    end else if (_T_6 & 16'h0 == waddr) begin // @[VCR.scala 161:55]
      reg_0 <= io_host_w_bits_data; // @[VCR.scala 162:12]
    end
    if (reset) begin // @[VCR.scala 108:37]
      reg_1 <= 32'h0; // @[VCR.scala 108:37]
    end else if (io_vcr_ecnt_0_valid) begin // @[VCR.scala 166:32]
      reg_1 <= io_vcr_ecnt_0_bits; // @[VCR.scala 167:19]
    end else if (_T_6 & 16'h4 == waddr) begin // @[VCR.scala 168:62]
      reg_1 <= io_host_w_bits_data; // @[VCR.scala 169:19]
    end
    if (reset) begin // @[VCR.scala 108:37]
      reg_2 <= 32'h0; // @[VCR.scala 108:37]
    end else if (_T_6 & 16'h8 == waddr) begin // @[VCR.scala 174:56]
      reg_2 <= io_host_w_bits_data; // @[VCR.scala 175:19]
    end
    if (reset) begin // @[VCR.scala 108:37]
      reg_3 <= 32'h0; // @[VCR.scala 108:37]
    end else if (_T_6 & 16'hc == waddr) begin // @[VCR.scala 174:56]
      reg_3 <= io_host_w_bits_data; // @[VCR.scala 175:19]
    end
    if (reset) begin // @[VCR.scala 108:37]
      reg_4 <= 32'h0; // @[VCR.scala 108:37]
    end else if (_T_6 & 16'h10 == waddr) begin // @[VCR.scala 174:56]
      reg_4 <= io_host_w_bits_data; // @[VCR.scala 175:19]
    end
    if (reset) begin // @[VCR.scala 108:37]
      reg_5 <= 32'h0; // @[VCR.scala 108:37]
    end else if (_T_6 & 16'h14 == waddr) begin // @[VCR.scala 174:56]
      reg_5 <= io_host_w_bits_data; // @[VCR.scala 175:19]
    end
    if (reset) begin // @[VCR.scala 108:37]
      reg_6 <= 32'h0; // @[VCR.scala 108:37]
    end else if (_T_6 & 16'h18 == waddr) begin // @[VCR.scala 174:56]
      reg_6 <= io_host_w_bits_data; // @[VCR.scala 175:19]
    end
    if (reset) begin // @[VCR.scala 108:37]
      reg_7 <= 32'h0; // @[VCR.scala 108:37]
    end else if (_T_6 & 16'h1c == waddr) begin // @[VCR.scala 174:56]
      reg_7 <= io_host_w_bits_data; // @[VCR.scala 175:19]
    end
    if (reset) begin // @[VCR.scala 108:37]
      reg_8 <= 32'h0; // @[VCR.scala 108:37]
    end else if (_T_6 & 16'h20 == waddr) begin // @[VCR.scala 174:56]
      reg_8 <= io_host_w_bits_data; // @[VCR.scala 175:19]
    end
    if (reset) begin // @[VCR.scala 108:37]
      reg_9 <= 32'h0; // @[VCR.scala 108:37]
    end else if (io_vcr_ucnt_0_valid) begin // @[VCR.scala 200:32]
      reg_9 <= io_vcr_ucnt_0_bits; // @[VCR.scala 201:19]
    end else if (_T_6 & 16'h24 == waddr) begin // @[VCR.scala 202:62]
      reg_9 <= io_host_w_bits_data; // @[VCR.scala 203:19]
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
  waddr = _RAND_0[15:0];
  _RAND_1 = {1{`RANDOM}};
  wstate = _RAND_1[1:0];
  _RAND_2 = {1{`RANDOM}};
  rstate = _RAND_2[0:0];
  _RAND_3 = {1{`RANDOM}};
  rdata = _RAND_3[31:0];
  _RAND_4 = {1{`RANDOM}};
  reg_0 = _RAND_4[31:0];
  _RAND_5 = {1{`RANDOM}};
  reg_1 = _RAND_5[31:0];
  _RAND_6 = {1{`RANDOM}};
  reg_2 = _RAND_6[31:0];
  _RAND_7 = {1{`RANDOM}};
  reg_3 = _RAND_7[31:0];
  _RAND_8 = {1{`RANDOM}};
  reg_4 = _RAND_8[31:0];
  _RAND_9 = {1{`RANDOM}};
  reg_5 = _RAND_9[31:0];
  _RAND_10 = {1{`RANDOM}};
  reg_6 = _RAND_10[31:0];
  _RAND_11 = {1{`RANDOM}};
  reg_7 = _RAND_11[31:0];
  _RAND_12 = {1{`RANDOM}};
  reg_8 = _RAND_12[31:0];
  _RAND_13 = {1{`RANDOM}};
  reg_9 = _RAND_13[31:0];
`endif // RANDOMIZE_REG_INIT
  `endif // RANDOMIZE
end // initial
`ifdef FIRRTL_AFTER_INITIAL
`FIRRTL_AFTER_INITIAL
`endif
`endif // SYNTHESIS
endmodule