module TensorLoadWideVME(
  input          clock,
  input          reset,
  input          io_start,
  output         io_done,
  input  [127:0] io_inst,
  input  [31:0]  io_baddr,
  input          io_vme_rd_cmd_ready,
  output         io_vme_rd_cmd_valid,
  output [31:0]  io_vme_rd_cmd_bits_addr,
  output [7:0]   io_vme_rd_cmd_bits_len,
  output [20:0]  io_vme_rd_cmd_bits_tag,
  output         io_vme_rd_data_ready,
  input          io_vme_rd_data_valid,
  input  [63:0]  io_vme_rd_data_bits_data,
  input  [20:0]  io_vme_rd_data_bits_tag,
  input          io_vme_rd_data_bits_last,
  input          io_tensor_rd_0_idx_valid,
  input  [10:0]  io_tensor_rd_0_idx_bits,
  output         io_tensor_rd_0_data_valid,
  output [31:0]  io_tensor_rd_0_data_bits_0_0
);
`ifdef RANDOMIZE_MEM_INIT
  reg [31:0] _RAND_0;
  reg [31:0] _RAND_3;
`endif // RANDOMIZE_MEM_INIT
`ifdef RANDOMIZE_REG_INIT
  reg [31:0] _RAND_1;
  reg [31:0] _RAND_2;
  reg [31:0] _RAND_4;
  reg [31:0] _RAND_5;
  reg [31:0] _RAND_6;
  reg [31:0] _RAND_7;
  reg [31:0] _RAND_8;
  reg [31:0] _RAND_9;
`endif // RANDOMIZE_REG_INIT
  wire  vmeCmd_clock; // @[TensorLoadWideVME.scala 105:23]
  wire  vmeCmd_reset; // @[TensorLoadWideVME.scala 105:23]
  wire  vmeCmd_io_start; // @[TensorLoadWideVME.scala 105:23]
  wire  vmeCmd_io_isBusy; // @[TensorLoadWideVME.scala 105:23]
  wire [127:0] vmeCmd_io_inst; // @[TensorLoadWideVME.scala 105:23]
  wire [31:0] vmeCmd_io_baddr; // @[TensorLoadWideVME.scala 105:23]
  wire  vmeCmd_io_vmeCmd_ready; // @[TensorLoadWideVME.scala 105:23]
  wire  vmeCmd_io_vmeCmd_valid; // @[TensorLoadWideVME.scala 105:23]
  wire [31:0] vmeCmd_io_vmeCmd_bits_addr; // @[TensorLoadWideVME.scala 105:23]
  wire [7:0] vmeCmd_io_vmeCmd_bits_len; // @[TensorLoadWideVME.scala 105:23]
  wire [20:0] vmeCmd_io_vmeCmd_bits_tag; // @[TensorLoadWideVME.scala 105:23]
  wire [8:0] vmeCmd_io_readLen; // @[TensorLoadWideVME.scala 105:23]
  wire  vmeCmd_io_done; // @[TensorLoadWideVME.scala 105:23]
  wire  readData_clock; // @[TensorLoadWideVME.scala 150:24]
  wire  readData_reset; // @[TensorLoadWideVME.scala 150:24]
  wire  readData_io_start; // @[TensorLoadWideVME.scala 150:24]
  wire  readData_io_vmeData_ready; // @[TensorLoadWideVME.scala 150:24]
  wire  readData_io_vmeData_valid; // @[TensorLoadWideVME.scala 150:24]
  wire [63:0] readData_io_vmeData_bits_data; // @[TensorLoadWideVME.scala 150:24]
  wire [20:0] readData_io_vmeData_bits_tag; // @[TensorLoadWideVME.scala 150:24]
  wire  readData_io_vmeData_bits_last; // @[TensorLoadWideVME.scala 150:24]
  wire [10:0] readData_io_destIdx_0; // @[TensorLoadWideVME.scala 150:24]
  wire [10:0] readData_io_destIdx_1; // @[TensorLoadWideVME.scala 150:24]
  wire [31:0] readData_io_destData_0; // @[TensorLoadWideVME.scala 150:24]
  wire [31:0] readData_io_destData_1; // @[TensorLoadWideVME.scala 150:24]
  wire  readData_io_destMask_0; // @[TensorLoadWideVME.scala 150:24]
  wire  readData_io_destMask_1; // @[TensorLoadWideVME.scala 150:24]
  wire  fillPadding_clock; // @[TensorLoadWideVME.scala 166:27]
  wire  fillPadding_reset; // @[TensorLoadWideVME.scala 166:27]
  wire  fillPadding_io_canWriteMem; // @[TensorLoadWideVME.scala 166:27]
  wire [127:0] fillPadding_io_inst; // @[TensorLoadWideVME.scala 166:27]
  wire  fillPadding_io_tensorIdx_valid; // @[TensorLoadWideVME.scala 166:27]
  wire [10:0] fillPadding_io_tensorIdx_bits; // @[TensorLoadWideVME.scala 166:27]
  wire  fillPadding_io_start; // @[TensorLoadWideVME.scala 166:27]
  wire  fillPadding_io_done; // @[TensorLoadWideVME.scala 166:27]
  reg [31:0] tensorFile_0 [0:1023]; // @[TensorLoadWideVME.scala 193:16]
  wire [31:0] tensorFile_0_rdataVec_MPORT_data; // @[TensorLoadWideVME.scala 193:16]
  wire [9:0] tensorFile_0_rdataVec_MPORT_addr; // @[TensorLoadWideVME.scala 193:16]
  wire [31:0] tensorFile_0_MPORT_data; // @[TensorLoadWideVME.scala 193:16]
  wire [9:0] tensorFile_0_MPORT_addr; // @[TensorLoadWideVME.scala 193:16]
  wire  tensorFile_0_MPORT_mask; // @[TensorLoadWideVME.scala 193:16]
  wire  tensorFile_0_MPORT_en; // @[TensorLoadWideVME.scala 193:16]
  reg  tensorFile_0_rdataVec_MPORT_en_pipe_0;
  reg [9:0] tensorFile_0_rdataVec_MPORT_addr_pipe_0;
  reg [31:0] tensorFile_1 [0:1023]; // @[TensorLoadWideVME.scala 193:16]
  wire [31:0] tensorFile_1_rdataVec_MPORT_1_data; // @[TensorLoadWideVME.scala 193:16]
  wire [9:0] tensorFile_1_rdataVec_MPORT_1_addr; // @[TensorLoadWideVME.scala 193:16]
  wire [31:0] tensorFile_1_MPORT_1_data; // @[TensorLoadWideVME.scala 193:16]
  wire [9:0] tensorFile_1_MPORT_1_addr; // @[TensorLoadWideVME.scala 193:16]
  wire  tensorFile_1_MPORT_1_mask; // @[TensorLoadWideVME.scala 193:16]
  wire  tensorFile_1_MPORT_1_en; // @[TensorLoadWideVME.scala 193:16]
  reg  tensorFile_1_rdataVec_MPORT_1_en_pipe_0;
  reg [9:0] tensorFile_1_rdataVec_MPORT_1_addr_pipe_0;
  reg  state; // @[TensorLoadWideVME.scala 84:22]
  reg [10:0] clInFlight; // @[TensorLoadWideVME.scala 132:23]
  wire  loadDone = clInFlight == 11'h0 & vmeCmd_io_done & state; // @[TensorLoadWideVME.scala 315:53]
  wire  localDone = loadDone & fillPadding_io_done; // @[TensorLoadWideVME.scala 316:25]
  wire  _GEN_0 = localDone ? 1'h0 : state; // @[TensorLoadWideVME.scala 90:25 TensorLoadWideVME.scala 91:11 TensorLoadWideVME.scala 84:22]
  wire  _GEN_1 = io_start | _GEN_0; // @[TensorLoadWideVME.scala 88:18 TensorLoadWideVME.scala 89:11]
  wire  vmeDataFirePipe = io_vme_rd_data_valid & io_vme_rd_data_ready; // @[TensorLoadWideVME.scala 100:42]
  wire  _T = io_vme_rd_cmd_ready & io_vme_rd_cmd_valid; // @[Decoupled.scala 40:37]
  wire  _T_1 = state & _T; // @[TensorLoadWideVME.scala 135:21]
  wire  _T_3 = state & _T & ~vmeDataFirePipe; // @[TensorLoadWideVME.scala 135:45]
  wire [10:0] _GEN_26 = {{2'd0}, vmeCmd_io_readLen}; // @[TensorLoadWideVME.scala 136:30]
  wire [10:0] _clInFlight_T_1 = clInFlight + _GEN_26; // @[TensorLoadWideVME.scala 136:30]
  wire  _T_6 = _T_1 & vmeDataFirePipe; // @[TensorLoadWideVME.scala 137:45]
  wire [10:0] _clInFlight_T_5 = _clInFlight_T_1 - 11'h1; // @[TensorLoadWideVME.scala 138:40]
  wire  _T_10 = state & ~_T & vmeDataFirePipe; // @[TensorLoadWideVME.scala 139:46]
  wire [10:0] _clInFlight_T_7 = clInFlight - 11'h1; // @[TensorLoadWideVME.scala 141:30]
  wire [9:0] zpDestIdx = fillPadding_io_tensorIdx_bits[10:1]; // @[TensorLoadWideVME.scala 172:49]
  wire [1:0] zpDestMask = 2'h1 << fillPadding_io_tensorIdx_bits[0]; // @[OneHot.scala 58:35]
  wire  _wmask_0_T = ~state; // @[TensorLoadWideVME.scala 223:33]
  wire  _wmask_0_T_3 = vmeDataFirePipe & readData_io_destMask_0; // @[TensorLoadWideVME.scala 228:18]
  wire  _wmask_0_T_4 = fillPadding_io_tensorIdx_valid ? zpDestMask[0] : _wmask_0_T_3; // @[TensorLoadWideVME.scala 225:16]
  wire  _wmask_1_T_3 = vmeDataFirePipe & readData_io_destMask_1; // @[TensorLoadWideVME.scala 228:18]
  wire  _wmask_1_T_4 = fillPadding_io_tensorIdx_valid ? zpDestMask[1] : _wmask_1_T_3; // @[TensorLoadWideVME.scala 225:16]
  wire [31:0] _wdata_0_WIRE_2 = readData_io_destData_0;
  wire [31:0] _wdata_0_T_2 = fillPadding_io_tensorIdx_valid ? 32'h0 : _wdata_0_WIRE_2; // @[TensorLoadWideVME.scala 243:12]
  wire [31:0] _wdata_1_WIRE_2 = readData_io_destData_1;
  wire [31:0] _wdata_1_T_2 = fillPadding_io_tensorIdx_valid ? 32'h0 : _wdata_1_WIRE_2; // @[TensorLoadWideVME.scala 243:12]
  wire [10:0] _widx_0_T_1 = fillPadding_io_tensorIdx_valid ? {{1'd0}, zpDestIdx} : readData_io_destIdx_0; // @[TensorLoadWideVME.scala 259:16]
  wire [10:0] widx_0 = _wmask_0_T ? 11'h0 : _widx_0_T_1; // @[TensorLoadWideVME.scala 256:14]
  wire [10:0] _widx_1_T_1 = fillPadding_io_tensorIdx_valid ? {{1'd0}, zpDestIdx} : readData_io_destIdx_1; // @[TensorLoadWideVME.scala 259:16]
  wire [10:0] widx_1 = _wmask_0_T ? 11'h0 : _widx_1_T_1; // @[TensorLoadWideVME.scala 256:14]
  wire [1:0] _rMask_T_1 = 2'h1 << io_tensor_rd_0_idx_bits[0]; // @[OneHot.scala 58:35]
  wire [1:0] rMask = io_tensor_rd_0_idx_valid ? _rMask_T_1 : 2'h0; // @[TensorLoadWideVME.scala 290:10]
  reg [1:0] rdata_r; // @[Reg.scala 15:16]
  wire [31:0] _rdataVec_WIRE_2_0 = tensorFile_0_rdataVec_MPORT_data; // @[TensorLoadWideVME.scala 297:14 TensorLoadWideVME.scala 297:14]
  wire [31:0] _rdata_T_2 = rdata_r[0] ? _rdataVec_WIRE_2_0 : 32'h0; // @[Mux.scala 27:72]
  wire [31:0] _rdataVec_WIRE_5_0 = tensorFile_1_rdataVec_MPORT_1_data; // @[TensorLoadWideVME.scala 297:14 TensorLoadWideVME.scala 297:14]
  wire [31:0] _rdata_T_3 = rdata_r[1] ? _rdataVec_WIRE_5_0 : 32'h0; // @[Mux.scala 27:72]
  reg  rvalid; // @[Reg.scala 27:20]
  GenVMECmdWideTL vmeCmd ( // @[TensorLoadWideVME.scala 105:23]
    .clock(vmeCmd_clock),
    .reset(vmeCmd_reset),
    .io_start(vmeCmd_io_start),
    .io_isBusy(vmeCmd_io_isBusy),
    .io_inst(vmeCmd_io_inst),
    .io_baddr(vmeCmd_io_baddr),
    .io_vmeCmd_ready(vmeCmd_io_vmeCmd_ready),
    .io_vmeCmd_valid(vmeCmd_io_vmeCmd_valid),
    .io_vmeCmd_bits_addr(vmeCmd_io_vmeCmd_bits_addr),
    .io_vmeCmd_bits_len(vmeCmd_io_vmeCmd_bits_len),
    .io_vmeCmd_bits_tag(vmeCmd_io_vmeCmd_bits_tag),
    .io_readLen(vmeCmd_io_readLen),
    .io_done(vmeCmd_io_done)
  );
  ReadVMEDataWide readData ( // @[TensorLoadWideVME.scala 150:24]
    .clock(readData_clock),
    .reset(readData_reset),
    .io_start(readData_io_start),
    .io_vmeData_ready(readData_io_vmeData_ready),
    .io_vmeData_valid(readData_io_vmeData_valid),
    .io_vmeData_bits_data(readData_io_vmeData_bits_data),
    .io_vmeData_bits_tag(readData_io_vmeData_bits_tag),
    .io_vmeData_bits_last(readData_io_vmeData_bits_last),
    .io_destIdx_0(readData_io_destIdx_0),
    .io_destIdx_1(readData_io_destIdx_1),
    .io_destData_0(readData_io_destData_0),
    .io_destData_1(readData_io_destData_1),
    .io_destMask_0(readData_io_destMask_0),
    .io_destMask_1(readData_io_destMask_1)
  );
  ZeroPadding fillPadding ( // @[TensorLoadWideVME.scala 166:27]
    .clock(fillPadding_clock),
    .reset(fillPadding_reset),
    .io_canWriteMem(fillPadding_io_canWriteMem),
    .io_inst(fillPadding_io_inst),
    .io_tensorIdx_valid(fillPadding_io_tensorIdx_valid),
    .io_tensorIdx_bits(fillPadding_io_tensorIdx_bits),
    .io_start(fillPadding_io_start),
    .io_done(fillPadding_io_done)
  );
  assign tensorFile_0_rdataVec_MPORT_addr = tensorFile_0_rdataVec_MPORT_addr_pipe_0;
  assign tensorFile_0_rdataVec_MPORT_data = tensorFile_0[tensorFile_0_rdataVec_MPORT_addr]; // @[TensorLoadWideVME.scala 193:16]
  assign tensorFile_0_MPORT_data = _wmask_0_T ? 32'h0 : _wdata_0_T_2;
  assign tensorFile_0_MPORT_addr = widx_0[9:0];
  assign tensorFile_0_MPORT_mask = 1'h1;
  assign tensorFile_0_MPORT_en = _wmask_0_T ? 1'h0 : _wmask_0_T_4;
  assign tensorFile_1_rdataVec_MPORT_1_addr = tensorFile_1_rdataVec_MPORT_1_addr_pipe_0;
  assign tensorFile_1_rdataVec_MPORT_1_data = tensorFile_1[tensorFile_1_rdataVec_MPORT_1_addr]; // @[TensorLoadWideVME.scala 193:16]
  assign tensorFile_1_MPORT_1_data = _wmask_0_T ? 32'h0 : _wdata_1_T_2;
  assign tensorFile_1_MPORT_1_addr = widx_1[9:0];
  assign tensorFile_1_MPORT_1_mask = 1'h1;
  assign tensorFile_1_MPORT_1_en = _wmask_0_T ? 1'h0 : _wmask_1_T_4;
  assign io_done = loadDone & fillPadding_io_done; // @[TensorLoadWideVME.scala 316:25]
  assign io_vme_rd_cmd_valid = vmeCmd_io_vmeCmd_valid; // @[TensorLoadWideVME.scala 110:20]
  assign io_vme_rd_cmd_bits_addr = vmeCmd_io_vmeCmd_bits_addr; // @[TensorLoadWideVME.scala 110:20]
  assign io_vme_rd_cmd_bits_len = vmeCmd_io_vmeCmd_bits_len; // @[TensorLoadWideVME.scala 110:20]
  assign io_vme_rd_cmd_bits_tag = vmeCmd_io_vmeCmd_bits_tag; // @[TensorLoadWideVME.scala 110:20]
  assign io_vme_rd_data_ready = 1'h1; // @[TensorLoadWideVME.scala 156:24]
  assign io_tensor_rd_0_data_valid = rvalid; // @[TensorLoadWideVME.scala 311:37]
  assign io_tensor_rd_0_data_bits_0_0 = _rdata_T_2 | _rdata_T_3; // @[Mux.scala 27:72]
  assign vmeCmd_clock = clock;
  assign vmeCmd_reset = reset;
  assign vmeCmd_io_start = io_start; // @[TensorLoadWideVME.scala 106:19]
  assign vmeCmd_io_isBusy = state; // @[TensorLoadWideVME.scala 86:22]
  assign vmeCmd_io_inst = io_inst; // @[TensorLoadWideVME.scala 108:18]
  assign vmeCmd_io_baddr = io_baddr; // @[TensorLoadWideVME.scala 109:19]
  assign vmeCmd_io_vmeCmd_ready = io_vme_rd_cmd_ready; // @[TensorLoadWideVME.scala 110:20]
  assign readData_clock = clock;
  assign readData_reset = reset;
  assign readData_io_start = io_start; // @[TensorLoadWideVME.scala 151:21]
  assign readData_io_vmeData_valid = io_vme_rd_data_valid; // @[TensorLoadWideVME.scala 152:29]
  assign readData_io_vmeData_bits_data = io_vme_rd_data_bits_data; // @[TensorLoadWideVME.scala 153:28]
  assign readData_io_vmeData_bits_tag = io_vme_rd_data_bits_tag; // @[TensorLoadWideVME.scala 153:28]
  assign readData_io_vmeData_bits_last = io_vme_rd_data_bits_last; // @[TensorLoadWideVME.scala 153:28]
  assign fillPadding_clock = clock;
  assign fillPadding_reset = reset;
  assign fillPadding_io_canWriteMem = ~vmeDataFirePipe; // @[TensorLoadWideVME.scala 167:33]
  assign fillPadding_io_inst = io_inst; // @[TensorLoadWideVME.scala 168:23]
  assign fillPadding_io_start = io_start; // @[TensorLoadWideVME.scala 169:24]
  always @(posedge clock) begin
    if(tensorFile_0_MPORT_en & tensorFile_0_MPORT_mask) begin
      tensorFile_0[tensorFile_0_MPORT_addr] <= tensorFile_0_MPORT_data; // @[TensorLoadWideVME.scala 193:16]
    end
    tensorFile_0_rdataVec_MPORT_en_pipe_0 <= rMask[0];
    if (rMask[0]) begin
      tensorFile_0_rdataVec_MPORT_addr_pipe_0 <= io_tensor_rd_0_idx_bits[10:1];
    end
    if(tensorFile_1_MPORT_1_en & tensorFile_1_MPORT_1_mask) begin
      tensorFile_1[tensorFile_1_MPORT_1_addr] <= tensorFile_1_MPORT_1_data; // @[TensorLoadWideVME.scala 193:16]
    end
    tensorFile_1_rdataVec_MPORT_1_en_pipe_0 <= rMask[1];
    if (rMask[1]) begin
      tensorFile_1_rdataVec_MPORT_1_addr_pipe_0 <= io_tensor_rd_0_idx_bits[10:1];
    end
    if (reset) begin // @[TensorLoadWideVME.scala 84:22]
      state <= 1'h0; // @[TensorLoadWideVME.scala 84:22]
    end else begin
      state <= _GEN_1;
    end
    if (io_start) begin // @[TensorLoadWideVME.scala 133:18]
      clInFlight <= 11'h0; // @[TensorLoadWideVME.scala 134:16]
    end else if (state & _T & ~vmeDataFirePipe) begin // @[TensorLoadWideVME.scala 135:66]
      clInFlight <= _clInFlight_T_1; // @[TensorLoadWideVME.scala 136:16]
    end else if (_T_1 & vmeDataFirePipe) begin // @[TensorLoadWideVME.scala 137:65]
      clInFlight <= _clInFlight_T_5; // @[TensorLoadWideVME.scala 138:16]
    end else if (state & ~_T & vmeDataFirePipe) begin // @[TensorLoadWideVME.scala 139:66]
      clInFlight <= _clInFlight_T_7; // @[TensorLoadWideVME.scala 141:16]
    end
    if (io_tensor_rd_0_idx_valid) begin // @[TensorLoadWideVME.scala 290:10]
      rdata_r <= _rMask_T_1;
    end else begin
      rdata_r <= 2'h0;
    end
    if (reset) begin // @[Reg.scala 27:20]
      rvalid <= 1'h0; // @[Reg.scala 27:20]
    end else begin
      rvalid <= io_tensor_rd_0_idx_valid;
    end
    `ifndef SYNTHESIS
    `ifdef PRINTF_COND
      if (`PRINTF_COND) begin
    `endif
        if (~io_start & ~_T_3 & ~_T_6 & _T_10 & ~(clInFlight > 11'h0 | reset)) begin
          $fwrite(32'h80000002,"Assertion failed\n    at TensorLoadWideVME.scala:140 assert(clInFlight > 0.U)\n"); // @[TensorLoadWideVME.scala 140:11]
        end
    `ifdef PRINTF_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef STOP_COND
      if (`STOP_COND) begin
    `endif
        if (~io_start & ~_T_3 & ~_T_6 & _T_10 & ~(clInFlight > 11'h0 | reset)) begin
          $fatal; // @[TensorLoadWideVME.scala 140:11]
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
`ifdef RANDOMIZE_MEM_INIT
  _RAND_0 = {1{`RANDOM}};
  for (initvar = 0; initvar < 1024; initvar = initvar+1)
    tensorFile_0[initvar] = _RAND_0[31:0];
  _RAND_3 = {1{`RANDOM}};
  for (initvar = 0; initvar < 1024; initvar = initvar+1)
    tensorFile_1[initvar] = _RAND_3[31:0];
`endif // RANDOMIZE_MEM_INIT
`ifdef RANDOMIZE_REG_INIT
  _RAND_1 = {1{`RANDOM}};
  tensorFile_0_rdataVec_MPORT_en_pipe_0 = _RAND_1[0:0];
  _RAND_2 = {1{`RANDOM}};
  tensorFile_0_rdataVec_MPORT_addr_pipe_0 = _RAND_2[9:0];
  _RAND_4 = {1{`RANDOM}};
  tensorFile_1_rdataVec_MPORT_1_en_pipe_0 = _RAND_4[0:0];
  _RAND_5 = {1{`RANDOM}};
  tensorFile_1_rdataVec_MPORT_1_addr_pipe_0 = _RAND_5[9:0];
  _RAND_6 = {1{`RANDOM}};
  state = _RAND_6[0:0];
  _RAND_7 = {1{`RANDOM}};
  clInFlight = _RAND_7[10:0];
  _RAND_8 = {1{`RANDOM}};
  rdata_r = _RAND_8[1:0];
  _RAND_9 = {1{`RANDOM}};
  rvalid = _RAND_9[0:0];
`endif // RANDOMIZE_REG_INIT
  `endif // RANDOMIZE
end // initial
`ifdef FIRRTL_AFTER_INITIAL
`FIRRTL_AFTER_INITIAL
`endif
`endif // SYNTHESIS
endmodule