module TensorLoadNarrowVME(
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
  input          io_tensor_rd_0_idx_valid,
  input  [10:0]  io_tensor_rd_0_idx_bits,
  output         io_tensor_rd_0_data_valid,
  output [31:0]  io_tensor_rd_0_data_bits_0_0,
  output [31:0]  io_tensor_rd_0_data_bits_0_1,
  output [31:0]  io_tensor_rd_0_data_bits_0_2,
  output [31:0]  io_tensor_rd_0_data_bits_0_3,
  output [31:0]  io_tensor_rd_0_data_bits_0_4,
  output [31:0]  io_tensor_rd_0_data_bits_0_5,
  output [31:0]  io_tensor_rd_0_data_bits_0_6,
  output [31:0]  io_tensor_rd_0_data_bits_0_7,
  output [31:0]  io_tensor_rd_0_data_bits_0_8,
  output [31:0]  io_tensor_rd_0_data_bits_0_9,
  output [31:0]  io_tensor_rd_0_data_bits_0_10,
  output [31:0]  io_tensor_rd_0_data_bits_0_11,
  output [31:0]  io_tensor_rd_0_data_bits_0_12,
  output [31:0]  io_tensor_rd_0_data_bits_0_13,
  output [31:0]  io_tensor_rd_0_data_bits_0_14,
  output [31:0]  io_tensor_rd_0_data_bits_0_15
);
`ifdef RANDOMIZE_MEM_INIT
  reg [63:0] _RAND_0;
  reg [63:0] _RAND_3;
  reg [63:0] _RAND_6;
  reg [63:0] _RAND_9;
  reg [63:0] _RAND_12;
  reg [63:0] _RAND_15;
  reg [63:0] _RAND_18;
  reg [63:0] _RAND_21;
`endif // RANDOMIZE_MEM_INIT
`ifdef RANDOMIZE_REG_INIT
  reg [31:0] _RAND_1;
  reg [31:0] _RAND_2;
  reg [31:0] _RAND_4;
  reg [31:0] _RAND_5;
  reg [31:0] _RAND_7;
  reg [31:0] _RAND_8;
  reg [31:0] _RAND_10;
  reg [31:0] _RAND_11;
  reg [31:0] _RAND_13;
  reg [31:0] _RAND_14;
  reg [31:0] _RAND_16;
  reg [31:0] _RAND_17;
  reg [31:0] _RAND_19;
  reg [31:0] _RAND_20;
  reg [31:0] _RAND_22;
  reg [31:0] _RAND_23;
  reg [31:0] _RAND_24;
  reg [31:0] _RAND_25;
  reg [63:0] _RAND_26;
  reg [31:0] _RAND_27;
  reg [31:0] _RAND_28;
  reg [31:0] _RAND_29;
  reg [127:0] _RAND_30;
  reg [31:0] _RAND_31;
  reg [31:0] _RAND_32;
`endif // RANDOMIZE_REG_INIT
  wire  vmeCmd_clock; // @[TensorLoadNarrowVME.scala 75:23]
  wire  vmeCmd_reset; // @[TensorLoadNarrowVME.scala 75:23]
  wire  vmeCmd_io_start; // @[TensorLoadNarrowVME.scala 75:23]
  wire  vmeCmd_io_isBusy; // @[TensorLoadNarrowVME.scala 75:23]
  wire [127:0] vmeCmd_io_inst; // @[TensorLoadNarrowVME.scala 75:23]
  wire [31:0] vmeCmd_io_baddr; // @[TensorLoadNarrowVME.scala 75:23]
  wire  vmeCmd_io_vmeCmd_ready; // @[TensorLoadNarrowVME.scala 75:23]
  wire  vmeCmd_io_vmeCmd_valid; // @[TensorLoadNarrowVME.scala 75:23]
  wire [31:0] vmeCmd_io_vmeCmd_bits_addr; // @[TensorLoadNarrowVME.scala 75:23]
  wire [7:0] vmeCmd_io_vmeCmd_bits_len; // @[TensorLoadNarrowVME.scala 75:23]
  wire [20:0] vmeCmd_io_vmeCmd_bits_tag; // @[TensorLoadNarrowVME.scala 75:23]
  wire [8:0] vmeCmd_io_readLen; // @[TensorLoadNarrowVME.scala 75:23]
  wire  vmeCmd_io_done; // @[TensorLoadNarrowVME.scala 75:23]
  wire  readData_clock; // @[TensorLoadNarrowVME.scala 105:24]
  wire  readData_reset; // @[TensorLoadNarrowVME.scala 105:24]
  wire  readData_io_start; // @[TensorLoadNarrowVME.scala 105:24]
  wire  readData_io_vmeData_ready; // @[TensorLoadNarrowVME.scala 105:24]
  wire  readData_io_vmeData_valid; // @[TensorLoadNarrowVME.scala 105:24]
  wire [20:0] readData_io_vmeData_bits_tag; // @[TensorLoadNarrowVME.scala 105:24]
  wire [10:0] readData_io_idx; // @[TensorLoadNarrowVME.scala 105:24]
  wire [2:0] readData_io_col; // @[TensorLoadNarrowVME.scala 105:24]
  wire  fillPadding_clock; // @[TensorLoadNarrowVME.scala 119:27]
  wire  fillPadding_reset; // @[TensorLoadNarrowVME.scala 119:27]
  wire  fillPadding_io_canWriteMem; // @[TensorLoadNarrowVME.scala 119:27]
  wire [127:0] fillPadding_io_inst; // @[TensorLoadNarrowVME.scala 119:27]
  wire  fillPadding_io_tensorIdx_valid; // @[TensorLoadNarrowVME.scala 119:27]
  wire [10:0] fillPadding_io_tensorIdx_bits; // @[TensorLoadNarrowVME.scala 119:27]
  wire  fillPadding_io_start; // @[TensorLoadNarrowVME.scala 119:27]
  wire  fillPadding_io_done; // @[TensorLoadNarrowVME.scala 119:27]
  reg [63:0] tensorFile_0 [0:2047]; // @[TensorLoadNarrowVME.scala 152:16]
  wire [63:0] tensorFile_0_MPORT_8_data; // @[TensorLoadNarrowVME.scala 152:16]
  wire [10:0] tensorFile_0_MPORT_8_addr; // @[TensorLoadNarrowVME.scala 152:16]
  wire [63:0] tensorFile_0_MPORT_data; // @[TensorLoadNarrowVME.scala 152:16]
  wire [10:0] tensorFile_0_MPORT_addr; // @[TensorLoadNarrowVME.scala 152:16]
  wire  tensorFile_0_MPORT_mask; // @[TensorLoadNarrowVME.scala 152:16]
  wire  tensorFile_0_MPORT_en; // @[TensorLoadNarrowVME.scala 152:16]
  reg  tensorFile_0_MPORT_8_en_pipe_0;
  reg [10:0] tensorFile_0_MPORT_8_addr_pipe_0;
  reg [63:0] tensorFile_1 [0:2047]; // @[TensorLoadNarrowVME.scala 152:16]
  wire [63:0] tensorFile_1_MPORT_9_data; // @[TensorLoadNarrowVME.scala 152:16]
  wire [10:0] tensorFile_1_MPORT_9_addr; // @[TensorLoadNarrowVME.scala 152:16]
  wire [63:0] tensorFile_1_MPORT_1_data; // @[TensorLoadNarrowVME.scala 152:16]
  wire [10:0] tensorFile_1_MPORT_1_addr; // @[TensorLoadNarrowVME.scala 152:16]
  wire  tensorFile_1_MPORT_1_mask; // @[TensorLoadNarrowVME.scala 152:16]
  wire  tensorFile_1_MPORT_1_en; // @[TensorLoadNarrowVME.scala 152:16]
  reg  tensorFile_1_MPORT_9_en_pipe_0;
  reg [10:0] tensorFile_1_MPORT_9_addr_pipe_0;
  reg [63:0] tensorFile_2 [0:2047]; // @[TensorLoadNarrowVME.scala 152:16]
  wire [63:0] tensorFile_2_MPORT_10_data; // @[TensorLoadNarrowVME.scala 152:16]
  wire [10:0] tensorFile_2_MPORT_10_addr; // @[TensorLoadNarrowVME.scala 152:16]
  wire [63:0] tensorFile_2_MPORT_2_data; // @[TensorLoadNarrowVME.scala 152:16]
  wire [10:0] tensorFile_2_MPORT_2_addr; // @[TensorLoadNarrowVME.scala 152:16]
  wire  tensorFile_2_MPORT_2_mask; // @[TensorLoadNarrowVME.scala 152:16]
  wire  tensorFile_2_MPORT_2_en; // @[TensorLoadNarrowVME.scala 152:16]
  reg  tensorFile_2_MPORT_10_en_pipe_0;
  reg [10:0] tensorFile_2_MPORT_10_addr_pipe_0;
  reg [63:0] tensorFile_3 [0:2047]; // @[TensorLoadNarrowVME.scala 152:16]
  wire [63:0] tensorFile_3_MPORT_11_data; // @[TensorLoadNarrowVME.scala 152:16]
  wire [10:0] tensorFile_3_MPORT_11_addr; // @[TensorLoadNarrowVME.scala 152:16]
  wire [63:0] tensorFile_3_MPORT_3_data; // @[TensorLoadNarrowVME.scala 152:16]
  wire [10:0] tensorFile_3_MPORT_3_addr; // @[TensorLoadNarrowVME.scala 152:16]
  wire  tensorFile_3_MPORT_3_mask; // @[TensorLoadNarrowVME.scala 152:16]
  wire  tensorFile_3_MPORT_3_en; // @[TensorLoadNarrowVME.scala 152:16]
  reg  tensorFile_3_MPORT_11_en_pipe_0;
  reg [10:0] tensorFile_3_MPORT_11_addr_pipe_0;
  reg [63:0] tensorFile_4 [0:2047]; // @[TensorLoadNarrowVME.scala 152:16]
  wire [63:0] tensorFile_4_MPORT_12_data; // @[TensorLoadNarrowVME.scala 152:16]
  wire [10:0] tensorFile_4_MPORT_12_addr; // @[TensorLoadNarrowVME.scala 152:16]
  wire [63:0] tensorFile_4_MPORT_4_data; // @[TensorLoadNarrowVME.scala 152:16]
  wire [10:0] tensorFile_4_MPORT_4_addr; // @[TensorLoadNarrowVME.scala 152:16]
  wire  tensorFile_4_MPORT_4_mask; // @[TensorLoadNarrowVME.scala 152:16]
  wire  tensorFile_4_MPORT_4_en; // @[TensorLoadNarrowVME.scala 152:16]
  reg  tensorFile_4_MPORT_12_en_pipe_0;
  reg [10:0] tensorFile_4_MPORT_12_addr_pipe_0;
  reg [63:0] tensorFile_5 [0:2047]; // @[TensorLoadNarrowVME.scala 152:16]
  wire [63:0] tensorFile_5_MPORT_13_data; // @[TensorLoadNarrowVME.scala 152:16]
  wire [10:0] tensorFile_5_MPORT_13_addr; // @[TensorLoadNarrowVME.scala 152:16]
  wire [63:0] tensorFile_5_MPORT_5_data; // @[TensorLoadNarrowVME.scala 152:16]
  wire [10:0] tensorFile_5_MPORT_5_addr; // @[TensorLoadNarrowVME.scala 152:16]
  wire  tensorFile_5_MPORT_5_mask; // @[TensorLoadNarrowVME.scala 152:16]
  wire  tensorFile_5_MPORT_5_en; // @[TensorLoadNarrowVME.scala 152:16]
  reg  tensorFile_5_MPORT_13_en_pipe_0;
  reg [10:0] tensorFile_5_MPORT_13_addr_pipe_0;
  reg [63:0] tensorFile_6 [0:2047]; // @[TensorLoadNarrowVME.scala 152:16]
  wire [63:0] tensorFile_6_MPORT_14_data; // @[TensorLoadNarrowVME.scala 152:16]
  wire [10:0] tensorFile_6_MPORT_14_addr; // @[TensorLoadNarrowVME.scala 152:16]
  wire [63:0] tensorFile_6_MPORT_6_data; // @[TensorLoadNarrowVME.scala 152:16]
  wire [10:0] tensorFile_6_MPORT_6_addr; // @[TensorLoadNarrowVME.scala 152:16]
  wire  tensorFile_6_MPORT_6_mask; // @[TensorLoadNarrowVME.scala 152:16]
  wire  tensorFile_6_MPORT_6_en; // @[TensorLoadNarrowVME.scala 152:16]
  reg  tensorFile_6_MPORT_14_en_pipe_0;
  reg [10:0] tensorFile_6_MPORT_14_addr_pipe_0;
  reg [63:0] tensorFile_7 [0:2047]; // @[TensorLoadNarrowVME.scala 152:16]
  wire [63:0] tensorFile_7_MPORT_15_data; // @[TensorLoadNarrowVME.scala 152:16]
  wire [10:0] tensorFile_7_MPORT_15_addr; // @[TensorLoadNarrowVME.scala 152:16]
  wire [63:0] tensorFile_7_MPORT_7_data; // @[TensorLoadNarrowVME.scala 152:16]
  wire [10:0] tensorFile_7_MPORT_7_addr; // @[TensorLoadNarrowVME.scala 152:16]
  wire  tensorFile_7_MPORT_7_mask; // @[TensorLoadNarrowVME.scala 152:16]
  wire  tensorFile_7_MPORT_7_en; // @[TensorLoadNarrowVME.scala 152:16]
  reg  tensorFile_7_MPORT_15_en_pipe_0;
  reg [10:0] tensorFile_7_MPORT_15_addr_pipe_0;
  reg  state; // @[TensorLoadNarrowVME.scala 54:22]
  reg [13:0] blocksInFlight; // @[TensorLoadNarrowVME.scala 87:27]
  wire  loadDone = blocksInFlight == 14'h0 & vmeCmd_io_done & state; // @[TensorLoadNarrowVME.scala 292:57]
  wire  localDone = loadDone & fillPadding_io_done; // @[TensorLoadNarrowVME.scala 293:25]
  wire  _GEN_0 = localDone ? 1'h0 : state; // @[TensorLoadNarrowVME.scala 61:25 TensorLoadNarrowVME.scala 62:11 TensorLoadNarrowVME.scala 54:22]
  wire  _GEN_1 = io_start | _GEN_0; // @[TensorLoadNarrowVME.scala 59:18 TensorLoadNarrowVME.scala 60:11]
  reg [63:0] vmeDataBitsPipe_data; // @[TensorLoadNarrowVME.scala 67:32]
  reg [20:0] vmeDataBitsPipe_tag; // @[TensorLoadNarrowVME.scala 67:32]
  reg  vmeDataValidPipe; // @[TensorLoadNarrowVME.scala 68:33]
  reg  vmeDataReadyPipe; // @[TensorLoadNarrowVME.scala 69:33]
  wire  vmeDataFirePipe = vmeDataValidPipe & vmeDataReadyPipe; // @[TensorLoadNarrowVME.scala 70:42]
  wire  _T = io_vme_rd_cmd_ready & io_vme_rd_cmd_valid; // @[Decoupled.scala 40:37]
  wire  _T_1 = state & _T; // @[TensorLoadNarrowVME.scala 90:21]
  wire  _T_3 = state & _T & ~vmeDataFirePipe; // @[TensorLoadNarrowVME.scala 90:45]
  wire [13:0] _GEN_58 = {{5'd0}, vmeCmd_io_readLen}; // @[TensorLoadNarrowVME.scala 91:38]
  wire [13:0] _blocksInFlight_T_1 = blocksInFlight + _GEN_58; // @[TensorLoadNarrowVME.scala 91:38]
  wire  _T_6 = _T_1 & vmeDataFirePipe; // @[TensorLoadNarrowVME.scala 92:45]
  wire [13:0] _blocksInFlight_T_5 = _blocksInFlight_T_1 - 14'h1; // @[TensorLoadNarrowVME.scala 93:48]
  wire  _T_10 = state & ~_T & vmeDataFirePipe; // @[TensorLoadNarrowVME.scala 94:46]
  wire [13:0] _blocksInFlight_T_7 = blocksInFlight - 14'h1; // @[TensorLoadNarrowVME.scala 96:38]
  reg [127:0] fillPadding_io_inst_REG; // @[TensorLoadNarrowVME.scala 121:33]
  reg  fillPadding_io_start_REG; // @[TensorLoadNarrowVME.scala 122:34]
  wire [10:0] waddrTensInstrTmp = fillPadding_io_tensorIdx_valid ? fillPadding_io_tensorIdx_bits : readData_io_idx; // @[TensorLoadNarrowVME.scala 166:30]
  wire  _waddr_0_T = ~state; // @[TensorLoadNarrowVME.scala 186:27]
  wire  wenTensInstr_0 = fillPadding_io_tensorIdx_valid | readData_io_col == 3'h0 & vmeDataFirePipe; // @[TensorLoadNarrowVME.scala 197:8]
  wire  wenTensInstr_1 = fillPadding_io_tensorIdx_valid | readData_io_col == 3'h1 & vmeDataFirePipe; // @[TensorLoadNarrowVME.scala 197:8]
  wire  wenTensInstr_2 = fillPadding_io_tensorIdx_valid | readData_io_col == 3'h2 & vmeDataFirePipe; // @[TensorLoadNarrowVME.scala 197:8]
  wire  wenTensInstr_3 = fillPadding_io_tensorIdx_valid | readData_io_col == 3'h3 & vmeDataFirePipe; // @[TensorLoadNarrowVME.scala 197:8]
  wire  wenTensInstr_4 = fillPadding_io_tensorIdx_valid | readData_io_col == 3'h4 & vmeDataFirePipe; // @[TensorLoadNarrowVME.scala 197:8]
  wire  wenTensInstr_5 = fillPadding_io_tensorIdx_valid | readData_io_col == 3'h5 & vmeDataFirePipe; // @[TensorLoadNarrowVME.scala 197:8]
  wire  wenTensInstr_6 = fillPadding_io_tensorIdx_valid | readData_io_col == 3'h6 & vmeDataFirePipe; // @[TensorLoadNarrowVME.scala 197:8]
  wire  wenTensInstr_7 = fillPadding_io_tensorIdx_valid | readData_io_col == 3'h7 & vmeDataFirePipe; // @[TensorLoadNarrowVME.scala 197:8]
  wire [63:0] wdataTensInstr_0 = fillPadding_io_tensorIdx_valid ? 64'h0 : vmeDataBitsPipe_data; // @[TensorLoadNarrowVME.scala 234:29]
  reg  rvalid; // @[Reg.scala 27:20]
  wire [63:0] _WIRE_8_1 = tensorFile_1_MPORT_9_data; // @[TensorLoadNarrowVME.scala 284:14 TensorLoadNarrowVME.scala 284:14]
  wire [63:0] _WIRE_8_0 = tensorFile_0_MPORT_8_data; // @[TensorLoadNarrowVME.scala 284:14 TensorLoadNarrowVME.scala 284:14]
  wire [63:0] _WIRE_8_3 = tensorFile_3_MPORT_11_data; // @[TensorLoadNarrowVME.scala 284:14 TensorLoadNarrowVME.scala 284:14]
  wire [63:0] _WIRE_8_2 = tensorFile_2_MPORT_10_data; // @[TensorLoadNarrowVME.scala 284:14 TensorLoadNarrowVME.scala 284:14]
  wire [63:0] _WIRE_8_5 = tensorFile_5_MPORT_13_data; // @[TensorLoadNarrowVME.scala 284:14 TensorLoadNarrowVME.scala 284:14]
  wire [63:0] _WIRE_8_4 = tensorFile_4_MPORT_12_data; // @[TensorLoadNarrowVME.scala 284:14 TensorLoadNarrowVME.scala 284:14]
  wire [63:0] _WIRE_8_7 = tensorFile_7_MPORT_15_data; // @[TensorLoadNarrowVME.scala 284:14 TensorLoadNarrowVME.scala 284:14]
  wire [63:0] _WIRE_8_6 = tensorFile_6_MPORT_14_data; // @[TensorLoadNarrowVME.scala 284:14 TensorLoadNarrowVME.scala 284:14]
  wire [511:0] _T_36 = {_WIRE_8_7,_WIRE_8_6,_WIRE_8_5,_WIRE_8_4,_WIRE_8_3,_WIRE_8_2,_WIRE_8_1,_WIRE_8_0}; // @[TensorLoadNarrowVME.scala 288:18]
  GenVMECmd vmeCmd ( // @[TensorLoadNarrowVME.scala 75:23]
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
  ReadVMEData readData ( // @[TensorLoadNarrowVME.scala 105:24]
    .clock(readData_clock),
    .reset(readData_reset),
    .io_start(readData_io_start),
    .io_vmeData_ready(readData_io_vmeData_ready),
    .io_vmeData_valid(readData_io_vmeData_valid),
    .io_vmeData_bits_tag(readData_io_vmeData_bits_tag),
    .io_idx(readData_io_idx),
    .io_col(readData_io_col)
  );
  ZeroPadding fillPadding ( // @[TensorLoadNarrowVME.scala 119:27]
    .clock(fillPadding_clock),
    .reset(fillPadding_reset),
    .io_canWriteMem(fillPadding_io_canWriteMem),
    .io_inst(fillPadding_io_inst),
    .io_tensorIdx_valid(fillPadding_io_tensorIdx_valid),
    .io_tensorIdx_bits(fillPadding_io_tensorIdx_bits),
    .io_start(fillPadding_io_start),
    .io_done(fillPadding_io_done)
  );
  assign tensorFile_0_MPORT_8_addr = tensorFile_0_MPORT_8_addr_pipe_0;
  assign tensorFile_0_MPORT_8_data = tensorFile_0[tensorFile_0_MPORT_8_addr]; // @[TensorLoadNarrowVME.scala 152:16]
  assign tensorFile_0_MPORT_data = _waddr_0_T ? 64'h0 : wdataTensInstr_0;
  assign tensorFile_0_MPORT_addr = _waddr_0_T ? 11'h0 : waddrTensInstrTmp;
  assign tensorFile_0_MPORT_mask = 1'h1;
  assign tensorFile_0_MPORT_en = _waddr_0_T ? 1'h0 : wenTensInstr_0;
  assign tensorFile_1_MPORT_9_addr = tensorFile_1_MPORT_9_addr_pipe_0;
  assign tensorFile_1_MPORT_9_data = tensorFile_1[tensorFile_1_MPORT_9_addr]; // @[TensorLoadNarrowVME.scala 152:16]
  assign tensorFile_1_MPORT_1_data = _waddr_0_T ? 64'h0 : wdataTensInstr_0;
  assign tensorFile_1_MPORT_1_addr = _waddr_0_T ? 11'h0 : waddrTensInstrTmp;
  assign tensorFile_1_MPORT_1_mask = 1'h1;
  assign tensorFile_1_MPORT_1_en = _waddr_0_T ? 1'h0 : wenTensInstr_1;
  assign tensorFile_2_MPORT_10_addr = tensorFile_2_MPORT_10_addr_pipe_0;
  assign tensorFile_2_MPORT_10_data = tensorFile_2[tensorFile_2_MPORT_10_addr]; // @[TensorLoadNarrowVME.scala 152:16]
  assign tensorFile_2_MPORT_2_data = _waddr_0_T ? 64'h0 : wdataTensInstr_0;
  assign tensorFile_2_MPORT_2_addr = _waddr_0_T ? 11'h0 : waddrTensInstrTmp;
  assign tensorFile_2_MPORT_2_mask = 1'h1;
  assign tensorFile_2_MPORT_2_en = _waddr_0_T ? 1'h0 : wenTensInstr_2;
  assign tensorFile_3_MPORT_11_addr = tensorFile_3_MPORT_11_addr_pipe_0;
  assign tensorFile_3_MPORT_11_data = tensorFile_3[tensorFile_3_MPORT_11_addr]; // @[TensorLoadNarrowVME.scala 152:16]
  assign tensorFile_3_MPORT_3_data = _waddr_0_T ? 64'h0 : wdataTensInstr_0;
  assign tensorFile_3_MPORT_3_addr = _waddr_0_T ? 11'h0 : waddrTensInstrTmp;
  assign tensorFile_3_MPORT_3_mask = 1'h1;
  assign tensorFile_3_MPORT_3_en = _waddr_0_T ? 1'h0 : wenTensInstr_3;
  assign tensorFile_4_MPORT_12_addr = tensorFile_4_MPORT_12_addr_pipe_0;
  assign tensorFile_4_MPORT_12_data = tensorFile_4[tensorFile_4_MPORT_12_addr]; // @[TensorLoadNarrowVME.scala 152:16]
  assign tensorFile_4_MPORT_4_data = _waddr_0_T ? 64'h0 : wdataTensInstr_0;
  assign tensorFile_4_MPORT_4_addr = _waddr_0_T ? 11'h0 : waddrTensInstrTmp;
  assign tensorFile_4_MPORT_4_mask = 1'h1;
  assign tensorFile_4_MPORT_4_en = _waddr_0_T ? 1'h0 : wenTensInstr_4;
  assign tensorFile_5_MPORT_13_addr = tensorFile_5_MPORT_13_addr_pipe_0;
  assign tensorFile_5_MPORT_13_data = tensorFile_5[tensorFile_5_MPORT_13_addr]; // @[TensorLoadNarrowVME.scala 152:16]
  assign tensorFile_5_MPORT_5_data = _waddr_0_T ? 64'h0 : wdataTensInstr_0;
  assign tensorFile_5_MPORT_5_addr = _waddr_0_T ? 11'h0 : waddrTensInstrTmp;
  assign tensorFile_5_MPORT_5_mask = 1'h1;
  assign tensorFile_5_MPORT_5_en = _waddr_0_T ? 1'h0 : wenTensInstr_5;
  assign tensorFile_6_MPORT_14_addr = tensorFile_6_MPORT_14_addr_pipe_0;
  assign tensorFile_6_MPORT_14_data = tensorFile_6[tensorFile_6_MPORT_14_addr]; // @[TensorLoadNarrowVME.scala 152:16]
  assign tensorFile_6_MPORT_6_data = _waddr_0_T ? 64'h0 : wdataTensInstr_0;
  assign tensorFile_6_MPORT_6_addr = _waddr_0_T ? 11'h0 : waddrTensInstrTmp;
  assign tensorFile_6_MPORT_6_mask = 1'h1;
  assign tensorFile_6_MPORT_6_en = _waddr_0_T ? 1'h0 : wenTensInstr_6;
  assign tensorFile_7_MPORT_15_addr = tensorFile_7_MPORT_15_addr_pipe_0;
  assign tensorFile_7_MPORT_15_data = tensorFile_7[tensorFile_7_MPORT_15_addr]; // @[TensorLoadNarrowVME.scala 152:16]
  assign tensorFile_7_MPORT_7_data = _waddr_0_T ? 64'h0 : wdataTensInstr_0;
  assign tensorFile_7_MPORT_7_addr = _waddr_0_T ? 11'h0 : waddrTensInstrTmp;
  assign tensorFile_7_MPORT_7_mask = 1'h1;
  assign tensorFile_7_MPORT_7_en = _waddr_0_T ? 1'h0 : wenTensInstr_7;
  assign io_done = loadDone & fillPadding_io_done; // @[TensorLoadNarrowVME.scala 293:25]
  assign io_vme_rd_cmd_valid = vmeCmd_io_vmeCmd_valid; // @[TensorLoadNarrowVME.scala 80:20]
  assign io_vme_rd_cmd_bits_addr = vmeCmd_io_vmeCmd_bits_addr; // @[TensorLoadNarrowVME.scala 80:20]
  assign io_vme_rd_cmd_bits_len = vmeCmd_io_vmeCmd_bits_len; // @[TensorLoadNarrowVME.scala 80:20]
  assign io_vme_rd_cmd_bits_tag = vmeCmd_io_vmeCmd_bits_tag; // @[TensorLoadNarrowVME.scala 80:20]
  assign io_vme_rd_data_ready = 1'h1; // @[TensorLoadNarrowVME.scala 111:24]
  assign io_tensor_rd_0_data_valid = rvalid; // @[TensorLoadNarrowVME.scala 278:36]
  assign io_tensor_rd_0_data_bits_0_0 = _T_36[31:0]; // @[TensorLoadNarrowVME.scala 288:18]
  assign io_tensor_rd_0_data_bits_0_1 = _T_36[63:32]; // @[TensorLoadNarrowVME.scala 288:18]
  assign io_tensor_rd_0_data_bits_0_2 = _T_36[95:64]; // @[TensorLoadNarrowVME.scala 288:18]
  assign io_tensor_rd_0_data_bits_0_3 = _T_36[127:96]; // @[TensorLoadNarrowVME.scala 288:18]
  assign io_tensor_rd_0_data_bits_0_4 = _T_36[159:128]; // @[TensorLoadNarrowVME.scala 288:18]
  assign io_tensor_rd_0_data_bits_0_5 = _T_36[191:160]; // @[TensorLoadNarrowVME.scala 288:18]
  assign io_tensor_rd_0_data_bits_0_6 = _T_36[223:192]; // @[TensorLoadNarrowVME.scala 288:18]
  assign io_tensor_rd_0_data_bits_0_7 = _T_36[255:224]; // @[TensorLoadNarrowVME.scala 288:18]
  assign io_tensor_rd_0_data_bits_0_8 = _T_36[287:256]; // @[TensorLoadNarrowVME.scala 288:18]
  assign io_tensor_rd_0_data_bits_0_9 = _T_36[319:288]; // @[TensorLoadNarrowVME.scala 288:18]
  assign io_tensor_rd_0_data_bits_0_10 = _T_36[351:320]; // @[TensorLoadNarrowVME.scala 288:18]
  assign io_tensor_rd_0_data_bits_0_11 = _T_36[383:352]; // @[TensorLoadNarrowVME.scala 288:18]
  assign io_tensor_rd_0_data_bits_0_12 = _T_36[415:384]; // @[TensorLoadNarrowVME.scala 288:18]
  assign io_tensor_rd_0_data_bits_0_13 = _T_36[447:416]; // @[TensorLoadNarrowVME.scala 288:18]
  assign io_tensor_rd_0_data_bits_0_14 = _T_36[479:448]; // @[TensorLoadNarrowVME.scala 288:18]
  assign io_tensor_rd_0_data_bits_0_15 = _T_36[511:480]; // @[TensorLoadNarrowVME.scala 288:18]
  assign vmeCmd_clock = clock;
  assign vmeCmd_reset = reset;
  assign vmeCmd_io_start = io_start; // @[TensorLoadNarrowVME.scala 76:19]
  assign vmeCmd_io_isBusy = state; // @[TensorLoadNarrowVME.scala 56:22]
  assign vmeCmd_io_inst = io_inst; // @[TensorLoadNarrowVME.scala 78:18]
  assign vmeCmd_io_baddr = io_baddr; // @[TensorLoadNarrowVME.scala 79:19]
  assign vmeCmd_io_vmeCmd_ready = io_vme_rd_cmd_ready; // @[TensorLoadNarrowVME.scala 80:20]
  assign readData_clock = clock;
  assign readData_reset = reset;
  assign readData_io_start = io_start; // @[TensorLoadNarrowVME.scala 106:21]
  assign readData_io_vmeData_valid = vmeDataValidPipe; // @[TensorLoadNarrowVME.scala 107:29]
  assign readData_io_vmeData_bits_tag = vmeDataBitsPipe_tag; // @[TensorLoadNarrowVME.scala 108:28]
  assign fillPadding_clock = clock;
  assign fillPadding_reset = reset;
  assign fillPadding_io_canWriteMem = ~vmeDataFirePipe; // @[TensorLoadNarrowVME.scala 120:33]
  assign fillPadding_io_inst = fillPadding_io_inst_REG; // @[TensorLoadNarrowVME.scala 121:23]
  assign fillPadding_io_start = fillPadding_io_start_REG; // @[TensorLoadNarrowVME.scala 122:24]
  always @(posedge clock) begin
    if(tensorFile_0_MPORT_en & tensorFile_0_MPORT_mask) begin
      tensorFile_0[tensorFile_0_MPORT_addr] <= tensorFile_0_MPORT_data; // @[TensorLoadNarrowVME.scala 152:16]
    end
    tensorFile_0_MPORT_8_en_pipe_0 <= io_tensor_rd_0_idx_valid;
    if (io_tensor_rd_0_idx_valid) begin
      tensorFile_0_MPORT_8_addr_pipe_0 <= io_tensor_rd_0_idx_bits;
    end
    if(tensorFile_1_MPORT_1_en & tensorFile_1_MPORT_1_mask) begin
      tensorFile_1[tensorFile_1_MPORT_1_addr] <= tensorFile_1_MPORT_1_data; // @[TensorLoadNarrowVME.scala 152:16]
    end
    tensorFile_1_MPORT_9_en_pipe_0 <= io_tensor_rd_0_idx_valid;
    if (io_tensor_rd_0_idx_valid) begin
      tensorFile_1_MPORT_9_addr_pipe_0 <= io_tensor_rd_0_idx_bits;
    end
    if(tensorFile_2_MPORT_2_en & tensorFile_2_MPORT_2_mask) begin
      tensorFile_2[tensorFile_2_MPORT_2_addr] <= tensorFile_2_MPORT_2_data; // @[TensorLoadNarrowVME.scala 152:16]
    end
    tensorFile_2_MPORT_10_en_pipe_0 <= io_tensor_rd_0_idx_valid;
    if (io_tensor_rd_0_idx_valid) begin
      tensorFile_2_MPORT_10_addr_pipe_0 <= io_tensor_rd_0_idx_bits;
    end
    if(tensorFile_3_MPORT_3_en & tensorFile_3_MPORT_3_mask) begin
      tensorFile_3[tensorFile_3_MPORT_3_addr] <= tensorFile_3_MPORT_3_data; // @[TensorLoadNarrowVME.scala 152:16]
    end
    tensorFile_3_MPORT_11_en_pipe_0 <= io_tensor_rd_0_idx_valid;
    if (io_tensor_rd_0_idx_valid) begin
      tensorFile_3_MPORT_11_addr_pipe_0 <= io_tensor_rd_0_idx_bits;
    end
    if(tensorFile_4_MPORT_4_en & tensorFile_4_MPORT_4_mask) begin
      tensorFile_4[tensorFile_4_MPORT_4_addr] <= tensorFile_4_MPORT_4_data; // @[TensorLoadNarrowVME.scala 152:16]
    end
    tensorFile_4_MPORT_12_en_pipe_0 <= io_tensor_rd_0_idx_valid;
    if (io_tensor_rd_0_idx_valid) begin
      tensorFile_4_MPORT_12_addr_pipe_0 <= io_tensor_rd_0_idx_bits;
    end
    if(tensorFile_5_MPORT_5_en & tensorFile_5_MPORT_5_mask) begin
      tensorFile_5[tensorFile_5_MPORT_5_addr] <= tensorFile_5_MPORT_5_data; // @[TensorLoadNarrowVME.scala 152:16]
    end
    tensorFile_5_MPORT_13_en_pipe_0 <= io_tensor_rd_0_idx_valid;
    if (io_tensor_rd_0_idx_valid) begin
      tensorFile_5_MPORT_13_addr_pipe_0 <= io_tensor_rd_0_idx_bits;
    end
    if(tensorFile_6_MPORT_6_en & tensorFile_6_MPORT_6_mask) begin
      tensorFile_6[tensorFile_6_MPORT_6_addr] <= tensorFile_6_MPORT_6_data; // @[TensorLoadNarrowVME.scala 152:16]
    end
    tensorFile_6_MPORT_14_en_pipe_0 <= io_tensor_rd_0_idx_valid;
    if (io_tensor_rd_0_idx_valid) begin
      tensorFile_6_MPORT_14_addr_pipe_0 <= io_tensor_rd_0_idx_bits;
    end
    if(tensorFile_7_MPORT_7_en & tensorFile_7_MPORT_7_mask) begin
      tensorFile_7[tensorFile_7_MPORT_7_addr] <= tensorFile_7_MPORT_7_data; // @[TensorLoadNarrowVME.scala 152:16]
    end
    tensorFile_7_MPORT_15_en_pipe_0 <= io_tensor_rd_0_idx_valid;
    if (io_tensor_rd_0_idx_valid) begin
      tensorFile_7_MPORT_15_addr_pipe_0 <= io_tensor_rd_0_idx_bits;
    end
    if (reset) begin // @[TensorLoadNarrowVME.scala 54:22]
      state <= 1'h0; // @[TensorLoadNarrowVME.scala 54:22]
    end else begin
      state <= _GEN_1;
    end
    if (io_start) begin // @[TensorLoadNarrowVME.scala 88:18]
      blocksInFlight <= 14'h0; // @[TensorLoadNarrowVME.scala 89:20]
    end else if (state & _T & ~vmeDataFirePipe) begin // @[TensorLoadNarrowVME.scala 90:66]
      blocksInFlight <= _blocksInFlight_T_1; // @[TensorLoadNarrowVME.scala 91:20]
    end else if (_T_1 & vmeDataFirePipe) begin // @[TensorLoadNarrowVME.scala 92:65]
      blocksInFlight <= _blocksInFlight_T_5; // @[TensorLoadNarrowVME.scala 93:20]
    end else if (state & ~_T & vmeDataFirePipe) begin // @[TensorLoadNarrowVME.scala 94:66]
      blocksInFlight <= _blocksInFlight_T_7; // @[TensorLoadNarrowVME.scala 96:20]
    end
    vmeDataBitsPipe_data <= io_vme_rd_data_bits_data; // @[TensorLoadNarrowVME.scala 67:32]
    vmeDataBitsPipe_tag <= io_vme_rd_data_bits_tag; // @[TensorLoadNarrowVME.scala 67:32]
    if (reset) begin // @[TensorLoadNarrowVME.scala 68:33]
      vmeDataValidPipe <= 1'h0; // @[TensorLoadNarrowVME.scala 68:33]
    end else begin
      vmeDataValidPipe <= io_vme_rd_data_valid; // @[TensorLoadNarrowVME.scala 68:33]
    end
    if (reset) begin // @[TensorLoadNarrowVME.scala 69:33]
      vmeDataReadyPipe <= 1'h0; // @[TensorLoadNarrowVME.scala 69:33]
    end else begin
      vmeDataReadyPipe <= io_vme_rd_data_ready; // @[TensorLoadNarrowVME.scala 69:33]
    end
    fillPadding_io_inst_REG <= io_inst; // @[TensorLoadNarrowVME.scala 121:33]
    if (reset) begin // @[TensorLoadNarrowVME.scala 122:34]
      fillPadding_io_start_REG <= 1'h0; // @[TensorLoadNarrowVME.scala 122:34]
    end else begin
      fillPadding_io_start_REG <= io_start; // @[TensorLoadNarrowVME.scala 122:34]
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
        if (~io_start & ~_T_3 & ~_T_6 & _T_10 & ~(blocksInFlight > 14'h0 | reset)) begin
          $fwrite(32'h80000002,"Assertion failed\n    at TensorLoadNarrowVME.scala:95 assert(blocksInFlight > 0.U)\n"); // @[TensorLoadNarrowVME.scala 95:11]
        end
    `ifdef PRINTF_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef STOP_COND
      if (`STOP_COND) begin
    `endif
        if (~io_start & ~_T_3 & ~_T_6 & _T_10 & ~(blocksInFlight > 14'h0 | reset)) begin
          $fatal; // @[TensorLoadNarrowVME.scala 95:11]
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
  _RAND_0 = {2{`RANDOM}};
  for (initvar = 0; initvar < 2048; initvar = initvar+1)
    tensorFile_0[initvar] = _RAND_0[63:0];
  _RAND_3 = {2{`RANDOM}};
  for (initvar = 0; initvar < 2048; initvar = initvar+1)
    tensorFile_1[initvar] = _RAND_3[63:0];
  _RAND_6 = {2{`RANDOM}};
  for (initvar = 0; initvar < 2048; initvar = initvar+1)
    tensorFile_2[initvar] = _RAND_6[63:0];
  _RAND_9 = {2{`RANDOM}};
  for (initvar = 0; initvar < 2048; initvar = initvar+1)
    tensorFile_3[initvar] = _RAND_9[63:0];
  _RAND_12 = {2{`RANDOM}};
  for (initvar = 0; initvar < 2048; initvar = initvar+1)
    tensorFile_4[initvar] = _RAND_12[63:0];
  _RAND_15 = {2{`RANDOM}};
  for (initvar = 0; initvar < 2048; initvar = initvar+1)
    tensorFile_5[initvar] = _RAND_15[63:0];
  _RAND_18 = {2{`RANDOM}};
  for (initvar = 0; initvar < 2048; initvar = initvar+1)
    tensorFile_6[initvar] = _RAND_18[63:0];
  _RAND_21 = {2{`RANDOM}};
  for (initvar = 0; initvar < 2048; initvar = initvar+1)
    tensorFile_7[initvar] = _RAND_21[63:0];
`endif // RANDOMIZE_MEM_INIT
`ifdef RANDOMIZE_REG_INIT
  _RAND_1 = {1{`RANDOM}};
  tensorFile_0_MPORT_8_en_pipe_0 = _RAND_1[0:0];
  _RAND_2 = {1{`RANDOM}};
  tensorFile_0_MPORT_8_addr_pipe_0 = _RAND_2[10:0];
  _RAND_4 = {1{`RANDOM}};
  tensorFile_1_MPORT_9_en_pipe_0 = _RAND_4[0:0];
  _RAND_5 = {1{`RANDOM}};
  tensorFile_1_MPORT_9_addr_pipe_0 = _RAND_5[10:0];
  _RAND_7 = {1{`RANDOM}};
  tensorFile_2_MPORT_10_en_pipe_0 = _RAND_7[0:0];
  _RAND_8 = {1{`RANDOM}};
  tensorFile_2_MPORT_10_addr_pipe_0 = _RAND_8[10:0];
  _RAND_10 = {1{`RANDOM}};
  tensorFile_3_MPORT_11_en_pipe_0 = _RAND_10[0:0];
  _RAND_11 = {1{`RANDOM}};
  tensorFile_3_MPORT_11_addr_pipe_0 = _RAND_11[10:0];
  _RAND_13 = {1{`RANDOM}};
  tensorFile_4_MPORT_12_en_pipe_0 = _RAND_13[0:0];
  _RAND_14 = {1{`RANDOM}};
  tensorFile_4_MPORT_12_addr_pipe_0 = _RAND_14[10:0];
  _RAND_16 = {1{`RANDOM}};
  tensorFile_5_MPORT_13_en_pipe_0 = _RAND_16[0:0];
  _RAND_17 = {1{`RANDOM}};
  tensorFile_5_MPORT_13_addr_pipe_0 = _RAND_17[10:0];
  _RAND_19 = {1{`RANDOM}};
  tensorFile_6_MPORT_14_en_pipe_0 = _RAND_19[0:0];
  _RAND_20 = {1{`RANDOM}};
  tensorFile_6_MPORT_14_addr_pipe_0 = _RAND_20[10:0];
  _RAND_22 = {1{`RANDOM}};
  tensorFile_7_MPORT_15_en_pipe_0 = _RAND_22[0:0];
  _RAND_23 = {1{`RANDOM}};
  tensorFile_7_MPORT_15_addr_pipe_0 = _RAND_23[10:0];
  _RAND_24 = {1{`RANDOM}};
  state = _RAND_24[0:0];
  _RAND_25 = {1{`RANDOM}};
  blocksInFlight = _RAND_25[13:0];
  _RAND_26 = {2{`RANDOM}};
  vmeDataBitsPipe_data = _RAND_26[63:0];
  _RAND_27 = {1{`RANDOM}};
  vmeDataBitsPipe_tag = _RAND_27[20:0];
  _RAND_28 = {1{`RANDOM}};
  vmeDataValidPipe = _RAND_28[0:0];
  _RAND_29 = {1{`RANDOM}};
  vmeDataReadyPipe = _RAND_29[0:0];
  _RAND_30 = {4{`RANDOM}};
  fillPadding_io_inst_REG = _RAND_30[127:0];
  _RAND_31 = {1{`RANDOM}};
  fillPadding_io_start_REG = _RAND_31[0:0];
  _RAND_32 = {1{`RANDOM}};
  rvalid = _RAND_32[0:0];
`endif // RANDOMIZE_REG_INIT
  `endif // RANDOMIZE
end // initial
`ifdef FIRRTL_AFTER_INITIAL
`FIRRTL_AFTER_INITIAL
`endif
`endif // SYNTHESIS
endmodule