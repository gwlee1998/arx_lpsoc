module TensorStoreNarrowVME(
  input          clock,
  input          reset,
  input          io_start,
  output         io_done,
  input  [127:0] io_inst,
  input  [31:0]  io_baddr,
  input          io_vme_wr_cmd_ready,
  output         io_vme_wr_cmd_valid,
  output [31:0]  io_vme_wr_cmd_bits_addr,
  output [7:0]   io_vme_wr_cmd_bits_len,
  input          io_vme_wr_data_ready,
  output         io_vme_wr_data_valid,
  output [63:0]  io_vme_wr_data_bits_data,
  input          io_vme_wr_ack,
  input          io_tensor_wr_0_valid,
  input  [10:0]  io_tensor_wr_0_bits_idx,
  input  [31:0]  io_tensor_wr_0_bits_data_0_0,
  input  [31:0]  io_tensor_wr_0_bits_data_0_1,
  input  [31:0]  io_tensor_wr_0_bits_data_0_2,
  input  [31:0]  io_tensor_wr_0_bits_data_0_3,
  input  [31:0]  io_tensor_wr_0_bits_data_0_4,
  input  [31:0]  io_tensor_wr_0_bits_data_0_5,
  input  [31:0]  io_tensor_wr_0_bits_data_0_6,
  input  [31:0]  io_tensor_wr_0_bits_data_0_7,
  input  [31:0]  io_tensor_wr_0_bits_data_0_8,
  input  [31:0]  io_tensor_wr_0_bits_data_0_9,
  input  [31:0]  io_tensor_wr_0_bits_data_0_10,
  input  [31:0]  io_tensor_wr_0_bits_data_0_11,
  input  [31:0]  io_tensor_wr_0_bits_data_0_12,
  input  [31:0]  io_tensor_wr_0_bits_data_0_13,
  input  [31:0]  io_tensor_wr_0_bits_data_0_14,
  input  [31:0]  io_tensor_wr_0_bits_data_0_15
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
  reg [31:0] _RAND_26;
  reg [31:0] _RAND_27;
  reg [31:0] _RAND_28;
  reg [31:0] _RAND_29;
  reg [31:0] _RAND_30;
  reg [31:0] _RAND_31;
  reg [31:0] _RAND_32;
  reg [31:0] _RAND_33;
  reg [31:0] _RAND_34;
  reg [31:0] _RAND_35;
`endif // RANDOMIZE_REG_INIT
  reg [63:0] tensorFile_0_0 [0:2047]; // @[TensorStoreNarrowVME.scala 167:16]
  wire [63:0] tensorFile_0_0_MPORT_1_data; // @[TensorStoreNarrowVME.scala 167:16]
  wire [10:0] tensorFile_0_0_MPORT_1_addr; // @[TensorStoreNarrowVME.scala 167:16]
  wire [63:0] tensorFile_0_0_MPORT_data; // @[TensorStoreNarrowVME.scala 167:16]
  wire [10:0] tensorFile_0_0_MPORT_addr; // @[TensorStoreNarrowVME.scala 167:16]
  wire  tensorFile_0_0_MPORT_mask; // @[TensorStoreNarrowVME.scala 167:16]
  wire  tensorFile_0_0_MPORT_en; // @[TensorStoreNarrowVME.scala 167:16]
  reg  tensorFile_0_0_MPORT_1_en_pipe_0;
  reg [10:0] tensorFile_0_0_MPORT_1_addr_pipe_0;
  reg [63:0] tensorFile_0_1 [0:2047]; // @[TensorStoreNarrowVME.scala 167:16]
  wire [63:0] tensorFile_0_1_MPORT_1_data; // @[TensorStoreNarrowVME.scala 167:16]
  wire [10:0] tensorFile_0_1_MPORT_1_addr; // @[TensorStoreNarrowVME.scala 167:16]
  wire [63:0] tensorFile_0_1_MPORT_data; // @[TensorStoreNarrowVME.scala 167:16]
  wire [10:0] tensorFile_0_1_MPORT_addr; // @[TensorStoreNarrowVME.scala 167:16]
  wire  tensorFile_0_1_MPORT_mask; // @[TensorStoreNarrowVME.scala 167:16]
  wire  tensorFile_0_1_MPORT_en; // @[TensorStoreNarrowVME.scala 167:16]
  reg  tensorFile_0_1_MPORT_1_en_pipe_0;
  reg [10:0] tensorFile_0_1_MPORT_1_addr_pipe_0;
  reg [63:0] tensorFile_0_2 [0:2047]; // @[TensorStoreNarrowVME.scala 167:16]
  wire [63:0] tensorFile_0_2_MPORT_1_data; // @[TensorStoreNarrowVME.scala 167:16]
  wire [10:0] tensorFile_0_2_MPORT_1_addr; // @[TensorStoreNarrowVME.scala 167:16]
  wire [63:0] tensorFile_0_2_MPORT_data; // @[TensorStoreNarrowVME.scala 167:16]
  wire [10:0] tensorFile_0_2_MPORT_addr; // @[TensorStoreNarrowVME.scala 167:16]
  wire  tensorFile_0_2_MPORT_mask; // @[TensorStoreNarrowVME.scala 167:16]
  wire  tensorFile_0_2_MPORT_en; // @[TensorStoreNarrowVME.scala 167:16]
  reg  tensorFile_0_2_MPORT_1_en_pipe_0;
  reg [10:0] tensorFile_0_2_MPORT_1_addr_pipe_0;
  reg [63:0] tensorFile_0_3 [0:2047]; // @[TensorStoreNarrowVME.scala 167:16]
  wire [63:0] tensorFile_0_3_MPORT_1_data; // @[TensorStoreNarrowVME.scala 167:16]
  wire [10:0] tensorFile_0_3_MPORT_1_addr; // @[TensorStoreNarrowVME.scala 167:16]
  wire [63:0] tensorFile_0_3_MPORT_data; // @[TensorStoreNarrowVME.scala 167:16]
  wire [10:0] tensorFile_0_3_MPORT_addr; // @[TensorStoreNarrowVME.scala 167:16]
  wire  tensorFile_0_3_MPORT_mask; // @[TensorStoreNarrowVME.scala 167:16]
  wire  tensorFile_0_3_MPORT_en; // @[TensorStoreNarrowVME.scala 167:16]
  reg  tensorFile_0_3_MPORT_1_en_pipe_0;
  reg [10:0] tensorFile_0_3_MPORT_1_addr_pipe_0;
  reg [63:0] tensorFile_0_4 [0:2047]; // @[TensorStoreNarrowVME.scala 167:16]
  wire [63:0] tensorFile_0_4_MPORT_1_data; // @[TensorStoreNarrowVME.scala 167:16]
  wire [10:0] tensorFile_0_4_MPORT_1_addr; // @[TensorStoreNarrowVME.scala 167:16]
  wire [63:0] tensorFile_0_4_MPORT_data; // @[TensorStoreNarrowVME.scala 167:16]
  wire [10:0] tensorFile_0_4_MPORT_addr; // @[TensorStoreNarrowVME.scala 167:16]
  wire  tensorFile_0_4_MPORT_mask; // @[TensorStoreNarrowVME.scala 167:16]
  wire  tensorFile_0_4_MPORT_en; // @[TensorStoreNarrowVME.scala 167:16]
  reg  tensorFile_0_4_MPORT_1_en_pipe_0;
  reg [10:0] tensorFile_0_4_MPORT_1_addr_pipe_0;
  reg [63:0] tensorFile_0_5 [0:2047]; // @[TensorStoreNarrowVME.scala 167:16]
  wire [63:0] tensorFile_0_5_MPORT_1_data; // @[TensorStoreNarrowVME.scala 167:16]
  wire [10:0] tensorFile_0_5_MPORT_1_addr; // @[TensorStoreNarrowVME.scala 167:16]
  wire [63:0] tensorFile_0_5_MPORT_data; // @[TensorStoreNarrowVME.scala 167:16]
  wire [10:0] tensorFile_0_5_MPORT_addr; // @[TensorStoreNarrowVME.scala 167:16]
  wire  tensorFile_0_5_MPORT_mask; // @[TensorStoreNarrowVME.scala 167:16]
  wire  tensorFile_0_5_MPORT_en; // @[TensorStoreNarrowVME.scala 167:16]
  reg  tensorFile_0_5_MPORT_1_en_pipe_0;
  reg [10:0] tensorFile_0_5_MPORT_1_addr_pipe_0;
  reg [63:0] tensorFile_0_6 [0:2047]; // @[TensorStoreNarrowVME.scala 167:16]
  wire [63:0] tensorFile_0_6_MPORT_1_data; // @[TensorStoreNarrowVME.scala 167:16]
  wire [10:0] tensorFile_0_6_MPORT_1_addr; // @[TensorStoreNarrowVME.scala 167:16]
  wire [63:0] tensorFile_0_6_MPORT_data; // @[TensorStoreNarrowVME.scala 167:16]
  wire [10:0] tensorFile_0_6_MPORT_addr; // @[TensorStoreNarrowVME.scala 167:16]
  wire  tensorFile_0_6_MPORT_mask; // @[TensorStoreNarrowVME.scala 167:16]
  wire  tensorFile_0_6_MPORT_en; // @[TensorStoreNarrowVME.scala 167:16]
  reg  tensorFile_0_6_MPORT_1_en_pipe_0;
  reg [10:0] tensorFile_0_6_MPORT_1_addr_pipe_0;
  reg [63:0] tensorFile_0_7 [0:2047]; // @[TensorStoreNarrowVME.scala 167:16]
  wire [63:0] tensorFile_0_7_MPORT_1_data; // @[TensorStoreNarrowVME.scala 167:16]
  wire [10:0] tensorFile_0_7_MPORT_1_addr; // @[TensorStoreNarrowVME.scala 167:16]
  wire [63:0] tensorFile_0_7_MPORT_data; // @[TensorStoreNarrowVME.scala 167:16]
  wire [10:0] tensorFile_0_7_MPORT_addr; // @[TensorStoreNarrowVME.scala 167:16]
  wire  tensorFile_0_7_MPORT_mask; // @[TensorStoreNarrowVME.scala 167:16]
  wire  tensorFile_0_7_MPORT_en; // @[TensorStoreNarrowVME.scala 167:16]
  reg  tensorFile_0_7_MPORT_1_en_pipe_0;
  reg [10:0] tensorFile_0_7_MPORT_1_addr_pipe_0;
  wire [15:0] dec_sram_offset = io_inst[25:10]; // @[TensorStoreNarrowVME.scala 60:29]
  wire [31:0] dec_dram_offset = io_inst[57:26]; // @[TensorStoreNarrowVME.scala 60:29]
  wire [15:0] dec_ysize = io_inst[79:64]; // @[TensorStoreNarrowVME.scala 60:29]
  wire [15:0] dec_xsize = io_inst[95:80]; // @[TensorStoreNarrowVME.scala 60:29]
  wire [15:0] dec_xstride = io_inst[111:96]; // @[TensorStoreNarrowVME.scala 60:29]
  reg [31:0] waddr_cur; // @[TensorStoreNarrowVME.scala 61:22]
  reg [31:0] waddr_nxt; // @[TensorStoreNarrowVME.scala 62:22]
  reg [7:0] xcnt; // @[TensorStoreNarrowVME.scala 63:17]
  reg [7:0] xlen; // @[TensorStoreNarrowVME.scala 64:17]
  reg [15:0] xrem; // @[TensorStoreNarrowVME.scala 65:17]
  wire [18:0] xsize = {dec_xsize, 3'h0}; // @[TensorStoreNarrowVME.scala 66:26]
  reg [15:0] ycnt; // @[TensorStoreNarrowVME.scala 69:17]
  reg [7:0] tag; // @[TensorStoreNarrowVME.scala 71:20]
  reg [7:0] set; // @[TensorStoreNarrowVME.scala 72:20]
  reg [31:0] xfer_bytes; // @[TensorStoreNarrowVME.scala 74:23]
  //wire [19:0] xstride_bytes = {dec_xstride, 4'h0}; // @[TensorStoreNarrowVME.scala 75:35]
  wire [19:0] xstride_bytes = {4'h0, dec_xstride}; // @[TensorStoreNarrowVME.scala 75:35] // set stride in bytes by hujang
  wire [37:0] _xfer_init_addr_T = {dec_dram_offset, 6'h0}; // @[TensorStoreNarrowVME.scala 80:66]
  wire [37:0] _xfer_init_addr_T_1 = 38'hffffffff & _xfer_init_addr_T; // @[TensorStoreNarrowVME.scala 80:47]
  wire [37:0] _GEN_116 = {{6'd0}, io_baddr}; // @[TensorStoreNarrowVME.scala 80:33]
  wire [37:0] xfer_init_addr = _GEN_116 | _xfer_init_addr_T_1; // @[TensorStoreNarrowVME.scala 80:33]
  wire [31:0] xfer_split_addr = waddr_cur + xfer_bytes; // @[TensorStoreNarrowVME.scala 81:35]
  wire [31:0] _GEN_117 = {{12'd0}, xstride_bytes}; // @[TensorStoreNarrowVME.scala 82:36]
  wire [31:0] xfer_stride_addr = waddr_nxt + _GEN_117; // @[TensorStoreNarrowVME.scala 82:36]
  wire [37:0] _GEN_2 = xfer_init_addr % 38'h800; // @[TensorStoreNarrowVME.scala 84:55]
  wire [11:0] _xfer_init_bytes_T = _GEN_2[11:0]; // @[TensorStoreNarrowVME.scala 84:55]
  wire [11:0] xfer_init_bytes = 12'h800 - _xfer_init_bytes_T; // @[TensorStoreNarrowVME.scala 84:38]
  wire [8:0] xfer_init_pulses = xfer_init_bytes[11:3]; // @[TensorStoreNarrowVME.scala 85:43]
  wire [31:0] _GEN_5 = xfer_split_addr % 32'h800; // @[TensorStoreNarrowVME.scala 86:56]
  wire [11:0] _xfer_split_bytes_T = _GEN_5[11:0]; // @[TensorStoreNarrowVME.scala 86:56]
  wire [11:0] xfer_split_bytes = 12'h800 - _xfer_split_bytes_T; // @[TensorStoreNarrowVME.scala 86:38]
  wire [8:0] xfer_split_pulses = xfer_split_bytes[11:3]; // @[TensorStoreNarrowVME.scala 87:44]
  wire [31:0] _GEN_15 = xfer_stride_addr % 32'h800; // @[TensorStoreNarrowVME.scala 88:57]
  wire [11:0] _xfer_stride_bytes_T = _GEN_15[11:0]; // @[TensorStoreNarrowVME.scala 88:57]
  wire [11:0] xfer_stride_bytes = 12'h800 - _xfer_stride_bytes_T; // @[TensorStoreNarrowVME.scala 88:38]
  wire [8:0] xfer_stride_pulses = xfer_stride_bytes[11:3]; // @[TensorStoreNarrowVME.scala 89:45]
  reg [2:0] state; // @[TensorStoreNarrowVME.scala 92:22]
  wire  _T = 3'h0 == state; // @[Conditional.scala 37:30]
  wire [18:0] _GEN_118 = {{10'd0}, xfer_init_pulses}; // @[TensorStoreNarrowVME.scala 100:21]
  wire  _T_1 = xsize < _GEN_118; // @[TensorStoreNarrowVME.scala 100:21]
  wire  _T_5 = ~(xsize > 19'h0 | reset); // @[TensorStoreNarrowVME.scala 101:17]
  wire [18:0] _xlen_T_1 = xsize - 19'h1; // @[TensorStoreNarrowVME.scala 102:25]
  wire [8:0] _xlen_T_3 = xfer_init_pulses - 9'h1; // @[TensorStoreNarrowVME.scala 105:36]
  wire [18:0] _xrem_T_1 = xsize - _GEN_118; // @[TensorStoreNarrowVME.scala 107:25]
  wire [18:0] _GEN_0 = xsize < _GEN_118 ? _xlen_T_1 : {{10'd0}, _xlen_T_3}; // @[TensorStoreNarrowVME.scala 100:41 TensorStoreNarrowVME.scala 102:16 TensorStoreNarrowVME.scala 105:16]
  wire [18:0] _GEN_1 = xsize < _GEN_118 ? 19'h0 : _xrem_T_1; // @[TensorStoreNarrowVME.scala 100:41 TensorStoreNarrowVME.scala 103:16 TensorStoreNarrowVME.scala 107:16]
  wire [18:0] _GEN_3 = io_start ? _GEN_0 : {{11'd0}, xlen}; // @[TensorStoreNarrowVME.scala 98:25 TensorStoreNarrowVME.scala 64:17]
  wire [18:0] _GEN_4 = io_start ? _GEN_1 : {{3'd0}, xrem}; // @[TensorStoreNarrowVME.scala 98:25 TensorStoreNarrowVME.scala 65:17]
  wire  _T_10 = 3'h1 == state; // @[Conditional.scala 37:30]
  wire  _T_11 = 3'h2 == state; // @[Conditional.scala 37:30]
  wire  _T_13 = tag == 8'h7; // @[TensorStoreNarrowVME.scala 120:24]
  wire [2:0] _GEN_6 = tag == 8'h7 ? 3'h3 : state; // @[TensorStoreNarrowVME.scala 120:49 TensorStoreNarrowVME.scala 121:17 TensorStoreNarrowVME.scala 92:22]
  wire [2:0] _GEN_7 = xcnt == xlen ? 3'h4 : _GEN_6; // @[TensorStoreNarrowVME.scala 118:29 TensorStoreNarrowVME.scala 119:17]
  wire [2:0] _GEN_8 = io_vme_wr_data_ready ? _GEN_7 : state; // @[TensorStoreNarrowVME.scala 117:34 TensorStoreNarrowVME.scala 92:22]
  wire  _T_14 = 3'h3 == state; // @[Conditional.scala 37:30]
  wire  _T_15 = 3'h4 == state; // @[Conditional.scala 37:30]
  wire  _T_16 = xrem == 16'h0; // @[TensorStoreNarrowVME.scala 130:19]
  wire [15:0] _T_18 = dec_ysize - 16'h1; // @[TensorStoreNarrowVME.scala 131:31]
  wire  _T_19 = ycnt == _T_18; // @[TensorStoreNarrowVME.scala 131:21]
  wire [18:0] _GEN_121 = {{10'd0}, xfer_stride_pulses}; // @[TensorStoreNarrowVME.scala 136:24]
  wire  _T_20 = xsize < _GEN_121; // @[TensorStoreNarrowVME.scala 136:24]
  wire [8:0] _xlen_T_7 = xfer_stride_pulses - 9'h1; // @[TensorStoreNarrowVME.scala 141:42]
  wire [18:0] _xrem_T_3 = xsize - _GEN_121; // @[TensorStoreNarrowVME.scala 143:29]
  wire [18:0] _GEN_9 = xsize < _GEN_121 ? _xlen_T_1 : {{10'd0}, _xlen_T_7}; // @[TensorStoreNarrowVME.scala 136:46 TensorStoreNarrowVME.scala 138:20 TensorStoreNarrowVME.scala 141:20]
  wire [18:0] _GEN_10 = xsize < _GEN_121 ? 19'h0 : _xrem_T_3; // @[TensorStoreNarrowVME.scala 136:46 TensorStoreNarrowVME.scala 139:20 TensorStoreNarrowVME.scala 143:20]
  wire [2:0] _GEN_11 = ycnt == _T_18 ? 3'h0 : 3'h1; // @[TensorStoreNarrowVME.scala 131:38 TensorStoreNarrowVME.scala 132:19 TensorStoreNarrowVME.scala 134:19]
  wire [31:0] _GEN_12 = ycnt == _T_18 ? xfer_bytes : {{20'd0}, xfer_stride_bytes}; // @[TensorStoreNarrowVME.scala 131:38 TensorStoreNarrowVME.scala 74:23 TensorStoreNarrowVME.scala 135:24]
  wire [18:0] _GEN_13 = ycnt == _T_18 ? {{11'd0}, xlen} : _GEN_9; // @[TensorStoreNarrowVME.scala 131:38 TensorStoreNarrowVME.scala 64:17]
  wire [18:0] _GEN_14 = ycnt == _T_18 ? {{3'd0}, xrem} : _GEN_10; // @[TensorStoreNarrowVME.scala 131:38 TensorStoreNarrowVME.scala 65:17]
  wire [15:0] _GEN_124 = {{7'd0}, xfer_split_pulses}; // @[TensorStoreNarrowVME.scala 147:24]
  wire  _T_29 = xrem < _GEN_124; // @[TensorStoreNarrowVME.scala 147:24]
  wire [15:0] _xlen_T_9 = xrem - 16'h1; // @[TensorStoreNarrowVME.scala 151:24]
  wire [8:0] _xlen_T_11 = xfer_split_pulses - 9'h1; // @[TensorStoreNarrowVME.scala 157:37]
  wire [15:0] _xrem_T_5 = xrem - _GEN_124; // @[TensorStoreNarrowVME.scala 159:24]
  wire [15:0] _GEN_17 = xrem < _GEN_124 ? _xlen_T_9 : {{7'd0}, _xlen_T_11}; // @[TensorStoreNarrowVME.scala 147:45 TensorStoreNarrowVME.scala 151:16 TensorStoreNarrowVME.scala 157:16]
  wire [15:0] _GEN_18 = xrem < _GEN_124 ? 16'h0 : _xrem_T_5; // @[TensorStoreNarrowVME.scala 147:45 TensorStoreNarrowVME.scala 152:16 TensorStoreNarrowVME.scala 159:16]
  wire [2:0] _GEN_19 = xrem == 16'h0 ? _GEN_11 : 3'h1; // @[TensorStoreNarrowVME.scala 130:28]
  wire [31:0] _GEN_20 = xrem == 16'h0 ? _GEN_12 : {{20'd0}, xfer_split_bytes}; // @[TensorStoreNarrowVME.scala 130:28]
  wire [18:0] _GEN_21 = xrem == 16'h0 ? _GEN_13 : {{3'd0}, _GEN_17}; // @[TensorStoreNarrowVME.scala 130:28]
  wire [18:0] _GEN_22 = xrem == 16'h0 ? _GEN_14 : {{3'd0}, _GEN_18}; // @[TensorStoreNarrowVME.scala 130:28]
  wire [2:0] _GEN_23 = io_vme_wr_ack ? _GEN_19 : state; // @[TensorStoreNarrowVME.scala 129:27 TensorStoreNarrowVME.scala 92:22]
  wire [31:0] _GEN_24 = io_vme_wr_ack ? _GEN_20 : xfer_bytes; // @[TensorStoreNarrowVME.scala 129:27 TensorStoreNarrowVME.scala 74:23]
  wire [18:0] _GEN_25 = io_vme_wr_ack ? _GEN_21 : {{11'd0}, xlen}; // @[TensorStoreNarrowVME.scala 129:27 TensorStoreNarrowVME.scala 64:17]
  wire [18:0] _GEN_26 = io_vme_wr_ack ? _GEN_22 : {{3'd0}, xrem}; // @[TensorStoreNarrowVME.scala 129:27 TensorStoreNarrowVME.scala 65:17]
  wire [2:0] _GEN_27 = _T_15 ? _GEN_23 : state; // @[Conditional.scala 39:67 TensorStoreNarrowVME.scala 92:22]
  wire [31:0] _GEN_28 = _T_15 ? _GEN_24 : xfer_bytes; // @[Conditional.scala 39:67 TensorStoreNarrowVME.scala 74:23]
  wire [18:0] _GEN_29 = _T_15 ? _GEN_25 : {{11'd0}, xlen}; // @[Conditional.scala 39:67 TensorStoreNarrowVME.scala 64:17]
  wire [18:0] _GEN_30 = _T_15 ? _GEN_26 : {{3'd0}, xrem}; // @[Conditional.scala 39:67 TensorStoreNarrowVME.scala 65:17]
  wire [2:0] _GEN_31 = _T_14 ? 3'h2 : _GEN_27; // @[Conditional.scala 39:67 TensorStoreNarrowVME.scala 126:13]
  wire [18:0] _GEN_33 = _T_14 ? {{11'd0}, xlen} : _GEN_29; // @[Conditional.scala 39:67 TensorStoreNarrowVME.scala 64:17]
  wire [18:0] _GEN_34 = _T_14 ? {{3'd0}, xrem} : _GEN_30; // @[Conditional.scala 39:67 TensorStoreNarrowVME.scala 65:17]
  wire [18:0] _GEN_37 = _T_11 ? {{11'd0}, xlen} : _GEN_33; // @[Conditional.scala 39:67 TensorStoreNarrowVME.scala 64:17]
  wire [18:0] _GEN_38 = _T_11 ? {{3'd0}, xrem} : _GEN_34; // @[Conditional.scala 39:67 TensorStoreNarrowVME.scala 65:17]
  wire [18:0] _GEN_41 = _T_10 ? {{11'd0}, xlen} : _GEN_37; // @[Conditional.scala 39:67 TensorStoreNarrowVME.scala 64:17]
  wire [18:0] _GEN_42 = _T_10 ? {{3'd0}, xrem} : _GEN_38; // @[Conditional.scala 39:67 TensorStoreNarrowVME.scala 65:17]
  wire [18:0] _GEN_45 = _T ? _GEN_3 : _GEN_41; // @[Conditional.scala 40:58]
  wire [18:0] _GEN_46 = _T ? _GEN_4 : _GEN_42; // @[Conditional.scala 40:58]
  wire [255:0] inWrData_lo = {io_tensor_wr_0_bits_data_0_7,io_tensor_wr_0_bits_data_0_6,io_tensor_wr_0_bits_data_0_5,
    io_tensor_wr_0_bits_data_0_4,io_tensor_wr_0_bits_data_0_3,io_tensor_wr_0_bits_data_0_2,io_tensor_wr_0_bits_data_0_1,
    io_tensor_wr_0_bits_data_0_0}; // @[TensorStoreNarrowVME.scala 178:49]
  wire [511:0] _inWrData_T = {io_tensor_wr_0_bits_data_0_15,io_tensor_wr_0_bits_data_0_14,io_tensor_wr_0_bits_data_0_13,
    io_tensor_wr_0_bits_data_0_12,io_tensor_wr_0_bits_data_0_11,io_tensor_wr_0_bits_data_0_10,
    io_tensor_wr_0_bits_data_0_9,io_tensor_wr_0_bits_data_0_8,inWrData_lo}; // @[TensorStoreNarrowVME.scala 178:49]
  wire  _stride_T_1 = state == 3'h4 & io_vme_wr_ack; // @[TensorStoreNarrowVME.scala 186:36]
  wire [7:0] _stride_T_3 = xlen + 8'h1; // @[TensorStoreNarrowVME.scala 188:19]
  wire  _stride_T_4 = xcnt == _stride_T_3; // @[TensorStoreNarrowVME.scala 188:10]
  wire  _stride_T_5 = _stride_T_1 & _stride_T_4; // @[TensorStoreNarrowVME.scala 187:19]
  wire  _stride_T_7 = _stride_T_5 & _T_16; // @[TensorStoreNarrowVME.scala 188:25]
  wire  _stride_T_10 = ycnt != _T_18; // @[TensorStoreNarrowVME.scala 190:10]
  wire  stride = _stride_T_7 & _stride_T_10; // @[TensorStoreNarrowVME.scala 189:18]
  wire  _T_38 = state == 3'h0; // @[TensorStoreNarrowVME.scala 192:14]
  wire [15:0] _ycnt_T_1 = ycnt + 16'h1; // @[TensorStoreNarrowVME.scala 195:18]
  wire  _T_39 = state == 3'h1; // @[TensorStoreNarrowVME.scala 198:42]
  wire  _T_42 = io_vme_wr_data_ready & io_vme_wr_data_valid; // @[Decoupled.scala 40:37]
  wire [7:0] _tag_T_1 = tag + 8'h1; // @[TensorStoreNarrowVME.scala 201:16]
  wire  _T_45 = set == 8'h0; // @[TensorStoreNarrowVME.scala 205:55]
  wire [7:0] _set_T_1 = set + 8'h1; // @[TensorStoreNarrowVME.scala 208:16]
  reg [10:0] raddr_cur; // @[TensorStoreNarrowVME.scala 211:22]
  reg [10:0] raddr_nxt; // @[TensorStoreNarrowVME.scala 212:22]
  wire [10:0] _raddr_cur_T_1 = raddr_cur + 11'h1; // @[TensorStoreNarrowVME.scala 217:28]
  wire [15:0] _GEN_127 = {{5'd0}, raddr_nxt}; // @[TensorStoreNarrowVME.scala 219:28]
  wire [15:0] _raddr_cur_T_3 = _GEN_127 + dec_xsize; // @[TensorStoreNarrowVME.scala 219:28]
  wire [15:0] _GEN_90 = stride ? _raddr_cur_T_3 : {{5'd0}, raddr_cur}; // @[TensorStoreNarrowVME.scala 218:22 TensorStoreNarrowVME.scala 219:15 TensorStoreNarrowVME.scala 211:22]
  wire [15:0] _GEN_91 = stride ? _raddr_cur_T_3 : {{5'd0}, raddr_nxt}; // @[TensorStoreNarrowVME.scala 218:22 TensorStoreNarrowVME.scala 220:15 TensorStoreNarrowVME.scala 212:22]
  wire [15:0] _GEN_92 = _T_42 & _T_45 & _T_13 ? {{5'd0}, _raddr_cur_T_1} : _GEN_90; // @[TensorStoreNarrowVME.scala 216:100 TensorStoreNarrowVME.scala 217:15]
  wire [15:0] _GEN_93 = _T_42 & _T_45 & _T_13 ? {{5'd0}, raddr_nxt} : _GEN_91; // @[TensorStoreNarrowVME.scala 216:100 TensorStoreNarrowVME.scala 212:22]
  wire [15:0] _GEN_94 = _T_38 ? dec_sram_offset : _GEN_92; // @[TensorStoreNarrowVME.scala 213:25 TensorStoreNarrowVME.scala 214:15]
  wire [15:0] _GEN_95 = _T_38 ? dec_sram_offset : _GEN_93; // @[TensorStoreNarrowVME.scala 213:25 TensorStoreNarrowVME.scala 215:15]
  wire  _T_60 = state == 3'h3; // @[TensorStoreNarrowVME.scala 225:65]
  wire [63:0] mdata_0 = 8'h0 == set ? tensorFile_0_0_MPORT_1_data : 64'h0; // @[Mux.scala 80:57]
  wire [63:0] mdata_1 = 8'h0 == set ? tensorFile_0_1_MPORT_1_data : 64'h0; // @[Mux.scala 80:57]
  wire [63:0] mdata_2 = 8'h0 == set ? tensorFile_0_2_MPORT_1_data : 64'h0; // @[Mux.scala 80:57]
  wire [63:0] mdata_3 = 8'h0 == set ? tensorFile_0_3_MPORT_1_data : 64'h0; // @[Mux.scala 80:57]
  wire [63:0] mdata_4 = 8'h0 == set ? tensorFile_0_4_MPORT_1_data : 64'h0; // @[Mux.scala 80:57]
  wire [63:0] mdata_5 = 8'h0 == set ? tensorFile_0_5_MPORT_1_data : 64'h0; // @[Mux.scala 80:57]
  wire [63:0] mdata_6 = 8'h0 == set ? tensorFile_0_6_MPORT_1_data : 64'h0; // @[Mux.scala 80:57]
  wire [63:0] mdata_7 = 8'h0 == set ? tensorFile_0_7_MPORT_1_data : 64'h0; // @[Mux.scala 80:57]
  wire [31:0] _GEN_100 = stride ? xfer_stride_addr : waddr_cur; // @[TensorStoreNarrowVME.scala 235:22 TensorStoreNarrowVME.scala 236:15 TensorStoreNarrowVME.scala 61:22]
  wire [31:0] _GEN_101 = stride ? xfer_stride_addr : waddr_nxt; // @[TensorStoreNarrowVME.scala 235:22 TensorStoreNarrowVME.scala 237:15 TensorStoreNarrowVME.scala 62:22]
  wire [31:0] _GEN_102 = _stride_T_1 & xrem != 16'h0 ? xfer_split_addr : _GEN_100; // @[TensorStoreNarrowVME.scala 233:68 TensorStoreNarrowVME.scala 234:15]
  wire [31:0] _GEN_103 = _stride_T_1 & xrem != 16'h0 ? waddr_nxt : _GEN_101; // @[TensorStoreNarrowVME.scala 233:68 TensorStoreNarrowVME.scala 62:22]
  wire [37:0] _GEN_104 = _T_38 ? xfer_init_addr : {{6'd0}, _GEN_102}; // @[TensorStoreNarrowVME.scala 230:25 TensorStoreNarrowVME.scala 231:15]
  wire [37:0] _GEN_105 = _T_38 ? xfer_init_addr : {{6'd0}, _GEN_103}; // @[TensorStoreNarrowVME.scala 230:25 TensorStoreNarrowVME.scala 232:15]
  wire [63:0] _GEN_107 = 3'h1 == tag[2:0] ? mdata_1 : mdata_0; // @[TensorStoreNarrowVME.scala 246:28 TensorStoreNarrowVME.scala 246:28]
  wire [63:0] _GEN_108 = 3'h2 == tag[2:0] ? mdata_2 : _GEN_107; // @[TensorStoreNarrowVME.scala 246:28 TensorStoreNarrowVME.scala 246:28]
  wire [63:0] _GEN_109 = 3'h3 == tag[2:0] ? mdata_3 : _GEN_108; // @[TensorStoreNarrowVME.scala 246:28 TensorStoreNarrowVME.scala 246:28]
  wire [63:0] _GEN_110 = 3'h4 == tag[2:0] ? mdata_4 : _GEN_109; // @[TensorStoreNarrowVME.scala 246:28 TensorStoreNarrowVME.scala 246:28]
  wire [63:0] _GEN_111 = 3'h5 == tag[2:0] ? mdata_5 : _GEN_110; // @[TensorStoreNarrowVME.scala 246:28 TensorStoreNarrowVME.scala 246:28]
  wire [63:0] _GEN_112 = 3'h6 == tag[2:0] ? mdata_6 : _GEN_111; // @[TensorStoreNarrowVME.scala 246:28 TensorStoreNarrowVME.scala 246:28]
  wire [7:0] _xcnt_T_1 = xcnt + 8'h1; // @[TensorStoreNarrowVME.scala 252:18]
  wire  _GEN_130 = _T & io_start; // @[TensorStoreNarrowVME.scala 101:17]
  wire  _GEN_148 = ~_T & ~_T_10 & ~_T_11 & ~_T_14 & _T_15 & io_vme_wr_ack; // @[TensorStoreNarrowVME.scala 137:21]
  wire  _GEN_151 = ~_T & ~_T_10 & ~_T_11 & ~_T_14 & _T_15 & io_vme_wr_ack & _T_16 & ~_T_19; // @[TensorStoreNarrowVME.scala 137:21]
  wire  _GEN_204 = _GEN_148 & ~_T_16; // @[TensorStoreNarrowVME.scala 150:17]
  assign tensorFile_0_0_MPORT_1_addr = tensorFile_0_0_MPORT_1_addr_pipe_0;
  assign tensorFile_0_0_MPORT_1_data = tensorFile_0_0[tensorFile_0_0_MPORT_1_addr]; // @[TensorStoreNarrowVME.scala 167:16]
  assign tensorFile_0_0_MPORT_data = _inWrData_T[63:0];
  assign tensorFile_0_0_MPORT_addr = io_tensor_wr_0_bits_idx;
  assign tensorFile_0_0_MPORT_mask = 1'h1;
  assign tensorFile_0_0_MPORT_en = io_tensor_wr_0_valid;
  assign tensorFile_0_1_MPORT_1_addr = tensorFile_0_1_MPORT_1_addr_pipe_0;
  assign tensorFile_0_1_MPORT_1_data = tensorFile_0_1[tensorFile_0_1_MPORT_1_addr]; // @[TensorStoreNarrowVME.scala 167:16]
  assign tensorFile_0_1_MPORT_data = _inWrData_T[127:64];
  assign tensorFile_0_1_MPORT_addr = io_tensor_wr_0_bits_idx;
  assign tensorFile_0_1_MPORT_mask = 1'h1;
  assign tensorFile_0_1_MPORT_en = io_tensor_wr_0_valid;
  assign tensorFile_0_2_MPORT_1_addr = tensorFile_0_2_MPORT_1_addr_pipe_0;
  assign tensorFile_0_2_MPORT_1_data = tensorFile_0_2[tensorFile_0_2_MPORT_1_addr]; // @[TensorStoreNarrowVME.scala 167:16]
  assign tensorFile_0_2_MPORT_data = _inWrData_T[191:128];
  assign tensorFile_0_2_MPORT_addr = io_tensor_wr_0_bits_idx;
  assign tensorFile_0_2_MPORT_mask = 1'h1;
  assign tensorFile_0_2_MPORT_en = io_tensor_wr_0_valid;
  assign tensorFile_0_3_MPORT_1_addr = tensorFile_0_3_MPORT_1_addr_pipe_0;
  assign tensorFile_0_3_MPORT_1_data = tensorFile_0_3[tensorFile_0_3_MPORT_1_addr]; // @[TensorStoreNarrowVME.scala 167:16]
  assign tensorFile_0_3_MPORT_data = _inWrData_T[255:192];
  assign tensorFile_0_3_MPORT_addr = io_tensor_wr_0_bits_idx;
  assign tensorFile_0_3_MPORT_mask = 1'h1;
  assign tensorFile_0_3_MPORT_en = io_tensor_wr_0_valid;
  assign tensorFile_0_4_MPORT_1_addr = tensorFile_0_4_MPORT_1_addr_pipe_0;
  assign tensorFile_0_4_MPORT_1_data = tensorFile_0_4[tensorFile_0_4_MPORT_1_addr]; // @[TensorStoreNarrowVME.scala 167:16]
  assign tensorFile_0_4_MPORT_data = _inWrData_T[319:256];
  assign tensorFile_0_4_MPORT_addr = io_tensor_wr_0_bits_idx;
  assign tensorFile_0_4_MPORT_mask = 1'h1;
  assign tensorFile_0_4_MPORT_en = io_tensor_wr_0_valid;
  assign tensorFile_0_5_MPORT_1_addr = tensorFile_0_5_MPORT_1_addr_pipe_0;
  assign tensorFile_0_5_MPORT_1_data = tensorFile_0_5[tensorFile_0_5_MPORT_1_addr]; // @[TensorStoreNarrowVME.scala 167:16]
  assign tensorFile_0_5_MPORT_data = _inWrData_T[383:320];
  assign tensorFile_0_5_MPORT_addr = io_tensor_wr_0_bits_idx;
  assign tensorFile_0_5_MPORT_mask = 1'h1;
  assign tensorFile_0_5_MPORT_en = io_tensor_wr_0_valid;
  assign tensorFile_0_6_MPORT_1_addr = tensorFile_0_6_MPORT_1_addr_pipe_0;
  assign tensorFile_0_6_MPORT_1_data = tensorFile_0_6[tensorFile_0_6_MPORT_1_addr]; // @[TensorStoreNarrowVME.scala 167:16]
  assign tensorFile_0_6_MPORT_data = _inWrData_T[447:384];
  assign tensorFile_0_6_MPORT_addr = io_tensor_wr_0_bits_idx;
  assign tensorFile_0_6_MPORT_mask = 1'h1;
  assign tensorFile_0_6_MPORT_en = io_tensor_wr_0_valid;
  assign tensorFile_0_7_MPORT_1_addr = tensorFile_0_7_MPORT_1_addr_pipe_0;
  assign tensorFile_0_7_MPORT_1_data = tensorFile_0_7[tensorFile_0_7_MPORT_1_addr]; // @[TensorStoreNarrowVME.scala 167:16]
  assign tensorFile_0_7_MPORT_data = _inWrData_T[511:448];
  assign tensorFile_0_7_MPORT_addr = io_tensor_wr_0_bits_idx;
  assign tensorFile_0_7_MPORT_mask = 1'h1;
  assign tensorFile_0_7_MPORT_en = io_tensor_wr_0_valid;
  assign io_done = _stride_T_1 & _T_16 & _T_19; // @[TensorStoreNarrowVME.scala 259:65]
  assign io_vme_wr_cmd_valid = state == 3'h1; // @[TensorStoreNarrowVME.scala 240:32]
  assign io_vme_wr_cmd_bits_addr = waddr_cur; // @[TensorStoreNarrowVME.scala 241:27]
  assign io_vme_wr_cmd_bits_len = xlen; // @[TensorStoreNarrowVME.scala 242:26]
  assign io_vme_wr_data_valid = state == 3'h2; // @[TensorStoreNarrowVME.scala 245:33]
  assign io_vme_wr_data_bits_data = 3'h7 == tag[2:0] ? mdata_7 : _GEN_112; // @[TensorStoreNarrowVME.scala 246:28 TensorStoreNarrowVME.scala 246:28]
  always @(posedge clock) begin
    if(tensorFile_0_0_MPORT_en & tensorFile_0_0_MPORT_mask) begin
      tensorFile_0_0[tensorFile_0_0_MPORT_addr] <= tensorFile_0_0_MPORT_data; // @[TensorStoreNarrowVME.scala 167:16]
    end
    tensorFile_0_0_MPORT_1_en_pipe_0 <= _T_39 | _T_60;
    if (_T_39 | _T_60) begin
      tensorFile_0_0_MPORT_1_addr_pipe_0 <= raddr_cur;
    end
    if(tensorFile_0_1_MPORT_en & tensorFile_0_1_MPORT_mask) begin
      tensorFile_0_1[tensorFile_0_1_MPORT_addr] <= tensorFile_0_1_MPORT_data; // @[TensorStoreNarrowVME.scala 167:16]
    end
    tensorFile_0_1_MPORT_1_en_pipe_0 <= _T_39 | _T_60;
    if (_T_39 | _T_60) begin
      tensorFile_0_1_MPORT_1_addr_pipe_0 <= raddr_cur;
    end
    if(tensorFile_0_2_MPORT_en & tensorFile_0_2_MPORT_mask) begin
      tensorFile_0_2[tensorFile_0_2_MPORT_addr] <= tensorFile_0_2_MPORT_data; // @[TensorStoreNarrowVME.scala 167:16]
    end
    tensorFile_0_2_MPORT_1_en_pipe_0 <= _T_39 | _T_60;
    if (_T_39 | _T_60) begin
      tensorFile_0_2_MPORT_1_addr_pipe_0 <= raddr_cur;
    end
    if(tensorFile_0_3_MPORT_en & tensorFile_0_3_MPORT_mask) begin
      tensorFile_0_3[tensorFile_0_3_MPORT_addr] <= tensorFile_0_3_MPORT_data; // @[TensorStoreNarrowVME.scala 167:16]
    end
    tensorFile_0_3_MPORT_1_en_pipe_0 <= _T_39 | _T_60;
    if (_T_39 | _T_60) begin
      tensorFile_0_3_MPORT_1_addr_pipe_0 <= raddr_cur;
    end
    if(tensorFile_0_4_MPORT_en & tensorFile_0_4_MPORT_mask) begin
      tensorFile_0_4[tensorFile_0_4_MPORT_addr] <= tensorFile_0_4_MPORT_data; // @[TensorStoreNarrowVME.scala 167:16]
    end
    tensorFile_0_4_MPORT_1_en_pipe_0 <= _T_39 | _T_60;
    if (_T_39 | _T_60) begin
      tensorFile_0_4_MPORT_1_addr_pipe_0 <= raddr_cur;
    end
    if(tensorFile_0_5_MPORT_en & tensorFile_0_5_MPORT_mask) begin
      tensorFile_0_5[tensorFile_0_5_MPORT_addr] <= tensorFile_0_5_MPORT_data; // @[TensorStoreNarrowVME.scala 167:16]
    end
    tensorFile_0_5_MPORT_1_en_pipe_0 <= _T_39 | _T_60;
    if (_T_39 | _T_60) begin
      tensorFile_0_5_MPORT_1_addr_pipe_0 <= raddr_cur;
    end
    if(tensorFile_0_6_MPORT_en & tensorFile_0_6_MPORT_mask) begin
      tensorFile_0_6[tensorFile_0_6_MPORT_addr] <= tensorFile_0_6_MPORT_data; // @[TensorStoreNarrowVME.scala 167:16]
    end
    tensorFile_0_6_MPORT_1_en_pipe_0 <= _T_39 | _T_60;
    if (_T_39 | _T_60) begin
      tensorFile_0_6_MPORT_1_addr_pipe_0 <= raddr_cur;
    end
    if(tensorFile_0_7_MPORT_en & tensorFile_0_7_MPORT_mask) begin
      tensorFile_0_7[tensorFile_0_7_MPORT_addr] <= tensorFile_0_7_MPORT_data; // @[TensorStoreNarrowVME.scala 167:16]
    end
    tensorFile_0_7_MPORT_1_en_pipe_0 <= _T_39 | _T_60;
    if (_T_39 | _T_60) begin
      tensorFile_0_7_MPORT_1_addr_pipe_0 <= raddr_cur;
    end
    waddr_cur <= _GEN_104[31:0];
    waddr_nxt <= _GEN_105[31:0];
    if (_T_39) begin // @[TensorStoreNarrowVME.scala 249:29]
      xcnt <= 8'h0; // @[TensorStoreNarrowVME.scala 250:10]
    end else if (_T_42) begin // @[TensorStoreNarrowVME.scala 251:37]
      xcnt <= _xcnt_T_1; // @[TensorStoreNarrowVME.scala 252:10]
    end
    xlen <= _GEN_45[7:0];
    xrem <= _GEN_46[15:0];
    if (state == 3'h0) begin // @[TensorStoreNarrowVME.scala 192:25]
      ycnt <= 16'h0; // @[TensorStoreNarrowVME.scala 193:10]
    end else if (stride) begin // @[TensorStoreNarrowVME.scala 194:22]
      ycnt <= _ycnt_T_1; // @[TensorStoreNarrowVME.scala 195:10]
    end
    if (reset) begin // @[TensorStoreNarrowVME.scala 71:20]
      tag <= 8'h0; // @[TensorStoreNarrowVME.scala 71:20]
    end else if (io_vme_wr_data_ready) begin // @[TensorStoreNarrowVME.scala 198:30]
      if (state == 3'h1 | _T_13) begin // @[TensorStoreNarrowVME.scala 198:88]
        tag <= 8'h0; // @[TensorStoreNarrowVME.scala 199:9]
      end else if (_T_42) begin // @[TensorStoreNarrowVME.scala 200:37]
        tag <= _tag_T_1; // @[TensorStoreNarrowVME.scala 201:9]
      end
    end
    if (reset) begin // @[TensorStoreNarrowVME.scala 72:20]
      set <= 8'h0; // @[TensorStoreNarrowVME.scala 72:20]
    end else if (io_vme_wr_data_ready) begin // @[TensorStoreNarrowVME.scala 204:30]
      if (_T_39 | state != 3'h3 & set == 8'h0 & _T_13) begin // @[TensorStoreNarrowVME.scala 205:113]
        set <= 8'h0; // @[TensorStoreNarrowVME.scala 206:9]
      end else if (_T_42 & _T_13) begin // @[TensorStoreNarrowVME.scala 207:68]
        set <= _set_T_1; // @[TensorStoreNarrowVME.scala 208:9]
      end
    end
    if (_T) begin // @[Conditional.scala 40:58]
      xfer_bytes <= {{20'd0}, xfer_init_bytes}; // @[TensorStoreNarrowVME.scala 97:18]
    end else if (!(_T_10)) begin // @[Conditional.scala 39:67]
      if (!(_T_11)) begin // @[Conditional.scala 39:67]
        if (!(_T_14)) begin // @[Conditional.scala 39:67]
          xfer_bytes <= _GEN_28;
        end
      end
    end
    if (reset) begin // @[TensorStoreNarrowVME.scala 92:22]
      state <= 3'h0; // @[TensorStoreNarrowVME.scala 92:22]
    end else if (_T) begin // @[Conditional.scala 40:58]
      if (io_start) begin // @[TensorStoreNarrowVME.scala 98:25]
        state <= 3'h1; // @[TensorStoreNarrowVME.scala 99:15]
      end
    end else if (_T_10) begin // @[Conditional.scala 39:67]
      if (io_vme_wr_cmd_ready) begin // @[TensorStoreNarrowVME.scala 112:33]
        state <= 3'h2; // @[TensorStoreNarrowVME.scala 113:15]
      end
    end else if (_T_11) begin // @[Conditional.scala 39:67]
      state <= _GEN_8;
    end else begin
      state <= _GEN_31;
    end
    raddr_cur <= _GEN_94[10:0];
    raddr_nxt <= _GEN_95[10:0];
    `ifndef SYNTHESIS
    `ifdef PRINTF_COND
      if (`PRINTF_COND) begin
    `endif
        if (_T & io_start & _T_1 & ~(xsize > 19'h0 | reset)) begin
          $fwrite(32'h80000002,"Assertion failed\n    at TensorStoreNarrowVME.scala:101 assert(xsize > 0.U)\n"); // @[TensorStoreNarrowVME.scala 101:17]
        end
    `ifdef PRINTF_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef STOP_COND
      if (`STOP_COND) begin
    `endif
        if (_T & io_start & _T_1 & ~(xsize > 19'h0 | reset)) begin
          $fatal; // @[TensorStoreNarrowVME.scala 101:17]
        end
    `ifdef STOP_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef PRINTF_COND
      if (`PRINTF_COND) begin
    `endif
        if (_GEN_130 & ~_T_1 & ~(xsize >= _GEN_118 | reset)) begin
          $fwrite(32'h80000002,
            "Assertion failed\n    at TensorStoreNarrowVME.scala:106 assert(xsize >= xfer_init_pulses)\n"); // @[TensorStoreNarrowVME.scala 106:17]
        end
    `ifdef PRINTF_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef STOP_COND
      if (`STOP_COND) begin
    `endif
        if (_GEN_130 & ~_T_1 & ~(xsize >= _GEN_118 | reset)) begin
          $fatal; // @[TensorStoreNarrowVME.scala 106:17]
        end
    `ifdef STOP_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef PRINTF_COND
      if (`PRINTF_COND) begin
    `endif
        if (~_T & ~_T_10 & ~_T_11 & ~_T_14 & _T_15 & io_vme_wr_ack & _T_16 & ~_T_19 & _T_20 & _T_5) begin
          $fwrite(32'h80000002,"Assertion failed\n    at TensorStoreNarrowVME.scala:137 assert(xsize > 0.U)\n"); // @[TensorStoreNarrowVME.scala 137:21]
        end
    `ifdef PRINTF_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef STOP_COND
      if (`STOP_COND) begin
    `endif
        if (~_T & ~_T_10 & ~_T_11 & ~_T_14 & _T_15 & io_vme_wr_ack & _T_16 & ~_T_19 & _T_20 & _T_5) begin
          $fatal; // @[TensorStoreNarrowVME.scala 137:21]
        end
    `ifdef STOP_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef PRINTF_COND
      if (`PRINTF_COND) begin
    `endif
        if (_GEN_151 & ~_T_20 & ~(xsize >= _GEN_121 | reset)) begin
          $fwrite(32'h80000002,
            "Assertion failed\n    at TensorStoreNarrowVME.scala:142 assert(xsize >= xfer_stride_pulses)\n"); // @[TensorStoreNarrowVME.scala 142:21]
        end
    `ifdef PRINTF_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef STOP_COND
      if (`STOP_COND) begin
    `endif
        if (_GEN_151 & ~_T_20 & ~(xsize >= _GEN_121 | reset)) begin
          $fatal; // @[TensorStoreNarrowVME.scala 142:21]
        end
    `ifdef STOP_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef PRINTF_COND
      if (`PRINTF_COND) begin
    `endif
        if (_GEN_148 & ~_T_16 & _T_29 & ~(xrem > 16'h0 | reset)) begin
          $fwrite(32'h80000002,"Assertion failed\n    at TensorStoreNarrowVME.scala:150 assert(xrem > 0.U)\n"); // @[TensorStoreNarrowVME.scala 150:17]
        end
    `ifdef PRINTF_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef STOP_COND
      if (`STOP_COND) begin
    `endif
        if (_GEN_148 & ~_T_16 & _T_29 & ~(xrem > 16'h0 | reset)) begin
          $fatal; // @[TensorStoreNarrowVME.scala 150:17]
        end
    `ifdef STOP_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef PRINTF_COND
      if (`PRINTF_COND) begin
    `endif
        if (_GEN_204 & ~_T_29 & ~(xrem >= _GEN_124 | reset)) begin
          $fwrite(32'h80000002,
            "Assertion failed\n    at TensorStoreNarrowVME.scala:158 assert(xrem >= xfer_split_pulses)\n"); // @[TensorStoreNarrowVME.scala 158:17]
        end
    `ifdef PRINTF_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef STOP_COND
      if (`STOP_COND) begin
    `endif
        if (_GEN_204 & ~_T_29 & ~(xrem >= _GEN_124 | reset)) begin
          $fatal; // @[TensorStoreNarrowVME.scala 158:17]
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
    tensorFile_0_0[initvar] = _RAND_0[63:0];
  _RAND_3 = {2{`RANDOM}};
  for (initvar = 0; initvar < 2048; initvar = initvar+1)
    tensorFile_0_1[initvar] = _RAND_3[63:0];
  _RAND_6 = {2{`RANDOM}};
  for (initvar = 0; initvar < 2048; initvar = initvar+1)
    tensorFile_0_2[initvar] = _RAND_6[63:0];
  _RAND_9 = {2{`RANDOM}};
  for (initvar = 0; initvar < 2048; initvar = initvar+1)
    tensorFile_0_3[initvar] = _RAND_9[63:0];
  _RAND_12 = {2{`RANDOM}};
  for (initvar = 0; initvar < 2048; initvar = initvar+1)
    tensorFile_0_4[initvar] = _RAND_12[63:0];
  _RAND_15 = {2{`RANDOM}};
  for (initvar = 0; initvar < 2048; initvar = initvar+1)
    tensorFile_0_5[initvar] = _RAND_15[63:0];
  _RAND_18 = {2{`RANDOM}};
  for (initvar = 0; initvar < 2048; initvar = initvar+1)
    tensorFile_0_6[initvar] = _RAND_18[63:0];
  _RAND_21 = {2{`RANDOM}};
  for (initvar = 0; initvar < 2048; initvar = initvar+1)
    tensorFile_0_7[initvar] = _RAND_21[63:0];
`endif // RANDOMIZE_MEM_INIT
`ifdef RANDOMIZE_REG_INIT
  _RAND_1 = {1{`RANDOM}};
  tensorFile_0_0_MPORT_1_en_pipe_0 = _RAND_1[0:0];
  _RAND_2 = {1{`RANDOM}};
  tensorFile_0_0_MPORT_1_addr_pipe_0 = _RAND_2[10:0];
  _RAND_4 = {1{`RANDOM}};
  tensorFile_0_1_MPORT_1_en_pipe_0 = _RAND_4[0:0];
  _RAND_5 = {1{`RANDOM}};
  tensorFile_0_1_MPORT_1_addr_pipe_0 = _RAND_5[10:0];
  _RAND_7 = {1{`RANDOM}};
  tensorFile_0_2_MPORT_1_en_pipe_0 = _RAND_7[0:0];
  _RAND_8 = {1{`RANDOM}};
  tensorFile_0_2_MPORT_1_addr_pipe_0 = _RAND_8[10:0];
  _RAND_10 = {1{`RANDOM}};
  tensorFile_0_3_MPORT_1_en_pipe_0 = _RAND_10[0:0];
  _RAND_11 = {1{`RANDOM}};
  tensorFile_0_3_MPORT_1_addr_pipe_0 = _RAND_11[10:0];
  _RAND_13 = {1{`RANDOM}};
  tensorFile_0_4_MPORT_1_en_pipe_0 = _RAND_13[0:0];
  _RAND_14 = {1{`RANDOM}};
  tensorFile_0_4_MPORT_1_addr_pipe_0 = _RAND_14[10:0];
  _RAND_16 = {1{`RANDOM}};
  tensorFile_0_5_MPORT_1_en_pipe_0 = _RAND_16[0:0];
  _RAND_17 = {1{`RANDOM}};
  tensorFile_0_5_MPORT_1_addr_pipe_0 = _RAND_17[10:0];
  _RAND_19 = {1{`RANDOM}};
  tensorFile_0_6_MPORT_1_en_pipe_0 = _RAND_19[0:0];
  _RAND_20 = {1{`RANDOM}};
  tensorFile_0_6_MPORT_1_addr_pipe_0 = _RAND_20[10:0];
  _RAND_22 = {1{`RANDOM}};
  tensorFile_0_7_MPORT_1_en_pipe_0 = _RAND_22[0:0];
  _RAND_23 = {1{`RANDOM}};
  tensorFile_0_7_MPORT_1_addr_pipe_0 = _RAND_23[10:0];
  _RAND_24 = {1{`RANDOM}};
  waddr_cur = _RAND_24[31:0];
  _RAND_25 = {1{`RANDOM}};
  waddr_nxt = _RAND_25[31:0];
  _RAND_26 = {1{`RANDOM}};
  xcnt = _RAND_26[7:0];
  _RAND_27 = {1{`RANDOM}};
  xlen = _RAND_27[7:0];
  _RAND_28 = {1{`RANDOM}};
  xrem = _RAND_28[15:0];
  _RAND_29 = {1{`RANDOM}};
  ycnt = _RAND_29[15:0];
  _RAND_30 = {1{`RANDOM}};
  tag = _RAND_30[7:0];
  _RAND_31 = {1{`RANDOM}};
  set = _RAND_31[7:0];
  _RAND_32 = {1{`RANDOM}};
  xfer_bytes = _RAND_32[31:0];
  _RAND_33 = {1{`RANDOM}};
  state = _RAND_33[2:0];
  _RAND_34 = {1{`RANDOM}};
  raddr_cur = _RAND_34[10:0];
  _RAND_35 = {1{`RANDOM}};
  raddr_nxt = _RAND_35[10:0];
`endif // RANDOMIZE_REG_INIT
  `endif // RANDOMIZE
end // initial
`ifdef FIRRTL_AFTER_INITIAL
`FIRRTL_AFTER_INITIAL
`endif
`endif // SYNTHESIS
endmodule

