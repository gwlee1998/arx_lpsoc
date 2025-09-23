module VME(
  input         clock,
  input         reset,
  input         io_mem_aw_ready,
  output        io_mem_aw_valid,
  output [31:0] io_mem_aw_bits_addr,
  output [7:0]  io_mem_aw_bits_len,
  input         io_mem_w_ready,
  output        io_mem_w_valid,
  output [63:0] io_mem_w_bits_data,
  output        io_mem_w_bits_last,
  output        io_mem_b_ready,
  input         io_mem_b_valid,
  input         io_mem_ar_ready,
  output        io_mem_ar_valid,
  output [31:0] io_mem_ar_bits_addr,
  output [7:0]  io_mem_ar_bits_id,
  output [7:0]  io_mem_ar_bits_len,
  input         io_mem_r_valid,
  input  [63:0] io_mem_r_bits_data,
  input         io_mem_r_bits_last,
  input  [7:0]  io_mem_r_bits_id,
  output        io_vme_rd_0_cmd_ready,
  input         io_vme_rd_0_cmd_valid,
  input  [31:0] io_vme_rd_0_cmd_bits_addr,
  input  [7:0]  io_vme_rd_0_cmd_bits_len,
  input         io_vme_rd_0_data_ready,
  output        io_vme_rd_0_data_valid,
  output [63:0] io_vme_rd_0_data_bits_data,
  output        io_vme_rd_1_cmd_ready,
  input         io_vme_rd_1_cmd_valid,
  input  [31:0] io_vme_rd_1_cmd_bits_addr,
  input  [7:0]  io_vme_rd_1_cmd_bits_len,
  input  [20:0] io_vme_rd_1_cmd_bits_tag,
  output        io_vme_rd_1_data_valid,
  output [63:0] io_vme_rd_1_data_bits_data,
  output [20:0] io_vme_rd_1_data_bits_tag,
  output        io_vme_rd_1_data_bits_last,
  output        io_vme_rd_2_cmd_ready,
  input         io_vme_rd_2_cmd_valid,
  input  [31:0] io_vme_rd_2_cmd_bits_addr,
  input  [7:0]  io_vme_rd_2_cmd_bits_len,
  input  [20:0] io_vme_rd_2_cmd_bits_tag,
  output        io_vme_rd_2_data_valid,
  output [63:0] io_vme_rd_2_data_bits_data,
  output [20:0] io_vme_rd_2_data_bits_tag,
  output        io_vme_rd_3_cmd_ready,
  input         io_vme_rd_3_cmd_valid,
  input  [31:0] io_vme_rd_3_cmd_bits_addr,
  input  [7:0]  io_vme_rd_3_cmd_bits_len,
  input  [20:0] io_vme_rd_3_cmd_bits_tag,
  output        io_vme_rd_3_data_valid,
  output [63:0] io_vme_rd_3_data_bits_data,
  output [20:0] io_vme_rd_3_data_bits_tag,
  output        io_vme_rd_4_cmd_ready,
  input         io_vme_rd_4_cmd_valid,
  input  [31:0] io_vme_rd_4_cmd_bits_addr,
  input  [7:0]  io_vme_rd_4_cmd_bits_len,
  input  [20:0] io_vme_rd_4_cmd_bits_tag,
  output        io_vme_rd_4_data_valid,
  output [63:0] io_vme_rd_4_data_bits_data,
  output [20:0] io_vme_rd_4_data_bits_tag,
  output        io_vme_wr_0_cmd_ready,
  input         io_vme_wr_0_cmd_valid,
  input  [31:0] io_vme_wr_0_cmd_bits_addr,
  input  [7:0]  io_vme_wr_0_cmd_bits_len,
  output        io_vme_wr_0_data_ready,
  input         io_vme_wr_0_data_valid,
  input  [63:0] io_vme_wr_0_data_bits_data,
  output        io_vme_wr_0_ack
);
`ifdef RANDOMIZE_MEM_INIT
  reg [31:0] _RAND_0;
  reg [31:0] _RAND_2;
`endif // RANDOMIZE_MEM_INIT
`ifdef RANDOMIZE_REG_INIT
  reg [31:0] _RAND_1;
  reg [31:0] _RAND_3;
  reg [31:0] _RAND_4;
  reg [31:0] _RAND_5;
  reg [63:0] _RAND_6;
  reg [31:0] _RAND_7;
  reg [63:0] _RAND_8;
  reg [31:0] _RAND_9;
  reg [31:0] _RAND_10;
  reg [63:0] _RAND_11;
  reg [31:0] _RAND_12;
  reg [63:0] _RAND_13;
  reg [31:0] _RAND_14;
  reg [63:0] _RAND_15;
  reg [31:0] _RAND_16;
  reg [31:0] _RAND_17;
  reg [31:0] _RAND_18;
  reg [31:0] _RAND_19;
`endif // RANDOMIZE_REG_INIT
  reg [2:0] vmeTag_array_client_id [0:15]; // @[VME.scala 204:33]
  wire [2:0] vmeTag_array_client_id_localTag_out_MPORT_data; // @[VME.scala 204:33]
  wire [3:0] vmeTag_array_client_id_localTag_out_MPORT_addr; // @[VME.scala 204:33]
  wire [2:0] vmeTag_array_client_id_rdwrPort_data; // @[VME.scala 204:33]
  wire [3:0] vmeTag_array_client_id_rdwrPort_addr; // @[VME.scala 204:33]
  wire  vmeTag_array_client_id_rdwrPort_mask; // @[VME.scala 204:33]
  wire  vmeTag_array_client_id_rdwrPort_en; // @[VME.scala 204:33]
  reg [3:0] vmeTag_array_client_id_localTag_out_MPORT_addr_pipe_0;
  reg [20:0] vmeTag_array_client_tag [0:15]; // @[VME.scala 204:33]
  wire [20:0] vmeTag_array_client_tag_localTag_out_MPORT_data; // @[VME.scala 204:33]
  wire [3:0] vmeTag_array_client_tag_localTag_out_MPORT_addr; // @[VME.scala 204:33]
  wire [20:0] vmeTag_array_client_tag_rdwrPort_data; // @[VME.scala 204:33]
  wire [3:0] vmeTag_array_client_tag_rdwrPort_addr; // @[VME.scala 204:33]
  wire  vmeTag_array_client_tag_rdwrPort_mask; // @[VME.scala 204:33]
  wire  vmeTag_array_client_tag_rdwrPort_en; // @[VME.scala 204:33]
  reg [3:0] vmeTag_array_client_tag_localTag_out_MPORT_addr_pipe_0;
  wire  VMEcmd_Qs_0_clock; // @[VME.scala 231:45]
  wire  VMEcmd_Qs_0_reset; // @[VME.scala 231:45]
  wire  VMEcmd_Qs_0_io_enq_ready; // @[VME.scala 231:45]
  wire  VMEcmd_Qs_0_io_enq_valid; // @[VME.scala 231:45]
  wire [31:0] VMEcmd_Qs_0_io_enq_bits_addr; // @[VME.scala 231:45]
  wire [7:0] VMEcmd_Qs_0_io_enq_bits_len; // @[VME.scala 231:45]
  wire [20:0] VMEcmd_Qs_0_io_enq_bits_tag; // @[VME.scala 231:45]
  wire  VMEcmd_Qs_0_io_deq_ready; // @[VME.scala 231:45]
  wire  VMEcmd_Qs_0_io_deq_valid; // @[VME.scala 231:45]
  wire [31:0] VMEcmd_Qs_0_io_deq_bits_addr; // @[VME.scala 231:45]
  wire [7:0] VMEcmd_Qs_0_io_deq_bits_len; // @[VME.scala 231:45]
  wire [20:0] VMEcmd_Qs_0_io_deq_bits_tag; // @[VME.scala 231:45]
  wire  VMEcmd_Qs_1_clock; // @[VME.scala 231:45]
  wire  VMEcmd_Qs_1_reset; // @[VME.scala 231:45]
  wire  VMEcmd_Qs_1_io_enq_ready; // @[VME.scala 231:45]
  wire  VMEcmd_Qs_1_io_enq_valid; // @[VME.scala 231:45]
  wire [31:0] VMEcmd_Qs_1_io_enq_bits_addr; // @[VME.scala 231:45]
  wire [7:0] VMEcmd_Qs_1_io_enq_bits_len; // @[VME.scala 231:45]
  wire [20:0] VMEcmd_Qs_1_io_enq_bits_tag; // @[VME.scala 231:45]
  wire  VMEcmd_Qs_1_io_deq_ready; // @[VME.scala 231:45]
  wire  VMEcmd_Qs_1_io_deq_valid; // @[VME.scala 231:45]
  wire [31:0] VMEcmd_Qs_1_io_deq_bits_addr; // @[VME.scala 231:45]
  wire [7:0] VMEcmd_Qs_1_io_deq_bits_len; // @[VME.scala 231:45]
  wire [20:0] VMEcmd_Qs_1_io_deq_bits_tag; // @[VME.scala 231:45]
  wire  VMEcmd_Qs_2_clock; // @[VME.scala 231:45]
  wire  VMEcmd_Qs_2_reset; // @[VME.scala 231:45]
  wire  VMEcmd_Qs_2_io_enq_ready; // @[VME.scala 231:45]
  wire  VMEcmd_Qs_2_io_enq_valid; // @[VME.scala 231:45]
  wire [31:0] VMEcmd_Qs_2_io_enq_bits_addr; // @[VME.scala 231:45]
  wire [7:0] VMEcmd_Qs_2_io_enq_bits_len; // @[VME.scala 231:45]
  wire [20:0] VMEcmd_Qs_2_io_enq_bits_tag; // @[VME.scala 231:45]
  wire  VMEcmd_Qs_2_io_deq_ready; // @[VME.scala 231:45]
  wire  VMEcmd_Qs_2_io_deq_valid; // @[VME.scala 231:45]
  wire [31:0] VMEcmd_Qs_2_io_deq_bits_addr; // @[VME.scala 231:45]
  wire [7:0] VMEcmd_Qs_2_io_deq_bits_len; // @[VME.scala 231:45]
  wire [20:0] VMEcmd_Qs_2_io_deq_bits_tag; // @[VME.scala 231:45]
  wire  VMEcmd_Qs_3_clock; // @[VME.scala 231:45]
  wire  VMEcmd_Qs_3_reset; // @[VME.scala 231:45]
  wire  VMEcmd_Qs_3_io_enq_ready; // @[VME.scala 231:45]
  wire  VMEcmd_Qs_3_io_enq_valid; // @[VME.scala 231:45]
  wire [31:0] VMEcmd_Qs_3_io_enq_bits_addr; // @[VME.scala 231:45]
  wire [7:0] VMEcmd_Qs_3_io_enq_bits_len; // @[VME.scala 231:45]
  wire [20:0] VMEcmd_Qs_3_io_enq_bits_tag; // @[VME.scala 231:45]
  wire  VMEcmd_Qs_3_io_deq_ready; // @[VME.scala 231:45]
  wire  VMEcmd_Qs_3_io_deq_valid; // @[VME.scala 231:45]
  wire [31:0] VMEcmd_Qs_3_io_deq_bits_addr; // @[VME.scala 231:45]
  wire [7:0] VMEcmd_Qs_3_io_deq_bits_len; // @[VME.scala 231:45]
  wire [20:0] VMEcmd_Qs_3_io_deq_bits_tag; // @[VME.scala 231:45]
  wire  VMEcmd_Qs_4_clock; // @[VME.scala 231:45]
  wire  VMEcmd_Qs_4_reset; // @[VME.scala 231:45]
  wire  VMEcmd_Qs_4_io_enq_ready; // @[VME.scala 231:45]
  wire  VMEcmd_Qs_4_io_enq_valid; // @[VME.scala 231:45]
  wire [31:0] VMEcmd_Qs_4_io_enq_bits_addr; // @[VME.scala 231:45]
  wire [7:0] VMEcmd_Qs_4_io_enq_bits_len; // @[VME.scala 231:45]
  wire [20:0] VMEcmd_Qs_4_io_enq_bits_tag; // @[VME.scala 231:45]
  wire  VMEcmd_Qs_4_io_deq_ready; // @[VME.scala 231:45]
  wire  VMEcmd_Qs_4_io_deq_valid; // @[VME.scala 231:45]
  wire [31:0] VMEcmd_Qs_4_io_deq_bits_addr; // @[VME.scala 231:45]
  wire [7:0] VMEcmd_Qs_4_io_deq_bits_len; // @[VME.scala 231:45]
  wire [20:0] VMEcmd_Qs_4_io_deq_bits_tag; // @[VME.scala 231:45]
  reg [15:0] availableEntries; // @[VME.scala 212:33]
  wire  oneHotIdx_0 = availableEntries[0]; // @[VME.scala 239:11]
  wire  oneHotIdx_1 = availableEntries[1] & ~(|oneHotIdx_0); // @[VME.scala 242:20]
  wire  oneHotIdx_2 = availableEntries[2] & ~(|availableEntries[1:0]); // @[VME.scala 242:20]
  wire  oneHotIdx_3 = availableEntries[3] & ~(|availableEntries[2:0]); // @[VME.scala 242:20]
  wire  oneHotIdx_4 = availableEntries[4] & ~(|availableEntries[3:0]); // @[VME.scala 242:20]
  wire  oneHotIdx_5 = availableEntries[5] & ~(|availableEntries[4:0]); // @[VME.scala 242:20]
  wire  oneHotIdx_6 = availableEntries[6] & ~(|availableEntries[5:0]); // @[VME.scala 242:20]
  wire  oneHotIdx_7 = availableEntries[7] & ~(|availableEntries[6:0]); // @[VME.scala 242:20]
  wire  oneHotIdx_8 = availableEntries[8] & ~(|availableEntries[7:0]); // @[VME.scala 242:20]
  wire  oneHotIdx_9 = availableEntries[9] & ~(|availableEntries[8:0]); // @[VME.scala 242:20]
  wire  oneHotIdx_10 = availableEntries[10] & ~(|availableEntries[9:0]); // @[VME.scala 242:20]
  wire  oneHotIdx_11 = availableEntries[11] & ~(|availableEntries[10:0]); // @[VME.scala 242:20]
  wire  oneHotIdx_12 = availableEntries[12] & ~(|availableEntries[11:0]); // @[VME.scala 242:20]
  wire  oneHotIdx_13 = availableEntries[13] & ~(|availableEntries[12:0]); // @[VME.scala 242:20]
  wire  oneHotIdx_14 = availableEntries[14] & ~(|availableEntries[13:0]); // @[VME.scala 242:20]
  wire  oneHotIdx_15 = availableEntries[15] & ~(|availableEntries[14:0]); // @[VME.scala 242:20]
  wire [7:0] oHot_lo = {oneHotIdx_7,oneHotIdx_6,oneHotIdx_5,oneHotIdx_4,oneHotIdx_3,oneHotIdx_2,oneHotIdx_1,oneHotIdx_0}
    ; // @[VME.scala 245:35]
  wire [15:0] resetEntry = {oneHotIdx_15,oneHotIdx_14,oneHotIdx_13,oneHotIdx_12,oneHotIdx_11,oneHotIdx_10,oneHotIdx_9,
    oneHotIdx_8,oHot_lo}; // @[VME.scala 245:35]
  wire [15:0] _newVec_T = ~resetEntry; // @[VME.scala 246:22]
  wire [15:0] newEntry = availableEntries & _newVec_T; // @[VME.scala 246:20]
  wire [3:0] _bitPostn_T = oneHotIdx_14 ? 4'he : 4'hf; // @[Mux.scala 47:69]
  wire [3:0] _bitPostn_T_1 = oneHotIdx_13 ? 4'hd : _bitPostn_T; // @[Mux.scala 47:69]
  wire [3:0] _bitPostn_T_2 = oneHotIdx_12 ? 4'hc : _bitPostn_T_1; // @[Mux.scala 47:69]
  wire [3:0] _bitPostn_T_3 = oneHotIdx_11 ? 4'hb : _bitPostn_T_2; // @[Mux.scala 47:69]
  wire [3:0] _bitPostn_T_4 = oneHotIdx_10 ? 4'ha : _bitPostn_T_3; // @[Mux.scala 47:69]
  wire [3:0] _bitPostn_T_5 = oneHotIdx_9 ? 4'h9 : _bitPostn_T_4; // @[Mux.scala 47:69]
  wire [3:0] _bitPostn_T_6 = oneHotIdx_8 ? 4'h8 : _bitPostn_T_5; // @[Mux.scala 47:69]
  wire [3:0] _bitPostn_T_7 = oneHotIdx_7 ? 4'h7 : _bitPostn_T_6; // @[Mux.scala 47:69]
  wire [3:0] _bitPostn_T_8 = oneHotIdx_6 ? 4'h6 : _bitPostn_T_7; // @[Mux.scala 47:69]
  wire [3:0] _bitPostn_T_9 = oneHotIdx_5 ? 4'h5 : _bitPostn_T_8; // @[Mux.scala 47:69]
  wire [3:0] _bitPostn_T_10 = oneHotIdx_4 ? 4'h4 : _bitPostn_T_9; // @[Mux.scala 47:69]
  wire [3:0] _bitPostn_T_11 = oneHotIdx_3 ? 4'h3 : _bitPostn_T_10; // @[Mux.scala 47:69]
  wire [3:0] _bitPostn_T_12 = oneHotIdx_2 ? 4'h2 : _bitPostn_T_11; // @[Mux.scala 47:69]
  wire [3:0] _bitPostn_T_13 = oneHotIdx_1 ? 4'h1 : _bitPostn_T_12; // @[Mux.scala 47:69]
  wire [3:0] firstPostn = oneHotIdx_0 ? 4'h0 : _bitPostn_T_13; // @[Mux.scala 47:69]
  wire  _T = io_mem_r_bits_last & io_mem_r_valid; // @[VME.scala 216:27]
  wire [7:0] updateEntry_lo = {8'h7 == io_mem_r_bits_id,8'h6 == io_mem_r_bits_id,8'h5 == io_mem_r_bits_id,8'h4 ==
    io_mem_r_bits_id,8'h3 == io_mem_r_bits_id,8'h2 == io_mem_r_bits_id,8'h1 == io_mem_r_bits_id,8'h0 == io_mem_r_bits_id
    }; // @[VME.scala 228:108]
  wire [15:0] _updateEntry_T_16 = {8'hf == io_mem_r_bits_id,8'he == io_mem_r_bits_id,8'hd == io_mem_r_bits_id,8'hc ==
    io_mem_r_bits_id,8'hb == io_mem_r_bits_id,8'ha == io_mem_r_bits_id,8'h9 == io_mem_r_bits_id,8'h8 == io_mem_r_bits_id
    ,updateEntry_lo}; // @[VME.scala 228:108]
  wire [15:0] updateEntry = reset ? 16'h0 : _updateEntry_T_16; // @[VME.scala 223:21 VME.scala 225:15 VME.scala 228:15]
  wire [15:0] _availableEntriesNext_T = updateEntry | availableEntries; // @[VME.scala 217:39]
  wire  _T_1 = availableEntries != 16'h0; // @[VME.scala 218:53]
  wire  any_cmd_valid = VMEcmd_Qs_0_io_deq_valid | VMEcmd_Qs_1_io_deq_valid | VMEcmd_Qs_2_io_deq_valid |
    VMEcmd_Qs_3_io_deq_valid | VMEcmd_Qs_4_io_deq_valid; // @[VME.scala 258:69]
  wire  availableEntriesEn = io_mem_ar_ready & any_cmd_valid; // @[VME.scala 259:41]
  wire  _T_4 = ~_T; // @[VME.scala 218:64]
  wire [2:0] _vme_select_T = VMEcmd_Qs_4_io_deq_valid ? 3'h4 : 3'h5; // @[Mux.scala 47:69]
  wire [2:0] _vme_select_T_1 = VMEcmd_Qs_3_io_deq_valid ? 3'h3 : _vme_select_T; // @[Mux.scala 47:69]
  wire [2:0] _vme_select_T_2 = VMEcmd_Qs_2_io_deq_valid ? 3'h2 : _vme_select_T_1; // @[Mux.scala 47:69]
  wire [2:0] _vme_select_T_3 = VMEcmd_Qs_1_io_deq_valid ? 3'h1 : _vme_select_T_2; // @[Mux.scala 47:69]
  wire [2:0] vme_select = VMEcmd_Qs_0_io_deq_valid ? 3'h0 : _vme_select_T_3; // @[Mux.scala 47:69]
  wire  _VMEcmd_Qs_0_io_deq_ready_T = vme_select == 3'h0; // @[VME.scala 265:17]
  wire  _VMEcmd_Qs_0_io_deq_ready_T_1 = io_mem_ar_ready & _VMEcmd_Qs_0_io_deq_ready_T; // @[VME.scala 264:50]
  wire  _VMEcmd_Qs_1_io_deq_ready_T = vme_select == 3'h1; // @[VME.scala 265:17]
  wire  _VMEcmd_Qs_1_io_deq_ready_T_1 = io_mem_ar_ready & _VMEcmd_Qs_1_io_deq_ready_T; // @[VME.scala 264:50]
  wire  _VMEcmd_Qs_2_io_deq_ready_T = vme_select == 3'h2; // @[VME.scala 265:17]
  wire  _VMEcmd_Qs_2_io_deq_ready_T_1 = io_mem_ar_ready & _VMEcmd_Qs_2_io_deq_ready_T; // @[VME.scala 264:50]
  wire  _VMEcmd_Qs_3_io_deq_ready_T = vme_select == 3'h3; // @[VME.scala 265:17]
  wire  _VMEcmd_Qs_3_io_deq_ready_T_1 = io_mem_ar_ready & _VMEcmd_Qs_3_io_deq_ready_T; // @[VME.scala 264:50]
  wire  _VMEcmd_Qs_4_io_deq_ready_T = vme_select == 3'h4; // @[VME.scala 265:17]
  wire  _VMEcmd_Qs_4_io_deq_ready_T_1 = io_mem_ar_ready & _VMEcmd_Qs_4_io_deq_ready_T; // @[VME.scala 264:50]
  wire  _any_cmd_ready_T_3 = VMEcmd_Qs_0_io_deq_ready | VMEcmd_Qs_1_io_deq_ready | VMEcmd_Qs_2_io_deq_ready |
    VMEcmd_Qs_3_io_deq_ready; // @[VME.scala 274:69]
  wire [20:0] _GEN_16 = VMEcmd_Qs_4_io_deq_ready ? VMEcmd_Qs_4_io_deq_bits_tag : 21'h0; // @[VME.scala 291:36 VME.scala 297:39 VME.scala 287:24]
  wire [20:0] _GEN_23 = VMEcmd_Qs_3_io_deq_ready ? VMEcmd_Qs_3_io_deq_bits_tag : _GEN_16; // @[VME.scala 291:36 VME.scala 297:39]
  wire [20:0] _GEN_30 = VMEcmd_Qs_2_io_deq_ready ? VMEcmd_Qs_2_io_deq_bits_tag : _GEN_23; // @[VME.scala 291:36 VME.scala 297:39]
  wire [20:0] _GEN_37 = VMEcmd_Qs_1_io_deq_ready ? VMEcmd_Qs_1_io_deq_bits_tag : _GEN_30; // @[VME.scala 291:36 VME.scala 297:39]
  wire [2:0] _GEN_15 = VMEcmd_Qs_4_io_deq_ready ? 3'h4 : 3'h0; // @[VME.scala 291:36 VME.scala 296:39 VME.scala 287:24]
  wire [2:0] _GEN_22 = VMEcmd_Qs_3_io_deq_ready ? 3'h3 : _GEN_15; // @[VME.scala 291:36 VME.scala 296:39]
  wire [2:0] _GEN_29 = VMEcmd_Qs_2_io_deq_ready ? 3'h2 : _GEN_22; // @[VME.scala 291:36 VME.scala 296:39]
  wire [2:0] _GEN_36 = VMEcmd_Qs_1_io_deq_ready ? 3'h1 : _GEN_29; // @[VME.scala 291:36 VME.scala 296:39]
  wire [31:0] _GEN_11 = VMEcmd_Qs_4_io_deq_ready ? VMEcmd_Qs_4_io_deq_bits_addr : 32'h0; // @[VME.scala 291:36 VME.scala 292:27 VME.scala 283:23]
  wire [7:0] _GEN_12 = VMEcmd_Qs_4_io_deq_ready ? VMEcmd_Qs_4_io_deq_bits_len : 8'h0; // @[VME.scala 291:36 VME.scala 293:27 VME.scala 284:23]
  wire  _GEN_13 = VMEcmd_Qs_4_io_deq_ready & VMEcmd_Qs_4_io_deq_valid; // @[VME.scala 291:36 VME.scala 294:27 VME.scala 285:23]
  wire [3:0] _GEN_14 = VMEcmd_Qs_4_io_deq_ready ? firstPostn : 4'h0; // @[VME.scala 291:36 VME.scala 295:27 VME.scala 286:23]
  wire [31:0] _GEN_18 = VMEcmd_Qs_3_io_deq_ready ? VMEcmd_Qs_3_io_deq_bits_addr : _GEN_11; // @[VME.scala 291:36 VME.scala 292:27]
  wire [7:0] _GEN_19 = VMEcmd_Qs_3_io_deq_ready ? VMEcmd_Qs_3_io_deq_bits_len : _GEN_12; // @[VME.scala 291:36 VME.scala 293:27]
  wire  _GEN_20 = VMEcmd_Qs_3_io_deq_ready ? VMEcmd_Qs_3_io_deq_valid : _GEN_13; // @[VME.scala 291:36 VME.scala 294:27]
  wire [3:0] _GEN_21 = VMEcmd_Qs_3_io_deq_ready ? firstPostn : _GEN_14; // @[VME.scala 291:36 VME.scala 295:27]
  wire [31:0] _GEN_25 = VMEcmd_Qs_2_io_deq_ready ? VMEcmd_Qs_2_io_deq_bits_addr : _GEN_18; // @[VME.scala 291:36 VME.scala 292:27]
  wire [7:0] _GEN_26 = VMEcmd_Qs_2_io_deq_ready ? VMEcmd_Qs_2_io_deq_bits_len : _GEN_19; // @[VME.scala 291:36 VME.scala 293:27]
  wire  _GEN_27 = VMEcmd_Qs_2_io_deq_ready ? VMEcmd_Qs_2_io_deq_valid : _GEN_20; // @[VME.scala 291:36 VME.scala 294:27]
  wire [3:0] _GEN_28 = VMEcmd_Qs_2_io_deq_ready ? firstPostn : _GEN_21; // @[VME.scala 291:36 VME.scala 295:27]
  wire [31:0] _GEN_32 = VMEcmd_Qs_1_io_deq_ready ? VMEcmd_Qs_1_io_deq_bits_addr : _GEN_25; // @[VME.scala 291:36 VME.scala 292:27]
  wire [7:0] _GEN_33 = VMEcmd_Qs_1_io_deq_ready ? VMEcmd_Qs_1_io_deq_bits_len : _GEN_26; // @[VME.scala 291:36 VME.scala 293:27]
  wire  _GEN_34 = VMEcmd_Qs_1_io_deq_ready ? VMEcmd_Qs_1_io_deq_valid : _GEN_27; // @[VME.scala 291:36 VME.scala 294:27]
  wire [3:0] _GEN_35 = VMEcmd_Qs_1_io_deq_ready ? firstPostn : _GEN_28; // @[VME.scala 291:36 VME.scala 295:27]
  wire [3:0] _GEN_42 = VMEcmd_Qs_0_io_deq_ready ? firstPostn : _GEN_35; // @[VME.scala 291:36 VME.scala 295:27]
  reg  io_vme_rd_0_data_valid_REG; // @[VME.scala 312:41]
  wire [2:0] localTag_out_client_id = vmeTag_array_client_id_localTag_out_MPORT_data; // @[VME.scala 209:27 VME.scala 308:24]
  wire  _io_vme_rd_0_data_valid_T_1 = io_vme_rd_0_data_valid_REG & localTag_out_client_id == 3'h0; // @[VME.scala 312:75]
  reg [63:0] io_vme_rd_0_data_bits_data_REG; // @[VME.scala 316:43]
  reg  io_vme_rd_1_data_valid_REG; // @[VME.scala 312:41]
  reg [63:0] io_vme_rd_1_data_bits_data_REG; // @[VME.scala 316:43]
  reg  io_vme_rd_1_data_bits_last_REG; // @[VME.scala 317:43]
  reg  io_vme_rd_2_data_valid_REG; // @[VME.scala 312:41]
  reg [63:0] io_vme_rd_2_data_bits_data_REG; // @[VME.scala 316:43]
  reg  io_vme_rd_3_data_valid_REG; // @[VME.scala 312:41]
  reg [63:0] io_vme_rd_3_data_bits_data_REG; // @[VME.scala 316:43]
  reg  io_vme_rd_4_data_valid_REG; // @[VME.scala 312:41]
  reg [63:0] io_vme_rd_4_data_bits_data_REG; // @[VME.scala 316:43]
  reg [7:0] wr_len; // @[VME.scala 322:23]
  reg [31:0] wr_addr; // @[VME.scala 323:24]
  reg [1:0] wstate; // @[VME.scala 325:23]
  reg [7:0] wr_cnt; // @[VME.scala 326:23]
  wire  _io_vme_wr_0_cmd_ready_T = wstate == 2'h0; // @[VME.scala 327:36]
  wire  _io_vme_wr_0_data_ready_T = wstate == 2'h2; // @[VME.scala 329:37]
  wire  _io_mem_w_bits_last_T = wr_cnt == wr_len; // @[VME.scala 337:32]
  wire  _T_32 = io_vme_wr_0_cmd_ready & io_vme_wr_0_cmd_valid; // @[Decoupled.scala 40:37]
  wire  _T_34 = io_mem_w_ready & io_mem_w_valid; // @[Decoupled.scala 40:37]
  wire [7:0] _wr_cnt_T_1 = wr_cnt + 8'h1; // @[VME.scala 348:22]
  wire  _T_35 = 2'h0 == wstate; // @[Conditional.scala 37:30]
  wire  _T_36 = 2'h1 == wstate; // @[Conditional.scala 37:30]
  wire  _T_37 = 2'h2 == wstate; // @[Conditional.scala 37:30]
  wire [1:0] _GEN_52 = io_vme_wr_0_data_valid & io_mem_w_ready & _io_mem_w_bits_last_T ? 2'h3 : wstate; // @[VME.scala 362:76 VME.scala 363:16 VME.scala 325:23]
  wire  _T_41 = 2'h3 == wstate; // @[Conditional.scala 37:30]
  wire [1:0] _GEN_53 = io_mem_b_valid ? 2'h0 : wstate; // @[VME.scala 367:28 VME.scala 368:16 VME.scala 325:23]
  wire [1:0] _GEN_54 = _T_41 ? _GEN_53 : wstate; // @[Conditional.scala 39:67 VME.scala 325:23]
  Queue_1 VMEcmd_Qs_0 ( // @[VME.scala 231:45]
    .clock(VMEcmd_Qs_0_clock),
    .reset(VMEcmd_Qs_0_reset),
    .io_enq_ready(VMEcmd_Qs_0_io_enq_ready),
    .io_enq_valid(VMEcmd_Qs_0_io_enq_valid),
    .io_enq_bits_addr(VMEcmd_Qs_0_io_enq_bits_addr),
    .io_enq_bits_len(VMEcmd_Qs_0_io_enq_bits_len),
    .io_enq_bits_tag(VMEcmd_Qs_0_io_enq_bits_tag),
    .io_deq_ready(VMEcmd_Qs_0_io_deq_ready),
    .io_deq_valid(VMEcmd_Qs_0_io_deq_valid),
    .io_deq_bits_addr(VMEcmd_Qs_0_io_deq_bits_addr),
    .io_deq_bits_len(VMEcmd_Qs_0_io_deq_bits_len),
    .io_deq_bits_tag(VMEcmd_Qs_0_io_deq_bits_tag)
  );
  Queue_1 VMEcmd_Qs_1 ( // @[VME.scala 231:45]
    .clock(VMEcmd_Qs_1_clock),
    .reset(VMEcmd_Qs_1_reset),
    .io_enq_ready(VMEcmd_Qs_1_io_enq_ready),
    .io_enq_valid(VMEcmd_Qs_1_io_enq_valid),
    .io_enq_bits_addr(VMEcmd_Qs_1_io_enq_bits_addr),
    .io_enq_bits_len(VMEcmd_Qs_1_io_enq_bits_len),
    .io_enq_bits_tag(VMEcmd_Qs_1_io_enq_bits_tag),
    .io_deq_ready(VMEcmd_Qs_1_io_deq_ready),
    .io_deq_valid(VMEcmd_Qs_1_io_deq_valid),
    .io_deq_bits_addr(VMEcmd_Qs_1_io_deq_bits_addr),
    .io_deq_bits_len(VMEcmd_Qs_1_io_deq_bits_len),
    .io_deq_bits_tag(VMEcmd_Qs_1_io_deq_bits_tag)
  );
  Queue_1 VMEcmd_Qs_2 ( // @[VME.scala 231:45]
    .clock(VMEcmd_Qs_2_clock),
    .reset(VMEcmd_Qs_2_reset),
    .io_enq_ready(VMEcmd_Qs_2_io_enq_ready),
    .io_enq_valid(VMEcmd_Qs_2_io_enq_valid),
    .io_enq_bits_addr(VMEcmd_Qs_2_io_enq_bits_addr),
    .io_enq_bits_len(VMEcmd_Qs_2_io_enq_bits_len),
    .io_enq_bits_tag(VMEcmd_Qs_2_io_enq_bits_tag),
    .io_deq_ready(VMEcmd_Qs_2_io_deq_ready),
    .io_deq_valid(VMEcmd_Qs_2_io_deq_valid),
    .io_deq_bits_addr(VMEcmd_Qs_2_io_deq_bits_addr),
    .io_deq_bits_len(VMEcmd_Qs_2_io_deq_bits_len),
    .io_deq_bits_tag(VMEcmd_Qs_2_io_deq_bits_tag)
  );
  Queue_1 VMEcmd_Qs_3 ( // @[VME.scala 231:45]
    .clock(VMEcmd_Qs_3_clock),
    .reset(VMEcmd_Qs_3_reset),
    .io_enq_ready(VMEcmd_Qs_3_io_enq_ready),
    .io_enq_valid(VMEcmd_Qs_3_io_enq_valid),
    .io_enq_bits_addr(VMEcmd_Qs_3_io_enq_bits_addr),
    .io_enq_bits_len(VMEcmd_Qs_3_io_enq_bits_len),
    .io_enq_bits_tag(VMEcmd_Qs_3_io_enq_bits_tag),
    .io_deq_ready(VMEcmd_Qs_3_io_deq_ready),
    .io_deq_valid(VMEcmd_Qs_3_io_deq_valid),
    .io_deq_bits_addr(VMEcmd_Qs_3_io_deq_bits_addr),
    .io_deq_bits_len(VMEcmd_Qs_3_io_deq_bits_len),
    .io_deq_bits_tag(VMEcmd_Qs_3_io_deq_bits_tag)
  );
  Queue_1 VMEcmd_Qs_4 ( // @[VME.scala 231:45]
    .clock(VMEcmd_Qs_4_clock),
    .reset(VMEcmd_Qs_4_reset),
    .io_enq_ready(VMEcmd_Qs_4_io_enq_ready),
    .io_enq_valid(VMEcmd_Qs_4_io_enq_valid),
    .io_enq_bits_addr(VMEcmd_Qs_4_io_enq_bits_addr),
    .io_enq_bits_len(VMEcmd_Qs_4_io_enq_bits_len),
    .io_enq_bits_tag(VMEcmd_Qs_4_io_enq_bits_tag),
    .io_deq_ready(VMEcmd_Qs_4_io_deq_ready),
    .io_deq_valid(VMEcmd_Qs_4_io_deq_valid),
    .io_deq_bits_addr(VMEcmd_Qs_4_io_deq_bits_addr),
    .io_deq_bits_len(VMEcmd_Qs_4_io_deq_bits_len),
    .io_deq_bits_tag(VMEcmd_Qs_4_io_deq_bits_tag)
  );
  assign vmeTag_array_client_id_localTag_out_MPORT_addr = vmeTag_array_client_id_localTag_out_MPORT_addr_pipe_0;
  assign vmeTag_array_client_id_localTag_out_MPORT_data =
    vmeTag_array_client_id[vmeTag_array_client_id_localTag_out_MPORT_addr]; // @[VME.scala 204:33]
  assign vmeTag_array_client_id_rdwrPort_data = VMEcmd_Qs_0_io_deq_ready ? 3'h0 : _GEN_36;
  assign vmeTag_array_client_id_rdwrPort_addr = oneHotIdx_0 ? 4'h0 : _bitPostn_T_13;
  assign vmeTag_array_client_id_rdwrPort_mask = 1'h1;
  assign vmeTag_array_client_id_rdwrPort_en = _any_cmd_ready_T_3 | VMEcmd_Qs_4_io_deq_ready;
  assign vmeTag_array_client_tag_localTag_out_MPORT_addr = vmeTag_array_client_tag_localTag_out_MPORT_addr_pipe_0;
  assign vmeTag_array_client_tag_localTag_out_MPORT_data =
    vmeTag_array_client_tag[vmeTag_array_client_tag_localTag_out_MPORT_addr]; // @[VME.scala 204:33]
  assign vmeTag_array_client_tag_rdwrPort_data = VMEcmd_Qs_0_io_deq_ready ? VMEcmd_Qs_0_io_deq_bits_tag : _GEN_37;
  assign vmeTag_array_client_tag_rdwrPort_addr = oneHotIdx_0 ? 4'h0 : _bitPostn_T_13;
  assign vmeTag_array_client_tag_rdwrPort_mask = 1'h1;
  assign vmeTag_array_client_tag_rdwrPort_en = _any_cmd_ready_T_3 | VMEcmd_Qs_4_io_deq_ready;
  assign io_mem_aw_valid = wstate == 2'h1; // @[VME.scala 330:29]
  assign io_mem_aw_bits_addr = wr_addr; // @[VME.scala 331:23]
  assign io_mem_aw_bits_len = wr_len; // @[VME.scala 332:22]
  assign io_mem_w_valid = _io_vme_wr_0_data_ready_T & io_vme_wr_0_data_valid; // @[VME.scala 334:43]
  assign io_mem_w_bits_data = io_vme_wr_0_data_bits_data; // @[VME.scala 335:22]
  assign io_mem_w_bits_last = wr_cnt == wr_len; // @[VME.scala 337:32]
  assign io_mem_b_ready = wstate == 2'h3; // @[VME.scala 339:28]
  assign io_mem_ar_valid = VMEcmd_Qs_0_io_deq_ready ? VMEcmd_Qs_0_io_deq_valid : _GEN_34; // @[VME.scala 291:36 VME.scala 294:27]
  assign io_mem_ar_bits_addr = VMEcmd_Qs_0_io_deq_ready ? VMEcmd_Qs_0_io_deq_bits_addr : _GEN_32; // @[VME.scala 291:36 VME.scala 292:27]
  assign io_mem_ar_bits_id = {{4'd0}, _GEN_42}; // @[VME.scala 291:36 VME.scala 295:27]
  assign io_mem_ar_bits_len = VMEcmd_Qs_0_io_deq_ready ? VMEcmd_Qs_0_io_deq_bits_len : _GEN_33; // @[VME.scala 291:36 VME.scala 293:27]
  assign io_vme_rd_0_cmd_ready = VMEcmd_Qs_0_io_enq_ready; // @[VME.scala 267:28]
  assign io_vme_rd_0_data_valid = _io_vme_rd_0_data_valid_T_1 & io_vme_rd_0_data_ready; // @[VME.scala 313:5]
  assign io_vme_rd_0_data_bits_data = io_vme_rd_0_data_bits_data_REG; // @[VME.scala 316:33]
  assign io_vme_rd_1_cmd_ready = VMEcmd_Qs_1_io_enq_ready; // @[VME.scala 267:28]
  assign io_vme_rd_1_data_valid = io_vme_rd_1_data_valid_REG & localTag_out_client_id == 3'h1; // @[VME.scala 312:75]
  assign io_vme_rd_1_data_bits_data = io_vme_rd_1_data_bits_data_REG; // @[VME.scala 316:33]
  assign io_vme_rd_1_data_bits_tag = vmeTag_array_client_tag_localTag_out_MPORT_data; // @[VME.scala 209:27 VME.scala 308:24]
  assign io_vme_rd_1_data_bits_last = io_vme_rd_1_data_bits_last_REG; // @[VME.scala 317:33]
  assign io_vme_rd_2_cmd_ready = VMEcmd_Qs_2_io_enq_ready; // @[VME.scala 267:28]
  assign io_vme_rd_2_data_valid = io_vme_rd_2_data_valid_REG & localTag_out_client_id == 3'h2; // @[VME.scala 312:75]
  assign io_vme_rd_2_data_bits_data = io_vme_rd_2_data_bits_data_REG; // @[VME.scala 316:33]
  assign io_vme_rd_2_data_bits_tag = vmeTag_array_client_tag_localTag_out_MPORT_data; // @[VME.scala 209:27 VME.scala 308:24]
  assign io_vme_rd_3_cmd_ready = VMEcmd_Qs_3_io_enq_ready; // @[VME.scala 267:28]
  assign io_vme_rd_3_data_valid = io_vme_rd_3_data_valid_REG & localTag_out_client_id == 3'h3; // @[VME.scala 312:75]
  assign io_vme_rd_3_data_bits_data = io_vme_rd_3_data_bits_data_REG; // @[VME.scala 316:33]
  assign io_vme_rd_3_data_bits_tag = vmeTag_array_client_tag_localTag_out_MPORT_data; // @[VME.scala 209:27 VME.scala 308:24]
  assign io_vme_rd_4_cmd_ready = VMEcmd_Qs_4_io_enq_ready; // @[VME.scala 267:28]
  assign io_vme_rd_4_data_valid = io_vme_rd_4_data_valid_REG & localTag_out_client_id == 3'h4; // @[VME.scala 312:75]
  assign io_vme_rd_4_data_bits_data = io_vme_rd_4_data_bits_data_REG; // @[VME.scala 316:33]
  assign io_vme_rd_4_data_bits_tag = vmeTag_array_client_tag_localTag_out_MPORT_data; // @[VME.scala 209:27 VME.scala 308:24]
  assign io_vme_wr_0_cmd_ready = wstate == 2'h0; // @[VME.scala 327:36]
  assign io_vme_wr_0_data_ready = wstate == 2'h2 & io_mem_w_ready; // @[VME.scala 329:52]
  assign io_vme_wr_0_ack = io_mem_b_ready & io_mem_b_valid; // @[Decoupled.scala 40:37]
  assign VMEcmd_Qs_0_clock = clock;
  assign VMEcmd_Qs_0_reset = reset;
  assign VMEcmd_Qs_0_io_enq_valid = io_vme_rd_0_cmd_valid & VMEcmd_Qs_0_io_enq_ready; // @[VME.scala 262:58]
  assign VMEcmd_Qs_0_io_enq_bits_addr = io_vme_rd_0_cmd_bits_addr; // @[VME.scala 263:31]
  assign VMEcmd_Qs_0_io_enq_bits_len = io_vme_rd_0_cmd_bits_len; // @[VME.scala 263:31]
  assign VMEcmd_Qs_0_io_enq_bits_tag = 21'h0; // @[VME.scala 263:31]
  assign VMEcmd_Qs_0_io_deq_ready = _VMEcmd_Qs_0_io_deq_ready_T_1 & _T_1 & _T_4; // @[VME.scala 265:62]
  assign VMEcmd_Qs_1_clock = clock;
  assign VMEcmd_Qs_1_reset = reset;
  assign VMEcmd_Qs_1_io_enq_valid = io_vme_rd_1_cmd_valid & VMEcmd_Qs_1_io_enq_ready; // @[VME.scala 262:58]
  assign VMEcmd_Qs_1_io_enq_bits_addr = io_vme_rd_1_cmd_bits_addr; // @[VME.scala 263:31]
  assign VMEcmd_Qs_1_io_enq_bits_len = io_vme_rd_1_cmd_bits_len; // @[VME.scala 263:31]
  assign VMEcmd_Qs_1_io_enq_bits_tag = io_vme_rd_1_cmd_bits_tag; // @[VME.scala 263:31]
  assign VMEcmd_Qs_1_io_deq_ready = _VMEcmd_Qs_1_io_deq_ready_T_1 & _T_1 & _T_4; // @[VME.scala 265:62]
  assign VMEcmd_Qs_2_clock = clock;
  assign VMEcmd_Qs_2_reset = reset;
  assign VMEcmd_Qs_2_io_enq_valid = io_vme_rd_2_cmd_valid & VMEcmd_Qs_2_io_enq_ready; // @[VME.scala 262:58]
  assign VMEcmd_Qs_2_io_enq_bits_addr = io_vme_rd_2_cmd_bits_addr; // @[VME.scala 263:31]
  assign VMEcmd_Qs_2_io_enq_bits_len = io_vme_rd_2_cmd_bits_len; // @[VME.scala 263:31]
  assign VMEcmd_Qs_2_io_enq_bits_tag = io_vme_rd_2_cmd_bits_tag; // @[VME.scala 263:31]
  assign VMEcmd_Qs_2_io_deq_ready = _VMEcmd_Qs_2_io_deq_ready_T_1 & _T_1 & _T_4; // @[VME.scala 265:62]
  assign VMEcmd_Qs_3_clock = clock;
  assign VMEcmd_Qs_3_reset = reset;
  assign VMEcmd_Qs_3_io_enq_valid = io_vme_rd_3_cmd_valid & VMEcmd_Qs_3_io_enq_ready; // @[VME.scala 262:58]
  assign VMEcmd_Qs_3_io_enq_bits_addr = io_vme_rd_3_cmd_bits_addr; // @[VME.scala 263:31]
  assign VMEcmd_Qs_3_io_enq_bits_len = io_vme_rd_3_cmd_bits_len; // @[VME.scala 263:31]
  assign VMEcmd_Qs_3_io_enq_bits_tag = io_vme_rd_3_cmd_bits_tag; // @[VME.scala 263:31]
  assign VMEcmd_Qs_3_io_deq_ready = _VMEcmd_Qs_3_io_deq_ready_T_1 & _T_1 & _T_4; // @[VME.scala 265:62]
  assign VMEcmd_Qs_4_clock = clock;
  assign VMEcmd_Qs_4_reset = reset;
  assign VMEcmd_Qs_4_io_enq_valid = io_vme_rd_4_cmd_valid & VMEcmd_Qs_4_io_enq_ready; // @[VME.scala 262:58]
  assign VMEcmd_Qs_4_io_enq_bits_addr = io_vme_rd_4_cmd_bits_addr; // @[VME.scala 263:31]
  assign VMEcmd_Qs_4_io_enq_bits_len = io_vme_rd_4_cmd_bits_len; // @[VME.scala 263:31]
  assign VMEcmd_Qs_4_io_enq_bits_tag = io_vme_rd_4_cmd_bits_tag; // @[VME.scala 263:31]
  assign VMEcmd_Qs_4_io_deq_ready = _VMEcmd_Qs_4_io_deq_ready_T_1 & _T_1 & _T_4; // @[VME.scala 265:62]
  always @(posedge clock) begin
    if(vmeTag_array_client_id_rdwrPort_en & vmeTag_array_client_id_rdwrPort_mask) begin
      vmeTag_array_client_id[vmeTag_array_client_id_rdwrPort_addr] <= vmeTag_array_client_id_rdwrPort_data; // @[VME.scala 204:33]
    end
    vmeTag_array_client_id_localTag_out_MPORT_addr_pipe_0 <= io_mem_r_bits_id[3:0];
    if(vmeTag_array_client_tag_rdwrPort_en & vmeTag_array_client_tag_rdwrPort_mask) begin
      vmeTag_array_client_tag[vmeTag_array_client_tag_rdwrPort_addr] <= vmeTag_array_client_tag_rdwrPort_data; // @[VME.scala 204:33]
    end
    vmeTag_array_client_tag_localTag_out_MPORT_addr_pipe_0 <= io_mem_r_bits_id[3:0];
    if (reset) begin // @[VME.scala 223:21]
      availableEntries <= 16'hffff; // @[VME.scala 224:20]
    end else if (io_mem_r_bits_last & io_mem_r_valid) begin // @[VME.scala 216:44]
      availableEntries <= _availableEntriesNext_T; // @[VME.scala 217:24]
    end else if (availableEntriesEn & availableEntries != 16'h0 & ~_T) begin // @[VME.scala 218:103]
      availableEntries <= newEntry; // @[VME.scala 219:23]
    end
    if (reset) begin // @[VME.scala 312:41]
      io_vme_rd_0_data_valid_REG <= 1'h0; // @[VME.scala 312:41]
    end else begin
      io_vme_rd_0_data_valid_REG <= io_mem_r_valid; // @[VME.scala 312:41]
    end
    if (reset) begin // @[VME.scala 316:43]
      io_vme_rd_0_data_bits_data_REG <= 64'h0; // @[VME.scala 316:43]
    end else begin
      io_vme_rd_0_data_bits_data_REG <= io_mem_r_bits_data; // @[VME.scala 316:43]
    end
    if (reset) begin // @[VME.scala 312:41]
      io_vme_rd_1_data_valid_REG <= 1'h0; // @[VME.scala 312:41]
    end else begin
      io_vme_rd_1_data_valid_REG <= io_mem_r_valid; // @[VME.scala 312:41]
    end
    if (reset) begin // @[VME.scala 316:43]
      io_vme_rd_1_data_bits_data_REG <= 64'h0; // @[VME.scala 316:43]
    end else begin
      io_vme_rd_1_data_bits_data_REG <= io_mem_r_bits_data; // @[VME.scala 316:43]
    end
    if (reset) begin // @[VME.scala 317:43]
      io_vme_rd_1_data_bits_last_REG <= 1'h0; // @[VME.scala 317:43]
    end else begin
      io_vme_rd_1_data_bits_last_REG <= io_mem_r_bits_last; // @[VME.scala 317:43]
    end
    if (reset) begin // @[VME.scala 312:41]
      io_vme_rd_2_data_valid_REG <= 1'h0; // @[VME.scala 312:41]
    end else begin
      io_vme_rd_2_data_valid_REG <= io_mem_r_valid; // @[VME.scala 312:41]
    end
    if (reset) begin // @[VME.scala 316:43]
      io_vme_rd_2_data_bits_data_REG <= 64'h0; // @[VME.scala 316:43]
    end else begin
      io_vme_rd_2_data_bits_data_REG <= io_mem_r_bits_data; // @[VME.scala 316:43]
    end
    if (reset) begin // @[VME.scala 312:41]
      io_vme_rd_3_data_valid_REG <= 1'h0; // @[VME.scala 312:41]
    end else begin
      io_vme_rd_3_data_valid_REG <= io_mem_r_valid; // @[VME.scala 312:41]
    end
    if (reset) begin // @[VME.scala 316:43]
      io_vme_rd_3_data_bits_data_REG <= 64'h0; // @[VME.scala 316:43]
    end else begin
      io_vme_rd_3_data_bits_data_REG <= io_mem_r_bits_data; // @[VME.scala 316:43]
    end
    if (reset) begin // @[VME.scala 312:41]
      io_vme_rd_4_data_valid_REG <= 1'h0; // @[VME.scala 312:41]
    end else begin
      io_vme_rd_4_data_valid_REG <= io_mem_r_valid; // @[VME.scala 312:41]
    end
    if (reset) begin // @[VME.scala 316:43]
      io_vme_rd_4_data_bits_data_REG <= 64'h0; // @[VME.scala 316:43]
    end else begin
      io_vme_rd_4_data_bits_data_REG <= io_mem_r_bits_data; // @[VME.scala 316:43]
    end
    if (reset) begin // @[VME.scala 322:23]
      wr_len <= 8'h0; // @[VME.scala 322:23]
    end else if (_T_32) begin // @[VME.scala 340:33]
      wr_len <= io_vme_wr_0_cmd_bits_len; // @[VME.scala 341:12]
    end
    if (reset) begin // @[VME.scala 323:24]
      wr_addr <= 32'h0; // @[VME.scala 323:24]
    end else if (_T_32) begin // @[VME.scala 340:33]
      wr_addr <= io_vme_wr_0_cmd_bits_addr; // @[VME.scala 342:13]
    end
    if (reset) begin // @[VME.scala 325:23]
      wstate <= 2'h0; // @[VME.scala 325:23]
    end else if (_T_35) begin // @[Conditional.scala 40:58]
      if (io_vme_wr_0_cmd_valid) begin // @[VME.scala 352:35]
        wstate <= 2'h1; // @[VME.scala 353:16]
      end
    end else if (_T_36) begin // @[Conditional.scala 39:67]
      if (io_mem_aw_ready) begin // @[VME.scala 357:28]
        wstate <= 2'h2; // @[VME.scala 358:16]
      end
    end else if (_T_37) begin // @[Conditional.scala 39:67]
      wstate <= _GEN_52;
    end else begin
      wstate <= _GEN_54;
    end
    if (reset) begin // @[VME.scala 326:23]
      wr_cnt <= 8'h0; // @[VME.scala 326:23]
    end else if (_io_vme_wr_0_cmd_ready_T) begin // @[VME.scala 344:31]
      wr_cnt <= 8'h0; // @[VME.scala 345:12]
    end else if (_T_34) begin // @[VME.scala 347:29]
      wr_cnt <= _wr_cnt_T_1; // @[VME.scala 348:12]
    end
    `ifndef SYNTHESIS
    `ifdef PRINTF_COND
      if (`PRINTF_COND) begin
    `endif
        if (~(io_vme_rd_0_data_ready | ~io_vme_rd_0_data_valid | reset)) begin
          $fwrite(32'h80000002,
            "Assertion failed\n    at VME.scala:315 assert(io.vme.rd(i).data.ready || ~io.vme.rd(i).data.valid)\n"); // @[VME.scala 315:11]
        end
    `ifdef PRINTF_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef STOP_COND
      if (`STOP_COND) begin
    `endif
        if (~(io_vme_rd_0_data_ready | ~io_vme_rd_0_data_valid | reset)) begin
          $fatal; // @[VME.scala 315:11]
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
  for (initvar = 0; initvar < 16; initvar = initvar+1)
    vmeTag_array_client_id[initvar] = _RAND_0[2:0];
  _RAND_2 = {1{`RANDOM}};
  for (initvar = 0; initvar < 16; initvar = initvar+1)
    vmeTag_array_client_tag[initvar] = _RAND_2[20:0];
`endif // RANDOMIZE_MEM_INIT
`ifdef RANDOMIZE_REG_INIT
  _RAND_1 = {1{`RANDOM}};
  vmeTag_array_client_id_localTag_out_MPORT_addr_pipe_0 = _RAND_1[3:0];
  _RAND_3 = {1{`RANDOM}};
  vmeTag_array_client_tag_localTag_out_MPORT_addr_pipe_0 = _RAND_3[3:0];
  _RAND_4 = {1{`RANDOM}};
  availableEntries = _RAND_4[15:0];
  _RAND_5 = {1{`RANDOM}};
  io_vme_rd_0_data_valid_REG = _RAND_5[0:0];
  _RAND_6 = {2{`RANDOM}};
  io_vme_rd_0_data_bits_data_REG = _RAND_6[63:0];
  _RAND_7 = {1{`RANDOM}};
  io_vme_rd_1_data_valid_REG = _RAND_7[0:0];
  _RAND_8 = {2{`RANDOM}};
  io_vme_rd_1_data_bits_data_REG = _RAND_8[63:0];
  _RAND_9 = {1{`RANDOM}};
  io_vme_rd_1_data_bits_last_REG = _RAND_9[0:0];
  _RAND_10 = {1{`RANDOM}};
  io_vme_rd_2_data_valid_REG = _RAND_10[0:0];
  _RAND_11 = {2{`RANDOM}};
  io_vme_rd_2_data_bits_data_REG = _RAND_11[63:0];
  _RAND_12 = {1{`RANDOM}};
  io_vme_rd_3_data_valid_REG = _RAND_12[0:0];
  _RAND_13 = {2{`RANDOM}};
  io_vme_rd_3_data_bits_data_REG = _RAND_13[63:0];
  _RAND_14 = {1{`RANDOM}};
  io_vme_rd_4_data_valid_REG = _RAND_14[0:0];
  _RAND_15 = {2{`RANDOM}};
  io_vme_rd_4_data_bits_data_REG = _RAND_15[63:0];
  _RAND_16 = {1{`RANDOM}};
  wr_len = _RAND_16[7:0];
  _RAND_17 = {1{`RANDOM}};
  wr_addr = _RAND_17[31:0];
  _RAND_18 = {1{`RANDOM}};
  wstate = _RAND_18[1:0];
  _RAND_19 = {1{`RANDOM}};
  wr_cnt = _RAND_19[7:0];
`endif // RANDOMIZE_REG_INIT
  `endif // RANDOMIZE
end // initial
`ifdef FIRRTL_AFTER_INITIAL
`FIRRTL_AFTER_INITIAL
`endif
`endif // SYNTHESIS
endmodule