module GenVMECmdWideTL(
  input          clock,
  input          reset,
  input          io_start,
  input          io_isBusy,
  input  [127:0] io_inst,
  input  [31:0]  io_baddr,
  input          io_vmeCmd_ready,
  output         io_vmeCmd_valid,
  output [31:0]  io_vmeCmd_bits_addr,
  output [7:0]   io_vmeCmd_bits_len,
  output [20:0]  io_vmeCmd_bits_tag,
  output [8:0]   io_readLen,
  output         io_done
);
  wire  cmdGen_clock; // @[TensorLoadWideVME.scala 746:23]
  wire  cmdGen_reset; // @[TensorLoadWideVME.scala 746:23]
  wire  cmdGen_io_start; // @[TensorLoadWideVME.scala 746:23]
  wire  cmdGen_io_isBusy; // @[TensorLoadWideVME.scala 746:23]
  wire  cmdGen_io_updateState; // @[TensorLoadWideVME.scala 746:23]
  wire [31:0] cmdGen_io_baddr; // @[TensorLoadWideVME.scala 746:23]
  wire  cmdGen_io_vmeCmd_valid; // @[TensorLoadWideVME.scala 746:23]
  wire [31:0] cmdGen_io_vmeCmd_bits_addr; // @[TensorLoadWideVME.scala 746:23]
  wire [7:0] cmdGen_io_vmeCmd_bits_len; // @[TensorLoadWideVME.scala 746:23]
  wire [20:0] cmdGen_io_vmeCmd_bits_tag; // @[TensorLoadWideVME.scala 746:23]
  wire [8:0] cmdGen_io_readLen; // @[TensorLoadWideVME.scala 746:23]
  wire  cmdGen_io_done; // @[TensorLoadWideVME.scala 746:23]
  wire [15:0] cmdGen_io_ysize; // @[TensorLoadWideVME.scala 746:23]
  wire [15:0] cmdGen_io_xsize; // @[TensorLoadWideVME.scala 746:23]
  wire [15:0] cmdGen_io_xstride; // @[TensorLoadWideVME.scala 746:23]
  wire [31:0] cmdGen_io_dram_offset; // @[TensorLoadWideVME.scala 746:23]
  wire [15:0] cmdGen_io_sram_offset; // @[TensorLoadWideVME.scala 746:23]
  wire [3:0] cmdGen_io_xpad_0; // @[TensorLoadWideVME.scala 746:23]
  wire [3:0] cmdGen_io_xpad_1; // @[TensorLoadWideVME.scala 746:23]
  wire [3:0] cmdGen_io_ypad_0; // @[TensorLoadWideVME.scala 746:23]
  GenVMECmdWide cmdGen ( // @[TensorLoadWideVME.scala 746:23]
    .clock(cmdGen_clock),
    .reset(cmdGen_reset),
    .io_start(cmdGen_io_start),
    .io_isBusy(cmdGen_io_isBusy),
    .io_updateState(cmdGen_io_updateState),
    .io_baddr(cmdGen_io_baddr),
    .io_vmeCmd_valid(cmdGen_io_vmeCmd_valid),
    .io_vmeCmd_bits_addr(cmdGen_io_vmeCmd_bits_addr),
    .io_vmeCmd_bits_len(cmdGen_io_vmeCmd_bits_len),
    .io_vmeCmd_bits_tag(cmdGen_io_vmeCmd_bits_tag),
    .io_readLen(cmdGen_io_readLen),
    .io_done(cmdGen_io_done),
    .io_ysize(cmdGen_io_ysize),
    .io_xsize(cmdGen_io_xsize),
    .io_xstride(cmdGen_io_xstride),
    .io_dram_offset(cmdGen_io_dram_offset),
    .io_sram_offset(cmdGen_io_sram_offset),
    .io_xpad_0(cmdGen_io_xpad_0),
    .io_xpad_1(cmdGen_io_xpad_1),
    .io_ypad_0(cmdGen_io_ypad_0)
  );
  assign io_vmeCmd_valid = cmdGen_io_vmeCmd_valid; // @[TensorLoadWideVME.scala 751:13]
  assign io_vmeCmd_bits_addr = cmdGen_io_vmeCmd_bits_addr; // @[TensorLoadWideVME.scala 751:13]
  assign io_vmeCmd_bits_len = cmdGen_io_vmeCmd_bits_len; // @[TensorLoadWideVME.scala 751:13]
  assign io_vmeCmd_bits_tag = cmdGen_io_vmeCmd_bits_tag; // @[TensorLoadWideVME.scala 751:13]
  assign io_readLen = cmdGen_io_readLen; // @[TensorLoadWideVME.scala 752:14]
  assign io_done = cmdGen_io_done; // @[TensorLoadWideVME.scala 753:11]
  assign cmdGen_clock = clock;
  assign cmdGen_reset = reset;
  assign cmdGen_io_start = io_start; // @[TensorLoadWideVME.scala 748:19]
  assign cmdGen_io_isBusy = io_isBusy; // @[TensorLoadWideVME.scala 749:20]
  assign cmdGen_io_updateState = io_vmeCmd_ready & io_vmeCmd_valid; // @[Decoupled.scala 40:37]
  assign cmdGen_io_baddr = io_baddr; // @[TensorLoadWideVME.scala 750:19]
  assign cmdGen_io_ysize = io_inst[79:64]; // @[TensorLoadWideVME.scala 744:29]
  assign cmdGen_io_xsize = io_inst[95:80]; // @[TensorLoadWideVME.scala 744:29]
  assign cmdGen_io_xstride = io_inst[111:96]; // @[TensorLoadWideVME.scala 744:29]
  assign cmdGen_io_dram_offset = io_inst[57:26]; // @[TensorLoadWideVME.scala 744:29]
  assign cmdGen_io_sram_offset = io_inst[25:10]; // @[TensorLoadWideVME.scala 744:29]
  assign cmdGen_io_xpad_0 = io_inst[123:120]; // @[TensorLoadWideVME.scala 744:29]
  assign cmdGen_io_xpad_1 = io_inst[127:124]; // @[TensorLoadWideVME.scala 744:29]
  assign cmdGen_io_ypad_0 = io_inst[115:112]; // @[TensorLoadWideVME.scala 744:29]
endmodule