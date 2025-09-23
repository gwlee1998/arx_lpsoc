module ReadVMEDataWide(
  input         clock,
  input         reset,
  input         io_start,
  output        io_vmeData_ready,
  input         io_vmeData_valid,
  input  [63:0] io_vmeData_bits_data,
  input  [20:0] io_vmeData_bits_tag,
  input         io_vmeData_bits_last,
  output [10:0] io_destIdx_0,
  output [10:0] io_destIdx_1,
  output [31:0] io_destData_0,
  output [31:0] io_destData_1,
  output        io_destMask_0,
  output        io_destMask_1
);
`ifdef RANDOMIZE_REG_INIT
  reg [31:0] _RAND_0;
  reg [31:0] _RAND_1;
  reg [31:0] _RAND_2;
`endif // RANDOMIZE_REG_INIT
  reg [20:0] vmeTagDecodeLast; // @[TensorLoadWideVME.scala 352:29]
  wire [16:0] rdDataElemIdx = io_vmeData_bits_tag[20:4]; // @[TensorLoadWideVME.scala 356:35]
  wire [1:0] rdFstOffsetNb = io_vmeData_bits_tag[3:2]; // @[TensorLoadWideVME.scala 360:35]
  wire [1:0] rdLstNb = io_vmeData_bits_tag[1:0]; // @[TensorLoadWideVME.scala 366:31]
  wire [1:0] _wrMask1st_T_1 = 2'h2 - rdFstOffsetNb; // @[TensorLoadWideVME.scala 374:32]
  wire  _wrMask1st_T_2 = 2'h0 < _wrMask1st_T_1; // @[TensorLoadWideVME.scala 374:13]
  wire  _wrMask1st_T_5 = 2'h1 < _wrMask1st_T_1; // @[TensorLoadWideVME.scala 374:13]
  wire [1:0] _wrMask1st_T_6 = {_wrMask1st_T_5,_wrMask1st_T_2}; // @[TensorLoadWideVME.scala 375:8]
  wire  wrMask1st_hi = _wrMask1st_T_6[0]; // @[Bitwise.scala 109:18]
  wire  wrMask1st_lo = _wrMask1st_T_6[1]; // @[Bitwise.scala 109:44]
  wire [1:0] wrMask1st = {wrMask1st_hi,wrMask1st_lo}; // @[Cat.scala 30:58]
  wire  _wrMaskLast_T = 2'h0 < rdLstNb; // @[TensorLoadWideVME.scala 381:13]
  wire  _wrMaskLast_T_1 = 2'h1 < rdLstNb; // @[TensorLoadWideVME.scala 381:13]
  wire [1:0] wrMaskLast = {_wrMaskLast_T_1,_wrMaskLast_T}; // @[TensorLoadWideVME.scala 382:8]
  reg [10:0] rdDataElemDestIdxNext; // @[TensorLoadWideVME.scala 385:34]
  wire  _T_1 = io_vmeData_ready & io_vmeData_valid; // @[Decoupled.scala 40:37]
  reg  vmeTagDecodeLastValidNext; // @[TensorLoadWideVME.scala 390:42]
  wire  _T_3 = io_vmeData_bits_tag != vmeTagDecodeLast; // @[TensorLoadWideVME.scala 421:29]
  wire  _T_4 = vmeTagDecodeLastValidNext & _T_3; // @[TensorLoadWideVME.scala 420:34]
  wire  _T_5 = ~vmeTagDecodeLastValidNext | _T_4; // @[TensorLoadWideVME.scala 419:34]
  wire [16:0] _GEN_4 = _T_5 ? rdDataElemIdx : {{6'd0}, rdDataElemDestIdxNext}; // @[TensorLoadWideVME.scala 421:59 TensorLoadWideVME.scala 425:25 TensorLoadWideVME.scala 430:25]
  wire [10:0] rdDataElemDestIdx = _GEN_4[10:0]; // @[TensorLoadWideVME.scala 384:31]
  wire [9:0] rdDataClDestIdx = rdDataElemDestIdx[10:1]; // @[TensorLoadWideVME.scala 386:43]
  wire [10:0] _GEN_1 = rdDataElemDestIdx % 11'h2; // @[TensorLoadWideVME.scala 387:48]
  wire [1:0] rdDataDestElemOffset = _GEN_1[1:0]; // @[TensorLoadWideVME.scala 387:48]
  wire  _GEN_0 = _T_1 | vmeTagDecodeLastValidNext; // @[TensorLoadWideVME.scala 395:33 TensorLoadWideVME.scala 396:27 TensorLoadWideVME.scala 398:27]
  wire  isFirstPulse = _T_1 & _T_5; // @[TensorLoadWideVME.scala 417:27 TensorLoadWideVME.scala 416:16]
  wire  _wmaskSel_T = isFirstPulse & io_vmeData_bits_last; // @[TensorLoadWideVME.scala 405:20]
  wire [1:0] _wmaskSel_T_1 = wrMask1st & wrMaskLast; // @[TensorLoadWideVME.scala 406:17]
  wire [1:0] _wmaskSel_T_2 = io_vmeData_bits_last ? wrMaskLast : 2'h3; // @[TensorLoadWideVME.scala 410:12]
  wire [1:0] _wmaskSel_T_3 = isFirstPulse ? wrMask1st : _wmaskSel_T_2; // @[TensorLoadWideVME.scala 407:10]
  wire [1:0] wmaskSel = _wmaskSel_T ? _wmaskSel_T_1 : _wmaskSel_T_3; // @[TensorLoadWideVME.scala 404:8]
  wire [1:0] wmask = _T_1 ? wmaskSel : 2'h0; // @[TensorLoadWideVME.scala 414:18]
  wire [1:0] _rdDataElemDestIdxNext_T_2 = wmask[0] + wmask[1]; // @[Bitwise.scala 47:55]
  wire [16:0] _GEN_10 = {{15'd0}, _rdDataElemDestIdxNext_T_2}; // @[TensorLoadWideVME.scala 427:46]
  wire [16:0] _rdDataElemDestIdxNext_T_5 = rdDataElemIdx + _GEN_10; // @[TensorLoadWideVME.scala 427:46]
  wire [10:0] _GEN_11 = {{9'd0}, _rdDataElemDestIdxNext_T_2}; // @[TensorLoadWideVME.scala 429:54]
  wire [10:0] _rdDataElemDestIdxNext_T_11 = rdDataElemDestIdxNext + _GEN_11; // @[TensorLoadWideVME.scala 429:54]
  wire [16:0] _GEN_5 = _T_5 ? _rdDataElemDestIdxNext_T_5 : {{6'd0}, _rdDataElemDestIdxNext_T_11}; // @[TensorLoadWideVME.scala 421:59 TensorLoadWideVME.scala 427:29 TensorLoadWideVME.scala 429:29]
  wire [16:0] _GEN_9 = _T_1 ? _GEN_5 : {{6'd0}, rdDataElemDestIdxNext}; // @[TensorLoadWideVME.scala 417:27 TensorLoadWideVME.scala 385:34]
  wire [31:0] srcData_0 = io_vmeData_bits_data[31:0]; // @[TensorLoadWideVME.scala 435:47]
  wire [31:0] srcData_1 = io_vmeData_bits_data[63:32]; // @[TensorLoadWideVME.scala 435:47]
  wire [1:0] _srcOffset_0_T = isFirstPulse ? rdFstOffsetNb : 2'h0; // @[TensorLoadWideVME.scala 441:30]
  wire [2:0] _srcOffset_0_T_1 = {{1'd0}, _srcOffset_0_T}; // @[TensorLoadWideVME.scala 441:25]
  wire [1:0] srcOffset_0 = _srcOffset_0_T_1[1:0]; // @[TensorLoadWideVME.scala 441:25]
  wire [1:0] _srcIdx_0_T_1 = srcOffset_0 - rdDataDestElemOffset; // @[TensorLoadWideVME.scala 442:31]
  wire  srcIdx_0 = _srcIdx_0_T_1[0]; // @[TensorLoadWideVME.scala 437:20 TensorLoadWideVME.scala 442:15]
  wire [1:0] srcIdxOH = 2'h1 << srcIdx_0; // @[OneHot.scala 58:35]
  wire [31:0] _io_destData_0_T_2 = srcIdxOH[0] ? srcData_0 : 32'h0; // @[Mux.scala 27:72]
  wire [31:0] _io_destData_0_T_3 = srcIdxOH[1] ? srcData_1 : 32'h0; // @[Mux.scala 27:72]
  wire [1:0] _io_destMask_0_T = srcIdxOH & wmask; // @[Mux.scala 30:47]
  wire  incrIdx = srcOffset_0 >= rdDataDestElemOffset ? 1'h0 : 1'h1; // @[TensorLoadWideVME.scala 451:10]
  wire [9:0] _GEN_12 = {{9'd0}, incrIdx}; // @[TensorLoadWideVME.scala 453:38]
  wire [9:0] _io_destIdx_0_T_1 = rdDataClDestIdx + _GEN_12; // @[TensorLoadWideVME.scala 453:38]
  wire [1:0] srcOffset_1 = 2'h1 + _srcOffset_0_T; // @[TensorLoadWideVME.scala 441:25]
  wire [1:0] _srcIdx_1_T_1 = srcOffset_1 - rdDataDestElemOffset; // @[TensorLoadWideVME.scala 442:31]
  wire  srcIdx_1 = _srcIdx_1_T_1[0]; // @[TensorLoadWideVME.scala 437:20 TensorLoadWideVME.scala 442:15]
  wire [1:0] srcIdxOH_1 = 2'h1 << srcIdx_1; // @[OneHot.scala 58:35]
  wire [31:0] _io_destData_1_T_2 = srcIdxOH_1[0] ? srcData_0 : 32'h0; // @[Mux.scala 27:72]
  wire [31:0] _io_destData_1_T_3 = srcIdxOH_1[1] ? srcData_1 : 32'h0; // @[Mux.scala 27:72]
  wire [1:0] _io_destMask_1_T = srcIdxOH_1 & wmask; // @[Mux.scala 30:47]
  wire  incrIdx_1 = srcOffset_1 >= rdDataDestElemOffset ? 1'h0 : 1'h1; // @[TensorLoadWideVME.scala 451:10]
  wire [9:0] _GEN_13 = {{9'd0}, incrIdx_1}; // @[TensorLoadWideVME.scala 453:38]
  wire [9:0] _io_destIdx_1_T_1 = rdDataClDestIdx + _GEN_13; // @[TensorLoadWideVME.scala 453:38]
  assign io_vmeData_ready = 1'h1; // @[TensorLoadWideVME.scala 344:20]
  assign io_destIdx_0 = {{1'd0}, _io_destIdx_0_T_1}; // @[TensorLoadWideVME.scala 453:38]
  assign io_destIdx_1 = {{1'd0}, _io_destIdx_1_T_1}; // @[TensorLoadWideVME.scala 453:38]
  assign io_destData_0 = _io_destData_0_T_2 | _io_destData_0_T_3; // @[Mux.scala 27:72]
  assign io_destData_1 = _io_destData_1_T_2 | _io_destData_1_T_3; // @[Mux.scala 27:72]
  assign io_destMask_0 = |_io_destMask_0_T; // @[Mux.scala 30:53]
  assign io_destMask_1 = |_io_destMask_1_T; // @[Mux.scala 30:53]
  always @(posedge clock) begin
    if (_T_1) begin // @[TensorLoadWideVME.scala 417:27]
      if (_T_5) begin // @[TensorLoadWideVME.scala 421:59]
        vmeTagDecodeLast <= io_vmeData_bits_tag; // @[TensorLoadWideVME.scala 423:24]
      end
    end
    rdDataElemDestIdxNext <= _GEN_9[10:0];
    if (reset) begin // @[TensorLoadWideVME.scala 390:42]
      vmeTagDecodeLastValidNext <= 1'h0; // @[TensorLoadWideVME.scala 390:42]
    end else if (io_start) begin // @[TensorLoadWideVME.scala 393:18]
      vmeTagDecodeLastValidNext <= 1'h0; // @[TensorLoadWideVME.scala 394:27]
    end else begin
      vmeTagDecodeLastValidNext <= _GEN_0;
    end
    `ifndef SYNTHESIS
    `ifdef PRINTF_COND
      if (`PRINTF_COND) begin
    `endif
        if (~(~io_vmeData_valid | rdLstNb > 2'h0 | reset)) begin
          $fwrite(32'h80000002,
            "Assertion failed: -F- Expecting some elements to read\n    at TensorLoadWideVME.scala:367 assert(!io.vmeData.valid || readNb > 0.U,\"-F- Expecting some elements to read\")\n"
            ); // @[TensorLoadWideVME.scala 367:11]
        end
    `ifdef PRINTF_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef STOP_COND
      if (`STOP_COND) begin
    `endif
        if (~(~io_vmeData_valid | rdLstNb > 2'h0 | reset)) begin
          $fatal; // @[TensorLoadWideVME.scala 367:11]
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
  vmeTagDecodeLast = _RAND_0[20:0];
  _RAND_1 = {1{`RANDOM}};
  rdDataElemDestIdxNext = _RAND_1[10:0];
  _RAND_2 = {1{`RANDOM}};
  vmeTagDecodeLastValidNext = _RAND_2[0:0];
`endif // RANDOMIZE_REG_INIT
  `endif // RANDOMIZE
end // initial
`ifdef FIRRTL_AFTER_INITIAL
`FIRRTL_AFTER_INITIAL
`endif
`endif // SYNTHESIS
endmodule