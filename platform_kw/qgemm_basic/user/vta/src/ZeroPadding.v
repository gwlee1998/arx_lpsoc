module ZeroPadding(
  input          clock,
  input          reset,
  input          io_canWriteMem,
  input  [127:0] io_inst,
  output         io_tensorIdx_valid,
  output [10:0]  io_tensorIdx_bits,
  input          io_start,
  output         io_done
);
`ifdef RANDOMIZE_REG_INIT
  reg [31:0] _RAND_0;
  reg [31:0] _RAND_1;
  reg [31:0] _RAND_2;
  reg [31:0] _RAND_3;
`endif // RANDOMIZE_REG_INIT
  wire [15:0] dec_sram_offset = io_inst[25:10]; // @[TensorLoadNarrowVME.scala 329:29]
  wire [15:0] dec_ysize = io_inst[79:64]; // @[TensorLoadNarrowVME.scala 329:29]
  wire [15:0] dec_xsize = io_inst[95:80]; // @[TensorLoadNarrowVME.scala 329:29]
  wire [3:0] dec_ypad_0 = io_inst[115:112]; // @[TensorLoadNarrowVME.scala 329:29]
  wire [3:0] dec_ypad_1 = io_inst[119:116]; // @[TensorLoadNarrowVME.scala 329:29]
  wire [3:0] dec_xpad_0 = io_inst[123:120]; // @[TensorLoadNarrowVME.scala 329:29]
  wire [3:0] dec_xpad_1 = io_inst[127:124]; // @[TensorLoadNarrowVME.scala 329:29]
  reg [2:0] zpState; // @[TensorLoadNarrowVME.scala 335:24]
  reg [23:0] zpColIdx; // @[TensorLoadNarrowVME.scala 337:21]
  reg [23:0] zpRowIdx; // @[TensorLoadNarrowVME.scala 340:21]
  reg [15:0] zpDestRowOffset; // @[TensorLoadNarrowVME.scala 342:28]
  wire [15:0] _GEN_35 = {{12'd0}, dec_ypad_0}; // @[TensorLoadNarrowVME.scala 349:47]
  wire [15:0] _zpLastDataRow_T_1 = _GEN_35 + dec_ysize; // @[TensorLoadNarrowVME.scala 349:47]
  wire [15:0] _zpLastDataRow_T_3 = _zpLastDataRow_T_1 - 16'h1; // @[TensorLoadNarrowVME.scala 349:59]
  wire [23:0] _GEN_36 = {{8'd0}, _zpLastDataRow_T_3}; // @[TensorLoadNarrowVME.scala 349:32]
  wire  zpLastDataRow = zpRowIdx == _GEN_36; // @[TensorLoadNarrowVME.scala 349:32]
  wire [15:0] _GEN_37 = {{12'd0}, dec_xpad_0}; // @[TensorLoadNarrowVME.scala 350:33]
  wire [15:0] _zpTopLastIdx_T_1 = _GEN_37 + dec_xsize; // @[TensorLoadNarrowVME.scala 350:33]
  wire [15:0] _GEN_38 = {{12'd0}, dec_xpad_1}; // @[TensorLoadNarrowVME.scala 350:45]
  wire [15:0] _zpTopLastIdx_T_3 = _zpTopLastIdx_T_1 + _GEN_38; // @[TensorLoadNarrowVME.scala 350:45]
  wire [15:0] zpTopLastIdx = _zpTopLastIdx_T_3 - 16'h1; // @[TensorLoadNarrowVME.scala 350:58]
  wire  _zpWideLineEnd_T = zpState == 3'h4; // @[TensorLoadNarrowVME.scala 351:32]
  wire  _zpWideLineEnd_T_1 = zpState == 3'h3; // @[TensorLoadNarrowVME.scala 351:59]
  wire [23:0] _GEN_39 = {{8'd0}, zpTopLastIdx}; // @[TensorLoadNarrowVME.scala 351:89]
  wire  _zpWideLineEnd_T_3 = zpColIdx == _GEN_39; // @[TensorLoadNarrowVME.scala 351:89]
  wire  zpWideLineEnd = (zpState == 3'h4 | zpState == 3'h3) & zpColIdx == _GEN_39; // @[TensorLoadNarrowVME.scala 351:77]
  wire [3:0] _zpNarwLineEnd_T_2 = dec_xpad_0 - 4'h1; // @[TensorLoadNarrowVME.scala 352:74]
  wire [23:0] _GEN_40 = {{20'd0}, _zpNarwLineEnd_T_2}; // @[TensorLoadNarrowVME.scala 352:59]
  wire  _zpNarwLineEnd_T_3 = zpColIdx == _GEN_40; // @[TensorLoadNarrowVME.scala 352:59]
  wire  zpNarwLineEnd = zpState == 3'h2 & zpColIdx == _GEN_40; // @[TensorLoadNarrowVME.scala 352:47]
  wire  zpFillLineEnd = zpWideLineEnd | zpNarwLineEnd; // @[TensorLoadNarrowVME.scala 353:37]
  wire  _T_1 = dec_xpad_1 != 4'h0; // @[TensorLoadNarrowVME.scala 360:43]
  wire  _T_2 = dec_xpad_0 == 4'h0 & dec_xpad_1 != 4'h0; // @[TensorLoadNarrowVME.scala 360:29]
  wire [15:0] _GEN_0 = dec_xpad_0 == 4'h0 & dec_xpad_1 != 4'h0 & dec_ypad_0 == 4'h0 ? _zpTopLastIdx_T_1 : 16'h0; // @[TensorLoadNarrowVME.scala 360:74 TensorLoadNarrowVME.scala 361:16 TensorLoadNarrowVME.scala 359:14]
  wire  _T_6 = dec_xpad_0 != 4'h0; // @[TensorLoadNarrowVME.scala 365:27]
  wire  _T_15 = dec_ypad_1 != 4'h0; // @[TensorLoadNarrowVME.scala 371:27]
  wire [2:0] _GEN_1 = dec_ypad_1 != 4'h0 ? 3'h5 : 3'h0; // @[TensorLoadNarrowVME.scala 371:36 TensorLoadNarrowVME.scala 372:15 TensorLoadNarrowVME.scala 374:15]
  wire [2:0] _GEN_2 = _T_6 & _T_1 ? 3'h4 : _GEN_1; // @[TensorLoadNarrowVME.scala 369:58 TensorLoadNarrowVME.scala 370:15]
  wire [2:0] _GEN_3 = _T_2 ? 3'h3 : _GEN_2; // @[TensorLoadNarrowVME.scala 367:58 TensorLoadNarrowVME.scala 368:15]
  wire [2:0] _GEN_4 = dec_xpad_0 != 4'h0 & dec_xpad_1 == 4'h0 ? 3'h2 : _GEN_3; // @[TensorLoadNarrowVME.scala 365:58 TensorLoadNarrowVME.scala 366:15]
  wire  _T_16 = zpState == 3'h1; // @[TensorLoadNarrowVME.scala 378:14]
  wire  _T_17 = io_canWriteMem & _T_16; // @[TensorLoadNarrowVME.scala 377:20]
  wire [3:0] _T_19 = dec_ypad_0 - 4'h1; // @[TensorLoadNarrowVME.scala 379:29]
  wire [23:0] _GEN_42 = {{20'd0}, _T_19}; // @[TensorLoadNarrowVME.scala 379:14]
  wire  _T_20 = zpRowIdx == _GEN_42; // @[TensorLoadNarrowVME.scala 379:14]
  wire  _T_21 = _T_17 & _T_20; // @[TensorLoadNarrowVME.scala 378:25]
  wire  _T_23 = _T_21 & _zpWideLineEnd_T_3; // @[TensorLoadNarrowVME.scala 379:35]
  wire [15:0] _GEN_6 = _T_2 ? _zpTopLastIdx_T_1 : 16'h0; // @[TensorLoadNarrowVME.scala 384:52 TensorLoadNarrowVME.scala 385:16 TensorLoadNarrowVME.scala 383:14]
  wire  _T_38 = zpState == 3'h5; // @[TensorLoadNarrowVME.scala 401:15]
  wire  _T_39 = zpFillLineEnd & io_canWriteMem | _T_38; // @[TensorLoadNarrowVME.scala 400:40]
  wire  _T_40 = zpLastDataRow & _T_39; // @[TensorLoadNarrowVME.scala 399:19]
  wire [23:0] _GEN_11 = _T_15 ? 24'h0 : zpColIdx; // @[TensorLoadNarrowVME.scala 405:30 TensorLoadNarrowVME.scala 406:16 TensorLoadNarrowVME.scala 344:12]
  wire [2:0] _GEN_12 = _T_15 ? 3'h6 : 3'h0; // @[TensorLoadNarrowVME.scala 405:30 TensorLoadNarrowVME.scala 407:15 TensorLoadNarrowVME.scala 409:15]
  wire  _T_42 = zpState == 3'h6; // @[TensorLoadNarrowVME.scala 413:14]
  wire  _T_43 = io_canWriteMem & _T_42; // @[TensorLoadNarrowVME.scala 412:20]
  wire [15:0] _GEN_46 = {{12'd0}, dec_ypad_1}; // @[TensorLoadNarrowVME.scala 414:41]
  wire [15:0] _T_47 = _zpLastDataRow_T_1 + _GEN_46; // @[TensorLoadNarrowVME.scala 414:41]
  wire [15:0] _T_49 = _T_47 - 16'h1; // @[TensorLoadNarrowVME.scala 414:54]
  wire [23:0] _GEN_47 = {{8'd0}, _T_49}; // @[TensorLoadNarrowVME.scala 414:14]
  wire  _T_50 = zpRowIdx == _GEN_47; // @[TensorLoadNarrowVME.scala 414:14]
  wire  _T_51 = _T_43 & _T_50; // @[TensorLoadNarrowVME.scala 413:25]
  wire  _T_53 = _T_51 & _zpWideLineEnd_T_3; // @[TensorLoadNarrowVME.scala 414:60]
  wire [23:0] _GEN_14 = _T_53 ? 24'h0 : zpColIdx; // @[TensorLoadNarrowVME.scala 415:32 TensorLoadNarrowVME.scala 418:14 TensorLoadNarrowVME.scala 344:12]
  wire [2:0] _GEN_15 = _T_53 ? 3'h0 : zpState; // @[TensorLoadNarrowVME.scala 415:32 TensorLoadNarrowVME.scala 419:13 TensorLoadNarrowVME.scala 421:13]
  wire  _GEN_16 = _T_40 | _T_53; // @[TensorLoadNarrowVME.scala 401:63 TensorLoadNarrowVME.scala 403:20]
  wire [23:0] _GEN_17 = _T_40 ? _GEN_11 : _GEN_14; // @[TensorLoadNarrowVME.scala 401:63]
  wire  _GEN_19 = _T_23 | _GEN_16; // @[TensorLoadNarrowVME.scala 380:32 TensorLoadNarrowVME.scala 381:20]
  wire [23:0] _GEN_20 = _T_23 ? {{8'd0}, _GEN_6} : _GEN_17; // @[TensorLoadNarrowVME.scala 380:32]
  wire [23:0] _GEN_24 = io_start ? {{8'd0}, _GEN_0} : _GEN_20; // @[TensorLoadNarrowVME.scala 355:18]
  wire  zpNewFillBlock = io_start ? 1'h0 : _GEN_19; // @[TensorLoadNarrowVME.scala 355:18 TensorLoadNarrowVME.scala 346:18]
  wire  isZeroPadWrite = zpState != 3'h0 & zpState != 3'h5 & io_canWriteMem; // @[TensorLoadNarrowVME.scala 424:68]
  wire [23:0] _GEN_49 = {{8'd0}, zpDestRowOffset}; // @[TensorLoadNarrowVME.scala 425:32]
  wire [23:0] _zpDestIdx_T_1 = _GEN_49 + zpColIdx; // @[TensorLoadNarrowVME.scala 425:32]
  wire  _T_62 = (_T_16 | _zpWideLineEnd_T | _zpWideLineEnd_T_1 | _T_42) & _zpWideLineEnd_T_3; // @[TensorLoadNarrowVME.scala 432:106]
  wire  _T_68 = _T_62 | zpNarwLineEnd; // @[TensorLoadNarrowVME.scala 433:34]
  wire  incrementRow = _T_68 & io_canWriteMem | _T_38; // @[TensorLoadNarrowVME.scala 434:84]
  wire [15:0] _zpDestRowOffset_T_1 = zpDestRowOffset + zpTopLastIdx; // @[TensorLoadNarrowVME.scala 437:40]
  wire [15:0] _zpDestRowOffset_T_3 = _zpDestRowOffset_T_1 + 16'h1; // @[TensorLoadNarrowVME.scala 437:55]
  wire [23:0] _zpRowIdx_T_1 = zpRowIdx + 24'h1; // @[TensorLoadNarrowVME.scala 438:26]
  wire  _T_72 = ~zpNewFillBlock; // @[TensorLoadNarrowVME.scala 440:10]
  wire [15:0] _GEN_27 = _zpWideLineEnd_T_1 ? _zpTopLastIdx_T_1 : 16'h0; // @[TensorLoadNarrowVME.scala 441:38 TensorLoadNarrowVME.scala 442:18 TensorLoadNarrowVME.scala 444:18]
  wire [23:0] _GEN_54 = {{8'd0}, dec_xsize}; // @[TensorLoadNarrowVME.scala 452:28]
  wire [23:0] _zpColIdx_T_7 = zpColIdx + _GEN_54; // @[TensorLoadNarrowVME.scala 452:28]
  wire [23:0] _zpColIdx_T_9 = _zpColIdx_T_7 + 24'h1; // @[TensorLoadNarrowVME.scala 452:40]
  wire [23:0] _zpColIdx_T_11 = zpColIdx + 24'h1; // @[TensorLoadNarrowVME.scala 455:28]
  wire [15:0] zpDestIdx = _zpDestIdx_T_1[15:0]; // @[TensorLoadNarrowVME.scala 332:23 TensorLoadNarrowVME.scala 425:13]
  assign io_tensorIdx_valid = zpState != 3'h0 & zpState != 3'h5 & io_canWriteMem; // @[TensorLoadNarrowVME.scala 424:68]
  assign io_tensorIdx_bits = zpDestIdx[10:0]; // @[TensorLoadNarrowVME.scala 460:21]
  assign io_done = zpState == 3'h0; // @[TensorLoadNarrowVME.scala 458:22]
  always @(posedge clock) begin
    if (reset) begin // @[TensorLoadNarrowVME.scala 335:24]
      zpState <= 3'h0; // @[TensorLoadNarrowVME.scala 335:24]
    end else if (io_start) begin // @[TensorLoadNarrowVME.scala 355:18]
      if (dec_ypad_0 != 4'h0) begin // @[TensorLoadNarrowVME.scala 363:30]
        zpState <= 3'h1; // @[TensorLoadNarrowVME.scala 364:15]
      end else begin
        zpState <= _GEN_4;
      end
    end else if (_T_23) begin // @[TensorLoadNarrowVME.scala 380:32]
      zpState <= _GEN_4;
    end else if (_T_40) begin // @[TensorLoadNarrowVME.scala 401:63]
      zpState <= _GEN_12;
    end else begin
      zpState <= _GEN_15;
    end
    if (isZeroPadWrite & _T_72 & ~incrementRow) begin // @[TensorLoadNarrowVME.scala 450:60]
      if (_zpWideLineEnd_T & _zpNarwLineEnd_T_3) begin // @[TensorLoadNarrowVME.scala 451:68]
        zpColIdx <= _zpColIdx_T_9; // @[TensorLoadNarrowVME.scala 452:16]
      end else begin
        zpColIdx <= _zpColIdx_T_11; // @[TensorLoadNarrowVME.scala 455:16]
      end
    end else if (incrementRow) begin // @[TensorLoadNarrowVME.scala 435:30]
      if (~zpNewFillBlock) begin // @[TensorLoadNarrowVME.scala 440:27]
        zpColIdx <= {{8'd0}, _GEN_27};
      end else begin
        zpColIdx <= _GEN_24;
      end
    end else begin
      zpColIdx <= _GEN_24;
    end
    if (incrementRow) begin // @[TensorLoadNarrowVME.scala 435:30]
      zpRowIdx <= _zpRowIdx_T_1; // @[TensorLoadNarrowVME.scala 438:14]
    end else if (io_start) begin // @[TensorLoadNarrowVME.scala 355:18]
      zpRowIdx <= 24'h0; // @[TensorLoadNarrowVME.scala 356:14]
    end
    if (incrementRow) begin // @[TensorLoadNarrowVME.scala 435:30]
      zpDestRowOffset <= _zpDestRowOffset_T_3; // @[TensorLoadNarrowVME.scala 437:21]
    end else if (io_start) begin // @[TensorLoadNarrowVME.scala 355:18]
      zpDestRowOffset <= dec_sram_offset; // @[TensorLoadNarrowVME.scala 357:21]
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
  zpState = _RAND_0[2:0];
  _RAND_1 = {1{`RANDOM}};
  zpColIdx = _RAND_1[23:0];
  _RAND_2 = {1{`RANDOM}};
  zpRowIdx = _RAND_2[23:0];
  _RAND_3 = {1{`RANDOM}};
  zpDestRowOffset = _RAND_3[15:0];
`endif // RANDOMIZE_REG_INIT
  `endif // RANDOMIZE
end // initial
`ifdef FIRRTL_AFTER_INITIAL
`FIRRTL_AFTER_INITIAL
`endif
`endif // SYNTHESIS
endmodule