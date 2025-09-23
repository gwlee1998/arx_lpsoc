module GenVMECmdWide(
  input         clock,
  input         reset,
  input         io_start,
  input         io_isBusy,
  input         io_updateState,
  input  [31:0] io_baddr,
  output        io_vmeCmd_valid,
  output [31:0] io_vmeCmd_bits_addr,
  output [7:0]  io_vmeCmd_bits_len,
  output [20:0] io_vmeCmd_bits_tag,
  output [8:0]  io_readLen,
  output        io_done,
  input  [15:0] io_ysize,
  input  [15:0] io_xsize,
  input  [15:0] io_xstride,
  input  [31:0] io_dram_offset,
  input  [15:0] io_sram_offset,
  input  [3:0]  io_xpad_0,
  input  [3:0]  io_xpad_1,
  input  [3:0]  io_ypad_0
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
`endif // RANDOMIZE_REG_INIT
  reg [15:0] dramLineIdx; // @[TensorLoadWideVME.scala 501:24]
  wire [15:0] _dramLineIdx_T_1 = dramLineIdx + 16'h1; // @[TensorLoadWideVME.scala 505:32]
  reg [14:0] clReadIdx; // @[TensorLoadWideVME.scala 591:22]
  wire [17:0] rdLineBytes = {io_xsize, 2'h0}; // @[TensorLoadWideVME.scala 552:30]
  reg [31:0] rdLineElemBeginAddr; // @[TensorLoadWideVME.scala 520:32]
  wire [31:0] _GEN_0 = rdLineElemBeginAddr % 32'h8; // @[TensorLoadWideVME.scala 562:51]
  wire [3:0] rd1stPulseOffsetBytes = _GEN_0[3:0]; // @[TensorLoadWideVME.scala 562:51]
  wire [17:0] _GEN_30 = {{14'd0}, rd1stPulseOffsetBytes}; // @[TensorLoadWideVME.scala 570:22]
  wire [17:0] _rdLineClNb_T_1 = rdLineBytes + _GEN_30; // @[TensorLoadWideVME.scala 570:22]
  wire [17:0] _GEN_1 = _rdLineClNb_T_1 % 18'h8; // @[TensorLoadWideVME.scala 570:47]
  wire [3:0] _rdLineClNb_T_2 = _GEN_1[3:0]; // @[TensorLoadWideVME.scala 570:47]
  wire [14:0] rdLineClNbTmp = _rdLineClNb_T_1[17:3]; // @[TensorLoadWideVME.scala 568:61]
  wire [14:0] _rdLineClNb_T_5 = rdLineClNbTmp + 15'h1; // @[TensorLoadWideVME.scala 570:97]
  wire [14:0] rdLineClNb = _rdLineClNb_T_2 == 4'h0 ? rdLineClNbTmp : _rdLineClNb_T_5; // @[TensorLoadWideVME.scala 570:8]
  wire  newReadRow = clReadIdx == 15'h0; // @[TensorLoadWideVME.scala 592:30]
  wire [14:0] clRemained = rdLineClNb - clReadIdx; // @[TensorLoadWideVME.scala 642:31]
  wire [31:0] rdLineClBeginAddr = rdLineElemBeginAddr & 32'hfffffff8; // @[TensorLoadWideVME.scala 521:47]
  wire [31:0] _GEN_2 = rdLineClBeginAddr % 32'h800; // @[TensorLoadWideVME.scala 557:62]
  wire [11:0] _rdLen1stMaxTransBytes_T = _GEN_2[11:0]; // @[TensorLoadWideVME.scala 557:62]
  wire [11:0] rdLen1stMaxTransBytes = 12'h800 - _rdLen1stMaxTransBytes_T; // @[TensorLoadWideVME.scala 557:42]
  wire [8:0] rdLen1stMaxTransClNb = rdLen1stMaxTransBytes[11:3]; // @[TensorLoadWideVME.scala 559:52]
  wire [14:0] _GEN_32 = {{6'd0}, rdLen1stMaxTransClNb}; // @[TensorLoadWideVME.scala 644:21]
  wire [14:0] _GEN_16 = clRemained < _GEN_32 ? clRemained : {{6'd0}, rdLen1stMaxTransClNb}; // @[TensorLoadWideVME.scala 644:45 TensorLoadWideVME.scala 645:13 TensorLoadWideVME.scala 647:13]
  wire [14:0] _GEN_17 = clRemained < 15'h100 ? clRemained : 15'h100; // @[TensorLoadWideVME.scala 650:36 TensorLoadWideVME.scala 651:13 TensorLoadWideVME.scala 653:13]
  wire [14:0] _GEN_18 = newReadRow ? _GEN_16 : _GEN_17; // @[TensorLoadWideVME.scala 643:21]
  wire [8:0] rdLen = _GEN_18[8:0]; // @[TensorLoadWideVME.scala 537:19]
  wire [14:0] _GEN_33 = {{6'd0}, rdLen}; // @[TensorLoadWideVME.scala 635:34]
  wire [14:0] _T_28 = rdLineClNb - _GEN_33; // @[TensorLoadWideVME.scala 635:34]
  wire  _T_29 = clReadIdx == _T_28; // @[TensorLoadWideVME.scala 635:19]
  wire [15:0] _T_31 = io_ysize - 16'h1; // @[TensorLoadWideVME.scala 635:72]
  wire  stride = clReadIdx == _T_28 & dramLineIdx != _T_31 & io_updateState; // @[TensorLoadWideVME.scala 635:79]
  wire [33:0] _dramInitialAddr_T = {io_dram_offset, 2'h0}; // @[TensorLoadWideVME.scala 512:41]
  wire [31:0] dramInitialAddr = _dramInitialAddr_T[31:0]; // @[TensorLoadWideVME.scala 512:73 TensorLoadWideVME.scala 512:73]
  wire [31:0] xferElemInitAddr = io_baddr | dramInitialAddr; // @[TensorLoadWideVME.scala 513:35]
  wire [31:0] xferClInitAddr = xferElemInitAddr & 32'hfffffff8; // @[TensorLoadWideVME.scala 519:41]
  wire [17:0] _nextLineBeginElemAddr_T = {io_xstride, 2'h0}; // @[TensorLoadWideVME.scala 523:65]
  wire [31:0] _GEN_34 = {{14'd0}, _nextLineBeginElemAddr_T}; // @[TensorLoadWideVME.scala 523:51]
  wire [31:0] nextLineBeginElemAddr = rdLineElemBeginAddr + _GEN_34; // @[TensorLoadWideVME.scala 523:51]
  wire [31:0] nextLineBeginClAddr = nextLineBeginElemAddr & 32'hfffffff8; // @[TensorLoadWideVME.scala 524:51]
  reg [31:0] rdLineAddr; // @[TensorLoadWideVME.scala 538:23]
  wire [11:0] _rdLineAddr_T = {rdLen, 3'h0}; // @[TensorLoadWideVME.scala 545:41]
  wire [31:0] _GEN_35 = {{20'd0}, _rdLineAddr_T}; // @[TensorLoadWideVME.scala 545:32]
  wire [31:0] _rdLineAddr_T_2 = rdLineAddr + _GEN_35; // @[TensorLoadWideVME.scala 545:32]
  wire [31:0] _GEN_36 = {{14'd0}, rdLineBytes}; // @[TensorLoadWideVME.scala 573:48]
  wire [31:0] _rdLastPulseBytes_T_1 = rdLineElemBeginAddr + _GEN_36; // @[TensorLoadWideVME.scala 573:48]
  wire [31:0] _GEN_3 = _rdLastPulseBytes_T_1 % 32'h8; // @[TensorLoadWideVME.scala 573:63]
  wire [3:0] rdLastPulseBytes = _GEN_3[3:0]; // @[TensorLoadWideVME.scala 573:63]
  wire [1:0] rdLastPulseTensNb = rdLastPulseBytes[3:2] == 2'h0 ? 2'h2 : rdLastPulseBytes[3:2]; // @[TensorLoadWideVME.scala 578:28]
  reg [10:0] rdCmdStartIdx; // @[TensorLoadWideVME.scala 588:26]
  reg  commandsDone; // @[TensorLoadWideVME.scala 589:29]
  wire [14:0] nextClIdx = clReadIdx + _GEN_33; // @[TensorLoadWideVME.scala 600:31]
  wire  _GEN_7 = nextClIdx == rdLineClNb & dramLineIdx == _T_31 | commandsDone; // @[TensorLoadWideVME.scala 602:71 TensorLoadWideVME.scala 603:20 TensorLoadWideVME.scala 595:16]
  wire  _GEN_9 = io_updateState ? _GEN_7 : commandsDone; // @[TensorLoadWideVME.scala 599:31 TensorLoadWideVME.scala 595:16]
  wire  _GEN_11 = io_start | stride ? 1'h0 : _GEN_9; // @[TensorLoadWideVME.scala 596:29 TensorLoadWideVME.scala 598:18]
  wire [1:0] rdCmd1stPluseOffsetTensNb = newReadRow ? rd1stPulseOffsetBytes[3:2] : 2'h0; // @[TensorLoadWideVME.scala 619:20 TensorLoadWideVME.scala 621:31 TensorLoadWideVME.scala 624:31]
  wire [1:0] rdCmdLastPluseTensNb = _T_29 ? rdLastPulseTensNb : 2'h2; // @[TensorLoadWideVME.scala 626:43 TensorLoadWideVME.scala 628:26 TensorLoadWideVME.scala 631:26]
  wire [15:0] _GEN_40 = {{12'd0}, io_xpad_0}; // @[TensorLoadWideVME.scala 657:29]
  wire [15:0] _totalWidth_T_1 = io_xsize + _GEN_40; // @[TensorLoadWideVME.scala 657:29]
  wire [15:0] _GEN_41 = {{12'd0}, io_xpad_1}; // @[TensorLoadWideVME.scala 657:41]
  wire [15:0] totalWidth = _totalWidth_T_1 + _GEN_41; // @[TensorLoadWideVME.scala 657:41]
  reg [19:0] currentRowIdx; // @[TensorLoadWideVME.scala 661:26]
  wire [19:0] _GEN_42 = {{16'd0}, io_ypad_0}; // @[TensorLoadWideVME.scala 663:39]
  wire [15:0] _GEN_43 = {{12'd0}, io_ypad_0}; // @[TensorLoadWideVME.scala 664:31]
  wire [15:0] _rdCmdStartIdxValid_T_2 = io_ysize + _GEN_43; // @[TensorLoadWideVME.scala 664:31]
  wire [19:0] _GEN_44 = {{4'd0}, _rdCmdStartIdxValid_T_2}; // @[TensorLoadWideVME.scala 664:19]
  wire  _rdCmdStartIdxValid_T_3 = currentRowIdx < _GEN_44; // @[TensorLoadWideVME.scala 664:19]
  wire  _rdCmdStartIdxValid_T_4 = currentRowIdx >= _GEN_42 & _rdCmdStartIdxValid_T_3; // @[TensorLoadWideVME.scala 663:52]
  wire  _rdCmdStartIdxValid_T_5 = _rdCmdStartIdxValid_T_4 & io_isBusy; // @[TensorLoadWideVME.scala 664:44]
  wire  _rdCmdStartIdxValid_T_6 = ~commandsDone; // @[TensorLoadWideVME.scala 666:5]
  wire  rdCmdStartIdxValid = _rdCmdStartIdxValid_T_5 & _rdCmdStartIdxValid_T_6; // @[TensorLoadWideVME.scala 665:15]
  wire [15:0] _rdCmdStartIdx_T_1 = io_sram_offset + _GEN_40; // @[TensorLoadWideVME.scala 669:37]
  wire [15:0] _GEN_47 = {{5'd0}, rdCmdStartIdx}; // @[TensorLoadWideVME.scala 671:36]
  wire [15:0] _rdCmdStartIdx_T_3 = _GEN_47 + totalWidth; // @[TensorLoadWideVME.scala 671:36]
  wire [19:0] _currentRowIdx_T_1 = currentRowIdx + 20'h1; // @[TensorLoadWideVME.scala 672:36]
  wire [15:0] _GEN_19 = io_isBusy & (currentRowIdx < _GEN_42 | stride) ? _rdCmdStartIdx_T_3 : {{5'd0}, rdCmdStartIdx}; // @[TensorLoadWideVME.scala 670:67 TensorLoadWideVME.scala 671:19 TensorLoadWideVME.scala 588:26]
  wire [15:0] _GEN_22 = io_start ? _rdCmdStartIdx_T_1 : _GEN_19; // @[TensorLoadWideVME.scala 667:19 TensorLoadWideVME.scala 669:19]
  wire  startIssueCmdRead = newReadRow & rdCmdStartIdxValid; // @[TensorLoadWideVME.scala 675:19]
  reg [10:0] rdCmdDestElemIdxNext; // @[TensorLoadWideVME.scala 688:33]
  wire [9:0] _rdCmdTransactionTensNb_T = {rdLen, 1'h0}; // @[TensorLoadWideVME.scala 694:39]
  wire [9:0] _GEN_48 = {{8'd0}, rdCmd1stPluseOffsetTensNb}; // @[TensorLoadWideVME.scala 694:71]
  wire [9:0] rdCmdTransactionTensNb = _rdCmdTransactionTensNb_T - _GEN_48; // @[TensorLoadWideVME.scala 694:71]
  wire [10:0] _GEN_49 = {{1'd0}, rdCmdTransactionTensNb}; // @[TensorLoadWideVME.scala 700:44]
  wire [10:0] _rdCmdDestElemIdxNext_T_1 = rdCmdStartIdx + _GEN_49; // @[TensorLoadWideVME.scala 700:44]
  wire [10:0] _rdCmdDestElemIdxNext_T_3 = rdCmdDestElemIdxNext + _GEN_49; // @[TensorLoadWideVME.scala 703:51]
  wire [10:0] _GEN_25 = startIssueCmdRead ? rdCmdStartIdx : rdCmdDestElemIdxNext; // @[TensorLoadWideVME.scala 698:29 TensorLoadWideVME.scala 699:24 TensorLoadWideVME.scala 690:20]
  wire [10:0] rdCmdDestElemIdx = rdCmdStartIdxValid ? _GEN_25 : rdCmdDestElemIdxNext; // @[TensorLoadWideVME.scala 696:28 TensorLoadWideVME.scala 690:20]
  wire [8:0] _io_vmeCmd_bits_len_T_1 = rdLen - 9'h1; // @[TensorLoadWideVME.scala 716:31]
  wire [31:0] _GEN_4 = rdLineAddr % 32'h800; // @[TensorLoadWideVME.scala 717:87]
  wire [11:0] _T_49 = _GEN_4[11:0]; // @[TensorLoadWideVME.scala 717:87]
  wire [11:0] _T_51 = 12'h800 - _T_49; // @[TensorLoadWideVME.scala 717:74]
  wire [14:0] _io_vmeCmd_bits_tag_T = {rdCmdDestElemIdx,rdCmd1stPluseOffsetTensNb,rdCmdLastPluseTensNb}; // @[Cat.scala 30:58]
  assign io_vmeCmd_valid = _rdCmdStartIdxValid_T_5 & _rdCmdStartIdxValid_T_6; // @[TensorLoadWideVME.scala 665:15]
  assign io_vmeCmd_bits_addr = rdLineAddr; // @[TensorLoadWideVME.scala 715:23]
  assign io_vmeCmd_bits_len = _io_vmeCmd_bits_len_T_1[7:0]; // @[TensorLoadWideVME.scala 716:22]
  assign io_vmeCmd_bits_tag = {{6'd0}, _io_vmeCmd_bits_tag_T}; // @[Cat.scala 30:58]
  assign io_readLen = _GEN_18[8:0]; // @[TensorLoadWideVME.scala 537:19]
  assign io_done = commandsDone; // @[TensorLoadWideVME.scala 726:11]
  always @(posedge clock) begin
    if (io_start) begin // @[TensorLoadWideVME.scala 502:19]
      dramLineIdx <= 16'h0; // @[TensorLoadWideVME.scala 503:17]
    end else if (stride) begin // @[TensorLoadWideVME.scala 504:23]
      dramLineIdx <= _dramLineIdx_T_1; // @[TensorLoadWideVME.scala 505:17]
    end
    if (io_start | stride) begin // @[TensorLoadWideVME.scala 596:29]
      clReadIdx <= 15'h0; // @[TensorLoadWideVME.scala 597:15]
    end else if (io_updateState) begin // @[TensorLoadWideVME.scala 599:31]
      clReadIdx <= nextClIdx; // @[TensorLoadWideVME.scala 601:15]
    end
    if (io_start) begin // @[TensorLoadWideVME.scala 525:19]
      rdLineElemBeginAddr <= xferElemInitAddr; // @[TensorLoadWideVME.scala 526:25]
    end else if (stride) begin // @[TensorLoadWideVME.scala 527:23]
      rdLineElemBeginAddr <= nextLineBeginElemAddr; // @[TensorLoadWideVME.scala 528:25]
    end
    if (io_start) begin // @[TensorLoadWideVME.scala 539:19]
      rdLineAddr <= xferClInitAddr; // @[TensorLoadWideVME.scala 540:16]
    end else if (io_updateState) begin // @[TensorLoadWideVME.scala 541:31]
      if (stride) begin // @[TensorLoadWideVME.scala 542:18]
        rdLineAddr <= nextLineBeginClAddr; // @[TensorLoadWideVME.scala 543:18]
      end else begin
        rdLineAddr <= _rdLineAddr_T_2; // @[TensorLoadWideVME.scala 545:18]
      end
    end
    rdCmdStartIdx <= _GEN_22[10:0];
    commandsDone <= reset | _GEN_11; // @[TensorLoadWideVME.scala 589:29 TensorLoadWideVME.scala 589:29]
    if (io_start) begin // @[TensorLoadWideVME.scala 667:19]
      currentRowIdx <= 20'h0; // @[TensorLoadWideVME.scala 668:19]
    end else if (io_isBusy & (currentRowIdx < _GEN_42 | stride)) begin // @[TensorLoadWideVME.scala 670:67]
      currentRowIdx <= _currentRowIdx_T_1; // @[TensorLoadWideVME.scala 672:19]
    end
    if (rdCmdStartIdxValid) begin // @[TensorLoadWideVME.scala 696:28]
      if (startIssueCmdRead) begin // @[TensorLoadWideVME.scala 698:29]
        rdCmdDestElemIdxNext <= _rdCmdDestElemIdxNext_T_1; // @[TensorLoadWideVME.scala 700:27]
      end else if (io_updateState) begin // @[TensorLoadWideVME.scala 701:33]
        rdCmdDestElemIdxNext <= _rdCmdDestElemIdxNext_T_3; // @[TensorLoadWideVME.scala 703:27]
      end
    end
    `ifndef SYNTHESIS
    `ifdef PRINTF_COND
      if (`PRINTF_COND) begin
    `endif
        if (~(rd1stPulseOffsetBytes[3:2] <= 2'h2 | reset)) begin
          $fwrite(32'h80000002,
            "Assertion failed: -F- Expecting the number of tensors to skip in CL\n    at TensorLoadWideVME.scala:563 assert(rd1stPulseOffsetBytes >> log2Ceil(elemBytes) <= tp.clSizeRatio.U,\n"
            ); // @[TensorLoadWideVME.scala 563:9]
        end
    `ifdef PRINTF_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef STOP_COND
      if (`STOP_COND) begin
    `endif
        if (~(rd1stPulseOffsetBytes[3:2] <= 2'h2 | reset)) begin
          $fatal; // @[TensorLoadWideVME.scala 563:9]
        end
    `ifdef STOP_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef PRINTF_COND
      if (`PRINTF_COND) begin
    `endif
        if (~(rdLastPulseBytes[3:2] <= 2'h2 | reset)) begin
          $fwrite(32'h80000002,
            "Assertion failed: -F- Expecting the number of active tensors in CL\n    at TensorLoadWideVME.scala:574 assert(rdLastPulseBytes >> log2Ceil(elemBytes) <= (clBytes/elemBytes).U,\n"
            ); // @[TensorLoadWideVME.scala 574:9]
        end
    `ifdef PRINTF_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef STOP_COND
      if (`STOP_COND) begin
    `endif
        if (~(rdLastPulseBytes[3:2] <= 2'h2 | reset)) begin
          $fatal; // @[TensorLoadWideVME.scala 574:9]
        end
    `ifdef STOP_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef PRINTF_COND
      if (`PRINTF_COND) begin
    `endif
        if (~(~io_isBusy | rdLineClNb >= clReadIdx | reset)) begin
          $fwrite(32'h80000002,
            "Assertion failed\n    at TensorLoadWideVME.scala:641 assert(!io.isBusy || rdLineClNb >= clReadIdx)// define how many cachelines to read at this cycle\n"
            ); // @[TensorLoadWideVME.scala 641:9]
        end
    `ifdef PRINTF_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef STOP_COND
      if (`STOP_COND) begin
    `endif
        if (~(~io_isBusy | rdLineClNb >= clReadIdx | reset)) begin
          $fatal; // @[TensorLoadWideVME.scala 641:9]
        end
    `ifdef STOP_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef PRINTF_COND
      if (`PRINTF_COND) begin
    `endif
        if (~(~io_vmeCmd_valid | _rdLineAddr_T <= _T_51 | reset)) begin
          $fwrite(32'h80000002,
            "Assertion failed: -F- uop DRAM page alignment failure. DRAM address + len overlaps mp.lenBits*memBlockSize alignment %x %x\n    at TensorLoadWideVME.scala:717 assert(!io.vmeCmd.valid || ((rdLen << log2Ceil(clBytes)) <= maxTrBytes - rdLineAddr %% maxTrBytes),\n"
            ,rdLineAddr,rdLen); // @[TensorLoadWideVME.scala 717:9]
        end
    `ifdef PRINTF_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef STOP_COND
      if (`STOP_COND) begin
    `endif
        if (~(~io_vmeCmd_valid | _rdLineAddr_T <= _T_51 | reset)) begin
          $fatal; // @[TensorLoadWideVME.scala 717:9]
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
  dramLineIdx = _RAND_0[15:0];
  _RAND_1 = {1{`RANDOM}};
  clReadIdx = _RAND_1[14:0];
  _RAND_2 = {1{`RANDOM}};
  rdLineElemBeginAddr = _RAND_2[31:0];
  _RAND_3 = {1{`RANDOM}};
  rdLineAddr = _RAND_3[31:0];
  _RAND_4 = {1{`RANDOM}};
  rdCmdStartIdx = _RAND_4[10:0];
  _RAND_5 = {1{`RANDOM}};
  commandsDone = _RAND_5[0:0];
  _RAND_6 = {1{`RANDOM}};
  currentRowIdx = _RAND_6[19:0];
  _RAND_7 = {1{`RANDOM}};
  rdCmdDestElemIdxNext = _RAND_7[10:0];
`endif // RANDOMIZE_REG_INIT
  `endif // RANDOMIZE
end // initial
`ifdef FIRRTL_AFTER_INITIAL
`FIRRTL_AFTER_INITIAL
`endif
`endif // SYNTHESIS
endmodule