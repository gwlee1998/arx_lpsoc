module GenVMECmd(
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
`endif // RANDOMIZE_REG_INIT
  wire [15:0] dec_sram_offset = io_inst[25:10]; // @[TensorLoadNarrowVME.scala 566:29]
  wire [31:0] dec_dram_offset = io_inst[57:26]; // @[TensorLoadNarrowVME.scala 566:29]
  wire [15:0] dec_ysize = io_inst[79:64]; // @[TensorLoadNarrowVME.scala 566:29]
  wire [15:0] dec_xsize = io_inst[95:80]; // @[TensorLoadNarrowVME.scala 566:29]
  wire [15:0] dec_xstride = io_inst[111:96]; // @[TensorLoadNarrowVME.scala 566:29]
  wire [3:0] dec_ypad_0 = io_inst[115:112]; // @[TensorLoadNarrowVME.scala 566:29]
  wire [3:0] dec_xpad_0 = io_inst[123:120]; // @[TensorLoadNarrowVME.scala 566:29]
  wire [3:0] dec_xpad_1 = io_inst[127:124]; // @[TensorLoadNarrowVME.scala 566:29]
  reg [31:0] rdCmdExtAddr; // @[TensorLoadNarrowVME.scala 568:25]
  wire [37:0] _xfer_init_addr_T = {dec_dram_offset, 6'h0}; // @[TensorLoadNarrowVME.scala 573:66]
  wire [37:0] _xfer_init_addr_T_1 = 38'hffffffff & _xfer_init_addr_T; // @[TensorLoadNarrowVME.scala 573:47]
  wire [37:0] _GEN_31 = {{6'd0}, io_baddr}; // @[TensorLoadNarrowVME.scala 573:33]
  wire [37:0] xfer_init_addr = _GEN_31 | _xfer_init_addr_T_1; // @[TensorLoadNarrowVME.scala 573:33]
  wire [31:0] _GEN_0 = rdCmdExtAddr % 32'h800; // @[TensorLoadNarrowVME.scala 577:53]
  wire [11:0] _firstMaxTransfer_T = _GEN_0[11:0]; // @[TensorLoadNarrowVME.scala 577:53]
  wire [11:0] _firstMaxTransfer_T_2 = 12'h800 - _firstMaxTransfer_T; // @[TensorLoadNarrowVME.scala 577:38]
  wire [8:0] firstMaxTransfer = _firstMaxTransfer_T_2[11:3]; // @[TensorLoadNarrowVME.scala 577:67]
  reg [10:0] rdCmdStartIdx; // @[TensorLoadNarrowVME.scala 586:26]
  reg  commandsDone; // @[TensorLoadNarrowVME.scala 588:29]
  wire [18:0] blocksReadSize = {dec_xsize, 3'h0}; // @[TensorLoadNarrowVME.scala 590:35]
  reg [18:0] blocksReadNb; // @[TensorLoadNarrowVME.scala 591:25]
  reg [31:0] rdCmdExtAddrRowBegin; // @[TensorLoadNarrowVME.scala 592:33]
  reg  newReadRow; // @[TensorLoadNarrowVME.scala 593:23]
  reg [15:0] srcRowIdx; // @[TensorLoadNarrowVME.scala 596:22]
  wire [15:0] _srcRowIdx_T_1 = srcRowIdx + 16'h1; // @[TensorLoadNarrowVME.scala 600:28]
  wire [18:0] blocksRemained = blocksReadSize - blocksReadNb; // @[TensorLoadNarrowVME.scala 628:39]
  wire [18:0] _GEN_32 = {{10'd0}, firstMaxTransfer}; // @[TensorLoadNarrowVME.scala 630:25]
  wire [18:0] _GEN_8 = blocksRemained < _GEN_32 ? blocksRemained : {{10'd0}, firstMaxTransfer}; // @[TensorLoadNarrowVME.scala 630:45 TensorLoadNarrowVME.scala 631:15 TensorLoadNarrowVME.scala 633:15]
  wire [18:0] _GEN_9 = blocksRemained < 19'h100 ? blocksRemained : 19'h100; // @[TensorLoadNarrowVME.scala 636:40 TensorLoadNarrowVME.scala 637:15 TensorLoadNarrowVME.scala 639:15]
  wire [18:0] _GEN_10 = newReadRow ? _GEN_8 : _GEN_9; // @[TensorLoadNarrowVME.scala 629:21]
  wire [8:0] readLen = _GEN_10[8:0]; // @[TensorLoadNarrowVME.scala 587:21]
  wire [18:0] _GEN_33 = {{10'd0}, readLen}; // @[TensorLoadNarrowVME.scala 621:41]
  wire [18:0] _T_8 = blocksReadSize - _GEN_33; // @[TensorLoadNarrowVME.scala 621:41]
  wire [15:0] _T_11 = dec_ysize - 16'h1; // @[TensorLoadNarrowVME.scala 621:80]
  wire  _T_14 = io_vmeCmd_ready & io_vmeCmd_valid; // @[Decoupled.scala 40:37]
  wire  stride = blocksReadNb == _T_8 & srcRowIdx != _T_11 & _T_14; // @[TensorLoadNarrowVME.scala 621:87]
  wire [18:0] nextBlRNb = blocksReadNb + _GEN_33; // @[TensorLoadNarrowVME.scala 611:34]
  wire  _GEN_2 = nextBlRNb == blocksReadSize & srcRowIdx == _T_11 | commandsDone; // @[TensorLoadNarrowVME.scala 613:74 TensorLoadNarrowVME.scala 614:20 TensorLoadNarrowVME.scala 606:16]
  wire  _GEN_4 = _T_14 ? _GEN_2 : commandsDone; // @[TensorLoadNarrowVME.scala 610:33 TensorLoadNarrowVME.scala 606:16]
  wire  _GEN_6 = io_start | stride ? 1'h0 : _GEN_4; // @[TensorLoadNarrowVME.scala 607:29 TensorLoadNarrowVME.scala 609:18]
  wire [15:0] _GEN_35 = {{12'd0}, dec_xpad_0}; // @[TensorLoadNarrowVME.scala 643:30]
  wire [15:0] _totalWidth_T_1 = dec_xsize + _GEN_35; // @[TensorLoadNarrowVME.scala 643:30]
  wire [15:0] _GEN_36 = {{12'd0}, dec_xpad_1}; // @[TensorLoadNarrowVME.scala 643:43]
  wire [15:0] totalWidth = _totalWidth_T_1 + _GEN_36; // @[TensorLoadNarrowVME.scala 643:43]
  reg [19:0] currentRowIdx; // @[TensorLoadNarrowVME.scala 647:26]
  wire [19:0] _GEN_37 = {{16'd0}, dec_ypad_0}; // @[TensorLoadNarrowVME.scala 649:39]
  wire [15:0] _GEN_38 = {{12'd0}, dec_ypad_0}; // @[TensorLoadNarrowVME.scala 650:32]
  wire [15:0] _rdCmdStartIdxValid_T_2 = dec_ysize + _GEN_38; // @[TensorLoadNarrowVME.scala 650:32]
  wire [19:0] _GEN_39 = {{4'd0}, _rdCmdStartIdxValid_T_2}; // @[TensorLoadNarrowVME.scala 650:19]
  wire  _rdCmdStartIdxValid_T_3 = currentRowIdx < _GEN_39; // @[TensorLoadNarrowVME.scala 650:19]
  wire  _rdCmdStartIdxValid_T_4 = currentRowIdx >= _GEN_37 & _rdCmdStartIdxValid_T_3; // @[TensorLoadNarrowVME.scala 649:53]
  wire  _rdCmdStartIdxValid_T_5 = _rdCmdStartIdxValid_T_4 & io_isBusy; // @[TensorLoadNarrowVME.scala 650:46]
  wire  _rdCmdStartIdxValid_T_6 = ~commandsDone; // @[TensorLoadNarrowVME.scala 652:5]
  wire  rdCmdStartIdxValid = _rdCmdStartIdxValid_T_5 & _rdCmdStartIdxValid_T_6; // @[TensorLoadNarrowVME.scala 651:15]
  wire [15:0] _rdCmdStartIdx_T_1 = dec_sram_offset + _GEN_35; // @[TensorLoadNarrowVME.scala 655:38]
  wire [15:0] _GEN_42 = {{5'd0}, rdCmdStartIdx}; // @[TensorLoadNarrowVME.scala 657:36]
  wire [15:0] _rdCmdStartIdx_T_3 = _GEN_42 + totalWidth; // @[TensorLoadNarrowVME.scala 657:36]
  wire [19:0] _currentRowIdx_T_1 = currentRowIdx + 20'h1; // @[TensorLoadNarrowVME.scala 658:36]
  wire [15:0] _GEN_11 = io_isBusy & (currentRowIdx < _GEN_37 | stride) ? _rdCmdStartIdx_T_3 : {{5'd0}, rdCmdStartIdx}; // @[TensorLoadNarrowVME.scala 656:68 TensorLoadNarrowVME.scala 657:19 TensorLoadNarrowVME.scala 586:26]
  wire [15:0] _GEN_14 = io_start ? _rdCmdStartIdx_T_1 : _GEN_11; // @[TensorLoadNarrowVME.scala 653:19 TensorLoadNarrowVME.scala 655:19]
  wire  startIssueCmdRead = blocksReadNb == 19'h0 & rdCmdStartIdxValid; // @[TensorLoadNarrowVME.scala 661:29]
  //wire [21:0] _memRow_T = {dec_xstride, 6'h0}; // @[TensorLoadNarrowVME.scala 672:56]
  wire [21:0] _memRow_T = {6'h0, dec_xstride}; // @[TensorLoadNarrowVME.scala 672:56] // set stride in bytes by hujang
  wire [31:0] _GEN_43 = {{10'd0}, _memRow_T}; // @[TensorLoadNarrowVME.scala 672:41]
  wire [31:0] memRow = rdCmdExtAddrRowBegin + _GEN_43; // @[TensorLoadNarrowVME.scala 672:41]
  wire [11:0] _rdCmdExtAddr_T = {readLen, 3'h0}; // @[TensorLoadNarrowVME.scala 679:47]
  wire [31:0] _GEN_44 = {{20'd0}, _rdCmdExtAddr_T}; // @[TensorLoadNarrowVME.scala 679:36]
  wire [31:0] _rdCmdExtAddr_T_2 = rdCmdExtAddr + _GEN_44; // @[TensorLoadNarrowVME.scala 679:36]
  wire [31:0] _GEN_16 = stride ? memRow : _rdCmdExtAddr_T_2; // @[TensorLoadNarrowVME.scala 671:18 TensorLoadNarrowVME.scala 673:20 TensorLoadNarrowVME.scala 679:20]
  wire [31:0] _GEN_17 = stride ? memRow : rdCmdExtAddrRowBegin; // @[TensorLoadNarrowVME.scala 671:18 TensorLoadNarrowVME.scala 674:28 TensorLoadNarrowVME.scala 664:24]
  wire [31:0] _GEN_19 = _T_14 ? _GEN_16 : rdCmdExtAddr; // @[TensorLoadNarrowVME.scala 670:33 TensorLoadNarrowVME.scala 682:18]
  wire [31:0] _GEN_20 = _T_14 ? _GEN_17 : rdCmdExtAddrRowBegin; // @[TensorLoadNarrowVME.scala 670:33 TensorLoadNarrowVME.scala 664:24]
  wire  _GEN_21 = _T_14 ? stride : newReadRow; // @[TensorLoadNarrowVME.scala 670:33 TensorLoadNarrowVME.scala 683:16]
  wire [37:0] _GEN_22 = io_start ? xfer_init_addr : {{6'd0}, _GEN_19}; // @[TensorLoadNarrowVME.scala 666:19 TensorLoadNarrowVME.scala 667:18]
  wire [37:0] _GEN_23 = io_start ? xfer_init_addr : {{6'd0}, _GEN_20}; // @[TensorLoadNarrowVME.scala 666:19 TensorLoadNarrowVME.scala 668:26]
  reg [13:0] rdCmdDestBlockIdxNext; // @[TensorLoadNarrowVME.scala 700:34]
  wire [13:0] _rdCmdDestBlockIdx_T = {rdCmdStartIdx, 3'h0}; // @[TensorLoadNarrowVME.scala 710:42]
  wire [13:0] _GEN_26 = startIssueCmdRead ? _rdCmdDestBlockIdx_T : rdCmdDestBlockIdxNext; // @[TensorLoadNarrowVME.scala 709:29 TensorLoadNarrowVME.scala 710:25 TensorLoadNarrowVME.scala 702:21]
  wire [13:0] rdCmdDestBlockIdx = rdCmdStartIdxValid ? _GEN_26 : rdCmdDestBlockIdxNext; // @[TensorLoadNarrowVME.scala 707:28 TensorLoadNarrowVME.scala 702:21]
  wire [13:0] _GEN_45 = {{5'd0}, readLen}; // @[TensorLoadNarrowVME.scala 711:49]
  wire [13:0] _rdCmdDestBlockIdxNext_T_1 = rdCmdDestBlockIdx + _GEN_45; // @[TensorLoadNarrowVME.scala 711:49]
  wire [13:0] _rdCmdDestBlockIdxNext_T_3 = rdCmdDestBlockIdxNext + _GEN_45; // @[TensorLoadNarrowVME.scala 714:53]
  wire [8:0] _io_vmeCmd_bits_len_T_1 = readLen - 9'h1; // @[TensorLoadNarrowVME.scala 732:33]
  assign io_vmeCmd_valid = _rdCmdStartIdxValid_T_5 & _rdCmdStartIdxValid_T_6; // @[TensorLoadNarrowVME.scala 651:15]
  assign io_vmeCmd_bits_addr = rdCmdExtAddr; // @[TensorLoadNarrowVME.scala 731:23]
  assign io_vmeCmd_bits_len = _io_vmeCmd_bits_len_T_1[7:0]; // @[TensorLoadNarrowVME.scala 732:22]
  assign io_vmeCmd_bits_tag = {{7'd0}, rdCmdDestBlockIdx}; // @[TensorLoadNarrowVME.scala 707:28 TensorLoadNarrowVME.scala 702:21]
  assign io_readLen = _GEN_10[8:0]; // @[TensorLoadNarrowVME.scala 587:21]
  assign io_done = commandsDone; // @[TensorLoadNarrowVME.scala 739:11]
  always @(posedge clock) begin
    rdCmdExtAddr <= _GEN_22[31:0];
    rdCmdStartIdx <= _GEN_14[10:0];
    commandsDone <= reset | _GEN_6; // @[TensorLoadNarrowVME.scala 588:29 TensorLoadNarrowVME.scala 588:29]
    if (io_start | stride) begin // @[TensorLoadNarrowVME.scala 607:29]
      blocksReadNb <= 19'h0; // @[TensorLoadNarrowVME.scala 608:18]
    end else if (_T_14) begin // @[TensorLoadNarrowVME.scala 610:33]
      blocksReadNb <= nextBlRNb; // @[TensorLoadNarrowVME.scala 612:18]
    end
    rdCmdExtAddrRowBegin <= _GEN_23[31:0];
    newReadRow <= io_start | _GEN_21; // @[TensorLoadNarrowVME.scala 666:19 TensorLoadNarrowVME.scala 669:16]
    if (io_start) begin // @[TensorLoadNarrowVME.scala 597:19]
      srcRowIdx <= 16'h0; // @[TensorLoadNarrowVME.scala 598:15]
    end else if (stride) begin // @[TensorLoadNarrowVME.scala 599:23]
      srcRowIdx <= _srcRowIdx_T_1; // @[TensorLoadNarrowVME.scala 600:15]
    end
    if (io_start) begin // @[TensorLoadNarrowVME.scala 653:19]
      currentRowIdx <= 20'h0; // @[TensorLoadNarrowVME.scala 654:19]
    end else if (io_isBusy & (currentRowIdx < _GEN_37 | stride)) begin // @[TensorLoadNarrowVME.scala 656:68]
      currentRowIdx <= _currentRowIdx_T_1; // @[TensorLoadNarrowVME.scala 658:19]
    end
    if (rdCmdStartIdxValid) begin // @[TensorLoadNarrowVME.scala 707:28]
      if (startIssueCmdRead) begin // @[TensorLoadNarrowVME.scala 709:29]
        rdCmdDestBlockIdxNext <= _rdCmdDestBlockIdxNext_T_1; // @[TensorLoadNarrowVME.scala 711:28]
      end else if (_T_14) begin // @[TensorLoadNarrowVME.scala 712:35]
        rdCmdDestBlockIdxNext <= _rdCmdDestBlockIdxNext_T_3; // @[TensorLoadNarrowVME.scala 714:28]
      end
    end
    `ifndef SYNTHESIS
    `ifdef PRINTF_COND
      if (`PRINTF_COND) begin
    `endif
        if (~(~io_isBusy | blocksReadSize >= blocksReadNb | reset)) begin
          $fwrite(32'h80000002,
            "Assertion failed\n    at TensorLoadNarrowVME.scala:627 assert(!io.isBusy || blocksReadSize >= blocksReadNb)// define how many block to read at this cycle\n"
            ); // @[TensorLoadNarrowVME.scala 627:9]
        end
    `ifdef PRINTF_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef STOP_COND
      if (`STOP_COND) begin
    `endif
        if (~(~io_isBusy | blocksReadSize >= blocksReadNb | reset)) begin
          $fatal; // @[TensorLoadNarrowVME.scala 627:9]
        end
    `ifdef STOP_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef PRINTF_COND
      if (`PRINTF_COND) begin
    `endif
        if (~(~io_vmeCmd_valid | _rdCmdExtAddr_T <= _firstMaxTransfer_T_2 | reset)) begin
          $fwrite(32'h80000002,
            "Assertion failed: -F- inp DRAM page alignment failure. DRAM address + len overlaps mp.lenBits*memBlockSize alignment %x %x\n    at TensorLoadNarrowVME.scala:733 assert(!io.vmeCmd.valid || ((readLen << log2Ceil(mp.dataBits/8)) <= (maxTrBytes - rdCmdExtAddr %% maxTrBytes)),\n"
            ,rdCmdExtAddr,readLen); // @[TensorLoadNarrowVME.scala 733:9]
        end
    `ifdef PRINTF_COND
      end
    `endif
    `endif // SYNTHESIS
    `ifndef SYNTHESIS
    `ifdef STOP_COND
      if (`STOP_COND) begin
    `endif
        if (~(~io_vmeCmd_valid | _rdCmdExtAddr_T <= _firstMaxTransfer_T_2 | reset)) begin
          $fatal; // @[TensorLoadNarrowVME.scala 733:9]
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
  rdCmdExtAddr = _RAND_0[31:0];
  _RAND_1 = {1{`RANDOM}};
  rdCmdStartIdx = _RAND_1[10:0];
  _RAND_2 = {1{`RANDOM}};
  commandsDone = _RAND_2[0:0];
  _RAND_3 = {1{`RANDOM}};
  blocksReadNb = _RAND_3[18:0];
  _RAND_4 = {1{`RANDOM}};
  rdCmdExtAddrRowBegin = _RAND_4[31:0];
  _RAND_5 = {1{`RANDOM}};
  newReadRow = _RAND_5[0:0];
  _RAND_6 = {1{`RANDOM}};
  srcRowIdx = _RAND_6[15:0];
  _RAND_7 = {1{`RANDOM}};
  currentRowIdx = _RAND_7[19:0];
  _RAND_8 = {1{`RANDOM}};
  rdCmdDestBlockIdxNext = _RAND_8[13:0];
`endif // RANDOMIZE_REG_INIT
  `endif // RANDOMIZE
end // initial
`ifdef FIRRTL_AFTER_INITIAL
`FIRRTL_AFTER_INITIAL
`endif
`endif // SYNTHESIS
endmodule

