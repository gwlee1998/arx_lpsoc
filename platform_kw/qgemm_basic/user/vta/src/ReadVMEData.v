module ReadVMEData(
  input         clock,
  input         reset,
  input         io_start,
  output        io_vmeData_ready,
  input         io_vmeData_valid,
  input  [20:0] io_vmeData_bits_tag,
  output [10:0] io_idx,
  output [2:0]  io_col
);
`ifdef RANDOMIZE_REG_INIT
  reg [31:0] _RAND_0;
  reg [31:0] _RAND_1;
  reg [31:0] _RAND_2;
  reg [31:0] _RAND_3;
`endif // RANDOMIZE_REG_INIT
  reg [20:0] vmeTagDecodeLast; // @[TensorLoadNarrowVME.scala 502:29]
  wire [17:0] rdDataIdx = io_vmeData_bits_tag[20:3]; // @[TensorLoadNarrowVME.scala 503:31]
  wire [2:0] rdDataCol = io_vmeData_bits_tag[2:0]; // @[TensorLoadNarrowVME.scala 504:65]
  reg [2:0] rdDataDestColNext; // @[TensorLoadNarrowVME.scala 505:30]
  reg [15:0] rdDataDestIdxNext; // @[TensorLoadNarrowVME.scala 506:30]
  reg  vmeTagDecodeLastValidNext; // @[TensorLoadNarrowVME.scala 509:42]
  wire  _T = io_vmeData_ready & io_vmeData_valid; // @[Decoupled.scala 40:37]
  wire  _GEN_0 = _T | vmeTagDecodeLastValidNext; // @[TensorLoadNarrowVME.scala 514:33 TensorLoadNarrowVME.scala 515:27 TensorLoadNarrowVME.scala 517:27]
  wire  _T_3 = io_vmeData_bits_tag != vmeTagDecodeLast; // @[TensorLoadNarrowVME.scala 525:29]
  wire  _T_4 = vmeTagDecodeLastValidNext & _T_3; // @[TensorLoadNarrowVME.scala 524:34]
  wire  _T_5 = ~vmeTagDecodeLastValidNext | _T_4; // @[TensorLoadNarrowVME.scala 523:34]
  wire [2:0] _rdDataDestColNext_T_1 = rdDataCol + 3'h1; // @[TensorLoadNarrowVME.scala 530:38]
  wire [2:0] _rdDataDestColNext_T_3 = rdDataDestColNext + 3'h1; // @[TensorLoadNarrowVME.scala 534:46]
  wire [2:0] rdDataDestCol = _T_5 ? rdDataCol : rdDataDestColNext; // @[TensorLoadNarrowVME.scala 525:59 TensorLoadNarrowVME.scala 528:21 TensorLoadNarrowVME.scala 533:21]
  wire [15:0] _rdDataDestIdxNext_T_1 = rdDataDestIdxNext + 16'h1; // @[TensorLoadNarrowVME.scala 537:48]
  wire [15:0] _GEN_2 = rdDataDestCol == 3'h7 ? _rdDataDestIdxNext_T_1 : rdDataDestIdxNext; // @[TensorLoadNarrowVME.scala 536:54 TensorLoadNarrowVME.scala 537:27 TensorLoadNarrowVME.scala 506:30]
  wire [17:0] _GEN_5 = _T_5 ? rdDataIdx : {{2'd0}, rdDataDestIdxNext}; // @[TensorLoadNarrowVME.scala 525:59 TensorLoadNarrowVME.scala 529:21 TensorLoadNarrowVME.scala 535:21]
  wire [17:0] _GEN_7 = _T_5 ? rdDataIdx : {{2'd0}, _GEN_2}; // @[TensorLoadNarrowVME.scala 525:59 TensorLoadNarrowVME.scala 531:25]
  wire [17:0] _GEN_12 = _T ? _GEN_7 : {{2'd0}, rdDataDestIdxNext}; // @[TensorLoadNarrowVME.scala 521:27 TensorLoadNarrowVME.scala 506:30]
  wire [15:0] rdDataDestIdx = _GEN_5[15:0]; // @[TensorLoadNarrowVME.scala 497:27]
  assign io_vmeData_ready = 1'h1; // @[TensorLoadNarrowVME.scala 498:20]
  assign io_idx = rdDataDestIdx[10:0]; // @[TensorLoadNarrowVME.scala 542:10]
  assign io_col = _T_5 ? rdDataCol : rdDataDestColNext; // @[TensorLoadNarrowVME.scala 525:59 TensorLoadNarrowVME.scala 528:21 TensorLoadNarrowVME.scala 533:21]
  always @(posedge clock) begin
    if (_T) begin // @[TensorLoadNarrowVME.scala 521:27]
      if (_T_5) begin // @[TensorLoadNarrowVME.scala 525:59]
        vmeTagDecodeLast <= io_vmeData_bits_tag; // @[TensorLoadNarrowVME.scala 527:24]
      end
    end
    if (_T) begin // @[TensorLoadNarrowVME.scala 521:27]
      if (_T_5) begin // @[TensorLoadNarrowVME.scala 525:59]
        rdDataDestColNext <= _rdDataDestColNext_T_1; // @[TensorLoadNarrowVME.scala 530:25]
      end else begin
        rdDataDestColNext <= _rdDataDestColNext_T_3; // @[TensorLoadNarrowVME.scala 534:25]
      end
    end
    rdDataDestIdxNext <= _GEN_12[15:0];
    if (reset) begin // @[TensorLoadNarrowVME.scala 509:42]
      vmeTagDecodeLastValidNext <= 1'h0; // @[TensorLoadNarrowVME.scala 509:42]
    end else if (io_start) begin // @[TensorLoadNarrowVME.scala 512:18]
      vmeTagDecodeLastValidNext <= 1'h0; // @[TensorLoadNarrowVME.scala 513:27]
    end else begin
      vmeTagDecodeLastValidNext <= _GEN_0;
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
  vmeTagDecodeLast = _RAND_0[20:0];
  _RAND_1 = {1{`RANDOM}};
  rdDataDestColNext = _RAND_1[2:0];
  _RAND_2 = {1{`RANDOM}};
  rdDataDestIdxNext = _RAND_2[15:0];
  _RAND_3 = {1{`RANDOM}};
  vmeTagDecodeLastValidNext = _RAND_3[0:0];
`endif // RANDOMIZE_REG_INIT
  `endif // RANDOMIZE
end // initial
`ifdef FIRRTL_AFTER_INITIAL
`FIRRTL_AFTER_INITIAL
`endif
`endif // SYNTHESIS
endmodule