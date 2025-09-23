module AluReg(
  input         clock,
  input  [2:0]  io_opcode,
  input         io_a_valid,
  input  [31:0] io_a_bits,
  input         io_b_valid,
  input  [31:0] io_b_bits,
  output        io_y_valid,
  output [31:0] io_y_bits
);
`ifdef RANDOMIZE_REG_INIT
  reg [31:0] _RAND_0;
  reg [31:0] _RAND_1;
  reg [31:0] _RAND_2;
`endif // RANDOMIZE_REG_INIT
  wire [2:0] alu_io_opcode; // @[TensorAlu.scala 58:19]
  wire [31:0] alu_io_a; // @[TensorAlu.scala 58:19]
  wire [31:0] alu_io_b; // @[TensorAlu.scala 58:19]
  wire [31:0] alu_io_y; // @[TensorAlu.scala 58:19]
  reg [31:0] rA; // @[Reg.scala 15:16]
  reg [31:0] rB; // @[Reg.scala 15:16]
  reg  valid; // @[TensorAlu.scala 61:22]
  Alu alu ( // @[TensorAlu.scala 58:19]
    .io_opcode(alu_io_opcode),
    .io_a(alu_io_a),
    .io_b(alu_io_b),
    .io_y(alu_io_y)
  );
  assign io_y_valid = valid; // @[TensorAlu.scala 70:14]
  assign io_y_bits = alu_io_y; // @[TensorAlu.scala 71:25]
  assign alu_io_opcode = io_opcode; // @[TensorAlu.scala 63:17]
  assign alu_io_a = rA; // @[TensorAlu.scala 66:18]
  assign alu_io_b = rB; // @[TensorAlu.scala 67:18]
  always @(posedge clock) begin
    if (io_a_valid) begin // @[Reg.scala 16:19]
      rA <= io_a_bits; // @[Reg.scala 16:23]
    end
    if (io_b_valid) begin // @[Reg.scala 16:19]
      rB <= io_b_bits; // @[Reg.scala 16:23]
    end
    valid <= io_b_valid; // @[TensorAlu.scala 61:22]
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
  rA = _RAND_0[31:0];
  _RAND_1 = {1{`RANDOM}};
  rB = _RAND_1[31:0];
  _RAND_2 = {1{`RANDOM}};
  valid = _RAND_2[0:0];
`endif // RANDOMIZE_REG_INIT
  `endif // RANDOMIZE
end // initial
`ifdef FIRRTL_AFTER_INITIAL
`FIRRTL_AFTER_INITIAL
`endif
`endif // SYNTHESIS
endmodule