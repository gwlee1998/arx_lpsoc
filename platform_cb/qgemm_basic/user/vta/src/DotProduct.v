module DotProduct(
  input         clock,
  input  [31:0] io_a_0,
  input  [31:0] io_a_1,
  input  [31:0] io_a_2,
  input  [31:0] io_a_3,
  input  [31:0] io_a_4,
  input  [31:0] io_a_5,
  input  [31:0] io_a_6,
  input  [31:0] io_a_7,
  input  [31:0] io_a_8,
  input  [31:0] io_a_9,
  input  [31:0] io_a_10,
  input  [31:0] io_a_11,
  input  [31:0] io_a_12,
  input  [31:0] io_a_13,
  input  [31:0] io_a_14,
  input  [31:0] io_a_15,
  input  [31:0] io_b_0,
  input  [31:0] io_b_1,
  input  [31:0] io_b_2,
  input  [31:0] io_b_3,
  input  [31:0] io_b_4,
  input  [31:0] io_b_5,
  input  [31:0] io_b_6,
  input  [31:0] io_b_7,
  input  [31:0] io_b_8,
  input  [31:0] io_b_9,
  input  [31:0] io_b_10,
  input  [31:0] io_b_11,
  input  [31:0] io_b_12,
  input  [31:0] io_b_13,
  input  [31:0] io_b_14,
  input  [31:0] io_b_15,
  output [68:0] io_y
);
  wire  m_0_clock; // @[TensorGemm.scala 100:32]
  wire [31:0] m_0_io_a; // @[TensorGemm.scala 100:32]
  wire [31:0] m_0_io_b; // @[TensorGemm.scala 100:32]
  wire [64:0] m_0_io_y; // @[TensorGemm.scala 100:32]
  wire  m_1_clock; // @[TensorGemm.scala 100:32]
  wire [31:0] m_1_io_a; // @[TensorGemm.scala 100:32]
  wire [31:0] m_1_io_b; // @[TensorGemm.scala 100:32]
  wire [64:0] m_1_io_y; // @[TensorGemm.scala 100:32]
  wire  m_2_clock; // @[TensorGemm.scala 100:32]
  wire [31:0] m_2_io_a; // @[TensorGemm.scala 100:32]
  wire [31:0] m_2_io_b; // @[TensorGemm.scala 100:32]
  wire [64:0] m_2_io_y; // @[TensorGemm.scala 100:32]
  wire  m_3_clock; // @[TensorGemm.scala 100:32]
  wire [31:0] m_3_io_a; // @[TensorGemm.scala 100:32]
  wire [31:0] m_3_io_b; // @[TensorGemm.scala 100:32]
  wire [64:0] m_3_io_y; // @[TensorGemm.scala 100:32]
  wire  m_4_clock; // @[TensorGemm.scala 100:32]
  wire [31:0] m_4_io_a; // @[TensorGemm.scala 100:32]
  wire [31:0] m_4_io_b; // @[TensorGemm.scala 100:32]
  wire [64:0] m_4_io_y; // @[TensorGemm.scala 100:32]
  wire  m_5_clock; // @[TensorGemm.scala 100:32]
  wire [31:0] m_5_io_a; // @[TensorGemm.scala 100:32]
  wire [31:0] m_5_io_b; // @[TensorGemm.scala 100:32]
  wire [64:0] m_5_io_y; // @[TensorGemm.scala 100:32]
  wire  m_6_clock; // @[TensorGemm.scala 100:32]
  wire [31:0] m_6_io_a; // @[TensorGemm.scala 100:32]
  wire [31:0] m_6_io_b; // @[TensorGemm.scala 100:32]
  wire [64:0] m_6_io_y; // @[TensorGemm.scala 100:32]
  wire  m_7_clock; // @[TensorGemm.scala 100:32]
  wire [31:0] m_7_io_a; // @[TensorGemm.scala 100:32]
  wire [31:0] m_7_io_b; // @[TensorGemm.scala 100:32]
  wire [64:0] m_7_io_y; // @[TensorGemm.scala 100:32]
  wire  m_8_clock; // @[TensorGemm.scala 100:32]
  wire [31:0] m_8_io_a; // @[TensorGemm.scala 100:32]
  wire [31:0] m_8_io_b; // @[TensorGemm.scala 100:32]
  wire [64:0] m_8_io_y; // @[TensorGemm.scala 100:32]
  wire  m_9_clock; // @[TensorGemm.scala 100:32]
  wire [31:0] m_9_io_a; // @[TensorGemm.scala 100:32]
  wire [31:0] m_9_io_b; // @[TensorGemm.scala 100:32]
  wire [64:0] m_9_io_y; // @[TensorGemm.scala 100:32]
  wire  m_10_clock; // @[TensorGemm.scala 100:32]
  wire [31:0] m_10_io_a; // @[TensorGemm.scala 100:32]
  wire [31:0] m_10_io_b; // @[TensorGemm.scala 100:32]
  wire [64:0] m_10_io_y; // @[TensorGemm.scala 100:32]
  wire  m_11_clock; // @[TensorGemm.scala 100:32]
  wire [31:0] m_11_io_a; // @[TensorGemm.scala 100:32]
  wire [31:0] m_11_io_b; // @[TensorGemm.scala 100:32]
  wire [64:0] m_11_io_y; // @[TensorGemm.scala 100:32]
  wire  m_12_clock; // @[TensorGemm.scala 100:32]
  wire [31:0] m_12_io_a; // @[TensorGemm.scala 100:32]
  wire [31:0] m_12_io_b; // @[TensorGemm.scala 100:32]
  wire [64:0] m_12_io_y; // @[TensorGemm.scala 100:32]
  wire  m_13_clock; // @[TensorGemm.scala 100:32]
  wire [31:0] m_13_io_a; // @[TensorGemm.scala 100:32]
  wire [31:0] m_13_io_b; // @[TensorGemm.scala 100:32]
  wire [64:0] m_13_io_y; // @[TensorGemm.scala 100:32]
  wire  m_14_clock; // @[TensorGemm.scala 100:32]
  wire [31:0] m_14_io_a; // @[TensorGemm.scala 100:32]
  wire [31:0] m_14_io_b; // @[TensorGemm.scala 100:32]
  wire [64:0] m_14_io_y; // @[TensorGemm.scala 100:32]
  wire  m_15_clock; // @[TensorGemm.scala 100:32]
  wire [31:0] m_15_io_a; // @[TensorGemm.scala 100:32]
  wire [31:0] m_15_io_b; // @[TensorGemm.scala 100:32]
  wire [64:0] m_15_io_y; // @[TensorGemm.scala 100:32]
  wire [64:0] a_0_0_io_a; // @[TensorGemm.scala 108:17]
  wire [64:0] a_0_0_io_b; // @[TensorGemm.scala 108:17]
  wire [65:0] a_0_0_io_y; // @[TensorGemm.scala 108:17]
  wire [64:0] a_0_1_io_a; // @[TensorGemm.scala 108:17]
  wire [64:0] a_0_1_io_b; // @[TensorGemm.scala 108:17]
  wire [65:0] a_0_1_io_y; // @[TensorGemm.scala 108:17]
  wire [64:0] a_0_2_io_a; // @[TensorGemm.scala 108:17]
  wire [64:0] a_0_2_io_b; // @[TensorGemm.scala 108:17]
  wire [65:0] a_0_2_io_y; // @[TensorGemm.scala 108:17]
  wire [64:0] a_0_3_io_a; // @[TensorGemm.scala 108:17]
  wire [64:0] a_0_3_io_b; // @[TensorGemm.scala 108:17]
  wire [65:0] a_0_3_io_y; // @[TensorGemm.scala 108:17]
  wire [64:0] a_0_4_io_a; // @[TensorGemm.scala 108:17]
  wire [64:0] a_0_4_io_b; // @[TensorGemm.scala 108:17]
  wire [65:0] a_0_4_io_y; // @[TensorGemm.scala 108:17]
  wire [64:0] a_0_5_io_a; // @[TensorGemm.scala 108:17]
  wire [64:0] a_0_5_io_b; // @[TensorGemm.scala 108:17]
  wire [65:0] a_0_5_io_y; // @[TensorGemm.scala 108:17]
  wire [64:0] a_0_6_io_a; // @[TensorGemm.scala 108:17]
  wire [64:0] a_0_6_io_b; // @[TensorGemm.scala 108:17]
  wire [65:0] a_0_6_io_y; // @[TensorGemm.scala 108:17]
  wire [64:0] a_0_7_io_a; // @[TensorGemm.scala 108:17]
  wire [64:0] a_0_7_io_b; // @[TensorGemm.scala 108:17]
  wire [65:0] a_0_7_io_y; // @[TensorGemm.scala 108:17]
  wire [65:0] a_1_0_io_a; // @[TensorGemm.scala 108:17]
  wire [65:0] a_1_0_io_b; // @[TensorGemm.scala 108:17]
  wire [66:0] a_1_0_io_y; // @[TensorGemm.scala 108:17]
  wire [65:0] a_1_1_io_a; // @[TensorGemm.scala 108:17]
  wire [65:0] a_1_1_io_b; // @[TensorGemm.scala 108:17]
  wire [66:0] a_1_1_io_y; // @[TensorGemm.scala 108:17]
  wire [65:0] a_1_2_io_a; // @[TensorGemm.scala 108:17]
  wire [65:0] a_1_2_io_b; // @[TensorGemm.scala 108:17]
  wire [66:0] a_1_2_io_y; // @[TensorGemm.scala 108:17]
  wire [65:0] a_1_3_io_a; // @[TensorGemm.scala 108:17]
  wire [65:0] a_1_3_io_b; // @[TensorGemm.scala 108:17]
  wire [66:0] a_1_3_io_y; // @[TensorGemm.scala 108:17]
  wire  a_2_0_clock; // @[TensorGemm.scala 105:17]
  wire [66:0] a_2_0_io_a; // @[TensorGemm.scala 105:17]
  wire [66:0] a_2_0_io_b; // @[TensorGemm.scala 105:17]
  wire [67:0] a_2_0_io_y; // @[TensorGemm.scala 105:17]
  wire  a_2_1_clock; // @[TensorGemm.scala 105:17]
  wire [66:0] a_2_1_io_a; // @[TensorGemm.scala 105:17]
  wire [66:0] a_2_1_io_b; // @[TensorGemm.scala 105:17]
  wire [67:0] a_2_1_io_y; // @[TensorGemm.scala 105:17]
  wire [67:0] a_3_0_io_a; // @[TensorGemm.scala 108:17]
  wire [67:0] a_3_0_io_b; // @[TensorGemm.scala 108:17]
  wire [68:0] a_3_0_io_y; // @[TensorGemm.scala 108:17]
  MAC m_0 ( // @[TensorGemm.scala 100:32]
    .clock(m_0_clock),
    .io_a(m_0_io_a),
    .io_b(m_0_io_b),
    .io_y(m_0_io_y)
  );
  MAC m_1 ( // @[TensorGemm.scala 100:32]
    .clock(m_1_clock),
    .io_a(m_1_io_a),
    .io_b(m_1_io_b),
    .io_y(m_1_io_y)
  );
  MAC m_2 ( // @[TensorGemm.scala 100:32]
    .clock(m_2_clock),
    .io_a(m_2_io_a),
    .io_b(m_2_io_b),
    .io_y(m_2_io_y)
  );
  MAC m_3 ( // @[TensorGemm.scala 100:32]
    .clock(m_3_clock),
    .io_a(m_3_io_a),
    .io_b(m_3_io_b),
    .io_y(m_3_io_y)
  );
  MAC m_4 ( // @[TensorGemm.scala 100:32]
    .clock(m_4_clock),
    .io_a(m_4_io_a),
    .io_b(m_4_io_b),
    .io_y(m_4_io_y)
  );
  MAC m_5 ( // @[TensorGemm.scala 100:32]
    .clock(m_5_clock),
    .io_a(m_5_io_a),
    .io_b(m_5_io_b),
    .io_y(m_5_io_y)
  );
  MAC m_6 ( // @[TensorGemm.scala 100:32]
    .clock(m_6_clock),
    .io_a(m_6_io_a),
    .io_b(m_6_io_b),
    .io_y(m_6_io_y)
  );
  MAC m_7 ( // @[TensorGemm.scala 100:32]
    .clock(m_7_clock),
    .io_a(m_7_io_a),
    .io_b(m_7_io_b),
    .io_y(m_7_io_y)
  );
  MAC m_8 ( // @[TensorGemm.scala 100:32]
    .clock(m_8_clock),
    .io_a(m_8_io_a),
    .io_b(m_8_io_b),
    .io_y(m_8_io_y)
  );
  MAC m_9 ( // @[TensorGemm.scala 100:32]
    .clock(m_9_clock),
    .io_a(m_9_io_a),
    .io_b(m_9_io_b),
    .io_y(m_9_io_y)
  );
  MAC m_10 ( // @[TensorGemm.scala 100:32]
    .clock(m_10_clock),
    .io_a(m_10_io_a),
    .io_b(m_10_io_b),
    .io_y(m_10_io_y)
  );
  MAC m_11 ( // @[TensorGemm.scala 100:32]
    .clock(m_11_clock),
    .io_a(m_11_io_a),
    .io_b(m_11_io_b),
    .io_y(m_11_io_y)
  );
  MAC m_12 ( // @[TensorGemm.scala 100:32]
    .clock(m_12_clock),
    .io_a(m_12_io_a),
    .io_b(m_12_io_b),
    .io_y(m_12_io_y)
  );
  MAC m_13 ( // @[TensorGemm.scala 100:32]
    .clock(m_13_clock),
    .io_a(m_13_io_a),
    .io_b(m_13_io_b),
    .io_y(m_13_io_y)
  );
  MAC m_14 ( // @[TensorGemm.scala 100:32]
    .clock(m_14_clock),
    .io_a(m_14_io_a),
    .io_b(m_14_io_b),
    .io_y(m_14_io_y)
  );
  MAC m_15 ( // @[TensorGemm.scala 100:32]
    .clock(m_15_clock),
    .io_a(m_15_io_a),
    .io_b(m_15_io_b),
    .io_y(m_15_io_y)
  );
  Adder a_0_0 ( // @[TensorGemm.scala 108:17]
    .io_a(a_0_0_io_a),
    .io_b(a_0_0_io_b),
    .io_y(a_0_0_io_y)
  );
  Adder a_0_1 ( // @[TensorGemm.scala 108:17]
    .io_a(a_0_1_io_a),
    .io_b(a_0_1_io_b),
    .io_y(a_0_1_io_y)
  );
  Adder a_0_2 ( // @[TensorGemm.scala 108:17]
    .io_a(a_0_2_io_a),
    .io_b(a_0_2_io_b),
    .io_y(a_0_2_io_y)
  );
  Adder a_0_3 ( // @[TensorGemm.scala 108:17]
    .io_a(a_0_3_io_a),
    .io_b(a_0_3_io_b),
    .io_y(a_0_3_io_y)
  );
  Adder a_0_4 ( // @[TensorGemm.scala 108:17]
    .io_a(a_0_4_io_a),
    .io_b(a_0_4_io_b),
    .io_y(a_0_4_io_y)
  );
  Adder a_0_5 ( // @[TensorGemm.scala 108:17]
    .io_a(a_0_5_io_a),
    .io_b(a_0_5_io_b),
    .io_y(a_0_5_io_y)
  );
  Adder a_0_6 ( // @[TensorGemm.scala 108:17]
    .io_a(a_0_6_io_a),
    .io_b(a_0_6_io_b),
    .io_y(a_0_6_io_y)
  );
  Adder a_0_7 ( // @[TensorGemm.scala 108:17]
    .io_a(a_0_7_io_a),
    .io_b(a_0_7_io_b),
    .io_y(a_0_7_io_y)
  );
  Adder_8 a_1_0 ( // @[TensorGemm.scala 108:17]
    .io_a(a_1_0_io_a),
    .io_b(a_1_0_io_b),
    .io_y(a_1_0_io_y)
  );
  Adder_8 a_1_1 ( // @[TensorGemm.scala 108:17]
    .io_a(a_1_1_io_a),
    .io_b(a_1_1_io_b),
    .io_y(a_1_1_io_y)
  );
  Adder_8 a_1_2 ( // @[TensorGemm.scala 108:17]
    .io_a(a_1_2_io_a),
    .io_b(a_1_2_io_b),
    .io_y(a_1_2_io_y)
  );
  Adder_8 a_1_3 ( // @[TensorGemm.scala 108:17]
    .io_a(a_1_3_io_a),
    .io_b(a_1_3_io_b),
    .io_y(a_1_3_io_y)
  );
  PipeAdder a_2_0 ( // @[TensorGemm.scala 105:17]
    .clock(a_2_0_clock),
    .io_a(a_2_0_io_a),
    .io_b(a_2_0_io_b),
    .io_y(a_2_0_io_y)
  );
  PipeAdder a_2_1 ( // @[TensorGemm.scala 105:17]
    .clock(a_2_1_clock),
    .io_a(a_2_1_io_a),
    .io_b(a_2_1_io_b),
    .io_y(a_2_1_io_y)
  );
  Adder_12 a_3_0 ( // @[TensorGemm.scala 108:17]
    .io_a(a_3_0_io_a),
    .io_b(a_3_0_io_b),
    .io_y(a_3_0_io_y)
  );
  assign io_y = a_3_0_io_y; // @[TensorGemm.scala 134:8]
  assign m_0_clock = clock;
  assign m_0_io_a = io_a_0; // @[TensorGemm.scala 114:15]
  assign m_0_io_b = io_b_0; // @[TensorGemm.scala 115:15]
  assign m_1_clock = clock;
  assign m_1_io_a = io_a_1; // @[TensorGemm.scala 114:15]
  assign m_1_io_b = io_b_1; // @[TensorGemm.scala 115:15]
  assign m_2_clock = clock;
  assign m_2_io_a = io_a_2; // @[TensorGemm.scala 114:15]
  assign m_2_io_b = io_b_2; // @[TensorGemm.scala 115:15]
  assign m_3_clock = clock;
  assign m_3_io_a = io_a_3; // @[TensorGemm.scala 114:15]
  assign m_3_io_b = io_b_3; // @[TensorGemm.scala 115:15]
  assign m_4_clock = clock;
  assign m_4_io_a = io_a_4; // @[TensorGemm.scala 114:15]
  assign m_4_io_b = io_b_4; // @[TensorGemm.scala 115:15]
  assign m_5_clock = clock;
  assign m_5_io_a = io_a_5; // @[TensorGemm.scala 114:15]
  assign m_5_io_b = io_b_5; // @[TensorGemm.scala 115:15]
  assign m_6_clock = clock;
  assign m_6_io_a = io_a_6; // @[TensorGemm.scala 114:15]
  assign m_6_io_b = io_b_6; // @[TensorGemm.scala 115:15]
  assign m_7_clock = clock;
  assign m_7_io_a = io_a_7; // @[TensorGemm.scala 114:15]
  assign m_7_io_b = io_b_7; // @[TensorGemm.scala 115:15]
  assign m_8_clock = clock;
  assign m_8_io_a = io_a_8; // @[TensorGemm.scala 114:15]
  assign m_8_io_b = io_b_8; // @[TensorGemm.scala 115:15]
  assign m_9_clock = clock;
  assign m_9_io_a = io_a_9; // @[TensorGemm.scala 114:15]
  assign m_9_io_b = io_b_9; // @[TensorGemm.scala 115:15]
  assign m_10_clock = clock;
  assign m_10_io_a = io_a_10; // @[TensorGemm.scala 114:15]
  assign m_10_io_b = io_b_10; // @[TensorGemm.scala 115:15]
  assign m_11_clock = clock;
  assign m_11_io_a = io_a_11; // @[TensorGemm.scala 114:15]
  assign m_11_io_b = io_b_11; // @[TensorGemm.scala 115:15]
  assign m_12_clock = clock;
  assign m_12_io_a = io_a_12; // @[TensorGemm.scala 114:15]
  assign m_12_io_b = io_b_12; // @[TensorGemm.scala 115:15]
  assign m_13_clock = clock;
  assign m_13_io_a = io_a_13; // @[TensorGemm.scala 114:15]
  assign m_13_io_b = io_b_13; // @[TensorGemm.scala 115:15]
  assign m_14_clock = clock;
  assign m_14_io_a = io_a_14; // @[TensorGemm.scala 114:15]
  assign m_14_io_b = io_b_14; // @[TensorGemm.scala 115:15]
  assign m_15_clock = clock;
  assign m_15_io_a = io_a_15; // @[TensorGemm.scala 114:15]
  assign m_15_io_b = io_b_15; // @[TensorGemm.scala 115:15]
  assign a_0_0_io_a = m_0_io_y; // @[TensorGemm.scala 124:22]
  assign a_0_0_io_b = m_1_io_y; // @[TensorGemm.scala 125:22]
  assign a_0_1_io_a = m_2_io_y; // @[TensorGemm.scala 124:22]
  assign a_0_1_io_b = m_3_io_y; // @[TensorGemm.scala 125:22]
  assign a_0_2_io_a = m_4_io_y; // @[TensorGemm.scala 124:22]
  assign a_0_2_io_b = m_5_io_y; // @[TensorGemm.scala 125:22]
  assign a_0_3_io_a = m_6_io_y; // @[TensorGemm.scala 124:22]
  assign a_0_3_io_b = m_7_io_y; // @[TensorGemm.scala 125:22]
  assign a_0_4_io_a = m_8_io_y; // @[TensorGemm.scala 124:22]
  assign a_0_4_io_b = m_9_io_y; // @[TensorGemm.scala 125:22]
  assign a_0_5_io_a = m_10_io_y; // @[TensorGemm.scala 124:22]
  assign a_0_5_io_b = m_11_io_y; // @[TensorGemm.scala 125:22]
  assign a_0_6_io_a = m_12_io_y; // @[TensorGemm.scala 124:22]
  assign a_0_6_io_b = m_13_io_y; // @[TensorGemm.scala 125:22]
  assign a_0_7_io_a = m_14_io_y; // @[TensorGemm.scala 124:22]
  assign a_0_7_io_b = m_15_io_y; // @[TensorGemm.scala 125:22]
  assign a_1_0_io_a = a_0_0_io_y; // @[TensorGemm.scala 127:22]
  assign a_1_0_io_b = a_0_1_io_y; // @[TensorGemm.scala 128:22]
  assign a_1_1_io_a = a_0_2_io_y; // @[TensorGemm.scala 127:22]
  assign a_1_1_io_b = a_0_3_io_y; // @[TensorGemm.scala 128:22]
  assign a_1_2_io_a = a_0_4_io_y; // @[TensorGemm.scala 127:22]
  assign a_1_2_io_b = a_0_5_io_y; // @[TensorGemm.scala 128:22]
  assign a_1_3_io_a = a_0_6_io_y; // @[TensorGemm.scala 127:22]
  assign a_1_3_io_b = a_0_7_io_y; // @[TensorGemm.scala 128:22]
  assign a_2_0_clock = clock;
  assign a_2_0_io_a = a_1_0_io_y; // @[TensorGemm.scala 127:22]
  assign a_2_0_io_b = a_1_1_io_y; // @[TensorGemm.scala 128:22]
  assign a_2_1_clock = clock;
  assign a_2_1_io_a = a_1_2_io_y; // @[TensorGemm.scala 127:22]
  assign a_2_1_io_b = a_1_3_io_y; // @[TensorGemm.scala 128:22]
  assign a_3_0_io_a = a_2_0_io_y; // @[TensorGemm.scala 127:22]
  assign a_3_0_io_b = a_2_1_io_y; // @[TensorGemm.scala 128:22]
endmodule