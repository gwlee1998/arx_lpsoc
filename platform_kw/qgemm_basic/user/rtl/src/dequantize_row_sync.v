module dequantize_row_sync #(
  parameter integer FP_DATA_W   = 32,
  parameter integer FP_MANT_W   = 23,
  parameter integer FP_EXP_W    = 8,
  parameter integer FP_EXP_BIAS = 127,
  parameter integer BIT_NUM     = 8,

  parameter integer LANES_NUM   = 4,      // 행/열 크기 (MAT_SIZE)
  parameter integer ROWS_MAX    = 1024,   // 전역 행 인덱스 wrap 기준
  parameter integer ROW_W       = 10      // ceil(log2(ROWS_MAX))을 외부에서 지정
)(
  input  wire                         clk,
  input  wire                         rstn,    // async active-low reset
  input  wire                         clear,   // sync clear

  // ACC 행 입력 (한 행 = LANES_NUM lanes)
  input  wire                         acc_row_valid,
  output wire                         acc_row_ready,
  input  wire [FP_DATA_W*LANES_NUM-1:0] acc_vec_i,

  // SCALE 행렬 입력(FIFO) : 한 번에 LANES×LANES
  input  wire                         fifo_rd_valid,
  output wire                         fifo_rd_ready,
  input  wire [FP_MANT_W*(LANES_NUM*LANES_NUM)-1:0] fifo_mant_dout,
  input  wire [FP_EXP_W *(LANES_NUM*LANES_NUM)-1:0] fifo_exp_dout,

  // 결과 (행 단위)
  output reg                          out_valid,
  input  wire                         out_ready,
  output reg  [FP_DATA_W*LANES_NUM-1:0] r_vec_o,

  // 결과와 함께 내보내는 전역 행 인덱스
  output reg  [ROW_W-1:0]             row_idx_o
);

  localparam integer ELEMS = LANES_NUM*LANES_NUM;

  // 현재 스케일 행렬 보유
  reg [FP_MANT_W*ELEMS-1:0] mant_mat_reg;
  reg [FP_EXP_W *ELEMS-1:0]  exp_mat_reg;
  reg                        have_scale;
  reg [31:0]                 row_in_mat; // 0..LANES_NUM-1

  // 전역 행 카운터
  reg [ROW_W-1:0] row_cnt;

  // stage0 (출력 버퍼 역할)
  reg                           s0_valid;      // *** CHG: 제대로 up/down
  reg [FP_DATA_W*LANES_NUM-1:0] s0_acc_vec;
  reg [FP_MANT_W*LANES_NUM-1:0] s0_mant_row;
  reg [FP_EXP_W *LANES_NUM-1:0] s0_exp_row;
  reg [ROW_W-1:0]               s0_row_idx;

  // dequant 결과 (조합)
  wire [FP_DATA_W*LANES_NUM-1:0] r_vec_next;

  // 핸드셰이크
  wire fire_out;  // *** CHG
  wire fire_in;   // *** CHG
  wire s0_ready;  // *** CHG

  assign fire_out = s0_valid & out_ready;                 // 소비 시점
  assign s0_ready = (~s0_valid) | fire_out;               // 1-entry 버퍼 공식
  assign fire_in  = acc_row_valid & s0_ready & have_scale;

  // FIFO(스케일 행렬) 읽기
  assign fifo_rd_ready = (~have_scale) & s0_ready & fifo_rd_valid;

  // ACC 수락 가능
  assign acc_row_ready = s0_ready & have_scale;

  // 스케일 행 슬라이싱 (조합)
  integer c;
  integer idx;

  reg [FP_MANT_W*LANES_NUM-1:0] comb_mant_row;
  reg [FP_EXP_W *LANES_NUM-1:0] comb_exp_row;

  always @* begin
    comb_mant_row = {FP_MANT_W*LANES_NUM{1'b0}};
    comb_exp_row  = {FP_EXP_W *LANES_NUM{1'b0}};

    if (have_scale) begin
      for (c = 0; c < LANES_NUM; c = c + 1) begin
        idx = row_in_mat*LANES_NUM + c;
        // mantissa
        comb_mant_row[(c+1)*FP_MANT_W-1 -: FP_MANT_W]
          = mant_mat_reg[idx*FP_MANT_W +: FP_MANT_W];
        // exponent
        comb_exp_row[(c+1)*FP_EXP_W-1 -: FP_EXP_W]
          = exp_mat_reg[idx*FP_EXP_W +: FP_EXP_W];
      end
    end
  end

  // 시퀀셜
  always @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      have_scale   <= 1'b0;
      mant_mat_reg <= '0;
      exp_mat_reg  <= '0;
      row_in_mat   <= 32'd0;

      s0_valid     <= 1'b0;
      s0_acc_vec   <= '0;
      s0_mant_row  <= '0;
      s0_exp_row   <= '0;
      s0_row_idx   <= '0;

      out_valid    <= 1'b0;
      r_vec_o      <= '0;
      row_idx_o    <= '0;
      row_cnt      <= '0;

    end else if (clear) begin
      have_scale   <= 1'b0;
      row_in_mat   <= 32'd0;

      s0_valid     <= 1'b0;

      out_valid    <= 1'b0;

      row_cnt      <= '0;

    end else begin
      // (B) 스케일 행렬 로드
      if ((!have_scale) && s0_ready && fifo_rd_valid) begin
        mant_mat_reg <= fifo_mant_dout;
        exp_mat_reg  <= fifo_exp_dout;
        have_scale   <= 1'b1;
        row_in_mat   <= 32'd0;
      end

      // (C) 소비/수락 순서: bubble-free 갱신
      // 1) 소비되면 기본적으로 s0_valid를 내리되,
      // 2) 같은 클럭에 새 입력을 받으면 유지(교체)
      if (fire_out && !fire_in)
        s0_valid <= 1'b0;

      if (fire_in) begin
        s0_acc_vec  <= acc_vec_i;
        s0_mant_row <= comb_mant_row;
        s0_exp_row  <= comb_exp_row;
        s0_row_idx  <= row_cnt;
        s0_valid    <= 1'b1; // 새 페이로드 적재
        // 전역 행 카운터
        if (row_cnt == ROWS_MAX[ROW_W-1:0]-1)
          row_cnt <= '0;
        else
          row_cnt <= row_cnt + {{(ROW_W-1){1'b0}}, 1'b1};

        // 다음 행 준비
        if (row_in_mat == (LANES_NUM-1)) begin
          row_in_mat <= 32'd0;
          have_scale <= 1'b0;  // 다음 행렬 필요
        end else begin
          row_in_mat <= row_in_mat + 32'd1;
        end
      end

      // (D) 출력: s0(레지스터) 내용을 그대로 내보냄
      out_valid <= s0_valid;          // *** CHG: 단일 지점에서만 갱신
      r_vec_o   <= r_vec_next;        // s0_* 기반 조합결과
      row_idx_o <= s0_row_idx;
    end
  end

  // dequantize_vector
  dequantize_vector #(
    .FP_DATA_W   (FP_DATA_W),
    .FP_MANT_W   (FP_MANT_W),
    .FP_EXP_W    (FP_EXP_W),
    .FP_EXP_BIAS (FP_EXP_BIAS),
    .BIT_NUM     (BIT_NUM),
    .LANES_NUM   (LANES_NUM)
  ) u_dequantize_vector (
    .acc_vec_i            (s0_acc_vec),
    .mantissa_scale_vec_i (s0_mant_row),
    .exp_scale_vec_i      (s0_exp_row),
    .r_vec_o              (r_vec_next)
  );

endmodule
