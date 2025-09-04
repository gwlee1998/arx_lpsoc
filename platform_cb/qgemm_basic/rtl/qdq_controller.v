`timescale 1ns/1ps

// ============================================================================
// QDQ Controller
// - 하위 모듈:
//    * quantize_array
//    * dequantize_vector
//    * scale_cross_product
//    * scale_fifo
// ============================================================================

module qdq_controller #(
    parameter integer BIT_NUM          = 8,
    parameter integer MAT_SIZE         = 16,
    parameter integer FP_DATA_W        = 32,
    parameter integer FP_EXP_W         = 8,
    parameter integer FP_MANT_W        = 23,
    parameter integer FP_EXP_BIAS      = 127,
    parameter integer LANES_NUM        = 16,
    parameter integer SCALE_FIFO_DEPTH = 4
)(
    input  wire                         clk,
    input  wire                         rstnn,

    // A tile → quantize
    input  wire                         a_s_valid_i,
    output wire                         a_s_ready_o,
    input  wire [LANES_NUM*FP_DATA_W-1:0] a_s_data_i,

    output wire                         a_m_valid_o,
    input  wire                         a_m_ready_i,
    output wire [LANES_NUM*FP_DATA_W-1:0] a_m_data_o,

    // B tile → quantize
    input  wire                         b_s_valid_i,
    output wire                         b_s_ready_o,
    input  wire [LANES_NUM*FP_DATA_W-1:0] b_s_data_i,

    output wire                         b_m_valid_o,
    input  wire                         b_m_ready_i,
    output wire [LANES_NUM*FP_DATA_W-1:0] b_m_data_o,

    // ACC tile → dequantize
    input  wire                         dq_s_valid_i,
    output wire                         dq_s_ready_o,
    input  wire                         dq_tfirst_i,    // 타일 첫 beat 표시
    input  wire [LANES_NUM*FP_DATA_W-1:0] dq_s_data_i,  // (정수/acc 포맷. 설계에 맞게 연결)

    // dequantized FP out
    output wire                         dq_m_valid_o,
    input  wire                         dq_m_ready_i,
    output wire [LANES_NUM*FP_DATA_W-1:0] dq_m_data_o
);
    // --------------------------------------------
    // utils
    function integer clog2; input integer v; integer i; begin i=0; while((1<<i)<v) i=i+1; clog2=i; end endfunction
    localparam integer ELEMS   = MAT_SIZE*MAT_SIZE;
    localparam integer BEATS   = (ELEMS + LANES_NUM - 1)/LANES_NUM;

    // --------------------------------------------
    // 1) Quantize path (A/B)

    // A
    wire                           a_scl_valid_w, a_scl_ready_w;
    wire [FP_MANT_W*MAT_SIZE-1:0]  a_mant_vec_w;
    wire [FP_EXP_W *MAT_SIZE-1:0]  a_exp_vec_w;
    wire                           a_q_valid_w, a_q_ready_w;
    wire [LANES_NUM*FP_DATA_W-1:0] a_q_data_w;

    quantize_array #(
        .BIT_NUM     (BIT_NUM),
        .MAT_SIZE    (MAT_SIZE),
        .FP_DATA_W   (FP_DATA_W),
        .FP_EXP_W    (FP_EXP_W),
        .FP_MANT_W   (FP_MANT_W),
        .FP_EXP_BIAS (FP_EXP_BIAS),
        .LANES_NUM   (LANES_NUM)
    ) A_quantize_array_i (
        .clk              (clk),
        .rstnn            (rstnn),
        .s_valid_i        (a_s_valid_i),
        .s_ready_o        (a_s_ready_o),
        .s_data_i         (a_s_data_i),
        .scl_valid_o      (a_scl_valid_w),
        .scl_ready_i      (a_scl_ready_w),
        .mantissa_scale_o (a_mant_vec_w),
        .exp_scale_o      (a_exp_vec_w),
        .m_valid_o        (a_q_valid_w),
        .m_ready_i        (a_q_ready_w),
        .m_data_o         (a_q_data_w)
    );
    assign a_m_valid_o = a_q_valid_w;
    assign a_q_ready_w = a_m_ready_i;
    assign a_m_data_o  = a_q_data_w;

    // B
    wire                           b_scl_valid_w, b_scl_ready_w;
    wire [FP_MANT_W*MAT_SIZE-1:0]  b_mant_vec_w;
    wire [FP_EXP_W *MAT_SIZE-1:0]  b_exp_vec_w;
    wire                           b_q_valid_w, b_q_ready_w;
    wire [LANES_NUM*FP_DATA_W-1:0] b_q_data_w;

    quantize_array #(
        .BIT_NUM     (BIT_NUM),
        .MAT_SIZE    (MAT_SIZE),
        .FP_DATA_W   (FP_DATA_W),
        .FP_EXP_W    (FP_EXP_W),
        .FP_MANT_W   (FP_MANT_W),
        .FP_EXP_BIAS (FP_EXP_BIAS),
        .LANES_NUM   (LANES_NUM)
    ) B_quantize_array_i (
        .clk              (clk),
        .rstnn            (rstnn),
        .s_valid_i        (b_s_valid_i),
        .s_ready_o        (b_s_ready_o),
        .s_data_i         (b_s_data_i),
        .scl_valid_o      (b_scl_valid_w),
        .scl_ready_i      (b_scl_ready_w),
        .mantissa_scale_o (b_mant_vec_w),
        .exp_scale_o      (b_exp_vec_w),
        .m_valid_o        (b_q_valid_w),
        .m_ready_i        (b_q_ready_w),
        .m_data_o         (b_q_data_w)
    );
    assign b_m_valid_o = b_q_valid_w;
    assign b_q_ready_w = b_m_ready_i;
    assign b_m_data_o  = b_q_data_w;

    // --------------------------------------------
    // 2) 스케일 벡터 래치 → cross-product 계산
    reg                          a_hold_v, b_hold_v;
    reg [FP_MANT_W*MAT_SIZE-1:0] a_mant_hold, b_mant_hold;
    reg [FP_EXP_W *MAT_SIZE-1:0] a_exp_hold,  b_exp_hold;

    assign a_scl_ready_w = (~a_hold_v);
    assign b_scl_ready_w = (~b_hold_v);

    always @(posedge clk or negedge rstnn) begin
        if (!rstnn) begin
            a_hold_v    <= 1'b0;
            b_hold_v    <= 1'b0;
            a_mant_hold <= {FP_MANT_W*MAT_SIZE{1'b0}};
            a_exp_hold  <= {FP_EXP_W *MAT_SIZE{1'b0}};
            b_mant_hold <= {FP_MANT_W*MAT_SIZE{1'b0}};
            b_exp_hold  <= {FP_EXP_W *MAT_SIZE{1'b0}};
        end else begin
            if (a_scl_valid_w && a_scl_ready_w) begin
                a_hold_v    <= 1'b1;
                a_mant_hold <= a_mant_vec_w;
                a_exp_hold  <= a_exp_vec_w;
            end
            if (b_scl_valid_w && b_scl_ready_w) begin
                b_hold_v    <= 1'b1;
                b_mant_hold <= b_mant_vec_w;
                b_exp_hold  <= b_exp_vec_w;
            end
        end
    end

    // cross-product
    wire [FP_MANT_W*ELEMS-1:0] mant_mat_w;
    wire [FP_EXP_W *ELEMS-1:0] exp_mat_w;

    scale_cross_product #(
        .MAT_SIZE     (MAT_SIZE),
        .FP_MANT_W    (FP_MANT_W),
        .FP_EXP_W     (FP_EXP_W),
        .FP_EXP_BIAS  (FP_EXP_BIAS)
    ) scale_cross_product_i (
        .vec1_mant     (a_mant_hold),
        .vec2_mant     (b_mant_hold),
        .vec1_exp_raw  (a_exp_hold),
        .vec2_exp_raw  (b_exp_hold),
        .mant_mat_o    (mant_mat_w),
        .exp_mat_raw_o (exp_mat_w)
    );

    // --------------------------------------------
    // 3) scale_fifo 로 계산 결과 저장
    wire fifo_wr_ready, fifo_rd_valid, fifo_rd_ready;
    wire [FP_MANT_W*ELEMS-1:0] fifo_mant_dout;
    wire [FP_EXP_W *ELEMS-1:0] fifo_exp_dout;
    wire fifo_empty, fifo_full, fifo_aempty, fifo_afull;

    wire xp_push = a_hold_v & b_hold_v & fifo_wr_ready;

    scale_fifo #(
        .MAT_SIZE   (MAT_SIZE),
        .FP_MANT_W  (FP_MANT_W),
        .FP_EXP_W   (FP_EXP_W),
        .DEPTH      (SCALE_FIFO_DEPTH)
    ) scale_fifo_i (
        .clk          (clk),
        .rstnn        (rstnn),
        .wr_valid_i   (xp_push),
        .wr_ready_o   (fifo_wr_ready),
        .mant_in_i    (mant_mat_w),
        .exp_in_i     (exp_mat_w),
        .rd_valid_o   (fifo_rd_valid),
        .rd_ready_i   (fifo_rd_ready),
        .mant_out_o   (fifo_mant_dout),
        .exp_out_o    (fifo_exp_dout),
        .flush_i      (1'b0),
        .empty_o      (fifo_empty),
        .full_o       (fifo_full),
        .almost_empty_o (fifo_aempty),
        .almost_full_o  (fifo_afull)
    );

    // push가 실제 일어났을 때 홀드 해제
    always @(posedge clk or negedge rstnn) begin
        if (!rstnn) begin
            // 이미 초기화됨
        end else begin
            if (xp_push) begin
                a_hold_v <= 1'b0;
                b_hold_v <= 1'b0;
            end
        end
    end

    // --------------------------------------------
    // 4) Dequantize path (LANES 단위로 스케일 슬라이스)

    // 진행 상태/비트수 관리만 남김
    reg                  dq_active;
    reg [clog2(BEATS):0] dq_beats_left;

    // 시작/진행 비트
    wire dq_start = (~dq_active) & fifo_rd_valid & dq_s_valid_i & dq_tfirst_i & dq_m_ready_i;
    wire dq_step  = (dq_active)  & dq_s_valid_i & dq_m_ready_i;

    // FIFO pop은 시작 사이클에만
    assign fifo_rd_ready = dq_start;

    // upstream ready: idle→(FIFO있고 tfirst고 소비가능) / active→(소비가능)
    assign dq_s_ready_o  = ( (~dq_active) ? (fifo_rd_valid & dq_tfirst_i & dq_m_ready_i)
                                          : (dq_m_ready_i) );

    // 진행 카운트
    always @(posedge clk or negedge rstnn) begin
      if (!rstnn) begin
        dq_active     <= 1'b0;
        dq_beats_left <= '0;
      end else begin
        if (dq_start) begin
          dq_beats_left <= BEATS[clog2(BEATS):0];
          dq_active     <= 1'b1;
        end else if (dq_step) begin
          dq_beats_left <= dq_beats_left - 1'b1;
          if (dq_beats_left == 1) dq_active <= 1'b0;
        end
      end
    end

    // === 스케일/ACC beat 동기화 ===
    wire [LANES_NUM*FP_MANT_W-1:0] cur_mant_lanes;
    wire [LANES_NUM*FP_EXP_W -1:0] cur_exp_lanes;

    scale_acc_sync #(
      .LANES_NUM (LANES_NUM),
      .FP_MANT_W (FP_MANT_W),
      .FP_EXP_W  (FP_EXP_W),
      .ELEMS     (ELEMS)
    ) scale_acc_sync_i (
      .clk                (clk),
      .rstnn              (rstnn),
      .load_i             (dq_start),            // pop하는 그 사이클
      .step_i             (dq_step),             // 매 beat 소비
      .fifo_mant_full_i   (fifo_mant_dout),      // show-ahead head
      .fifo_exp_full_i    (fifo_exp_dout),
      .cur_mant_lanes_o   (cur_mant_lanes),      // dequant에 바로 사용
      .cur_exp_lanes_o    (cur_exp_lanes)
    );

    // Dequantize (조합/레지스터 옵션은 dequantize_vector 내부 설정에 따름)
    dequantize_vector #(
      .BIT_NUM(BIT_NUM), .FP_DATA_W(FP_DATA_W),
      .FP_MANT_W(FP_MANT_W), .FP_EXP_W(FP_EXP_W),
      .FP_EXP_BIAS(FP_EXP_BIAS), .LANES_NUM(LANES_NUM)
    ) dequantize_vector_i (
      .q_data_i        (dq_s_data_i),    // 현재 beat의 ACC
      .mantissa_scale_i(cur_mant_lanes), // 현재 beat의 스케일
      .exp_scale_i     (cur_exp_lanes),
      .r_data_o        (dq_m_data_o)
    );

    // 출력 valid는 '현재 beat를 실제 소비하는 사이클'에만
    assign dq_m_valid_o = dq_step;

    /////////////////////////////////////////////////////////////
    // 디버깅용 로직
    /////////////////////////////////////////////////////////////
    // 플랫 입력 버스 -> 2차원 reg (웨이브 보기용)
    reg [FP_DATA_W-1:0] dq_data_i_reg [0:LANES_NUM-1];
    reg [FP_DATA_W-1:0] dq_data_o_reg [0:LANES_NUM-1];
    
    integer __r, __idx;
    always @* begin
        for (__r = 0; __r < LANES_NUM; __r = __r + 1) begin
            dq_data_i_reg[__r] = dq_s_data_i[(__r+1)*FP_DATA_W-1 -: FP_DATA_W];
            dq_data_o_reg[__r] = dq_m_data_o[(__r+1)*FP_DATA_W-1 -: FP_DATA_W];
        end
    end


endmodule
