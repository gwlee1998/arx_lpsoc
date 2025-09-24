`timescale 1ns/1ps

// ============================================================================
// QDQ Controller
// - Submodules:
//    * quantize_array
//    * dequantize_array
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
    parameter integer SCALE_FIFO_DEPTH = 4,
    parameter integer ROW_W_DQ         = 4
)(
    input  wire                           clk,
    input  wire                           rstnn,

    // start
    input  wire                           q_start_i,
    input  wire                           dq_start_i,

    // A tile → quantize
    input  wire                           a_s_valid_i,
    output wire                           a_s_ready_o,
    input  wire [LANES_NUM*FP_DATA_W-1:0] a_s_data_i,

    output wire                           a_m_valid_o,
    input  wire                           a_m_ready_i,
    output wire [LANES_NUM*FP_DATA_W-1:0] a_m_data_o,

    // B tile → quantize
    input  wire                           b_s_valid_i,
    output wire                           b_s_ready_o,
    input  wire [LANES_NUM*FP_DATA_W-1:0] b_s_data_i,

    output wire                           b_m_valid_o,
    input  wire                           b_m_ready_i,
    output wire [LANES_NUM*FP_DATA_W-1:0] b_m_data_o,

    // ACC tile → dequantize
    input  wire                           dq_s_valid_i,
    output wire                           dq_s_ready_o,
    input  wire [LANES_NUM*FP_DATA_W-1:0] dq_s_data_i,

    // dequantized FP out
    output wire                           dq_m_valid_o,
    input  wire                           dq_m_ready_i,
    output wire [LANES_NUM*FP_DATA_W-1:0] dq_m_data_o,
    output wire [ROW_W_DQ-1:0]            dq_m_index_o
);
    // --------------------------------------------
    // utils
    function integer clog2; input integer v; integer i; begin i=0; while((1<<i)<v) i=i+1; clog2=i; end endfunction
    localparam integer ELEMS = MAT_SIZE*MAT_SIZE;

    // --------------------------------------------
    // 1) Quantize path (A/B)

    // ---- A
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
        .start_i          (q_start_i),
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

    // ---- B
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
        .start_i          (q_start_i),
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

    // cross-product → 행렬 스케일
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
    // 3) scale_fifo (타일 단위 저장)
    wire fifo_wr_ready, fifo_rd_valid, fifo_rd_ready;
    wire [FP_MANT_W*ELEMS-1:0] fifo_mant_dout;
    wire [FP_EXP_W *ELEMS-1:0] fifo_exp_dout;

    wire xp_push = a_hold_v & b_hold_v & fifo_wr_ready;

    scale_fifo #(
        .MAT_SIZE   (MAT_SIZE),
        .FP_MANT_W  (FP_MANT_W),
        .FP_EXP_W   (FP_EXP_W),
        .DEPTH      (SCALE_FIFO_DEPTH)
    ) scale_fifo_i (
        .clk            (clk),
        .rstnn          (rstnn),
        .wr_valid_i     (xp_push),
        .wr_ready_o     (fifo_wr_ready),
        .mant_in_i      (mant_mat_w),
        .exp_in_i       (exp_mat_w),
        .rd_valid_o     (fifo_rd_valid),   // show-ahead
        .rd_ready_i     (fifo_rd_ready),   // pop pulse
        .mant_out_o     (fifo_mant_dout),
        .exp_out_o      (fifo_exp_dout),
        .flush_i        (1'b0),
        .empty_o        (),
        .full_o         (),
        .almost_empty_o (),
        .almost_full_o  ()
    );

    // push 성공시에만 홀드 해제
    always @(posedge clk or negedge rstnn) begin
        if (!rstnn) begin
            // no-op
        end else if (xp_push) begin
            a_hold_v <= 1'b0;
            b_hold_v <= 1'b0;
        end
    end

    // --------------------------------------------
    // 4) Dequantize path (+ dq_start 게이팅)
    wire                          dqcore_acc_ready_w;
    wire                          dqcore_out_valid_w;
    wire [LANES_NUM*FP_DATA_W-1:0] dqcore_out_data_w;
    wire [3:0]                    dqcore_row_idx_w;

    reg dq_en;
    always @(posedge clk or negedge rstnn) begin
      if (!rstnn) dq_en <= 1'b0;
      else if (dq_start_i) dq_en <= 1'b1;    // 필요 시 완료 지점에서 0으로 내릴 수 있음
    end

    // dequantize_row_sync
    assign dq_m_valid_o = dqcore_out_valid_w;
    assign dq_m_data_o  = dqcore_out_data_w;
    assign dq_m_index_o = dqcore_row_idx_w;
    wire dqcore_out_ready_w = dq_m_ready_i;

    assign dq_s_ready_o = dq_en ? dqcore_acc_ready_w : 1'b0;

    dequantize_row_sync #(
      .FP_DATA_W   (FP_DATA_W),
      .FP_MANT_W   (FP_MANT_W),
      .FP_EXP_W    (FP_EXP_W),
      .FP_EXP_BIAS (FP_EXP_BIAS),
      .BIT_NUM     (BIT_NUM),

      .LANES_NUM   (LANES_NUM),      // MAT_SIZE == LANES_NUM 가정
      .ROWS_MAX    (1<<ROW_W_DQ),    // 전역 행 인덱스 래핑 폭
      .ROW_W       (ROW_W_DQ)
    ) dequantize_row_sync_i (
      .clk          (clk),
      .rstn         (rstnn),
      .clear        (1'b0),

      // ACC 행 입력 스트림
      .acc_row_valid (dq_s_valid_i),
      .acc_row_ready (dqcore_acc_ready_w),
      .acc_vec_i     (dq_s_data_i),

      // 스케일 행렬 입력 (scale_fifo 의 show-ahead 출력)
      .fifo_rd_valid  (fifo_rd_valid),
      .fifo_rd_ready  (fifo_rd_ready),
      .fifo_mant_dout (fifo_mant_dout),
      .fifo_exp_dout  (fifo_exp_dout),

      // 결과 스트림 (행 단위)
      .out_valid (dqcore_out_valid_w),
      .out_ready (dqcore_out_ready_w),
      .r_vec_o   (dqcore_out_data_w),

      // 전역 행 인덱스(옵션)
      .row_idx_o (dqcore_row_idx_w)
    );

endmodule
