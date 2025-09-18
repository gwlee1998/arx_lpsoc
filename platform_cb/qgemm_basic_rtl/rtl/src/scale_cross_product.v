module scale_cross_product #(
    parameter integer MAT_SIZE    = 16,
    parameter integer FP_MANT_W   = 23,
    parameter integer FP_EXP_W    = 8,
    parameter integer FP_EXP_BIAS = 127
)(
    input  wire [FP_MANT_W*MAT_SIZE-1:0]          vec1_mant,
    input  wire [FP_MANT_W*MAT_SIZE-1:0]          vec2_mant,
    input  wire [FP_EXP_W*MAT_SIZE-1:0]           vec1_exp_raw,
    input  wire [FP_EXP_W*MAT_SIZE-1:0]           vec2_exp_raw,
    output wire [FP_MANT_W*MAT_SIZE*MAT_SIZE-1:0] mant_mat_o,
    output wire [FP_EXP_W*MAT_SIZE*MAT_SIZE-1:0]  exp_mat_raw_o
);
    // Mantissa 외적: 1.m × 1.m → 정규화 + bump 생성
    wire [FP_MANT_W*MAT_SIZE*MAT_SIZE-1:0] mant_mat_w;
    wire [MAT_SIZE*MAT_SIZE-1:0]             bump_w;

    mts_cross_product #(
        .MAT_SIZE_1  (MAT_SIZE),
        .MAT_SIZE_2  (MAT_SIZE),
        .FP_MANT_W (FP_MANT_W)
    ) mts_cross_product_i (
        .vec_1       (vec1_mant),
        .vec_2       (vec2_mant),
        .mant_matrix (mant_mat_w),
        .bump_matrix (bump_w)
    );

    // Exponent 외적: RAW 규칙(e1+e2-FP_EXP_BIAS+bump)
    exp_cross_product #(
        .MAT_SIZE_1 (MAT_SIZE),
        .MAT_SIZE_2 (MAT_SIZE),
        .FP_EXP_W   (FP_EXP_W),
        .FP_EXP_BIAS  (FP_EXP_BIAS)
    ) exp_cross_product_i (
        .vec_1_raw      (vec1_exp_raw),
        .vec_2_raw      (vec2_exp_raw),
        .bump_matrix    (bump_w),
        .out_matrix_raw (exp_mat_raw_o)
    );

    assign mant_mat_o = mant_mat_w;

endmodule
