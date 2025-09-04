module mts_cross_product #(
    parameter integer MAT_SIZE_1   = 16,
    parameter integer MAT_SIZE_2   = 16,
    parameter integer FP_MANT_W    = 23
)(
    input  wire [FP_MANT_W*MAT_SIZE_1-1:0]                vec_1,      // 23b fractions
    input  wire [FP_MANT_W*MAT_SIZE_2-1:0]                vec_2,      // 23b fractions
    output wire [FP_MANT_W*MAT_SIZE_1*MAT_SIZE_2-1:0]     mant_matrix,// 23b fractions (normalized)
    output wire [MAT_SIZE_1*MAT_SIZE_2-1:0]               bump_matrix // 1 if mant>=2.0
);
    localparam integer MANT_W = FP_MANT_W + 1; // 1 + 23 = 24 (1.xxx 형식)

    genvar i, j;
    generate
        for (i = 0; i < MAT_SIZE_1; i = i + 1) begin : ROW
            wire [FP_MANT_W-1:0] m1_frac = vec_1[(i+1)*FP_MANT_W-1 -: FP_MANT_W];
            wire [MANT_W-1:0]      m1_uq   = {1'b1, m1_frac};  // 1.xxx
            for (j = 0; j < MAT_SIZE_2; j = j + 1) begin : COL
                localparam integer OUT_IDX  = i*MAT_SIZE_2 + j;
                localparam integer OUT_LSBM = OUT_IDX*FP_MANT_W;
                localparam integer OUT_MSBM = OUT_LSBM + FP_MANT_W - 1;

                wire [FP_MANT_W-1:0] m2_frac = vec_2[(j+1)*FP_MANT_W-1 -: FP_MANT_W];
                wire [MANT_W-1:0]      m2_uq   = {1'b1, m2_frac}; // 1.xxx

                // 1.23 × 1.23 = 2.46 → 48b
                wire [2*MANT_W-1:0] prod = m1_uq * m2_uq;

                // >>23 → 2.23 (정수부 2비트, 소수부 23비트), 총 25비트
                wire [FP_MANT_W+1:0] pre25 = prod[2*MANT_W-1 -: (FP_MANT_W+2)];
                // MSB == 1 이면 [2.0,4.0) → 한 비트 내리고 bump=1
                wire bump = pre25[FP_MANT_W+1];
                wire [MANT_W-1:0] norm24 = bump ? pre25[FP_MANT_W+1:1] : pre25[FP_MANT_W:0];

                assign bump_matrix[OUT_IDX] = bump;
                assign mant_matrix[OUT_MSBM:OUT_LSBM] = norm24[FP_MANT_W-1:0]; // 히든비트 제외한 23b만 보관
            end
        end
    endgenerate
endmodule
