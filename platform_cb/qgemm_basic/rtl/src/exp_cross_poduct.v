module exp_cross_product #(
    parameter integer MAT_SIZE_1  = 16,
    parameter integer MAT_SIZE_2  = 16,
    parameter integer FP_EXP_W    = 8,
    parameter integer FP_EXP_BIAS = 127
)(
    input  wire [FP_EXP_W*MAT_SIZE_1-1:0]                 vec_1_raw,   // RAW exponents
    input  wire [FP_EXP_W*MAT_SIZE_2-1:0]                 vec_2_raw,   // RAW exponents
    input  wire [MAT_SIZE_1*MAT_SIZE_2-1:0]             bump_matrix, // from mantissa norm
    output wire [FP_EXP_W*MAT_SIZE_1*MAT_SIZE_2-1:0]      out_matrix_raw // RAW exponents
);
    localparam integer RAW_MAX_NORM = (1<<FP_EXP_W) - 2;  // 254 for FP32

    genvar i, j;
    generate
        for (i = 0; i < MAT_SIZE_1; i = i + 1) begin : ROW
            wire [FP_EXP_W-1:0] e1_raw = vec_1_raw[(i+1)*FP_EXP_W-1 -: FP_EXP_W];
            for (j = 0; j < MAT_SIZE_2; j = j + 1) begin : COL
                localparam integer OUT_IDX  = i*MAT_SIZE_2 + j;
                localparam integer OUT_LSBE = OUT_IDX*FP_EXP_W;
                localparam integer OUT_MSBE = OUT_LSBE + FP_EXP_W - 1;

                wire [FP_EXP_W-1:0] e2_raw = vec_2_raw[(j+1)*FP_EXP_W-1 -: FP_EXP_W];
                wire               bump   = bump_matrix[OUT_IDX];

                // e_out_raw = e1_raw + e2_raw - FP_EXP_BIAS + bump (RAW 규칙)
                // 중간폭 넉넉히: FP_EXP_W+2
                wire [FP_EXP_W+1:0] sum_ext = {2'b00, e1_raw} + {2'b00, e2_raw} + { {(FP_EXP_W){1'b0}}, bump };
                // 뺄셈 후 포화
                // signed 확장해서 음수/양수 체크
                wire signed [FP_EXP_W+2:0] sub_ext = $signed({1'b0, sum_ext}) - $signed(FP_EXP_BIAS);

                // saturation to [0 .. RAW_MAX_NORM]
                wire [FP_EXP_W-1:0] sat_hi = RAW_MAX_NORM[FP_EXP_W-1:0];
                wire [FP_EXP_W-1:0] raw_clip =
                    (sub_ext <= 0)                 ? {FP_EXP_W{1'b0}} :
                    (sub_ext > RAW_MAX_NORM)      ? sat_hi         :
                                                    sub_ext[FP_EXP_W-1:0];

                assign out_matrix_raw[OUT_MSBE:OUT_LSBE] = raw_clip;
            end
        end
    endgenerate
endmodule
