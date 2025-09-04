module quantize_elem #(
    parameter FP_DATA_W   = 32,
    parameter FP_MANT_W   = 23,
    parameter FP_EXP_W    = 8,
    parameter FP_EXP_BIAS = 127,
    parameter BIT_NUM     = 8,
    parameter DE_MARGIN   = 16
)(
    input  wire [FP_DATA_W-1:0] r_data,
    input  wire [FP_MANT_W-1:0] mantissa_scale,
    input  wire [FP_EXP_W-1:0]  exp_scale,
    output wire [BIT_NUM-1:0] q_data
);

    localparam integer PREC     = FP_MANT_W; 
    localparam integer MANT_W   = FP_MANT_W + 1;
    // 중간폭: (Mx × Q) → MANT_W + BIT_NUM, <<PREC, + DE_MARGIN 여유
    localparam integer WIDE     = MANT_W + BIT_NUM + PREC + DE_MARGIN;

    wire signed [BIT_NUM-1:0] QMAX_S = {1'b0, {(BIT_NUM-1){1'b1}}};
    wire signed [BIT_NUM-1:0] QMIN_S = {1'b1, {(BIT_NUM-1){1'b0}}};

    wire [BIT_NUM-1:0] QMAX_U = {1'b0, {(BIT_NUM-1){1'b1}}};

    // ------------------------------------------------------------
    // [1] 입력 분해 (IEEE754 → 부호/지수/맨티사)
    wire                    x_sign   = r_data[FP_DATA_W-1];
    wire [FP_EXP_W-1:0]       x_expraw = r_data[FP_MANT_W+FP_EXP_W-1 : FP_MANT_W];
    wire signed [FP_EXP_W:0]  x_exp    = $signed({1'b0, x_expraw}) - FP_EXP_BIAS;
    wire [FP_MANT_W-1:0]  x_frac   = r_data[FP_MANT_W-1:0];
    wire [MANT_W-1:0]       x_mant   = {1'b1, x_frac}; // UQ1.(FP_MANT_W)

    // x==0이면 바로 0 출력
    wire x_is_zero = (r_data[FP_DATA_W-2:0] == {(FP_DATA_W-1){1'b0}});

    // ------------------------------------------------------------
    // [2] absmax A 분해
    wire [MANT_W-1:0]       A_mant   = {1'b1, mantissa_scale};           // UQ1.(FP_MANT_W)
    wire signed [FP_EXP_W:0]  A_exp    = $signed({1'b0, exp_scale}) - FP_EXP_BIAS;

    // 스케일이 0인 경우 방어(이론상 불가능하지만)
    wire A_is_zero = (A_mant == {MANT_W{1'b0}});

    // ------------------------------------------------------------
    // [3] ratio = round( (Mx * Q * 2^PREC) / MA )
    wire [WIDE-1:0] mul_mxQ     = x_mant * QMAX_U;
    wire [WIDE-1:0] ratio_num   = mul_mxQ << PREC;
    wire [WIDE-1:0] A_mant_ext  = {{(WIDE-MANT_W){1'b0}}, A_mant};

    // (N + D/2) / D  → 정수 반올림 나눗셈
    wire [WIDE-1:0] ratio_round = (ratio_num + (A_mant_ext >> 1)) / A_mant_ext; // ≈ Q * (mx/mA) * 2^PREC

    // ------------------------------------------------------------
    // [4] 지수차 반영: de = ex - eA
    //     (ratio_round << de) 후, 최종적으로 >> PREC (반올림)
    wire signed [FP_EXP_W:0] de_s      = x_exp - A_exp;
    wire                   de_ge0    = ~de_s[FP_EXP_W]; // MSB=0 → >=0
    wire [FP_EXP_W:0]        de_abs    = de_ge0 ? de_s[FP_EXP_W:0] : (~de_s + 1'b1);
    wire [WIDE-1:0] rshift_add_de = (de_abs == 0) ? {WIDE{1'b0}}
                                                  : ({{(WIDE-1){1'b0}},1'b1} << (de_abs - 1));
    wire [WIDE-1:0] val_after_de  = de_ge0
        ? ( (de_abs >= WIDE) ? {WIDE{1'b1}} : (ratio_round << de_abs) )
        : ( (de_abs == 0) ? ratio_round : ((ratio_round + rshift_add_de) >> de_abs) );
    wire [WIDE-1:0] round_prec  = (PREC == 0) ? {WIDE{1'b0}}
                                              : ({{(WIDE-1){1'b0}},1'b1} << (PREC-1));
    wire [WIDE-1:0] q_mag_scaled = (val_after_de + round_prec) >> PREC;

    // ------------------------------------------------------------
    // [5] 크기 포화(|q| > QMAX → QMAX), 부호 적용
    wire [WIDE-1:0] QMAX_WIDE     = {{(WIDE-BIT_NUM){1'b0}}, QMAX_U};
    wire [WIDE-1:0] q_mag_clamped = (q_mag_scaled > QMAX_WIDE) ? QMAX_WIDE : q_mag_scaled;

    wire signed [WIDE:0] q_signed_full = x_sign
        ? -$signed({1'b0, q_mag_clamped})
        :  $signed({1'b0, q_mag_clamped});

    wire signed [BIT_NUM-1:0] q_raw = q_signed_full[BIT_NUM-1:0];
    wire signed [BIT_NUM-1:0] q_sat =
        (q_raw > QMAX_S) ? QMAX_S :
        (q_raw < QMIN_S) ? QMIN_S :
                           q_raw;

    // ------------------------------------------------------------
    // [6] 출력
    assign q_data = (x_is_zero || A_is_zero) ? {BIT_NUM{1'b0}} : q_sat;

endmodule