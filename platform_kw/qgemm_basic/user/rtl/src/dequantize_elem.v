module dequantize_elem #(
    parameter FP_DATA_W   = 32,
    parameter FP_MANT_W   = 23,
    parameter FP_EXP_W    = 8,
    parameter FP_EXP_BIAS = 127,
    parameter BIT_NUM     = 8
)(
    input  wire signed [FP_DATA_W-1:0] acc_i,            // signed accumulator (integer)
    input  wire        [FP_MANT_W-1:0] mantissa_scale,   // S mantissa (hidden bit excluded)
    input  wire        [FP_EXP_W-1:0]  exp_scale,        // S exponent (RAW, biased)
    output wire        [FP_DATA_W-1:0] r_data            // IEEE754 float32
);

    // ---------- utils ----------
    function integer clog2;
        input integer v;
        integer i;
    begin
        i = 0;
        while ((1 << i) < v) i = i + 1;
        clog2 = i;
    end
    endfunction

    // ---------- constants ----------
    localparam integer MANT_W          = FP_MANT_W + 1; // 1.m (24 for FP32)
    localparam integer QMAX            = (1 << (BIT_NUM-1)) - 1; // e.g. 127
    localparam integer QSQ             = QMAX * QMAX;            // e.g. 16129
    localparam integer LOG2_QSQ_CEIL   = clog2(QSQ);             // 14 for 127*127

    // normalized exponent range (unbiased)
    localparam integer EXP_RAW_MIN_NORM = 1;
    localparam integer EXP_RAW_MAX_NORM = (1 << FP_EXP_W) - 2;
    localparam integer UNB_MIN          = EXP_RAW_MIN_NORM - FP_EXP_BIAS; // -126
    localparam integer UNB_MAX          = EXP_RAW_MAX_NORM - FP_EXP_BIAS; // +127

    // exponent scratch width
    localparam integer EW = FP_EXP_W + 6;

    // ---------- sign / |acc| ----------
    wire sign_acc;
    wire signed [FP_DATA_W:0] acc_ext;
    wire        [FP_DATA_W:0] acc_abs_w;
    wire        [FP_DATA_W-1:0] acc_abs;
    wire acc_is_zero;

    assign sign_acc   = acc_i[FP_DATA_W-1];
    assign acc_ext    = {acc_i[FP_DATA_W-1], acc_i}; // 1-bit extend
    assign acc_abs_w  = acc_ext[FP_DATA_W] ? (~acc_ext + 1'b1) : acc_ext;
    assign acc_abs    = acc_abs_w[FP_DATA_W-1:0];
    assign acc_is_zero= (acc_abs == {FP_DATA_W{1'b0}});

    // ---------- S split ----------
    wire [MANT_W-1:0] A_mant; // integer (1.m) scaled by 2^FP_MANT_W
    wire A_is_zero;
    integer A_exp_unb;

    assign A_mant   = {1'b1, mantissa_scale};
    assign A_is_zero= (exp_scale == {FP_EXP_W{1'b0}}) && (mantissa_scale == {FP_MANT_W{1'b0}});
    always @* begin
        A_exp_unb = exp_scale - FP_EXP_BIAS; // unbiased exponent of S
    end

    // ---------- integer product P = |acc| * (1.m) ----------
    localparam integer PW = FP_DATA_W + MANT_W; // product width
    wire [PW-1:0] P = acc_abs * A_mant; // unsigned multiply

    wire is_zero_path = acc_is_zero | A_is_zero | (P == {PW{1'b0}});

    // ---------- find MSB index of P (floor(log2 P)) ----------
    reg [31:0] msb_idx;
    integer k;
    reg found;

    always @* begin
        msb_idx = 0;
        found   = 1'b0;
        for (k = PW-1; k >= 0; k = k - 1) begin
            if (!found && P[k]) begin
                msb_idx = k;
                found   = 1'b1;
            end
        end
    end

    // ---------- normalize to 24-bit (1.h) with robust rounding ----------
    reg  [31:0] sr, sl;
    wire        shift_right = (msb_idx >= (MANT_W-1));
    wire [PW-1:0] P_shifted_r;
    wire        guard, sticky, round_up;
    wire [PW-1:0] lower_mask;
    wire [MANT_W:0]   mant_with_carry_r2;
    wire [MANT_W-1:0] mant24_right;
    reg  [MANT_W-1:0] mant24_left;
    wire [MANT_W-1:0] mant24_norm_pre;

    always @* begin
        if (shift_right) begin
            sr = msb_idx - (MANT_W-1);
            sl = 0;
        end else begin
            sr = 0;
            sl = (MANT_W-1) - msb_idx;
        end
    end

    assign P_shifted_r = P >> sr;

    // guard + sticky (변수 범위 슬라이스 금지: 마스크로 계산)
    assign guard      = (sr > 0) ? P[sr-1] : 1'b0;
    assign lower_mask = (sr > 1) ? ({PW{1'b1}} >> (PW - (sr-1))) : {PW{1'b0}}; // LSB에 (sr-1)개 1
    assign sticky     = |(P & lower_mask);

    // ties-to-even
    assign round_up         = guard & (sticky | P_shifted_r[0]);
    assign mant_with_carry_r2 =
        {1'b0, P_shifted_r[MANT_W-1:0]} + {{MANT_W{1'b0}}, round_up};

    // 캐리 발생 시 합산 결과 기준으로 정확히 1비트 이동
    assign mant24_right = mant_with_carry_r2[MANT_W] ?
                          mant_with_carry_r2[MANT_W:1] :
                          mant_with_carry_r2[MANT_W-1:0];

    // 왼쪽 경로: 상단 비트 슬라이싱 (변수 시프트 사용)
    always @* begin
        mant24_left = (P << sl) >> (PW - MANT_W);
    end

    assign mant24_norm_pre = shift_right ? mant24_right : mant24_left; // 24-bit (1.h)

    // ---------- (근사) /Q^2 ≈ 1/2^14 : 지수 -14만 반영 ----------
    localparam integer K_Q2 = LOG2_QSQ_CEIL; // 14

    // ---------- 가수부만 미세 보정: 1.01578 ~= 1 + 1/64 + 1/8192 + 1/32768 ----------
    // 1 + 1/64 + 1/8192 + 1/32768 = 1.0157775879... (1.01578과 오차 ~2.4e-06)
    wire [MANT_W+2:0] base  = {3'b000, mant24_norm_pre};
    wire [MANT_W+2:0] s6    = base >> 6;
    wire [MANT_W+2:0] s13   = base >> 13;
    wire [MANT_W+2:0] s15   = base >> 15;
    wire [MANT_W+2:0] adj_sum = base + s6 + s13 + s15;

    // 캐리 정규화 (2.x → 1.x) 및 캐리 플래그
    wire                    adj_carry = adj_sum[MANT_W];
    wire [MANT_W-1:0]       mant24_fixed = adj_carry ? adj_sum[MANT_W:1]
                                                     : adj_sum[MANT_W-1:0];
    wire                    carry_up     = adj_carry;

    // ---------- exponent combine ----------
    integer new_unb;
    always @* begin
        // /2^K_Q2 는 지수에서 -K_Q2 로 처리, 가수 보정 캐리는 +1
        new_unb = msb_idx + A_exp_unb - FP_MANT_W - K_Q2 + (carry_up ? 1 : 0);
    end

    // ---------- overflow / underflow / subnormal ----------
    wire oflow = (new_unb >  UNB_MAX);
    wire uflow = (new_unb <  UNB_MIN);

    integer need_shift_i;
    reg [MANT_W-1:0] mant24_sub;
    reg [MANT_W-1:0] add_rnd;

    always @* begin
        need_shift_i = UNB_MIN - new_unb; // positive => need to shift right
        if (need_shift_i < 0) need_shift_i = 0;

        if (need_shift_i >= MANT_W) begin
            mant24_sub = {MANT_W{1'b0}};
        end else if (need_shift_i == 0) begin
            mant24_sub = mant24_fixed;
        end else begin
            // subnormal 라운딩 (RN)
            add_rnd    = {{(MANT_W-1){1'b0}}, 1'b1} << (need_shift_i - 1);
            mant24_sub = (mant24_fixed + add_rnd) >> need_shift_i;
        end
    end

    // ---------- assemble IEEE754 ----------
    reg  [FP_EXP_W-1:0]  exp_out;
    reg  [FP_MANT_W-1:0] frac_out;
    reg  [EW-1:0]        exp_new_raw_vec;

    always @* begin
        if (is_zero_path) begin
            exp_out  = {FP_EXP_W{1'b0}};
            frac_out = {FP_MANT_W{1'b0}};
        end else if (oflow) begin
            exp_out  = { {(FP_EXP_W-1){1'b1}}, 1'b0 }; // max finite
            frac_out = { FP_MANT_W{1'b1} };
        end else if (uflow) begin
            exp_out  = {FP_EXP_W{1'b0}};               // subnormal
            frac_out = mant24_sub[FP_MANT_W-1:0];
        end else begin
            exp_new_raw_vec = new_unb + FP_EXP_BIAS;
            exp_out  = exp_new_raw_vec[FP_EXP_W-1:0];
            frac_out = mant24_fixed[FP_MANT_W-1:0];
        end
    end

    wire [FP_DATA_W-1:0] mag_fp = {1'b0, exp_out, frac_out};
    assign r_data = sign_acc ? {1'b1, mag_fp[FP_DATA_W-2:0]} : mag_fp;

endmodule
