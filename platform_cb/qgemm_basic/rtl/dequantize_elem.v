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
    localparam integer LOG2_QSQ_CEIL   = clog2(QSQ);             // ceil(log2(Q^2)) e.g. 14

    // normalized exponent range (unbiased)
    localparam integer EXP_RAW_MIN_NORM = 1;
    localparam integer EXP_RAW_MAX_NORM = (1 << FP_EXP_W) - 2;
    localparam integer UNB_MIN          = EXP_RAW_MIN_NORM - FP_EXP_BIAS; // -126 for FP32
    localparam integer UNB_MAX          = EXP_RAW_MAX_NORM - FP_EXP_BIAS; // +127 for FP32

    // exponent scratch width (for intermediate sums)
    localparam integer EW = FP_EXP_W + 6;  // margin bits

    // fixed-point mantissa adjustment to cancel /Q^2 -> /2^k bias
    // MANT_ADJ ~= round( (2^LOG2_QSQ_CEIL * 2^FIXP_K) / QSQ )
    localparam integer FIXP_K   = 16; // precision (12~20 OK)
    localparam integer MANT_ADJ = ((1 << (LOG2_QSQ_CEIL + FIXP_K)) + (QSQ >> 1)) / QSQ;

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
    integer A_exp_unb;   // <<<<< 모듈 상단 선언

    assign A_mant   = {1'b1, mantissa_scale};
    assign A_is_zero= (exp_scale == {FP_EXP_W{1'b0}}) && (mantissa_scale == {FP_MANT_W{1'b0}});
    always @* begin
        A_exp_unb = exp_scale - FP_EXP_BIAS; // unbiased exponent of S
    end

    // ---------- integer product P = |acc| * (1.m) ----------
    localparam integer PW = FP_DATA_W + MANT_W; // product width
    wire [PW-1:0] P;
    assign P = acc_abs * A_mant; // unsigned multiply

    wire is_zero_path;
    assign is_zero_path = acc_is_zero | A_is_zero | (P == {PW{1'b0}});

    // ---------- find MSB index of P (floor(log2 P)) ----------
    reg [31:0] msb_idx;  // <<<<< 모듈 상단 선언
    integer k;           // <<<<< 모듈 상단 선언 (for 루프 변수)
    reg found;           // <<<<< 모듈 상단 선언 (브레이크 대용)

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

    // ---------- normalize to 24-bit (1.h) with rounding ----------
    reg  [31:0] sr, sl;                 // <<<<< 모듈 상단 선언
    wire        shift_right;
    wire [PW-1:0] P_shifted_r;
    reg  round_bit;                     // <<<<< 모듈 상단 선언
    wire [MANT_W:0]   mant_with_carry_r;
    wire [MANT_W-1:0] mant24_right;
    reg  [MANT_W-1:0] mant24_left;      // <<<<< 모듈 상단 선언
    wire [MANT_W-1:0] mant24_norm_pre;

    assign shift_right = (msb_idx >= (MANT_W-1));
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
    always @* begin
        if (sr == 0) round_bit = 1'b0;
        else          round_bit = P[sr-1];
    end

    assign mant_with_carry_r = {1'b0, P_shifted_r[MANT_W-1:0]} + {{MANT_W{1'b0}}, round_bit};
    assign mant24_right      = mant_with_carry_r[MANT_W] ?
                               {1'b1, P_shifted_r[MANT_W-1:1]} :
                               mant_with_carry_r[MANT_W-1:0];

    always @* begin
        // (P << sl)의 상위 MANT_W 비트 취득: (P << sl)[PW-1 : PW-MANT_W]
        mant24_left = (P << sl) >> (PW - MANT_W);
    end

    assign mant24_norm_pre = shift_right ? mant24_right : mant24_left; // 24-bit 1.h

    // ---------- mantissa adjustment to cancel /Q^2 bias ----------
    reg [MANT_W+FIXP_K-1:0] prod_adj;           // <<<<< 모듈 상단 선언
    reg [MANT_W:0]          mant24_adj_with_carry; // <<<<< 모듈 상단 선언
    reg [MANT_W-1:0]        mant24_fixed;       // <<<<< 모듈 상단 선언
    reg                     carry_up;           // <<<<< 모듈 상단 선언

    always @* begin
        prod_adj = mant24_norm_pre * MANT_ADJ;
        mant24_adj_with_carry = (prod_adj + (1 << (FIXP_K-1))) >> FIXP_K;
        if (mant24_adj_with_carry[MANT_W]) begin
            mant24_fixed = {1'b1, mant24_adj_with_carry[MANT_W:2]}; // 2.x -> 1.x
            carry_up     = 1'b1;
        end else begin
            mant24_fixed = mant24_adj_with_carry[MANT_W-1:0];
            carry_up     = 1'b0;
        end
    end

    // ---------- exponent combine ----------
    integer new_unb;         // <<<<< 모듈 상단 선언
    always @* begin
        new_unb = msb_idx + A_exp_unb - FP_MANT_W - LOG2_QSQ_CEIL + (carry_up ? 1 : 0);
    end

    // ---------- overflow / underflow / subnormal ----------
    wire oflow, uflow;
    assign oflow = (new_unb >  UNB_MAX);
    assign uflow = (new_unb <  UNB_MIN);

    integer need_shift_i;           // <<<<< 모듈 상단 선언
    reg [MANT_W-1:0] mant24_sub;   // <<<<< 모듈 상단 선언
    reg [MANT_W-1:0] add_rnd;      // <<<<< 모듈 상단 선언

    always @* begin
        need_shift_i = UNB_MIN - new_unb; // positive => need to shift right
        if (need_shift_i < 0) need_shift_i = 0;

        if (need_shift_i >= MANT_W) begin
            mant24_sub = {MANT_W{1'b0}};
        end else if (need_shift_i == 0) begin
            mant24_sub = mant24_fixed;
        end else begin
            add_rnd    = {{(MANT_W-1){1'b0}}, 1'b1} << (need_shift_i - 1);
            mant24_sub = (mant24_fixed + add_rnd) >> need_shift_i;
        end
    end

    // ---------- assemble IEEE754 ----------
    reg  [FP_EXP_W-1:0]  exp_out;           // <<<<< 모듈 상단 선언
    reg  [FP_MANT_W-1:0] frac_out;          // <<<<< 모듈 상단 선언
    reg  [EW-1:0]        exp_new_raw_vec;   // <<<<< 모듈 상단 선언

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

    wire [FP_DATA_W-1:0] mag_fp;
    assign mag_fp = {1'b0, exp_out, frac_out};
    assign r_data = sign_acc ? {1'b1, mag_fp[FP_DATA_W-2:0]} : mag_fp;

endmodule
