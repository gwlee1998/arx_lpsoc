module max_finder #(
    parameter integer MAT_SIZE     = 16,  // 반드시 2의 거듭제곱
    parameter integer FP_DATA_W    = 32,
    parameter integer FP_EXP_W     = 8,
    parameter integer FP_MANT_W    = 23
)(
    input  wire [FP_DATA_W*MAT_SIZE-1:0]  data_i,
    output wire [FP_EXP_W-1:0]            max_exp_o,
    output wire [FP_MANT_W-1:0]           max_mantissa_o
);
    // ------------------------------------------------------------
    // utils
    function integer clog2; input integer v; integer i; begin i=0; while ((1<<i)<v) i=i+1; clog2=i; end endfunction
    function integer is_pow2; input integer v; integer t; begin
        if (v<1) is_pow2=0; else begin t=v; while ((t & 1)==0) t = t >> 1; is_pow2=(t==1); end
    end endfunction

    localparam integer STAGES = clog2(MAT_SIZE);

    initial begin
        if (!is_pow2(MAT_SIZE)) $fatal("max_finder_absmax: MAT_SIZE(%0d) must be power-of-two.", MAT_SIZE);
    end

    // ------------------------------------------------------------
    // flatten lanes
    // 슬라이스 컨벤션 통일: [(i+1)*W-1 -: W]
    wire [FP_DATA_W-1:0] lane [0:MAT_SIZE-1];
    genvar gi;
    generate
        for (gi=0; gi<MAT_SIZE; gi=gi+1) begin : SLICE
            assign lane[gi] = data_i[(gi+1)*FP_DATA_W-1 -: FP_DATA_W];
        end
    endgenerate

    // 각 lane의 절댓값 및 필드
    wire [FP_DATA_W-1:0]      mag   [0:MAT_SIZE-1];
    wire [FP_EXP_W-1:0]       exp0  [0:MAT_SIZE-1];
    wire [FP_MANT_W-1:0]  man0  [0:MAT_SIZE-1];
    wire [FP_MANT_W:0]    wh0   [0:MAT_SIZE-1]; // hidden|mantissa (정규수=1.xxx, subnorm=0.xxx)

    generate
        for (gi=0; gi<MAT_SIZE; gi=gi+1) begin : FLD0
            assign mag [gi] = {1'b0, lane[gi][FP_DATA_W-2:0]}; // abs (sign 제거)
            assign exp0[gi] = mag[gi][FP_MANT_W+FP_EXP_W-1 -: FP_EXP_W];
            assign man0[gi] = mag[gi][FP_MANT_W-1:0];
            assign wh0 [gi] = { (exp0[gi] != {FP_EXP_W{1'b0}}), man0[gi] };
        end
    endgenerate

    // ------------------------------------------------------------
    // 2:1 선택 함수 (지수→hidden|mant 우선)
    function automatic [FP_EXP_W+FP_MANT_W-1:0] pick;
        input [FP_EXP_W-1:0]      e0, e1;
        input [FP_MANT_W:0]   w0, w1;
        input [FP_MANT_W-1:0] m0, m1;
        begin
            if (e0 > e1)       pick = {e0, m0};
            else if (e1 > e0)  pick = {e1, m1};
            else               pick = (w0 >= w1) ? {e0, m0} : {e1, m1};
        end
    endfunction

    // ------------------------------------------------------------
    // 다단 비교 트리 (일반화)
    wire [FP_EXP_W-1:0]       e [0:STAGES][0:MAT_SIZE-1];
    wire [FP_MANT_W-1:0]  m [0:STAGES][0:MAT_SIZE-1];
    wire [FP_MANT_W:0]    w [0:STAGES][0:MAT_SIZE-1];

    // stage 0 입력
    generate
        for (gi=0; gi<MAT_SIZE; gi=gi+1) begin : ST0
            assign e[0][gi] = exp0[gi];
            assign m[0][gi] = man0[gi];
            assign w[0][gi] = wh0[gi];
        end
    endgenerate

    // stage s → s+1
    genvar s, i;
    generate
        for (s=0; s<STAGES; s=s+1) begin : STAGES_GEN
            localparam integer N_CUR = (MAT_SIZE >> s);
            localparam integer N_NXT = (MAT_SIZE >> (s+1));
            for (i=0; i<N_NXT; i=i+1) begin : REDUCE
                wire [FP_EXP_W+FP_MANT_W-1:0] sel =
                    pick(e[s][2*i], e[s][2*i+1], w[s][2*i], w[s][2*i+1], m[s][2*i], m[s][2*i+1]);
                assign e[s+1][i] = sel[FP_EXP_W+FP_MANT_W-1 -: FP_EXP_W];
                assign m[s+1][i] = sel[FP_MANT_W-1:0];
                assign w[s+1][i] = { (e[s+1][i] != {FP_EXP_W{1'b0}}), m[s+1][i] };
            end
        end
    endgenerate

    assign max_exp_o      = e[STAGES][0];   // RAW exponent
    assign max_mantissa_o = m[STAGES][0];

endmodule