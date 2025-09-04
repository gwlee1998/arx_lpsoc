module scale_acc_sync #(
    parameter integer LANES_NUM  = 16,
    parameter integer FP_MANT_W  = 23,
    parameter integer FP_EXP_W   = 8,
    parameter integer ELEMS      = 256  // MAT_SIZE*MAT_SIZE
)(
    input  wire                           clk,
    input  wire                           rstnn,

    // 타일 로드/스텝 제어 (qdq_controller에서 생성)
    input  wire                           load_i,  // = dq_start (pop하는 그 사이클)
    input  wire                           step_i,  // = dq_step  (매 beat 소비)

    // FIFO에서 꺼낸 '한 타일' 전체 (show-ahead로 이미 head가 나와 있음)
    input  wire [FP_MANT_W*ELEMS-1:0]     fifo_mant_full_i,
    input  wire [FP_EXP_W *ELEMS-1:0]     fifo_exp_full_i,

    // 현재 beat에서 dequant에 쓸 LANE 묶음(스케일)
    output wire [LANES_NUM*FP_MANT_W-1:0] cur_mant_lanes_o,
    output wire [LANES_NUM*FP_EXP_W -1:0] cur_exp_lanes_o
);
    localparam integer SHIFT_MANT_W = LANES_NUM*FP_MANT_W;
    localparam integer SHIFT_EXP_W  = LANES_NUM*FP_EXP_W;

    // 타일 전체를 들고 있는 시프트 레지스터
    reg [FP_MANT_W*ELEMS-1:0] mant_shift_q;
    reg [FP_EXP_W *ELEMS-1:0] exp_shift_q;

    // 첫 beat에서 바로 쓸 수 있도록 FIFO의 '첫 LANE 묶음'을 분리
    wire [SHIFT_MANT_W-1:0] lanes_mant_first = fifo_mant_full_i[SHIFT_MANT_W-1:0];
    wire [SHIFT_EXP_W -1:0] lanes_exp_first  = fifo_exp_full_i [SHIFT_EXP_W -1:0];

    // 출력: load_i 사이클에는 FIFO의 첫 묶음, 그 다음부터는 시프트 레지스터 하위 비트
    assign cur_mant_lanes_o = load_i ? lanes_mant_first : mant_shift_q[SHIFT_MANT_W-1:0];
    assign cur_exp_lanes_o  = load_i ? lanes_exp_first  : exp_shift_q [SHIFT_EXP_W -1:0];

    // 로드/시프트
    always @(posedge clk or negedge rstnn) begin
        if (!rstnn) begin
            mant_shift_q <= '0;
            exp_shift_q  <= '0;
        end else begin
            if (load_i) begin
                // pop된 타일을 통째로 적재 (다음 사이클부터 하위 묶음이 1st beat)
                mant_shift_q <= fifo_mant_full_i;
                exp_shift_q  <= fifo_exp_full_i;
            end else if (step_i) begin
                // 한 beat 소비 → 다음 beat를 하위로 당김
                mant_shift_q <= mant_shift_q >> SHIFT_MANT_W;
                exp_shift_q  <= exp_shift_q  >> SHIFT_EXP_W;
            end
        end
    end
endmodule
