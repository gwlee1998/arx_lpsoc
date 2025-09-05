// 레지스터 필요함?
module dequantize_vector #(
    parameter integer BIT_NUM     = 8,
    parameter integer FP_DATA_W   = 32,
    parameter integer FP_MANT_W   = 23,
    parameter integer FP_EXP_W    = 8,
    parameter integer FP_EXP_BIAS = 127,
    parameter integer LANES_NUM   = 16
)(
    input  wire signed [LANES_NUM*FP_DATA_W-1:0]  q_data_i,
    input  wire        [LANES_NUM*FP_MANT_W-1:0]  mantissa_scale_i,
    input  wire        [LANES_NUM*FP_EXP_W-1:0]   exp_scale_i,
    output wire        [LANES_NUM*FP_DATA_W-1:0]  r_data_o
);

    genvar l;
    generate
      for (l=0; l<LANES_NUM; l=l+1) begin : G_DEQ
        wire signed [FP_DATA_W-1:0] acc_l  = q_data_i [(l+1)*FP_DATA_W-1      -: FP_DATA_W];
        wire        [FP_MANT_W-1:0]  mant_l = mantissa_scale_i [(l+1)*FP_MANT_W-1 -: FP_MANT_W];
        wire        [FP_EXP_W-1:0]   exp_l  = exp_scale_i [(l+1)*FP_EXP_W-1      -: FP_EXP_W];
        wire        [FP_DATA_W-1:0]  r_l;

        dequantize_elem #(
            .FP_DATA_W(FP_DATA_W), .FP_MANT_W(FP_MANT_W), .FP_EXP_W(FP_EXP_W),
            .FP_EXP_BIAS(FP_EXP_BIAS), .BIT_NUM(BIT_NUM)
        ) dequantize_elem_i (
            .acc_i         (acc_l),
            .mantissa_scale(mant_l),
            .exp_scale     (exp_l),
            .r_data        (r_l)
        );

        assign r_data_o[(l+1)*FP_DATA_W-1 -: FP_DATA_W] = r_l;
      end
    endgenerate


    /////////////////////////////////////////////////////////////
    // 디버깅용 로직
    /////////////////////////////////////////////////////////////
    // 플랫 입력 버스 -> 2차원 reg (웨이브 보기용)
    reg [FP_DATA_W-1:0] r_data_o_reg [0:LANES_NUM-1];
    reg [FP_DATA_W-1:0] q_data_i_reg [0:LANES_NUM-1];
    reg [FP_MANT_W-1:0] mts_i_reg [0:LANES_NUM-1];
    reg [FP_EXP_W-1:0] exp_i_reg [0:LANES_NUM-1];
    
    integer __r, __idx;
    always @* begin
        for (__r = 0; __r < LANES_NUM; __r = __r + 1) begin
            q_data_i_reg[__r] = q_data_i[(__r+1)*FP_DATA_W-1 -: FP_DATA_W];
            r_data_o_reg[__r] = r_data_o[(__r+1)*FP_DATA_W-1 -: FP_DATA_W];
            mts_i_reg[__r] = mantissa_scale_i[(__r+1)*FP_MANT_W-1 -: FP_MANT_W];
            exp_i_reg[__r] = exp_scale_i[(__r+1)*FP_EXP_W-1 -: FP_EXP_W];
        end
    end

endmodule
