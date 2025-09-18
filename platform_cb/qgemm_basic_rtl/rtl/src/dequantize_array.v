module dequantize_array #(
    parameter integer BIT_NUM     = 8,
    parameter integer MAT_SIZE    = 16,
    parameter integer FP_DATA_W   = 32,
    parameter integer FP_MANT_W   = 23,
    parameter integer FP_EXP_W    = 8,
    parameter integer FP_EXP_BIAS = 127,
    parameter integer LANES_NUM   = 16
)(
    input  wire                             clk,
    input  wire                             rstnn,

    // ACC tile in (LANES_NUM per beat)
    input  wire                             s_valid_i,
    output wire                             s_ready_o,
    input  wire [LANES_NUM*FP_DATA_W-1:0]   s_data_i,

    // SCALE tile in (full matrix from scale FIFO head)
    input  wire                             scl_valid_i,
    output wire                             scl_ready_o,
    input  wire [FP_MANT_W*MAT_SIZE*MAT_SIZE-1:0] mantissa_scale_i,
    input  wire [FP_EXP_W *MAT_SIZE*MAT_SIZE-1:0] exp_scale_i,

    // dequantized FP out
    output wire                             m_valid_o,
    input  wire                             m_ready_i,
    output wire [LANES_NUM*FP_DATA_W-1:0]   m_data_o
);
    // ---------------- utils ----------------
    function integer clog2; input integer v; integer i; begin i=0; while((1<<i)<v) i=i+1; clog2=i; end endfunction
    localparam integer ELEMS     = MAT_SIZE*MAT_SIZE;
    localparam integer IN_BEATS  = (ELEMS + LANES_NUM - 1) / LANES_NUM;
    localparam integer EIDX_W    = clog2((ELEMS>0)?ELEMS:1);
    localparam integer INBW_W    = clog2((IN_BEATS>0)?IN_BEATS:1);

    initial begin
        if (LANES_NUM<=0) $fatal("LANES_NUM>0");
        if (ELEMS % LANES_NUM != 0)
            $fatal("ELEMS(%0d) %% LANES_NUM(%0d)!=0", ELEMS, LANES_NUM);
    end

    // ---------------- FSM (scale-first) ----------------
    localparam [1:0] S_WAIT_SCL=2'd0, S_FILL_ACC=2'd1, S_EMIT=2'd2;
    reg [1:0] state, state_n;

    reg [INBW_W-1:0] in_beat;
    reg [INBW_W-1:0] out_beat;

    // scale latch
    reg [FP_MANT_W*ELEMS-1:0] mant_mat_q;
    reg [FP_EXP_W *ELEMS-1:0] exp_mat_q;
    reg scl_captured;

    // ACC tile buffer
    reg [FP_DATA_W-1:0] acc_tile [0:ELEMS-1];

    wire fill_fire = (state==S_FILL_ACC) & s_valid_i & s_ready_o;
    wire emit_fire = (state==S_EMIT)     & m_valid_o & m_ready_i;

    always @* begin
        state_n = state;
        case (state)
          S_WAIT_SCL: if (scl_valid_i && scl_ready_o)           state_n = S_FILL_ACC;
          S_FILL_ACC: if (fill_fire && (in_beat==IN_BEATS-1))   state_n = S_EMIT;
          S_EMIT    : if (emit_fire && (out_beat==IN_BEATS-1))  state_n = S_WAIT_SCL;
        endcase
    end

    always @(posedge clk or negedge rstnn) begin
        if (!rstnn) begin
            state        <= S_WAIT_SCL;
            in_beat      <= '0;
            out_beat     <= '0;
            scl_captured <= 1'b0;
            mant_mat_q   <= '0;
            exp_mat_q    <= '0;
        end else begin
            state <= state_n;

            // scale handshaking & latch
            if (state==S_WAIT_SCL) begin
                if (scl_valid_i && scl_ready_o) begin
                    mant_mat_q   <= mantissa_scale_i;
                    exp_mat_q    <= exp_scale_i;
                    scl_captured <= 1'b1;
                end else begin
                    scl_captured <= 1'b0;
                end
            end

            // ACC fill
            if (state==S_FILL_ACC && fill_fire)
                in_beat <= (in_beat==IN_BEATS-1) ? '0 : in_beat + 1'b1;
            else if (state!=S_FILL_ACC)
                in_beat <= '0;

            // emit
            if (state==S_EMIT && emit_fire)
                out_beat <= (out_beat==IN_BEATS-1) ? '0 : out_beat + 1'b1;
            else if (state!=S_EMIT)
                out_beat <= '0;
        end
    end

    assign scl_ready_o = (state==S_WAIT_SCL) & (~scl_captured); // pop 1펄스 동작
    assign s_ready_o   = (state==S_FILL_ACC);

    // ---------------- FILL_ACC ----------------
    wire [EIDX_W-1:0] in_base_idx = in_beat * LANES_NUM;
    integer li;
    always @(posedge clk) begin
        if (state==S_FILL_ACC && s_valid_i && s_ready_o) begin
            for (li=0; li<LANES_NUM; li=li+1) begin
                acc_tile[in_base_idx + li] <= s_data_i[(li+1)*FP_DATA_W-1 -: FP_DATA_W];
            end
        end
    end

    // ---------------- EMIT (ACC+Scale per element) ----------------
    function [FP_MANT_W-1:0] elem_get_mant;
        input [FP_MANT_W*ELEMS-1:0] v; input [EIDX_W-1:0] idx;
        elem_get_mant = v[(idx+1)*FP_MANT_W-1 -: FP_MANT_W];
    endfunction
    function [FP_EXP_W-1:0] elem_get_exp;
        input [FP_EXP_W*ELEMS-1:0] v; input [EIDX_W-1:0] idx;
        elem_get_exp = v[(idx+1)*FP_EXP_W-1 -: FP_EXP_W];
    endfunction

    wire [EIDX_W-1:0] out_base_idx = out_beat * LANES_NUM;

    // 조합으로 계산되는 한 사이클 앞 데이터
    wire [LANES_NUM*FP_DATA_W-1:0] m_data_c;

    genvar gl;
    generate
        for (gl=0; gl<LANES_NUM; gl=gl+1) begin : G_EMIT
            wire [EIDX_W-1:0] elem_idx = out_base_idx + gl;

            wire signed [FP_DATA_W-1:0] acc_l  = acc_tile[elem_idx];
            wire        [FP_MANT_W-1:0] mant_l = elem_get_mant(mant_mat_q, elem_idx);
            wire        [FP_EXP_W -1:0] exp_l  = elem_get_exp (exp_mat_q,  elem_idx);

            wire [FP_DATA_W-1:0] r_l;
            dequantize_elem #(
                .FP_DATA_W(FP_DATA_W), .FP_MANT_W(FP_MANT_W), .FP_EXP_W(FP_EXP_W),
                .FP_EXP_BIAS(FP_EXP_BIAS), .BIT_NUM(BIT_NUM)
            ) dequantize_elem_i (
                .acc_i           (acc_l),
                .mantissa_scale_i(mant_l),
                .exp_scale_i     (exp_l),
                .r_data_o        (r_l)
            );

            assign m_data_c[(gl+1)*FP_DATA_W-1 -: FP_DATA_W] = r_l;
        end
    endgenerate

    reg                           m_valid_q;
    reg [LANES_NUM*FP_DATA_W-1:0] m_data_q;

    always @(posedge clk or negedge rstnn) begin
        if (!rstnn) begin
            m_valid_q <= 1'b0;
            m_data_q  <= '0;
        end else begin
            if (state==S_EMIT) begin
                m_valid_q <= 1'b1;
                m_data_q  <= m_data_c;
            end else begin
                m_valid_q <= 1'b0;
                m_data_q  <= '0;
            end
        end
    end

    assign m_valid_o = m_valid_q;
    assign m_data_o  = m_data_q;

endmodule
