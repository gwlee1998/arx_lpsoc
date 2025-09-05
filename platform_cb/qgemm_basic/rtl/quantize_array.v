// 행렬 딘위로 양자화??
module quantize_array #(
    parameter integer BIT_NUM     = 8,
    parameter integer MAT_SIZE    = 16,
    parameter integer FP_DATA_W   = 32,
    parameter integer FP_EXP_W    = 8,
    parameter integer FP_MANT_W   = 23,
    parameter integer FP_EXP_BIAS = 127,
    parameter integer LANES_NUM   = 16
)(
    input  wire                             clk,
    input  wire                             rstnn,

    // input stream (scalar-in, LANES_NUM per beat)
    input  wire                             s_valid_i,
    output wire                             s_ready_o,
    input  wire [LANES_NUM*FP_DATA_W-1:0]   s_data_i,

    // scale (one set per tile)
    output wire                             scl_valid_o,
    input  wire                             scl_ready_i,
    output reg  [FP_MANT_W*MAT_SIZE-1:0]    mantissa_scale_o,
    output reg  [FP_EXP_W*MAT_SIZE-1:0]     exp_scale_o,

    // quantized lanes-out stream
    output wire                             m_valid_o,
    input  wire                             m_ready_i,
    output wire [LANES_NUM*FP_DATA_W-1:0]   m_data_o
);

    // ------------------------------------------------------------
    // utils
    function integer clog2;
        input integer v;
        integer i;
        begin
            i=0; while((1<<i)<v) i=i+1; clog2=i;
        end
    endfunction

    localparam integer ELEMS   = MAT_SIZE*MAT_SIZE;
    localparam integer IN_BEATS= (ELEMS + LANES_NUM - 1) / LANES_NUM;
    localparam integer EIDX_W  = clog2((ELEMS>0)?ELEMS:1);
    localparam integer INBW_W  = clog2((IN_BEATS>0)?IN_BEATS:1);
    localparam integer IDX_W   = clog2((MAT_SIZE>1)?MAT_SIZE:2);

    initial begin
        if (LANES_NUM<=0) $fatal("LANES_NUM>0");
        if (ELEMS % LANES_NUM != 0)
            $fatal("ELEMS(%0d) %% LANES_NUM(%0d)!=0", ELEMS, LANES_NUM);
    end

    // ------------------------------------------------------------
    // FSM
    localparam [1:0] S_FILL=2'd0, S_SCALE=2'd1, S_EMIT=2'd2;
    reg [1:0] state, state_n;

    reg [INBW_W-1:0] in_beat;   // FILL beats
    reg [INBW_W-1:0] out_beat;

    wire fill_fire = (state==S_FILL) && s_valid_i && s_ready_o;
    wire emit_fire = (state==S_EMIT) && m_valid_o && m_ready_i;

    always @* begin
        state_n = state;
        case (state)
            S_FILL : if (fill_fire && (in_beat==IN_BEATS-1))  state_n = S_SCALE;
            S_SCALE: if (scl_valid_o && scl_ready_i)          state_n = S_EMIT;
            S_EMIT : if (emit_fire && (out_beat==IN_BEATS-1)) state_n = S_FILL;
        endcase
    end

    always @(posedge clk or negedge rstnn) begin
        if (!rstnn) begin
            state    <= S_FILL;
            in_beat  <= '0;
            out_beat <= '0;
        end else begin
            state <= state_n;

            if (state==S_FILL && fill_fire)
                in_beat <= (in_beat==IN_BEATS-1) ? '0 : in_beat + 1'b1;

            if (state==S_EMIT && emit_fire)
                out_beat <= (out_beat==IN_BEATS-1) ? '0 : out_beat + 1'b1;

            if (state!=S_EMIT) out_beat <= '0;
        end
    end

    // ------------------------------------------------------------
    // FILL: store tile
    reg [FP_DATA_W-1:0] tile_mem [0:ELEMS-1];

    assign s_ready_o = (state==S_FILL);

    wire [EIDX_W-1:0] base_idx = in_beat * LANES_NUM;

    integer li, k;
    always @(posedge clk or negedge rstnn) begin
        if (!rstnn) begin
            // no-op
        end else if (state==S_FILL && s_valid_i && s_ready_o) begin
            for (li=0; li<LANES_NUM; li=li+1) begin
                tile_mem[base_idx + li] <= s_data_i[(li+1)*FP_DATA_W-1 -: FP_DATA_W];
            end
        end
    end

    // build ROW vectors & run max_finder (row-wise only)
    // axis k = row k: vec[c] = tile[k*MAT_SIZE + c]
    wire [FP_EXP_W-1:0]  axis_exp_w  [0:MAT_SIZE-1];
    wire [FP_MANT_W-1:0] axis_mant_w [0:MAT_SIZE-1];

    genvar ax, pos;
    generate
        for (ax=0; ax<MAT_SIZE; ax=ax+1) begin : AXIS_LOOP
            wire [FP_DATA_W*MAT_SIZE-1:0] axis_vec;
            for (pos=0; pos<MAT_SIZE; pos=pos+1) begin : VEC_MAKE
                localparam integer IDX = ax*MAT_SIZE + pos; // row=ax, col=pos
                assign axis_vec[(pos+1)*FP_DATA_W-1 -: FP_DATA_W] = tile_mem[IDX];
            end
            max_finder #(
                .MAT_SIZE   (MAT_SIZE),
                .FP_DATA_W  (FP_DATA_W),
                .FP_EXP_W   (FP_EXP_W),
                .FP_MANT_W  (FP_MANT_W)
            ) u_max (
                .data_i       (axis_vec),
                .max_exp_o    (axis_exp_w [ax]),  // RAW exponent
                .max_mantissa_o(axis_mant_w[ax])
            );
        end
    endgenerate

    // ------------------------------------------------------------
    // SCALE: latch max_finder outputs (one-cycle after entering S_SCALE)
    reg scl_pending, scl_latched; // S_SCALE 진입 후 첫 클럭에만 래치

    always @(posedge clk or negedge rstnn) begin
        if (!rstnn) begin
            scl_pending      <= 1'b0;
            scl_latched      <= 1'b0;
            mantissa_scale_o <= '0;
            exp_scale_o      <= '0;
        end else begin
            if (state==S_SCALE) begin
                if (!scl_latched) begin
                    for (k=0; k<MAT_SIZE; k=k+1) begin
                        mantissa_scale_o[(k+1)*FP_MANT_W-1 -: FP_MANT_W] <= axis_mant_w[k];
                        exp_scale_o [(k+1)*FP_EXP_W  -1 -: FP_EXP_W ]   <= axis_exp_w [k];
                    end
                    scl_pending <= 1'b1;
                    scl_latched <= 1'b1;
                end
                if (scl_valid_o && scl_ready_i)
                    scl_pending <= 1'b0;
            end else begin
                scl_latched <= 1'b0;
            end
        end
    end

    assign scl_valid_o = (state==S_SCALE) && scl_pending;

    // ------------------------------------------------------------
    // EMIT: quantize with row-scale

    // helpers to index packed row scales
    function [FP_MANT_W-1:0] vec_get_mant;
        input [FP_MANT_W*MAT_SIZE-1:0] v;
        input [IDX_W-1:0] idx;
        vec_get_mant = v[(idx+1)*FP_MANT_W-1 -: FP_MANT_W];
    endfunction
    function [FP_EXP_W-1:0] vec_get_exp;
        input [FP_EXP_W*MAT_SIZE-1:0] v;
        input [IDX_W-1:0] idx;
        vec_get_exp = v[(idx+1)*FP_EXP_W-1 -: FP_EXP_W];
    endfunction

    // sign-extend BIT_NUM -> FP_DATA_W
    function [FP_DATA_W-1:0] sext_to_bw;
        input [BIT_NUM-1:0] s;
        begin
            sext_to_bw = {{(FP_DATA_W-BIT_NUM){s[BIT_NUM-1]}}, s};
        end
    endfunction

    // beat base index for EMIT
    wire [EIDX_W-1:0] out_base_idx = out_beat * LANES_NUM;

    // lane-wise quantization & packing
    genvar gl;
    generate
        for (gl=0; gl<LANES_NUM; gl=gl+1) begin : EMIT_LANES
            wire [EIDX_W-1:0] elem_idx = out_base_idx + gl;
            wire [IDX_W-1:0]  ridx_lane = elem_idx / MAT_SIZE;

            wire [FP_DATA_W-1:0]  r_elem_lane = tile_mem[elem_idx];
            wire [FP_MANT_W-1:0]  m_sel_lane  = vec_get_mant(mantissa_scale_o, ridx_lane);
            wire [FP_EXP_W-1:0]   e_sel_lane  = vec_get_exp (exp_scale_o,      ridx_lane);

            wire [BIT_NUM-1:0] q_elem_lane;

            quantize_elem #(
                .FP_DATA_W  (FP_DATA_W),
                .FP_MANT_W  (FP_MANT_W),
                .FP_EXP_W   (FP_EXP_W),
                .BIT_NUM    (BIT_NUM),
                .FP_EXP_BIAS(FP_EXP_BIAS),
                .DE_MARGIN  (16)
            ) quantize_elem_i (
                .r_data        (r_elem_lane),
                .mantissa_scale(m_sel_lane),
                .exp_scale     (e_sel_lane),
                .q_data        (q_elem_lane)
            );

            wire [FP_DATA_W-1:0] lane_out = sext_to_bw(q_elem_lane);  // BIT_NUM 크기로 양자화된 값을 FP_DATA_W로 늘림
            assign m_data_o[(gl+1)*FP_DATA_W-1 -: FP_DATA_W] = lane_out;
        end
    endgenerate

    assign m_valid_o = (state==S_EMIT);

endmodule
