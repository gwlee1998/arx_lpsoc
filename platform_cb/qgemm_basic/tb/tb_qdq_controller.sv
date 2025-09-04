`timescale 1ns/1ps

`define TRACE_IN_X        "./trace/in_x.txt"
`define TRACE_IN_W        "./trace/in_w.txt"
`define TRACE_IN_Q_X_SW   "./trace/in_q_x_sw.txt"
`define TRACE_IN_Q_W_SW   "./trace/in_q_w_sw.txt"
`define TRACE_IN_Q_X_RTL  "./trace/in_q_x_rtl.txt"
`define TRACE_IN_Q_W_RTL  "./trace/in_q_w_rtl.txt"
`define TRACE_OT_RESULT   "./trace/ot_result.txt"
`define TRACE_OT_RTL_RESULT "./trace/ot_rtl_result.txt"
`define TRACE_OT_ERROR    "./trace/ot_error.txt"

// 필요하면 +define+TB_HAS_SCALE_PORTS
`ifdef FORCE_TB_HAS_SCALE_PORTS
  `ifndef TB_HAS_SCALE_PORTS
    `define TB_HAS_SCALE_PORTS
  `endif
`endif

localparam int WATCHDOG_CYCLES = 20000;

module tb_qdq_controller;

    // ---------------- Parameters ----------------
    localparam int BIT_NUM      = 8;
    localparam int MAT_SIZE     = 16;
    localparam int FP_EXP_W     = 8;
    localparam int FP_MANT_W    = 23;
    localparam int FP_EXP_BIAS  = 127;
    localparam int FP_DATA_W    = 32;
    localparam int LANES_NUM    = MAT_SIZE;   // MAT_SIZE*MAT_SIZE % LANES_NUM == 0
    localparam int ELEMS        = MAT_SIZE*MAT_SIZE;
    localparam int BEATS        = (ELEMS + LANES_NUM - 1)/LANES_NUM;
    localparam int SCALE_FIFO_DEPTH = 4;

    // --------------- Clock / Reset ---------------
    reg clk, rstnn;
    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk; // 100MHz
    end

    // --------------- DUT I/O ---------------------
    // A tile in
    reg                            a_s_valid_i;
    wire                           a_s_ready_o;
    reg  [LANES_NUM*FP_DATA_W-1:0] a_s_data_i;
    // A quantized out
    wire                           a_m_valid_o;
    reg                            a_m_ready_i;
    wire [LANES_NUM*FP_DATA_W-1:0] a_m_data_o;

    // B tile in
    reg                            b_s_valid_i;
    wire                           b_s_ready_o;
    reg  [LANES_NUM*FP_DATA_W-1:0] b_s_data_i;
    // B quantized out
    wire                           b_m_valid_o;
    reg                            b_m_ready_i;
    wire [LANES_NUM*FP_DATA_W-1:0] b_m_data_o;

    // Dequant ACC in
    reg                            dq_s_valid_i;
    wire                           dq_s_ready_o;
    reg                            dq_tfirst_i;
    reg  [LANES_NUM*FP_DATA_W-1:0] dq_s_data_i;
    // Dequant FP out
    wire                           dq_m_valid_o;
    reg                            dq_m_ready_i;
    wire [LANES_NUM*FP_DATA_W-1:0] dq_m_data_o;

`ifdef TB_HAS_SCALE_PORTS
    // (선택) DUT 스케일 디버그 포트
    wire                          a_scl_valid_o, b_scl_valid_o;
    reg                           a_scl_ready_i, b_scl_ready_i;
    wire [FP_MANT_W*MAT_SIZE-1:0] a_mantissa_scale_o, b_mantissa_scale_o;
    wire [FP_EXP_W *MAT_SIZE-1:0] a_exp_scale_o,      b_exp_scale_o;
`endif

    // ---------------- DUT -----------------------
    qdq_controller #(
        .BIT_NUM          (BIT_NUM),
        .MAT_SIZE         (MAT_SIZE),
        .FP_DATA_W        (FP_DATA_W),
        .FP_EXP_W         (FP_EXP_W),
        .FP_MANT_W        (FP_MANT_W),
        .FP_EXP_BIAS      (FP_EXP_BIAS),
        .LANES_NUM        (LANES_NUM),
        .SCALE_FIFO_DEPTH (SCALE_FIFO_DEPTH)
    ) dut (
        .clk            (clk),
        .rstnn          (rstnn),

        .a_s_valid_i    (a_s_valid_i),
        .a_s_ready_o    (a_s_ready_o),
        .a_s_data_i     (a_s_data_i),

        .a_m_valid_o    (a_m_valid_o),
        .a_m_ready_i    (a_m_ready_i),
        .a_m_data_o     (a_m_data_o),

        .b_s_valid_i    (b_s_valid_i),
        .b_s_ready_o    (b_s_ready_o),
        .b_s_data_i     (b_s_data_i),

        .b_m_valid_o    (b_m_valid_o),
        .b_m_ready_i    (b_m_ready_i),
        .b_m_data_o     (b_m_data_o),

        .dq_s_valid_i   (dq_s_valid_i),
        .dq_s_ready_o   (dq_s_ready_o),
        .dq_tfirst_i    (dq_tfirst_i),
        .dq_s_data_i    (dq_s_data_i),

        .dq_m_valid_o   (dq_m_valid_o),
        .dq_m_ready_i   (dq_m_ready_i),
        .dq_m_data_o    (dq_m_data_o)
`ifdef TB_HAS_SCALE_PORTS
        ,
        .a_scl_valid_o      (a_scl_valid_o),
        .a_scl_ready_i      (a_scl_ready_i),
        .a_mantissa_scale_o (a_mantissa_scale_o),
        .a_exp_scale_o      (a_exp_scale_o),

        .b_scl_valid_o      (b_scl_valid_o),
        .b_scl_ready_i      (b_scl_ready_i),
        .b_mantissa_scale_o (b_mantissa_scale_o),
        .b_exp_scale_o      (b_exp_scale_o)
`endif
    );

    // --------------- TB helpers -----------------
    function [31:0] real_to_fp32(input real r);
        shortreal s;
        begin s = r; real_to_fp32 = $shortrealtobits(s); end
    endfunction

    function real fp32_to_real(input [31:0] f);
        shortreal s;
        begin s = $bitstoshortreal(f); fp32_to_real = s; end
    endfunction

    function int clamp_int(input int v, input int lo, input int hi);
        begin
            if (v > hi) clamp_int = hi;
            else if (v < lo) clamp_int = lo;
            else clamp_int = v;
        end
    endfunction

    // -------------- Test data -------------------
    reg  [FP_DATA_W-1:0] A_bits [0:ELEMS-1];
    reg  [FP_DATA_W-1:0] B_bits [0:ELEMS-1];
    real A     [0:MAT_SIZE-1][0:MAT_SIZE-1];
    real B     [0:MAT_SIZE-1][0:MAT_SIZE-1];

    // SW 양자화/ACC/결과
    int  qA_sw [0:MAT_SIZE-1][0:MAT_SIZE-1];
    int  qB_sw [0:MAT_SIZE-1][0:MAT_SIZE-1];
    real A_absmax_row [0:MAT_SIZE-1];
    real B_absmax_row [0:MAT_SIZE-1];
    int  ACC  [0:MAT_SIZE-1][0:MAT_SIZE-1];
    reg  [FP_DATA_W-1:0] Cexp_bits [0:ELEMS-1];
    real   Cexp [0:MAT_SIZE-1][0:MAT_SIZE-1];

    // DUT 양자화/역양자화 캡처
    int  qA_dut [0:MAT_SIZE-1][0:MAT_SIZE-1];
    int  qB_dut [0:MAT_SIZE-1][0:MAT_SIZE-1];
    reg  [FP_DATA_W-1:0] Cdut_bits [0:ELEMS-1];

    // --- SW scale 저장 ---
    reg  [FP_MANT_W-1:0] A_mant_sw [0:MAT_SIZE-1], B_mant_sw [0:MAT_SIZE-1];
    reg  [FP_EXP_W -1:0] A_exp_sw  [0:MAT_SIZE-1], B_exp_sw  [0:MAT_SIZE-1];
    real                  A_scale_real_sw [0:MAT_SIZE-1], B_scale_real_sw [0:MAT_SIZE-1];

`ifdef TB_HAS_SCALE_PORTS
    // --- RTL scale 캡처 저장 ---
    reg  [FP_MANT_W-1:0] A_mant_dut [0:MAT_SIZE-1], B_mant_dut [0:MAT_SIZE-1];
    reg  [FP_EXP_W -1:0] A_exp_dut  [0:MAT_SIZE-1], B_exp_dut  [0:MAT_SIZE-1];
    real                  A_scale_real_dut [0:MAT_SIZE-1], B_scale_real_dut [0:MAT_SIZE-1];
    reg                   a_scale_captured, b_scale_captured;
`endif

    function int count_floats(string fname);
        int fd, rc, n; real tmp;
        n  = 0;
        fd = $fopen(fname, "r");
        if (!fd) return 0;
        while (1) begin
            rc = $fscanf(fd, "%f", tmp);
            if (rc != 1) break;
            n++;
        end
        $fclose(fd);
        return n;
    endfunction

    // 입력 trace 파일을 "읽기만" 한다. 부족/없으면 즉시 fatal.
    task automatic puts_and_load();
        int na, nb;
        // 파일에 float이 ELEMS개씩 있는지 직접 체크
        na = count_floats(`TRACE_IN_X);
        nb = count_floats(`TRACE_IN_W);

        if (na < ELEMS) begin
            $fatal(1, "[%0t] TRACE_IN_X has only %0d floats; need %0d. (read-only mode, no auto-generate)",
                   $time, na, ELEMS);
        end
        if (nb < ELEMS) begin
            $fatal(1, "[%0t] TRACE_IN_W has only %0d floats; need %0d. (read-only mode, no auto-generate)",
                   $time, nb, ELEMS);
        end

        // 최종 로드
        read_trace_fp32(A_bits, B_bits);
        $display("[%0t] using existing traces (read-only): X=%0d, W=%0d", $time, na, nb);
    endtask


    // 콘솔에 입력 일부 덤프(확인용, 옵션)
    task automatic dump_inputs_console();
        int r,c;
        $display("\n[INPUT X]");
        for (r=0; r<MAT_SIZE; r++) begin
            $write("  ");
            for (c=0; c<MAT_SIZE; c++) $write("%0.6f ", A[r][c]);
            $write("\n");
        end
        $display("[INPUT W]");
        for (r=0; r<MAT_SIZE; r++) begin
            $write("  ");
            for (c=0; c<MAT_SIZE; c++) $write("%0.6f ", B[r][c]);
            $write("\n");
        end
    endtask

    // -------------- I/O model -------------------
    task automatic init_arrays();
        int r, c, idx;
        begin
            for (r=0; r<MAT_SIZE; r++)
                for (c=0; c<MAT_SIZE; c++) begin
                    qA_sw [r][c] = 0; qB_sw [r][c] = 0;
                    qA_dut[r][c] = 0; qB_dut[r][c] = 0;
                end
            for (idx=0; idx<ELEMS; idx++) begin
                Cdut_bits[idx] = 32'h0;
                Cexp_bits[idx] = 32'h0;
            end
        end
    endtask

    task automatic drive_A_tile();
        int beat,lane,idx;
        begin
            a_s_valid_i = 1'b0;
            a_s_data_i  = '0;
            @(posedge clk);
            for (beat=0; beat<BEATS; beat++) begin
                for (lane=0; lane<LANES_NUM; lane++) begin
                    idx = beat*LANES_NUM + lane;
                    a_s_data_i[(lane+1)*FP_DATA_W-1 -: FP_DATA_W] = (idx < ELEMS) ? A_bits[idx] : '0;
                end
                a_s_valid_i = 1'b1;
                while (!a_s_ready_o) @(posedge clk);
                @(posedge clk);
                a_s_valid_i = 1'b0;
            end
        end
    endtask

    task automatic drive_B_tile();
        int beat,lane,idx;
        begin
            b_s_valid_i = 1'b0;
            b_s_data_i  = '0;
            @(posedge clk);
            for (beat=0; beat<BEATS; beat++) begin
                for (lane=0; lane<LANES_NUM; lane++) begin
                    idx = beat*LANES_NUM + lane;
                    b_s_data_i[(lane+1)*FP_DATA_W-1 -: FP_DATA_W] = (idx < ELEMS) ? B_bits[idx] : '0;
                end
                b_s_valid_i = 1'b1;
                while (!b_s_ready_o) @(posedge clk);
                @(posedge clk);
                b_s_valid_i = 1'b0;
            end
        end
    endtask

    task automatic drive_ACC_to_dq();
        int beat,lane,idx,r,c;
        int acc_val;
        begin
            dq_s_valid_i = 1'b0;
            dq_s_data_i  = '0;
            dq_tfirst_i  = 1'b0;
            @(posedge clk);
            for (beat=0; beat<BEATS; beat++) begin
                for (lane=0; lane<LANES_NUM; lane++) begin
                    idx = beat*LANES_NUM + lane;
                    if (idx < ELEMS) begin
                        r = idx / MAT_SIZE;
                        c = idx % MAT_SIZE;
                        acc_val = ACC[r][c];
                        dq_s_data_i[(lane+1)*FP_DATA_W-1 -: FP_DATA_W] = $signed(acc_val);
                    end else begin
                        dq_s_data_i[(lane+1)*FP_DATA_W-1 -: FP_DATA_W] = '0;
                    end
                end
                dq_tfirst_i  = (beat==0);
                dq_s_valid_i = 1'b1;
                while (!dq_s_ready_o) @(posedge clk);
                @(posedge clk);
                dq_s_valid_i = 1'b0;
            end
            dq_tfirst_i = 1'b0;
        end
    endtask

    // 캡처용
    int a_q_recv, b_q_recv, dq_recv;
    int laneA, idxA, rA, cA;
    int laneB, idxB, rB, cB;
    int laneD, idxD;

    always @(posedge clk) begin
        if (a_m_valid_o && a_m_ready_i) begin
            for (laneA=0; laneA<LANES_NUM; laneA++) begin
                idxA = a_q_recv + laneA;
                if (idxA < ELEMS) begin
                    rA = idxA / MAT_SIZE;
                    cA = idxA % MAT_SIZE;
                    qA_dut[rA][cA] = $signed(a_m_data_o[(laneA+1)*FP_DATA_W-1 -: FP_DATA_W]);
                end
            end
            a_q_recv += LANES_NUM;
        end
        if (b_m_valid_o && b_m_ready_i) begin
            for (laneB=0; laneB<LANES_NUM; laneB++) begin
                idxB = b_q_recv + laneB;
                if (idxB < ELEMS) begin
                    rB = idxB / MAT_SIZE;
                    cB = idxB % MAT_SIZE;
                    qB_dut[rB][cB] = $signed(b_m_data_o[(laneB+1)*FP_DATA_W-1 -: FP_DATA_W]);
                end
            end
            b_q_recv += LANES_NUM;
        end
    end

    always @(posedge clk) begin
        if (dq_m_valid_o && dq_m_ready_i) begin
            for (laneD=0; laneD<LANES_NUM; laneD++) begin
                idxD = dq_recv + laneD;
                if (idxD < ELEMS) begin
                    Cdut_bits[idxD] = dq_m_data_o[(laneD+1)*FP_DATA_W-1 -: FP_DATA_W];
                end
            end
            dq_recv += LANES_NUM;
        end
    end

`ifdef TB_HAS_SCALE_PORTS
    // RTL 스케일 캡처
    always @(posedge clk or negedge rstnn) begin
        integer i;
        if (!rstnn) begin
            a_scale_captured <= 1'b0;
            b_scale_captured <= 1'b0;
        end else begin
            if (a_scl_valid_o && a_scl_ready_i && !a_scale_captured) begin
                for (i=0; i<MAT_SIZE; i++) begin
                    A_mant_dut[i] = a_mantissa_scale_o[(i+1)*FP_MANT_W-1 -: FP_MANT_W];
                    A_exp_dut [i] = a_exp_scale_o     [(i+1)*FP_EXP_W -1 -: FP_EXP_W ];
                    A_scale_real_dut[i] = mant_exp_to_real(A_mant_dut[i], A_exp_dut[i]);
                end
                a_scale_captured <= 1'b1;
            end
            if (b_scl_valid_o && b_scl_ready_i && !b_scale_captured) begin
                for (i=0; i<MAT_SIZE; i++) begin
                    B_mant_dut[i] = b_mantissa_scale_o[(i+1)*FP_MANT_W-1 -: FP_MANT_W];
                    B_exp_dut [i] = b_exp_scale_o     [(i+1)*FP_EXP_W -1 -: FP_EXP_W ];
                    B_scale_real_dut[i] = mant_exp_to_real(B_mant_dut[i], B_exp_dut[i]);
                end
                b_scale_captured <= 1'b1;
            end
        end
    end
`endif

    // ----------- File writes & checks -----------
    task automatic write_trace_int_q(
        input string fname,
        input int Q [0:MAT_SIZE-1][0:MAT_SIZE-1]
    );
        int fd,r,c;
        begin
            fd = $fopen(fname, "w");
            if (!fd) begin $display("ERR: open %s", fname); $finish; end
            for (r=0;r<MAT_SIZE;r++) begin
                for (c=0;c<MAT_SIZE;c++) $fwrite(fd, "%0d ", Q[r][c]);
                $fwrite(fd, "\n");
            end
            $fclose(fd);
        end
    endtask

    task automatic write_trace_fp32_mat(
        input string fname,
        input reg [FP_DATA_W-1:0] M [0:ELEMS-1]
    );
        int fd,idx,r,c;
        begin
            fd = $fopen(fname, "w");
            if (!fd) begin $display("ERR: open %s", fname); $finish; end
            for (r=0;r<MAT_SIZE;r++) begin
                for (c=0;c<MAT_SIZE;c++) begin
                    idx = r*MAT_SIZE + c;
                    $fwrite(fd, "%f ", fp32_to_real(M[idx]));
                end
                $fwrite(fd, "\n");
            end
            $fclose(fd);
        end
    endtask

    task automatic write_error_rate(
        input reg [FP_DATA_W-1:0] GOLD [0:ELEMS-1],
        input reg [FP_DATA_W-1:0] DUT  [0:ELEMS-1],
        input string fname
    );
        int fd, idx, r, c;
        real g, d, err, rel, sumrel;
        begin
            fd = $fopen(fname, "w");
            if (!fd) begin $display("ERR: open %s", fname); $finish; end
            sumrel = 0.0;
            for (r=0;r<MAT_SIZE;r++) begin
                for (c=0;c<MAT_SIZE;c++) begin
                    idx = r*MAT_SIZE + c;
                    g = fp32_to_real(GOLD[idx]);
                    d = fp32_to_real(DUT [idx]);
                    err = (g>d)?(g-d):(d-g);
                    rel = (g==0.0)? err : err/((g>0)?g:-g);
                    sumrel += rel;
                    $fwrite(fd, "Err[%0d,%0d] = %e (rel=%e)\n", r,c, err, rel);
                end
            end
            $fwrite(fd, "\nAvg Rel Err = %e\n", sumrel/(MAT_SIZE*MAT_SIZE));
            $fclose(fd);
        end
    endtask

    // -------------- SW model --------------------
    task automatic read_trace_fp32(
        output reg [FP_DATA_W-1:0] A_bits [0:ELEMS-1],
        output reg [FP_DATA_W-1:0] B_bits [0:ELEMS-1]
    );
        integer fpA, fpB, r, c, idx, rcA, rcB;
        real ax, bx;
        shortreal s;
        begin
            fpA = $fopen(`TRACE_IN_X, "r");
            fpB = $fopen(`TRACE_IN_W, "r");
            if (!fpA || !fpB) begin
                $display("[%0t] ERR: cannot open input traces", $time); $finish;
            end
            for (r=0; r<MAT_SIZE; r++) begin
                for (c=0; c<MAT_SIZE; c++) begin
                    idx = r*MAT_SIZE + c;
                    rcA = $fscanf(fpA, "%f", ax);
                    rcB = $fscanf(fpB, "%f", bx);
                    if (rcA!=1 || rcB!=1) begin
                        $display("[%0t] ERR: fscanf failed at (%0d,%0d): rcA=%0d rcB=%0d", $time, r,c, rcA,rcB);
                        $finish;
                    end
                    s = shortreal'(ax); A_bits[idx] = $shortrealtobits(s);
                    s = shortreal'(bx); B_bits[idx] = $shortrealtobits(s);
                    A[r][c] = ax; B[r][c] = bx;
                end
            end
            $fclose(fpA); $fclose(fpB);

            // 디버그: 입력 일부 찍기
            $display("[DBG] A(0,:)= %e %e %e %e",
                A[0][0],A[0][1],A[0][2],A[0][3]);
            $display("[DBG] B(0,:)= %e %e %e %e",
                B[0][0],B[0][1],B[0][2],B[0][3]);
        end
    endtask

    task automatic build_sw_scales_and_acc;
        int r,c,k,j,sacc;
        int QMAX, QMIN;
        real qf;
        real amax, bmax, av, bv;
        real den, val;
        begin
            QMAX = (1<<(BIT_NUM-1)) - 1;
            QMIN = -(1<<(BIT_NUM-1));

            // 행별 absmax
            for (r=0; r<MAT_SIZE; r++) begin
                amax = 0.0; bmax = 0.0;
                for (c=0; c<MAT_SIZE; c++) begin
                    av = (A[r][c]>=0.0)?A[r][c]:-A[r][c];
                    bv = (B[r][c]>=0.0)?B[r][c]:-B[r][c];
                    if (av>amax) amax = av;
                    if (bv>bmax) bmax = bv;
                end
                if (amax==0.0) amax = 1.0;
                if (bmax==0.0) bmax = 1.0;
                A_absmax_row[r] = amax;
                B_absmax_row[r] = bmax;
            end

            // SW 양자화 (qA_sw/qB_sw)
            for (r=0; r<MAT_SIZE; r++) begin
                for (c=0; c<MAT_SIZE; c++) begin
                    if (A_absmax_row[r]==0.0) qA_sw[r][c] = 0;
                    else begin
                        qf = A[r][c]/A_absmax_row[r]*QMAX;
                        qA_sw[r][c] = (qf>=0.0)? clamp_int($rtoi(qf+0.5),QMIN,QMAX)
                                              : clamp_int($rtoi(qf-0.5),QMIN,QMAX);
                    end
                    if (B_absmax_row[r]==0.0) qB_sw[r][c] = 0;
                    else begin
                        qf = B[r][c]/B_absmax_row[r]*QMAX;
                        qB_sw[r][c] = (qf>=0.0)? clamp_int($rtoi(qf+0.5),QMIN,QMAX)
                                              : clamp_int($rtoi(qf-0.5),QMIN,QMAX);
                    end
                end
            end

            // ACC = qA_sw(i,k) * qB_sw(j,k) sum_k
            for (r=0; r<MAT_SIZE; r++) begin
                for (j=0; j<MAT_SIZE; j++) begin
                    sacc = 0;
                    for (k=0; k<MAT_SIZE; k++) sacc += qA_sw[r][k]*qB_sw[j][k];
                    ACC[r][j] = sacc;
                end
            end

            // 기대 복원값 Cexp(i,j) = ACC * (Amax[i]*Bmax[j]/QMAX^2)
            den = ( ( (1<<(BIT_NUM-1)) - 1 ) * ( (1<<(BIT_NUM-1)) - 1 ) );
            for (r=0; r<MAT_SIZE; r++) begin
                for (j=0; j<MAT_SIZE; j++) begin
                    val = ACC[r][j]*(A_absmax_row[r]*B_absmax_row[j]/den);
                    Cexp[r][j] = val;
                    Cexp_bits[r*MAT_SIZE+j] = real_to_fp32(val);
                end
            end
        end
        // === SW absmax를 FP비트로 분해하여 mant/exp/real 저장 ===
        begin : sw_scale_bits_extract
            int r; shortreal s; bit [FP_DATA_W-1:0] bitsA, bitsB;
            for (r=0; r<MAT_SIZE; r++) begin
                s = shortreal'(A_absmax_row[r]); bitsA = $shortrealtobits(s);
                s = shortreal'(B_absmax_row[r]); bitsB = $shortrealtobits(s);

                A_mant_sw[r] = bitsA[FP_MANT_W-1:0];
                A_exp_sw [r] = bitsA[FP_MANT_W+FP_EXP_W-1 : FP_MANT_W];
                B_mant_sw[r] = bitsB[FP_MANT_W-1:0];
                B_exp_sw [r] = bitsB[FP_MANT_W+FP_EXP_W-1 : FP_MANT_W];

                A_scale_real_sw[r] = A_absmax_row[r];
                B_scale_real_sw[r] = B_absmax_row[r];
            end
        end
    endtask

    // --- SW 스케일 outer product를 FP32로 쪼개어 (mant, exp) 한줄씩 출력 ---
    task automatic print_sw_scale_outer_product();
        int r, j;
        real prod;
        shortreal s;
        bit [FP_DATA_W-1:0] bits;
        bit [FP_MANT_W-1:0] mant;
        bit [FP_EXP_W-1:0]  exp;
        begin
            $display("\n[SW] Outer product (A_scale_real_sw x B_scale_real_sw), one line per value (mant/exp):");
            for (r = 0; r < MAT_SIZE; r++) begin
                for (j = 0; j < MAT_SIZE; j++) begin
                    // 실수값 곱 → FP32 비트 → mant/exp 추출
                    prod = A_scale_real_sw[r] * B_scale_real_sw[j];
                    s    = shortreal'(prod);
                    bits = $shortrealtobits(s);
                    mant = bits[FP_MANT_W-1:0];
                    exp  = bits[FP_MANT_W+FP_EXP_W-1 : FP_MANT_W];

                    // 한 값당 한 줄: 인덱스, mant(HEX), exp(DEC), (옵션) 실수값
                    $display("S[%0d,%0d] mant=0x%0h exp=%0d val=%e", r, j, mant, exp, prod);
                end
            end
        end
    endtask

    // ----------- 콘솔 출력 유틸 -----------
    task automatic print_scale_table_sw();
        int r;
        begin
            $display("\n[SW] A/B scales (idx: A_mant(hex),A_exp,A_real | B_mant(hex),B_exp,B_real)");
            for (r=0;r<MAT_SIZE;r++)
                $display("  [%0d]: 0x%0h,%0d,%e | 0x%0h,%0d,%e",
                    r, A_mant_sw[r], A_exp_sw[r], A_scale_real_sw[r],
                       B_mant_sw[r], B_exp_sw[r], B_scale_real_sw[r]);
        end
    endtask

`ifdef TB_HAS_SCALE_PORTS
    task automatic print_scale_table_rtl();
        int r;
        begin
            if (!(a_scale_captured && b_scale_captured)) begin
                $display("\n[RTL] Scale not captured yet (no valid seen).");
            end else begin
                $display("\n[RTL] A/B scales (idx: A_mant(hex),A_exp,A_real | B_mant(hex),B_exp,B_real)");
                for (r=0;r<MAT_SIZE;r++)
                    $display("  [%0d]: 0x%0h,%0d,%e | 0x%0h,%0d,%e",
                        r, A_mant_dut[r], A_exp_dut[r], A_scale_real_dut[r],
                           B_mant_dut[r], B_exp_dut[r], B_scale_real_dut[r]);
            end
        end
    endtask

    task automatic print_scale_diff();
        int r; real diff, rel;
        begin
            if (!(a_scale_captured && b_scale_captured)) begin
                $display("\n[DIFF] Skip scale diff (RTL not captured).");
            end else begin
                $display("\n[DIFF] A scale: idx  sw  rtl  |abs|  rel");
                for (r=0;r<MAT_SIZE;r++) begin
                    diff = (A_scale_real_sw[r] > A_scale_real_dut[r]) ?
                           (A_scale_real_sw[r] - A_scale_real_dut[r]) :
                           (A_scale_real_dut[r] - A_scale_real_sw[r]);
                    rel  = (A_scale_real_sw[r]==0.0) ? diff :
                           diff / ((A_scale_real_sw[r]>0)?A_scale_real_sw[r]:-A_scale_real_sw[r]);
                    $display("  A[%0d]: %e  %e  %e  %e", r, A_scale_real_sw[r], A_scale_real_dut[r], diff, rel);
                end
                $display("[DIFF] B scale: idx  sw  rtl  |abs|  rel");
                for (r=0;r<MAT_SIZE;r++) begin
                    diff = (B_scale_real_sw[r] > B_scale_real_dut[r]) ?
                           (B_scale_real_sw[r] - B_scale_real_dut[r]) :
                           (B_scale_real_dut[r] - B_scale_real_sw[r]);
                    rel  = (B_scale_real_sw[r]==0.0) ? diff :
                           diff / ((B_scale_real_sw[r]>0)?B_scale_real_sw[r]:-B_scale_real_sw[r]);
                    $display("  B[%0d]: %e  %e  %e  %e", r, B_scale_real_sw[r], B_scale_real_dut[r], diff, rel);
                end
            end
        end
    endtask
`endif
    
    task automatic print_matrix_diff(
        input reg [FP_DATA_W-1:0] GOLD [0:ELEMS-1],
        input reg [FP_DATA_W-1:0] DUT  [0:ELEMS-1]
    );
        int  r, c, idx;
        real g, d, err, rel;
        real sumrel;                  // element-wise mean relative error 누적
        real sum_sq_diff, sum_sq_gold; // Frobenius용 누적 제곱합
        real frob_rel;                // Frobenius 상대오차
        real max_abs_err;             // 참고: 최대 절대오차
        int  max_r, max_c;
    
        begin
            sumrel = 0.0;
            sum_sq_diff = 0.0;
            sum_sq_gold = 0.0;
            max_abs_err = 0.0; max_r = 0; max_c = 0;
    
            $display("\n[C DIFF] SW vs RTL (i,j: SW  RTL  |abs|  rel)");
            for (r = 0; r < MAT_SIZE; r++) begin
                for (c = 0; c < MAT_SIZE; c++) begin
                    idx = r*MAT_SIZE + c;
                    g = fp32_to_real(GOLD[idx]);
                    d = fp32_to_real(DUT [idx]);
    
                    err = (g > d) ? (g - d) : (d - g);
                    rel = (g == 0.0) ? err : (err / ((g > 0.0) ? g : -g));
    
                    sumrel      += rel;
                    sum_sq_diff += (g - d) * (g - d);
                    sum_sq_gold += (g * g);
    
                    if (err > max_abs_err) begin
                        max_abs_err = err;
                        max_r = r; max_c = c;
                    end
    
                    $display("  C[%0d,%0d]: %e  %e  %e  %e", r, c, g, d, err, rel);
                end
            end
    
            // Frobenius 상대오차: ||G-D||_F / ||G||_F
            if (sum_sq_gold == 0.0) begin
                // gold가 전부 0이면: diff도 0이면 0, 아니면 1로 표시
                frob_rel = (sum_sq_diff == 0.0) ? 0.0 : 1.0;
            end else begin
                frob_rel = $sqrt(sum_sq_diff) / $sqrt(sum_sq_gold);
            end
    
            $display("[C DIFF] Avg Rel Err (element-wise) = %e", sumrel / (MAT_SIZE*MAT_SIZE));
            $display("[C DIFF] Frobenius Rel Err          = %e", frob_rel);
            $display("[C DIFF] Max |abs err| at (%0d,%0d) = %e\n", max_r, max_c, max_abs_err);
        end
    endtask


    // 2^e
    function real two_power(input integer e);
        integer i; real p;
        begin
            p = 1.0;
            if (e >= 0) begin
                for (i=0;i<e;i++) p = p*2.0;
            end else begin
                for (i=0;i<-e;i++) p = p/2.0;
            end
            two_power = p;
        end
    endfunction

    function real mant_exp_to_real(
        input [FP_MANT_W-1:0] mant,
        input [FP_EXP_W -1:0] exp
    );
        integer e; real frac;
        begin
            e    = $signed({1'b0, exp}) - FP_EXP_BIAS;
            frac = 1.0 + real'(mant) / real'(1<<FP_MANT_W);
            mant_exp_to_real = frac * two_power(e);
        end
    endfunction

    // ---------------- Scenario ------------------
    initial begin : MAIN
        // ===== 선언을 블록 맨 위로 몰아넣기 =====
        integer cycles;
        integer i;
        bit done;  // dequant 수신 완료 플래그
        reg [FP_DATA_W-1:0] Cdut_padded [0:ELEMS-1];
        // 전역에 bit done; 이 이미 있으면 여기서 done 재선언하지 마세요.
        // (전역 done을 그대로 사용)

        // ===== 실행문 시작 =====
        // trace 폴더 생성(윈도우)
        void'($system("if not exist trace mkdir trace"));

        // init
        rstnn = 1'b0;
        a_s_valid_i = 0; a_s_data_i = '0;
        b_s_valid_i = 0; b_s_data_i = '0;
        dq_s_valid_i= 0; dq_s_data_i= '0; dq_tfirst_i=0;
        a_m_ready_i = 1'b1;
        b_m_ready_i = 1'b1;
        dq_m_ready_i= 1'b1;
        a_q_recv = 0; b_q_recv = 0; dq_recv = 0;

    `ifdef TB_HAS_SCALE_PORTS
        a_scale_captured = 1'b0;
        b_scale_captured = 1'b0;
        a_scl_ready_i    = 1'b1;
        b_scl_ready_i    = 1'b1;
    `endif

        init_arrays();

        $display("[%0t] TB start", $time);
        repeat (5) @(posedge clk);
        rstnn = 1'b1;
        repeat (3) @(posedge clk);

        // 입력 보장(없으면 생성→파일에 기록) + 메모리로 로드
        puts_and_load();
        // dump_inputs_console();  // 옵션: 입력 확인
        $display("[%0t] Inputs ready", $time);

        // SW 스케일/양자화/ACC/기대 복원값
        build_sw_scales_and_acc();
        $display("[%0t] SW scale/quant/acc built", $time);

        // A/B 타일 투입
        fork
            drive_A_tile();
            drive_B_tile();
        join
        $display("[%0t] A/B tiles pushed", $time);

        // dequant용 ACC 투입
        drive_ACC_to_dq();
        $display("[%0t] ACC pushed to dequant", $time);

        // 수신 완료 대기(워치독)
        done = 0;                  // 전역 done 사용 (여기서 재선언 금지)
        fork
            begin : WAIT_DONE
                wait (dq_recv >= ELEMS);
                done = 1;
                $display("[%0t] Dequant received all (%0d/%0d)", $time, dq_recv, ELEMS);
            end
            begin : WATCHDOG
                cycles = WATCHDOG_CYCLES;  // 선언과 분리해서 대입
                repeat (cycles) @(posedge clk);
                if (!done)
                    $display("[%0t] WARN: timeout waiting dequant (%0d/%0d). Continue.", $time, dq_recv, ELEMS);
            end
        join_any
        disable fork;

        // RTL C 출력 패딩/절단 (받은 만큼 사용)
        for (i=0;i<ELEMS;i++)
            Cdut_padded[i] = (i<dq_recv) ? Cdut_bits[i] : 32'h0;

        // 파일 쓰기: SW/RTL 각각 분리 덤프
        write_trace_int_q(`TRACE_IN_Q_X_SW , qA_sw );
        write_trace_int_q(`TRACE_IN_Q_W_SW , qB_sw );
        write_trace_int_q(`TRACE_IN_Q_X_RTL, qA_dut);
        write_trace_int_q(`TRACE_IN_Q_W_RTL, qB_dut);
        write_trace_fp32_mat(`TRACE_OT_RESULT,     Cexp_bits);
        write_trace_fp32_mat(`TRACE_OT_RTL_RESULT, Cdut_padded);
        write_error_rate      (Cexp_bits, Cdut_padded, `TRACE_OT_ERROR);

        // 콘솔 출력
        print_scale_table_sw();
        print_sw_scale_outer_product();
    `ifdef TB_HAS_SCALE_PORTS
        print_scale_table_rtl();
        print_scale_diff();
    `else
        $display("\n[INFO] RTL scale ports 미사용. (+define+TB_HAS_SCALE_PORTS 로 활성화 가능)");
    `endif
        print_matrix_diff(Cexp_bits, Cdut_padded);

        $display("\n*** TB DONE (MAT_SIZE=%0d, LANES=%0d) ***\n", MAT_SIZE, LANES_NUM);
        #50 $finish;
    end


endmodule
