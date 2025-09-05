module scale_fifo #(
    parameter integer MAT_SIZE            = 16,
    parameter integer FP_MANT_W           = 23,
    parameter integer FP_EXP_W            = 8,
    parameter integer DEPTH               = 4,   // 엔트리 개수
    parameter integer AFULL_MARGIN        = 1,   // almost_full 여유
    parameter integer AEMPTY_MARGIN       = 1    // almost_empty 경계
)(
    input  wire clk,
    input  wire rstnn,

    // write(push)
    input  wire                         wr_valid_i,
    output wire                         wr_ready_o,
    input  wire [FP_MANT_W*MAT_SIZE*MAT_SIZE-1:0] mant_in_i,
    input  wire [FP_EXP_W *MAT_SIZE*MAT_SIZE-1:0] exp_in_i,

    // read(pop)
    output wire                         rd_valid_o,
    input  wire                         rd_ready_i,
    output wire [FP_MANT_W*MAT_SIZE*MAT_SIZE-1:0] mant_out_o,
    output wire [FP_EXP_W *MAT_SIZE*MAT_SIZE-1:0] exp_out_o,

    // control / status
    input  wire                         flush_i,
    output wire                         empty_o,
    output wire                         full_o,
    output wire                         almost_empty_o,
    output wire                         almost_full_o
);
    // --------------------------------------------
    // utils
    function integer clog2; input integer v; integer i; begin i=0; while((1<<i)<v) i=i+1; clog2=i; end endfunction
    localparam integer MANT_MAT_W = FP_MANT_W*MAT_SIZE*MAT_SIZE;
    localparam integer EXP_MAT_W  = FP_EXP_W *MAT_SIZE*MAT_SIZE;

    // --------------------------------------------
    // storage
    reg [MANT_MAT_W-1:0] mem_mant [0:DEPTH-1];
    reg [EXP_MAT_W -1:0] mem_exp  [0:DEPTH-1];

    reg [clog2(DEPTH)-1:0] head, tail;
    reg [clog2(DEPTH+1)-1:0] count;

    wire full  = (count == DEPTH);
    wire empty = (count == 0);

    assign wr_ready_o      = ~full;
    assign rd_valid_o      = ~empty;
    assign empty_o         = empty;
    assign full_o          = full;
    assign almost_full_o   = (count >= (DEPTH - AFULL_MARGIN));
    assign almost_empty_o  = (count <= AEMPTY_MARGIN);

    // show-ahead: 현재 head 내용이 즉시 out으로 보임
    assign mant_out_o = mem_mant[head];
    assign exp_out_o  = mem_exp [head];

    // --------------------------------------------
    // push/pop
    wire do_push = wr_valid_i & wr_ready_o;
    wire do_pop  = rd_valid_o & rd_ready_i;

    always @(posedge clk or negedge rstnn) begin
        if (!rstnn) begin
            head  <= {clog2(DEPTH){1'b0}};
            tail  <= {clog2(DEPTH){1'b0}};
            count <= {clog2(DEPTH+1){1'b0}};
        end else begin
            if (flush_i) begin
                head  <= {clog2(DEPTH){1'b0}};
                tail  <= {clog2(DEPTH){1'b0}};
                count <= {clog2(DEPTH+1){1'b0}};
            end else begin
                // write
                if (do_push) begin
                    mem_mant[tail] <= mant_in_i;
                    mem_exp [tail] <= exp_in_i;
                    tail <= (tail == DEPTH-1) ? {clog2(DEPTH){1'b0}} : (tail + 1'b1);
                end
                // read
                if (do_pop) begin
                    head <= (head == DEPTH-1) ? {clog2(DEPTH){1'b0}} : (head + 1'b1);
                end
                // count update
                case ({do_push, do_pop})
                    2'b10: count <= count + 1'b1;
                    2'b01: count <= count - 1'b1;
                    default: /* no change */ ;
                endcase
            end
        end
    end


    /////////////////////////////////////////////////////////////
    // 디버깅용 로직
    /////////////////////////////////////////////////////////////
    // 플랫 입력 버스 -> 2차원 reg (웨이브 보기용)
    reg [FP_MANT_W-1:0] mant_in_reg [0:MAT_SIZE-1][0:MAT_SIZE-1];
    reg [FP_EXP_W -1:0] exp_in_reg  [0:MAT_SIZE-1][0:MAT_SIZE-1];
    
    integer __r, __c, __idx;
    always @* begin
        for (__r = 0; __r < MAT_SIZE; __r = __r + 1) begin
            for (__c = 0; __c < MAT_SIZE; __c = __c + 1) begin
                __idx = __r*MAT_SIZE + __c;
                mant_in_reg[__r][__c] = mant_in_i[(__idx+1)*FP_MANT_W-1 -: FP_MANT_W];
                exp_in_reg [__r][__c] = exp_in_i [(__idx+1)*FP_EXP_W -1  -: FP_EXP_W];
            end
        end
    end

endmodule