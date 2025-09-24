`timescale 1ns/1ps
// ============================================================================
// DCA_MATRIX_QDQ_MMIOX_MLSU
// [X, W matrix]
//  - LOAD: MMIO → LOAD2MREG → MREG(TYPE3) → qdq controller
//  - STORE: qdq controller → MREG(TYPE3) → MREG2STORE → AXI
// [O matrix]
//  - LOAD: AXI → LOAD2MREG → MREG(TYPE3) → qdq controller
//  - STORE: qdq controller → MREG(TYPE3) → MREG2STORE → MMIO
// ============================================================================

`include "ervp_global.vh"
`include "ervp_endian.vh"
`include "dca_matrix_qgemm_defines.vh"
`include "dca_matrix_info.vh"
`include "dca_matrix_lsu_inst.vh"
`include "ervp_axi_define.vh"

module DCA_MATRIX_QGEMM_MMIOX_MLSU
(
  clk, rstnn,

  // Control
  control_rmx_core_config,
  control_rmx_core_status,
  control_rmx_clear_request,
  control_rmx_clear_finish,
  control_rmx_log_fifo_wready,
  control_rmx_log_fifo_wrequest,
  control_rmx_log_fifo_wdata,
  control_rmx_inst_fifo_rready,
  control_rmx_inst_fifo_rdata,
  control_rmx_inst_fifo_rrequest,
  control_rmx_operation_finish,
  control_rmx_input_fifo_rready,
  control_rmx_input_fifo_rdata,
  control_rmx_input_fifo_rrequest,
  control_rmx_output_fifo_wready,
  control_rmx_output_fifo_wrequest,
  control_rmx_output_fifo_wdata,

  // LSU X(A)
  mx_sinst_wvalid, mx_sinst_wdata, mx_sinst_wready,
  mx_sinst_decode_finish, mx_sinst_execute_finish, mx_sinst_busy,
  mx_sload_tensor_row_wvalid, mx_sload_tensor_row_wlast,
  mx_sload_tensor_row_wdata,  mx_sload_tensor_row_wready,
  mx_sstore_tensor_row_rvalid, mx_sstore_tensor_row_rlast,
  mx_sstore_tensor_row_rready, mx_sstore_tensor_row_rdata,

  // LSU W(B)
  mw_sinst_wvalid, mw_sinst_wdata, mw_sinst_wready,
  mw_sinst_decode_finish, mw_sinst_execute_finish, mw_sinst_busy,
  mw_sload_tensor_row_wvalid, mw_sload_tensor_row_wlast,
  mw_sload_tensor_row_wdata,  mw_sload_tensor_row_wready,
  mw_sstore_tensor_row_rvalid, mw_sstore_tensor_row_rlast,
  mw_sstore_tensor_row_rready, mw_sstore_tensor_row_rdata,

  // LSU O
  mo_sinst_wvalid, mo_sinst_wdata, mo_sinst_wready,
  mo_sinst_decode_finish, mo_sinst_execute_finish, mo_sinst_busy,
  mo_sload_tensor_row_wvalid, mo_sload_tensor_row_wlast,
  mo_sload_tensor_row_wdata,  mo_sload_tensor_row_wready,
  mo_sstore_tensor_row_rvalid, mo_sstore_tensor_row_rlast,
  mo_sstore_tensor_row_rready, mo_sstore_tensor_row_rdata,

  // AXI
	mq2vta_rxawid, mq2vta_rxawaddr, mq2vta_rxawlen, mq2vta_rxawsize, mq2vta_rxawburst, mq2vta_rxawvalid, mq2vta_rxawready,
	mq2vta_rxwid,  mq2vta_rxwdata,  mq2vta_rxwstrb,  mq2vta_rxwlast,  mq2vta_rxwvalid,  mq2vta_rxwready,
	mq2vta_rxbid,  mq2vta_rxbresp,  mq2vta_rxbvalid, mq2vta_rxbready,
	mq2vta_rxarid, mq2vta_rxaraddr, mq2vta_rxarlen, mq2vta_rxarsize, mq2vta_rxarburst, mq2vta_rxarvalid, mq2vta_rxarready,
	mq2vta_rxrid,  mq2vta_rxrdata,  mq2vta_rxrresp,  mq2vta_rxrlast,  mq2vta_rxrvalid,  mq2vta_rxrready
);

parameter BIT_NUM          = 8;
parameter FP_EXP_W         = 8;
parameter FP_MANT_W        = 23;
parameter FP_EXP_BIAS      = 127;
parameter SCALE_FIFO_DEPTH = 4;
parameter BW_AXI_DATA      = 128;
parameter BW_AXI_TID       = 4;
parameter BW_ADDR          = 32;

parameter integer INPUT_MATRIX_SIZE  = 16;
parameter integer WEIGHT_MATRIX_SIZE = 16;
parameter integer OUTPUT_MATRIX_SIZE = 16;
parameter TENSOR_PARA = 0;

parameter int PORT_QAB = 0;
parameter int PORT_ACC = 1;

parameter int QA_BASE  = 0;
parameter int QB_BASE  = 64;
parameter int ACC_BASE = 0;

localparam BW_CONFIG = 1;
localparam BW_STATUS = `BW_DCA_MATRIX_QDQ_STATUS;
localparam BW_LOG    = `BW_DCA_MATRIX_QDQ_LOG;
localparam BW_INST   = `BW_DCA_MATRIX_QDQ_INST;
localparam BW_INPUT  = 32;
localparam BW_OUTPUT = 32;

// ----------------------------------------------------------------------------
`include "dca_matrix_dim_util.vb"
`include "dca_tensor_scalar_lpara.vb"
// ----------------------------------------------------------------------------

localparam integer INPUT_MATRIX_NUM_COL  = GET_MATRIX_NUM_COL(INPUT_MATRIX_SIZE);
localparam integer WEIGHT_MATRIX_NUM_COL = GET_MATRIX_NUM_COL(WEIGHT_MATRIX_SIZE);
localparam integer OUTPUT_MATRIX_NUM_COL = GET_MATRIX_NUM_COL(OUTPUT_MATRIX_SIZE);

localparam integer BW_INPUT_TENSOR_ROW   = BW_TENSOR_SCALAR*INPUT_MATRIX_NUM_COL;
localparam integer BW_WEIGHT_TENSOR_ROW  = BW_TENSOR_SCALAR*WEIGHT_MATRIX_NUM_COL;
localparam integer BW_OUTPUT_TENSOR_ROW  = BW_TENSOR_SCALAR*OUTPUT_MATRIX_NUM_COL;

localparam [`BW_DCA_MATRIX_LSU_INST_OPCODE-1:0] OPC_READ  = `DCA_MATRIX_LSU_INST_OPCODE_READ;
localparam [`BW_DCA_MATRIX_LSU_INST_OPCODE-1:0] OPC_WRITE = `DCA_MATRIX_LSU_INST_OPCODE_WRITE;

localparam MREG_RESET_VALUE = TENSOR_ZERO;

input  wire clk;
input  wire rstnn;

// ---------------- Control ties ----------------
input  wire [(BW_CONFIG)-1:0] control_rmx_core_config;
output wire [(BW_STATUS)-1:0] control_rmx_core_status;
input  wire control_rmx_clear_request;
output wire control_rmx_clear_finish;
input  wire control_rmx_log_fifo_wready;
output wire control_rmx_log_fifo_wrequest;
output wire [(BW_LOG)-1:0] control_rmx_log_fifo_wdata;
input  wire control_rmx_inst_fifo_rready;
input  wire [(BW_INST)-1:0] control_rmx_inst_fifo_rdata;
output wire control_rmx_inst_fifo_rrequest;
output wire control_rmx_operation_finish;
input  wire control_rmx_input_fifo_rready;
input  wire [(BW_INPUT)-1:0] control_rmx_input_fifo_rdata;
output wire control_rmx_input_fifo_rrequest;
input  wire control_rmx_output_fifo_wready;
output wire control_rmx_output_fifo_wrequest;
output wire [(BW_OUTPUT)-1:0] control_rmx_output_fifo_wdata;

assign control_rmx_core_status          = '0;
assign control_rmx_clear_finish         = 1'b0;
assign control_rmx_log_fifo_wrequest    = 1'b0;
assign control_rmx_log_fifo_wdata       = '0;
assign control_rmx_input_fifo_rrequest  = 1'b0;
assign control_rmx_output_fifo_wrequest = 1'b0;
assign control_rmx_output_fifo_wdata    = '0;

// ---------------- LSU (ports) ----------------
output wire mx_sinst_wvalid;
output wire [(`BW_DCA_MATRIX_LSU_INST)-1:0] mx_sinst_wdata;
input  wire mx_sinst_wready, mx_sinst_decode_finish, mx_sinst_execute_finish, mx_sinst_busy;
input  wire mx_sload_tensor_row_wvalid, mx_sload_tensor_row_wlast;
input  wire [BW_INPUT_TENSOR_ROW-1:0] mx_sload_tensor_row_wdata;
output wire mx_sload_tensor_row_wready;
input  wire mx_sstore_tensor_row_rvalid, mx_sstore_tensor_row_rlast;
output wire mx_sstore_tensor_row_rready;
output wire [BW_INPUT_TENSOR_ROW-1:0] mx_sstore_tensor_row_rdata;

output wire mw_sinst_wvalid;
output wire [(`BW_DCA_MATRIX_LSU_INST)-1:0] mw_sinst_wdata;
input  wire mw_sinst_wready, mw_sinst_decode_finish, mw_sinst_execute_finish, mw_sinst_busy;
input  wire mw_sload_tensor_row_wvalid, mw_sload_tensor_row_wlast;
input  wire [BW_WEIGHT_TENSOR_ROW-1:0] mw_sload_tensor_row_wdata;
output wire mw_sload_tensor_row_wready;
input  wire mw_sstore_tensor_row_rvalid, mw_sstore_tensor_row_rlast;
output wire mw_sstore_tensor_row_rready;
output wire [BW_WEIGHT_TENSOR_ROW-1:0] mw_sstore_tensor_row_rdata;

output wire mo_sinst_wvalid;
output wire [(`BW_DCA_MATRIX_LSU_INST)-1:0] mo_sinst_wdata;
input  wire mo_sinst_wready, mo_sinst_decode_finish, mo_sinst_execute_finish, mo_sinst_busy;
input  wire mo_sload_tensor_row_wvalid, mo_sload_tensor_row_wlast;
input  wire [BW_OUTPUT_TENSOR_ROW-1:0] mo_sload_tensor_row_wdata;
output wire mo_sload_tensor_row_wready;
input  wire mo_sstore_tensor_row_rvalid, mo_sstore_tensor_row_rlast;
output wire mo_sstore_tensor_row_rready;
output wire [BW_OUTPUT_TENSOR_ROW-1:0] mo_sstore_tensor_row_rdata;

// ---------------- AXI (tied-off) ----------------
input  wire [BW_AXI_TID-1:0]   mq2vta_rxawid;
input  wire [BW_ADDR-1:0]      mq2vta_rxawaddr;
input  wire [`BW_AXI_ALEN-1:0] mq2vta_rxawlen;
input  wire [`BW_AXI_ASIZE-1:0] mq2vta_rxawsize;
input  wire [`BW_AXI_ABURST-1:0] mq2vta_rxawburst;
input  wire                    mq2vta_rxawvalid;
output wire                    mq2vta_rxawready;

input  wire [BW_AXI_TID-1:0]   mq2vta_rxwid;
input  wire [BW_AXI_DATA-1:0]  mq2vta_rxwdata;
input  wire [`BW_AXI_WSTRB(BW_AXI_DATA)-1:0] mq2vta_rxwstrb;
input  wire                    mq2vta_rxwlast;
input  wire                    mq2vta_rxwvalid;
output wire                    mq2vta_rxwready;

output wire [BW_AXI_TID-1:0]   mq2vta_rxbid;
output wire [`BW_AXI_BRESP-1:0] mq2vta_rxbresp;
output wire                    mq2vta_rxbvalid;
input  wire                    mq2vta_rxbready;

input  wire [BW_AXI_TID-1:0]   mq2vta_rxarid;
input  wire [BW_ADDR-1:0]      mq2vta_rxaraddr;
input  wire [`BW_AXI_ALEN-1:0] mq2vta_rxarlen;
input  wire [`BW_AXI_ASIZE-1:0] mq2vta_rxarsize;
input  wire [`BW_AXI_ABURST-1:0] mq2vta_rxarburst;
input  wire                    mq2vta_rxarvalid;
output wire                    mq2vta_rxarready;

output wire [BW_AXI_TID-1:0]   mq2vta_rxrid;
output wire [BW_AXI_DATA-1:0]  mq2vta_rxrdata;
output wire [`BW_AXI_RRESP-1:0] mq2vta_rxrresp;
output wire                    mq2vta_rxrlast;
output wire                    mq2vta_rxrvalid;
input  wire                    mq2vta_rxrready;

// ============================================================================
// ---------------- Inst decode (wire) → latch(q) → Q/DQ 공유 -----------------
wire [`BW_DCA_MATRIX_INFO_ALIGNED-1:0] mx_info_w, mw_info_w, mo_info_w;
wire mx_trans_w, mw_trans_w, q_start_w, dq_start_w;
assign {mo_info_w, mw_info_w, mx_info_w, mx_trans_w, mw_trans_w, q_start_w, dq_start_w}
  = control_rmx_inst_fifo_rdata;

reg [`BW_DCA_MATRIX_INFO_ALIGNED-1:0] mx_info_q, mw_info_q, mo_info_q;
reg mx_trans_q, mw_trans_q;
reg q_start_lvl, dq_start_lvl;

reg have_inst;
reg want_next_inst;

wire q_go_load, q_go_exec, q_go_store, q_go_idle;

// rready & rrequest 동시 시 래치
always @(posedge clk or negedge rstnn) begin
  if(!rstnn) begin
    have_inst      <= 1'b0;
    want_next_inst <= 1'b0;
    {mx_info_q, mw_info_q, mo_info_q} <= '0;
    {mx_trans_q, mw_trans_q} <= 2'b0;
    {q_start_lvl, dq_start_lvl} <= 2'b0;
  end else begin
    if (control_rmx_inst_fifo_rready && control_rmx_inst_fifo_rrequest) begin
      have_inst      <= 1'b1;
      mx_info_q      <= mx_info_w;
      mw_info_q      <= mw_info_w;
      mo_info_q      <= mo_info_w;
      mx_trans_q     <= mx_trans_w;
      mw_trans_q     <= mw_trans_w;
      q_start_lvl    <= q_start_w;
      dq_start_lvl   <= dq_start_w;
      want_next_inst <= 1'b0;
    end
    if (q_go_store)
      want_next_inst <= 1'b1;
  end
end

// ============================================================================
// ---------------- Dual FSM ----------------
localparam [1:0] Q_IDLE=2'd0, Q_LOAD=2'd1, Q_EXEC=2'd2, Q_STORE=2'd3;
localparam [1:0] DQ_IDLE=2'd0,            DQ_EXEC=2'd1, DQ_STORE=2'd2;

reg [1:0] q_state, dq_state;
wire dq_go_exec, dq_go_store, dq_go_idle;

assign control_rmx_inst_fifo_rrequest =
       ((q_state==Q_IDLE) & ~have_inst) | want_next_inst;

always @(posedge clk or negedge rstnn) begin
  if(!rstnn) q_state <= Q_IDLE;
  else case(q_state)
    Q_IDLE : if(q_go_load)  q_state <= Q_LOAD;
    Q_LOAD : if(q_go_exec)  q_state <= Q_EXEC;
    Q_EXEC : if(q_go_store) q_state <= Q_STORE;
    Q_STORE:                q_state <= Q_IDLE;
  endcase
end

always @(posedge clk or negedge rstnn) begin
  if(!rstnn) dq_state <= DQ_IDLE;
  else case(dq_state)
    DQ_IDLE : if(dq_go_exec)  dq_state <= DQ_EXEC;
    DQ_EXEC : if(dq_go_store) dq_state <= DQ_STORE;
    DQ_STORE:                 dq_state <= DQ_IDLE;
  endcase
end

// ============================================================================
// ---------------- LSU 인스트 발사 (컨트롤 FIFO 신호 의존 금지) ---------------
wire q_issue_ready = (q_state==Q_IDLE) & have_inst & mx_sinst_wready & mw_sinst_wready;

// X/W LSU READ 인스트 1클럭
assign mx_sinst_wvalid = q_issue_ready;
assign mw_sinst_wvalid = q_issue_ready;
assign mx_sinst_wdata  = {mx_info_q, OPC_READ};
assign mw_sinst_wdata  = {mw_info_q, OPC_READ};

wire q_inst_fire = q_issue_ready;
assign q_go_load = q_inst_fire;

// ============================================================================
// ---------------- A/B transpose 1클럭 펄스 + renable 게이팅 ------------------
// Q_EXEC 들어가는 순간 arm → Q_EXEC에서 정확히 1클럭 pulse를 transpose로 사용
reg  a_do_trans, b_do_trans;
wire a_trans_pulse = (q_state==Q_EXEC) & a_do_trans;
wire b_trans_pulse = (q_state==Q_EXEC) & b_do_trans;

always @(posedge clk or negedge rstnn) begin
  if (!rstnn) begin
    a_do_trans <= 1'b0;
    b_do_trans <= 1'b0;
  end else begin
    if (q_go_exec) begin
      a_do_trans <= mx_trans_q;
      b_do_trans <= mw_trans_q;
    end else begin
      if (a_trans_pulse) a_do_trans <= 1'b0;
      if (b_trans_pulse) b_do_trans <= 1'b0;
    end
  end
end

// ============================================================================
// ---------------- LOAD2MREG → MREG(TYPE3) → qdq ----------------
wire a_s_ready_o, b_s_ready_o, dq_s_ready_o;

// ===== A stream =====
localparam int A_ROWS = INPUT_MATRIX_NUM_COL;
reg  [$clog2(A_ROWS+1)-1:0] a_sent;
wire a_valid = (q_state==Q_EXEC) & (a_sent < A_ROWS);

wire a_load_wready, a_load_rready, a_load_busy;
wire a_src_mreg_wen, a_src_mreg_ren;
wire [BW_INPUT_TENSOR_ROW-1:0] a_src_mreg_wdata, a_src_mreg_rdata;

assign mx_sload_tensor_row_wready = a_load_wready;

DCA_MATRIX_LOAD2MREG #(
  .MATRIX_SIZE_PARA(INPUT_MATRIX_SIZE), .TENSOR_PARA(0)
) i_load2mreg_A (
  .clk(clk), .rstnn(rstnn),
  .clear(1'b0), .enable(1'b1), .busy(a_load_busy),
  .load_tensor_row_wvalid (mx_sload_tensor_row_wvalid),
  .load_tensor_row_wlast  (mx_sload_tensor_row_wlast),
  .load_tensor_row_wdata  (mx_sload_tensor_row_wdata),
  .load_tensor_row_wready (a_load_wready),

  .mreg_move_wenable      (a_src_mreg_wen),
  .mreg_move_wdata_list1d (a_src_mreg_wdata),

  .loadreg_rready         (a_load_rready),
  .loadreg_rrequest       (1'b1)
);

DCA_MATRIX_REGISTER_TYPE3 #(
  .MATRIX_SIZE_PARA(INPUT_MATRIX_SIZE),
  .BW_TENSOR_SCALAR(BW_TENSOR_SCALAR),
  .BW_MOVE_DATA(BW_INPUT_TENSOR_ROW),
  .RESET_VALUE(MREG_RESET_VALUE)
) i_mregA (
  .clk(clk), .rstnn(rstnn),
  .move_wenable (a_src_mreg_wen),
  .move_wdata_list(a_src_mreg_wdata),
  .move_renable (a_src_mreg_ren),
  .move_rdata_list(a_src_mreg_rdata),
  .shift_up(1'b0), .shift_left(1'b0), .transpose(a_trans_pulse), // ← 1클럭만
  .all_rdata_list2d(), .upmost_rdata_list1d()
);

wire a_fire = a_valid & a_s_ready_o;
assign a_src_mreg_ren = a_fire & ~a_trans_pulse;  // ← transpose 펄스 싸이클엔 read 금지

wire a_s_valid_i = a_valid;
wire [BW_INPUT_TENSOR_ROW-1:0] a_s_data_i = a_src_mreg_rdata;

always @(posedge clk or negedge rstnn) begin
  if(!rstnn)          a_sent <= '0;
  else if (q_go_exec) a_sent <= '0;
  else if (a_fire & ~a_trans_pulse) a_sent <= a_sent + 1'b1;
end

// ===== B stream =====
localparam int B_ROWS = WEIGHT_MATRIX_NUM_COL;
reg  [$clog2(B_ROWS+1)-1:0] b_sent;
wire b_valid = (q_state==Q_EXEC) & (b_sent < B_ROWS);

wire b_load_wready, b_load_rready, b_load_busy;
wire b_src_mreg_wen, b_src_mreg_ren;
wire [BW_WEIGHT_TENSOR_ROW-1:0] b_src_mreg_wdata, b_src_mreg_rdata;

assign mw_sload_tensor_row_wready = b_load_wready;

DCA_MATRIX_LOAD2MREG #(
  .MATRIX_SIZE_PARA(WEIGHT_MATRIX_SIZE), .TENSOR_PARA(0)
) i_load2mreg_B (
  .clk(clk), .rstnn(rstnn),
  .clear(1'b0), .enable(1'b1), .busy(b_load_busy),
  .load_tensor_row_wvalid (mw_sload_tensor_row_wvalid),
  .load_tensor_row_wlast  (mw_sload_tensor_row_wlast),
  .load_tensor_row_wdata  (mw_sload_tensor_row_wdata),
  .load_tensor_row_wready (b_load_wready),

  .mreg_move_wenable      (b_src_mreg_wen),
  .mreg_move_wdata_list1d (b_src_mreg_wdata),

  .loadreg_rready         (b_load_rready),
  .loadreg_rrequest       (1'b1)
);

DCA_MATRIX_REGISTER_TYPE3 #(
  .MATRIX_SIZE_PARA(WEIGHT_MATRIX_SIZE),
  .BW_TENSOR_SCALAR(BW_TENSOR_SCALAR),
  .BW_MOVE_DATA(BW_WEIGHT_TENSOR_ROW),
  .RESET_VALUE(MREG_RESET_VALUE)
) i_mregB (
  .clk(clk), .rstnn(rstnn),
  .move_wenable (b_src_mreg_wen),
  .move_wdata_list(b_src_mreg_wdata),
  .move_renable (b_src_mreg_ren),
  .move_rdata_list(b_src_mreg_rdata),
  .shift_up(1'b0), .shift_left(1'b0), .transpose(b_trans_pulse), // ← 1클럭만
  .all_rdata_list2d(), .upmost_rdata_list1d()
);

wire b_fire = b_valid & b_s_ready_o;
assign b_src_mreg_ren = b_fire & ~b_trans_pulse;

wire [BW_WEIGHT_TENSOR_ROW-1:0] b_s_data_i = b_src_mreg_rdata;
wire b_s_valid_i = b_valid;

always @(posedge clk or negedge rstnn) begin
  if(!rstnn)          b_sent <= '0;
  else if (q_go_exec) b_sent <= '0;
  else if (b_fire & ~b_trans_pulse) b_sent <= b_sent + 1'b1;
end

// ===== OUT_ROWS / dq 입력 행 수 =====
localparam int OUT_ROWS = OUTPUT_MATRIX_NUM_COL;

// ============================================================================
// ---------------- qdq controller ----------------
wire a_m_valid_o, b_m_valid_o, dq_m_valid_o;
wire a_m_ready_i, b_m_ready_i, dq_m_ready_i;
wire [BW_INPUT_TENSOR_ROW-1:0]   a_m_data_o;
wire [BW_WEIGHT_TENSOR_ROW-1:0]  b_m_data_o;
wire [BW_OUTPUT_TENSOR_ROW-1:0]  dq_m_data_o;
wire [3:0]                       dq_m_index_o;
wire  [BW_OUTPUT_TENSOR_ROW-1:0] dq_s_data_i;

reg q_start_pulse, dq_start_pulse;

qdq_controller #(
  .BIT_NUM(BIT_NUM), .MAT_SIZE(OUTPUT_MATRIX_SIZE),
  .FP_DATA_W(BW_TENSOR_SCALAR), .FP_EXP_W(FP_EXP_W),
  .FP_MANT_W(FP_MANT_W), .FP_EXP_BIAS(FP_EXP_BIAS),
  .LANES_NUM(OUTPUT_MATRIX_NUM_COL), .SCALE_FIFO_DEPTH(SCALE_FIFO_DEPTH), .ROW_W_DQ(4)
) i_qdq (
  .clk(clk), .rstnn(rstnn),

  .q_start_i   ( q_start_pulse ),
  .dq_start_i  ( dq_start_pulse ),

  // A quant
  .a_s_valid_i ( a_s_valid_i ),
  .a_s_ready_o ( a_s_ready_o ),
  .a_s_data_i  ( a_s_data_i ),
  .a_m_valid_o ( a_m_valid_o ),
  .a_m_ready_i ( a_m_ready_i ),
  .a_m_data_o  ( a_m_data_o ),

  // B quant
  .b_s_valid_i ( b_s_valid_i ),
  .b_s_ready_o ( b_s_ready_o ),
  .b_s_data_i  ( b_s_data_i ),
  .b_m_valid_o ( b_m_valid_o ),
  .b_m_ready_i ( b_m_ready_i ),
  .b_m_data_o  ( b_m_data_o ),

  // ACC dequant
  .dq_s_valid_i ( dq_s_valid_i ),
  .dq_s_ready_o ( dq_s_ready_o ),
  .dq_s_data_i  ( dq_s_data_i ),
  .dq_m_valid_o ( dq_m_valid_o ),
  .dq_m_ready_i ( dq_m_ready_i ),
  .dq_m_data_o  ( dq_m_data_o ),
  .dq_m_index_o ( dq_m_index_o )
);

assign dq_m_ready_i = 1'b1;

// === Q: LOAD 완료 감시 → EXEC 진입 ===
reg a_busy_seen, b_busy_seen;
wire a_load_done  = a_busy_seen  & ~a_load_busy;
wire b_load_done  = b_busy_seen  & ~b_load_busy;
wire q_exec_enter = (q_state==Q_LOAD) & a_load_done & b_load_done;

always @(posedge clk or negedge rstnn) begin
  if(!rstnn) begin
    a_busy_seen <= 1'b0; b_busy_seen <= 1'b0;
  end else begin
    if (q_state==Q_LOAD) begin
      if (a_load_busy) a_busy_seen <= 1'b1;
      if (b_load_busy) b_busy_seen <= 1'b1;
    end else if (q_state==Q_IDLE) begin
      a_busy_seen <= 1'b0; b_busy_seen <= 1'b0;
    end
  end
end

// q_start 펄스
reg q_start_hold;
always @(posedge clk or negedge rstnn) begin
  if(!rstnn) q_start_hold <= 1'b0;
  else begin
    if (q_inst_fire)  q_start_hold <= q_start_lvl; // 인스트 레벨 래치
    if (q_exec_enter) q_start_hold <= 1'b0;
  end
end

always @(posedge clk or negedge rstnn) begin
  if(!rstnn) q_start_pulse <= 1'b0;
  else       q_start_pulse <= q_exec_enter & q_start_hold;
end

// ---------------- STORE/라운드 카운터 ----------------
reg [$clog2(A_ROWS+1)-1:0]   qa_row_cnt;
reg [$clog2(B_ROWS+1)-1:0]   qb_row_cnt;
reg [$clog2(OUT_ROWS+1)-1:0] out_row_cnt;

wire qa_full  = (qa_row_cnt  == A_ROWS);
wire qb_full  = (qb_row_cnt  == B_ROWS);
wire out_full = (out_row_cnt == OUT_ROWS);

assign q_go_exec  = q_exec_enter;
assign q_go_store = (q_state==Q_EXEC) & qa_full & qb_full;
assign q_go_idle  = (q_state==Q_STORE);

// ============================================================================
// ---------------- Q→DQ 인스트 공유: DQ arm & dq_start_pulse -----------------
reg dq_token;  // Q가 끝나면 같은 인스트의 dq_start로 arm

always @(posedge clk or negedge rstnn) begin
  if(!rstnn) dq_token <= 1'b0;
  else begin
    if (q_go_store)      dq_token <= dq_start_lvl; // arm (필요 시)
    else if (dq_go_exec) dq_token <= 1'b0;         // 소비
  end
end

// O-LSU WRITE 인스트 1clk 발사 (DQ_IDLE & token & mo ready)
wire dq_exec_enter = (dq_state==DQ_IDLE) & dq_token & mo_sinst_wready;

assign mo_sinst_wvalid = dq_exec_enter;
assign mo_sinst_wdata  = {mo_info_q, OPC_WRITE};

// dq_start 1클럭 펄스
always @(posedge clk or negedge rstnn) begin
  if(!rstnn) dq_start_pulse <= 1'b0;
  else       dq_start_pulse <= dq_exec_enter;
end

assign dq_go_exec  = dq_exec_enter;
assign dq_go_store = (dq_state==DQ_EXEC) & out_full;
assign dq_go_idle  = (dq_state==DQ_STORE);

// ============================================================================
// ---------------- QA/QB/OUT MREG + MREG2STORE (OUT만 유지) -------------------
wire                           out_mreg2store_ren;
wire [BW_OUTPUT_TENSOR_ROW-1:0] out_mreg2store_rdata;

DCA_MATRIX_REGISTER_TYPE3 #(
  .MATRIX_SIZE_PARA(OUTPUT_MATRIX_SIZE),
  .BW_TENSOR_SCALAR(BW_TENSOR_SCALAR),
  .BW_MOVE_DATA(BW_OUTPUT_TENSOR_ROW),
  .RESET_VALUE(MREG_RESET_VALUE)
) i_mregOUT (
  .clk(clk), .rstnn(rstnn),
  .move_wenable(dq_m_valid_o),
  .move_wdata_list(dq_m_data_o),
  .move_renable(out_mreg2store_ren),
  .move_rdata_list(out_mreg2store_rdata),
  .shift_up(1'b0), .shift_left(1'b0), .transpose(1'b0),
  .all_rdata_list2d(), .upmost_rdata_list1d()
);

reg out_store_req;
wire out_store_busy, out_store_wready;

always @(posedge clk or negedge rstnn) begin
  if (!rstnn)                               out_store_req <= 1'b0;
  else if ((dq_state==DQ_EXEC) && out_full) out_store_req <= 1'b1;
  else if (out_store_wready)                out_store_req <= 1'b0;
end

DCA_MATRIX_MREG2STORE #(
  .MATRIX_SIZE_PARA(OUTPUT_MATRIX_SIZE),
  .BW_TENSOR_SCALAR(BW_TENSOR_SCALAR)
) i_mreg2store_OUT (
  .clk(clk), .rstnn(rstnn),
  .clear(1'b0), .enable(1'b1), .busy(out_store_busy),
  .storereg_wready(out_store_wready),
  .storereg_wrequest(out_store_req),
  .mreg_move_renable(out_mreg2store_ren),
  .mreg_move_rdata_list1d(out_mreg2store_rdata),
  .store_tensor_row_rvalid(mo_sstore_tensor_row_rvalid),
  .store_tensor_row_rlast (mo_sstore_tensor_row_rlast),
  .store_tensor_row_rready(mo_sstore_tensor_row_rready),
  .store_tensor_row_rdata (mo_sstore_tensor_row_rdata)
);

// ============================================================================
// *********************  AXI BRAM 연결부  ************************************
localparam int CAPACITY              = 32768;   // bytes
localparam int CAPA_PER_PORT         = 16384;   // bytes
localparam int CELL_ARRAY_WIDTH      = 512;     // 512b/line
localparam int NUM_AXI2RAM_PORT      = CAPACITY / CAPA_PER_PORT; // 2
localparam int NUM_CELL_PER_PORT     = CELL_ARRAY_WIDTH / BW_AXI_DATA; // 512/128=4
localparam int BW_BYTE_WEN           = (BW_AXI_DATA/8);           // 16
localparam int BW_CELL_ARRAY_BYTE_WEN= (CELL_ARRAY_WIDTH/8);      // 64
localparam int CELL_SIZE             = (CAPA_PER_PORT/NUM_CELL_PER_PORT); // 4096B
localparam int CELL_DEPTH            = (CELL_SIZE / BW_BYTE_WEN); // 4096/16=256
localparam int BW_CELL_INDEX         = $clog2(CELL_DEPTH);        // 8

// int_* (NUM_AXI2RAM_PORT=2)
wire [NUM_AXI2RAM_PORT-1:0]                        int_renable_list;
wire [CELL_ARRAY_WIDTH*NUM_AXI2RAM_PORT-1:0]       int_rdata_list;
wire [NUM_AXI2RAM_PORT-1:0]                        int_rvalid_list;
wire [BW_CELL_INDEX*NUM_AXI2RAM_PORT-1:0]          int_index_list;
wire [NUM_AXI2RAM_PORT-1:0]                        int_wenable_list;
wire [BW_CELL_ARRAY_BYTE_WEN*NUM_AXI2RAM_PORT-1:0] int_byte_enable_list;
wire [CELL_ARRAY_WIDTH*NUM_AXI2RAM_PORT-1:0]       int_wdata_list;

// Port0 write (QA/QB)
reg  [NUM_AXI2RAM_PORT-1:0]                        int_wenable_list_w;
reg  [BW_CELL_INDEX*NUM_AXI2RAM_PORT-1:0]          int_index_list_w;
reg  [BW_CELL_ARRAY_BYTE_WEN*NUM_AXI2RAM_PORT-1:0] int_byte_enable_list_w;
reg  [CELL_ARRAY_WIDTH*NUM_AXI2RAM_PORT-1:0]       int_wdata_list_w;

// QA/QB 스키드 + RR
reg                          qa_buf_valid, qb_buf_valid;
reg  [CELL_ARRAY_WIDTH-1:0]  qa_buf_data,  qb_buf_data;
reg                          rr_toggle;

always @(posedge clk or negedge rstnn) begin
  if(!rstnn) begin
    qa_buf_valid <= 1'b0; qb_buf_valid <= 1'b0;
    rr_toggle    <= 1'b0;
  end else begin
    if (a_m_valid_o && !qa_buf_valid) begin
      qa_buf_valid <= 1'b1;
      qa_buf_data  <= a_m_data_o;
    end
    if (b_m_valid_o && !qb_buf_valid) begin
      qb_buf_valid <= 1'b1;
      qb_buf_data  <= b_m_data_o;
    end
  end
end

assign a_m_ready_i  = ~qa_buf_valid;
assign b_m_ready_i  = ~qb_buf_valid;

// 카운터 리셋
always @(posedge clk or negedge rstnn) begin
  if(!rstnn) begin
    qa_row_cnt <= '0;
    qb_row_cnt <= '0;
  end else if (q_go_exec) begin
    qa_row_cnt <= '0;
    qb_row_cnt <= '0;
  end
end

always @(posedge clk or negedge rstnn) begin
  if(!rstnn) out_row_cnt <= '0;
  else if (dq_go_exec)   out_row_cnt <= '0;
end

// Port0 write 직렬화
wire pick_qa = qa_buf_valid & (~qb_buf_valid |  rr_toggle);
wire pick_qb = qb_buf_valid & (~qa_buf_valid | ~rr_toggle);

always @* begin
  int_wenable_list_w      = '0;
  int_index_list_w        = '0;
  int_byte_enable_list_w  = '0;
  int_wdata_list_w        = '0;

  if (pick_qa) begin
    int_wenable_list_w[PORT_QAB] = 1'b1;
    int_index_list_w [BW_CELL_INDEX*(PORT_QAB+1)-1 -: BW_CELL_INDEX] = (QA_BASE + qa_row_cnt);
    int_byte_enable_list_w[BW_CELL_ARRAY_BYTE_WEN*(PORT_QAB+1)-1 -: BW_CELL_ARRAY_BYTE_WEN] = {BW_CELL_ARRAY_BYTE_WEN{1'b1}};
    int_wdata_list_w  [CELL_ARRAY_WIDTH*(PORT_QAB+1)-1 -: CELL_ARRAY_WIDTH] = qa_buf_data;
  end
  else if (pick_qb) begin
    int_wenable_list_w[PORT_QAB] = 1'b1;
    int_index_list_w [BW_CELL_INDEX*(PORT_QAB+1)-1 -: BW_CELL_INDEX] = (QB_BASE + qb_row_cnt);
    int_byte_enable_list_w[BW_CELL_ARRAY_BYTE_WEN*(PORT_QAB+1)-1 -: BW_CELL_ARRAY_BYTE_WEN] = {BW_CELL_ARRAY_BYTE_WEN{1'b1}};
    int_wdata_list_w  [CELL_ARRAY_WIDTH*(PORT_QAB+1)-1 -: CELL_ARRAY_WIDTH] = qb_buf_data;
  end
end

always @(posedge clk or negedge rstnn) begin
  if(!rstnn) begin
    // no-op
  end else begin
    if (pick_qa) begin
      qa_buf_valid <= 1'b0;
      qa_row_cnt   <= qa_row_cnt + 1'b1;
      rr_toggle    <= ~rr_toggle;
    end else if (pick_qb) begin
      qb_buf_valid <= 1'b0;
      qb_row_cnt   <= qb_row_cnt + 1'b1;
      rr_toggle    <= ~rr_toggle;
    end
  end
end

// Port1: ACC read → qdq 입력
reg  [BW_CELL_INDEX-1:0]   acc_idx_r;
reg                        acc_ren_r;
reg                        dq_run;

wire [CELL_ARRAY_WIDTH-1:0] acc_rdata =
    int_rdata_list[CELL_ARRAY_WIDTH*(PORT_ACC+1)-1 -: CELL_ARRAY_WIDTH];

assign dq_s_valid_i = int_rvalid_list[PORT_ACC];
assign dq_s_data_i  = acc_rdata;

wire need_issue_next =
       dq_run
    && (dq_state==DQ_EXEC)
    && (out_row_cnt < OUT_ROWS)
    && ( !int_rvalid_list[PORT_ACC] || (int_rvalid_list[PORT_ACC] && dq_s_ready_o) );

always @(posedge clk or negedge rstnn) begin
  if(!rstnn) begin
    dq_run     <= 1'b0;
    acc_idx_r  <= ACC_BASE;
    acc_ren_r  <= 1'b0;
    out_row_cnt<= '0;
  end else begin
    acc_ren_r <= 1'b0; // 1clk 펄스 기본

    if (dq_start_pulse) begin
      dq_run     <= 1'b1;
      acc_idx_r  <= ACC_BASE;
      out_row_cnt<= '0;
    end

    if (dq_state != DQ_EXEC)
      dq_run <= 1'b0;

    if (int_rvalid_list[PORT_ACC] && dq_s_ready_o)
      out_row_cnt <= out_row_cnt + 1'b1;

    if (need_issue_next) begin
      acc_ren_r <= 1'b1;
      acc_idx_r <= acc_idx_r + 1'b1;
    end
  end
end

// int_* 결선
assign int_wenable_list     = int_wenable_list_w;
assign int_byte_enable_list = int_byte_enable_list_w;
assign int_wdata_list       = int_wdata_list_w;

wire [BW_CELL_INDEX-1:0] port0_index_w = int_index_list_w[BW_CELL_INDEX*(PORT_QAB+1)-1 -: BW_CELL_INDEX];
wire [BW_CELL_INDEX-1:0] port1_index_w = acc_idx_r;
assign int_index_list = {port1_index_w, port0_index_w};

assign int_renable_list = (PORT_ACC==1) ? {acc_ren_r, 1'b0} : {1'b0, acc_ren_r};

// ---------------- AXI BRAM 인스턴스 ----------------
axi_dual_port_sram128 i_sram (
  .clk(clk), .rstnn(rstnn),

  // AXI
  .rxawid   (mq2vta_rxawid),
  .rxawaddr (mq2vta_rxawaddr),
  .rxawlen  (mq2vta_rxawlen),
  .rxawsize (mq2vta_rxawsize),
  .rxawburst(mq2vta_rxawburst),
  .rxawvalid(mq2vta_rxawvalid),
  .rxawready(mq2vta_rxawready),

  .rxwid    (mq2vta_rxwid),
  .rxwdata  (mq2vta_rxwdata),
  .rxwstrb  (mq2vta_rxwstrb),
  .rxwlast  (mq2vta_rxwlast),
  .rxwvalid (mq2vta_rxwvalid),
  .rxwready (mq2vta_rxwready),

  .rxbid    (mq2vta_rxbid),
  .rxbresp  (mq2vta_rxbresp),
  .rxbvalid (mq2vta_rxbvalid),
  .rxbready (mq2vta_rxbready),

  .rxarid   (mq2vta_rxarid),
  .rxaraddr (mq2vta_rxaraddr),
  .rxarlen  (mq2vta_rxarlen),
  .rxarsize (mq2vta_rxarsize),
  .rxarburst(mq2vta_rxarburst),
  .rxarvalid(mq2vta_rxarvalid),
  .rxarready(mq2vta_rxarready),

  .rxrid    (mq2vta_rxrid),
  .rxrdata  (mq2vta_rxrdata),
  .rxrresp  (mq2vta_rxrresp),
  .rxrlast  (mq2vta_rxrlast),
  .rxrvalid (mq2vta_rxrvalid),
  .rxrready (mq2vta_rxrready),

  // int_*
  .int_renable_list     (int_renable_list),
  .int_rdata_list       (int_rdata_list),
  .int_rvalid_list      (int_rvalid_list),
  .int_index_list       (int_index_list),
  .int_wenable_list     (int_wenable_list),
  .int_byte_enable_list (int_byte_enable_list),
  .int_wdata_list       (int_wdata_list)
);

endmodule
