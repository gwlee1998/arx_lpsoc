`timescale 1ns/1ps
// ============================================================================
// DCA_MATRIX_QDQ_MMIOX_MLSU
// [X, W matrix]
// - LOAD: MMIO → LOAD2MREG → MREG(TYPE3) → qdq controller
// - STORE: qdq controller → MREG(TYPE3) → MREG2STORE → AXI
// [O matrix]
// - LOAD: AXI → LOAD2MREG → MREG(TYPE3) → qdq controller
// - STORE: qdq controller → MREG(TYPE3) → MREG2STORE → MMIO
// ============================================================================

`include "ervp_global.vh"
`include "ervp_endian.vh"
`include "dca_matrix_qgemm_defines.vh"  // 수정됨
`include "dca_matrix_info.vh"
`include "dca_matrix_lsu_inst.vh"

//---------------- Params ----------------
module DCA_MATRIX_QGEMM_MMIOX_MLSU
(
  clk,
  rstnn,

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

  mx_sinst_wvalid,
  mx_sinst_wdata,
  mx_sinst_wready,
  mx_sinst_decode_finish,
  mx_sinst_execute_finish,
  mx_sinst_busy,
  mx_sload_tensor_row_wvalid,
  mx_sload_tensor_row_wlast,
  mx_sload_tensor_row_wdata,
  mx_sload_tensor_row_wready,
  mx_sstore_tensor_row_rvalid,
  mx_sstore_tensor_row_rlast,
  mx_sstore_tensor_row_rready,
  mx_sstore_tensor_row_rdata,

  mw_sinst_wvalid,
  mw_sinst_wdata,
  mw_sinst_wready,
  mw_sinst_decode_finish,
  mw_sinst_execute_finish,
  mw_sinst_busy,
  mw_sload_tensor_row_wvalid,
  mw_sload_tensor_row_wlast,
  mw_sload_tensor_row_wdata,
  mw_sload_tensor_row_wready,
  mw_sstore_tensor_row_rvalid,
  mw_sstore_tensor_row_rlast,
  mw_sstore_tensor_row_rready,
  mw_sstore_tensor_row_rdata,

  mo_sinst_wvalid,
  mo_sinst_wdata,
  mo_sinst_wready,
  mo_sinst_decode_finish,
  mo_sinst_execute_finish,
  mo_sinst_busy,
  mo_sload_tensor_row_wvalid,
  mo_sload_tensor_row_wlast,
  mo_sload_tensor_row_wdata,
  mo_sload_tensor_row_wready,
  mo_sstore_tensor_row_rvalid,
  mo_sstore_tensor_row_rlast,
  mo_sstore_tensor_row_rready,
  mo_sstore_tensor_row_rdata,

  

	mq2vta_rxawid,
	mq2vta_rxawaddr,
	mq2vta_rxawlen,
	mq2vta_rxawsize,
	mq2vta_rxawburst,
	mq2vta_rxawvalid,
	mq2vta_rxawready,

	mq2vta_rxwid,
	mq2vta_rxwdata,
	mq2vta_rxwstrb,
	mq2vta_rxwlast,
	mq2vta_rxwvalid,
	mq2vta_rxwready,

	mq2vta_rxbid,
	mq2vta_rxbresp,
	mq2vta_rxbvalid,
	mq2vta_rxbready,

	mq2vta_rxarid,
	mq2vta_rxaraddr,
	mq2vta_rxarlen,
	mq2vta_rxarsize,
	mq2vta_rxarburst,
	mq2vta_rxarvalid,
	mq2vta_rxarready,

	mq2vta_rxrid,
	mq2vta_rxrdata,
	mq2vta_rxrresp,
	mq2vta_rxrlast,
	mq2vta_rxrvalid,
  mq2vta_rxrready
);

parameter BIT_NUM          = 8;
parameter FP_EXP_W         = 8;
parameter FP_MANT_W        = 23;
parameter FP_EXP_BIAS      = 127;
parameter SCALE_FIFO_DEPTH = 4;
parameter BW_AXI_DATA = 128;
parameter BW_AXI_TID = 4;

parameter integer INPUT_MATRIX_SIZE  = 16;
parameter integer WEIGHT_MATRIX_SIZE = 16;
parameter integer OUTPUT_MATRIX_SIZE = 16;
parameter TENSOR_PARA = 0;

localparam BW_CONFIG = 1;
localparam BW_STATUS = `BW_DCA_MATRIX_QDQ_STATUS;  // not used
localparam BW_LOG = `BW_DCA_MATRIX_QDQ_LOG;  // not used
localparam BW_INST = `BW_DCA_MATRIX_QDQ_INST;
localparam BW_INPUT = 32;
localparam BW_OUTPUT = 32;

// ----------------------------------------------------------------------------
`include "dca_matrix_dim_util.vb"
`include "dca_tensor_scalar_lpara.vb"
// ----------------------------------------------------------------------------

localparam integer INPUT_MATRIX_NUM_COL   = GET_MATRIX_NUM_COL(INPUT_MATRIX_SIZE);
localparam integer WEIGHT_MATRIX_NUM_COL  = GET_MATRIX_NUM_COL(WEIGHT_MATRIX_SIZE);
localparam integer OUTPUT_MATRIX_NUM_COL  = GET_MATRIX_NUM_COL(OUTPUT_MATRIX_SIZE);

localparam integer BW_INPUT_TENSOR_ROW   = BW_TENSOR_SCALAR*INPUT_MATRIX_NUM_COL;
localparam integer BW_WEIGHT_TENSOR_ROW  = BW_TENSOR_SCALAR*WEIGHT_MATRIX_NUM_COL;
localparam integer BW_OUTPUT_TENSOR_ROW  = BW_TENSOR_SCALAR*OUTPUT_MATRIX_NUM_COL;

localparam [`BW_DCA_MATRIX_LSU_INST_OPCODE-1:0] OPC_READ  = `DCA_MATRIX_LSU_INST_OPCODE_READ;
localparam [`BW_DCA_MATRIX_LSU_INST_OPCODE-1:0] OPC_WRITE = `DCA_MATRIX_LSU_INST_OPCODE_WRITE;

localparam MREG_RESET_VALUE = TENSOR_ZERO;

input  wire clk;
input  wire rstnn;

// Control / MMIO
input wire [(BW_CONFIG)-1:0] control_rmx_core_config;
output wire [(BW_STATUS)-1:0] control_rmx_core_status;
input wire control_rmx_clear_request;
output wire control_rmx_clear_finish;
input wire control_rmx_log_fifo_wready;
output wire control_rmx_log_fifo_wrequest;
output wire [(BW_LOG)-1:0] control_rmx_log_fifo_wdata;
input wire control_rmx_inst_fifo_rready;
input wire [(BW_INST)-1:0] control_rmx_inst_fifo_rdata;
output wire control_rmx_inst_fifo_rrequest;
output wire control_rmx_operation_finish;
input wire control_rmx_input_fifo_rready;
input wire [(BW_INPUT)-1:0] control_rmx_input_fifo_rdata;
output wire control_rmx_input_fifo_rrequest;
input wire control_rmx_output_fifo_wready;
output wire control_rmx_output_fifo_wrequest;
output wire [(BW_OUTPUT)-1:0] control_rmx_output_fifo_wdata;


// A LSU (FP in → QA out)
output wire mx_sinst_wvalid;
output wire [(`BW_DCA_MATRIX_LSU_INST)-1:0] mx_sinst_wdata;
input wire mx_sinst_wready;
input wire mx_sinst_decode_finish;
input wire mx_sinst_execute_finish;
input wire mx_sinst_busy;
input wire mx_sload_tensor_row_wvalid;
input wire mx_sload_tensor_row_wlast;
input wire [BW_INPUT_TENSOR_ROW-1:0] mx_sload_tensor_row_wdata;
output wire mx_sload_tensor_row_wready;
input wire mx_sstore_tensor_row_rvalid;
input wire mx_sstore_tensor_row_rlast;
output wire mx_sstore_tensor_row_rready;
output wire [BW_INPUT_TENSOR_ROW-1:0] mx_sstore_tensor_row_rdata;

output wire mw_sinst_wvalid;
output wire [(`BW_DCA_MATRIX_LSU_INST)-1:0] mw_sinst_wdata;
input wire mw_sinst_wready;
input wire mw_sinst_decode_finish;
input wire mw_sinst_execute_finish;
input wire mw_sinst_busy;
input wire mw_sload_tensor_row_wvalid;
input wire mw_sload_tensor_row_wlast;
input wire [BW_WEIGHT_TENSOR_ROW-1:0] mw_sload_tensor_row_wdata;
output wire mw_sload_tensor_row_wready;
input wire mw_sstore_tensor_row_rvalid;
input wire mw_sstore_tensor_row_rlast;
output wire mw_sstore_tensor_row_rready;
output wire [BW_WEIGHT_TENSOR_ROW-1:0] mw_sstore_tensor_row_rdata;

output wire mo_sinst_wvalid;
output wire [(`BW_DCA_MATRIX_LSU_INST)-1:0] mo_sinst_wdata;
input wire mo_sinst_wready;
input wire mo_sinst_decode_finish;
input wire mo_sinst_execute_finish;
input wire mo_sinst_busy;
input wire mo_sload_tensor_row_wvalid;
input wire mo_sload_tensor_row_wlast;
input wire [BW_OUTPUT_TENSOR_ROW-1:0] mo_sload_tensor_row_wdata;
output wire mo_sload_tensor_row_wready;
input wire mo_sstore_tensor_row_rvalid;
input wire mo_sstore_tensor_row_rlast;
output wire mo_sstore_tensor_row_rready;
output wire [BW_OUTPUT_TENSOR_ROW-1:0] mo_sstore_tensor_row_rdata;

output wire mq2vta_rxawready;
input wire mq2vta_rxawvalid;
input wire [(BW_ADDR)-1:0] mq2vta_rxawaddr;
input wire [(BW_AXI_TID)-1:0] mq2vta_rxawid;
input wire [(8)-1:0] mq2vta_rxawlen;
input wire [(3)-1:0] mq2vta_rxawsize;
input wire [(2)-1:0] mq2vta_rxawburst;
output wire mq2vta_rxwready;
input wire mq2vta_rxwvalid;
input wire [(BW_AXI_TID)-1:0] mq2vta_rxwid;
input wire [(BW_AXI_DATA)-1:0] mq2vta_rxwdata;
input wire [(BW_AXI_DATA/8)-1:0] mq2vta_rxwstrb;
input wire mq2vta_rxwlast;
input wire mq2vta_rxbready;
output wire mq2vta_rxbvalid;
output wire [(BW_AXI_TID)-1:0] mq2vta_rxbid;
output wire [(2)-1:0] mq2vta_rxbresp;
output wire mq2vta_rxarready;
input wire mq2vta_rxarvalid;
input wire [(BW_ADDR)-1:0] mq2vta_rxaraddr;
input wire [(BW_AXI_TID)-1:0] mq2vta_rxarid;
input wire [(8)-1:0] mq2vta_rxarlen;
input wire [(3)-1:0] mq2vta_rxarsize;
input wire [(2)-1:0] mq2vta_rxarburst;
input wire mq2vta_rxrready;
output wire mq2vta_rxrvalid;
output wire [(BW_AXI_TID)-1:0] mq2vta_rxrid;
output wire [(BW_AXI_DATA)-1:0] mq2vta_rxrdata;
output wire mq2vta_rxrlast;
output wire [(2)-1:0] mq2vta_rxrresp;

// ---------------- Control ties ----------------
// not used
assign control_rmx_core_status = 0;
assign control_rmx_clear_finish = 0;
assign control_rmx_log_fifo_wrequest = 0;
assign control_rmx_log_fifo_wdata = 0;
assign control_rmx_input_fifo_rrequest = 0;
assign control_rmx_output_fifo_wrequest = 0;
assign control_rmx_output_fifo_wdata = 0;

// ---------------- Inst decode ----------------
wire [`BW_DCA_MATRIX_INFO_ALIGNED-1:0] mx_info, mw_info, mo_info;
assign {mo_info,mw_info,mx_info} = control_rmx_inst_fifo_rdata;

// ---------------- Dual FSM ----------------
localparam [1:0] S_IDLE=2'd0, S_LOAD=2'd1, S_EXEC=2'd2, S_STORE=2'd3;
reg [1:0] q_state, dq_state;

wire q_go_load,  q_go_exec,  q_go_store,  q_go_idle;
wire dq_go_load, dq_go_exec, dq_go_store, dq_go_idle;

always @(posedge clk or negedge rstnn) begin
  if(!rstnn) q_state <= S_IDLE;
  else case(q_state)
    S_IDLE:  if(q_go_load)  q_state <= S_LOAD;
    S_LOAD:  if(q_go_exec)  q_state <= S_EXEC;
    S_EXEC:  if(q_go_store) q_state <= S_STORE;
    S_STORE: if(q_go_idle)  q_state <= S_IDLE;
  endcase
end

always @(posedge clk or negedge rstnn) begin
  if(!rstnn) dq_state <= S_IDLE;
  else case(dq_state)
    S_IDLE:  if(dq_go_load) dq_state <= S_LOAD;
    S_LOAD:  if(dq_go_exec) dq_state <= S_EXEC;
    S_EXEC:  if(dq_go_store) dq_state <= S_STORE;
    S_STORE: if(dq_go_idle) dq_state <= S_IDLE;
  endcase
end

// inst write & op-finish
assign mx_sinst_wvalid = (q_state==S_IDLE)  & control_rmx_inst_fifo_rready & mx_sinst_wready & mw_sinst_wready;
assign mw_sinst_wvalid = mx_sinst_wvalid;
assign mo_sinst_wvalid = (dq_state==S_IDLE) & control_rmx_inst_fifo_rready & mo_sinst_wready;

assign mx_sinst_wdata  = {mx_info, OPC_READ};
assign mw_sinst_wdata  = {mw_info, OPC_READ};
assign mo_sinst_wdata  = {mo_info, OPC_WRITE};

assign control_rmx_inst_fifo_rrequest = ((q_state==S_STORE) & q_go_idle) | ((dq_state==S_STORE) & dq_go_idle);
assign control_rmx_operation_finish   = (q_state==S_IDLE) & (dq_state==S_IDLE);

// ---------------- LOAD2MREG → MREG(TYPE3) → qdq ----------------
// sent 카운터 기반 valid/ready → fire로만 진행

// A, B path는 LOAD2MREG에서 MREG로 수정 완료
// O path는 AXI READ LSU에서 받도록 수정 필요

// qdq handshakes
wire a_s_ready_o, b_s_ready_o, dq_s_ready_o;

// A stream (X) : MMIO → LOAD2MREG → MREG → qdq로 수정됨
localparam int A_ROWS = INPUT_MATRIX_NUM_COL;
reg  [$clog2(A_ROWS+1)-1:0] a_sent;
wire a_valid = (q_state==S_EXEC) & (a_sent < A_ROWS);
wire a_fire;

// LOAD2MREG(A)
wire                       a_load_wready, a_load_rready, a_load_busy;
wire                       a_src_mreg_wen, a_src_mreg_ren;
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
  .loadreg_rrequest       (1'b1) // load 완료 즉시 ack
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
  .shift_up(1'b0), .shift_left(1'b0), .transpose(1'b0),
  .all_rdata_list2d(), .upmost_rdata_list1d()
);

// qdq 입력 (A)
assign a_fire      = a_valid & a_s_ready_o;   // 한 행 소비
assign a_src_mreg_ren = a_fire;

wire a_s_valid_i = a_valid;
wire [BW_INPUT_TENSOR_ROW-1:0] a_s_data_i = a_src_mreg_rdata;

always @(posedge clk or negedge rstnn) begin
  if(!rstnn)          a_sent <= '0;
  else if (q_go_exec) a_sent <= '0;          // 시작 펄스에서만 초기화
  else if (a_fire)    a_sent <= a_sent + 1'b1;
end


// B stream (W) : MMIO → LOAD2MREG → MREG → qdq로 수정됨
localparam int B_ROWS = WEIGHT_MATRIX_NUM_COL;
reg  [$clog2(B_ROWS+1)-1:0] b_sent;
wire b_valid = (q_state==S_EXEC) & (b_sent < B_ROWS);
wire b_fire;

// LOAD2MREG(B)
wire                       b_load_wready, b_load_rready, b_load_busy;
wire                       b_src_mreg_wen, b_src_mreg_ren;
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
  .shift_up(1'b0), .shift_left(1'b0), .transpose(1'b0),
  .all_rdata_list2d(), .upmost_rdata_list1d()
);

// qdq 입력 (B)
assign b_fire      = b_valid & b_s_ready_o;
assign b_src_mreg_ren = b_fire;

wire b_s_valid_i = b_valid;
wire [BW_WEIGHT_TENSOR_ROW-1:0] b_s_data_i = b_src_mreg_rdata;

always @(posedge clk or negedge rstnn) begin
  if(!rstnn)          b_sent <= '0;
  else if (q_go_exec) b_sent <= '0;
  else if (b_fire)    b_sent <= b_sent + 1'b1;
end


// ACC stream (O_src) : (AXI →) LOAD2MREG → MREG → qdq
localparam int OUT_ROWS = OUTPUT_MATRIX_NUM_COL;
reg  [$clog2(OUT_ROWS+1)-1:0] acc_sent;
wire acc_valid = (dq_state==S_EXEC) & (acc_sent < OUT_ROWS);
wire acc_fire;

// "AXI READ LSU → LOAD2MREG" 인지, "AXI READ LSU → MREG" 인지 모르겠어요..
// AXI READ LSU에서 분해된 32xN 스트림을 여기에 연결
wire                       qo_load_wready, qo_load_rready, qo_load_busy;
wire                       qo_src_mreg_wen, qo_src_mreg_ren;
wire [BW_OUTPUT_TENSOR_ROW-1:0] qo_src_mreg_wdata, qo_src_mreg_rdata;

wire qo_axi_wvalid, qo_axi_wlast;
wire [BW_OUTPUT_TENSOR_ROW-1:0] qo_axi_wdata;

// qdq 쪽 back-pressure로부터 ready 제공
assign mo_sload_tensor_row_wready = qo_load_wready;

DCA_MATRIX_LOAD2MREG #(
  .MATRIX_SIZE_PARA(OUTPUT_MATRIX_SIZE), .TENSOR_PARA(0)
) i_load2mreg_QO (
  .clk(clk), .rstnn(rstnn),
  .clear(1'b0), .enable(1'b1), .busy(qo_load_busy),
  .load_tensor_row_wvalid (qo_axi_wvalid),
  .load_tensor_row_wlast  (qo_axi_wlast),
  .load_tensor_row_wdata  (qo_axi_wdata),
  .load_tensor_row_wready (qo_load_wready),

  .mreg_move_wenable      (qo_src_mreg_wen),
  .mreg_move_wdata_list1d (qo_src_mreg_wdata),

  .loadreg_rready         (qo_load_rready),
  .loadreg_rrequest       (1'b1)
);

DCA_MATRIX_REGISTER_TYPE3 #(
  .MATRIX_SIZE_PARA(OUTPUT_MATRIX_SIZE),
  .BW_TENSOR_SCALAR(BW_TENSOR_SCALAR),
  .BW_MOVE_DATA(BW_OUTPUT_TENSOR_ROW),
  .RESET_VALUE(MREG_RESET_VALUE)
) i_mregQO (
  .clk(clk), .rstnn(rstnn),
  .move_wenable (qo_src_mreg_wen),
  .move_wdata_list(qo_src_mreg_wdata),
  .move_renable (qo_src_mreg_ren),
  .move_rdata_list(qo_src_mreg_rdata),
  .shift_up(1'b0), .shift_left(1'b0), .transpose(1'b0),
  .all_rdata_list2d(), .upmost_rdata_list1d()
);

// qdq 입력 (dequant 입력원)
assign acc_fire      = acc_valid & dq_s_ready_o;
assign qo_src_mreg_ren = acc_fire;

wire dq_s_valid_i = acc_valid;
wire [BW_OUTPUT_TENSOR_ROW-1:0] dq_s_data_i = qo_src_mreg_rdata;

always @(posedge clk or negedge rstnn) begin
  if(!rstnn)           acc_sent <= '0;
  else if (dq_go_exec) acc_sent <= '0;
  else if (acc_fire)   acc_sent <= acc_sent + 1'b1;
end

// ---------------- qdq controller ----------------
wire a_m_valid_o, b_m_valid_o, dq_m_valid_o;
wire a_m_ready_i, b_m_ready_i, dq_m_ready_i;
wire [BW_INPUT_TENSOR_ROW-1:0]  a_m_data_o;
wire [BW_WEIGHT_TENSOR_ROW-1:0] b_m_data_o;
wire [BW_OUTPUT_TENSOR_ROW-1:0] dq_m_data_o;

qdq_controller #(
  .BIT_NUM(BIT_NUM), .MAT_SIZE(OUTPUT_MATRIX_SIZE),
  .FP_DATA_W(BW_TENSOR_SCALAR), .FP_EXP_W(FP_EXP_W),
  .FP_MANT_W(FP_MANT_W), .FP_EXP_BIAS(FP_EXP_BIAS),
  .LANES_NUM(OUTPUT_MATRIX_NUM_COL), .SCALE_FIFO_DEPTH(SCALE_FIFO_DEPTH)
) i_qdq (
  .clk(clk), .rstnn(rstnn),

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
  .dq_m_data_o  ( dq_m_data_o )
);

// qdq out → QA/QB/OUT MREG write
wire a_out_fire  = a_m_valid_o  & a_m_ready_i;
wire b_out_fire  = b_m_valid_o  & b_m_ready_i;
wire dq_out_fire = dq_m_valid_o & dq_m_ready_i;

assign a_m_ready_i  = 1'b1;
assign b_m_ready_i  = 1'b1;
assign dq_m_ready_i = 1'b1;

// fill counters (for STORE enter)
reg [$clog2(A_ROWS+1)-1:0]   qa_wcnt;
reg [$clog2(B_ROWS+1)-1:0]   qb_wcnt;
reg [$clog2(OUT_ROWS+1)-1:0] out_wcnt;

wire qa_full  = (qa_wcnt  == A_ROWS);
wire qb_full  = (qb_wcnt  == B_ROWS);
wire out_full = (out_wcnt == OUT_ROWS);

always @(posedge clk or negedge rstnn) begin
  if(!rstnn)             qa_wcnt <= '0;
  else if (q_go_exec)    qa_wcnt <= '0;
  else if (a_out_fire && !qa_full) qa_wcnt <= qa_wcnt + 1'b1;
end
always @(posedge clk or negedge rstnn) begin
  if(!rstnn)             qb_wcnt <= '0;
  else if (q_go_exec)    qb_wcnt <= '0;
  else if (b_out_fire && !qb_full) qb_wcnt <= qb_wcnt + 1'b1;
end
always @(posedge clk or negedge rstnn) begin
  if(!rstnn)              out_wcnt <= '0;
  else if (dq_go_exec)    out_wcnt <= '0;
  else if (dq_out_fire && !out_full) out_wcnt <= out_wcnt + 1'b1;
end

// FSM go conditions
reg  qa_store_req,    qb_store_req,    out_store_req;
wire qa_store_busy, qb_store_busy, out_store_busy;

assign q_go_load  = mx_sinst_wvalid;                 // same-cycle guard
assign q_go_exec  = (q_state==S_LOAD);               // LOAD→EXEC 바로 진입
assign q_go_store = (q_state==S_EXEC) & qa_full & qb_full;
assign q_go_idle  = (q_state==S_STORE) &
                    (~qa_store_req) & (~qb_store_req) &
                    (~qa_store_busy) & (~qb_store_busy);

assign dq_go_load  = mo_sinst_wvalid;
assign dq_go_exec  = (dq_state==S_LOAD);
assign dq_go_store = (dq_state==S_EXEC) & out_full;
assign dq_go_idle  = (dq_state==S_STORE) &
                     (~out_store_req) & (~out_store_busy);

// ---------------- QA/QB/OUT MREG + MREG2STORE ----------------

// QA path, QB path이 AXI로 연결되도록 수정 필요.

wire                           qa_mreg2store_ren;
wire [BW_INPUT_TENSOR_ROW-1:0] qa_mreg2store_rdata;
wire                           qb_mreg2store_ren;
wire [BW_WEIGHT_TENSOR_ROW-1:0] qb_mreg2store_rdata;
wire                           out_mreg2store_ren;
wire [BW_OUTPUT_TENSOR_ROW-1:0] out_mreg2store_rdata;


// QA buffer
DCA_MATRIX_REGISTER_TYPE3 #(
  .MATRIX_SIZE_PARA(INPUT_MATRIX_SIZE),
  .BW_TENSOR_SCALAR(BW_TENSOR_SCALAR),
  .BW_MOVE_DATA(BW_INPUT_TENSOR_ROW),
  .RESET_VALUE(MREG_RESET_VALUE)
) i_mregQA (
  .clk(clk), .rstnn(rstnn),
  .move_wenable(a_out_fire),
  .move_wdata_list(a_m_data_o),
  .move_renable(qa_mreg2store_ren),
  .move_rdata_list(qa_mreg2store_rdata),
  .shift_up(1'b0), .shift_left(1'b0), .transpose(1'b0),
  .all_rdata_list2d(), .upmost_rdata_list1d()
);

// QB buffer
DCA_MATRIX_REGISTER_TYPE3 #(
  .MATRIX_SIZE_PARA(WEIGHT_MATRIX_SIZE),
  .BW_TENSOR_SCALAR(BW_TENSOR_SCALAR),
  .BW_MOVE_DATA(BW_WEIGHT_TENSOR_ROW),
  .RESET_VALUE(MREG_RESET_VALUE)
) i_mregQB (
  .clk(clk), .rstnn(rstnn),
  .move_wenable(b_out_fire),
  .move_wdata_list(b_m_data_o),
  .move_renable(qb_mreg2store_ren),
  .move_rdata_list(qb_mreg2store_rdata),
  .shift_up(1'b0), .shift_left(1'b0), .transpose(1'b0),
  .all_rdata_list2d(), .upmost_rdata_list1d()
);

// OUT buffer
DCA_MATRIX_REGISTER_TYPE3 #(
  .MATRIX_SIZE_PARA(OUTPUT_MATRIX_SIZE),
  .BW_TENSOR_SCALAR(BW_TENSOR_SCALAR),
  .BW_MOVE_DATA(BW_OUTPUT_TENSOR_ROW),
  .RESET_VALUE(MREG_RESET_VALUE)
) i_mregOUT (
  .clk(clk), .rstnn(rstnn),
  .move_wenable(dq_out_fire),
  .move_wdata_list(dq_m_data_o),
  .move_renable(out_mreg2store_ren),
  .move_rdata_list(out_mreg2store_rdata),
  .shift_up(1'b0), .shift_left(1'b0), .transpose(1'b0),
  .all_rdata_list2d(), .upmost_rdata_list1d()
);

// MREG2STORE
wire qa_store_wready, qb_store_wready, out_store_wready;

always @(posedge clk or negedge rstnn) begin
  if (!rstnn)                          qa_store_req <= 1'b0;
  else if ((q_state==S_EXEC) && qa_full) qa_store_req <= 1'b1;
  else if (qa_store_wready)            qa_store_req <= 1'b0;
end

always @(posedge clk or negedge rstnn) begin
  if (!rstnn)                          qb_store_req <= 1'b0;
  else if ((q_state==S_EXEC) && qb_full) qb_store_req <= 1'b1;
  else if (qb_store_wready)            qb_store_req <= 1'b0;
end

// OUT 쪽도 동일 패턴 유지
always @(posedge clk or negedge rstnn) begin
  if (!rstnn)                             out_store_req <= 1'b0;
  else if ((dq_state==S_EXEC) && out_full) out_store_req <= 1'b1;
  else if (out_store_wready)              out_store_req <= 1'b0;
end

// QA, QB는 AXI 포트로 연결 필요

// DCA_MATRIX_MREG2STORE #(
//   .MATRIX_SIZE_PARA(INPUT_MATRIX_SIZE),
//   .BW_TENSOR_SCALAR(BW_TENSOR_SCALAR)
// ) i_mreg2store_QA (
//   .clk(clk), .rstnn(rstnn),
//   .clear(1'b0), .enable(1'b1), .busy(qa_store_busy),  
//   .storereg_wready(qa_store_wready),
//   .storereg_wrequest(qa_store_req),
//   .mreg_move_renable(qa_mreg2store_ren),
//   .mreg_move_rdata_list1d(qa_mreg2store_rdata),
//   .store_tensor_row_rvalid(mx_sstore_tensor_row_rvalid),
//   .store_tensor_row_rlast (mx_sstore_tensor_row_rlast),
//   .store_tensor_row_rready(mx_sstore_tensor_row_rready),
//   .store_tensor_row_rdata (mx_sstore_tensor_row_rdata)
// );

// DCA_MATRIX_MREG2STORE #(
//   .MATRIX_SIZE_PARA(WEIGHT_MATRIX_SIZE),
//   .BW_TENSOR_SCALAR(BW_TENSOR_SCALAR)
// ) i_mreg2store_QB (
//   .clk(clk), .rstnn(rstnn),
//   .clear(1'b0), .enable(1'b1), .busy(qb_store_busy),  
//   .storereg_wready(qb_store_wready),
//   .storereg_wrequest(qb_store_req),
//   .mreg_move_renable(qb_mreg2store_ren),
//   .mreg_move_rdata_list1d(qb_mreg2store_rdata),
//   .store_tensor_row_rvalid(mw_sstore_tensor_row_rvalid),
//   .store_tensor_row_rlast (mw_sstore_tensor_row_rlast),
//   .store_tensor_row_rready(mw_sstore_tensor_row_rready),
//   .store_tensor_row_rdata (mw_sstore_tensor_row_rdata)
// );

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

/////////////////////////////////////////////////////////////
// 디버깅용 로직
/////////////////////////////////////////////////////////////
// 플랫 입력 버스 -> 2차원 reg (웨이브 보기용)
reg [BW_TENSOR_SCALAR-1:0] mx_sload_tensor_row_wdata_reg [0:GET_MATRIX_NUM_COL(16)-1];
reg [BW_TENSOR_SCALAR-1:0] mx_sstore_tensor_row_rdata_reg [0:GET_MATRIX_NUM_COL(16)-1];

integer __r, __idx;
always @* begin
    for (__r = 0; __r < GET_MATRIX_NUM_COL(16); __r = __r + 1) begin
        mx_sload_tensor_row_wdata_reg[__r] = mx_sload_tensor_row_wdata[(__r+1)*BW_TENSOR_SCALAR-1 -: BW_TENSOR_SCALAR];
        mx_sstore_tensor_row_rdata_reg[__r] = mx_sstore_tensor_row_rdata[(__r+1)*BW_TENSOR_SCALAR-1 -: BW_TENSOR_SCALAR];
    end
end

endmodule