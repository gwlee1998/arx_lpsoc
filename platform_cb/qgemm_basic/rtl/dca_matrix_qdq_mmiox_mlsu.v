`timescale 1ns/1ps
// ============================================================================
// DCA_MATRIX_QDQ_MMIOX_MLSU  (conv2d-style load/store)
// - LOAD: sload → qdq (bypass)
// - STORE: qdq out → MREG(TYPE3) → MREG2STORE
// - PIPE:  A/B quant → (외부 GEMM) → ACC sload → dequant → OUT store(FP)
// ============================================================================

`include "ervp_global.vh"
`include "ervp_endian.vh"
`include "dca_module_memorymap_offset.vh"
`include "dca_matrix_info.vh"
`include "dca_matrix_lsu_inst.vh"

//---------------- Params ----------------
module DCA_MATRIX_QDQ_MMIOX_MLSU
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

  mi_sinst_wvalid,
  mi_sinst_wdata,
  mi_sinst_wready,
  mi_sinst_decode_finish,
  mi_sinst_execute_finish,
  mi_sinst_busy,
  mi_sload_tensor_row_wvalid,
  mi_sload_tensor_row_wlast,
  mi_sload_tensor_row_wdata,
  mi_sload_tensor_row_wready,
  mi_sstore_tensor_row_rvalid,
  mi_sstore_tensor_row_rlast,
  mi_sstore_tensor_row_rready,
  mi_sstore_tensor_row_rdata,

  mk_sinst_wvalid,
  mk_sinst_wdata,
  mk_sinst_wready,
  mk_sinst_decode_finish,
  mk_sinst_execute_finish,
  mk_sinst_busy,
  mk_sload_tensor_row_wvalid,
  mk_sload_tensor_row_wlast,
  mk_sload_tensor_row_wdata,
  mk_sload_tensor_row_wready,
  mk_sstore_tensor_row_rvalid,
  mk_sstore_tensor_row_rlast,
  mk_sstore_tensor_row_rready,
  mk_sstore_tensor_row_rdata,

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
  mo_sstore_tensor_row_rdata
);

parameter BIT_NUM          = 8;
parameter FP_EXP_W         = 8;
parameter FP_MANT_W        = 23;
parameter FP_EXP_BIAS      = 127;
parameter SCALE_FIFO_DEPTH = 4;

parameter integer INPUT_MATRIX_SIZE  = 16;
parameter integer KERNEL_MATRIX_SIZE = 16;
parameter integer OUTPUT_MATRIX_SIZE = 16;
parameter TENSOR_PARA = 0;

localparam BW_CONFIG = 1;
localparam BW_STATUS = `BW_DCA_MATRIX_CONV2D_STATUS;
localparam BW_LOG = `BW_DCA_MATRIX_CONV2D_LOG;
localparam BW_INST = `BW_DCA_MATRIX_CONV2D_INST;
localparam BW_INPUT = 32;
localparam BW_OUTPUT = 32;

// ----------------------------------------------------------------------------
`include "dca_matrix_dim_util.vb"
`include "dca_tensor_scalar_lpara.vb"
// ----------------------------------------------------------------------------

localparam integer INPUT_MATRIX_NUM_COL   = GET_MATRIX_NUM_COL(INPUT_MATRIX_SIZE);
localparam integer KERNEL_MATRIX_NUM_COL  = GET_MATRIX_NUM_COL(KERNEL_MATRIX_SIZE);
localparam integer OUTPUT_MATRIX_NUM_COL  = GET_MATRIX_NUM_COL(OUTPUT_MATRIX_SIZE);

localparam integer BW_INPUT_TENSOR_ROW   = BW_TENSOR_SCALAR*INPUT_MATRIX_NUM_COL;
localparam integer BW_KERNEL_TENSOR_ROW  = BW_TENSOR_SCALAR*KERNEL_MATRIX_NUM_COL;
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
output wire mi_sinst_wvalid;
output wire [(`BW_DCA_MATRIX_LSU_INST)-1:0] mi_sinst_wdata;
input wire mi_sinst_wready;
input wire mi_sinst_decode_finish;
input wire mi_sinst_execute_finish;
input wire mi_sinst_busy;
input wire mi_sload_tensor_row_wvalid;
input wire mi_sload_tensor_row_wlast;
input wire [BW_INPUT_TENSOR_ROW-1:0] mi_sload_tensor_row_wdata;
output wire mi_sload_tensor_row_wready;
input wire mi_sstore_tensor_row_rvalid;
input wire mi_sstore_tensor_row_rlast;
output wire mi_sstore_tensor_row_rready;
output wire [BW_INPUT_TENSOR_ROW-1:0] mi_sstore_tensor_row_rdata;

output wire mk_sinst_wvalid;
output wire [(`BW_DCA_MATRIX_LSU_INST)-1:0] mk_sinst_wdata;
input wire mk_sinst_wready;
input wire mk_sinst_decode_finish;
input wire mk_sinst_execute_finish;
input wire mk_sinst_busy;
input wire mk_sload_tensor_row_wvalid;
input wire mk_sload_tensor_row_wlast;
input wire [BW_KERNEL_TENSOR_ROW-1:0] mk_sload_tensor_row_wdata;
output wire mk_sload_tensor_row_wready;
input wire mk_sstore_tensor_row_rvalid;
input wire mk_sstore_tensor_row_rlast;
output wire mk_sstore_tensor_row_rready;
output wire [BW_KERNEL_TENSOR_ROW-1:0] mk_sstore_tensor_row_rdata;

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
wire [`BW_DCA_MATRIX_INFO_ALIGNED-1:0] mi_info, mk_info, mo_info;
wire [`BW_DCA_MATRIX_CONV2D_INST_STRIDE_M1-1:0] inst_stride_m1;
wire [`BW_DCA_MATRIX_CONV2D_INST_PAD-1:0]       inst_pad;
assign {inst_pad,inst_stride_m1,mo_info,mk_info,mi_info} = control_rmx_inst_fifo_rdata;

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
assign mi_sinst_wvalid = (q_state==S_IDLE)  & control_rmx_inst_fifo_rready & mi_sinst_wready & mk_sinst_wready;
assign mk_sinst_wvalid = mi_sinst_wvalid;
assign mo_sinst_wvalid = (dq_state==S_IDLE) & control_rmx_inst_fifo_rready & mo_sinst_wready;

assign mi_sinst_wdata  = {mi_info, OPC_READ};
assign mk_sinst_wdata  = {mk_info, OPC_READ};
assign mo_sinst_wdata  = {mo_info, OPC_WRITE};

assign control_rmx_inst_fifo_rrequest = ((q_state==S_STORE) & q_go_idle) | ((dq_state==S_STORE) & dq_go_idle);
assign control_rmx_operation_finish   = (q_state==S_IDLE) & (dq_state==S_IDLE);

// ---------------- conv2d-style LOAD bypass → qdq ----------------
// sent 카운터 기반 valid/ready → fire로만 진행

// qdq handshakes
wire a_s_ready_o, b_s_ready_o, dq_s_ready_o;

// A stream
localparam int A_ROWS = INPUT_MATRIX_NUM_COL;
reg  [$clog2(A_ROWS+1)-1:0] a_sent;
wire a_valid = (q_state==S_EXEC) & (a_sent < A_ROWS);
assign mi_sload_tensor_row_wready = a_valid & a_s_ready_o;
wire a_fire = mi_sload_tensor_row_wvalid & mi_sload_tensor_row_wready;

always @(posedge clk or negedge rstnn) begin
  if(!rstnn)             a_sent <= '0;
  else if (q_go_exec)    a_sent <= '0;          // 시작 펄스에서만 초기화
  else if (a_fire)       a_sent <= a_sent + 1'b1;
end

// B stream
localparam int B_ROWS = KERNEL_MATRIX_NUM_COL;
reg  [$clog2(B_ROWS+1)-1:0] b_sent;
wire b_valid = (q_state==S_EXEC) & (b_sent < B_ROWS);
assign mk_sload_tensor_row_wready = b_valid & b_s_ready_o;
wire b_fire = mk_sload_tensor_row_wvalid & mk_sload_tensor_row_wready;

always @(posedge clk or negedge rstnn) begin
  if(!rstnn)             b_sent <= '0;
  else if (q_go_exec)    b_sent <= '0;
  else if (b_fire)       b_sent <= b_sent + 1'b1;
end

// ACC stream
wire dq_s_valid_i;
wire [BW_TENSOR_SCALAR*GET_MATRIX_NUM_COL(16)-1:0] dq_s_data_i;
localparam int OUT_ROWS = OUTPUT_MATRIX_NUM_COL;

reg  [$clog2(OUT_ROWS+1)-1:0] acc_sent;
wire acc_valid = (dq_state==S_EXEC) & (acc_sent < OUT_ROWS);

assign mo_sload_tensor_row_wready = acc_valid & dq_s_ready_o;          // 다운스트림 ready와 동기
assign dq_s_valid_i               = acc_valid & mo_sload_tensor_row_wvalid;
assign dq_s_data_i                = mo_sload_tensor_row_wdata;

wire acc_fire = mo_sload_tensor_row_wvalid & mo_sload_tensor_row_wready;

always @(posedge clk or negedge rstnn) begin
  if(!rstnn)            acc_sent <= '0;
  else if (dq_go_exec)  acc_sent <= '0;
  else if (acc_fire)    acc_sent <= acc_sent + 1'b1;
end

// ---------------- qdq controller ----------------
wire a_m_valid_o, b_m_valid_o, dq_m_valid_o;
wire a_m_ready_i, b_m_ready_i, dq_m_ready_i;
wire [BW_INPUT_TENSOR_ROW-1:0]  a_m_data_o;
wire [BW_KERNEL_TENSOR_ROW-1:0] b_m_data_o;
wire [BW_OUTPUT_TENSOR_ROW-1:0] dq_m_data_o;

qdq_controller #(
  .BIT_NUM(BIT_NUM), .MAT_SIZE(OUTPUT_MATRIX_SIZE),
  .FP_DATA_W(BW_TENSOR_SCALAR), .FP_EXP_W(FP_EXP_W),
  .FP_MANT_W(FP_MANT_W), .FP_EXP_BIAS(FP_EXP_BIAS),
  .LANES_NUM(OUTPUT_MATRIX_NUM_COL), .SCALE_FIFO_DEPTH(SCALE_FIFO_DEPTH)
) i_qdq (
  .clk(clk), .rstnn(rstnn),

  // A quant (bypass from sload)
  .a_s_valid_i ( a_valid & mi_sload_tensor_row_wvalid ),
  .a_s_ready_o ( a_s_ready_o ),
  .a_s_data_i  ( mi_sload_tensor_row_wdata ),
  .a_m_valid_o ( a_m_valid_o ),
  .a_m_ready_i ( a_m_ready_i ),
  .a_m_data_o  ( a_m_data_o ),

  // B quant (bypass from sload)
  .b_s_valid_i ( b_valid & mk_sload_tensor_row_wvalid ),
  .b_s_ready_o ( b_s_ready_o ),
  .b_s_data_i  ( mk_sload_tensor_row_wdata ),
  .b_m_valid_o ( b_m_valid_o ),
  .b_m_ready_i ( b_m_ready_i ),
  .b_m_data_o  ( b_m_data_o ),

  // ACC dequant (bypass from sload)
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

// FSM go conditions: conv2d 스트리밍 패턴
reg  qa_store_req,    qb_store_req,    out_store_req;
wire qa_store_busy, qb_store_busy, out_store_busy;

assign q_go_load  = mi_sinst_wvalid;                 // same-cycle guard
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
wire                           qa_mreg2store_ren;
wire [BW_INPUT_TENSOR_ROW-1:0] qa_mreg2store_rdata;
wire                           qb_mreg2store_ren;
wire [BW_KERNEL_TENSOR_ROW-1:0] qb_mreg2store_rdata;
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
  .MATRIX_SIZE_PARA(KERNEL_MATRIX_SIZE),
  .BW_TENSOR_SCALAR(BW_TENSOR_SCALAR),
  .BW_MOVE_DATA(BW_KERNEL_TENSOR_ROW),
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

// MREG2STORE (conv2d 스타일: store_wrequest 레벨 유지)
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

DCA_MATRIX_MREG2STORE #(
  .MATRIX_SIZE_PARA(INPUT_MATRIX_SIZE),
  .BW_TENSOR_SCALAR(BW_TENSOR_SCALAR)
) i_mreg2store_QA (
  .clk(clk), .rstnn(rstnn),
  .clear(1'b0), .enable(1'b1), .busy(qa_store_busy),  
  .storereg_wready(qa_store_wready),
  .storereg_wrequest(qa_store_req),
  .mreg_move_renable(qa_mreg2store_ren),
  .mreg_move_rdata_list1d(qa_mreg2store_rdata),
  .store_tensor_row_rvalid(mi_sstore_tensor_row_rvalid),
  .store_tensor_row_rlast (mi_sstore_tensor_row_rlast),
  .store_tensor_row_rready(mi_sstore_tensor_row_rready),
  .store_tensor_row_rdata (mi_sstore_tensor_row_rdata)
);

DCA_MATRIX_MREG2STORE #(
  .MATRIX_SIZE_PARA(KERNEL_MATRIX_SIZE),
  .BW_TENSOR_SCALAR(BW_TENSOR_SCALAR)
) i_mreg2store_QB (
  .clk(clk), .rstnn(rstnn),
  .clear(1'b0), .enable(1'b1), .busy(qb_store_busy),  
  .storereg_wready(qb_store_wready),
  .storereg_wrequest(qb_store_req),
  .mreg_move_renable(qb_mreg2store_ren),
  .mreg_move_rdata_list1d(qb_mreg2store_rdata),
  .store_tensor_row_rvalid(mk_sstore_tensor_row_rvalid),
  .store_tensor_row_rlast (mk_sstore_tensor_row_rlast),
  .store_tensor_row_rready(mk_sstore_tensor_row_rready),
  .store_tensor_row_rdata (mk_sstore_tensor_row_rdata)
);

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
reg [BW_TENSOR_SCALAR-1:0] mi_sload_tensor_row_wdata_reg [0:GET_MATRIX_NUM_COL(16)-1];
reg [BW_TENSOR_SCALAR-1:0] mi_sstore_tensor_row_rdata_reg [0:GET_MATRIX_NUM_COL(16)-1];

integer __r, __idx;
always @* begin
    for (__r = 0; __r < GET_MATRIX_NUM_COL(16); __r = __r + 1) begin
        mi_sload_tensor_row_wdata_reg[__r] = mi_sload_tensor_row_wdata[(__r+1)*BW_TENSOR_SCALAR-1 -: BW_TENSOR_SCALAR];
        mi_sstore_tensor_row_rdata_reg[__r] = mi_sstore_tensor_row_rdata[(__r+1)*BW_TENSOR_SCALAR-1 -: BW_TENSOR_SCALAR];
    end
end

endmodule
