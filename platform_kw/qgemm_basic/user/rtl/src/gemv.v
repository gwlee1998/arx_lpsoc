module GEMV (
  clk,
  rstnn,

  start_i,
  busy_o,

  qx_valid_i,
  qx_ready_o,
  qx_data_i,
  qx_index_i,

  qw_valid_i,
  qw_ready_o,
  qw_data_i,
  qw_index_i,

  qo_valid_o,
  qo_ready_i,
  qo_data_o,
  qo_index_o
);

  parameter MATRIX_SIZE       = 16;
  parameter BW_IN_DATA        = 32;
  parameter BW_OUT_DATA       = 69;
  parameter DP_LATENCY        = 1;
  parameter MREG_RESET_VALUE  = 0;
  parameter BW_TENSOR_SCALAR  = 32;

  localparam BW_TENSOR_MATRIX = BW_IN_DATA * MATRIX_SIZE * MATRIX_SIZE;
  localparam INDEX_W           = $clog2(MATRIX_SIZE+1);

  input  wire clk;
  input  wire rstnn;
  input  wire start_i;
  output wire busy_o;

  input  wire                              qx_valid_i;
  output wire                              qx_ready_o;
  input  wire [MATRIX_SIZE*BW_IN_DATA-1:0] qx_data_i;
  input  wire [INDEX_W-1:0]                qx_index_i;

  input  wire                              qw_valid_i;
  output wire                              qw_ready_o;
  input  wire [MATRIX_SIZE*BW_IN_DATA-1:0] qw_data_i;
  input  wire [INDEX_W-1:0]                qw_index_i;

  output wire                              qo_valid_o;
  input  wire                              qo_ready_i;
  output wire [MATRIX_SIZE*BW_IN_DATA-1:0] qo_data_o;
  output wire [INDEX_W-1:0]                qo_index_o;

  wire                              mqwreg_move_wenable;
  wire [MATRIX_SIZE*BW_IN_DATA-1:0] mqwreg_move_wdata_list;
  wire                              mqwreg_move_renable;
  wire [MATRIX_SIZE*BW_IN_DATA-1:0] mqwreg_move_rdata_list;
  wire                         		  mqwreg_shift_up;
  wire                         		  mqwreg_shift_left;
  wire                         		  mqwreg_transpose;
  wire [BW_TENSOR_MATRIX-1:0]  		  mqwreg_all_rdata_list2d;
  wire [MATRIX_SIZE*BW_IN_DATA-1:0] mqwreg_upmost_rdata_list1d;

  wire                              mqoreg_move_wenable;
  wire [MATRIX_SIZE*BW_IN_DATA-1:0] mqoreg_move_wdata_list;
  wire                              mqoreg_move_renable;
  wire [MATRIX_SIZE*BW_IN_DATA-1:0] mqoreg_move_rdata_list;
  wire                         		  mqoreg_shift_up;
  wire                         		  mqoreg_shift_left;
  wire                         		  mqoreg_transpose;
  wire [BW_TENSOR_MATRIX-1:0]  		  mqoreg_all_rdata_list2d;
  wire [MATRIX_SIZE*BW_IN_DATA-1:0] mqoreg_upmost_rdata_list1d;

  wire [MATRIX_SIZE*BW_IN_DATA-1:0] i_gemv_x_row;
  wire [BW_TENSOR_MATRIX-1:0] 		  i_gemv_tw_mat;
  wire [MATRIX_SIZE*BW_IN_DATA-1:0] i_gemv_out_row; 

  wire gemv_feed_fire;
  wire gemv_out_valid;
  wire mqoreg_out_hs;
  wire go_load, go_run, go_idle;

  reg mqwreg_full;
  reg [INDEX_W-1:0] gemv_feed_cnt;
  reg [INDEX_W-1:0] gemv_out_cnt;
  reg [INDEX_W-1:0] mqoreg_out_cnt;

  reg [1:0] state;
  localparam S_IDLE = 2'b00;
  localparam S_LOAD = 2'b01;
  localparam S_RUN  = 2'b10;
  localparam S_STORE = 2'b11;

  assign go_load = start_i;
  assign go_run  = mqwreg_full;
  assign go_store = (gemv_out_cnt == MATRIX_SIZE - 1);
  assign go_idle = (state == S_STORE) && mqoreg_out_hs && (mqoreg_out_cnt == MATRIX_SIZE-1);

  always @(posedge clk or negedge rstnn) begin
    if (!rstnn) begin
      state <= S_IDLE;
    end else begin
      case (state)
        S_IDLE: if (go_load) state <= S_LOAD;
        S_LOAD: if (go_run)  state <= S_RUN;
        S_RUN : if (go_store) state <= S_STORE;
        S_STORE: if (go_idle) state <= S_IDLE;
        default:             state <= S_IDLE;
      endcase
    end
  end

  assign busy_o     = (state==S_RUN);
  assign qw_ready_o = (state==S_LOAD);
  assign qx_ready_o = (state==S_RUN);

  assign i_gemv_x_row = qx_data_i;

  DCA_MATRIX_REGISTER_TYPE3 #(
    .MATRIX_SIZE_PARA (MATRIX_SIZE),
    .BW_TENSOR_SCALAR (BW_TENSOR_SCALAR),
    .BW_MOVE_DATA     (MATRIX_SIZE*BW_TENSOR_SCALAR),
    .RESET_VALUE      (MREG_RESET_VALUE)
  ) i_mqwreg (
    .clk                  (clk),
    .rstnn                (rstnn),
    .move_wenable         (mqwreg_move_wenable),
    .move_wdata_list      (mqwreg_move_wdata_list),
    .move_renable         (mqwreg_move_renable),
    .move_rdata_list      (mqwreg_move_rdata_list),
    .shift_up             (mqwreg_shift_up),
    .shift_left           (mqwreg_shift_left),
    .transpose            (mqwreg_transpose),
    .all_rdata_list2d     (mqwreg_all_rdata_list2d),
    .upmost_rdata_list1d  (mqwreg_upmost_rdata_list1d)
  );

  assign mqwreg_move_wenable    = qw_valid_i && (state == S_LOAD);
  assign mqwreg_move_wdata_list = qw_data_i;

  DCA_MATRIX_REGISTER_TYPE3 #(
    .MATRIX_SIZE_PARA (MATRIX_SIZE),
    .BW_TENSOR_SCALAR (BW_TENSOR_SCALAR),
    .BW_MOVE_DATA     (MATRIX_SIZE*BW_TENSOR_SCALAR),
    .RESET_VALUE      (MREG_RESET_VALUE)
  ) i_mqoreg (
    .clk                  (clk),
    .rstnn                (rstnn),
    .move_wenable         (mqoreg_move_wenable),
    .move_wdata_list      (mqoreg_move_wdata_list),
    .move_renable         (mqoreg_move_renable),
    .move_rdata_list      (mqoreg_move_rdata_list),
    .shift_up             (mqoreg_shift_up),
    .shift_left           (mqoreg_shift_left),
    .transpose            (mqoreg_transpose),
    .all_rdata_list2d     (mqoreg_all_rdata_list2d),
    .upmost_rdata_list1d  (mqoreg_upmost_rdata_list1d)
  );

  assign mqoreg_move_wenable    = gemv_out_valid && (state == S_RUN);
  assign mqoreg_move_wdata_list = i_gemv_out_row;
  assign mqoreg_move_renable    = qo_ready_i;
  assign qo_data_o              = mqoreg_move_rdata_list;
  assign qo_valid_o             = (state == S_STORE);
  assign qo_index_o             = mqoreg_out_cnt;

  always @(posedge clk or negedge rstnn) begin
    if(!rstnn) mqwreg_full <= 1'b0;
    else if (go_idle) mqwreg_full <= 1'b0;
    else if ((state == S_LOAD) && qw_valid_i && (qw_index_i == MATRIX_SIZE-1))
      mqwreg_full <= 1'b1;
  end

  genvar i;
  generate
    for (i = 0; i < MATRIX_SIZE; i = i + 1) begin : GEN_GEMV
      wire [BW_OUT_DATA-1:0] out_wide;
      DotProduct #(
        .IN_DATA_W (BW_IN_DATA),
        .OUT_DATA_W(BW_OUT_DATA)
      ) i_dp (
        .clock  (clk),
        .io_a_0 (i_gemv_x_row[ 0*BW_IN_DATA +: BW_IN_DATA]),
        .io_a_1 (i_gemv_x_row[ 1*BW_IN_DATA +: BW_IN_DATA]),
        .io_a_2 (i_gemv_x_row[ 2*BW_IN_DATA +: BW_IN_DATA]),
        .io_a_3 (i_gemv_x_row[ 3*BW_IN_DATA +: BW_IN_DATA]),
        .io_a_4 (i_gemv_x_row[ 4*BW_IN_DATA +: BW_IN_DATA]),
        .io_a_5 (i_gemv_x_row[ 5*BW_IN_DATA +: BW_IN_DATA]),
        .io_a_6 (i_gemv_x_row[ 6*BW_IN_DATA +: BW_IN_DATA]),
        .io_a_7 (i_gemv_x_row[ 7*BW_IN_DATA +: BW_IN_DATA]),
        .io_a_8 (i_gemv_x_row[ 8*BW_IN_DATA +: BW_IN_DATA]),
        .io_a_9 (i_gemv_x_row[ 9*BW_IN_DATA +: BW_IN_DATA]),
        .io_a_10(i_gemv_x_row[10*BW_IN_DATA +: BW_IN_DATA]),
        .io_a_11(i_gemv_x_row[11*BW_IN_DATA +: BW_IN_DATA]),
        .io_a_12(i_gemv_x_row[12*BW_IN_DATA +: BW_IN_DATA]),
        .io_a_13(i_gemv_x_row[13*BW_IN_DATA +: BW_IN_DATA]),
        .io_a_14(i_gemv_x_row[14*BW_IN_DATA +: BW_IN_DATA]),
        .io_a_15(i_gemv_x_row[15*BW_IN_DATA +: BW_IN_DATA]),
        .io_b_0 (i_gemv_tw_mat[( 0*MATRIX_SIZE + i)*BW_IN_DATA +: BW_IN_DATA]),
        .io_b_1 (i_gemv_tw_mat[( 1*MATRIX_SIZE + i)*BW_IN_DATA +: BW_IN_DATA]),
        .io_b_2 (i_gemv_tw_mat[( 2*MATRIX_SIZE + i)*BW_IN_DATA +: BW_IN_DATA]),
        .io_b_3 (i_gemv_tw_mat[( 3*MATRIX_SIZE + i)*BW_IN_DATA +: BW_IN_DATA]),
        .io_b_4 (i_gemv_tw_mat[( 4*MATRIX_SIZE + i)*BW_IN_DATA +: BW_IN_DATA]),
        .io_b_5 (i_gemv_tw_mat[( 5*MATRIX_SIZE + i)*BW_IN_DATA +: BW_IN_DATA]),
        .io_b_6 (i_gemv_tw_mat[( 6*MATRIX_SIZE + i)*BW_IN_DATA +: BW_IN_DATA]),
        .io_b_7 (i_gemv_tw_mat[( 7*MATRIX_SIZE + i)*BW_IN_DATA +: BW_IN_DATA]),
        .io_b_8 (i_gemv_tw_mat[( 8*MATRIX_SIZE + i)*BW_IN_DATA +: BW_IN_DATA]),
        .io_b_9 (i_gemv_tw_mat[( 9*MATRIX_SIZE + i)*BW_IN_DATA +: BW_IN_DATA]),
        .io_b_10(i_gemv_tw_mat[(10*MATRIX_SIZE + i)*BW_IN_DATA +: BW_IN_DATA]),
        .io_b_11(i_gemv_tw_mat[(11*MATRIX_SIZE + i)*BW_IN_DATA +: BW_IN_DATA]),
        .io_b_12(i_gemv_tw_mat[(12*MATRIX_SIZE + i)*BW_IN_DATA +: BW_IN_DATA]),
        .io_b_13(i_gemv_tw_mat[(13*MATRIX_SIZE + i)*BW_IN_DATA +: BW_IN_DATA]),
        .io_b_14(i_gemv_tw_mat[(14*MATRIX_SIZE + i)*BW_IN_DATA +: BW_IN_DATA]),
        .io_b_15(i_gemv_tw_mat[(15*MATRIX_SIZE + i)*BW_IN_DATA +: BW_IN_DATA]),
        .io_y   (out_wide)
      );
      assign i_gemv_out_row[i*BW_IN_DATA +: BW_IN_DATA] = out_wide[BW_IN_DATA-1:0];
    end
  endgenerate

  // transpose
  genvar j, r;
  generate
    for (j = 0; j < MATRIX_SIZE; j = j + 1) begin
      for (r = 0; r < MATRIX_SIZE; r = r + 1) begin
        assign i_gemv_tw_mat[((j*MATRIX_SIZE + r)*BW_IN_DATA) +: BW_IN_DATA]
             = mqwreg_all_rdata_list2d[((r*MATRIX_SIZE + j)*BW_IN_DATA) +: BW_IN_DATA];
      end
    end
  endgenerate

  always @(posedge clk or negedge rstnn) begin
    if (!rstnn) gemv_feed_cnt <= 0;
    else if (gemv_feed_fire)  gemv_feed_cnt <= gemv_feed_cnt + 1'b1;
    else if (state==S_RUN && go_idle) gemv_feed_cnt <= 0;
  end

  always @(posedge clk or negedge rstnn) begin
    if (!rstnn) gemv_out_cnt <= 0;
    else if (gemv_out_valid)  gemv_out_cnt <= gemv_out_cnt + 1'b1;
    else if (state==S_RUN && go_idle) gemv_out_cnt <= 0;
  end

  assign mqoreg_out_hs = (state == S_STORE) && qo_ready_i;
  always @(posedge clk or negedge rstnn) begin
    if (!rstnn) mqoreg_out_cnt <= 0;
    else if (mqoreg_out_hs) mqoreg_out_cnt <= mqoreg_out_cnt + 1'b1;
    else if (state == S_STORE && go_idle) mqoreg_out_cnt <= 0;
  end

  assign gemv_feed_fire = (state==S_RUN) && (gemv_feed_cnt < MATRIX_SIZE) && (qx_valid_i);

  reg [DP_LATENCY:0] vpipe;
  always @(posedge clk or negedge rstnn) begin
    if (!rstnn) vpipe <= 0;
    else        vpipe <= {vpipe[DP_LATENCY-1:0], gemv_feed_fire};
  end
  assign gemv_out_valid = vpipe[DP_LATENCY];

endmodule
