// ****************************************************************************
// ****************************************************************************
// Copyright SoC Design Research Group, All rights reserved.    
// Electronics and Telecommunications Research Institute (ETRI)
//
// THESE DOCUMENTS CONTAIN CONFIDENTIAL INFORMATION AND KNOWLEDGE 
// WHICH IS THE PROPERTY OF ETRI. NO PART OF THIS PUBLICATION IS 
// TO BE USED FOR ANY OTHER PURPOSE, AND THESE ARE NOT TO BE 
// REPRODUCED, COPIED, DISCLOSED, TRANSMITTED, STORED IN A RETRIEVAL 
// SYSTEM OR TRANSLATED INTO ANY OTHER HUMAN OR COMPUTER LANGUAGE, 
// IN ANY FORM, BY ANY MEANS, IN WHOLE OR IN PART, WITHOUT THE 
// COMPLETE PRIOR WRITTEN PERMISSION OF ETRI.
// ****************************************************************************
// 2021-10-08
// Kyuseung Han (han@etri.re.kr)
// ****************************************************************************
// ****************************************************************************

`include "ervp_global.vh"

module DCA_MATRIX_REGISTER_TEMPLATE
(
  clk,
  rstnn,

  init_list2d,

  each_wenable_list2d,
  each_wdata_list2d,

  downmost_wenable_list1d,
  downmost_wdata_list1d,

  upmost_wenable_list1d,
  upmost_wdata_list1d,

  rightmost_wenable_list1d,
  rightmost_wdata_list1d,

  leftmost_wenable_list1d,
  leftmost_wdata_list1d,
  
  shift_up_list2d,
  shift_down_list2d,
  shift_left_list2d,
  shift_right_list2d,

  transpose_list2d,
  
  all_rdata_list2d,
  upmost_rdata_list1d,
  downmost_data_list1d,
  leftmost_data_list1d,
  rightmost_data_list1d
);

////////////////////////////
/* parameter input output */
////////////////////////////

parameter MATRIX_SIZE_PARA = 8;
parameter BW_TENSOR_SCALAR = 32;
parameter RESET_VALUE = 0;
parameter INIT_VALUE = RESET_VALUE;

`include "dca_matrix_dim_util.vb"
`include "dca_matrix_dim_lpara.vb"
`include "dca_tensor_dim_lpara.vb"

input wire clk, rstnn;
input wire [MATRIX_NUM_ELEMENT-1:0] init_list2d;

input wire [MATRIX_NUM_ELEMENT-1:0] each_wenable_list2d;
input wire [BW_TENSOR_MATRIX-1:0] each_wdata_list2d;

input wire [MATRIX_NUM_COL-1:0] downmost_wenable_list1d;
input wire [BW_TENSOR_ROW-1:0] downmost_wdata_list1d;

input wire [MATRIX_NUM_COL-1:0] upmost_wenable_list1d;
input wire [BW_TENSOR_ROW-1:0] upmost_wdata_list1d;

input wire [MATRIX_NUM_ROW-1:0] rightmost_wenable_list1d;
input wire [BW_TENSOR_COL-1:0] rightmost_wdata_list1d;

input wire [MATRIX_NUM_ROW-1:0] leftmost_wenable_list1d;
input wire [BW_TENSOR_COL-1:0] leftmost_wdata_list1d;

input wire [MATRIX_NUM_ELEMENT-1:0] shift_up_list2d;
input wire [MATRIX_NUM_ELEMENT-1:0] shift_down_list2d;
input wire [MATRIX_NUM_ELEMENT-1:0] shift_left_list2d;
input wire [MATRIX_NUM_ELEMENT-1:0] shift_right_list2d;

input wire [MATRIX_NUM_ELEMENT-1:0] transpose_list2d;

output wire [BW_TENSOR_MATRIX-1:0] all_rdata_list2d;
output wire [BW_TENSOR_ROW-1:0] upmost_rdata_list1d;
output wire [BW_TENSOR_ROW-1:0] downmost_data_list1d;
output wire [BW_TENSOR_COL-1:0] leftmost_data_list1d;
output wire [BW_TENSOR_COL-1:0] rightmost_data_list1d;

/////////////
/* signals */
/////////////

localparam FIRST_RI = 0;
localparam LAST_RI = MATRIX_NUM_ROW-1;
localparam FIRST_CI = 0;
localparam LAST_CI = MATRIX_NUM_COL-1;

genvar i;
genvar i_row, i_col;

// (0,0) locates an upmost and leftmost point
reg [BW_TENSOR_SCALAR-1:0] data [0:MATRIX_NUM_ROW-1][0:MATRIX_NUM_COL-1];
wire [BW_TENSOR_ROW-1:0] row_data_list1d [0:MATRIX_NUM_ROW-1];
wire [BW_TENSOR_COL-1:0] col_data_list1d [0:MATRIX_NUM_COL-1];

wire init [0:MATRIX_NUM_ROW-1][0:MATRIX_NUM_COL-1];
wire each_wenable [0:MATRIX_NUM_ROW-1][0:MATRIX_NUM_COL-1];
wire [BW_TENSOR_SCALAR-1:0] each_wdata [0:MATRIX_NUM_ROW-1][0:MATRIX_NUM_COL-1];
wire downmost_wenable [0:MATRIX_NUM_COL-1];
wire [BW_TENSOR_SCALAR-1:0] downmost_wdata [0:MATRIX_NUM_COL-1];
wire upmost_wenable [0:MATRIX_NUM_COL-1];
wire [BW_TENSOR_SCALAR-1:0] upmost_wdata [0:MATRIX_NUM_COL-1];
wire rightmost_wenable [0:MATRIX_NUM_ROW-1];
wire [BW_TENSOR_SCALAR-1:0] rightmost_wdata [0:MATRIX_NUM_ROW-1];
wire leftmost_wenable [0:MATRIX_NUM_ROW-1];
wire [BW_TENSOR_SCALAR-1:0] leftmost_wdata [0:MATRIX_NUM_ROW-1];

wire shift_up [0:MATRIX_NUM_ROW-1][0:MATRIX_NUM_COL-1];
wire shift_down [0:MATRIX_NUM_ROW-1][0:MATRIX_NUM_COL-1];
wire shift_left [0:MATRIX_NUM_ROW-1][0:MATRIX_NUM_COL-1];
wire shift_right [0:MATRIX_NUM_ROW-1][0:MATRIX_NUM_COL-1];

wire transpose [0:MATRIX_NUM_ROW-1][0:MATRIX_NUM_COL-1];

localparam MATRIX_MAX_SIZE = `MAX(MATRIX_NUM_ROW,MATRIX_NUM_COL);

wire [BW_TENSOR_SCALAR-1:0] square_data [0:MATRIX_MAX_SIZE-1][0:MATRIX_MAX_SIZE-1];

////////////
/* logics */
////////////

generate
  for(i_row=0; i_row<MATRIX_NUM_ROW; i_row=i_row+1)
  begin : i_gen_row_data_list1d
    for(i_col=0; i_col<MATRIX_NUM_COL; i_col=i_col+1)
    begin : i_concat_col
      assign row_data_list1d[i_row][(i_col+1)*BW_TENSOR_SCALAR-1 -:BW_TENSOR_SCALAR] = data[i_row][i_col];
    end
  end
endgenerate

generate
  for(i_col=0; i_col<MATRIX_NUM_COL; i_col=i_col+1)
  begin : i_gen_col_data_list1d
    for(i_row=0; i_row<MATRIX_NUM_ROW; i_row=i_row+1)
    begin : i_concat_row
      assign col_data_list1d[i_col][(i_row+1)*BW_TENSOR_SCALAR-1 -:BW_TENSOR_SCALAR] = data[i_row][i_col];
    end
  end
endgenerate

generate
  for(i_row=0; i_row<MATRIX_NUM_ROW; i_row=i_row+1)
  begin : i_concat_all_rdata_list2d
    assign all_rdata_list2d[(i_row+1)*BW_TENSOR_ROW-1 -:BW_TENSOR_ROW] = row_data_list1d[i_row];
  end
endgenerate

generate
  for(i_row=0; i_row<MATRIX_MAX_SIZE; i_row=i_row+1)
  begin : i_gen_square_data_row
    for(i_col=0; i_col<MATRIX_MAX_SIZE; i_col=i_col+1)
    begin : i_gen_square_data_col
      if(i_row>=MATRIX_NUM_ROW)
      begin
        assign square_data[i_row][i_col] = 0;
      end
      else if(i_col>=MATRIX_NUM_COL)
      begin
        assign square_data[i_row][i_col] = 0;
      end
      else
      begin
        assign square_data[i_row][i_col] = data[i_row][i_col];
      end
    end
  end
endgenerate

assign upmost_rdata_list1d = row_data_list1d[FIRST_RI];
assign downmost_data_list1d = row_data_list1d[LAST_RI];
assign leftmost_data_list1d = col_data_list1d[FIRST_RI];
assign rightmost_data_list1d = col_data_list1d[LAST_RI];

// 
generate
  for(i_row=0; i_row<MATRIX_NUM_ROW; i_row=i_row+1)
  begin : i_split_input_row
    for(i_col=0; i_col<MATRIX_NUM_COL; i_col=i_col+1)
    begin : i_split_input_col
      assign init[i_row][i_col] = init_list2d[(i_row*MATRIX_NUM_COL + i_col + 1)*1-1 -:1];
      assign each_wenable[i_row][i_col] = each_wenable_list2d[(i_row*MATRIX_NUM_COL + i_col + 1)*1-1 -:1];
      assign each_wdata[i_row][i_col] = each_wdata_list2d[(i_row*MATRIX_NUM_COL + i_col + 1)*BW_TENSOR_SCALAR-1 -:BW_TENSOR_SCALAR];
      assign shift_up[i_row][i_col] = shift_up_list2d[(i_row*MATRIX_NUM_COL + i_col + 1)*1-1 -:1];
      assign shift_down[i_row][i_col] = shift_down_list2d[(i_row*MATRIX_NUM_COL + i_col + 1)*1-1 -:1];
      assign shift_left[i_row][i_col] = shift_left_list2d[(i_row*MATRIX_NUM_COL + i_col + 1)*1-1 -:1];
      assign shift_right[i_row][i_col] = shift_right_list2d[(i_row*MATRIX_NUM_COL + i_col + 1)*1-1 -:1];
      assign transpose[i_row][i_col] = transpose_list2d[(i_row*MATRIX_NUM_COL + i_col + 1)*1-1 -:1];
    end
  end
endgenerate

generate
  for(i_col=0; i_col<MATRIX_NUM_COL; i_col=i_col+1)
  begin : i_split_input_col2
    assign downmost_wenable[i_col] = downmost_wenable_list1d[(i_col+1)*1-1 -:1];
    assign downmost_wdata[i_col] = downmost_wdata_list1d[(i_col+1)*BW_TENSOR_SCALAR-1 -:BW_TENSOR_SCALAR];
    assign upmost_wenable[i_col] = upmost_wenable_list1d[(i_col+1)*1-1 -:1];
    assign upmost_wdata[i_col] = upmost_wdata_list1d[(i_col+1)*BW_TENSOR_SCALAR-1 -:BW_TENSOR_SCALAR];
  end
endgenerate

generate
  for(i_row=0; i_row<MATRIX_NUM_ROW; i_row=i_row+1)
  begin : i_split_input_row2
    assign rightmost_wenable[i_row] = rightmost_wenable_list1d[(i_row+1)*1-1 -:1];
    assign rightmost_wdata[i_row] = rightmost_wdata_list1d[(i_row+1)*BW_TENSOR_SCALAR-1 -:BW_TENSOR_SCALAR];
    assign leftmost_wenable[i_row] = leftmost_wenable_list1d[(i_row+1)*1-1 -:1];
    assign leftmost_wdata[i_row] = leftmost_wdata_list1d[(i_row+1)*BW_TENSOR_SCALAR-1 -:BW_TENSOR_SCALAR];
  end
endgenerate

//

generate
  for(i_row=1; i_row<(MATRIX_NUM_ROW-1); i_row=i_row+1)
  begin : i_reg_center_row
    for(i_col=1; i_col<(MATRIX_NUM_COL-1); i_col=i_col+1)
    begin : i_reg_center_col
      always@(posedge clk, negedge rstnn)
      begin
        if(rstnn==0)
          data[i_row][i_col] <= RESET_VALUE;
        else if(init[i_row][i_col])
          data[i_row][i_col] <= INIT_VALUE;
        else if(each_wenable[i_row][i_col])
          data[i_row][i_col] <= each_wdata[i_row][i_col];
        else if(shift_up[i_row][i_col])
          data[i_row][i_col] <= data[i_row+1][i_col];
        else if(shift_down[i_row][i_col])
          data[i_row][i_col] <= data[i_row-1][i_col];
        else if(shift_left[i_row][i_col])
          data[i_row][i_col] <= data[i_row][i_col+1];
        else if(shift_right[i_row][i_col])
          data[i_row][i_col] <= data[i_row][i_col-1];
        else if(transpose[i_row][i_col])
        begin
          if(i_row!=i_col)
            data[i_row][i_col] <= square_data[i_col][i_row];
        end
      end
    end
  end
endgenerate

generate
  for(i_col=1; i_col<(MATRIX_NUM_COL-1); i_col=i_col+1)
  begin : i_reg_up_row
    always@(posedge clk, negedge rstnn)
    begin
      if(rstnn==0)
        data[FIRST_RI][i_col] <= RESET_VALUE;
      else if(init[FIRST_RI][i_col])
        data[FIRST_RI][i_col] <= INIT_VALUE;
      else if(each_wenable[FIRST_RI][i_col])
        data[FIRST_RI][i_col] <= each_wdata[FIRST_RI][i_col];
      else if(upmost_wenable[i_col])
        data[FIRST_RI][i_col] <= upmost_wdata[i_col];
      else if(shift_up[FIRST_RI][i_col])
        data[FIRST_RI][i_col] <= data[FIRST_RI+1][i_col];
      else if(shift_down[FIRST_RI][i_col])
        data[FIRST_RI][i_col] <= data[LAST_RI][i_col];
      else if(shift_left[FIRST_RI][i_col])
        data[FIRST_RI][i_col] <= data[FIRST_RI][i_col+1];
      else if(shift_right[FIRST_RI][i_col])
        data[FIRST_RI][i_col] <= data[FIRST_RI][i_col-1];
      else if(transpose[FIRST_RI][i_col])
        data[FIRST_RI][i_col] <= square_data[i_col][FIRST_RI];
    end
  end
endgenerate

generate
  for(i_col=1; i_col<(MATRIX_NUM_COL-1); i_col=i_col+1)
  begin : i_reg_down_row
    always@(posedge clk, negedge rstnn)
    begin
      if(rstnn==0)
        data[LAST_RI][i_col] <= RESET_VALUE;
      else if(init[LAST_RI][i_col])
        data[LAST_RI][i_col] <= INIT_VALUE;
      else if(each_wenable[LAST_RI][i_col])
        data[LAST_RI][i_col] <= each_wdata[LAST_RI][i_col];
      else if(downmost_wenable[i_col])
        data[LAST_RI][i_col] <= downmost_wdata[i_col];
      else if(shift_up[LAST_RI][i_col])
        data[LAST_RI][i_col] <= data[FIRST_RI][i_col];
      else if(shift_down[LAST_RI][i_col])
        data[LAST_RI][i_col] <= data[LAST_RI-1][i_col];
      else if(shift_left[LAST_RI][i_col])
        data[LAST_RI][i_col] <= data[LAST_RI][i_col+1];
      else if(shift_right[LAST_RI][i_col])
        data[LAST_RI][i_col] <= data[LAST_RI][i_col-1];
      else if(transpose[LAST_RI][i_col])
        data[LAST_RI][i_col] <= square_data[i_col][LAST_RI];
    end
  end
endgenerate

generate
  for(i_row=1; i_row<(MATRIX_NUM_ROW-1); i_row=i_row+1)
  begin : i_reg_left_col
    always@(posedge clk, negedge rstnn)
    begin
      if(rstnn==0)
        data[i_row][FIRST_CI] <= RESET_VALUE;
      else if(init[i_row][FIRST_CI])
        data[i_row][FIRST_CI] <= INIT_VALUE;
      else if(each_wenable[i_row][FIRST_CI])
        data[i_row][FIRST_CI] <= each_wdata[i_row][FIRST_CI];
      else if(leftmost_wenable[i_row])
        data[i_row][FIRST_CI] <= leftmost_wdata[i_row];
      else if(shift_up[i_row][FIRST_CI])
        data[i_row][FIRST_CI] <= data[i_row+1][FIRST_CI];
      else if(shift_down[i_row][FIRST_CI])
        data[i_row][FIRST_CI] <= data[i_row-1][FIRST_CI];
      else if(shift_left[i_row][FIRST_CI])
        data[i_row][FIRST_CI] <= data[i_row][FIRST_CI+1];
      else if(shift_right[i_row][FIRST_CI])
        data[i_row][FIRST_CI] <= data[i_row][LAST_CI];
      else if(transpose[i_row][FIRST_CI])
        data[i_row][FIRST_CI] <= square_data[FIRST_CI][i_row];
    end
  end
endgenerate

generate
  for(i_row=1; i_row<(MATRIX_NUM_ROW-1); i_row=i_row+1)
  begin : i_reg_right_col
    always@(posedge clk, negedge rstnn)
    begin
      if(rstnn==0)
        data[i_row][LAST_CI] <= RESET_VALUE;
      else if(init[i_row][LAST_CI])
        data[i_row][LAST_CI] <= INIT_VALUE;
      else if(each_wenable[i_row][LAST_CI])
        data[i_row][LAST_CI] <= each_wdata[i_row][LAST_CI];
      else if(rightmost_wenable[i_row])
        data[i_row][LAST_CI] <= rightmost_wdata[i_row];
      else if(shift_up[i_row][LAST_CI])
        data[i_row][LAST_CI] <= data[i_row+1][LAST_CI];
      else if(shift_down[i_row][LAST_CI])
        data[i_row][LAST_CI] <= data[i_row-1][LAST_CI];
      else if(shift_left[i_row][LAST_CI])
        data[i_row][LAST_CI] <= data[i_row][FIRST_CI];
      else if(shift_right[i_row][LAST_CI])
        data[i_row][LAST_CI] <= data[i_row][LAST_CI-1];
      else if(transpose[i_row][LAST_CI])
        data[i_row][LAST_CI] <= square_data[LAST_CI][i_row];
    end
  end
endgenerate

// upmost and leftmost
always@(posedge clk, negedge rstnn)
begin
  if(rstnn==0)
    data[FIRST_RI][FIRST_CI] <= RESET_VALUE;
  else if(init[FIRST_RI][FIRST_CI])
    data[FIRST_RI][FIRST_CI] <= INIT_VALUE;
  else if(each_wenable[FIRST_RI][FIRST_CI])
    data[FIRST_RI][FIRST_CI] <= each_wdata[FIRST_RI][FIRST_CI];
  else if(upmost_wenable[FIRST_RI])
    data[FIRST_RI][FIRST_CI] <= upmost_wdata[FIRST_RI];
  else if(leftmost_wenable[FIRST_RI])
    data[FIRST_RI][FIRST_CI] <= leftmost_wdata[FIRST_RI];
  else if(shift_up[FIRST_RI][FIRST_CI])
    data[FIRST_RI][FIRST_CI] <= data[FIRST_RI+1][FIRST_CI];
  else if(shift_down[FIRST_RI][FIRST_CI])
    data[FIRST_RI][FIRST_CI] <= data[LAST_RI][FIRST_CI];
  else if(shift_left[FIRST_RI][FIRST_CI])
    data[FIRST_RI][FIRST_CI] <= data[FIRST_RI][FIRST_CI+1];
  else if(shift_right[FIRST_RI][FIRST_CI])
    data[FIRST_RI][FIRST_CI] <= data[FIRST_RI][LAST_CI];
end

// upmost and rightmost
always@(posedge clk, negedge rstnn)
begin
  if(rstnn==0)
    data[FIRST_RI][LAST_CI] <= RESET_VALUE;
  else if(init[FIRST_RI][LAST_CI])
    data[FIRST_RI][LAST_CI] <= INIT_VALUE;
  else if(each_wenable[FIRST_RI][LAST_CI])
    data[FIRST_RI][LAST_CI] <= each_wdata[FIRST_RI][LAST_CI];
  else if(upmost_wenable[LAST_RI])
    data[FIRST_RI][LAST_CI] <= upmost_wdata[LAST_RI];
  else if(rightmost_wenable[FIRST_RI])
    data[FIRST_RI][LAST_CI] <= rightmost_wdata[FIRST_RI];
  else if(shift_up[FIRST_RI][LAST_CI])
    data[FIRST_RI][LAST_CI] <= data[FIRST_RI+1][LAST_CI];
  else if(shift_down[FIRST_RI][LAST_CI])
    data[FIRST_RI][LAST_CI] <= data[LAST_RI][LAST_CI];
  else if(shift_left[FIRST_RI][LAST_CI])
    data[FIRST_RI][LAST_CI] <= data[FIRST_RI][FIRST_CI];
  else if(shift_right[FIRST_RI][LAST_CI])
    data[FIRST_RI][LAST_CI] <= data[FIRST_RI][LAST_CI-1];
  else if(transpose[FIRST_RI][LAST_CI])
    data[FIRST_RI][LAST_CI] <= data[LAST_RI][FIRST_CI];
end

// downmost and leftmost
always@(posedge clk, negedge rstnn)
begin
  if(rstnn==0)
    data[LAST_RI][FIRST_CI] <= RESET_VALUE;
  else if(init[LAST_RI][FIRST_CI])
    data[LAST_RI][FIRST_CI] <= INIT_VALUE;
  else if(each_wenable[LAST_RI][FIRST_CI])
    data[LAST_RI][FIRST_CI] <= each_wdata[LAST_RI][FIRST_CI];
  else if(downmost_wenable[FIRST_RI])
    data[LAST_RI][FIRST_CI] <= downmost_wdata[FIRST_RI];
  else if(leftmost_wenable[LAST_RI])
    data[LAST_RI][FIRST_CI] <= leftmost_wdata[LAST_RI];
  else if(shift_up[LAST_RI][FIRST_CI])
    data[LAST_RI][FIRST_CI] <= data[FIRST_RI][FIRST_CI];
  else if(shift_down[LAST_RI][FIRST_CI])
    data[LAST_RI][FIRST_CI] <= data[LAST_RI-1][FIRST_CI];
  else if(shift_left[LAST_RI][FIRST_CI])
    data[LAST_RI][FIRST_CI] <= data[LAST_RI][FIRST_CI+1];
  else if(shift_right[LAST_RI][FIRST_CI])
    data[LAST_RI][FIRST_CI] <= data[LAST_RI][LAST_CI];
  else if(transpose[LAST_RI][FIRST_CI])
    data[LAST_RI][FIRST_CI] <= square_data[FIRST_RI][LAST_CI];
end

// downmost and rightmost
always@(posedge clk, negedge rstnn)
begin
  if(rstnn==0)
    data[LAST_RI][LAST_CI] <= RESET_VALUE;
  else if(init[LAST_RI][LAST_CI])
    data[LAST_RI][LAST_CI] <= INIT_VALUE;
  else if(each_wenable[LAST_RI][LAST_CI])
    data[LAST_RI][LAST_CI] <= each_wdata[LAST_RI][LAST_CI];
  else if(downmost_wenable[LAST_RI])
    data[LAST_RI][LAST_CI] <= downmost_wdata[LAST_RI];
  else if(rightmost_wenable[LAST_RI])
    data[LAST_RI][LAST_CI] <= rightmost_wdata[LAST_RI];
  else if(shift_up[LAST_RI][LAST_CI])
    data[LAST_RI][LAST_CI] <= data[FIRST_RI][LAST_CI];
  else if(shift_down[LAST_RI][LAST_CI])
    data[LAST_RI][LAST_CI] <= data[LAST_RI-1][LAST_CI];
  else if(shift_left[LAST_RI][LAST_CI])
    data[LAST_RI][LAST_CI] <= data[LAST_RI][FIRST_CI];
  else if(shift_right[LAST_RI][LAST_CI])
    data[LAST_RI][LAST_CI] <= data[LAST_RI][LAST_CI-1];
end

endmodule
