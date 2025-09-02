// ****************************************************************************
// ****************************************************************************
// Copyright SoC Design Research Group, All rights reservxd.
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
// 2025-08-12
// Kyuseung Han (han@etri.re.kr)
// ****************************************************************************
// ****************************************************************************

`include "ervp_global.vh"


module DCA_MATRIX_ITERATOR
(
  clk,
  rstnn,
  clear,
  enable,

  is_col_first,
  num_row_m1,
  num_col_m1,
  iterate,
  go_next_base,

  is_first_row,
  is_last_row,
  is_first_col,
  is_last_col,
  is_first_element,
  is_last_element,

  is_first_x,
  is_last_x,
  is_first_y,
  is_last_y
);


parameter BW_NUM_ROW = 1;
parameter BW_NUM_COL = 1;

input wire clk;
input wire rstnn;
input wire clear;
input wire enable;

input wire is_col_first;
input wire [BW_NUM_ROW-1:0] num_row_m1;
input wire [BW_NUM_COL-1:0] num_col_m1;
input wire iterate;
input wire go_next_base;

output wire is_first_row;
output wire is_last_row;

output wire is_first_col;
output wire is_last_col;

output wire is_first_element;
output wire is_last_element;

output wire is_first_x;
output wire is_last_x;

output wire is_first_y;
output wire is_last_y;

reg [BW_NUM_ROW-1:0] dca_signal_1;
reg [BW_NUM_COL-1:0] dca_signal_0;

always@(posedge clk, negedge rstnn)
begin
  if(rstnn==0)
    dca_signal_0 <= 0;
  else if(clear)
    dca_signal_0 <= 0;
  else if(enable)
  begin
    if(is_col_first)
    begin
      if(go_next_base)
        dca_signal_0 <= 0;
      else if(iterate)
      begin
        if(is_last_col)
          dca_signal_0 <= 0;
        else
          dca_signal_0 <= dca_signal_0 + 1'b 1;
      end
    end
    else if(go_next_base)
    begin
      if(is_last_col)
        dca_signal_0 <= 0;
      else
        dca_signal_0 <= dca_signal_0 + 1'b 1;
    end
  end
end

assign is_first_col = (dca_signal_0==0);
assign is_last_col = (dca_signal_0==num_col_m1);

always@(posedge clk, negedge rstnn)
begin
  if(rstnn==0)
    dca_signal_1 <= 0;
  else if(clear)
    dca_signal_1 <= 0;
  else if(enable)
  begin
    if(~is_col_first)
    begin
      if(go_next_base)
        dca_signal_1 <= 0;
      else if(iterate)
      begin
        if(is_last_row)
          dca_signal_1 <= 0;
        else
          dca_signal_1 <= dca_signal_1 + 1'b 1;
      end
    end
    else if(go_next_base)
    begin
      if(is_last_row)
        dca_signal_1 <= 0;
      else
        dca_signal_1 <= dca_signal_1 + 1'b 1;
    end
  end
end

assign is_first_row = (dca_signal_1==0);
assign is_last_row = (dca_signal_1==num_row_m1);

assign is_first_element = is_first_col & is_first_row;
assign is_last_element = is_last_col & is_last_row;

assign is_first_x = is_col_first? is_first_col : is_first_row;
assign is_last_x = is_col_first? is_last_col : is_last_row;

assign is_first_y = is_col_first? is_first_row : is_first_col;
assign is_last_y = is_col_first? is_last_row : is_last_col;

endmodule
