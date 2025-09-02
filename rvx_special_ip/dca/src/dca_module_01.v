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
`include "ervp_axi_define.vh"
`include "dca_matrix_lsu_inst.vh"




module DCA_MODULE_01
(
  dca_port_12,
  dca_port_00,
  dca_port_21,
  dca_port_19,
  dca_port_06,

  dca_port_07,

  dca_port_09,
  dca_port_24,
  dca_port_17,

  dca_port_10,
  dca_port_03,
  dca_port_23,

  dca_port_14,
  dca_port_01,
  dca_port_02,

  dca_port_08,
  dca_port_20,
  dca_port_22,

  dca_port_11,
  dca_port_26,
  dca_port_16,

  dca_port_18,
  dca_port_04,
  dca_port_15,

  dca_port_13,
  dca_port_25,
  dca_port_05
);




parameter LSU_PARA = 0;
parameter AXI_PARA = 32;
parameter MATRIX_SIZE_PARA = 4;

`include "ervp_axi_lpara.vb"
`include "dca_matrix_dim_util.vb"
`include "dca_matrix_dim_lpara.vb"

`include "ervp_log_util.vf"
`include "ervp_bitwidth_util.vf"

input wire dca_port_12;
input wire dca_port_00;
input wire dca_port_21;
input wire dca_port_19;
output wire dca_port_06;

input wire dca_port_07;

localparam  DCA_LPARA_10 = `BW_DCA_MATRIX_LSU_INST;

input wire dca_port_09;
input wire [DCA_LPARA_10-1:0] dca_port_24;
output wire dca_port_17;

`include "dca_lsu_util.vb"
`include "dca_lsu_lpara.vb"

input wire dca_port_10;
output wire dca_port_03;
output wire [BW_TXN_INFO-1:0] dca_port_23;

input wire dca_port_14;
output wire dca_port_01;
output wire [BW_TXN_INFO-1:0] dca_port_02;

input wire dca_port_08;
output wire dca_port_20;
output wire [BW_TXN_INFO-1:0] dca_port_22;

input wire dca_port_11;
output wire dca_port_26;
output wire [BW_TXN_INFO-1:0] dca_port_16;

input wire dca_port_18;
output wire dca_port_04;
output wire [BW_RRESP_INFO-1:0] dca_port_15;

input wire dca_port_13;
output wire dca_port_25;
output wire [BW_WDATA_INFO-1:0] dca_port_05;

integer i;

wire [`BW_DCA_MATRIX_LSU_INST_OPCODE-1:0] dca_signal_01;
wire [`BW_DCA_MATRIX_INFO_ADDR-1:0] dca_signal_30;
wire [`BW_DCA_MATRIX_INFO_STRIDE_LS3-1:0] dca_signal_37;
wire [`BW_DCA_MATRIX_INFO_NUM_ROW_M1-1:0] dca_signal_19;
wire [`BW_DCA_MATRIX_INFO_NUM_COL_M1-1:0] dca_signal_12;
wire [`BW_DCA_MATRIX_INFO_IS_SIGNED-1:0] dca_signal_21;
wire [`BW_DCA_MATRIX_INFO_IS_FLOAT-1:0] dca_signal_06;
wire [`BW_DCA_MATRIX_INFO_ADDR_LSA_P3-1:0] dca_signal_10;

wire dca_signal_03;
wire dca_signal_08;

localparam  DCA_LPARA_12 = 2;
localparam  DCA_LPARA_06 = 0;
localparam  DCA_LPARA_05 = 1;
localparam  DCA_LPARA_14 = 2;

reg [DCA_LPARA_12-1:0] dca_signal_23;
wire dca_signal_35;
wire dca_signal_31;

reg [MATRIX_NUM_COL-1:0] dca_signal_38;

localparam  DCA_LPARA_00 = 3;
localparam  DCA_LPARA_13 = 0;
localparam  DCA_LPARA_03 = 1;
localparam  DCA_LPARA_11 = 3;
localparam  DCA_LPARA_01 = 5;
localparam  DCA_LPARA_02 = 7;

reg [DCA_LPARA_00-1:0] dca_signal_28;
wire dca_signal_16;
wire dca_signal_09;

localparam  DCA_LPARA_08 = REQUIRED_BITWIDTH_INDEX(MATRIX_NUM_ROW);

reg [DCA_LPARA_08-1:0] dca_signal_33;
wire dca_signal_02;

reg [BW_BITADDR-1:0] dca_signal_26;
reg [BW_BITADDR-1:0] dca_signal_17;
wire [BW_BITADDR-1:0] dca_signal_27;

localparam  DCA_LPARA_04 = `BW_AXI_ALEN+BW_BITADDR_OFFSET;

wire [DCA_LPARA_04-1:0] dca_signal_39;
wire [DCA_LPARA_04-1:0] dca_signal_18;
wire [DCA_LPARA_04-1:0] dca_signal_11;

reg [`BW_AXI_ALEN-1:0] dca_signal_24;
wire [`BW_AXI_ALEN-1:0] dca_signal_00;

reg dca_signal_32;
reg dca_signal_29;
reg dca_signal_22;
reg dca_signal_15;
wire dca_signal_13;

wire dca_signal_14;
wire dca_signal_04;

wire dca_signal_34;
wire dca_signal_36;
wire [BW_TXN_INFO-1:0] dca_signal_20;

localparam  DCA_LPARA_07 = DCA_LPARA_04;
localparam  DCA_LPARA_09 = `BW_DCA_MATRIX_INFO_ADDR_LSA_P3;

wire [DCA_LPARA_07-1:0] dca_signal_05;
wire [DCA_LPARA_09-1:0] dca_signal_07;
wire [DCA_LPARA_07-1:0] dca_signal_25;

assign {dca_signal_10, dca_signal_06, dca_signal_21, dca_signal_12, dca_signal_19, dca_signal_37, dca_signal_30, dca_signal_01} = dca_port_24;

assign dca_signal_03 = (dca_signal_01==`DCA_MATRIX_LSU_INST_OPCODE_READ);
assign dca_signal_08 = (dca_signal_01==`DCA_MATRIX_LSU_INST_OPCODE_WRITE);

always@(posedge dca_port_12, negedge dca_port_00)
begin
  if(dca_port_00==0)
    dca_signal_23 <= DCA_LPARA_06;
  else if(dca_port_19)
    case(dca_signal_23)
      DCA_LPARA_06:
        if(dca_port_09)
          dca_signal_23 <= DCA_LPARA_05;
      DCA_LPARA_05:
        if(dca_signal_35)
          dca_signal_23 <= DCA_LPARA_14;
      DCA_LPARA_14:
        if(dca_port_17)
          dca_signal_23 <= DCA_LPARA_06;
    endcase
end

always@(posedge dca_port_12, negedge dca_port_00)
begin
  if(dca_port_00==0)
    dca_signal_38 <= 0;
  else if(dca_port_09)
  begin
    for(i=0; i<MATRIX_NUM_COL; i=i+1)
      dca_signal_38[i] <= (i<=dca_signal_12);
  end
end

assign dca_signal_35 = dca_signal_03? dca_port_18 : dca_port_13;
assign dca_signal_31 = (dca_signal_23==DCA_LPARA_05);

assign dca_port_04 = dca_signal_03 & dca_signal_31;
assign dca_port_15 = {dca_signal_38,dca_port_24};
assign dca_port_25 = dca_signal_08 & dca_signal_31;
assign dca_port_05 = {dca_signal_38,dca_port_24};

always@(posedge dca_port_12, negedge dca_port_00)
begin
  if(dca_port_00==0)
    dca_signal_28 <= DCA_LPARA_13;
  else if(dca_port_19)
    case(dca_signal_28)
      DCA_LPARA_13:
        if(dca_port_09)
          dca_signal_28 <= DCA_LPARA_03;
      DCA_LPARA_03:
        dca_signal_28 <= DCA_LPARA_11;
      DCA_LPARA_11:
        dca_signal_28 <= DCA_LPARA_01;
      DCA_LPARA_01:
        if(dca_port_17)
          dca_signal_28 <= DCA_LPARA_13;
        else if(dca_signal_09)
          dca_signal_28 <= DCA_LPARA_02;
      DCA_LPARA_02:
        if(dca_port_17)
          dca_signal_28 <= DCA_LPARA_13;
    endcase
end

assign dca_port_17 = (dca_signal_23==DCA_LPARA_14) & (((dca_signal_28==DCA_LPARA_01) & dca_signal_09) | (dca_signal_28==DCA_LPARA_02));
assign dca_signal_16 = dca_signal_36 & dca_signal_34;
assign dca_signal_09 = dca_signal_16 & dca_signal_02;

always@(posedge dca_port_12, negedge dca_port_00)
begin
  if(dca_port_00==0)
    dca_signal_33 <= 0;
  else if(dca_port_19)
  begin
    if(dca_signal_09)
      dca_signal_33 <= 0;
    else if(dca_signal_16)
      dca_signal_33 <= dca_signal_33 + 1'b 1;
  end
end

assign dca_signal_02 = (dca_signal_33 == dca_signal_19[DCA_LPARA_08-1:0])? 1: 0;

always@(posedge dca_port_12 or negedge dca_port_00)
begin
  if(dca_port_00==0)
    dca_signal_26 <= 0;
  else if(dca_port_19)
    case(dca_signal_28)
      DCA_LPARA_11:
        dca_signal_26 <= dca_signal_17;
      DCA_LPARA_01:
        if(dca_signal_16)
          dca_signal_26 <= dca_signal_17;
    endcase
end

always@(posedge dca_port_12 or negedge dca_port_00)
begin
  if(dca_port_00==0)
    dca_signal_17 <= 0;
  else if(dca_port_19)
    case(dca_signal_28)
      DCA_LPARA_03:
        dca_signal_17 <= (dca_signal_30<<3);
      DCA_LPARA_11:
        dca_signal_17 <= dca_signal_27;
      DCA_LPARA_01:
        if(dca_signal_16)
          dca_signal_17 <= dca_signal_27;
    endcase
end
assign dca_signal_27 = dca_signal_17 + dca_signal_37;

ERVP_BARREL_SHIFTER
#(
  .BW_DATA(DCA_LPARA_07),
  .BW_SHIFT_AMOUNT(DCA_LPARA_09),
  .SIGNED_AMOUNT(0),
  .PLUS_TO_LEFT(1),
  .CIRCULAR_SHIFT(0),
  .LSB_FILL_VALUE(1)
)
i_dca_instance_0
(
  .data_input(dca_signal_05),
  .shift_amount(dca_signal_07),
  .data_output(dca_signal_25)
);

assign dca_signal_05 = $unsigned(dca_signal_12);
assign dca_signal_07 = dca_signal_10;
assign dca_signal_39 = dca_signal_25;

assign dca_signal_18 = $unsigned({dca_signal_17[BW_BITADDR_OFFSET-1:3],3'b 000});
assign dca_signal_11 = ($unsigned(dca_signal_18) + $unsigned(dca_signal_39)) >> 3;
assign dca_signal_00 = dca_signal_11 >> BW_AXI_ADDR_OFFSET;

always@(posedge dca_port_12 or negedge dca_port_00)
begin
  if(dca_port_00==0)
    dca_signal_24 <= 0;
  else if(dca_port_19)
    case(dca_signal_28)
      DCA_LPARA_11:
        dca_signal_24 <= dca_signal_00;
      DCA_LPARA_01:
        if(dca_signal_16)
          dca_signal_24 <= dca_signal_00;
    endcase
end

always@(posedge dca_port_12 or negedge dca_port_00)
begin
  if(dca_port_00==0)
    dca_signal_32 <= 0;
  else if(dca_port_19)
    case(dca_signal_28)
      DCA_LPARA_03:
        if(dca_signal_37[`BW_DCA_MATRIX_INFO_STRIDE_LS3-1:BW_BITADDR_OFFSET]==0)
          dca_signal_32 <= 1;
        else
          dca_signal_32 <= 0;
    endcase
end

always@(*)
begin
  dca_signal_29 = 0;
  if(dca_signal_32)
    if(dca_signal_17[BW_BITADDR_OFFSET]==dca_signal_26[BW_BITADDR_OFFSET])
      dca_signal_29 = 1;
end

always@(*)
begin
  dca_signal_22 = 0;
  if(dca_signal_00==dca_signal_24)
    dca_signal_22 = 1;
end

always@(posedge dca_port_12 or negedge dca_port_00)
begin
  if(dca_port_00==0)
    dca_signal_15 <= 0;
  else if(dca_port_19)
    case(dca_signal_28)
      DCA_LPARA_03:
        dca_signal_15 <= 0;
      DCA_LPARA_01:
        if(dca_signal_16)
          dca_signal_15 <= dca_signal_29 & dca_signal_22;
    endcase
end

assign dca_signal_13 = dca_port_07 & dca_signal_15;

assign dca_signal_14 = dca_signal_03 & dca_port_14;
assign dca_signal_04 = dca_signal_08 & dca_port_11;

assign dca_signal_34 = dca_signal_14 | dca_signal_04;
assign dca_signal_36 = (dca_signal_28==DCA_LPARA_01);
assign dca_signal_20 = {dca_signal_13, dca_signal_02, dca_signal_24, dca_signal_26};

assign dca_port_03 = dca_signal_14 & dca_signal_36 & (~dca_signal_13);
assign dca_port_23 = dca_signal_20;

assign dca_port_01 = dca_signal_14 & dca_signal_36;
assign dca_port_02 = dca_signal_20;

assign dca_port_20 = dca_signal_04 & dca_signal_36 & (~dca_signal_13);
assign dca_port_22 = dca_signal_20;

assign dca_port_26 = dca_signal_04 & dca_signal_36;
assign dca_port_16 = dca_signal_20;

assign dca_port_06 = (dca_signal_28!=DCA_LPARA_13);

endmodule
