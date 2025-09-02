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

`include "dca_matrix_info.vh"
`include "dca_matrix_lsu_inst.vh"
`include "dca_module_memorymap_offset.vh"




module DCA_MODULE_26
(
  dca_port_05,
  dca_port_17,
  dca_port_02,
  dca_port_03,
  dca_port_04,

  dca_port_19,
  dca_port_10,
  dca_port_09,

  dca_port_00,
  dca_port_21,
  dca_port_14,

  dca_port_13,
  dca_port_15,
  dca_port_12,
  dca_port_06,
  dca_port_18,
  dca_port_16,
  dca_port_08,
  dca_port_11,
  dca_port_22,
  dca_port_20,
  dca_port_01,
  dca_port_07
);




parameter MATRIX_SIZE_PARA = 8;

`include "dca_matrix_dim_util.vb"
`include "dca_matrix_dim_lpara.vb"
`include "dca_include_2.vh"

input wire dca_port_05;
input wire dca_port_17;
input wire dca_port_02;
input wire dca_port_03;
output wire dca_port_04;

localparam  DCA_LPARA_0 = `BW_DCA_MATRIX_MAC_INST;

input wire dca_port_19;
input wire [DCA_LPARA_0-1:0] dca_port_10;
output wire dca_port_09;
input wire dca_port_00;
input wire dca_port_21;
input wire dca_port_14;

input wire dca_port_13;
output wire dca_port_15;
output reg [BW_BLOCKED_STEP_INST-1:0] dca_port_12;

input wire dca_port_06;
output wire dca_port_18;
output wire [`BW_DCA_MATRIX_LSU_INST-1:0] dca_port_16;

input wire dca_port_08;
output wire dca_port_11;
output wire [`BW_DCA_MATRIX_LSU_INST-1:0] dca_port_22;

input wire dca_port_20;
output wire dca_port_01;
output wire [`BW_DCA_MATRIX_LSU_INST-1:0] dca_port_07;

wire [`BW_DCA_MATRIX_INFO_ALIGNED-1:0] dca_signal_41;
wire [`BW_DCA_MATRIX_INFO_ALIGNED-1:0] dca_signal_56;
wire [`BW_DCA_MATRIX_INFO_ALIGNED-1:0] dca_signal_40;
wire [`BW_DCA_MATRIX_MAC_OPCODE-1:0] dca_signal_30;

wire [`BW_DCA_MATRIX_INFO-1:0] dca_signal_63;
wire dca_signal_68;
wire dca_signal_29;
wire dca_signal_20;
reg dca_signal_67;
wire [`BW_DCA_MATRIX_INFO-1:0] dca_signal_44;
wire dca_signal_11;
wire dca_signal_09;
wire dca_signal_54;
wire dca_signal_42;
wire dca_signal_22;
wire dca_signal_33;

wire [`BW_DCA_MATRIX_INFO-1:0] dca_signal_62;
wire dca_signal_03;
wire dca_signal_17;
wire dca_signal_16;
reg dca_signal_04;
wire [`BW_DCA_MATRIX_INFO-1:0] dca_signal_00;
wire dca_signal_66;
wire dca_signal_43;
wire dca_signal_70;
wire dca_signal_50;
wire dca_signal_28;
wire dca_signal_15;

wire [`BW_DCA_MATRIX_INFO-1:0] dca_signal_51;
wire dca_signal_18;
wire dca_signal_71;
wire dca_signal_46;
reg dca_signal_25;
wire [`BW_DCA_MATRIX_INFO-1:0] dca_signal_61;
wire dca_signal_13;
wire dca_signal_05;
wire dca_signal_45;
wire dca_signal_57;
wire dca_signal_12;
wire dca_signal_60;

wire [`BW_DCA_MATRIX_LSU_INST_OPCODE-1:0] dca_signal_10;
wire [`BW_DCA_MATRIX_LSU_INST_OPCODE-1:0] dca_signal_53;
wire [`BW_DCA_MATRIX_LSU_INST_OPCODE-1:0] dca_signal_14;

localparam  DCA_LPARA_2 = 1;
localparam  DCA_LPARA_3 = 0;
localparam  DCA_LPARA_4 = 1;

reg [DCA_LPARA_2-1:0] dca_signal_59;
wire dca_signal_23;
wire dca_signal_35;
wire dca_signal_49;
wire dca_signal_19;

reg dca_signal_69;

localparam  DCA_LPARA_1 = 3;

reg [DCA_LPARA_1-1:0] dca_signal_08;
wire dca_signal_26;
wire dca_signal_52;

wire dca_signal_36;
wire dca_signal_55;
wire dca_signal_21;
wire dca_signal_37;
wire dca_signal_47;
wire dca_signal_65;

wire dca_signal_31;
wire dca_signal_02;
wire dca_signal_48;
wire dca_signal_01;
wire dca_signal_32;
wire dca_signal_58;
wire dca_signal_27;
wire dca_signal_34;
wire dca_signal_64;
wire dca_signal_07;

wire [MATRIX_NUM_ROW-1:0] dca_signal_39;
wire [MATRIX_NUM_COL-1:0] dca_signal_24;
wire [MATRIX_MAX_DIM-1:0] dca_signal_06;

wire [MATRIX_NUM_ROW-1:0] dca_signal_38;

assign {dca_signal_30,dca_signal_40,dca_signal_56,dca_signal_41} = dca_port_10;

assign dca_signal_31 = dca_signal_30[`DCA_MATRIX_MAC_OPCODE_INDEX_MULT_COND];
assign dca_signal_02 = 0;
assign dca_signal_48 = dca_signal_31 & dca_signal_11;
assign dca_signal_01 = dca_signal_31 & dca_signal_09;
assign dca_signal_32 = dca_signal_02;
assign dca_signal_58 = dca_signal_02;

assign dca_signal_27 = dca_signal_48 | dca_signal_32;
assign dca_signal_34 = dca_signal_01 | dca_signal_58;

assign dca_signal_64 = dca_signal_22 & dca_signal_12;
assign dca_signal_07 = dca_signal_33 & dca_signal_60;

DCA_MATRIX_SPLITER
#(
  .MATRIX_SIZE_PARA(MATRIX_SIZE_PARA)
)
i_dca_instance_2
(
  .clk(dca_port_05),
  .rstnn(dca_port_17),
  .clear(dca_port_02),
  .enable(dca_port_03),

  .matrix_info(dca_signal_63),
  .init(dca_signal_68),
  .is_col_first(dca_signal_29),
  .iterate(dca_signal_20),
  .go_next_base(dca_signal_67),

  .block_info(dca_signal_44),
  .is_first_x(dca_signal_11),
  .is_last_x(dca_signal_09),
  .is_first_y(dca_signal_54),
  .is_last_y(dca_signal_42),
  .is_first_element(dca_signal_22),
  .is_last_element(dca_signal_33),
  .valid_row_list(),
  .valid_col_list()
);

assign dca_signal_63 = dca_signal_41;
assign dca_signal_68 = dca_signal_23;
assign dca_signal_29 = 1;
assign dca_signal_20 = dca_signal_19 & dca_signal_55;

DCA_MATRIX_SPLITER
#(
  .MATRIX_SIZE_PARA(MATRIX_SIZE_PARA)
)
i_dca_instance_1
(
  .clk(dca_port_05),
  .rstnn(dca_port_17),
  .clear(dca_port_02),
  .enable(dca_port_03),

  .matrix_info(dca_signal_62),
  .init(dca_signal_03),
  .is_col_first(dca_signal_17),
  .iterate(dca_signal_16),
  .go_next_base(dca_signal_04),

  .block_info(dca_signal_00),
  .is_first_x(dca_signal_66),
  .is_last_x(dca_signal_43),
  .is_first_y(dca_signal_70),
  .is_last_y(dca_signal_50),
  .is_first_element(dca_signal_28),
  .is_last_element(dca_signal_15),
  .valid_row_list(dca_signal_38),
  .valid_col_list()
);

assign dca_signal_62 = dca_signal_56;
assign dca_signal_03 = dca_signal_23;
assign dca_signal_17 = ~dca_signal_31;
assign dca_signal_16 = dca_signal_19 & dca_signal_21;

DCA_MATRIX_SPLITER
#(
  .MATRIX_SIZE_PARA(MATRIX_SIZE_PARA)
)
i_dca_instance_0
(
  .clk(dca_port_05),
  .rstnn(dca_port_17),
  .clear(dca_port_02),
  .enable(dca_port_03),

  .matrix_info(dca_signal_51),
  .init(dca_signal_18),
  .is_col_first(dca_signal_71),
  .iterate(dca_signal_46),
  .go_next_base(dca_signal_25),

  .block_info(dca_signal_61),
  .is_first_x(dca_signal_13),
  .is_last_x(dca_signal_05),
  .is_first_y(dca_signal_45),
  .is_last_y(dca_signal_57),
  .is_first_element(dca_signal_12),
  .is_last_element(dca_signal_60),
  .valid_row_list(dca_signal_39),
  .valid_col_list(dca_signal_24)
);

assign dca_signal_51 = dca_signal_40;
assign dca_signal_18 = dca_signal_23;
assign dca_signal_71 = 1;
assign dca_signal_46 = dca_signal_19 & dca_signal_37;

always@(*)
begin
  dca_signal_67 = 0;
  dca_signal_04 = 0;
  dca_signal_25 = 0;
  if(dca_signal_19 && dca_signal_35)
  begin
    if(dca_signal_31)
    begin
      dca_signal_67 = dca_signal_01 & dca_signal_05;
      dca_signal_04 = dca_signal_43;
      dca_signal_25 = dca_signal_01 & dca_signal_05;
    end
    else
    begin
      dca_signal_67 = dca_signal_09;
      dca_signal_04 = dca_signal_43;
      dca_signal_25 = dca_signal_05;
    end
  end
end

assign dca_signal_36 = dca_signal_30[`DCA_MATRIX_MAC_OPCODE_INDEX_INIT_ACC] & dca_signal_27;
assign dca_signal_55 = (~dca_signal_47) & dca_signal_30[`DCA_MATRIX_MAC_OPCODE_INDEX_LSU0_REQ];
assign dca_signal_21 = (~dca_signal_47) & dca_signal_30[`DCA_MATRIX_MAC_OPCODE_INDEX_LSU1_REQ] & (dca_signal_30[`DCA_MATRIX_MAC_OPCODE_INDEX_RSRC_CONSTANT]? dca_signal_64 : 1);
assign dca_signal_37 = dca_signal_30[`DCA_MATRIX_MAC_OPCODE_INDEX_LSU2_REQ] & (~dca_signal_47) & (dca_signal_31? dca_signal_01 : 1);
assign dca_signal_47 = dca_signal_30[`DCA_MATRIX_MAC_OPCODE_INDEX_LOAD_ACC] & (~dca_signal_69);
assign dca_signal_65 = dca_signal_37 | dca_signal_47;

always@(posedge dca_port_05, negedge dca_port_17)
begin
  if(dca_port_17==0)
    dca_signal_59 <= DCA_LPARA_3;
  else if(dca_port_03)
    case(dca_signal_59)
      DCA_LPARA_3:
        if(dca_signal_23)
          dca_signal_59 <= DCA_LPARA_4;
      DCA_LPARA_4:
        if(dca_signal_49)
          dca_signal_59 <= DCA_LPARA_3;
    endcase
end

assign dca_signal_23 = (dca_signal_59==DCA_LPARA_3) & dca_port_19;
assign dca_signal_19 = dca_port_13 & dca_port_15;
assign dca_signal_35 = dca_signal_30[`DCA_MATRIX_MAC_OPCODE_INDEX_LOAD_ACC]? dca_signal_69 : 1;
assign dca_signal_49 = dca_signal_19 & dca_signal_07 & dca_signal_35;

always@(posedge dca_port_05, negedge dca_port_17)
begin
  if(dca_port_17==0)
    dca_signal_69 <= 0;
  else if(dca_signal_19)
  begin
    if(dca_signal_69 && dca_signal_34)
      dca_signal_69 <= 0;
    else if(dca_signal_47)
      dca_signal_69 <= 1;
  end
end

always@(posedge dca_port_05, negedge dca_port_17)
begin
  if(dca_port_17==0)
    dca_signal_08 <= 1;
  else if(dca_signal_52)
  begin
    if(dca_signal_19 & dca_signal_26)
      ;
    else
      dca_signal_08 <= dca_signal_08>>1;
  end
  else if(dca_signal_19 & dca_signal_26)
    dca_signal_08 <= dca_signal_08<<1;
end

assign dca_signal_26 = dca_signal_55 | dca_signal_21 | dca_signal_47;
assign dca_signal_52 = dca_port_00 | dca_port_21 | dca_port_14;

assign dca_port_09 = dca_signal_49;

assign dca_port_15 = (dca_signal_59==DCA_LPARA_4) & dca_port_13 & (dca_signal_55? dca_port_06 : 1) & (dca_signal_21? dca_port_08 : 1) & (dca_signal_65? dca_port_20 : 1) & ((dca_signal_26)? (~dca_signal_08[DCA_LPARA_1-1]) : 1);

assign dca_signal_06 = dca_signal_38;

always@(*)
begin
  dca_port_12 = {dca_signal_06, dca_signal_39, dca_signal_24, dca_signal_07, dca_signal_30};
  if(dca_signal_47)
    dca_port_12[`BW_DCA_MATRIX_MAC_OPCODE-1:0] = `DCA_MATRIX_MAC_OPCODE_NO_CAL;
  dca_port_12[`DCA_MATRIX_MAC_OPCODE_INDEX_INIT_ACC] = dca_signal_36;
  dca_port_12[`DCA_MATRIX_MAC_OPCODE_INDEX_LSU0_REQ] = dca_signal_55;
  dca_port_12[`DCA_MATRIX_MAC_OPCODE_INDEX_LSU1_REQ] = dca_signal_21;
  dca_port_12[`DCA_MATRIX_MAC_OPCODE_INDEX_LSU2_REQ] = dca_signal_37;
  dca_port_12[`DCA_MATRIX_MAC_OPCODE_INDEX_LOAD_ACC] = dca_signal_47;
end

assign dca_port_18 = dca_port_15 & dca_signal_55;
assign dca_port_11 = dca_port_15 & dca_signal_21;
assign dca_port_01 = dca_port_15 & dca_signal_65;

assign dca_signal_10 = `DCA_MATRIX_LSU_INST_OPCODE_READ;
assign dca_signal_53 = `DCA_MATRIX_LSU_INST_OPCODE_READ;
assign dca_signal_14 = dca_signal_47? `DCA_MATRIX_LSU_INST_OPCODE_READ : `DCA_MATRIX_LSU_INST_OPCODE_WRITE;

assign dca_port_16 = {dca_signal_44, dca_signal_10};
assign dca_port_22 = {dca_signal_00, dca_signal_53};
assign dca_port_07 = {dca_signal_61, dca_signal_14};

assign dca_port_04 = (dca_signal_59==DCA_LPARA_4);

endmodule
