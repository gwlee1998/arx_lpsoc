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




module DCA_MODULE_14
(
  dca_port_12,
  dca_port_13,
  dca_port_05,
  dca_port_08,
  dca_port_14,

  dca_port_01,
  dca_port_16,
  dca_port_10,
  dca_port_03,

  dca_port_11,
  dca_port_15,
  dca_port_04,
  dca_port_09,
  dca_port_06,
  dca_port_17,
  dca_port_02,
  dca_port_07,
  dca_port_00
);




parameter MATRIX_SIZE_PARA = 8;

`include "dca_matrix_dim_util.vb"
`include "dca_matrix_dim_lpara.vb"
`include "dca_include_0.vh"

input wire dca_port_12;
input wire dca_port_13;
input wire dca_port_05;
input wire dca_port_08;
output wire dca_port_14;

localparam  DCA_LPARA_3 = `BW_DCA_MRU_INST;

input wire dca_port_01;
input wire [DCA_LPARA_3-1:0] dca_port_16;
output wire dca_port_10;
input wire dca_port_03;

input wire dca_port_11;
output wire dca_port_15;
output wire [BW_BLOCKED_STEP_INST-1:0] dca_port_04;

input wire dca_port_09;
output wire dca_port_06;
output wire [`BW_DCA_MATRIX_LSU_INST-1:0] dca_port_17;

input wire dca_port_02;
output wire dca_port_07;
output wire [`BW_DCA_MATRIX_LSU_INST-1:0] dca_port_00;

wire [`BW_DCA_MATRIX_INFO_ALIGNED-1:0] dca_signal_32;
wire [`BW_DCA_MATRIX_INFO_ALIGNED-1:0] dca_signal_37;
wire [`BW_DCA_MRU_OPCODE-1:0] dca_signal_33;

wire dca_signal_30;
wire dca_signal_41;
wire dca_signal_35;

wire dca_signal_11;
wire [`BW_DCA_MATRIX_INFO-1:0] dca_signal_13;
wire dca_signal_23;
wire dca_signal_38;
wire dca_signal_00;
wire dca_signal_15;
wire [`BW_DCA_MATRIX_INFO-1:0] dca_signal_14;
wire dca_signal_36;
wire dca_signal_18;
wire dca_signal_28;
wire dca_signal_27;
wire dca_signal_22;
wire dca_signal_03;

wire dca_signal_39;
wire [`BW_DCA_MATRIX_INFO-1:0] dca_signal_01;
wire dca_signal_08;
wire dca_signal_12;
wire dca_signal_16;
wire dca_signal_31;
wire [`BW_DCA_MATRIX_INFO-1:0] dca_signal_07;
wire dca_signal_21;
wire dca_signal_29;
wire dca_signal_25;
wire dca_signal_04;
wire dca_signal_24;
wire dca_signal_40;

wire dca_signal_19;

wire [`BW_DCA_MATRIX_LSU_INST_OPCODE-1:0] dca_signal_06;
wire [`BW_DCA_MATRIX_LSU_INST_OPCODE-1:0] dca_signal_05;

localparam  DCA_LPARA_4 = 1;
localparam  DCA_LPARA_2 = 0;
localparam  DCA_LPARA_0 = 1;

reg [DCA_LPARA_4-1:0] dca_signal_10;
wire dca_signal_17;
wire dca_signal_02;
wire dca_signal_34;

localparam  DCA_LPARA_1 = 3;

reg [DCA_LPARA_1-1:0] dca_signal_09;
wire dca_signal_26;
wire dca_signal_20;

assign {dca_signal_33, dca_signal_37, dca_signal_32} = dca_port_16;

assign dca_signal_30 = dca_signal_33[`DCA_MRU_OPCODE_INDEX_TRANSPOSE];
assign dca_signal_41 = dca_signal_33[`DCA_MRU_OPCODE_INDEX_LSU0_REQ];
assign dca_signal_35 = 1;

DCA_MATRIX_SPLITER
#(
  .MATRIX_SIZE_PARA(MATRIX_SIZE_PARA)
)
i_dca_instance_1
(
  .clk(dca_port_12),
  .rstnn(dca_port_13),
  .clear(dca_port_05),
  .enable(dca_signal_11),

  .matrix_info(dca_signal_13),
  .init(dca_signal_23),
  .is_col_first(dca_signal_38),
  .iterate(dca_signal_00),
  .go_next_base(dca_signal_15),

  .block_info(dca_signal_14),
  .is_first_x(dca_signal_36),
  .is_last_x(dca_signal_18),
  .is_first_y(dca_signal_28),
  .is_last_y(dca_signal_27),
  .is_first_element(dca_signal_22),
  .is_last_element(dca_signal_03),
  .valid_row_list(),
  .valid_col_list()
);

assign dca_signal_11 = dca_port_08 & dca_signal_41;
assign dca_signal_13 = dca_signal_32;
assign dca_signal_23 = dca_signal_17;
assign dca_signal_38 = 1;
assign dca_signal_00 = dca_signal_34;
assign dca_signal_15 = dca_signal_34 & dca_signal_18;

DCA_MATRIX_SPLITER
#(
  .MATRIX_SIZE_PARA(MATRIX_SIZE_PARA)
)
i_dca_instance_0
(
  .clk(dca_port_12),
  .rstnn(dca_port_13),
  .clear(dca_port_05),
  .enable(dca_signal_39),

  .matrix_info(dca_signal_01),
  .init(dca_signal_08),
  .is_col_first(dca_signal_12),
  .iterate(dca_signal_16),
  .go_next_base(dca_signal_31),

  .block_info(dca_signal_07),
  .is_first_x(dca_signal_21),
  .is_last_x(dca_signal_29),
  .is_first_y(dca_signal_25),
  .is_last_y(dca_signal_04),
  .is_first_element(dca_signal_24),
  .is_last_element(dca_signal_40),
  .valid_row_list(),
  .valid_col_list()
);

assign dca_signal_39 = dca_port_08 & dca_signal_35;
assign dca_signal_01 = dca_signal_37;
assign dca_signal_08 = dca_signal_17;
assign dca_signal_12 = ~dca_signal_30;
assign dca_signal_16 = dca_signal_34;
assign dca_signal_31 = dca_signal_34 & dca_signal_29;

assign dca_signal_19 = dca_signal_40;

always@(posedge dca_port_12, negedge dca_port_13)
begin
  if(dca_port_13==0)
    dca_signal_10 <= DCA_LPARA_2;
  else if(dca_port_08)
    case(dca_signal_10)
      DCA_LPARA_2:
        if(dca_signal_17)
          dca_signal_10 <= DCA_LPARA_0;
      DCA_LPARA_0:
        if(dca_signal_02)
          dca_signal_10 <= DCA_LPARA_2;
    endcase
end
assign dca_signal_17 = (dca_signal_10==DCA_LPARA_2) & dca_port_01;
assign dca_signal_02 = dca_signal_34 & dca_signal_19;

always@(posedge dca_port_12, negedge dca_port_13)
begin
  if(dca_port_13==0)
    dca_signal_09 <= 1;
  else if(dca_signal_20)
  begin
    if(dca_signal_34 & dca_signal_26)
      ;
    else
      dca_signal_09 <= dca_signal_09>>1;
  end
  else if(dca_signal_34 & dca_signal_26)
    dca_signal_09 <= dca_signal_09<<1;
end
assign dca_signal_26 = dca_signal_41;
assign dca_signal_20 = dca_port_03;
assign dca_port_10 = dca_signal_02;

assign dca_signal_34 = dca_port_11 & dca_port_15;
assign dca_port_15 = (dca_signal_10==DCA_LPARA_0) & dca_port_11 & (dca_signal_41? dca_port_09 : 1) & (dca_signal_35? dca_port_02 : 1) & ((dca_signal_26)? (~dca_signal_09[DCA_LPARA_1-1]) : 1);
assign dca_port_04 = {dca_signal_19, dca_signal_33};

assign dca_port_06 = dca_port_15 & dca_signal_41;
assign dca_port_07 = dca_port_15 & dca_signal_35;
assign dca_signal_06 = `DCA_MATRIX_LSU_INST_OPCODE_READ;
assign dca_signal_05 = `DCA_MATRIX_LSU_INST_OPCODE_WRITE;
assign dca_port_17 = {dca_signal_14, dca_signal_06};
assign dca_port_00 = {dca_signal_07, dca_signal_05};

assign dca_port_14 = (dca_signal_10==DCA_LPARA_0);

endmodule
