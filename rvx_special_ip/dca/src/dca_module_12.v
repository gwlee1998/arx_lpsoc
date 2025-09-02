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
`include "dca_module_memorymap_offset.vh"




module DCA_MODULE_12
(
  dca_port_17,
  dca_port_00,
  dca_port_21,
  dca_port_07,
  dca_port_15,

  dca_port_10,
  dca_port_05,
  dca_port_11,

  dca_port_20,
  dca_port_04,
  dca_port_13,

  dca_port_18,
  dca_port_16,
  dca_port_19,

  dca_port_01,
  dca_port_06,

  dca_port_22,
  dca_port_09,
  dca_port_02,
  dca_port_03,

  dca_port_12,
  dca_port_08,
  dca_port_14
);




parameter MATRIX_SIZE_PARA = 4;
parameter TENSOR_PARA = 0;

`include "dca_matrix_dim_util.vb"
`include "dca_matrix_dim_lpara.vb"
`include "dca_include_1.vh"
`include "dca_tensor_scalar_lpara.vb"
`include "dca_tensor_dim_lpara.vb"

input wire dca_port_17;
input wire dca_port_00;
input wire dca_port_21;
input wire dca_port_07;
output wire dca_port_15;

input wire dca_port_10;
input wire [BW_BLOCKED_STEP_INST-1:0] dca_port_05;
output wire dca_port_11;

input wire [BW_TENSOR_MATRIX-1:0] dca_port_20;
output wire dca_port_04;
output wire dca_port_13;

input wire [BW_TENSOR_MATRIX-1:0] dca_port_18;
output wire dca_port_16;
output wire dca_port_19;

output wire dca_port_01;
input wire [BW_TENSOR_MATRIX-1:0] dca_port_06;

input wire [BW_TENSOR_MATRIX-1:0] dca_port_22;
output wire dca_port_09;
output wire [MATRIX_NUM_ELEMENT-1:0] dca_port_02;
output reg [BW_TENSOR_MATRIX-1:0] dca_port_03;

input wire [BW_TENSOR_MATRIX-1:0] dca_port_12;
output wire [MATRIX_NUM_ELEMENT-1:0] dca_port_08;
output wire [BW_TENSOR_MATRIX-1:0] dca_port_14;

wire [`BW_DCA_NEUGEMM_OPCODE-1:0] dca_signal_73;
wire dca_signal_46;
wire [MATRIX_NUM_COL-1:0] dca_signal_70;
wire [MATRIX_NUM_ROW-1:0] dca_signal_79;
wire [MATRIX_MAX_DIM-1:0] dca_signal_56;

wire dca_signal_58;
wire dca_signal_06;
wire dca_signal_26;
wire dca_signal_61;
wire dca_signal_65;
wire dca_signal_39;
wire dca_signal_32;
wire dca_signal_51;
wire dca_signal_75;
wire dca_signal_04;
wire dca_signal_41;

localparam  DCA_LPARA_00 = 1;
localparam  DCA_LPARA_11 = 0;
localparam  DCA_LPARA_13 = 1;

reg [DCA_LPARA_00-1:0] dca_signal_30;

genvar i;
genvar i_row, i_col;

reg dca_signal_33;
reg dca_signal_24;
reg dca_signal_21;
reg dca_signal_80;
reg dca_signal_19;
wire dca_signal_15;

localparam  DCA_LPARA_03 = 1;
localparam  DCA_LPARA_19 = 3;
localparam  DCA_LPARA_20 = 2;
localparam  DCA_LPARA_17 = 3;
localparam  DCA_LPARA_18 = 3; 

reg dca_signal_63;
wire dca_signal_71;
reg dca_signal_59;
wire [DCA_LPARA_18-1:0] dca_signal_64;

reg dca_signal_07;
reg dca_signal_09;
reg [BW_TENSOR_MATRIX-1:0] dca_signal_72;
reg [BW_TENSOR_MATRIX-1:0] dca_signal_76;
wire [BW_TENSOR_MATRIX-1:0] dca_signal_17;

reg dca_signal_16;
reg dca_signal_38;
reg [BW_TENSOR_MATRIX-1:0] dca_signal_05;
reg [BW_TENSOR_MATRIX-1:0] dca_signal_22;
wire [BW_TENSOR_MATRIX-1:0] dca_signal_66;

localparam  DCA_LPARA_01 = DCA_LPARA_20;
localparam  DCA_LPARA_07 = DCA_LPARA_03;
localparam  DCA_LPARA_09 = 1;
localparam  DCA_LPARA_04 = MATRIX_MAX_DIM;

wire dca_signal_78;
wire [DCA_LPARA_09-1:0] dca_signal_36;
wire [DCA_LPARA_04-1:0] dca_signal_29;
wire dca_signal_34;
wire dca_signal_35;
wire dca_signal_60;
wire dca_signal_47;
wire dca_signal_55;
wire dca_signal_57;
wire dca_signal_69;
wire dca_signal_01;

localparam  DCA_LPARA_22 = DCA_LPARA_17;
localparam  DCA_LPARA_06 = DCA_LPARA_19;
localparam  DCA_LPARA_23 = 1;
localparam  DCA_LPARA_21 = MATRIX_MAX_DIM;

wire dca_signal_50;
wire [DCA_LPARA_23-1:0] dca_signal_67;
wire [DCA_LPARA_21-1:0] dca_signal_18;
wire dca_signal_12;
wire dca_signal_10;
wire dca_signal_44;
wire dca_signal_54;
wire dca_signal_42;
wire dca_signal_48;
wire dca_signal_02;
wire dca_signal_25;

localparam  DCA_LPARA_15 = DCA_LPARA_20;
localparam  DCA_LPARA_02 = DCA_LPARA_03;
localparam  DCA_LPARA_14 = MATRIX_NUM_ROW;
localparam  DCA_LPARA_10 = MATRIX_NUM_COL;

wire dca_signal_13;
wire [DCA_LPARA_14-1:0] dca_signal_23;
wire [DCA_LPARA_10-1:0] dca_signal_08;
wire dca_signal_31;
wire dca_signal_43;
wire dca_signal_00;
wire dca_signal_74;
wire dca_signal_77;
wire dca_signal_40;
wire dca_signal_28;
wire dca_signal_45;

localparam  DCA_LPARA_12 = DCA_LPARA_17;
localparam  DCA_LPARA_16 = DCA_LPARA_19;
localparam  DCA_LPARA_05 = MATRIX_NUM_ROW;
localparam  DCA_LPARA_08 = MATRIX_NUM_COL;

wire dca_signal_62;
wire [DCA_LPARA_05-1:0] dca_signal_03;
wire [DCA_LPARA_08-1:0] dca_signal_11;
wire dca_signal_20;
wire dca_signal_27;
wire dca_signal_14;
wire dca_signal_53;
wire dca_signal_52;
wire dca_signal_37;
wire dca_signal_49;
wire dca_signal_68;

assign {dca_signal_56, dca_signal_79, dca_signal_70, dca_signal_46, dca_signal_73} = dca_port_05;

assign dca_signal_58 = dca_signal_73[`DCA_NEUGEMM_OPCODE_INDEX_ADDSUB];
assign dca_signal_06 = dca_signal_73[`DCA_NEUGEMM_OPCODE_INDEX_RSRC_INV];
assign dca_signal_26 = dca_signal_73[`DCA_NEUGEMM_OPCODE_INDEX_EWMULT];
assign dca_signal_61 = dca_signal_73[`DCA_NEUGEMM_OPCODE_INDEX_MULT_COND];
assign dca_signal_65 = dca_signal_73[`DCA_NEUGEMM_OPCODE_INDEX_CONV_COND];
assign dca_signal_39 = dca_signal_73[`DCA_NEUGEMM_OPCODE_INDEX_INIT_ACC];
assign dca_signal_32 = dca_signal_73[`DCA_NEUGEMM_OPCODE_INDEX_IS_FLOAT];
assign dca_signal_51 = dca_signal_73[`DCA_NEUGEMM_OPCODE_INDEX_LSU0_REQ];
assign dca_signal_75 = dca_signal_73[`DCA_NEUGEMM_OPCODE_INDEX_LSU1_REQ];
assign dca_signal_04 = dca_signal_73[`DCA_NEUGEMM_OPCODE_INDEX_LSU2_REQ];
assign dca_signal_41 = dca_signal_73[`DCA_NEUGEMM_OPCODE_INDEX_RSRC_CONSTANT];

always@(posedge dca_port_17, negedge dca_port_00)
begin
  if(dca_port_00==0)
    dca_signal_30 <= DCA_LPARA_11;
  else if(dca_port_21)
    dca_signal_30 <= DCA_LPARA_11;
  else
    case(dca_signal_30)
      DCA_LPARA_11:
        if(dca_port_10)
          dca_signal_30 <= DCA_LPARA_13;
      DCA_LPARA_13:
        if(dca_port_11)
          dca_signal_30 <= DCA_LPARA_11;
    endcase
end

assign dca_port_09 = dca_port_10 & dca_signal_39;

assign dca_port_11 = dca_signal_33 | dca_signal_24 | dca_signal_21 | dca_signal_80 | dca_signal_19;

ERVP_COUNTER
#(
  .BW_COUNTER(DCA_LPARA_18)
)
i_dca_instance_6
(
  .clk(dca_port_17),
  .rstnn(dca_port_00),
  .enable(dca_signal_63),
  .init(dca_signal_71),
  .count(dca_signal_59),
  .value(dca_signal_64),
  .is_first_count(),
  .is_last_count()
);

assign dca_signal_71 = dca_port_11;

always@(*)
begin
  dca_signal_63 = 0;
  dca_signal_59 = 0;
  if(dca_signal_30==DCA_LPARA_13)
    if(dca_signal_58 || dca_signal_26)
    begin
      dca_signal_63 = 1;
      dca_signal_59 = 1;
    end
end

always@(*)
begin
  dca_signal_33 = 0;
  dca_signal_24 = 0;
  dca_signal_21 = 0;
  dca_signal_80 = 0;
  dca_signal_19 = 0;
  if(dca_signal_30==DCA_LPARA_13)
  begin
    if(dca_signal_58)
    begin
      if(dca_signal_32)
      begin
        if(dca_signal_64==(DCA_LPARA_19-1))
          dca_signal_33 = 1;
      end
      else
      begin
        if(dca_signal_64==(DCA_LPARA_03-1))
          dca_signal_33 = 1;
      end
    end
    else if(dca_signal_26)
    begin
      if(dca_signal_32)
      begin
        if(dca_signal_64==(DCA_LPARA_17-1))
          dca_signal_24 = 1;
      end
      else
      begin
        if(dca_signal_64==(DCA_LPARA_20-1))
          dca_signal_24 = 1;
      end
    end
    else if(dca_signal_61)
    begin
      if(dca_signal_32)
        dca_signal_21 = dca_signal_10;
      else
        dca_signal_21 = dca_signal_35;
    end
    else if(dca_signal_65)
    begin
      if(dca_signal_32)
        dca_signal_80 = dca_signal_27;
      else
        dca_signal_80 = dca_signal_43;
    end
    else
      dca_signal_19 = dca_signal_39;
  end
end

assign dca_signal_15 = dca_signal_33 | dca_signal_24;

generate
for(i=0; i<MATRIX_NUM_ELEMENT; i=i+1)
begin : i_generate_adder
  DCA_MODULE_24
  #(
    .TENSOR_PARA(TENSOR_PARA)
  )
  i_dca_instance_1
  (
    .dca_port_00(dca_port_17),
    .dca_port_01(dca_port_00),
    .dca_port_06(dca_signal_07),
    .dca_port_09(dca_signal_06),
    .dca_port_05(dca_signal_32),
    .dca_port_02(dca_signal_09),
    .dca_port_08(dca_signal_72[BW_TENSOR_SCALAR*(i+1)-1-:BW_TENSOR_SCALAR]),
    .dca_port_03(dca_signal_76[BW_TENSOR_SCALAR*(i+1)-1-:BW_TENSOR_SCALAR]),
    .dca_port_04(),
    .dca_port_07(dca_signal_17[BW_TENSOR_SCALAR*(i+1)-1-:BW_TENSOR_SCALAR])
  );
end
endgenerate

always@(*)
begin
  dca_signal_07 = 0;
  if(dca_signal_30==DCA_LPARA_13)
    if(dca_signal_58 || dca_signal_61 || dca_signal_65)
      dca_signal_07 = 1;
end

always@(*)
begin
  dca_signal_09 = 0;
  if(dca_signal_30==DCA_LPARA_13)
  begin
    if(dca_signal_58)
      dca_signal_09 = 1;
    else if(dca_signal_61)
    begin
      if(dca_signal_32)
        dca_signal_09 = dca_signal_54;
      else
        dca_signal_09 = dca_signal_47;
    end
    else if(dca_signal_65)
    begin
      if(dca_signal_32)
        dca_signal_09 = dca_signal_53;
      else
        dca_signal_09 = dca_signal_74;
    end
  end
end

always@(*)
begin
  dca_signal_72 = dca_port_20;
  dca_signal_76 = dca_port_18;
  if(dca_signal_58 || dca_signal_26)
  begin
    dca_signal_72 = dca_port_20;
    dca_signal_76 = dca_port_18;
  end
  else if(dca_signal_61 || dca_signal_65)
  begin
    dca_signal_72 = dca_port_12;
    dca_signal_76 = dca_port_22;
  end
end

generate
for(i=0; i<MATRIX_NUM_ELEMENT; i=i+1)
begin : i_generate_multiplier
  DCA_MODULE_21
  #(
    .TENSOR_PARA(TENSOR_PARA)
  )
  i_dca_instance_2
  (
    .dca_port_3(dca_port_17),
    .dca_port_0(dca_port_00),
    .dca_port_4(dca_signal_16),
    .dca_port_7(dca_signal_32),
    .dca_port_5(dca_signal_38),
    .dca_port_6(dca_signal_05[BW_TENSOR_SCALAR*(i+1)-1-:BW_TENSOR_SCALAR]),
    .dca_port_1(dca_signal_22[BW_TENSOR_SCALAR*(i+1)-1-:BW_TENSOR_SCALAR]),
    .dca_port_8(),
    .dca_port_2(dca_signal_66[BW_TENSOR_SCALAR*(i+1)-1-:BW_TENSOR_SCALAR])
  );
end
endgenerate

always@(*)
begin
  dca_signal_16 = 0;
  if(dca_signal_30==DCA_LPARA_13)
    if(dca_signal_26 || dca_signal_61 || dca_signal_65)
      dca_signal_16 = 1;
end

always@(*)
begin
  dca_signal_38 = 0;
  if(dca_signal_30==DCA_LPARA_13)
  begin
    if(dca_signal_26)
      dca_signal_38 = 1;
    else if(dca_signal_61)
    begin
      if(dca_signal_32)
        dca_signal_38 = dca_signal_44;
      else
        dca_signal_38 = dca_signal_60;
    end
    else if(dca_signal_65)
    begin
      if(dca_signal_32)
        dca_signal_38 = dca_signal_14;
      else
        dca_signal_38 = dca_signal_00;
    end
  end
end

function integer GET_MATRIX_INDEX;
  input integer row_index;
  input integer col_index;
begin
  GET_MATRIX_INDEX = (row_index*BW_TENSOR_ROW) + (col_index*BW_TENSOR_SCALAR) + BW_TENSOR_SCALAR - 1;
end
endfunction

integer n, m;

always@(*)
begin
  for(m=0; m<MATRIX_NUM_ROW; m=m+1)
  begin
    for(n=0; n<MATRIX_NUM_COL; n=n+1)
    begin
      dca_signal_05[GET_MATRIX_INDEX(m,n)-:BW_TENSOR_SCALAR] = dca_port_20[GET_MATRIX_INDEX(m,n)-:BW_TENSOR_SCALAR];
      dca_signal_22[GET_MATRIX_INDEX(m,n)-:BW_TENSOR_SCALAR] = dca_port_18[GET_MATRIX_INDEX(m,n)-:BW_TENSOR_SCALAR];
      if(dca_signal_26 || dca_signal_65)
      begin
        dca_signal_05[GET_MATRIX_INDEX(m,n)-:BW_TENSOR_SCALAR] = dca_port_20[GET_MATRIX_INDEX(m,n)-:BW_TENSOR_SCALAR];
        dca_signal_22[GET_MATRIX_INDEX(m,n)-:BW_TENSOR_SCALAR] = dca_port_18[GET_MATRIX_INDEX(m,n)-:BW_TENSOR_SCALAR];
      end
      if(dca_signal_61)
      begin
        dca_signal_05[GET_MATRIX_INDEX(m,n)-:BW_TENSOR_SCALAR] = dca_port_20[GET_MATRIX_INDEX(m,0)-:BW_TENSOR_SCALAR];
        dca_signal_22[GET_MATRIX_INDEX(m,n)-:BW_TENSOR_SCALAR] = dca_port_18[GET_MATRIX_INDEX(0,n)-:BW_TENSOR_SCALAR];
      end
      if(dca_signal_41)
        dca_signal_22[GET_MATRIX_INDEX(m,n)-:BW_TENSOR_SCALAR] = dca_port_06[GET_MATRIX_INDEX(0,n)-:BW_TENSOR_SCALAR];
    end
  end
end

DCA_MODULE_22
#(
  .DCA_GPARA_2(DCA_LPARA_01),
  .DCA_GPARA_3(DCA_LPARA_07),
  .DCA_GPARA_0(DCA_LPARA_09),
  .DCA_GPARA_1(DCA_LPARA_04)
)
i_dca_instance_4
(
  .dca_port_07(dca_port_17),
  .dca_port_06(dca_port_00),
  .dca_port_03(dca_signal_78),

  .dca_port_10(dca_signal_36),
  .dca_port_04(dca_signal_29),

  .dca_port_02(dca_signal_34),
  .dca_port_12(dca_signal_35),
  
  .dca_port_01(dca_signal_60),
  .dca_port_08(dca_signal_47),

  .dca_port_11(dca_signal_55),
  .dca_port_05(dca_signal_57),
  .dca_port_00(dca_signal_69),
  .dca_port_09(dca_signal_01)
);

assign dca_signal_78 = dca_signal_61 & (~dca_signal_32);
assign dca_signal_36 = 1;
assign dca_signal_29 = dca_signal_56;
assign dca_signal_34 = dca_port_10;

DCA_MODULE_22
#(
  .DCA_GPARA_2(DCA_LPARA_22),
  .DCA_GPARA_3(DCA_LPARA_06),
  .DCA_GPARA_0(DCA_LPARA_23),
  .DCA_GPARA_1(DCA_LPARA_21)
)
i_dca_instance_5
(
  .dca_port_07(dca_port_17),
  .dca_port_06(dca_port_00),
  .dca_port_03(dca_signal_50),

  .dca_port_10(dca_signal_67),
  .dca_port_04(dca_signal_18),

  .dca_port_02(dca_signal_12),
  .dca_port_12(dca_signal_10),
  
  .dca_port_01(dca_signal_44),
  .dca_port_08(dca_signal_54),

  .dca_port_11(dca_signal_42),
  .dca_port_05(dca_signal_48),
  .dca_port_00(dca_signal_02),
  .dca_port_09(dca_signal_25)
);

assign dca_signal_50 = 0; 
assign dca_signal_67 = 1;
assign dca_signal_18 = dca_signal_56;
assign dca_signal_12 = dca_port_10;

DCA_MODULE_22
#(
  .DCA_GPARA_2(DCA_LPARA_15),
  .DCA_GPARA_3(DCA_LPARA_02),
  .DCA_GPARA_0(DCA_LPARA_14),
  .DCA_GPARA_1(DCA_LPARA_10)
)
i_dca_instance_0
(
  .dca_port_07(dca_port_17),
  .dca_port_06(dca_port_00),
  .dca_port_03(dca_signal_13),

  .dca_port_10(dca_signal_23),
  .dca_port_04(dca_signal_08),

  .dca_port_02(dca_signal_31),
  .dca_port_12(dca_signal_43),
  
  .dca_port_01(dca_signal_00),
  .dca_port_08(dca_signal_74),

  .dca_port_11(dca_signal_77),
  .dca_port_05(dca_signal_40),
  .dca_port_00(dca_signal_28),
  .dca_port_09(dca_signal_45)
);

assign dca_signal_13 = dca_signal_65 & (~dca_signal_32);
assign dca_signal_23 = dca_signal_56;
assign dca_signal_08 = dca_signal_56;
assign dca_signal_31 = dca_port_10;

DCA_MODULE_22
#(
  .DCA_GPARA_2(DCA_LPARA_12),
  .DCA_GPARA_3(DCA_LPARA_16),
  .DCA_GPARA_0(DCA_LPARA_05),
  .DCA_GPARA_1(DCA_LPARA_08)
)
i_dca_instance_3
(
  .dca_port_07(dca_port_17),
  .dca_port_06(dca_port_00),
  .dca_port_03(dca_signal_62),

  .dca_port_10(dca_signal_03),
  .dca_port_04(dca_signal_11),

  .dca_port_02(dca_signal_20),
  .dca_port_12(dca_signal_27),
  
  .dca_port_01(dca_signal_14),
  .dca_port_08(dca_signal_53),

  .dca_port_11(dca_signal_52),
  .dca_port_05(dca_signal_37),
  .dca_port_00(dca_signal_49),
  .dca_port_09(dca_signal_68)
);

assign dca_signal_62 = 0; 
assign dca_signal_03 = dca_signal_56;
assign dca_signal_11 = dca_signal_56;
assign dca_signal_20 = dca_port_10;

assign dca_port_04 = dca_signal_77 | dca_signal_52;
assign dca_port_13 = dca_signal_57 | dca_signal_57 | dca_signal_40 | dca_signal_37;

assign dca_port_16 = dca_signal_57 | dca_signal_57 | dca_signal_77 | dca_signal_52;
assign dca_port_19 = dca_signal_40 | dca_signal_37;

assign dca_port_01 = (dca_port_10 & dca_signal_41) | dca_signal_65;

generate
  for(i_row=0; i_row<MATRIX_NUM_ROW; i_row=i_row+1)
  begin : i_gen_all_wenable_list2d_row
    for(i_col=0; i_col<MATRIX_NUM_COL; i_col=i_col+1)
    begin : i_gen_all_wenable_list2d_col
      assign dca_port_08[i_row*MATRIX_NUM_COL + i_col] = (dca_signal_69 | dca_signal_02 | dca_signal_28 | dca_signal_49) & dca_signal_70[i_col] & dca_signal_79[i_row];
      assign dca_port_02[i_row*MATRIX_NUM_COL + i_col] = (dca_signal_15 | dca_signal_01 | dca_signal_25 | dca_signal_45 | dca_signal_68) & dca_signal_70[i_col] & dca_signal_79[i_row];
    end
  end
endgenerate

assign dca_port_14 = dca_signal_66;

always@(*)
begin
  dca_port_03 = dca_signal_17;
  if(dca_signal_58)
    dca_port_03 = dca_signal_17;
  else if(dca_signal_61 || dca_signal_65)
    dca_port_03 = dca_signal_17;
  else if(dca_signal_26)
    dca_port_03 = dca_signal_66;
end

assign dca_port_15 = (dca_signal_30!=DCA_LPARA_11);

`ifdef SIM_ENV
`ifdef PRINT_DCA_MATRIX_CAL

always@(posedge dca_port_17)
begin
  if(dca_signal_30==DCA_LPARA_13)
    $display("\n\n");
end

always@(negedge dca_port_17)
begin
  if(dca_port_10)
  begin
    $write("\n[CAL_START]");
    for(m=0; m<MATRIX_NUM_ROW; m=m+1)
    begin
      $write("\n");
      for(n=0; n<MATRIX_NUM_COL; n=n+1)
      begin
        $write(" %d", $signed(dca_port_20[GET_MATRIX_INDEX(m,n)-:BW_TENSOR_SCALAR]));
      end
    end
    $write("\n");
    for(m=0; m<MATRIX_NUM_ROW; m=m+1)
    begin
      $write("\n");
      for(n=0; n<MATRIX_NUM_COL; n=n+1)
      begin
        $write(" %d", $signed(dca_port_18[GET_MATRIX_INDEX(m,n)-:BW_TENSOR_SCALAR]));
      end
    end
  end
end

always@(negedge dca_port_17)
begin
  if(dca_signal_69 | dca_signal_02 | dca_signal_28 | dca_signal_49 | dca_signal_01 | dca_signal_25 | dca_signal_45 | dca_signal_68)
  begin
    $write("\n[CAL_CONSTANT] %d", $signed(dca_port_06[GET_MATRIX_INDEX(0,0)-:BW_TENSOR_SCALAR]));
  end
end

always@(negedge dca_port_17)
begin
  if(dca_signal_69 | dca_signal_02 | dca_signal_28 | dca_signal_49)
  begin
    $write("\n[CAL_TEMP]");
    for(m=0; m<MATRIX_NUM_ROW; m=m+1)
    begin
      $write("\n");
      for(n=0; n<MATRIX_NUM_COL; n=n+1)
      begin
        $write(" %d", $signed(dca_port_14[GET_MATRIX_INDEX(m,n)-:BW_TENSOR_SCALAR]));
      end
    end
  end
end

always@(negedge dca_port_17)
begin
  if(dca_signal_01 | dca_signal_25 | dca_signal_45 | dca_signal_68)
  begin
    $write("\n[CAL_OUTPUT]");
    for(m=0; m<MATRIX_NUM_ROW; m=m+1)
    begin
      $write("\n");
      for(n=0; n<MATRIX_NUM_COL; n=n+1)
      begin
        $write(" %d", $signed(dca_port_03[GET_MATRIX_INDEX(m,n)-:BW_TENSOR_SCALAR]));
      end
    end
  end
end

`endif
`endif

endmodule
