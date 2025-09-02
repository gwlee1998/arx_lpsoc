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



module DCA_MATRIX_LSU_LPI2P
(
  clk,
  rstnn,
  clear,
  enable,
  busy,

  inst_wvalid,
  inst_wdata,
  inst_wready,
  inst_is_write,
  inst_decode_finish,
  inst_execute_finish,

  load_tensor_row_wvalid,
  load_tensor_row_wlast,
  load_tensor_row_wdata,
  load_tensor_row_wready,

  store_tensor_row_rvalid,
  store_tensor_row_rlast,
  store_tensor_row_rready,
  store_tensor_row_rdata,

  read_slxqdready,
  read_slxqvalid,
  read_slxqhint,
  read_slxqlast,
  read_slxqafy,
  read_slxqdata,
  read_slxydready,
  read_slxyvalid,
  read_slxyhint,
  read_slxylast,
  read_slxydata,

  write_slxqdready,
  write_slxqvalid,
  write_slxqhint,
  write_slxqlast,
  write_slxqafy,
  write_slxqdata,
  write_slxydready,
  write_slxyvalid,
  write_slxyhint,
  write_slxylast,
  write_slxydata
);



parameter LSU_PARA = 0;
parameter AXI_PARA = 32;
parameter BW_LPI_BURDEN = 1;
parameter MATRIX_SIZE_PARA = 4;
parameter TENSOR_PARA = 0;

`include "ervp_axi_lpara.vb"
`include "dca_matrix_dim_util.vb"
`include "dca_matrix_dim_lpara.vb"
`include "dca_tensor_scalar_lpara.vb"
`include "dca_tensor_dim_lpara.vb"

`include "lpit_function.vb"
`include "lpixm_function.vb"

localparam  BW_LPIXM_ADDR = BW_AXI_ADDR;
localparam  BW_LPIXM_DATA = BW_AXI_DATA;

`include "lpixm_lpara.vb"

input wire clk;
input wire rstnn;
input wire clear;
input wire enable;
output wire busy;

localparam  DCA_LPARA_04 = `BW_DCA_MATRIX_LSU_INST;

input wire inst_wvalid;
input wire [DCA_LPARA_04-1:0] inst_wdata;
output wire inst_wready;
output reg inst_is_write;
output wire inst_decode_finish;
output wire inst_execute_finish;

output wire load_tensor_row_wvalid;
output wire load_tensor_row_wlast;
output wire [BW_TENSOR_ROW-1:0] load_tensor_row_wdata;
input wire load_tensor_row_wready;

output wire store_tensor_row_rvalid;
output wire store_tensor_row_rlast;
input wire store_tensor_row_rready;
input wire [BW_TENSOR_ROW-1:0] store_tensor_row_rdata;

input wire [2-1:0] read_slxqdready;
output wire read_slxqvalid;
output wire read_slxqhint;
output wire read_slxqlast;
output wire read_slxqafy;
output wire [BW_LPI_QDATA-1:0] read_slxqdata;
output wire [2-1:0] read_slxydready;
input wire read_slxyvalid;
input wire read_slxyhint;
input wire read_slxylast;
input wire [BW_LPI_YDATA-1:0] read_slxydata;

input wire [2-1:0] write_slxqdready;
output wire write_slxqvalid;
output wire write_slxqhint;
output wire write_slxqlast;
output wire write_slxqafy;
output wire [BW_LPI_QDATA-1:0] write_slxqdata;
output wire [2-1:0] write_slxydready;
input wire write_slxyvalid;
input wire write_slxyhint;
input wire write_slxylast;
input wire [BW_LPI_YDATA-1:0] write_slxydata;

`include "dca_lsu_util.vb"
`include "dca_lsu_lpara.vb"
`include "motype_lpara.vb"

wire dca_signal_32;
wire dca_signal_60;
wire dca_signal_42;

wire dca_signal_57;

localparam  DCA_LPARA_07 = DCA_LPARA_04;
localparam  DCA_LPARA_03 = INST_FIFO_DEPTH;

wire dca_signal_45;
wire dca_signal_58;
wire [DCA_LPARA_07-1:0] dca_signal_21;
wire dca_signal_39;
wire dca_signal_64;
wire [DCA_LPARA_07-1:0] dca_signal_05;

localparam  DCA_LPARA_05 = BW_TXN_INFO;
localparam  DCA_LPARA_10 = MATRIX_NUM_COL;

wire dca_signal_38;
wire dca_signal_40;
wire [DCA_LPARA_05-1:0] dca_signal_54;
wire dca_signal_04;
wire dca_signal_48;
wire [DCA_LPARA_05-1:0] dca_signal_18;

wire dca_signal_27;
wire dca_signal_23;
wire [DCA_LPARA_05-1:0] dca_signal_52;
wire dca_signal_09;
wire dca_signal_28;
wire [DCA_LPARA_05-1:0] dca_signal_06;

wire dca_signal_61;
wire dca_signal_19;
wire [DCA_LPARA_05-1:0] dca_signal_26;
wire dca_signal_30;
wire dca_signal_65;
wire [DCA_LPARA_05-1:0] dca_signal_24;

wire dca_signal_37;
wire dca_signal_10;
wire [DCA_LPARA_05-1:0] dca_signal_03;
wire dca_signal_43;
wire dca_signal_16;
wire [DCA_LPARA_05-1:0] dca_signal_22;

localparam  DCA_LPARA_09 = BW_RRESP_INFO;
localparam  DCA_LPARA_08 = 2;

wire dca_signal_59;
wire dca_signal_07;
wire [DCA_LPARA_09-1:0] dca_signal_47;
wire dca_signal_50;
wire dca_signal_13;
wire [DCA_LPARA_09-1:0] dca_signal_53;

localparam  DCA_LPARA_12 = BW_WDATA_INFO;
localparam  DCA_LPARA_00 = 4;

wire dca_signal_49;
wire dca_signal_17;
wire [DCA_LPARA_12-1:0] dca_signal_25;
wire dca_signal_00;
wire dca_signal_33;
wire [DCA_LPARA_12-1:0] dca_signal_35;

wire dca_signal_12;
wire dca_signal_20;
wire [BW_AXI_DATA-1:0] dca_signal_62;
wire [`BW_AXI_WSTRB(BW_AXI_DATA)-1:0] dca_signal_51;
wire dca_signal_56;
wire dca_signal_44;

localparam  DCA_LPARA_01 = 2;
localparam  DCA_LPARA_11 = 0;
localparam  DCA_LPARA_02 = 1;
localparam  DCA_LPARA_06 = 2;

reg [DCA_LPARA_01-1:0] dca_signal_46;
wire dca_signal_63;
wire [DCA_LPARA_04-1:0] dca_signal_31;

wire [`BW_DCA_MATRIX_LSU_INST_OPCODE-1:0] dca_signal_15;

reg dca_signal_08;

wire dca_signal_14;
wire dca_signal_11;
wire dca_signal_02;
wire dca_signal_34;
wire dca_signal_55;
wire dca_signal_29;
wire dca_signal_01;
wire dca_signal_36;
wire dca_signal_41;

assign dca_signal_32 = (INCLUDE_INST_FIFO==1) & enable;
assign dca_signal_60 = READ_SUPPORTED & enable;
assign dca_signal_42 = WRITE_SUPPORTED & enable;

ERVP_FIFO
#(
  .BW_DATA(DCA_LPARA_07),
  .DEPTH(DCA_LPARA_03)
)
i_dca_instance_05
(
  .clk(clk),
  .rstnn(rstnn),
  .enable(dca_signal_32),
  .clear(clear),

  .wready(dca_signal_45),
  .wfull(),
  .wrequest(dca_signal_58),
  .wdata(dca_signal_21),
  .wnum(),
  .rready(dca_signal_39),
  .rempty(),
  .rrequest(dca_signal_64),
  .rdata(dca_signal_05),
  .rnum()
);

assign dca_signal_58 = inst_wvalid;
assign dca_signal_21 = inst_wdata;
assign dca_signal_64 = inst_decode_finish;

always@(posedge clk, negedge rstnn)
begin
  if(rstnn==0)
  begin
    dca_signal_46 <= DCA_LPARA_11;
    inst_is_write <= 0;
  end
  else if(enable)
    case(dca_signal_46)
      DCA_LPARA_11:
        if(dca_signal_63)
          dca_signal_46 <= DCA_LPARA_02;
      DCA_LPARA_02:
        if(dca_signal_08)
        begin
          dca_signal_46 <= DCA_LPARA_06;
          inst_is_write <= (dca_signal_15==`DCA_MATRIX_LSU_INST_OPCODE_WRITE);
        end
      DCA_LPARA_06:
        if(inst_decode_finish)
          dca_signal_46 <= DCA_LPARA_11;
    endcase
end

assign dca_signal_63 = (INCLUDE_INST_FIFO==0)? inst_wvalid : dca_signal_39;
assign dca_signal_31 = (INCLUDE_INST_FIFO==0)? inst_wdata : dca_signal_05;
assign inst_wready = (INCLUDE_INST_FIFO==0)? inst_decode_finish : dca_signal_45;

assign dca_signal_15 = dca_signal_31;

always@(*)
begin
  dca_signal_08 = 0;
  if(dca_signal_46==DCA_LPARA_02)
    case(dca_signal_15)
      `DCA_MATRIX_LSU_INST_OPCODE_READ:
        dca_signal_08 = ~dca_signal_01;
      `DCA_MATRIX_LSU_INST_OPCODE_WRITE:
        dca_signal_08 = ~dca_signal_02;
    endcase
end

DCA_MODULE_01
#(
  .LSU_PARA(LSU_PARA),
  .AXI_PARA(AXI_PARA),
  .MATRIX_SIZE_PARA(MATRIX_SIZE_PARA)
)
i_dca_instance_09
(
  .dca_port_12(clk),
  .dca_port_00(rstnn),
  .dca_port_21(clear),
  .dca_port_19(enable),
  .dca_port_06(dca_signal_14),

  .dca_port_07(dca_signal_57),

  .dca_port_09(dca_signal_08),
  .dca_port_24(dca_signal_31),
  .dca_port_17(),

  .dca_port_10(dca_signal_38),
  .dca_port_03(dca_signal_40),
  .dca_port_23(dca_signal_54),

  .dca_port_14(dca_signal_27),
  .dca_port_01(dca_signal_23),
  .dca_port_02(dca_signal_52),

  .dca_port_08(dca_signal_61),
  .dca_port_20(dca_signal_19),
  .dca_port_22(dca_signal_26),

  .dca_port_11(dca_signal_37),
  .dca_port_26(dca_signal_10),
  .dca_port_16(dca_signal_03),

  .dca_port_18(dca_signal_59),
  .dca_port_04(dca_signal_07),
  .dca_port_15(dca_signal_47),

  .dca_port_13(dca_signal_49),
  .dca_port_25(dca_signal_17),
  .dca_port_05(dca_signal_25)
);

assign dca_signal_57 = 1;

ERVP_FIFO
#(
  .BW_DATA(DCA_LPARA_05),
  .DEPTH(DCA_LPARA_10)
)
i_dca_instance_11
(
  .clk(clk),
  .rstnn(rstnn),
  .enable(dca_signal_60),
  .clear(clear),

  .wready(dca_signal_38),
  .wfull(),
  .wrequest(dca_signal_40),
  .wdata(dca_signal_54),
  .wnum(),
  .rready(dca_signal_04),
  .rempty(),
  .rrequest(dca_signal_48),
  .rdata(dca_signal_18),
  .rnum()
);

ERVP_FIFO
#(
  .BW_DATA(DCA_LPARA_05),
  .DEPTH(DCA_LPARA_10)
)
i_dca_instance_10
(
  .clk(clk),
  .rstnn(rstnn),
  .enable(dca_signal_60),
  .clear(clear),

  .wready(dca_signal_27),
  .wfull(),
  .wrequest(dca_signal_23),
  .wdata(dca_signal_52),
  .wnum(),
  .rready(dca_signal_09),
  .rempty(),
  .rrequest(dca_signal_28),
  .rdata(dca_signal_06),
  .rnum()
);

ERVP_FIFO
#(
  .BW_DATA(DCA_LPARA_05),
  .DEPTH(DCA_LPARA_10)
)
i_dca_instance_12
(
  .clk(clk),
  .rstnn(rstnn),
  .enable(dca_signal_42),
  .clear(clear),

  .wready(dca_signal_61),
  .wfull(),
  .wrequest(dca_signal_19),
  .wdata(dca_signal_26),
  .wnum(),
  .rready(dca_signal_30),
  .rempty(),
  .rrequest(dca_signal_65),
  .rdata(dca_signal_24),
  .rnum()
);

DCA_MODULE_28
#(
  .LSU_PARA(LSU_PARA),
  .AXI_PARA(AXI_PARA),
  .MATRIX_SIZE_PARA(MATRIX_SIZE_PARA),
  .DCA_GPARA_0(DCA_LPARA_10)
)
i_dca_instance_04
(
  .dca_port_07(clk),
  .dca_port_08(rstnn),
  .dca_port_09(dca_signal_42),
  .dca_port_02(clear),

  .dca_port_03(dca_signal_37),
  .dca_port_01(dca_signal_10),
  .dca_port_05(dca_signal_03),
  .dca_port_06(dca_signal_43),
  .dca_port_00(dca_signal_16),
  .dca_port_04(dca_signal_22)
);

ERVP_FIFO
#(
  .BW_DATA(DCA_LPARA_09),
  .DEPTH(DCA_LPARA_08)
)
i_dca_instance_03
(
  .clk(clk),
  .rstnn(rstnn),
  .enable(dca_signal_60),
  .clear(clear),

  .wready(dca_signal_59),
  .wfull(),
  .wrequest(dca_signal_07),
  .wdata(dca_signal_47),
  .wnum(),
  .rready(dca_signal_50),
  .rempty(),
  .rrequest(dca_signal_13),
  .rdata(dca_signal_53),
  .rnum()
);

ERVP_FIFO
#(
  .BW_DATA(DCA_LPARA_12),
  .DEPTH(DCA_LPARA_00)
)
i_dca_instance_06
(
  .clk(clk),
  .rstnn(rstnn),
  .enable(dca_signal_42),
  .clear(clear),

  .wready(dca_signal_49),
  .wfull(),
  .wrequest(dca_signal_17),
  .wdata(dca_signal_25),
  .wnum(),
  .rready(dca_signal_00),
  .rempty(),
  .rrequest(dca_signal_33),
  .rdata(dca_signal_35),
  .rnum()
);

DCA_MODULE_11
#(
  .LSU_PARA(LSU_PARA),
  .AXI_PARA(AXI_PARA),
  .BW_LPI_BURDEN(BW_LPI_BURDEN),
  .MATRIX_SIZE_PARA(MATRIX_SIZE_PARA)
)
i_dca_instance_01
(
  .dca_port_05(clk),
  .dca_port_09(rstnn),
  .dca_port_08(clear),
  .dca_port_07(dca_signal_60),
  .dca_port_03(dca_signal_11),

  .dca_port_06(dca_signal_31),

  .dca_port_01(dca_signal_04),
  .dca_port_11(dca_signal_18),
  .dca_port_12(dca_signal_48),

  .dca_port_00(read_slxqdready),
  .dca_port_13(read_slxqvalid),
  .dca_port_04(read_slxqhint),
  .dca_port_14(read_slxqlast),
  .dca_port_02(read_slxqafy),
  .dca_port_10(read_slxqdata)
);

assign inst_decode_finish = (dca_signal_46==DCA_LPARA_06) & (~dca_signal_14) & (~dca_signal_04) & (~dca_signal_30);

DCA_MODULE_16
#(
  .LSU_PARA(LSU_PARA),
  .AXI_PARA(AXI_PARA),
  .BW_LPI_BURDEN(BW_LPI_BURDEN),
  .MATRIX_SIZE_PARA(MATRIX_SIZE_PARA),
  .TENSOR_PARA(TENSOR_PARA)
)
i_dca_instance_02
(
  .dca_port_05(clk),
  .dca_port_00(rstnn),
  .dca_port_13(clear),
  .dca_port_19(dca_signal_60),
  .dca_port_20(dca_signal_02),

  .dca_port_07(dca_signal_34),

  .dca_port_10(dca_signal_50),
  .dca_port_02(dca_signal_53),
  .dca_port_03(dca_signal_13),

  .dca_port_04(dca_signal_09),
  .dca_port_06(dca_signal_06),
  .dca_port_12(dca_signal_28),

  .dca_port_16(read_slxydready),
  .dca_port_08(read_slxyvalid),
  .dca_port_18(read_slxyhint),
  .dca_port_17(read_slxylast),
  .dca_port_15(read_slxydata),

  .dca_port_01(load_tensor_row_wvalid),
  .dca_port_09(load_tensor_row_wlast),
  .dca_port_14(load_tensor_row_wdata),
  .dca_port_11(load_tensor_row_wready)
);

DCA_MODULE_19
#(
  .LSU_PARA(LSU_PARA),
  .AXI_PARA(AXI_PARA),
  .BW_LPI_BURDEN(BW_LPI_BURDEN),
  .MATRIX_SIZE_PARA(MATRIX_SIZE_PARA),
  .TENSOR_PARA(TENSOR_PARA)
)
i_dca_instance_00
(
  .dca_port_06(clk),
  .dca_port_20(rstnn),
  .dca_port_09(clear),
  .dca_port_15(dca_signal_42),
  .dca_port_18(dca_signal_29),

  .dca_port_12(dca_signal_00),
  .dca_port_07(dca_signal_35),
  .dca_port_00(dca_signal_33),

  .dca_port_13(dca_signal_43),
  .dca_port_11(dca_signal_22),
  .dca_port_04(dca_signal_16),

  .dca_port_01(store_tensor_row_rvalid),
  .dca_port_10(store_tensor_row_rlast),
  .dca_port_14(store_tensor_row_rready),
  .dca_port_16(store_tensor_row_rdata),

  .dca_port_02(dca_signal_12),
  .dca_port_08(dca_signal_20),
  .dca_port_05(dca_signal_62),
  .dca_port_03(dca_signal_51),
  .dca_port_17(dca_signal_56),
  .dca_port_19(dca_signal_44)
);

DCA_MODULE_07
#(
  .LSU_PARA(LSU_PARA),
  .AXI_PARA(AXI_PARA),
  .MATRIX_SIZE_PARA(MATRIX_SIZE_PARA),
  .BW_LPI_BURDEN(BW_LPI_BURDEN),
  .DCA_GPARA_0(DCA_LPARA_00)
)
i_dca_instance_07
(
  .dca_port_05(clk),
  .dca_port_13(rstnn),
  .dca_port_10(clear),
  .dca_port_04(dca_signal_42),
  .dca_port_03(dca_signal_01),

  .dca_port_12(dca_signal_36),
  .dca_port_06(dca_signal_41),

  .dca_port_01(dca_signal_12),
  .dca_port_11(dca_signal_20),
  .dca_port_07(dca_signal_56),
  .dca_port_02(dca_signal_44),

  .dca_port_08(write_slxydready),
  .dca_port_09(write_slxyvalid),
  .dca_port_00(write_slxyhint),
  .dca_port_14(write_slxylast),
  .dca_port_15(write_slxydata)
);

assign dca_signal_36 = dca_signal_34;

DCA_MODULE_23
#(
  .LSU_PARA(LSU_PARA),
  .AXI_PARA(AXI_PARA),
  .BW_LPI_BURDEN(BW_LPI_BURDEN),
  .MATRIX_SIZE_PARA(MATRIX_SIZE_PARA)
)
i_dca_instance_08
(
  .dca_port_04(clk),
  .dca_port_11(rstnn),
  .dca_port_12(clear),
  .dca_port_14(dca_signal_42),
  .dca_port_18(dca_signal_55),

  .dca_port_08(dca_signal_31),

  .dca_port_13(dca_signal_30),
  .dca_port_00(dca_signal_24),
  .dca_port_19(dca_signal_65),

  .dca_port_10(dca_signal_12),
  .dca_port_06(dca_signal_20),
  .dca_port_16(dca_signal_62),
  .dca_port_03(dca_signal_51),
  .dca_port_17(dca_signal_56),
  .dca_port_07(dca_signal_44),

  .dca_port_05(write_slxqdready),
  .dca_port_02(write_slxqvalid),
  .dca_port_15(write_slxqhint),
  .dca_port_09(write_slxqlast),
  .dca_port_20(write_slxqafy),
  .dca_port_01(write_slxqdata)
);

assign inst_execute_finish = dca_signal_34 | dca_signal_41;
assign busy = dca_signal_39 | (dca_signal_46!=DCA_LPARA_11) | dca_signal_14 | dca_signal_02 | dca_signal_29 | dca_signal_01;

endmodule
