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





module DCA_MODULE_28
(
  dca_port_07,
  dca_port_08,
  dca_port_02,
  dca_port_09,

  dca_port_03,
  dca_port_01,
  dca_port_05,
  dca_port_06,
  dca_port_04,
  dca_port_00
);





parameter LSU_PARA = 0;
parameter AXI_PARA = 32;
parameter MATRIX_SIZE_PARA = 4;
parameter DCA_GPARA_0 = 4;

`include "ervp_axi_lpara.vb"
`include "dca_matrix_dim_util.vb"
`include "dca_matrix_dim_lpara.vb"

input wire dca_port_07;
input wire dca_port_08;
input wire dca_port_02;
input wire dca_port_09;

`include "dca_lsu_util.vb"
`include "dca_lsu_lpara.vb"

output wire dca_port_03;
input wire dca_port_01;
input wire [BW_TXN_INFO-1:0] dca_port_05;
output wire dca_port_06;
output wire [BW_TXN_INFO-1:0] dca_port_04;
input wire dca_port_00;

wire [BW_TXN_INFO-1:0] dca_signal_11;
wire [BW_BITADDR-1:0] dca_signal_00;
wire [`BW_AXI_ALEN-1:0] dca_signal_02;
wire dca_signal_13;
wire dca_signal_09;

wire [BW_BITADDR-1:0] dca_signal_04;
wire [`BW_AXI_ALEN-1:0] dca_signal_01;
wire dca_signal_18;
wire dca_signal_07;

wire dca_signal_10;
reg dca_signal_14;
wire [BW_TXN_INFO-1:0] dca_signal_17;
wire dca_signal_05;
wire dca_signal_12;
wire [BW_TXN_INFO-1:0] dca_signal_15;

wire dca_signal_19;
wire dca_signal_16;

reg dca_signal_06;
reg [BW_TXN_INFO-1:0] dca_signal_03;
reg dca_signal_08;

assign dca_signal_11 = dca_port_05;
assign {dca_signal_09, dca_signal_13, dca_signal_02, dca_signal_00} = dca_signal_11;
assign {dca_signal_07, dca_signal_18, dca_signal_01, dca_signal_04} = dca_signal_03;

assign dca_signal_19 = dca_port_01 & dca_port_03;
assign dca_signal_16 = dca_signal_14 & dca_signal_10;

assign dca_port_03 = dca_signal_06? dca_signal_10 : 1;

always@(posedge dca_port_07 or negedge dca_port_08)
begin
  if(dca_port_08==0)
    dca_signal_06 <= 0;
  else if(dca_port_09)
  begin
    if(dca_port_01)
      dca_signal_06 <= 1;
    else if(dca_signal_16)
      dca_signal_06 <= 0;
  end
end

always@(posedge dca_port_07 or negedge dca_port_08)
begin
  if(dca_port_08==0)
    dca_signal_03 <= 0;
  else if(dca_port_09 && dca_signal_19)
    dca_signal_03 <= dca_port_05;
end

ERVP_FIFO
#(
  .BW_DATA(BW_TXN_INFO),
  .DEPTH(DCA_GPARA_0)
)
i_dca_instance_0
(
  .clk(dca_port_07),
  .rstnn(dca_port_08),
  .enable(dca_port_09),
  .clear(dca_port_02),
  .wready(dca_signal_10),
  .wfull(),
  .wrequest(dca_signal_14),
  .wdata(dca_signal_17),
  .wnum(),
  .rready(dca_signal_05),
  .rempty(),
  .rrequest(dca_signal_12),
  .rdata(dca_signal_15),
  .rnum()
);

always@(*)
begin
  dca_signal_14 = 0;
  dca_signal_08 = 0;
  if(dca_signal_06)
  begin
    if(dca_signal_18)
    begin
      dca_signal_14 = 1;
      dca_signal_08 = 0;
    end
    else if(dca_port_01)
    begin
      dca_signal_14 = 1;
      dca_signal_08 = dca_signal_09;
    end
  end
end

assign dca_signal_17 = {dca_signal_08, dca_signal_18, dca_signal_01, dca_signal_04};

assign dca_port_06 = dca_signal_05;
assign dca_port_04 = dca_signal_15;
assign dca_signal_12 = dca_port_00;

endmodule
