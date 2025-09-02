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





module DCA_MODULE_22 
(
  dca_port_07,
  dca_port_06,
  dca_port_03,

  dca_port_10,
  dca_port_04,

  dca_port_02,
  dca_port_12,
  
  dca_port_01,
  dca_port_08,

  dca_port_11,
  dca_port_05,
  dca_port_00,
  dca_port_09
);





parameter DCA_GPARA_2 = 10;
parameter DCA_GPARA_3 = 10;
parameter DCA_GPARA_0 = 8;
parameter DCA_GPARA_1 = 8;

input wire dca_port_07;
input wire dca_port_06;
input wire dca_port_03;

input wire [DCA_GPARA_0-1:0] dca_port_10;
input wire [DCA_GPARA_1-1:0] dca_port_04;

input wire dca_port_02;
output wire dca_port_12;

output wire dca_port_01;
output wire dca_port_08;

output wire dca_port_11;
output wire dca_port_05;
output wire dca_port_00;
output wire dca_port_09;

localparam  DCA_LPARA_5 = 2;
localparam  DCA_LPARA_4 = 0;
localparam  DCA_LPARA_2 = 1;
localparam  DCA_LPARA_6 = 2;

reg [DCA_LPARA_5-1:0] dca_signal_06;
wire dca_signal_05;

localparam  DCA_LPARA_3 = DCA_GPARA_0;

reg [DCA_LPARA_3-1:0] dca_signal_03;
wire dca_signal_01;

wire dca_signal_02;
wire dca_signal_14;
wire [DCA_LPARA_3-1:0] dca_signal_12;
wire dca_signal_00;

localparam  DCA_LPARA_1 = DCA_GPARA_1 + 1;

reg [DCA_LPARA_1-1:0] dca_signal_11;
reg [DCA_LPARA_1-1:0] dca_signal_07;
wire dca_signal_04;
wire dca_signal_18;

wire dca_signal_17;
wire dca_signal_08;
wire [DCA_LPARA_1-1:0] dca_signal_13;
wire dca_signal_10;

localparam  DCA_LPARA_0 = `MAX(DCA_GPARA_3,DCA_GPARA_2);

wire dca_signal_16;
wire dca_signal_09;
wire [DCA_LPARA_0-1:0] dca_signal_20;

wire dca_signal_15;
wire dca_signal_19;

always@(posedge dca_port_07, negedge dca_port_06)
begin
  if(dca_port_06==0)
    dca_signal_06 <= DCA_LPARA_4;
  else if(dca_port_03)
  begin
    if(dca_port_12)
      dca_signal_06 <= DCA_LPARA_4;
    else if(dca_port_02)
      dca_signal_06 <= DCA_LPARA_6;
    else if(dca_signal_14)
      dca_signal_06 <= DCA_LPARA_2;
    else if(dca_signal_06==DCA_LPARA_2)
      dca_signal_06 <= DCA_LPARA_6;
  end
end

assign dca_signal_05 = (dca_signal_06==DCA_LPARA_6);

always@(posedge dca_port_07, negedge dca_port_06)
begin
  if(dca_port_06==0)
    dca_signal_03 <= 0;
  else if(dca_port_03)
  begin
    if(dca_port_02)
      dca_signal_03 <= dca_port_10;
    else if(dca_signal_14)
      dca_signal_03 <= (dca_signal_03>>1);
  end
end

assign dca_signal_01 = dca_signal_05 & dca_signal_03[0];

ERVP_COUNTER_WITH_ONEHOT_ENCODING
#(
  .COUNT_LENGTH(DCA_LPARA_3),
  .UP(1),
  .RESET_INDEX(0),
  .CIRCULAR(1)
)
i_dca_instance_0
(
  .clk(dca_port_07),
  .rstnn(dca_port_06),
  .enable(dca_port_03),
  .init(dca_signal_02),
  .count(dca_signal_14),
  .value(dca_signal_12),
  .is_first_count(),
  .is_last_count(dca_signal_00)
);

assign dca_signal_02 = dca_port_02;
assign dca_signal_14 = dca_signal_08 & dca_signal_10;

always@(posedge dca_port_07, negedge dca_port_06)
begin
  if(dca_port_06==0)
    dca_signal_11 <= 0;
  else if(dca_port_03)
  begin
    if(dca_port_02 || dca_signal_14)
      dca_signal_11 <= dca_port_04;
    else if(dca_signal_08)
      dca_signal_11 <= (dca_signal_11>>1);
  end
end

assign dca_signal_04 =  dca_signal_01 & dca_signal_11[0];

always@(posedge dca_port_07, negedge dca_port_06)
begin
  if(dca_port_06==0)
    dca_signal_07 <= 0;
  else if(dca_port_03)
  begin
    if(dca_port_02 || dca_signal_14)
      dca_signal_07 <= {dca_port_04,1'b 0};
    else if(dca_signal_08)
      dca_signal_07 <= (dca_signal_07>>1);
  end
end

assign dca_signal_18 = dca_signal_01 & dca_signal_07[0];

ERVP_COUNTER_WITH_ONEHOT_ENCODING
#(
  .COUNT_LENGTH(DCA_LPARA_1),
  .UP(1),
  .RESET_INDEX(0),
  .CIRCULAR(1)
)
i_dca_instance_2
(
  .clk(dca_port_07),
  .rstnn(dca_port_06),
  .enable(dca_port_03),
  .init(dca_signal_17),
  .count(dca_signal_08),
  .value(dca_signal_13),
  .is_first_count(),
  .is_last_count(dca_signal_10)
);

assign dca_signal_17 = dca_port_02 | dca_signal_14;
assign dca_signal_08 = dca_signal_05 & dca_signal_19;
assign dca_signal_15 = dca_signal_04 | dca_signal_18;
assign dca_signal_19 = dca_signal_15? dca_signal_20[DCA_LPARA_0-1] : 1;

ERVP_COUNTER_WITH_ONEHOT_ENCODING
#(
  .COUNT_LENGTH(DCA_LPARA_0),
  .UP(1),
  .RESET_INDEX(0),
  .CIRCULAR(1)
)
i_dca_instance_1
(
  .clk(dca_port_07),
  .rstnn(dca_port_06),
  .enable(dca_port_03),
  .init(dca_signal_16),
  .count(dca_signal_09),
  .value(dca_signal_20),
  .is_first_count(),
  .is_last_count()
);

assign dca_signal_16 = dca_port_02;
assign dca_signal_09 = dca_signal_15;

assign dca_port_01 = dca_signal_04 & dca_signal_20[0];
assign dca_port_08 = dca_signal_18 & dca_signal_20[0];

assign dca_port_12 = dca_signal_14 & dca_signal_00;

assign dca_port_11 = dca_signal_05 & dca_signal_10 & dca_signal_20[0];
assign dca_port_05 = dca_signal_05 & (~dca_signal_10) & dca_signal_20[0];
assign dca_port_00 = dca_signal_04 & dca_signal_20[DCA_GPARA_2-1];
assign dca_port_09 = dca_signal_18 & dca_signal_20[DCA_GPARA_3-1];

endmodule
