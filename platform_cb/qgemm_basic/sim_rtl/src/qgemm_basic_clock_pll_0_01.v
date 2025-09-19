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
// 2025-09-18
// Kyuseung Han (han@etri.re.kr)
// ****************************************************************************
// ****************************************************************************

module QGEMM_BASIC_CLOCK_PLL_0_01
(
	external_clk,
	external_clk_pair,
	external_rstnn,
	clk_system,
	clk_dram_sys,
	clk_dram_ref
);

input wire external_clk;
input wire external_clk_pair;
input wire external_rstnn;
output wire clk_system;
output wire clk_dram_sys;
output wire clk_dram_ref;

wire clk_100000000;

assign clk_dram_sys = 0;
assign clk_dram_ref = 0;

CLOCK_GENERATOR
#(
	.CLK_INITIAL_DELAY(1),
	.CLK_PERIOD(10)
)
i_gen_clk_0
(
	.clk(clk_100000000),
	.rstnn(),
	.rst()
);


assign clk_system = clk_100000000;


endmodule