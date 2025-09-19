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
// 2025-09-19
// Kyuseung Han (han@etri.re.kr)
// ****************************************************************************
// ****************************************************************************

`include "ervp_platform_controller_memorymap_offset.vh"
`include "ervp_external_peri_group_memorymap_offset.vh"
`include "memorymap_info.vh"
`include "ervp_global.vh"
`include "platform_info.vh"
`include "munoc_network_include.vh"

module TEST_FAST_DRAM
(
	external_clk_0,
	external_clk_0_pair,
	external_rstnn,
	pjtag_rtck,
	pjtag_rtrstnn,
	pjtag_rtms,
	pjtag_rtdi,
	pjtag_rtdo,
	printf_tx,
	printf_rx
	`include "fast_dram_cell_port_dec.vh"
);


input wire external_clk_0;
input wire external_clk_0_pair;
input wire external_rstnn;
input wire pjtag_rtck;
input wire pjtag_rtrstnn;
input wire pjtag_rtms;
input wire pjtag_rtdi;
output wire pjtag_rtdo;
output wire printf_tx;
input wire printf_rx;

`include "fast_dram_cell_port_def.vh"


wire clk_system;
wire clk_core;
wire clk_system_external;
wire clk_system_debug;
wire clk_local_access;
wire clk_process_000;
wire clk_dram_if;
wire clk_dram_sys;
wire clk_dram_ref;
wire clk_noc;
wire gclk_system;
wire gclk_core;
wire gclk_system_external;
wire gclk_system_debug;
wire gclk_local_access;
wire gclk_process_000;
wire gclk_noc;
wire tick_1us;
wire tick_62d5ms;
wire tick_gpio;
wire spi_common_sclk;
wire spi_common_sdq0;
wire global_rstnn;
wire global_rstpp;
wire [(6)-1:0] rstnn_seqeunce;
wire [(6)-1:0] rstpp_seqeunce;
wire rstnn_user;
wire rstpp_user;
wire i_rtl_clk_system;
wire i_rtl_clk_core;
wire i_rtl_clk_system_external;
wire i_rtl_clk_system_debug;
wire i_rtl_clk_local_access;
wire i_rtl_clk_process_000;
wire i_rtl_clk_dram_if;
wire i_rtl_clk_dram_sys;
wire i_rtl_clk_dram_ref;
wire i_rtl_clk_noc;
wire i_rtl_gclk_system;
wire i_rtl_gclk_core;
wire i_rtl_gclk_system_external;
wire i_rtl_gclk_system_debug;
wire i_rtl_gclk_local_access;
wire i_rtl_gclk_process_000;
wire i_rtl_gclk_noc;
wire i_rtl_tick_1us;
wire i_rtl_tick_62d5ms;
wire i_rtl_tick_gpio;
wire i_rtl_spi_common_sclk;
wire i_rtl_spi_common_sdq0;
wire i_rtl_external_rstnn;
wire i_rtl_global_rstnn;
wire i_rtl_global_rstpp;
wire [(6)-1:0] i_rtl_rstnn_seqeunce;
wire [(6)-1:0] i_rtl_rstpp_seqeunce;
wire i_rtl_rstnn_user;
wire i_rtl_rstpp_user;
wire i_rtl_i_userip1_clk;
wire i_rtl_i_userip1_rstnn;
wire i_rtl_i_system_ddr_clk_ref;
wire i_rtl_i_system_ddr_clk_sys;
wire i_rtl_i_system_ddr_rstnn_sys;
wire i_rtl_i_system_ddr_clk_dram_if;
wire i_rtl_i_system_ddr_rstnn_dram_if;
wire i_rtl_i_system_ddr_initialized;
wire i_rtl_i_pll0_external_rstnn;
wire i_rtl_i_pll0_clk_system;
wire i_rtl_i_pll0_clk_dram_sys;
wire i_rtl_i_pll0_clk_dram_ref;
wire i_rtl_i_system_sram_clk;
wire i_rtl_i_system_sram_rstnn;
wire i_rtl_pjtag_rtck;
wire i_rtl_pjtag_rtrstnn;
wire i_rtl_pjtag_rtms;
wire i_rtl_pjtag_rtdi;
wire i_rtl_pjtag_rtdo;
wire i_rtl_printf_tx;
wire i_rtl_printf_rx;
wire i_rtl_i_userip1_sxawready;
wire i_rtl_i_userip1_sxawvalid;
wire [(32)-1:0] i_rtl_i_userip1_sxawaddr;
wire [(4)-1:0] i_rtl_i_userip1_sxawid;
wire [(8)-1:0] i_rtl_i_userip1_sxawlen;
wire [(3)-1:0] i_rtl_i_userip1_sxawsize;
wire [(2)-1:0] i_rtl_i_userip1_sxawburst;
wire i_rtl_i_userip1_sxwready;
wire i_rtl_i_userip1_sxwvalid;
wire [(4)-1:0] i_rtl_i_userip1_sxwid;
wire [(128)-1:0] i_rtl_i_userip1_sxwdata;
wire [(128/8)-1:0] i_rtl_i_userip1_sxwstrb;
wire i_rtl_i_userip1_sxwlast;
wire i_rtl_i_userip1_sxbready;
wire i_rtl_i_userip1_sxbvalid;
wire [(4)-1:0] i_rtl_i_userip1_sxbid;
wire [(2)-1:0] i_rtl_i_userip1_sxbresp;
wire i_rtl_i_userip1_sxarready;
wire i_rtl_i_userip1_sxarvalid;
wire [(32)-1:0] i_rtl_i_userip1_sxaraddr;
wire [(4)-1:0] i_rtl_i_userip1_sxarid;
wire [(8)-1:0] i_rtl_i_userip1_sxarlen;
wire [(3)-1:0] i_rtl_i_userip1_sxarsize;
wire [(2)-1:0] i_rtl_i_userip1_sxarburst;
wire i_rtl_i_userip1_sxrready;
wire i_rtl_i_userip1_sxrvalid;
wire [(4)-1:0] i_rtl_i_userip1_sxrid;
wire [(128)-1:0] i_rtl_i_userip1_sxrdata;
wire i_rtl_i_userip1_sxrlast;
wire [(2)-1:0] i_rtl_i_userip1_sxrresp;
wire i_rtl_i_system_ddr_sxawready;
wire i_rtl_i_system_ddr_sxawvalid;
wire [(32)-1:0] i_rtl_i_system_ddr_sxawaddr;
wire [(16)-1:0] i_rtl_i_system_ddr_sxawid;
wire [(8)-1:0] i_rtl_i_system_ddr_sxawlen;
wire [(3)-1:0] i_rtl_i_system_ddr_sxawsize;
wire [(2)-1:0] i_rtl_i_system_ddr_sxawburst;
wire i_rtl_i_system_ddr_sxwready;
wire i_rtl_i_system_ddr_sxwvalid;
wire [(16)-1:0] i_rtl_i_system_ddr_sxwid;
wire [(128)-1:0] i_rtl_i_system_ddr_sxwdata;
wire [(128/8)-1:0] i_rtl_i_system_ddr_sxwstrb;
wire i_rtl_i_system_ddr_sxwlast;
wire i_rtl_i_system_ddr_sxbready;
wire i_rtl_i_system_ddr_sxbvalid;
wire [(16)-1:0] i_rtl_i_system_ddr_sxbid;
wire [(2)-1:0] i_rtl_i_system_ddr_sxbresp;
wire i_rtl_i_system_ddr_sxarready;
wire i_rtl_i_system_ddr_sxarvalid;
wire [(32)-1:0] i_rtl_i_system_ddr_sxaraddr;
wire [(16)-1:0] i_rtl_i_system_ddr_sxarid;
wire [(8)-1:0] i_rtl_i_system_ddr_sxarlen;
wire [(3)-1:0] i_rtl_i_system_ddr_sxarsize;
wire [(2)-1:0] i_rtl_i_system_ddr_sxarburst;
wire i_rtl_i_system_ddr_sxrready;
wire i_rtl_i_system_ddr_sxrvalid;
wire [(16)-1:0] i_rtl_i_system_ddr_sxrid;
wire [(128)-1:0] i_rtl_i_system_ddr_sxrdata;
wire i_rtl_i_system_ddr_sxrlast;
wire [(2)-1:0] i_rtl_i_system_ddr_sxrresp;
wire i_rtl_i_system_sram_sxawready;
wire i_rtl_i_system_sram_sxawvalid;
wire [(32)-1:0] i_rtl_i_system_sram_sxawaddr;
wire [(`REQUIRED_BW_OF_SLAVE_TID)-1:0] i_rtl_i_system_sram_sxawid;
wire [(8)-1:0] i_rtl_i_system_sram_sxawlen;
wire [(3)-1:0] i_rtl_i_system_sram_sxawsize;
wire [(2)-1:0] i_rtl_i_system_sram_sxawburst;
wire i_rtl_i_system_sram_sxwready;
wire i_rtl_i_system_sram_sxwvalid;
wire [(`REQUIRED_BW_OF_SLAVE_TID)-1:0] i_rtl_i_system_sram_sxwid;
wire [(128)-1:0] i_rtl_i_system_sram_sxwdata;
wire [(128/8)-1:0] i_rtl_i_system_sram_sxwstrb;
wire i_rtl_i_system_sram_sxwlast;
wire i_rtl_i_system_sram_sxbready;
wire i_rtl_i_system_sram_sxbvalid;
wire [(`REQUIRED_BW_OF_SLAVE_TID)-1:0] i_rtl_i_system_sram_sxbid;
wire [(2)-1:0] i_rtl_i_system_sram_sxbresp;
wire i_rtl_i_system_sram_sxarready;
wire i_rtl_i_system_sram_sxarvalid;
wire [(32)-1:0] i_rtl_i_system_sram_sxaraddr;
wire [(`REQUIRED_BW_OF_SLAVE_TID)-1:0] i_rtl_i_system_sram_sxarid;
wire [(8)-1:0] i_rtl_i_system_sram_sxarlen;
wire [(3)-1:0] i_rtl_i_system_sram_sxarsize;
wire [(2)-1:0] i_rtl_i_system_sram_sxarburst;
wire i_rtl_i_system_sram_sxrready;
wire i_rtl_i_system_sram_sxrvalid;
wire [(`REQUIRED_BW_OF_SLAVE_TID)-1:0] i_rtl_i_system_sram_sxrid;
wire [(128)-1:0] i_rtl_i_system_sram_sxrdata;
wire i_rtl_i_system_sram_sxrlast;
wire [(2)-1:0] i_rtl_i_system_sram_sxrresp;
wire i_userip1_clk;
wire i_userip1_rstnn;
wire i_userip1_rxawready;
wire i_userip1_rxawvalid;
wire [(32)-1:0] i_userip1_rxawaddr;
wire [(4)-1:0] i_userip1_rxawid;
wire [(8)-1:0] i_userip1_rxawlen;
wire [(3)-1:0] i_userip1_rxawsize;
wire [(2)-1:0] i_userip1_rxawburst;
wire i_userip1_rxwready;
wire i_userip1_rxwvalid;
wire [(4)-1:0] i_userip1_rxwid;
wire [(128)-1:0] i_userip1_rxwdata;
wire [(128/8)-1:0] i_userip1_rxwstrb;
wire i_userip1_rxwlast;
wire i_userip1_rxbready;
wire i_userip1_rxbvalid;
wire [(4)-1:0] i_userip1_rxbid;
wire [(2)-1:0] i_userip1_rxbresp;
wire i_userip1_rxarready;
wire i_userip1_rxarvalid;
wire [(32)-1:0] i_userip1_rxaraddr;
wire [(4)-1:0] i_userip1_rxarid;
wire [(8)-1:0] i_userip1_rxarlen;
wire [(3)-1:0] i_userip1_rxarsize;
wire [(2)-1:0] i_userip1_rxarburst;
wire i_userip1_rxrready;
wire i_userip1_rxrvalid;
wire [(4)-1:0] i_userip1_rxrid;
wire [(128)-1:0] i_userip1_rxrdata;
wire i_userip1_rxrlast;
wire [(2)-1:0] i_userip1_rxrresp;
wire i_system_ddr_clk_ref;
wire i_system_ddr_clk_sys;
wire i_system_ddr_rstnn_sys;
wire i_system_ddr_clk_dram_if;
wire i_system_ddr_rstnn_dram_if;
wire i_system_ddr_initialized;
wire i_system_ddr_rxawready;
wire i_system_ddr_rxawvalid;
wire [(32)-1:0] i_system_ddr_rxawaddr;
wire [(16)-1:0] i_system_ddr_rxawid;
wire [(8)-1:0] i_system_ddr_rxawlen;
wire [(3)-1:0] i_system_ddr_rxawsize;
wire [(2)-1:0] i_system_ddr_rxawburst;
wire i_system_ddr_rxwready;
wire i_system_ddr_rxwvalid;
wire [(16)-1:0] i_system_ddr_rxwid;
wire [(128)-1:0] i_system_ddr_rxwdata;
wire [(128/8)-1:0] i_system_ddr_rxwstrb;
wire i_system_ddr_rxwlast;
wire i_system_ddr_rxbready;
wire i_system_ddr_rxbvalid;
wire [(16)-1:0] i_system_ddr_rxbid;
wire [(2)-1:0] i_system_ddr_rxbresp;
wire i_system_ddr_rxarready;
wire i_system_ddr_rxarvalid;
wire [(32)-1:0] i_system_ddr_rxaraddr;
wire [(16)-1:0] i_system_ddr_rxarid;
wire [(8)-1:0] i_system_ddr_rxarlen;
wire [(3)-1:0] i_system_ddr_rxarsize;
wire [(2)-1:0] i_system_ddr_rxarburst;
wire i_system_ddr_rxrready;
wire i_system_ddr_rxrvalid;
wire [(16)-1:0] i_system_ddr_rxrid;
wire [(128)-1:0] i_system_ddr_rxrdata;
wire i_system_ddr_rxrlast;
wire [(2)-1:0] i_system_ddr_rxrresp;
wire i_pll0_external_clk;
wire i_pll0_external_clk_pair;
wire i_pll0_external_rstnn;
wire i_pll0_clk_system;
wire i_pll0_clk_dram_sys;
wire i_pll0_clk_dram_ref;
wire i_system_sram_clk;
wire i_system_sram_rstnn;
wire i_system_sram_rxawready;
wire i_system_sram_rxawvalid;
wire [(32)-1:0] i_system_sram_rxawaddr;
wire [(`REQUIRED_BW_OF_SLAVE_TID)-1:0] i_system_sram_rxawid;
wire [(8)-1:0] i_system_sram_rxawlen;
wire [(3)-1:0] i_system_sram_rxawsize;
wire [(2)-1:0] i_system_sram_rxawburst;
wire i_system_sram_rxwready;
wire i_system_sram_rxwvalid;
wire [(`REQUIRED_BW_OF_SLAVE_TID)-1:0] i_system_sram_rxwid;
wire [(128)-1:0] i_system_sram_rxwdata;
wire [(128/8)-1:0] i_system_sram_rxwstrb;
wire i_system_sram_rxwlast;
wire i_system_sram_rxbready;
wire i_system_sram_rxbvalid;
wire [(`REQUIRED_BW_OF_SLAVE_TID)-1:0] i_system_sram_rxbid;
wire [(2)-1:0] i_system_sram_rxbresp;
wire i_system_sram_rxarready;
wire i_system_sram_rxarvalid;
wire [(32)-1:0] i_system_sram_rxaraddr;
wire [(`REQUIRED_BW_OF_SLAVE_TID)-1:0] i_system_sram_rxarid;
wire [(8)-1:0] i_system_sram_rxarlen;
wire [(3)-1:0] i_system_sram_rxarsize;
wire [(2)-1:0] i_system_sram_rxarburst;
wire i_system_sram_rxrready;
wire i_system_sram_rxrvalid;
wire [(`REQUIRED_BW_OF_SLAVE_TID)-1:0] i_system_sram_rxrid;
wire [(128)-1:0] i_system_sram_rxrdata;
wire i_system_sram_rxrlast;
wire [(2)-1:0] i_system_sram_rxrresp;

TEST_FAST_DRAM_RTL
i_rtl
(
	.clk_system(i_rtl_clk_system),
	.clk_core(i_rtl_clk_core),
	.clk_system_external(i_rtl_clk_system_external),
	.clk_system_debug(i_rtl_clk_system_debug),
	.clk_local_access(i_rtl_clk_local_access),
	.clk_process_000(i_rtl_clk_process_000),
	.clk_dram_if(i_rtl_clk_dram_if),
	.clk_dram_sys(i_rtl_clk_dram_sys),
	.clk_dram_ref(i_rtl_clk_dram_ref),
	.clk_noc(i_rtl_clk_noc),
	.gclk_system(i_rtl_gclk_system),
	.gclk_core(i_rtl_gclk_core),
	.gclk_system_external(i_rtl_gclk_system_external),
	.gclk_system_debug(i_rtl_gclk_system_debug),
	.gclk_local_access(i_rtl_gclk_local_access),
	.gclk_process_000(i_rtl_gclk_process_000),
	.gclk_noc(i_rtl_gclk_noc),
	.tick_1us(i_rtl_tick_1us),
	.tick_62d5ms(i_rtl_tick_62d5ms),
	.tick_gpio(i_rtl_tick_gpio),
	.spi_common_sclk(i_rtl_spi_common_sclk),
	.spi_common_sdq0(i_rtl_spi_common_sdq0),
	.external_rstnn(i_rtl_external_rstnn),
	.global_rstnn(i_rtl_global_rstnn),
	.global_rstpp(i_rtl_global_rstpp),
	.rstnn_seqeunce(i_rtl_rstnn_seqeunce),
	.rstpp_seqeunce(i_rtl_rstpp_seqeunce),
	.rstnn_user(i_rtl_rstnn_user),
	.rstpp_user(i_rtl_rstpp_user),
	.i_userip1_clk(i_rtl_i_userip1_clk),
	.i_userip1_rstnn(i_rtl_i_userip1_rstnn),
	.i_system_ddr_clk_ref(i_rtl_i_system_ddr_clk_ref),
	.i_system_ddr_clk_sys(i_rtl_i_system_ddr_clk_sys),
	.i_system_ddr_rstnn_sys(i_rtl_i_system_ddr_rstnn_sys),
	.i_system_ddr_clk_dram_if(i_rtl_i_system_ddr_clk_dram_if),
	.i_system_ddr_rstnn_dram_if(i_rtl_i_system_ddr_rstnn_dram_if),
	.i_system_ddr_initialized(i_rtl_i_system_ddr_initialized),
	.i_pll0_external_rstnn(i_rtl_i_pll0_external_rstnn),
	.i_pll0_clk_system(i_rtl_i_pll0_clk_system),
	.i_pll0_clk_dram_sys(i_rtl_i_pll0_clk_dram_sys),
	.i_pll0_clk_dram_ref(i_rtl_i_pll0_clk_dram_ref),
	.i_system_sram_clk(i_rtl_i_system_sram_clk),
	.i_system_sram_rstnn(i_rtl_i_system_sram_rstnn),
	.pjtag_rtck(i_rtl_pjtag_rtck),
	.pjtag_rtrstnn(i_rtl_pjtag_rtrstnn),
	.pjtag_rtms(i_rtl_pjtag_rtms),
	.pjtag_rtdi(i_rtl_pjtag_rtdi),
	.pjtag_rtdo(i_rtl_pjtag_rtdo),
	.printf_tx(i_rtl_printf_tx),
	.printf_rx(i_rtl_printf_rx),
	.i_userip1_sxawready(i_rtl_i_userip1_sxawready),
	.i_userip1_sxawvalid(i_rtl_i_userip1_sxawvalid),
	.i_userip1_sxawaddr(i_rtl_i_userip1_sxawaddr),
	.i_userip1_sxawid(i_rtl_i_userip1_sxawid),
	.i_userip1_sxawlen(i_rtl_i_userip1_sxawlen),
	.i_userip1_sxawsize(i_rtl_i_userip1_sxawsize),
	.i_userip1_sxawburst(i_rtl_i_userip1_sxawburst),
	.i_userip1_sxwready(i_rtl_i_userip1_sxwready),
	.i_userip1_sxwvalid(i_rtl_i_userip1_sxwvalid),
	.i_userip1_sxwid(i_rtl_i_userip1_sxwid),
	.i_userip1_sxwdata(i_rtl_i_userip1_sxwdata),
	.i_userip1_sxwstrb(i_rtl_i_userip1_sxwstrb),
	.i_userip1_sxwlast(i_rtl_i_userip1_sxwlast),
	.i_userip1_sxbready(i_rtl_i_userip1_sxbready),
	.i_userip1_sxbvalid(i_rtl_i_userip1_sxbvalid),
	.i_userip1_sxbid(i_rtl_i_userip1_sxbid),
	.i_userip1_sxbresp(i_rtl_i_userip1_sxbresp),
	.i_userip1_sxarready(i_rtl_i_userip1_sxarready),
	.i_userip1_sxarvalid(i_rtl_i_userip1_sxarvalid),
	.i_userip1_sxaraddr(i_rtl_i_userip1_sxaraddr),
	.i_userip1_sxarid(i_rtl_i_userip1_sxarid),
	.i_userip1_sxarlen(i_rtl_i_userip1_sxarlen),
	.i_userip1_sxarsize(i_rtl_i_userip1_sxarsize),
	.i_userip1_sxarburst(i_rtl_i_userip1_sxarburst),
	.i_userip1_sxrready(i_rtl_i_userip1_sxrready),
	.i_userip1_sxrvalid(i_rtl_i_userip1_sxrvalid),
	.i_userip1_sxrid(i_rtl_i_userip1_sxrid),
	.i_userip1_sxrdata(i_rtl_i_userip1_sxrdata),
	.i_userip1_sxrlast(i_rtl_i_userip1_sxrlast),
	.i_userip1_sxrresp(i_rtl_i_userip1_sxrresp),
	.i_system_ddr_sxawready(i_rtl_i_system_ddr_sxawready),
	.i_system_ddr_sxawvalid(i_rtl_i_system_ddr_sxawvalid),
	.i_system_ddr_sxawaddr(i_rtl_i_system_ddr_sxawaddr),
	.i_system_ddr_sxawid(i_rtl_i_system_ddr_sxawid),
	.i_system_ddr_sxawlen(i_rtl_i_system_ddr_sxawlen),
	.i_system_ddr_sxawsize(i_rtl_i_system_ddr_sxawsize),
	.i_system_ddr_sxawburst(i_rtl_i_system_ddr_sxawburst),
	.i_system_ddr_sxwready(i_rtl_i_system_ddr_sxwready),
	.i_system_ddr_sxwvalid(i_rtl_i_system_ddr_sxwvalid),
	.i_system_ddr_sxwid(i_rtl_i_system_ddr_sxwid),
	.i_system_ddr_sxwdata(i_rtl_i_system_ddr_sxwdata),
	.i_system_ddr_sxwstrb(i_rtl_i_system_ddr_sxwstrb),
	.i_system_ddr_sxwlast(i_rtl_i_system_ddr_sxwlast),
	.i_system_ddr_sxbready(i_rtl_i_system_ddr_sxbready),
	.i_system_ddr_sxbvalid(i_rtl_i_system_ddr_sxbvalid),
	.i_system_ddr_sxbid(i_rtl_i_system_ddr_sxbid),
	.i_system_ddr_sxbresp(i_rtl_i_system_ddr_sxbresp),
	.i_system_ddr_sxarready(i_rtl_i_system_ddr_sxarready),
	.i_system_ddr_sxarvalid(i_rtl_i_system_ddr_sxarvalid),
	.i_system_ddr_sxaraddr(i_rtl_i_system_ddr_sxaraddr),
	.i_system_ddr_sxarid(i_rtl_i_system_ddr_sxarid),
	.i_system_ddr_sxarlen(i_rtl_i_system_ddr_sxarlen),
	.i_system_ddr_sxarsize(i_rtl_i_system_ddr_sxarsize),
	.i_system_ddr_sxarburst(i_rtl_i_system_ddr_sxarburst),
	.i_system_ddr_sxrready(i_rtl_i_system_ddr_sxrready),
	.i_system_ddr_sxrvalid(i_rtl_i_system_ddr_sxrvalid),
	.i_system_ddr_sxrid(i_rtl_i_system_ddr_sxrid),
	.i_system_ddr_sxrdata(i_rtl_i_system_ddr_sxrdata),
	.i_system_ddr_sxrlast(i_rtl_i_system_ddr_sxrlast),
	.i_system_ddr_sxrresp(i_rtl_i_system_ddr_sxrresp),
	.i_system_sram_sxawready(i_rtl_i_system_sram_sxawready),
	.i_system_sram_sxawvalid(i_rtl_i_system_sram_sxawvalid),
	.i_system_sram_sxawaddr(i_rtl_i_system_sram_sxawaddr),
	.i_system_sram_sxawid(i_rtl_i_system_sram_sxawid),
	.i_system_sram_sxawlen(i_rtl_i_system_sram_sxawlen),
	.i_system_sram_sxawsize(i_rtl_i_system_sram_sxawsize),
	.i_system_sram_sxawburst(i_rtl_i_system_sram_sxawburst),
	.i_system_sram_sxwready(i_rtl_i_system_sram_sxwready),
	.i_system_sram_sxwvalid(i_rtl_i_system_sram_sxwvalid),
	.i_system_sram_sxwid(i_rtl_i_system_sram_sxwid),
	.i_system_sram_sxwdata(i_rtl_i_system_sram_sxwdata),
	.i_system_sram_sxwstrb(i_rtl_i_system_sram_sxwstrb),
	.i_system_sram_sxwlast(i_rtl_i_system_sram_sxwlast),
	.i_system_sram_sxbready(i_rtl_i_system_sram_sxbready),
	.i_system_sram_sxbvalid(i_rtl_i_system_sram_sxbvalid),
	.i_system_sram_sxbid(i_rtl_i_system_sram_sxbid),
	.i_system_sram_sxbresp(i_rtl_i_system_sram_sxbresp),
	.i_system_sram_sxarready(i_rtl_i_system_sram_sxarready),
	.i_system_sram_sxarvalid(i_rtl_i_system_sram_sxarvalid),
	.i_system_sram_sxaraddr(i_rtl_i_system_sram_sxaraddr),
	.i_system_sram_sxarid(i_rtl_i_system_sram_sxarid),
	.i_system_sram_sxarlen(i_rtl_i_system_sram_sxarlen),
	.i_system_sram_sxarsize(i_rtl_i_system_sram_sxarsize),
	.i_system_sram_sxarburst(i_rtl_i_system_sram_sxarburst),
	.i_system_sram_sxrready(i_rtl_i_system_sram_sxrready),
	.i_system_sram_sxrvalid(i_rtl_i_system_sram_sxrvalid),
	.i_system_sram_sxrid(i_rtl_i_system_sram_sxrid),
	.i_system_sram_sxrdata(i_rtl_i_system_sram_sxrdata),
	.i_system_sram_sxrlast(i_rtl_i_system_sram_sxrlast),
	.i_system_sram_sxrresp(i_rtl_i_system_sram_sxrresp)
);

TEST_FAST_DRAM_FAST_DRAM_00
i_system_ddr
(
	.clk_ref(i_system_ddr_clk_ref),
	.clk_sys(i_system_ddr_clk_sys),
	.rstnn_sys(i_system_ddr_rstnn_sys),
	.clk_dram_if(i_system_ddr_clk_dram_if),
	.rstnn_dram_if(i_system_ddr_rstnn_dram_if),
	.initialized(i_system_ddr_initialized),
	.rxawready(i_system_ddr_rxawready),
	.rxawvalid(i_system_ddr_rxawvalid),
	.rxawaddr(i_system_ddr_rxawaddr),
	.rxawid(i_system_ddr_rxawid),
	.rxawlen(i_system_ddr_rxawlen),
	.rxawsize(i_system_ddr_rxawsize),
	.rxawburst(i_system_ddr_rxawburst),
	.rxwready(i_system_ddr_rxwready),
	.rxwvalid(i_system_ddr_rxwvalid),
	.rxwid(i_system_ddr_rxwid),
	.rxwdata(i_system_ddr_rxwdata),
	.rxwstrb(i_system_ddr_rxwstrb),
	.rxwlast(i_system_ddr_rxwlast),
	.rxbready(i_system_ddr_rxbready),
	.rxbvalid(i_system_ddr_rxbvalid),
	.rxbid(i_system_ddr_rxbid),
	.rxbresp(i_system_ddr_rxbresp),
	.rxarready(i_system_ddr_rxarready),
	.rxarvalid(i_system_ddr_rxarvalid),
	.rxaraddr(i_system_ddr_rxaraddr),
	.rxarid(i_system_ddr_rxarid),
	.rxarlen(i_system_ddr_rxarlen),
	.rxarsize(i_system_ddr_rxarsize),
	.rxarburst(i_system_ddr_rxarburst),
	.rxrready(i_system_ddr_rxrready),
	.rxrvalid(i_system_ddr_rxrvalid),
	.rxrid(i_system_ddr_rxrid),
	.rxrdata(i_system_ddr_rxrdata),
	.rxrlast(i_system_ddr_rxrlast),
	.rxrresp(i_system_ddr_rxrresp)
	`include "fast_dram_cell_port_mapping.vh"
);

TEST_FAST_DRAM_CLOCK_PLL_0_01
i_pll0
(
	.external_clk(i_pll0_external_clk),
	.external_clk_pair(i_pll0_external_clk_pair),
	.external_rstnn(i_pll0_external_rstnn),
	.clk_system(i_pll0_clk_system),
	.clk_dram_sys(i_pll0_clk_dram_sys),
	.clk_dram_ref(i_pll0_clk_dram_ref)
);

TEST_FAST_DRAM_SRAM_AXI_02
i_system_sram
(
	.clk(i_system_sram_clk),
	.rstnn(i_system_sram_rstnn),
	.rxawready(i_system_sram_rxawready),
	.rxawvalid(i_system_sram_rxawvalid),
	.rxawaddr(i_system_sram_rxawaddr),
	.rxawid(i_system_sram_rxawid),
	.rxawlen(i_system_sram_rxawlen),
	.rxawsize(i_system_sram_rxawsize),
	.rxawburst(i_system_sram_rxawburst),
	.rxwready(i_system_sram_rxwready),
	.rxwvalid(i_system_sram_rxwvalid),
	.rxwid(i_system_sram_rxwid),
	.rxwdata(i_system_sram_rxwdata),
	.rxwstrb(i_system_sram_rxwstrb),
	.rxwlast(i_system_sram_rxwlast),
	.rxbready(i_system_sram_rxbready),
	.rxbvalid(i_system_sram_rxbvalid),
	.rxbid(i_system_sram_rxbid),
	.rxbresp(i_system_sram_rxbresp),
	.rxarready(i_system_sram_rxarready),
	.rxarvalid(i_system_sram_rxarvalid),
	.rxaraddr(i_system_sram_rxaraddr),
	.rxarid(i_system_sram_rxarid),
	.rxarlen(i_system_sram_rxarlen),
	.rxarsize(i_system_sram_rxarsize),
	.rxarburst(i_system_sram_rxarburst),
	.rxrready(i_system_sram_rxrready),
	.rxrvalid(i_system_sram_rxrvalid),
	.rxrid(i_system_sram_rxrid),
	.rxrdata(i_system_sram_rxrdata),
	.rxrlast(i_system_sram_rxrlast),
	.rxrresp(i_system_sram_rxrresp)
);

assign i_rtl_external_rstnn = external_rstnn;
assign i_userip1_clk = i_rtl_i_userip1_clk;
assign i_userip1_rstnn = i_rtl_i_userip1_rstnn;
assign i_system_ddr_clk_ref = i_rtl_i_system_ddr_clk_ref;
assign i_system_ddr_clk_sys = i_rtl_i_system_ddr_clk_sys;
assign i_system_ddr_rstnn_sys = i_rtl_i_system_ddr_rstnn_sys;
assign i_rtl_i_system_ddr_clk_dram_if = i_system_ddr_clk_dram_if;
assign i_system_ddr_rstnn_dram_if = i_rtl_i_system_ddr_rstnn_dram_if;
assign i_rtl_i_system_ddr_initialized = i_system_ddr_initialized;
assign i_pll0_external_clk = external_clk_0;
assign i_pll0_external_clk_pair = external_clk_0_pair;
assign i_pll0_external_rstnn = i_rtl_i_pll0_external_rstnn;
assign i_rtl_i_pll0_clk_system = i_pll0_clk_system;
assign i_rtl_i_pll0_clk_dram_sys = i_pll0_clk_dram_sys;
assign i_rtl_i_pll0_clk_dram_ref = i_pll0_clk_dram_ref;
assign i_system_sram_clk = i_rtl_i_system_sram_clk;
assign i_system_sram_rstnn = i_rtl_i_system_sram_rstnn;
assign clk_system = i_rtl_clk_system;
assign clk_core = i_rtl_clk_core;
assign clk_system_external = i_rtl_clk_system_external;
assign clk_system_debug = i_rtl_clk_system_debug;
assign clk_local_access = i_rtl_clk_local_access;
assign clk_process_000 = i_rtl_clk_process_000;
assign clk_dram_if = i_rtl_clk_dram_if;
assign clk_dram_sys = i_rtl_clk_dram_sys;
assign clk_dram_ref = i_rtl_clk_dram_ref;
assign clk_noc = i_rtl_clk_noc;
assign gclk_system = i_rtl_gclk_system;
assign gclk_core = i_rtl_gclk_core;
assign gclk_system_external = i_rtl_gclk_system_external;
assign gclk_system_debug = i_rtl_gclk_system_debug;
assign gclk_local_access = i_rtl_gclk_local_access;
assign gclk_process_000 = i_rtl_gclk_process_000;
assign gclk_noc = i_rtl_gclk_noc;
assign tick_1us = i_rtl_tick_1us;
assign tick_62d5ms = i_rtl_tick_62d5ms;
assign tick_gpio = i_rtl_tick_gpio;
assign spi_common_sclk = i_rtl_spi_common_sclk;
assign spi_common_sdq0 = i_rtl_spi_common_sdq0;
assign global_rstnn = i_rtl_global_rstnn;
assign global_rstpp = i_rtl_global_rstpp;
assign rstnn_seqeunce = i_rtl_rstnn_seqeunce;
assign rstpp_seqeunce = i_rtl_rstpp_seqeunce;
assign rstnn_user = i_rtl_rstnn_user;
assign rstpp_user = i_rtl_rstpp_user;
assign i_rtl_pjtag_rtck = pjtag_rtck;
assign i_rtl_pjtag_rtrstnn = pjtag_rtrstnn;
assign i_rtl_pjtag_rtms = pjtag_rtms;
assign i_rtl_pjtag_rtdi = pjtag_rtdi;
assign pjtag_rtdo = i_rtl_pjtag_rtdo;
assign printf_tx = i_rtl_printf_tx;
assign i_rtl_printf_rx = printf_rx;
assign i_rtl_i_userip1_sxawready = i_userip1_rxawready;
assign i_userip1_rxawvalid = i_rtl_i_userip1_sxawvalid;
assign i_userip1_rxawaddr = i_rtl_i_userip1_sxawaddr;
assign i_userip1_rxawid = i_rtl_i_userip1_sxawid;
assign i_userip1_rxawlen = i_rtl_i_userip1_sxawlen;
assign i_userip1_rxawsize = i_rtl_i_userip1_sxawsize;
assign i_userip1_rxawburst = i_rtl_i_userip1_sxawburst;
assign i_rtl_i_userip1_sxwready = i_userip1_rxwready;
assign i_userip1_rxwvalid = i_rtl_i_userip1_sxwvalid;
assign i_userip1_rxwid = i_rtl_i_userip1_sxwid;
assign i_userip1_rxwdata = i_rtl_i_userip1_sxwdata;
assign i_userip1_rxwstrb = i_rtl_i_userip1_sxwstrb;
assign i_userip1_rxwlast = i_rtl_i_userip1_sxwlast;
assign i_userip1_rxbready = i_rtl_i_userip1_sxbready;
assign i_rtl_i_userip1_sxbvalid = i_userip1_rxbvalid;
assign i_rtl_i_userip1_sxbid = i_userip1_rxbid;
assign i_rtl_i_userip1_sxbresp = i_userip1_rxbresp;
assign i_rtl_i_userip1_sxarready = i_userip1_rxarready;
assign i_userip1_rxarvalid = i_rtl_i_userip1_sxarvalid;
assign i_userip1_rxaraddr = i_rtl_i_userip1_sxaraddr;
assign i_userip1_rxarid = i_rtl_i_userip1_sxarid;
assign i_userip1_rxarlen = i_rtl_i_userip1_sxarlen;
assign i_userip1_rxarsize = i_rtl_i_userip1_sxarsize;
assign i_userip1_rxarburst = i_rtl_i_userip1_sxarburst;
assign i_userip1_rxrready = i_rtl_i_userip1_sxrready;
assign i_rtl_i_userip1_sxrvalid = i_userip1_rxrvalid;
assign i_rtl_i_userip1_sxrid = i_userip1_rxrid;
assign i_rtl_i_userip1_sxrdata = i_userip1_rxrdata;
assign i_rtl_i_userip1_sxrlast = i_userip1_rxrlast;
assign i_rtl_i_userip1_sxrresp = i_userip1_rxrresp;
assign i_rtl_i_system_ddr_sxawready = i_system_ddr_rxawready;
assign i_system_ddr_rxawvalid = i_rtl_i_system_ddr_sxawvalid;
assign i_system_ddr_rxawaddr = i_rtl_i_system_ddr_sxawaddr;
assign i_system_ddr_rxawid = i_rtl_i_system_ddr_sxawid;
assign i_system_ddr_rxawlen = i_rtl_i_system_ddr_sxawlen;
assign i_system_ddr_rxawsize = i_rtl_i_system_ddr_sxawsize;
assign i_system_ddr_rxawburst = i_rtl_i_system_ddr_sxawburst;
assign i_rtl_i_system_ddr_sxwready = i_system_ddr_rxwready;
assign i_system_ddr_rxwvalid = i_rtl_i_system_ddr_sxwvalid;
assign i_system_ddr_rxwid = i_rtl_i_system_ddr_sxwid;
assign i_system_ddr_rxwdata = i_rtl_i_system_ddr_sxwdata;
assign i_system_ddr_rxwstrb = i_rtl_i_system_ddr_sxwstrb;
assign i_system_ddr_rxwlast = i_rtl_i_system_ddr_sxwlast;
assign i_system_ddr_rxbready = i_rtl_i_system_ddr_sxbready;
assign i_rtl_i_system_ddr_sxbvalid = i_system_ddr_rxbvalid;
assign i_rtl_i_system_ddr_sxbid = i_system_ddr_rxbid;
assign i_rtl_i_system_ddr_sxbresp = i_system_ddr_rxbresp;
assign i_rtl_i_system_ddr_sxarready = i_system_ddr_rxarready;
assign i_system_ddr_rxarvalid = i_rtl_i_system_ddr_sxarvalid;
assign i_system_ddr_rxaraddr = i_rtl_i_system_ddr_sxaraddr;
assign i_system_ddr_rxarid = i_rtl_i_system_ddr_sxarid;
assign i_system_ddr_rxarlen = i_rtl_i_system_ddr_sxarlen;
assign i_system_ddr_rxarsize = i_rtl_i_system_ddr_sxarsize;
assign i_system_ddr_rxarburst = i_rtl_i_system_ddr_sxarburst;
assign i_system_ddr_rxrready = i_rtl_i_system_ddr_sxrready;
assign i_rtl_i_system_ddr_sxrvalid = i_system_ddr_rxrvalid;
assign i_rtl_i_system_ddr_sxrid = i_system_ddr_rxrid;
assign i_rtl_i_system_ddr_sxrdata = i_system_ddr_rxrdata;
assign i_rtl_i_system_ddr_sxrlast = i_system_ddr_rxrlast;
assign i_rtl_i_system_ddr_sxrresp = i_system_ddr_rxrresp;
assign i_rtl_i_system_sram_sxawready = i_system_sram_rxawready;
assign i_system_sram_rxawvalid = i_rtl_i_system_sram_sxawvalid;
assign i_system_sram_rxawaddr = i_rtl_i_system_sram_sxawaddr;
assign i_system_sram_rxawid = i_rtl_i_system_sram_sxawid;
assign i_system_sram_rxawlen = i_rtl_i_system_sram_sxawlen;
assign i_system_sram_rxawsize = i_rtl_i_system_sram_sxawsize;
assign i_system_sram_rxawburst = i_rtl_i_system_sram_sxawburst;
assign i_rtl_i_system_sram_sxwready = i_system_sram_rxwready;
assign i_system_sram_rxwvalid = i_rtl_i_system_sram_sxwvalid;
assign i_system_sram_rxwid = i_rtl_i_system_sram_sxwid;
assign i_system_sram_rxwdata = i_rtl_i_system_sram_sxwdata;
assign i_system_sram_rxwstrb = i_rtl_i_system_sram_sxwstrb;
assign i_system_sram_rxwlast = i_rtl_i_system_sram_sxwlast;
assign i_system_sram_rxbready = i_rtl_i_system_sram_sxbready;
assign i_rtl_i_system_sram_sxbvalid = i_system_sram_rxbvalid;
assign i_rtl_i_system_sram_sxbid = i_system_sram_rxbid;
assign i_rtl_i_system_sram_sxbresp = i_system_sram_rxbresp;
assign i_rtl_i_system_sram_sxarready = i_system_sram_rxarready;
assign i_system_sram_rxarvalid = i_rtl_i_system_sram_sxarvalid;
assign i_system_sram_rxaraddr = i_rtl_i_system_sram_sxaraddr;
assign i_system_sram_rxarid = i_rtl_i_system_sram_sxarid;
assign i_system_sram_rxarlen = i_rtl_i_system_sram_sxarlen;
assign i_system_sram_rxarsize = i_rtl_i_system_sram_sxarsize;
assign i_system_sram_rxarburst = i_rtl_i_system_sram_sxarburst;
assign i_system_sram_rxrready = i_rtl_i_system_sram_sxrready;
assign i_rtl_i_system_sram_sxrvalid = i_system_sram_rxrvalid;
assign i_rtl_i_system_sram_sxrid = i_system_sram_rxrid;
assign i_rtl_i_system_sram_sxrdata = i_system_sram_rxrdata;
assign i_rtl_i_system_sram_sxrlast = i_system_sram_rxrlast;
assign i_rtl_i_system_sram_sxrresp = i_system_sram_rxrresp;

`include "test_fast_dram_user_region.vh"

`ifdef USE_ILA
`include "ila_description.vh"
`endif

endmodule