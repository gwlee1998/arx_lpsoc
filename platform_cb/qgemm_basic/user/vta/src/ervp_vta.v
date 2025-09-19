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
// 2023-11-13
// Kyuseung Han (han@etri.re.kr)
// ****************************************************************************
// ****************************************************************************

`include "ervp_global.vh"
`include "ervp_axi_define.vh"
`include "platform_info.vh"

module ERVP_VTA
(
	clk,
	rstnn,

	data_sx4rready,
	data_sx4rvalid,
	data_sx4rlast,
	data_sx4rresp,
	data_sx4rdata,
	data_sx4rid,
	data_sx4arready,
	data_sx4arvalid,
	data_sx4arburst,
	data_sx4arsize,
	data_sx4arlen,
	data_sx4araddr,
	data_sx4arid,
	data_sx4bready,
	data_sx4bvalid,
	data_sx4bresp,
	data_sx4bid,
	data_sx4wready,
	data_sx4wvalid,
	data_sx4wlast,
	data_sx4wstrb,
	data_sx4wdata,
	data_sx4awready,
	data_sx4awvalid,
	data_sx4awburst,
	data_sx4awsize,
	data_sx4awlen,
	data_sx4awaddr,
	data_sx4awid,

	config_rx4lawaddr,
	config_rx4lawvalid,
	config_rx4lawready,
	config_rx4lwdata,
	config_rx4lwstrb,
	config_rx4lwvalid,
	config_rx4lwready, 
	config_rx4lbresp,
	config_rx4lbvalid,
	config_rx4lbready,
	config_rx4laraddr,
	config_rx4larvalid,
	config_rx4larready,
	config_rx4lrdata,
	config_rx4lrresp,
	config_rx4lrvalid,
	config_rx4lrready
);

////////////////////////////
/* parameter input output */
////////////////////////////

localparam DATA_BW_ADDR = 32;
localparam DATA_BW_DATA = 64;
localparam DATA_BW_TID = 4;

localparam CONFIG_BW_ADDR = 32;
localparam CONFIG_BW_DATA = 32;

input wire clk;
input wire rstnn;

output wire [DATA_BW_TID-1:0] data_sx4awid;
output wire [DATA_BW_ADDR-1:0] data_sx4awaddr;
output wire [`BW_AXI_ALEN-1:0] data_sx4awlen;
output wire [`BW_AXI_ASIZE-1:0] data_sx4awsize;
output wire [`BW_AXI_ABURST-1:0] data_sx4awburst;
output wire data_sx4awvalid;
input wire data_sx4awready;

output wire [DATA_BW_DATA-1:0] data_sx4wdata;
output wire [`BW_AXI_WSTRB(DATA_BW_DATA)-1:0] data_sx4wstrb;
output wire data_sx4wlast;
output wire data_sx4wvalid;
input wire data_sx4wready;

input wire [DATA_BW_TID-1:0] data_sx4bid;
input wire [`BW_AXI_BRESP-1:0] data_sx4bresp;
input wire data_sx4bvalid;
output wire data_sx4bready;

output wire [DATA_BW_TID-1:0] data_sx4arid;
output wire [DATA_BW_ADDR-1:0] data_sx4araddr;
output wire [`BW_AXI_ALEN-1:0] data_sx4arlen;
output wire [`BW_AXI_ASIZE-1:0] data_sx4arsize;
output wire [`BW_AXI_ABURST-1:0] data_sx4arburst;
output wire data_sx4arvalid;
input wire data_sx4arready;

input wire [DATA_BW_TID-1:0] data_sx4rid;
input wire [DATA_BW_DATA-1:0] data_sx4rdata;
input wire [`BW_AXI_RRESP-1:0] data_sx4rresp;
input wire data_sx4rlast;
input wire data_sx4rvalid;
output wire data_sx4rready;

input wire [CONFIG_BW_ADDR-1:0] config_rx4lawaddr;
input wire config_rx4lawvalid;
output wire config_rx4lawready;

input wire [CONFIG_BW_DATA-1:0] config_rx4lwdata;
input wire [`BW_AXI_WSTRB(CONFIG_BW_DATA)-1:0] config_rx4lwstrb;
input wire config_rx4lwvalid;
output wire config_rx4lwready;

output wire [`BW_AXI_BRESP-1:0] config_rx4lbresp;
output wire config_rx4lbvalid;
input wire config_rx4lbready;

input wire [CONFIG_BW_ADDR-1:0] config_rx4laraddr;
input wire config_rx4larvalid;
output wire config_rx4larready;

output wire [CONFIG_BW_DATA-1:0] config_rx4lrdata;
output wire [`BW_AXI_RRESP-1:0] config_rx4lrresp;
output wire config_rx4lrvalid;
input wire config_rx4lrready;

/////////////
/* signals */
/////////////

wire io_host_aw_ready;
wire io_host_aw_valid;
wire [15:0] io_host_aw_bits_addr;
wire io_host_w_ready;
wire io_host_w_valid;
wire [31:0] io_host_w_bits_data;
wire io_host_b_ready;
wire io_host_b_valid;
wire io_host_ar_ready;
wire io_host_ar_valid;
wire [15:0] io_host_ar_bits_addr;
wire io_host_r_ready;
wire io_host_r_valid;
wire [31:0] io_host_r_bits_data;

wire io_mem_aw_ready;
wire io_mem_aw_valid;
wire [31:0] io_mem_aw_bits_addr;
wire [7:0]  io_mem_aw_bits_len;
wire io_mem_w_ready;
wire io_mem_w_valid;
wire [63:0] io_mem_w_bits_data;
wire io_mem_w_bits_last;
wire io_mem_b_ready;
wire io_mem_b_valid;
wire io_mem_ar_ready;
wire io_mem_ar_valid;
wire [31:0] io_mem_ar_bits_addr;
wire [7:0]  io_mem_ar_bits_id;
wire [7:0]  io_mem_ar_bits_len;
wire io_mem_r_valid;
wire [63:0] io_mem_r_bits_data;
wire io_mem_r_bits_last;
wire [7:0]  io_mem_r_bits_id;

////////////
/* logics */
////////////

VTAShell
i_imporeted
(
  .clock(clk),
  .reset(~rstnn),

  .io_host_aw_ready(io_host_aw_ready),
  .io_host_aw_valid(io_host_aw_valid),
  .io_host_aw_bits_addr(io_host_aw_bits_addr),
  .io_host_w_ready(io_host_w_ready),
  .io_host_w_valid(io_host_w_valid),
  .io_host_w_bits_data(io_host_w_bits_data),
  .io_host_b_ready(io_host_b_ready),
  .io_host_b_valid(io_host_b_valid),
  .io_host_ar_ready(io_host_ar_ready),
  .io_host_ar_valid(io_host_ar_valid),
  .io_host_ar_bits_addr(io_host_ar_bits_addr),
  .io_host_r_ready(io_host_r_ready),
  .io_host_r_valid(io_host_r_valid),
  .io_host_r_bits_data(io_host_r_bits_data),

  .io_mem_aw_ready(io_mem_aw_ready),
  .io_mem_aw_valid(io_mem_aw_valid),
  .io_mem_aw_bits_addr(io_mem_aw_bits_addr),
  .io_mem_aw_bits_len(io_mem_aw_bits_len),
  .io_mem_w_ready(io_mem_w_ready),
  .io_mem_w_valid(io_mem_w_valid),
  .io_mem_w_bits_data(io_mem_w_bits_data),
  .io_mem_w_bits_last(io_mem_w_bits_last),
  .io_mem_b_ready(io_mem_b_ready),
  .io_mem_b_valid(io_mem_b_valid),
  .io_mem_ar_ready(io_mem_ar_ready),
  .io_mem_ar_valid(io_mem_ar_valid),
  .io_mem_ar_bits_addr(io_mem_ar_bits_addr),
  .io_mem_ar_bits_id(io_mem_ar_bits_id),
  .io_mem_ar_bits_len(io_mem_ar_bits_len),
  .io_mem_r_valid(io_mem_r_valid),
  .io_mem_r_bits_data(io_mem_r_bits_data),
  .io_mem_r_bits_last(io_mem_r_bits_last),
  .io_mem_r_bits_id(io_mem_r_bits_id)
);

assign config_rx4lawready = io_host_aw_ready;
assign io_host_aw_valid = config_rx4lawvalid;
assign io_host_aw_bits_addr = config_rx4lawaddr;
assign config_rx4lwready = io_host_w_ready;
assign io_host_w_valid = config_rx4lwvalid;
assign io_host_w_bits_data = config_rx4lwdata;
assign io_host_b_ready = config_rx4lbready;
assign config_rx4lbvalid = io_host_b_valid;
assign config_rx4larready = io_host_ar_ready;
assign io_host_ar_valid = config_rx4larvalid;
assign io_host_ar_bits_addr = config_rx4laraddr;
assign io_host_r_ready = config_rx4lrready;
assign config_rx4lrvalid = io_host_r_valid;
assign config_rx4lrdata = io_host_r_bits_data;

assign config_rx4lbresp = 0;
assign config_rx4lrresp = 0;

assign io_mem_aw_ready = data_sx4awready;
assign data_sx4awvalid = io_mem_aw_valid;
assign data_sx4awaddr = io_mem_aw_bits_addr;
assign data_sx4awlen = io_mem_aw_bits_len;
assign io_mem_w_ready = data_sx4wready;
assign data_sx4wvalid = io_mem_w_valid;
assign data_sx4wdata = io_mem_w_bits_data;
assign data_sx4wlast = io_mem_w_bits_last;
assign data_sx4bready = io_mem_b_ready;
assign io_mem_b_valid = data_sx4bvalid;
assign io_mem_ar_ready = data_sx4arready;
assign data_sx4arvalid = io_mem_ar_valid;
assign data_sx4araddr = io_mem_ar_bits_addr;
assign data_sx4arid = io_mem_ar_bits_id;
assign data_sx4arlen = io_mem_ar_bits_len;
assign io_mem_r_valid = data_sx4rvalid;
assign io_mem_r_bits_data = data_sx4rdata;
assign io_mem_r_bits_last = data_sx4rlast;
assign io_mem_r_bits_id = data_sx4rid;


assign data_sx4awid = 0;
assign data_sx4awsize = `GET_AXI_SIZE(DATA_BW_DATA);
assign data_sx4awburst = `AXI_BURST_INCR;
assign data_sx4wstrb = `ALL_ONE;
assign data_sx4arsize = `GET_AXI_SIZE(DATA_BW_DATA);
assign data_sx4arburst = `AXI_BURST_INCR;
assign data_sx4rready = 1;

endmodule
