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


module ERVP_MATRIX_LSU
(
  clk,
  rstnn,

  rinst_wvalid,
  rinst_wdata,
  rinst_wready,
  rinst_decode_finish,
  rinst_execute_finish,
  rinst_busy,

  slxqdready,
  slxqvalid,
  slxqlast,
  slxqwrite,
  slxqlen,
  slxqsize,
  slxqburst,
  slxqwstrb,
  slxqwdata,
  slxqaddr,
  slxqburden,
  slxydready,
  slxyvalid,
  slxylast,
  slxywreply,
  slxyresp,
  slxyrdata,
  slxyburden,
  
  rload_tensor_row_wvalid,
  rload_tensor_row_wlast,
  rload_tensor_row_wdata,
  rload_tensor_row_wready,

  rstore_tensor_row_rvalid,
  rstore_tensor_row_rlast,
  rstore_tensor_row_rready,
  rstore_tensor_row_rdata
);


parameter BW_AXI_ADDR = 32;
parameter BW_AXI_DATA = 32;
parameter LSU_PARA = 0;
parameter BW_TENSOR_SCALAR = 33;
parameter MATRIX_NUM_COL = 4;

localparam  BW_LPI_BURDEN = 1;
localparam  DCA_LPARA_0 = `BW_DCA_MATRIX_LSU_INST;
localparam  DCA_LPARA_1 = BW_TENSOR_SCALAR*MATRIX_NUM_COL;

input wire clk;
input wire rstnn;

input wire rinst_wvalid;
input wire [DCA_LPARA_0-1:0] rinst_wdata;
output wire rinst_wready;
output wire rinst_decode_finish;
output wire rinst_execute_finish;
output wire rinst_busy;

output wire rload_tensor_row_wvalid;
output wire rload_tensor_row_wlast;
output wire [DCA_LPARA_1-1:0] rload_tensor_row_wdata;
input wire rload_tensor_row_wready;

output wire rstore_tensor_row_rvalid;
output wire rstore_tensor_row_rlast;
input wire rstore_tensor_row_rready;
input wire [DCA_LPARA_1-1:0] rstore_tensor_row_rdata;

input wire [(2)-1:0] slxqdready;
output wire slxqvalid;
output wire slxqlast;
output wire slxqwrite;
output wire [`BW_AXI_ALEN-1:0] slxqlen;
output wire [`BW_AXI_ASIZE-1:0] slxqsize;
output wire [`BW_AXI_ABURST-1:0] slxqburst;
output wire [(BW_AXI_DATA/8)-1:0] slxqwstrb;
output wire [BW_AXI_DATA-1:0] slxqwdata;
output wire [BW_AXI_ADDR-1:0] slxqaddr;
output wire [BW_LPI_BURDEN-1:0] slxqburden;

output wire [(2)-1:0] slxydready;
input wire slxyvalid;
input wire slxylast;
input wire slxywreply;
input wire [`BW_AXI_RESP-1:0] slxyresp;
input wire [BW_AXI_DATA-1:0] slxyrdata;
input wire [BW_LPI_BURDEN-1:0] slxyburden;

DCA_MATRIX_LSU_XMI1P
#(
  .LSU_PARA(LSU_PARA),
  .AXI_PARA(BW_AXI_DATA),
  .BW_LPI_BURDEN(BW_LPI_BURDEN),
  .MATRIX_SIZE_PARA(MATRIX_NUM_COL),
  .TENSOR_PARA(BW_TENSOR_SCALAR)
)
i_dca_instance_0
(
  .clk(clk),
  .rstnn(rstnn),
  .clear(1'b 0),
  .enable(1'b 1),
  .busy(rinst_busy),

  .inst_wvalid(rinst_wvalid),
  .inst_wdata(rinst_wdata),
  .inst_wready(rinst_wready),
  .inst_decode_finish(rinst_decode_finish),
  .inst_execute_finish(rinst_execute_finish),

  .load_tensor_row_wvalid(rload_tensor_row_wvalid),
  .load_tensor_row_wlast(rload_tensor_row_wlast),
  .load_tensor_row_wdata(rload_tensor_row_wdata),
  .load_tensor_row_wready(rload_tensor_row_wready),

  .store_tensor_row_rvalid(rstore_tensor_row_rvalid),
  .store_tensor_row_rlast(rstore_tensor_row_rlast),
  .store_tensor_row_rready(rstore_tensor_row_rready),
  .store_tensor_row_rdata(rstore_tensor_row_rdata),

  .slxqdready(slxqdready),
  .slxqvalid(slxqvalid),
  .slxqlast(slxqlast),
  .slxqwrite(slxqwrite),
  .slxqlen(slxqlen),
  .slxqsize(slxqsize),
  .slxqburst(slxqburst),
  .slxqwstrb(slxqwstrb),
  .slxqwdata(slxqwdata),
  .slxqaddr(slxqaddr),
  .slxqburden(slxqburden),
  .slxydready(slxydready),
  .slxyvalid(slxyvalid),
  .slxylast(slxylast),
  .slxywreply(slxywreply),
  .slxyresp(slxyresp),
  .slxyrdata(slxyrdata),
  .slxyburden(slxyburden)
);

endmodule
