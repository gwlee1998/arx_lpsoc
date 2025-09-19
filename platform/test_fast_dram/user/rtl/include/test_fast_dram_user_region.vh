/*****************/
/* Custom Region */
/*****************/

// wire clk_system;
// wire clk_core;
// wire clk_system_external;
// wire clk_system_debug;
// wire clk_local_access;
// wire clk_process_000;
// wire clk_noc;
// wire gclk_system;
// wire gclk_core;
// wire gclk_system_external;
// wire gclk_system_debug;
// wire gclk_local_access;
// wire gclk_process_000;
// wire gclk_noc;
// wire tick_1us;
// wire tick_62d5ms;
// wire tick_gpio;
// wire spi_common_sclk;
// wire spi_common_sdq0;
// wire global_rstnn;
// wire global_rstpp;
// wire [(6)-1:0] rstnn_seqeunce;
// wire [(6)-1:0] rstpp_seqeunce;
// wire rstnn_user;
// wire rstpp_user;
// wire i_userip1_clk;
// wire i_userip1_rstnn;
// wire i_userip1_rxawready;
// wire i_userip1_rxawvalid;
// wire [(32)-1:0] i_userip1_rxawaddr;
// wire [(4)-1:0] i_userip1_rxawid;
// wire [(8)-1:0] i_userip1_rxawlen;
// wire [(3)-1:0] i_userip1_rxawsize;
// wire [(2)-1:0] i_userip1_rxawburst;
// wire i_userip1_rxwready;
// wire i_userip1_rxwvalid;
// wire [(4)-1:0] i_userip1_rxwid;
// wire [(128)-1:0] i_userip1_rxwdata;
// wire [(128/8)-1:0] i_userip1_rxwstrb;
// wire i_userip1_rxwlast;
// wire i_userip1_rxbready;
// wire i_userip1_rxbvalid;
// wire [(4)-1:0] i_userip1_rxbid;
// wire [(2)-1:0] i_userip1_rxbresp;
// wire i_userip1_rxarready;
// wire i_userip1_rxarvalid;
// wire [(32)-1:0] i_userip1_rxaraddr;
// wire [(4)-1:0] i_userip1_rxarid;
// wire [(8)-1:0] i_userip1_rxarlen;
// wire [(3)-1:0] i_userip1_rxarsize;
// wire [(2)-1:0] i_userip1_rxarburst;
// wire i_userip1_rxrready;
// wire i_userip1_rxrvalid;
// wire [(4)-1:0] i_userip1_rxrid;
// wire [(128)-1:0] i_userip1_rxrdata;
// wire i_userip1_rxrlast;
// wire [(2)-1:0] i_userip1_rxrresp;

/* DO NOT MODIFY THE ABOVE */
/* MUST MODIFY THE BELOW   */


/*
USER_IP
#(
	.SIZE_OF_MEMORYMAP((32'h 8000)),
	.BW_AXI_TID(4),
	.BW_ADDR(32),
	.BW_DATA(128)
)
i_userip1
(
	.clk(i_userip1_clk),
	.rstnn(i_userip1_rstnn),
	.rxawready(i_userip1_rxawready),
	.rxawvalid(i_userip1_rxawvalid),
	.rxawaddr(i_userip1_rxawaddr),
	.rxawid(i_userip1_rxawid),
	.rxawlen(i_userip1_rxawlen),
	.rxawsize(i_userip1_rxawsize),
	.rxawburst(i_userip1_rxawburst),
	.rxwready(i_userip1_rxwready),
	.rxwvalid(i_userip1_rxwvalid),
	.rxwid(i_userip1_rxwid),
	.rxwdata(i_userip1_rxwdata),
	.rxwstrb(i_userip1_rxwstrb),
	.rxwlast(i_userip1_rxwlast),
	.rxbready(i_userip1_rxbready),
	.rxbvalid(i_userip1_rxbvalid),
	.rxbid(i_userip1_rxbid),
	.rxbresp(i_userip1_rxbresp),
	.rxarready(i_userip1_rxarready),
	.rxarvalid(i_userip1_rxarvalid),
	.rxaraddr(i_userip1_rxaraddr),
	.rxarid(i_userip1_rxarid),
	.rxarlen(i_userip1_rxarlen),
	.rxarsize(i_userip1_rxarsize),
	.rxarburst(i_userip1_rxarburst),
	.rxrready(i_userip1_rxrready),
	.rxrvalid(i_userip1_rxrvalid),
	.rxrid(i_userip1_rxrid),
	.rxrdata(i_userip1_rxrdata),
	.rxrlast(i_userip1_rxrlast),
	.rxrresp(i_userip1_rxrresp)
);
*/
//assign `NOT_CONNECT = i_userip1_clk;
//assign `NOT_CONNECT = i_userip1_rstnn;
assign i_userip1_rxawready = 0;
//assign `NOT_CONNECT = i_userip1_rxawvalid;
//assign `NOT_CONNECT = i_userip1_rxawaddr;
//assign `NOT_CONNECT = i_userip1_rxawid;
//assign `NOT_CONNECT = i_userip1_rxawlen;
//assign `NOT_CONNECT = i_userip1_rxawsize;
//assign `NOT_CONNECT = i_userip1_rxawburst;
assign i_userip1_rxwready = 0;
//assign `NOT_CONNECT = i_userip1_rxwvalid;
//assign `NOT_CONNECT = i_userip1_rxwid;
//assign `NOT_CONNECT = i_userip1_rxwdata;
//assign `NOT_CONNECT = i_userip1_rxwstrb;
//assign `NOT_CONNECT = i_userip1_rxwlast;
//assign `NOT_CONNECT = i_userip1_rxbready;
assign i_userip1_rxbvalid = 0;
assign i_userip1_rxbid = 0;
assign i_userip1_rxbresp = 0;
assign i_userip1_rxarready = 0;
//assign `NOT_CONNECT = i_userip1_rxarvalid;
//assign `NOT_CONNECT = i_userip1_rxaraddr;
//assign `NOT_CONNECT = i_userip1_rxarid;
//assign `NOT_CONNECT = i_userip1_rxarlen;
//assign `NOT_CONNECT = i_userip1_rxarsize;
//assign `NOT_CONNECT = i_userip1_rxarburst;
//assign `NOT_CONNECT = i_userip1_rxrready;
assign i_userip1_rxrvalid = 0;
assign i_userip1_rxrid = 0;
assign i_userip1_rxrdata = 0;
assign i_userip1_rxrlast = 0;
assign i_userip1_rxrresp = 0;

