module SimShell(
  input         clock,
  input         reset,
  output        mem_aw_ready,
  input         mem_aw_valid,
  input  [31:0] mem_aw_bits_addr,
  input  [7:0]  mem_aw_bits_len,
  output        mem_w_ready,
  input         mem_w_valid,
  input  [63:0] mem_w_bits_data,
  input         mem_w_bits_last,
  input         mem_b_ready,
  output        mem_b_valid,
  output        mem_ar_ready,
  input         mem_ar_valid,
  input  [31:0] mem_ar_bits_addr,
  input  [7:0]  mem_ar_bits_id,
  input  [7:0]  mem_ar_bits_len,
  output        mem_r_valid,
  output [63:0] mem_r_bits_data,
  output        mem_r_bits_last,
  output [7:0]  mem_r_bits_id,
  input         host_aw_ready,
  output        host_aw_valid,
  output [15:0] host_aw_bits_addr,
  input         host_w_ready,
  output        host_w_valid,
  output [31:0] host_w_bits_data,
  output        host_b_ready,
  input         host_b_valid,
  input         host_ar_ready,
  output        host_ar_valid,
  output [15:0] host_ar_bits_addr,
  output        host_r_ready,
  input         host_r_valid,
  input  [31:0] host_r_bits_data,
  input         sim_clock,
  output        sim_wait
);
  wire  mod_sim_clock; // @[SimShell.scala 90:23]
  wire  mod_sim_reset; // @[SimShell.scala 90:23]
  wire  mod_sim_sim_wait; // @[SimShell.scala 90:23]
  wire  mod_host_clock; // @[SimShell.scala 91:24]
  wire  mod_host_reset; // @[SimShell.scala 91:24]
  wire  mod_host_io_axi_aw_ready; // @[SimShell.scala 91:24]
  wire  mod_host_io_axi_aw_valid; // @[SimShell.scala 91:24]
  wire [15:0] mod_host_io_axi_aw_bits_addr; // @[SimShell.scala 91:24]
  wire  mod_host_io_axi_w_ready; // @[SimShell.scala 91:24]
  wire  mod_host_io_axi_w_valid; // @[SimShell.scala 91:24]
  wire [31:0] mod_host_io_axi_w_bits_data; // @[SimShell.scala 91:24]
  wire  mod_host_io_axi_b_ready; // @[SimShell.scala 91:24]
  wire  mod_host_io_axi_b_valid; // @[SimShell.scala 91:24]
  wire  mod_host_io_axi_ar_ready; // @[SimShell.scala 91:24]
  wire  mod_host_io_axi_ar_valid; // @[SimShell.scala 91:24]
  wire [15:0] mod_host_io_axi_ar_bits_addr; // @[SimShell.scala 91:24]
  wire  mod_host_io_axi_r_ready; // @[SimShell.scala 91:24]
  wire  mod_host_io_axi_r_valid; // @[SimShell.scala 91:24]
  wire [31:0] mod_host_io_axi_r_bits_data; // @[SimShell.scala 91:24]
  wire  mod_mem_clock; // @[SimShell.scala 92:23]
  wire  mod_mem_reset; // @[SimShell.scala 92:23]
  wire  mod_mem_io_axi_aw_ready; // @[SimShell.scala 92:23]
  wire  mod_mem_io_axi_aw_valid; // @[SimShell.scala 92:23]
  wire [31:0] mod_mem_io_axi_aw_bits_addr; // @[SimShell.scala 92:23]
  wire [7:0] mod_mem_io_axi_aw_bits_len; // @[SimShell.scala 92:23]
  wire  mod_mem_io_axi_w_ready; // @[SimShell.scala 92:23]
  wire  mod_mem_io_axi_w_valid; // @[SimShell.scala 92:23]
  wire [63:0] mod_mem_io_axi_w_bits_data; // @[SimShell.scala 92:23]
  wire  mod_mem_io_axi_w_bits_last; // @[SimShell.scala 92:23]
  wire  mod_mem_io_axi_b_ready; // @[SimShell.scala 92:23]
  wire  mod_mem_io_axi_b_valid; // @[SimShell.scala 92:23]
  wire  mod_mem_io_axi_ar_ready; // @[SimShell.scala 92:23]
  wire  mod_mem_io_axi_ar_valid; // @[SimShell.scala 92:23]
  wire [31:0] mod_mem_io_axi_ar_bits_addr; // @[SimShell.scala 92:23]
  wire [7:0] mod_mem_io_axi_ar_bits_id; // @[SimShell.scala 92:23]
  wire [7:0] mod_mem_io_axi_ar_bits_len; // @[SimShell.scala 92:23]
  wire  mod_mem_io_axi_r_valid; // @[SimShell.scala 92:23]
  wire [63:0] mod_mem_io_axi_r_bits_data; // @[SimShell.scala 92:23]
  wire  mod_mem_io_axi_r_bits_last; // @[SimShell.scala 92:23]
  wire [7:0] mod_mem_io_axi_r_bits_id; // @[SimShell.scala 92:23]
  VTASim mod_sim ( // @[SimShell.scala 90:23]
    .clock(mod_sim_clock),
    .reset(mod_sim_reset),
    .sim_wait(mod_sim_sim_wait)
  );
  VTAHost mod_host ( // @[SimShell.scala 91:24]
    .clock(mod_host_clock),
    .reset(mod_host_reset),
    .io_axi_aw_ready(mod_host_io_axi_aw_ready),
    .io_axi_aw_valid(mod_host_io_axi_aw_valid),
    .io_axi_aw_bits_addr(mod_host_io_axi_aw_bits_addr),
    .io_axi_w_ready(mod_host_io_axi_w_ready),
    .io_axi_w_valid(mod_host_io_axi_w_valid),
    .io_axi_w_bits_data(mod_host_io_axi_w_bits_data),
    .io_axi_b_ready(mod_host_io_axi_b_ready),
    .io_axi_b_valid(mod_host_io_axi_b_valid),
    .io_axi_ar_ready(mod_host_io_axi_ar_ready),
    .io_axi_ar_valid(mod_host_io_axi_ar_valid),
    .io_axi_ar_bits_addr(mod_host_io_axi_ar_bits_addr),
    .io_axi_r_ready(mod_host_io_axi_r_ready),
    .io_axi_r_valid(mod_host_io_axi_r_valid),
    .io_axi_r_bits_data(mod_host_io_axi_r_bits_data)
  );
  VTAMem mod_mem ( // @[SimShell.scala 92:23]
    .clock(mod_mem_clock),
    .reset(mod_mem_reset),
    .io_axi_aw_ready(mod_mem_io_axi_aw_ready),
    .io_axi_aw_valid(mod_mem_io_axi_aw_valid),
    .io_axi_aw_bits_addr(mod_mem_io_axi_aw_bits_addr),
    .io_axi_aw_bits_len(mod_mem_io_axi_aw_bits_len),
    .io_axi_w_ready(mod_mem_io_axi_w_ready),
    .io_axi_w_valid(mod_mem_io_axi_w_valid),
    .io_axi_w_bits_data(mod_mem_io_axi_w_bits_data),
    .io_axi_w_bits_last(mod_mem_io_axi_w_bits_last),
    .io_axi_b_ready(mod_mem_io_axi_b_ready),
    .io_axi_b_valid(mod_mem_io_axi_b_valid),
    .io_axi_ar_ready(mod_mem_io_axi_ar_ready),
    .io_axi_ar_valid(mod_mem_io_axi_ar_valid),
    .io_axi_ar_bits_addr(mod_mem_io_axi_ar_bits_addr),
    .io_axi_ar_bits_id(mod_mem_io_axi_ar_bits_id),
    .io_axi_ar_bits_len(mod_mem_io_axi_ar_bits_len),
    .io_axi_r_valid(mod_mem_io_axi_r_valid),
    .io_axi_r_bits_data(mod_mem_io_axi_r_bits_data),
    .io_axi_r_bits_last(mod_mem_io_axi_r_bits_last),
    .io_axi_r_bits_id(mod_mem_io_axi_r_bits_id)
  );
  assign mem_aw_ready = mod_mem_io_axi_aw_ready; // @[SimShell.scala 93:7]
  assign mem_w_ready = mod_mem_io_axi_w_ready; // @[SimShell.scala 93:7]
  assign mem_b_valid = mod_mem_io_axi_b_valid; // @[SimShell.scala 93:7]
  assign mem_ar_ready = mod_mem_io_axi_ar_ready; // @[SimShell.scala 93:7]
  assign mem_r_valid = mod_mem_io_axi_r_valid; // @[SimShell.scala 93:7]
  assign mem_r_bits_data = mod_mem_io_axi_r_bits_data; // @[SimShell.scala 93:7]
  assign mem_r_bits_last = mod_mem_io_axi_r_bits_last; // @[SimShell.scala 93:7]
  assign mem_r_bits_id = mod_mem_io_axi_r_bits_id; // @[SimShell.scala 93:7]
  assign host_aw_valid = mod_host_io_axi_aw_valid; // @[SimShell.scala 94:8]
  assign host_aw_bits_addr = mod_host_io_axi_aw_bits_addr; // @[SimShell.scala 94:8]
  assign host_w_valid = mod_host_io_axi_w_valid; // @[SimShell.scala 94:8]
  assign host_w_bits_data = mod_host_io_axi_w_bits_data; // @[SimShell.scala 94:8]
  assign host_b_ready = mod_host_io_axi_b_ready; // @[SimShell.scala 94:8]
  assign host_ar_valid = mod_host_io_axi_ar_valid; // @[SimShell.scala 94:8]
  assign host_ar_bits_addr = mod_host_io_axi_ar_bits_addr; // @[SimShell.scala 94:8]
  assign host_r_ready = mod_host_io_axi_r_ready; // @[SimShell.scala 94:8]
  assign sim_wait = mod_sim_sim_wait; // @[SimShell.scala 97:12]
  assign mod_sim_clock = sim_clock; // @[SimShell.scala 96:17]
  assign mod_sim_reset = reset; // @[SimShell.scala 95:17]
  assign mod_host_clock = clock;
  assign mod_host_reset = reset;
  assign mod_host_io_axi_aw_ready = host_aw_ready; // @[SimShell.scala 94:8]
  assign mod_host_io_axi_w_ready = host_w_ready; // @[SimShell.scala 94:8]
  assign mod_host_io_axi_b_valid = host_b_valid; // @[SimShell.scala 94:8]
  assign mod_host_io_axi_ar_ready = host_ar_ready; // @[SimShell.scala 94:8]
  assign mod_host_io_axi_r_valid = host_r_valid; // @[SimShell.scala 94:8]
  assign mod_host_io_axi_r_bits_data = host_r_bits_data; // @[SimShell.scala 94:8]
  assign mod_mem_clock = clock;
  assign mod_mem_reset = reset;
  assign mod_mem_io_axi_aw_valid = mem_aw_valid; // @[SimShell.scala 93:7]
  assign mod_mem_io_axi_aw_bits_addr = mem_aw_bits_addr; // @[SimShell.scala 93:7]
  assign mod_mem_io_axi_aw_bits_len = mem_aw_bits_len; // @[SimShell.scala 93:7]
  assign mod_mem_io_axi_w_valid = mem_w_valid; // @[SimShell.scala 93:7]
  assign mod_mem_io_axi_w_bits_data = mem_w_bits_data; // @[SimShell.scala 93:7]
  assign mod_mem_io_axi_w_bits_last = mem_w_bits_last; // @[SimShell.scala 93:7]
  assign mod_mem_io_axi_b_ready = mem_b_ready; // @[SimShell.scala 93:7]
  assign mod_mem_io_axi_ar_valid = mem_ar_valid; // @[SimShell.scala 93:7]
  assign mod_mem_io_axi_ar_bits_addr = mem_ar_bits_addr; // @[SimShell.scala 93:7]
  assign mod_mem_io_axi_ar_bits_id = mem_ar_bits_id; // @[SimShell.scala 93:7]
  assign mod_mem_io_axi_ar_bits_len = mem_ar_bits_len; // @[SimShell.scala 93:7]
endmodule