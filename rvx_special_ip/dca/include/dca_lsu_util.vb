localparam LSU_PARA_1_BIT = 8;
localparam LSU_PARA_2_BIT = 2;
localparam LSU_PARA_3_BIT = 2;
localparam LSU_PARA_4_BIT = 16;

function integer MAKE_LSU_READ_ONLY;
	input integer para;
  integer temp;
begin
  temp = (2**LSU_PARA_2_BIT)-1;
  temp = temp << LSU_PARA_1_BIT;
  temp = ~temp;
  MAKE_LSU_READ_ONLY = para & temp;
  MAKE_LSU_READ_ONLY = MAKE_LSU_READ_ONLY | (1<<LSU_PARA_1_BIT);
end
endfunction

function integer MAKE_LSU_WRITE_ONLY;
	input integer para;
  integer temp;
begin
  temp = (2**LSU_PARA_2_BIT)-1;
  temp = temp << LSU_PARA_1_BIT;
  temp = ~temp;
  MAKE_LSU_WRITE_ONLY = para & temp;
  MAKE_LSU_WRITE_ONLY = MAKE_LSU_WRITE_ONLY | (2<<LSU_PARA_1_BIT);
end
endfunction

function integer GEN_LSU_PARA;
  input integer max_bw_memory_single;
  input integer memory_operation_type;
  input integer byte_support_type;
  input integer inst_fifo_depth;
begin
  GEN_LSU_PARA = max_bw_memory_single;
  GEN_LSU_PARA = GEN_LSU_PARA | (memory_operation_type<<LSU_PARA_1_BIT);
  GEN_LSU_PARA = GEN_LSU_PARA | (byte_support_type<<(LSU_PARA_1_BIT+LSU_PARA_2_BIT));
  GEN_LSU_PARA = GEN_LSU_PARA | (inst_fifo_depth<<(LSU_PARA_1_BIT+LSU_PARA_2_BIT+LSU_PARA_3_BIT));
end
endfunction
