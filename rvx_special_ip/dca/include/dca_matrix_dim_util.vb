// MATRIX_SIZE_PARA

localparam MATRIX_SIZE_PARA_SEPARATOR = 10000;

function integer GEN_MATRIX_SIZE_PARA;
  input integer num_col;
  input integer num_row;
begin
  GEN_MATRIX_SIZE_PARA = num_row + (MATRIX_SIZE_PARA_SEPARATOR*num_col);
end
endfunction

function integer GET_MATRIX_NUM_COL;
  input integer para;
begin
  GET_MATRIX_NUM_COL = para % MATRIX_SIZE_PARA_SEPARATOR;
end
endfunction
