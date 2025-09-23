#include "test_matrix.h"

#include "core_dependent.h"
#include "ervp_variable_allocation.h"
#include "ervp_float.h"
#include "ervp_printf.h"
#include "ervp_assert.h"
#include "ervp_matrix_element.h"

void generate_test_matrix(ErvpMatrixInfo* matrix_info, int index)
{
  int i,j;
  int invert = 0;
  int index2 = index % 5;

  assert(matrix_info);
  assert(matrix_info->addr);

  if(matrix_info->datatype==MATRIX_DATATYPE_FLOAT32)
  {
    for(j=0; j<matrix_info->num_row; j++)
      for(i=0; i<matrix_info->num_col; i++)
      {
        float value;
        switch(index2)
        {
          case 0:
            if((i&3)==0)
              value = ((float)(j*((int)matrix_info->num_col) + i + 1) - 8.0f) * 4355;
            else
              value = (float)(j*((int)matrix_info->num_col) + i + 1) - 8.0f;
            break;
          case 1:
            if((i&1)==0)
              value = ((float)(i*((int)matrix_info->num_col) + j + 1) - 6.0f)/(7.0f*1024.0f*1024.0f*256.0f);
            else
              value = ((float)(i*((int)matrix_info->num_col) + j + 1) - 6.0f)/3120312;
            break;
          case 2:
            value = ((float)(i*((int)matrix_info->num_col) + j + 1) - 6.0f)/7.0f;
            break;
          case 3:
            value = (float)((int)((j*((int)matrix_info->num_col) + i + 1)>>3) - 8);
            break;
          case 4:
            value = (float)((int)((j*((int)matrix_info->num_col) + i + 1)&7) - 4);
            break;
          default:
            value = 0;
            assert(0);
        }
        matrix_write_float_element(matrix_info, j, i, value);
      }
  }
  else
  {
    for(j=0; j<matrix_info->num_row; j++)
      for(i=0; i<matrix_info->num_col; i++)
      {
        int value;
        switch(index2)
        {
          case 0:
            value = j*matrix_info->num_col + i + 1 - 4;
            break;
          case 1:
            value = j*matrix_info->num_col + i + 8;
            break;
          case 2:
            value = i*matrix_info->num_col + j - 6;
            break;
          case 3:
            value = ((j*matrix_info->num_col + i + 1)>>3) - 8;
            break;
          case 4:
            value = ((j*matrix_info->num_col + i + 1)&7) - 4;
            break;
          default:
            value = 0;
            assert(0);
        }
        if(invert==2)
        {
          value = -value;
          invert = 0;
        }
        else
          invert++;
        matrix_write_fixed_element(matrix_info, j, i, value);
      }
  }
}