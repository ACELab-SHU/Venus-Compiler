/*
 * @Author: YihaoShen
 * @LastEditTime: 2024-10-22 23:09:13
 * Copyright (c) 2024 by ACE_Lab, All Rights Reserved.
 */
#include "data_type.h"
#include "riscv_printf.h"
#include "venus.h"
#include <stdint.h>

typedef short __v2048i16 __attribute__((ext_vector_type(2048)));
typedef char __v4096i8 __attribute__((ext_vector_type(4096)));

VENUS_INLINE __v2048i16 gemm_venus(__v2048i16 a, __v2048i16 b, __v2048i16 index_a, __v2048i16 index_b, short m, short n, short k, short outlength)
{
    __v2048i16 result;
    vbrdcst(result, 0, MASKREAD_OFF, outlength);
    __v2048i16 temp_1;
    __v2048i16 temp_2;
    vclaim(temp_1);
    vclaim(temp_2);
    __v2048i16 temp_result;
    vclaim(temp_result);

    __v2048i16 index_a_i;
    __v2048i16 index_b_i;
    for (int i = 0; i < n; i++)
    {
        index_a_i = vadd(index_a, i, MASKREAD_OFF, outlength);
        index_b_i = vadd(index_b, i * k, MASKREAD_OFF, outlength);
        vshuffle(temp_1, index_a_i, a, SHUFFLE_GATHER, outlength);
        vshuffle(temp_2, index_b_i, b, SHUFFLE_GATHER, outlength);
        temp_result = vmul(temp_1, temp_2, MASKREAD_OFF, outlength);
        result = vadd(result, temp_result, MASKREAD_OFF, outlength);
    }

    return result;
};

int Matrix_test_a[12] = {1, 2, 3, 4, 11, 12, 13, 14, 21, 22, 23, 24};                                // 3x4
int Matrix_test_b[20] = {1, 2, 3, 4, 5, 11, 12, 13, 14, 15, 21, 22, 23, 24, 25, 31, 32, 33, 34, 35}; // 4x5

int main()
{
    // Read in Test Matrix
    __v2048i16 Matrix_a;
    __v2048i16 Matrix_b;
    vclaim(Matrix_a);
    vclaim(Matrix_b);
    vbarrier();
    VSPM_OPEN();
    int Matrix_a_addr = vaddr(Matrix_a);
    for (int i = 0; i < 12; i++)
    {
        *(volatile unsigned short *)(Matrix_a_addr + (i << 1)) = Matrix_test_a[i];
    }
    int Matrix_b_addr = vaddr(Matrix_b);
    for (int i = 0; i < 20; i++)
    {
        *(volatile unsigned short *)(Matrix_b_addr + (i << 1)) = Matrix_test_b[i];
    }
    VSPM_CLOSE();

    //  Dimension of the matrix m,n,k
    short m = 3;
    short n = 4;
    short k = 5;

    // genrate shuffle_index_a
    short out_length = m * k;
    short shuffle_index_a[out_length];
    short shuffle_index_b[out_length];
    for (int i = 0; i < m; i++)
    {
        for (int j = 0; j < k; j++)
        {
            shuffle_index_a[i * k + j] = n * i;
        }
    }

    for (int i = 0; i < m; i++)
    {
        for (int j = 0; j < k; j++)
        {
            shuffle_index_b[i * k + j] = j;
        }
    }

    // Put data into vector scratchpad memory
    __v2048i16 index_a;
    __v2048i16 index_b;
    vclaim(index_a);
    vclaim(index_b);
    vbarrier();
    VSPM_OPEN();
    int index_a_addr = vaddr(index_a);
    for (int i = 0; i < out_length; i++)
    {
        *(volatile unsigned short *)(index_a_addr + (i << 1)) = shuffle_index_a[i];
    }
    int index_b_addr = vaddr(index_b);
    for (int i = 0; i < out_length; i++)
    {
        *(volatile unsigned short *)(index_b_addr + (i << 1)) = shuffle_index_b[i];
    }
    VSPM_CLOSE();

    // GEMM kernel
    __v2048i16 output_data;
    output_data = gemm_venus(Matrix_a, Matrix_b, index_a, index_b, m, n, k, out_length);

    return 0;
}