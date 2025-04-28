/**
 * ****************************************
 * @file        Task_inputSpilt.c
 * @brief       split input data(real & imag)
 * @author      yuanfeng
 * @date        2024.6.19
 * @copyright   ACE-Lab(Shanghai University)
 * ****************************************
 */

 #include "riscv_printf.h"
 #include "venus.h"
 
 typedef char  __v8192i8 __attribute__((ext_vector_type(8192)));
 typedef short __v4096i16 __attribute__((ext_vector_type(4096)));
 typedef char  __v4096i8 __attribute__((ext_vector_type(4096)));

 
 int symbolLength = 2192;
 
 int Task_test_float(__v4096i8 testdata) {
    __v4096i8 result;
    vclaim(result);
    __v4096i16 result_1;
    vclaim(result_1);
    
    testdata = vadd(testdata,0,MASKREAD_OFF,10);

    //__v4096i8 temp;
    //vclaim(temp);    
    
    //复数乘运算
    __v4096i8 tempWnResult_real;//a
    vclaim(tempWnResult_real);
    __v4096i8 tempWnResult_imag;//b
    vclaim(tempWnResult_imag);
    __v4096i8 sin_stage0;//d
    vclaim(sin_stage0);
    __v4096i8 cos_stage0;//c    
    vclaim(cos_stage0); 

    vbrdcst(tempWnResult_real,1,MASKREAD_OFF,10);
    vbrdcst(tempWnResult_imag,2,MASKREAD_OFF,10);
    vbrdcst(sin_stage0,3,MASKREAD_OFF,10);
    vbrdcst(cos_stage0,4,MASKREAD_OFF,10);

    __v4096i8* cmxreal_part = &tempWnResult_real;
    __v4096i8* cmximag_part = &tempWnResult_imag;


   vcmxmul(cmximag_part,cmxreal_part,tempWnResult_real,tempWnResult_imag,sin_stage0,cos_stage0,MASKREAD_OFF,10);
   // result = vsadd(tempWnResult_imag,0,MASKREAD_OFF, 10);


    //result_1 = vrange(result_1, 10);//vrange要i16
    //result = vsadd(temp,1,MASKREAD_OFF, 15);
    //result = vmul(result,temp,MASKREAD_OFF, 10);//6
    //result = vmul(result,100,MASKREAD_OFF, 10);//600
    //result = vdiv(result,43,MASKREAD_OFF, 10);//
    //result = vrem(result,1230,MASKREAD_OFF, 10);//
    //result = vseq(result,result_1, MASKREAD_OFF, MASKWRITE_OFF, 256);//有返回值是I型
    //vsgt(result, 1, MASKREAD_OFF, MASKWRITE_ON, 15);//无返回值是M型
    //result = vsadd(result,1,MASKREAD_ON, 15);

    
 
    vreturn(result,4096);
    //vreturn(result_1,4096);
 }
 