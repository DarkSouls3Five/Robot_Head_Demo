#ifndef _MATH_LIB_H_
#define _MATH_LIB_H_
#define PI 3.14159265358979f
#include "struct_typedef.h"


#define f_Lx 16.0f                     //两轮轴之间距离的一半            cm
#define f_Ly 18.2f                     //两轮毂之间距离的一半            cm
#define f_R 7.7f                       //麦轮半径                       cm
#define f_GimbalDistance 0.3f          //云台位置相对底盘中心向前偏移量  cm



//限幅函数，将输入值控制在一个区间,短整型(min,max)
void fn_Uint16Limit(uint16_t *input,uint16_t min,uint16_t max);

//限幅函数，将输入值控制在一个区间,浮点型(min,max)
void fn_Fp32Limit(fp32 *input,fp32 min,fp32 max);

//循环限幅函数，将输入数转换到一个区间,短整型(minValue,maxValue)
uint16_t fn_Uint16LoopLimit(uint16_t Input,uint16_t minValue,uint16_t maxValue);

//循环限幅函数，将输入数转换到一个区间,浮点数(minValue,maxValue)
fp32 fn_Fp32LoopLimit(fp32 Input,fp32 minValue,fp32 maxValue);

//弧度转换(-pi,pi)
fp32 fn_RadFormat(fp32 rad);

//平方根倒数
float fn_InvSqrt(float x);

//绝对限制
void abs_limit(fp32 *num, fp32 Limit);

//范围判断函数 在范围内则返回0 超出范围则返回1
bool_t fn_scope_judgment(fp32 angle,fp32 min_angle,fp32 max_angle);

int float_to_uint(float x, float x_min, float x_max, int bits);

float uint_to_float(int x_int, float x_min, float x_max, int bits);

#endif
