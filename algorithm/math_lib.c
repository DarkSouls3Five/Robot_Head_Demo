#include "math_lib.h"
#include "struct_typedef.h"
#include "math.h"
#include "can_task.h"
#include "ins_task.h"


//限幅函数，将输入值控制在一个区间,短整型
void fn_Uint16Limit(uint16_t *input,uint16_t min,uint16_t max){
  if(*input > max){
	  *input = max;
	}
	else if(*input < min){
	  *input = min;
	}
}

//限幅函数，将输入值控制在一个区间,浮点型
void fn_Fp32Limit(fp32 *input,fp32 min,fp32 max){
  if(*input > max){
	  *input = max;
	}
	else if(*input < min){
	  *input = min;
	}
}

//循环限幅函数，将输入数转换到一个区间,短整型
uint16_t fn_Uint16LoopLimit(uint16_t Input,uint16_t minValue,uint16_t maxValue){
     if (maxValue < minValue)
    {
        return Input;
    }

    if (Input > maxValue)
    {
        uint16_t len = maxValue - minValue;
        while (Input > maxValue)
        {
            Input -= len;
        }
    }
    else if (Input < minValue)
    {
        uint16_t len = maxValue - minValue;
        while (Input < minValue)
        {
            Input += len;
        }
    }
    return Input;
}

//循环限幅函数，将输入数转换到一个区间,浮点数
fp32 fn_Fp32LoopLimit(fp32 Input,fp32 minValue,fp32 maxValue){
     if (maxValue < minValue)
    {
        return Input;
    }

    if (Input > maxValue)
    {
        fp32 len = maxValue - minValue;
        while (Input > maxValue)
        {
            Input -= len;
        }
    }
    else if (Input < minValue)
    {
        fp32 len = maxValue - minValue;
        while (Input < minValue)
        {
            Input += len;
        }
    }
    return Input;
}

//弧度转换
fp32 fn_RadFormat(fp32 rad){
  if(rad > PI){
	  while(rad > PI){
		  rad -= 2*PI;
		}
	}
	if(rad < -PI){
	  while(rad < -PI){
		  rad += 2*PI;
		}
	}
	return rad;
}

//平方根倒数
float fn_InvSqrt(float x){
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

//绝对限制
void abs_limit(fp32 *num, fp32 Limit)
{
    if (*num > Limit)
    {
        *num = Limit;
    }
    else if (*num < -Limit)
    {
        *num = -Limit;
    }
}
//范围判断函数 在范围内则返回0 超出范围则返回1
bool_t fn_scope_judgment(fp32 angle,fp32 min_angle,fp32 max_angle){
    if(min_angle < angle && angle < max_angle){
        return 0;
    }
    return 1;
}


int float_to_uint(float x, float x_min, float x_max, int bits){
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
    }
    
    
float uint_to_float(int x_int, float x_min, float x_max, int bits){
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
    }

