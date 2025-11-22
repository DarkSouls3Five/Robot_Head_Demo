/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       pid.c/h
  * @brief      pid实现函数，包括初始化，PID计算函数，
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef PID_H
#define PID_H
#include "struct_typedef.h"
enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct
{
    uint8_t mode;
    //PID 三参数
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  //最大输出
    fp32 max_iout; //最大积分输出

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  //微分项 0最新 1上一次 2上上次
    fp32 error[3]; //误差项 0最新 1上一次 2上上次

} pid_type_def;

extern void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);

extern fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set);

extern void PID_clear(pid_type_def *pid);


//PID_NEW
typedef struct{
  //coefficient
	fp32 f_Kp;
	fp32 f_Ki;
	fp32 f_Kd;
	
	//error
	fp32 f_Error;       //误差
	fp32 f_ErrorAngle; //
	fp32 f_LastError;  // 上一次误差值
	
	//pid limitation
	fp32 f_MinIout;
	fp32 f_MaxIout;
	fp32 f_MinOut;
	fp32 f_MaxOut;
	
	//real and target
	fp32 f_Set;
	fp32 f_Fdb;
	
	//pid out
	fp32 f_Out;
	fp32 f_Pout;
	fp32 f_Iout;
	fp32 f_Dout;
	
}pid_type_def_new;


//PID初始化
void fn_PidInit(pid_type_def_new *pid,const fp32 PID[3],fp32 min_out,fp32 max_out,fp32 min_iout,fp32 max_iout);

//PID计算
fp32 fn_PidClac(pid_type_def_new *pid,fp32 ref,fp32 set);

fp32 fn_PidClacAngle(pid_type_def_new *pid,fp32 ref,fp32 set);


#endif
