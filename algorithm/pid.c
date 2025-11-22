/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
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
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "pid.h"
#include "main.h"
#include "math_lib.h"

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

/**
  * @brief          pid struct data init
  * @param[out]     pid: PID struct data point
  * @param[in]      mode: PID_POSITION: normal pid
  *                 PID_DELTA: delta pid
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid max out
  * @param[in]      max_iout: pid max iout
  * @retval         none
  */
/**
  * @brief          pid struct data init
  * @param[out]     pid: PID结构数据指针
  * @param[in]      mode: PID_POSITION:普通PID
  *                 PID_DELTA: 差分PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid最大输出
  * @param[in]      max_iout: pid最大积分输出
  * @retval         none
  */
void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

/**
  * @brief          pid calculate 
  * @param[out]     pid: PID struct data point
  * @param[in]      ref: feedback data 
  * @param[in]      set: set point
  * @retval         pid out
  */
/**
  * @brief          pid计算
  * @param[out]     pid: PID结构数据指针
  * @param[in]      ref: 反馈数据
  * @param[in]      set: 设定值
  * @retval         pid输出
  */
fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    return pid->out;
}

/**
  * @brief          pid out clear
  * @param[out]     pid: PID struct data point
  * @retval         none
  */
/**
  * @brief          pid 输出清除
  * @param[out]     pid: PID结构数据指针
  * @retval         none
  */
void PID_clear(pid_type_def *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}

//另一个类型pid相关
//PID初始化 此处最小值代表负方向的最大值
void fn_PidInit(pid_type_def_new *pid,const fp32 PID[3],fp32 min_out,fp32 max_out,fp32 min_iout,fp32 max_iout){
  if (pid == NULL || PID == NULL){
	  return;
	}
  pid->f_Kp = PID[0];
	pid->f_Ki = PID[1];
	pid->f_Kd = PID[2];
  pid->f_MinOut = min_out;
	pid->f_MaxOut = max_out;
  pid->f_MinIout = min_iout;
	pid->f_MaxIout = max_iout;
	pid->f_ErrorAngle = 0.0f;
	pid->f_Error = 0.0f;
	pid->f_LastError = 0.0f;
	pid->f_Iout = 0;
	pid->f_Dout = 0;
	pid->f_Pout = 0;
	pid->f_Out = 0;
}

//PID计算
fp32 fn_PidClac(pid_type_def_new *pid,fp32 ref,fp32 set){
	
    pid->f_Set = set;
	pid->f_Fdb = ref;
	pid->f_Error = set-ref;

	pid->f_Pout = pid->f_Kp * pid->f_Error;
	pid->f_Iout += pid->f_Ki * pid->f_Error;
	pid->f_Dout = pid->f_Kd * (pid->f_Error - pid->f_LastError);
	pid->f_LastError = pid->f_Error;
	
	fn_Fp32Limit(&pid->f_Iout,pid->f_MinIout,pid->f_MaxIout);
	
	pid->f_Out=pid->f_Pout + pid->f_Iout + pid->f_Dout;
	
	fn_Fp32Limit(&pid->f_Out,pid->f_MinOut,pid->f_MaxOut);
	
	return pid->f_Out;
	
}

//旋转取劣弧PID
fp32 fn_PidClacAngle(pid_type_def_new *pid,fp32 ref,fp32 set){
	
  pid->f_Set = set;
	pid->f_Fdb = ref;
	pid->f_Error = fn_RadFormat(set-ref);       //弧度转换函数
	pid->f_ErrorAngle = pid->f_Error*180/PI;   //jiao du

	pid->f_Pout = pid->f_Kp * pid->f_Error;
	pid->f_Iout += pid->f_Ki * pid->f_Error;
	pid->f_Dout = pid->f_Kd * (pid->f_Error - pid->f_LastError);
	pid->f_LastError = pid->f_Error;
	
	fn_Fp32Limit(&pid->f_Iout,pid->f_MinIout,pid->f_MaxIout);
	
	pid->f_Out=pid->f_Pout + pid->f_Iout + pid->f_Dout;
	
	fn_Fp32Limit(&pid->f_Out,pid->f_MinOut,pid->f_MaxOut);
	
	return pid->f_Out;
	
}
