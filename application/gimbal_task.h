#ifndef _GIMBAL_TASK_H_
#define _GIMBAL_TASK_H_

#include "pid.h"
#include "struct_typedef.h"
#include "can_task.h"

//ENCODE控制模式
//yaw轴角度双环外环
#define GimbalMotor6020PosPid1_Yaw_kp 20.0f
#define GimbalMotor6020PosPid1_Yaw_ki 0.0f
#define GimbalMotor6020PosPid1_Yaw_kd 0.0f
//yaw轴角度双环内环
#define GimbalMotor6020PosPid2_Yaw_kp 2000.0f
#define GimbalMotor6020PosPid2_Yaw_ki 0.0f
#define GimbalMotor6020PosPid2_Yaw_kd 300.0f
//pitch轴角度双环外环
#define GimbalMotor6020PosPid1_Pit_kp 0.6f
#define GimbalMotor6020PosPid1_Pit_ki 0.0f
#define GimbalMotor6020PosPid1_Pit_kd 0.0f
//pitch轴角度双环内环
#define GimbalMotor6020PosPid2_Pit_kp 1.0f
#define GimbalMotor6020PosPid2_Pit_ki 0.0f
#define GimbalMotor6020PosPid2_Pit_kd 0.0f
//PID输出限制
//大疆6020
#define GimbalMotor6020MinOut -10000.0f
#define GimbalMotor6020MaxOut 10000.0f
#define GimbalMotor6020MinIOut -5000.0f
#define GimbalMotor6020MaxIOut 5000.0f
//DM4310
#define GimbalMotor4310MinOut -10.0f
#define GimbalMotor4310MaxOut 10.0f
#define GimbalMotor4310MinIOut -5.0f
#define GimbalMotor4310MaxIOut 5.0f

//GYRO控制模式

// yaw外10-30
// 
// pitch看哨兵外10 内0.1


//Yaw陀螺仪双环外环pid参数
#define GimbalPid1Yaw_kp 80.0f
#define GimbalPid1Yaw_ki 0.0f
#define GimbalPid1Yaw_kd 4000.0f

#define GimbalPid2Yaw_kp 3000.0f
#define GimbalPid2Yaw_ki 0.0f
#define GimbalPid2Yaw_kd 0.0f

//Pitch陀螺仪双环外环pid参数
#define GimbalPid1Pitch_kp 15.0f
#define GimbalPid1Pitch_ki 0.0f
#define GimbalPid1Pitch_kd 500.0f

#define GimbalPid2Pitch_kp 1.2f
#define GimbalPid2Pitch_ki 0.0f
#define GimbalPid2Pitch_kd 0.0f

//陀螺仪双环内环最大输出
//GM6020
#define GimbalPidMinOut_Yaw -30000.0f
#define GimbalPidMaxOut_Yaw 30000.0f
#define GimbalPidMinIOut_Yaw -1000.0f
#define GimbalPidMaxIOut_Yaw 1000.0f

//DM4310
#define GimbalPidMinOut_Pit -10.0f
#define GimbalPidMaxOut_Pit 10.0f
#define GimbalPidMinIOut_Pit -0.01f
#define GimbalPidMaxIOut_Pit 0.01f

//遥控器模式云台数据
#define WMax 0.004f                    //rad/ms
#define PitAngleMax 0.32               //Pitch轴限位    最大角度
#define PitAngleMin -0.39              //Pitch轴限位    最小角度
#define YawAngleMax 1.3                //Yaw轴限位    最大角度
#define YawAngleMin -1.1               //Yaw轴限位    最小角度


//遥控器模式底盘数据
#define WCoef 0.000075f
#define low_W 0.0003f                   //QE微调速度


//云台状态机参数
typedef enum{
    GIMBAL_ZERO_FORCE,
    GIMBAL_INIT,
    GIMBAL_GYRO,
    GIMBAL_ENCONDE,
    GIMBAL_AUTO,

}gimbal_behaviour_e;

//云台电机控制模式
typedef enum{

  GIMBAL_Motor_DOWN,    // 电机电流发零
	GIMBAL_MOTOR_GYRO,	  // 电机陀螺仪角度控制
	GIMBAL_MOTOR_ENCONDE, // 电机编码值角度控制

} gimbal_motor_mode_e;

typedef struct{
	
    gimbal_behaviour_e gimbal_behaviour;
    gimbal_behaviour_e last_gimbal_behaviour;

    gimbal_motor_mode_e gimbal_motor_yaw_mode;
    gimbal_motor_mode_e gimbal_motor_pit_mode;

    pid_type_def GimbalIMUYawPid1;                       //Yaw PID控制陀螺仪外环数据
    fp32 f_GimbalYawPidMid;
    pid_type_def GimbalIMUYawPid2;                       //Yaw PID控制陀螺仪内环数据

    pid_type_def GimbalIMUPitPid1;                       //Pit PID控制陀螺仪外环数据
    fp32 f_GimbalPitPidMid;
    pid_type_def GimbalIMUPitPid2;                       //Pit PID控制陀螺仪内环数据

    fp32 gyro_yaw_angle_add;                             //Yaw轴控制角度增量     rad
    fp32 gyro_pit_angle_add;                             //Pitch轴控制角度增量   rad

    fp32 gyro_yaw_target_angle;
    fp32 gyro_pit_target_angle;


}gimbal_data_t;

extern gimbal_data_t gimbal_data;


#endif
