#ifndef _MOVE_TASK_H_
#define _MOVE_TASK_H_

#include "pid.h"
#include "struct_typedef.h"
#include "can_task.h"

#define PI					3.14159265358979f

//Pit复位速度
#define PIT_RST_SPEED 500.0f

//电机中值
#define MID_YAW_ANGLE 2.17f
#define MID_PIT_ANGLE -40.0f

//角度限幅
#define MAX_YAW_ANGLE MID_YAW_ANGLE+1.4f
#define MIN_YAW_ANGLE MID_YAW_ANGLE-1.4f


#define MIN_PIT_ANGLE -85.0f
#define MAX_PIT_ANGLE 5.0f

//PS2遥控器灵敏度系数
#define PS2_Coef_Yaw 0.005f
#define PS2_Coef_Pit 0.2f

//M2006 PID
//速度环
#define PIT_MOTOR_SPEED_PID_KP 8.0f
#define PIT_MOTOR_SPEED_PID_KI 0.5f
#define PIT_MOTOR_SPEED_PID_KD -0.05f

#define PIT_MOTOR_SPEED_PID_MAX_OUT 6000.f
#define PIT_MOTOR_SPEED_PID_MAX_IOUT 600.0f
//位置环
#define PIT_MOTOR_ANGLE_PID_KP 150.0f
#define PIT_MOTOR_ANGLE_PID_KI 1.0f
#define PIT_MOTOR_ANGLE_PID_KD -2.0f

#define PIT_MOTOR_ANGLE_PID_MAX_OUT 5000.0f
#define PIT_MOTOR_ANGLE_PID_MAX_IOUT 5.0f

//锁死模式
#define PIT_MOTOR_LOCK_PID_KP 600.0f
#define PIT_MOTOR_LOCK_PID_KI 5.0f
#define PIT_MOTOR_LOCK_PID_KD -2.0f

#define PIT_MOTOR_LOCK_PID_MAX_OUT 5000.0f
#define PIT_MOTOR_LOCK_PID_MAX_IOUT 200.0f

//陀螺仪模式
#define PIT_MOTOR_GYRO_PID_KP 350.0f//120.0f
#define PIT_MOTOR_GYRO_PID_KI 0.0f//1.0f
#define PIT_MOTOR_GYRO_PID_KD -30.0f//-25.0f

#define PIT_MOTOR_GYRO_PID_MAX_OUT 5000.0f
#define PIT_MOTOR_GYRO_PID_MAX_IOUT 200.0f

//DM4310 PID
//MIT模式位置控制pid
#define DM4310_MIT_P_KP 2.5f
#define DM4310_MIT_P_KD 1.5f
#define DM4310_MIT_P_KI 1.5f

//yaw轴角度双环外环
#define DM4310PosPid1_Yaw_kp 0.6f
#define DM4310PosPid1_Yaw_ki 0.0f
#define DM4310PosPid1_Yaw_kd 0.0f
//yaw轴角度双环内环
#define DM4310PosPid2_Yaw_kp 3.0f
#define DM4310PosPid2_Yaw_ki 0.0f
#define DM4310PosPid2_Yaw_kd 0.01f


//PID输出限制
//DM4310
#define DM4310MinOut -5.0f
#define DM4310MaxOut 5.0f
#define DM4310MinIOut -0.01f
#define DM4310MaxIOut 0.01f





//头部pid参数
typedef struct
{
    fp32 kp;
    fp32 ki;
    fp32 kd;

    fp32 set;
    fp32 get;
    fp32 err;
		fp32 last_err;
	
    fp32 max_out;
    fp32 max_iout;
		fp32 max_dout;
	
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;

    fp32 out;
		fp32 dT;
} move_PID_t;

//头部状态机参数
typedef enum{
    MOVE_FREE,
    MOVE_WORK,	//遥控器控制
		MOVE_GYRO,	//陀螺仪控制
		MOVE_NOD,		//点头
		MOVE_SHAKE,	//摇头
		MOVE_SCAN_ALL,//全方位扫描
		MOVE_LOC		//声源定位

}move_mode_e;

//头部电机控制模式
typedef enum{

  MOVE_MOTOR_DOWN,    // 电机电流发零
	MOVE_MOTOR_GYRO,	  // 电机陀螺仪角度控制
	MOVE_MOTOR_ENCONDE, // 电机编码值角度控制
	MOVE_MOTOR_WAVE,			//电机来回旋转扫描
	MOVE_MOTOR_SCAN,			//电机同时扫描	
	MOVE_MOTOR_LOC,			//电机声源定位		
	MOVE_MOTOR_RST				//电机复位


} move_motor_mode_e;

//头部大疆电机
typedef struct
{
  const motor_measure_t *move_motor_measure;
	const DM_motor_measure_t *DM_motor_measure;
  fp32 motor_angle;     //rad
  fp32 motor_angle_set; //rad
	fp32 motor_ecd;     //rad
  fp32 motor_ecd_set; //rad
  fp32 motor_speed;
	fp32 motor_speed_set;
  int16_t give_current;
	
	//大疆电机pid参数
	move_PID_t move_angle_pid;
	move_PID_t move_lock_pid;
	move_PID_t move_gyro_pid;
	pid_type_def move_speed_pid;             
	
	//达妙电机MIT模式参数
	float DM_target_torque;
	float DM_kp;	
	float DM_kd;		
	
	//gyro模式目标角度
	fp32 gyro_angle;     //rad
  fp32 gyro_angle_set; //rad
	
} move_motor_t;

//头部数据结构体
typedef struct{
	
		//电机数据
		move_motor_t pit_motor_data;	
		move_motor_t yaw_motor_data;	
	
		//状态机
    move_mode_e move_mode;
    move_mode_e last_move_mode;
	
		//电机运动模式
    move_motor_mode_e move_motor_yaw_mode;
    move_motor_mode_e move_motor_pit_mode;
		

}move_data_t;



#endif
