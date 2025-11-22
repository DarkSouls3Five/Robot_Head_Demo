#ifndef _CAN_TASK_H_
#define _CAN_TASK_H_

#include "struct_typedef.h"
#include "pid.h"


#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -10.0f
#define T_MAX 10.0f

//电机ID设置
typedef enum
{
		CAN_PIT_MOTOR_ID = 0x201,
    CAN_YAW_MOTOR_ID = 0x15,
	
    CAN_3508_FRICR_ID = 0x201,
    CAN_3508_FRICL_ID = 0x202,

    CAN_TRIGGER_MOTOR_ID = 0x203,

    CAN_AUTOAIM_ID = 0x0FF,

} can1_msg_id_e;

//达妙电机数据
typedef struct{
    int32_t p_int;
    int32_t v_int;
    int32_t t_int;
    float position;
    float velocity;
    float torque;

    fp32 offecd_angle;
    fp32 target_angle;

    pid_type_def motor_pid1;
    fp32 double_pid_mid;
    pid_type_def motor_pid2;


}DM_motor_measure_t;

//大疆电机原始数据
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;       //此电流与其余数据同步
    uint8_t temperate;
    int16_t last_ecd;

} MotorMeasure_t;

//大疆电机原始数据
typedef struct
{

	uint16_t ecd;
	uint16_t init_ecd;
	int16_t speed_rpm;
	int16_t given_current;
	uint8_t temperate;
	uint16_t last_ecd;
	int16_t ecd_count;
	int16_t round;
	fp32 distance;
	
} motor_measure_t;

//自瞄数据
typedef struct
{
    char vision_state;
    bool_t shoot;
    float yaw;
    float pitch;

} autoaim_measure_t;



//电机解算数据
//3508
typedef struct{
	
    int32_t round_num;                //转子多圈编码

    fp32 relative_raw_angle;          //转轴根据最新原始数据解算出的未滤波角度
    fp32 raw_angle[6];                //转轴相对于offecd.ecd的角度（弧度制）（未滤波）  数据：（新--旧，[0]--[5]）
    fp32 filter_angle[2];             //滤波后的当前角度                               数据：（新--旧，[0]--[2]）

    fp32 relative_raw_speed;          //转轴根据最新原始速度解算出的未滤波速度
    fp32 raw_speed[6];                //未滤波转轴的速度                                 数据：（新--旧，[0]--[5]）
    fp32 filter_speed[2];             //滤波后的当前速度                                 数据：（新--旧，[0]--[2]）

    uint16_t offecd_ecd;              //舵轮指向正前方时候的码盘值
    fp32 target_angle;                //需要转轴转到的角度
    fp32 target_speed;
    fp32 filter_given_current;        //滤波后的返回的电流
    fp32 given_current;               //发送给电机的电流值  范围 [-16384,16384]

    pid_type_def motor_pid1;          //双环外环PID参数
    fp32 double_pid_mid;              //双环PID的中间变量
		pid_type_def motor_pid2;          //双环内环PID参数

    pid_type_def motor_pid3;          //单速度环PID参数
	
}Motor3508Data_t;

//电机解算数据
//2006
typedef struct{
	
    int32_t round_num;                //转子多圈编码

    fp32 relative_raw_angle;          //转轴根据最新原始数据解算出的未滤波角度
    fp32 raw_angle[6];                //转轴相对于offecd.ecd的角度（弧度制）（未滤波）  数据：（新--旧，[0]--[5]）
    fp32 filter_angle[2];             //滤波后的当前角度                               数据：（新--旧，[0]--[2]）

    fp32 relative_raw_speed;          //转轴根据最新原始速度解算出的未滤波速度
    fp32 raw_speed[6];                //未滤波转轴的速度                                 数据：（新--旧，[0]--[5]）
    fp32 filter_speed[2];             //滤波后的当前速度                                 数据：（新--旧，[0]--[2]）

    uint16_t offecd_ecd;              //舵轮指向正前方时候的码盘值
    fp32 target_angle;                //需要转轴转到的角度
    fp32 target_speed;
    fp32 filter_given_current;        //滤波后的返回的电流
    fp32 given_current;               //发送给电机的电流值  范围 [-16384,16384]

    pid_type_def motor_pid1;          //双环外环PID参数
    fp32 double_pid_mid;              //双环PID的中间变量
		pid_type_def motor_pid2;          //双环内环PID参数

    pid_type_def motor_pid3;          //单速度环PID参数
	
}Motor2006Data_t;
//6020
typedef struct{
	
    //int32_t round_num;
    fp32 relative_raw_angle;          //转轴根据最新原始数据解算出的未滤波角度
    fp32 raw_angle[6];      //转轴相对于offecd.ecd的角度（弧度制）（未滤波）  数据：（新--旧，[0]--[5]）
    fp32 filter_angle[2];   //滤波后的当前角度                               数据：（新--旧，[0]--[2]）

		fp32 relative_raw_speed;          //转轴的速度

		uint16_t offecd_ecd;            //电机指向云台正前方的绝对码盘值
		fp32 target_angle;               //转轴需要转到的角度
		fp32 given_voltage;              //发送给电机的电流值  范围 [-30000,30000]

    pid_type_def motor_pid1;          //双环外环PID参数
    fp32 double_pid_mid;              //双环PID的中间变量
    pid_type_def motor_pid2;          //双环内环PID参数

	//pid_type_def motor_pid_3;         //单速度环PID参数
	
}Motor6020Data_t;



//extern MotorMeasure_t gimbal_motor6020_measure[1];
//extern MotorMeasure_t trigger_motor2006_measure[1];
//extern MotorMeasure_t gimbal_motor3508_measure[2];

//extern Motor6020Data_t gimbal_motor6020_data[1];
//extern Motor3508Data_t trigger_motor2006_data[1];
//extern Motor3508Data_t gimbal_motor3508_data[2];
//extern DM_motor_measure_t gimbal_DM_data[1];
extern DM_motor_measure_t YAW_DM_data;
extern motor_measure_t gimbal_motor2006_measure;	

//extern autoaim_measure_t autoaim_measure;


//达妙电机使能
void fn_DM_start_motor(void);

//达妙电机保存零点
void fn_DM_record_init_state(void);

//达妙电机发送函数
void fn_ctrl_DM_motor(float _pos, float _vel, float _KP, float _KD, float _torq);

//can1电流发送函数
void fn_cmd_CAN1GimbalMotor(int16_t motor1);

//can2电流发送函数
void fn_cmd_CAN2GimbalMotor1(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

////拨弹轮电机电流发送函数
//void fn_cmd_CAN2GimbalMotor2(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8);

////四元数发送函数  CAN1
//void fn_cmd_quat_to_computer(fp32 x, fp32 y, fp32 z, fp32 w);

////射击状态发送函数  CAN1
//void fn_cmd_shoot_data_to_computer(fp32 speed, char mode);

////云台6020电机数据解算
//void fn_GimbalMotor6020Data(uint8_t i);

////云台3508拨弹轮数据解算
//void fn_TriggerMotor2006Data(uint8_t i);

////云台3508摩擦轮数据解算
//void fn_GimbalMotor3508Data(uint8_t i);

//2006初始ecd记录
void init_ecd_record(motor_measure_t *motor_2006);


#endif
