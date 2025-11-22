#ifndef _SHOOT_TASK_H_
#define _SHOOT_TASK_H_

#include "struct_typedef.h"

//拨弹轮2006电机PID参数
//单速度环参数
#define TriggerMotor2006PosPid3_ID207_kp 300.0f
#define TriggerMotor2006PosPid3_ID207_ki 1.0f
#define TriggerMotor2006PosPid3_ID207_kd 30.0f



#define TriggerMotor2006SpeedMinOut -100.0f
#define TriggerMotor2006SpeedMaxOut 100.0f
#define TriggerMotor2006SpeedMinIOut -20.0f
#define TriggerMotor2006SpeedMaxIOut 20.0f

#define TriggerMotor2006MinOut -16000.0f
#define TriggerMotor2006MaxOut 16000.0f
#define TriggerMotor2006MinIOut -2000.0f
#define TriggerMotor2006MaxIOut 2000.0f

//摩擦轮3508电机PID参数
//单速度环PID参数
#define GimbalMotor3508PosPid3_ID201_kp 20.0f
#define GimbalMotor3508PosPid3_ID201_ki 0.0f
#define GimbalMotor3508PosPid3_ID201_kd 0.2f

#define GimbalMotor3508PosPid3_ID202_kp 20.0f
#define GimbalMotor3508PosPid3_ID202_ki 0.0f
#define GimbalMotor3508PosPid3_ID202_kd 0.2f

#define GimbalMotor3508MinOut -16000.0f
#define GimbalMotor3508MaxOut 16000.0f
#define GimbalMotor3508MinIOut -1000.0f
#define GimbalMotor3508MaxIOut 1000.0f

//摩擦轮转速rad/s
#define FricSpeed 770.0f
//射击时拨弹轮转速
#define TriggerSpeed 22.5f

typedef enum{
    
    SHOOT_DOWN,                 //摩擦轮未打开，射击模块down
    SHOOT_READY_COUNTINUE,      //摩擦轮已打开，连发模式

}shoot_mode_e;

typedef enum{

		FRIC_OFF = 0,
		FRIC_ON = 1,
		FRIC_DOWN = 2,

}fric_state_e;

typedef struct{

    shoot_mode_e shoot_mode;

    fric_state_e fric_state;

    uint16_t fric_speed;

    uint8_t fric_state_change_flag;           //为1则可以改变摩擦轮状态，为0则不行，为了防止左摇杆一直放在上面导致状态一直变

}shoot_data_t;

extern shoot_data_t shoot_data;


//拨弹轮初始化
void fn_ShootMotorInit(void);
//摩擦轮初始化
void fn_shoot_motor3508_init(void);
//射击模块状态初始化
void fn_shoot_init(void);
//射击模块模式选择
void fn_fric_state(void);
//射击模块电流解算
void fn_ShootMove(void);

#endif
