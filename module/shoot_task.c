#include "shoot_task.h"
#include "struct_typedef.h"
#include "can_task.h"
#include "pid.h"
#include "math_lib.h"
#include "remote_control.h"
#include "cmsis_os.h"
#include "stdlib.h"
#include "math.h"

shoot_data_t shoot_data;

void fn_ShootMode(void);

// 拨弹轮初始化
void fn_ShootMotorInit(void)
{
	fp32 af_TriggerMotor2006PosPid3[3] = {TriggerMotor2006PosPid3_ID207_kp, TriggerMotor2006PosPid3_ID207_ki, TriggerMotor2006PosPid3_ID207_kd};

	trigger_motor2006_data[0].round_num = 0;
	trigger_motor2006_data[0].relative_raw_angle = 0.0f;

	for (uint8_t m = 0; m < 6; m++)
	{
		trigger_motor2006_data[0].raw_angle[m] = 0.0f;
		trigger_motor2006_data[0].raw_speed[m] = 0.0f;
	}
	for (uint8_t n = 0; n < 2; n++)
	{
		trigger_motor2006_data[0].filter_angle[n] = 0.0f;
		trigger_motor2006_data[0].filter_speed[n] = 0.0f;
	}

	trigger_motor2006_data[0].relative_raw_speed = 0.0f;
	trigger_motor2006_data[0].offecd_ecd = trigger_motor2006_measure[0].ecd;
	trigger_motor2006_data[0].target_angle = 0.0f;
	trigger_motor2006_data[0].target_speed = 0.0f;
	trigger_motor2006_data[0].filter_given_current = 0.0f;
	trigger_motor2006_data[0].given_current = 0.0f;
	trigger_motor2006_data[0].double_pid_mid = 0.0f;

	fn_PidInit(&trigger_motor2006_data[0].motor_pid3, af_TriggerMotor2006PosPid3, TriggerMotor2006MinOut, TriggerMotor2006MaxOut, TriggerMotor2006MinIOut, TriggerMotor2006MaxIOut);
}

// 摩擦轮初始化
void fn_shoot_motor3508_init(void)
{
	fp32 af_GimbalMotor3508PosPid3[2][3] = {{GimbalMotor3508PosPid3_ID201_kp, GimbalMotor3508PosPid3_ID201_ki, GimbalMotor3508PosPid3_ID201_kd},
											{GimbalMotor3508PosPid3_ID202_kp, GimbalMotor3508PosPid3_ID202_ki, GimbalMotor3508PosPid3_ID202_kd}};

	for (uint8_t i = 0; i < 2; i++)
	{

		gimbal_motor3508_data[i].round_num = 0;
		gimbal_motor3508_data[i].relative_raw_angle = 0.0f;

		for (uint8_t m = 0; m < 5; m++)
		{
			gimbal_motor3508_data[i].raw_angle[m] = 0.0f;
			gimbal_motor3508_data[i].raw_speed[m] = 0.0f;
		}
		for (uint8_t n = 0; n < 2; n++)
		{
			gimbal_motor3508_data[i].filter_angle[n] = 0.0f;
			gimbal_motor3508_data[i].filter_speed[n] = 0.0f;
		}

		gimbal_motor3508_data[i].relative_raw_speed = 0.0f;
		gimbal_motor3508_data[i].offecd_ecd = 0;
		gimbal_motor3508_data[i].target_angle = 0.0f;
		gimbal_motor3508_data[i].target_speed = 0.0f;
		gimbal_motor3508_data[i].given_current = 0.0f;
		gimbal_motor3508_data[i].double_pid_mid = 0.0f;

		fn_PidInit(&gimbal_motor3508_data[i].motor_pid3, af_GimbalMotor3508PosPid3[i], GimbalMotor3508MinOut, GimbalMotor3508MaxOut, GimbalMotor3508MinIOut, GimbalMotor3508MaxIOut);
	}
}

// 射击模块状态初始化
void fn_shoot_init(void)
{
	shoot_data.fric_state = FRIC_OFF;
	shoot_data.shoot_mode = SHOOT_DOWN;

	shoot_data.fric_speed = FricSpeed;
}

// 射击模块模式选择
void fn_fric_state(void)
{
	if (IF_RC_SW1_MID && !IF_MOUSE_PRESSED_LEFT && !IF_RC_SW2_DOWN)
	{
		shoot_data.fric_state_change_flag = 1;
	}
	// 摩擦轮状态选择
	if (IF_RC_SW2_DOWN)
	{
		shoot_data.fric_state = FRIC_DOWN;
	}
	if ((IF_RC_SW1_UP || IF_MOUSE_PRESSED_LEFT) && (shoot_data.fric_state == FRIC_OFF || shoot_data.fric_state == FRIC_DOWN) && shoot_data.fric_state_change_flag == 1 && !IF_RC_SW2_DOWN)
	{
		shoot_data.fric_state = FRIC_ON;
		shoot_data.fric_state_change_flag = 0;
	}
	if (IF_RC_SW1_UP && shoot_data.fric_state == FRIC_ON && shoot_data.fric_state_change_flag == 1 && !IF_RC_SW2_DOWN)
	{
		shoot_data.fric_state = FRIC_OFF;
		shoot_data.fric_state_change_flag = 0;
	}
	// 射击模式选择
	fn_ShootMode();
}

// 射击模式选择
void fn_ShootMode(void)
{
	// 射击模式选择
	if (shoot_data.fric_state == FRIC_OFF || shoot_data.fric_state == FRIC_DOWN)
	{
		shoot_data.shoot_mode = SHOOT_DOWN;
	}
	if (shoot_data.fric_state == FRIC_ON)
	{
		shoot_data.shoot_mode = SHOOT_READY_COUNTINUE;
	}
}

// 射击模块电流解算
void fn_ShootMove(void)
{
	// 拨弹轮
	// down
	if (shoot_data.shoot_mode == SHOOT_DOWN)
	{

		trigger_motor2006_data[0].target_angle = trigger_motor2006_data[0].relative_raw_angle;
		trigger_motor2006_data[0].target_speed = 0;
		trigger_motor2006_data[0].given_current = 0;
	}

	// 连发模式
	if (shoot_data.shoot_mode == SHOOT_READY_COUNTINUE)
	{
		// 射击
		if(IF_KEY_PRESSED_R)  // 按下R键反转，优先级更高
		{
			trigger_motor2006_data[0].target_speed = TriggerSpeed;
			trigger_motor2006_data[0].target_speed *= 0.3f;
		}
		else if (IF_MOUSE_PRESSED_LEFT || IF_RC_SW1_DOWN)
		{
			trigger_motor2006_data[0].target_speed = -TriggerSpeed;
		}
		// 未射击
		else
		{
			trigger_motor2006_data[0].target_speed = 0.0f;
		}

		// 连发模式下将角度目标值永远等于现在值，保证切换回单发模式的连续性
		// trigger_motor2006_data[0].target_angle = trigger_motor2006_data[0].relative_raw_angle;

		// 连发模式电流计算
		trigger_motor2006_data[0].given_current = fn_PidClac(&trigger_motor2006_data[0].motor_pid3,
															 trigger_motor2006_data[0].relative_raw_speed, trigger_motor2006_data[0].target_speed);
	}

	// 摩擦轮
	if (shoot_data.fric_state == FRIC_DOWN)
	{
		gimbal_motor3508_data[0].target_speed = 0.0f;
		gimbal_motor3508_data[1].target_speed = 0.0f;
		gimbal_motor3508_data[0].given_current = 0;
		gimbal_motor3508_data[1].given_current = 0;
	}

	if (shoot_data.fric_state == FRIC_OFF)
	{
		gimbal_motor3508_data[0].target_speed = 0.0f;
		gimbal_motor3508_data[1].target_speed = 0.0f;
		// 摩擦轮电流计算
		gimbal_motor3508_data[0].given_current = fn_PidClac(&gimbal_motor3508_data[0].motor_pid3,
															gimbal_motor3508_data[0].relative_raw_speed, gimbal_motor3508_data[0].target_speed);
		gimbal_motor3508_data[1].given_current = fn_PidClac(&gimbal_motor3508_data[1].motor_pid3,
															gimbal_motor3508_data[1].relative_raw_speed, gimbal_motor3508_data[1].target_speed);
	}

	if (shoot_data.fric_state == FRIC_ON)
	{
		// 赋予摩擦轮速度
		gimbal_motor3508_data[0].target_speed = shoot_data.fric_speed;
		gimbal_motor3508_data[1].target_speed = -shoot_data.fric_speed;
		// 摩擦轮电流计算
		gimbal_motor3508_data[0].given_current = fn_PidClac(&gimbal_motor3508_data[0].motor_pid3,
															gimbal_motor3508_data[0].relative_raw_speed, gimbal_motor3508_data[0].target_speed);
		gimbal_motor3508_data[1].given_current = fn_PidClac(&gimbal_motor3508_data[1].motor_pid3,
															gimbal_motor3508_data[1].relative_raw_speed, gimbal_motor3508_data[1].target_speed);
	}
}
