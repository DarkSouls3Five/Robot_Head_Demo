#include "gimbal_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "pid.h"
#include "tim.h"
#include "math_lib.h"
#include "math.h"
#include "filter.h"
#include "remote_control.h"
#include "can.h"
#include "can_task.h"
#include "ins_task.h"
#include "shoot_task.h"

// 云台数据结构定义
gimbal_data_t gimbal_data;

// 云台回中模式下回中后的时间
uint16_t gimbal_init_over_time;

// 云台进入回中模式的时间
uint16_t gimbal_init_time;

// 云台是否在回中模式
uint8_t gimbal_init_flag;

void fn_gimbal_motor_init(void);
void fn_GimbalInit(void);
void fn_GimbalMode(void);
void fn_MotorMode(void);
void fn_GimbalMove(void);
int gimbal_temp = 0;

// pitch重力补偿

fp32 pitch_k = -0.75f;
fp32 pitch_add = 0.0f;
fp32 filter_pitch_imu[40];
fp32 filter_yaw_imu[40];
fp32 filter_yaw_imu_gyro[40];

fp32 filtered_avg_pitch_imu=0;
fp32 filtered_avg_yaw_imu = 0;
fp32 filtered_avg_yaw_imu_gyro = 0;

float temp_pitch_imu;


void Gimbal_Task(void const *argument)
{
    vTaskDelay(500);
		gimbal_temp = 1;
    // 云台电机初始化
    fn_gimbal_motor_init();

    // 云台数据初始化
    fn_GimbalInit();

    // 拨弹轮2006初始化
    fn_ShootMotorInit();

    // 摩擦轮3508初始化
    fn_shoot_motor3508_init();

    // 射击模块初始化
    fn_shoot_init();

    int cnt = 0;
		int cnt40 = 0;
		
    while (1)
    {
        
        // 云台模式选择
        fn_GimbalMode();
			
				filtered_avg_pitch_imu = 0;
        filtered_avg_yaw_imu = 0;
        filtered_avg_yaw_imu_gyro = 0;
			
				filter_pitch_imu[cnt40] = INS_eulers[1];
        filter_yaw_imu[cnt40] = INS_eulers[0];
        filter_yaw_imu_gyro[cnt40] = INS_gyro[2];
			
				for (int i = 0; i < 40; i++)
        {
            filtered_avg_pitch_imu += filter_pitch_imu[i];
            filtered_avg_yaw_imu += filter_yaw_imu[i];
            filtered_avg_yaw_imu_gyro += filter_yaw_imu_gyro[i];
        }
				
				filtered_avg_pitch_imu = filtered_avg_pitch_imu / 40;
        filtered_avg_yaw_imu = filtered_avg_yaw_imu / 40;
        filtered_avg_yaw_imu_gyro = filtered_avg_yaw_imu_gyro / 40;
				
        // 射击模块模式选择
        fn_fric_state();

        // 计算云台电流
        fn_GimbalMove();

        // 计算射击电机电流
        fn_ShootMove();

        // 发送电流
        fn_cmd_CAN1GimbalMotor(gimbal_motor6020_data[0].given_voltage,0.0f,0.0f,0.0f);
				
				pitch_add = pitch_k * sin(filtered_avg_pitch_imu);
				fn_ctrl_DM_motor(0.0f,0.0f,0.0f,0.0f,gimbal_DM_data[0].target_torque + pitch_add);
				// fn_ctrl_DM_motor(0.0f,0.0f,0.0f,0.0f,pitch_add);

        fn_cmd_CAN2GimbalMotor1(gimbal_motor3508_data[0].given_current, gimbal_motor3508_data[1].given_current, trigger_motor2006_data[0].given_current, 0.0f);
				
				
        if (cnt % 2 == 0) // 以500Hz发送imu数据
           fn_cmd_quat_to_computer(INS_quat[1], INS_quat[2], INS_quat[3], INS_quat[0]);
				cnt++;
				cnt40++;
				if (cnt >= 1000)
            cnt = 0;
				if(cnt40 >= 40)
						cnt40 = 0;
        vTaskDelay(10);
    }
}

// 云台数据初始化
void fn_GimbalInit(void)
{
    fp32 af_GimbalIMUPosPid1[2][3] = {{GimbalPid1Yaw_kp, GimbalPid1Yaw_ki, GimbalPid1Yaw_kd},
                                      {GimbalPid1Pitch_kp, GimbalPid1Pitch_ki, GimbalPid1Pitch_kd}};
    fp32 af_GimbalIMUPosPid2[2][3] = {{GimbalPid2Yaw_kp, GimbalPid2Yaw_ki, GimbalPid2Yaw_kd},
                                      {GimbalPid2Pitch_kp, GimbalPid2Pitch_ki, GimbalPid2Pitch_kd}};

    gimbal_init_over_time = 0;
    gimbal_init_time = 0;
    gimbal_init_flag = 0;

    gimbal_data.gyro_yaw_target_angle = INS_eulers[0];
    gimbal_data.gyro_pit_target_angle = INS_eulers[1];

    gimbal_data.f_GimbalYawPidMid = 0.0f;
    gimbal_data.f_GimbalPitPidMid = 0.0f;

    gimbal_data.gimbal_behaviour = GIMBAL_ZERO_FORCE;
    gimbal_data.last_gimbal_behaviour = GIMBAL_ZERO_FORCE;

    gimbal_data.gimbal_motor_yaw_mode = GIMBAL_Motor_DOWN;
    gimbal_data.gimbal_motor_pit_mode = GIMBAL_Motor_DOWN;

    fn_PidInit(&gimbal_data.GimbalIMUYawPid1, af_GimbalIMUPosPid1[0], GimbalPidMinOut_Yaw, GimbalPidMaxOut_Yaw, GimbalPidMinIOut_Yaw, GimbalPidMaxIOut_Yaw);
    fn_PidInit(&gimbal_data.GimbalIMUYawPid2, af_GimbalIMUPosPid2[0], GimbalPidMinOut_Yaw, GimbalPidMaxOut_Yaw, GimbalPidMinIOut_Yaw, GimbalPidMaxIOut_Yaw);

    fn_PidInit(&gimbal_data.GimbalIMUPitPid1, af_GimbalIMUPosPid1[1], GimbalPidMinOut_Pit, GimbalPidMaxOut_Pit, GimbalPidMinIOut_Pit, GimbalPidMaxIOut_Pit);
    fn_PidInit(&gimbal_data.GimbalIMUPitPid2, af_GimbalIMUPosPid2[1], GimbalPidMinOut_Pit, GimbalPidMaxOut_Pit, GimbalPidMinIOut_Pit, GimbalPidMaxIOut_Pit);
		
		for(int i=0;i<20;i++)
		{
			filter_pitch_imu[i] = 0;
		}
}

// 云台电机初始化
void fn_gimbal_motor_init(void)
{

    uint16_t OffecdEcd = 5416;
    // 云台6020电机初始化
    fp32 af_GimbalMotor6020PosPid1[2][3] = {{GimbalMotor6020PosPid1_Yaw_kp, GimbalMotor6020PosPid1_Yaw_ki, GimbalMotor6020PosPid1_Yaw_kd},
                                            {GimbalMotor6020PosPid1_Pit_kp, GimbalMotor6020PosPid1_Pit_ki, GimbalMotor6020PosPid1_Pit_kd}};
    fp32 af_GimbalMotor6020PosPid2[2][3] = {{GimbalMotor6020PosPid2_Yaw_kp, GimbalMotor6020PosPid2_Yaw_ki, GimbalMotor6020PosPid2_Yaw_kd},
                                            {GimbalMotor6020PosPid2_Pit_kp, GimbalMotor6020PosPid2_Pit_ki, GimbalMotor6020PosPid2_Pit_kd}};

    gimbal_motor6020_data[0].relative_raw_angle = 0.0f;
    gimbal_motor6020_data[0].offecd_ecd = OffecdEcd;

    for (uint8_t m = 0; m < 6; m++)
    {
        gimbal_motor6020_data[0].raw_angle[m] = 0.0f;
    }
    for (uint8_t n = 0; n < 2; n++)
    {
        gimbal_motor6020_data[0].filter_angle[n] = 0.0f;
    }

    gimbal_motor6020_data[0].relative_raw_speed = 0.0f;
    gimbal_motor6020_data[0].target_angle = 0.0f;
    gimbal_motor6020_data[0].given_voltage = 0.0f;
    gimbal_motor6020_data[0].double_pid_mid = 0.0f;

    fn_PidInit(&gimbal_motor6020_data[0].motor_pid1, af_GimbalMotor6020PosPid1[0], GimbalMotor6020MinOut, GimbalMotor6020MaxOut, GimbalMotor6020MinIOut, GimbalMotor6020MaxIOut);
    fn_PidInit(&gimbal_motor6020_data[0].motor_pid2, af_GimbalMotor6020PosPid2[0], GimbalMotor6020MinOut, GimbalMotor6020MaxOut, GimbalMotor6020MinIOut, GimbalMotor6020MaxIOut);

    // 云台DM4310电机初始化
    fn_DM_start_motor();

    gimbal_DM_data[0].target_angle = 0.0f;
    gimbal_DM_data[0].target_torque = 0.0f;
    gimbal_DM_data[0].offecd_angle = 2.9131f;

    gimbal_DM_data[0].double_pid_mid = 0.0f;

    fn_PidInit(&gimbal_DM_data[0].motor_pid1, af_GimbalMotor6020PosPid1[1], GimbalMotor4310MinOut, GimbalMotor4310MaxOut, GimbalMotor4310MinIOut, GimbalMotor4310MaxIOut);
    fn_PidInit(&gimbal_DM_data[0].motor_pid2, af_GimbalMotor6020PosPid2[1], GimbalMotor4310MinOut, GimbalMotor4310MaxOut, GimbalMotor4310MinIOut, GimbalMotor4310MaxIOut);
}

// 云台状态选择
void fn_GimbalMode(void)
{
    // 正在回中过程中无法调整模式
    if (gimbal_init_flag == 1)
    {
        return;
    }

    // 遥控器
    if (!IF_RC_SW2_MID)
    {
        if (IF_RC_SW2_DOWN)
        {
            gimbal_data.last_gimbal_behaviour = gimbal_data.gimbal_behaviour;
            gimbal_data.gimbal_behaviour = GIMBAL_ZERO_FORCE;
        }

        if (IF_RC_SW2_UP)
        {
            gimbal_data.last_gimbal_behaviour = gimbal_data.gimbal_behaviour;
            gimbal_data.gimbal_behaviour = GIMBAL_GYRO;
        }
    }

    // 键鼠 长按V自瞄 只长按B打符
    if (IF_RC_SW2_MID)
    {
        if (!IF_KEY_PRESSED_V && !IF_KEY_PRESSED_B)
        {
            gimbal_data.last_gimbal_behaviour = gimbal_data.gimbal_behaviour;
            gimbal_data.gimbal_behaviour = GIMBAL_GYRO;
        }
        // 当V同时按下时判定为V自瞄模式
        if (IF_KEY_PRESSED_V)
        {
            gimbal_data.last_gimbal_behaviour = gimbal_data.gimbal_behaviour;
            gimbal_data.gimbal_behaviour = GIMBAL_AUTO;
        }
    }

    // 判断是否进入回中模式
    if (gimbal_data.last_gimbal_behaviour == GIMBAL_ZERO_FORCE && gimbal_data.gimbal_behaviour != GIMBAL_ZERO_FORCE)
    {
        gimbal_data.gimbal_behaviour = GIMBAL_INIT;
        gimbal_init_flag = 1;
    }

    // 根据云台状态选择电机控制模式
    fn_MotorMode();
}

// 电机模式选择
void fn_MotorMode(void)
{
    if (gimbal_data.gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        gimbal_data.gimbal_motor_yaw_mode = GIMBAL_Motor_DOWN;
        gimbal_data.gimbal_motor_pit_mode = GIMBAL_Motor_DOWN;
    }
    if (gimbal_data.gimbal_behaviour == GIMBAL_INIT)
    {
        gimbal_data.gimbal_motor_yaw_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_data.gimbal_motor_pit_mode = GIMBAL_MOTOR_ENCONDE;
    }
    if (gimbal_data.gimbal_behaviour == GIMBAL_GYRO)
    {
        gimbal_data.gimbal_motor_yaw_mode = GIMBAL_MOTOR_GYRO;
        gimbal_data.gimbal_motor_pit_mode = GIMBAL_MOTOR_GYRO;
    }
    if (gimbal_data.gimbal_behaviour == GIMBAL_ENCONDE)
    {
        gimbal_data.gimbal_motor_yaw_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_data.gimbal_motor_pit_mode = GIMBAL_MOTOR_ENCONDE;
    }
    if (gimbal_data.gimbal_behaviour == GIMBAL_AUTO)
    {
        gimbal_data.gimbal_motor_yaw_mode = GIMBAL_MOTOR_GYRO;
        gimbal_data.gimbal_motor_pit_mode = GIMBAL_MOTOR_GYRO;
    }
}

void fn_GimbalMove(void)
{

    // 电机为模式DOWN
    if (gimbal_data.gimbal_motor_yaw_mode == GIMBAL_Motor_DOWN && gimbal_data.gimbal_motor_pit_mode == GIMBAL_Motor_DOWN)
    {
        gimbal_motor6020_data[0].given_voltage = 0.0f;
        gimbal_DM_data[0].target_torque = 0.0f;
        // 覆盖自瞄数据，保证模式切换的连续性
        autoaim_measure.yaw = INS_eulers[0];
        autoaim_measure.pitch = INS_eulers[1];
    }

    // 电机模式为GYRO
    if (gimbal_data.gimbal_motor_yaw_mode == GIMBAL_MOTOR_GYRO && gimbal_data.gimbal_motor_pit_mode == GIMBAL_MOTOR_GYRO)
    {
        // 遥控器
        if (!IF_RC_SW2_MID)
        {
            // 陀螺仪控云台
            if (gimbal_data.gimbal_behaviour == GIMBAL_GYRO)
            {
                // 获取角速度
                gimbal_data.gyro_yaw_angle_add = -(float)(ctl.rc.ch2 - 1024) / 660.0f * WMax;
                gimbal_data.gyro_pit_angle_add = -(float)(ctl.rc.ch3 - 1024) / 660.0f * WMax;
                // pitch轴限角 暂时未考虑底盘斜置于斜坡上的情况即yaw轴对pitch角度的影响与pitch轴角度自增带来的超出限位情况以及自增超范围的情况
                if (fn_scope_judgment(gimbal_DM_data[0].position, PitAngleMin, PitAngleMax))
                {
                    if (fabs(gimbal_data.gyro_pit_target_angle + gimbal_data.gyro_pit_angle_add) > fabs(gimbal_data.gyro_pit_target_angle))
                    {
                        gimbal_data.gyro_pit_angle_add = 0.0f;
                    }
                }
								if (fn_scope_judgment(gimbal_motor6020_data[0].relative_raw_angle, YawAngleMin, YawAngleMax))
                {
                    if (fabs(gimbal_data.gyro_yaw_target_angle + gimbal_data.gyro_yaw_angle_add) > fabs(gimbal_data.gyro_yaw_target_angle))
                    {
                        gimbal_data.gyro_yaw_angle_add = 0.0f;
                    }
                }
                // 赋值目标角度
                gimbal_data.gyro_yaw_target_angle = fn_RadFormat(gimbal_data.gyro_yaw_target_angle + gimbal_data.gyro_yaw_angle_add);
                gimbal_data.gyro_pit_target_angle = fn_RadFormat(gimbal_data.gyro_pit_target_angle + gimbal_data.gyro_pit_angle_add);
            }
        }

        // 键鼠
        if (IF_RC_SW2_MID)
        {
            // 陀螺仪控云台
            if (gimbal_data.gimbal_behaviour == GIMBAL_GYRO)
            {
                gimbal_data.gyro_yaw_angle_add = -MOUSE_X_MOVE_SPEED * WCoef;
                gimbal_data.gyro_pit_angle_add = MOUSE_Y_MOVE_SPEED * WCoef;

                // pitch轴限角 暂时未考虑底盘斜置于斜坡上的情况即yaw轴对pitch角度的影响与pitch轴角度自增带来的超出限位情况以及自增超范围的情况
                if (fn_scope_judgment(gimbal_DM_data[0].position, PitAngleMin, PitAngleMax))
                {
                    if (fabs(gimbal_data.gyro_pit_target_angle + gimbal_data.gyro_pit_angle_add) > fabs(gimbal_data.gyro_pit_target_angle))
                    {
                        gimbal_data.gyro_pit_angle_add = 0.0f;
                    }
                }
                // 赋值目标角度
                gimbal_data.gyro_yaw_target_angle = fn_RadFormat(gimbal_data.gyro_yaw_target_angle + gimbal_data.gyro_yaw_angle_add);
                gimbal_data.gyro_pit_target_angle = fn_RadFormat(gimbal_data.gyro_pit_target_angle + gimbal_data.gyro_pit_angle_add);

                // 覆盖自瞄数据，保证模式切换的连续性
                autoaim_measure.yaw = INS_eulers[0];
                autoaim_measure.pitch = INS_eulers[1];
            }

            // 自瞄控云台
            if (gimbal_data.gimbal_behaviour == GIMBAL_AUTO)
            {
                // 赋予自瞄坐标
                if (autoaim_measure.vision_state == 1)
                {
                    gimbal_data.gyro_yaw_target_angle = autoaim_measure.yaw;
                    gimbal_data.gyro_pit_target_angle = autoaim_measure.pitch;
                }
            }
        }

        // 解算GYRO模式下两轴电流
        gimbal_data.f_GimbalYawPidMid = fn_PidClacAngle(&gimbal_data.GimbalIMUYawPid1, filtered_avg_yaw_imu, gimbal_data.gyro_yaw_target_angle);
        gimbal_motor6020_data[0].given_voltage = -fn_PidClac(&gimbal_data.GimbalIMUYawPid2, filtered_avg_yaw_imu_gyro, gimbal_data.f_GimbalYawPidMid);

				if(fabs(INS_eulers[1]) < 0.7)
				{
        gimbal_data.f_GimbalPitPidMid = fn_PidClacAngle(&gimbal_data.GimbalIMUPitPid1, INS_eulers[1], gimbal_data.gyro_pit_target_angle);
        gimbal_DM_data[0].target_torque = -fn_PidClac(&gimbal_data.GimbalIMUPitPid2, INS_gyro[0], gimbal_data.f_GimbalPitPidMid);
				}
    }

    // 电机模式为ENCODE
    if (gimbal_data.gimbal_motor_yaw_mode == GIMBAL_MOTOR_ENCONDE && gimbal_data.gimbal_motor_pit_mode == GIMBAL_MOTOR_ENCONDE)
    {
        // 云台初始化
        if (gimbal_data.gimbal_behaviour == GIMBAL_INIT)
        {

            // 赋予目标角度
            gimbal_motor6020_data[0].target_angle = 0.0f;
            gimbal_DM_data[0].target_angle = 0.0f;

            // 判断是否已经回中
            if ((fabs(gimbal_motor6020_data[0].relative_raw_angle - 0.0f)) < 0.05f && (fabs(gimbal_DM_data[0].position - 0.0f) < 0.05f))
            {
                gimbal_init_over_time++;
            }

            gimbal_init_time++;

            // 判断初始化完成 进入init模式3s或者回中后持续0.5s则认为init模式完成
            if (gimbal_init_time == 3000 || gimbal_init_over_time == 500)
            {
                gimbal_data.gyro_yaw_target_angle = INS_eulers[0];
                gimbal_data.gyro_pit_target_angle = INS_eulers[1];
                gimbal_init_over_time = 0;
                gimbal_init_time = 0;
                gimbal_init_flag = 0;
            }
        }

        // 解算ENCODE模式下两轴电流
        gimbal_motor6020_data[0].double_pid_mid = fn_PidClacAngle(&gimbal_motor6020_data[0].motor_pid1,
                                                                  gimbal_motor6020_data[0].relative_raw_angle, gimbal_motor6020_data[0].target_angle);
        gimbal_motor6020_data[0].given_voltage = fn_PidClac(&gimbal_motor6020_data[0].motor_pid2,
                                                            gimbal_motor6020_data[0].relative_raw_speed, gimbal_motor6020_data[0].double_pid_mid);
        gimbal_DM_data[0].double_pid_mid = fn_PidClacAngle(&gimbal_DM_data[0].motor_pid1,
                                                           gimbal_DM_data[0].position, gimbal_DM_data[0].target_angle);
        gimbal_DM_data[0].target_torque = fn_PidClac(&gimbal_DM_data[0].motor_pid2,
                                                     gimbal_DM_data[0].velocity, gimbal_DM_data[0].double_pid_mid);
    }
}
