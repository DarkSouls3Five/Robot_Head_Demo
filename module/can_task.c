#include "can_task.h"
#include "main.h"
#include "struct_typedef.h"
#include "math_lib.h"
#include "pid.h"

	
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

// 2006电机原始数据类型

motor_measure_t gimbal_motor2006_measure;	

// DM电机解算数据类型
DM_motor_measure_t YAW_DM_data;				// id [0x17,0x15]    [CAN,Master] CAN1

// 自瞄数据
autoaim_measure_t autoaim_measure;

//计算2006绝对角度
static void M2006_absolute_position_cal(motor_measure_t *adv_act_absolute);

// 云台6020电机数据解算
void fn_GimbalMotor6020Data(uint8_t i);

// 云台2006拨弹轮数据解算
void fn_TriggerMotor2006Data(uint8_t i);

// 云台3508摩擦轮数据解算
void fn_GimbalMotor3508Data(uint8_t i);

// 达妙电机数据获取
void get_dm_motor_data(DM_motor_measure_t *data, uint8_t rx_data[8])
{
	data->p_int = (rx_data[1] << 8) | rx_data[2];
	data->v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
	data->t_int = ((rx_data[4] & 0xF) << 8) | rx_data[5];
	data->position = uint_to_float(data->p_int, P_MIN, P_MAX, 16) - data->offecd_angle; // (-12.5,12.5)
	data->velocity = uint_to_float(data->v_int, V_MIN, V_MAX, 12);						// (-45.0,45.0)
	data->torque = uint_to_float(data->t_int, T_MIN, T_MAX, 12);						// (-18.0,18.0)
}

// 大疆电机数据获取
void get_motor_measure(motor_measure_t *ptr, uint8_t *data)                                    
{                                                                   
		(ptr)->last_ecd = (ptr)->ecd;                                   
		(ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            
		(ptr)->speed_rpm = (int16_t)((data)[2] << 8 | (data)[3]);      
		(ptr)->given_current = (int16_t)((data)[4] << 8 | (data)[5]);  
		(ptr)->temperate = (data)[6];
}

// 上位机自瞄数据获取
void get_autoaim_measure(autoaim_measure_t *autoaim_state, uint8_t rx_data[8])
{
	autoaim_state->vision_state = rx_data[0];
	autoaim_state->shoot = (rx_data[1] == 1);
	autoaim_state->yaw = (int16_t)(rx_data[2] << 8 | rx_data[3]) / 1e4f;
	autoaim_state->pitch = (int16_t)(rx_data[4] << 8 | rx_data[5]) / 1e4f;
}

// 将获取的数据解算到每个电机
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[8];

	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
	if (hcan == &hcan1)
	{
		switch (rx_header.StdId)
		{

		case CAN_PIT_MOTOR_ID:
		{
			get_motor_measure(&gimbal_motor2006_measure, rx_data);
			M2006_absolute_position_cal(&gimbal_motor2006_measure);
			break;
		}

		case CAN_YAW_MOTOR_ID:
		{
//			get_dm_motor_data(&gimbal_DM_data[0], rx_data);
			get_dm_motor_data(&YAW_DM_data, rx_data);			
			break;
		}

		default:
		{
			break;
		}
		}
	}

//	if (hcan == &hcan2)
//	{
//		switch (rx_header.StdId)
//		{

//		case CAN_3508_FRICL_ID:
//		case CAN_3508_FRICR_ID:
//		{
//			uint8_t i = 0;
//			i = rx_header.StdId - CAN_3508_FRICR_ID;
//			get_motor_measure(&gimbal_motor3508_measure[i], rx_data);
//			fn_GimbalMotor3508Data(i);
//			break;
//		}

//		case CAN_TRIGGER_MOTOR_ID:
//		{
//			uint8_t i = 0;
//			get_motor_measure(&trigger_motor2006_measure[i], rx_data);
//			fn_TriggerMotor2006Data(i);
//			break;
//		}
//		case CAN_AUTOAIM_ID:
//		{
//			get_autoaim_measure(&autoaim_measure, rx_data);
//			break;
//		}
//		default:
//		{
//			break;
//		}
//		}
//	}
}

// 达妙电机使能帧
void fn_DM_start_motor(void)
{
	uint32_t send_mail_box2;
	CAN_TxHeaderTypeDef Txheader2;
	Txheader2.StdId = 0x17;
	Txheader2.RTR = CAN_RTR_DATA; // 消息类型为数据帧
	Txheader2.IDE = CAN_ID_STD;	  // ID类型为标准ID
	Txheader2.DLC = 0x08;		  // 消息长度为8字节
	
	uint8_t CAN_send_data2[8];
	CAN_send_data2[0] = 0xFF;
	CAN_send_data2[1] = 0xFF;
	CAN_send_data2[2] = 0xFF;
	CAN_send_data2[3] = 0xFF;
	CAN_send_data2[4] = 0xFF;
	CAN_send_data2[5] = 0xFF;
	CAN_send_data2[6] = 0xFF;
	CAN_send_data2[7] = 0xFC;

	HAL_CAN_AddTxMessage(&hcan1, &Txheader2, CAN_send_data2, &send_mail_box2);
}

// 达妙电机保存零点
void fn_DM_record_init_state(void)
{
	uint32_t send_mail_box2;
	CAN_TxHeaderTypeDef Txheader2;
	Txheader2.StdId = 0x17;
	Txheader2.RTR = CAN_RTR_DATA; // 消息类型为数据帧
	Txheader2.IDE = CAN_ID_STD;	  // ID类型为标准ID
	Txheader2.DLC = 0x08;		  // 消息长度为8字节

	uint8_t CAN_send_data2[8];
	CAN_send_data2[0] = 0xFF;
	CAN_send_data2[1] = 0xFF;
	CAN_send_data2[2] = 0xFF;
	CAN_send_data2[3] = 0xFF;
	CAN_send_data2[4] = 0xFF;
	CAN_send_data2[5] = 0xFF;
	CAN_send_data2[6] = 0xFF;
	CAN_send_data2[7] = 0xFE;

	HAL_CAN_AddTxMessage(&hcan1, &Txheader2, CAN_send_data2, &send_mail_box2);
}

// CAN1 达妙电机控制帧
void fn_ctrl_DM_motor(float _pos, float _vel, float _KP, float _KD, float _torq)
{
	uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
	pos_tmp = float_to_uint(_pos, P_MIN, P_MAX, 16);
	vel_tmp = float_to_uint(_vel, V_MIN, V_MAX, 12);
	kp_tmp = float_to_uint(_KP, KP_MIN, KP_MAX, 12);
	kd_tmp = float_to_uint(_KD, KD_MIN, KD_MAX, 12);
	tor_tmp = float_to_uint(_torq, T_MIN, T_MAX, 12);

	uint32_t send_mail_box2;
	CAN_TxHeaderTypeDef Txheader2;
	Txheader2.StdId = 0x17;
	Txheader2.RTR = CAN_RTR_DATA; // 消息类型为数据帧
	Txheader2.IDE = CAN_ID_STD;	  // ID类型为标准ID
	Txheader2.DLC = 0x08;		  // 消息长度为8字节

	uint8_t CAN_send_data2[8];
	CAN_send_data2[0] = (pos_tmp >> 8);
	CAN_send_data2[1] = pos_tmp;
	CAN_send_data2[2] = (vel_tmp >> 4);
	CAN_send_data2[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
	CAN_send_data2[4] = kp_tmp;
	CAN_send_data2[5] = (kd_tmp >> 4);
	CAN_send_data2[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
	CAN_send_data2[7] = tor_tmp;

	HAL_CAN_AddTxMessage(&hcan1, &Txheader2, CAN_send_data2, &send_mail_box2);
}


// 2006电机电流发送函数
void fn_cmd_CAN1GimbalMotor(int16_t motor1)
{
	uint32_t send_mail_box2;
	CAN_TxHeaderTypeDef Txheader2;
	Txheader2.StdId = 0x200;
	Txheader2.RTR = CAN_RTR_DATA; // 消息类型为数据帧
	Txheader2.IDE = CAN_ID_STD;	  // ID类型为标准ID
	Txheader2.DLC = 0x08;		  // 消息长度为8字节
	
	uint8_t CAN_send_data2[8];
	CAN_send_data2[0] = motor1 >> 8;
	CAN_send_data2[1] = motor1;
	CAN_send_data2[2] = 0X00;
	CAN_send_data2[3] = 0X00;
	CAN_send_data2[4] = 0X00;
	CAN_send_data2[5] = 0X00;
	CAN_send_data2[6] = 0X00;
	CAN_send_data2[7] = 0X00;

	HAL_CAN_AddTxMessage(&hcan1, &Txheader2, CAN_send_data2, &send_mail_box2);
}

// 四元数发送函数  CAN1
void fn_cmd_quat_to_computer(fp32 x, fp32 y, fp32 z, fp32 w)
{
	uint32_t send_mail_box2;
	CAN_TxHeaderTypeDef Txheader2;
	Txheader2.StdId = 0x100;
	Txheader2.RTR = CAN_RTR_DATA; // 消息类型为数据帧
	Txheader2.IDE = CAN_ID_STD;	  // ID类型为标准ID
	Txheader2.DLC = 0x08;		      // 消息长度为8字节
	
	uint8_t CAN_send_data2[8];
	CAN_send_data2[0] = (int16_t)(x * 1e4f) >> 8;
	CAN_send_data2[1] = (int16_t)(x * 1e4f);
	CAN_send_data2[2] = (int16_t)(y * 1e4f) >> 8;
	CAN_send_data2[3] = (int16_t)(y * 1e4f);
	CAN_send_data2[4] = (int16_t)(z * 1e4f) >> 8;
	CAN_send_data2[5] = (int16_t)(z * 1e4f);
	CAN_send_data2[6] = (int16_t)(w * 1e4f) >> 8;
	CAN_send_data2[7] = (int16_t)(w * 1e4f);

	HAL_CAN_AddTxMessage(&hcan2, &Txheader2, CAN_send_data2, &send_mail_box2);
}

// 射击状态发送函数  CAN1
void fn_cmd_shoot_data_to_computer(fp32 speed, char mode)
{
	uint32_t send_mail_box2;
	CAN_TxHeaderTypeDef Txheader2;
	Txheader2.StdId = 0x101;
	Txheader2.RTR = CAN_RTR_DATA; // 消息类型为数据帧
	Txheader2.IDE = CAN_ID_STD;	  // ID类型为标准ID
	Txheader2.DLC = 0x03;		  // 消息长度为3字节
	
	uint8_t CAN_send_data2[3];
	CAN_send_data2[0] = (uint16_t)(speed * 1e2f) >> 8;
	CAN_send_data2[1] = (uint16_t)(speed * 1e2f);
	CAN_send_data2[2] = mode;

	HAL_CAN_AddTxMessage(&hcan1, &Txheader2, CAN_send_data2, &send_mail_box2);
}

/**
* @brief          计算2006电机绝对角度
* @param[in]      电机ID
* @retval         电机数据指针
  */
static void M2006_absolute_position_cal(motor_measure_t *motor_2006)
{
	if(motor_2006->ecd - motor_2006->last_ecd > 4096)
	{
		motor_2006->round --;
	}
	else if(motor_2006->ecd - motor_2006->last_ecd < -4096)
	{
		motor_2006->round ++;	
	}
	
	motor_2006->distance = (motor_2006->round + (fp32)(motor_2006->ecd-motor_2006->init_ecd)/8192.0f)*360/36;
}
/**
* @brief          记录2006电机初始ecd
* @param[in]      电机ID
* @retval         电机数据指针
  */
void init_ecd_record(motor_measure_t *motor_2006)
{
	motor_2006->init_ecd = motor_2006->ecd;
	if(motor_2006->init_ecd>4096)
		motor_2006->round ++;
}
