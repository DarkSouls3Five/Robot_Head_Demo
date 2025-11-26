#include "move_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "pid.h"
#include "tim.h"
#include "math_lib.h"
#include "math.h"
#include "filter.h"
#include "can.h"
#include "can_task.h"
#include "ins_task.h"
#include "ps2_typedef.h"
#include "mode_set_task.h"
#include "usart.h"

extern Ps2_s ps2;//引用遥控器数据
extern head_mode_t head_mode;//引用模式设置指针

// 头部运动数据结构定义
move_data_t move_data;

//点头摇头动作时间坐标
int scan_t = 0;
int direction = 1;//运动方向

//复位完成标志
int rst_flag = 0;

void fn_move_motor_init(void);// 头部电机初始化
void fn_moveInit(void);// 头部数据初始化
void fn_moveMode(void);// 头部模式选择
void fn_moveMotorMode(void);// 头部电机模式选择
void fn_move_mode_change_control_transit(void);//头部模式改变，有些数据需要保存
void fn_move_feedback_update(void);//头部电机数据更新
void fn_MoveControl(void);// 计算头部电机电流

//pid相关函数
static void move_PID_init(move_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd);
static fp32 move_PID_calc(move_PID_t *pid, fp32 get, fp32 set, fp32 error_delta);

// pitch重力前馈补偿相关数据
fp32 pitch_k = -100.0f;
fp32 pitch_add = 0.0f;
fp32 filter_pitch_imu[40];
fp32 filter_yaw_imu[40];
fp32 filter_pitch_imu_gyro[40];
fp32 filter_yaw_imu_gyro[40];

fp32 filtered_avg_pitch_imu=0;
fp32 filtered_avg_yaw_imu = 0;
fp32 filtered_avg_yaw_imu_gyro = 0;
fp32 filtered_avg_pitch_imu_gyro = 0;


//串口数据记录变量
volatile uint8_t buf_uart[10];
//串口改变状态标志变量 1点头 2摇头 3扫描 4停止 5声源定位
uint8_t mode_uart = 0;
//串口接收声源角度数据
int angle_uart = 0;

//串口数据接收中断
void USART6_IRQHandler(void)  
{

    static uint8_t q = 0;
//		HAL_UART_IRQHandler(&huart6);
    //receive interrupt 接收中断
    if(huart6.Instance->SR & UART_FLAG_RXNE)
    {
        buf_uart[q] = huart6.Instance->DR;
				q++;
    }
    //idle interrupt 空闲中断
    else if(huart6.Instance->SR & UART_FLAG_IDLE)
    {
        huart6.Instance->DR;
			
        // 判断是否收到"NOD"
        if(
           buf_uart[1] == 'N' && 
           buf_uart[2] == 'O' && 
           buf_uart[3] == 'D')
            mode_uart = 1;  // 设置模式标志位1

				//判断是否收到"SHAKE"
        else if(
           buf_uart[1] == 'S' && 
           buf_uart[2] == 'H' && 
           buf_uart[3] == 'A' &&
					 buf_uart[4] == 'K' &&
					 buf_uart[5] == 'E' 
				)
        {
            mode_uart = 2;  // 设置模式标志位2
        }	
				//判断是否收到"SCAN"
        else if( 
           buf_uart[1] == 'S' && 
           buf_uart[2] == 'C' && 
           buf_uart[3] == 'A' &&
					 buf_uart[4] == 'N' 
				)
        {
            mode_uart = 3;  // 设置模式标志位3
        }		
				//判断是否收到"STOP"
        else if(
           buf_uart[1] == 'S' && 
           buf_uart[2] == 'T' && 
           buf_uart[3] == 'O' &&
					 buf_uart[4] == 'P' 
				)
        {
            mode_uart = 4;  // 设置模式标志位4
        }					
				else if(q == 1)
				{
					mode_uart = 5;		// 设置模式标志位5
					angle_uart = ((int)buf_uart[0]) * 360/255;	//还原角度
				}
				q = 0;
    }


}

void move_task(void const *argument)
{
    vTaskDelay(500);

    // 头部电机初始化
    fn_move_motor_init();

    // 头部数据初始化
    fn_moveInit();
		//pitch imu数据过滤计数
		int cnt40 = 0;

		while(1)
		{

//			//串口接收相关
//			char buf[100];
//			char str1[]="\n";
//			int str2= ps2.joystick[0]*100;
//			snprintf(buf, sizeof(buf), "%s%s", buf_uart, str1);
//			
//			//打印串口接收结果
//			HAL_UART_Transmit(&huart6,(uint8_t*)buf_uart,strlen(buf_uart),100);
			
			// 头部模式选择
			fn_moveMode();
			
			//数据过渡
			fn_move_mode_change_control_transit();	
			
			//数据更新
			fn_move_feedback_update(); 
			
			//imu数据获取
			filtered_avg_pitch_imu = 0;
			filtered_avg_yaw_imu = 0;
			filtered_avg_yaw_imu_gyro = 0;
		
			filter_pitch_imu[cnt40] = INS_eulers[2];
			filter_pitch_imu_gyro[cnt40] = INS_gyro[0];
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
			
			
			// 计算头部电机电流
			fn_MoveControl();
			
			// 发送Pitch电流
			pitch_add = pitch_k * sin(filtered_avg_pitch_imu);	//重力前馈		
			fn_cmd_CAN1GimbalMotor(move_data.pit_motor_data.give_current + pitch_add);	
			// 发送Yaw电流
			fn_ctrl_DM_motor(move_data.yaw_motor_data.motor_angle_set,0.0f,move_data.yaw_motor_data.DM_kp,move_data.yaw_motor_data.DM_kd,0.0f);

			
      vTaskDelay(5);		
			move_data.last_move_mode = move_data.move_mode;
			cnt40++;		
				
			//imu均值计算数据清零
			if(cnt40 >= 40)
					cnt40 = 0;			
		}

}

// 头部数据初始化
void fn_moveInit(void)
{
    move_data.move_mode = MOVE_FREE;
    move_data.last_move_mode = MOVE_FREE;

    move_data.move_motor_yaw_mode = MOVE_MOTOR_DOWN;
    move_data.move_motor_pit_mode = MOVE_MOTOR_DOWN;
}

// 头部电机初始化
void fn_move_motor_init(void)
{
		//初始化头部两个电机指针
		move_data.pit_motor_data.move_motor_measure = &gimbal_motor2006_measure;
		move_data.yaw_motor_data.DM_motor_measure = &YAW_DM_data;
	
		//PID数据初始化
		const static fp32 DM_pid1[3] = {DM4310PosPid1_Yaw_kp,DM4310PosPid1_Yaw_ki,DM4310PosPid1_Yaw_kd};
		const static fp32 DM_pid2[3] = {DM4310PosPid2_Yaw_kp,DM4310PosPid2_Yaw_ki,DM4310PosPid2_Yaw_kd};		
		const static fp32 pit_speed_pid[3] = {PIT_MOTOR_SPEED_PID_KP, PIT_MOTOR_SPEED_PID_KI, PIT_MOTOR_SPEED_PID_KD};		
		
	
		
		// Pitch 2006初始化
		init_ecd_record(&gimbal_motor2006_measure);
		move_PID_init(&move_data.pit_motor_data.move_angle_pid, PIT_MOTOR_ANGLE_PID_MAX_OUT, PIT_MOTOR_ANGLE_PID_MAX_IOUT, PIT_MOTOR_ANGLE_PID_KP, PIT_MOTOR_ANGLE_PID_KI, PIT_MOTOR_ANGLE_PID_KD);
		move_PID_init(&move_data.pit_motor_data.move_lock_pid, PIT_MOTOR_LOCK_PID_MAX_OUT, PIT_MOTOR_LOCK_PID_MAX_IOUT, PIT_MOTOR_LOCK_PID_KP, PIT_MOTOR_LOCK_PID_KI, PIT_MOTOR_LOCK_PID_KD);	
		move_PID_init(&move_data.pit_motor_data.move_gyro_pid, PIT_MOTOR_GYRO_PID_MAX_OUT, PIT_MOTOR_GYRO_PID_MAX_IOUT, PIT_MOTOR_GYRO_PID_KP, PIT_MOTOR_GYRO_PID_KI, PIT_MOTOR_GYRO_PID_KD);
		PID_init(&move_data.pit_motor_data.move_speed_pid, PID_POSITION, pit_speed_pid, PIT_MOTOR_SPEED_PID_MAX_OUT, PIT_MOTOR_SPEED_PID_MAX_IOUT);		
    fn_move_feedback_update();

    // Yaw DM4310电机初始化
    fn_DM_start_motor();
		
		move_data.yaw_motor_data.DM_kp = 0.0f;
		move_data.yaw_motor_data.DM_kd = 0.0f;	
		move_data.yaw_motor_data.DM_target_torque = 0.0f;	
		move_data.yaw_motor_data.motor_angle_set = 0.0f;			

}

//头部状态选择
void fn_moveMode(void)
{

	//自由模式，所有电机无力
	if(head_mode.head_mode == MODE_FREE)
		move_data.move_mode = MOVE_FREE;
	
	//Pitch轴复位模式或建图模式
	else if(head_mode.head_mode == MODE_PIT_RST || head_mode.head_mode == MODE_SLAM)
	{
		move_data.move_mode = MOVE_WORK;		
	}
	//陀螺仪模式
	else if(head_mode.head_mode == MODE_GYRO)
	{
		move_data.move_mode = MOVE_GYRO;		
	}
	//工作模式
	else if(head_mode.head_mode == MODE_WORK)
	{

		/*********点头动作*************************/
		//工作模式 按下●持续0.5s，执行点头
		if(ps2.button[13] == 1 && scan_t == 0)
		{
			vTaskDelay(500);
			if(ps2.button[13] == 1)	
				move_data.move_mode = MOVE_NOD;
		}
		//或通过串口收到NOD指令，执行点头
		else if(mode_uart == 1)
				move_data.move_mode = MOVE_NOD;			

		
		/*********摇头动作*************************/
		//工作模式 按下×持续0.5s，执行摇头		
		else if(ps2.button[14] == 1 && scan_t == 0)
		{
			vTaskDelay(500);
			if(ps2.button[14] == 1)	
				move_data.move_mode = MOVE_SHAKE;

		}	
		//或通过串口收到SHAKE指令，执行摇头		
		else if(mode_uart == 2)
				move_data.move_mode = MOVE_SHAKE;			
		
		
		/*********扫描动作************************/
		//工作模式 按下▲持续0.5s，全方位扫描
		else if(ps2.button[12] == 1 && scan_t == 0)
		{
			vTaskDelay(500);
			if(ps2.button[12] == 1)	
				move_data.move_mode = MOVE_SCAN_ALL;
		}			
		//或通过串口收到SCAN指令，执行扫描		
		else if(mode_uart == 3)
				move_data.move_mode = MOVE_SCAN_ALL;			

		/*********声源定位**********************/
		else if(mode_uart == 5)
				move_data.move_mode = MOVE_LOC;			
		
		/*********普通控制*********************/		
		//scan_t归零，说明动作结束
		else if(scan_t == 0)
			move_data.move_mode = MOVE_WORK;	
		//或通过串口收到STOP指令，停止动作回归work模式	
		if(mode_uart == 4)
		{
			mode_uart = 0;
			move_data.move_mode = MOVE_WORK;	
			scan_t=0;//动作中止，清空时间戳			
			direction=1;			
		}			
	}



    fn_moveMotorMode();	

}
void fn_moveMotorMode(void)
{
    if (move_data.move_mode == MOVE_FREE)
    {
        move_data.move_motor_yaw_mode = MOVE_MOTOR_DOWN;
        move_data.move_motor_pit_mode = MOVE_MOTOR_DOWN;
    }	
		
		//工作模式
    if (move_data.move_mode == MOVE_WORK)
    {
        move_data.move_motor_yaw_mode = MOVE_MOTOR_ENCONDE;
        move_data.move_motor_pit_mode = MOVE_MOTOR_ENCONDE;
			
			//若为Pit复位或SLAM模式，2006进入复位	
			if(head_mode.head_mode == MODE_PIT_RST || head_mode.head_mode == MODE_SLAM)
				{
					move_data.move_motor_pit_mode = MOVE_MOTOR_RST;					
				}
		}	
		
		//陀螺仪模式
    if (move_data.move_mode == MOVE_GYRO)
    {
        move_data.move_motor_yaw_mode = MOVE_MOTOR_GYRO;
        move_data.move_motor_pit_mode = MOVE_MOTOR_GYRO;
    }		
		
		//点头，pitch电机进入扫描模式		
    if (move_data.move_mode == MOVE_NOD)
    {
        move_data.move_motor_yaw_mode = MOVE_MOTOR_ENCONDE;
        move_data.move_motor_pit_mode = MOVE_MOTOR_WAVE;
    }			

		//摇头，Yaw电机进入扫描模式
    if (move_data.move_mode == MOVE_SHAKE)
    {
        move_data.move_motor_yaw_mode = MOVE_MOTOR_WAVE;
        move_data.move_motor_pit_mode = MOVE_MOTOR_ENCONDE;
    }		

		//全方位扫描，两电机均进入扫描模式		
    if (move_data.move_mode == MOVE_SCAN_ALL)
    {
        move_data.move_motor_yaw_mode = MOVE_MOTOR_SCAN;
        move_data.move_motor_pit_mode = MOVE_MOTOR_SCAN;
    }		
		//声源定位,Pitch电机正常工作，Yaw电机进入定位模式
    if (move_data.move_mode == MOVE_LOC)
    {
        move_data.move_motor_yaw_mode = MOVE_MOTOR_LOC;
        move_data.move_motor_pit_mode = MOVE_MOTOR_ENCONDE;
    }	
}

//头部电机数据更新
void fn_move_feedback_update(void)
{
		//2006数据更新
		move_data.pit_motor_data.motor_ecd = move_data.pit_motor_data.move_motor_measure->ecd;//更新电机ecd
		move_data.pit_motor_data.motor_speed = move_data.pit_motor_data.move_motor_measure->speed_rpm;//更新电机转速
		move_data.pit_motor_data.motor_angle = move_data.pit_motor_data.move_motor_measure->distance;	//更新Pitch绝对角度
		move_data.pit_motor_data.gyro_angle = filtered_avg_pitch_imu/PI * 180.0f;	//更新Pitch陀螺仪角度	
	
		//DM数据
		move_data.yaw_motor_data.motor_angle = move_data.yaw_motor_data.DM_motor_measure->position; //更新电机角度
		move_data.yaw_motor_data.motor_speed = move_data.yaw_motor_data.DM_motor_measure->velocity; //更新电机转速
}


//模式切换数据过渡
void fn_move_mode_change_control_transit(void)
{
	//自由模式进入工作模式，设定当前位置为目标位置
	if(move_data.move_mode == MOVE_WORK && 
		move_data.last_move_mode == MOVE_FREE)
	{
			move_data.yaw_motor_data.motor_angle_set = move_data.yaw_motor_data.motor_angle;
		
			//设置达妙电机位置pid参数
			move_data.yaw_motor_data.DM_kp = DM4310_MIT_P_KP;
			move_data.yaw_motor_data.DM_kd = DM4310_MIT_P_KD;
		
			//设置pitch 2006当前角度为目标值
			move_data.pit_motor_data.motor_angle_set = move_data.pit_motor_data.motor_angle;
	}
	
	//进入陀螺仪模式，设定当前绝对角度为目标值
	if(move_data.move_mode == MOVE_GYRO && move_data.last_move_mode != MOVE_GYRO)
	{
		move_data.yaw_motor_data.gyro_angle_set = move_data.yaw_motor_data.gyro_angle;
		move_data.pit_motor_data.gyro_angle_set = move_data.pit_motor_data.gyro_angle;
	}

}
// 计算头部电机电流
void fn_MoveControl(void)
{
  /*****2006目标角度设置***********************/
	
	//1  down模式
	if(move_data.move_motor_pit_mode == MOVE_MOTOR_DOWN)	
	
			move_data.pit_motor_data.give_current = 0.0f;
	
	//2  复位模式
	else if(move_data.move_motor_pit_mode == MOVE_MOTOR_RST)
	{
		if(rst_flag == 0)
		//未到极限位置，按目标速度解算Pitch电机电流			
		{
			move_data.pit_motor_data.motor_speed_set = PIT_RST_SPEED;
			move_data.pit_motor_data.give_current = (int16_t)PID_calc(&move_data.pit_motor_data.move_speed_pid, 
																													move_data.pit_motor_data.motor_speed, move_data.pit_motor_data.motor_speed_set);
			
		}

		else if(rst_flag == 1)
		//已到极限位置，SLAM模式下锁死在当前位置，使用LOCK模式PID			
		{
			move_data.pit_motor_data.give_current = move_PID_calc(&move_data.pit_motor_data.move_lock_pid, 
																																		move_data.pit_motor_data.move_motor_measure->distance, 
																																		move_data.pit_motor_data.motor_angle_set, 
																																		move_data.pit_motor_data.motor_speed);			
		}
		
		//电流不断增大，说明已经堵转		
		if(move_data.pit_motor_data.give_current > 4600 && rst_flag ==0 && INS_eulers[2] > 0.40f)
		{		
			rst_flag = 1;				
			
			//RST模式下，堵转时说明已到极限位置，重设2006零点
			if(head_mode.head_mode == MODE_PIT_RST)
			{			
				vTaskDelay(500);//延时保证稳定
				gimbal_motor2006_measure.round = 0;
				init_ecd_record(&gimbal_motor2006_measure);		
				
				//退出复位模式
				head_mode.head_mode = MODE_FREE;		
			}
			
			//SLAM模式下，堵转时说明已到极限位置，锁死
			else if(head_mode.head_mode == MODE_SLAM)
			{
				vTaskDelay(500);//延时保证稳定
				move_data.pit_motor_data.motor_angle_set = 	move_data.pit_motor_data.motor_angle;	
			}
		}
	}
	
	//3  遥控器控制模式
	else if(move_data.move_motor_pit_mode == MOVE_MOTOR_ENCONDE)	
	{
			move_data.pit_motor_data.motor_angle_set += ps2.joystick[1]*PS2_Coef_Pit;
			fn_Fp32Limit(&move_data.pit_motor_data.motor_angle_set,MIN_PIT_ANGLE,MAX_PIT_ANGLE);			
	}
	
	//自动扫描
	else if(move_data.move_motor_pit_mode == MOVE_MOTOR_WAVE)	
	{
			scan_t++;
		
			//角度按余弦函数变化
			move_data.pit_motor_data.motor_angle_set = MID_PIT_ANGLE/2 + MID_PIT_ANGLE/2 * sin(0.03*scan_t + PI/2);				
		
			//执行三次点头动作
			if(scan_t > 6*PI/0.03)
			{
				mode_uart = 0;	//串口状态指示变量清零
				scan_t=0;
			}

	}
	//4  陀螺仪模式
	else if(move_data.move_motor_pit_mode == MOVE_MOTOR_GYRO)
	{
			move_data.pit_motor_data.gyro_angle_set += ps2.joystick[1]*PS2_Coef_Pit*0.5;		
			move_data.pit_motor_data.give_current = move_PID_calc(&move_data.pit_motor_data.move_gyro_pid, 
																																		move_data.pit_motor_data.gyro_angle, 
																																		move_data.pit_motor_data.gyro_angle_set, 
																																		INS_gyro[0] /PI*180.0f);						
	}
	
	/*****DM4310目标角度设置********************/
	
	//down模式
	if(move_data.move_motor_yaw_mode == MOVE_MOTOR_DOWN)		
	{
			//DM4310 全部设为0
			move_data.yaw_motor_data.motor_angle_set = 0.0f;
			move_data.yaw_motor_data.DM_kp = 0.0f;
			move_data.yaw_motor_data.DM_kd = 0.0f;		
			move_data.yaw_motor_data.DM_target_torque = 0.0f;			
	}
	
	//遥控器控制模式
	else if(move_data.move_motor_yaw_mode == MOVE_MOTOR_ENCONDE)	
	{
			move_data.yaw_motor_data.motor_angle_set += ps2.joystick[2]*PS2_Coef_Yaw;			
			fn_Fp32Limit(&move_data.yaw_motor_data.motor_angle_set,MIN_YAW_ANGLE,MAX_YAW_ANGLE);	//输出限幅	
	}
	else if(move_data.move_motor_yaw_mode == MOVE_MOTOR_LOC)	
	{
			scan_t++;		
			if(angle_uart > 80 && angle_uart < 180)	//80-180°，转到最大值
				move_data.yaw_motor_data.motor_angle_set = MAX_YAW_ANGLE;
			
			else if(angle_uart >= 180 && angle_uart < 280)	//180-280°，转到最小值
				move_data.yaw_motor_data.motor_angle_set = MIN_YAW_ANGLE;			
			
			else if(angle_uart >= 280 && angle_uart <= 360)	//280-360°，按定位角度设置目标值
				move_data.yaw_motor_data.motor_angle_set = angle_uart * 2*PI/360 - 2*PI + MID_YAW_ANGLE;				
			
			else if(angle_uart >= 0 && angle_uart <= 80)	//0-80°，按定位角度设置目标值
				move_data.yaw_motor_data.motor_angle_set = angle_uart * 2*PI/360 + MID_YAW_ANGLE;	
			
			if(scan_t > 1000)
			{
				scan_t=0;				

				if((angle_uart >= 280 && angle_uart < 360) || (angle_uart >= 0 && angle_uart <= 80))
				{
					mode_uart = 0;	//串口状态指示变量清零
					move_data.yaw_motor_data.motor_angle_set = MID_YAW_ANGLE;					//Yaw归中					
				}
				else
					mode_uart = 2;	//摇头表示看不到
			}
	}
	
	//自动扫描
	else if(move_data.move_motor_yaw_mode == MOVE_MOTOR_WAVE)		
	{
			scan_t++;			
		
			//角度按余弦函数变化
			move_data.yaw_motor_data.motor_angle_set = MID_YAW_ANGLE + PI/4 * sin(0.02*scan_t);				
		
			//执行三次摇头动作
			if(scan_t > 6*PI/0.02)
			{
				mode_uart = 0;	//串口状态指示变量清零				
				scan_t=0;
			}
	}
	
		/******联合扫描模式*************/
    if (move_data.move_mode == MOVE_SCAN_ALL)
		{
			scan_t++;			
			
			//p轴匀速扫描
			if(move_data.pit_motor_data.motor_angle_set > MAX_PIT_ANGLE)
			{
					move_data.pit_motor_data.motor_angle_set = MAX_PIT_ANGLE;
					direction = 1; // 反向
			}
			else if(move_data.pit_motor_data.motor_angle_set < MIN_PIT_ANGLE)
			{
					move_data.pit_motor_data.motor_angle_set = MIN_PIT_ANGLE;
					direction = -1; // 正向
			}

			//Pitch轴匀速线性变化
			move_data.pit_motor_data.motor_angle_set += -direction * 0.51;						
			//Yaw轴角度按余弦函数变化			
			move_data.yaw_motor_data.motor_angle_set = MID_YAW_ANGLE + 1.4 * sin(0.0025*scan_t);				
		
			//执行两次扫描动作
			if(scan_t > 4*PI/0.0025)
			{
				mode_uart = 0;	//串口状态指示变量清零						
				scan_t=0;		
				direction=1;
			}
		}
		 
	 //按下左摇杆，归中到初始位置
		if(move_data.move_mode == MOVE_WORK && ps2.button[1] == 1)
		{
			vTaskDelay(100);
			if(ps2.button[1] == 1)
			{
				move_data.yaw_motor_data.motor_angle_set = MID_YAW_ANGLE;
				move_data.pit_motor_data.motor_angle_set = -25.0f;			
			}
		}
	
		//电机模式非down、非复位、非陀螺仪时，按目标角度解算pitch轴电流
		if(move_data.move_motor_pit_mode != MOVE_MOTOR_DOWN && move_data.move_motor_pit_mode != MOVE_MOTOR_RST && move_data.move_motor_pit_mode != MOVE_MOTOR_GYRO)
		{
			//工作模式下，接近目标位置改用锁死pid
			if(fabs(move_data.pit_motor_data.motor_angle_set - move_data.pit_motor_data.motor_angle) < 10.0f 
				&& move_data.move_mode == MODE_WORK)
			{
				move_data.pit_motor_data.give_current = move_PID_calc(&move_data.pit_motor_data.move_lock_pid, 
																																		move_data.pit_motor_data.move_motor_measure->distance, 
																																		move_data.pit_motor_data.motor_angle_set, 
																																		move_data.pit_motor_data.motor_speed);
			}
			//运动过程中使用普通角度pid
			else
				move_data.pit_motor_data.give_current = move_PID_calc(&move_data.pit_motor_data.move_angle_pid, 
																																		move_data.pit_motor_data.move_motor_measure->distance, 
																																		move_data.pit_motor_data.motor_angle_set, 
																																		move_data.pit_motor_data.motor_speed);		
		}

 

}

//pid初始化函数
static void move_PID_init(move_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
{
    if (pid == NULL)
    {
        return;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->err = 0.0f;
    pid->get = 0.0f;

    pid->max_iout = max_iout;
    pid->max_out = maxout;
}

//pid计算函数
static fp32 move_PID_calc(move_PID_t *pid, fp32 get, fp32 set, fp32 error_delta)
{
    fp32 err;
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->get = get;
    pid->set = set;

    err = set - get;
    pid->err = err;
    pid->Pout = pid->kp * pid->err;
    pid->Iout += pid->ki * pid->err;
    pid->Dout = pid->kd * error_delta;
    abs_limit(&pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    abs_limit(&pid->out, pid->max_out);
    return pid->out;
}

