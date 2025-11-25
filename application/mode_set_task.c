/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       mode_set_task.c/h
  * @brief      自由模式/工作模式切换任务
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     March-25-2024   Ignis             1. done
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "main.h"
#include "cmsis_os.h"
#include "mode_set_task.h"
#include "ps2_typedef.h"
#include "ps2_typedef.h"

extern Ps2_s ps2;//引用遥控器数据

static void mode_init(head_mode_t *head_mode_init);
extern Ps2_s ps2;//引用遥控器数据

extern int rst_flag;//外部引用复位完成标识


/**
  * @brief          set gimbal control mode, mainly call 'gimbal_behaviour_mode_set' function
  * @param[out]     gimbal_set_mode: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          ???????????g????????'gimbal_behaviour_mode_set'??????i?
  * @param[out]     gimbal_set_mode:"gimbal_control"???????.
  * @retval         none
  */
static void mode_set(head_mode_t *head_mode_set);
/**
  * @brief          runner some measure data updata, such as motor enconde, euler angle, gyro
  * @param[out]     runner_feedback_update: "runner_act" valiable point
  * @retval         none
  */
/**
  * @brief          ???_?????????????????????y???????????????
  * @param[out]     runner_feedback_update:"runner_act"???????.
  * @retval         none
  */


head_mode_t head_mode;
/**
  * @brief          runner_task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          模式设置任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void mode_set_task(void const * argument)
{
	 //wait a time 
    //????h?????
    vTaskDelay(MODE_SET_TASK_INIT_TIME);
    //chassis init
    //?????'??
    mode_init(&head_mode);
		//上电自动复位
		head_mode.head_mode = MODE_PIT_RST;
		vTaskDelay(3000);

    while(1)
    {

			
			mode_set(&head_mode);                    //???????????g?
			vTaskDelay(MODE_SET_TIME_MS);
			head_mode.last_head_mode = head_mode.head_mode;
			

    }
}

/**
  * @brief          "runner_act" valiable initialization, include pid initialization, remote control data point initialization, runner motor
  *                 data point initialization.
  * @param[out]     runner_act_init: "runner_act" valiable point
  * @retval         none
  */
/**
  * @brief          ??'??"runner_act"??????????pid??'???? ?????????'????3508??????????'????????????'????????????????'??
  * @param[out]     runner_act_init:"runner_act"???????.
  * @retval         none
  */
static void mode_init(head_mode_t *head_mode_init)
{
	  if (head_mode_init == NULL)
    {
        return;
    }

    //runner motor speed PID
    //????????pid?
		head_mode_init->last_head_mode = head_mode_init->head_mode = MODE_FREE;
		
}

/**
  * @brief          set gimbal control mode, mainly call 'gimbal_behaviour_mode_set' function
  * @param[out]     gimbal_set_mode: "gimbal_control" valiable point
  * @retval         none
  */
/**
  * @brief          ???????????g????????'gimbal_behaviour_mode_set'??????i?
  * @param[out]     gimbal_set_mode:"gimbal_control"???????.
  * @retval         none
  */
static void mode_set(head_mode_t *head_mode_set)
{
    if (head_mode_set == NULL)
    {
        return;
    }

		
		if(ps2.mode != 115)
		{
			head_mode_set->head_mode = MODE_FREE;
		}
		else
		{
			//自由模式下ps手柄按下start持续1s，进入工作模式
			if(head_mode_set->head_mode == MODE_FREE && ps2.button[3] == 1)
			{
				vTaskDelay(1000);		
				
				if(ps2.button[3] == 1)	
					head_mode_set->head_mode = MODE_WORK;
			}
			
			//自由模式下ps手柄按下■持续1s，执行Pitch轴复位
			else if(head_mode_set->head_mode == MODE_FREE && ps2.button[15] == 1)
			{
				vTaskDelay(1000);		
				
				if(ps2.button[15] == 1)	
				{
					head_mode_set->head_mode = MODE_PIT_RST;
					
					//复位标志清零
					rst_flag = 0;
				}
			}
			//自由模式下ps手柄按下R1持续1s，进入陀螺仪模式
			else if(head_mode_set->head_mode == MODE_FREE && ps2.button[11] == 1)
			{
				vTaskDelay(1000);		
				
				if(ps2.button[11] == 1)	
					head_mode_set->head_mode = MODE_GYRO;			
			}
			
			//自由模式下ps手柄按下L1持续1s，进入SLAM建图模式
			else if(head_mode_set->head_mode == MODE_FREE && ps2.button[10] == 1)
			{
				vTaskDelay(1000);		
				
				if(ps2.button[10] == 1)	
				{
					head_mode_set->head_mode = MODE_SLAM;
					
					//复位标志清零
					rst_flag = 0;					
				}
			}			
		}
			
		//非自由模式下ps手柄按下select持续0.5s，进入自由模式
		if(head_mode_set->head_mode != MODE_FREE && ps2.button[0] == 1)
		{
			vTaskDelay(500);		
			if(ps2.button[0] == 1)	
			{
				head_mode_set->head_mode = MODE_FREE;
			}
		}
		
}




