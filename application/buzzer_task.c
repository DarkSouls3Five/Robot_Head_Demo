#include "buzzer_task.h"
#include "cmsis_os.h"
#include "tim.h"
#include "gpio.h"
#include "calibrate_task.h"
#include "mode_set_task.h"

//音符？音调？感兴趣的可以自己设计一款车载音乐
const uint32_t btnDelay2=1.5*300;//0.5
const uint32_t btnDelay3=600;//1
const uint32_t btnDelay4=2*150;//0.25
const uint32_t btnDelay5=700;//1+0.5

void tim1(uint16_t tune)
{
	__HAL_TIM_SET_AUTORELOAD(&htim4,tune);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0.1*tune);
	vTaskDelay(btnDelay2);
}

void tim2(uint16_t tune)
{
	__HAL_TIM_SET_AUTORELOAD(&htim4,tune);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0.1*tune);
	vTaskDelay(btnDelay3);
}

void tim3(uint16_t tune)
{
	__HAL_TIM_SET_AUTORELOAD(&htim4,tune);
    __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0.1*tune);
	vTaskDelay(btnDelay4);
}

void tim4(uint16_t tune)
{
	__HAL_TIM_SET_AUTORELOAD(&htim4,tune);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0.1*tune);
	vTaskDelay(btnDelay5);
}
//*******************

void fn_BuzzerPrompt(void);
void fn_BuzzerClose(void);
void fn_BuzzerWork(void);
void fn_BuzzerFree(void);
extern head_mode_t head_mode;
uint8_t BuzzerCount = 0;
uint8_t ModeCount = 0;


void Buzzer_Task(void const * argument){
	
	vTaskDelay(200);
	
	//启动提示音
	fn_BuzzerPrompt();

	//关闭提示音
	fn_BuzzerClose();

    while(1){

        //进入陀螺仪校准模式或者校准完成提示音
			if((calibrate_flag == 1 && BuzzerCount == 0) || (calibrate_flag == 0 && BuzzerCount == 1))
			{
				fn_BuzzerPrompt();
				BuzzerCount++;
			}
			if(calibrate_flag == 0 && BuzzerCount == 2)
			{
				fn_BuzzerClose();
				BuzzerCount = 0;
			}
		//进入或退出工作模式提示音
			if(head_mode.head_mode == MODE_WORK && ModeCount == 0)
			{
//				fn_BuzzerWork();
				fn_BuzzerClose();				
				ModeCount = 1;
			}
			if(head_mode.head_mode == MODE_FREE && ModeCount == 1)
			{
//				fn_BuzzerFree();
				fn_BuzzerClose();				
				ModeCount = 0;
			}			
			vTaskDelay(100);
	}
}


//提示音
void fn_BuzzerPrompt(void){
	tim1(M6);   //450
	tim1(M7);   //450

	tim3(H1);   //300
	tim1(M7);   //450

	tim2(0);
}

//进入工作模式
void fn_BuzzerWork(void){
	tim1(M6);   //450
	tim1(M7);   //450

//	tim3(H1);   //300
//	tim1(M7);   //450

	tim2(0);
}

//退出工作模式
void fn_BuzzerFree(void){
//	tim1(M6);   //450
//	tim1(M7);   //450

	tim3(H1);   //300
	tim1(M7);   //450

	tim2(0);
}

//天空之城
void fn_BuzzerSinging(void){
      tim2(0);
			tim2(0);    //600
		  tim2(0);    //600
	    tim1(M6);   //450
			tim1(M7);   //450

			tim3(H1);   //300
			tim1(M7);   //450
			tim2(H1);   //600
			tim2(H3);   //600

			tim4(M7);   //700
	    tim2(0); 

			tim2(0);
			tim2(M7);
			tim1(M7);
			tim1(M3);
			tim1(M3);

			tim3(M6);
			tim1(M5);
			tim2(M6);
			tim2(H1);

			tim4(M5);
			tim4(M5);
			tim4(M5);
			tim2(0);
			tim2(M3);

//			tim3(M5,duration3);
//			tim1(M3,duration3);
			tim3(M4);
			tim1(M3);
			tim2(M4);
			tim2(H1);

			tim3(M3);
			tim1(M3);
			tim1(ZERO);
			tim1(H1);
			tim1(H1);
			tim1(H1);
//			59.
			tim3(M7);
			tim1(half_M4);
			tim2(M4);
			tim2(M7);

			tim4(M7);
			tim2(M7);
			tim2(ZERO);
			tim1(M6);
			tim1(M7);
//			69

			tim3(H1);
			tim1(M7);
			tim2(H1);
			tim2(H3);

			tim4(M7);
			tim1(M7);
			tim1(M7);
			tim2(ZERO);
			tim1(M3);
			tim1(M3);

			tim3(M6);
			tim1(M5);
			tim2(M6);
			tim2(H1);
//			84
			tim4(M5);
			tim2(M5);
			tim1(M5);
			tim2(ZERO);
			tim1(M2);
			tim1(M3);

			tim2(M4);
			tim1(H1);
			tim2(M7);
			tim2(M7);
			tim3(H1);
			tim3(H1);

			tim1(H2);
			tim1(H2);
			tim1(H3);
			tim3(H1);
			tim2(ZERO);
//			100
			tim1(H1);
			tim1(H1);
			tim1(H1);
			tim1(M7);
			tim1(M6);
			tim1(M6);
			tim2(M7);
			tim2(half_M5);
			tim4(M6);
			tim4(M6);
			tim4(M6);
			tim1(H1);
			tim1(H2);

			tim3(H3);
			tim1(H2);
			tim2(H3);
			tim2(H5);

			tim4(H2);
			tim4(H2);
			tim4(H2);
			tim1(M5);
			tim1(M5);

			tim3(H1);
			tim1(M7);
			tim2(H1);
			tim2(H3);

			tim1(H3);
			tim1(H3);
			tim1(H3);
			tim1(H3);

			tim1(M6);
			tim1(M7);
			tim2(H1);
			tim2(M7);
			tim1(H2);
			tim1(H2);

			tim3(H1);
			tim1(M5);
			tim3(M5);
			tim1(M5);

			tim2(H4);
			tim2(H3);
			tim2(H2);
			tim2(H1);

			tim4(H3);
			tim2(H3);
			tim2(H3);
			tim2(H3);

			tim4(H6);
			tim4(H6);
			tim2(H5);
			tim2(H5);
			tim1(H3);
			tim1(H2);
			tim4(H1);
			tim4(H1);
			tim1(ZERO);
			tim1(H1);

			tim2(H2);
			tim1(H1);
			tim1(H2);
			tim2(H2);
			tim2(H5);

			tim1(H3);
			tim1(H3);
			tim1(H3);
			tim2(ZERO);
			tim1(H3);

			tim4(H6);
			tim4(H6);
			tim4(H5);
			tim4(H5);

			tim1(H3);
			tim1(H2);
			tim4(H1);
			tim4(H1);
			tim1(ZERO);
			tim1(H1);

			tim2(H2);
			tim1(H1);
			tim2(H2);
			tim2(H2);

			tim2(M7);
			tim2(M6);
}

//关闭蜂鸣器
void fn_BuzzerClose(void){
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0);
}
