/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId led_taskHandle;
osThreadId ins_taskHandle;
osThreadId calibrate_taskHandle;
osThreadId buzzer_taskHandle;
osThreadId PS2_ControlHandle;
osThreadId Move_TaskHandle;
osThreadId ModeSet_TaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void Led_Task(void const * argument);
void Ins_Task(void const * argument);
void Calibrate_Task(void const * argument);
void Buzzer_Task(void const * argument);
void PS2_task(void const * argument);
void move_task(void const * argument);
void mode_set_task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of led_task */
  osThreadDef(led_task, Led_Task, osPriorityLow, 0, 128);
  led_taskHandle = osThreadCreate(osThread(led_task), NULL);

  /* definition and creation of ins_task */
  osThreadDef(ins_task, Ins_Task, osPriorityHigh, 0, 1024);
  ins_taskHandle = osThreadCreate(osThread(ins_task), NULL);

  /* definition and creation of calibrate_task */
  osThreadDef(calibrate_task, Calibrate_Task, osPriorityNormal, 0, 128);
  calibrate_taskHandle = osThreadCreate(osThread(calibrate_task), NULL);

  /* definition and creation of buzzer_task */
  osThreadDef(buzzer_task, Buzzer_Task, osPriorityNormal, 0, 128);
  buzzer_taskHandle = osThreadCreate(osThread(buzzer_task), NULL);

  /* definition and creation of PS2_Control */
  osThreadDef(PS2_Control, PS2_task, osPriorityNormal, 0, 128);
  PS2_ControlHandle = osThreadCreate(osThread(PS2_Control), NULL);

  /* definition and creation of Move_Task */
  osThreadDef(Move_Task, move_task, osPriorityHigh, 0, 1024);
  Move_TaskHandle = osThreadCreate(osThread(Move_Task), NULL);

  /* definition and creation of ModeSet_Task */
  osThreadDef(ModeSet_Task, mode_set_task, osPriorityAboveNormal, 0, 128);
  ModeSet_TaskHandle = osThreadCreate(osThread(ModeSet_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Led_Task */
/**
* @brief Function implementing the led_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Led_Task */
__weak void Led_Task(void const * argument)
{
  /* USER CODE BEGIN Led_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Led_Task */
}

/* USER CODE BEGIN Header_Ins_Task */
/**
* @brief Function implementing the ins_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Ins_Task */
__weak void Ins_Task(void const * argument)
{
  /* USER CODE BEGIN Ins_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Ins_Task */
}

/* USER CODE BEGIN Header_Calibrate_Task */
/**
* @brief Function implementing the calibrate_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Calibrate_Task */
__weak void Calibrate_Task(void const * argument)
{
  /* USER CODE BEGIN Calibrate_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Calibrate_Task */
}

/* USER CODE BEGIN Header_Buzzer_Task */
/**
* @brief Function implementing the buzzer_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Buzzer_Task */
__weak void Buzzer_Task(void const * argument)
{
  /* USER CODE BEGIN Buzzer_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Buzzer_Task */
}

/* USER CODE BEGIN Header_PS2_task */
/**
* @brief Function implementing the PS2_Control thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PS2_task */
__weak void PS2_task(void const * argument)
{
  /* USER CODE BEGIN PS2_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END PS2_task */
}

/* USER CODE BEGIN Header_move_task */
/**
* @brief Function implementing the Move_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_move_task */
__weak void move_task(void const * argument)
{
  /* USER CODE BEGIN move_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END move_task */
}

/* USER CODE BEGIN Header_mode_set_task */
/**
* @brief Function implementing the ModeSet_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_mode_set_task */
__weak void mode_set_task(void const * argument)
{
  /* USER CODE BEGIN mode_set_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END mode_set_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
