#ifndef _INS_TASK_H_
#define _INS_TASK_H_

#include "struct_typedef.h"

//******IMU的PID温度控制参数
//bmi温度
#define ImuTemp 50.0f
//PID参数
#define ImuTemp_kp 1600.0f
#define ImuTemp_ki 0.2f
#define ImuTemp_kd 0.0f

#define  ImuTempMInOut -4500.0f
#define  ImuTempMaxOut 4500.0f
#define  ImuTempMInIOut -1000.0f
#define  ImuTempMaxIOut 1000.0f
//******

#define sampleFreq	1000.0f			//陀螺仪样本频率Hz
#define twoKpDef	(2.0f * 0.5f)	// 2 * proportional gain
#define twoKiDef	(2.0f * 0.0f)	// 2 * integral gain






//用于DMA接收陀螺仪数据的东西，不需要修改与理解
#define IMU_DR_SHFITS        0
#define IMU_SPI_SHFITS       1
#define IMU_UPDATE_SHFITS    2
#define IMU_NOTIFY_SHFITS    3

#define SPI_DMA_GYRO_LENGHT       8
#define SPI_DMA_ACCEL_LENGHT      9
#define SPI_DMA_ACCEL_TEMP_LENGHT 4

extern fp32 INS_gyro[3];
extern fp32 INS_quat[4];
extern fp32 INS_eulers[3];
extern fp32 INS_accel_z[6];

void Ins_Task(void const * argument);

//INS数据初始化
void fn_INSInit(void);

//Mahony六轴融合
void fn_MahonyAHRSupdateIMU(float q[4], float gx, float gy, float gz, float ax, float ay, float az);

//陀螺仪温度控制函数
void imu_temp_control(fp32 temp);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

void imu_cmd_spi_dma(void);

void DMA2_Stream2_IRQHandler(void);

#endif
