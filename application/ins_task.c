#include "ins_task.h"
#include "main.h"
#include "struct_typedef.h"
#include "BMI088driver.h"
#include "cmsis_os.h"
#include "BMI088Middleware.h"
#include "pid.h"
#include "bsp_spi.h"
#include "tim.h"
#include "stm32f4xx_it.h"
#include "math_lib.h"
#include "math.h"
#include "gimbal_task.h"
#include "filter.h"
#include "calibrate_task.h"


uint8_t ins_num = 0;

void fn_INSInit(void);
void fn_AHRS_init(fp32 quat[4], fp32 accel[3], fp32 mag[3]);
void fn_MahonyAHRSupdateIMU(float q[4], float gx, float gy, float gz, float ax, float ay, float az);
void quaternion2eulers(fp32 xyzw[], fp32 eulers[], int axis0, int axis1, int axis2, bool_t extrinsic);
void imu_temp_control(fp32 temp);
void imu_cmd_spi_dma(void);

//用于DMA接收陀螺仪数据的东西
extern SPI_HandleTypeDef hspi1;
extern DMA_HandleTypeDef hdma_spi1_rx;
static TaskHandle_t INS_task_local_handler;
uint8_t gyro_dma_rx_buf[8];
uint8_t gyro_dma_tx_buf[8] = {0x82,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
uint8_t accel_dma_rx_buf[9];
uint8_t accel_dma_tx_buf[9] = {0x92,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
uint8_t accel_temp_dma_rx_buf[4];
uint8_t accel_temp_dma_tx_buf[4] = {0xA2,0xFF,0xFF,0xFF};
volatile uint8_t gyro_update_flag = 0;
volatile uint8_t accel_update_flag = 0;
volatile uint8_t accel_temp_update_flag = 0;
volatile uint8_t imu_start_dma_flag = 0;
uint8_t first_temperate;
float timing_time = 0.001f;   //任务运行的时间 单位 s
//******

volatile float twoKp = twoKpDef;
volatile float twoKi = twoKiDef;
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;

//陀螺仪温度加热PID参数
fp32 af_ImuTempPID[3] = {ImuTemp_kp,ImuTemp_ki,ImuTemp_kd};
//imu温度的PID数据类型
pid_type_def_new imu_temp_pid;
//bmi088传回的数据
bmi088_real_data_t bmi088_real_data;
//零飘校准后的角速度
fp32 INS_gyro[3] = {0.0f, 0.0f, 0.0f};
//滤波后的加速度计数据
fp32 INS_accel[3] = {0.0f, 0.0f, 0.0f};
//四元数
fp32 INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
//解算出的欧拉角 YAW PITCH ROLL
fp32 INS_eulers[3] = {0.0f, 0.0f, 0.0f};


void Ins_Task(void const * argument){
	
    osDelay(7);
    while(BMI088_init())
    {
        osDelay(100);
    }

    fn_INSInit();

    //通过PID用PWM控制陀螺仪温度参数初始化
    fn_PidInit(&imu_temp_pid, af_ImuTempPID,ImuTempMInOut,ImuTempMaxOut,ImuTempMInIOut,ImuTempMaxIOut );                

	//获取当前任务的任务句柄，
    INS_task_local_handler = xTaskGetHandle(pcTaskGetName(NULL));

	//设置SPI分频值
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    
    //DMA初始化
    SPI1_DMA_init((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, 8);
	
    //可以开始更新陀螺仪数据
	imu_start_dma_flag = 1;
			
    while(1){
		
	    //wait spi DMA tansmit done
        //等待SPI DMA传输，通知进来之后，去跑任务
        while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS){
        }

        //接收六轴数据
        if(gyro_update_flag & (1 << 3)){
            gyro_update_flag &= ~(1 << 3);
            BMI088_gyro_read_over(gyro_dma_rx_buf + 1, bmi088_real_data.gyro);
        }

        if(accel_update_flag & (1 << 2)){
            accel_update_flag &= ~(1 << 2);
            BMI088_accel_read_over(accel_dma_rx_buf + 2, bmi088_real_data.accel, &bmi088_real_data.time);
        }

        if(accel_temp_update_flag & (1 << 2)){
            accel_temp_update_flag &= ~(1 << 2);
            BMI088_temperature_read_over(accel_temp_dma_rx_buf + 2, &bmi088_real_data.temp);
            imu_temp_control(bmi088_real_data.temp);
        }
	    
        //传感器角速度减去零飘值
        INS_gyro[0] = bmi088_real_data.gyro[0] - gyro_x_zero;
        INS_gyro[1] = bmi088_real_data.gyro[1] - gyro_y_zero;
        INS_gyro[2] = bmi088_real_data.gyro[2] - gyro_z_zero;

        //加速度一阶低通滤波
        //INS_accel[0] = INS_accel[0] * 0.94f + bmi088_real_data.accel[0] * 0.06f;
        //INS_accel[1] = INS_accel[1] * 0.94f + bmi088_real_data.accel[1] * 0.06f;
        //INS_accel[2] = INS_accel[2] * 0.94f + bmi088_real_data.accel[2] * 0.06f;
        fn_low_filter(&INS_accel[0],bmi088_real_data.accel[0],0.06f);
        fn_low_filter(&INS_accel[1],bmi088_real_data.accel[1],0.06f);
        fn_low_filter(&INS_accel[2],bmi088_real_data.accel[2],0.06f);

        //六轴数据融合更新四元数
        fn_MahonyAHRSupdateIMU(INS_quat,INS_gyro[0],INS_gyro[1],INS_gyro[2],INS_accel[0],INS_accel[1],INS_accel[2]);

        //四元数数组转换为欧拉角解算方程的四元数数组顺序  即系数qx顺序改为 i j k r
        fp32 q[4] = {INS_quat[1], INS_quat[2], INS_quat[3], INS_quat[0]};
        
        //解算欧拉角
        quaternion2eulers(q,INS_eulers,2,1,0,0);

	}
}


//INS数据初始化
void fn_INSInit(void){
    BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);

    INS_accel[0] = bmi088_real_data.accel[0];
    INS_accel[1] = bmi088_real_data.accel[1];
    INS_accel[2] = bmi088_real_data.accel[2];
    
    fn_AHRS_init(INS_quat,bmi088_real_data.accel,bmi088_real_data.gyro);
    //INS_quat[0] = 1.0f;
    //INS_quat[1] = 0.0f;
    //INS_quat[2] = 0.0f;
    //INS_quat[3] = 0.0f;
}

void fn_AHRS_init(fp32 quat[4], fp32 accel[3], fp32 mag[3])
{
    fp32 recipNorm;
    fp32 init_yaw, init_pitch, init_roll;
    fp32 cr2, cp2, cy2, sr2, sp2, sy2;
    fp32 q0 = quat[0], q1 = quat[1], q2 = quat[2], q3 = quat[3];
    fp32 ax = accel[0], ay = accel[1], az = accel[2];

    recipNorm = fn_InvSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    init_pitch = atan2f(-ax, az);
    init_roll = atan2f(ay, az);
    init_yaw  = 0.0f;

    cr2 = cosf(init_roll * 0.5f);
    cp2 = cosf(init_pitch * 0.5f);
    cy2 = cosf(init_yaw * 0.5f);
    sr2 = sinf(init_roll * 0.5f);
    sp2 = sinf(init_pitch * 0.5f);
    sy2 = sinf(init_yaw * 0.5f);

    q0 = cr2 * cp2 * cy2 + sr2 * sp2 * sy2;
    q1= sr2 * cp2 * cy2 - cr2 * sp2 * sy2;
    q2 = cr2 * sp2 * cy2 + sr2 * cp2 * sy2;
    q3= cr2 * cp2 * sy2 - sr2 * sp2 * cy2;

    recipNorm = fn_InvSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    quat[0] = q0;
    quat[1] = q1;
    quat[2] = q2;
    quat[3] = q3;
}

//Mahony六轴融合
void fn_MahonyAHRSupdateIMU(float q[4], float gx, float gy, float gz, float ax, float ay, float az){
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = fn_InvSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;        

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = q[1] * q[3] - q[0] * q[2];
		halfvy = q[0] * q[1] + q[2] * q[3];
		halfvz = q[0] * q[0] - 0.5f + q[3] * q[3];
	
		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q[0];
	qb = q[1];
	qc = q[2];
	q[0] += (-qb * gx - qc * gy - q[3] * gz);
	q[1] += (qa * gx + qc * gz - q[3] * gy);
	q[2] += (qa * gy - qb * gz + q[3] * gx);
	q[3] += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	recipNorm = fn_InvSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	q[0] *= recipNorm;
	q[1] *= recipNorm;
	q[2] *= recipNorm;
	q[3] *= recipNorm;
}


//四元数转欧拉角
// 四元数转欧拉角
// x = 0, y = 1, z = 2
// e.g. 先绕z轴旋转，再绕y轴旋转，最后绕x轴旋转：axis0=2, axis1=1, axis2=0
// 参考：https://github.com/evbernardes/quaternion_to_euler
void quaternion2eulers(fp32 xyzw[], fp32 _eulers[], int axis0, int axis1, int axis2, bool_t extrinsic){

	float eulers[3];
	
  if (!extrinsic) {
    int temp = axis0;
    axis0 = axis2;
    axis2 = temp;
  }
  int i = axis0, j = axis1, k = axis2;
  bool_t is_proper = (i == k);
  if (is_proper) k = 3 - i - j;
  int sign = (i - j) * (j - k) * (k - i) / 2;
  double a, b, c, d;
  if (is_proper) {
    a = xyzw[3];
    b = xyzw[i];
    c = xyzw[j];
    d = xyzw[k] * sign;
  } else {
    a = xyzw[3] - xyzw[j];
    b = xyzw[i] + xyzw[k] * sign;
    c = xyzw[j] + xyzw[3];
    d = xyzw[k] * sign - xyzw[i];
  }
  double n2 = a * a + b * b + c * c + d * d;
  eulers[1] = acos(2 * (a * a + b * b) / n2 - 1);
  double eps = 1e-7;
  bool_t safe1 = fabs(eulers[1]) >= eps;
  bool_t safe2 = fabs(eulers[1] - PI) >= eps;
  bool_t safe = safe1 && safe2;
  double half_sum = atan2(b, a);
  double half_diff = atan2(-d, c);
  if (safe) {
    eulers[0] = half_sum + half_diff;
    eulers[2] = half_sum - half_diff;
  } else {
    if (!extrinsic) {
      eulers[0] = 0;
      if (!safe1) eulers[2] = 2 * half_sum;
      if (!safe2) eulers[2] = -2 * half_diff;
    } else {
      eulers[2] = 0;
      if (!safe1) eulers[0] = 2 * half_sum;
      if (!safe2) eulers[0] = 2 * half_diff;
    }
  }
  for (int i = 0; i < 3; i++) {
      while (eulers[i] > PI) eulers[i] -= 2 * PI;
      while (eulers[i] <= -PI) eulers[i] += 2 * PI;
  }
  if (!is_proper) {
    eulers[2] *= sign;
    eulers[1] -= PI / 2;
  }
  if (!extrinsic){
    fp32 temp = eulers[0];
    eulers[0] = eulers[2];
    eulers[2] = temp;
  }
	_eulers[0] = eulers[0];
	_eulers[1] = eulers[1];
	_eulers[2] = eulers[2];
}


//陀螺仪温度控制函数
void imu_temp_control(fp32 temp)
{
    uint16_t tempPWM;
    static uint8_t temp_constant_time = 0;
    if (first_temperate)
    {
       fn_PidClac(&imu_temp_pid, temp, ImuTemp);   //目标温度值
        if (imu_temp_pid.f_Out < 0.0f)
        {
            imu_temp_pid.f_Out = 0.0f;
        }
        tempPWM = (uint16_t)imu_temp_pid.f_Out;
         __HAL_TIM_SetCompare(&htim10, TIM_CHANNEL_1, tempPWM);
    }
    else
    {
        //在没有达到设置的温度，一直最大功率加热
        //in beginning, max power
        if (temp > ImuTemp)
        {
            temp_constant_time++;
            if (temp_constant_time > 200)
            {
                
                first_temperate = 1;
                
            }
        }
        __HAL_TIM_SetCompare(&htim10, TIM_CHANNEL_1, 4999);
    }
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == INT1_ACCEL_Pin)
    {
        accel_update_flag |= 1 << IMU_DR_SHFITS;
        accel_temp_update_flag |= 1 << IMU_DR_SHFITS;
        if(imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    }
    else if(GPIO_Pin == INT1_GYRO_Pin)
    {
        gyro_update_flag |= 1 << IMU_DR_SHFITS;
        if(imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    }
    
    else if(GPIO_Pin == GPIO_PIN_0)
    {

        //唤醒任务
        if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
        {
            static BaseType_t xHigherPriorityTaskWoken;
            vTaskNotifyGiveFromISR(INS_task_local_handler, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }

    }
    
}



void imu_cmd_spi_dma(void)
{
    UBaseType_t uxSavedInterruptStatus;
    uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();

    //开启陀螺仪的DMA传输
    if( (gyro_update_flag & (1 << IMU_DR_SHFITS) ) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
    && !(accel_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        gyro_update_flag &= ~(1 << IMU_DR_SHFITS);
        gyro_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);
        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
        return;
    }
    //开启加速度计的DMA传输
    if((accel_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
    && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        accel_update_flag &= ~(1 << IMU_DR_SHFITS);
        accel_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)accel_dma_tx_buf, (uint32_t)accel_dma_rx_buf, SPI_DMA_ACCEL_LENGHT);
        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
        return;
    }
    

    if((accel_temp_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
    && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        accel_temp_update_flag &= ~(1 << IMU_DR_SHFITS);
        accel_temp_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)accel_temp_dma_tx_buf, (uint32_t)accel_temp_dma_rx_buf, SPI_DMA_ACCEL_TEMP_LENGHT);
        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
        return;
    }
    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
}


void DMA2_Stream2_IRQHandler(void)
{

    if(__HAL_DMA_GET_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx)) != RESET)
    {
        __HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx));

        //gyro read over
        //陀螺仪读取完毕
        if(gyro_update_flag & (1 << IMU_SPI_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_SPI_SHFITS);
            gyro_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
            
        }

        //accel read over
        //加速度计读取完毕
        if(accel_update_flag & (1 << IMU_SPI_SHFITS))
        {
            accel_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }
        //temperature read over
        //温度读取完毕
        if(accel_temp_update_flag & (1 << IMU_SPI_SHFITS))
        {
            accel_temp_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_temp_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }
        
        imu_cmd_spi_dma();

        if(gyro_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            gyro_update_flag |= (1 << IMU_NOTIFY_SHFITS);
            __HAL_GPIO_EXTI_GENERATE_SWIT(GPIO_PIN_0);
        }
    }
}

