#include "calibrate_task.h"
#include "struct_typedef.h"
#include "remote_control.h"
#include "ins_task.h"
#include "BMI088driver.h"
#include "cmsis_os.h"
#include "ps2_typedef.h"


uint8_t calibrate_flag;
uint16_t start_count;
uint16_t calibration_count;

//陀螺仪零飘数据
fp32 gyro_x_zero;
fp32 gyro_y_zero;
fp32 gyro_z_zero;

fp32 sum_x;
fp32 sum_y;
fp32 sum_z;

void fn_CalibrationInit(void);
void fn_CalibrationStart(void);
void fn_GyroCalibration(void);
extern Ps2_s ps2;

void Calibrate_Task(void const * argument){

    fn_CalibrationInit();

    while(1){

        fn_CalibrationStart();
        fn_GyroCalibration();
        vTaskDelay(1);

    }
}


//初始化函数
void fn_CalibrationInit(void){
    gyro_x_zero = GyroXZero ;
    gyro_y_zero = GyroYZero ;
    gyro_z_zero = GyroZZero ;
    sum_x = 0.0f;
    sum_y = 0.0f;
    sum_z = 0.0f;
    calibrate_flag = 0;
    start_count = 0;
    calibration_count = 0;
}

//右拨为下,遥感拨到"/ \"累计2s开始陀螺仪校准,校准开始有提示音,8s校准完毕有同样提示音
void fn_CalibrationStart(void){
    //判断是否为"/ \"
    if(ps2.button[12] == 1 && ps2.button[13]==1 ){
        //如果没有正在校准
        if(calibrate_flag == 0){
            if(start_count < 2000){
                start_count++;
            }
            //校准开始
            if(start_count > 2000 - 1){
                start_count = 0;
                calibrate_flag = 1;
				sum_x = 0.0f;
                sum_y = 0.0f;
                sum_z = 0.0f;
            }
        }
    } 
}

//陀螺仪均值校准
void fn_GyroCalibration(void){
    if(calibrate_flag == 1){
        if(calibration_count < 8000){
            sum_x += INS_gyro[0];
            sum_y += INS_gyro[1];
            sum_z += INS_gyro[2];
            calibration_count++;
            if(calibration_count == 8000){
                gyro_x_zero += sum_x / 8000;
                gyro_y_zero += sum_y / 8000;
                gyro_z_zero += sum_z / 8000;
            }
        }
        else{
            calibrate_flag = 0;
            calibration_count = 0;
        }
    }
}
