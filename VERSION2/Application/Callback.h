#ifndef CALLBACK_H
#define CALLBACK_H

#include "struct_typedef.h"


#define SPI_DMA_GYRO_LENGHT       8
#define SPI_DMA_ACCEL_LENGHT      9
#define SPI_DMA_ACCEL_TEMP_LENGHT 4


#define IMU_DR_SHFITS        0
#define IMU_SPI_SHFITS       1
#define IMU_UPDATE_SHFITS    2
#define IMU_NOTIFY_SHFITS    3


#define BMI088_GYRO_RX_BUF_DATA_OFFSET  1
#define BMI088_ACCEL_RX_BUF_DATA_OFFSET 2

//ist8310原始数据在缓冲区buf的位置
#define IST8310_RX_BUF_DATA_OFFSET 16


#define TEMPERATURE_PID_KP 4000.0f //温度控制PID的kp
#define TEMPERATURE_PID_KI 0.050f    //温度控制PID的ki
#define TEMPERATURE_PID_KD 0.000f    //温度控制PID的kd

#define TEMPERATURE_PID_MAX_OUT   4000.0f //温度控制PID的max_out
#define TEMPERATURE_PID_MAX_IOUT 1500.0f  //温度控制PID的max_iout

#define MPU6500_TEMP_PWM_MAX 4000 //mpu6500控制温度的设置TIM的重载值，即给PWM最大为 MPU6500_TEMP_PWM_MAX - 1


#define INS_TASK_INIT_TIME 7 //任务开始初期 delay 一段时间

#define INS_YAW_ADDRESS_OFFSET    0
#define INS_PITCH_ADDRESS_OFFSET  1
#define INS_ROLL_ADDRESS_OFFSET   2

#define INS_GYRO_X_ADDRESS_OFFSET 0
#define INS_GYRO_Y_ADDRESS_OFFSET 1
#define INS_GYRO_Z_ADDRESS_OFFSET 2

#define INS_ACCEL_X_ADDRESS_OFFSET 0
#define INS_ACCEL_Y_ADDRESS_OFFSET 1
#define INS_ACCEL_Z_ADDRESS_OFFSET 2

#define INS_MAG_X_ADDRESS_OFFSET 0
#define INS_MAG_Y_ADDRESS_OFFSET 1
#define INS_MAG_Z_ADDRESS_OFFSET 2

typedef struct{
	float accel[3];
	float raccel[3];	
	float integral_accel[3];	
	float volocity[3];
	float gyro[3];//解算出来实际的质态
	float rgyro[3];
	float integral_gyro[3];
	float mag[3];
	float displacement[3];//相对与初始位置的位移
	float mag_begin[3];//初始位置的mag角度
	float yaw;
}CAR;














#endif
