#include "movement.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "bmi088_driver.h"
#include "ist8310.h"

#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "dma.h"

#include "pid.h"
#include "Callback.h"
#include "frame.h"
#include "door_servo.h"
#include "can.h"
#include "math.h"
#include "AHRS_MiddleWare.h"
#include "AHRS.h"
#include "CAN_receive.h"

#include "user_lib.h"
#include "arm_math.h"
#include "codemove.h"
#include "movement.h"

extern CAR car;//记录开始的姿态
void car_roll(float angle,float w)
{
	car.begin_yaw+=angle;
	while(fabs((car.yaw-car.begin_yaw)/car.begin_yaw)<0.005)
	{
		
	}
	
}

void car_straight_motion(float v,float rate,double displacement)
{
}









