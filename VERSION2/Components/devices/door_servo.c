/*****************************

Name:		door_servo.c
Version:	1.00
Author:		AFShk
Date:		2021/8/20
Category:	Application

*****************************/

#include "door_servo.h"

void left_door_on()
{
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,1000);
	
}

void left_door_off()
{
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,500);
}
//left door control

void right_door_on()
{
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,2070);
}

void right_door_off()
{
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,2570);
}
//right door control

void door_left()
{
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,2100);
}

void door_middle()
{
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,1500);
}

void door_right()
{
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,1100);
}
//tim1_channle4_door_control


