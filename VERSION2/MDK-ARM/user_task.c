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
#include "door_servo.h"
#include "user_task.h"
#include "Callback.h"

extern uint8_t gyro_flag;
extern uint8_t rx_echo_buff[2];
extern uint8_t rx_line_buff[5];
extern uint8_t failure_warning;
uint8_t detect_hline_cnt[2]={0,0};
uint8_t	detect_sline_cnt=0;
extern uint8_t door_flag;
extern CAR car;
extern uint8_t spi_tx_buff[];
extern uint8_t spi_rx_buff[];
extern uint8_t	spicnt;
extern int pwm_set;
int16_t echo_distance=10000;
int16_t echo_distance_buff[8]={10000,10000,10000,10000,10000,10000,10000,10000};
int16_t* echo_p=NULL;
uint8_t movement_test=0;
int movement_osdelay=1000;

void line_detect_task(void const * argument)
{			
	HAL_SPI_Receive_IT(&hspi2,&spi_rx_buff[spicnt],1);
	while(gyro_flag!=2)	{osDelay(10);}
	echo_p=&echo_distance_buff[0];
	
  while(1)
	{
		echo_distance=rx_echo_buff[0]<<8|rx_echo_buff[1];
		*echo_p=echo_distance;
		if(echo_p==&echo_distance_buff[7])
		{
			echo_p=&echo_distance_buff[0];
		}
		else
		{
			echo_p++;			
		}
		osDelay(2);
//		if(movement_test==0)
//		{
//				car.vx=52*2;
//				car.vy=20*2;
//				osDelay(movement_osdelay);
//			  movement_test++;
//		}
//		if(movement_test==1)
//		{
//				car.vx=0;
//				car.vy=0;
//				osDelay(1000);
//				osDelay(1000);			
//		}
//		if(movement_test==2)
//		{
//				car.vx=-52*2;
//				car.vy=-20*2;
//				osDelay(movement_osdelay);
//			  movement_test--;
//		}

//		car.vx=0;
//		car.vy=20;
//		osDelay(1000);
//		osDelay(1000);
//		car.vx=-20;
//		car.vy=0;
//		osDelay(1000);
//		osDelay(1000);
//		car.vx=	0;
//		car.vy=-20;
//		osDelay(1000);
//		osDelay(1000);//
		
//		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwm_set);
//		{
//			if(rx_line_buff[0]==0xE0)
//			{
//				detect_hline_cnt[0]++;
//				osDelay(50);
//			}
//			if(rx_line_buff[1]==0x00)
//			{
//				detect_sline_cnt++;
//				osDelay(50);
//			}
//			if(rx_line_buff[2]==0x00)
//			{
//				detect_hline_cnt[1]++;
//				osDelay(50);
//			}
//			osDelay(10);
//		}
//			switch(door_flag)
//		{
//			case 0:door_middle();left_door_off();right_door_off();break;
//			case 1:door_left();left_door_on();right_door_off();break;
//			case 2:door_right();left_door_off();right_door_on();break;
//			case 4:__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,2500);break;
//			break;
//		}//across openmv to get order to control door
//		osDelay(100);
	}
}


void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)//
{
	if(hspi==&hspi2)
	{
			HAL_SPI_Transmit(&hspi2,&spi_tx_buff[spicnt],1,10);
			spicnt+=1;
			spicnt%=3;
			HAL_SPI_Receive_IT(&hspi2,&spi_rx_buff[spicnt],1);
	}
}




