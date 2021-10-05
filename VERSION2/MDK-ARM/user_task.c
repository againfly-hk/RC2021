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
extern uint8_t rx_echo_buff[2];
extern uint8_t rx_line_buff[5];
extern uint8_t failure_warning;
uint8_t detect_hline_cnt[2]={0,0};
uint8_t	detect_sline_cnt=0;
extern uint8_t door_flag;

extern uint8_t spi_tx_buff[8];
extern uint8_t spi_rx_buff[8];
	
void line_detect_task(void const * argument)
{
	while(1)
	{
		HAL_SPI_Transmit_DMA(&hspi2,spi_tx_buff,3);
		{
			if(rx_line_buff[0]==0xE0)
			{
				detect_hline_cnt[0]++;
				osDelay(50);
			}
			if(rx_line_buff[1]==0x00)
			{
				detect_sline_cnt++;
				osDelay(50);
			}
			if(rx_line_buff[2]==0x00)
			{
				detect_hline_cnt[1]++;
				osDelay(50);
			}
			osDelay(10);
		}
			switch(door_flag)
		{
			case 0:door_middle();left_door_off();right_door_off();break;
			case 1:door_left();left_door_on();right_door_off();break;
			case 2:door_right();left_door_off();right_door_on();break;
			break;
		}//across openmv to get order to control door
		osDelay(10);
	}
}


void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if(hspi==&hspi2)
	{
			HAL_SPI_Receive_DMA(&hspi2,spi_rx_buff,3);
	}
}



