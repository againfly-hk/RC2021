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
extern uint8_t rx_echo_buff[2];
extern uint8_t rx_line_buff[5];
extern uint8_t failure_warning;
uint8_t detect_hline_cnt[2]={0,0};
uint8_t	detect_sline_cnt=0;
extern uint8_t door_flag;
	
void line_detect_task(void const * argument)
{
	while(1)
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
		if(rx_line_buff[2])
		{
			detect_hline_cnt[1]++;
			osDelay(50);
		}
		osDelay(10);
	}
		switch(door_flag)
	{
		case 0:door_middle();break;
		case 1:door_left();break;
		case 2:door_right();break;
		break;
	}//across openmv to get order to control door
}





