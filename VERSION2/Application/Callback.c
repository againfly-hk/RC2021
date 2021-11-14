/****************************(C) COPYRIGHT 2021 AFShk***************************
  ==============================================================================
  * @file       Callback.c/h
  * @brief      
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2021/8/28      AFShk           finish
	这是一堆shit山，虽然有注释我还是不想看他了，快把我搞恶心了
  ==============================================================================
*****************************(C) COPYRIGHT 2021 AFShk**************************/
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

#define Communication 3
#define gravity				9.79484
#define move_test
bmi088_raw_data_t 	imu_raw_data;
bmi088_real_data_t 	imu_real_data;

uint8_t door_flag=4;
uint8_t temp_flag=0;
fp32 ist8310real[3];

pid_type_def motor_pid[7];	
pid_type_def imu_temp_pid;
uint8_t gyro_flag=0;
uint8_t accel_flag=0;
double cos_tri[3];
float accel_errodata[3];

extern fp32 BMI088_ACCEL_SEN;
extern fp32 BMI088_GYRO_SEN;
extern uint8_t rx_echo_buff[5];
extern uint8_t buf;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern code motor[4];
extern uint8_t rx_line_buff[];

extern uint16_t step_cnt;
extern order_t order[];

extern pid_type_def motor_move_displace_pid[4];
extern pid_type_def motor_move_speed_pid[4];

extern pid_type_def speed_pid;//整体速度环
extern pid_type_def motor_speed_pid[4];//电机速度环(电机环)
extern pid_type_def roll_pid;//roll环控制pid
//怎么解算整体速度环
//具体内容看速度的运动学状态解析（先完成直线平移和旋转的简单命令，并测试互补逻辑）
extern motor_measure_t motor_chassis[7];
uint8_t spi_rxbuff[20];
uint8_t spi_txbuff[20];
uint8_t rx_light[3];
extern uint8_t failure_warning;
float rx_echo;
float mag[3];
float quat[4]={1.0f,0.0f,0.0f,0.0f};
int pwm_set=1300;
static const fp32 imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};
float gyro_erro[3];
float accel_erro[3];
extern int echo_distance;
float angle;
float cosa;
float sina;

int displacement=0;

uint8_t code_record=1;
uint8_t order_step=0;
extern uint8_t spi_tx_buff[8];
extern uint8_t spi_rx_buff[8];
extern int frame_high;
first_order_filter_type_t accel_filter[3];
extern int motor_code_using;
//在中断里面解算姿态
CAR car;//记录开始的姿态
void move_pid_calc(void)
{
		car.v1=-1*(1*car.vx+1*car.vy+28*car.w)*23.7946;//cm/s->rpm
		car.v2=-1*(-1*car.vx+1*car.vy+28*car.w)*23.7946;//cm
		car.v3=-1*(-1*car.vx-1*car.vy+28*car.w)*23.7946;
		car.v4=-1*(1*car.vx+-1*car.vy+28*car.w)*23.7946;
		PID_calc(&motor_move_speed_pid[0],motor_chassis[0].speed_rpm,car.v1);
		PID_calc(&motor_move_speed_pid[1],motor_chassis[1].speed_rpm,car.v2);
		PID_calc(&motor_move_speed_pid[2],motor_chassis[2].speed_rpm,car.v3);
		PID_calc(&motor_move_speed_pid[3],motor_chassis[3].speed_rpm,car.v4);
		CAN_cmd_chassis(motor_move_speed_pid[0].out,motor_move_speed_pid[1].out,motor_move_speed_pid[2].out,motor_move_speed_pid[3].out);			
}

void test_task(void const * argument)//test_task用于imu的温度控制，以及灯光控制
{
	__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_1,500);
	door_left();
	left_door_off();
	right_door_off();	
	door_middle();
  front_door_down();
	back_door_down();
	
  while(BMI088_init())	{;}
 	while(ist8310_init())	{;}//初始化磁力计
	frame_pid_init();       //初始化龙门架（前面为初始化设置）
		
	HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);//can enable
		
	PID_init(&imu_temp_pid, PID_POSITION, imu_temp_PID, TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT);
	uint8_t cnt=0;
	while(temp_flag!=1)	{osDelay(1);}
	__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_1,1000);		
	if(temp_flag==1)
	{
		for(cnt=0;cnt<100;cnt++)
		{
			gyro_erro[0]+=imu_real_data.gyro[0];
			gyro_erro[1]+=imu_real_data.gyro[1];
			gyro_erro[2]+=imu_real_data.gyro[2];
		
			accel_erro[0]+=imu_real_data.accel[0];
			accel_erro[1]+=imu_real_data.accel[1];
			accel_erro[2]+=imu_real_data.accel[2];
			osDelay(20);
		}
		gyro_erro[0]=gyro_erro[0]/100.0f;
		gyro_erro[1]=gyro_erro[1]/100.0f;
		gyro_erro[2]=gyro_erro[2]/100.0f;
		
		accel_erro[0]=accel_erro[0]/100.0f;
		accel_erro[1]=accel_erro[1]/100.0f;
		accel_erro[2]=accel_erro[2]/100.0f;
		
		gyro_flag=1;//gyro_flag change
		accel_flag=1;//accel_flag change
		//代表已经完成误差值记录
		
		car.mag_begin[0]=mag[0];
		car.mag_begin[1]=mag[1];
		car.mag_begin[2]=mag[2];//Record the compass Angle at the beginning
		osDelay(20);//delay 0.5ms
		AHRS_init(quat,car.raccel,car.mag_begin);
	}			
	__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_1,0);//这前面都是传感器和pid的初始化
	codemove_init();
	frame_high=200000;		gyro_flag=2;//当flag=2时进行姿态的解算
	osDelay(20);
	car.begin_yaw=car.yaw;	
	motor_code_using=1;
	osDelay(10);
	
  for(;;)//运动控制部分,其他部分的代码已经写完,包括通信协议和物体检测
  {
		angle=(car.yaw-car.begin_yaw)*2;
		
		cosa=cos(angle);
		sina=sin(angle);//解析运动的姿态
		
		if(order[order_step].order_final==1)
		{
			order_step++;
		}
		
		{
		//相对于车体坐标系
		//要乘以一个-1,这里是根据vx,vy,w反解出来的速度，用于速度环的控制,控制rpm  
		
		if(code_record==1&&failure_warning!=10&&order_step==0)//这里是用编码器控制的代码，具体思路可以参考龙门架的代码
		{
			
			PID_calc(&motor_move_displace_pid[0],motor[0].change,order[order_step].displacement);		
			car.vx=motor_move_displace_pid[0].out;
			car.vy=car.vx*order[order_step].rata;
			move_pid_calc();
      if(motor[0].change==order[order_step].displacement)	order_step++;
		}
		else if(order_step==1&&failure_warning!=10)
		{
			car.vx=0;
			car.vy=0;
			car.w=0.5;
			while(!(rx_line_buff[1]==0xFB&&rx_line_buff[3]==0xDF))
//				(!(((rx_line_buff[0]&0x01)&&(rx_line_buff[2]&0x08))||((rx_line_buff[0]&0x04)&&(rx_line_buff[2]&0x20))||((rx_line_buff[0]&0x10)&&(rx_line_buff[2]&0x80))))
			{
				if(failure_warning!=10)
				move_pid_calc();
				osDelay(3);
			}
			car.w=0;
			car.vx=20;
			car.vy=0;
			while(echo_distance>700)
			{
				if(failure_warning!=10)
				move_pid_calc();
				osDelay(3);
			}
			car.vx=0;
			car.vy=10;
			while(echo_distance>150)
			{
				if(failure_warning!=10)
				move_pid_calc();
				osDelay(3);
			}
			frame_high=120000;
			spi_tx_buff[1]=0xF0;
			car.vx=10;
			car.vy=0;
			while(!(spi_rx_buff[1]==0xF0))
			{
				if(failure_warning!=10)
				move_pid_calc();
				osDelay(3);
			}
			car.vx=0;
			while(!(spi_rx_buff[1]==0xFA))
			{
				if(failure_warning!=10)
				move_pid_calc();
				osDelay(3);
			}
			car.vx=10;
			while(!(spi_rx_buff[1]==0xF0))
			{
				if(failure_warning!=10)
				move_pid_calc();
				osDelay(3);
			}
		}
		else if(code_record==0&&failure_warning!=10)//这里面是写通过速度控制的代码
		{	
			
		}

		osDelay(2);	//降低控制的频率
	}
}
}
//外部中断处理
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{ 
	if(GPIO_Pin==INT1_ACCEL_Pin)
	{			
		BMI088_read(imu_real_data.gyro,imu_real_data.accel,&imu_real_data.temp);	
		if(accel_flag!=0)//在校准完后开始读取数据
		{
			//还差处理坐标轴变换
			car.raccel[0]=1.0072*(imu_real_data.accel[0]-accel_errodata[0]);
			car.raccel[1]=1.0065*(imu_real_data.accel[1]-accel_errodata[1]);
			car.raccel[2]=1.0073*(imu_real_data.accel[2]-accel_errodata[2])+gravity;//修正校准误差
		}		
	}
	//加速度读取和校准
	
	if(GPIO_Pin==INT1_GYRO_Pin)
	{
		{
			BMI088_read_gyro(imu_real_data.gyro,&imu_real_data.temp);
			if((!temp_flag))
			{
				if(imu_real_data.temp>=38.5f)	
				{
					temp_flag=1;
					imu_temp_pid.Iout=MPU6500_TEMP_PWM_MAX/2;
					__HAL_TIM_SetCompare(&htim10,TIM_CHANNEL_1,imu_temp_pid.Iout);
					return;
				}
				__HAL_TIM_SetCompare(&htim10,TIM_CHANNEL_1,MPU6500_TEMP_PWM_MAX);
				return;
			}	
			else if(temp_flag!=0)
			{
				PID_calc(&imu_temp_pid,imu_real_data.temp,40.0f);
				if (imu_temp_pid.out < 0.0f)	imu_temp_pid.out = 0.0f;
				__HAL_TIM_SetCompare(&htim10,TIM_CHANNEL_1,imu_temp_pid.out);	//
			}
		}//BMI088的温度控制
	
		{
			if(gyro_flag!=0)
			{
				car.rgyro[0]=imu_real_data.gyro[0]-gyro_erro[0];
				car.rgyro[1]=imu_real_data.gyro[1]-gyro_erro[1];
				car.rgyro[2]=imu_real_data.gyro[2]-gyro_erro[2];
				if(gyro_flag==2)
				{
				AHRS_update(quat,0.001f,car.rgyro,car.raccel,car.mag);			
				car.yaw=get_yaw(quat);
				}
			}//陀螺仪校准成功后，开始计算角度偏差
		}
  }//陀螺仪和温度控制
	
	if(GPIO_Pin==IST8310_EXIT_Pin)//需要使用磁力计进行方向角的确定
	{
		ist8310_read_mag(mag);
		car.mag[0]=mag[0];
		car.mag[1]=mag[1];
		car.mag[2]=mag[2];//对变量进行处理
		HAL_UART_Transmit_DMA(&huart6,&buf,1);//使能超声波
	}
	
	if(GPIO_Pin==Failture_Pin)
	{
		failure_warning=10;//当值为10时触发故障保护
	}
}
