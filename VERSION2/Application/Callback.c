/****************************(C) COPYRIGHT 2021 AFShk***************************
  ==============================================================================
  * @file       Callback.c/h
  * @brief      
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2021/8/28      AFShk           finish
	����һ��shitɽ����Ȼ��ע���һ��ǲ��뿴���ˣ�����Ҹ������
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

extern pid_type_def speed_pid;//�����ٶȻ�
extern pid_type_def motor_speed_pid[4];//����ٶȻ�(�����)
extern pid_type_def roll_pid;//roll������pid
//��ô���������ٶȻ�
//�������ݿ��ٶȵ��˶�ѧ״̬�����������ֱ��ƽ�ƺ���ת�ļ���������Ի����߼���
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
//���ж����������̬
CAR car;//��¼��ʼ����̬
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

void test_task(void const * argument)//test_task����imu���¶ȿ��ƣ��Լ��ƹ����
{
	__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_1,500);
	door_left();
	left_door_off();
	right_door_off();	
	door_middle();
  front_door_down();
	back_door_down();
	
  while(BMI088_init())	{;}
 	while(ist8310_init())	{;}//��ʼ��������
	frame_pid_init();       //��ʼ�����żܣ�ǰ��Ϊ��ʼ�����ã�
		
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
		//�����Ѿ�������ֵ��¼
		
		car.mag_begin[0]=mag[0];
		car.mag_begin[1]=mag[1];
		car.mag_begin[2]=mag[2];//Record the compass Angle at the beginning
		osDelay(20);//delay 0.5ms
		AHRS_init(quat,car.raccel,car.mag_begin);
	}			
	__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_1,0);//��ǰ�涼�Ǵ�������pid�ĳ�ʼ��
	codemove_init();
	frame_high=200000;		gyro_flag=2;//��flag=2ʱ������̬�Ľ���
	osDelay(20);
	car.begin_yaw=car.yaw;	
	motor_code_using=1;
	osDelay(10);
	
  for(;;)//�˶����Ʋ���,�������ֵĴ����Ѿ�д��,����ͨ��Э���������
  {
		angle=(car.yaw-car.begin_yaw)*2;
		
		cosa=cos(angle);
		sina=sin(angle);//�����˶�����̬
		
		if(order[order_step].order_final==1)
		{
			order_step++;
		}
		
		{
		//����ڳ�������ϵ
		//Ҫ����һ��-1,�����Ǹ���vx,vy,w����������ٶȣ������ٶȻ��Ŀ���,����rpm  
		
		if(code_record==1&&failure_warning!=10&&order_step==0)//�������ñ��������ƵĴ��룬����˼·���Բο����żܵĴ���
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
		else if(code_record==0&&failure_warning!=10)//��������дͨ���ٶȿ��ƵĴ���
		{	
			
		}

		osDelay(2);	//���Ϳ��Ƶ�Ƶ��
	}
}
}
//�ⲿ�жϴ���
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{ 
	if(GPIO_Pin==INT1_ACCEL_Pin)
	{			
		BMI088_read(imu_real_data.gyro,imu_real_data.accel,&imu_real_data.temp);	
		if(accel_flag!=0)//��У׼���ʼ��ȡ����
		{
			//�����������任
			car.raccel[0]=1.0072*(imu_real_data.accel[0]-accel_errodata[0]);
			car.raccel[1]=1.0065*(imu_real_data.accel[1]-accel_errodata[1]);
			car.raccel[2]=1.0073*(imu_real_data.accel[2]-accel_errodata[2])+gravity;//����У׼���
		}		
	}
	//���ٶȶ�ȡ��У׼
	
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
		}//BMI088���¶ȿ���
	
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
			}//������У׼�ɹ��󣬿�ʼ����Ƕ�ƫ��
		}
  }//�����Ǻ��¶ȿ���
	
	if(GPIO_Pin==IST8310_EXIT_Pin)//��Ҫʹ�ô����ƽ��з���ǵ�ȷ��
	{
		ist8310_read_mag(mag);
		car.mag[0]=mag[0];
		car.mag[1]=mag[1];
		car.mag[2]=mag[2];//�Ա������д���
		HAL_UART_Transmit_DMA(&huart6,&buf,1);//ʹ�ܳ�����
	}
	
	if(GPIO_Pin==Failture_Pin)
	{
		failure_warning=10;//��ֵΪ10ʱ�������ϱ���
	}
}
