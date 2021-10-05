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

uint8_t door_flag=0;
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
extern int move_order[100][5];

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
int pwm_set=0;
static const fp32 imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};

float gyro_erro[3];
float accel_erro[3];

first_order_filter_type_t accel_filter[3];

//���ж����������̬

CAR car;//��¼��ʼ����̬

int test_move=0;
//float yaw,roll,pitch;

void move_order_pid_init()
{
	float pid[3]={0.1,0.1,60};
	PID_init(&speed_pid,PID_POSITION,pid,1000,500);//���ٶȻ�
	
	for(int i=0;i<4;i++)
		PID_init(&motor_speed_pid[i],PID_POSITION,pid,1000,500);//�ȸ����
	
	PID_init(&roll_pid,PID_POSITION,pid,1000,500);//ɵ������
}

void test_task(void const * argument)//test_task����imu���¶ȿ��ƣ��Լ��ƹ����
{

  while(BMI088_init())	{;}
 	while(ist8310_init())	{;}//��ʼ��������
	frame_pid_init();       //��ʼ�����żܣ�ǰ��Ϊ��ʼ�����ã�
	//��ʼ����ʼʹ��һ�����ж�
	HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);//can enable
	osDelay(20);

	left_door_off();
	right_door_off();	
	door_middle();		
	PID_init(&imu_temp_pid, PID_POSITION, imu_temp_PID, TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT);
	osDelay(50);
	car.mag_begin[0]=mag[0];
	car.mag_begin[1]=mag[1];
	car.mag_begin[2]=mag[2];//Record the compass Angle at the beginning
	__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_1,1000);
	uint8_t cnt=0;
	while(temp_flag!=1)	{osDelay(5);}
	__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_1,500);//��ʼ��������Ƴ���
	move_order_pid_init();//�����ʼ���˶���PID
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
			osDelay(50);
		}
		gyro_erro[0]=gyro_erro[0]/100.0f;
		gyro_erro[1]=gyro_erro[1]/100.0f;
		gyro_erro[2]=gyro_erro[2]/100.0f;
		
		accel_erro[0]=accel_erro[0]/100.0f;
		accel_erro[1]=accel_erro[1]/100.0f;
		accel_erro[2]=accel_erro[2]/100.0f;
		
		accel_errodata[0]=accel_erro[0];
		accel_errodata[1]=accel_erro[1];
		accel_errodata[2]=accel_erro[2];//using accel_errodata to save erro_data
//		accel_erro[0]=1.0072*(accel_erro[0])+0.01;
//		accel_erro[1]=1.0065*(accel_erro[1])+0.1147;
//		accel_erro[2]=1.0073*(accel_erro[2])+0.0071;//У׼ÿ����
//		cos_tri[0]=accel_erro[2]/gravity;
//		cos_tri[1]=accel_erro[0]/gravity;
//		cos_tri[2]=accel_erro[1]/gravity;
//		double cos_k=accel_erro[0]*accel_erro[0]*accel_erro[0]+accel_erro[1]*accel_erro[1]*accel_erro[1]+accel_erro[2]*accel_erro[2]*accel_erro[2]-3*accel_erro[0]*accel_erro[1]*accel_erro[2];
//		cos_tri[0]=(accel_erro[2]*accel_erro[2]-accel_erro[1]*accel_erro[0])*gravity/cos_k;
//		cos_tri[1]=(accel_erro[1]*accel_erro[1]-accel_erro[0]*accel_erro[2])*gravity/cos_k;
//		cos_tri[2]=(accel_erro[0]*accel_erro[0]-accel_erro[1]*accel_erro[2])*gravity/cos_k;//����Ƕȹ�ϵ(���ȸ���),������������(ֻ���������Ͱ������λ��ʼ�ղ��������²���ʹ��)
		
#ifdef accel_filter		
		fp32 time=0.00125;
		for(int i=0;i<3;i++)
			first_order_filter_init(&accel_filter[i],0.00125,&time);//ʹ����Ԫ������ʹ�õ�ͨ�˲���
#endif
		
		gyro_flag=1;//gyro_flag change
		accel_flag=1;//accel_flag change
		osDelay(100);//delay 0.5ms
		AHRS_init(quat,car.raccel,car.mag);
		osDelay(50);
		gyro_flag=2;//��flag=2ʱ������̬�Ľ���
		for(int i=0;i<4;i++)	motor[i].change=0;//��ʼ���������
		codemove_init();//��ʼ������������pid:speed,displace
	}//�¶�У׼��ʼ������ٶȼƺ�������ƫ��		
	__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_2,500);//��ǰ�涼�Ǵ�������pid�ĳ�ʼ��
	osDelay(40);
	car.begin_yaw=car.yaw;
  for(;;)
  {
		float angle;
		angle=(car.yaw-car.begin_yaw)*2;
		float cosa=cos(angle);
		float sina=sin(angle);
		
//		car.v1=-1*((-cosa-sina)*car.vx+(-cosa+sina)*car.vy+28*car.w)*23.7946;//cm/s->rpm
//		car.v2=-1*((-cosa+sina)*car.vx+(-cosa-sina)*car.vy+28*car.w)*23.7946;//cm
//		car.v3=-1*((cosa+sina)*car.vx+(cosa-sina)*car.vy+28*car.w)*23.7946;
//		car.v4=-1*((cosa-sina)*car.vx+(cosa+sina)*car.vy+28*car.w)*23.7946;
		car.v1=-1*(1*car.vx+1*car.vy+28*car.w)*23.7946;//cm/s->rpm
		car.v2=-1*(-1*car.vx+-1*car.vy+28*car.w)*23.7946;//cm
		car.v3=-1*(-1*car.vx+1*car.vy+28*car.w)*23.7946;
		car.v4=-1*(1*car.vx+-1*car.vy+28*car.w)*23.7946;
		//����ڳ�������ϵ
		//Ҫ����һ��-1,�����Ǹ���vx,vy,w����������ٶȣ������ٶȻ��Ŀ���,����rpm
		PID_calc(&motor_move_speed_pid[0],motor_chassis[0].speed_rpm,car.v1);
		PID_calc(&motor_move_speed_pid[1],motor_chassis[1].speed_rpm,car.v1);
		PID_calc(&motor_move_speed_pid[2],motor_chassis[2].speed_rpm,car.v1);
		PID_calc(&motor_move_speed_pid[3],motor_chassis[3].speed_rpm,car.v1);
		CAN_cmd_chassis(motor_move_speed_pid[0].out,motor_move_speed_pid[1].out,motor_move_speed_pid[2].out,motor_move_speed_pid[3].out);
		osDelay(2);	
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
//			car.raccel[0]=cos_tri[0]*car.accel[0]+cos_tri[1]*car.accel[1]+cos_tri[2]*car.accel[2];
//			car.raccel[1]=cos_tri[0]*car.accel[1]+cos_tri[1]*car.accel[2]+cos_tri[2]*car.accel[0];
//			car.raccel[2]=cos_tri[0]*car.accel[2]+cos_tri[1]*car.accel[0]+cos_tri[2]*car.accel[1];
			#ifdef accel_imu_using
			//���ͨ�˲�
			for(int i=0;i<3;i++)
			{
				first_order_filter_cali(&accel_filter[i],car.accel[i]);
				car.accel[i]=accel_filter[i].out;
			}
			//��ͨ�˲�������ٶ�����
			car.raccel[0]=car.accel[0];
			car.raccel[1]=car.accel[1];
			car.raccel[2]=car.accel[2];

			//ÿ����ļ��ٶȵ�����ֵ
			if(motor[1].delta==0&&motor[1].odelta==0&&motor[1].oodelta==0)//�û�е�������ٶ�(�����ֹʱ���ٶ�Ư�Ƶ�����)
			{
				car.volocity[0]=0;
				car.volocity[1]=0;
//				car.volocity[2]=0;				
			}
			else
			{
				switch(move_order[step_cnt][0])
				{
					case 0:
					case 4:
					{
						car.volocity[0]=0;
						car.volocity[1]=0;
						car.volocity[2]=0;//��ת�����ŶԵ�===
						break;
					}
					case 1:
					case 2:
					case 3:
					case 5:
					{
						car.volocity[0]+=car.raccel[0]*1.25;//mm/s
						car.volocity[1]+=car.raccel[1]*1.25;
						car.volocity[2]+=car.raccel[2]*1.25;
						break;
					}
				}
			}
			car.displacement[0]+=car.volocity[0]*0.00125+0.00078125*car.raccel[0];
			car.displacement[1]+=car.volocity[1]*0.00125+0.00078125*car.raccel[1];			
			car.displacement[2]+=car.volocity[2]*0.00125+0.00078125*car.raccel[2];			
			//������ٶȵ�λ��
			motor[0].odelta=motor[0].delta;
			motor[0].oodelta=motor[0].odelta;
			#endif
		}		
	}
	if(GPIO_Pin==INT1_GYRO_Pin)
	{
		BMI088_read_gyro(imu_real_data.gyro,&imu_real_data.temp);
		//�¶ȿ���
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
			__HAL_TIM_SetCompare(&htim10,TIM_CHANNEL_1,imu_temp_pid.out);	//����
		}
		//�¶�pid����
		
		if(gyro_flag!=0)//��У׼��������ǻ���
		{
			car.rgyro[0]=imu_real_data.gyro[0]-gyro_erro[0];
			car.rgyro[1]=imu_real_data.gyro[1]-gyro_erro[1];
			car.rgyro[2]=imu_real_data.gyro[2]-gyro_erro[2];
			#ifdef imu_gyro_using
			car.integral_gyro[0]+=0.0025*car.rgyro[0];
			car.integral_gyro[1]+=0.0025*car.rgyro[1];
			car.integral_gyro[2]+=0.0025*car.rgyro[2];
			#endif
			if(gyro_flag==2)
			{
			AHRS_update(quat,0.001f,car.rgyro,car.raccel,car.mag);			
	  	car.yaw=get_yaw(quat);
//			get_angle(quat,&yaw,&pitch,&roll);
			}
		}//������У׼�ɹ��󣬿�ʼ����Ƕ�ƫ��
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
	
//    if(GPIO_Pin == INT1_ACCEL_Pin)
//    {
//        detect_hook(BOARD_ACCEL_TOE);
//        accel_update_flag |= 1 << IMU_DR_SHFITS;
//        accel_temp_update_flag |= 1 << IMU_DR_SHFITS;
//        if(imu_start_dma_flag)
//        {
//            imu_cmd_spi_dma();
//        }
//    }
//    else if(GPIO_Pin == INT1_GYRO_Pin)
//    {
//        detect_hook(BOARD_GYRO_TOE);
//        gyro_update_flag |= 1 << IMU_DR_SHFITS;
//        if(imu_start_dma_flag)
//        {
//            imu_cmd_spi_dma();
//        }
//    }
//    else if(GPIO_Pin == DRDY_IST8310_Pin)
//    {
//        detect_hook(BOARD_MAG_TOE);
//        mag_update_flag |= 1 << IMU_DR_SHFITS;
//    }
//    else if(GPIO_Pin == GPIO_PIN_0)
//    {

//        //wake up the task
//        //��������
//        if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
//        {
//            static BaseType_t xHigherPriorityTaskWoken;
//            vTaskNotifyGiveFromISR(INS_task_local_handler, &xHigherPriorityTaskWoken);
//            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//        }

//    }

//void DMA2_Stream5_IRQHandler(void)
//{
//	__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_3,700);
//	if(__HAL_DMA_GET_FLAG(&hdma_usart1_rx,DMA_FLAG_TCIF1_5)!=RESET)
//	{

//		__HAL_DMA_CLEAR_FLAG(&hdma_usart1_rx,DMA_FLAG_TCIF1_5);
//		HAL_UART_DMAStop(&huart1);
////		if((__HAL_DMA_GET_COUNTER(&hdma_usart1_rx)!=0)||(rx_echo_buff[0]!=0xFF))	{;}
////	  else
////		{
//			rx_echo=(uint16_t)rx_echo_buff[1]<<8|rx_echo_buff[2];
//		}
////	}
//	HAL_UART_Receive_DMA(&huart1,rx_echo_buff,4);
//	HAL_DMA_IRQHandler(&hdma_usart1_rx);
//}

