/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             ������CAN�жϽ��պ��������յ������,CAN���ͺ������͵���������Ƶ��.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "CAN_receive.h"
#include "cmsis_os.h"
#include "main.h"
#include "bsp_rng.h"
#include "pid.h"

#include "AHRS_MiddleWare.h"
#include "AHRS.h"
#include "arm_math.h"
#include "bsp_drawer.h"
#include "frame.h"//���ڿ������ż�
#include "Callback.h"

#define HIGH_1 140000
#define HIGH_2 200000
#define HIGH_3 290000

#ifndef code_using
#define code_using 
#endif

extern CAN_HandleTypeDef hcan1;
extern pid_type_def motor_pid[4];//0,1,2,3 ������ÿ������ƣ������Ǻ���ȥ���
extern pid_type_def frame_pid[2];//4,5

extern uint8_t failure_warning;
extern CAR car;
pid_type_def speed_pid;//�����ٶȻ�
pid_type_def motor_speed_pid[4];//����ٶȻ�
pid_type_def roll_pid;
//������������order�ģ����鿴readme
int move_order[100][5]=
{
	1,1000,0,0,0
	
};
//=
//{
//	
//};
//move_order[n][0]->ָ������
//move_order[n][1,2,3,4]->ָ������


//N��ָ��
//ָ��������readme

//extern CAN_HandleTypeDef hcan2;

//FRAME 0left 1right
int frame_delta[2];
int frame_change[2];

float roll_set=0;
int frame_high=0;
int step_cnt=0;
code motor[4];


//motor data read
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }

motor_measure_t motor_chassis[7];

static CAN_TxHeaderTypeDef  portal_frame_tx_message;
static uint8_t              portal_frame_can_send_data[8];
		
static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];

/**
  * @brief          hal CAN fifo call back, receive motor data
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
/**
  * @brief          hal��CAN�ص�����,���յ������
  * @param[in]      hcan:CAN���ָ��
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
		int8_t i;
	
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
	
	  if(failure_warning==10)
		{
			CAN_cmd_chassis(0,0,0,0);
			CAN_cmd_portal_frame(0,0,0);//�ֱ�����������żܵ��
			return;
		}
 
    switch(rx_header.StdId)
    {
			{
        case CAN_3508_M1_ID:
        case CAN_3508_M2_ID:
        case CAN_3508_M3_ID:
        case CAN_3508_M4_ID:
        case CAN_PORTAL_FRAME_LEFT_ID://0x204
        case CAN_PORTAL_FRAME_RIGHT_ID://0x205
				
				i=rx_header.StdId-0x201;
				get_motor_measure(&motor_chassis[i],rx_data);//0,1,2,3 motor 4,5 frame
								
				if((i==4)||(i==5))
				{
					if((motor_chassis[i].last_ecd>6000)&&(motor_chassis[i].ecd<3000))
						frame_delta[i-4]=(8192-motor_chassis[i].last_ecd)+(motor_chassis[i].ecd);
					else if((motor_chassis[i].ecd>6000)&&(motor_chassis[i].last_ecd<1000))
						frame_delta[i-4]=-((8192-motor_chassis[i].ecd)+motor_chassis[i].last_ecd);
					else
						frame_delta[i-4]=motor_chassis[i].ecd-motor_chassis[i].last_ecd;
					frame_change[i-4]+=frame_delta[i-4];			
				}//����λ���жϳ���
				break;
       }
       default:
       {
				 break;
       }	 
    }
		if((i==4)||(i==5))//����frame_change��frame_high�����ڸ߶�,��i=4��ʱ��ſ��ƣ����Ϳ���Ƶ��
		{
			if(frame_high==0)	
			{
				CAN_cmd_portal_frame(0,0,0);
				if(frame_change[i-4]<10000)	frame_change[i-4]=0;
			}
			else
			{
				//PID���ڵ�Ԥ��λ��
				PID_calc(&frame_pid[i-4],frame_change[i-4],frame_high);
				PID_calc(&frame_pid[i-3],frame_change[i-3],frame_high);
			  CAN_cmd_portal_frame(frame_pid[i-4].out,frame_pid[i-3].out,0);
			}
//			//CAN_cmd_chassis(-0x0ff,0x0ff,0x0ff,-0x0ff);//��ǰ
//			if(car.integral_gyro[2]>roll_set+0.1)
//			{
//				CAN_cmd_chassis(0x22f,0x22f,0x22f,0x22f);
//			}
//			else if(car.integral_gyro[2]<roll_set-0.1)
//			{
//				CAN_cmd_chassis(-0x22f,-0x22f,-0x22f,-0x22f);
//			}
//			else
//			{
//				CAN_cmd_chassis(0,0,0,0);
//			}
//				
				
			
			
		}
	  #ifdef code_using
			if((i>=0)&&(i<=3))
			{
				if((motor_chassis[i].last_ecd>6000)&&(motor_chassis[i].ecd<3000))
						motor[i].delta=(8192-motor_chassis[i].last_ecd)+(motor_chassis[i].ecd);
				else if((motor_chassis[i].ecd>6000)&&(motor_chassis[i].last_ecd<1000))
						motor[i].delta=-((8192-motor_chassis[i].ecd)+motor_chassis[i].last_ecd);
				else
						motor[i].delta=motor_chassis[i].ecd-motor_chassis[i].last_ecd;
        motor[i].last_value=motor[i].change;
				motor[i].change+=motor[i].delta;
				if((motor[i].last_value<0x7fffffff)&&(motor[i].last_value>0x70000000)&&(motor[i].change<0xffffff))				
					motor[i].cnt++;
				if((motor[i].last_value<-0x7fffffff)&&(motor[i].last_value>-0x70000000)&&(motor[i].change>-0xffffff))				
					motor[i].cnt--;
			}
		#endif
		if(move_order[step_cnt][0]!=0)
		{
			switch(move_order[step_cnt][0])
			{
				case 1:
				{
					PID_calc(&speed_pid,car.displacement[0],move_order[step_cnt][1]);
//					CAN_cmd_chassis(speed_pid.out,-speed_pid.out,-speed_pid.out,speed_pid.out);
					break;
				}
				case 2:CAN_cmd_chassis(0,0,0,0);break;
				case 3:CAN_cmd_chassis(0,0,0,0);break;
				case 4:CAN_cmd_chassis(0,0,0,0);break;
				case 5:CAN_cmd_chassis(0,0,0,0);break;
				default:
					CAN_cmd_chassis(0,0,0,0);
					break;
			}
		}
		
}


/**
  * @brief          ���͵�����Ƶ���(0x205,0x206)
  * @param[in]      motor5: (0x205) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor6: (0x206) 3508������Ƶ���, ��Χ [-16384,16384]
  * @retval         none
  */


void CAN_cmd_portal_frame(int16_t left, int16_t right,int16_t drawer)
{
    uint32_t send_mail_box;
    portal_frame_tx_message.StdId = 0x1FF;
    portal_frame_tx_message.IDE = CAN_ID_STD;
    portal_frame_tx_message.RTR = CAN_RTR_DATA;
    portal_frame_tx_message.DLC = 0x08;
    portal_frame_can_send_data[0] = (left>> 8);
    portal_frame_can_send_data[1] = left;
    portal_frame_can_send_data[2] = (right >> 8);
    portal_frame_can_send_data[3] = right;
	  portal_frame_can_send_data[4] = (drawer >> 8);
    portal_frame_can_send_data[5] = drawer;
    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &portal_frame_tx_message,portal_frame_can_send_data, &send_mail_box);
}


/**
  * @brief          ���͵�����Ƶ���(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor2: (0x202) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor3: (0x203) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor4: (0x204) 3508������Ƶ���, ��Χ [-16384,16384]
  * @retval         none
  */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

