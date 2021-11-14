/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
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
#include "frame.h"//用于控制龙门架
#include "Callback.h"
#include "math.h"
#include "door_servo.h"

#define HIGH_1 140000
#define HIGH_2 200000
#define HIGH_3 290000

#ifndef code_using
#define code_using 
#endif

extern CAN_HandleTypeDef hcan1;
extern pid_type_def motor_pid[4];//0,1,2,3 电机采用开环控制，用三角函数去拟合
extern pid_type_def frame_pid[2];//4,5
extern uint8_t accel_flag;

extern uint8_t failure_warning;
extern CAR car;
pid_type_def speed_pid;//整体速度环
pid_type_def motor_speed_pid[4];//电机速度环
pid_type_def roll_pid;
float kflag=0.4;
float v1_control=0;
float v2_control=0;
int motor_code_using=0;
//这里是用来存order的，详情看readme

//typedef struct
//{
//	uint8_t	order_final;
//	float rata;
//	float v;
//	float w;
//	float angle;
//	double displacement;
//}order_t;

order_t order[50]={0,1/2.6,0,0,0,100000};
//uint8_t	order_num
//int16_t	vx
//int16_t	vy
//float		w
//float		angle
//int 		displacement

//=
//{
//	
//};
//move_order[n][0]->指令类型
//move_order[n][1,2,3,4]->指令数据


//N条指令
//指令命名看readme

//extern CAN_HandleTypeDef hcan2;

//FRAME 0left 1right
int frame_delta[2];
int frame_change[2];

float roll_set=0;
int frame_high=0;
uint16_t step_cnt=0;
code motor[4];
uint8_t frame_change_allow=1;

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
  * @brief          hal库CAN回调函数,接收电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
		int8_t i;
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
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
				}
				break;
       }//龙门架位置判断程序,用编码器来判断龙门架的高度
       default:
       {
				 break;
       }	 
    }
		
		if(i==4)//||(i==5))//根据frame_change和frame_high来调节高度,当i=4的时候才控制，降低控制频率
		{
			if(frame_high==0)	
			{
				CAN_cmd_portal_frame(0,0,0);
				if(frame_change[0]<5000)	frame_change[0]=0;
				if(frame_change[1]<5000)	frame_change[1]=0;
			}
			else
			{
				//PID调节到预定位置
				PID_calc(&frame_pid[0],frame_change[0],frame_high);
				PID_calc(&frame_pid[1],frame_change[1],frame_high);
				if(fabs((float)(frame_high-frame_change[0]))<20000)
				{
					v1_control=motor_chassis[4].speed_rpm;
					v2_control=motor_chassis[5].speed_rpm;
				}
				else
				{
					v1_control=0;
					v2_control=0;
				}
				frame_pid[0].out+=(frame_pid[1].fdb-frame_pid[0].fdb)*kflag;
			  CAN_cmd_portal_frame(frame_pid[0].out-v1_control*2,frame_pid[1].out-v2_control*2,0);
			}	
		}//龙门架代码调节
		
		if(failure_warning==10)
		{
			CAN_cmd_chassis(0,0,0,0);
			frame_high=0;
			frame_change_allow=0;
			front_door_down();
			door_middle();
			return;
		}
		
	  #ifdef code_using
		if(motor_code_using==1)
		{
			if((i==1)||(i==2))
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
			else if((i==0)||(i==3))
			{
				if((motor_chassis[i].last_ecd>6000)&&(motor_chassis[i].ecd<3000))
						motor[i].delta=(8192-motor_chassis[i].last_ecd)+(motor_chassis[i].ecd);
				else if((motor_chassis[i].ecd>6000)&&(motor_chassis[i].last_ecd<1000))
						motor[i].delta=-((8192-motor_chassis[i].ecd)+motor_chassis[i].last_ecd);
				else
						motor[i].delta=motor_chassis[i].ecd-motor_chassis[i].last_ecd;
        motor[i].last_value=motor[i].change;	
				motor[i].change-=motor[i].delta;
				if((motor[i].last_value<0x7fffffff)&&(motor[i].last_value>0x70000000)&&(motor[i].change<0xffffff))				
					motor[i].cnt++;
				if((motor[i].last_value<-0x7fffffff)&&(motor[i].last_value>-0x70000000)&&(motor[i].change>-0xffffff))				
					motor[i].cnt--;//用于转换方向
//				motor_chassis[i].speed_rpm=-motor_chassis[i].speed_rpm;//将速度转为正方向的
			}
		}
		#endif
		//移动控制的代码写在user_task里面
}


/**
  * @brief          发送电机控制电流(0x205,0x206)
  * @param[in]      motor5: (0x205) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor6: (0x206) 3508电机控制电流, 范围 [-16384,16384]
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
  * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
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


