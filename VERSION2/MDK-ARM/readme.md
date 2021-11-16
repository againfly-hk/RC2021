TIM5->Prescaler=8400-1
TIM5->PerIOD=1000-1
LED;

TIM8->Prescaler=168-1
TIM8->Period=20000-1
50HZ 舵机

TIM10->Prescaler=0
TIM->Period=5000-1
imu temperature control

drawer 380000

//底盘控制使用加速度计，线识别，编码器互补滤波
//先尝试一下用编码器的效果，定义宏编译，使其可以在缺少的条件下正常运行，通过两者算另外一个的数据范围是否合理
//800hz加速度计
//400hz陀螺仪
//200hz磁力计

//在中断里面解算姿态
struct CAR{
	float accel[3];
	float gyro[3];//解算出来实际的质态
	float mag[3];
	float integral_accel[3];
	float integral_gyro[3];
	float displacement[3];//相对与初始位置的位移
	float mag_begin[3];//初始位置的mag角度
}car;
//记录开始的姿态


//关于行走的协议：
Order_number	Move_type
num				1(沿x轴行走)	delta_x(沿x轴距离)
				2(沿y轴行走)	delta_y(沿y轴距离)
				3(斜着走)		delta_x,delta_y
				4(转向)			rad
				5(缓慢平移)		
			
x,y轴的平面，用加速度计和电机编码器，超声波




//飞思卡尔方案：
https://blog.csdn.net/qingfengxd1/article/details/106027819

//四元数方案：

https://www.zhihu.com/question/38298130


//问题，解决加速度角度问题

//现更具实际情况选择了两种方案：
一、编码器+灰度传感器+超声波；//code_enable
特殊判断点：
a.第一条线（编码器记录数据清零）,前行；1，3
b.第二条线，编码器数据清零，向左方移动；1，3
c.第三条线：编码器数据清零，使用超声波判断是否为取球区；2
d.z形走位，超声波取球；
e.……



二、惯导加其他互补：




//去看四元数姿态解算

//地空协同用速度配合位置pid使用

//机赛的思路是通过控制速度来控制位移——》到指定的位置，通过其他的辅助定位，编码器主要用于大致调节
//编码器记录转的距离，然后用速度来确定方向的比例
//行走的协议2.0：
使用一个n*6的数组来控制行走
其中数组的前面标号用来代表第几步
i代表第几步
struct order step{
order[i][0]-->运动的类型:0原地停止，1沿x轴运动，2沿y轴运动，3按比例控制（移动），4是旋转，5是否完成
	order[i][1]-->
	//1	vx,null,null,null,
	order[i][2]-->
	order[i][3]-->
	order[i][4]-->
	order[i][5]-->

}


//uint8_t	order_num
//int16_t	vx
//int16_t	vy
//float		w
//float		angle
//int 		displacement


//通信协议
to drawer:
//0b1111 1-e 代表不同的场景：0000-第一个高度	0101--第二个高度	1010--第三个高度	1111--第四个高度		
//0b1001 1-e 保留
//0b0110 1-e 保留

to car:
//ob1111 1-e 代表停止暂停；
//0b1001 1-e 保留
//0b0110 1-e 保留

//第一段 vx/vy=1673/647=2.6;

tim5_ch1代表初始化
tim5_ch2代表运动姿态
tim_ch3代表模式

lowest 120000
middle 200000

highest 280000


现在用的传感器：超声波、灰度巡线、编码器、陀螺仪、加速度计、磁力计

		if(code_record==1&&CAR_TURE&&order_step==0)//这里是用编码器控制的代码，具体思路可以参考龙门架的代码
		{
			
			PID_calc(&motor_move_displace_pid[0],motor[0].change,order[order_step].displacement);		
			car.vx=motor_move_displace_pid[0].out;
			car.vy=car.vx*order[order_step].rata;
			move_pid_calc();
      if(motor[0].change==order[order_step].displacement)	order_step++;
		}
		else if(order_step==1&&CAR_TURE)
		{
			car.vx=0;
			car.vy=0;
			car.w=0.5;
			while(!(rx_line_buff[1]==0xFB&&rx_line_buff[3]==0xDF))
//				(!(((rx_line_buff[0]&0x01)&&(rx_line_buff[2]&0x08))||((rx_line_buff[0]&0x04)&&(rx_line_buff[2]&0x20))||((rx_line_buff[0]&0x10)&&(rx_line_buff[2]&0x80))))
			{
				if(CAR_TURE)
				move_pid_calc();
				osDelay(3);
			}
			car.w=0;
			car.vx=20;
			car.vy=0;
			while(echo_distance>700)
			{
				if(CAR_TURE)
				move_pid_calc();
				osDelay(3);
			}
			car.vx=0;
			car.vy=10;
			while(echo_distance>150)
			{
				if(CAR_TURE)
				move_pid_calc();
				osDelay(3);
			}
			frame_high=120000;
			spi_tx_buff[1]=0xF0;
			car.vx=10;
			car.vy=0;
			while(!(spi_rx_buff[1]==0xF0))
			{
				if(CAR_TURE)
				move_pid_calc();
				osDelay(3);
			}
			car.vx=0;
			while(!(spi_rx_buff[1]==0xFA))
			{
				if(CAR_TURE)
				move_pid_calc();
				osDelay(3);
			}
			car.vx=10;
			while(!(spi_rx_buff[1]==0xF0))
			{
				if(CAR_TURE)
				move_pid_calc();
				osDelay(3);
			}
		}
		else if(code_record==0&&CAR_TURE)//这里面是写通过速度控制的代码
		{	
			
		}