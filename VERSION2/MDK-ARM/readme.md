TIM5->Prescaler=8400-1
TIM5->PerIOD=1000-1
LED;

TIM8->Prescaler=168-1
TIM8->Period=20000-1
50HZ ���

TIM10->Prescaler=0
TIM->Period=5000-1
imu temperature control

drawer 380000

//���̿���ʹ�ü��ٶȼƣ���ʶ�𣬱����������˲�
//�ȳ���һ���ñ�������Ч�����������룬ʹ�������ȱ�ٵ��������������У�ͨ������������һ�������ݷ�Χ�Ƿ����
//800hz���ٶȼ�
//400hz������
//200hz������

//���ж����������̬
struct CAR{
	float accel[3];
	float gyro[3];//�������ʵ�ʵ���̬
	float mag[3];
	float integral_accel[3];
	float integral_gyro[3];
	float displacement[3];//������ʼλ�õ�λ��
	float mag_begin[3];//��ʼλ�õ�mag�Ƕ�
}car;
//��¼��ʼ����̬


//�������ߵ�Э�飺
Order_number	Move_type
num				1(��x������)	delta_x(��x�����)
				2(��y������)	delta_y(��y�����)
				3(б����)		delta_x,delta_y
				4(ת��)			rad
				5(����ƽ��)		
			
x,y���ƽ�棬�ü��ٶȼƺ͵����������������




//��˼����������
https://blog.csdn.net/qingfengxd1/article/details/106027819

//��Ԫ��������

https://www.zhihu.com/question/38298130













