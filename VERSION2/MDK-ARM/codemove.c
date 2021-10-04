#include "codemove.h"
#include "pid.h"

pid_type_def motor_move_displace_pid[4];
pid_type_def motor_move_speed_pid[4];

void codemove_init()
{
	float codemove_pid[3]={2,0.0001,2};
	PID_init(&motor_move_displace_pid[0],PID_POSITION,codemove_pid,2000,500);
	PID_init(&motor_move_displace_pid[1],PID_POSITION,codemove_pid,2000,500);
	PID_init(&motor_move_displace_pid[2],PID_POSITION,codemove_pid,2000,500);
	PID_init(&motor_move_displace_pid[3],PID_POSITION,codemove_pid,2000,500);
	#ifdef speed_control_using
	float speed_pid[3]={0.02,0.001,20};
	PID_init(&motor_move_speed_pid[0],PID_POSITION,speed_pid,2000,500);
	PID_init(&motor_move_speed_pid[1],PID_POSITION,speed_pid,2000,500);
	PID_init(&motor_move_speed_pid[2],PID_POSITION,speed_pid,2000,500);
	PID_init(&motor_move_speed_pid[3],PID_POSITION,speed_pid,2000,500);
	#endif
}
