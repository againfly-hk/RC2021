#include "codemove.h"
#include "pid.h"

#define speed_control_using
pid_type_def motor_move_displace_pid[4];
pid_type_def motor_move_speed_pid[4];
pid_type_def yaw_control_pid;

void codemove_init()
{
	float codemove_pid[3]={2,0.0001,2};
	PID_init(&motor_move_displace_pid[0],PID_POSITION,codemove_pid,2000,500);
	PID_init(&motor_move_displace_pid[1],PID_POSITION,codemove_pid,2000,500);
	PID_init(&motor_move_displace_pid[2],PID_POSITION,codemove_pid,2000,500);
	PID_init(&motor_move_displace_pid[3],PID_POSITION,codemove_pid,2000,500);
	#ifdef speed_control_using
	float speed_pid[3]={3.5,0,0};
	PID_init(&motor_move_speed_pid[0],PID_POSITION,speed_pid,6000,2000);
	PID_init(&motor_move_speed_pid[1],PID_POSITION,speed_pid,6000,2000);
	PID_init(&motor_move_speed_pid[2],PID_POSITION,speed_pid,6000,2000);
	PID_init(&motor_move_speed_pid[3],PID_POSITION,speed_pid,6000,2000);
	#endif
	#ifdef yaw_control
	float yaw_pid[3]={0.02,,0,0};
	#endif
}
