#include "bsp_drawer.h"
#include "pid.h"

#define drawer_kp 0.15
#define drawer_ki 0
#define drawer_kd 0
#define drawer_max 0x3ff
#define drawer_imax 0x3ff

extern pid_type_def motor_pid[7];

void drawer_init()
{
  fp32 PID[3]={drawer_kp,drawer_ki,drawer_kd};
	
	motor_pid[6].mode=PID_POSITION;
	motor_pid[6].Kp=drawer_kp;
	motor_pid[6].Ki=drawer_ki;
	motor_pid[6].Kd=drawer_kd;	
	motor_pid[6].max_out=drawer_max;
	motor_pid[6].max_iout=drawer_imax;
	
	PID_init(&motor_pid[6],motor_pid[6].mode,PID,motor_pid[6].max_out,motor_pid[6].max_iout);
	
}
