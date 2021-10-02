#include "frame.h"
#include "pid.h"

#define frame_left_kp 0.30
#define frame_left_ki 0.001
#define frame_left_kd 70
#define frame_left_max_out  0xaff 
#define frame_left_max_iout 0x5ff

#define frame_right_kp 0.30
#define frame_right_ki 0.001
#define frame_right_kd 70
#define frame_right_max_out  0xaff 
#define frame_right_max_iout 0x5ff

pid_type_def frame_pid[2];

void frame_pid_init()
{
  float pid[3]={frame_left_kp,frame_left_ki,frame_left_kd};
	PID_init(&frame_pid[0],PID_POSITION,pid,frame_left_max_out,frame_left_max_iout);
  pid[0]=frame_right_kp;
	pid[1]=frame_right_ki;
	pid[2]=frame_right_kd;
	PID_init(&frame_pid[1],PID_POSITION,pid,frame_right_max_out,frame_right_max_iout);
}







