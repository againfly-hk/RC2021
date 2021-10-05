#ifndef __DOOR_SERVO_H__
#define __DOOR_SERVO_H__

#include "tim.h"

void left_door_on(void);
void left_door_off(void);

void right_door_on(void);
void right_door_off(void);

void door_left(void);
void door_middle(void);
void door_right(void);
void door_reset(void);

void front_door_lift(void);
void front_door_down(void);
	
#endif 


