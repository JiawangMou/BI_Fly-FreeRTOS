#ifndef __FLIP_H
#define __FLIP_H 
#include "stabilizer_types.h"

void flyerFlipCheck(setpoint_t* setpoint,control_t* control,state_t* state);	/* Flyer �������*/

void setFlipDir(u8 dir);

#endif 

