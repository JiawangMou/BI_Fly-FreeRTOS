#ifndef __POWER_CONTROL_H
#define __POWER_CONTROL_H
#include "stabilizer_types.h"

typedef struct 
{
	u32 f1;
	u32 f2;
	u32 s_left;
	u32 s_rgith;
	u32 s_middle;
	u32 r1;
	
}motorPWM_t;

void powerControlInit(void);
bool powerControlTest(void);
void motorControl(control_t *control);

void getMotorPWM(motorPWM_t* get);
void setMotorPWM(bool enable, u16 f1_set, u16 f2_set, u16 s1_set, u16 s2_set, u16 s3_set,u16 r1_set);
//return 为舵机的位置 单位为us
u16 limitServo(u8 id, float value);
#endif 
