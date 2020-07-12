#ifndef __POWER_CONTROL_H
#define __POWER_CONTROL_H
#include "stabilizer_types.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * ����������ƴ���	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2017/5/12
 * �汾��V1.3
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/

typedef struct 
{
	u32 f1;
	u32 f2;
	u32 s1;
	u32 s2;
	u32 s3;
	u32 r1;
	
}motorPWM_t;

void powerControlInit(void);
bool powerControlTest(void);
void powerControl(control_t *control);

void getMotorPWM(motorPWM_t* get);
void setMotorPWM(bool enable, u32 f1_set, u32 f2_set, u32 s1_set, u32 s2_set, u32 s3_set,u32 r1_set);
#endif 
