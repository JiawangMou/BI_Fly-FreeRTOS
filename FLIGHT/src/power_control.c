#include "power_control.h"
#include "motors.h"

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

static bool motorSetEnable = false;
static motorPWM_t motorPWM;
static motorPWM_t motorPWMSet={0, 0, 0, 0};


void powerControlInit(void)
{
	motorsInit();
}

bool powerControlTest(void)
{
	bool pass = true;

	pass &= motorsTest();

	return pass;
}

u16 limitThrust(int value)
{
	if(value > UINT16_MAX)
	{
		value = UINT16_MAX;
	}
	else if(value < 0)
	{
		value = 0;
	}

	return (u16)value;
}

void powerControl(control_t *control)	/*�����������*/
{
	s16 r = control->roll / 2.0f;
	s16 p = control->pitch / 2.0f;
//���Ʒ���	�ģ�
	motorPWM.f1 = limitThrust(control->thrust - r - p + control->yaw);
	motorPWM.f2 = limitThrust(control->thrust - r + p - control->yaw);
	motorPWM.t1 = limitThrust(control->thrust + r + p + control->yaw);
	motorPWM.t2 = limitThrust(control->thrust + r - p - control->yaw);	
	motorPWM.t3 = limitThrust(control->thrust + r - p - control->yaw);	
	motorPWM.t4 = limitThrust(control->thrust + r - p - control->yaw);		

	if (motorSetEnable)
	{
		motorPWM = motorPWMSet;
	}
	motorsSetRatio(PWMF1, motorPWM.f1);	/*���Ƶ������ٷֱ�*/
	motorsSetRatio(PWMF2, motorPWM.f2);
	motorsSetRatio(PWM1,  motorPWM.t1);
	motorsSetRatio(PWM2,  motorPWM.t2);
	motorsSetRatio(PWM3,  motorPWM.t3);
	motorsSetRatio(PWM4,  motorPWM.t4);
//	motorsSetRatio(PWMR, motorPWM.r1);
}

void getMotorPWM(motorPWM_t* get)
{
	*get = motorPWM;
}

void setMotorPWM(bool enable, u32 f1_set, u32 f2_set, u32 t1_set, u32 t2_set, u32 t3_set, u32 t4_set, u32 r1_set)
{
	motorSetEnable = enable;
	motorPWMSet.f1 = f1_set;
	motorPWMSet.f2 = f2_set;
	motorPWMSet.t1 = t1_set;	
	motorPWMSet.t2 = t2_set;
	motorPWMSet.t3 = t3_set;	
	motorPWMSet.t4 = t4_set;
	motorPWMSet.r1 = r1_set;
}
}
