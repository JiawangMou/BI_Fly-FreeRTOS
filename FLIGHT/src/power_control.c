#include "power_control.h"
#include "motors.h"
#include "config_param.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 功率输出控制代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

static bool motorSetEnable = false;
static motorPWM_t motorPWM;
static motorPWM_t motorPWMSet = {0, 0, 0, 0, 0, 0};

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
	if (value > UINT16_MAX)
	{
		value = UINT16_MAX;
	}
	else if (value < 0)
	{
		value = 0;
	}

	return (u16)value;
}

u16 limitServo(u8 id, float value)
{
	float ratio = 0;
	float PWM_Value = 0;
	if (value > INT16_MAX)
	{
		value = INT16_MAX;
	}
	else if (value < -INT16_MAX)
	{
		value = -INT16_MAX;
	}

	ratio = value / INT16_MAX;
	PWM_Value = ratio * SERVO_RANGE;
	PWM_Value += getservoinitpos_configParam(id);
#ifdef BI_Fly_1
	PWM_Value = servoPWMLimit(PWM_Value);
#endif
#ifdef BI_Fly_2
	PWM_Value = servoPWMLimit(id,PWM_Value);
#endif
	return PWM_Value;
}
void motorControl(control_t *control) /*功率输出控制*/
{
	// s16 r = control->roll / 2.0f;
	// s16 p = control->pitch / 2.0f;
	s16 r = control->roll;
	s16 p = control->pitch;
	//控制分配	改！
	motorPWM.f2 = limitThrust(control->thrust + r / 2);
	motorPWM.f1 = limitThrust(control->thrust - r / 2);
#ifdef BI_Fly_1
	motorPWM.s_left = limitServo(PWM_LEFT, p - control->yaw * 1.5f );
	motorPWM.s_middle = limitServo(PWM_MIDDLE, -p - control->yaw * 1.5f );
#endif
#ifdef BI_Fly_2
	motorPWM.s_left = limitServo(PWM_LEFT, p + control->yaw * 1.5f );
	motorPWM.s_middle = limitServo(PWM_MIDDLE, -p + control->yaw * 1.5f );
#endif

	if (motorSetEnable)
	{
		motorPWM = motorPWMSet;
	}
	motorsSetRatio(PWMF1, motorPWM.f1); /*控制电机输出百分比*/
	motorsSetRatio(PWMF2, motorPWM.f2);
	servoSetPWM(PWM_LEFT, motorPWM.s_left); /*舵机输出占空比设置*/
	servoSetPWM(PWM_MIDDLE, motorPWM.s_middle);
	//	motorsSetRatio(PWMR, motorPWM.r1);
}

void getMotorPWM(motorPWM_t *get)
{
	*get = motorPWM;
}

void setMotorPWM(bool enable, u16 f1_set, u16 f2_set, u16 s1_set, u16 s2_set, u16 s3_set, u16 r1_set)
{
	motorSetEnable = enable;
	motorPWMSet.f1 = f1_set;
	motorPWMSet.f2 = f2_set;
	motorPWMSet.s_left = s1_set;
	motorPWMSet.s_rgith = s2_set;
	motorPWMSet.s_middle = s3_set;
	motorPWMSet.r1 = r1_set;
}
