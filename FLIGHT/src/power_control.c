#include "power_control.h"
#include "motors.h"
#include "config_param.h"
#include "math.h"
#include "config.h"
#include "arm_math.h"
#include "sensfusion6.h"

#define rad_50 0.87266462f

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
static motorPWM_t motorPWMSet = {0, 0, 0, 0, 0, 0};

static float invSqrt(float x);	/*���ٿ�ƽ����*/

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
void motorControl(control_t *control) /*�����������*/
{
	// s16 r = control->roll / 2.0f;
	// s16 p = control->pitch / 2.0f;
	s16 r = control->roll;
	s16 p = control->pitch;
	float delta_angle_L = 0;
	float delta_angle_R = 0;
	float normalise;
	float  *rMat_z;
	float GZ[3] = {0,0,1};
	float angle_z = 0;
	static float result;
	float _temp = 0;


	//���Ʒ���	�ģ�
	motorPWM.f2 = limitThrust(control->thrust + r / 2);
	motorPWM.f1 = limitThrust(control->thrust - r / 2);
	delta_angle_L = motorPWM.s_left;
	delta_angle_R = motorPWM.s_middle;
#ifdef BI_Fly_1
	motorPWM.s_left = limitServo(PWM_LEFT, p - control->yaw * 1.5f );
	motorPWM.s_middle = limitServo(PWM_MIDDLE, -p - control->yaw * 1.5f );
#endif
#ifdef BI_Fly_2
	motorPWM.s_left = limitServo(PWM_LEFT, p + control->yaw * 1.5f );
	motorPWM.s_middle = limitServo(PWM_MIDDLE, -p + control->yaw * 1.5f );
#endif
	delta_angle_L = (motorPWM.s_left - delta_angle_L) * 100.0f / SERVO_RANGE * DEG2RAD;
	delta_angle_R = (motorPWM.s_middle - delta_angle_R) * 100.0f / SERVO_RANGE * DEG2RAD;
	if (motorSetEnable)
	{
		motorPWM = motorPWMSet;
	}
	rMat_z = getbodyZ();
	normalise = invSqrt(rMat_z[0]*rMat_z[0] + rMat_z[1]*rMat_z[1] + rMat_z[2]*rMat_z[2]);
	rMat_z[0] *= normalise;
	rMat_z[1] *= normalise;
	rMat_z[2] *= normalise;
	_temp = result;
	arm_dot_prod_f32(rMat_z, GZ, 3, &result);
	_temp = result - _temp;
	// Linearize based on motor test 20210606
	motorsSetRatio(PWMF1, sqrt(motorPWM.f1) * 256 * (1 + (delta_angle_R + _temp)*(delta_angle_R + _temp) )); /*���Ƶ������ٷֱ�*/
	motorsSetRatio(PWMF2, sqrt(motorPWM.f2) * 256 * (1 + (delta_angle_L+ _temp)*(delta_angle_L + _temp) ));
	servoSetPWM(PWM_LEFT, motorPWM.s_left); /*������ռ�ձ�����*/
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

float invSqrt(float x)	/*���ٿ�ƽ����*/
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
