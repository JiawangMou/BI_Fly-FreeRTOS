#ifndef __POWER_CONTROL_H
#define __POWER_CONTROL_H
#include "stabilizer_types.h"

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

