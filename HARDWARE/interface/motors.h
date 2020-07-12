#ifndef __MOTORS_H
#define __MOTORS_H
#include "sys.h"
#include <stdint.h>
#include <stdbool.h>
#include "config.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 电机驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/


/*拍打电机--- 180M主频下,2分频  8位精度输出约为351K PWM */
/*舵机  --- 180M主频下,180分频  3333预装载 输出300HZ PWM */
#define TIM_CLOCK_HZ 				180000000
#define MOTORS_PWM_BITS           	8
#define MOTORS_PWM_PERIOD         	((1<<MOTORS_PWM_BITS) - 1)
#define MOTORS_PWM_PRESCALE       	2

#define SERVOS_PWM_PRESCALE       	180
#define SERVOS_PWM_PERIOD           3333


#define ENABLE_THRUST_BAT_COMPENSATED	/*使能电池油门补偿*/

#define NBR_OF_MOTORS 	7
#define PWMF1  		0
#define PWMF2  		1
#define PWM1  		2
#define PWM2  		3
#define PWM3  		4
#define PWM4  		5
#define PWMR  		6

#define MOTORS_TEST_RATIO         (u16)(0.2*(1<<16))	//20%
#define MOTORS_TEST_ON_TIME_MS    50
#define MOTORS_TEST_DELAY_TIME_MS 150


void motorsInit(void);		/*电机初始化*/
bool motorsTest(void);		/*电机测试*/
void motorsSetRatio(u32 id, u16 ithrust);	/*设置电机占空比*/

#endif /* __MOTORS_H */

