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

/*拍打电机--- 180M主频下，APB1 Timer clk = 90Mzh,2分频  8位精度输出约为351K PWM */
/*舵机  --- 180M主频下,180分频  3333预装载 输出300HZ PWM */
#define TIM_CLOCK_HZ 90000000
#define MOTORS_PWM_BITS 8
#define MOTORS_PWM_PERIOD ((1 << MOTORS_PWM_BITS) - 1)
#define MOTORS_PWM_PRESCALE 0

#define SERVOS_PWM_PRESCALE 90
#define SERVOS_PWM_PERIOD 3333


//#define ENABLE_THRUST_BAT_COMPENSATED /*使能电池油门补偿*/


#define NBR_OF_MOTORS 7
#define PWMF1 0
#define PWMF2 1
#define PWM_LEFT 2
#define PWM_RIGHT 3
#define PWM_MIDDLE 4
#define PWMR 5

#define MOTORS_TEST_RATIO (u16)(0.2 * (1 << 16)) //20%
#define MOTORS_TEST_ON_TIME_MS 50
#define MOTORS_TEST_DELAY_TIME_MS 150

#define MOTORS_MAXPWM (0.4 * 65535)

#ifdef BI_Fly_1
#define SERVO_MAXPWM 2100 //2.1ms
#define SERVO_MINPWM 900  //0.9ms
#define SERVO_RANGE ((SERVO_MAXPWM - SERVO_MINPWM) / 2)
#endif

#ifdef BI_Fly_2
#define SERVO_RANGE 350
#define SERVO_MAXPWM 2100 //2.1ms
#define SERVO_MINPWM 900  //0.9ms
#endif

void motorsInit(void);                    /*电机初始化*/
bool motorsTest(void);                    /*电机测试*/
void motorsSetRatio(u32 id, u16 ithrust); /*设置电机占空比*/
void servoSetPWM(u8 id, u16 value);
#ifdef BI_Fly_1
u32 servoPWMLimit(u16 value); /*限制实际重装载值在900~2100范围内*/
#endif
#ifdef BI_Fly_2
u32 servoPWMLimit(u8 id, u16 value);
#endif
#endif /* __MOTORS_H */
