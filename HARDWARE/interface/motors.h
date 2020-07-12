#ifndef __MOTORS_H
#define __MOTORS_H
#include "sys.h"
#include <stdint.h>
#include <stdbool.h>
#include "config.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * �����������	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2017/5/12
 * �汾��V1.3
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/


/*�Ĵ���--- 180M��Ƶ��,2��Ƶ  8λ�������ԼΪ351K PWM */
/*���  --- 180M��Ƶ��,180��Ƶ  3333Ԥװ�� ���300HZ PWM */
#define TIM_CLOCK_HZ 				180000000
#define MOTORS_PWM_BITS           	8
#define MOTORS_PWM_PERIOD         	((1<<MOTORS_PWM_BITS) - 1)
#define MOTORS_PWM_PRESCALE       	2

#define SERVOS_PWM_PRESCALE       	180
#define SERVOS_PWM_PERIOD           3333


#define ENABLE_THRUST_BAT_COMPENSATED	/*ʹ�ܵ�����Ų���*/

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


void motorsInit(void);		/*�����ʼ��*/
bool motorsTest(void);		/*�������*/
void motorsSetRatio(u32 id, u16 ithrust);	/*���õ��ռ�ձ�*/

#endif /* __MOTORS_H */

