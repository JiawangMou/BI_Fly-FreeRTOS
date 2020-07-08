#include <stdbool.h>
#include "sys.h"
#include "exti.h"
/*FreeRTOS���ͷ�ļ�*/
#include "FreeRTOS.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * �ⲿ�ж���������	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2017/5/12
 * �汾��V1.3
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/


static bool isInit;

/* Interruption initialisation */
void extiInit()
{
//	static NVIC_InitTypeDef NVIC_InitStructure;
//	GPIO_InitTypeDef GPIO_InitStructure;
//	EXTI_InitTypeDef EXTI_InitStructure;

	if (isInit)	return;
	

//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE); 
//	
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
//	
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
//	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
//	NVIC_Init(&NVIC_InitStructure);

//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 12;
//	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
//	NVIC_Init(&NVIC_InitStructure);

//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 12;
//	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
//	NVIC_Init(&NVIC_InitStructure);

//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 12;
//	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
//	NVIC_Init(&NVIC_InitStructure);

	// NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 12;
	// NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
	// NVIC_Init(&NVIC_InitStructure);

//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 12;
//	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
//	NVIC_Init(&NVIC_InitStructure);

////MPU9250���ж� EXTI11  PD11
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 12;
//	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
//	NVIC_Init(&NVIC_InitStructure);

//���Դ���
//	/*ʹ��MPU6500�ж�*/
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
//	GPIO_Init(GPIOB, &GPIO_InitStructure);

//	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource1);

//	EXTI_InitStructure.EXTI_Line = EXTI_Line1;
//	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
//	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//	portDISABLE_INTERRUPTS();
//	EXTI_Init(&EXTI_InitStructure);
//	EXTI_ClearITPendingBit(EXTI_Line1);
//	portENABLE_INTERRUPTS();
	isInit = true;
}

bool extiTest(void)
{
	return isInit;
}

void __attribute__((used)) EXTI0_IRQHandler(void)
{
	EXTI_ClearITPendingBit(EXTI_Line0);
	EXTI0_Callback();
}

void __attribute__((used)) EXTI1_IRQHandler(void)
{
	EXTI_ClearITPendingBit(EXTI_Line1);
	EXTI1_Callback();
}

void __attribute__((used)) EXTI2_IRQHandler(void)
{
	EXTI_ClearITPendingBit(EXTI_Line2);
	EXTI2_Callback();
}

void __attribute__((used)) EXTI3_IRQHandler(void)
{
	EXTI_ClearITPendingBit(EXTI_Line3);
	EXTI3_Callback();
}

void __attribute__((used)) EXTI4_IRQHandler(void)
{
	EXTI_ClearITPendingBit(EXTI_Line4);
	EXTI4_Callback();
}

void __attribute__((used)) EXTI9_5_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line5) == SET) 
	{
		EXTI_ClearITPendingBit(EXTI_Line5);
		EXTI5_Callback();
	}
	if (EXTI_GetITStatus(EXTI_Line6) == SET) 
	{
		EXTI_ClearITPendingBit(EXTI_Line6);
		EXTI6_Callback();
	}
	if (EXTI_GetITStatus(EXTI_Line7) == SET)
	{
		EXTI_ClearITPendingBit(EXTI_Line7);
		EXTI7_Callback();
	}
	if (EXTI_GetITStatus(EXTI_Line8) == SET) 
	{
		EXTI_ClearITPendingBit(EXTI_Line8);
		EXTI8_Callback();
	}
	if (EXTI_GetITStatus(EXTI_Line9) == SET) 
	{
		EXTI_ClearITPendingBit(EXTI_Line9);
		EXTI9_Callback();
	}
}

void __attribute__((used)) EXTI15_10_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line10) == SET) 
	{
		EXTI_ClearITPendingBit(EXTI_Line10);
		EXTI10_Callback();
	}
	if (EXTI_GetITStatus(EXTI_Line11) == SET) 
	{
		EXTI_ClearITPendingBit(EXTI_Line11);
		EXTI11_Callback();
	}
	if (EXTI_GetITStatus(EXTI_Line12) == SET) 
	{
		EXTI_ClearITPendingBit(EXTI_Line12);
		EXTI12_Callback();
	}
	if (EXTI_GetITStatus(EXTI_Line13) == SET) 
	{
		EXTI_ClearITPendingBit(EXTI_Line13);
		EXTI13_Callback();
	}
	if (EXTI_GetITStatus(EXTI_Line14) == SET) 
	{
		EXTI_ClearITPendingBit(EXTI_Line14);
		EXTI14_Callback();
	}
	if (EXTI_GetITStatus(EXTI_Line15) == SET) 
	{
		EXTI_ClearITPendingBit(EXTI_Line15);
		EXTI15_Callback();
	}
}

void __attribute__((weak)) EXTI0_Callback(void) { }
void __attribute__((weak)) EXTI1_Callback(void) { }
void __attribute__((weak)) EXTI2_Callback(void) { }
void __attribute__((weak)) EXTI3_Callback(void) { }
void __attribute__((weak)) EXTI4_Callback(void) { }
void __attribute__((weak)) EXTI5_Callback(void) { }
void __attribute__((weak)) EXTI6_Callback(void) { }
void __attribute__((weak)) EXTI7_Callback(void) { }
void __attribute__((weak)) EXTI8_Callback(void) { }
void __attribute__((weak)) EXTI9_Callback(void) { }
void __attribute__((weak)) EXTI10_Callback(void) { }
void __attribute__((weak)) EXTI11_Callback(void) { }
void __attribute__((weak)) EXTI12_Callback(void) { }
void __attribute__((weak)) EXTI13_Callback(void) { }
void __attribute__((weak)) EXTI14_Callback(void) { }
void __attribute__((weak)) EXTI15_Callback(void) { }
