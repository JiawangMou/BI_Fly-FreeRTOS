#include <stdint.h>
#include "stdio.h"
#include "config.h"
#include "led.h"
#include "FreeRTOSConfig.h"

/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "task.h"

u32 traceTickCount;

void vApplicationMallocFailedHook( void )
{
	portDISABLE_INTERRUPTS();
	printf("\nMalloc failed!\n");
	ledSet(ERR_LED1, 1);	/*错误检测*/
	ledSet(ERR_LED2, 1);
	while(1);
}

#if (configCHECK_FOR_STACK_OVERFLOW == 1)
void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed portCHAR *pcTaskName)
{
	portDISABLE_INTERRUPTS();
	printf("\nStack overflow!\n");
	ledSet(ERR_LED1, 1);/*错误检测*/
	ledSet(ERR_LED2, 1);
	while(1);
}
#endif

#ifdef UART_OUTPUT_TRACE_DATA
void debugSendTraceInfo(u32 taskNbr)
{
	u32 traceData;
	traceData = (taskNbr << 29) | (((traceTickCount << 16) + TIM1->CNT) & 0x1FFFFFF);
	uartSendDataDma(sizeof(traceData), (u8*)&traceData);
}

void debugInitTrace(void)
{
	/*TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	//Enable the Timer
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	//Timer configuration
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseStructure.TIM_Prescaler = 72;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_TRACE_TIM_PRI;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	DBGMCU_Config(DBGMCU_TIM1_STOP, ENABLE);
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM1, ENABLE);

	traceTickCount = 0;*/
}
#else

void debugSendTraceInfo(u32 taskNbr)
{
}
#endif
