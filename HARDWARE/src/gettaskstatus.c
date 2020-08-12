#include "gettaskstatus.h"
#include "sys.h"

volatile unsigned long long FreeRTOSRunTimeTicks;

//��ʼ��TIM3ʹ��ΪFreeRTOS��ʱ��ͳ���ṩʱ��
void ConfigureTimeForRunTimeStats(void)
{
    //��ʱ��3��ʼ������ʱ��ʱ��Ϊ90M����Ƶϵ��Ϊ90-1�����Զ�ʱ��3��Ƶ��
    //Ϊ90M/90=1M���Զ���װ��Ϊ50-1����ô��ʱ�����ھ���50us
    FreeRTOSRunTimeTicks = 0;
    TIM5_Init(50 - 1, 90 - 1); //��ʼ��TIM5
}

//ͨ�ö�ʱ��3�жϳ�ʼ��
// arr���Զ���װֵ��
// psc��ʱ��Ԥ��Ƶ��
//��ʱ�����ʱ����㷽��:Tout=((arr+1)*(psc+1))/Ft us.
// Ft=��ʱ������Ƶ��,��λ:Mhz
//����ʹ�õ��Ƕ�ʱ��3!(��ʱ��3����APB1�ϣ�ʱ��ΪHCLK/2)
void TIM5_Init(u16 arr, u16 psc)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef        NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

    TIM_TimeBaseStructure.TIM_Period            = arr;                //�Զ���װ��ֵ
    TIM_TimeBaseStructure.TIM_Prescaler         = psc;                //��ʱ����Ƶ
    TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up; //���ϼ���ģʽ
    TIM_TimeBaseStructure.TIM_ClockDivision     = 0;                  //ʱ�ӷ�Ƶ
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;                  //�ظ���������
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);                   //��ʼ��TIM5

    TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel                   = TIM5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_ARRPreloadConfig(TIM5, ENABLE); // TIM5	ARPEʹ��
    TIM_Cmd(TIM5, ENABLE);              //ʹ��TIM5
}

void TIM5_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
        FreeRTOSRunTimeTicks++;
    }
}
