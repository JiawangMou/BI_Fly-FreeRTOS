#include "gettaskstatus.h"
#include "sys.h"

volatile unsigned long long FreeRTOSRunTimeTicks;

//初始化TIM3使其为FreeRTOS的时间统计提供时基
void ConfigureTimeForRunTimeStats(void)
{
    //定时器3初始化，定时器时钟为90M，分频系数为90-1，所以定时器3的频率
    //为90M/90=1M，自动重装载为50-1，那么定时器周期就是50us
    FreeRTOSRunTimeTicks = 0;
    TIM5_Init(50 - 1, 90 - 1); //初始化TIM5
}

//通用定时器3中断初始化
// arr：自动重装值。
// psc：时钟预分频数
//定时器溢出时间计算方法:Tout=((arr+1)*(psc+1))/Ft us.
// Ft=定时器工作频率,单位:Mhz
//这里使用的是定时器3!(定时器3挂在APB1上，时钟为HCLK/2)
void TIM5_Init(u16 arr, u16 psc)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef        NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

    TIM_TimeBaseStructure.TIM_Period            = arr;                //自动重装载值
    TIM_TimeBaseStructure.TIM_Prescaler         = psc;                //定时器分频
    TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up; //向上计数模式
    TIM_TimeBaseStructure.TIM_ClockDivision     = 0;                  //时钟分频
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;                  //重复计数次数
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);                   //初始化TIM5

    TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel                   = TIM5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_ARRPreloadConfig(TIM5, ENABLE); // TIM5	ARPE使能
    TIM_Cmd(TIM5, ENABLE);              //使能TIM5
}

void TIM5_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
        FreeRTOSRunTimeTicks++;
    }
}
