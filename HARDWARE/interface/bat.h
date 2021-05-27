#ifndef __BAT_H
#define __BAT_H
#include "stm32f4xx.h"

extern vu16 batteryVoltageRaw;

void batVoltInit();
void ADC1_Init(void);       //ADC1通道初始化
void ADC1DMA_Init(void);    //ADC1DMA初始化

#endif
