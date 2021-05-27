#include "bat.h"

vu16 batteryVoltageRaw;

void batVoltInit(){

    // GPIO_Settings
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    // PA4 灌电流
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;    // PA4 OpenDrain
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_ResetBits(GPIOA ,GPIO_Pin_4);  // 置低PA4

    //PA5 模拟输入
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; //模拟输入引脚
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    ADC1_Init();
}

void ADC1_Init(){
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1, ENABLE);    //ADC1复位
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1, DISABLE);    //复位结束

    // ADC 器件初始化
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    ADC_InitTypeDef ADC_InitStructure;

    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
    ADC_CommonInit(&ADC_CommonInitStructure);                           //通用初始化

    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;                        //模数转换工作在扫描模式
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;                  //模数转换工作在连续转换模式
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None; //外部触发转换关闭
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;              //ADC数据右对齐
    ADC_InitStructure.ADC_NbrOfConversion = 1;                             //顺序进行规则转换的ADC通道的数目
    ADC_Init(ADC1, &ADC_InitStructure);                                 //根据ADC_InitStruct中指定的参数初始化外设ADCx的寄存器

    //设置指定ADC的规则组通道，设置它们的转化顺序和采样时间
    //ADC1,ADC通道x,规则采样顺序值为y,采样时间为239.5周期
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_480Cycles);
    
    ADC_Cmd(ADC1, ENABLE); //使能指定的ADC1
	ADC_DMACmd(ADC1, ENABLE);
    

    // 初始化DMA
    ADC1DMA_Init();

    ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);//源数据变化时开启DMA传输
    ADC_SoftwareStartConv(ADC1);
}


void ADC1DMA_Init(){

    DMA_InitTypeDef DMA_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	
	  DMA_DeInit(DMA2_Stream0);                                                   //将DMA的通道1寄存器重设为缺省值

    DMA_InitStructure.DMA_Channel = DMA_Channel_0;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&ADC1->DR;                  //DMA外设ADC基地址
    DMA_InitStructure.DMA_Memory0BaseAddr = (u32)(&batteryVoltageRaw);          //DMA内存基地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;                     //内存作为数据传输的目的地
    DMA_InitStructure.DMA_BufferSize = 1;                                       //DMA通道的DMA缓存的大小
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;            //外设地址寄存器不变
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                     //内存地址寄存器递增
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; //数据宽度为16位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;         //数据宽度为16位
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                             //工作在循环缓存模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;                         //DMA通道 x拥有高优先级
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream0, &DMA_InitStructure);                                //根据DMA_InitStruct中指定的参数初始化DMA的通道

    // 启动DMA
    DMA_Cmd(DMA2_Stream0, ENABLE);

}

