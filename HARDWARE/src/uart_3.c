#include "uart_3.h"
#include "config.h"
#include "debug_assert.h"
#include "sys.h"
#include "system.h"
#include <string.h>

/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * usart2 串口通信驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

#define USB_DATA_TIMEOUT_MS 1000
#define USB_DATA_TIMEOUT_TICKS (USB_DATA_TIMEOUT_MS / portTICK_RATE_MS)
#define CCR_ENABLE_SET ((u32)0x00000001)

static bool isInit = false;
static bool connectState = false;

static xSemaphoreHandle waitUntilSendDone;
static xSemaphoreHandle uartBusy;
static xQueueHandle usbDataDelivery;

// static u8 dmaBuffer[64];
static u8* outDataIsr;
static u8 dataIndexIsr;
static u8 dataSizeIsr;

void usbInit(void) /*串口初始化*/
{
    waitUntilSendDone = xSemaphoreCreateBinary(); /*等待发送完成 二值信号量*/
    uartBusy = xSemaphoreCreateBinary(); /*串口忙 二值信号量*/
    xSemaphoreGive(uartBusy);

    usbDataDelivery = xQueueCreate(1024, sizeof(u8)); /*队列 1024个消息*/
    ASSERT(usbDataDelivery);

    USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // /* 使能GPIO 和 UART 时钟*/
    RCC_AHB1PeriphClockCmd(USB_GPIO_PERIF, ENABLE);
    ENABLE_USB_RCC(USB_PERIF, ENABLE);

    GPIO_PinAFConfig(USB_GPIO_PORT, USB_GPIO_AF_TX_PIN, USB_GPIO_AF_TX);
    GPIO_PinAFConfig(USB_GPIO_PORT, USB_GPIO_AF_RX_PIN, USB_GPIO_AF_RX);
    /* 配置USART Rx为浮空输入*/
    GPIO_InitStructure.GPIO_Pin = USB_GPIO_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(USB_GPIO_PORT, &GPIO_InitStructure);

    // /* 配置USART Tx 复用功能输出*/
    GPIO_InitStructure.GPIO_Pin = USB_GPIO_TX_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init(USB_GPIO_PORT, &GPIO_InitStructure);

    // /*端口映射*/
    USART_InitStructure.USART_BaudRate = USB_BAUDRATE;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USB_TYPE, &USART_InitStructure);

    // /*配置串口非空中断*/
    NVIC_InitStructure.NVIC_IRQChannel = USB_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // /*串口接收数据寄存器非空中断*/
    USART_ITConfig(USB_TYPE, USART_IT_RXNE, ENABLE);

    USART_Cmd(USB_TYPE, ENABLE); /*使能定时器*/

    isInit = true;
}

bool usbTest(void)
{
    return isInit;
}
/*从接收队列读取数据(带超时处理)*/
bool usbGetDataWithTimout(u8* c)
{
    /*接收usbDataDelivery(1024个容量)消息*/
    if (xQueueReceive(usbDataDelivery, c, USB_DATA_TIMEOUT_TICKS) == pdTRUE) {
        return true;
    }
    *c = 0;
    return false;
}
/*发送原始数据*/
void usbSendData(u8* data, u32 size)
{
    u32 i;

    if (!isInit)
        return;

    for (i = 0; i < size; i++) {
#ifdef USB_SPINLOOP_FLOWCTRL
        while (GPIO_ReadInputDataBit(USB_TXEN_PORT, USB_TXEN_PIN) == Bit_SET)
            ;
#endif
        while (!(USB_TYPE->SR & USART_FLAG_TXE)) {
        };
        USB_TYPE->DR = (data[i] & 0x00FF);
        // USART_SendData(USB_TYPE,data[i]);
    }
}
/*中断方式发送原始数据*/
void usbSendDataIsrBlocking(u32 size, u8* data)
{
    xSemaphoreTake(uartBusy, portMAX_DELAY);
    outDataIsr = data;
    dataSizeIsr = size;
    dataIndexIsr = 1;
    usbSendData(&data[0], 1);
    USART_ITConfig(USB_TYPE, USART_IT_TXE, ENABLE); /*串口发送数据寄存器为空中断*/
    xSemaphoreTake(waitUntilSendDone, portMAX_DELAY);
    outDataIsr = 0;
    xSemaphoreGive(uartBusy);
}
/*发送一个字符到串口*/
int usbPutchar(int ch)
{
    usbSendData((u8*)&ch, 1);

    return (u8)ch;
}

void usbIsr(void)
{
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    //DOWITHTEST
    // static u8 data[100] = { 0 };
    // static u16 i = 0;
    if ((USB_TYPE->SR & (1 << 5)) != 0) /*接收非空中断*/
    {
        // data[i] = (u8)USART_ReceiveData(USB_TYPE);
        // 		i++;
        // connectState = true;
        u8 rxDataInterrupt = (u8)USART_ReceiveData(USB_TYPE);
        xQueueSendFromISR(usbDataDelivery, &rxDataInterrupt, &xHigherPriorityTaskWoken);
        USART_ClearITPendingBit(USB_TYPE, USART_IT_RXNE);

    } else if (USART_GetITStatus(USB_TYPE, USART_IT_TXE) == SET) {
        if (outDataIsr && (dataIndexIsr < dataSizeIsr)) {
            USART_SendData(USB_TYPE, outDataIsr[dataIndexIsr] & 0x00FF);
            dataIndexIsr++;
        } else {
            USART_ITConfig(USB_TYPE, USART_IT_TXE, DISABLE);
            xSemaphoreGiveFromISR(waitUntilSendDone, &xHigherPriorityTaskWoken);
        }
    }
}

void __attribute__((used)) USART3_IRQHandler(void)
{
    usbIsr();
}

bool getusbConnectState(void)
{
    return connectState;
}
