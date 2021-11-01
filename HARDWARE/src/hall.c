#include "hall.h"
#include "FreeRTOS.h"
#include "task.h"
#include "delay.h"

#define NCS_PIN PCout(4)

static uint8_t send_buffer[] = {0x80 | 0x03, 0x00, 0x00};
static uint8_t angle_read[] = {0x00, 0x00, 0x00};


void hallInit(){

    GPIO_InitTypeDef GPIO_InitStructure;

    //初始化CS引脚
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); //使能时钟

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    vTaskDelay(50);

    NCS_PIN = 1;
    spi2Init();
    vTaskDelay(40);

    NCS_PIN = 0;

    delay_us(50);
    spiExchange(sizeof(angle_read), send_buffer, angle_read);
    delay_us(50);

    NCS_PIN = 1;
}

void readAngle(){

    spiBeginTransaction();

    NCS_PIN = 0;

    delay_us(50);
    spiExchange(sizeof(angle_read), send_buffer, angle_read);
    delay_us(50);

    NCS_PIN = 1;

    spiEndTransaction();
    delay_us(50);
}

void hallTask(void* param)
{
    u32        lastWakeTime = getSysTickCnt();

    hallInit();

    while (1) {
        vTaskDelayUntil(&lastWakeTime, 1); /*1000Hz 10ms周期延时*/
        // if的判断,去掉了isOpFlowOk==1的判断，防止进入死循环；
        readAngle();
    }
}