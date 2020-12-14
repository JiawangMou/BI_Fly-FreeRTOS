#include "system.h" /*头文件集合*/

TaskHandle_t startTaskHandle;
extern TaskHandle_t vl53l0xTaskHandle;
static void startTask(void *arg);
void ledTask(void *param);

#ifndef ENABLE_GET_TASK_STATUS

#else
    //任务优先级
    #define RUNTIMESTATS_TASK_PRIO	4
    //任务堆栈大小	
    #define RUNTIMESTATS_STK_SIZE 	128  
    //任务句柄
    TaskHandle_t RunTimeStats_Handler;
    //任务函数
    void RunTimeStats_task(void *pvParameters);

    char RunTimeInfo[400];		//保存任务运行时间信息
#endif


int main()
{
    systemInit(); /*底层硬件初始化*/

    xTaskCreate(startTask, "START_TASK", 300, NULL, 2, &startTaskHandle); /*创建起始任务*/

    vTaskStartScheduler(); /*开启任务调度*/

    while (1)
    {
    };
}
/*创建任务*/
void startTask(void *arg)
{
    taskENTER_CRITICAL(); /*进入临界区*/

    xTaskCreate(radiolinkTask, "RADIOLINK", 150, NULL, 5, NULL); /*创建无线连接任务*/

    xTaskCreate(usblinkRxTask, "USBLINK_RX", 150, NULL, 4, NULL); /*创建usb接收任务*/
    xTaskCreate(usblinkTxTask, "USBLINK_TX", 150, NULL, 3, NULL); /*创建usb发送任务*/

    xTaskCreate(atkpTxTask, "ATKP_TX", 150, NULL, 3, NULL);        /*创建atkp发送任务任务*/
    xTaskCreate(atkpRxAnlTask, "ATKP_RX_ANL", 300, NULL, 6, NULL); /*创建atkp解析任务*/

    xTaskCreate(configParamTask, "CONFIG_TASK", 150, NULL, 1, NULL); /*创建参数配置任务*/

    //    xTaskCreate(pmTask, "PWRMGNT", 150, NULL, 2, NULL);				/*创建电源管理任务*/
    xTaskCreate(sensorsTask, "SENSORS", 450, NULL, 4, NULL);          /*创建传感器处理任务*/
    xTaskCreate(vl53l1xTask, "VL53L1X", 300, NULL, 4, vl53l0xTaskHandle);          /*创建激光测距模块任务*/
    xTaskCreate(stabilizerTask, "STABILIZER", 450, NULL, 5, NULL);    /*创建姿态任务*/
	xTaskCreate(opticalFlowTask, "OPTICAL_FLOW", 300, NULL, 4, NULL); /*创建光流模块任务*/
    //    xTaskCreate(expModuleMgtTask, "EXP_MODULE", 150, NULL, 1, NULL);	/*创建扩展模块管理任务*/
    //以下为测试代码
    xTaskCreate(ledTask, "LEDTASK", 150, NULL, 5, NULL);

#ifndef ENABLE_GET_TASK_STATUS

#else
    xTaskCreate((TaskFunction_t )RunTimeStats_task,     
                (const char*    )"RunTimeStats_task",   
                (uint16_t       )RUNTIMESTATS_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )RUNTIMESTATS_TASK_PRIO,
                (TaskHandle_t*  )&RunTimeStats_Handler); 
#endif
    vTaskDelete(startTaskHandle); /*删除开始任务*/
    taskEXIT_CRITICAL(); /*退出临界区*/
}

void vApplicationIdleHook(void)
{
    static u32 tickWatchdogReset = 0;

    portTickType tickCount = getSysTickCnt();

    if (tickCount - tickWatchdogReset > WATCHDOG_RESET_MS)
    {
        tickWatchdogReset = tickCount;
        //     watchdogReset();
    }

       __WFI();	/*进入低功耗模式*/
}

void ledTask(void *param)
{
    u32 lastWakeTime = getSysTickCnt();
    while (1)
    {
        vTaskDelayUntil(&lastWakeTime, 1000); /*1s周期延时*/
        GPIO_ToggleBits(GPIOB, GPIO_Pin_3);
    }
}

#ifndef ENABLE_GET_TASK_STATUS

#else
void RunTimeStats_task(void *pvParameters)
{
    u32 lastWakeTime = getSysTickCnt();
	while(1)
	{
        vTaskDelayUntil(&lastWakeTime, 2000);   /*1s周期延时*/
        
		memset(RunTimeInfo,0,400);				//信息缓冲区清零
		vTaskGetRunTimeStats(RunTimeInfo);		//获取任务运行时间信息
		printf("任务名\t\t\t运行时间\t运行所占百分比\r\n");
		printf("%s\r\n",RunTimeInfo);

	}
}
#endif
