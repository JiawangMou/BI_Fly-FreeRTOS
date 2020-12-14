#include "system.h" /*ͷ�ļ�����*/

TaskHandle_t startTaskHandle;
extern TaskHandle_t vl53l0xTaskHandle;
static void startTask(void *arg);
void ledTask(void *param);

#ifndef ENABLE_GET_TASK_STATUS

#else
    //�������ȼ�
    #define RUNTIMESTATS_TASK_PRIO	4
    //�����ջ��С	
    #define RUNTIMESTATS_STK_SIZE 	128  
    //������
    TaskHandle_t RunTimeStats_Handler;
    //������
    void RunTimeStats_task(void *pvParameters);

    char RunTimeInfo[400];		//������������ʱ����Ϣ
#endif


int main()
{
    systemInit(); /*�ײ�Ӳ����ʼ��*/

    xTaskCreate(startTask, "START_TASK", 300, NULL, 2, &startTaskHandle); /*������ʼ����*/

    vTaskStartScheduler(); /*�����������*/

    while (1)
    {
    };
}
/*��������*/
void startTask(void *arg)
{
    taskENTER_CRITICAL(); /*�����ٽ���*/

    xTaskCreate(radiolinkTask, "RADIOLINK", 150, NULL, 5, NULL); /*����������������*/

    xTaskCreate(usblinkRxTask, "USBLINK_RX", 150, NULL, 4, NULL); /*����usb��������*/
    xTaskCreate(usblinkTxTask, "USBLINK_TX", 150, NULL, 3, NULL); /*����usb��������*/

    xTaskCreate(atkpTxTask, "ATKP_TX", 150, NULL, 3, NULL);        /*����atkp������������*/
    xTaskCreate(atkpRxAnlTask, "ATKP_RX_ANL", 300, NULL, 6, NULL); /*����atkp��������*/

    xTaskCreate(configParamTask, "CONFIG_TASK", 150, NULL, 1, NULL); /*����������������*/

    //    xTaskCreate(pmTask, "PWRMGNT", 150, NULL, 2, NULL);				/*������Դ��������*/
    xTaskCreate(sensorsTask, "SENSORS", 450, NULL, 4, NULL);          /*������������������*/
    xTaskCreate(vl53l1xTask, "VL53L1X", 300, NULL, 4, vl53l0xTaskHandle);          /*����������ģ������*/
    xTaskCreate(stabilizerTask, "STABILIZER", 450, NULL, 5, NULL);    /*������̬����*/
	xTaskCreate(opticalFlowTask, "OPTICAL_FLOW", 300, NULL, 4, NULL); /*��������ģ������*/
    //    xTaskCreate(expModuleMgtTask, "EXP_MODULE", 150, NULL, 1, NULL);	/*������չģ���������*/
    //����Ϊ���Դ���
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
    vTaskDelete(startTaskHandle); /*ɾ����ʼ����*/
    taskEXIT_CRITICAL(); /*�˳��ٽ���*/
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

       __WFI();	/*����͹���ģʽ*/
}

void ledTask(void *param)
{
    u32 lastWakeTime = getSysTickCnt();
    while (1)
    {
        vTaskDelayUntil(&lastWakeTime, 1000); /*1s������ʱ*/
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
        vTaskDelayUntil(&lastWakeTime, 2000);   /*1s������ʱ*/
        
		memset(RunTimeInfo,0,400);				//��Ϣ����������
		vTaskGetRunTimeStats(RunTimeInfo);		//��ȡ��������ʱ����Ϣ
		printf("������\t\t\t����ʱ��\t������ռ�ٷֱ�\r\n");
		printf("%s\r\n",RunTimeInfo);

	}
}
#endif
