#include <stdbool.h>
#include <string.h>
#include "math.h"
#include "config.h"
#include "config_param.h"
#include "watchdog.h"
#include "stmflash.h"
#include "delay.h"
#include "sensors.h"

/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "motors.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 配置参数驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

#define VERSION 0x13 /*13 表示V1.3*/

configParam_t configParam;

#ifdef BI_Fly_1
static configParam_t configParamDefault =
	{
		.version = VERSION, /*软件版本号*/

		.pidAngle = /*角度PID*/
		{
			.roll =
				{
					.kp = 8.0,
					.ki = 0.0,
					.kd = 0.0,
					.outputLimit = 0,
				},
			.pitch =
				{
					.kp = 7.5,
					.ki = 0.0,
					.kd = 0.0,
					.outputLimit = 0,
				},
			.yaw =
				{
					.kp = 0.0,
					.ki = 0.0,
					.kd = 0.0,
					.outputLimit = 0,
				},
		},
		.pidRate = /*角速度PID*/
		{
			.roll =
				{
					.kp = 80.0,
					.ki = 0.0,
					.kd = 6.5,
					.outputLimit = 0,
				},
			.pitch =
				{
					.kp = 98.0,
					.ki = 0.0,
					.kd = 6.5,
					.outputLimit = 0,
				},
			.yaw =
				{
					.kp = 70,
					.ki = 8.0,
					.kd = 0.0,
					.outputLimit = 0,
				},
		},
		.pidPos = /*位置PID*/
		{
			.vx =
				{
					.kp = 4.5,
					.ki = 0.0,
					.kd = 0.0,
				},
			.vy =
				{
					.kp = 4.5,
					.ki = 0.0,
					.kd = 0.0,
				},
			.vz =
				{
					.kp = 80.0,
					.ki = 130.0,
					.kd = 10.0,
				},

			.x =
				{
					.kp = 4.0,
					.ki = 0.0,
					.kd = 0.6,
				},
			.y =
				{
					.kp = 4.0,
					.ki = 0.0,
					.kd = 0.6,
				},
			.z =
				{
					.kp = 4.0,
					.ki = 0.0,
					.kd = 4.5,
				},
		},

		.servo_initpos =
			{
				.s_left = 1520,
				.s_right = 1520,
				.s_middle = 1520,
			},
		.accBias = 
		{
			.accZero = {32, -39, -92},
			.accGain = {2049, 2045, 2068},
			.bias_isfound = true,
		},
		.trimP = 0.f,		 /*pitch微调*/
		.trimR = 0.f,		 /*roll微调*/
		.thrustBase = 34000, /*定高油门基础值*/
};
#endif

#ifdef BI_Fly_2
static configParam_t configParamDefault =
	{
		.version = VERSION, /*软件版本号*/

		.pidAngle = /*角度PID*/
		{
			.roll =
				{
					.kp = 14.0,
					.ki = 0.0,
					.kd = 0.0,
					.outputLimit = 0,
				},
			.pitch =
				{
					.kp = 13.0,
					.ki = 0.0,
					.kd = 0.0,
					.outputLimit = 0,
				},
			.yaw =
				{
					.kp = 0.0,
					.ki = 0.0,
					.kd = 0.0,
					.outputLimit = 0,
				},
		},
		.pidRate = /*角速度PID*/
		{
			.roll =
				{
					.kp = 160.0,
					.ki = 0.0,
					.kd = 6.5,
					.outputLimit = 0,
				},
			.pitch =
				{
					.kp = 69.0,
					.ki = 0.0,
					.kd = 6.5,
					.outputLimit = 0,
				},
			.yaw =
				{
					.kp = 5.0,
					.ki = 0.0,
					.kd = 0.0,
					.outputLimit = 0,
				},
		},
		.pidPos = /*位置PID*/
		{
			.vx =
				{
					.kp = 4.5,
					.ki = 0.0,
					.kd = 0.0,
				},
			.vy =
				{
					.kp = 4.5,
					.ki = 0.0,
					.kd = 0.0,
				},
			.vz =
				{
					.kp = 80.0,
					.ki = 130.0,
					.kd = 10.0,
				},

			.x =
				{
					.kp = 4.0,
					.ki = 0.0,
					.kd = 0.6,
				},
			.y =
				{
					.kp = 4.0,
					.ki = 0.0,
					.kd = 0.6,
				},
			.z =
				{
					.kp = 4.0,
					.ki = 1.0,
					.kd = 1.0,
				},
		},

		.servo_initpos =
		{
			.s_left = 1520,
			.s_right = 1520,
			.s_middle = 1520,
		},
		.accBias = 
		{
			.accZero = {32, -39, -92},
			.accGain = {2049, 2045, 2068},
			.bias_isfound = true,
		},
		.trimP = 0.f,		 /*pitch微调*/
		.trimR = 0.f,		 /*roll微调*/
		.thrustBase = 40000, /*定高油门基础值*/
};
#endif

static u32 lenth = 0;
static bool isInit = false;
static bool isConfigParamOK = false;

static SemaphoreHandle_t xSemaphore = NULL;

static u8 configParamCksum(configParam_t *data)
{
	int i;
	u8 cksum = 0;
	u8 *c = (u8 *)data;
	size_t len = sizeof(configParam_t);

	for (i = 0; i < len; i++)
		cksum += *(c++);
	cksum -= data->cksum;

	return cksum;
}

void configParamInit(void) /*参数配置初始化*/
{
	if (isInit)
		return;

	lenth = sizeof(configParam);
	lenth = lenth / 4 + (lenth % 4 ? 1 : 0);

	STMFLASH_Read(CONFIG_PARAM_ADDR, (u32 *)&configParam, lenth);

	if (configParam.version == VERSION) /*版本正确*/
	{
		if (configParamCksum(&configParam) == configParam.cksum) /*校验正确*/
		{
			//			printf("Version V%1.1f check [OK]\r\n", configParam.version / 10.0f);
			isConfigParamOK = true;
		}
		else
		{
			//			printf("Version check [FAIL]\r\n");
			isConfigParamOK = false;
		}
	}
	else /*版本更新*/
	{
		isConfigParamOK = false;
	}

	if (isConfigParamOK == false) /*配置参数错误，写入默认参数*/
	{
		memcpy((u8 *)&configParam, (u8 *)&configParamDefault, sizeof(configParam));
		configParam.cksum = configParamCksum(&configParam);			   /*计算校验值*/
		STMFLASH_Write(CONFIG_PARAM_ADDR, (u32 *)&configParam, lenth); /*写入stm32 flash*/
		isConfigParamOK = true;
	}

	xSemaphore = xSemaphoreCreateBinary();

	isInit = true;
}

void configParamTask(void *param)
{
	u8 cksum = 0;

	while (1)
	{
		xSemaphoreTake(xSemaphore, portMAX_DELAY);
		cksum = configParamCksum(&configParam); /*数据校验*/

		if (configParam.cksum != cksum)
		{
			configParam.cksum = cksum;				/*数据校验*/
			//watchdogInit(500);					/*擦除时间比较长，看门狗时间设置大一些*/
			STMFLASH_Write(CONFIG_PARAM_ADDR, (u32 *)&configParam, lenth); /*写入stm32 flash*/
			//watchdogInit(WATCHDOG_RESET_MS);		/*重新设置看门狗*/
		}
	}
}

bool configParamTest(void)
{
	return isInit;
}

void configParamGiveSemaphore(void)
{
	xSemaphoreGive(xSemaphore);
}

void resetConfigParamPID(void)
{
	configParam.pidAngle = configParamDefault.pidAngle;
	configParam.pidRate = configParamDefault.pidRate;
	configParam.pidPos = configParamDefault.pidPos;
}

void saveConfigAndNotify(void)
{
	u8 cksum = configParamCksum(&configParam); /*数据校验*/
	if (configParam.cksum != cksum)
	{
		configParam.cksum = cksum;									   /*数据校验*/
		STMFLASH_Write(CONFIG_PARAM_ADDR, (u32 *)&configParam, lenth); /*写入stm32 flash*/
	}
}

void changeServoinitpos_configParam(u16 s1, u16 s2, u16 s3)
{
	configParam.servo_initpos.s_left = s1;
	configParam.servo_initpos.s_right = s2;
	configParam.servo_initpos.s_middle = s3;
}

u16 getservoinitpos_configParam(u8 pwm_id)
{
	u16 value = 0;
	switch (pwm_id)
	{
	case PWM_LEFT:
		value = configParam.servo_initpos.s_left;
		break;
	case PWM_RIGHT:
		value = configParam.servo_initpos.s_right;
		break;
	case PWM_MIDDLE:
		value = configParam.servo_initpos.s_middle;
		break;
	}
	return value;
}

accBias_t getaccbias_configParam( void )
{
	return configParam.accBias;
}

void accbias_writeFlash(void)
{
	Axis3f temp=getaccBias();
	configParam.accBias.accZero[0] = (int16_t)temp.x;
	configParam.accBias.accZero[1] = (int16_t)getaccBias().y;
	configParam.accBias.accZero[2] = (int16_t)getaccBias().z;	
	configParam.accBias.bias_isfound = true;
	configParam.accBias.accGain[0] = (int16_t)getaccScale().x;
	configParam.accBias.accGain[1] = (int16_t)getaccScale().y;	
	configParam.accBias.accGain[2] = (int16_t)getaccScale().z;
}

