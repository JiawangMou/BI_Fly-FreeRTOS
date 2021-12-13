#include "system.h"
#include "vl53lxx_i2c_hw.h"
#include "i2cdev.h"
#include "vl53lxx.h"
#include "vl53l1_api.h"
#include "math.h"
#include "FreeRTOS.h"
#include "task.h"
#include "arm_math.h"
#include "ina226.h"
/********************************************************************************
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * vl53lxxӦ�ô���, ����vl53l0x��vl53l1x
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2018/5/2
 * �汾��V1.0
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
 ********************************************************************************/

TaskHandle_t vl53l0xTaskHandle = NULL;
TaskHandle_t vl53l1xTaskHandle = NULL;

u16 vl53lxxId = 0;			 /*vl53оƬID*/
bool isEnableVl53lxx = true; /*�Ƿ�ʹ�ܼ���*/

// static bool isInitvl53l0x = false; /*��ʼ��vl53l0x*/
static bool isInitvl53l1x = false; /*��ʼ��vl53l1x*/
// static bool reInitvl53l0x = false; /*�ٴγ�ʼ��vl53l0x*/
// static bool reInitvl53l1x = false; /*�ٴγ�ʼ��vl53l1x*/

static u8 count = 0;
static u8 validCnt = 0;
static u8 inValidCnt = 0;
static bool isVl53l1xOk = false;

static u16 range_last = 0;
static u16 range_compensated = 0;
float quality = 1.0f;

zRange_t vl53lxx;

void vl53l0xTask(void *arg);
void vl53l1xTask(void *arg);

float Q_est_vl = 0.133;
float s_cur_vl, Coe_vl[6], SOC_vl, voltage_vl;
u16 ID_vl, calibrated_vl, configuration_vl;

void vl53lxxInit(void)
{
	// vl53_HWIIC_Init();
	// delay_us(10);

	/*vl53l0x ��ʼ��*/
	// vl53lxxId = vl53l0xGetModelID();
	// if(vl53lxxId == VL53L0X_ID)
	// {
	// 	if (isInitvl53l0x)
	// 	{
	// 		reInitvl53l0x = true;
	// 		vTaskResume(vl53l0xTaskHandle);	/*�ָ�����������*/
	// 	}
	// 	else	/*�״ν���vl53l0x����ģ��*/
	// 	{
	// 		isInitvl53l0x = true;
	// 		xTaskCreate(vl53l0xTask, "VL5310X", 300, NULL, 5, &vl53l0xTaskHandle);	/*����������ģ������*/
	// 	}
	// 	return;
	// }
	// delay_ms(10);

	/*vl53l1x ��ʼ��*/
	VL53L1_RdWord(&dev, 0x010F, &vl53lxxId);
	if (vl53lxxId == VL53L1X_ID)
	{
		isInitvl53l1x = true;
		isVl53l1xOk = true;
		// if (isInitvl53l1x)
		// {
		// 	reInitvl53l1x = true;
		// 	// vTaskResume(vl53l1xTaskHandle);	/*�ָ�����������*/
		// }
		// else	/*�״ν���vl53l1x����ģ��*/
		// {
		// 	isInitvl53l1x = true;
		// 	// xTaskCreate(vl53l1xTask, "VL53L1X", 300, NULL, 5, &vl53l1xTaskHandle);	/*����������ģ������*/
		// }
		// return;
	}

	// vl53lxxId = 0;
}

// void vl53l0xTask(void *arg)
// {
// 	TickType_t xLastWakeTime = xTaskGetTickCount();

// 	vl53l0xSetParam(); /*����vl53l0x ����*/

// 	while (1)
// 	{
// 		if (reInitvl53l0x == true)
// 		{
// 			count = 0;
// 			reInitvl53l0x = false;
// 			vl53l0xSetParam(); /*����vl53l0x ����*/
// 			xLastWakeTime = xTaskGetTickCount();
// 		}
// 		else
// 		{
// 			range_last = vl53l0xReadRangeContinuousMillimeters() * 0.1f; //��λcm

// 			if (range_last < VL53L0X_MAX_RANGE)
// 				validCnt++;
// 			else
// 				inValidCnt++;

// 			if (inValidCnt + validCnt == 10)
// 			{
// 				quality += (validCnt / 10.f - quality) * 0.1f; /*��ͨ*/
// 				validCnt = 0;
// 				inValidCnt = 0;
// 			}

// 			if (range_last >= 6550) /*vl53 ����*/
// 			{
// 				if (++count > 30)
// 				{
// 					count = 0;
// 					isVl53l1xOk = false;
// 					vTaskSuspend(vl53l0xTaskHandle); /*���𼤹�������*/
// 				}
// 			}
// 			else
// 				count = 0;

// 			vTaskDelayUntil(&xLastWakeTime, measurement_timing_budget_ms);
// 		}
// 	}
// }

static VL53L1_RangingMeasurementData_t rangingData;

void vl53l1xTask(void *arg)
{
	int status;
	attitude_t attitude_now; /*���������̬�ı���*/
	u8 isDataReady = 0;
	TickType_t xLastWakeTime = xTaskGetTickCount();
	vl53lxxInit();
	INA226_Init();
	vl53l1xSetParam(); /*����vl53l1x ����*/

	while (1)
	{
		u32 ticki2c = 0;
		u32 lastWakeTimei2c = getSysTickCnt();
		float dt = 0.01;
		// if(reInitvl53l1x == true)
		// {
		// 	count = 0;
		// 	reInitvl53l1x = false;
		// 	vl53l1xSetParam();	/*����vl53l1x ����*/
		// 	xLastWakeTime = xTaskGetTickCount();
		// }else
		// {
		if (RATE_DO_EXECUTE(RATE_100_HZ, ticki2c))
		{
			{
				ID_vl = INA226_Get_ID(INA226_ADDR1);
				calibrated_vl = INA226_GET_CAL_REG(INA226_ADDR1);
				configuration_vl = INA226_Get_CFG_REG(INA226_ADDR1);
				i2cdevWrite(I2C1_DEV, INA226_ADDR1, CAL_REG, 2, &ina226config[2]);
				GetPower();
				s_cur_vl = INA226_data.Shunt_Current;
				voltage_vl = INA226_data.voltageVal;
				if (s_cur_vl == 0)
				{
					SOC_vl = findSOC(voltage_vl);
				}
				SOC_vl = SOC_vl - 0.01f * s_cur_vl * dt / (3600 * Q_est_vl);
			}
			if (RATE_DO_EXECUTE(RATE_20_HZ, ticki2c))
			{
				status = VL53L1_GetMeasurementDataReady(&dev, &isDataReady);
				if (isDataReady)
				{
					status = VL53L1_GetRangingMeasurementData(&dev, &rangingData);
					if (status == 0)
					{
						range_last = rangingData.RangeMilliMeter * 0.1f; /*��λcm*/

						if (range_last < VL53L1X_MAX_RANGE)
						{
							validCnt++;
							getAttitudeData(&attitude_now);
							//					range_compensated = range_last * cosf(attitude_now.pitch*DEG2RAD) * cosf(attitude_now.roll*DEG2RAD);
							range_last = range_last * arm_cos_f32(attitude_now.pitch * DEG2RAD) * arm_cos_f32(attitude_now.roll * DEG2RAD);
						}
						else
							inValidCnt++;

						if (inValidCnt + validCnt == 10)
						{
							quality += (validCnt / 10.f - quality) * 0.1f; /*��ͨ*/
							validCnt = 0;
							inValidCnt = 0;
						}
					}
					status = VL53L1_ClearInterruptAndStartMeasurement(&dev);
				}
			}
		}
		// if(getModuleID() != OPTICAL_FLOW)
		// {
		// 	if(++count > 10)
		// 	{
		// 		count = 0;
		// 		VL53L1_StopMeasurement(&dev);
		// 		vTaskSuspend(vl53l1xTaskHandle);	/*���𼤹�������*/
		// 	}
		// }else count = 0;

		// vTaskDelayUntil(&xLastWakeTime, 50);
		//  }
	}
}
void getLaserData(int16_t *laserRaw, float *laserComp)
{
	*laserRaw = rangingData.RangeMilliMeter;
	*laserComp = range_last;
}

bool vl53lxxReadRange(zRange_t *zrange)
{
	//	if (vl53lxxId == VL53L0X_ID)
	//	{
	//		zrange->quality = quality; //���Ŷ�
	//		vl53lxx.quality = quality;

	//		if (range_last != 0 && range_last < VL53L0X_MAX_RANGE)
	//		{
	//			zrange->distance = (float)range_last; //��λ[cm]
	//			vl53lxx.distance = zrange->distance;
	//			return true;
	//		}
	//	}
	//	else if (vl53lxxId == VL53L1X_ID)
	//	{
	zrange->quality = quality; //���Ŷ�
	vl53lxx.quality = quality;

	if (range_last != 0 && range_last < VL53L1X_MAX_RANGE)
	{
		zrange->distance = (float)range_last; //��λ[cm]
		vl53lxx.distance = zrange->distance;
		return true;
	}
	//	}

	return false;
}
/*ʹ�ܼ���*/
void setVl53lxxState(u8 enable)
{
	isEnableVl53lxx = enable;
}

bool getVl53l1xstate(void)
{
	return isVl53l1xOk;
}

u16 getVl53l1xxrangecompensated(void)
{
	return range_compensated;
}

void getvoltage(float *v)
{
	*v = voltage_vl;
}
void getcurrent(float *c)
{
	*c = s_cur_vl;
}