#ifndef __SENSORS_H
#define __SENSORS_H
#include "stabilizer_types.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 传感器控制代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/

//#define SENSORS_ENABLE_MAG_AK8963
#define SENSORS_ENABLE_PRESSURE_BMP280	/*气压计使用bmp280*/
#define SENSORS_ENABLE_MAG_AK8963

#define BARO_UPDATE_RATE		RATE_50_HZ
#define SENSOR9_UPDATE_RATE   	RATE_500_HZ
#define SENSOR9_UPDATE_DT     	(1.0f/SENSOR9_UPDATE_RATE)

// typedef struct _acc_buf{
//     Axis3f acc;
//     int count;
// } Accbuffer;
// typedef struct _gyro_buf{
//     Axis3f gyro;
//     int count;
// } Gyrobuffer;
	
void sensorsTask(void *param);
void sensorsInit(void);			/*传感器初始化*/
bool sensorsTest(void);			/*传感器测试*/
bool sensorsAreCalibrated(void);	/*传感器数据校准*/
void sensorsAcquire(sensorData_t *sensors, const u32 tick);/*获取传感器数据*/
void gyroAcquire(Axis3f *gyro);
void getSensorRawData(Axis3i16* acc, Axis3i16* gyro, Axis3i16* mag);
bool getIsMPU9250Present(void);
bool getIsBaroPresent(void);

/* 单独测量传感器数据 */
bool sensorsReadGyro(Axis3f *gyro);
bool sensorsReadAcc(Axis3f *acc);
bool sensorsReadMag(Axis3f *mag);
bool sensorsReadBaro(baro_t *baro);

/*磁力计标定数据获取*/
void setMagCalibData(Axis3i16 offset, Axis3u16 radius);

/*设置当前的acc标定步骤*/
void setprocessAccBias_stepnum( u8 i );

/*复位accbias标定的buffer*/
void reset_accbiasRunning(void);

/*获取accBias的值*/
Axis3f getaccBias(void);
/*获取accScale的值*/
Axis3f getaccScale(void);
void resetaccBias_accScale(void);

float getBatteryVoltage(void);
#endif //__SENSORS_H
