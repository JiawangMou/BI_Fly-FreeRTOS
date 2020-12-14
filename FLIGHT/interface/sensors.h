#ifndef __SENSORS_H
#define __SENSORS_H
#include "stabilizer_types.h"

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

#endif //__SENSORS_H
