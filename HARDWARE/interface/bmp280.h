#ifndef __BMP280_H
#define __BMP280_H
#include "stm32f4xx.h"
#include "i2cdev.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * BMP280驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
********************************************************************************/


#define BMP3_I2C_ADDR			(0x76)
#define BMP3_CHIP_ID			(0x50)

/**\name Register Address */
#define BMP3_CHIP_ID_ADDR                       UINT8_C(0x00)
#define BMP3_ERR_REG_ADDR                       UINT8_C(0x02)
#define BMP3_SENS_STATUS_REG_ADDR               UINT8_C(0x03)
#define BMP3_DATA_ADDR                          UINT8_C(0x04)
#define BMP3_EVENT_ADDR                         UINT8_C(0x10)
#define BMP3_INT_STATUS_REG_ADDR                UINT8_C(0x11)
#define BMP3_FIFO_LENGTH_ADDR                   UINT8_C(0x12)
#define BMP3_FIFO_DATA_ADDR                     UINT8_C(0x14)
#define BMP3_FIFO_WM_ADDR                       UINT8_C(0x15)
#define BMP3_FIFO_CONFIG_1_ADDR                 UINT8_C(0x17)
#define BMP3_FIFO_CONFIG_2_ADDR                 UINT8_C(0x18)
#define BMP3_INT_CTRL_ADDR                      UINT8_C(0x19)
#define BMP3_IF_CONF_ADDR                       UINT8_C(0x1A)
#define BMP3_PWR_CTRL_ADDR                      UINT8_C(0x1B)
#define BMP3_OSR_ADDR                           UINT8_C(0X1C)
#define BMP3_CALIB_DATA_ADDR                    UINT8_C(0x31)
#define BMP3_CMD_ADDR                           UINT8_C(0x7E)


/**\name Power mode macros */
#define BMP3_SLEEP_MODE                         UINT8_C(0x00)
#define BMP3_FORCED_MODE                        UINT8_C(0x01)
#define BMP3_NORMAL_MODE                        UINT8_C(0x03)

#define BMP280_TEMPERATURE_CALIB_DIG_T1_LSB_REG             UINT8_C(0x31)
#define BMP280_PRESSURE_TEMPERATURE_CALIB_DATA_LENGTH       (24)
#define BMP280_DATA_FRAME_SIZE			(6)

#define BMP280_OVERSAMP_SKIPPED			(0x00)
#define BMP280_OVERSAMP_1X				(0x01)
#define BMP280_OVERSAMP_2X				(0x02)
#define BMP280_OVERSAMP_4X				(0x03)
#define BMP280_OVERSAMP_8X				(0x04)
#define BMP280_OVERSAMP_16X				(0x05)


bool bmp280Init(I2C_Dev *i2cPort);
void bmp280GetData(float* pressure, float* temperature, float* asl);

u32 bmp280CompensateT(s32 adcT);
u32 bmp280CompensateP(s32 adcP);
void pressureFilter(float* in, float* out);/*限幅平均滤波法*/
float bmp280PressureToAltitude(float* pressure/*, float* groundPressure, float* groundTemp*/);

#endif


