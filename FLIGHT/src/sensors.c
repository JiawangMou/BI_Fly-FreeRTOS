#include <math.h>
#include "stdio.h"
#include "delay.h"
#include "config.h"
#include "config_param.h"
#include "ledseq.h"
#include "mpu6500.h"
#include "sensors.h"
#include "ak8963.h"
#include "bmp3_defs.h"
#include "bmp3.h"
#include "filter.h"
#include "axis.h"
#include "spl06.h"
#include "atkp.h"

/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "task.h"

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

#define SENSORS_GYRO_FS_CFG MPU6500_GYRO_FS_2000
#define SENSORS_DEG_PER_LSB_CFG MPU6500_DEG_PER_LSB_2000

#define SENSORS_ACCEL_FS_CFG MPU6500_ACCEL_FS_16
#define SENSORS_G_PER_LSB_CFG MPU6500_G_PER_LSB_16

#define SENSORS_NBR_OF_BIAS_SAMPLES 1024 /* 计算方差的采样样本个数 */
#define GYRO_VARIANCE_BASE 300			 /* 陀螺仪零偏方差阈值 */
#define ACC_VARIANCE_BASE  300			 /* 陀螺仪零偏方差阈值 */
#define SENSORS_ACC_SCALE_SAMPLES 200	 /* 加速计采样个数 */

// MPU9250主机模式读取数据 缓冲区长度
#define SENSORS_MPU6500_BUFF_LEN 14
#define SENSORS_MAG_BUFF_LEN 8
#define SENSORS_BARO_STATUS_LEN 1
#define SENSORS_BARO_DATA_LEN 6
#define SENSORS_BARO_BUFF_LEN (SENSORS_BARO_STATUS_LEN + SENSORS_BARO_DATA_LEN)

#define ACC_ID 0
#define GYRO_ID 1
typedef struct
{
	Axis3f bias;
	bool isBiasValueFound;
	bool isBufferFilled;
	Axis3i16 *bufHead;
	Axis3i16 buffer[SENSORS_NBR_OF_BIAS_SAMPLES];
} BiasObj;

BiasObj accBiasRunning[6];
static Axis3f accBias;
BiasObj gyroBiasRunning;
static Axis3f gyroBias;

static bool gyroBiasFound = false;
static bool accBiasFound = false;

/*允许进行accbias计算标志位*/ 
static bool isreadytoprocessAccBias = false;   
//static float accScaleSum = 0;
static Axis3f accScale;

static bool isInit = false;
static sensorData_t sensors;


static Axis3i16 gyroRaw;
static Axis3i16 accRaw;
static Axis3i16 magRaw;

// static Axis3f gyroBff;

/*低通滤波参数*/
#define GYRO_LPF_CUTOFF_FREQ 30
#define ACCEL_LPF_CUTOFF_FREQ 10
#define BARO_LPF_CUTOFF_FREQ 20
static lpf2pData accLpf[3];
static lpf2pData gyroLpf[3];
static lpf2pData BaroLpf;

static bool isMPUPresent = false;
static bool isMagPresent = false;
static bool isBaroPresent = false;

static u8 processAccBias_stepnum = 0;	//为0时表示没有进行accBias计算，进入计算时，stepnum 表示当前的步骤号 0~5；

enum
{
	IDLE,
	BMP388,
	SPL06
} baroType = IDLE;

static uint8_t buffer[SENSORS_MPU6500_BUFF_LEN + SENSORS_MAG_BUFF_LEN + SENSORS_BARO_BUFF_LEN] = {0};

static xQueueHandle accelerometerDataQueue;
static xQueueHandle gyroDataQueue;
static xQueueHandle magnetometerDataQueue;
static xQueueHandle barometerDataQueue;
static xSemaphoreHandle sensorsDataReady;

static void applyAxis3fLpf(lpf2pData *data, Axis3f *in);
// static void sensorsBiasObjInit(BiasObj *bias);
static void sensorsCalculateVarianceAndMean(BiasObj *bias, Axis3f *varOut, Axis3f *meanOut);
static bool sensorsFindBiasValue(BiasObj *bias, u8 sensor_id);
static void sensorsAddBiasValue(BiasObj *bias, int16_t x, int16_t y, int16_t z);

/*从队列读取陀螺数据*/
bool sensorsReadGyro(Axis3f *gyro)
{
	Axis3f gyrobff;
	gyro->x = 0.0;
	gyro->y = 0.0;
	gyro->z = 0.0;
	int i = 0;
	while (xQueueReceive(gyroDataQueue, &gyrobff, 0))
	{
		gyro->x += gyrobff.x;
		gyro->y += gyrobff.y;
		gyro->z += gyrobff.z;
		i++;
	}
	if (i == 0)
	{
		return false;
	}

	gyro->x = gyro->x / i;
	gyro->y = gyro->y / i;
	gyro->z = gyro->z / i;
	return true;
}
/*从队列读取加速计数据*/
bool sensorsReadAcc(Axis3f *acc)
{
	Axis3f accbff;
	acc->x = 0.0;
	acc->y = 0.0;
	acc->z = 0.0;

	int i = 0;
	while (xQueueReceive(accelerometerDataQueue, &accbff, 0))
	{
		acc->x += accbff.x;
		acc->y += accbff.y;
		acc->z += accbff.z;
		i++;
	}
	if (i == 0)
	{
		return false;
	}

	acc->x = acc->x / i;
	acc->y = acc->y / i;
	acc->z = acc->z / i;
	return true;
}
/*从队列读取磁力计数据*/
bool sensorsReadMag(Axis3f *mag)
{
	return (pdTRUE == xQueueReceive(magnetometerDataQueue, mag, 0));
}
/*从队列读取气压数据*/
bool sensorsReadBaro(baro_t *baro)
{
	return (pdTRUE == xQueueReceive(barometerDataQueue, baro, 0));
}
/*传感器中断初始化*/
static void sensorsInterruptInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	/*使能MPU6500中断*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource11);

	EXTI_InitStructure.EXTI_Line = EXTI_Line11;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	portDISABLE_INTERRUPTS();
	EXTI_Init(&EXTI_InitStructure);
	EXTI_ClearITPendingBit(EXTI_Line11);
	portENABLE_INTERRUPTS();
}

/* 传感器器件初始化 */
void sensorsDeviceInit(void)
{
	i2cdevInit(I2C3_DEV);
	mpu6500Init(I2C3_DEV);

	vTaskDelay(10);
	mpu6500Reset(); // 复位MPU6500
	vTaskDelay(50); // 延时等待寄存器复位

	u8 temp = mpu6500GetDeviceID();
	if (temp == 0x38 || temp == 0x39)
	{
		isMPUPresent = true;
		//printf("MPU9250 I2C connection [OK].\n");
	}
	else
	{
		//printf("MPU9250 I2C connection [FAIL].\n");
	}

	mpu6500SetSleepEnabled(false); // 唤醒MPU6500
	vTaskDelay(10);
	mpu6500SetClockSource(MPU6500_CLOCK_PLL_XGYRO);		 // 设置X轴陀螺作为时钟
	vTaskDelay(10);										 // 延时等待时钟稳定
	mpu6500SetTempSensorEnabled(false);					 // 使能温度传感器
	mpu6500SetI2CBypassEnabled(true);					 // 旁路模式，磁力计和气压连接到主IIC
	mpu6500SetFullScaleAccelRange(SENSORS_ACCEL_FS_CFG); // 设置加速计量程
	mpu6500SetFullScaleGyroRange(SENSORS_GYRO_FS_CFG);	 // 设置陀螺量程
	mpu6500SetAccelDLPF(MPU9250_ACCEL_DLPF_BW_21);		 // 设置加速计数字低通滤波,同时ACCEL_FCHOICE_B没有设置，默认为0，与A_DLPF_CFG共同决定输出频率，截止频率为21.2Hz，输出频率为1KHz

	mpu6500SetRate(0);						// 设置采样速率: 1000 / (1 + 0) = 1000Hz
	mpu6500SetDLPFMode(MPU6500_DLPF_BW_41); // 设置陀螺数字低通滤波

	for (u8 i = 0; i < 3; i++) // 初始化加速计和陀螺二阶低通滤波
	{
		lpf2pInit(&gyroLpf[i], 1000, GYRO_LPF_CUTOFF_FREQ);
		lpf2pInit(&accLpf[i], 1000, ACCEL_LPF_CUTOFF_FREQ);
	}
	lpf2pInit(&BaroLpf, 1000, BARO_LPF_CUTOFF_FREQ);

#ifdef SENSORS_ENABLE_MAG_AK8963
	ak8963Init(I2C3_DEV); //ak8963磁力计初始化
	if (ak8963TestConnection() == true)
	{
		isMagPresent = true;
		ak8963SetMode(AK8963_MODE_16BIT | AK8963_MODE_CONT2); // 16bit 100Hz
		//printf("AK8963 I2C connection [OK].\n");
	}
	else
	{
		//printf("AK8963 I2C connection [FAIL].\n");
	}
#endif

	if (bmp3_init(BMP388_DEV) == true) //BMP388初始化
	{
		isBaroPresent = true;
		baroType = BMP388;

		bmp3_set_sensor_settings(BMP3_ALL_SETTINGS, BMP388_DEV);
		bmp3_set_op_mode(BMP388_DEV);
		vTaskDelay(100);
	}
	else if(SPL06Init(I2C3_DEV) == true)
	{
		isBaroPresent = true;
		baroType = SPL06;
		vTaskDelay(100);
	}
	else
	{
		isBaroPresent = false;
	}

	/*创建传感器数据队列*/
	accelerometerDataQueue = xQueueCreate(10, sizeof(Axis3f));
	gyroDataQueue = xQueueCreate(10, sizeof(Axis3f));
	magnetometerDataQueue = xQueueCreate(1, sizeof(Axis3f));
	barometerDataQueue = xQueueCreate(1, sizeof(baro_t));
}
/*传感器偏置初始化*/
static void gyroBiasObjInit(BiasObj *bias)
{
	bias->isBufferFilled = false;
	bias->bufHead = bias->buffer;
}

static void accBiasObjInit(BiasObj *bias)
{
	accBias_t accBias_init;
	accBias_init = getaccbias_configParam();
	accBiasFound = accBias_init.bias_isfound;
	if(accBiasFound)
	{
		accBias.x = accBias_init.accZero[0];
		accBias.y = accBias_init.accZero[1]; 
		accBias.z = accBias_init.accZero[2];

		accScale.x = accBias_init.accGain[0];
		accScale.y = accBias_init.accGain[1];
		accScale.z = accBias_init.accGain[2];
	}
	else
	{
		accBias.x = 0;
		accBias.y = 0; 
		accBias.z = 0;

		accScale.x = 1;
		accScale.y = 1;
		accScale.z = 1;		
	}
	

	bias->isBufferFilled = false;
	bias->bufHead = bias->buffer;
}
/*传感器测试*/
bool sensorsTest(void)
{
	bool testStatus = true;

	if (!isInit)
	{
		printf("Uninitialized\n");
		testStatus = false;
	}

	testStatus &= isBaroPresent;

	return testStatus;
}

/*计算方差和平均值*/
static void sensorsCalculateVarianceAndMean(BiasObj *bias, Axis3f *varOut, Axis3f *meanOut)
{
	u32 i;
	int64_t sum[3] = {0};
	int64_t sumsq[3] = {0};

	for (i = 0; i < SENSORS_NBR_OF_BIAS_SAMPLES; i++)
	{
		sum[0] += bias->buffer[i].x;
		sum[1] += bias->buffer[i].y;
		sum[2] += bias->buffer[i].z;
		sumsq[0] += bias->buffer[i].x * bias->buffer[i].x;
		sumsq[1] += bias->buffer[i].y * bias->buffer[i].y;
		sumsq[2] += bias->buffer[i].z * bias->buffer[i].z;
	}

	varOut->x = (sumsq[0] - ((int64_t)sum[0] * sum[0]) / SENSORS_NBR_OF_BIAS_SAMPLES) / SENSORS_NBR_OF_BIAS_SAMPLES;
	varOut->y = (sumsq[1] - ((int64_t)sum[1] * sum[1]) / SENSORS_NBR_OF_BIAS_SAMPLES) / SENSORS_NBR_OF_BIAS_SAMPLES;
	varOut->z = (sumsq[2] - ((int64_t)sum[2] * sum[2]) / SENSORS_NBR_OF_BIAS_SAMPLES) / SENSORS_NBR_OF_BIAS_SAMPLES;

	meanOut->x = (float)sum[0] / SENSORS_NBR_OF_BIAS_SAMPLES;
	meanOut->y = (float)sum[1] / SENSORS_NBR_OF_BIAS_SAMPLES;
	meanOut->z = (float)sum[2] / SENSORS_NBR_OF_BIAS_SAMPLES;
}
/*传感器查找偏置值*/
static bool sensorsFindBiasValue(BiasObj *bias, u8 sensor_id)
{
	bool foundbias = false;

	if (bias->isBufferFilled)
	{
		Axis3f mean;
		Axis3f variance;
		float variance_base = 0;

		if(sensor_id == ACC_ID)
			variance_base = ACC_VARIANCE_BASE;
		if(sensor_id == GYRO_ID)
			variance_base = GYRO_VARIANCE_BASE;	

		sensorsCalculateVarianceAndMean(bias, &variance, &mean);

		if (variance.x < variance_base && variance.y < variance_base && variance.z < variance_base)
		{
			bias->bias.x = mean.x;
			bias->bias.y = mean.y;
			bias->bias.z = mean.z;
			foundbias = true;
			bias->isBiasValueFound = true;
		}
		else
			bias->isBufferFilled = false;
	}
	return foundbias;
}

/* 传感器初始化 */
void sensorsInit(void)
{
	if (isInit)
		return;

	sensorsDataReady = xSemaphoreCreateBinary(); /*创建传感器数据就绪二值信号量*/
	gyroBiasObjInit(&gyroBiasRunning);
	for(u8 i=0; i < 6; i++)
		accBiasObjInit(&accBiasRunning[i]);
	sensorsDeviceInit();	/*传感器器件初始化*/
	sensorsInterruptInit(); /*传感器中断初始化*/


	isInit = true;
}
/*设置传感器从模式读取*/
static void sensorsSetupSlaveRead(void)
{
	mpu6500SetSlave4MasterDelay(19); // 从机读取速率: 100Hz = (1000Hz / (1 + 9))

	mpu6500SetI2CBypassEnabled(false); //主机模式
	mpu6500SetWaitForExternalSensorEnabled(true);
	mpu6500SetInterruptMode(0);						  // 中断高电平有效
	mpu6500SetInterruptDrive(0);					  // 推挽输出
	mpu6500SetInterruptLatch(0);					  // 中断锁存模式(0=50us-pulse, 1=latch-until-int-cleared)
	mpu6500SetInterruptLatchClear(1);				  // 中断清除模式(0=status-read-only, 1=any-register-read)
	mpu6500SetSlaveReadWriteTransitionEnabled(false); // 关闭从机读写传输
	mpu6500SetMasterClockSpeed(13);					  // 设置i2c速度400kHz

#ifdef SENSORS_ENABLE_MAG_AK8963
	if (isMagPresent)
	{
		// 设置MPU6500主机要读取的寄存器
		mpu6500SetSlaveAddress(0, 0x80 | AK8963_ADDRESS_00); // 设置磁力计为0号从机
		mpu6500SetSlaveRegister(0, AK8963_RA_ST1);			 // 从机0需要读取的寄存器
		mpu6500SetSlaveDataLength(0, SENSORS_MAG_BUFF_LEN);	 // 读取8个字节(ST1, x, y, z heading, ST2 (overflow check))
		mpu6500SetSlaveDelayEnabled(0, true);
		mpu6500SetSlaveEnabled(0, true);
	}
#endif

	if (isBaroPresent && baroType == BMP388)
	{
		// 设置MPU6500主机要读取BMP388的寄存器
		mpu6500SetSlaveAddress(1, 0x80 | BMP3_I2C_ADDR_PRIM);  // 设置气压计状态寄存器为1号从机
		mpu6500SetSlaveRegister(1, BMP3_SENS_STATUS_REG_ADDR); // 从机1需要读取的寄存器
		mpu6500SetSlaveDataLength(1, SENSORS_BARO_BUFF_LEN);   // 读取7个字节
		mpu6500SetSlaveDelayEnabled(1, true);
		mpu6500SetSlaveEnabled(1, true);
	}
	if (isBaroPresent && baroType == SPL06)
	{
		// 设置MPU6500主机要读取SPL06的寄存器
		mpu6500SetSlaveAddress(1, 0x80 | SPL06_I2C_ADDR);		// 设置气压计状态寄存器为1号从机
		mpu6500SetSlaveRegister(1, SPL06_MODE_CFG_REG);			// 从机1需要读取的寄存器
		mpu6500SetSlaveDataLength(1, SENSORS_BARO_STATUS_LEN);	// 读取1个字节
		mpu6500SetSlaveDelayEnabled(1, true);
		mpu6500SetSlaveEnabled(1, true);

		mpu6500SetSlaveAddress(2, 0x80 | SPL06_I2C_ADDR);		// 设置气压计数据寄存器为2号从机
		mpu6500SetSlaveRegister(2, SPL06_PRESSURE_MSB_REG);		// 从机2需要读取的寄存器
		mpu6500SetSlaveDataLength(2, SENSORS_BARO_DATA_LEN);	// 读取6个字节
		mpu6500SetSlaveDelayEnabled(2, true);
		mpu6500SetSlaveEnabled(2, true);
	}

	mpu6500SetI2CMasterModeEnabled(true); //使能mpu6500主机模式
	mpu6500SetIntDataReadyEnabled(true);  //数据就绪中断使能
}

/**
 * 往方差缓冲区（循环缓冲区）添加一个新值，缓冲区满后，替换旧的的值
 */
static void sensorsAddBiasValue(BiasObj *bias, int16_t x, int16_t y, int16_t z)
{
	bias->bufHead->x = x;
	bias->bufHead->y = y;
	bias->bufHead->z = z;
	bias->bufHead++;

	if (bias->bufHead >= &bias->buffer[SENSORS_NBR_OF_BIAS_SAMPLES])
	{
		bias->bufHead = bias->buffer;
		bias->isBufferFilled = true;
	}
}
/**
 * 根据样本计算重力加速度计算偏置和缩放因子
 */
static bool processAccBias_Scale(int16_t ax, int16_t ay, int16_t az, Axis3f *accBiasOut)
{
	static bool accBiasFound = false;
	static Axis3f accbias[6] = {0};

	sensorsAddBiasValue(&accBiasRunning[processAccBias_stepnum], ax, ay, az);

	if (!accBiasRunning[processAccBias_stepnum].isBiasValueFound)
	{
		sensorsFindBiasValue(&accBiasRunning[processAccBias_stepnum], ACC_ID);
	}
	else
	{
		accbias[processAccBias_stepnum].x = accBiasRunning[processAccBias_stepnum].bias.x;
		accbias[processAccBias_stepnum].y = accBiasRunning[processAccBias_stepnum].bias.y;
		accbias[processAccBias_stepnum].z = accBiasRunning[processAccBias_stepnum].bias.z;
		//找到一组accbias,停止buffer缓冲数据，停止计算accbias,等待下一次的指令
		isreadytoprocessAccBias = false;
		//计算accbias当前步骤计算完成，发送应答信号
		sendaccBiasprocess_ACK(processAccBias_stepnum,accbias[processAccBias_stepnum].x,accbias[processAccBias_stepnum].y,accbias[processAccBias_stepnum].z);
	}

	if((processAccBias_stepnum == 5 ) && 
		(accBiasRunning[0].isBiasValueFound)&& 
		(accBiasRunning[1].isBiasValueFound)&& 
		(accBiasRunning[2].isBiasValueFound)&& 
		(accBiasRunning[3].isBiasValueFound)&& 
		(accBiasRunning[4].isBiasValueFound)&& 
		(accBiasRunning[5].isBiasValueFound))
	{
		accBiasOut->x = (accbias[0].x + accbias[1].x) / 2;
		accBiasOut->y = (accbias[2].y + accbias[3].y) / 2;
		accBiasOut->z = (accbias[4].z + accbias[5].z) / 2;
		accScale.x = fabsf(accbias[1].x - accbias[0].x) / 2;
		accScale.y = fabsf(accbias[3].y - accbias[2].y) / 2;
		accScale.z = fabsf(accbias[5].z - accbias[4].z) / 2;
		//acc六面矫正完成发送给，计算得到的bias值
		sendaccBiasprocess_ACK(6,accBiasOut->x,accBiasOut->y,accBiasOut->z);

		accBiasFound = true;
		processAccBias_stepnum = 0;
	}

	return accBiasFound;
}

// /**
//  * 根据样本计算重力加速度缩放因子
//  */
// static bool processAccScale(int16_t ax, int16_t ay, int16_t az)
// {
// 	static bool accScaleFound = false;
// 	static uint32_t accScaleSumCount = 0;

// 	if (!accScaleFound)
// 	{
// 		accScaleSum += sqrtf(powf(ax * SENSORS_G_PER_LSB_CFG, 2) + powf(ay * SENSORS_G_PER_LSB_CFG, 2) + powf(az * SENSORS_G_PER_LSB_CFG, 2));
// 		accScaleSumCount++;

// 		if (accScaleSumCount == SENSORS_ACC_SCALE_SAMPLES)
// 		{
// 			accScale = accScaleSum / SENSORS_ACC_SCALE_SAMPLES;
// 			accScaleFound = true;
// 		}
// 	}

// 	return accScaleFound;
// }

/**
 * 计算陀螺方差
 */
static bool processGyroBias(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut)
{
	sensorsAddBiasValue(&gyroBiasRunning, gx, gy, gz);

	if (!gyroBiasRunning.isBiasValueFound)
	{
		sensorsFindBiasValue(&gyroBiasRunning,GYRO_ID);
	}

	gyroBiasOut->x = gyroBiasRunning.bias.x;
	gyroBiasOut->y = gyroBiasRunning.bias.y;
	gyroBiasOut->z = gyroBiasRunning.bias.z;

	return gyroBiasRunning.isBiasValueFound;
}

/*处理气压计数据*/
void processBarometerMeasurements(const u8 *buffer)
{
	static float temp;
	static float pressure;
	struct bmp3_uncomp_data uncomp_data = {0};
	struct bmp3_data comp_data = {0};
	if (baroType == BMP388)
	{
		// Check if there is a new data update
		if ((buffer[0] & 0x30)) /*转换完成*/
		{
			uncomp_data.pressure = (uint64_t)((((u32)(buffer[3])) << 16) | (((u32)(buffer[2])) << 8) | (u32)buffer[1]);
			uncomp_data.temperature = (int64_t)((((u32)(buffer[6])) << 16) | (((u32)(buffer[5])) << 8) | (u32)buffer[4]);
			compensate_data(BMP3_ALL, &uncomp_data, &comp_data, BMP388_DEV.calib_data);
			pressure = (float)comp_data.pressure / 100.0f;
			sensors.baro.temperature = (float)comp_data.temperature / 100.0f; /*单位度*/

			pressureFilter(&pressure, &sensors.baro.pressure);
			sensors.baro.asl = PressureToAltitude(&sensors.baro.pressure) * 100; /*转换成海拔，单位：cm*/
			sensors.baro.asl = lpf2pApply(&BaroLpf, sensors.baro.asl);
		}
	}
	else if (baroType == SPL06)
	{
		s32 rawPressure = (int32_t)buffer[1]<<16 | (int32_t)buffer[2]<<8 | (int32_t)buffer[3];
		rawPressure = (rawPressure & 0x800000) ? (0xFF000000 | rawPressure) : rawPressure;
		
		s32 rawTemp = (int32_t)buffer[4]<<16 | (int32_t)buffer[5]<<8 | (int32_t)buffer[6];
		rawTemp = (rawTemp & 0x800000) ? (0xFF000000 | rawTemp) : rawTemp;
		
		temp = spl0601_get_temperature(rawTemp);
		pressure = spl0601_get_pressure(rawPressure, rawTemp);
		sensors.baro.pressure = pressure / 100.0f;
		sensors.baro.temperature = (float)temp; /*??????*/
		sensors.baro.asl = SPL06PressureToAltitude(sensors.baro.pressure) * 100.f; //cm
	}
}
/*处理磁力计数据*/
void processMagnetometerMeasurements(const uint8_t *buffer)
{
	if (buffer[0] & (1 << AK8963_ST1_DRDY_BIT))
	{
		int16_t headingx = (((int16_t)buffer[2]) << 8) | buffer[1];
		int16_t headingy = (((int16_t)buffer[4]) << 8) | buffer[3];
		int16_t headingz = (((int16_t)buffer[6]) << 8) | buffer[5];

		sensors.mag.x = (float)headingx / MAG_GAUSS_PER_LSB;
		sensors.mag.y = (float)headingy / MAG_GAUSS_PER_LSB;
		sensors.mag.z = (float)headingz / MAG_GAUSS_PER_LSB;
		magRaw.x = headingx; /*用于上传到上位机*/
		magRaw.y = headingy;
		magRaw.z = headingz;
	}
}
/*处理加速计和陀螺仪数据*/
void processAccGyroMeasurements(const uint8_t *buffer)
{
	/*注意传感器读取方向(旋转270°x和y交换)*/
	// 板方向定义在config.h中
#ifdef BOARD_VERTICAL
	int16_t ay = (((int16_t)buffer[0]) << 8) | buffer[1];
	int16_t az = -( (((int16_t)buffer[2]) << 8) | buffer[3] );
	int16_t ax = (((int16_t)buffer[4]) << 8) | buffer[5];
	int16_t gy = (((int16_t)buffer[8]) << 8) | buffer[9];
	int16_t gz = - ( (((int16_t)buffer[10]) << 8) | buffer[11] );
	int16_t gx = (((int16_t)buffer[12]) << 8) | buffer[13];
#else
#ifdef BOARD_HORIZONTAL
	int16_t ay = (((int16_t)buffer[0]) << 8) | buffer[1];
	int16_t ax = (((int16_t)buffer[2]) << 8) | buffer[3];
	int16_t az = (((int16_t)buffer[4]) << 8) | buffer[5];
	int16_t gy = (((int16_t)buffer[8]) << 8) | buffer[9];
	int16_t gx = (((int16_t)buffer[10]) << 8) | buffer[11];
	int16_t gz = (((int16_t)buffer[12]) << 8) | buffer[13];
#else
	#error "Board alignment is not defined. Define BOARD_VERTICAL or BOARD_HORIZONTAL in config.h."
#endif
#endif

	accRaw.x = ax - accBias.x; /*用于上传到上位机*/
	accRaw.y = ay - accBias.y;
	accRaw.z = az - accBias.z;
	gyroRaw.x = gx - gyroBias.x;
	gyroRaw.y = gy - gyroBias.y;
	gyroRaw.z = gz - gyroBias.z;

	gyroBiasFound = processGyroBias(gx, gy, gz, &gyroBias);

	if (gyroBiasFound)
	{
		if(!accBiasFound && isreadytoprocessAccBias)
			processAccBias_Scale(ax, ay, az,&accBias);
		
		//processAccScale(ax, ay, az); /*计算accScale*/
	}

	sensors.gyro.x = -(gx - gyroBias.x) * SENSORS_DEG_PER_LSB_CFG; /*单位 °/s */
	sensors.gyro.y =  (gy - gyroBias.y) * SENSORS_DEG_PER_LSB_CFG;
	sensors.gyro.z =  (gz - gyroBias.z) * SENSORS_DEG_PER_LSB_CFG;
	applyAxis3fLpf(gyroLpf, &sensors.gyro);

	// sensors.acc.x = -(ax)*SENSORS_G_PER_LSB_CFG / accScale.x; /*单位 g(9.8m/s^2)*/
	// sensors.acc.y =  (ay)*SENSORS_G_PER_LSB_CFG / accScale.y;	/*重力加速度缩放因子accScale 根据样本计算得出*/
	// sensors.acc.z =  (az)*SENSORS_G_PER_LSB_CFG / accScale.z;
	sensors.acc.x = -(accRaw.x) / accScale.x;		/*单位 g(9.8m/s^2)*/
	sensors.acc.y =   accRaw.y	/ accScale.y;		/*单位 g(9.8m/s^2)*/
	sensors.acc.z =   accRaw.z	/ accScale.z;		/*单位 g(9.8m/s^2)*/

	applyAxis3fLpf(accLpf, &sensors.acc);

	// Axis3f gyroTmp;
	// gyroTmp.x = sensors.gyro.x;
	// gyroTmp.y = sensors.gyro.y;
	// gyroTmp.z = sensors.gyro.z;

	// sensors.gyro.x = (sensors.gyro.x + gyroBff.x) * 0.5f;
	// sensors.gyro.y = (sensors.gyro.y + gyroBff.y) * 0.5f;
	// sensors.gyro.z = (sensors.gyro.z + gyroBff.z) * 0.5f;

	// gyroBff.x = gyroTmp.x;
	// gyroBff.y = gyroTmp.y;
	// gyroBff.z = gyroTmp.z;
}

/*传感器任务*/
void sensorsTask(void *param)
{
//	float accraw_num[3] = {0, 0, 0};
	sensorsInit(); /*传感器初始化*/
	vTaskDelay(150);
	//MPU9250设置为IIC主机模式
	sensorsSetupSlaveRead(); /*设置传感器从模式读取*/

	while (1)
	{
		if (xSemaphoreTake(sensorsDataReady, portMAX_DELAY) == pdTRUE)
		{
			/*确定数据长度*/
			u8 dataLen = (u8)(SENSORS_MPU6500_BUFF_LEN +
							  (isMagPresent ? SENSORS_MAG_BUFF_LEN : 0) +
							  (isBaroPresent ? SENSORS_BARO_BUFF_LEN : 0));
			//读MPU9250 陀螺仪+加速度计+磁力计+气压计的数据
			i2cdevRead(I2C3_DEV, MPU6500_ADDRESS_AD0_HIGH, MPU6500_RA_ACCEL_XOUT_H, dataLen, buffer);

			/*处理原始数据，并放入数据队列中*/
			processAccGyroMeasurements(&(buffer[0]));
			if (isMagPresent)
			{
				processMagnetometerMeasurements(&(buffer[SENSORS_MPU6500_BUFF_LEN]));
			}
			if (isBaroPresent)
			{
				//				bmp3_get_sensor_data(BMP3_ALL, &BMP_buffer_comp , BMP388_DEV);
				processBarometerMeasurements(&(buffer[isMagPresent ? SENSORS_MPU6500_BUFF_LEN + SENSORS_MAG_BUFF_LEN : SENSORS_MPU6500_BUFF_LEN]));
			}

			vTaskSuspendAll(); /*确保同一时刻把数据放入队列中*/
			xQueueSend(accelerometerDataQueue, &sensors.acc, 0);
			xQueueSend(gyroDataQueue, &sensors.gyro, 0);
			if (isBaroPresent)
			{
				xQueueOverwrite(barometerDataQueue, &sensors.baro);
			}
			if (isMagPresent)
			{
				xQueueOverwrite(magnetometerDataQueue, &sensors.mag);
			}

			xTaskResumeAll();
		}
	}
}
/*获取传感器数据*/
void sensorsAcquire(sensorData_t *sensors, const u32 tick)
{
	sensorsReadGyro(&sensors->gyro);
	sensorsReadAcc(&sensors->acc);
	sensorsReadMag(&sensors->mag);
	sensorsReadBaro(&sensors->baro);
}

void __attribute__((used)) EXTI11_Callback(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(sensorsDataReady, &xHigherPriorityTaskWoken);

	if (xHigherPriorityTaskWoken)
	{
		portYIELD();
	}
}

//void __attribute__((used)) EXTI6_Callback(void)
//{
//	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
//	xSemaphoreGiveFromISR(BMP388_sensorsDataReady, &xHigherPriorityTaskWoken);

//	if (xHigherPriorityTaskWoken)
//	{
//		portYIELD();
//	}
//}
/*二阶低通滤波*/
static void applyAxis3fLpf(lpf2pData *data, Axis3f *in)
{
	for (u8 i = 0; i < 3; i++)
	{
		in->axis[i] = lpf2pApply(&data[i], in->axis[i]);
	}
}
/*传感器数据校准*/
bool sensorsAreCalibrated()
{
	return gyroBiasFound;
}
/*上位机获取读取原始数据*/
void getSensorRawData(Axis3i16 *acc, Axis3i16 *gyro, Axis3i16 *mag)
{
    *acc  = accRaw;
    *gyro = gyroRaw;
    *mag  = magRaw;
}

bool getIsMPU9250Present(void)
{
	bool value = isMPUPresent;
#ifdef SENSORS_ENABLE_MAG_AK8963
	value &= isMagPresent;
#endif
	return value;
}

bool getIsBaroPresent(void)
{
	return isBaroPresent;
}

void setMagCalibData(Axis3i16 offset, Axis3u16 radius)
{
	sensors.mag_calibration.offset = offset;
	sensors.mag_calibration.radius = radius;
}
void setprocessAccBias_stepnum( u8 i )
{
	processAccBias_stepnum = i;			//设置当前计算accbias到哪一步了
	isreadytoprocessAccBias = true;		//将允许进行accbias计算标志位使能
	accBiasFound = false;				//重新开始标定时，回复accBiasFound为false状态
}

void reset_accbiasRunning(void)
{
	for(u8 i = 0;i < 6; i++ )
	{
		accBiasRunning[i].isBufferFilled = false;
		accBiasRunning[i].isBiasValueFound = false;
		accBiasRunning[i].bufHead = accBiasRunning[i].buffer;
		accBiasRunning[i].bias.x = 0.0f;
		accBiasRunning[i].bias.y = 0.0f;
		accBiasRunning[i].bias.z = 0.0f;	
	}
}

Axis3f getaccBias(void)
{
	return accBias;
}

Axis3f getaccScale(void)
{
	return accScale;
}

void resetaccBias_accScale(void)
{
    accBias.x = 0;
    accBias.y = 0;
    accBias.z = 0;

    accScale.x = 1;
    accScale.y = 1;
    accScale.z = 1;
}
