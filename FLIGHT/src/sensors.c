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


/*FreeRTOS���ͷ�ļ�*/
#include "FreeRTOS.h"
#include "task.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ���������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * ���������ƴ���	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2017/5/12
 * �汾��V1.3
 * ��Ȩ���У�����ؾ���
 * Copyright(C) �������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/

#define SENSORS_GYRO_FS_CFG       MPU6500_GYRO_FS_2000
#define SENSORS_DEG_PER_LSB_CFG   MPU6500_DEG_PER_LSB_2000

#define SENSORS_ACCEL_FS_CFG      MPU6500_ACCEL_FS_16	
#define SENSORS_G_PER_LSB_CFG     MPU6500_G_PER_LSB_16

#define SENSORS_NBR_OF_BIAS_SAMPLES		1024	/* ���㷽��Ĳ����������� */
#define GYRO_VARIANCE_BASE				4000	/* ��������ƫ������ֵ */
#define SENSORS_ACC_SCALE_SAMPLES  		200		/* ���ټƲ������� */

// MPU9250����ģʽ��ȡ���� ����������
#define SENSORS_MPU6500_BUFF_LEN    14
#define SENSORS_MAG_BUFF_LEN       	8
#define SENSORS_BARO_STATUS_LEN		1
#define SENSORS_BARO_DATA_LEN		6
#define SENSORS_BARO_BUFF_LEN       (SENSORS_BARO_STATUS_LEN + SENSORS_BARO_DATA_LEN)

typedef struct
{
	Axis3f     bias;
	bool       isBiasValueFound;
	bool       isBufferFilled;
	Axis3i16*  bufHead;
	Axis3i16   buffer[SENSORS_NBR_OF_BIAS_SAMPLES];
}BiasObj;

BiasObj	gyroBiasRunning;
static Axis3f  gyroBias;

static bool gyroBiasFound = false;
static float accScaleSum = 0;
static float accScale = 1;

static bool isInit = false;
static sensorData_t sensors;
static Axis3i16	gyroRaw;
static Axis3i16	accRaw;
static Axis3i16 magRaw;

/*��ͨ�˲�����*/
#define GYRO_LPF_CUTOFF_FREQ  80
#define ACCEL_LPF_CUTOFF_FREQ 30
static lpf2pData accLpf[3];
static lpf2pData gyroLpf[3];

static bool isMPUPresent=false;
static bool isMagPresent=false;
static bool isBaroPresent=false;

enum {IDLE, BMP388, SPL06}baroType = IDLE;


static uint8_t buffer[SENSORS_MPU6500_BUFF_LEN + SENSORS_MAG_BUFF_LEN + SENSORS_BARO_BUFF_LEN] = {0};


static xQueueHandle accelerometerDataQueue;
static xQueueHandle gyroDataQueue;
static xQueueHandle magnetometerDataQueue;
static xQueueHandle barometerDataQueue;
static xSemaphoreHandle sensorsDataReady;



static void applyAxis3fLpf(lpf2pData *data, Axis3f* in);
static void sensorsBiasObjInit(BiasObj* bias);
static void sensorsCalculateVarianceAndMean(BiasObj* bias, Axis3f* varOut, Axis3f* meanOut);
static bool sensorsFindBiasValue(BiasObj* bias);
static void sensorsAddBiasValue(BiasObj* bias, int16_t x, int16_t y, int16_t z);


/*�Ӷ��ж�ȡ��������*/
bool sensorsReadGyro(Axis3f *gyro)
{
	return (pdTRUE == xQueueReceive(gyroDataQueue, gyro, 0));
}
/*�Ӷ��ж�ȡ���ټ�����*/
bool sensorsReadAcc(Axis3f *acc)
{
	return (pdTRUE == xQueueReceive(accelerometerDataQueue, acc, 0));
}
/*�Ӷ��ж�ȡ����������*/
bool sensorsReadMag(Axis3f *mag)
{
	return (pdTRUE == xQueueReceive(magnetometerDataQueue, mag, 0));
}
/*�Ӷ��ж�ȡ��ѹ����*/
bool sensorsReadBaro(baro_t *baro)
{
	return (pdTRUE == xQueueReceive(barometerDataQueue, baro, 0));
}
/*�������жϳ�ʼ��*/
static void sensorsInterruptInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	/*ʹ��MPU6500�ж�*/
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

/* ������������ʼ�� */
void sensorsDeviceInit(void)
{
	i2cdevInit(I2C3_DEV);
	mpu6500Init(I2C3_DEV);	
	
	vTaskDelay(10);
	mpu6500Reset();	// ��λMPU6500
	vTaskDelay(50);	// ��ʱ�ȴ��Ĵ�����λ
	
	u8 temp = mpu6500GetDeviceID();
	if (temp == 0x38 || temp == 0x39)
	{
		isMPUPresent=true;
		//printf("MPU9250 I2C connection [OK].\n");
	}
	else
	{
		//printf("MPU9250 I2C connection [FAIL].\n");
	}
	
	mpu6500SetSleepEnabled(false);	// ����MPU6500
	vTaskDelay(10);		
	mpu6500SetClockSource(MPU6500_CLOCK_PLL_XGYRO);	// ����X��������Ϊʱ��	
	vTaskDelay(10);		// ��ʱ�ȴ�ʱ���ȶ�	
	mpu6500SetTempSensorEnabled(false);	// ʹ���¶ȴ�����	
	mpu6500SetI2CBypassEnabled(true);	// ��·ģʽ�������ƺ���ѹ���ӵ���IIC	
	mpu6500SetFullScaleAccelRange(SENSORS_ACCEL_FS_CFG);// ���ü��ټ�����	
	mpu6500SetFullScaleGyroRange(SENSORS_GYRO_FS_CFG);	// ������������	
	mpu6500SetAccelDLPF(MPU9250_ACCEL_DLPF_BW_44);		// ���ü��ټ����ֵ�ͨ�˲�,ͬʱACCEL_FCHOICE_Bû�����ã�Ĭ��Ϊ0����A_DLPF_CFG��ͬ�������Ƶ�ʣ���ֹƵ��Ϊ44.8Hz�����Ƶ��Ϊ1KHz

	mpu6500SetRate(0);// ���ò�������: 1000 / (1 + 0) = 1000Hz
	mpu6500SetDLPFMode(MPU6500_DLPF_BW_92);// �����������ֵ�ͨ�˲�
	
	for (u8 i = 0; i < 3; i++)// ��ʼ�����ټƺ����ݶ��׵�ͨ�˲�
	{
		lpf2pInit(&gyroLpf[i], 1000, GYRO_LPF_CUTOFF_FREQ);
		lpf2pInit(&accLpf[i],  1000, ACCEL_LPF_CUTOFF_FREQ);
	}


#ifdef SENSORS_ENABLE_MAG_AK8963
	ak8963Init(I2C3_DEV);	//ak8963�����Ƴ�ʼ��
	if (ak8963TestConnection() == true)
	{
		isMagPresent = true;
		ak8963SetMode(AK8963_MODE_16BIT | AK8963_MODE_CONT2); // 16bit 100Hz
//		printf("AK8963 I2C connection [OK].\n");
	}
	else
	{
//		printf("AK8963 I2C connection [FAIL].\n");
	}
#endif

	if ( bmp3_init(BMP388_DEV) == true)//BMP388��ʼ��
	{
		isBaroPresent = true;
		baroType = BMP388;

		bmp3_set_sensor_settings(BMP3_ALL_SETTINGS,BMP388_DEV);
		bmp3_set_op_mode(BMP388_DEV);						
		vTaskDelay(100);
//		bmp3_get_sensor_settings(BMP388_DEV);
		
	}
	else
	{
		isBaroPresent = false;
	}

	/*�������������ݶ���*/
	accelerometerDataQueue = xQueueCreate(1, sizeof(Axis3f));
	gyroDataQueue = xQueueCreate(1, sizeof(Axis3f));
	magnetometerDataQueue = xQueueCreate(1, sizeof(Axis3f));
	barometerDataQueue = xQueueCreate(1, sizeof(baro_t));
}
/*������ƫ�ó�ʼ��*/
static void sensorsBiasObjInit(BiasObj* bias)
{
	bias->isBufferFilled = false;
	bias->bufHead = bias->buffer;
}
/*����������*/
bool sensorsTest(void)
{
	bool testStatus = true;

	if (!isInit)
	{
		printf("Uninitialized\n");
		testStatus = false;
	}

	testStatus&=isBaroPresent;
	
	return testStatus;
}

/*���㷽���ƽ��ֵ*/
static void sensorsCalculateVarianceAndMean(BiasObj* bias, Axis3f* varOut, Axis3f* meanOut)
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

	varOut->x = (sumsq[0] - ((int64_t)sum[0] * sum[0]) / SENSORS_NBR_OF_BIAS_SAMPLES);
	varOut->y = (sumsq[1] - ((int64_t)sum[1] * sum[1]) / SENSORS_NBR_OF_BIAS_SAMPLES);
	varOut->z = (sumsq[2] - ((int64_t)sum[2] * sum[2]) / SENSORS_NBR_OF_BIAS_SAMPLES);

	meanOut->x = (float)sum[0] / SENSORS_NBR_OF_BIAS_SAMPLES;
	meanOut->y = (float)sum[1] / SENSORS_NBR_OF_BIAS_SAMPLES;
	meanOut->z = (float)sum[2] / SENSORS_NBR_OF_BIAS_SAMPLES;
}
/*����������ƫ��ֵ*/
static bool sensorsFindBiasValue(BiasObj* bias)
{
	bool foundbias = false;

	if (bias->isBufferFilled)
	{
		
		Axis3f mean;
		Axis3f variance;
		sensorsCalculateVarianceAndMean(bias, &variance, &mean);

		if (variance.x < GYRO_VARIANCE_BASE && variance.y < GYRO_VARIANCE_BASE && variance.z < GYRO_VARIANCE_BASE)
		{
			bias->bias.x = mean.x;
			bias->bias.y = mean.y;
			bias->bias.z = mean.z;
			foundbias = true;
			bias->isBiasValueFound= true;
		}else
			bias->isBufferFilled=false;
	}
	return foundbias;
}

/* ��������ʼ�� */
void sensorsInit(void)
{
	if(isInit) return;

	sensorsDataReady = xSemaphoreCreateBinary();/*�������������ݾ�����ֵ�ź���*/
	sensorsBiasObjInit(&gyroBiasRunning);
	sensorsDeviceInit();	/*������������ʼ��*/
	sensorsInterruptInit();	/*�������жϳ�ʼ��*/
	
	isInit = true;
}
/*���ô�������ģʽ��ȡ*/
static void sensorsSetupSlaveRead(void)
{
	mpu6500SetSlave4MasterDelay(19); 	// �ӻ���ȡ����: 100Hz = (1000Hz / (1 + 9))

	mpu6500SetI2CBypassEnabled(false);	//����ģʽ
	mpu6500SetWaitForExternalSensorEnabled(true); 	
	mpu6500SetInterruptMode(0); 		// �жϸߵ�ƽ��Ч
	mpu6500SetInterruptDrive(0); 		// �������
	mpu6500SetInterruptLatch(0); 		// �ж�����ģʽ(0=50us-pulse, 1=latch-until-int-cleared)
	mpu6500SetInterruptLatchClear(1); 	// �ж����ģʽ(0=status-read-only, 1=any-register-read)
	mpu6500SetSlaveReadWriteTransitionEnabled(false); // �رմӻ���д����
	mpu6500SetMasterClockSpeed(13); 	// ����i2c�ٶ�400kHz

#ifdef SENSORS_ENABLE_MAG_AK8963
	if (isMagPresent)
	{
		// ����MPU6500����Ҫ��ȡ�ļĴ���
		mpu6500SetSlaveAddress(0, 0x80 | AK8963_ADDRESS_00); 	// ���ô�����Ϊ0�Ŵӻ�
		mpu6500SetSlaveRegister(0, AK8963_RA_ST1); 				// �ӻ�0��Ҫ��ȡ�ļĴ���
		mpu6500SetSlaveDataLength(0, SENSORS_MAG_BUFF_LEN); 	// ��ȡ8���ֽ�(ST1, x, y, z heading, ST2 (overflow check))
		mpu6500SetSlaveDelayEnabled(0, true);
		mpu6500SetSlaveEnabled(0, true);
	}
#endif


	if (isBaroPresent && baroType == BMP388)
	{
		// ����MPU6500����Ҫ��ȡBMP388�ļĴ���
		mpu6500SetSlaveAddress(1, 0x80 | BMP3_I2C_ADDR_PRIM);	// ������ѹ��״̬�Ĵ���Ϊ1�Ŵӻ�
		mpu6500SetSlaveRegister(1, BMP3_SENS_STATUS_REG_ADDR);	// �ӻ�1��Ҫ��ȡ�ļĴ���
		mpu6500SetSlaveDataLength(1, SENSORS_BARO_BUFF_LEN);	// ��ȡ7���ֽ�
		mpu6500SetSlaveDelayEnabled(1, true);
		mpu6500SetSlaveEnabled(1, true);
	}

	mpu6500SetI2CMasterModeEnabled(true);	//ʹ��mpu6500����ģʽ
	mpu6500SetIntDataReadyEnabled(true);	//���ݾ����ж�ʹ��
}

/**
 * �����������ѭ��������������һ����ֵ�������������滻�ɵĵ�ֵ
 */
static void sensorsAddBiasValue(BiasObj* bias, int16_t x, int16_t y, int16_t z)
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
 * �������������������ٶ���������
 */
static bool processAccScale(int16_t ax, int16_t ay, int16_t az)
{
	static bool accBiasFound = false;
	static uint32_t accScaleSumCount = 0;

	if (!accBiasFound)
	{
		accScaleSum += sqrtf(powf(ax * SENSORS_G_PER_LSB_CFG, 2) + powf(ay * SENSORS_G_PER_LSB_CFG, 2) + powf(az * SENSORS_G_PER_LSB_CFG, 2));
		accScaleSumCount++;

		if (accScaleSumCount == SENSORS_ACC_SCALE_SAMPLES)
		{
			accScale = accScaleSum / SENSORS_ACC_SCALE_SAMPLES;
			accBiasFound = true;
		}
	}

	return accBiasFound;
}

/**
 * �������ݷ���
 */
static bool processGyroBias(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut)
{
	sensorsAddBiasValue(&gyroBiasRunning, gx, gy, gz);

	if (!gyroBiasRunning.isBiasValueFound)
	{
		sensorsFindBiasValue(&gyroBiasRunning);
	}

	gyroBiasOut->x = gyroBiasRunning.bias.x;
	gyroBiasOut->y = gyroBiasRunning.bias.y;
	gyroBiasOut->z = gyroBiasRunning.bias.z;

	return gyroBiasRunning.isBiasValueFound;
}

/*������ѹ������*/
void processBarometerMeasurements(const u8 *buffer)
{
	static float temp;
    static float pressure;
	struct bmp3_uncomp_data uncomp_data = { 0 };
	struct bmp3_data comp_data = { 0 };
	if (baroType == BMP388)
	{
		// Check if there is a new data update
		if ((buffer[0] & 0x30)) /*ת�����*/
		{
			uncomp_data.pressure 	= (uint64_t)((((u32)(buffer[3] )) << 16) | (((u32)(buffer[2])) << 8) | (u32)buffer[1]);
			uncomp_data.temperature =  (int64_t)((((u32)(buffer[6])) << 16) | (((u32)(buffer[5])) << 8) | (u32)buffer[4]);	
			compensate_data(BMP3_ALL, &uncomp_data, &comp_data, BMP388_DEV.calib_data);		
			pressure = (float)comp_data.pressure/100.0f;
			sensors.baro.temperature = (float)comp_data.temperature/100.0f;/*��λ��*/

			pressureFilter(&pressure, &sensors.baro.pressure);	
			sensors.baro.asl = PressureToAltitude(&sensors.baro.pressure) ;	/*ת���ɺ���*/
		}
	}

}
/*��������������*/
void processMagnetometerMeasurements(const uint8_t *buffer)
{
	if (buffer[0] & (1 << AK8963_ST1_DRDY_BIT)) 
	{
		int16_t headingx = (((int16_t) buffer[2]) << 8) | buffer[1];
		int16_t headingy = (((int16_t) buffer[4]) << 8) | buffer[3];
		int16_t headingz = (((int16_t) buffer[6]) << 8) | buffer[5];

		sensors.mag.x = (float)headingx / MAG_GAUSS_PER_LSB;
		sensors.mag.y = (float)headingy / MAG_GAUSS_PER_LSB;
		sensors.mag.z = (float)headingz / MAG_GAUSS_PER_LSB;		
		magRaw.x = headingx;/*�����ϴ�����λ��*/
		
		magRaw.y = headingy;
		magRaw.z = headingz;
	}
}
/*�������ټƺ�����������*/
void processAccGyroMeasurements(const uint8_t *buffer)
{
	/*ע�⴫������ȡ����(��ת270��x��y����)*/
	int16_t ay = (((int16_t) buffer[0]) << 8) | buffer[1];
	int16_t ax = ((((int16_t) buffer[2]) << 8) | buffer[3]);
	int16_t az = (((int16_t) buffer[4]) << 8) | buffer[5];
	int16_t gy = (((int16_t) buffer[8]) << 8) | buffer[9];
	int16_t gx = (((int16_t) buffer[10]) << 8) | buffer[11];
	int16_t gz = (((int16_t) buffer[12]) << 8) | buffer[13];

	accRaw.x = ax;/*�����ϴ�����λ��*/
	accRaw.y = ay;
	accRaw.z = az;
	gyroRaw.x = gx - gyroBias.x;
	gyroRaw.y = gy - gyroBias.y;
	gyroRaw.z = gz - gyroBias.z;

	gyroBiasFound = processGyroBias(gx, gy, gz, &gyroBias);
	
	if (gyroBiasFound)
	{
		processAccScale(ax, ay, az);	/*����accScale*/
	}
	
	sensors.gyro.x = -(gx - gyroBias.x) * SENSORS_DEG_PER_LSB_CFG;	/*��λ ��/s */
	sensors.gyro.y =  (gy - gyroBias.y) * SENSORS_DEG_PER_LSB_CFG;
	sensors.gyro.z =  (gz - gyroBias.z) * SENSORS_DEG_PER_LSB_CFG;
	applyAxis3fLpf(gyroLpf, &sensors.gyro);	

	sensors.acc.x = -(ax) * SENSORS_G_PER_LSB_CFG / accScale;	/*��λ g(9.8m/s^2)*/
	sensors.acc.y =  (ay) * SENSORS_G_PER_LSB_CFG / accScale;	/*�������ٶ���������accScale ������������ó�*/
	sensors.acc.z =  (az) * SENSORS_G_PER_LSB_CFG / accScale;

	applyAxis3fLpf(accLpf, &sensors.acc);
}
/*����������*/
void sensorsTask(void *param)
{
	sensorsInit();	/*��������ʼ��*/
	vTaskDelay(150);
//MPU9250����ΪIIC����ģʽ
	sensorsSetupSlaveRead();/*���ô�������ģʽ��ȡ*/

	while (1)
	{
		if (xSemaphoreTake(sensorsDataReady, portMAX_DELAY) == pdTRUE)
		{

			/*ȷ�����ݳ���*/
			u8 dataLen = (u8) (SENSORS_MPU6500_BUFF_LEN +
				(isMagPresent ? SENSORS_MAG_BUFF_LEN : 0) +
				(isBaroPresent ? SENSORS_BARO_BUFF_LEN : 0));
			//��MPU9250 ������+���ٶȼ�+������+��ѹ�Ƶ�����
			i2cdevRead(I2C3_DEV, MPU6500_ADDRESS_AD0_HIGH, MPU6500_RA_ACCEL_XOUT_H, dataLen, buffer);
			
			/*����ԭʼ���ݣ����������ݶ�����*/
			processAccGyroMeasurements(&(buffer[0]));

			if (isMagPresent)
			{
				processMagnetometerMeasurements(&(buffer[SENSORS_MPU6500_BUFF_LEN]));
			}
			if (isBaroPresent)
			{
//				bmp3_get_sensor_data(BMP3_ALL, &BMP_buffer_comp , BMP388_DEV);
				processBarometerMeasurements(&(buffer[isMagPresent ?
					SENSORS_MPU6500_BUFF_LEN + SENSORS_MAG_BUFF_LEN : SENSORS_MPU6500_BUFF_LEN]));
			}

			vTaskSuspendAll();	/*ȷ��ͬһʱ�̰����ݷ��������*/
			xQueueOverwrite(accelerometerDataQueue, &sensors.acc);
			xQueueOverwrite(gyroDataQueue, &sensors.gyro);
			if (isBaroPresent)
			{
				xQueueOverwrite(barometerDataQueue, &sensors.baro);
			}
			if(isMagPresent)
			{
				xQueueOverwrite(magnetometerDataQueue, &sensors.mag);
			}

			xTaskResumeAll();
		}





	}	
}
/*��ȡ����������*/
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
/*���׵�ͨ�˲�*/
static void applyAxis3fLpf(lpf2pData *data, Axis3f* in)
{
	for (u8 i = 0; i < 3; i++) 
	{
		in->axis[i] = lpf2pApply(&data[i], in->axis[i]);
	}
}
/*����������У׼*/
bool sensorsAreCalibrated()	
{
	return gyroBiasFound;
}
/*��λ����ȡ��ȡԭʼ����*/
void getSensorRawData(Axis3i16* acc, Axis3i16* gyro, Axis3i16* mag)
{
	*acc = accRaw;
	*gyro = gyroRaw;
	*mag = magRaw;
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


