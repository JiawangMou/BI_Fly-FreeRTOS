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

/*FreeRTOS���ͷ�ļ�*/
#include "FreeRTOS.h"
#include "task.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * ���������ƴ���	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2017/5/12
 * �汾��V1.3
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/

#define SENSORS_GYRO_FS_CFG MPU6500_GYRO_FS_2000
#define SENSORS_DEG_PER_LSB_CFG MPU6500_DEG_PER_LSB_2000

#define SENSORS_ACCEL_FS_CFG MPU6500_ACCEL_FS_16
#define SENSORS_G_PER_LSB_CFG MPU6500_G_PER_LSB_16

#define SENSORS_NBR_OF_BIAS_SAMPLES 1024 /* ���㷽��Ĳ����������� */
#define GYRO_VARIANCE_BASE 300			 /* ��������ƫ������ֵ */
#define ACC_VARIANCE_BASE  300			 /* ��������ƫ������ֵ */
#define SENSORS_ACC_SCALE_SAMPLES 200	 /* ���ټƲ������� */

// MPU9250����ģʽ��ȡ���� ����������
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

/*�������accbias�����־λ*/ 
static bool isreadytoprocessAccBias = false;   
//static float accScaleSum = 0;
static Axis3f accScale;

static bool isInit = false;
static sensorData_t sensors;


static Axis3i16 gyroRaw;
static Axis3i16 accRaw;
static Axis3i16 magRaw;

// static Axis3f gyroBff;

/*��ͨ�˲�����*/
#define GYRO_LPF_CUTOFF_FREQ 30
#define ACCEL_LPF_CUTOFF_FREQ 10
#define BARO_LPF_CUTOFF_FREQ 20
static lpf2pData accLpf[3];
static lpf2pData gyroLpf[3];
static lpf2pData BaroLpf;

static bool isMPUPresent = false;
static bool isMagPresent = false;
static bool isBaroPresent = false;

static u8 processAccBias_stepnum = 0;	//Ϊ0ʱ��ʾû�н���accBias���㣬�������ʱ��stepnum ��ʾ��ǰ�Ĳ���� 0~5��

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

/*�Ӷ��ж�ȡ��������*/
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
/*�Ӷ��ж�ȡ���ټ�����*/
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
	mpu6500Reset(); // ��λMPU6500
	vTaskDelay(50); // ��ʱ�ȴ��Ĵ�����λ

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

	mpu6500SetSleepEnabled(false); // ����MPU6500
	vTaskDelay(10);
	mpu6500SetClockSource(MPU6500_CLOCK_PLL_XGYRO);		 // ����X��������Ϊʱ��
	vTaskDelay(10);										 // ��ʱ�ȴ�ʱ���ȶ�
	mpu6500SetTempSensorEnabled(false);					 // ʹ���¶ȴ�����
	mpu6500SetI2CBypassEnabled(true);					 // ��·ģʽ�������ƺ���ѹ���ӵ���IIC
	mpu6500SetFullScaleAccelRange(SENSORS_ACCEL_FS_CFG); // ���ü��ټ�����
	mpu6500SetFullScaleGyroRange(SENSORS_GYRO_FS_CFG);	 // ������������
	mpu6500SetAccelDLPF(MPU9250_ACCEL_DLPF_BW_21);		 // ���ü��ټ����ֵ�ͨ�˲�,ͬʱACCEL_FCHOICE_Bû�����ã�Ĭ��Ϊ0����A_DLPF_CFG��ͬ�������Ƶ�ʣ���ֹƵ��Ϊ21.2Hz�����Ƶ��Ϊ1KHz

	mpu6500SetRate(0);						// ���ò�������: 1000 / (1 + 0) = 1000Hz
	mpu6500SetDLPFMode(MPU6500_DLPF_BW_41); // �����������ֵ�ͨ�˲�

	for (u8 i = 0; i < 3; i++) // ��ʼ�����ټƺ����ݶ��׵�ͨ�˲�
	{
		lpf2pInit(&gyroLpf[i], 1000, GYRO_LPF_CUTOFF_FREQ);
		lpf2pInit(&accLpf[i], 1000, ACCEL_LPF_CUTOFF_FREQ);
	}
	lpf2pInit(&BaroLpf, 1000, BARO_LPF_CUTOFF_FREQ);

#ifdef SENSORS_ENABLE_MAG_AK8963
	ak8963Init(I2C3_DEV); //ak8963�����Ƴ�ʼ��
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

	if (bmp3_init(BMP388_DEV) == true) //BMP388��ʼ��
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

	/*�������������ݶ���*/
	accelerometerDataQueue = xQueueCreate(10, sizeof(Axis3f));
	gyroDataQueue = xQueueCreate(10, sizeof(Axis3f));
	magnetometerDataQueue = xQueueCreate(1, sizeof(Axis3f));
	barometerDataQueue = xQueueCreate(1, sizeof(baro_t));
}
/*������ƫ�ó�ʼ��*/
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
/*����������*/
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

/*���㷽���ƽ��ֵ*/
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
/*����������ƫ��ֵ*/
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

/* ��������ʼ�� */
void sensorsInit(void)
{
	if (isInit)
		return;

	sensorsDataReady = xSemaphoreCreateBinary(); /*�������������ݾ�����ֵ�ź���*/
	gyroBiasObjInit(&gyroBiasRunning);
	for(u8 i=0; i < 6; i++)
		accBiasObjInit(&accBiasRunning[i]);
	sensorsDeviceInit();	/*������������ʼ��*/
	sensorsInterruptInit(); /*�������жϳ�ʼ��*/


	isInit = true;
}
/*���ô�������ģʽ��ȡ*/
static void sensorsSetupSlaveRead(void)
{
	mpu6500SetSlave4MasterDelay(19); // �ӻ���ȡ����: 100Hz = (1000Hz / (1 + 9))

	mpu6500SetI2CBypassEnabled(false); //����ģʽ
	mpu6500SetWaitForExternalSensorEnabled(true);
	mpu6500SetInterruptMode(0);						  // �жϸߵ�ƽ��Ч
	mpu6500SetInterruptDrive(0);					  // �������
	mpu6500SetInterruptLatch(0);					  // �ж�����ģʽ(0=50us-pulse, 1=latch-until-int-cleared)
	mpu6500SetInterruptLatchClear(1);				  // �ж����ģʽ(0=status-read-only, 1=any-register-read)
	mpu6500SetSlaveReadWriteTransitionEnabled(false); // �رմӻ���д����
	mpu6500SetMasterClockSpeed(13);					  // ����i2c�ٶ�400kHz

#ifdef SENSORS_ENABLE_MAG_AK8963
	if (isMagPresent)
	{
		// ����MPU6500����Ҫ��ȡ�ļĴ���
		mpu6500SetSlaveAddress(0, 0x80 | AK8963_ADDRESS_00); // ���ô�����Ϊ0�Ŵӻ�
		mpu6500SetSlaveRegister(0, AK8963_RA_ST1);			 // �ӻ�0��Ҫ��ȡ�ļĴ���
		mpu6500SetSlaveDataLength(0, SENSORS_MAG_BUFF_LEN);	 // ��ȡ8���ֽ�(ST1, x, y, z heading, ST2 (overflow check))
		mpu6500SetSlaveDelayEnabled(0, true);
		mpu6500SetSlaveEnabled(0, true);
	}
#endif

	if (isBaroPresent && baroType == BMP388)
	{
		// ����MPU6500����Ҫ��ȡBMP388�ļĴ���
		mpu6500SetSlaveAddress(1, 0x80 | BMP3_I2C_ADDR_PRIM);  // ������ѹ��״̬�Ĵ���Ϊ1�Ŵӻ�
		mpu6500SetSlaveRegister(1, BMP3_SENS_STATUS_REG_ADDR); // �ӻ�1��Ҫ��ȡ�ļĴ���
		mpu6500SetSlaveDataLength(1, SENSORS_BARO_BUFF_LEN);   // ��ȡ7���ֽ�
		mpu6500SetSlaveDelayEnabled(1, true);
		mpu6500SetSlaveEnabled(1, true);
	}
	if (isBaroPresent && baroType == SPL06)
	{
		// ����MPU6500����Ҫ��ȡSPL06�ļĴ���
		mpu6500SetSlaveAddress(1, 0x80 | SPL06_I2C_ADDR);		// ������ѹ��״̬�Ĵ���Ϊ1�Ŵӻ�
		mpu6500SetSlaveRegister(1, SPL06_MODE_CFG_REG);			// �ӻ�1��Ҫ��ȡ�ļĴ���
		mpu6500SetSlaveDataLength(1, SENSORS_BARO_STATUS_LEN);	// ��ȡ1���ֽ�
		mpu6500SetSlaveDelayEnabled(1, true);
		mpu6500SetSlaveEnabled(1, true);

		mpu6500SetSlaveAddress(2, 0x80 | SPL06_I2C_ADDR);		// ������ѹ�����ݼĴ���Ϊ2�Ŵӻ�
		mpu6500SetSlaveRegister(2, SPL06_PRESSURE_MSB_REG);		// �ӻ�2��Ҫ��ȡ�ļĴ���
		mpu6500SetSlaveDataLength(2, SENSORS_BARO_DATA_LEN);	// ��ȡ6���ֽ�
		mpu6500SetSlaveDelayEnabled(2, true);
		mpu6500SetSlaveEnabled(2, true);
	}

	mpu6500SetI2CMasterModeEnabled(true); //ʹ��mpu6500����ģʽ
	mpu6500SetIntDataReadyEnabled(true);  //���ݾ����ж�ʹ��
}

/**
 * �����������ѭ�������������һ����ֵ�������������滻�ɵĵ�ֵ
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
 * �������������������ٶȼ���ƫ�ú���������
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
		//�ҵ�һ��accbias,ֹͣbuffer�������ݣ�ֹͣ����accbias,�ȴ���һ�ε�ָ��
		isreadytoprocessAccBias = false;
		//����accbias��ǰ���������ɣ�����Ӧ���ź�
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
		//acc���������ɷ��͸�������õ���biasֵ
		sendaccBiasprocess_ACK(6,accBiasOut->x,accBiasOut->y,accBiasOut->z);

		accBiasFound = true;
		processAccBias_stepnum = 0;
	}

	return accBiasFound;
}

// /**
//  * �������������������ٶ���������
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
 * �������ݷ���
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

/*������ѹ������*/
void processBarometerMeasurements(const u8 *buffer)
{
	static float temp;
	static float pressure;
	struct bmp3_uncomp_data uncomp_data = {0};
	struct bmp3_data comp_data = {0};
	if (baroType == BMP388)
	{
		// Check if there is a new data update
		if ((buffer[0] & 0x30)) /*ת�����*/
		{
			uncomp_data.pressure = (uint64_t)((((u32)(buffer[3])) << 16) | (((u32)(buffer[2])) << 8) | (u32)buffer[1]);
			uncomp_data.temperature = (int64_t)((((u32)(buffer[6])) << 16) | (((u32)(buffer[5])) << 8) | (u32)buffer[4]);
			compensate_data(BMP3_ALL, &uncomp_data, &comp_data, BMP388_DEV.calib_data);
			pressure = (float)comp_data.pressure / 100.0f;
			sensors.baro.temperature = (float)comp_data.temperature / 100.0f; /*��λ��*/

			pressureFilter(&pressure, &sensors.baro.pressure);
			sensors.baro.asl = PressureToAltitude(&sensors.baro.pressure) * 100; /*ת���ɺ��Σ���λ��cm*/
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
/*�������������*/
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
		magRaw.x = headingx; /*�����ϴ�����λ��*/
		magRaw.y = headingy;
		magRaw.z = headingz;
	}
}
/*������ټƺ�����������*/
void processAccGyroMeasurements(const uint8_t *buffer)
{
	/*ע�⴫������ȡ����(��ת270��x��y����)*/
	// �巽������config.h��
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

	accRaw.x = ax - accBias.x; /*�����ϴ�����λ��*/
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
		
		//processAccScale(ax, ay, az); /*����accScale*/
	}

	sensors.gyro.x = -(gx - gyroBias.x) * SENSORS_DEG_PER_LSB_CFG; /*��λ ��/s */
	sensors.gyro.y =  (gy - gyroBias.y) * SENSORS_DEG_PER_LSB_CFG;
	sensors.gyro.z =  (gz - gyroBias.z) * SENSORS_DEG_PER_LSB_CFG;
	applyAxis3fLpf(gyroLpf, &sensors.gyro);

	// sensors.acc.x = -(ax)*SENSORS_G_PER_LSB_CFG / accScale.x; /*��λ g(9.8m/s^2)*/
	// sensors.acc.y =  (ay)*SENSORS_G_PER_LSB_CFG / accScale.y;	/*�������ٶ���������accScale ������������ó�*/
	// sensors.acc.z =  (az)*SENSORS_G_PER_LSB_CFG / accScale.z;
	sensors.acc.x = -(accRaw.x) / accScale.x;		/*��λ g(9.8m/s^2)*/
	sensors.acc.y =   accRaw.y	/ accScale.y;		/*��λ g(9.8m/s^2)*/
	sensors.acc.z =   accRaw.z	/ accScale.z;		/*��λ g(9.8m/s^2)*/

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

/*����������*/
void sensorsTask(void *param)
{
//	float accraw_num[3] = {0, 0, 0};
	sensorsInit(); /*��������ʼ��*/
	vTaskDelay(150);
	//MPU9250����ΪIIC����ģʽ
	sensorsSetupSlaveRead(); /*���ô�������ģʽ��ȡ*/

	while (1)
	{
		if (xSemaphoreTake(sensorsDataReady, portMAX_DELAY) == pdTRUE)
		{
			/*ȷ�����ݳ���*/
			u8 dataLen = (u8)(SENSORS_MPU6500_BUFF_LEN +
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
				processBarometerMeasurements(&(buffer[isMagPresent ? SENSORS_MPU6500_BUFF_LEN + SENSORS_MAG_BUFF_LEN : SENSORS_MPU6500_BUFF_LEN]));
			}

			vTaskSuspendAll(); /*ȷ��ͬһʱ�̰����ݷ��������*/
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
static void applyAxis3fLpf(lpf2pData *data, Axis3f *in)
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
	processAccBias_stepnum = i;			//���õ�ǰ����accbias����һ����
	isreadytoprocessAccBias = true;		//���������accbias�����־λʹ��
	accBiasFound = false;				//���¿�ʼ�궨ʱ���ظ�accBiasFoundΪfalse״̬
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
