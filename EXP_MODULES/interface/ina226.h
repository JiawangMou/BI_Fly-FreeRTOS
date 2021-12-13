#ifndef __INA226_H
#define __INA226_H
#include "sys.h"
#include "vl53lxx_i2c.h"
#include "vl53l1_api.h"
#include "math.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#define RATE_5_HZ		5
#define RATE_10_HZ		10
#define RATE_20_HZ		20
#define RATE_25_HZ		25
#define RATE_50_HZ		50
#define RATE_100_HZ		100
#define RATE_200_HZ 	200
#define RATE_250_HZ 	250
#define RATE_500_HZ 	500
#define RATE_1000_HZ 	1000

#define SOC_ESTIMAT_RATE	RATE_200_HZ
#define RATE_DO_EXECUTE(RATE_HZ, TICK) ((TICK % (MAIN_LOOP_RATE / RATE_HZ)) == 0)
#define INA_MAIN_LOOP_RATE 			RATE_200_HZ
#define INA_MAIN_LOOP_DT			(u32)(1000/INA_MAIN_LOOP_RATE)	/*��λms*/

#define 	CFG_REG	 		0x00		//
#define 	SV_REG 			0x01		//分流电压＄1�7 此处分流电阻丄1�7 0.1欄1�7
#define 	BV_REG 			0x02		//总线电压
#define 	PWR_REG 		0x03		//电源功率
#define 	CUR_REG 		0x04		//电流
#define 	CAL_REG 		0x05		//校准，设定满量程范围以及电流和功率测数的 
#define 	ONFF_REG 		0x06		//屏蔽 使能 警报配置和转换准备就组1�7
#define 	AL_REG 			0x07		//包含与所选警报功能相比较的限定��1�7
#define 	INA226_GET_ADDR 0XFF		//包含唯一的芯片标识号
#define   	INA226_ADDR1	0x40 

//#define   	INA226_GETALADDR	0x14 
typedef struct
{
	float voltageVal;			//mV
	float Shunt_voltage;		//uV
	float Shunt_Current;		//mA，分路电电流
	float powerVal;
}INA226;


void ina226Task(void *arg);
void INA226_Init(void);
//void INA226_SetRegPointer(u8 addr,u8 reg);
//void INA226_SendData(u8 addr,u8 reg,u16 data);

u16 INA226_ReadData(u8 addr);
//u8	INA226_AlertAddr(void);
u16 INA226_Get_ID(u8 addr);				//获取 id
u16 INA226_GetVoltage( u8 addr);		//获取总线电压
u16 INA226_GetShunt_Current(u8 addr);	//获取分流电流
u16 INA226_GetShuntVoltage(u8 addr);	//分流电压
u16 INA226_Get_Power(u8 addr);			//获取功率
u16 INA226_Get_CFG_REG(u8 addr);

u16 INA226_GET_CAL_REG(u8 addr);
void GetVoltage(float *Voltage);		
void Get_Shunt_voltage(float *Current);
void Get_Shunt_Current(float *Current);
//void Get_Power(float *Current);
void GetPower(void);//W
float findSOC(float U);
float *findCoe(float S);
void getPower(float* p);
void getSOC(float* soc);
// void getvoltage(float* v);
// void getcurrent(float* c);
extern INA226 INA226_data;
extern uint8_t ina226config[4];
#endif

