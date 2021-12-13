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
#define INA_MAIN_LOOP_DT			(u32)(1000/INA_MAIN_LOOP_RATE)	/*单位ms*/

#define 	CFG_REG	 		0x00		//
#define 	SV_REG 			0x01		//鍒嗘祦鐢靛帇锛� 姝ゅ鍒嗘祦鐢甸樆涓� 0.1娆�
#define 	BV_REG 			0x02		//鎬荤嚎鐢靛帇
#define 	PWR_REG 		0x03		//鐢垫簮鍔熺巼
#define 	CUR_REG 		0x04		//鐢垫祦
#define 	CAL_REG 		0x05		//鏍″噯锛岃瀹氭弧閲忕▼鑼冨洿浠ュ強鐢垫祦鍜屽姛鐜囨祴鏁扮殑 
#define 	ONFF_REG 		0x06		//灞忚斀 浣胯兘 璀︽姤閰嶇疆鍜岃浆鎹㈠噯澶囧氨缁�
#define 	AL_REG 			0x07		//鍖呭惈涓庢墍閫夎鎶ュ姛鑳界浉姣旇緝鐨勯檺瀹氬€�
#define 	INA226_GET_ADDR 0XFF		//鍖呭惈鍞竴鐨勮姱鐗囨爣璇嗗彿
#define   	INA226_ADDR1	0x40 

//#define   	INA226_GETALADDR	0x14 
typedef struct
{
	float voltageVal;			//mV
	float Shunt_voltage;		//uV
	float Shunt_Current;		//mA锛屽垎璺數鐢垫祦
	float powerVal;
}INA226;


void ina226Task(void *arg);
void INA226_Init(void);
//void INA226_SetRegPointer(u8 addr,u8 reg);
//void INA226_SendData(u8 addr,u8 reg,u16 data);

u16 INA226_ReadData(u8 addr);
//u8	INA226_AlertAddr(void);
u16 INA226_Get_ID(u8 addr);				//鑾峰彇 id
u16 INA226_GetVoltage( u8 addr);		//鑾峰彇鎬荤嚎鐢靛帇
u16 INA226_GetShunt_Current(u8 addr);	//鑾峰彇鍒嗘祦鐢垫祦
u16 INA226_GetShuntVoltage(u8 addr);	//鍒嗘祦鐢靛帇
u16 INA226_Get_Power(u8 addr);			//鑾峰彇鍔熺巼
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

