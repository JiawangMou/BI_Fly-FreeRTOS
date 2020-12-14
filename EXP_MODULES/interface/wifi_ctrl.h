#ifndef WIFI_CONTROL_H
#define WIFI_CONTROL_H

#include "config.h"
#include "stabilizer_types.h"
#include "module_mgt.h"

#define WIFI_POWER_ENABLE	PBout(0)

typedef struct
{
	u8 keyFlight 	: 1;	/*bit0 一键起飞*/
	u8 keyLand 		: 1;	/*bit1 一键降落*/
	u8 emerStop 	: 1;	/*bit2 紧急停机*/
	u8 flipOne 		: 1;	/*bit3 固定方向翻滚*/
	u8 flightMode 	: 1;	/*bit4 飞行模式 1=无头 0=有头*/
	u8 flipFour 	: 1;	/*bit5 4D翻滚*/
	u8 ledControl 	: 1;	/*bit6 灯光控制*/
	u8 gyroCalib 	: 1;	/*bit7 陀螺校准*/	
}wifiCmd_t;


void wifiModuleInit(void);			/*wifi模块初始化*/
void wifiPowerControl(bool state);	/*wifi电源控制*/
void wifiLinkTask(void *param);


#endif /* WIFI_CONTROL_H */

