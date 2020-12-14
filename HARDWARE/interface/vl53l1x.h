#ifndef __VL53L1X_H
#define __VL53L1X_H

#include "vl53l1_platform.h"

#include "stabilizer_types.h"
#include "module_mgt.h"

#define VL53L1X_MAX_RANGE			410		//410cm

#define VL53L1X_ADDR 				0x52
#define VL53L1X_DEFAULT_ADDRESS 	0x29	//0b0101001

#define VL53L1X_ID			0xEACC

extern VL53L1_Dev_t	dev;	/*vl53l1x 设备*/
int vl53l1xSetParam(void);	/*设置vl53l1x 参数*/

#endif /* __VL53L1X_H */

