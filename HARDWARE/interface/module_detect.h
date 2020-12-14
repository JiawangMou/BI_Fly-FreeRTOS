#ifndef __EXP_MODULE_DRIVER_H
#define __EXP_MODULE_DRIVER_H
#include "sys.h"
#include <stdbool.h>

enum expModuleID
{
	NO_MODULE,
	LED_RING,
	WIFI_CAMERA,
	OPTICAL_FLOW,
	MODULE1,
};


void expModuleDriverInit(void);
void expModuleDriverDmaIsr(void);
enum expModuleID getModuleDriverID(void);

#endif /* __EXP_MODULE_DRIVER_H */

