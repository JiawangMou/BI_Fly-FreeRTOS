#include "config.h"
#include "watchdog.h"
#include "debug_assert.h"

bool watchdogTest(void)
{
	bool wasNormalStart = true;

	if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST)) 
	{
		RCC_ClearFlag();
		wasNormalStart = false;
//		printf("The system resumed after watchdog timeout [WARNING]\n");
//		printAssertSnapshotData();
	}
	return wasNormalStart;
}


void watchdogInit(u16 xms)
{
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	IWDG_SetPrescaler(IWDG_Prescaler_32);

	/* 47000/32Hz => 1.47  1ms*/
	IWDG_SetReload((u16)(1.47*xms));

	watchdogReset();
	IWDG_Enable();
}
