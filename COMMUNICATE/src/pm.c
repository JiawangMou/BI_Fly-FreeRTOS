#include "pm.h"
#include "config.h"
#include "sys.h"
#include "system.h"
#include <stdbool.h>
#include <string.h>
// #include "led.h"
// #include "ledseq.h"
#include "commander.h"
#include "radiolink.h"
#include "remoter_ctrl.h"
#include "sensfusion6.h"
#include "stabilizer.h"

/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#if defined(__CC_ARM)
#pragma anon_unions
#endif

typedef __packed struct _PmSyslinkInfo {
    __packed union {
        u8 flags;
        __packed struct
        {
            u8 pgood : 1;
            u8 chg : 1;
            u8 unused : 6;
        };
    };
    float vBat;

} PmSyslinkInfo;

static float batteryVoltage;
static float batteryVoltageMin = 6.0;
static float batteryVoltageMax = 0.0;

static bool isInit;
static bool isLowpower;
static PMStates pmState;
static PmSyslinkInfo pmSyslinkInfo;
static u32 batteryLowTimeStamp;

static void pmSetBatteryVoltage(float voltage);

void pmInit(void)
{
    if (isInit)
        return;

    pmSyslinkInfo.vBat = 3.7f;
    pmSetBatteryVoltage(pmSyslinkInfo.vBat);

    isInit = true;
}

bool pmTest(void)
{
    return isInit;
}

static void pmSetBatteryVoltage(float voltage) /*设置电池电压最大最小值*/
{
    batteryVoltage = voltage - 0.3;

    if (batteryVoltageMax < voltage) {
        batteryVoltageMax = voltage;
    }
    if (batteryVoltageMin > voltage) {
        batteryVoltageMin = voltage;
    }
}

float pmGetBatteryVoltage(void)
{
    return batteryVoltage;
}

void pmSyslinkUpdate(atkp_t* slp)
{
    memcpy(&pmSyslinkInfo, &slp->data[0], sizeof(pmSyslinkInfo));
    pmSetBatteryVoltage(pmSyslinkInfo.vBat);
}

PMStates pmUpdateState() /* 更新电源状态 */
{
    PMStates state;
    bool isCharging = pmSyslinkInfo.chg;
    bool isPgood = pmSyslinkInfo.pgood;
    u32 batteryLowTime;

    batteryLowTime = getSysTickCnt() - batteryLowTimeStamp;

    if (isPgood && !isCharging) {
        state = charged;
    } else if (isPgood && isCharging) {
        state = charging;
    } else if (!isPgood && !isCharging && (batteryLowTime > PM_BAT_LOW_TIMEOUT)) {
        state = lowPower;
    } else {
        state = battery;
    }

    return state;
}

void pmTask(void* param) /* 电源管理任务 */
{
    PMStates pmStateOld = battery;
    u32 tickCount;

    tickCount = getSysTickCnt();
    batteryLowTimeStamp = tickCount;

    vTaskDelay(500);

    while (1) {
        vTaskDelay(100);
        tickCount = getSysTickCnt();

        if (pmGetBatteryVoltage() > PM_BAT_LOW_VOLTAGE) {
            batteryLowTimeStamp = tickCount;
        }

        pmState = pmUpdateState();

        if (pmState != pmStateOld) {
            switch (pmState) /*电源状态切换*/
            {
            // case charged:
            // 	// ledseqStop(CHG_LED, seq_charging);
            // 	// ledseqRun(CHG_LED, seq_charged);
            // 	break;
            case charging:
                isLowpower = false;
                // ledseqStop(LOWBAT_LED, seq_lowbat);
                // if(getIsCalibrated())
                // ledseqRun(SYS_LED, seq_calibrated);
                // else
                // ledseqRun(SYS_LED, seq_alive);
                // ledseqStop(CHG_LED, seq_charged);
                // ledseqRun(CHG_LED, seq_charging);
                break;
            case lowPower:
                isLowpower = true;
                // ledseqStop(CHG_LED, seq_charging);
                // ledseqStop(SYS_LED, seq_alive);
                // ledseqStop(SYS_LED, seq_calibrated);
                // ledseqRun(LOWBAT_LED, seq_lowbat);
                break;
            // case battery:
            // 	// ledseqStop(CHG_LED, seq_charging);
            // 	// ledseqRun(CHG_LED, seq_charged);
            // 	break;
            default:
                break;
            }
            pmStateOld = pmState;
        }
    }
}

bool getIsLowpower(void)
{
    return isLowpower;
}
