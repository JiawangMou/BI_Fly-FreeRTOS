#include "state_control.h"
#include "attitude_pid.h"
#include "config.h"
#include "config_param.h"
#include "maths.h"
#include "position_pid.h"
#include "stabilizer.h"
#include <math.h>

/********************************************************************************
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * 四轴姿态控制代码
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2017/5/12
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
 ********************************************************************************/

static float      actualThrust;
static attitude_t attitudeDesired;
static attitude_t rateDesired;

// remoter setpoint(roll,pitch) filter
static lpf2pData setpointFilter[2];

void stateControlInit(void)
{
    attitudeControlInit(RATE_PID_DT, ANGEL_PID_DT);        /*初始化姿态PID*/
    positionControlInit(VELOCITY_PID_DT, POSITION_PID_DT); /*初始化位置PID*/

    // Filter the setpoint
    lpf2pInit(&setpointFilter[0], ANGEL_PID_RATE, 20);
    lpf2pInit(&setpointFilter[1], ANGEL_PID_RATE, 20);
}

bool stateControlTest(void)
{
    bool pass = true;
    pass &= attitudeControlTest();
    return pass;
}

void stateControl(control_t* control, sensorData_t* sensors, state_t* state, setpoint_t* setpoint, const u32 tick)
{
    static u16 cnt = 0;

    if (RATE_DO_EXECUTE(POSITION_PID_RATE, tick)) {
        if (setpoint->mode.x != modeDisable || setpoint->mode.y != modeDisable || setpoint->mode.z != modeDisable) {
            positionController(&actualThrust, &attitudeDesired, setpoint, state, POSITION_PID_DT);
        }
    }

    //角度环（外环）
    if (RATE_DO_EXECUTE(ANGEL_PID_RATE, tick)) {
        if (setpoint->mode.z == modeDisable) {
            actualThrust = setpoint->thrust;
        }
        if (setpoint->mode.x == modeDisable || setpoint->mode.y == modeDisable) {
            attitudeDesired.roll  = setpoint->attitude.roll;
            attitudeDesired.pitch = setpoint->attitude.pitch;
        }

        if (control->flipDir == CENTER) {
            attitudeDesired.yaw += setpoint->attitude.yaw / ANGEL_PID_RATE; /*期望YAW 速率模式*/
            if (attitudeDesired.yaw > 180.0f)
                attitudeDesired.yaw -= 360.0f;
            if (attitudeDesired.yaw < -180.0f)
                attitudeDesired.yaw += 360.0f;
        }

        attitudeDesired.roll += configParam.trimR; //叠加微调值
        attitudeDesired.pitch += configParam.trimP;

        attitudeDesired.roll = lpf2pApply(&setpointFilter[0], attitudeDesired.roll);
        attitudeDesired.pitch = lpf2pApply(&setpointFilter[1], attitudeDesired.pitch);

        attitudeAnglePID(&state->attitude, &attitudeDesired, &rateDesired);
    }

    //角速度环（内环）
    if (RATE_DO_EXECUTE(RATE_PID_RATE, tick)) {
        if (setpoint->mode.roll == modeVelocity) {
            rateDesired.roll = setpoint->attitudeRate.roll;
            attitudeControllerResetRollAttitudePID();
        }
        if (setpoint->mode.pitch == modeVelocity) {
            rateDesired.pitch = setpoint->attitudeRate.pitch;
            attitudeControllerResetPitchAttitudePID();
        }
        extern u8 fstate;
        if (control->flipDir != CENTER && fstate == 4) /*空翻过程只使用内环PID*/
        {
            rateDesired.pitch = setpoint->attitude.pitch;
            rateDesired.roll  = setpoint->attitude.roll;
        }

        // 如果直接控制角速度，在此截断
        // rateDesired.pitch = setpoint -> attitude.pitch * 4;
        // rateDesired.roll = setpoint -> attitude.roll * 4;
        attitudeRatePID(&sensors->gyro, &rateDesired, control);

// #ifdef BI_Fly_2
        // control->yaw = setpoint->attitude.yaw * 100;
// #endif
    }
    control->thrust = constrainf(actualThrust, 0.0f, 55000.0f);
    // control->thrust = actualThrust;

    if (control->thrust < 5.f) {
        control->roll = 0;
        // control->pitch = 0;
        // control->yaw = 0;

        attitudeResetAllPID_TEST();
        // attitudeResetAllPID();	/*复位姿态PID*/
        // /*这里取消复位的原因是，让飞行器翅膀不拍动的时候，还能看到舵机的反应，从而确认PID计算结果是否正常，或者是接线是否有问题*/
        positionResetAllPID();                     /*复位位置PID*/
        attitudeDesired.yaw = state->attitude.yaw; /*复位计算的期望yaw值*/

        if (cnt++ > 1500) {
            cnt = 0;
            configParamGiveSemaphore();
        }
    } else {
        cnt = 0;
    }
}

void getRateDesired(attitude_t *get){
    *get = rateDesired;
}

void getAngleDesired(attitude_t *get){
    *get = attitudeDesired;
}
