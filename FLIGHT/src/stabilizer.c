#include "stabilizer.h"
#include "anomal_detec.h"
#include "commander.h"
#include "flip.h"
#include "maths.h"
#include "optical_flow.h"
#include "position_pid.h"
#include "power_control.h"
#include "sensfusion6.h"
#include "sensors.h"
#include "state_control.h"
#include "state_estimator.h"
#include "system.h"
#include "vl53lxx.h"

/*FreeRTOS���ͷ�ļ�*/
#include "FreeRTOS.h"
#include "task.h"

#include "stabilization_attitude.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * �������ȿ��ƴ���	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2017/5/12
 * �汾��V1.3
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/

static bool isInit;

static setpoint_t setpoint; /*����Ŀ��״̬*/
static sensorData_t sensorData; /*����������*/
static state_t state; /*������̬*/
static control_t control; /*������Ʋ���*/

static u16 velModeTimes = 0; /*����ģʽ����*/
static u16 absModeTimes = 0; /*����ֵģʽ����*/
static float setHeight = 0.f; /*�趨Ŀ��߶� ��λcm*/
static float baroLast = 0.f;
static float baroVelLpf = 0.f;

void stabilizerTask(void* param);

void stabilizerInit(void)
{
    if (isInit)
        return;

    stateControlInit(); /*��̬PID��ʼ��*/
    powerControlInit(); /*�����ʼ��*/

    isInit = true;
}

bool stabilizerTest(void)
{
    bool pass = true;

    pass &= stateControlTest();
    pass &= powerControlTest();

    return pass;
}

/*���ÿ��ٵ�������*/
void setFastAdjustPosParam(u16 velTimes, u16 absTimes, float height)
{
    if (velTimes != 0 && velModeTimes == 0) {
        baroLast = sensorData.baro.asl;
        baroVelLpf = 0.f;
        velModeTimes = velTimes;
    }
    if (absTimes != 0 && absModeTimes == 0) {
        setHeight = height;
        absModeTimes = absTimes;
    }
}

/*���ٵ����߶�*/
static void fastAdjustPosZ(void)
{
    if (velModeTimes > 0) {
        velModeTimes--;
        estRstHeight(); /*��λ����߶�*/

        float baroVel = (sensorData.baro.asl - baroLast) / 0.004f; /*250Hz*/
        baroLast = sensorData.baro.asl;
        baroVelLpf += (baroVel - baroVelLpf) * 0.35f;

        setpoint.mode.z = modeVelocity;
        state.velocity.z = baroVelLpf; /*��ѹ���ں�*/
        setpoint.velocity.z = -1.0f * baroVelLpf;

        if (velModeTimes == 0) {
            if (getVl53l1xstate())
                setHeight = getFusedHeight();
            else
                setHeight = state.position.z;
        }
    } else if (absModeTimes > 0) {
        absModeTimes--;
        estRstAll(); /*��λ����*/
        setpoint.mode.z = modeAbs;
        setpoint.position.z = setHeight;
    }
}

void stabilizerTask(void* param)
{
    u32 tick = 0;
    u32 lastWakeTime = getSysTickCnt();

    //	ledseqRun(SYS_LED, seq_alive);

    while (!sensorsAreCalibrated()) {
        vTaskDelayUntil(&lastWakeTime, MAIN_LOOP_DT);
    }

    atkp_t send_debug;

    // Init Paparazzi Control
    stabilization_attitude_init();
    bool on_flight = false;

    while (1) {
        vTaskDelayUntil(&lastWakeTime, MAIN_LOOP_DT); /*1ms������ʱ*/

        // ��ȡ6�����ѹ���ݣ�500Hz��
        if (RATE_DO_EXECUTE(RATE_500_HZ, tick)) {
            sensorsAcquire(&sensorData, tick); /*��ȡ6�����ѹ����*/
        }



        //��Ԫ����ŷ���Ǽ��㣨250Hz��
        if (RATE_DO_EXECUTE(ATTITUDE_ESTIMAT_RATE, tick)) {
            // sensorsAcquire(&sensorData, tick); /*��ȡ6�����ѹ����*/
            imuUpdate(sensorData.acc, sensorData.gyro, &state, ATTITUDE_ESTIMAT_DT);
            send_debug.msgID = 0xF1;
            send_debug.dataLen = 13;
            send_debug.data[0] = 4;

            u32 timestamp = getCurrentUs();
            send_debug.data[1] = timestamp >> 24;
            send_debug.data[2] = timestamp >> 16;
            send_debug.data[3] = timestamp >> 8;
            send_debug.data[4] = timestamp;

            float q0, q1, q2, q3;
            getStateQuanternion(&q0, &q1, &q2, &q3);
            s16 tmp = q0 * 10000;
            send_debug.data[5] = tmp >> 8;
            send_debug.data[6] = tmp;
            tmp = q1 * 10000;;
            send_debug.data[7] = tmp >> 8;
            send_debug.data[8] = tmp;
            tmp = q2 * 10000;
            send_debug.data[9] = tmp >> 8;
            send_debug.data[10] = tmp;
            tmp = q3 * 10000;
            send_debug.data[11] = tmp >> 8;
            send_debug.data[12] = tmp;
            usblinkSendPacket(&send_debug);
        }

        //λ��Ԥ�����㣨250Hz��
        if (RATE_DO_EXECUTE(POSITION_ESTIMAT_RATE, tick)) {
            positionEstimate(&sensorData, &state, POSITION_ESTIMAT_DT);
            send_debug.msgID = 0xF1;
            send_debug.dataLen = 9;
            send_debug.data[0] = 5;

            u32 timestamp = getCurrentUs();
            send_debug.data[1] = timestamp >> 24;
            send_debug.data[2] = timestamp >> 16;
            send_debug.data[3] = timestamp >> 8;
            send_debug.data[4] = timestamp;

            s16 tmp = state.position.z * 10;
            send_debug.data[5] = tmp >> 8;
            send_debug.data[6] = tmp;
            tmp = state.velocity.z * 10;
            send_debug.data[7] = tmp >> 8;
            send_debug.data[8] = tmp;
            usblinkSendPacket(&send_debug);
        }

        //Ŀ����̬�ͷ���ģʽ�趨��100Hz��
        // if (RATE_DO_EXECUTE(RATE_100_HZ, tick) && getIsCalibrated() == true) {
        //     commanderGetSetpoint(&setpoint, &state); /*Ŀ�����ݺͷ���ģʽ�趨*/
        // }

        // if (RATE_DO_EXECUTE(RATE_250_HZ, tick)) {
        //     fastAdjustPosZ(); /*���ٵ����߶�*/
        // }
        // //
        // /*��ȡ��������(100Hz)*/
        // if (RATE_DO_EXECUTE(RATE_100_HZ, tick)) {
        //     getOpFlowData(&state, 0.01f);
        // }

        // /*�������(500Hz) �Ƕ���ģʽ*/
        // if (RATE_DO_EXECUTE(RATE_500_HZ, tick) && (getCommanderCtrlMode() != 0x03)) {
        //     flyerFlipCheck(&setpoint, &control, &state);
        // }

        // /*�쳣���*/
        // anomalDetec(&sensorData, &state, &control);

        // /*PID����*/
        // if (RATE_DO_EXECUTE(RATE_500_HZ, tick)){

        //     control.thrust = setpoint.thrust;
        //     if(control.thrust > 5.f && !on_flight){
        //         stabilization_attitude_enter(RadOfDeg(state.attitude.yaw));
        //         on_flight = true;
        //     }
        //     if(control.thrust <= 5.f && on_flight){
        //         stabilization_attitude_quit();
        //         on_flight = false;
        //     }
        //     stabilization_attitude_run(&control, &sensorData, &state, &setpoint);
        //     if(!on_flight){
        //         control.roll = 0;
        //     }
        // }
        // // stateControl(&control, &sensorData, &state, &setpoint, tick);

        // //���Ƶ�������500Hz��
        // if (RATE_DO_EXECUTE(RATE_500_HZ, tick)) {
        //     motorControl(&control);
        // }

        tick++;
    }
}

void getAttitudeData(attitude_t* get)
{
    get->pitch = state.attitude.pitch;
    get->roll = state.attitude.roll;
    get->yaw = state.attitude.yaw;
    get->timestamp = state.attitude.timestamp;
}

float getBaroData(void)
{
    return sensorData.baro.asl;
}

void getSensorData(sensorData_t* get)
{
    *get = sensorData;
}

void getStateData(Axis3f* acc, Axis3f* vel, Axis3f* pos)
{
    acc->x = 1.0f * state.acc.x;
    acc->y = 1.0f * state.acc.y;
    acc->z = 1.0f * state.acc.z;
    vel->x = 1.0f * state.velocity.x;
    vel->y = 1.0f * state.velocity.y;
    vel->z = 1.0f * state.velocity.z;
    pos->x = 1.0f * state.position.x;
    pos->y = 1.0f * state.position.y;
    pos->z = 1.0f * state.position.z;
}

control_t getControlData(void)
{
    return control;
}

mode_e getZmode(void)
{
    return setpoint.mode.z;
}

