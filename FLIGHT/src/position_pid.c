#include "position_pid.h"
#include "commander.h"
#include "config_param.h"
#include "maths.h"
#include "pid.h"
#include "remoter_ctrl.h"
#include <math.h>

/********************************************************************************
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * λ��PID���ƴ���
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2017/5/12
 * �汾��V1.3
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
 *
 * �޸�˵��:
 * �汾V1.3 ˮƽ����PID����ϴ�������λ�û��������0.1��ϵ����
        ���ʻ��������0.15ϵ�����Ӷ�����PID�Ŀɵ��ԡ�
********************************************************************************/

#define THRUST_BASE (35000) /*��������ֵ*/

#define PIDVX_OUTPUT_LIMIT 120.0f  // ROLL�޷�	(��λ���0.15��ϵ��)
#define PIDVY_OUTPUT_LIMIT 120.0f  // PITCH�޷�	(��λ���0.15��ϵ��)
#define PIDVZ_OUTPUT_LIMIT (65500) /*PID VZ�޷�*/

#define PIDX_OUTPUT_LIMIT 1200.0f   // X���ٶ��޷�(��λcm/s ��0.1��ϵ��)
#define PIDY_OUTPUT_LIMIT 1200.0f   // Y���ٶ��޷�(��λcm/s ��0.1��ϵ��)
#define PIDZ_OUTPUT_LIMIT  100.0f	//Z���ٶ��޷�(��λcm/s)
//��ʱ�汾�����߱�Ϊ����PID,����޷�ֱ�ӵ��������
// #define PIDZ_OUTPUT_LIMIT 65500.0f // Z���ٶ��޷�(������)

static float thrustLpf        = THRUST_BASE; /*���ŵ�ͨ*/
static float thrustHover      = 0.f;
static bool  enterVelModeFlag = false;

PidObject pidVX;
PidObject pidVY;
PidObject pidVZ;

PidObject pidX;
PidObject pidY;
PidObject pidZ;

void positionControlInit(float velocityPidDt, float posPidDt)
{
    pidInit(&pidVX, 0, configParam.pidPos.vx, velocityPidDt);     /*vx PID��ʼ��*/
    pidInit(&pidVY, 0, configParam.pidPos.vy, velocityPidDt);     /*vy PID��ʼ��*/
    pidInit(&pidVZ, 0, configParam.pidPos.vz, velocityPidDt);     /*vz PID��ʼ��*/
    pidSetOutputLimit(&pidVX, configParam.pidPos.vx.outputLimit); /* ����޷� */
    pidSetOutputLimit(&pidVY, configParam.pidPos.vy.outputLimit); /* ����޷� */
    pidSetOutputLimit(&pidVZ, configParam.pidPos.vz.outputLimit); /* ����޷� */

    pidInit(&pidX, 0, configParam.pidPos.x, posPidDt);          /*x PID��ʼ��*/
    pidInit(&pidY, 0, configParam.pidPos.y, posPidDt);          /*y PID��ʼ��*/
    pidInit(&pidZ, 0, configParam.pidPos.z, posPidDt);          /*z PID��ʼ��*/
    pidSetOutputLimit(&pidX, configParam.pidPos.x.outputLimit); /* ����޷� */
    pidSetOutputLimit(&pidY, configParam.pidPos.y.outputLimit); /* ����޷� */
    pidSetOutputLimit(&pidZ, configParam.pidPos.z.outputLimit); /* ����޷� */
}

static void velocityController(float* thrust, attitude_t* attitude, setpoint_t* setpoint, const state_t* state)
{
    static u16 altholdCount = 0;

    // Roll and Pitch
    attitude->pitch = 0.15f * pidUpdate(&pidVX, setpoint->velocity.x - state->velocity.x);
    attitude->roll  = 0.15f * pidUpdate(&pidVY, setpoint->velocity.y - state->velocity.y);

    // Thrust
    float thrustRaw = pidUpdate(&pidVZ, setpoint->velocity.z - state->velocity.z);

    *thrust = constrainf(thrustRaw + THRUST_BASE, 1000, 65500); /*�����޷�*/

    //��ֹPID�������Ž���̫�죬�÷�����ͣ����Ӱ�춨��Ч�������ԶԵ��ڻ������ŵ����Ž���С�仯��Χ�����������ʹ���Ų�����35000
    // if (*thrust < THRUST_BASE)
    //     *thrust = *thrust / 8 + 35000;

    thrustLpf += (*thrust - thrustLpf) * 0.003f;

    if (getCommanderKeyFlight()) /*���߷���״̬*/
    {
        //TEST: �Ƴ�����ģʽ��ʱ����»�������
        // if (enterVelModeFlag && (setpoint->mode.z != modeVelocity)) {
        //     enterVelModeFlag       = false;
        //     configParam.thrustBase = thrustLpf;
        // }
        if (fabs(state->acc.z) < 35.f) {
            altholdCount++;
            if (altholdCount > 1000) {
                altholdCount = 0;
                if (fabs(configParam.thrustBase - thrustLpf) > 1000.f) /*���»�������ֵ*/
                    configParam.thrustBase = thrustLpf;
            }
        } else {
            altholdCount = 0;
        }
    } else if (getCommanderKeyland() == false) /*������ɣ���������*/
    {
        *thrust = 0;
    }
}

void positionController(float* thrust, attitude_t* attitude, setpoint_t* setpoint, const state_t* state, float dt)
{
    if (setpoint->mode.x == modeAbs || setpoint->mode.y == modeAbs) {
        setpoint->velocity.x = 0.1f * pidUpdate(&pidX, setpoint->position.x - state->position.x);
        setpoint->velocity.y = 0.1f * pidUpdate(&pidY, setpoint->position.y - state->position.y);
    }

    if (setpoint->mode.z == modeAbs) {
        setpoint->velocity.z = constrainf(0.1f * pidUpdate(&pidZ, setpoint->position.z - state->position.z), -PIDZ_OUTPUT_LIMIT, PIDZ_OUTPUT_LIMIT);
    }

    velocityController(thrust, attitude, setpoint, state);
}

/*��ȡ��������ֵ*/
float getAltholdThrust(void) { return thrustLpf; }

void positionResetAllPID(void)
{
    pidReset(&pidVX);
    pidReset(&pidVY);
    pidReset(&pidVZ);

    pidReset(&pidX);
    pidReset(&pidY);
    pidReset(&pidZ);
}

void positionPIDwriteToConfigParam(void)
{
    configParam.pidPos.vx.kp          = pidVX.kp;
    configParam.pidPos.vx.ki          = pidVX.ki;
    configParam.pidPos.vx.kd          = pidVX.kd;
    configParam.pidPos.vx.outputLimit = pidVX.outputLimit;

    configParam.pidPos.vy.kp          = pidVY.kp;
    configParam.pidPos.vy.ki          = pidVY.ki;
    configParam.pidPos.vy.kd          = pidVY.kd;
    configParam.pidPos.vy.outputLimit = pidVY.outputLimit;

    configParam.pidPos.vz.kp          = pidVZ.kp;
    configParam.pidPos.vz.ki          = pidVZ.ki;
    configParam.pidPos.vz.kd          = pidVZ.kd;
    configParam.pidPos.vz.outputLimit = pidVZ.outputLimit;

    configParam.pidPos.x.kp          = pidX.kp;
    configParam.pidPos.x.ki          = pidX.ki;
    configParam.pidPos.x.kd          = pidX.kd;
    configParam.pidPos.x.outputLimit = pidX.outputLimit;

    configParam.pidPos.y.kp          = pidY.kp;
    configParam.pidPos.y.ki          = pidY.ki;
    configParam.pidPos.y.kd          = pidY.kd;
    configParam.pidPos.y.outputLimit = pidY.outputLimit;

    configParam.pidPos.z.kp          = pidZ.kp;
    configParam.pidPos.z.ki          = pidZ.ki;
    configParam.pidPos.z.kd          = pidZ.kd;
    configParam.pidPos.z.outputLimit = pidZ.outputLimit;
}
