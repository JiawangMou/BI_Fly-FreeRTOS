#include "motors.h"
#include "config_param.h"
#include "delay.h"
#include "pm.h"
#include "sys.h"
#include "ina226.h"

/********************************************************************************
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * �����������
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2017/5/12
 * �汾��V1.3
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
 ********************************************************************************/

static bool      isInit         = false;
u32              motor_ratios[] = { 0, 0, 0, 0 };
static const u32 MOTORS[]       = { PWMF1, PWMF2, PWM_LEFT, PWM_RIGHT };

static u16 ratioToCCRx(u16 val) { return ((val) >> (16 - MOTORS_PWM_BITS) & ((1 << MOTORS_PWM_BITS) - 1)); }

void motorsInit(void) /*�����ʼ��*/
{
    GPIO_InitTypeDef        GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef       TIM_OCInitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD,
        ENABLE);                                                               //ʹ��PORTA PORTB PORTC PORTDʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE); // TIM3��TIM4ʱ��ʹ��

    // TIM8 ʱ��
#ifdef PC7_OUT_ENABLE
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
#endif
    TIM_DeInit(TIM8);


    TIM_DeInit(TIM4); //���³�ʼ��TIM4ΪĬ��״̬
    TIM_DeInit(TIM3); //���³�ʼ��TIM3ΪĬ��״̬

    // Servos AF
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM3);  // PC8 ����ΪTIM3 CH3	PWM_LEFT
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3);  // PA6 ����ΪTIM3 CH1	PWM_RIght
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3);  // PB1 ����ΪTIM3 CH4	PWM_MIDDLE

    // Motors AF
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM8);  // PC7 ����ΪTIM8 CH2	PWMR
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4); // PD12����ΪTIM4 CH1	PWMF1
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4); // PD13����ΪTIM4 CH2	PWMF2

    // GPIO Inits
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7 | GPIO_Pin_8; // PC7 8
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;            //���ù���
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;       //�ٶ�100MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;           //���츴�����
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;            //����
    GPIO_Init(GPIOC, &GPIO_InitStructure);                   //��ʼ��PC7 8

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; // PA6
    GPIO_Init(GPIOA, &GPIO_InitStructure);    //��ʼ��PA6

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; // PB1
    GPIO_Init(GPIOB, &GPIO_InitStructure);    //��ʼ��PB1

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13; // PD12 13
    GPIO_Init(GPIOD, &GPIO_InitStructure);                   //��ʼ��PD12 13

    // TIM Base Inits
    TIM_TimeBaseStructure.TIM_Period            = MOTORS_PWM_PERIOD;   //�Զ���װ��ֵ
    TIM_TimeBaseStructure.TIM_Prescaler         = MOTORS_PWM_PRESCALE; //��ʱ����Ƶ
    TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;  //���ϼ���ģʽ
    TIM_TimeBaseStructure.TIM_ClockDivision     = 0;                   //ʱ�ӷ�Ƶ
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;                   //�ظ���������
    TIM_TimeBaseInit(TIM_MOTOR, &TIM_TimeBaseStructure);                //��ʼ��TIM_MOTOR (TIM4)

    TIM_TimeBaseStructure.TIM_Prescaler         = MOTORS_2_PWM_PRESCALE; //��ʱ����Ƶ
    TIM_TimeBaseInit(TIM_MOTOR_2, &TIM_TimeBaseStructure);              //��ʼ��TIM_MOTOR_2 (TIM8)

    TIM_TimeBaseStructure.TIM_Period    = SERVOS_PWM_PERIOD;        //�Զ���װ��ֵ
    TIM_TimeBaseStructure.TIM_Prescaler = SERVOS_PWM_PRESCALE;      //��ʱ����Ƶ
    TIM_TimeBaseInit(TIM_SERVO, &TIM_TimeBaseStructure);            //��ʼ��TIM_SERVO (TIM3)

    // TIM OC Inits
    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;        // PWMģʽ1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //ʹ�����
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;   
    TIM_OCInitStructure.TIM_Pulse       = 0;                      // CCRx
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;    //�ߵ�ƽ��Ч
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;    //���иߵ�ƽ
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Set;

    TIM_OC1Init(TIM_MOTOR, &TIM_OCInitStructure); //��ʼ��TIM4 CH1����Ƚ�
    TIM_OC2Init(TIM_MOTOR, &TIM_OCInitStructure); //��ʼ��TIM4 CH2����Ƚ�
    TIM_OC2Init(TIM_MOTOR_2, &TIM_OCInitStructure); //��ʼ��TIM8 CH2����Ƚ�

    /* Automatic Output enable, Break, dead time and lock configuration*/
    TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
    TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
    TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
    TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1; 
    TIM_BDTRInitStructure.TIM_DeadTime = DEADTIME;
    TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
    TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
    TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;
    TIM_BDTRConfig(TIM_MOTOR_2, &TIM_BDTRInitStructure);

    TIM_OCInitStructure.TIM_Pulse = getservoinitpos_configParam(PWM_LEFT);   //�����λֵ
    TIM_OC3Init(TIM_SERVO, &TIM_OCInitStructure);                                 //��ʼ��TIM3 CH3����Ƚ�	PWM_LEFT
    TIM_OCInitStructure.TIM_Pulse = getservoinitpos_configParam(PWM_RIGHT);  //�����λֵ
    TIM_OC1Init(TIM_SERVO, &TIM_OCInitStructure);                                 //��ʼ��TIM3 CH1����Ƚ�	PWM_RIGHT
    TIM_OCInitStructure.TIM_Pulse = getservoinitpos_configParam(PWM_MIDDLE); //�����λֵ
    TIM_OC4Init(TIM_SERVO, &TIM_OCInitStructure);                                 //��ʼ��TIM3 CH4����Ƚ�	PWM_MIDDLE

    // TIM Preload Enable
    TIM_OC2PreloadConfig(TIM_MOTOR, TIM_OCPreload_Enable); //ʹ��TIM4��CCR2�ϵ�Ԥװ�ؼĴ���
    TIM_OC1PreloadConfig(TIM_MOTOR, TIM_OCPreload_Enable); //ʹ��TIM4��CCR1�ϵ�Ԥװ�ؼĴ���
    TIM_OC2PreloadConfig(TIM_MOTOR_2, TIM_OCPreload_Enable); //ʹ��TIM8��CCR2�ϵ�Ԥװ�ؼĴ���

    TIM_OC1PreloadConfig(TIM_SERVO, TIM_OCPreload_Enable); //ʹ��TIM3��CCR1�ϵ�Ԥװ�ؼĴ���
    TIM_OC3PreloadConfig(TIM_SERVO, TIM_OCPreload_Enable); //ʹ��TIM3��CCR3�ϵ�Ԥװ�ؼĴ���
    TIM_OC4PreloadConfig(TIM_SERVO, TIM_OCPreload_Enable); //ʹ��TIM3��CCR4�ϵ�Ԥװ�ؼĴ���

    TIM_ARRPreloadConfig(TIM_MOTOR, ENABLE); // TIM4	ARPEʹ��
    TIM_ARRPreloadConfig(TIM_SERVO, ENABLE); // TIM3	ARPEʹ��
    TIM_ARRPreloadConfig(TIM_MOTOR_2, ENABLE); // TIM8	ARPEʹ��

    TIM_Cmd(TIM_MOTOR, ENABLE); //ʹ��TIM4
    TIM_Cmd(TIM_SERVO, ENABLE); //ʹ��TIM3

#ifdef PC7_OUT_ENABLE
    TIM_Cmd(TIM_MOTOR_2, ENABLE); //ʹ��TIM8
    TIM_CtrlPWMOutputs(TIM_MOTOR_2, ENABLE);
#endif

    isInit = true;
}

/*�������*/
bool motorsTest(void)
{
    int i;

    for (i = 0; i < sizeof(MOTORS) / sizeof(*MOTORS); i++) {
        motorsSetRatio(MOTORS[i], MOTORS_TEST_RATIO);
        delay_xms(MOTORS_TEST_ON_TIME_MS);
        motorsSetRatio(MOTORS[i], 0);
        delay_xms(MOTORS_TEST_DELAY_TIME_MS);
    }

    return isInit;
}

extern bool isExitFlip;

/*���õ��PWMռ�ձ�*/
void motorsSetRatio(u32 id, u16 ithrust)
{
    if (isInit) {
        u16 ratio = ithrust;

#ifdef ENABLE_THRUST_BAT_COMPENSATED
        if (isExitFlip == true) /*500Hz*/
        {
            if (ithrust > 0) {
                float thrust         = ((float)(ithrust - MOTORS_MAXPWM) / (65536.0f - MOTORS_MAXPWM)) * 40.0f;
                float volts          = 0.063f * thrust + 1.68f;
                float supply_voltage = pmGetBatteryVoltage();
                float percentage     = volts / supply_voltage;
                percentage           = percentage > 1.0f ? 1.0f : percentage;
                ratio                = percentage * UINT16_MAX;
                motor_ratios[id]     = ratio;
            }
        }
#endif

#ifdef ENABLE_BATTERY_INFOR_GET
        if (ithrust > 0) {
            float thrust         = ((float)(ithrust - MOTORS_MAXPWM) / (65536.0f - MOTORS_MAXPWM)) * 40.0f;
            float volts          = 0.063f * thrust + 1.68f;
            float supply_voltage;
            UOC utemp;
            getVoltage(&utemp);
            supply_voltage = utemp.estiVal;
            float percentage     = volts / supply_voltage;
            percentage           = percentage > 1.0f ? 1.0f : percentage;
            ratio                = percentage * UINT16_MAX;
            motor_ratios[id]     = ratio;
        }
#endif
        switch (id) {
        case PWMPD12:
            TIM_SetCompare1(TIM_MOTOR, ratioToCCRx(ratio));
            break;
        case PWMPD13:
            TIM_SetCompare2(TIM_MOTOR, ratioToCCRx(ratio));
            break;
            // case 2: /*PWM_LEFT*/
            // 	TIM_SetCompare3(TIM3, ratioToCCRx(ratio));
            // 	break;
            // case 3: /*PWM_RIGHT*/
            // 	TIM_SetCompare1(TIM3, ratioToCCRx(ratio));
            // 	break;
            // case 4: /*PWM_MIDDLE
            // 	TIM_SetCompare4(TIM3, ratioToCCRx(ratio));
            // 	break;
        case PWMPC7:
            TIM_SetCompare2(TIM_MOTOR_2, ratioToCCRx(ratio));
            break;
        default:
            break;
        }
    }
}

void servoSetPWM(u8 id, u16 value)
{
    switch (id) {
    case PWM_LEFT:
        TIM_SetCompare3(TIM3, (uint32_t)value);
        break;
    case PWM_RIGHT:
        TIM_SetCompare1(TIM3, (uint32_t)value);
        break;
    case PWM_MIDDLE:
        TIM_SetCompare4(TIM3, (uint32_t)value);
        break;
    }
}

#ifdef BI_Fly_1
u32 servoPWMLimit(u16 value)
{
    u16 _temp = 0;
    if (value > SERVO_MAXPWM)
        _temp = SERVO_MAXPWM;
    else if (value < SERVO_MINPWM)
        _temp = SERVO_MINPWM;
    else
        _temp = value;

    return (u32)_temp;
}
#endif

#ifdef BI_Fly_2
u32 servoPWMLimit(u8 id, u16 value)
{
    u16 _temp      = 0;
    u16 HLimit     = SERVO_MAXPWM - getservoinitpos_configParam(id);
    u16 LLimit     = getservoinitpos_configParam(id) - SERVO_MINPWM;
    u16 servoRange = (HLimit > LLimit) ? LLimit : HLimit;
	
    if (value > getservoinitpos_configParam(id) + servoRange)
        _temp = getservoinitpos_configParam(id) + servoRange;
    else if (value < getservoinitpos_configParam(id) - servoRange)
        _temp = getservoinitpos_configParam(id) - servoRange;
    else
        _temp = value;

    return (u32)_temp;
}
#endif
