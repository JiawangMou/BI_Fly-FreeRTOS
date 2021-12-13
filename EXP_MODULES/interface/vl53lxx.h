#ifndef __VL53LXX_H
#define __VL53LXX_H

#include "vl53l0x.h"
#include "vl53l1x.h"

#include "stabilizer_types.h"
#include "module_mgt.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly	
 * vl53lxxӦ�ô���, ����vl53l0x��vl53l1x
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2018/5/2
 * �汾��V1.0
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/


extern u16 vl53lxxId;	/*vl53оƬID*/
extern bool isEnableVl53lxx;
extern zRange_t vl53lxx;


void vl53lxxInit(void);
bool vl53lxxReadRange(zRange_t* zrange);
void setVl53lxxState(u8 enable);
void vl53l1xTask(void* arg);
bool getVl53l1xstate(void);
u16 getVl53l1xxrangecompensated(void);
void getLaserData(int16_t* laserRaw, float* laserComp);

void getvoltage(float* v);
void getcurrent(float* c);

#endif /* __VL53LXX_H */

