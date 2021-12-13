#ifndef __VL53LXX_I2C_HW_H
#define __VL53LXX_I2C_HW_H
#include "sys.h" 
#include "stdbool.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * VL53 IIC��������	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2018/5/2
 * �汾��V1.3
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
 *
 * �޸�˵��:
 * �汾V1.3 ���Ӷ�vl53l1x��IIC������
********************************************************************************/



//VL53���в�������
void vl53_HWIIC_Init(void);		/*��ʼ��VL53��IIC DMA ����GPIO*/				 
void vl53l1Read(u8 devaddr,u16 addr,u8 len,u8 *rbuf);   /*������ȡ����ֽ�*/
void vl53l1Write(u8 devaddr,u16 addr,u8 len,u8 *wbuf);	/*����д�����ֽ�*/



#endif 


