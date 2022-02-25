#ifndef __VL53LXX_I2C_HW_H
#define __VL53LXX_I2C_HW_H
#include "sys.h" 
#include "stdbool.h"

/********************************************************************************	 
 * 本程序只供学习使用，未经作者许可，不得用于其它任何用途
 * ALIENTEK MiniFly
 * VL53 IIC驱动代码	
 * 正点原子@ALIENTEK
 * 技术论坛:www.openedv.com
 * 创建日期:2018/5/2
 * 版本：V1.3
 * 版权所有，盗版必究。
 * Copyright(C) 广州市星翼电子科技有限公司 2014-2024
 * All rights reserved
 *
 * 修改说明:
 * 版本V1.3 增加对vl53l1x的IIC驱动。
********************************************************************************/



//VL53所有操作函数
void vl53_HWIIC_Init(void);		/*初始化VL53的IIC DMA 还有GPIO*/				 
void vl53l1Read(u8 devaddr,u16 addr,u8 len,u8 *rbuf);   /*连续读取多个字节*/
void vl53l1Write(u8 devaddr,u16 addr,u8 len,u8 *wbuf);	/*连续写入多个字节*/



#endif 


