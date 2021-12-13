#include "vl53lxx_i2c_hw.h"
#include "i2cdev.h"
#include "delay.h"
#include "stdbool.h"	

/*********************************************************************************
  *Copyright(C),
  *FileName:  vl53lxx_i2c_hw.c
  *Author:  Jiawang Mou
  *Version:  V1.0
  *Date:  2021.12.07
  *Description: 
  *Others:  //其他内容说明
  *Function List:  //主要函数列表，每条记录应包含函数名及功能简要说明
     1.…………
     2.…………
  *History:  //修改历史记录列表，每条修改记录应包含修改日期、修改者及修改内容简介
     1.Date:2021.12.07
       Author:Jiawang Mou
       Modification: read or write vel53lxx using hardware I2C
     2.…………
**********************************************************************************/



//初始化iic
void vl53_HWIIC_Init(void)
{	
	i2cdevInit(I2C1_DEV);  //I2C DMA init
}

void vl53l1Read(u8 devaddr,u16 addr,u8 len,u8 *rbuf)
{
	i2cdevRead16(I2C1_DEV,devaddr,addr,len,rbuf);
}

//连续写多个字节
//addr:起始地址
//wbuf:写数据缓存
//len:数据的长度
void vl53l1Write(u8 devaddr,u16 addr,u8 len,u8 *wbuf)
{
	i2cdevWrite16(I2C1_DEV, devaddr, addr, len,wbuf);
}










