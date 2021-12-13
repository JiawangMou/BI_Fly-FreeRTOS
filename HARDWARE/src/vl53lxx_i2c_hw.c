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
  *Others:  //��������˵��
  *Function List:  //��Ҫ�����б�ÿ����¼Ӧ���������������ܼ�Ҫ˵��
     1.��������
     2.��������
  *History:  //�޸���ʷ��¼�б�ÿ���޸ļ�¼Ӧ�����޸����ڡ��޸��߼��޸����ݼ��
     1.Date:2021.12.07
       Author:Jiawang Mou
       Modification: read or write vel53lxx using hardware I2C
     2.��������
**********************************************************************************/



//��ʼ��iic
void vl53_HWIIC_Init(void)
{	
	i2cdevInit(I2C1_DEV);  //I2C DMA init
}

void vl53l1Read(u8 devaddr,u16 addr,u8 len,u8 *rbuf)
{
	i2cdevRead16(I2C1_DEV,devaddr,addr,len,rbuf);
}

//����д����ֽ�
//addr:��ʼ��ַ
//wbuf:д���ݻ���
//len:���ݵĳ���
void vl53l1Write(u8 devaddr,u16 addr,u8 len,u8 *wbuf)
{
	i2cdevWrite16(I2C1_DEV, devaddr, addr, len,wbuf);
}










