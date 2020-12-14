#ifndef __VL53LXX_I2C_H
#define __VL53LXX_I2C_H
#include "sys.h" 
#include "stdbool.h"

/*IO方向设置*/
#define SDA_IN()  {GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=0<<9*2;}	//PB4输入模式
#define SDA_OUT() {GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=1<<9*2;} //PB4输出模式
/*IO操作函数*/	 
#define VL53_SCL    PBout(8) 	//SCL
#define VL53_SDA    PBout(9) 	//SDA	 
#define READ_SDA	PBin(9)  	//输入SDA 


//VL53所有操作函数
void vl53IICInit(void);			/*初始化VL53的IO口*/				 
u8 vl53IICReadByte(u8 devaddr,u8 addr, u8* data);		/*读一字节*/
void vl53IICWriteByte(u8 devaddr,u8 addr,u8 data);		/*写一字节*/
void vl53IICRead(u8 devaddr,u8 addr,u8 len,u8 *rbuf);	/*连续读取多个字节*/
void vl53IICWrite(u8 devaddr,u8 addr,u8 len,u8 *wbuf);	/*连续写入多个字节*/
bool vl53IICWriteBit(u8 devaddr,u8 addr, u8 bitNum, u8 data);	/*iic 写入某个位*/

void vl53l1Read(u8 devaddr,u16 addr,u8 len,u8 *rbuf);	/*连续读取多个字节*/
void vl53l1Write(u8 devaddr,u16 addr,u8 len,u8 *wbuf);	/*连续写入多个字节*/
	
#endif 


