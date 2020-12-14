#ifndef __UART1_H
#define __UART1_H
#include "sys.h"
#include <stdbool.h>

void uart1Init(u32 baudrate);	/*串口1初始化*/
bool uart1Test(void);			/*串口1测试*/
bool uart1GetDataWithTimout(u8 *c);	/*阻塞式接收一个字符*/		
void uart1SendData(uint32_t size, u8* data);	/*发送原始数据*/
int uart1Putchar(int ch);
void uart1Getchar(char * ch);

#endif /* __UART1_H */
