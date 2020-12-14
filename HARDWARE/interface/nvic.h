#ifndef __NVIC_H
#define __NVIC_H
#include "sys.h"

void nvicInit(void);
u32 getSysTickCnt(void);	

#endif /* __NVIC_H */
