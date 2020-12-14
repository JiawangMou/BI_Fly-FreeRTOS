#ifndef __USBLINK_H
#define __USBLINK_H
#include <stdbool.h>
#include "atkp.h"

void usblinkInit(void);
bool usblinkSendPacket(const atkp_t *p);
int usblinkGetFreeTxQueuePackets(void);
void usblinkRxTask(void *param);
void usblinkTxTask(void *param);


#endif /*usblink.h*/

