#ifndef __RADIO_H
#define __RADIO_H
#include <stdint.h>
#include <stdbool.h>
#include "atkp.h"

void radiolinkInit(void);
void radiolinkTask(void *param);
bool radiolinkSendPacket(const atkp_t *p);
bool radiolinkSendPacketBlocking(const atkp_t *p);
int radiolinkGetFreeTxQueuePackets(void);

#endif /*__RADIO_H */

