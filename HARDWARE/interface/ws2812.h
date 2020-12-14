#ifndef __WS2812_H__
#define __WS2812_H__
#include <stdbool.h>
#include "sys.h"

void ws2812Init(void);
void setHeadlightsOn(bool state);
void ws2812PowerControl(bool state);
void ws2812Send(uint8_t (*color)[3], uint16_t len);
void ws2812DmaIsr(void);

#endif /*__WS2812_H__*/
