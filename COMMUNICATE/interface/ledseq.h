#ifndef __LEDSEQ_H
#define __LEDSEQ_H

#include <stdint.h>
#include <stdbool.h>
#include <led.h>

/********************************
*led序列动作: 
*延时、停止、周期循环
*********************************/
#define LEDSEQ_WAITMS(X) (X)
#define LEDSEQ_STOP      -1
#define LEDSEQ_LOOP      -2

typedef struct 
{
	bool value;
	int action;
} ledseq_t;

extern const ledseq_t seq_calibrated[];
extern const ledseq_t seq_lowbat[];
extern const ledseq_t seq_alive[];
extern const ledseq_t seq_linkup[];
extern const ledseq_t seq_charged[];
extern const ledseq_t seq_charging[];


void ledseqInit(void);
bool ledseqTest(void);
void ledseqEnable(bool enable);
void ledseqRun(led_e led, const ledseq_t * sequence);
void ledseqStop(led_e led, const ledseq_t * sequence);
void ledseqSetTimes(ledseq_t *sequence, s32 onTime, s32 offTime);


#endif
