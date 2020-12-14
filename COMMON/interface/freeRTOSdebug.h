
#ifndef DEBUG_H_
#define DEBUG_H_

/**
 * Initializes peripherals used for creating trace
 * information
 */
void debugInitTrace(void);

/**
 * Sends trace information using the UART1. This function is
 * used with the freertos traceTASK_SWITCHED_IN() macro.
 * @param Task number currently running
 */
void debugSendTraceInfo(unsigned int taskNbr);

#endif /* DEBUG_H_ */
