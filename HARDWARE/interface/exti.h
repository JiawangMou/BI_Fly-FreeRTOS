#ifndef __EXTI_H__
#define __EXTI_H__
#include <stdbool.h>

void extiInit(void);
bool extiTest(void);

void EXTI0_Callback(void);
void EXTI1_Callback(void);
void EXTI2_Callback(void);
void EXTI3_Callback(void);
void EXTI4_Callback(void);
void EXTI5_Callback(void);
void EXTI6_Callback(void);
void EXTI7_Callback(void);
void EXTI8_Callback(void);
void EXTI9_Callback(void);
void EXTI10_Callback(void);
void EXTI11_Callback(void);
void EXTI12_Callback(void);
void EXTI13_Callback(void);
void EXTI14_Callback(void);
void EXTI15_Callback(void);

#endif /* __EXTI_H__ */

