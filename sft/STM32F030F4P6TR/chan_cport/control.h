
#ifndef _CONTROL_H_
#define _CONTROL_H_


#define PWM_OUT TIM14->CCR1

#include "stm32f0xx.h"

void init_servo(uint8_t mode);
void servo_operation();
int16_t muls1616(int16_t i16, uint16_t u8_8);



#endif
