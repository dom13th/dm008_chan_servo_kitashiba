#ifndef D13CORE_H
#define D13CORE_H

#include "stm32f0xx.h"

#define MKS_PER_STICK 						(1000)											//SysTick каждые мкс
#define MSEC_TO_MKS(x) 						(x*1000)										//миллисекунды в микросекунды

#define GlobalSYSTickCounterRTRIGQ 			(ddwSTIC_CNTR!=ddwSTIC_CNTR_OLD)


typedef struct 
{
      unsigned char in;
      unsigned char out;
      unsigned char out_;
      unsigned char f_trig;
      unsigned char r_trig;
      unsigned char f_trig_;
      unsigned char r_trig_;
      int wait_pressed;
      int wait_unpressed;
      int pressed_time;
}t_in;


extern void Delay(volatile uint32_t nTime);
extern void scan_in(t_in *key);
	
extern uint64_t		ddwSTIC_CNTR;
extern uint64_t		ddwSTIC_CNTR_OLD;



//таймер, где таймер это  переменная, временная метка
//во время сброса которой ей присваивается системное время
#define TON_RST(TON) TON=ddwSTIC_CNTR

//возвращает TRUE, если с момента сброса таймера TON прошло времени TIME
//значение времени тацймера (TIME) можно изменять в процкссе работы таймера
#define TON_Q(TON,TIME) ((ddwSTIC_CNTR-TON)>=TIME)

//возвращает  прошедшее время
#define TON_ET(TON) (ddwSTIC_CNTR-TON)	

#endif 
