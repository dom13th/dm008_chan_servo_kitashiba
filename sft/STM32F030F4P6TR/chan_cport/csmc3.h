/**
 * SMC4 servo motor controller v4
 *
 */

#ifndef _CSCM3_H_
#define _CSCM3_H_

#include <stdint.h>
#include "stm32f0xx.h"


#define BEGIN_CRITICAL	__disable_irq();
#define END_CRITICAL	__enable_irq();


/* --- System configuration ----------------------------------------------- */

//#define SYSCLK	16000000	// System clock
#define BPS	38400		// UART bps
#define TL_TIME	1500		// Error timer (tError(ms)=TL_TIME/3)



/* --- LED pin mapping ---------------------------------------------------- */


#define LED_ERROR	GPIOA,GPIO_Pin_0
#define LED_TORQUE	GPIOA,GPIO_Pin_1
#define LED_READY	GPIOB,GPIO_Pin_1
#define PWM_EN		GPIOA,GPIO_Pin_5



//#define SET_BIT(PORT,BIT)	PORT|=(1<<BIT)
//#define RST_BIT(PORT,BIT)	PORT&=~(1<<BIT)

#define SET_PBIT(GPIOx,GPIO_Pin)	GPIOx->BSRR = GPIO_Pin
#define RST_PBIT(GPIOx,GPIO_Pin)	GPIOx->BRR = GPIO_Pin

#define SET_OUT(SPEC)	SET_PBIT(SPEC)
#define RST_OUT(SPEC)	RST_PBIT(SPEC)






/* --- control parameters -------------------------------------------------- */

#define N_PARM	8	// Number of parameter words per bank.

struct control_s {
	uint16_t	P[N_PARM];
};


/* --- global variables --------------------------------------------------- */



//;----------------------------------------------------------;
//; Data memory area

//; Servo / G command parameters
extern uint16_t Parms[N_PARM];

#define LimSpd	Parms[0]	//P0,Velocity limit		Integer
#define GaSpd	Parms[1]	//P1,Velocity feedback gain	8.8 fixed point
#define GaTqP	Parms[2]	//P2,Proportional gain		8.8 fixed point
#define GaTqI	Parms[3]	//P3,Integral gain		8.8 fixed point
#define LimTrq	Parms[4]	//P4,Torque limit		Integer
#define GaEG	Parms[5]	//P5,EG feedback gain		8.8 fixed point
#define MvSpd	Parms[6]	//P6,G0 velocity			Integer
#define MvAcc	Parms[7]	//P7,G0 acceleration		Integer

//; Command/Servo registers
extern int32_t	CtPos;	// Position 		g/j	mode 3
extern int16_t	CtSub;	// Sub command   	s	mode 0/1/2
extern int16_t	PvInt;	// Integration register
//extern int16_t	PvPos;	// Velocity detection register
extern uint16_t	OvTmr;	// Torque limit timer
extern uint8_t	Mode;	// Servo Mode		m

//; Status registers
extern volatile int32_t	Pos;	// current position
extern int8_t  Flags;

extern inline  void clear_flag(uint8_t bit)
{
	Flags &= ~(1<<bit);
};

extern inline  void set_flag(uint8_t bit)
{
	Flags |= (1<<bit);
};

extern inline  uint8_t flag(uint8_t bit)
{
	return (Flags & (1<<bit));
};

#endif
