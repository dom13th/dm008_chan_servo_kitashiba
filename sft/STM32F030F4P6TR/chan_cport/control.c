#include "csmc3.h"
#include "control.h"
#include "cli.h"


static int16_t T2;	// current velocity





/**
 * Initialize servo system
 *
 * Zero out all status variables and turn off all LEDs.
 */
void init_servo(uint8_t mode)
{
	
	PWM_OUT=240;
	BEGIN_CRITICAL
	Mode = mode;
	CtPos =
	CtSub =
	PvInt =
//	PvPos =
	OvTmr = 0;

	Pos = 0;
	END_CRITICAL

	RST_OUT(LED_ERROR);
	RST_OUT(LED_TORQUE);
	RST_OUT(LED_READY);
}


/**
 * postion control loop (mode 3)
 *
 * FIXME: compiles very inefficient
 */
static inline int16_t tap_position()
{
	int32_t T0;	// position error
	int32_t max;	// temp. variabe to ensure proper signiness

	BEGIN_CRITICAL
	T0 = CtPos - Pos;
	END_CRITICAL

	max = (int32_t) LimSpd;
	// clamp position error to LimSpd (P0)
	if (T0 >= max) T0 = max;
	else if (T0 <= -max) T0 = -max;

	return (int16_t) T0;
}


/**
 * velocity control loop (mode 2)
 *
 * flag 5: lower torque limit active
 * flag 6: lower torque limit active
 * FIXME: combine these two flags into one. That simplyfies everything.
 *
 * @parms:
 * T0:	desired velocity value. This value is expected to be already clamped
 *	to +/-LimSpd
 *
 * used global variables, side effects:
 * GaSpd (r), GaTqP (r), PvInt (rw), GaTqI (r), LimTrq (r), T2/speed (r)
 *
 * @returns: torque value
 */
static inline int16_t tap_velocity(int16_t T0)
{
	
	int16_t	Z;

	T0 -= muls1616(T2,GaSpd);	// error = desired_value - speed*P1
	
	_VelError200+=T0;
	
	Z = muls1616(T0,GaTqP)
	  + muls1616(PvInt,GaTqI);	// Z = Pval*error + Ival*Isum

	// torque limit (P4)
	clear_flag(5);
	clear_flag(6);
	if (Z >= (int16_t) LimTrq) {	// force a signed compare
		Z = LimTrq;
		set_flag(6);
	}
	if (Z <= (int16_t) -LimTrq) {	// force a signed compare
		Z = -LimTrq;
		set_flag(5);
	}

	// calculate integral part, with anti-windup
	if ( ((T0<0)&&!flag(5)) || ((T0>=0)&&!flag(6)) ) {
		PvInt += T0;
	}

	if (flag(5) || flag(6)) {
		SET_OUT(LED_TORQUE);
		OvTmr += 3;
	} else {
		SET_OUT(LED_TORQUE);
		if (OvTmr == 0) return (Z);
		OvTmr--;
	}

	if (OvTmr >= TL_TIME) {
		// trigger a servo error. Go back into save mode 0 (and make
		// servo_operation() skip tap_torque)
		init_servo(0);		// Enter mode 0
		SET_OUT(LED_ERROR);
		return (0);
	}

	return (Z);
}


static inline int16_t tap_torque(int16_t T0)
{
	
	return (T0 + muls1616(T2,GaEG));	// T0 + speed*P5
}


/**
 * output a PWM value (mode 0)
 *
 * Clip output voltage between -240 and +240. Limit minimum duty ratio to
 * 15/16 for bootstrap type FET driver.
 *
 * NOTE: This function compiles quite inefficient for AVR. The direct
 * translation of the original assembler code (v1) even blocks the CPU for
 * negative values of T0 (it works with simavr, though)
 * - v1: 64 bytes and blocking the CPU (too slow to finish in one interrupt?)
 * - v2: 38 bytes, but doesn't work for T0>255 or T0<-255
 * - v3: 46 bytes, works.
 *
 * Consider using the original assembler function instead. (42 bytes)
 */
static inline void tap_voltage(int16_t T0)
{ // v3: 0x1e-0x4c = 46 bytes
	int16_t v;

	// Clip output voltage between -240 and +240.
	if (T0 > 240) {
		PWM_OUT = 480;
	} else if (T0 < -240) {
		PWM_OUT = 0;
	} else {
		PWM_OUT = T0+240;
	};
	
};

void servo_operation()
{
	
	int16_t T0;	// input value to the module

	set_flag(7);		// 1kHz interrupt flag
	
__disable_irq ();
	//скорость
	T2=(int16_t)TIM3->CNT;
__enable_irq ();	
	
	TIM3->CNT=0;	
	//позиция
	Pos+=T2;
	//накопление для 200гц
	_Velocity+=T2;	
	
	
	
	T0 = CtSub;	// only needed for mode 0,1,2

	
	if (Mode>=3) T0 = tap_position();
	if (Mode>=2) T0 = tap_velocity(T0);
	if (Mode>=1) T0 = tap_torque(T0);
	
	tap_voltage(T0);

	
}

int16_t muls1616(int16_t i16, uint16_t u8_8)
{
	int32_t tmp32;

	
	
	tmp32=i16*(u8_8>>8);
	tmp32+=((int32_t)i16*(u8_8&0xff))/256;
	
	i16=tmp32;
	return i16;
	
};	