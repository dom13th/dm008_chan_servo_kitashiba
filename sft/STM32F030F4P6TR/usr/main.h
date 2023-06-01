#include "csmc3.h"
#include "d13core.h"
#include "stm32f0xx.h"
#include "control.h"

#define TRUE 1
#define FALSE 0
#define move_down  0
#define move_up 1



extern char m_start[];
extern char m_prompt[];
extern char m_error[];



typedef struct{
	uint16_t	P[N_PARM];
}tsEEPROM_PARAM;


