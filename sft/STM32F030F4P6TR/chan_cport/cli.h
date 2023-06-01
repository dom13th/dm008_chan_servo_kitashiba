#ifndef _CLI_H_
#define _CLI_H_

#include "stm32f0xx.h"
#define def_cmd_ok uart_SendByte('+');xxCMD=0;xxWaitCMD=1;return



// getval(): 0=kein Wert, 1=Wert ok. Wert ist in val
uint8_t get_val(void);
extern int32_t val;

void task_cli(void);

void cmd_err(void);
 void dp_str(char *pstr);	//	;Display string in flash
extern  void do_loc(void);

extern volatile uint8_t xxCMD;
extern volatile uint32_t xxWaitCMD;
extern volatile uint8_t xxWaitSIZE;
extern volatile uint8_t UART_COMMAND;
extern volatile uint8_t UART_DATA_SIZE;
extern volatile uint8_t	UART_IN[16];



extern	uint16_t	_200HzCnt; 	//счетчик 200 Hz импульсов
extern	uint16_t	VelError200; 	//отколнение скорости (200Hz)
extern	uint16_t	Velocity; 		//скорость	
extern	uint16_t	_Velocity; 		//скорость	
extern	uint16_t	_VelError200; 		//скорость		


#endif
