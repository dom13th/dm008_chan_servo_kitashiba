#include "main.h"


#include "csmc3.h"
#include "cli.h"
#include "control.h"
#include "d13_uart.h"

	uint64_t	ddwTPWM_EN_TL;				//временная метка таймера 1

int16_t F200hz_CNT;

tsEEPROM_PARAM EEPROM[20];

char m_start[] = "\r\nSMC type 3c\r\n";
char m_prompt[]  = "\r\n%";
char m_error[]  = "X";//"\n?";

int8_t  Flags;

// this could be in control.c
//; Servo / G command parameters
uint16_t Parms[N_PARM];

//; Command/Servo registers
int32_t		CtPos;	// Position 		g/j	mode 3
int16_t		CtSub;	// Sub command   	s	mode 0/1/2
int16_t		PvInt;	// Integration register
//int16_t		PvPos;	// Velocity detection register
uint16_t	OvTmr;	// Torque limit timer
uint8_t		Mode;	// Servo Mode		m

// this could be in encoder.c
//; Status registers
volatile int32_t	Pos;		// current position









void load_parms(uint8_t idx);
void save_parms(uint8_t idx);

void load_parms(uint8_t idx)
{
	//eeprom_read_block(Parms, &parameter_bank[idx], sizeof(struct control_s));
	Parms[0]=EEPROM[idx].P[0];
	Parms[1]=EEPROM[idx].P[1];
	Parms[2]=EEPROM[idx].P[2];
	Parms[3]=EEPROM[idx].P[3];
	Parms[4]=EEPROM[idx].P[4];
	Parms[5]=EEPROM[idx].P[5];
	Parms[6]=EEPROM[idx].P[6];
	Parms[7]=EEPROM[idx].P[7];

}

void save_parms(uint8_t idx)
{
	//eeprom_update_block(Parms, &parameter_bank[idx], sizeof(struct control_s));
	EEPROM[idx].P[0]=Parms[0];
	EEPROM[idx].P[1]=Parms[1];
	EEPROM[idx].P[2]=Parms[2];
	EEPROM[idx].P[3]=Parms[3];
	EEPROM[idx].P[4]=Parms[4];
	EEPROM[idx].P[5]=Parms[5];
	EEPROM[idx].P[6]=Parms[6];
	EEPROM[idx].P[7]=Parms[7];
}







int main(void) {
	uint64_t	ddwT02_TL;				//временная метка таймера 1
		
	TON_RST(ddwT02_TL);
	
#define   MAX_COUNT               100
	
	muls1616(3,5);
	
	RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOB, ENABLE);

	
//****************pwm	
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14 , ENABLE);
	
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
  
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_4);



	TIM_TimeBaseStructure.TIM_Prescaler = 1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 480;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;


	TIM_TimeBaseInit(TIM14, &TIM_TimeBaseStructure);


	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC1Init(TIM14, &TIM_OCInitStructure);


	TIM_Cmd(TIM14, ENABLE);
  
	TIM_CtrlPWMOutputs(TIM14, ENABLE);

/*
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , ENABLE);
	
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
  
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_2);



	TIM_TimeBaseStructure.TIM_Prescaler = 1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 480;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;


	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);


	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC2Init(TIM1, &TIM_OCInitStructure);


	TIM_Cmd(TIM1, ENABLE);
  
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
*/
	
	
//enc *****************************************************************************

    GPIO_InitTypeDef gpio;
    GPIO_StructInit(&gpio);


    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOA, &gpio);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_1);


  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  TIM_TimeBaseInitTypeDef timer_base;
  TIM_TimeBaseStructInit(&timer_base);
  timer_base.TIM_Period = 0xffff;
  timer_base.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &timer_base);

  TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12,
      TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
  
  TIM3->CNT=0; //начальное значение
  TIM_Cmd(TIM3, ENABLE);


///**********************************************************************************************************************************************************************************************
	GPIO_InitTypeDef 	GPIO_Init_SPI_RST;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	GPIO_Init_SPI_RST.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_10|GPIO_Pin_5;
	GPIO_Init_SPI_RST.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Init_SPI_RST.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init_SPI_RST.GPIO_OType = GPIO_OType_PP;
	GPIO_Init_SPI_RST.GPIO_PuPd = GPIO_PuPd_NOPULL;
	
	
	
	GPIO_Init(GPIOA, &GPIO_Init_SPI_RST);
	
	GPIO_Init_SPI_RST.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOB, &GPIO_Init_SPI_RST);


//*************************************************************************************************
//Serial.begin(BPS);
//void InitUART(void)
	
	GPIO_InitTypeDef GPIO_Init_USART;
	
	RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOAEN,ENABLE);
	GPIO_Init_USART.GPIO_Pin=GPIO_Pin_2|GPIO_Pin_3;
	GPIO_Init_USART.GPIO_Mode=GPIO_Mode_AF;
	GPIO_Init_USART.GPIO_Speed=GPIO_Speed_10MHz;
	GPIO_Init_USART.GPIO_OType=GPIO_OType_PP;
	GPIO_Init_USART.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA,&GPIO_Init_USART);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);
	
	
	RCC_APB2PeriphClockCmd(RCC_APB2ENR_USART1EN,ENABLE);
	USART_InitTypeDef USART_InitUser;
	USART_InitUser.USART_BaudRate=38400;
	USART_InitUser.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	USART_InitUser.USART_Mode=USART_Mode_Tx|USART_Mode_Rx;
	USART_InitUser.USART_Parity=USART_Parity_No;
	USART_InitUser.USART_StopBits=USART_StopBits_1;
	USART_InitUser.USART_WordLength=USART_WordLength_8b;
	USART_Init(USART1, &USART_InitUser);
	
	USART_Cmd(USART1, ENABLE);
	

	NVIC_EnableIRQ(USART1_IRQn);
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);



	

	if (SysTick_Config(SystemCoreClock/(1000000/MKS_PER_STICK)))	while (1);		//инициализация SysTick на прерывания каждые MKS_PER_PLCC микросекунд)



// rmp max 30 000 imp


LimSpd	=250;		//Parms[0]	//P0,Velocity limit		Integer
GaSpd	=0x0C80;		//Parms[1]	//P1,Velocity feedback gain	8.8 fixed point
GaTqP	=0x0300;		//Parms[2]	//P2,Proportional gain		8.8 fixed point
GaTqI	=0x00CD;		//Parms[3]	//P3,Integral gain		8.8 fixed point
LimTrq	=154;			//Parms[4]	//P4,Torque limit		Integer
GaEG	=0x07DA;		//Parms[5]	//P5,EG feedback gain		8.8 fixed point  24[v] / 30[imp/ms]
MvSpd	=0;			//Parms[6]	//P6,G0 velocity			Integer
MvAcc	=0;			//Parms[7]	//P7,G0 acceleration		Integer



uint8_t i;


	init_servo(0);		//	;Initial servo mode = 0
	dp_str(m_start);	//	;Start up message

while(i<255) {UART_DBG[i]=0;i++;};

	SET_OUT(LED_READY);
	while (1) {

		
	if ((GlobalSYSTickCounterRTRIGQ))
	{ddwSTIC_CNTR_OLD=ddwSTIC_CNTR;								//сброс GlobalSYSTickCounterRTRIGQ

	};	
	//xxCMD = xxCMD > 0 ?  xxCMD : 0;
	//Pos++;
	//Delay(100);
	//GPIO_ResetBits(GPIOA, GPIO_Pin_0);
	//Delay(100);
	//uart_SendString("SERVO V0.2 INT!");
	//if (uart_ReadByte()==0xff) {uart_SendString("X");};
	//uart_SendString("\n\r> ");
	//uart_SendString(__func__);
	//uart_SendByte(0x31);
	//USART_SendData(USART1,0x31);
	//dp_dec(-2147483639);
	//xmit(0x33);
	//USART_SendData(USART1, 0x32);
	//do_loc();
	//dp_dec(uartRX_CountByte());
	//dp_dec(Serial_available());
	//dp_dec(UART_RxHead);
	//if (uart_ReadByte()==0x31) GPIO_ResetBits(GPIOA, GPIO_Pin_0);
	//if (uart_ReadByte()==0x32) GPIO_SetBits(GPIOA, GPIO_Pin_0);
	if (xxCMD) task_cli();
	//dp_str(m_prompt);
};};
//************************************************************************************

/*
 * Tim0 interrupt routine
 *
 * called 83.3 times per millisecond (divider 192=3*64@16MHz).
 *
 * every time:
 * - Polls the position encoder, update the position counter
 * Every millisecond:
 * - Calls the control loop
 * - updates the position display
 */



void SysTick_Handler(void) {
	ddwSTIC_CNTR+=MKS_PER_STICK;

	if (PWM_OUT!=240) TON_RST(ddwTPWM_EN_TL);
	//if TON_Q(ddwTPWM_EN_TL,500000) GPIO_ResetBits(GPIOA, GPIO_Pin_5); else GPIO_SetBits(GPIOA, GPIO_Pin_5);
	if TON_Q(ddwTPWM_EN_TL,500000) RST_OUT(PWM_EN); else SET_OUT(PWM_EN);
	
	
//********************************************	



//*****************************************
	servo_operation();



	
	F200hz_CNT++;
	if (F200hz_CNT>=5){
		F200hz_CNT=0;
		_200HzCnt++;
		
		Velocity=_Velocity;_Velocity=0;
		
	
		VelError200=_VelError200;_VelError200=0;
		

	};
	
	
	

}








