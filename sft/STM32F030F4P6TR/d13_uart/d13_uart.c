
#include "stm32f0xx.h" 
#include "main.h"
#include "d13_uart.h"
#include "cli.h"

 unsigned char UART_DBG[1024];
 volatile uint16_t UART_DBG_i;


//! RX buffer for uart.
unsigned volatile char UART_RxBuffer[UART_RX_BUFFER_SIZE];
//! RX buffer head pointer.
static volatile unsigned char UART_RxHead;
//! RX buffer tail pointer.
static volatile unsigned char UART_RxTail;

// Static Variables.
//! TX buffer for uart.
static unsigned char UART_TxBuffer[UART_TX_BUFFER_SIZE];
//! TX buffer head pointer.
static volatile unsigned char UART_TxHead;
//! TX buffer tail pointer.
static volatile unsigned char UART_TxTail;


void uart_SendByte(unsigned char data)
{
  unsigned char tmphead;

  // Calculate buffer index
  tmphead = ( UART_TxHead + 1 ) & UART_TX_BUFFER_MASK;
  // Wait for free space in buffer
  while ( tmphead == UART_TxTail )
    ;
  // Store data in buffer
  UART_TxBuffer[tmphead] = data;
  // Store new index
  UART_TxHead = tmphead;
  // Enable UDRE interrupt
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
  // ===============================SET_UDRIE;
}


void uart_SendString(char Str[])
{
  unsigned char n = 0;
  while(Str[n])
    uart_SendByte(Str[n++]);
}


void uart_SendInt(long long x)
{
  static const char dec[] = "0123456789";
  uint32_t div_val = 1000000000;

  if (x < 0){
    x = - x;
    uart_SendByte('-');
  }
  while (div_val > 1 && div_val > x)
    div_val /= 10;
  do{
    uart_SendByte (dec[x / div_val]);
    x %= div_val;
    div_val /= 10;
  }while(div_val);
}




unsigned char  uart_ReadByte(void){
unsigned char tmp;
	if (UART_RxHead!=UART_RxTail) {
	tmp=UART_RxBuffer[UART_RxHead];
	UART_RxHead++;
	UART_RxHead&=UART_RX_BUFFER_MASK;
	return tmp;
	}else return 0;	
}


void USART1_IRQHandler(void)
{
	unsigned char UART_TxTail_tmp;
	unsigned char r16;
	
//WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWww
	if (USART_GetITStatus(USART1, USART_IT_TXE) == SET)
	{	
		USART_ClearITPendingBit(USART1, USART_IT_TXE);
		
		UART_TxTail_tmp = UART_TxTail;
		// Check if all data is transmitted
	if ( UART_TxHead !=  UART_TxTail_tmp )
	{
    // Calculate buffer index
    UART_TxTail_tmp = ( UART_TxTail + 1 ) & UART_TX_BUFFER_MASK;
    // Store new index
    UART_TxTail =  UART_TxTail_tmp;
    // Start transmition
    //=================================UDR0= UART_TxBuffer[ UART_TxTail_tmp];
	USART_SendData(USART1, UART_TxBuffer[ UART_TxTail_tmp]);

	}
	else
		// Disable UDRE interrupt
		USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
	}
		
//RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR
	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
	{
	USART_ClearITPendingBit(USART1, USART_IT_RXNE);		
	r16=USART_ReceiveData(USART1);


	
	if (xxCMD) 
	{//* qq	
		xxWaitCMD=1;
		xxWaitSIZE=0;
		
		USART_SendData(USART1, 'X');
	}
	else
	{//* inl
		if (xxWaitCMD)
		{
			UART_COMMAND=r16;
			xxWaitCMD=0;
			xxWaitSIZE=1;
			
			if (r16=='.') {xxCMD=1;};
		}
		else
		{///* r_no_cmd
			if (xxWaitSIZE)
			{
				if (r16<16)
				{//* r_s_ok
				if (r16==0)
					{//* r_s_ok
						xxCMD=1;
					}
					else
					{//* r_st
						r16--;
						UART_DATA_SIZE=r16;
						xxWaitSIZE=0;
					}
				}
				else
				{//* qq
					xxWaitCMD=1;
					xxWaitSIZE=0;
					USART_SendData(USART1, 'X');
				}			
			}
			else
			{//* r_no_siz
				UART_IN[UART_DATA_SIZE]=r16;
				if (UART_DATA_SIZE==0)
				{
					xxCMD=1;
				}
				else
				{//* r_dec
					UART_DATA_SIZE--;
				}
				
			}
			
		}	
	};		



	

	UART_DBG[UART_DBG_i]=r16;

	
	if (UART_DBG_i<240)UART_DBG_i++;


	
	
	}
//********************************************************	
	
}




void xmit(char c)
{
	uart_SendByte(c);
}


