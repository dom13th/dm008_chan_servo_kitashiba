#ifndef UART_H
#define UART_H

// UART Buffer Defines
#define UART_RX_BUFFER_SIZE 256 // 2,4,8,16,32,64,128 or 256 bytes
#define UART_RX_BUFFER_MASK ( UART_RX_BUFFER_SIZE - 1 )
#if ( UART_RX_BUFFER_SIZE & UART_RX_BUFFER_MASK )
	#error RX buffer size is not a power of 2
#endif

#define UART_TX_BUFFER_SIZE 256 // 2,4,8,16,32,64,128 or 256 bytes
#define UART_TX_BUFFER_MASK ( UART_TX_BUFFER_SIZE - 1 )
#if ( UART_TX_BUFFER_SIZE & UART_TX_BUFFER_MASK )
	#error TX buffer size is not a power of 2
#endif

extern  unsigned char UART_DBG[1024];
extern  volatile uint16_t UART_DBG_i;
	
	

void InitUART(void);
void uart_SendByte(unsigned char data);
void uart_SendString( char Tab[]);
void uart_SendInt(long long x);

unsigned char  uart_ReadByte(void);


	
//! Buffer with received string from uart.
extern volatile unsigned char UART_RxBuffer[UART_RX_BUFFER_SIZE];

	
	
//; Host command
#define echo xmit
void xmit(char);	//	transmit one byte


void Serial_write(uint8_t c);
	
	
	
#endif
