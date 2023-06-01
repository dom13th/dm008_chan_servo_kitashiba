

#include "csmc3.h"
#include "cli.h"
#include "control.h"
#include "motion-planning.h"
#include "d13_uart.h"
#include "main.h"




static void do_jump();
//static void do_go();
//static void do_loc();

static void do_mode();
static void do_sub();
static void do_parm();

static void ds_set(int16_t *ptr);

volatile uint8_t xxCMD;
volatile uint32_t xxWaitCMD;
volatile uint8_t xxWaitSIZE;
volatile uint8_t UART_COMMAND;
volatile uint8_t UART_DATA_SIZE;
volatile uint8_t UART_IN[16];

void dp_dec(int32_t v);	// print an integer value (should be 24 bit)
uint8_t get_line(void);

int32_t val;

uint8_t LineBuf[20];	// Command line input buffer
uint8_t *buf;		// read pointer into line buffer



	uint16_t	_200HzCnt; 	//счетчик 200 Hz импульсов
	uint16_t	VelError200; 	//отколнение скорости (200Hz)
	uint16_t	Velocity; 		//скорость	
	uint16_t	_Velocity; 		//скорость	
	uint16_t	_VelError200; 		//скорость	
	
//;----------------------------------------------------------;
//; Command processing loop

void task_cli(void)
{
	uint8_t c;

	
	c =UART_COMMAND;

	
	
	if (c=='p') {uart_SendByte(Pos); uart_SendByte(Pos>>8);uart_SendByte(Pos>>16);def_cmd_ok;}; //Get Position xxxx
	
	


	
	//_200HzCntH _200HzCntL _VelocityH _VelocityL VelError+1 VelError+0
	if (c=='V') {
		
		uart_SendByte(_200HzCnt);uart_SendByte(_200HzCnt>>8);
		uart_SendByte(Velocity);uart_SendByte(Velocity>>8);
		uart_SendByte(VelError200);uart_SendByte(VelError200>>8);
		def_cmd_ok;
	}; //Get Velocity and 1kHz counter
	
	//CtSub+0 CtSub+1
	if (c=='B') {CtSub=(uint16_t)UART_IN[0]<<8|(uint16_t)UART_IN[1];def_cmd_ok;}; //Set sub-command


	
	//CtPos+0 CtPos+1 CtPos+2
	if (c=='P') {CtPos=(uint32_t)UART_IN[0]<<16|(uint32_t)UART_IN[1]<<8|(uint32_t)UART_IN[2];def_cmd_ok;}; //Set Position
	
	// _Mode+0  rcall	InitServo
	if (c=='M') {init_servo(UART_IN[0]);def_cmd_ok;}; //Set Servo Mode
	
	//_Mode
	if (c=='m') {uart_SendByte(Mode); def_cmd_ok;}; //Get Servo Mode xx

	if (c=='L') {load_parms(UART_IN[0]);def_cmd_ok;}; //Load from EEPROM bank
	if (c=='S') {save_parms(UART_IN[0]);def_cmd_ok;}; //Save to EEPROM bank
	
	if (c=='I') {uart_SendString("SERVO V0.2 INT!"); def_cmd_ok;}; //Get Welcome string and Version
	
	if (c=='W') {if (UART_IN[2]<=N_PARM) {Parms[UART_IN[2]]=(uint16_t)UART_IN[0]<<8|(uint16_t)UART_IN[1]; def_cmd_ok;};	}; 

	if (c=='R') {if (UART_IN[0]<=N_PARM ){
					
			uart_SendByte(Parms[UART_IN[0]]); 
			uart_SendByte(Parms[UART_IN[0]]>>8);
			def_cmd_ok;};
	}; //Read param X xxx
	
	//CtPos+0 CtPos+1 CtPos+2
	if (c=='z') {uart_SendByte(CtPos); uart_SendByte(CtPos>>8);uart_SendByte(CtPos>>16);def_cmd_ok;}; //Read position 2

	
	if (c=='.') {
		
		uint8_t i=0;
	

		while (i<240&&UART_DBG[i]!='.') {
			uart_SendByte(UART_DBG[i]);
			i++;
		};
		def_cmd_ok;
	};
	
	
	uart_SendByte('-');
	xxCMD=0;
	xxWaitCMD=1;
//***************************************************************************	

return;

};



/**
 * print the current location
 *
 * keep updating the output until a key press is received.
 *
 * It is sufficient to compare only the lower 8 bits of the position counter.
 * To help the compiler avoiding an unneeded cast to int for the comparision
 * the two temp variabes a and b are used. The (nonsense) variable assignments
 * are cut out by the optimizer and never make it into the output.
 */
void do_loc(void)
{
	int32_t	p;
	uint8_t a,b;	// temp variables to help the compiler casting the type

	uart_SendString("do_loc");
	xmit(10);
	do {
		xmit(13);
		BEGIN_CRITICAL
		p = Pos;
		END_CRITICAL
		dp_dec(p);
		xmit(32);
		/*13
		do {
			if (Serial_available()) return;
			a = (uint8_t) p;	// all this is optimized away
			b = (uint8_t) Pos;
		} while (a == b);	// and this results in a 8 bit compare
		*/
	} while (1);

}


//Change position command register immediately.
void do_jump(void)
{
	if (get_val()) {
		cmd_err();
		return;
	}
	CtPos = val;
}



void cmd_err(void)
{
	dp_str(m_error);
}




//; Change parameters, command regs or servo mode.
static void do_mode(void)	//:	; Change servo mode
{
	if (get_val()) {
		cmd_err();
		return;
	}
	init_servo(val);
}


static void do_sub(void)		//:	; Set subcommand reg.
{
	ds_set(&CtSub);
}


static void do_parm(void)		// Set parameters
{
	if (get_val() || (val >= N_PARM)) {
		cmd_err();
		return;
	}

	ds_set((int16_t*)&Parms[val]);
}

static void ds_set(int16_t *ptr)
{
	uint8_t c;

	c = get_val();
	if (c == 2) {
		cmd_err();
		return;
	}
	if (c) {	// no value given:
		// show current value
		xmit('\n');
		dp_dec(*ptr);
		xmit(':');

		// allow to enter a new value
		get_line();
		c = get_val();
		if (c == 2) {
			cmd_err();
			return;
		}
		if (c) return;	// don't change anything for empty input
	}
	//FIXME: BEGIN_CRITICAL
	*ptr = val;
	//FIXME: END_CRITICAL
}


/*
 * read one line from serial
 *
 * reads until CR (13) is received or the line buffer is full.
 * This is similar to Arduino Serial.readBytesUntil(), but without any timeout.
 *
 * @returns: number of valid bytes in the line buffer
 */
uint8_t get_line(void)
{
	uint8_t n;	// BH: number of char in buffer
	uint8_t c;
	uint8_t *ptr;		// write pointer in line buffer


	ptr = LineBuf;
	n = 0;
	do {
//		while ((c=receive())<0);	// wait for one char
//13		while(Serial_available()<=0);
//13		c = Serial_read();
		*ptr = c;
		if (c==13) break;
		if (c==8) {	// BS
			if (n) {
				echo(c);
				ptr--;
				n--;
			}
			continue;
		}
		if (c<32) continue;
		if (n==19) continue;
		echo(c);
		ptr++;
		n++;
	} while(1);
	echo(c);
	buf = LineBuf;


	return n;
}


/**
 * print a zero-terminated string in flash over uart
 *
 *
 * This could be _much_ more efficient if the parameter would be in Z already:

b82:     rcall  xmit
dp_str: lpm     AL, Z+
        tst     AL
        brne    b82
        ret

 */
void dp_str(char *pstr)	//	;Display string
{

	unsigned char n = 0;
	while(pstr[n])
    uart_SendByte(pstr[n++]);
	
}

/*
;--------------------------------------;
; Get value of decimal string
;
; Call: X -> ASCII string
; Ret:  X = updated
;         if    C=1: error
;         elsif Z=1: end of line, value=0
;         else:      BL:AL = 24bit value
;
;  Positive:   "300"
;  Negative:   "-125000"
 *
 * The meaning of the return value is different from the assembler version.
 * In C zero is ok.
 *
 * used/effected global variables:
 *	buf: in: the first byte to be read
 *	     out: the first unprocessed byte
 *
 * @returns:
 *	0: ok, valid value in global variable val
 *	1: empty line, val=0
 *	2: syntax error, val invalid
 */
uint8_t get_val(void)
{
	uint8_t	neg=0;
	uint8_t c;			// unsigned is essential, see below

	val = 0;

	// ignore leading space
	do {
		c = (uint8_t) *buf++;
		if (c < ' ') return (1);	// found empty line
	} while (c == ' ');		// read until non-space character

	// handle minus sign
	if (c == '-') {
		neg = 1;
		c = (uint8_t) *buf++;
	}

	// read all characters up to CR or space
	while (c > ' ') {
		c -= '0';	// c is unsigned, so values below 0x30 wrap over
		if (c>9) {
			buf--;
			return (2);	// syntax error
		}
		val = val*10 + c;
		c = (uint8_t) *buf++;
	}

	// success
	buf--;
	if (neg) val = -val;
	return (0);	// ok
}


void dp_dec(int32_t v)	// print an integer value (should be 24 bit)
{

	char sign = ' ';
	char stk[11];
	uint8_t digits = 0;

	if (v<0) {sign = '-';	v = -v;};

	do {
		stk[digits]=((v%10)+'0');
		digits++;
	} while (v /= 10);

	xmit(sign);
	while (digits--) {
	xmit(stk[digits]);
	}	

}


/*
 unsigned division Q,R = N/D

Q := 0                  -- Initialize quotient and remainder to zero
R := 0                     
for i := n − 1 .. 0 do  -- Where n is number of bits in N
  R := R << 1           -- Left-shift R by 1 bit
  R(0) := N(i)          -- Set the least-significant bit of R equal to bit i of the numerator
  if R ≥ D then
    R := R − D
    Q(i) := 1
  end
end

BH	R
AL	Q
*/
