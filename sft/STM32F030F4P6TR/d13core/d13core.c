#include "d13core.h"


uint64_t	ddwSTIC_CNTR;
uint64_t	ddwSTIC_CNTR_OLD;

uint64_t	ddwT01_TL;				//временная метка таймера 1
uint64_t	ddwT01_ET;				//прошедшее время таймера 1


void scan_in(t_in *key)
{
	if ((key->in)&&(key->out==0)) key->wait_pressed++; else key->wait_pressed=0;
	if (key->wait_pressed>4) {key->wait_pressed=0;key->out=1;}
	if (!(key->in)&&(key->out!=0)) key->wait_unpressed++;	else key->wait_unpressed=0;
	if (!(key->in)&&(key->out!=0)&&(key->wait_unpressed>4))	{key->out=0;}
	if (key->out==key->out_)  {key->f_trig=0; key->r_trig=0;};
	if ((key->out!=0)&&(key->out_==0))  {key->r_trig=1;};
	if ((key->out==0)&&(key->out_!=0))  {key->f_trig=1;};
	key->out_=key->out;
	if ((key->pressed_time<500)&&(key->out!=0)) key->pressed_time++;
	
}





void Delay(volatile uint32_t nTime) {
	uint64_t	ddwT01_TL;
		
	TON_RST(ddwT01_TL);
	while (!TON_Q(ddwT01_TL,MSEC_TO_MKS(nTime)))
		;
}
