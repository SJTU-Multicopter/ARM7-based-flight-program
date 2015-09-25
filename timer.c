#include "at91sam7s256.h"
#include "timer.h"
#include "LED.h"
#include "myusart.h"
#include "global.h"
static int delay_count=0;
static int clock=0;//in unit of ms


void clock_init(){
	AT91S_AIC * pAIC = AT91C_BASE_AIC;
	*AT91C_PMC_PCER |=(1<<AT91C_ID_TC0);
	pAIC->AIC_SMR[AT91C_ID_TC0] = AT91C_AIC_SRCTYPE_INT_HIGH_LEVEL | 5;
	pAIC->AIC_SVR[AT91C_ID_TC0] = (unsigned long) clock_int_handler;
	pAIC->AIC_ICCR |= (1 << AT91C_ID_TC0); 
	pAIC->AIC_IECR |= (1 << AT91C_ID_TC0);
	*AT91C_TC0_CMR= AT91C_TC_WAVE |
    				AT91C_TC_WAVESEL_UP_AUTO|
                    AT91C_TC_ACPC_NONE |
                    AT91C_TC_ACPC_NONE |
                    AT91C_TC_AEEVT_NONE|
                    AT91C_TC_ASWTRG_NONE|
                    AT91C_TC_BCPB_NONE|
                    AT91C_TC_BCPC_NONE |
                    AT91C_TC_BEEVT_NONE|
                    AT91C_TC_BSWTRG_NONE |
                    AT91C_TC_CLKS_TIMER_DIV1_CLOCK |								
                    AT91C_TC_EEVT_TIOB;								
	*AT91C_TC0_IER=AT91C_TC_CPCS;
	#if ORIGINAL_FREQ
	*AT91C_TC0_RC=23962;//T=(2/MCK)*RC, where T=0.001s, MCK=18.432e6*(26/5)/2=47923200Hz
	#elif DOUBLED_FREQ
	*AT91C_TC0_RC=47924;
	#endif
	
	*AT91C_TC0_CCR |=(0x0<<1);
	*AT91C_TC0_CCR |=AT91C_TC_CLKEN;
	*AT91C_TC0_CCR |=AT91C_TC_SWTRG; 
}

__irq void clock_int_handler(void){
   	int status0;
	status0=*AT91C_TC0_SR;
	status0=status0;
	clock++;
	*AT91C_AIC_ICCR |= (1 << AT91C_ID_TC0);
	*AT91C_AIC_EOICR=1;	
}

long get_time(void)//in unit of 1us
{
	#if ORIGINAL_FREQ
	return (clock*1000+(*AT91C_TC0_CV)/24);
	#elif DOUBLED_FREQ
	return (clock*1000+(*AT91C_TC0_CV)/48);
	#endif
	
}
void reset_clock(void)
{
	clock=0;
	*AT91C_TC0_CCR |=AT91C_TC_SWTRG;
}

void timer1_init()
{
	AT91S_AIC * pAIC = AT91C_BASE_AIC;
	*AT91C_PMC_PCER |=(1<<AT91C_ID_TC1);
	pAIC->AIC_SMR[AT91C_ID_TC1] = AT91C_AIC_SRCTYPE_INT_HIGH_LEVEL | 4;
	pAIC->AIC_SVR[AT91C_ID_TC1] = (unsigned long) timer1_int_handler;
	pAIC->AIC_ICCR |= (1 << AT91C_ID_TC1); 
	pAIC->AIC_IECR |= (1 << AT91C_ID_TC1);
	*AT91C_TC1_CMR= AT91C_TC_WAVE |
    				AT91C_TC_WAVESEL_UP_AUTO|
                    AT91C_TC_ACPC_NONE |
                    AT91C_TC_ACPC_NONE |
                    AT91C_TC_AEEVT_NONE|
                    AT91C_TC_ASWTRG_NONE|
                    AT91C_TC_BCPB_NONE|
                    AT91C_TC_BCPC_NONE |
                    AT91C_TC_BEEVT_NONE|
                    AT91C_TC_BSWTRG_NONE |
                    AT91C_TC_CLKS_TIMER_DIV1_CLOCK |								
                    AT91C_TC_EEVT_TIOB;								
	*AT91C_TC1_IER=AT91C_TC_CPCS;
	#if ORIGINAL_FREQ
	*AT91C_TC1_RC=23962;//T=(2/MCK)*RC, where T=0.001s, MCK=18.432e6*(26/5)/2=47923200Hz
	#elif DOUBLED_FREQ
	*AT91C_TC1_RC=47924;
	#endif
//	*AT91C_TC1_RA=0;
//	*AT91C_TC1_RB=0;
	*AT91C_TC1_CCR |=(0x0<<1);
	*AT91C_TC1_CCR |=AT91C_TC_CLKEN; 
}

void delay_ms(int i)//i is in unit of ms
{	 
	 delay_count=i;
	 *AT91C_TC1_CCR |=AT91C_TC_CLKEN;
	 *AT91C_TC1_CCR |=AT91C_TC_SWTRG;
	 while(!(delay_count==0));

}



__irq void timer1_int_handler(void)
{
	int status1;
	status1=*AT91C_TC1_SR;
	status1=status1;
	delay_count--;
	if(delay_count==0){
		*AT91C_TC1_CCR|=AT91C_TC_CLKDIS;
	}
	*AT91C_AIC_ICCR |= (1 << AT91C_ID_TC1);
	*AT91C_AIC_EOICR=1;	 
}
