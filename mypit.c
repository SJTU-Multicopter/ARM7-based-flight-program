#include "at91sam7s256.h"
#include "mypit.h"
#include "main.h"
#include "timer.h"
#include "global.h"
#if ORIGINAL_FREQ
#define PIV (3000000/PIT_FREQ-1)//599//59
#elif DOUBLED_FREQ
#define PIV (6000000/PIT_FREQ-1)
#endif

__irq void pit_int_handler(void)
{

	*AT91C_AIC_ICCR |= (1 << AT91C_ID_SYS); // Clear the SYS interrupt
	*AT91C_AIC_EOICR = *AT91C_PITC_PIVR; /* Ack & End of Interrupt */
}

void pit_init () 
{
	AT91S_AIC * pAIC = AT91C_BASE_AIC;
	*AT91C_PITC_PIMR = AT91C_PITC_PITIEN | AT91C_PITC_PITEN | (PIV);	

	pAIC->AIC_SMR[AT91C_ID_SYS] = AT91C_AIC_SRCTYPE_INT_HIGH_LEVEL | 6;
	pAIC->AIC_SVR[AT91C_ID_SYS] = (unsigned long) pit_int_handler;
	pAIC->AIC_ICCR |= (1 << AT91C_ID_SYS); 
	pAIC->AIC_IECR |= (1 << AT91C_ID_SYS); 
	*AT91C_PMC_PCER |= (1<<AT91C_ID_PIOA);
	pAIC->AIC_SMR[AT91C_ID_PIOA] = AT91C_AIC_SRCTYPE_INT_HIGH_LEVEL | 7;
	pAIC->AIC_SVR[AT91C_ID_PIOA] = (unsigned long)PIO_handler;
	pAIC->AIC_ICCR |= (1 << AT91C_ID_PIOA); 
	pAIC->AIC_IECR |= (1 << AT91C_ID_PIOA);
	*AT91C_PIOA_PER |= IN0|USB_VBUS|USB_OUT_I;
	*AT91C_PIOA_ODR |= IN0|USB_VBUS|USB_OUT_I;
	*AT91C_PIOA_IDR = 0xFFFFFFFF;
	
	*AT91C_PIOA_MDDR |= IN0|USB_VBUS|USB_OUT_I;
	*AT91C_PIOA_PPUDR |= IN0|USB_VBUS|USB_OUT_I;
	*AT91C_PIOA_ASR &= ~(IN0|USB_VBUS|USB_OUT_I);
	*AT91C_PIOA_BSR &= ~(IN0|USB_VBUS|USB_OUT_I);
	*AT91C_PIOA_OWDR |= IN0|USB_VBUS|USB_OUT_I;
	*AT91C_PIOA_ISR;
	*AT91C_PIOA_IER = IN0|USB_VBUS|USB_OUT_I;
}

__irq void PIO_handler(void){
	int timePPM=0;
	int status;
	static unsigned short channel=0;
	status = *AT91C_PIOA_ISR;
    status &= *AT91C_PIOA_IMR;
	if(status & USB_VBUS){
		if (*AT91C_PIOA_PDSR & USB_VBUS){
			myusb.connect_flag = PLUG_IN;
			*AT91C_PIOA_IDR = USB_VBUS;
		}
	}
	if(status & USB_OUT_I){
		if (*AT91C_PIOA_PDSR & USB_OUT_I){
			myusb.out_coming = 1;
		} else{
		//	myusb.out_coming = 1;
		}
	}

	if(status & IN0){
		if (*AT91C_PIOA_PDSR & IN0){
			timePPM = ppm_get_time();
			if(timePPM>=0 && timePPM<=4000){
				if(channel < 9){
				#if F450
					cmd.rc[channel]=244*(timePPM-1220)/100;
				#elif XINSONG
					cmd.rc[channel]=244*(timePPM-1220)/100;
				#elif F330
					cmd.rc[channel]=242*(timePPM-1222)/100;
				#elif F240
					cmd.rc[channel]=-1024;
				#endif
				}
				channel++;				
			} 
			else{
				channel=0;
			}		
		}
		else{
			ppm_reset_clock();
		}
	}
	*AT91C_AIC_ICCR |= (1 << AT91C_ID_PIOA);
	*AT91C_AIC_EOICR = *AT91C_PITC_PIVR;
	return;
}
