#include "at91sam7s256.h"	
#include "main.h"
#include "LED.h"	
void led_init(){	
	*AT91C_PMC_PCER= (1<<AT91C_ID_PIOA);
	*AT91C_PIOA_PER=LED_MASK|BP|USB_IN_O;	  	
	*AT91C_PIOA_OER=LED_MASK|BP|USB_IN_O;
	usb_in_pin(OFF);
}
void led_ctrl(unsigned int channel,unsigned int control){
	if(channel==LED1){
		if(control==ON){
			*AT91C_PIOA_CODR=LED1;
		}
		else if(control==OFF){
			*AT91C_PIOA_SODR=LED1;
		}
	}
	else if(channel==LED2){
		if(control==ON){
			*AT91C_PIOA_CODR=LED2;
		}
		else if(control==OFF){
			*AT91C_PIOA_SODR=LED2;
		}
	}
}
void beep(unsigned int control)
{
	if(control==ON){
		*AT91C_PIOA_SODR|=BP;
	}
	else if(control==OFF){
		*AT91C_PIOA_CODR|=BP;
	}
}
void usb_in_pin(unsigned int control)
{
	if(control==ON){
		*AT91C_PIOA_SODR|=USB_IN_O;
	}
	else if(control==OFF){
		*AT91C_PIOA_CODR|=USB_IN_O;
	}
}
