#ifndef LED_H
#define LED_H
#include "at91sam7s256.h"
#define LED1 AT91C_PIO_PA28
#define LED2 AT91C_PIO_PA8
#define ON 1
#define OFF 0
void led_init(void);
void led_ctrl(unsigned int channel,unsigned int control);
#endif
