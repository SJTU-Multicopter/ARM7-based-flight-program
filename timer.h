#ifndef TIMER_H
#define TIMER_H

#include "at91sam7s256.h"

__irq void clock_int_handler(void);
__irq void timer1_int_handler(void);
extern void clock_init(void);
extern void timer1_init(void);
extern void delay_ms(int i);
extern void reset_clock(void);
long get_time(void);//in unit of 0.01ms, use TC0
extern int clock;//in unit of ms, use TC1
//extern int delay_count;


#endif
