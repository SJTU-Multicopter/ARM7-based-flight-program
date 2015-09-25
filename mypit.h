#ifndef PIT_H
#define PIT_H
//extern float x_last;
__irq void pit_int_handler(void);
void pit_init (void);
#endif
