#ifndef USART_H
#define USART_H  
#include "global.h"
extern void xbee_init(void);
extern __irq void usart_int_handler(void);
extern unsigned short crc_update (unsigned short crc, unsigned char data);
extern unsigned short crc16(void* data, unsigned short cnt);
extern void uart_write_char(unsigned char ch);
extern void uart_send_data(unsigned char *buffer,unsigned char length);
extern void uart_send_packet(void *data,unsigned short length,short step);
#if XBEE_DMA
extern void get_xbee_data(void);
#endif
//#define RBREAD 0
//#define RBWRITE 1
//#define RBFREE  2 
//#define RINGBUFFERSIZE	384
#endif  
