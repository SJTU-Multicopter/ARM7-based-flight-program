#include "at91sam7s256.h"
#include "myusart.h"
#include "mypwm.h"
#include "timer.h"
//#include "stdlib.h"
#include "string.h"
#include "LED.h"
#include "global.h"
#include "gps.h"
#include "PosEst.h"
//extern unsigned short crc_update (unsigned short crc, unsigned char data);
//extern  unsigned short crc16(void* data, unsigned short cnt);
unsigned char protocol_head[3]={'>','*','>'};
unsigned char protocol_last[3]={'<','#','<'};
unsigned char packetdescriptor='c';
char xbee_buffer1[14];
char xbee_buffer2[14];
unsigned char xbee_buffer_num=0;
void xbee_init(void)
{
	AT91S_AIC * pAIC = AT91C_BASE_AIC;
	*AT91C_PMC_PCER|=(1<<AT91C_ID_US0);		
	*AT91C_PIOA_PDR|=0x00000060;		
	*AT91C_PIOA_ASR|=0x00000060;		
	*AT91C_US0_CR=0XAC;				
	*AT91C_US0_MR=0X8C0;	
	#if XBEE_INT		
		*AT91C_US0_IER|=AT91C_US_RXRDY;//|AT91C_US_ENDRX;//new
	#elif XBEE_DMA
		*AT91C_US0_IER=0x00;
		*AT91C_US0_IDR=0xFFFF;
		*AT91C_US0_IER=AT91C_US_ENDRX;
	#endif	
	#if ORIGINAL_FREQ			
	*AT91C_US0_BRGR=52;//26;//312;//52;	57600 Baud 52,115200 26 
	#elif DOUBLED_FREQ
	*AT91C_US0_BRGR=104;
	#endif
	#if XBEE_DMA
		*AT91C_US0_RPR=(unsigned int)xbee_buffer1;
		*AT91C_US0_RCR=14;
		*AT91C_US0_RNPR=(unsigned int)xbee_buffer2;
		*AT91C_US0_RNCR=14;	
	#endif			
	pAIC->AIC_SMR[AT91C_ID_US0] = AT91C_AIC_SRCTYPE_INT_HIGH_LEVEL | 6;
	pAIC->AIC_SVR[AT91C_ID_US0] = (unsigned long) usart_int_handler;
	pAIC->AIC_ICCR |= (1 << AT91C_ID_US0); 
	pAIC->AIC_IECR |= (1 << AT91C_ID_US0); 
	*AT91C_US0_CR=0X50;	
	*AT91C_US0_PTCR=AT91C_PDC_RXTEN;			
}

void uart_send_data(unsigned char *buffer,unsigned char length){	//write continuously
	while (!((*AT91C_US0_CSR) & AT91C_US_TXRDY));
	while(length--)
	{
		//Uart_write_char((unsigned char *)&(buffer++));
		uart_write_char(*(buffer++));
	}	
} 


void uart_write_char(unsigned char ch){		//most basic
	//*AT91C_PIOA_SODR=0x00000080;
	while (!((*AT91C_US0_CSR) & AT91C_US_TXRDY));   
    *AT91C_US0_THR = ch;
}
#if INDOOR
void uart_send_packet(void *data,unsigned short length,short step)	 //good to send char, but to send int data, need conversion first
{
	short crc_result;// = crc16(data,length);
	if(step==0){
		uart_send_data(protocol_head, 3);
	}
	if(step==1){
		
		uart_send_data((unsigned char *) &length, 2);
		uart_send_data((unsigned char *)&packetdescriptor, 1);
		
	//	uart_write_char('h');
	}
	if(step==2){
		uart_send_data(data, 3);
	}
	if(step==3){
		uart_send_data(((unsigned char *)data+3), 3);
	}
	if(step==4){
		uart_send_data(((unsigned char *)data+6), 3);
	}
	if(step==5){
		uart_send_data(((unsigned char *)data+9), 3);
	}
	if(step==6){
		uart_send_data(((unsigned char *)data+12), 3);
	}
	if(step==7){
		uart_send_data(((unsigned char *)data+15), 3);
	//	uart_write_char('\r');
	//	uart_write_char('\n');
	}
	if(step==8){
		crc_result = crc16(data,length);
		uart_send_data((unsigned char *)&crc_result, 2);
	}
	if(step==9){
		uart_send_data(protocol_last, 3);
	}

}
#elif OUTDOOR
void uart_send_packet(void *data,unsigned short length,short step)	 //good to send char, but to send int data, need conversion first
{
//	short crc_result;// = crc16(data,length);
	if(step==0){
//		uart_send_data(protocol_head, 3);
	}
	if(step==1){
		
//		uart_send_data((unsigned char *) &length, 2);
//		uart_send_data((unsigned char *)&packetdescriptor, 1);
		
		uart_write_char('h');
	}
	if(step==2){
		uart_send_data(data, 3);
	}
	if(step==3){
		uart_send_data(((unsigned char *)data+3), 3);
	}
	if(step==4){
		uart_send_data(((unsigned char *)data+6), 3);
	}
	if(step==5){
		uart_send_data(((unsigned char *)data+9), 3);
	}
	if(step==6){
		uart_send_data(((unsigned char *)data+12), 3);
	}
	if(step==7){
		uart_send_data(((unsigned char *)data+15), 3);
		
	}
	if(step==8){
		uart_write_char('\r');
		uart_write_char('\n');
//		crc_result = crc16(data,length);
//		uart_send_data((unsigned char *)&crc_result, 2);
	}
	if(step==9){
//		uart_send_data(protocol_last, 3);
	}

}

#endif


#if INDOOR

__irq void usart_int_handler(void){
	smpl.xbeeflag = 1;
	if(xbee_buffer_num==0){
		xbee_buffer_num=1;
		*AT91C_US0_RNPR=(unsigned int)xbee_buffer1;
		*AT91C_US0_RNCR=14;
	}
	else{
		xbee_buffer_num=0;
		*AT91C_US0_RNPR=(unsigned int)xbee_buffer2;
		*AT91C_US0_RNCR=14;
	}
	*AT91C_AIC_ICCR |= (1 << AT91C_ID_US0); // Clear the SYS interrupt
	*AT91C_AIC_EOICR = 0x1; // Ack & End of Interrupt
}

void get_xbee_data(void){
	#define PACKET_SIZE 8
	short x,y,z,zero;	
	unsigned char bt;
	static int i = 0;
	int j;
	static short sync = 0,sync2 = 0;
	static unsigned char descriptor = 0;
//	static unsigned char crc_received[2]={0,0};
//	unsigned short crc_sent;
//	unsigned short crc_result;
	static unsigned char data_store[20] ={	0, 0, 0, 0, 0, 
											0, 0, 0, 0, 0,
											0, 0, 0, 0, 0, 
											0, 0, 0, 0, 0 };
	for(j=0;j<14;j++)
	{
		if(xbee_buffer_num){
			bt = xbee_buffer1[j];
		}
		else{
			bt = xbee_buffer2[j];
		}
		if (sync == 0){
			if (bt == '>')
				sync = 1;
			else
				sync = 0;
		}
		else if (sync == 1){
			if (bt == '*')
				sync = 2;
			else
				sync = 0;
		}
		else if (sync == 2){
			if (bt == '>'){
				sync = 3;
				descriptor = 0;
				for (i = 0; i < 20; i++){
					data_store[i] = 0;
				}
				i=0;
			}	
			else
				sync = 0;
		}
		else if (sync == 3){
			descriptor = bt;
			if (descriptor == 0x63){
				sync2 = 1;
				i = 0;
			}
			sync = 0;
		}
		if ((sync2 == 1) && (i<PACKET_SIZE)){//data receiving
			if (i<20)
				data_store[i] = bt;
			i++;
		}
		else if ((sync2 == 1) && (i==PACKET_SIZE)){//conclude	
//			crc_received[0] = bt;
			sync2 = 2;
		}
		else if (sync2 == 2){
//			crc_received[1] = bt;
//			crc_result = crc16(data_store,8);
//			memcpy(&crc_sent,crc_received,2);//(unsigned short)crc_received[0] + (unsigned short)crc_received[1] <<8;
			if(1){//crc_result == crc_sent){
				if (descriptor == 0x63){//c
					memcpy(&x,data_store+1,2);
					memcpy(&y,data_store+3,2);
					memcpy(&z,data_store+5,2);
					memcpy(&zero,data_store+7,2);
					zero = zero;
					if(x!=0)
						vicon.x = x;
					if(y!=0)
						vicon.y = y;
					if(z!=0)
						vicon.z = z;	
				}
 		 	}
			sync = 0;
			sync2 = 0;
			descriptor = 0;
			for (i = 0; i < 20; i++){
				data_store[i] = 0;
			}
			i=0;
		}
	}
}	
#elif OUTDOOR
__irq void usart_int_handler(void){	
	unsigned char bt;
	static int i = 0;
	static short sync = 0, sync2 = 0;
	static unsigned char descriptor = 0;
	static unsigned char size[2];
	static short packet_size = 0;
	static unsigned char data_store[20] ={	0, 0, 0, 0, 0, 
											0, 0, 0, 0, 0,
											0, 0, 0, 0, 0, 
											0, 0, 0, 0, 0 };
	if(*AT91C_US0_CSR & AT91C_US_RXRDY)//new this is a receive interrupt
	{
		bt = *AT91C_US0_RHR;
		if (sync2 == 0){
			if (bt == '<')
				sync2 = 1;
			else
				sync2 = 0;
		}
		else if (sync2 == 1){
			if (bt == '#')
				sync2 = 2;
			else
				sync2 = 0;
		}
		else if (sync2 == 2){
			if (bt == '<'){
				sync = 0;
				sync2 = 0;
				descriptor = 0;
				size[0] = 0;
				size[1] = 0;
				packet_size = 0;
				for (i = 0; i < 20; i++){
					data_store[i] = 0;
				}
				i=0;
			}
			else{
				sync2 = 0;
			}
		}
		if (sync == 0){
			if (bt == '>')
				sync = 1;
			else
				sync = 0;
		}
		else if (sync == 1){
			if (bt == '*')
				sync = 2;
			else
				sync = 0;
		}
		else if (sync == 2){
			if (bt == '>')
				sync = 3;
			else
				sync = 0;
		}
		else if (sync == 3){
			size[0] = bt;
			sync = 4;
		}
		else if (sync == 4){
			size[1] = bt;
			packet_size = size[0] + size[1] * 256;
			sync = 5;
		}
		else if (sync == 5){
			descriptor = bt;
				sync = 6;
			i = 0;
		}
		else if ((sync == 6) && (i<packet_size)){//data receiving
			if (i<20)
				data_store[i] = bt;
			i++;
		}
		else if ((sync == 6) && (bt == '#')&&(i==packet_size)){//conclude	
			if (descriptor == 0x50){//P
//				memcpy(&rollPID.P,data_store,4);
//				memcpy(&pitchPID.P,data_store,4);
				pos_xPID.P=(float)data_store[0]*10+(float)data_store[1]*1
					+(float)data_store[2]*0.1+(float)data_store[3]*0.01;
				pos_yPID.P=pos_xPID.P;
			}
			else if (descriptor == 0x70){//p
//				memcpy(&rollPID.Prate,data_store,4);
//				memcpy(&pitchPID.Prate,data_store,4);
				pos_xPID.Prate=(float)data_store[0]*10+(float)data_store[1]*1
					+(float)data_store[2]*0.1+(float)data_store[3]*0.01;
				pos_yPID.Prate=pos_xPID.Prate;			
			}
			else if (descriptor == 0x64){//d
//				memcpy(&rollPID.Drate,data_store,4);
//				memcpy(&pitchPID.Drate,data_store,4);
				pos_xPID.Irate=(float)data_store[0]*1+(float)data_store[1]*0.1
					+(float)data_store[2]*0.01+(float)data_store[3]*0.001;
				pos_yPID.Drate=pos_xPID.Drate;			
			}
			else if (descriptor == 'S'){
				switch(data_store[0]){
				case sendNON:
					cmd.data2send=sendNON;
				break;
				case sendSENS:
					cmd.data2send=sendSENS;
				break;
				case sendGPS:
					cmd.data2send=sendGPS;
				break;
				case sendATT:
					cmd.data2send=sendATT;
				break;
				case sendPOS:
					cmd.data2send=sendPOS;
				break;
				case sendPID:
					cmd.data2send=sendPID;
				break;
				case sendCMD:
					cmd.data2send=sendCMD;
				break;
				case sendOUT:
					cmd.data2send=sendOUT;
				break;
				case KILL:
					pwm_set_duty_cycle(0, 100);
					pwm_set_duty_cycle(1, 100);
					pwm_set_duty_cycle(2, 100);
					pwm_set_duty_cycle(3, 100);
					mode.CalibrationMode = MOTOR_CUT;
				break;
				default:
				break;
				}
			} 		 
			sync = 0;
			descriptor = 0;
			size[0] = 0;
			size[1] = 0;
			packet_size = 0;
			for (i = 0; i < 20; i++){
				data_store[i] = 0;
			}
			i=0;
		}
	}
	*AT91C_AIC_ICCR |= (1 << AT91C_ID_US0); // Clear the SYS interrupt
	*AT91C_AIC_EOICR = 0x1; // Ack & End of Interrupt
}

#endif
unsigned short crc_update (unsigned short crc, unsigned char data)
{
    data ^= (crc & 0xff);
    data ^= data << 4;

    return ((((unsigned short )data << 8) | ((crc>>8)&0xff)) ^ (unsigned char )(data >> 4)
         ^ ((unsigned short )data << 3));
}

 unsigned short crc16(void* data, unsigned short cnt)
{
    unsigned short crc=0xff;
    unsigned char * ptr=(unsigned char *) data;
    int i;

    for (i=0;i<cnt;i++)
    {
        crc=crc_update(crc,*ptr);
        ptr++;
    }
    return crc;

}

