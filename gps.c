#include "at91sam7s256.h"
#include "gps.h"
#include "myusart.h"
#include "global.h"
#if OUTDOOR
char gps_buffer[160];//change bigger?
char gps_buffer2[160];
unsigned char gpsbuffernumber=0;

void gps_init(void)
{
#if OUTDOOR
	AT91S_AIC * pAIC = AT91C_BASE_AIC;
	*AT91C_PMC_PCER|=(1<<AT91C_ID_US1);		
	*AT91C_PIOA_PDR|=0x00600000;		
	*AT91C_PIOA_ASR|=0x00600000;		
	*AT91C_US1_CR=0XAC;				
	*AT91C_US1_MR=0X8C0;			
//	#if notCONFIG_GPS//DMA
		*AT91C_US1_IER=0x00;
		*AT91C_US1_IDR=0xFFFF;
		
		*AT91C_US1_IER=AT91C_US_ENDRX;//AT91C_US_RXRDY;				
#if ORIGINAL_FREQ
	*AT91C_US1_BRGR=78;
#elif DOUBLED_FREQ
	*AT91C_US1_BRGR=156;
#endif
		
		*AT91C_US1_RPR=(unsigned int)gps_buffer;
		*AT91C_US1_RCR=160;
		*AT91C_US1_RNPR=(unsigned int)gps_buffer2;
		*AT91C_US1_RNCR=160;
		pAIC->AIC_SMR[AT91C_ID_US1] = AT91C_AIC_SRCTYPE_INT_HIGH_LEVEL | 6;
		pAIC->AIC_SVR[AT91C_ID_US1] = (unsigned long) gps_int_handler;
		pAIC->AIC_ICCR |= (1 << AT91C_ID_US1); 
		pAIC->AIC_IECR |= (1 << AT91C_ID_US1); 
		*AT91C_US1_CR=0X10;	//only receiving enabled
		*AT91C_US1_PTCR=AT91C_PDC_RXTEN|AT91C_PDC_TXTDIS;
/*	#else//interrupt	
		*AT91C_US1_BRGR=156;
		*AT91C_US1_IER|=AT91C_US_RXRDY;
		pAIC->AIC_SMR[AT91C_ID_US1] = AT91C_AIC_SRCTYPE_INT_HIGH_LEVEL | 6;
		pAIC->AIC_SVR[AT91C_ID_US1] = (unsigned long) gps_int_handler;
		pAIC->AIC_ICCR |= (1 << AT91C_ID_US1); 
		pAIC->AIC_IECR |= (1 << AT91C_ID_US1);
//		*AT91C_US1_IER=AT91C_US_ENDRX;
//		*AT91C_US1_IDR=~AT91C_US_ENDRX; 
		*AT91C_US1_CR=0X50;
	#endif*/
#endif
}

float char_to_float(char *a)//transfer ascII code(with)
{
	float value=0.0;
	int i=0;
	int j=1;
	int sign=0;
	float decade=1;
	int point=-1;//if there is no decimal point, point==-1
	if(a[0]=='-')
		sign=1;	
	for(i=0;i<14;i++){ //find decimal position
	 	if(a[i+sign]=='.'){
		   point=i;
		   break;
		}
	}
	for(j=1;j<(point);j++){//find the decade for max bit
		decade=decade*10;
	}
	for(i=0;i<point;i++){		
	   value+=(a[i+sign]-48)*decade;
	   decade=decade*0.1f;
	}//now we have got the int part
	decade=0.1;

	for(i=point+1;i<14;i++){
	 	if(a[i+sign]!=0){
			value+=(a[i+sign]-48)*decade;
			decade=decade*0.1;
		}
		else{
			break;
		}
	}
	if(sign)
		value=-value;
	return value;
}


//#if notCONFIG_GPS
void get_gps_data(void)//read the data we want in gps_buffer[160]
{
#if OUTDOOR
	static char string[15];//to store ascII of a data between two ","
	static int string_offset=0;
	static int received_type=0;//the type of the data between two ","
	static int frame=0;//GGA or RMC
	static int gps_temp_data[2]={0,0};//to store unsigned LAT and LON, and wait for the NSEW letter
	unsigned char gps_in;
	int i,j;
	for(j=0;j<160;j++)
	{
		if(gpsbuffernumber==1){
			gps_in=gps_buffer[j]; 
		}
		else{
			gps_in=gps_buffer2[j]; 
		}

		if(gps_in=='$'){
			string_offset=0;
			received_type=0;
		}
		else if(gps_in==',')//deal with one string
		{	string_offset=0;		
			if(received_type==0){
				if(string[0]=='G'&&string[1]=='P'&&string[2]=='G'
				&&string[3]=='G'&&string[4]=='A')
					frame=GPGGA;
	
				else if(string[0]=='G'&&string[1]=='P'&&string[2]=='R'
				&&string[3]=='M'&&string[4]=='C')
					frame=GPRMC;

				else 
				 	frame=OTH_FRM;
			}
			if(frame==OTH_FRM)
				continue;
			if(frame==GPGGA){
				if(received_type==6){
				//gps status: 0 not positioning, 1 non-diff positioning, 
				//2 diff positioning, 3 invalid PPS, 6 estimating
					gps.status=(string[0]-48);
				}
				else if(received_type==7){//# of sat in use (00~12)
					gps.sat=(string[0]-48)*10+(string[1]-48);
				}
				else if(received_type==9){//altitude -9,999.9 ~ 99,999.9
					gps.alt=char_to_float(string)*1000;//unit of mm
				}
				else{
		
				}
			}
			if(frame==GPRMC){
				if(received_type==3){//latitude ddmm.mmmm
					gps_temp_data[LAT]=
					(string[0]-48)*10000000
					+(string[1]-48)*1000000
						+((string[2]-48)*1000000
						+(string[3]-48)*100000
						+(string[5]-48)*10000
						+(string[6]-48)*1000
						+(string[7]-48)*100
						+(string[8]-48)*10
						+(string[9]-48)*1)/6;//10^6deg
				}
				else if(received_type==4){//latitude NS
					if(string[0]=='N'){
						gps.lat=gps_temp_data[LAT];
					}
					else if(string[0]=='S'){
						gps.lat=-gps_temp_data[LAT];
					}
				}
				else if(received_type==5){//longitude dddmm.mmmm
					gps_temp_data[LON]=
					(string[0]-48)*100000000
					+(string[1]-48)*10000000
					+(string[2]-48)*1000000
						+((string[3]-48)*1000000
						+(string[4]-48)*100000
						+(string[6]-48)*10000
						+(string[7]-48)*1000
						+(string[8]-48)*100
						+(string[9]-48)*10
						+(string[10]-48)*1)/6;
				}
				else if(received_type==6){//longitude EW			
					if(string[0]=='E'){
						gps.lon=gps_temp_data[LON];
					}
					else if(string[0]=='W'){
						gps.lon=-gps_temp_data[LON];
					}
				}
				else if(received_type==7){//velocity Knots{
					gps.vel=char_to_float(string)*514.4f;//1 knot = 0.5144m/s = 514.4mm/s
				}
				else if(received_type==8){//azimuth deg			
					gps.azm=char_to_float(string) * DEG2RAD * 16384;
				}
				else{

				}
			}
			received_type++;
			string_offset=0;
			for(i=0;i<15;i++){
				string[i]='0';
			}
		}
		else if(gps_in=='\r'||gps_in=='\n'){
		}
		else if(gps_in=='*'){
		}
		else{
			if(string_offset<15)
				string[string_offset]=gps_in;
			string_offset++;
		}
	}
#endif
}
#if OUTDOOR
__irq void gps_int_handler(void)
{
	
	gps.gpsflag=1;
	if (gpsbuffernumber==0){
		gpsbuffernumber=1;
   		*AT91C_US1_RNPR=(unsigned int)gps_buffer;
		*AT91C_US1_RNCR=160;
	}
	else{
		gpsbuffernumber=0;
		*AT91C_US1_RNPR=(unsigned int)gps_buffer2;
		*AT91C_US1_RNCR=160;
	}
	*AT91C_AIC_ICCR |= (1 << AT91C_ID_US1);
	*AT91C_AIC_EOICR=1;
}

#endif
#endif

