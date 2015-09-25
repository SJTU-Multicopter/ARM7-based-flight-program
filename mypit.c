#include "at91sam7s256.h"
#include "mypit.h"
#include "main.h"
#include "timer.h"
#include "global.h"
#if ORIGINAL_FREQ
#define PIV 119//59
#elif DOUBLED_FREQ
#define PIV 119
#endif

__irq void pit_int_handler(void){
	unsigned char currentPPM=0;
	static int cntPPM;
//	static int cntSonar;
	static char lastPPM;
	unsigned char maskPPM=0;
	static unsigned short channel=0;
	short timePPM=0;
//	short time=0;
//	static int Period_20ms=0;
//	unsigned int CurrentPins=0;
//	unsigned int Mask=0; 	
//	static unsigned int LastPins;
	if (*AT91C_PIOA_PDSR & IN0){
		currentPPM=1;
	}
//	if (*AT91C_PIOA_PDSR & IN1){CurrentPins|=(unsigned int)1<<0;}
//	if (*AT91C_PIOA_PDSR & IN2){CurrentPins|=(unsigned int)1<<1;}
//	if (*AT91C_PIOA_PDSR & IN3){CurrentPins|=(unsigned int)1<<2;}
//	if (*AT91C_PIOA_PDSR & IN4){CurrentPins|=(unsigned int)1<<3;}

		 
	maskPPM=currentPPM ^ lastPPM;
	lastPPM = currentPPM;
	if(maskPPM){
		if(currentPPM==0){
			cntPPM=0;
		} 
		else{
			timePPM=cntPPM;
			if(timePPM>=0 && timePPM<=100){
				if(channel < 9){
					cmd.rc[channel]=timePPM*95-2921;//20-30.75-41.5
					//((float)timePPM-40)/15;//25-40-55
				}
				channel++;				
			} 
			else{
				channel=0;
			}
		}				
	}
	cntPPM++;
//	if (*AT91C_PIOA_PDSR & IN5){CurrentPins|=1;}
 /*
	Mask=CurrentPins ^ LastPins;
	LastPins=CurrentPins;
	if(Mask){
		if(CurrentPins){//上升沿
			cntSonar=0;
		}
		else{//下降沿
			time=cntSonar;//记录高电平时间
			if(cmd.SonarEnable)
				pos.sonarPos=(float)time*17/2500;//cm unit			
		}
	}
	cntSonar++;

//	for(i=4;i<5;i++){pin[i].TimeCount++;}


	if(Period_20ms==1000){
		Period_20ms=0;
		if(cmd.SonarEnable)
			*AT91C_PIOA_SODR|=SONAR_TRIG;
	}
	if(Period_20ms>=2){
		*AT91C_PIOA_CODR|=SONAR_TRIG;
	}
	Period_20ms++;
*/
	smpl.attComputeCount++;
	smpl.UARTSendCount++;
	smpl.BaroCount++;
	smpl.RadioCount++;
	smpl.ViconCount++;
	if (smpl.attComputeCount == 25000/smpl.attComputeRate){
		smpl.attComputeCount =0;
		smpl.attComputeNow = 1;
	}
	if (smpl.UARTSendCount == 25000/smpl.UARTSendRate){
		smpl.UARTSendCount =0;
		smpl.UARTSendNow = 1;
	}
	if (smpl.BaroCount == 25000/smpl.BaroRate){
		smpl.BaroCount =0;
		smpl.BaroNow = 1;
	}
	if (smpl.RadioCount == 25000/smpl.RadioRate){
		smpl.RadioCount =0;
		smpl.RadioNow = 1;
	}	
	*AT91C_AIC_ICCR |= (1 << AT91C_ID_SYS); // Clear the SYS interrupt
	*AT91C_AIC_EOICR = *AT91C_PITC_PIVR; /* Ack & End of Interrupt */
}

void pit_init () 
{
	AT91S_AIC * pAIC = AT91C_BASE_AIC;
	*AT91C_PITC_PIMR = AT91C_PITC_PITIEN | AT91C_PITC_PITEN | PIV;	
    *AT91C_PMC_PCER |= (1<<AT91C_ID_PIOA);
	*AT91C_PIOA_PER |= PWMIN_MASK;	  
	*AT91C_PIOA_ODR |= PWMIN_MASK;
	*AT91C_PIOA_PPUDR |= PWMIN_MASK;
	*AT91C_PIOA_PER |= SONAR_TRIG;	  	
	*AT91C_PIOA_OER |= SONAR_TRIG;
	pAIC->AIC_SMR[AT91C_ID_SYS] = AT91C_AIC_SRCTYPE_INT_HIGH_LEVEL | 7;
	pAIC->AIC_SVR[AT91C_ID_SYS] = (unsigned long) pit_int_handler;
	pAIC->AIC_ICCR |= (1 << AT91C_ID_SYS); 
	pAIC->AIC_IECR |= (1 << AT91C_ID_SYS); 
}



