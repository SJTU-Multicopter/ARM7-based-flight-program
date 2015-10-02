#include "at91sam7s256.h"
#include "spibaro.h"
#include "timer.h"	
#include "math.h"
#include "myusart.h"
#include "main.h"
#include "global.h"
#include "PosEst.h"
#if MY_BOARD
	#define  C1 47208				//my
	#define  C2 49509
	#define  C3 29245
	#define  C4 26165
	#define  C5 33245
	#define  C6 28076
#elif FANGS_BOARD
	#define  C1 52649				//Fang's
	#define  C2 53099
	#define  C3 32318
	#define  C4 29234
	#define  C5 33330
	#define  C6 27940
#elif NEW_BOARD
	#define  C1 0xB743				//Fang's
	#define  C2 0xC86B
	#define  C3 0x71C6
	#define  C4 0x6C87
	#define  C5 0x808B
	#define  C6 0x6E0A
#elif BOARD_V4
	#define  C1 0xB785				//Fang's
	#define  C2 0xBE94
	#define  C3 0x70FE
	#define  C4 0x64EF
	#define  C5 0x7CCE
	#define  C6 0x6CFF
#endif
unsigned short setup,CRC;
float dT,Temperature,Pressure;//,refPressure=101000;
unsigned long D1,D2; 
void spi_init(void)
{	   
	*AT91C_PMC_PCER|=(1<<AT91C_ID_SPI);
	*AT91C_PIOA_PDR|=0x7000;	//使能4个相应的引脚的外设功能
	*AT91C_PIOA_ASR|=0x7000;//PA12,13,14，配置成外设A,PA11单独片选用作普通的口
	*AT91C_SPI_CR|=AT91C_SPI_SPIEN;
	*AT91C_SPI_MR=AT91C_SPI_MSTR|AT91C_SPI_PS_FIXED|AT91C_SPI_MODFDIS|0xe0000;
	//后面两个：第一个固定外设选择之后选择的外设号，第二个一个NPCS无效到其它NPCS有效之间的时间延迟
	*AT91C_SPI_IDR|=0x03FF;//all disabled	
    *AT91C_SPI_CSR|=(0x0<<24)|(0x0<<16)|(0x92 << 8)|AT91C_SPI_BITS_8|AT91C_SPI_NCPHA|AT91C_SPI_CSAAT;
	*AT91C_AIC_IDCR = (1<<AT91C_ID_SPI);	
	*AT91C_PIOA_PER=1<<11;		  	
	*AT91C_PIOA_OER=1<<11;
	*AT91C_PIOA_CODR=1<<11;//low-active CSB
		 //这段话指一直让CSB用作普通引脚输出低电平		
	MS5611_RESET();
	delay_ms(10);
//	MS5611_PROM_READ();	
//	delay_ms(10);
}
/*
void pressureinit(void)
{
	short i;
	for (i=0;i<10;i++){
		MS5611_RESET();
		delay_ms(10);
	 	//得到温度
		SPIenable();
		SPI_B0_Strobe(CMD_CONVERT_D2_OSR4096);
		D2=MS5611_SPI_read_ADC();
		SPIdisable();
		data2tempeture();

	   //得到压力
		SPIenable();
		SPI_B0_Strobe(CMD_CONVERT_D1_OSR4096);
		D1=MS5611_SPI_read_ADC();
		SPIdisable();
		data2pressure();
	//	if(i>=10)//data at the beginning are wrong
	// 		testPressure += Pressure;		
	}
	//refPressure=testPressure/10;
	//baro.refPressure=refPressure;
}*/
void baro_process(short step)
{
	 static unsigned char TH,TM,TL,PH,PM,PL;
	 unsigned char dummy=0;
	 float rawAlt;
	 if(step==1){
	 	
		SPIenable();
		sendorder(CMD_CONVERT_D2_OSR4096);		
	 }
	 else if(step==3){  
		dummy=SPI_getdata();
		SPIdisable(); //end of strobe
   		SPIenable();
		sendorder(MS5611_ADC);
	 }
	 else if(step==6){
		dummy=SPI_getdata();
		send0data();
		TH=SPI_getdata();
		send0data();
		TM=SPI_getdata();
		send0data();
		TL=SPI_getdata();
		SPIdisable();
		D2=(((unsigned long)TH)<<16) | (((unsigned long)TM)<<8) | ((unsigned long)TL);
		data2tempeture();
		SPIenable();
		sendorder(CMD_CONVERT_D1_OSR4096);
	 }
	 else if(step==8){
		dummy=SPI_getdata();
		SPIdisable();
		SPIenable();
		sendorder(MS5611_ADC);
	 }
	 else if(step==11){
		dummy=SPI_getdata();
		send0data();
		PH=SPI_getdata();
		send0data();
		PM=SPI_getdata();
		send0data();
		PL=SPI_getdata();
		SPIdisable();
		D1=(((unsigned long)PH)<<16) | (((unsigned long)PM)<<8) | ((unsigned long)PL);
		data2pressure();
		if(baro.refPressure>100){
			rawAlt=get_altitude()-0.1;
			baro.alt=kalman_filter(rawAlt,0.5,1.0,BARO_FILT)*1000;
		}				
	 }
	 dummy=dummy;
}
void MS5611_getTemperature(void)
{
  SPIenable();
  SPI_B0_Strobe(CMD_CONVERT_D2_OSR4096);
  D2=MS5611_SPI_read_ADC(); 
  SPIdisable();
  data2tempeture();
}
void data2tempeture()
{  
   	dT=D2 - (((unsigned long)C5)<<8);
  	Temperature=2000+dT*(((unsigned long)C6)/8388608.0);
	baro.temp=Temperature;
}
void MS5611_getPressure(void)
{
  SPIenable();
  SPI_B0_Strobe(CMD_CONVERT_D1_OSR4096);
  D1=MS5611_SPI_read_ADC();
  SPIdisable();
  data2pressure();
//  OFF=(unsigned long)C2*65536+((unsigned long)C4*dT)/128;
//  SENS=(unsigned long)C1*32768+((unsigned long)C3*dT)/256;
//  Pressure=(D1_Pres*SENS/2097152-OFF)/32768;
}
void data2pressure()
{
	float TEMP2,Aux,OFF,OFF2,SENS,SENS2,Aux2;
	OFF=(unsigned long)C2*65536+((unsigned long)C4*dT)/128;
	SENS=(unsigned long)C1*32768+((unsigned long)C3*dT)/256; 
	if(Temperature<2000){
  	  // second order temperature compensation when under 20 degrees C
		TEMP2 = (dT*dT) / 0x80000000;
		Aux = (Temperature-2000)*(Temperature-2000);
		OFF2 = 2.5*Aux;
		SENS2 = 1.25*Aux;
		if(Temperature<1500){
			Aux2=(Temperature+1500)*(Temperature+1500);
			OFF2=OFF2+7*Aux2;
    		SENS2= SENS2+5.5*Aux2;
		}
    	Temperature = Temperature - TEMP2;
    	OFF = OFF - OFF2;
    	SENS = SENS - SENS2; 
	}
	Pressure=(D1*SENS/2097152-OFF)/32768;
	baro.pressure=Pressure;
}
float get_altitude()
{
  float tmp_float,Altitude;
  float refPressure=baro.refPressure;
//  tmp_float = (Pressure / 101325.0);
  tmp_float = (Pressure / refPressure);
  tmp_float = pow(tmp_float, 0.190295);//x^y
  Altitude = 44330 * (1.0 - tmp_float);
  return (Altitude);
}




unsigned char SPI_B0_Send_byte(unsigned char byte)
{    
	while (!((*AT91C_SPI_SR) & AT91C_SPI_TDRE));
  	*AT91C_SPI_TDR=byte;  
  	delay_ms(10);//must be long enough
  	while (!((*AT91C_SPI_SR) & AT91C_SPI_RDRF));  
  	return(*AT91C_SPI_RDR);
}







unsigned char SPI_B0_Receive_byte(void)
{ 
	unsigned char data;
	data=SPI_B0_Send_byte(0);
	return data;
}
unsigned long MS5611_SPI_read_ADC()
{
  	unsigned char byteH,byteM,byteL;
  	unsigned long return_value;
  	SPIenable(); 
    SPI_B0_Send_byte(MS5611_ADC);
  	byteH = SPI_B0_Receive_byte();
  	byteM = SPI_B0_Receive_byte();
  	byteL = SPI_B0_Receive_byte();
  	SPIdisable();
  	return_value = (((long)byteH)<<16) | (((long)byteM)<<8) | (byteL);
  	return(return_value);
}
void SPI_B0_Strobe(unsigned char strobe)
{ 	
	SPIenable();
  	while (!((*AT91C_SPI_SR) & AT91C_SPI_TDRE)); //等待上一个数据发送完毕
   	SPI_B0_Send_byte(strobe);             // Send strobe
   	SPIdisable();
}
void sendorder(unsigned char order)
{	               
 	while (!((*AT91C_SPI_SR) & AT91C_SPI_TDRE));
  	*AT91C_SPI_TDR=order; 
}
void send0data(void)
{                    
 	while (!((*AT91C_SPI_SR) & AT91C_SPI_TDRE));
  	*AT91C_SPI_TDR=0;
}
unsigned char SPI_getdata()
{
 	while (!((*AT91C_SPI_SR) & AT91C_SPI_RDRF));
	return(*AT91C_SPI_RDR);
}
unsigned short SPI_B0_Read_16bits(unsigned char addr)
{
  	unsigned char byteH,byteL;
  	unsigned short return_value;
   	SPIenable();
	SPI_B0_Send_byte(addr);
	byteH = SPI_B0_Receive_byte();
	delay_ms(1);
	byteL = SPI_B0_Receive_byte();
	SPIdisable();
	return_value = (((unsigned short)byteH)<<8) | (byteL);
	return(return_value);    
}
void MS5611_PROM_READ()
{
  	short data3[9];
  	SPIenable();
  	data3[0] = SPI_B0_Read_16bits(CMD_MS5611_PROM_C1);
  	data3[1] = SPI_B0_Read_16bits(CMD_MS5611_PROM_C2);
  	data3[2] = SPI_B0_Read_16bits(CMD_MS5611_PROM_C3);
  	data3[3] = SPI_B0_Read_16bits(CMD_MS5611_PROM_C4);
  	data3[4] = SPI_B0_Read_16bits(CMD_MS5611_PROM_C5);
  	data3[5] = SPI_B0_Read_16bits(CMD_MS5611_PROM_C6);
	data3[0] = SPI_B0_Read_16bits(CMD_MS5611_PROM_C1);
  	data3[6] = SPI_B0_Read_16bits(CMD_MS5611_RESET);
  	data3[7] = SPI_B0_Read_16bits(CMD_MS5611_PROM_CRC);
	data3[0] = data3[0];
//	data3[0]=C1;
//	data3[1]=C2;
//	data3[2]=C3;
//	data3[3]=C4;
//	data3[4]=C5;
//	data3[5]=C6;
//	data3[6]=setup;
//	data3[7]=CRC;
//	data3[8]=Pressure;																		
//	Uart_send_packet(data3,18);  
	SPIdisable();
}

void SPIenable(void)
{
	*AT91C_PIOA_CODR=1<<11;//low-active CSB
}
void SPIdisable(void)
{
	*AT91C_PIOA_SODR=1<<11;//disable CSB
}
void MS5611_RESET(void)
{		
//	SPIenable();
	sendorder(CMD_MS5611_RESET);
//	SPIdisable();
}

