/*
 *	Flight Control Program
 *	Processor: AT91SAM7S256
 *	Board Version: 3.3
 *	June, 2014 - December, 2015
 *	Author: Siyao HU, Tong WU, Jiahao FANG, Xin YE
 */
#include "at91sam7s256.h"
#include "ADC.h"
#include "SDcard.h"
#include "LED.h"
#include "timer.h"
#include "compass_on_gps.h"
#include "gps.h"
#include "mypwm.h"
#include "myusart.h"
#include "mytwi_new.h"
#include "mypit.h"
#include "IMU.h"
#include "math.h"
#include "global.h"
#include "attitude.h"
#include "spibaro.h"
#include "inout.h"
#include "main.h"
#include "PID.h"
#include "PosEst.h"
#include "twi_fast.h"
#include "EEPROM.h"
#include "fixed_matrix_control.h"
#include "test_cpp.h"
#include "USB.h"

void init_loop(void);
int self_check(void);
void data_select(void);
void ground_work_once(void);
void Process500Hz(void);
void Process250Hz_A(void);
void Process250Hz_B(void);
void Process50Hz(void);
#if OUTDOOR
struct _gps gps = {0,0,
				0,0,0,
				0,0,0,
				0,0,0,
				0,0};
#elif INDOOR
struct _vicon vicon = {0,0,0,0,0,0,0};
#endif
struct _smpl smpl = {0,0,1,0};
struct _att att = {0,0,0,
				0,0,0,
				{{DSCRT_I,0,0},
				 {0,DSCRT_I,0},
				 {0,0,DSCRT_I}},
				 {0,0,0,0}};
struct _sens sens = {0,0,0,
					0,0,0,
					0,0,0,0};
struct _mode mode = {MANUEL,0,MANUEL,0,0};
struct _baro baro = {0,0,0,0,0};
struct _pos pos = {{0,0},{0,0},{0,0},
					0,0,0,
					0};
struct _cmd cmd = {{0,0,0,0,0,0,0,0,0},
					0,0,0,
					0,0,0,
					0,0,0,
					0,0,0,
					0,SonarOFF,sendUSB};
struct _ctrl ctrl = {{DSCRT_I,0,0,0},0};
struct _output output = {0,0,0,0};
struct _adc adc = {0};
struct _AT91S_CDC 	pCDC;
struct _myusb myusb = {0,0,{0,0,0},0,0,0};
short data2[9]={0,0,0,0,0,0,0,0,0};
unsigned int process_count=0;
unsigned int l_uart_time=0;
unsigned int uart_time=0;
short uart_dt;
short idle_time=0;

void data_select(void)
{
	switch(cmd.data2send){
	case sendNON:
		break;
	case sendSENS://1
		data2[0]=att.pitchspeed*573>>DSCRT;
		data2[1]=att.rollspeed*573>>DSCRT;		
		data2[2]=att.yawspeed*573>>DSCRT;				
		data2[3]=sens.gx;
		data2[4]=sens.gy;
		data2[5]=sens.gz;
		data2[6]=sens.mx;
		data2[7]=sens.my;
		data2[8]=sens.mz;//baro.alt;
		break;
	case sendGPS://2
	#if OUTDOOR
		data2[0] = gps.x;
		data2[1] = gps.y;		
		data2[2] = gps.z;				
		data2[3] = gps.vx;		//sends in unit 0.1 deg
		data2[4] = gps.vy;	
		data2[5] = 0;		
		data2[6] = gps.azm*57.3;
		data2[7] = gps.sat;
		data2[8] = gps.vel;
	#endif	
		break;
	case sendATT://3
		data2[0] = att.pitch*573>>DSCRT;
		data2[1] = att.roll*573>>DSCRT;		
		data2[2] = att.yaw*573>>DSCRT;				
		data2[3] = cmd.pitch_sp*573>>DSCRT;
		data2[4] = cmd.roll_sp*573>>DSCRT;
		data2[5] = cmd.yaw_sp*573>>DSCRT;;
		data2[6] = pos.x_est[0] / 1000;
		data2[7] = pos.x_est[1] / 1000;		
		data2[8] = rollPID.int_RateErr>>16;
		break;
	case sendPOS://4
		data2[0] = pos.x_est[0] / 1000;//cmd.pos_x_sp;
		data2[1] = pos.y_est[0] / 1000;
		data2[2] = pos.z_est[0] / 1000;//cmd.pitch_sp*573>>DSCRT;			
		data2[3] = pos.x_est[1] / 1000;//cmd.pos_y_sp
		data2[4] = pos.y_est[1] / 1000;
		data2[5] = pos.z_est[1] / 1000;
		data2[6] = cmd.pos_x_sp;
		data2[7] = cmd.pos_y_sp;
		data2[8] = cmd.pos_z_sp;

		
/*		data2[0] = 0;
		data2[1] = pos.Acc_x;//pos.y_est[1];
		data2[2] = pos.x_est[1]/1000;//pos.z_est[0];			
		data2[3] = gps.vx;//gps.vx;//pos.x_est[1];
		data2[4] = 0;//gps.vy;//pos.y_est[1];
		data2[5] = pos.Acc_y;//pos.z_est[1];
		data2[6] = pos.y_est[1];//baro.alt;
		data2[7] = gps.vy;//adc.battery;
		data2[8] = 0;
*/		
		break;
	case sendPID://5
		break;
		case sendCMD://6
		data2[0]=cmd.rc[4];
		data2[1]=cmd.rc[5];		
		data2[2]=cmd.rc[6];				
		data2[3]=cmd.rc[7];
		data2[4]=cmd.rc[8];
		data2[5]=cmd.rc[2];
		data2[6]=idle_time;
		data2[7]=l_uart_time;
		data2[8]=uart_dt;;
		

		break;
	case sendOUT://7
		data2[0]=0;
		data2[1]=output.pitchMmt/10;//cmd.pitch_sp*573>>DSCRT;		
		data2[2]=output.rollMmt/10;				
		data2[3]=output.thrustForce/10;//cmd.roll_sp*573>>DSCRT;
		data2[4]=0;
		data2[5]=cmd.rc[4];
		data2[6]=0;//cmd.Thrust;
		data2[7]=0;
		data2[8]=0;
		break;
	default:
		break;
	}
}
int main (void)
{	
	unsigned char pos_corr_flag=0;	
	led_init();
	led_ctrl(LED1,OFF);
	led_ctrl(LED2,ON);
	pwm_init();
	motor_cut();
	pit_init();		
	ppm_clock_init();
	timer_init();
	twi_init();
#if HMC_COMPASS
	hmc5883_config();
#endif
	g_a_config();	
	compass_config();
	compass_init_read();
	imu_IIR_init();
	twi_fast_init();
#if OUTDOOR
	gps_init();
#endif
	spi_init();
	MS5611_PROM_READ();
	xbee_init();
	USB_init();
	adc_init();
	adc_start_conversion();
	if(USB_armed()){
		myusb.connect_flag = PLUG_IN;
	}
	USB_check();
	delay_ms(1000);
	led_ctrl(LED2,OFF);
	led_ctrl(LED1,ON);
	/*wait until the vehicle is stablized for calibration
	of the gyro and acc at geo coord*/		
	while(1){
	/*The following 3 initiations must be done 
	when the velcle is in static state, but not 
	required to be placed horizontally*/
		//wait for unlock	
		gyro_calibration();		
		quarternion_init();		
		init_loop();
#if ON_FLIGHT
#if USB_TEST
#else
		while(self_check());
#endif
#endif
		led_ctrl(LED1, OFF);
		
		beep(ON);
		delay_ms(200);
		beep(OFF);
		delay_ms(200);
		beep(ON);
		delay_ms(200);
		beep(OFF);
		delay_ms(200);
		beep(ON);
		delay_ms(200);
		beep(OFF);
		delay_ms(200);
		if(USB_armed()){
			myusb.connect_flag = 1;
		}	  
		while(1){//flight loop		
		/*deal with attitude (and position prediction), xbee send, gps (and xy position correction),
		radio control (and motor output), baro (and altitude correction)*/												
			if(smpl.Flag500Hz){
				smpl.Flag500Hz = 0;
			//	beep(ON);
				Process500Hz();
				switch(process_count){
				case 0:
					Process50Hz();
				case 2: case 4: case 6: case 8:
					Process250Hz_B();
					break;
				case 1: case 3: case 5: case 7: case 9:
					Process250Hz_A();
					break;
				default:
					break;
				}
				process_count++;
				if(process_count==10)
					process_count=0;

			#if OUTDOOR
				if(pos_corr_flag){
					pos_corr_flag = 0;
					gps_pos_corr(uart_dt);
				}
				if(gps.gpsflag){
					gps.gpsflag = 0;
					uart_time = timer_get();
					get_gps_data();
					uart_dt = (uart_time - l_uart_time)/1000;
					pos_corr_flag = 1;//leave the cpu load to next round
					l_uart_time = uart_time;
				}
			#elif INDOOR
				if(pos_corr_flag){
					pos_corr_flag = 0;
					vicon_pos_corr(uart_dt);
				}			
				if(vicon.xbeeflag){
					vicon.xbeeflag = 0;
					uart_time = timer_get();
					pos_corr_flag = get_xbee_data();//get_xbee_data decides if filterable
					if(pos_corr_flag){
						uart_dt = (uart_time - l_uart_time)/1000;
						l_uart_time = uart_time;
					}								
				}
			#endif
				if(USB_check()){
					if(myusb.out_coming){
						myusb.out_coming = 0;
						USB_read_Raspberry();
					}
				}
				if(myusb.out_read_flag){
					myusb.out_read_flag = 0;
					myusb.rcv_timeout = TIMEOUT_MS;
					//refill per 8ms, -1 each ms in timer
					USB_read_Process();
				}					
			//	beep(OFF);			
			}//attitude compute end			
			idle_time++;
			//lock break
		}//flight loop end
	}//big loop end
}//main end

void init_loop(void)
{
	/*This function executes loops similar to the flight loop
	during which average values are got. After that, reference
	pressure, reference latitude longitude, and static acceleration 
	corrections in geographical coordinate are obtained*/
	#define LOOP_TIMES 300
	float sum_pressure = 0;
	int avgGlobAcc[3] = {0,0,0},sumGlobAcc[3] = {0,0,0},GlobAcc[3] = {0,0,0},avgBdyAcc[3] = {0,0,0};
	short baro_cnt = 0,acc_cnt = 0;
	short i = 0,j = 0;
#if OUTDOOR
#if WAIT_GPS
	while(gps.sat<6 || gps.lat == 0 || gps.lon == 0){
		if(gps.gpsflag){
			j = !j;
			if(j)
				beep(ON);
			else
				beep(OFF);
			gps.gpsflag = 0;
			get_gps_data();
		}		
	}
	while(!(gps.gpsflag == 1));
	gps.gpsflag = 0;
	get_gps_data();
	gps_pos_init();
	beep(OFF);
#endif	
#endif
	smpl.baroStep = 0;
	while(i < LOOP_TIMES){//init loop
		if(smpl.Flag500Hz){
			smpl.Flag500Hz = 0;
			i++;			
			attitude_compute();
			get_geo_acc(GlobAcc);
			if(i > 200){
				for(j = 0; j < 3; j++)
					sumGlobAcc[j] += GlobAcc[j];
				acc_cnt++;
			}
			switch(process_count){
			case 0: case 2: case 4: case 6: case 8:
				if(smpl.baroStep >= BARO_STEPS){
					smpl.baroStep = 1;
					if(i > 100){
						sum_pressure += baro.pressure;
						baro_cnt++;
					}				
					baro.temp_pres_switch = !baro.temp_pres_switch;
				}
				else {
					smpl.baroStep++;
				}			
				if(baro.temp_pres_switch == PRES_SWITCH)
					temperature_process(smpl.baroStep);					
				else
					pressure_process(smpl.baroStep);	
				break;
			case 1: case 3: case 5: case 7: case 9:
				break;
			default:
				break;
			}
			process_count++;
			if(process_count==10)
				process_count=0;
		}	
	#if INDOOR
		if(vicon.xbeeflag){
			vicon.xbeeflag = 0;
			uart_time = timer_get();
			get_xbee_data();
			l_uart_time = uart_time;
		}
	#elif OUTDOOR
		if(gps.gpsflag){
			gps.gpsflag = 0;
			uart_time = timer_get();
			get_gps_data();
			gps_pos_corr((uart_time - l_uart_time)/1000);//smpl.UARTreceiveCount*1000/PIT_FREQ);
			l_uart_time = uart_time;
		}
	#endif
	}//end of i++ loops

#if INDOOR
	vicon_pos_init();
#elif OUTDOOR
	gps_pos_init();
#endif
	command_init();
	baro.refPressure = sum_pressure / baro_cnt;
	for(j = 0; j < 3; j++){
		avgGlobAcc[j] = sumGlobAcc[j] / acc_cnt;
	}
	glob2body(avgBdyAcc, avgGlobAcc, 3);
	corr_geo_acc(avgBdyAcc);
	smpl.baroStep = 0;
	process_count=0;
}
int self_check(void)
{
	/*Self check if sensor data is available, 
	and if they are reasonable. Returns 1 if good*/
	if(sens.az < 5000)
		return 1;
	else if(cmd.rc[2] > -850 || cmd.rc[2] < -1200)
		return 2;
	else if(baro.refPressure > 120000 || baro.refPressure < 80000)
		return 3;
	else if(pos.x_est[0] / 1000 > 3000||pos.x_est[0] / 1000< -3000
	||pos.y_est[0] / 1000 > 3000||pos.y_est[0] / 1000 < -3000
	||pos.z_est[0] / 1000 > 1000||pos.z_est[0] / 1000 < -1000)
		return 4;
	else
		return 0;
}
void ground_work_once(void)
{


}
void Process500Hz(void)
{
	attitude_compute();
	pos_predict(ATT_PERIOD,process_count);
	attitude_control(ATT_PERIOD);
	put_motors();
}
void Process250Hz_A(void)
{
	static unsigned int usb_count = 0;
	usb_count++;
	if(mode.FlightMode == POS_CTRL){
		position_control(POS_CTRL_PERIOD);
	}
	else if(mode.FlightMode == MANUEL || mode.FlightMode == ALT_CTRL){
		manual_R_sp_generate();
		altitude_control(POS_CTRL_PERIOD);
	}
	else if(mode.FlightMode == RASP_MANUEL || mode.FlightMode == RASP_ALT 
		|| mode.FlightMode == RASP_POS || mode.FlightMode == RASP_NURBS)
		{// in all raspPi modes, rasp gives q_sp and thrust force
			Qsp2Rsp();
			set_thrust_force();
		}
	if(usb_count > 1){//125Hz
		usb_count=0;
		if(myusb.connected){
			USB_write_Raspberry(1024,'s');
		}
	}
}
void Process250Hz_B(void)
{
	static unsigned char temperature_count=1;
	static unsigned char pos_corr_flag=0;
	smpl.baroStep++;	
	if(smpl.baroStep > BARO_STEPS){
		smpl.baroStep = 1;
		temperature_count++;
		if(temperature_count>10){
			temperature_count = 0;
			baro.temp_pres_switch = TEMP_SWITCH;
		}
		else{
			baro.temp_pres_switch = PRES_SWITCH;
		}
	}	
	if(baro.temp_pres_switch == TEMP_SWITCH)
		temperature_process(smpl.baroStep);
	else if(baro.temp_pres_switch == PRES_SWITCH){
		pressure_process(smpl.baroStep);		
	}
	if(pos_corr_flag){
		pos_corr_flag =0;
		baro_pos_corr(BARO_STEPS * BARO_PERIOD);
	}
	if(baro.temp_pres_switch == PRES_SWITCH && smpl.baroStep==4){								
		pos_corr_flag = 1;
		//leave the corr to next round(4ms later)
		//so as to distribute CPU duty averagely			
	}	
}

void Process50Hz(void)
{
	static unsigned int adc_count=0,uart_count=0,led_count=0;
	get_rc(RADIO_PERIOD);
	if(mode.l_FlightMode != mode.FlightMode){						
		if(cmd.rc[2] < -819){
			if(mode.FlightMode == POS_CTRL){
			#if OUTDOOR
				gps_pos_init();			
			#endif			
			}
		}
		command_init();
	}
	if(cmd.rc[2] < -819 && mode.FlightMode == MANUEL){//-0.8
		reset_variables();
		//must not reset when changing mode, the int of att will continue to serve in pos_ctrl
	}
	if(mode.CalibrationMode != 0){
		motor_cut();
		led_ctrl(LED1,ON);
		led_ctrl(LED2,ON);
		if(mode.CalibrationMode != mode.l_CalMode){
			ground_work_once();//data storage etc.
		}	
	}
	else {		
	//0 off, 1 blink, 2 on
	//blue(front) 
	//two manuel modes 0; two alt modes 1; two pos modes 2
	//yellow(back) 
	//no usb, onboard modes 0; usb connect and offboard 1; 
	//usb connect but onboard 2
		led_count++;
		if(led_count>=10){
			led_count = 0;
			if(mode.FlightMode == POS_CTRL || mode.FlightMode == RASP_POS
				||mode.FlightMode == ALT_CTRL || mode.FlightMode == RASP_ALT) 
				led_ctrl(LED2,ON);
			else led_ctrl(LED2,OFF);
			if(myusb.rcv_timeout)//usb running
				led_ctrl(LED1,ON);
			else
				led_ctrl(LED1,OFF);
		}
		else if(led_count==5){
			if(mode.FlightMode == ALT_CTRL || mode.FlightMode == RASP_ALT)
				led_ctrl(LED2,OFF);
			if(mode.offboard)
				led_ctrl(LED1,OFF);
		}
	}
	
	adc_count++;
	if(adc_count>=50){//1Hz
		adc_count = 0;
		adc.battery = adc_get_converted();
		if(adc.battery < BAT_WARNING) beep(ON);
		else beep(OFF);
		adc_start_conversion();	
	}
	else if(adc_count==20){
		beep(OFF);
	}
	
	uart_count++;
	if(uart_count>=5){//10Hz
		uart_count = 0;
		data_select();
		idle_time = 0;
		if(cmd.data2send != sendNON){
//		#if USB_TEST
//		#else
			refill_tx(data2,18);
//		#endif
		}
	}
	
	if(cmd.SonarEnable)
		sonar_pos_corr(RADIO_PERIOD);
	
}
