/*
 *	Flight Control Program
 *	Processor: AT91SAM7S256
 *	Board Version: 2.0
 *	June, 2014 - August, 2015
 *	Author: Siyao HU, Tong WU, Jiahao FANG, Xin YE
 */
#include "at91sam7s256.h"
#include "mypwm.h"
#include "myusart.h"
#include "mytwi_new.h"
#include "mypit.h"
#include "timer.h"
#include "LED.h"
#include "math.h"
#include "gps.h"
#include "global.h"
#include "attitude.h"
#include "IMU.h"
#include "spibaro.h"
#include "inout.h"
#include "PID.h"
#include "PosEst.h"
#include "compass_on_gps.h"
#include "EEPROM.h"
#include "fixed_matrix_control.h"

void init_loop(void);
int self_check(void);
void data_select(void);
#if OUTDOOR
struct gps gps = {0.0,0.0,
				0.0,0.0,0.0,
				0,0,0,
				0.0,0.0,0.0};
#elif INDOOR
struct vicon vicon = {0,0,0,0,0,0,0};
#endif
struct smpl smpl = {250,100,1,100,
					0,0,0,0,0,50,0,25,0,
					0,0,0,0};
struct att att = {0,0,0,
				0,0,0,
				{{16384,0,0},
				 {0,16384,0},
				 {0,0,16384}}};
struct sens sens = {0,0,0,
					0,0,0,
					0,0,0};
struct mode mode = {MANUEL,0,MANUEL};
struct baro baro = {0,0,0,0};
struct pos pos = {{0,0},{0,0},{0,0},
					0,0,0,
					0};
struct cmd cmd = {{0,0,0,0,0,0,0,0,0},
					0,0,0,0,
					0,0,0,
					0,SonarOFF,sendATT};
struct output output = {0,0,0,0,0};
short data2[9]={8,7,6,5,4,3,2,1,0xabcd};
long toc;
short idle_time=0;
void data_select(void)
{
	switch(cmd.data2send){
		case sendNON:
		break;
		case sendSENS://1
		data2[0]=sens.ax;
		data2[1]=sens.ay;		
		data2[2]=sens.az;				
		data2[3]=sens.gx;
		data2[4]=sens.gy;
		data2[5]=sens.gz;
		data2[6]=sens.mx;
		data2[7]=sens.my;
		data2[8]=sens.mz;
		break;
		case sendGPS://2
		data2[0] = cmd.pitch_sp*573>>14;
		data2[1] = cmd.roll_sp*573>>14;		
		data2[2] = cmd.yaw_sp*573>>14;				
		data2[3] = cmd.alt_sp;		//sends in unit 0.1 deg
		data2[4] = cmd.pos_x_sp;	
		data2[5] = cmd.pos_y_sp;		
		data2[6] = cmd.Thrust;
		data2[7] = 0;
		data2[8] = idle_time;
		break;
		case sendATT://3
		data2[0] = cmd.pitch_sp*573>>14;
		data2[1] = att.pitch*573>>14;		
		data2[2] = cmd.roll_sp*573>>14;				
		data2[3] = cmd.alt_sp;
		data2[4] = cmd.pos_x_sp;
		data2[5] = cmd.pos_y_sp;
		data2[6] = pos.x_est[0];
		data2[7] = pos.y_est[0];		
		data2[8] = pos.z_est[0];
		break;
		case sendPOS://4
		data2[0] = pos.x_est[0];
		data2[1] = pos.y_est[0];
		data2[2] = pos.z_est[0];			
		data2[3] = pos.x_est[1];
		data2[4] = pos.y_est[1];
		data2[5] = pos.z_est[1];
		data2[6] = pos.Acc_x;
		data2[7] = pos.Acc_y;
		data2[8] = pos.Acc_z;
		break;
		case sendPID://5
		break;
		case sendCMD://6
		data2[0]=cmd.rc[0];
		data2[1]=cmd.rc[1];		
		data2[2]=cmd.rc[2];				
		data2[3]=cmd.rc[3];
		data2[4]=cmd.rc[4];
		data2[5]=cmd.rc[5];
		data2[6]=cmd.rc[6];
		data2[7]=cmd.rc[7];
		data2[8]=idle_time;
		break;
		case sendOUT://7
		data2[0]=output.pitch;
		data2[1]=cmd.pitch_sp*573>>14;		
		data2[2]=output.roll;				
		data2[3]=cmd.roll_sp*573>>14;
		data2[4]=baro.alt/10;
		data2[5]=pos.Acc_z/10;
		data2[6]=output.thrust;
		data2[7]=cmd.altRate_sp/10;
		data2[8]=pos.z_est[1]/10;
		break;
		default:
		break;
	}
}
int main (void)
{		
	smpl.halfT=1000 / smpl.attComputeRate / 2;
	led_init();
	led_ctrl(LED1,OFF);
	led_ctrl(LED2,ON);
	pwm_init();
	motor_cut();
	pit_init();		
	clock_init();
	timer1_init();
	twi_init();
	hmc5883_init();
	g_a_init();	
	compass_init();
	compass_init_read();
#if OUTDOOR
	gps_init();
#endif
	spi_init();
	xbee_init();
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
		while(!self_check());
		led_ctrl(LED1, OFF);	  
		while(1){//flight loop		
		/*deal with attitude (and position prediction), xbee send, gps (and xy position correction),
		radio control (and motor output), baro (and altitude correction)*/												
			if(smpl.attComputeNow){			
				smpl.attComputeNow = 0;	
				get_imu();										   
				attitude_compute();
				pos_predict(1000/smpl.attComputeRate);			
				if(smpl.UARTSendNow){
					smpl.UARTSendNow = 0;
					if(smpl.UARTStep >= 9){//0~9, nine steps to send 27 byte
						smpl.UARTStep = 0;
						data_select();
						idle_time = 0;
					}
					else {
						smpl.UARTStep++;
					}																		
					if(cmd.data2send != sendNON)
						uart_send_packet(data2, 18, smpl.UARTStep);				
				}
			#if OUTDOOR
				if(gps.gpsflag){
					gps.gpsflag = 0;
					get_gps_data();
					gps_pos_corr(GPS_PERIOD);
				}
			#else			
				if(smpl.xbeeflag){
					smpl.xbeeflag = 0;
					get_xbee_data();
					vicon_pos_corr(smpl.ViconCount/25);
					smpl.ViconCount = 0;
				}
			#endif				
				if(smpl.RadioNow){
					smpl.RadioNow = 0;
					get_rc(1000 / smpl.RadioRate);
					if(cmd.SonarEnable)
						sonar_pos_corr(1000/smpl.RadioRate);
					if(mode.FlightMode == ALT_CTRL) led_ctrl(LED2,ON);
					else led_ctrl(LED2,OFF);
					if(mode.FlightMode == POS_CTRL) led_ctrl(LED1,ON);
					else led_ctrl(LED1,OFF);
					if(mode.l_FlightMode != mode.FlightMode){
						command_init();
					#if OUTDOOR
						if(mode.FlightMode == POS_CTRL){
							gps_pos_init();						
						}
					#endif	
					}
					if(cmd.rc[2] < -819){//-0.8
						reset_variables();
					}
					if(mode.FlightMode == POS_CTRL){
						position_control(1000 / smpl.RadioRate);
						attitude_control(1000 / smpl.RadioRate);
					}else{
						manual_R_sp_generate();
						attitude_control(1000 / smpl.RadioRate);
						altitude_control(1000 / smpl.RadioRate);
					}
					if(mode.CalibrationMode == MOTOR_CUT)
						motor_cut();
					else put_motors();
				}
			#if INDOOR
				if(smpl.BaroNow){
					smpl.BaroNow = 0;	
/*					if(a){
						a=0;
						led_ctrl(LED2,OFF);
						led_ctrl(LED1,ON);
					}
					else{
						a=1;
						led_ctrl(LED1,OFF);
						led_ctrl(LED2,ON);
					}
*/
				}
			#elif OUTDOOR		
				if(smpl.BaroNow){
					smpl.BaroNow = 0;
					if(smpl.baroStep >= 11){
						smpl.baroStep = 1;					
						baro_pos_corr(11000 / smpl.BaroRate);
					}else {
						smpl.baroStep++;
					}
					baro_process(smpl.baroStep);
				}
			#endif			
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
	int avgGlobAcc[3] = {0,0,0},sumGlobAcc[3] = {0,0,0},GlobAcc[3] = {0,0,0};
	short baro_cnt = 0,acc_cnt = 0;
	short i = 0,j = 0;
	while(i < LOOP_TIMES){//init loop
		if(smpl.attComputeNow){			
			smpl.attComputeNow = 0;						
			get_imu();										   
			attitude_compute();
			get_geo_acc(GlobAcc);
			if(i > 200){
				for(j = 0; j < 3; j++)
					sumGlobAcc[j] += GlobAcc[j];
				acc_cnt++;
			}
			i++;
		}
	#if INDOOR
		if(smpl.xbeeflag){
			smpl.xbeeflag = 0;
			get_xbee_data();
			smpl.ViconCount = 0;
		}
	#endif		
		if(smpl.BaroNow){
			smpl.BaroNow = 0;
			if(smpl.baroStep >= 11){
				smpl.baroStep = 1;
				if(i > 100){
					sum_pressure += baro.pressure;
					baro_cnt++;
				}						
			}else {
				smpl.baroStep++;
			}
			baro_process(smpl.baroStep);
		}
	}
#if OUTDOOR
//	while(!(gps.gpsflag == 1));
//	gps.gpsflag = 0;
//	get_gps_data();
//	gps_pos_init();
#endif
#if INDOOR
	vicon_pos_init();
#endif
	command_init();
	baro.refPressure = sum_pressure / baro_cnt;
	for(j = 0; j < 3; j++){
		avgGlobAcc[j] = sumGlobAcc[j] / acc_cnt;
	}
	corr_geo_acc(avgGlobAcc);
}
int self_check(void)
{
	/*Self check if sensor data is available, 
	and if they are reasonable. Returns 1 if good*/
	if(sens.az > 5000 && cmd.rc[2] < -819 && cmd.rc[2] > -1433)
		return 1;
	else
		return 0;
}
