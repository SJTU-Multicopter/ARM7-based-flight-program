#include "inout.h"
#include "global.h"
#include "mypwm.h"
#include "attitude.h"
void command_init(void)
{
	cmd.alt_sp = pos.z_est[0];	
	#if INDOOR
	cmd.pos_x_sp = pos.x_est[0];
	cmd.pos_y_sp = pos.y_est[0];
	#endif	
}
void get_rc(short dt)
{
	int expctBodyVel[2];
	int expctGlobVel[2];
	int expctAltRate,expctYawRate;
	mode.l_FlightMode = mode.FlightMode;
	if (cmd.rc[4] > 307){	   //switch upward	
		mode.FlightMode = MANUEL;
	}
	else if(cmd.rc[4] > -307){		//in the middle
		mode.FlightMode = MANUEL;
	}
	else{		//downward
		mode.FlightMode = MANUEL;
	}
	mode.CalibrationMode = 0;
	//pitch roll att command for acrob, manuel, altctrl
	if(mode.FlightMode == MANUEL || mode.FlightMode==ACROBATIC || mode.FlightMode == ALT_CTRL|| mode.FlightMode == ALT_RATE_CTRL){
		cmd.pitch_sp = dead_zone(cmd.rc[1] * MAX_ATT_MANUEL>>10, 0);
		cmd.roll_sp = dead_zone(-cmd.rc[0] * MAX_ATT_MANUEL>>10, 0);
	}
	//xy pos command for posctrl
	else if(mode.FlightMode == POS_RATE_CTRL){
		expctBodyVel[0] = dead_zone(cmd.rc[1] * MAX_XY_VEL>>10, 400);
		expctBodyVel[1] = dead_zone(cmd.rc[0] * MAX_XY_VEL>>10, 400);
		body2glob(expctBodyVel, expctGlobVel, 2);
		cmd.pos_x_sp = expctGlobVel[0];
		cmd.pos_y_sp = expctGlobVel[1];	
	}
	else if(mode.FlightMode == POS_CTRL|| mode.FlightMode == XY_CTRL){
		expctBodyVel[0] = dead_zone(cmd.rc[1] * MAX_XY_VEL>>10, 400);
		expctBodyVel[1] = dead_zone(cmd.rc[0] * MAX_XY_VEL>>10, 400);
		body2glob(expctBodyVel, expctGlobVel, 2);
		cmd.pos_x_sp += expctGlobVel[0] * dt>>10;	//it should be dt/1000	
		cmd.pos_y_sp += expctGlobVel[1] * dt>>10;  //but it's only manuel control
	}												//so regard >>10 /1024 as it
	//yaw command for all
	expctYawRate = dead_zone(cmd.rc[3] * MAX_YAW_RATE>>10, 4260);//deadzone 15 deg
	cmd.yaw_sp += expctYawRate * dt>>10;		
	//throtle command for manuel, acrob
	if(mode.FlightMode == MANUEL || mode.FlightMode == ACROBATIC|| mode.FlightMode == XY_CTRL){
		cmd.Thrust = dead_zone(constrain((cmd.rc[2] + 1024)/2 * thrCmndRatio, 0, 1400), 50);
	}
	//alt command for altctrl, posctrl
	else if(mode.FlightMode==ALT_CTRL||mode.FlightMode==POS_CTRL||mode.FlightMode == POS_RATE_CTRL){
		expctAltRate = dead_zone(cmd.rc[2] * MAX_ALT_RATE>>10, 200);
		cmd.alt_sp += expctAltRate * dt>>10;
	}
	else if(mode.FlightMode==ALT_RATE_CTRL){
		cmd.altRate_sp = dead_zone(cmd.rc[2] * MAX_ALT_RATE>>10, 50); //+-0.5m/s		
	}
	//tuning
	pos_yPID.Prate = 2.65 + (cmd.rc[5]/1024.0)*1.0;
	pos_xPID.Prate = 2.65 + (cmd.rc[6]/1024.0)*1.0;
	//pos_yPID.Irate = 0.02 + (cmd.rc[7]/1024.0)*0.02;
	
	pos_xPID.P = pos_yPID.P;
//	pos_yPID.Prate = pos_xPID.Prate;	
//	pos_yPID.Irate = pos_xPID.Irate;
//	pos_yPID.Drate = pos_xPID.Drate;	
//	altPID.P = 0.8 + (cmd.rc[5]/1024.0)*0.3;
//	altPID.Prate = 1.1 + (cmd.rc[6]/1024.0)*0.3;
//	altPID.Drate = 0.001 + cmd.rc[7]*0.001;
//	altPID.Irate = 0.1 + cmd.rc[7]*0.1;

	
//	pitchPID.Prate = 22.5 + (cmd.rc[6]/1024.0)*2.0;
//	pitchPID.Drate = 0.03 + (cmd.rc[7]/1024.0)*0.03;
//	pitchPID.P = 2.21 + (cmd.rc[5]/1024.0)*0.6;
//	rollPID.Prate = 22.4 + (cmd.rc[6]/1024.0)*6.0;
//	rollPID.Drate = 0.03 + (cmd.rc[7]/1024.0)*0.03;
//	rollPID.P = pitchPID.P;
	//mode change
	
}
void put_motors(void)
{
	int motorVal[4] = {0,0,0,0};
	unsigned short motorDuty[4] = {100,100,100,100};
	short i;
	if(cmd.rc[2] < -819){//-0.8*1024
		motorVal[0] = 0;
		motorVal[1] = 0;
		motorVal[2] = 0;
		motorVal[3] = 0;
		for(i=0;i<4;i++){
			motorDuty[i]=2400;
		}
	}
	else{
	#if PLUS
		motorVal[0] = -output.pitch + output.yaw + output.thrust * 100;
		motorVal[1] = -output.roll - output.yaw + output.thrust * 100;
		motorVal[2] = output.pitch + output.yaw + output.thrust * 100;
		motorVal[3] = output.roll - output.yaw + output.thrust * 100;
	#elif CROSS
		motorVal[0] = (output.pitch + output.roll) * HALF_SQRT_2 - output.yaw + output.thrust * 100;
		motorVal[1] = (-output.pitch - output.roll) * HALF_SQRT_2 - output.yaw + output.thrust * 100;
		motorVal[2] = (output.pitch - output.roll) * HALF_SQRT_2 + output.yaw + output.thrust * 100;
		motorVal[3] = (-output.pitch + output.roll) * HALF_SQRT_2 + output.yaw + output.thrust * 100;
	#endif		
		for(i=0;i<4;i++){
			motorDuty[i]=constrain((unsigned short)((motorVal[i]*24>>11) + 2640), 2400, 5280);
		}
	}
	//channel-PWMlinesign order- 1324
	pwm_set_duty_cycle(0, motorDuty[0]);
	pwm_set_duty_cycle(1, motorDuty[1]);
	pwm_set_duty_cycle(2, motorDuty[2]);
	pwm_set_duty_cycle(3, motorDuty[3]);

}
//insert a into [b,c]
//#define constrain(a,b,c) ((a)<(b)?(b):(a)>(c)?c:a)
int constrain(int a, int b, int c){
	return ((a)<(b)?(b):(a)>(c)?c:a);
}
//dead zone +-b
//#define dead_zone(a,b) ((a)>(b)?(a):(a)<(-b)?(a):0)
int dead_zone(int a, int b){
	return ((a)>(b)?(a):(a)<(-b)?(a):0);
}

//#define interpol(x,x1,y1,x2,y2) (y1+(y2-y1)/(x2-x1)*(x-x1))
//#define myabs(a) ((a)>=0?(a):(-a))

