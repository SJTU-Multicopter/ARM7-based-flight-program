#include "global.h"
#include "attitude.h"
#include "math.h"
#include "fixed_matrix_control.h"
#if R_CTRL_PID
#if F330
PID pitchPID={0,0,0,0,
			2.64,
			22.5,0,0.037};//.006};//I0.0082*57.3
PID rollPID={0,0,0,0,
			2.27,
			24.5,0,0.036};//.006};
PID yawPID={0,0,0,0,
			3.0,
			34.4,0,0};
PID altPID={0,0,0,0,
			1.1,
			1.4,0.0,0.0};//0.0068,0.0218};
PID pos_xPID={0,0,0,0,
			0.31,
			2.79,0.0,0.006};//0.0076};
PID pos_yPID={0,0,0,0,
			0.31,
			2.49,0.0,0.006};//0.0076};
#elif F450
PID pitchPID={0,0,0,0,
			2.64,
			22.5,0,0.037};//.006};//I0.0082*57.3
PID rollPID={0,0,0,0,
			2.27,
			24.5,0,0.036};//.006};
PID yawPID={0,0,0,0,
			3.0,
			34.4,0,0};
PID altPID={0,0,0,0,
			1.1,
			1.4,0.0,0.0};//0.0068,0.0218};
PID pos_xPID={0,0,0,0,
			0.31,
			2.79,0.0,0.006};//0.0076};
PID pos_yPID={0,0,0,0,
			0.31,
			2.49,0.0,0.006};//0.0076};
#endif
#endif
int external_err_pid(PID *pid, int err, short dt);
int internal_err_pid(PID *pid, int rate_err, short dt);
int force2output(int force, unsigned int battery);
int R_sp[3][3];
void position_control(short dt)
{
	int globVel_sp[3], globAcc_sp[3], Z_b[3], y_C[3];
	int body_x_sp[3], body_y_sp[3], body_z_sp[3];
	int norm;
	short i;	
	//get acc sp from position setpoint
	globVel_sp[0] = constrain(external_err_pid(&pos_xPID, cmd.pos_x_sp - pos.x_est[0], dt), -MAX_XY_VEL, MAX_XY_VEL);
	globVel_sp[1] = constrain(external_err_pid(&pos_yPID, cmd.pos_y_sp - pos.y_est[0], dt), -MAX_XY_VEL, MAX_XY_VEL);
	globVel_sp[2] = constrain(external_err_pid(&altPID, cmd.alt_sp - pos.z_est[0], dt), -MAX_ALT_VEL, MAX_ALT_VEL);
	
	globAcc_sp[0] = internal_err_pid(&pos_xPID, globVel_sp[0] - pos.x_est[1], dt);
	globAcc_sp[1] = internal_err_pid(&pos_yPID, globVel_sp[1] - pos.y_est[1], dt);
	globAcc_sp[2] = GRAVITY + internal_err_pid(&altPID, globVel_sp[2] - pos.z_est[1], dt);
	//dot product of setpoint and current z axis of body
	for(i=0;i<3;i++)
		Z_b[i]=att.R[i][2];
//in c++,	cmd.Thrust = (globAcc_sp * Z_b) * VEHICLE_MASS;
	cmd.Thrust = ((Z_b[0] * globAcc_sp[0] + Z_b[1] * globAcc_sp[1] + Z_b[2] * globAcc_sp[2])>>14) * VEHICLE_MASS/1000 ;
//cmd.Thrust in mNewton
	output.thrust = force2output(cmd.Thrust, adc.battery);
	//get the last col of R_sp from acc setpoint
	for(i=0;i<3;i++)
		body_z_sp[i] = globAcc_sp[i];
//in c++,	body_z_sp.normalize();
	norm = sqrt(body_z_sp[0] * body_z_sp[0] + body_z_sp[1] * body_z_sp[1] + body_z_sp[2] * body_z_sp[2]);
	for(i=0;i<3;i++)
		body_z_sp[i] = (body_z_sp[i]<<14) / norm;
	//according to yaw_sp get the rest of R_sp
	y_C[0] = -sin(cmd.yaw_sp/16384.0f) * 16384.0f;
	y_C[1] = cos(cmd.yaw_sp/16384.0f) * 16384.0f;
	y_C[2] = 0;
//c = a % c
//rc = (ra % rb)/r
//in c++	body_x_sp = y_C % body_z_sp;
	body_x_sp[0] = (y_C[1] * body_z_sp[2] - y_C[2] * body_z_sp[1])>>14;
	body_x_sp[1] = (y_C[2] * body_z_sp[0] - y_C[0] * body_z_sp[2])>>14;
	body_x_sp[2] = (y_C[0] * body_z_sp[1] - y_C[1] * body_z_sp[0])>>14;
//in c++	body_x_sp.normalize();
	norm = sqrt(body_x_sp[0] * body_x_sp[0] + body_x_sp[1] * body_x_sp[1] + body_x_sp[2] * body_x_sp[2]);
	for(i=0;i<3;i++)
		body_x_sp[i] = (body_x_sp[i]<<14) / norm;	
//in c++	body_y_sp = body_z_sp % body_x_sp;
	body_y_sp[0] = (body_z_sp[1] * body_x_sp[2] - body_z_sp[2] * body_x_sp[1])>>14;
	body_y_sp[1] = (body_z_sp[2] * body_x_sp[0] - body_z_sp[0] * body_x_sp[2])>>14;
	body_y_sp[2] = (body_z_sp[0] * body_x_sp[1] - body_z_sp[1] * body_x_sp[0])>>14;
	for (i = 0; i < 3; i++) {
		R_sp[i][0] = body_x_sp[i];
		R_sp[i][1] = body_y_sp[i];
		R_sp[i][2] = body_z_sp[i];
	}
	cmd.pitch_sp = -asin(R_sp[2][0]/16384.0f)*16384;
	cmd.roll_sp = atan2(R_sp[2][1], R_sp[2][2])*16384;
//	cmd.yaw_sp = -atan2(R_sp[0][1], R_sp[1][1])*16384;
}
void altitude_control(short dt)
{
	int altRate_sp, altAcc_sp;
	if(mode.FlightMode == MANUEL || mode.FlightMode == ACROBATIC || mode.FlightMode == XY_CTRL){
	//pass command directly into output				
		output.thrust = cmd.Thrust;	
	}
	else if(mode.FlightMode==ALT_CTRL||mode.FlightMode==POS_CTRL||mode.FlightMode == POS_RATE_CTRL){	
		altRate_sp = constrain(external_err_pid(&altPID, cmd.alt_sp - pos.z_est[0], dt), -MAX_ALT_VEL, MAX_ALT_VEL);
		altAcc_sp = GRAVITY + internal_err_pid(&altPID, altRate_sp - pos.z_est[1], dt);
		cmd.Thrust = ((altAcc_sp<<14) / att.R[2][2]) * VEHICLE_MASS/1000;
		output.thrust = force2output(cmd.Thrust, adc.battery);
	}
}
void manual_R_sp_generate(void)
{
	float cp = cos(cmd.pitch_sp/16384.0f);
	float sp = sin(cmd.pitch_sp/16384.0f);
	float sr = sin(cmd.roll_sp/16384.0f);
	float cr = cos(cmd.roll_sp/16384.0f);
	float sy = sin(cmd.yaw_sp/16384.0f);
	float cy = cos(cmd.yaw_sp/16384.0f);
	R_sp[0][0] = cp * cy * 16384;
	R_sp[0][1] = ((sr * sp * cy) - (cr * sy)) * 16384;
	R_sp[0][2] = ((cr * sp * cy) + (sr * sy)) * 16384;
	R_sp[1][0] = cp * sy * 16384;
	R_sp[1][1] = ((sr * sp * sy) + (cr * cy)) * 16384;
	R_sp[1][2] = ((cr * sp * sy) - (sr * cy)) * 16384;
	R_sp[2][0] = -sp * 16384;
	R_sp[2][1] = sr * cp * 16384;
	R_sp[2][2] = cr * cp * 16384;
}
void reset_variables(void)
{
	altPID.int_RateErr = 0;
	pitchPID.int_RateErr = 0;
	rollPID.int_RateErr = 0;
	yawPID.int_RateErr = 0;
	pos_xPID.int_RateErr = 0;
	pos_yPID.int_RateErr = 0;
	cmd.yaw_sp = att.yaw;
	cmd.alt_sp = pos.z_est[0];
}
void attitude_control(short dt)
{
	//R_sp 	world to desired body
	//att.R 		world to measured body
	int e_R_hat[3][3];//measured body to desired body
	int e_R[3], e_w[3];
	int w_sp[3];
//in c++	e_R_hat = (R_sp.transposed() * R - R.transposed() * R_sp) * (0.5f);
//	e_R_hat[0][0] = R_sp[0][0] * att.R[0][0] + R_sp[1][0] * att.R[1][0] + R_sp[2][0] * att.R[2][0];
	e_R_hat[0][1] = R_sp[0][0] * att.R[0][1] + R_sp[1][0] * att.R[1][1] + R_sp[2][0] * att.R[2][1];
	e_R_hat[0][2] = R_sp[0][0] * att.R[0][2] + R_sp[1][0] * att.R[1][2] + R_sp[2][0] * att.R[2][2];
	e_R_hat[1][0] = R_sp[0][1] * att.R[0][0] + R_sp[1][1] * att.R[1][0] + R_sp[2][1] * att.R[2][0];
//	e_R_hat[1][1] = R_sp[0][1] * att.R[0][1] + R_sp[1][1] * att.R[1][1] + R_sp[2][1] * att.R[2][1];
	e_R_hat[1][2] = R_sp[0][1] * att.R[0][2] + R_sp[1][1] * att.R[1][2] + R_sp[2][1] * att.R[2][2];	
	e_R_hat[2][0] = R_sp[0][2] * att.R[0][0] + R_sp[1][2] * att.R[1][0] + R_sp[2][2] * att.R[2][0];
	e_R_hat[2][1] = R_sp[0][2] * att.R[0][1] + R_sp[1][2] * att.R[1][1] + R_sp[2][2] * att.R[2][1];
//	e_R_hat[2][2] = R_sp[0][2] * att.R[0][2] + R_sp[1][2] * att.R[1][2] + R_sp[2][2] * att.R[2][2];
	
//	e_R_hat[0][0] -= att.R[0][0] * R_sp[0][0] + att.R[1][0] * R_sp[1][0] + att.R[2][0] * R_sp[2][0];
	e_R_hat[0][1] -= att.R[0][0] * R_sp[0][1] + att.R[1][0] * R_sp[1][1] + att.R[2][0] * R_sp[2][1];
	e_R_hat[0][2] -= att.R[0][0] * R_sp[0][2] + att.R[1][0] * R_sp[1][2] + att.R[2][0] * R_sp[2][2];
	e_R_hat[1][0] -= att.R[0][1] * R_sp[0][0] + att.R[1][1] * R_sp[1][0] + att.R[2][1] * R_sp[2][0];
//	e_R_hat[1][1] -= att.R[0][1] * R_sp[0][1] + att.R[1][1] * R_sp[1][1] + att.R[2][1] * R_sp[2][1];
	e_R_hat[1][2] -= att.R[0][1] * R_sp[0][2] + att.R[1][1] * R_sp[1][2] + att.R[2][1] * R_sp[2][2];
	e_R_hat[2][0] -= att.R[0][2] * R_sp[0][0] + att.R[1][2] * R_sp[1][0] + att.R[2][2] * R_sp[2][0];
	e_R_hat[2][1] -= att.R[0][2] * R_sp[0][1] + att.R[1][2] * R_sp[1][1] + att.R[2][2] * R_sp[2][1];
//	e_R_hat[2][2] -= att.R[0][2] * R_sp[0][2] + att.R[1][2] * R_sp[1][2] + att.R[2][2] * R_sp[2][2];
		
//	e_R_hat[0][0] = e_R_hat[0][0]>>15;
	e_R_hat[0][1] = e_R_hat[0][1]>>15;
	e_R_hat[0][2] = e_R_hat[0][2]>>15;
	e_R_hat[1][0] = e_R_hat[1][0]>>15;
//	e_R_hat[1][1] = e_R_hat[1][1]>>15;
	e_R_hat[1][2] = e_R_hat[1][2]>>15;
	e_R_hat[2][0] = e_R_hat[2][0]>>15;
	e_R_hat[2][1] = e_R_hat[2][1]>>15;
//	e_R_hat[2][2] = e_R_hat[2][2]>>15;
	
//in c++	math::Vector<3> e_R
	e_R[0] = (e_R_hat[1][2] - e_R_hat[2][1])>>1;//roll err in rad<<14
	e_R[1] = (e_R_hat[2][0] - e_R_hat[0][2])>>1;//pitch err in rad<<14
	e_R[2] = (e_R_hat[0][1] - e_R_hat[1][0])>>1;//yaw err in rad<<14
	
	w_sp[0] = external_err_pid(&rollPID, e_R[0], dt);//RollRate_sp
	w_sp[1] = external_err_pid(&pitchPID, e_R[1], dt);//PitchRate_sp
	w_sp[2] = external_err_pid(&yawPID, e_R[2], dt);//YawRate_sp
	
//	e_w = R.transposed * R_sp * w_sp - w
	
/*	e_w[0] = (R_sp[0][0] * w_sp[0] + R_sp[0][1] * w_sp[1] + R_sp[0][2] * w_sp[2])>>14;
	e_w[1] = (R_sp[1][0] * w_sp[0] + R_sp[1][1] * w_sp[1] + R_sp[1][2] * w_sp[2])>>14;
	e_w[2] = (R_sp[2][0] * w_sp[0] + R_sp[2][1] * w_sp[1] + R_sp[2][2] * w_sp[2])>>14;	
	
	e_w[0] = (att.R[0][0] * e_w[0] + att.R[1][0] * e_w[1] + att.R[2][0] * e_w[2])>>14;
	e_w[1] = (att.R[0][1] * e_w[0] + att.R[1][1] * e_w[1] + att.R[2][1] * e_w[2])>>14;
	e_w[2] = (att.R[0][2] * e_w[0] + att.R[1][2] * e_w[1] + att.R[2][2] * e_w[2])>>14;
	
	e_w[0] -= att.rollspeed;
	e_w[1] -= att.pitchspeed;
	e_w[2] -= att.yawspeed;
*/
	e_w[0] = w_sp[0] - att.rollspeed;
	e_w[1] = w_sp[1] - att.pitchspeed;
	e_w[2] = w_sp[2] - att.yawspeed;	
	output.roll = internal_err_pid(&rollPID, e_w[0], dt)>>4;
	output.pitch = internal_err_pid(&pitchPID, e_w[1], dt)>>4;
	output.yaw = internal_err_pid(&yawPID, e_w[2], dt)>>4;
	
}

int external_err_pid(PID *pid, int err, short dt)
{
 	 int retValue;								
 	 pid->Err = err;
 	 retValue = pid->P * err;
	 return retValue;
}
int internal_err_pid(PID *pid, int rate_err, short dt)
{
   	int rateOuput;								
  	pid->RateErr = rate_err;
  	rateOuput = pid->Prate * rate_err + pid->Drate * (rate_err - pid->l_RateErr) *1000/ dt + pid->Irate * pid->int_RateErr;
  	pid->l_RateErr = rate_err;
	pid->int_RateErr += rate_err * dt/1000;
  	return rateOuput;
}
int force2output(int force, unsigned int battery)
//converts a force in Newton to a output value between 0 and 1 (1.2)
//mNewton to 0~1024
{
	int k;
	if(battery == 0){
		return force*110/1087 + 10;
	} else{
		//full k=100, low k=120
		k = constrain(310-(int)battery/20,95,125);
		return force*k/1087 + 10;
	}
}
