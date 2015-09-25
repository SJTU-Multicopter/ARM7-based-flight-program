#include "attitude.h"
#include "math.h"
#include "global.h"
#include "IMU.h"
#include "timer.h"
volatile int q0=16384,q1=0,q2=0,q3=0;//<<14
#define Kp_ACC 2.0f //too much noise if  large, too slow to recover the drift if small
#define Ki_ACC 0.005f//0.005f	flutuates if too large, slow towards steady state if zero
#define Kp_MAG 5.0f
#define Ki_MAG 0.005f

void imu_update(void) //(1<<14)rad
{
	//angular parameters should use rad but not raw data
	float norm;
	int m[3],a[3],w[3];
	int v[3];
	int ex, ey, ez;
	int eMx,eMy,eMz;
	int h[3],b[3];
	static int exInt=0,eyInt=0,ezInt=0;
	static int eMxInt=0,eMyInt=0,eMzInt=0;
	int g_roll = att.rollspeed, g_pitch = att.pitchspeed, g_yaw = att.yawspeed;	       
	norm = inv_sqrt((int)sens.ax*sens.ax + sens.ay*sens.ay + sens.az*sens.az);       
	a[0] = ((int)sens.ax<<14)*norm;
	a[1] = ((int)sens.ay<<14)*norm;
	a[2] = ((int)sens.az<<14)*norm;      	
	norm = inv_sqrt((int)sens.mx*sens.mx + sens.my*sens.my + sens.mz*sens.mz);
	m[0]=((int)sens.mx<<14)*norm;
	m[1]=((int)sens.my<<14)*norm;
	m[2]=((int)sens.mz<<14)*norm;
//  body mag (m xyz) towards geo mag (b xyz)
	body2glob(m, h, 3);	
	b[0]=0;
	b[1]=sqrt(h[0]*h[0] + h[1]*h[1]);
    b[2]=h[2];
//	geo mag (b xyz) towards estimated body mag (w xyz)
	glob2body(w, b, 3);
//	estimated gravity direction (v xyz) is the last rol of R
	v[0] = att.R[2][0];
	v[1] = att.R[2][1];
	v[2] = att.R[2][2];
	// error is obtained by cross production
	//c = a % c
	//rc = (ra % rb)/r
	ex = (a[1]*v[2] - a[2]*v[1])>>14;
	ey = (a[2]*v[0] - a[0]*v[2])>>14;
	ez = (a[0]*v[1] - a[1]*v[0])>>14;
	eMx =(m[1]*w[2] - m[2]*w[1])>>14;
	eMy =(m[2]*w[0] - m[0]*w[2])>>14;
	eMz =(m[0]*w[1] - m[1]*w[0])>>14;   
	if(ex != 0 && ey != 0 && ez != 0){
	// integration term of error
		exInt += ex*smpl.halfT*Ki_ACC;
		eyInt += ey*smpl.halfT*Ki_ACC;
		ezInt += ez*smpl.halfT*Ki_ACC;
		eMxInt += eMx*smpl.halfT*Ki_ACC;
		eMyInt += eMy*smpl.halfT*Ki_ACC;
		eMzInt += eMz*smpl.halfT*Ki_ACC; 
	// corrected gyro data
	//err is in (<<14), g is in (<<14)
		g_roll += (Kp_ACC*ex + exInt/1000.0f + Kp_MAG*eMx + eMxInt/1000.0f);
		g_pitch += (Kp_ACC*ey + eyInt/1000.0f + Kp_MAG*eMy + eMyInt/1000.0f);
		g_yaw += (Kp_ACC*ez + ezInt/1000.0f + Kp_MAG*eMz + eMzInt/1000.0f);
	}
	// updating quarternion using first order approximation
	//q and g is in (<<14),  halfT in 1e-3
	q0 += ((-q1*g_roll - q2*g_pitch - q3*g_yaw)>>14)*smpl.halfT/1000;
	q1 += ((q0*g_roll + q2*g_yaw - q3*g_pitch)>>14)*smpl.halfT/1000;
	q2 += ((q0*g_pitch - q1*g_yaw + q3*g_roll)>>14)*smpl.halfT/1000;
	q3 += ((q0*g_yaw + q1*g_pitch - q2*g_roll)>>14)*smpl.halfT/1000;  
	// quarternion normalization
	norm = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 =(q0<<14)*norm;
	q1 =(q1<<14)*norm;
	q2 =(q2<<14)*norm;
	q3 =(q3<<14)*norm;
	//obtain the rotation matrix that can be applied elsewhere
	att.R[0][0]=(1<<14) - ((q2 * q2 + q3 * q3)>>13);
	att.R[0][1]=(q1 * q2 - q0 * q3)>>13;
	att.R[0][2]=(q1 * q3 + q0 * q2)>>13;
	att.R[1][0]=(q1 * q2 + q0 * q3)>>13;
	att.R[1][1]=(1<<14) - ((q1 * q1 + q3 * q3)>>13);
	att.R[1][2]=(q2 * q3 - q0 * q1)>>13;
	att.R[2][0]=(q1 * q3 - q0 * q2)>>13;
	att.R[2][1]=(q2 * q3 + q0 * q1)>>13;
	att.R[2][2]=(1<<14) - ((q1 * q1 + q2 * q2)>>13);
}

void attitude_compute(void)
{
	#define scale_gyr 7506 //correspond to 1 rad/s
	#define scale_acc_gravity 8192//correspond to 9.8 m/s2
	#define beta 199/200
	#define beta_v 19/20
	static int pitch_v=0,roll_v=0,yaw_v=0;   //in m_rad
	static short turn=0;
	static int l_yaw=0;
	int pitch,roll,yaw;  //in (1<<14)rad
	int ax_comp, ay_comp;
	imu_update();
	pitch = -asin(att.R[2][0]/16384.0f)*16384;
	roll  = atan2(att.R[2][1], att.R[2][2])*16384;
	yaw = -atan2(att.R[0][1], att.R[1][1])*16384;
	//yaw changes continuously without jumping from -180 to 180
	if(yaw < (-15565*PI) && l_yaw>(15565*PI))turn++;//0.95*16384=15565
	if(yaw > (15565*PI) && l_yaw<(-15565*PI))turn--;	
	l_yaw = yaw;
	yaw += turn*(32768*PI);
		
	pitch_v = ((int)sens.gx<<14)/scale_gyr;
	roll_v = ((int)sens.gy<<14)/scale_gyr;
	//high pass filter for angular velocity
	pitch_v = pitch_v - pitch_v * beta_v + ((int)sens.gy<<14)/scale_gyr * beta_v;   
	roll_v = roll_v - roll_v * beta_v + ((int)sens.gx<<14)/scale_gyr * beta_v;
	yaw_v = yaw_v - yaw_v * beta_v + ((int)sens.gz<<14)/scale_gyr * beta_v;
	//compensation filter for angles
	ax_comp = ((int)sens.ax<<14)/scale_acc_gravity;
	ay_comp = ((int)sens.ay<<14)/scale_acc_gravity;
	pitch = pitch * beta + pitch_v / smpl.attComputeRate * beta - ax_comp + ax_comp * beta;
	roll = roll * beta + roll_v / smpl.attComputeRate * beta + ay_comp - ay_comp * beta; 

	att.pitchspeed=pitch_v;
	att.rollspeed=roll_v;
	att.yawspeed=yaw_v;
	att.pitch=pitch;
	att.roll=roll;
	att.yaw=yaw;
	
//	yaw_v = sens.gz*1000/scale_gyr;	
}
float data_2_angle(float x, float y, float z)	 //in rad
{
	float res;
	res = atan2(x,sqrt(y*y+z*z));
	return res;
}
void quarternion_init(void)
{
	int i;
	long sum_ax=0,sum_ay=0,sum_az=0;
	long sum_mx=0,sum_my=0,sum_mz=0;
	short accx,accy,accz,magx,magy,magz; 
	float xh,yh;
	float pitch,roll,yaw; 
	for(i=0;i<50;i++){
		get_imu();
		sum_ax+=sens.ax;
		sum_ay+=sens.ay;
		sum_az+=sens.az;
		sum_mx+=sens.mx;
		sum_my+=sens.my;
		sum_mz+=sens.mz;
		delay_ms(8);
	}
	accx = sum_ax/50;
	accy = sum_ay/50;
	accz = sum_az/50;
	magx = sum_mx/50;
	magy = sum_my/50;
	magz = sum_mz/50;
	pitch = -data_2_angle(accx,accy,accz);//rad
	roll = data_2_angle(accy,accx,accz); //rad
	xh = magy*cos(roll)+magx*sin(roll)*sin(pitch)-magz*cos(pitch)*sin(roll);		 
	yh = magx*cos(pitch)+magz*sin(pitch);
	yaw = -atan2(xh,yh)+1.57f;

	if(yaw < -PI )
		yaw = yaw + 2*PI;
	if(yaw > PI )
		yaw = yaw - 2*PI;
	att.pitch = pitch*16384;
	att.roll = roll*16384;
	att.yaw = yaw*16384;
	q0 = (cos(0.5*roll)*cos(0.5*pitch)*cos(0.5*yaw) + sin(0.5*roll)*sin(0.5*pitch)*sin(0.5*yaw))*16384;  
	q1 = (sin(0.5*roll)*cos(0.5*pitch)*cos(0.5*yaw) - cos(0.5*roll)*sin(0.5*pitch)*sin(0.5*yaw))*16384;    
	q2 = (cos(0.5*roll)*sin(0.5*pitch)*cos(0.5*yaw) + sin(0.5*roll)*cos(0.5*pitch)*sin(0.5*yaw))*16384; 
	q3 = (cos(0.5*roll)*cos(0.5*pitch)*sin(0.5*yaw) - sin(0.5*roll)*sin(0.5*pitch)*cos(0.5*yaw))*16384;  
	att.R[0][0]=(1<<14) - ((q2 * q2 + q3 * q3)>>13);
	att.R[0][1]=(q1 * q2 - q0 * q3)>>13;
	att.R[0][2]=(q1 * q3 + q0 * q2)>>13;
	att.R[1][0]=(q1 * q2 + q0 * q3)>>13;
	att.R[1][1]=(1<<14) - ((q1 * q1 + q3 * q3)>>13);
	att.R[1][2]=(q2 * q3 - q0 * q1)>>13;
	att.R[2][0]=(q1 * q3 - q0 * q2)>>13;
	att.R[2][1]=(q2 * q3 + q0 * q1)>>13;
	att.R[2][2]=(1<<14) - ((q1 * q1 + q2 * q2)>>13);
	for(i=0;i<50;i++){
		get_imu();
		attitude_compute();
		delay_ms(7);
	}
}
float inv_sqrt(float x) 
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
void body2glob(int body[], int glob[], short dimension)
{	
	if(dimension == 2){
		glob[0] = body[0]*cos(att.yaw/16384.0) + body[1]*sin(-att.yaw/16384.0);
		glob[1] = body[0]*sin(att.yaw/16384.0) + body[1]*cos(att.yaw/16384.0); 
	}
	else if(dimension == 3){
		glob[0] = (body[0]*att.R[0][0] + body[1]*att.R[0][1] + body[2]*att.R[0][2])>>14;
		glob[1] = (body[0]*att.R[1][0] + body[1]*att.R[1][1] + body[2]*att.R[1][2])>>14;
		glob[2] = (body[0]*att.R[2][0] + body[1]*att.R[2][1] + body[2]*att.R[2][2])>>14;
	}
}
void glob2body(int body[], int glob[], short dimension)//body=inv(R)*glob
{	
	if(dimension == 2){
		body[0] = glob[0]*cos(att.yaw/16384.0) + glob[1]*sin(att.yaw/16384.0);
		body[1] = glob[0]*sin(-att.yaw/16384.0) + glob[1]*cos(att.yaw/16384.0); 
	}
	else if(dimension == 3){
		body[0] = (glob[0]*att.R[0][0] + glob[1]*att.R[1][0] + glob[2]*att.R[2][0])>>14;
		body[1] = (glob[0]*att.R[0][1] + glob[1]*att.R[1][1] + glob[2]*att.R[2][1])>>14;
		body[2] = (glob[0]*att.R[0][2] + glob[1]*att.R[1][2] + glob[2]*att.R[2][2])>>14;
	}
}
