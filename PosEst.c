#include "PosEst.h"
#include "global.h"
#include "math.h"
#include "attitude.h"
#define baro_weight 0.0//1.0//0.7
#define sonar_weight 1.0
#if OUTDOOR
#define gps_xy_weight 1.0
#define gps_z_weight 0.0
#elif INDOOR
#define vicon_xy_weight 5.0
#define vicon_z_weight 5.0
#endif
int accStaticCorr[3] = {0,0,0};
#if OUTDOOR
int home_lat, home_lon, home_gps_alt;
void gps_pos_init(void)
{
	home_lat = gps.lat;
	home_lon = gps.lon;
	home_gps_alt = gps.alt;
	pos.x_est[0] = 0;
	pos.y_est[0] = 0;
	pos.x_est[1] = 0;
	pos.y_est[1] = 0;
	cmd.pos_x_sp = 0;
	cmd.pos_y_sp = 0;
}
void gps2xyz(void)
{
	double relative_lat_rad = (gps.lat - home_lat) / 3508.0;//*1/1000000 * DEG2RAD*16384;
	double relative_lon_rad = (gps.lon - home_lon) / 3508.0;
	gps.y = (int)(relative_lat_rad * EARTH_RADIAS*1000)>>14;
	gps.x = relative_lon_rad * EARTH_RADIAS * cos(home_lat /57471264.0)*1000;
	gps.z = (gps.alt - home_gps_alt)*1000;
}
void gps_pos_corr(short dt)
{
	int corr_x, corr_y, corr_z;
	short corr_vx, corr_vy, corr_vz;
	static int l_x=0,l_y=0,l_z=0;
	static char start = 1;
	if(start == 1){
		l_x = gps.x;
		l_y = gps.y;	
		l_z = gps.z;
		start = 0;
	}	
	gps2xyz();
	corr_x = gps.x - pos.x_est[0];
	corr_y = gps.y - pos.y_est[0];
	corr_z = gps.z - pos.z_est[0];
	inertial_filter_correct(corr_x, dt, pos.x_est, 0, gps_xy_weight);
	inertial_filter_correct(corr_y, dt, pos.y_est, 0, gps_xy_weight);
	inertial_filter_correct(corr_z, dt, pos.z_est, 0, gps_z_weight);
	gps.vx = (gps.x - l_x) *1000/ dt;
	gps.vy = (gps.y - l_y) *1000/ dt;
	gps.vz = (gps.z - l_z) *1000/ dt;
	//compare with gps.vel
	l_x=gps.x;
	l_y=gps.y;
	l_z=gps.z;			
	corr_vx = gps.vx - pos.x_est[1];
	corr_vy = gps.vy - pos.y_est[1];
	corr_vz = gps.vz - pos.z_est[1];
	inertial_filter_correct(corr_vx, dt, pos.x_est, 1, gps_xy_weight);
	inertial_filter_correct(corr_vy, dt, pos.y_est, 1, gps_xy_weight);
	inertial_filter_correct(corr_vz, dt, pos.z_est, 1, gps_z_weight);
}
#elif INDOOR
void vicon_pos_init(void)
{
	pos.x_est[0] = vicon.x;
	pos.y_est[0] = vicon.y;
	pos.z_est[0] = vicon.z;
	pos.x_est[1] = 0;
	pos.y_est[1] = 0;
	pos.z_est[1] = 0;
}
void vicon_pos_corr(short dt)
{
	short corr_x, corr_y, corr_z;
	short corr_vx, corr_vy, corr_vz;
	static short l_x=0,l_y=0,l_z=0;
	static char start = 1;
	if(start == 1){
		l_x = vicon.x;
		l_y = vicon.y;	
		l_z = vicon.z;
		start = 0;
	}
	vicon.corr_flag = 0;
	corr_x = vicon.x - pos.x_est[0];
	corr_y = vicon.y - pos.y_est[0];
	corr_z = vicon.z - pos.z_est[0];
	inertial_filter_correct(corr_x, dt, pos.x_est, 0, vicon_xy_weight);
	inertial_filter_correct(corr_y, dt, pos.y_est, 0, vicon_xy_weight);
	inertial_filter_correct(corr_z, dt, pos.z_est, 0, vicon_z_weight);
	
	vicon.vx = (vicon.x - l_x) *1000/ dt;
	vicon.vy = (vicon.y - l_y) *1000/ dt;
	vicon.vz = (vicon.z - l_z) *1000/ dt;
	l_x=vicon.x;
	l_y=vicon.y;
	l_z=vicon.z;			
	corr_vx = vicon.vx - pos.x_est[1];
	corr_vy = vicon.vy - pos.y_est[1];
	corr_vz = vicon.vz - pos.z_est[1];
	inertial_filter_correct(corr_vx, dt, pos.x_est, 1, vicon_xy_weight);
	inertial_filter_correct(corr_vy, dt, pos.y_est, 1, vicon_xy_weight);
	inertial_filter_correct(corr_vz, dt, pos.z_est, 1, vicon_z_weight);			
}
#endif

void corr_geo_acc(int avgAcc[3])
{
	int i;
	for(i=0;i<3;i++)
		accStaticCorr[i] = -avgAcc[i];
}
void get_geo_acc(int globAcc[3])
{
	int bdyAcc[3]={0,0,0};
	short i;
	//加速度计量程必须能容得下振动的峰值，否则算出加速度一直低于0
	bdyAcc[0] = sens.ax * GRAVITY / 8192;
	bdyAcc[1] = sens.ay * GRAVITY / 8192;
	bdyAcc[2] = sens.az * GRAVITY / 8192;
	for(i=0;i<3;i++)
		bdyAcc[i] += accStaticCorr[i];
	body2glob(bdyAcc, globAcc, 3);
	globAcc[2] -= GRAVITY;
}
void pos_predict(short dt)
{
	int globAcc[3]={0,0,0};
	get_geo_acc(globAcc);
	pos.Acc_x = globAcc[0];
	pos.Acc_y = globAcc[1];
	pos.Acc_z = globAcc[2];
	inertial_filter_predict(dt, pos.x_est, pos.Acc_x);
	inertial_filter_predict(dt, pos.y_est, pos.Acc_y);
	inertial_filter_predict(dt, pos.z_est, pos.Acc_z);
}
void baro_pos_corr(short dt)//this has same freq as baro update
{
//	float dt = 1.0 / smpl.BaroRate * 11;
	int corr_z_baro =  baro.alt - pos.z_est[0];	
	inertial_filter_correct(corr_z_baro, dt, pos.z_est, 0, baro_weight);
}
void sonar_pos_corr(short dt)//this has same freq as sonar update
{
	int corr_z_sonar;
	if(pos.sonarPos < 200){//ground effect avoid baro
		corr_z_sonar =  pos.sonarPos - pos.z_est[0];
	}
	else{
		corr_z_sonar = 0;
	}	
	inertial_filter_correct(corr_z_sonar, dt, pos.z_est, 0, sonar_weight);
}

void inertial_filter_predict(short dt, int x[2], int acc)//dt in ms, x in mm and mm/s, acc in mm/s2
{
	x[0] += x[1] * dt / 1000 + acc * dt /1000 * dt / 2000;
	x[1] += acc * dt / 1000;	
}
void inertial_filter_correct(int e, short dt, int x[2], char i, float w)//e in mm
{
	int ewdt = e * w * dt / 1000;
	x[i] += ewdt;
	if (i == 0){
		x[1] += w * ewdt;
	}
}
float kalman_filter(float ResrcData,float ProcessNiose_Q,float MeasureNoise_R,short channel)
{
	float R = MeasureNoise_R;
    float Q = ProcessNiose_Q;
	static float x_last[5];
	float x_mid;
	float x_now;
	static float p_last[5];
	float p_mid;
	float p_now;
	float kg;        
	if(channel>4||channel<0)
		return -1;
    x_mid=x_last[channel];       //x_last=x(k-1|k-1),x_mid=x(k|k-1)
    p_mid=p_last[channel]+Q;     //p_mid=p(k|k-1),p_last=p(k-1|k-1)
    kg=p_mid/(p_mid+R); //
    x_now=x_mid+kg*(ResrcData-x_mid);
                
    p_now=(1-kg)*p_mid;     
    p_last[channel] = p_now; 
    x_last[channel] = x_now; 
    return x_now;
}
