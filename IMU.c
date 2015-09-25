#include "global.h"
#include "IMU.h"
#include "timer.h"
#include "mytwi_new.h"
#include "compass_on_gps.h"
short maxMagX=0,maxMagY=0,maxMagZ=0;
short minMagX=0,minMagY=0,minMagZ=0;
char testing;
#if MY_BOARD
short gyr_x_bias=1;
short gyr_y_bias=-30;
short gyr_z_bias=-15;
short acc_x_bias=204;
short acc_y_bias=61;
short acc_z_bias=-578;
short mag_x_bias=-81+65;
short mag_y_bias=-50;
short mag_z_bias=39-65;
#elif FANGS_BOARD
short gyr_x_bias=-102;
short gyr_y_bias=10;
short gyr_z_bias=-91;
short acc_x_bias=409/2;
short acc_y_bias=136/2;
short acc_z_bias=(-466-498+2)/2;
short mag_x_bias=57;
short mag_y_bias=8;
short mag_z_bias=-34;
#elif NEW_BOARD
short gyr_x_bias=0;
short gyr_y_bias=0;
short gyr_z_bias=0;
short acc_x_bias=98;
short acc_y_bias=-51;
short acc_z_bias=-851;
#if MPU_COMPASS
short mag_x_bias=-80;
short mag_y_bias=-11;
short mag_z_bias=8;
#elif HMC_COMPASS
short mag_x_bias=-130;
short mag_y_bias=-100;
short mag_z_bias=-35;
#endif
#endif



void compass_calibration_xy(void)
{
	while(1){//wait for init calibrating
		if(testing){
			mag_x_bias=0;
			mag_y_bias=0;
			maxMagX=0,maxMagY=0,maxMagZ=0;
			minMagX=0,minMagY=0,minMagZ=0;
			while(testing){
				get_compass_bias();
			}
			compass_calibration_conclude_xy();
			break;
		}
	}
}
void compass_calibration_z(void)
{
	while(1){//wait for init calibrating
		if(testing){
			mag_z_bias=0;
			maxMagX=0,maxMagY=0,maxMagZ=0;
			minMagX=0,minMagY=0,minMagZ=0;
			while(testing){
				get_compass_bias();
			}
			compass_calibration_conclude_z();
			break;
		}
	}
}
void get_compass_bias(void)
{
	get_imu();
	if(sens.mx>maxMagX)//max
		maxMagX=sens.mx;
	if(sens.mx<minMagX)//min
		minMagX=sens.mx;
	if(sens.my>maxMagY)//max
		maxMagY=sens.my;
	if(sens.my<minMagY)//min
		minMagY=sens.my;
	if(sens.mz>maxMagZ)//max
		maxMagZ=sens.mz;
	if(sens.mz<minMagZ)//min
		minMagZ=sens.mz;		
}
void compass_calibration_conclude_xy(void)
{
	short MXgain,MYgain;
	if((maxMagX - minMagX) >= (maxMagY - minMagY))
	{
		MXgain = 1;
		MYgain = (maxMagX - minMagX) / (maxMagY - minMagY);
	}
	else{ //(maxMagY - minMagY) > (maxMagX - minMagX)
		MXgain = (maxMagY - minMagY) / (maxMagX - minMagX);
		MYgain = 1;
  	}
	mag_x_bias = -0.5 * MXgain * (maxMagX + minMagX);
	mag_y_bias = -0.5 * MYgain * (maxMagY + minMagY);
}
void compass_calibration_conclude_z(void)
{
	short MZgain;
	if(((maxMagX - minMagX) >= (maxMagY - minMagY)) 
		&& ((maxMagX - minMagX) >= (maxMagZ - minMagZ)))
		MZgain = (maxMagX - minMagX) / (maxMagZ - minMagZ);	 
	else if(((maxMagY - minMagY) > (maxMagX - minMagX)) 
	&& ((maxMagY - minMagY) >= (maxMagZ - minMagZ)))
		MZgain = (maxMagY - minMagY) / (maxMagZ - minMagZ);
	else if(((maxMagZ - minMagZ) > (maxMagX - minMagX)) 
	&& ((maxMagZ - minMagZ) > (maxMagY - minMagY)))
		MZgain = 1.0;
	mag_z_bias = -0.5 * MZgain * (maxMagZ + minMagZ);
}
void gyro_calibration(void)
{
	long temp1=0,temp2=0,temp3=0;
	int i;
	gyr_x_bias=0;gyr_y_bias=0;gyr_z_bias=0;
	for(i=0;i<200;i++)
	{
		get_imu();
		temp1+=sens.gx;
		temp2+=sens.gy;
		temp3+=sens.gz;
		delay_ms(2);
	}
	gyr_x_bias=-temp1/200;
	gyr_y_bias=-temp2/200;
	gyr_z_bias=-temp3/200;
}
void get_imu(void)
{
	typedef union myun{
		char b[2];
		short c;
	}un;
	un gx_get,gy_get,gz_get;
	un ax_get,ay_get,az_get;
	un mx_get,my_get,mz_get; 
	char IMU_received[12];
	char compass_received[6];
	short temp;
	short gx,gy,gz,ax,ay,az,mx,my,mz;
	int compass_available;
	twi_read_all(IMU_received);
#if MPU_COMPASS
	compass_available = twi_readcompass(compass_received);
#elif HMC_COMPASS
	compass_available = hmc5883_read(compass_received);
#endif
	ax_get.b[1] = IMU_received[0];
	ax_get.b[0] = IMU_received[1];
	ay_get.b[1] = IMU_received[2];
	ay_get.b[0] = IMU_received[3];
	az_get.b[1] = IMU_received[4];
	az_get.b[0] = IMU_received[5];
	gx_get.b[1] = IMU_received[6];
	gx_get.b[0] = IMU_received[7];
	gy_get.b[1] = IMU_received[8];
	gy_get.b[0] = IMU_received[9];
	gz_get.b[1] = IMU_received[10];
	gz_get.b[0] = IMU_received[11];
#if MPU_COMPASS
	mx_get.b[0] = compass_received[0];
	mx_get.b[1] = compass_received[1];
	my_get.b[0] = compass_received[2];
	my_get.b[1] = compass_received[3];
	mz_get.b[0] = compass_received[4];
	mz_get.b[1] = compass_received[5];
#elif HMC_COMPASS
	mx_get.b[1] = compass_received[0];
	mx_get.b[0] = compass_received[1];
	my_get.b[1] = compass_received[2];
	my_get.b[0] = compass_received[3];
	mz_get.b[1] = compass_received[4];
	mz_get.b[0] = compass_received[5];
#endif
	ax = ax_get.c;
	ay = ay_get.c;
	az = az_get.c;
	gx = gx_get.c;
	gy = gy_get.c;
	gz = gz_get.c;
	mx = mx_get.c;
	my = my_get.c;
	mz = mz_get.c;
#if MPU_COMPASS
	temp = my;				//mag sensors have different reference
	my = mx;
	mx = temp;
	mz = -mz;
#elif HMC_COMPASS
	temp = mz;
	mz = -my;	
	my = -mx;
	mx = -temp;
#endif	
	sens.gx = gx + gyr_x_bias;
	sens.gy = gy + gyr_y_bias;
	sens.gz = gz + gyr_z_bias;
	sens.ax = ax + acc_x_bias;
	sens.ay = ay + acc_y_bias;
	sens.az = az + acc_z_bias;
	if(compass_available){
		sens.mx = ((mx + mag_x_bias)<<10)/546;
		sens.my = ((my + mag_y_bias)<<10)/523;
		sens.mz = ((mz + mag_z_bias)<<10)/443;
	}
}
