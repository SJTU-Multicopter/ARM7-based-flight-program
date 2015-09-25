#ifndef ATTITUDE_H
#define ATTITUDE_H
void imu_update(void);
void attitude_compute(void);
float data_2_angle(float x, float y, float z);
void quarternion_init(void);
float inv_sqrt(float x);
void body2glob(int body[], int glob[], short dimension);
void glob2body(int body[], int glob[], short dimension);//body=inv(R)*glob
#endif
