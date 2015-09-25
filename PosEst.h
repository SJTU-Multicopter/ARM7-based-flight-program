#ifndef FILTER_H
#define FILTER_H
float kalman_filter(float ResrcData,float ProcessNiose_Q,float MeasureNoise_R,short channel);
void inertial_filter_predict(short dt, int x[2], int acc);
void inertial_filter_correct(int e, short dt, int x[2], char i, float w);
#if OUTDOOR
void gps_pos_init(void);
void gps2xyz(void);
void gps_pos_corr(short dt);
#elif INDOOR
void vicon_pos_init(void);
void vicon_pos_corr(short dt);
#endif

void corr_geo_acc(int avgGlobAcc[3]);
void get_geo_acc(int globAcc[3]);
void pos_predict(short dt);
void baro_pos_corr(short dt);
void sonar_pos_corr(short dt);

#define BARO_FILT 0
#define ACC_Z_FILT 1
#define ACC_Y_FILT 2
#define ACC_X_FILT 3
#endif
