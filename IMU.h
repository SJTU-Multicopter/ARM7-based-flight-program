#ifndef IMU_H
#define IMU_H
void get_compass_bias(void);
void compass_calibration_conclude_xy(void);
void compass_calibration_conclude_z(void);
extern void compass_calibration_xy(void);
extern void compass_calibration_z(void);
extern void gyro_calibration(void);
extern void get_imu(void);
#endif
