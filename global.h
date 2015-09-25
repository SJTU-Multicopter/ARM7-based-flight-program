#ifndef GLOBAL_H
#define GLOBAL_H

#define PLUS 0
#define CROSS 1
#define MY_BOARD 0
#define FANGS_BOARD 0
#define NEW_BOARD 1
#define F330 1
#define F240 0
#define INDOOR 1
#define OUTDOOR 0
#define XBEE_INT 0
#define XBEE_DMA 1
#define HMC_COMPASS 1
#define MPU_COMPASS 0
#define ALT_INCREAMENTAL_PID 0
#define ALT_POSITIONAL_PID 1
#define ORIGINAL_FREQ 1
#define DOUBLED_FREQ 0
#define EULER_CTRL_PID 0
#define R_CTRL_PID 1
#if OUTDOOR
struct gps{
	double lat;
	double lon;
	float alt;
	float vel;
	float azm; //azimuth, degrees
	short sat;
	short status;
	char gpsflag;
	int x;
	int y;
	int z;
	#define GPS_PERIOD 200
};
extern struct gps gps;
#elif INDOOR
struct vicon{
	int x;//mm
	int y;
	int z;
	int vx;//mm/s
	int vy;
	int vz;
	char corr_flag;
	#define VICON_PERIOD 100//ms
};
extern struct vicon vicon;
#endif
struct smpl{
	 short attComputeRate;
	 short UARTSendRate;
	 short BaroRate;
	 short RadioRate;
	 volatile char attComputeNow;
	 volatile char UARTSendNow;
	 volatile char BaroNow;
	 volatile char RadioNow;
	 long attComputeCount;
	 long UARTSendCount;
	 long BaroCount;
	 long RadioCount;
	 long ViconCount;
	 short halfT;
	 short baroStep;
	 short UARTStep;
	 char xbeeflag;
};
extern struct smpl smpl;

struct att {
	int pitch;
	int roll;
	int yaw;
	int pitchspeed;
	int rollspeed;
	int yawspeed;
	int R[3][3];//glob=R*body//1 ~ 2^14 ~ 16384
};
extern struct att att;

struct sens {
	short gx;
	short gy;
	short gz;
	short ax;
	short ay;
	short az;
	short mx;
	short my;
	short mz;	 
};
extern struct sens sens;

struct mode {
	char FlightMode;
	char CalibrationMode;
	char l_FlightMode;
	#define MANUEL 1
	#define ACROBATIC 0
	#define ALT_CTRL 2
	#define ALT_RATE_CTRL 3
	#define POS_CTRL 4
	#define POS_RATE_CTRL 6
	#define XY_CTRL 5
	#define MOTOR_CUT 9
};
extern struct mode mode;

struct baro {
	float temp;
	float pressure;
	float refPressure;	 
	int alt; 
};
extern struct baro baro;

struct pos {	 //unit mm
	int x_est[2];
	int y_est[2];
	int z_est[2];
	int Acc_x;
	int Acc_y;
	int Acc_z;
	int sonarPos;	
};
extern struct pos pos;

struct cmd {
	short rc[9];//-1024 ~ +1024
	int pitch_sp;
	int roll_sp;
	int yaw_sp;
	int alt_sp;
	int altRate_sp;		
	int pos_x_sp;
	int pos_y_sp;
	int Thrust;//1 mNewton//0~1024
	char SonarEnable;
	char data2send;
	#define SonarOFF 0
	#define SonarON 1
	#define sendNON 0
	#define sendSENS 1
	#define sendGPS 2
	#define sendATT 3
	#define sendPOS 4
	#define sendPID 5
	#define sendCMD 6
	#define sendOUT 7
	#define KILL 11
};
extern struct cmd cmd;

struct output {
	int pitch;//+-1024//U1=(F3-F1)L
	int roll;//+-1024//U2=(F2-F4)L
	int yaw;//+-1024//U4=M2+M4-M1-M3
	int thrust;//+-1024//U1=F1+F2+F3+F4
	int ref_thrust;
};
extern struct output output;
extern short data2[9];
typedef struct myPID {
	int Err;
	int RateErr;
	int l_RateErr;
	int int_RateErr;
	float P;		
	float Prate;
	float Irate;
	float Drate;
}PID;


extern PID rollPID;
extern PID pitchPID;
extern PID yawPID;

extern PID altPID;

extern PID pos_xPID;
extern PID pos_yPID;



#define RAD2DEG 57.3
#define DEG2RAD 0.0174
#define PI 3.1416
#define EARTH_RADIAS 6371000
#define GRAVITY 9810
#define HALF_SQRT_2 0.7071


#define yawRateCmndRatio 4.0 //max 50 deg/sec
#define AltRateCmndRatio 0.01//m/s
#if F330
	#define thrCmndRatio 19/10
#elif F240
	#define thrCmndRatio 2.6
#endif
#define MAX_ATT_SP 5718//20deg,0.349rad
#define MAX_ATT_MANUEL 11437//40deg,0.698rad
#define MAX_ALT_VEL 1000
#define MAX_XY_VEL 1500
#define MAX_YAW_RATE 14303//50.0
#define MAX_ALT_RATE 500
#define MIN_THRUST 154//0.15*1024
#define MAX_THRUST 1331//1.3
#define MID_THROTLE 717//0.7
#if F330
	#define VEHICLE_MASS 950//g
#elif F240
	#define VEHICLE_MASS 650
#endif
extern int constrain(int a, int b, int c);
extern int dead_zone(int a, int b);

#endif
