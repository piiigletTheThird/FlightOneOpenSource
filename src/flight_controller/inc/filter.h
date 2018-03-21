#pragma once

#include <arm_math.h>

#define ATTENUATION_CURVE_SIZE 9

//Single axis kalman filter
typedef struct
{
	float i[4]; //identity matrix
	arm_matrix_instance_f32 armMatI; //process noise covariance
	float q[4]; //process noise covariance
	arm_matrix_instance_f32 armMatQ; //process noise covariance
	float r[4]; //measurement noise covariance
	arm_matrix_instance_f32 armMatR; //measurement noise covariance
	float x[2]; //value
	arm_matrix_instance_f32 armMatX; //measurement noise covariance
	float lastX[2]; //value
	arm_matrix_instance_f32 armMatLastX; //measurement noise covariance
	float y[2]; //value
	arm_matrix_instance_f32 armMatY; //measurement input matrix
	float temp2x1a[2]; //value
	arm_matrix_instance_f32 armMatTemp2x1a; //measurement noise covariance
	float temp2x1b[2]; //value
	arm_matrix_instance_f32 armMatTemp2x1b; //measurement noise covariance
	float p[4]; //estimation error covariance matrix
	arm_matrix_instance_f32 armMatP;
	float lastP[4]; //estimation error covariance matrix
	arm_matrix_instance_f32 armMatLastP;
	float a[4]; //estimation error covariance matrix
	arm_matrix_instance_f32 armMatA;
	float aT[4]; //estimation error covariance matrix
	arm_matrix_instance_f32 armMatAT;
	float temp4x4a[4]; //estimation error covariance matrix
	arm_matrix_instance_f32 armMatTemp4x4a;
	float temp4x4b[4]; //estimation error covariance matrix
	arm_matrix_instance_f32 armMatTemp4x4b;
	float k[4]; //paf gain
	arm_matrix_instance_f32 armMatK;
	float lastMeasurment; //standard deviation counter
	float output; //paf gain
} kalman_state;

typedef struct 
{
	float x;
	float k;
	float r;
} kd_filter;

//Single axis paf filter
typedef struct {
	float q; //process noise covariance
	float r; //measurement noise covariance
	float x; //value
	float lastX; //value
	float lastP; //value
	float tempP; //value
	float p; //estimation error covariance
	float k; //paf gain
	float output;
} paf_state;

//config structure which is loaded by config
typedef struct
{
    float q;
    float r;
    float p;
} paf_filter_config_record;

typedef struct
{
	int						 resRedux;
	int                      filterMod;
	int                      filterType;
	int                      ga;
	int                      wc;
	float                    gaMultiplier;
/*
	float                    omega0;
	float                    omega1;
	float                    omega2;
	float                    omega3;*/
	paf_filter_config_record kd;
	paf_filter_config_record gyro;
	paf_filter_config_record acc;
	float                    throttleCurve[ATTENUATION_CURVE_SIZE];
	float                    tpaKpCurve[ATTENUATION_CURVE_SIZE];
	float                    tpaKiCurve[ATTENUATION_CURVE_SIZE];
	float                    tpaKdCurve[ATTENUATION_CURVE_SIZE];
	int                      tpaKpCurveType;
	int                      tpaKiCurveType;
	int                      tpaKdCurveType;
} filter_device;

typedef struct
{
    float a0, a1, a2, a3, a4;
    float x1, x2, y1, y2;
} biquad_state;

typedef struct
{
	float state;
	float rC;
	float dT;
} lpf_state;

typedef struct
{
	//arm_matrix_instance_f32 x; //state matrix
	//arm_matrix_instance_f32 a; //a matrix
	//arm_matrix_instance_f32 u; //control variable matrix
	//arm_matrix_instance_f32 b; //b matrix
	//arm_matrix_instance_f32 w; //w matrix
	//arm_matrix_instance_f32 p; //Process variance/covariance matrix
	//arm_matrix_instance_f32 k; //Kalman gain matrix
	//arm_matrix_instance_f32 k; //Kalman gain matrix
	//arm_matrix_instance_f32 h; //h matrix
	float xOld[6];
	float x[6];
	float y[6];
	float z[6];
	float aTx_matrix[6];
	float aTp_matrix[6];
	float aTranpose_matrix[6];
	float bTu_matrix[18];
	float u[3];
	float a_matrix[36];
	float b_matrix[18];
	float q_matrix[36];
	float k_matrix[36];
	float r_matrix[36];
	float p_matrix[36];
	float p_matrix_temp[36];
} kalman_test_state;

extern kalman_state kalmanState;
extern paf_state pafGyroStates[];

extern void OldInitPaf(paf_state *state, float q, float r, float p, float intial_value);
extern void OldPafUpdate(paf_state *state, float measurement);

extern void InitKdFilter(kd_filter *kdFilter);
extern void KdFilterUpdate(kd_filter *kdFilter, float measurement);

extern void InitPaf(paf_state *pafState, float q, float r, float p, float intial_value);
extern void PafUpdate(paf_state *state, float measurement);

extern void  InitBiquad(float filterCutFreq, biquad_state *newState, float refreshRateSeconds, uint32_t filterType, biquad_state *oldState, float bandwidth);
extern float BiquadUpdate(float sample, biquad_state *bQstate);
extern void  LpfInit(lpf_state *filter, float frequencyCut, float refreshRateSeconds);
extern float LpfUpdate(float input, lpf_state *filter);

#define M_LN2_FLOAT	0.69314718055994530942f
#define M_PI_FLOAT	3.14159265358979323846f
#define BIQUAD_BANDWIDTH 1.92f

#define FILTER_TYPE_LOWPASS  0
#define FILTER_TYPE_NOTCH    1
#define FILTER_TYPE_PEEK     2
#define FILTER_TYPE_HIGHPASS 3
