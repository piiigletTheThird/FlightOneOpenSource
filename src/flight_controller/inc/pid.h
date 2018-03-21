#pragma once

#define AXIS_NUMBER 3
#define VECTOR_NUMBER 3
#define KD_RING_BUFFER_SIZE 256

#define MAX_KP_FM1 0.90f
#define MAX_KI_FM1 0.30f
#define MAX_KD_FM1 0.55f

#define MAX_KP 0.75f
#define MAX_KI 0.25f
#define MAX_KD 0.30f


/*
set yaw_kp1=19.000
set roll_kp1=26.000
set pitch_kp1=27.000
set yaw_ki1=30.000
set roll_ki1=30.000
set pitch_ki1=30.000
set yaw_kd1=50.000
set roll_kd1=18.000
set pitch_kd1=26.000
*/

#define DEFAULT_PID_CONFIG_VALUE 30.0f

#define DEFAULT_ROLL_KP          112.0f
#define DEFAULT_PITCH_KP         126.2f
#define DEFAULT_YAW_KP           095.0f

#define DEFAULT_ROLL_KI          816.5f
#define DEFAULT_PITCH_KI         857.1f
#define DEFAULT_YAW_KI           909.1f

#define DEFAULT_ROLL_KD          0952.0f
#define DEFAULT_PITCH_KD         1130.4f
#define DEFAULT_YAW_KD           1315.8f

//#define DEFAULT_YAW_KP           154.0f
//#define DEFAULT_ROLL_KP          150.0f
//#define DEFAULT_PITCH_KP         147.0f

//#define DEFAULT_YAW_KI           888.0f
//#define DEFAULT_ROLL_KI          740.0f
//#define DEFAULT_PITCH_KI         840.0f

//#define DEFAULT_YAW_KD           1100.0f
//#define DEFAULT_ROLL_KD          1470.0f
//#define DEFAULT_PITCH_KD         1500.0f


enum
{
    KP = 0,
    KD = 1,
    KI = 2,
};

typedef struct
{
    float kp;
    float ki;
    float kd;
} pid_output;

typedef struct {
    float kp;
    float ki;
    float kd;
    uint32_t wc;
    uint32_t ga;
    float slp;
    float sli;
    float sla;
    float sld;
    float kdLimit;
    float kiLimit;
} pid_terms;

extern volatile float kiTrim[AXIS_NUMBER];
extern float pidSetpoint[AXIS_NUMBER];    //3 axis for pidc. range is in DPS.
extern pid_output pids[AXIS_NUMBER];
extern float currentKdFilterConfig[AXIS_NUMBER];

void InitPid (void);
//void InlineInitPidFilters(void);
uint32_t InlinePidController (float filteredGyroData[], float flightSetPoints[], pid_output flightPids[], float actuatorRange);
uint32_t SpinStopper(int32_t axis, float pidError);
void InlineUpdateWitchcraft(pid_terms pidConfig[]);


/*


set yaw_ga=4
set roll_ga=0
set pitch_ga=0

set slp=5.000
set sli=0.100
set sla=35.000
set sld=0.030

set filter_mode0=0
set filter_mode1=0
set filter_mode2=2

set yaw_quick=40.000
set yaw_rap=88.000
set roll_quick=70.000
set roll_rap=88.000
set pitch_quick=70.000
set pitch_rap=88.000
set yaw_kd_rap=90.000
set roll_kd_rap=90.000
set pitch_kd_rap=90.000
set x_vector_quick=2.000
set x_vector_rap=25.000
set y_vector_quick=2.000
set y_vector_rap=25.000
set z_vector_quick=2.000
set z_vector_rap=25.000

set rx_protocol=2
set rx_usart=0
set arm_method=1
set rx_inv_direction=0
set pitch_deadband=0.004
set roll_deadband=0.004
set yaw_deadband=0.005
set throttle_deadband=0.000
set aux1_deadband=0.000
set aux2_deadband=0.000
set aux3_deadband=0.000
set aux4_deadband=0.000
set pitch_midrc=1032
set roll_midrc=1029
set yaw_midrc=1029
set throttle_midrc=1030
set aux1_midrc=1028
set aux2_midrc=1028
set aux3_midrc=1028
set aux4_midrc=1028
set pitch_minrc=230
set roll_minrc=212
set yaw_minrc=207
set throttle_minrc=291
set aux1_minrc=207
set aux2_minrc=207
set aux3_minrc=207
set aux4_minrc=207
set pitch_maxrc=1823
set roll_maxrc=1832
set yaw_maxrc=1838
set throttle_maxrc=1770
set aux1_maxrc=1838
set aux2_maxrc=1838
set aux3_maxrc=1838
set aux4_maxrc=1838
set pitch_map=2
set roll_map=1
set yaw_map=3
set throttle_map=0
set aux1_map=4
set aux2_map=5
set aux3_map=6
set aux4_map=7
set aux5_map=8
set aux6_map=9
set aux7_map=10
set aux8_map=11
set aux9_map=12
set aux10_map=13
set aux11_map=14
set aux12_map=15
set rc_calibrated=1
set stick_curve=4
set throttle_curve=0
set aux1_curve=0
set aux2_curve=0
set aux3_curve=0
set aux4_curve=0
set pitch_expo=58.000
set roll_expo=58.000
set yaw_expo=58.000
set throttle_expo=0.000
set aux1_expo=0.000
set aux2_expo=0.000
set aux3_expo=0.000
set aux4_expo=0.000
set bind=0
set pitch_rate=350.000
set roll_rate=350.000
set yaw_rate=350.000
set pitch_acrop=175.000
set roll_acrop=197.000
set yaw_acrop=88.000
set drunk=0.010
set skunk=50.000
*/