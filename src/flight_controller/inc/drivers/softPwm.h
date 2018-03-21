#pragma once

typedef enum
{
    CAM_ENTER,
    CAM_LEFT,
    CAM_UP,
    CAM_RIGHT,
    CAM_DOWN,
} cam_key_t;

typedef enum
{
	SPT_0_0 = 0,
	SPT_0_3 = 1,
	SPT_0_6 = 2,
	SPT_0_9 = 3,
	SPT_1_2 = 4,
	SPT_1_5 = 5,
	SPT_1_8 = 6,
	SPT_2_1 = 7,
	SPT_2_4 = 8,
	SPT_2_7 = 9,
	SPT_3_0 = 10,
	SPT_3_3 = 11,
} software_pwm_voltage_t;

typedef enum
{
    SPT_DISABLED = 0,
    SPT_ENABLED  = 1,
} software_pwm_status_t;

typedef struct
{
    cam_key_t               cameraKeyPressed;
    software_pwm_voltage_t  softwarePwmVoltage;
    software_pwm_status_t   softwarePwmSatus;
    uint32_t                port;
    uint32_t                pin;
    uint32_t                bumpCount;
    uint32_t                pwmStopTime;
} software_pwm_record;

extern volatile software_pwm_record softPwmRecord;


extern int SoftPwmVoltage(uint32_t port, uint32_t pin, software_pwm_voltage_t voltage);
extern int StopSoftPwm(void);
extern int InitSoftPwm(void);
extern void SoftPwmTimerCallback(uint32_t callbackNumber);