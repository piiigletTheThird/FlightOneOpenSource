#pragma once

// all MPU device drivers (e.g. mpu9250) should implement the following
// functions

typedef struct {
    uint8_t rateDiv;
    uint8_t gyroDlpf;
    uint8_t gyroDlpfBypass;
    uint8_t accDlpf;
    uint8_t accDlpfBypass;
    uint8_t accDenom;
} gyro_device_config;

extern gyro_device_config gyroConfig;
extern int32_t deviceWhoAmI;
extern volatile uint32_t gyroInterrupting;

int AccGyroDeviceInit(loopCtrl_e gyroLoop);
int AccGyroDeviceDetect(void);

void accgyroDeviceReadGyro(void);
void accgyroDeviceReadAccGyro(void);
void accgyroDeviceReadComplete(void);

void accgyroDeviceCalibrate(int32_t *gyroData);
void accgyroDeviceApplyCalibration(int32_t *gyroData);

extern void DeInitGyroExti(void);
extern void InitGyroExti(void);
