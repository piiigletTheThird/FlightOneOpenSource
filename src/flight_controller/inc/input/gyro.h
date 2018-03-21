#pragma once

#define GYRO_CALIBRATION_CYCLES 1000

// value returned on WHO_AM_I register
#define MPU6000_WHO_AM_I    0x68
#define MPU6500_WHO_AM_I    0x70
#define MPU6555_WHO_AM_I    0x7C
#define MPU9250_WHO_AM_I    0x71
#define ICM20689_WHO_AM_I   0x98
#define ICM20608G_WHO_AM_I  0xAF
#define ICM20602_WHO_AM_I   0x12
#define ICM20601_WHO_AM_I   0xAC

extern volatile uint32_t gyroCalibrationCycles;
extern float geeForceAccArray[3];

//config structure which is loaded by config
typedef struct {
    float minorBoardRotation[3]; //X, Y, Z
    uint32_t gyroRotation;
    uint32_t boardCalibrated; //board calibration complete?
    uint32_t loopCtrl;
    int      skunk;
    int      drunk;
} gyro_config;

enum {
    X=0,
    Y=1,
    Z=2,
};

enum
{
    CW0       = 0,
    CW90      = 1,
    CW180     = 2,
    CW270     = 3,
    CW0_INV   = 4,
    CW90_INV  = 5,
    CW180_INV = 6,
    CW270_INV = 7,
    CW45      = 8,
    CW135     = 9,
    CW225     = 10,
    CW315     = 11,
    CW45_INV  = 12,
    CW135_INV = 13,
    CW225_INV = 14,
    CW315_INV = 15,
};

typedef enum {
    LOOP_L1     = 0,
    LOOP_M1     = 1,
    LOOP_M2     = 2,
    LOOP_M4     = 3,
    LOOP_M8     = 4,
    LOOP_H1     = 5,
    LOOP_H2     = 6,
    LOOP_H4     = 7,
    LOOP_H8     = 8,
    LOOP_H16    = 9,
    LOOP_H32    = 10,
    LOOP_UH1    = 11,
    LOOP_UH2    = 12,
    LOOP_UH4    = 13,
    LOOP_UH8    = 14,
    LOOP_UH16   = 15,
    LOOP_UH32   = 16,
    LOOP_UH_500 = 17,
    LOOP_UH_250 = 18,
    LOOP_UH_062 = 19,
} loopCtrl_e;

extern void InitOrientation(void);
extern void ResetGyroCalibration(void);
extern void InlineUpdateAcc(int32_t rawAcc[], float scale);
extern void InlineUpdateGyro(int32_t rawGyro[], float scale);
extern void InlineApplyGyroAccRotationAndScale (int32_t rawData[], float dataArray[], float scale );
