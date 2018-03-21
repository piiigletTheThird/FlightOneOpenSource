#pragma once


typedef struct {
	volatile float dT;
	volatile float truedT;
	volatile uint32_t gyroAccDiv;
	volatile float gyrodT;
	volatile float halfGyrodT;
	volatile float halfGyrodTSquared;
	volatile float halfGyrodTSquaredI;
	volatile float accdT;
	volatile float InversedT;
	volatile uint32_t uhohNumber;
	volatile uint32_t khzDivider;
	volatile uint32_t gyroDivider;
	volatile uint32_t fsCount;
} loop_speed_record;

extern volatile loop_speed_record loopSpeed;
extern float filteredGyroData[];
extern volatile uint32_t boardArmed, calibrateMotors, fullKiLatched;
extern float pitchAttitude, rollAttitude, yawAttitude;
extern volatile uint32_t SKIP_GYRO;
extern float    accNoise[];
extern float    filteredAccData[];
extern volatile uint32_t armedTime;
extern volatile uint32_t armedTimeSincePower;
extern int   usedSkunk;
extern volatile float geeForceZ;

enum { CALIBRATE_BOARD_FAILED = 0, CALIBRATE_BOARD_UPRIGHT = 1, CALIBRATE_BOARD_INVERTED = 2, };

extern void     DeinitFlight(void);
extern int      InitFlight(uint32_t escProtocol, uint32_t escFrequency);
extern void     ArmBoard(void);
extern void     DisarmBoard(void);
extern int      SetCalibrate1(void);
extern int      SetCalibrate2(void);
extern void     InitFlightCode(uint32_t loopUsed);
extern void     InlineFlightCode(float dpsGyroArray[]);
extern void     InlineUpdateAttitude(float geeForceAccArray[]);
extern uint32_t SanityCheckEscProtocolAndFrequency(uint32_t *escProtocol, uint32_t *escFrequency);