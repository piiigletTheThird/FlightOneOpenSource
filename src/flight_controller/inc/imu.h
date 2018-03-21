#pragma once



//	+X to the right
//	+Y straight up
//	+Z axis toward viewer
//	Heading = rotation about y axis
//	Attitude = rotation about z axis
//	Bank = rotation about x axis

//30 degrees per second
#define MAX_SPIN_RATE_RAD 0.523599f

typedef struct {
	volatile float x;
	volatile float y;
	volatile float z;
	volatile float w;
} quaternion_record;

typedef struct {
	volatile float x;
	volatile float y;
	volatile float z;
} vector_record;

extern volatile int quadInverted;
extern volatile float currentSpinRate;
extern volatile quaternion_record attitudeFrameQuat;
extern volatile float requestedDegrees[3];

extern void InitImu(void);
extern void ImuResetCommandQuat(void);
extern void ImuUpdateCommandQuat(float rollDps, float pitchDps, float yawDps, float halfdT);
extern void UpdateImu(float gx, float gy, float gz, float ax, float ay, float az);
