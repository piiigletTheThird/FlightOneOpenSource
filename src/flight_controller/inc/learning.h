#pragma once

typedef struct
{
	volatile float m;
	volatile float b;
} learned_ki_model;

extern volatile learned_ki_model learnedKiModel[3];

extern int LearningInit(void);
extern int LearningModelInit(void);
extern float ApplyLearningModelToKi(float throttle, uint32_t axis);
extern int TrimKi(pid_output flightPids[]);
extern int TrimMotors(void);
extern int8_t ConvertFloatToInt8ForKi(float kiNumber);
extern float ConvertInt8ToFloatForKi(int8_t kiNumber);