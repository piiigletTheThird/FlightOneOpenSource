#pragma once

extern void DeInitActuators(void);
extern void InitActuators(uint32_t escProtocol, uint32_t escFrequency);
extern void OutputActuators(volatile float motorOutputHere[], volatile float servoOutput[]);
extern void ZeroActuators(uint32_t delayUs);
extern void IdleActuator(uint32_t motorNum);
extern void DirectActuator(uint32_t motorNum, float throttle);