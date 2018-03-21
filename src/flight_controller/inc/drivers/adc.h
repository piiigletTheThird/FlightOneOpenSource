#pragma once

extern ADC_HandleTypeDef adcHandleT;
extern float adcVoltage;
extern float adcCurrent;
extern float adcMAh;

extern void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle);
extern void PollAdc(void);
extern void InitAdc(void);
extern void CheckBatteryCellCount(void);
extern float lowVoltage;
extern float runningVoltage;
extern float fullVoltage;
extern float averageVoltage;
extern float cellCutoff;

typedef struct
{
	float voltage;
	uint32_t storageTime;
}VoltageStorageRec;

