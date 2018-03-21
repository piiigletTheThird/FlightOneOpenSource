#pragma once

extern uint32_t skipGyro;

void AccGyroDeinit(void);
// all gyroscopes should define this function which initializes all requesite
// hardware resources
uint32_t AccGyroInit(loopCtrl_e gyroLoop);

// functions used to read and write to hardware

extern void GyroExtiCallback(uint32_t callbackNumber);
extern void GyroRxDmaCallback(uint32_t callbackNumber);

uint32_t AccGyroWriteData(uint8_t *data, uint8_t length);
uint32_t AccGyroWriteRegister(uint8_t reg, uint8_t data);
uint32_t AccGyroVerifyWriteRegister(uint8_t reg, uint8_t data);

uint32_t AccGyroReadData(uint8_t reg, uint8_t *data, uint8_t length);
uint32_t AccGyroSlowReadData(uint8_t reg, uint8_t *data, uint8_t length);

uint32_t AccGyroDMAReadWriteData(uint8_t *txData, uint8_t *rxData, uint8_t length);
