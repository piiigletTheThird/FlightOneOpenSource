#pragma once

#define MSP_VERSION  0
#define MSP_STATUS   101
#define MSP_RAW_IMU  102
#define MSP_ATTITUDE 108
#define MSP_ANALOG   110
#define MSP_BOXIDS   119

extern volatile uint8_t mspRxBuffer[];
extern volatile uint8_t mspTxBuffer[];

extern void InitMsp(uint32_t usartNumber);
extern void SendMspAttitude(void);
extern void SendMspAnalog(void);
extern void SendMspStatus(void);
extern void SendMspBoxIds(void);
