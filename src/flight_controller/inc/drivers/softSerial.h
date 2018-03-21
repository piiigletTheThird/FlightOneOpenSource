#pragma once

typedef void (*softserial_function_pointer)(uint8_t serialBuffer[], uint32_t outputLength);

extern volatile softserial_function_pointer softserialCallbackFunctionArray[];

extern void     SoftSerialCheckLineIdle(void);
extern void     SoftSerialExtiCallback(uint32_t callbackNumber);
extern void     SoftSerialDmaCallback(uint32_t callbackNumber);
extern uint32_t SoftSerialSendReceiveBlocking(uint8_t serialOutBuffer[], uint32_t serialOutBufferLength, uint8_t inBuffer[], motor_type actuator, uint32_t timeoutMs);
extern uint32_t SoftSerialSendNonBlocking(uint8_t serialOutBuffer[], uint32_t serialOutBufferLength, motor_type actuator);
extern uint32_t SoftSerialReceiveNonBlocking(uint8_t inBuffer[], uint32_t *inBufferIdx, motor_type actuator);
