#pragma once

enum {
	SERIAL_STOP_BITS_1_0 = 0,
	SERIAL_STOP_BITS_1_5 = 1,
	SERIAL_STOP_BITS_2_0 = 2,
};

enum {
	SERIAL_START_BIT_OFF = 0,
	SERIAL_START_BIT_ON  = 1,
};

enum {
	SERIAL_NORMAL   = 0,
	SERIAL_INVERTED = 1,
};

enum {
	SERIAL_LSB = 0,
	SERIAL_MSB = 1,
};

enum {
	TBS_HANDLING_OFF = 0,
	TBS_HANDLING_ON  = 1,
};

enum {
	SERIAL_4800BAUD   = 4800,
	SERIAL_9600BAUD   = 9600,
	SERIAL_19200BAUD  = 19200,
	SERIAL_14400BAUD  = 14400,
	SERIAL_28800BAUD  = 28800,
	SERIAL_38400BAUD  = 38400,
	SERIAL_56000BAUD  = 56000,
	SERIAL_57600BAUD  = 57600,
	SERIAL_115200BAUD = 115200,
};

typedef struct
{
	uint32_t init;
	uint32_t inverted;
	uint32_t baud;
	float    bitWidthUs;
	float    byteWidthUs;
	uint32_t bitsPerByte;
	uint32_t gpio;
	uint32_t pin;
	uint32_t msb;
	uint32_t stopBits;
	uint32_t startBit;
	uint32_t tbsHandling;
} soft_serial_record;

extern uint32_t softSerialInit;

extern void     InitBlockingSoftSerialPort( uint32_t baudrate, uint32_t inverted, uint32_t stopBits, uint32_t startBit, uint32_t port, uint32_t pin, uint32_t msb, uint32_t tbsHandling );
extern uint32_t NewProcessSoftSerialBits(volatile uint32_t timerBuffer[], volatile uint32_t *timerBufferIndex, volatile uint8_t serialBuffer[], volatile uint32_t *serialBufferIndex, float bitWidthUs, uint32_t bitsInByte, uint32_t tbsHandling);
extern uint32_t SendSoftSerialBlocking(uint8_t byteArray[], uint32_t numBytesToSend, uint32_t timeoutMs);
extern uint32_t SendSoftSerialByteBlocking(uint8_t byte, uint32_t timeoutMs);
extern uint32_t DeInitBlockingSoftSerialPort(void);
extern uint32_t ReceiveSoftSerialBlocking(uint8_t rxBuffer[], uint32_t *rxBufferCount, uint32_t timeoutMs);
extern void     NewSoftSerialExtiCallback(uint32_t callbackNumber);
extern void     NewSoftSerialTimerCallback(uint32_t callbackNumber);

extern void TIM7_IRQHandler(void);
