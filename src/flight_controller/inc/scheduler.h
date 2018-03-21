#pragma once

enum {
	GYRO_INIT_FAILIURE                =  (1 << 0),
	SERIAL_INIT_FAILIURE              =  (1 << 1),
	DMA_INIT_FAILIURE                 =  (1 << 2),
	FLASH_INIT_FAILIURE               =  (1 << 3),
	WS2812_LED_INIT_FAILIURE          =  (1 << 4),
	FLASH_SPI_INIT_FAILIURE           =  (1 << 5),
	GYRO_SETUP_COMMUNICATION_FAILIURE =  (1 << 6),
	SERIAL_HALF_DUPLEX_INIT_FAILURE   =  (1 << 7),
	SERIAL_INIT_FAILURE               =  (1 << 8),
	MSP_DMA_GYRO_RX_INIT_FAILIURE     =  (1 << 9),
	MSP_DMA_GYRO_TX_INIT_FAILIURE     =  (1 << 10),
	MSP_DMA_SPI_RX_INIT_FAILIURE      =  (1 << 11),
	MSP_DMA_SPI_TX_INIT_FAILIURE      =  (1 << 12),
	NOT_USED1					      =  (1 << 13),
	NOT_USED2					      =  (1 << 14),
	NOT_USED3					      =  (1 << 15),
	NOT_USED4					      =  (1 << 16),
	HARD_FAULT                        =  (1 << 17),
	MEM_FAULT                         =  (1 << 18),
	BUS_FAULT                         =  (1 << 19),
	USAGE_FAULT                       =  (1 << 20),
	GYRO_SPI_INIT_FAILIURE            =  (1 << 21),
	TIMER_INPUT_INIT_FAILIURE         =  (1 << 22),
	ADC_INIT_FAILIURE                 =  (1 << 23),
	ADC_DMA_INIT_FAILIURE             =  (1 << 24),
	OSD_SPI_INIT_FAILIURE             =  (1 << 25),
	BAD_TELEMETRY_SETUP               =  (1 << 26),
};

#define SOFT_SERIAL_BIT_TIME_ARRAY_SIZE	80

extern volatile int      taskDshotActuators;
extern volatile int      taskIdleActuators[];
extern volatile int      taskDdsActuators;
extern volatile uint8_t  tOutBuffer[];
extern volatile uint8_t  tInBuffer[];
extern volatile uint32_t errorMask;
extern volatile uint32_t softSerialEnabled;
extern volatile uint32_t softSerialBuf[][SOFT_SERIAL_BIT_TIME_ARRAY_SIZE];
extern volatile uint32_t softSerialInd[];
extern volatile uint32_t softSerialCurBuf;
extern volatile uint32_t softSerialLastByteProcessedLocation;
extern volatile uint32_t softSerialSwitchBuffer;
extern volatile uint32_t turnOnVtxNow;
extern volatile uint32_t safeLoopCounter;

extern void InitScheduler(void);
extern void ErrorHandler(uint32_t error);
extern void Scheduler(int32_t count);
extern void SoftSerialCallback(void);
extern void DeInitFakeGyroExti(void);
extern void InitFakeGyroExti(void);
extern void GeneralInterruptTimerCallback(uint32_t callbackNumber);
