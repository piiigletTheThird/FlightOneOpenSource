#pragma once


#include "mcu_include.h"

#define MAX_MOTOR_NUMBER 8
#define MAX_SERVO_NUMBER 8
#define MAX_USARTS       6
#define RXBUFFERSIZE     64
#define TXBUFFERSIZE     64


#define uid0_1 (*(uint8_t*)0x1fff7a10)
#define uid0_2 (*(uint8_t*)0x1fff7a11)
#define uid0_3 (*(uint8_t*)0x1fff7a12)
#define uid0_4 (*(uint8_t*)0x1fff7a13)
#define uid1_1 (*(uint8_t*)0x1fff7a14)
#define uid1_2 (*(uint8_t*)0x1fff7a15)
#define uid1_3 (*(uint8_t*)0x1fff7a16)
#define uid1_4 (*(uint8_t*)0x1fff7a17)
#define uid2_1 (*(uint8_t*)0x1fff7a18)
#define uid2_2 (*(uint8_t*)0x1fff7a19)
#define uid2_3 (*(uint8_t*)0x1fff7a1a)
#define uid2_4 (*(uint8_t*)0x1fff7a1b)

enum {
	ADC_DISABLED = 0,
	ADC_CURRENT  = 1,
	ADC_VOLTAGE  = 2,
	ADC_RSSI     = 3,
};

enum {
	ENUM_ACTUATOR_TYPE_DISABLED		= 0,
	ENUM_ACTUATOR_TYPE_SERVO		= 1,
	ENUM_ACTUATOR_TYPE_MOTOR		= 2,
	ENUM_ACTUATOR_TYPE_SOFT_SERIAL	= 3,
	ENUM_ACTUATOR_TYPE_WS2812		= 4,
	ENUM_ACTUATOR_TYPE_SPORT		= 5,
	ENUM_ACTUATOR_TYPE_SPMLAPTIMER	= 6,
};

enum {
	ENUM_SS_BAUD_19200_STD = 0,
	ENUM_SS_BAUD_57600_INV = 1,
};

enum {
	ENUM_ADC1 = 0,
	ENUM_ADC2 = 1,
	ENUM_ADC3 = 2,
};

enum {
	ENUM_SPI1 = 0,
	ENUM_SPI2 = 1,
	ENUM_SPI3 = 2,
	ENUM_SPI4 = 3,
	ENUM_SPI5 = 4,
	ENUM_SPI6 = 5,
};

enum {
	FP_EXTI0     = 0,
	FP_EXTI1     = 1,
	FP_EXTI2     = 2,
	FP_EXTI3     = 3,
	FP_EXTI4     = 4,
	FP_EXTI9_5   = 5,
	FP_EXTI15_10 = 6,
	FP_DMA1_S0   = 7,
	FP_DMA1_S1   = 8,
	FP_DMA1_S2   = 9,
	FP_DMA1_S3   = 10,
	FP_DMA1_S4   = 11,
	FP_DMA1_S5   = 12,
	FP_DMA1_S6   = 13,
	FP_DMA1_S7   = 14,
	FP_DMA2_S0   = 15,
	FP_DMA2_S1   = 16,
	FP_DMA2_S2   = 17,
	FP_DMA2_S3   = 18,
	FP_DMA2_S4   = 19,
	FP_DMA2_S5   = 20,
	FP_DMA2_S6   = 21,
	FP_DMA2_S7   = 22,
	FP_TIM1,
	FP_TIM2,
	FP_TIM3,
	FP_TIM4,
	FP_TIM5,
	FP_TIM6,
	FP_TIM7,
	FP_TIM8,
	FP_TIM9,
	FP_TIM10,
	FP_TIM11,
	FP_TIM12,
	FP_TIM13,
	FP_TIM14,
	IRQH_FP_TOT,
};

//TIM interrupt types
enum
{
	TIM_BRK = 0,
	TIM_UP,
	TIM_TRG,
	TIM_CC,
};

enum {
	ENUM_PORTA = 0,
	ENUM_PORTB = 1,
	ENUM_PORTC = 2,
	ENUM_PORTD = 3,
	ENUM_PORTE = 4,
	ENUM_PORTF = 5,
	ENUM_PORTG = 6,
	ENUM_PORTH = 7,
	ENUM_PORTI = 8,
};

enum {
	ENUM_USART1_IRQn = 0,
	ENUM_USART2_IRQn = 1,
	ENUM_USART3_IRQn = 2,
	ENUM_USART4_IRQn = 3,
	ENUM_USART5_IRQn = 4,
	ENUM_USART6_IRQn = 5,
};

enum {
	ENUM_TIM1  = 0,
	ENUM_TIM2  = 1,
	ENUM_TIM3  = 2,
	ENUM_TIM4  = 3,
	ENUM_TIM5  = 4,
	ENUM_TIM6  = 5,
	ENUM_TIM7  = 6,
	ENUM_TIM8  = 7,
	ENUM_TIM9  = 8,
	ENUM_TIM10 = 9,
	ENUM_TIM11 = 10,
	ENUM_TIM12 = 11,
	ENUM_TIM13 = 12,
	ENUM_TIM14 = 13,
};

enum {
	ENUM_TIM1CCR1  = 0,
	ENUM_TIM1CCR2  = 1,
	ENUM_TIM1CCR3  = 2,
	ENUM_TIM1CCR4  = 3,
	ENUM_TIM2CCR1  = 4,
	ENUM_TIM2CCR2  = 5,
	ENUM_TIM2CCR3  = 6,
	ENUM_TIM2CCR4  = 7,
	ENUM_TIM3CCR1  = 8,
	ENUM_TIM3CCR2  = 9,
	ENUM_TIM3CCR3  = 10,
	ENUM_TIM3CCR4  = 11,
	ENUM_TIM4CCR1  = 12,
	ENUM_TIM4CCR2  = 13,
	ENUM_TIM4CCR3  = 14,
	ENUM_TIM4CCR4  = 15,
	ENUM_TIM5CCR1  = 16,
	ENUM_TIM5CCR2  = 17,
	ENUM_TIM5CCR3  = 18,
	ENUM_TIM5CCR4  = 19,
	ENUM_TIM6CCR1  = 20,
	ENUM_TIM6CCR2  = 21,
	ENUM_TIM6CCR3  = 22,
	ENUM_TIM6CCR4  = 23,
	ENUM_TIM7CCR1  = 24,
	ENUM_TIM7CCR2  = 25,
	ENUM_TIM7CCR3  = 26,
	ENUM_TIM7CCR4  = 27,
	ENUM_TIM8CCR1  = 28,
	ENUM_TIM8CCR2  = 29,
	ENUM_TIM8CCR3  = 30,
	ENUM_TIM8CCR4  = 31,
	ENUM_TIM9CCR1  = 32,
	ENUM_TIM9CCR2  = 33,
	ENUM_TIM9CCR3  = 34,
	ENUM_TIM9CCR4  = 35,
	ENUM_TIM10CCR1 = 36,
	ENUM_TIM10CCR2 = 37,
	ENUM_TIM10CCR3 = 38,
	ENUM_TIM10CCR4 = 39,
	ENUM_TIM11CCR1 = 40,
	ENUM_TIM11CCR2 = 41,
	ENUM_TIM11CCR3 = 42,
	ENUM_TIM11CCR4 = 43,
	ENUM_TIM12CCR1 = 44,
	ENUM_TIM12CCR2 = 45,
	ENUM_TIM12CCR3 = 46,
	ENUM_TIM12CCR4 = 47,
	ENUM_TIM13CCR1 = 48,
	ENUM_TIM13CCR2 = 49,
	ENUM_TIM13CCR3 = 50,
	ENUM_TIM13CCR4 = 51,
	ENUM_TIM14CCR1 = 52,
	ENUM_TIM14CCR2 = 53,
	ENUM_TIM14CCR3 = 54,
	ENUM_TIM14CCR4 = 55,
};

enum {
	ENUM_USART1 = 0,
	ENUM_USART2 = 1,
	ENUM_USART3 = 2,
	ENUM_USART4 = 3,
	ENUM_USART5 = 4,
	ENUM_USART6 = 5,
};

enum {
	ENUM_USART_TX_PIN = 0,
	ENUM_USART_RX_PIN = 1,
};

enum {
	ENUM_DMA1_STREAM_0 = 0,
	ENUM_DMA1_STREAM_1 = 1,
	ENUM_DMA1_STREAM_2 = 2,
	ENUM_DMA1_STREAM_3 = 3,
	ENUM_DMA1_STREAM_4 = 4,
	ENUM_DMA1_STREAM_5 = 5,
	ENUM_DMA1_STREAM_6 = 6,
	ENUM_DMA1_STREAM_7 = 7,
	ENUM_DMA2_STREAM_0 = 8,
	ENUM_DMA2_STREAM_1 = 9,
	ENUM_DMA2_STREAM_2 = 10,
	ENUM_DMA2_STREAM_3 = 11,
	ENUM_DMA2_STREAM_4 = 12,
	ENUM_DMA2_STREAM_5 = 13,
	ENUM_DMA2_STREAM_6 = 14,
	ENUM_DMA2_STREAM_7 = 15,
};

//USB config
#define RFFW_HID_PRODUCT_STRING "RaceFlight FC"
#define RFBL_HID_PRODUCT_STRING "RaceFlight Boot Loader"
#define RFRC_HID_PRODUCT_STRING "RaceFlight Recovery"

//TODO: Should use better defines
#if defined(RVTF7)
	#include "RVTF7.h"
#elif defined(REVOLTF7)
	#include "REVOLTF7.h"
#elif defined(REVOLT)
	#include "REVOLT.h"
#elif defined(MICROVOLT)
	#include "MICROVOLT.h"
#elif defined(SPMFC400)
	#include "SPMFC400.h"
#endif


typedef struct {
	USART_TypeDef			*port;
	uint32_t				async;
} serial_type;


typedef struct {
	SPI_TypeDef				*port;
} spi_type;

typedef struct {
	uint32_t				timer;
	uint32_t				timerIRQn;
} general_timer;

typedef struct {
	uint32_t				enabled;
	uint32_t				instance;

	uint32_t				NSSPin;
	uint32_t				SCKPin;
	uint32_t				SCKPull;
	uint32_t				MISOPin;
	uint32_t				MOSIPin;

	uint32_t				NSSPort;
	uint32_t				SCKPort;
	uint32_t				MISOPort;
	uint32_t				MOSIPort;
	
	//uint32_t				NSSAlternate;
	uint32_t				SCKAlternate;
	uint32_t				MISOAlternate;
	uint32_t				MOSIAlternate;
	
	//uint32_t				GPIOSpeed;
	
	uint32_t				SPI_IRQn;
	uint32_t 				spiHandle;
	uint32_t 				priority;

	uint32_t				TXDma;
	uint32_t				RXDma;

	uint32_t				TXDMA_IRQn;
	uint32_t				RXDMA_IRQn;

	//uint32_t				TXDMA_IRQ_Handler;
	//uint32_t				RXDMA_IRQ_Handler;
} board_spi;

typedef struct {
	uint32_t				enabled;
	uint32_t				PinMode;
	uint32_t				Pull;
	uint32_t				Speed;
	uint32_t				TXAlternate;
	uint32_t				TXPin;
	uint32_t				RXAlternate;
	uint32_t				RXPin;

	uint32_t				RXPort; // loaded from port array
	uint32_t				TXPort; // loaded from port array

	uint32_t				SerialInstance; // loaded from usart array

	uint32_t				BaudRate;
	uint32_t				WordLength;
	uint32_t				StopBits;
	uint32_t				Parity;
	uint32_t				HwFlowCtl;
	uint32_t				Mode;

	uint32_t				USART_IRQn;
	uint32_t				usartHandle;

	uint32_t				TXDma;
	uint32_t				RXDma;
	uint32_t				TXDmaIrqn;
	uint32_t				RXDmaIrqn;
	uint32_t				TXDmaChannel;
	uint32_t				RXDmaChannel;

	uint32_t				Protocol;
	uint32_t				FrameSize;

	int32_t					serialTxBuffer;
	int32_t					serialRxBuffer;

	uint32_t				serialTxInverted;
	uint32_t				serialRxInverted;

} board_serial;

typedef struct {
	uint32_t				enabled;
	uint32_t				pin;
	uint32_t				port;
	uint32_t				adcInstance;
	uint32_t				adcChannel;
	uint32_t				adcHandle;
} board_adc;

typedef struct {
	uint32_t				enabled;
	uint32_t				dmaStream;
	uint32_t				dmaChannel;
	uint32_t				dmaDirection;
	uint32_t				dmaPeriphInc;
	uint32_t				dmaMemInc;
	uint32_t				dmaPeriphAlignment;
	uint32_t				dmaMemAlignment;
	uint32_t				dmaMode;
	uint32_t				dmaPriority;
	uint32_t				fifoMode;
	uint32_t				fifoThreshold;
	uint32_t				MemBurst;
	uint32_t				PeriphBurst;
	uint32_t				dmaIRQn;
	uint32_t				dmaHandle;
	uint32_t				priority;
} board_dma;

typedef struct {
	uint32_t				port;
	uint32_t				pin;
	uint32_t				inverted;
	uint32_t				enabled;
} internal_led_type;

typedef struct {
	uint32_t				enabled;
	uint32_t				spiNumber;
	uint32_t				csPin;
	uint32_t				csPort;
	uint32_t				extiPin;
	uint32_t				extiPort;
	uint32_t				extiIRQn;
	uint32_t				spiFastBaud;
	uint32_t				spiSlowBaud;
} gyro_type;

typedef struct {
	SPI_TypeDef *			SPIInstance;	//use this to determine spi irqn and irq handlers to use. No need to re define them here
	uint32_t				SPINumber;		
	uint32_t				csPin;
	GPIO_TypeDef *			csPort;
	uint32_t				extiPin;
	GPIO_TypeDef *			extiPort;
	uint32_t				extiIRQn;
	uint32_t				spiFastBaud;
	uint32_t				spiSlowBaud;
	//i2c stuff here?
} board_gyro;

typedef struct {
	uint32_t				enabled;
	uint32_t				timer;
	uint32_t				pin;
	uint32_t				port;
	uint32_t				AF;	
	uint32_t				timChannel;
	uint32_t				activeTim;
	uint32_t				timCCR;
	uint32_t				polarity;
	uint32_t				isNChannel;
	uint32_t				actuatorArrayNum;
	uint32_t				Dma;
	uint32_t				dmaEnabled;
	uint32_t				dmaChannel;
	uint32_t				dmaIRQn;
	uint32_t				CcDmaHandle;
	uint32_t				timerIRQn;
	uint32_t				timerIRQn_BRK;
	uint32_t				timerIRQn_UP;
	uint32_t				timerIRQn_TRG;
	uint32_t				timerIRQn_CC;
 	uint32_t				EXTIn;
 	uint32_t				EXTICallback;
 	uint32_t				DmaCallback;
} motor_type;


typedef struct {
	uint32_t				fc_pllm;
	uint32_t				fc_plln;
	uint32_t				fc_pllp;
	uint32_t				fc_pllq;
	
	internal_led_type		internalLeds[3];

	uint32_t				buzzerPort;
	uint32_t				buzzerPin;
	uint32_t				buzzerPolarity;

	gyro_type				gyros[2];
	gyro_type				flash[1];
	gyro_type				maxOsd[1];
	
	motor_type				motors[MAX_MOTOR_NUMBER];
	motor_type				servos[MAX_SERVO_NUMBER];

	board_gyro				gyro_pins;

	board_spi				spis[6];

	board_serial			serials[6];

	board_dma				dmasSerial[16]; //TODO: Make work based on serial enum number

	board_dma				dmasSpi[16]; //TODO: Make work based on Spi enum number

	board_dma				dmasMotor[16]; //TODO: Rename dmasActuators

	board_dma				dmasActive[16];

	board_adc				boardADC[3];

	general_timer			generalTimer[3];

} board_type;






//#define USARTx_DMA_TX_IRQHandler   DMA1_Stream3_IRQHandler
//#define USARTx_DMA_RX_IRQHandler   DMA1_Stream1_IRQHandler
//#define USARTx_IRQHandler          USART3_IRQHandler

typedef enum {INVERTED_RX, RX, GPS, OSD} usart_usage ;





//uint16_t pins;

/*
typedef struct
{
	unsigned char usart;

	unsigned char rxAF;
	unsigned char rxIRQn;
	unsigned char rxGPIO;
	uint16_t rxPIN;

	unsigned char txAF;
	unsigned char txIRQn;
	unsigned char txGPIO;
	uint16_t txPIN;

	usart_usage usart_type; //verify this turns into a byte, and if not switch it to them


} usart_pinout;

typedef struct
{
	usart_pinout usart_ports[8];


} target_pinout;

*/

typedef void (*function_pointer)(uint32_t callbackNumber);

extern volatile function_pointer callbackFunctionArray[];

#if defined(f3) || defined(f1)

#elif defined(f4) || defined(f7)
extern DMA_Stream_TypeDef *dmaStream[];
#endif

extern board_type          board;
extern GPIO_TypeDef       *ports[];
extern volatile uint32_t  *ccr[];
extern TIM_TypeDef        *timers[];
extern serial_type         usarts[];
extern UART_HandleTypeDef  uartHandles[];
extern DMA_HandleTypeDef   dmaHandles[];
extern TIM_HandleTypeDef   pwmTimers[];
extern TIM_OC_InitTypeDef  sConfigOCHandles[];
extern SPI_HandleTypeDef   spiHandles[];
extern SPI_TypeDef        *spiInstance[];
extern ADC_TypeDef        *adcInstance[];
extern ADC_HandleTypeDef   adcHandle[];

extern unsigned char serialTxBuffer[][TXBUFFERSIZE];
extern unsigned char serialRxBuffer[][RXBUFFERSIZE];
extern uint32_t motorOutputBuffer[][150];

extern int InitializeMCUSettings();
extern int GetBoardHardwareDefs();
