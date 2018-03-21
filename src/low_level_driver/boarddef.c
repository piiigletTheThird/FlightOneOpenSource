#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>


#include "includes.h"


board_type          board;
GPIO_TypeDef       *ports[11];
serial_type         usarts[6];
spi_type            spis[6];
TIM_TypeDef        *timers[14];
volatile uint32_t  *ccr[56];
DMA_Stream_TypeDef *dmaStream[16];
UART_HandleTypeDef  uartHandles[6];
DMA_HandleTypeDef   dmaHandles[16];
TIM_HandleTypeDef   pwmTimers[16];
TIM_OC_InitTypeDef  sConfigOCHandles[16];
SPI_HandleTypeDef   spiHandles[6];
SPI_TypeDef        *spiInstance[6];
ADC_TypeDef        *adcInstance[3];
ADC_HandleTypeDef   adcHandle[3];


uint32_t            boardSize;





volatile function_pointer callbackFunctionArray[IRQH_FP_TOT] = {0,};
uint32_t motorOutputBuffer[8][150];
unsigned char serialRxBuffer[6][RXBUFFERSIZE];
unsigned char serialTxBuffer[6][TXBUFFERSIZE];

int InitializeMCUSettings(void)
{

	bzero(spiInstance, sizeof(spiInstance));
	spiInstance[0] = SPI1;
	spiInstance[1] = SPI2;
	spiInstance[2] = SPI3;//TODO: Add enum for other SPIs so all boards can use what they have avilable.

	bzero(adcInstance, sizeof(adcInstance));
	adcInstance[0] = _ADC1;
	adcInstance[1] = _ADC2;
	adcInstance[2] = _ADC3;

	bzero(dmaStream, sizeof(dmaStream));
	dmaStream[0]  = DMA1_Stream0;
	dmaStream[1]  = DMA1_Stream1;
	dmaStream[2]  = DMA1_Stream2;
	dmaStream[3]  = DMA1_Stream3;
	dmaStream[4]  = DMA1_Stream4;
	dmaStream[5]  = DMA1_Stream5;
	dmaStream[6]  = DMA1_Stream6;
	dmaStream[7]  = DMA1_Stream7;
	dmaStream[8]  = DMA2_Stream0;
	dmaStream[9]  = DMA2_Stream1;
	dmaStream[10] = DMA2_Stream2;
	dmaStream[11] = DMA2_Stream3;
	dmaStream[12] = DMA2_Stream4;
	dmaStream[13] = DMA2_Stream5;
	dmaStream[14] = DMA2_Stream6;
	dmaStream[15] = DMA2_Stream7;

	bzero(ports, sizeof(ports));
	ports[0]  = _GPIOA;
	ports[1]  = _GPIOB;
	ports[2]  = _GPIOC;
	ports[3]  = _GPIOD;
	ports[4]  = _GPIOE;
	ports[5]  = _GPIOF;
	ports[6]  = _GPIOG;
	ports[7]  = _GPIOH;
	ports[8]  = _GPIOI;
	ports[9]  = _GPIOJ;
	ports[10] = _GPIOK;

	bzero(usarts, sizeof(usarts));
	usarts[0].port=_USART1;
	usarts[0].async=_USART1s;

	usarts[1].port=_USART2;
	usarts[1].async=_USART2s;

	usarts[2].port=_USART3;
	usarts[2].async=_USART3s;

	usarts[3].port=_USART4;
	usarts[3].async=_USART4s;

	usarts[4].port=_USART5;
	usarts[4].async=_USART5s;

	usarts[5].port=_USART6;
	usarts[5].async=_USART6s;

	bzero(timers, sizeof(timers));
	timers[0]=_TIM1;
	timers[1]=_TIM2;
	timers[2]=_TIM3;
	timers[3]=_TIM4;
	timers[4]=_TIM5;
	timers[5]=_TIM6;
	timers[6]=_TIM7;
	timers[7]=_TIM8;
	timers[8]=_TIM9;
	timers[9]=_TIM10;
	timers[10]=_TIM11;
	timers[11]=_TIM12;
	timers[12]=_TIM13;
	timers[13]=_TIM14;

	bzero(ccr, sizeof(ccr));
#ifdef TIM1
	ccr[0] = &_TIM1->CCR1;
	ccr[1] = &_TIM1->CCR2;
	ccr[2] = &_TIM1->CCR3;
	ccr[3] = &_TIM1->CCR4;
#endif
#ifdef TIM2
	ccr[4] = &_TIM2->CCR1;
	ccr[5] = &_TIM2->CCR2;
	ccr[6] = &_TIM2->CCR3;
	ccr[7] = &_TIM2->CCR4;
#endif
#ifdef TIM3
	ccr[8] = &_TIM3->CCR1;
	ccr[9] = &_TIM3->CCR2;
	ccr[10] = &_TIM3->CCR3;
	ccr[11] = &_TIM3->CCR4;
#endif
#ifdef TIM4
	ccr[12] = &_TIM4->CCR1;
	ccr[13] = &_TIM4->CCR2;
	ccr[14] = &_TIM4->CCR3;
	ccr[15] = &_TIM4->CCR4;
#endif
#ifdef TIM5
	ccr[16] = &_TIM5->CCR1;
	ccr[17] = &_TIM5->CCR2;
	ccr[18] = &_TIM5->CCR3;
	ccr[19] = &_TIM5->CCR4;
#endif
#ifdef TIM6
	ccr[20] = &_TIM6->CCR1;
	ccr[21] = &_TIM6->CCR2;
	ccr[22] = &_TIM6->CCR3;
	ccr[23] = &_TIM6->CCR4;
#endif
#ifdef TIM7
	ccr[24] = &_TIM7->CCR1;
	ccr[25] = &_TIM7->CCR2;
	ccr[26] = &_TIM7->CCR3;
	ccr[27] = &_TIM7->CCR4;
#endif
#ifdef TIM8
	ccr[28] = &_TIM8->CCR1;
	ccr[29] = &_TIM8->CCR2;
	ccr[30] = &_TIM8->CCR3;
	ccr[31] = &_TIM8->CCR4;
#endif
#ifdef TIM9
	ccr[32] = &_TIM9->CCR1;
	ccr[33] = &_TIM9->CCR2;
	ccr[34] = &_TIM9->CCR3;
	ccr[35] = &_TIM9->CCR4;
#endif
#ifdef TIM10
	ccr[36] = &_TIM10->CCR1;
	ccr[37] = &_TIM10->CCR2;
	ccr[38] = &_TIM10->CCR3;
	ccr[39] = &_TIM10->CCR4;
#endif
#ifdef TIM11
	ccr[40] = &_TIM11->CCR1;
	ccr[41] = &_TIM11->CCR2;
	ccr[42] = &_TIM11->CCR3;
	ccr[43] = &_TIM11->CCR4;
#endif
#ifdef TIM12
	ccr[44] = &_TIM12->CCR1;
	ccr[45] = &_TIM12->CCR2;
	ccr[46] = &_TIM12->CCR3;
	ccr[47] = &_TIM12->CCR4;
#endif
#ifdef TIM13
	ccr[48] = &_TIM13->CCR1;
	ccr[49] = &_TIM13->CCR2;
	ccr[50] = &_TIM13->CCR3;
	ccr[51] = &_TIM13->CCR4;
#endif
#ifdef TIM14
	ccr[52] = &_TIM14->CCR1;
	ccr[53] = &_TIM14->CCR2;
	ccr[54] = &_TIM14->CCR3;
	ccr[55] = &_TIM14->CCR4;
#endif

	return(0);
}


int GetBoardHardwareDefs(void)
{

	boardSize = sizeof(board);
	bzero(&board, sizeof(board));


	//PLL settings
	board.fc_pllm = FC_PLLM;
	board.fc_plln = FC_PLLN;
	board.fc_pllp = FC_PLLP;
	board.fc_pllq = FC_PLLQ;

	board.generalTimer[0].timer     = GENERAL_TIMER1;
	board.generalTimer[0].timerIRQn = GENERAL_TIMER1_IRQN;
	board.generalTimer[1].timer     = GENERAL_TIMER2;
	board.generalTimer[1].timerIRQn = GENERAL_TIMER2_IRQN;
	board.generalTimer[2].timer     = GENERAL_TIMER3;
	board.generalTimer[2].timerIRQn = GENERAL_TIMER3_IRQN;

	board.boardADC[0].enabled      = ADC0_TYPE;
	board.boardADC[0].port         = ADC0_PORT;
	board.boardADC[0].pin          = ADC0_PIN;
	board.boardADC[0].adcInstance  = ADC0_INSTANCE;
	board.boardADC[0].adcChannel   = ADC0_CHANNEL;
	board.boardADC[0].adcHandle    = 0;

	board.boardADC[1].enabled      = ADC1_TYPE;
	board.boardADC[1].port         = ADC1_PORT;
	board.boardADC[1].pin          = ADC1_PIN;
	board.boardADC[1].adcInstance  = ADC1_INSTANCE;
	board.boardADC[1].adcChannel   = ADC1_CHANNEL;
	board.boardADC[1].adcHandle    = 1;

	board.boardADC[2].enabled      = ADC2_TYPE;
	board.boardADC[2].port         = ADC2_PORT;
	board.boardADC[2].pin          = ADC2_PIN;
	board.boardADC[2].adcInstance  = ADC2_INSTANCE;
	board.boardADC[2].adcChannel   = ADC2_CHANNEL;
	board.boardADC[2].adcHandle    = 2;

	//LED Settings
	board.internalLeds[0].enabled  = LED1_ENABLED;
	board.internalLeds[0].pin      = LED1_GPIO_Pin;
	board.internalLeds[0].port     = LED1_GPIO_Port;
	board.internalLeds[0].inverted = LED1_INVERTED;

	board.internalLeds[1].enabled  = LED2_ENABLED;
	board.internalLeds[1].pin      = LED2_GPIO_Pin;
	board.internalLeds[1].port     = LED2_GPIO_Port;
	board.internalLeds[1].inverted = LED2_INVERTED;

	board.internalLeds[2].enabled  = LED3_ENABLED;
	board.internalLeds[2].pin      = LED3_GPIO_Pin;
	board.internalLeds[2].port     = LED3_GPIO_Port;
	board.internalLeds[2].inverted = LED3_INVERTED;


	//Buzzer Settings
	board.buzzerPort = BUZZER_GPIO_Port;	//Port C
	board.buzzerPin  = BUZZER_GPIO_Pin;
	board.buzzerPolarity = BUZZER_POLARITY;


	//Motor output assignments
	board.motors[0].actuatorArrayNum  = 0;
	board.motors[0].enabled           = ACTUATOR1_TYPE;
	board.motors[0].timer             = ACTUATOR1_TIM;
	board.motors[0].pin               = ACTUATOR1_PIN;
	board.motors[0].port              = ACTUATOR1_GPIO;
	board.motors[0].AF                = ACTUATOR1_ALTERNATE;
	board.motors[0].timChannel        = ACTUATOR1_TIM_CH;
	board.motors[0].activeTim	      = ACTUATOR1_ACTIVE_TIM;
	board.motors[0].timCCR            = ACTUATOR1_TIM_CCR;
	board.motors[0].polarity          = ACTUATOR1_POLARITY;
	board.motors[0].isNChannel        = ACTUATOR1_IS_N_CHANNEL;
	board.motors[0].Dma               = ACTUATOR1_DMA;
	board.motors[0].dmaIRQn           = ACTUATOR1_DMA_IRQN;
	board.motors[0].dmaChannel        = ACTUATOR1_DMA_CHANNEL;
	board.motors[0].CcDmaHandle       = ACTUATOR1_DMA_HANDLE;
	board.motors[0].timerIRQn         = ACTUATOR1_IRQN;
	board.motors[0].EXTIn             = ACTUATOR1_EXTIN; //used for input
	board.motors[0].EXTICallback      = ACTUATOR1_EXTICALLBACK; //used for input
	board.motors[0].DmaCallback       = ACTUATOR1_DMACALLBACK; //used for input
	board.motors[0].dmaEnabled        = ACTUATOR1_DMA_ENABLED;
	//Motor Specific DMA settings
	board.dmasMotor[0].enabled        = board.motors[0].dmaEnabled;
	board.dmasMotor[0].dmaStream      = board.motors[0].Dma;    //motor out
	board.dmasMotor[0].dmaHandle      = board.motors[0].Dma;    //motor out
	board.dmasMotor[0].dmaChannel     = board.motors[0].dmaChannel;         //motor out
	board.dmasMotor[0].dmaIRQn        = board.motors[0].dmaIRQn;     //motor out



	board.motors[1].actuatorArrayNum  = 1;
	board.motors[1].enabled           = ACTUATOR2_TYPE;
	board.motors[1].timer             = ACTUATOR2_TIM;
	board.motors[1].pin               = ACTUATOR2_PIN;
	board.motors[1].port              = ACTUATOR2_GPIO;
	board.motors[1].AF                = ACTUATOR2_ALTERNATE;
	board.motors[1].timChannel        = ACTUATOR2_TIM_CH;
	board.motors[1].activeTim	      = ACTUATOR2_ACTIVE_TIM;
	board.motors[1].timCCR            = ACTUATOR2_TIM_CCR;
	board.motors[1].polarity          = ACTUATOR2_POLARITY;
	board.motors[1].isNChannel        = ACTUATOR2_IS_N_CHANNEL;
	board.motors[1].Dma               = ACTUATOR2_DMA;
	board.motors[1].dmaIRQn           = ACTUATOR2_DMA_IRQN;
	board.motors[1].dmaChannel        = ACTUATOR2_DMA_CHANNEL;
	board.motors[1].CcDmaHandle       = ACTUATOR2_DMA_HANDLE;
	board.motors[1].timerIRQn         = ACTUATOR2_IRQN;
	board.motors[1].EXTIn             = ACTUATOR2_EXTIN; //used for input
	board.motors[1].EXTICallback      = ACTUATOR2_EXTICALLBACK; //used for input
	board.motors[1].DmaCallback       = ACTUATOR2_DMACALLBACK; //used for input
	board.motors[1].dmaEnabled        = ACTUATOR2_DMA_ENABLED;
	//Motor Specific DMA settings
	board.dmasMotor[1].enabled        = board.motors[1].dmaEnabled;
	board.dmasMotor[1].dmaStream      = board.motors[1].Dma;    //motor out
	board.dmasMotor[1].dmaHandle      = board.motors[1].Dma;    //motor out
	board.dmasMotor[1].dmaChannel     = board.motors[1].dmaChannel;         //motor out
	board.dmasMotor[1].dmaIRQn        = board.motors[1].dmaIRQn;     //motor out


	board.motors[2].actuatorArrayNum  = 2;
	board.motors[2].enabled           = ACTUATOR3_TYPE;
	board.motors[2].timer             = ACTUATOR3_TIM;
	board.motors[2].pin               = ACTUATOR3_PIN;
	board.motors[2].port              = ACTUATOR3_GPIO;
	board.motors[2].AF                = ACTUATOR3_ALTERNATE;
	board.motors[2].timChannel        = ACTUATOR3_TIM_CH;
	board.motors[2].activeTim	      = ACTUATOR3_ACTIVE_TIM;
	board.motors[2].timCCR            = ACTUATOR3_TIM_CCR;
	board.motors[2].polarity          = ACTUATOR3_POLARITY;
	board.motors[2].isNChannel        = ACTUATOR3_IS_N_CHANNEL;
	board.motors[2].Dma               = ACTUATOR3_DMA;
	board.motors[2].dmaIRQn           = ACTUATOR3_DMA_IRQN;
	board.motors[2].dmaChannel        = ACTUATOR3_DMA_CHANNEL;
	board.motors[2].CcDmaHandle       = ACTUATOR3_DMA_HANDLE;
	board.motors[2].timerIRQn         = ACTUATOR3_IRQN;
	board.motors[2].EXTIn             = ACTUATOR3_EXTIN; //used for input
 	board.motors[2].EXTICallback      = ACTUATOR3_EXTICALLBACK; //used for input
 	board.motors[2].DmaCallback       = ACTUATOR3_DMACALLBACK; //used for input
 	board.motors[2].dmaEnabled        = ACTUATOR3_DMA_ENABLED;
 	//Motor Specific DMA settings
	board.dmasMotor[2].enabled        = board.motors[2].dmaEnabled;
	board.dmasMotor[2].dmaStream      = board.motors[2].Dma;    //motor out
	board.dmasMotor[2].dmaHandle      = board.motors[2].Dma;    //motor out
	board.dmasMotor[2].dmaChannel     = board.motors[2].dmaChannel;         //motor out
	board.dmasMotor[2].dmaIRQn        = board.motors[2].dmaIRQn;     //motor out



	board.motors[3].actuatorArrayNum  = 3;
	board.motors[3].enabled           = ACTUATOR4_TYPE;
	board.motors[3].timer             = ACTUATOR4_TIM;
	board.motors[3].pin               = ACTUATOR4_PIN;
	board.motors[3].port              = ACTUATOR4_GPIO;
	board.motors[3].AF                = ACTUATOR4_ALTERNATE;
	board.motors[3].timChannel        = ACTUATOR4_TIM_CH;
	board.motors[3].activeTim	      = ACTUATOR4_ACTIVE_TIM;
	board.motors[3].timCCR            = ACTUATOR4_TIM_CCR;
	board.motors[3].polarity          = ACTUATOR4_POLARITY;
	board.motors[3].isNChannel        = ACTUATOR4_IS_N_CHANNEL;
	board.motors[3].Dma               = ACTUATOR4_DMA;
	board.motors[3].dmaIRQn           = ACTUATOR4_DMA_IRQN;
	board.motors[3].dmaChannel        = ACTUATOR4_DMA_CHANNEL;
	board.motors[3].CcDmaHandle       = ACTUATOR4_DMA_HANDLE;
	board.motors[3].timerIRQn         = ACTUATOR4_IRQN;
	board.motors[3].EXTIn             = ACTUATOR4_EXTIN; //used for input
	board.motors[3].EXTICallback      = ACTUATOR4_EXTICALLBACK; //used for input
	board.motors[3].DmaCallback       = ACTUATOR4_DMACALLBACK; //used for input
	board.motors[3].dmaEnabled        = ACTUATOR4_DMA_ENABLED;
	//Motor Specific DMA settings
	board.dmasMotor[3].enabled        = board.motors[3].dmaEnabled;
	board.dmasMotor[3].dmaStream      = board.motors[3].Dma;    //motor out
	board.dmasMotor[3].dmaHandle      = board.motors[3].Dma;    //motor out
	board.dmasMotor[3].dmaChannel     = board.motors[3].dmaChannel;         //motor out
	board.dmasMotor[3].dmaIRQn        = board.motors[3].dmaIRQn;     //motor out


	board.motors[5].actuatorArrayNum  = 5;
	board.motors[5].enabled           = ACTUATOR6_TYPE;
	board.motors[5].timer             = ACTUATOR6_TIM;
	board.motors[5].pin               = ACTUATOR6_PIN;
	board.motors[5].port              = ACTUATOR6_GPIO;
	board.motors[5].AF                = ACTUATOR6_ALTERNATE;
	board.motors[5].timChannel        = ACTUATOR6_TIM_CH;
	board.motors[5].activeTim	      = ACTUATOR6_ACTIVE_TIM;
	board.motors[5].timCCR            = ACTUATOR6_TIM_CCR;
	board.motors[5].polarity          = ACTUATOR6_POLARITY;
	board.motors[5].isNChannel        = ACTUATOR6_IS_N_CHANNEL;
	board.motors[5].Dma               = ACTUATOR6_DMA;
	board.motors[5].dmaIRQn           = ACTUATOR6_DMA_IRQN;
	board.motors[5].dmaChannel        = ACTUATOR6_DMA_CHANNEL;
	board.motors[5].CcDmaHandle       = ACTUATOR6_DMA_HANDLE;
	board.motors[5].timerIRQn         = ACTUATOR6_IRQN;
	board.motors[5].timerIRQn_BRK     = ACTUATOR6_IRQN_BRK;
	board.motors[5].timerIRQn_UP      = ACTUATOR6_IRQN_UP;
	board.motors[5].timerIRQn_TRG     = ACTUATOR6_IRQN_TRG;
	board.motors[5].timerIRQn_CC      = ACTUATOR6_IRQN_CC;
	board.motors[5].EXTIn             = ACTUATOR6_EXTIN; //used for input
	board.motors[5].EXTICallback      = ACTUATOR6_EXTICALLBACK; //used for input
	board.motors[5].DmaCallback       = ACTUATOR6_DMACALLBACK; //used for input
	board.motors[5].dmaEnabled        = ACTUATOR6_DMA_ENABLED;
	//Motor Specific DMA settings
	board.dmasMotor[5].enabled        = board.motors[5].dmaEnabled;
	board.dmasMotor[5].dmaStream      = board.motors[5].Dma;
	board.dmasMotor[5].dmaHandle      = board.motors[5].Dma;
	board.dmasMotor[5].dmaChannel     = board.motors[5].dmaChannel;
	board.dmasMotor[5].dmaIRQn        = board.motors[5].dmaIRQn;


	board.motors[6].actuatorArrayNum  = 6;
	board.motors[6].enabled           = ACTUATOR7_TYPE;
	board.motors[6].timer             = ACTUATOR7_TIM;
	board.motors[6].pin               = ACTUATOR7_PIN;
	board.motors[6].port              = ACTUATOR7_GPIO;
	board.motors[6].AF                = ACTUATOR7_ALTERNATE;
	board.motors[6].timChannel        = ACTUATOR7_TIM_CH;
	board.motors[6].activeTim	      = ACTUATOR7_ACTIVE_TIM;
	board.motors[6].timCCR            = ACTUATOR7_TIM_CCR;
	board.motors[6].polarity          = ACTUATOR7_POLARITY;
	board.motors[6].isNChannel        = ACTUATOR7_IS_N_CHANNEL;
	board.motors[6].Dma               = ACTUATOR7_DMA;
	board.motors[6].dmaIRQn           = ACTUATOR7_DMA_IRQN;
	board.motors[6].dmaChannel        = ACTUATOR7_DMA_CHANNEL;
	board.motors[6].CcDmaHandle       = ACTUATOR7_DMA_HANDLE;
	board.motors[6].timerIRQn         = ACTUATOR7_IRQN;
	board.motors[6].EXTIn             = ACTUATOR7_EXTIN; //used for input
	board.motors[6].EXTICallback      = ACTUATOR7_EXTICALLBACK; //used for input
	board.motors[6].DmaCallback       = ACTUATOR7_DMACALLBACK; //used for input
	board.motors[6].dmaEnabled        = ACTUATOR7_DMA_ENABLED;
	//Motor Specific DMA settings
	board.dmasMotor[6].enabled        = board.motors[6].dmaEnabled;
	board.dmasMotor[6].dmaStream      = board.motors[6].Dma;
	board.dmasMotor[6].dmaHandle      = board.motors[6].Dma;
	board.dmasMotor[6].dmaChannel     = board.motors[6].dmaChannel;
	board.dmasMotor[6].dmaIRQn        = board.motors[6].dmaIRQn;



	board.motors[7].actuatorArrayNum  = 7;
	board.motors[7].enabled           = ACTUATOR8_TYPE;
	board.motors[7].timer             = ACTUATOR8_TIM;
	board.motors[7].pin               = ACTUATOR8_PIN;
	board.motors[7].port              = ACTUATOR8_GPIO;
	board.motors[7].AF                = ACTUATOR8_ALTERNATE;
	board.motors[7].timChannel        = ACTUATOR8_TIM_CH;
	board.motors[7].activeTim	      = ACTUATOR8_ACTIVE_TIM;
	board.motors[7].timCCR            = ACTUATOR8_TIM_CCR;
	board.motors[7].polarity          = ACTUATOR8_POLARITY;
	board.motors[7].isNChannel        = ACTUATOR8_IS_N_CHANNEL;
	board.motors[7].Dma               = ACTUATOR8_DMA;
	board.motors[7].dmaIRQn           = ACTUATOR8_DMA_IRQN;
	board.motors[7].dmaChannel        = ACTUATOR8_DMA_CHANNEL;
	board.motors[7].CcDmaHandle       = ACTUATOR8_DMA_HANDLE;
	board.motors[7].timerIRQn         = ACTUATOR8_IRQN;
	board.motors[7].EXTIn             = ACTUATOR8_EXTIN; //used for input
	board.motors[7].EXTICallback      = ACTUATOR8_EXTICALLBACK; //used for input
	board.motors[7].DmaCallback       = ACTUATOR8_DMACALLBACK; //used for input
	board.motors[7].dmaEnabled        = ACTUATOR8_DMA_ENABLED;
	//Motor Specific DMA settings
	board.dmasMotor[7].enabled        = board.motors[7].dmaEnabled;
	board.dmasMotor[7].dmaStream      = board.motors[7].Dma;
	board.dmasMotor[7].dmaHandle      = board.motors[7].Dma;
	board.dmasMotor[7].dmaChannel     = board.motors[7].dmaChannel;
	board.dmasMotor[7].dmaIRQn        = board.motors[7].dmaIRQn;


	//GYRO connection settings	------------------------------------------------------------------------------------------------------------------------------------------------------------
	board.gyros[0].enabled     = 1;
	board.gyros[0].spiNumber   = GYRO_SPI_NUMBER;
	board.gyros[0].csPin       = GYRO_SPI_CS_GPIO_Pin;
	board.gyros[0].csPort      = GYRO_SPI_CS_GPIO_Port;
	board.gyros[0].extiPin     = GYRO_EXTI_GPIO_Pin;
	board.gyros[0].extiPort    = GYRO_EXTI_GPIO_Port;
	board.gyros[0].extiIRQn    = GYRO_EXTI_IRQn;
	board.gyros[0].spiFastBaud = GYRO_SPI_FAST_BAUD;
	board.gyros[0].spiSlowBaud = GYRO_SPI_SLOW_BAUD;
	callbackFunctionArray[GYRO_EXTI_IRQn_FP] = GyroExtiCallback;
	callbackFunctionArray[GYRO_RX_DMA_FP]    = GyroRxDmaCallback;

	//Flash connection settings	------------------------------------------------------------------------------------------------------------------------------------------------------------
	board.flash[0].enabled     = FLASH_ENABLED;
	board.flash[0].spiNumber   = FLASH_SPI_NUMBER;
	board.flash[0].csPin       = FLASH_SPI_CS_GPIO_Pin;
	board.flash[0].csPort      = FLASH_SPI_CS_GPIO_Port;
	board.flash[0].extiPin     = 0;
	board.flash[0].extiPort    = 0;
	board.flash[0].extiIRQn    = 0;
	board.flash[0].spiFastBaud = FLASH_SPI_FAST_BAUD;
	board.flash[0].spiSlowBaud = FLASH_SPI_SLOW_BAUD;
	callbackFunctionArray[FLASH_RX_DMA_FP] = FlashDmaRxCallback;

	//Max OSD connection settings	------------------------------------------------------------------------------------------------------------------------------------------------------------
	board.maxOsd[0].enabled     = MAX_OSD_ENABLED;
	if (board.maxOsd[0].enabled == 1)
	{
		//1 is lone spi, 2 is shared spi
		board.maxOsd[0].spiNumber   = MAX_OSD_SPI_NUMBER;
		board.maxOsd[0].csPin       = MAX_OSD_SPI_CS_GPIO_Pin;
		board.maxOsd[0].csPort      = MAX_OSD_SPI_CS_GPIO_Port;
		board.maxOsd[0].extiPin     = 0;
		board.maxOsd[0].extiPort    = 0;
		board.maxOsd[0].extiIRQn    = 0;
		board.maxOsd[0].spiFastBaud = MAX_OSD_SPI_FAST_BAUD;
		board.maxOsd[0].spiSlowBaud = MAX_OSD_SPI_SLOW_BAUD;
		callbackFunctionArray[MAX_OSD_TX_DMA_FP] = MaxOsdDmaTxCallback;
	}

	//SPI settings ------------------------------------------------------------------------------------------------------------------------------------------------------------
	board.spis[ENUM_SPI1].enabled                                 = SPI1_ENABLE;
	board.spis[ENUM_SPI1].instance                                = ENUM_SPI1;

	board.spis[ENUM_SPI1].NSSPin                                  = SPI1_NSS_PIN;
	board.spis[ENUM_SPI1].SCKPin                                  = SPI1_SCK_PIN;
	board.spis[ENUM_SPI1].MISOPin                                 = SPI1_MISO_PIN;
	board.spis[ENUM_SPI1].MOSIPin                                 = SPI1_MOSI_PIN;

	board.spis[ENUM_SPI1].NSSPort                                 = SPI1_NSS_GPIO_PORT;
	board.spis[ENUM_SPI1].SCKPort                                 = SPI1_SCK_GPIO_PORT;
	board.spis[ENUM_SPI1].MISOPort                                = SPI1_MISO_GPIO_PORT;
	board.spis[ENUM_SPI1].MOSIPort                                = SPI1_MOSI_GPIO_PORT;

	board.spis[ENUM_SPI1].SCKPull                                 = SPI1_SCK_PULL;

	board.spis[ENUM_SPI1].SCKAlternate                            = SPI1_SCK_AF;
	board.spis[ENUM_SPI1].MISOAlternate                           = SPI1_MISO_AF;
	board.spis[ENUM_SPI1].MOSIAlternate                           = SPI1_MOSI_AF;

	board.spis[ENUM_SPI1].SPI_IRQn                                = SPI1_IRQn;
	board.spis[ENUM_SPI1].spiHandle                               = ENUM_SPI1;
	board.spis[ENUM_SPI1].priority                                = SPI1_PRIORITY;


	board.spis[ENUM_SPI1].TXDMA_IRQn                              = SPI1_TX_DMA_IRQn;
	board.spis[ENUM_SPI1].RXDMA_IRQn                              = SPI1_RX_DMA_IRQn;

	board.spis[ENUM_SPI1].TXDma 		                          = SPI1_TX_DMA_STREAM;
	board.spis[ENUM_SPI1].RXDma 		                          = SPI1_RX_DMA_STREAM;

	board.dmasSpi[board.spis[ENUM_SPI1].TXDma].enabled            = SPI1_DMA_ENABLED;
	board.dmasSpi[board.spis[ENUM_SPI1].TXDma].dmaStream          = SPI1_TX_DMA_STREAM;
	board.dmasSpi[board.spis[ENUM_SPI1].TXDma].dmaChannel         = SPI1_TX_DMA_CHANNEL;
	board.dmasSpi[board.spis[ENUM_SPI1].TXDma].dmaDirection       = DMA_MEMORY_TO_PERIPH;
	board.dmasSpi[board.spis[ENUM_SPI1].TXDma].dmaPeriphInc       = DMA_PINC_DISABLE;
	board.dmasSpi[board.spis[ENUM_SPI1].TXDma].dmaMemInc          = DMA_MINC_ENABLE;
	board.dmasSpi[board.spis[ENUM_SPI1].TXDma].dmaPeriphAlignment = DMA_PDATAALIGN_BYTE;
	board.dmasSpi[board.spis[ENUM_SPI1].TXDma].dmaMemAlignment    = DMA_MDATAALIGN_BYTE;
	board.dmasSpi[board.spis[ENUM_SPI1].TXDma].dmaMode            = DMA_NORMAL;
//here2	board.dmasSpi[board.spis[ENUM_SPI1].TXDma].dmaPriority        = DMA_PRIORITY_VERY_HIGH;
	board.dmasSpi[board.spis[ENUM_SPI1].TXDma].dmaPriority        = DMA_PRIORITY_HIGH;
	board.dmasSpi[board.spis[ENUM_SPI1].TXDma].fifoMode           = DMA_FIFOMODE_DISABLE;
	board.dmasSpi[board.spis[ENUM_SPI1].TXDma].dmaIRQn            = SPI1_TX_DMA_IRQn;
	board.dmasSpi[board.spis[ENUM_SPI1].TXDma].dmaHandle          = SPI1_TX_DMA_STREAM;
	board.dmasSpi[board.spis[ENUM_SPI1].TXDma].priority           = SPI1_RX_DMA_PRIORITY;

	board.dmasSpi[board.spis[ENUM_SPI1].RXDma].enabled            = SPI1_DMA_ENABLED;
	board.dmasSpi[board.spis[ENUM_SPI1].RXDma].dmaStream          = SPI1_RX_DMA_STREAM;
	board.dmasSpi[board.spis[ENUM_SPI1].RXDma].dmaChannel         = SPI1_RX_DMA_CHANNEL;
	board.dmasSpi[board.spis[ENUM_SPI1].RXDma].dmaDirection       = DMA_PERIPH_TO_MEMORY;
	board.dmasSpi[board.spis[ENUM_SPI1].RXDma].dmaPeriphInc       = DMA_PINC_DISABLE;
	board.dmasSpi[board.spis[ENUM_SPI1].RXDma].dmaMemInc          = DMA_MINC_ENABLE;
	board.dmasSpi[board.spis[ENUM_SPI1].RXDma].dmaPeriphAlignment = DMA_PDATAALIGN_BYTE;
	board.dmasSpi[board.spis[ENUM_SPI1].RXDma].dmaMemAlignment    = DMA_MDATAALIGN_BYTE;
	board.dmasSpi[board.spis[ENUM_SPI1].RXDma].dmaMode            = DMA_NORMAL;
	//here2 board.dmasSpi[board.spis[ENUM_SPI1].RXDma].dmaPriority        = DMA_PRIORITY_VERY_HIGH;
	board.dmasSpi[board.spis[ENUM_SPI1].RXDma].dmaPriority        = DMA_PRIORITY_HIGH;
	board.dmasSpi[board.spis[ENUM_SPI1].RXDma].fifoMode           = DMA_FIFOMODE_DISABLE;
	board.dmasSpi[board.spis[ENUM_SPI1].RXDma].dmaIRQn            = SPI1_RX_DMA_IRQn;
	board.dmasSpi[board.spis[ENUM_SPI1].RXDma].dmaHandle          = SPI1_RX_DMA_STREAM;
	board.dmasSpi[board.spis[ENUM_SPI1].RXDma].priority           = SPI1_RX_DMA_PRIORITY;


	board.spis[ENUM_SPI2].enabled                                 = SPI2_ENABLE;
	board.spis[ENUM_SPI2].instance                                = ENUM_SPI2;

	board.spis[ENUM_SPI2].NSSPin                                  = SPI2_NSS_PIN;
	board.spis[ENUM_SPI2].SCKPin                                  = SPI2_SCK_PIN;
	board.spis[ENUM_SPI2].MISOPin                                 = SPI2_MISO_PIN;
	board.spis[ENUM_SPI2].MOSIPin                                 = SPI2_MOSI_PIN;

	board.spis[ENUM_SPI2].NSSPort                                 = SPI2_NSS_GPIO_PORT;
	board.spis[ENUM_SPI2].SCKPort                                 = SPI2_SCK_GPIO_PORT;
	board.spis[ENUM_SPI2].MISOPort                                = SPI2_MISO_GPIO_PORT;
	board.spis[ENUM_SPI2].MOSIPort                                = SPI2_MOSI_GPIO_PORT;

	board.spis[ENUM_SPI2].SCKPull                                 = SPI2_SCK_PULL;

	board.spis[ENUM_SPI2].SCKAlternate                            = SPI2_SCK_AF;
	board.spis[ENUM_SPI2].MISOAlternate                           = SPI2_MISO_AF;
	board.spis[ENUM_SPI2].MOSIAlternate                           = SPI2_MOSI_AF;

	board.spis[ENUM_SPI2].SPI_IRQn                                = SPI2_IRQn;
	board.spis[ENUM_SPI2].spiHandle                               = ENUM_SPI2;
	board.spis[ENUM_SPI2].priority                                = SPI2_PRIORITY;

	board.spis[ENUM_SPI2].TXDMA_IRQn                              = SPI2_TX_DMA_IRQn;
	board.spis[ENUM_SPI2].RXDMA_IRQn                              = SPI2_RX_DMA_IRQn;

	board.spis[ENUM_SPI2].TXDma 		                          = SPI2_TX_DMA_STREAM;
	board.spis[ENUM_SPI2].RXDma 		                          = SPI2_RX_DMA_STREAM;

	board.dmasSpi[board.spis[ENUM_SPI2].TXDma].enabled            = SPI2_DMA_ENABLED;
	board.dmasSpi[board.spis[ENUM_SPI2].TXDma].dmaStream          = SPI2_TX_DMA_STREAM;   //diff between all SPIs
	board.dmasSpi[board.spis[ENUM_SPI2].TXDma].dmaChannel         = SPI2_TX_DMA_CHANNEL;  //diff
	board.dmasSpi[board.spis[ENUM_SPI2].TXDma].dmaDirection       = DMA_MEMORY_TO_PERIPH; //same between all SPIs, diff between TX/RX
	board.dmasSpi[board.spis[ENUM_SPI2].TXDma].dmaPeriphInc       = DMA_PINC_DISABLE;     //same
	board.dmasSpi[board.spis[ENUM_SPI2].TXDma].dmaMemInc          = DMA_MINC_ENABLE;      //same
	board.dmasSpi[board.spis[ENUM_SPI2].TXDma].dmaPeriphAlignment = DMA_PDATAALIGN_BYTE;  //same
	board.dmasSpi[board.spis[ENUM_SPI2].TXDma].dmaMemAlignment    = DMA_MDATAALIGN_BYTE;  //same
	board.dmasSpi[board.spis[ENUM_SPI2].TXDma].dmaMode            = DMA_NORMAL;           //same
	board.dmasSpi[board.spis[ENUM_SPI2].TXDma].dmaPriority        = DMA_PRIORITY_HIGH;    //same, maybe we should change them
	board.dmasSpi[board.spis[ENUM_SPI2].TXDma].fifoMode           = DMA_FIFOMODE_DISABLE; //same
	board.dmasSpi[board.spis[ENUM_SPI2].TXDma].dmaIRQn            = SPI2_TX_DMA_IRQn;     //diff
	board.dmasSpi[board.spis[ENUM_SPI2].TXDma].dmaHandle          = SPI2_TX_DMA_STREAM;   //diff
	board.dmasSpi[board.spis[ENUM_SPI2].TXDma].priority           = SPI2_RX_DMA_PRIORITY;

	board.dmasSpi[board.spis[ENUM_SPI2].RXDma].enabled            = SPI2_DMA_ENABLED;
	board.dmasSpi[board.spis[ENUM_SPI2].RXDma].dmaStream          = SPI2_RX_DMA_STREAM;
	board.dmasSpi[board.spis[ENUM_SPI2].RXDma].dmaChannel         = SPI2_RX_DMA_CHANNEL;
	board.dmasSpi[board.spis[ENUM_SPI2].RXDma].dmaDirection       = DMA_PERIPH_TO_MEMORY;
	board.dmasSpi[board.spis[ENUM_SPI2].RXDma].dmaPeriphInc       = DMA_PINC_DISABLE;
	board.dmasSpi[board.spis[ENUM_SPI2].RXDma].dmaMemInc          = DMA_MINC_ENABLE;
	board.dmasSpi[board.spis[ENUM_SPI2].RXDma].dmaPeriphAlignment = DMA_PDATAALIGN_BYTE;
	board.dmasSpi[board.spis[ENUM_SPI2].RXDma].dmaMemAlignment    = DMA_MDATAALIGN_BYTE;
	board.dmasSpi[board.spis[ENUM_SPI2].RXDma].dmaMode            = DMA_NORMAL;
	board.dmasSpi[board.spis[ENUM_SPI2].RXDma].dmaPriority        = DMA_PRIORITY_HIGH;
	board.dmasSpi[board.spis[ENUM_SPI2].RXDma].fifoMode           = DMA_FIFOMODE_DISABLE;
	board.dmasSpi[board.spis[ENUM_SPI2].RXDma].dmaIRQn            = SPI2_RX_DMA_IRQn;
	board.dmasSpi[board.spis[ENUM_SPI2].RXDma].dmaHandle          = SPI2_RX_DMA_STREAM;
	board.dmasSpi[board.spis[ENUM_SPI2].RXDma].priority           = SPI2_RX_DMA_PRIORITY;


	board.spis[ENUM_SPI3].enabled                                 = SPI3_ENABLE;
	board.spis[ENUM_SPI3].instance                                = ENUM_SPI3;

	board.spis[ENUM_SPI3].NSSPin                                  = SPI3_NSS_PIN;
	board.spis[ENUM_SPI3].SCKPin                                  = SPI3_SCK_PIN;
	board.spis[ENUM_SPI3].MISOPin                                 = SPI3_MISO_PIN;
	board.spis[ENUM_SPI3].MOSIPin                                 = SPI3_MOSI_PIN;

	board.spis[ENUM_SPI3].SCKPull                                 = SPI3_SCK_PULL;

	board.spis[ENUM_SPI3].NSSPort                                 = SPI3_NSS_GPIO_PORT;
	board.spis[ENUM_SPI3].SCKPort                                 = SPI3_SCK_GPIO_PORT;
	board.spis[ENUM_SPI3].MISOPort                                = SPI3_MISO_GPIO_PORT;
	board.spis[ENUM_SPI3].MOSIPort                       	      = SPI3_MOSI_GPIO_PORT;

	board.spis[ENUM_SPI3].SCKAlternate                  	      = SPI3_SCK_AF;
	board.spis[ENUM_SPI3].MISOAlternate                 	      = SPI3_MISO_AF;
	board.spis[ENUM_SPI3].MOSIAlternate                 	      = SPI3_MOSI_AF;

	board.spis[ENUM_SPI3].SPI_IRQn                   	          = SPI3_IRQn;
	board.spis[ENUM_SPI3].spiHandle                  	          = ENUM_SPI3;
	board.spis[ENUM_SPI3].priority                   	          = SPI3_PRIORITY;

	board.spis[ENUM_SPI3].TXDMA_IRQn                 	          = SPI3_TX_DMA_IRQn;
	board.spis[ENUM_SPI3].RXDMA_IRQn                 	          = SPI3_RX_DMA_IRQn;

	board.spis[ENUM_SPI3].TXDMA_IRQn                 	          = SPI3_TX_DMA_IRQn;
	board.spis[ENUM_SPI3].RXDMA_IRQn                 	          = SPI3_RX_DMA_IRQn;

	board.spis[ENUM_SPI3].TXDma 		                     	  = SPI3_TX_DMA_STREAM;
	board.spis[ENUM_SPI3].RXDma 		                     	  = SPI3_RX_DMA_STREAM;

	board.dmasSpi[board.spis[ENUM_SPI3].TXDma].enabled            = SPI3_DMA_ENABLED;
	board.dmasSpi[board.spis[ENUM_SPI3].TXDma].dmaStream          = board.spis[ENUM_SPI3].TXDma;   //diff between all SPIs
	board.dmasSpi[board.spis[ENUM_SPI3].TXDma].dmaChannel         = SPI3_TX_DMA_CHANNEL;  //diff
	board.dmasSpi[board.spis[ENUM_SPI3].TXDma].dmaDirection       = DMA_MEMORY_TO_PERIPH; //same between all SPIs, diff between TX/RX
	board.dmasSpi[board.spis[ENUM_SPI3].TXDma].dmaPeriphInc       = DMA_PINC_DISABLE;     //same
	board.dmasSpi[board.spis[ENUM_SPI3].TXDma].dmaMemInc          = DMA_MINC_ENABLE;      //same
	board.dmasSpi[board.spis[ENUM_SPI3].TXDma].dmaPeriphAlignment = DMA_PDATAALIGN_BYTE;  //same
	board.dmasSpi[board.spis[ENUM_SPI3].TXDma].dmaMemAlignment    = DMA_MDATAALIGN_BYTE;  //same
	board.dmasSpi[board.spis[ENUM_SPI3].TXDma].dmaMode            = DMA_NORMAL;           //same
	board.dmasSpi[board.spis[ENUM_SPI3].TXDma].dmaPriority        = DMA_PRIORITY_HIGH;    //same, maybe we should change them
	board.dmasSpi[board.spis[ENUM_SPI3].TXDma].fifoMode           = DMA_FIFOMODE_DISABLE; //same
	board.dmasSpi[board.spis[ENUM_SPI3].TXDma].dmaIRQn            = SPI3_TX_DMA_IRQn;     //diff
	board.dmasSpi[board.spis[ENUM_SPI3].TXDma].dmaHandle          = board.spis[ENUM_SPI3].TXDma;   //diff
	board.dmasSpi[board.spis[ENUM_SPI3].TXDma].priority           = SPI3_TX_DMA_PRIORITY;

	board.dmasSpi[board.spis[ENUM_SPI3].RXDma].enabled            = SPI3_DMA_ENABLED;
	board.dmasSpi[board.spis[ENUM_SPI3].RXDma].dmaStream          = board.spis[ENUM_SPI3].RXDma;
	board.dmasSpi[board.spis[ENUM_SPI3].RXDma].dmaChannel         = SPI3_RX_DMA_CHANNEL;
	board.dmasSpi[board.spis[ENUM_SPI3].RXDma].dmaDirection       = DMA_PERIPH_TO_MEMORY;
	board.dmasSpi[board.spis[ENUM_SPI3].RXDma].dmaPeriphInc       = DMA_PINC_DISABLE;
	board.dmasSpi[board.spis[ENUM_SPI3].RXDma].dmaMemInc          = DMA_MINC_ENABLE;
	board.dmasSpi[board.spis[ENUM_SPI3].RXDma].dmaPeriphAlignment = DMA_PDATAALIGN_BYTE;
	board.dmasSpi[board.spis[ENUM_SPI3].RXDma].dmaMemAlignment    = DMA_MDATAALIGN_BYTE;
	board.dmasSpi[board.spis[ENUM_SPI3].RXDma].dmaMode            = DMA_NORMAL;
	board.dmasSpi[board.spis[ENUM_SPI3].RXDma].dmaPriority        = DMA_PRIORITY_HIGH;
	board.dmasSpi[board.spis[ENUM_SPI3].RXDma].fifoMode           = DMA_FIFOMODE_DISABLE;
	board.dmasSpi[board.spis[ENUM_SPI3].RXDma].dmaIRQn            = SPI3_RX_DMA_IRQn;
	board.dmasSpi[board.spis[ENUM_SPI3].RXDma].dmaHandle          = board.spis[ENUM_SPI3].RXDma;
	board.dmasSpi[board.spis[ENUM_SPI3].RXDma].priority           = SPI3_RX_DMA_PRIORITY;



	//UART settings ------------------------------------------------------------------------------------------------------------------------------------------------------------
	board.serials[ENUM_USART1].enabled        = USART1_ENABLED;
	board.serials[ENUM_USART1].SerialInstance = ENUM_USART1;
	board.serials[ENUM_USART1].usartHandle    = ENUM_USART1;
	board.serials[ENUM_USART1].PinMode        = USART1_PINMODE;
	board.serials[ENUM_USART1].Pull           = USART1_PINPULL;
	board.serials[ENUM_USART1].Speed          = GPIO_SPEED_HIGH;
	board.serials[ENUM_USART1].TXAlternate    = USART1_TXALT;
	board.serials[ENUM_USART1].TXPin          = USART1_TXPIN;
	board.serials[ENUM_USART1].TXPort         = USART1_TXPORT;
	board.serials[ENUM_USART1].RXAlternate    = USART1_RXALT;
	board.serials[ENUM_USART1].RXPin          = USART1_RXPIN;
	board.serials[ENUM_USART1].RXPort         = USART1_RXPORT;
	board.serials[ENUM_USART1].TXDmaIrqn      = USART1_TXDMA_IRQN;
	board.serials[ENUM_USART1].TXDma          = USART1_TXDMA;
	board.serials[ENUM_USART1].TXDmaChannel   = USART1_TXDMA_CHANNEL;
	board.serials[ENUM_USART1].RXDmaIrqn      = USART1_RXDMA_IRQN;
	board.serials[ENUM_USART1].RXDma          = USART1_RXDMA;
	board.serials[ENUM_USART1].RXDmaChannel   = USART1_RXDMA_CHANNEL;

	board.serials[ENUM_USART1].serialTxBuffer = 1;
	board.serials[ENUM_USART1].serialRxBuffer = 1;

	board.serials[ENUM_USART1].TXDma 		  = USART1_TXDMA;
	board.serials[ENUM_USART1].RXDma 		  = USART1_RXDMA;

	board.dmasSerial[board.serials[ENUM_USART1].TXDma].enabled            = 1;
	board.dmasSerial[board.serials[ENUM_USART1].TXDma].dmaStream          = board.serials[ENUM_USART1].TXDma;
	board.dmasSerial[board.serials[ENUM_USART1].TXDma].dmaIRQn            = board.serials[ENUM_USART1].TXDmaIrqn;
	board.dmasSerial[board.serials[ENUM_USART1].TXDma].dmaHandle          = board.serials[ENUM_USART1].TXDma;
	board.dmasSerial[board.serials[ENUM_USART1].TXDma].dmaChannel         = board.serials[ENUM_USART1].TXDmaChannel;

	board.dmasSerial[board.serials[ENUM_USART1].RXDma].enabled            = 1;
	board.dmasSerial[board.serials[ENUM_USART1].RXDma].dmaStream          = board.serials[ENUM_USART1].RXDma;
	board.dmasSerial[board.serials[ENUM_USART1].RXDma].dmaIRQn            = board.serials[ENUM_USART1].RXDmaIrqn;
	board.dmasSerial[board.serials[ENUM_USART1].RXDma].dmaHandle          = board.serials[ENUM_USART1].RXDma;
	board.dmasSerial[board.serials[ENUM_USART1].RXDma].dmaChannel         = board.serials[ENUM_USART1].RXDmaChannel;




	board.serials[ENUM_USART2].enabled        = USART2_ENABLED;
	board.serials[ENUM_USART2].SerialInstance = ENUM_USART2;
	board.serials[ENUM_USART2].usartHandle    = ENUM_USART2;
	board.serials[ENUM_USART2].PinMode        = USART2_PINMODE;
	board.serials[ENUM_USART2].Pull           = USART2_PINPULL;
	board.serials[ENUM_USART2].Speed          = GPIO_SPEED_HIGH;
	board.serials[ENUM_USART2].TXAlternate    = USART2_TXALT;
	board.serials[ENUM_USART2].TXPin          = USART2_TXPIN;
	board.serials[ENUM_USART2].TXPort         = USART2_TXPORT;
	board.serials[ENUM_USART2].RXAlternate    = USART2_RXALT;
	board.serials[ENUM_USART2].RXPin          = USART2_RXPIN;
	board.serials[ENUM_USART2].RXPort         = USART2_RXPORT;
	board.serials[ENUM_USART2].TXDmaIrqn      = USART2_TXDMA_IRQN;
	board.serials[ENUM_USART2].TXDma          = USART2_TXDMA;
	board.serials[ENUM_USART2].TXDmaChannel   = USART2_TXDMA_CHANNEL;
	board.serials[ENUM_USART2].RXDmaIrqn      = USART2_RXDMA_IRQN;
	board.serials[ENUM_USART2].RXDma          = USART2_RXDMA;
	board.serials[ENUM_USART2].RXDmaChannel   = USART2_RXDMA_CHANNEL;

	board.serials[ENUM_USART2].serialTxBuffer = 2;
	board.serials[ENUM_USART2].serialRxBuffer = 2;

	board.serials[ENUM_USART2].TXDma 		  = USART2_TXDMA;
	board.serials[ENUM_USART2].RXDma 		  = USART2_RXDMA;

	board.dmasSerial[board.serials[ENUM_USART2].TXDma].enabled            = 1;
	board.dmasSerial[board.serials[ENUM_USART2].TXDma].dmaStream          = board.serials[ENUM_USART2].TXDma;
	board.dmasSerial[board.serials[ENUM_USART2].TXDma].dmaIRQn            = board.serials[ENUM_USART2].TXDmaIrqn;
	board.dmasSerial[board.serials[ENUM_USART2].TXDma].dmaHandle          = board.serials[ENUM_USART2].TXDma;
	board.dmasSerial[board.serials[ENUM_USART2].TXDma].dmaChannel         = board.serials[ENUM_USART2].TXDmaChannel;

	board.dmasSerial[board.serials[ENUM_USART2].RXDma].enabled            = 1;
	board.dmasSerial[board.serials[ENUM_USART2].RXDma].dmaStream          = board.serials[ENUM_USART2].RXDma;
	board.dmasSerial[board.serials[ENUM_USART2].RXDma].dmaIRQn            = board.serials[ENUM_USART2].RXDmaIrqn;
	board.dmasSerial[board.serials[ENUM_USART2].RXDma].dmaHandle          = board.serials[ENUM_USART2].RXDma;
	board.dmasSerial[board.serials[ENUM_USART2].RXDma].dmaChannel         = board.serials[ENUM_USART2].RXDmaChannel;




	board.serials[ENUM_USART3].enabled        = 1;
	board.serials[ENUM_USART3].SerialInstance = ENUM_USART3;
	board.serials[ENUM_USART3].usartHandle    = ENUM_USART3;
	board.serials[ENUM_USART3].PinMode        = USART3_PINMODE;
	board.serials[ENUM_USART3].Pull           = USART3_PINPULL;
	board.serials[ENUM_USART3].Speed          = GPIO_SPEED_HIGH;
	board.serials[ENUM_USART3].TXAlternate    = USART3_TXALT;
	board.serials[ENUM_USART3].TXPin          = USART3_TXPIN;
	board.serials[ENUM_USART3].TXPort         = USART3_TXPORT;
	board.serials[ENUM_USART3].RXAlternate    = USART3_RXALT;
	board.serials[ENUM_USART3].RXPin          = USART3_RXPIN;
	board.serials[ENUM_USART3].RXPort         = USART3_RXPORT;
	board.serials[ENUM_USART3].TXDmaIrqn      = USART3_TXDMA_IRQN;
	board.serials[ENUM_USART3].TXDma          = USART3_TXDMA;
	board.serials[ENUM_USART3].TXDmaChannel   = USART3_TXDMA_CHANNEL;
	board.serials[ENUM_USART3].RXDmaIrqn      = USART3_RXDMA_IRQN;
	board.serials[ENUM_USART3].RXDma          = USART3_RXDMA;
	board.serials[ENUM_USART3].RXDmaChannel   = USART3_RXDMA_CHANNEL;

	board.serials[ENUM_USART3].serialTxBuffer = 3;
	board.serials[ENUM_USART3].serialRxBuffer = 3;

	board.serials[ENUM_USART3].TXDma 		  = USART3_TXDMA;
	board.serials[ENUM_USART3].RXDma 		  = USART3_RXDMA;

	board.dmasSerial[board.serials[ENUM_USART3].TXDma].enabled            = USART3_ENABLED;
	board.dmasSerial[board.serials[ENUM_USART3].TXDma].dmaStream          = board.serials[ENUM_USART3].TXDma;
	board.dmasSerial[board.serials[ENUM_USART3].TXDma].dmaIRQn            = board.serials[ENUM_USART3].TXDmaIrqn;
	board.dmasSerial[board.serials[ENUM_USART3].TXDma].dmaHandle          = board.serials[ENUM_USART3].TXDma;
	board.dmasSerial[board.serials[ENUM_USART3].TXDma].dmaChannel         = board.serials[ENUM_USART3].TXDmaChannel;

	board.dmasSerial[board.serials[ENUM_USART3].RXDma].enabled            = 1;
	board.dmasSerial[board.serials[ENUM_USART3].RXDma].dmaStream          = board.serials[ENUM_USART3].RXDma;
	board.dmasSerial[board.serials[ENUM_USART3].RXDma].dmaIRQn            = board.serials[ENUM_USART3].RXDmaIrqn;
	board.dmasSerial[board.serials[ENUM_USART3].RXDma].dmaHandle          = board.serials[ENUM_USART3].RXDma;
	board.dmasSerial[board.serials[ENUM_USART3].RXDma].dmaChannel         = board.serials[ENUM_USART3].RXDmaChannel;




	board.serials[ENUM_USART4].enabled        = USART4_ENABLED;
	board.serials[ENUM_USART4].SerialInstance = ENUM_USART4;
	board.serials[ENUM_USART4].usartHandle    = ENUM_USART4;
	board.serials[ENUM_USART4].PinMode        = USART4_PINMODE;
	board.serials[ENUM_USART4].Pull           = USART4_PINPULL;
	board.serials[ENUM_USART4].Speed          = GPIO_SPEED_HIGH;
	board.serials[ENUM_USART4].TXAlternate    = USART4_TXALT;
	board.serials[ENUM_USART4].TXPin          = USART4_TXPIN;
	board.serials[ENUM_USART4].TXPort         = USART4_TXPORT;
	board.serials[ENUM_USART4].RXAlternate    = USART4_RXALT;
	board.serials[ENUM_USART4].RXPin          = USART4_RXPIN;
	board.serials[ENUM_USART4].RXPort         = USART4_RXPORT;
	board.serials[ENUM_USART4].TXDmaIrqn      = USART4_TXDMA_IRQN;
	board.serials[ENUM_USART4].TXDma          = USART4_TXDMA;
	board.serials[ENUM_USART4].TXDmaChannel   = USART4_TXDMA_CHANNEL;
	board.serials[ENUM_USART4].RXDmaIrqn      = USART4_RXDMA_IRQN;
	board.serials[ENUM_USART4].RXDma          = USART4_RXDMA;
	board.serials[ENUM_USART4].RXDmaChannel   = USART4_RXDMA_CHANNEL;

	board.serials[ENUM_USART4].serialTxBuffer = 4;
	board.serials[ENUM_USART4].serialRxBuffer = 4;

	board.serials[ENUM_USART4].TXDma 		  = USART4_TXDMA;
	board.serials[ENUM_USART4].RXDma 		  = USART4_RXDMA;

	board.dmasSerial[board.serials[ENUM_USART4].TXDma].enabled            = 1;
	board.dmasSerial[board.serials[ENUM_USART4].TXDma].dmaStream          = board.serials[ENUM_USART4].TXDma;
	board.dmasSerial[board.serials[ENUM_USART4].TXDma].dmaIRQn            = board.serials[ENUM_USART4].TXDmaIrqn;
	board.dmasSerial[board.serials[ENUM_USART4].TXDma].dmaHandle          = board.serials[ENUM_USART4].TXDma;
	board.dmasSerial[board.serials[ENUM_USART4].TXDma].dmaChannel         = board.serials[ENUM_USART4].TXDmaChannel;


	board.dmasSerial[board.serials[ENUM_USART4].RXDma].enabled            = 1;
	board.dmasSerial[board.serials[ENUM_USART4].RXDma].dmaStream          = board.serials[ENUM_USART4].RXDma;
	board.dmasSerial[board.serials[ENUM_USART4].RXDma].dmaIRQn            = board.serials[ENUM_USART4].RXDmaIrqn;
	board.dmasSerial[board.serials[ENUM_USART4].RXDma].dmaHandle          = board.serials[ENUM_USART4].RXDma;
	board.dmasSerial[board.serials[ENUM_USART4].RXDma].dmaChannel         = board.serials[ENUM_USART4].RXDmaChannel;



	board.serials[ENUM_USART5].enabled        = USART5_ENABLED;
	board.serials[ENUM_USART5].SerialInstance = ENUM_USART5;
	board.serials[ENUM_USART5].usartHandle    = ENUM_USART5;
	board.serials[ENUM_USART5].PinMode        = USART5_PINMODE;
	board.serials[ENUM_USART5].Pull           = USART5_PINPULL;
	board.serials[ENUM_USART5].Speed          = GPIO_SPEED_HIGH;
	board.serials[ENUM_USART5].TXAlternate    = USART5_TXALT;
	board.serials[ENUM_USART5].TXPin          = USART5_TXPIN;
	board.serials[ENUM_USART5].TXPort         = USART5_TXPORT;
	board.serials[ENUM_USART5].RXAlternate    = USART5_RXALT;
	board.serials[ENUM_USART5].RXPin          = USART5_RXPIN;
	board.serials[ENUM_USART5].RXPort         = USART5_RXPORT;
	board.serials[ENUM_USART5].TXDmaIrqn      = USART5_TXDMA_IRQN;
	board.serials[ENUM_USART5].TXDma          = USART5_TXDMA;
	board.serials[ENUM_USART5].TXDmaChannel   = USART5_TXDMA_CHANNEL;
	board.serials[ENUM_USART5].RXDmaIrqn      = USART5_RXDMA_IRQN;
	board.serials[ENUM_USART5].RXDma          = USART5_RXDMA;
	board.serials[ENUM_USART5].RXDmaChannel   = USART5_RXDMA_CHANNEL;

	board.serials[ENUM_USART5].serialTxBuffer = 5;
	board.serials[ENUM_USART5].serialRxBuffer = 5;

	board.serials[ENUM_USART5].TXDma 		  = USART5_TXDMA;
	board.serials[ENUM_USART5].RXDma 		  = USART5_RXDMA;

	board.dmasSerial[board.serials[ENUM_USART5].TXDma].enabled            = 1;
	board.dmasSerial[board.serials[ENUM_USART5].TXDma].dmaStream          = board.serials[ENUM_USART5].TXDma;
	board.dmasSerial[board.serials[ENUM_USART5].TXDma].dmaIRQn            = board.serials[ENUM_USART5].TXDmaIrqn;
	board.dmasSerial[board.serials[ENUM_USART5].TXDma].dmaHandle          = board.serials[ENUM_USART5].TXDma;
	board.dmasSerial[board.serials[ENUM_USART5].TXDma].dmaChannel         = board.serials[ENUM_USART5].TXDmaChannel;

	board.dmasSerial[board.serials[ENUM_USART5].RXDma].enabled            = 1;
	board.dmasSerial[board.serials[ENUM_USART5].RXDma].dmaStream          = board.serials[ENUM_USART5].RXDma;
	board.dmasSerial[board.serials[ENUM_USART5].RXDma].dmaIRQn            = board.serials[ENUM_USART5].RXDmaIrqn;
	board.dmasSerial[board.serials[ENUM_USART5].RXDma].dmaHandle          = board.serials[ENUM_USART5].RXDma;
	board.dmasSerial[board.serials[ENUM_USART5].RXDma].dmaChannel         = board.serials[ENUM_USART5].RXDmaChannel;




	board.serials[ENUM_USART6].enabled        = USART6_ENABLED;
	board.serials[ENUM_USART6].SerialInstance = ENUM_USART6;
	board.serials[ENUM_USART6].usartHandle    = ENUM_USART6;
	board.serials[ENUM_USART6].PinMode        = USART6_PINMODE;
	board.serials[ENUM_USART6].Pull           = USART6_PINPULL;
	board.serials[ENUM_USART6].Speed          = GPIO_SPEED_HIGH;
	board.serials[ENUM_USART6].TXAlternate    = USART6_TXALT;
	board.serials[ENUM_USART6].TXPin          = USART6_TXPIN;
	board.serials[ENUM_USART6].TXPort         = USART6_TXPORT;
	board.serials[ENUM_USART6].RXAlternate    = USART6_RXALT;
	board.serials[ENUM_USART6].RXPin          = USART6_RXPIN;
	board.serials[ENUM_USART6].RXPort         = USART6_RXPORT;
	board.serials[ENUM_USART6].TXDmaIrqn      = USART6_TXDMA_IRQN;
	board.serials[ENUM_USART6].TXDma          = USART6_TXDMA;
	board.serials[ENUM_USART6].TXDmaChannel   = USART6_TXDMA_CHANNEL;
	board.serials[ENUM_USART6].RXDmaIrqn      = USART6_RXDMA_IRQN;
	board.serials[ENUM_USART6].RXDma          = USART6_RXDMA;
	board.serials[ENUM_USART6].RXDmaChannel   = USART6_RXDMA_CHANNEL;

	board.serials[ENUM_USART6].serialTxBuffer = 6;
	board.serials[ENUM_USART6].serialRxBuffer = 6;

	board.serials[ENUM_USART6].TXDma 		  = USART6_TXDMA;
	board.serials[ENUM_USART6].RXDma 		  = USART6_RXDMA;

	board.dmasSerial[board.serials[ENUM_USART6].TXDma].enabled            = 1;
	board.dmasSerial[board.serials[ENUM_USART6].TXDma].dmaStream          = board.serials[ENUM_USART6].TXDma;
	board.dmasSerial[board.serials[ENUM_USART6].TXDma].dmaIRQn            = board.serials[ENUM_USART6].TXDmaIrqn;
	board.dmasSerial[board.serials[ENUM_USART6].TXDma].dmaHandle          = board.serials[ENUM_USART6].TXDma;
	board.dmasSerial[board.serials[ENUM_USART6].TXDma].dmaChannel         = board.serials[ENUM_USART6].TXDmaChannel;

	board.dmasSerial[board.serials[ENUM_USART6].RXDma].enabled            = 1;
	board.dmasSerial[board.serials[ENUM_USART6].RXDma].dmaStream          = board.serials[ENUM_USART6].RXDma;
	board.dmasSerial[board.serials[ENUM_USART6].RXDma].dmaIRQn            = board.serials[ENUM_USART6].RXDmaIrqn;
	board.dmasSerial[board.serials[ENUM_USART6].RXDma].dmaHandle          = board.serials[ENUM_USART6].RXDma;
	board.dmasSerial[board.serials[ENUM_USART6].RXDma].dmaChannel         = board.serials[ENUM_USART6].RXDmaChannel;

	return(0);
}
