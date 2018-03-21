#include "includes.h"

uint8_t lastLEDMode = 0;
uint8_t ledColor = 254;
DMA_HandleTypeDef  ws2812_led;
TIM_HandleTypeDef  pwmTimerBase;

ws2812_led_record ws2812LedRecord;

uint32_t onePulseWidth[17];
uint32_t zeroPulseWidth[17];
uint32_t alonePulseWidth[17];
uint32_t normalPulseWidth[17];
uint32_t endPulseWidth[17];
uint32_t loPulseWidth[17];
uint8_t  tempBuffer[1500];
uint32_t longMotorOutputBuffer[1500];

static void TimDmaInit(TIM_HandleTypeDef *htim, uint32_t handlerIndex, board_dma actuatorDma);
static void InitOutputForDma(motor_type actuator, uint32_t pwmHz, uint32_t timerHz, uint32_t inverted);

#define NO_PULSE 0
#define LO_PULSE 1
#define HI_PULSE 2

uint32_t ddshot48To49[16] = 
{
	8,
	16,
	25,
	33,
	41,
	49,
	57,
	65,
	74,
	82,
	90,
	98,
	106,
	114,
	123,
	131
};

//THROTTLE 11, THROTTLE 10, THROTTLE 9, THROTTLE 8,         THROTTLE 7, THROTTLE 6, THROTTLE 5, THROTTLE 4,                THROTTLE 3, THROTTLE 2, Unused bit, Direction Bit,              CRC 3, CRC 2, CRC 1, CRC 0
//void OutputDDShotDma(motor_type actuator, uint32_t reverse, float throttle)
void OutputDDShotDma(motor_type actuator, int reverse, int digitalThrottle)
{
	//volatile uint32_t digitalThrottle = CONSTRAIN(lrintf(throttle * 1023.0f), 0, 1023);
	digitalThrottle = CONSTRAIN(digitalThrottle, 0, 1023);
	volatile uint32_t nibble;         //set
	volatile uint32_t crcNibble = 0;

	//digitalThrottle holds throttle like 11 1111 1111
	//sends like 1111, 1111, 11xx

	//11 1111 1111 to 1111 
	motorOutputBuffer[actuator.actuatorArrayNum][0] = 0;
	nibble = ( (digitalThrottle >> 6) & 0x0F );
	crcNibble += nibble;
	motorOutputBuffer[actuator.actuatorArrayNum][1] = ddshot48To49[nibble];
	nibble = ( (digitalThrottle >> 2) & 0x0F );
	crcNibble += nibble;
	motorOutputBuffer[actuator.actuatorArrayNum][2] = ddshot48To49[nibble];
	nibble = ( (digitalThrottle << 2) & 0x0C ) | (reverse & 0x01);
	crcNibble += nibble;
	motorOutputBuffer[actuator.actuatorArrayNum][3] = ddshot48To49[nibble];
	motorOutputBuffer[actuator.actuatorArrayNum][4] = ddshot48To49[(crcNibble & 0x0F)];
	motorOutputBuffer[actuator.actuatorArrayNum][5] = 0;

	if(actuator.isNChannel)
		HAL_TIMEx_PWMN_Start_DMA(&pwmTimers[actuator.actuatorArrayNum], actuator.timChannel, (uint32_t *)motorOutputBuffer[actuator.actuatorArrayNum], 6);
	else
		HAL_TIM_PWM_Start_DMA(&pwmTimers[actuator.actuatorArrayNum], actuator.timChannel, (uint32_t *)motorOutputBuffer[actuator.actuatorArrayNum], 6);
}

void OutputSerialDmaByte(uint8_t *serialOutBuffer, uint32_t outputLength, motor_type actuator, uint32_t msb, uint32_t sendFrame, uint32_t noEndPadding)
{

	int32_t  bitIdx;
	uint32_t bufferIdx = 0;
	uint32_t outputIndex;
	uint32_t bitsPerFrame = 8;
	uint32_t *outBuffer;

	tempBuffer[bufferIdx++] = NO_PULSE;

    for (outputIndex = 0; outputIndex < outputLength; outputIndex++) //Send Data MSB by default
    {

    	if (!msb)
    		serialOutBuffer[outputIndex] = BitReverse8(serialOutBuffer[outputIndex]); //for LSB we do this

    	if (sendFrame)
    		tempBuffer[bufferIdx++] = HI_PULSE;//frame start

        for (bitIdx = (bitsPerFrame - 1); bitIdx >= 0; bitIdx--)
        {
        	tempBuffer[bufferIdx++] = (serialOutBuffer[outputIndex] & (1 << bitIdx)) ? LO_PULSE : HI_PULSE; //load data into framedata one bit at a time
        }

        if (sendFrame)
        	tempBuffer[bufferIdx++] = LO_PULSE; //stop bit, single stop bit only right now.

    }

	tempBuffer[bufferIdx++] = NO_PULSE;
	tempBuffer[bufferIdx++] = NO_PULSE;

	if (outputLength < 18)
	{
		outBuffer = motorOutputBuffer[actuator.actuatorArrayNum];
	}
	else
	{
		outBuffer = longMotorOutputBuffer;
	}

	outBuffer[0] = 0;
	outBuffer[bufferIdx] = 0;

	for (uint32_t x = 1; x < (bufferIdx - 1); x++) //first bit is always a 0, last bit is always a 0
	{

		if (tempBuffer[x] == HI_PULSE) //this is a high bit high bit
		{
			if (tempBuffer[x+1] == HI_PULSE) //After bit is high so this is a normal bit
			{
				outBuffer[x] = normalPulseWidth[actuator.actuatorArrayNum+1];
			}
			else if ( ( tempBuffer[x-1] < HI_PULSE ) && (tempBuffer[x+1] < HI_PULSE) ) //B4 bit is low and AR bit low, so this is an ALONE BIT
			{
				outBuffer[x] = alonePulseWidth[actuator.actuatorArrayNum+1];;
			}
			else if ( ( tempBuffer[x-1] == HI_PULSE ) && (tempBuffer[x+1] < HI_PULSE) ) //B4 bit is high and AR bit low, so this is an END BIT
			{
				outBuffer[x] = endPulseWidth[actuator.actuatorArrayNum+1];
			}
			else
			{
				outBuffer[x] = normalPulseWidth[actuator.actuatorArrayNum+1];
			}
		}
		else
		{
			outBuffer[x] = loPulseWidth[actuator.actuatorArrayNum+1];
		}

	}

	if (noEndPadding)
	{
		outBuffer[bufferIdx - 2] = 0;
		outBuffer[bufferIdx - 1] = 0;
		outBuffer[bufferIdx] = 0;
	}

	if(actuator.isNChannel)
		HAL_TIMEx_PWMN_Start_DMA(&pwmTimers[actuator.actuatorArrayNum], actuator.timChannel, (uint32_t *)outBuffer, bufferIdx);
	else
		HAL_TIM_PWM_Start_DMA(&pwmTimers[actuator.actuatorArrayNum], actuator.timChannel, (uint32_t *)outBuffer, bufferIdx);
	
}

int InitWs2812(void)
{

	uint32_t actuatorNumOutput;

	//volatile int cat = 000;
	//fix for revolt. If 1wire has run we can't use LEDs or motor outputs won't work right.
	if (oneWireHasRun)
		return(0);

	ws2812LedRecord.enabled = 0;
	//TODO: We need more actuators, no more max motor number, instead we use max_actuator number.
	for (actuatorNumOutput = 0; actuatorNumOutput < MAX_MOTOR_NUMBER; actuatorNumOutput++)
	{

		if (board.motors[actuatorNumOutput].enabled == ENUM_ACTUATOR_TYPE_WS2812)
		{

			if (!DoesDmaConflictWithActiveDmas(board.motors[actuatorNumOutput]))
			{
				ws2812LedRecord.enabled = 1;
				ws2812LedRecord.ws2812Actuator = board.motors[actuatorNumOutput];
				SetActiveDmaToActuatorDma(ws2812LedRecord.ws2812Actuator);
				InitDmaOutputForSoftSerial(DMA_OUTPUT_WS2812_LEDS, ws2812LedRecord.ws2812Actuator);
				uint8_t rgbArray[] = {0xFF, 0xAA, 0x11, 0x11, 0xAA, 0xFF};
				OutputSerialDmaByte(rgbArray, 6, ws2812LedRecord.ws2812Actuator, 0, 0, 1);
			}

		}

	}

	return(1);
}

void InitDmaInputOnMotors(motor_type actuator) {

	GPIO_InitTypeDef       GPIO_InitStruct;
	TIM_TypeDef           *timer;
	TIM_IC_InitTypeDef     sConfig;
	TIM_SlaveConfigTypeDef sSlaveConfig;

	timer = timers[actuator.timer];

    // GPIO Init
    HAL_GPIO_DeInit(ports[actuator.port], actuator.pin);

    GPIO_InitStruct.Pin       = actuator.pin;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP; //GPIO_MODE_AF_PP
    GPIO_InitStruct.Pull      = GPIO_PULLUP; //GPIO_PULLUP //pull up for non inverted, pull down for inverted
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = actuator.AF;

    HAL_GPIO_Init(ports[actuator.port], &GPIO_InitStruct);

    /*##-2- Configure the NVIC for TIMx #########################################*/
    //HAL_NVIC_SetPriority(actuator.timerIRQn, 5, 0);
    //HAL_NVIC_EnableIRQ(actuator.timerIRQn);

    HAL_TIM_Base_DeInit(&pwmTimers[actuator.actuatorArrayNum]);
	pwmTimers[actuator.actuatorArrayNum].Instance           	= timer;
	pwmTimers[actuator.actuatorArrayNum].Init.Period     	= 0xFFFF;
	pwmTimers[actuator.actuatorArrayNum].Init.Prescaler   	= 191; //1MHz
	pwmTimers[actuator.actuatorArrayNum].Init.ClockDivision 	= TIM_CLOCKDIVISION_DIV1;
	pwmTimers[actuator.actuatorArrayNum].Init.CounterMode 	= TIM_COUNTERMODE_UP;

	pwmTimers[actuator.actuatorArrayNum].State               = HAL_TIM_STATE_RESET;

	HAL_TIM_Base_Init(&pwmTimers[actuator.actuatorArrayNum]);
	HAL_TIM_IC_Init(&pwmTimers[actuator.actuatorArrayNum]);

	// Configure the Input Capture channels
	sConfig.ICPrescaler = TIM_ICPSC_DIV1;
	sConfig.ICFilter    = 0x3;
	sConfig.ICPolarity  = TIM_ICPOLARITY_FALLING;
	//TIM_ICPOLARITY_FALLING
	//TIM_ICPOLARITY_RISING
	//TIM_ICPOLARITY_BOTHEDGE
	//sConfig.ICSelection = TIM_ICSELECTION_INDIRECTTI;
	//if(HAL_TIM_IC_ConfigChannel(&pwmTimers[actuator.actuatorArrayNum], &sConfig, actuator.timChannelC) != HAL_OK)
	//{
	//	/* Configuration Error */
	//	ErrorHandler(TIMER_INPUT_INIT_FAILIURE);
	//}

	sConfig.ICPolarity = TIM_ICPOLARITY_RISING;
	sConfig.ICSelection = TIM_ICSELECTION_DIRECTTI;
	if(HAL_TIM_IC_ConfigChannel(&pwmTimers[actuator.actuatorArrayNum], &sConfig, actuator.timChannel) != HAL_OK)
	{
		/* Configuration Error */
		ErrorHandler(TIMER_INPUT_INIT_FAILIURE);
	}

	/* Select the slave Mode: Reset Mode */
	sSlaveConfig.SlaveMode     = TIM_SLAVEMODE_RESET;
	sSlaveConfig.InputTrigger  = TIM_TS_TI2FP2;
	if(HAL_TIM_SlaveConfigSynchronization(&pwmTimers[actuator.actuatorArrayNum], &sSlaveConfig) != HAL_OK)
	{
		/* Configuration Error */
		ErrorHandler(TIMER_INPUT_INIT_FAILIURE);
	}

	HAL_NVIC_SetPriority(actuator.timerIRQn,5,1);
	HAL_NVIC_EnableIRQ(actuator.timerIRQn);
	/*##-4- Start the Input Capture in interrupt mode ##########################*/
	if(HAL_TIM_IC_Start_IT(&pwmTimers[actuator.actuatorArrayNum], actuator.timChannel) != HAL_OK)
	{
		/* Starting Error */
		ErrorHandler(TIMER_INPUT_INIT_FAILIURE);
	}

	///*##-5- Start the Input Capture in interrupt mode ##########################*/
	//if(HAL_TIM_IC_Start_IT(&pwmTimers[actuator.actuatorArrayNum], actuator.timChannelC) != HAL_OK)
	//{
	//	/* Starting Error */
	//	ErrorHandler(TIMER_INPUT_INIT_FAILIURE);
	//}


	return;
	if(HAL_TIM_IC_Init(&pwmTimers[actuator.actuatorArrayNum]) != HAL_OK)
	{
		ErrorHandler(TIMER_INPUT_INIT_FAILIURE);
	}

	/*##-2- Configure the Input Capture channels ###############################*/
	/* Common configuration */
	sConfig.ICPrescaler = TIM_ICPSC_DIV1;
	sConfig.ICFilter = 0x0; //0x0 to 0xF

	/* Configure the Input Capture of channel of actuator */
	sConfig.ICPolarity  = TIM_ICPOLARITY_FALLING;     //trigger on falling and rising edge
	//TIM_ICPOLARITY_BOTHEDGE
	//TIM_ICPOLARITY_RISING
	//TIM_ICPOLARITY_FALLING
	sConfig.ICSelection = TIM_ICSELECTION_DIRECTTI;
	//sConfig.ICFilter    = 0xF;  //0x0 - 0xF
	//TIM_ICSELECTION_TRC
	//TIM_ICSELECTION_INDIRECTTI
	//TIM_ICSELECTION_DIRECTTI
	if(HAL_TIM_IC_ConfigChannel(&pwmTimers[actuator.actuatorArrayNum], &sConfig, actuator.timChannel) != HAL_OK)
	{
		/* Configuration Error */
		ErrorHandler(TIMER_INPUT_INIT_FAILIURE);
	}

	/*##-3- Configure the slave mode ###########################################*/
	/* Select the slave Mode: Reset Mode */
	sSlaveConfig.SlaveMode     = TIM_SLAVEMODE_TRIGGER;
	//TIM_SLAVEMODE_TRIGGER
	//TIM_SLAVEMODE_RESET
	//TIM_SLAVEMODE_GATED
	//TIM_SLAVEMODE_DISABLE
	//TIM_SLAVEMODE_EXTERNAL1
	sSlaveConfig.InputTrigger  = TIM_TS_TI2FP2;
	//sSlaveConfig.TriggerPolarity
	if(HAL_TIM_SlaveConfigSynchronization(&pwmTimers[actuator.actuatorArrayNum], &sSlaveConfig) != HAL_OK)
	{
		/* Configuration Error */
		ErrorHandler(TIMER_INPUT_INIT_FAILIURE);
	}

	/*##-4- Start the Input Capture in interrupt mode ##########################*/
	if(HAL_TIM_IC_Start_IT(&pwmTimers[actuator.actuatorArrayNum], actuator.timChannel) != HAL_OK)
	{
		/* Starting Error */
		ErrorHandler(TIMER_INPUT_INIT_FAILIURE);
	}

	HAL_TIM_Base_Init(&pwmTimers[actuator.actuatorArrayNum]);
}

uint32_t IsDshotEnabled()
{
	if ( (mainConfig.mixerConfig.escProtocol == ESC_DSHOT1200) || (mainConfig.mixerConfig.escProtocol == ESC_DDSHOT) || (mainConfig.mixerConfig.escProtocol == ESC_DSHOT600) || (mainConfig.mixerConfig.escProtocol == ESC_DSHOT300) || (mainConfig.mixerConfig.escProtocol == ESC_DSHOT150) )
	{
		return(1);
	}
	return(0);
}

uint32_t IsDshotActiveOnActuator(motor_type actuator)
{

	if ( !IsDshotEnabled() )
		return 0;

	if ( (actuator.enabled == ENUM_ACTUATOR_TYPE_MOTOR) )
		return 1;

	return 0;

}

uint32_t DoesDmaConflictWithDshot(motor_type dShotActuator, motor_type actuator)
{

	if (dShotActuator.Dma == actuator.Dma)
		return 1;
	else
		return 0;
}

void SetActiveDmaToActuatorDma(motor_type actuator)
{

	memcpy( &board.dmasActive[actuator.Dma], &board.dmasMotor[actuator.actuatorArrayNum], sizeof(board_dma) );

	//Set DMA settings for this use
	board.dmasActive[actuator.Dma].dmaDirection       = DMA_MEMORY_TO_PERIPH;
	board.dmasActive[actuator.Dma].dmaPeriphInc       = DMA_PINC_DISABLE;
	board.dmasActive[actuator.Dma].dmaMemInc          = DMA_MINC_ENABLE;
	board.dmasActive[actuator.Dma].dmaPeriphAlignment = DMA_PDATAALIGN_WORD;
	board.dmasActive[actuator.Dma].dmaMemAlignment    = DMA_MDATAALIGN_WORD;
	board.dmasActive[actuator.Dma].dmaMode            = DMA_NORMAL;
	board.dmasActive[actuator.Dma].dmaPriority        = DMA_PRIORITY_HIGH;
	board.dmasActive[actuator.Dma].fifoMode           = DMA_FIFOMODE_DISABLE;
	board.dmasActive[actuator.Dma].fifoThreshold      = DMA_FIFO_THRESHOLD_FULL;
	board.dmasActive[actuator.Dma].MemBurst           = DMA_MBURST_SINGLE;
	board.dmasActive[actuator.Dma].PeriphBurst        = DMA_PBURST_SINGLE;

}


uint32_t DoesDmaConflictWithActiveDmas(motor_type actuator) {

	//todo: I don't like how this function operates
	uint32_t x;

	for (x=0;x<16;x++)
		if (board.dmasActive[actuator.Dma].enabled)
			if (board.dmasActive[actuator.Dma].dmaStream == board.dmasActive[x].dmaStream)
				return (1);

	return(0);

}

//TODO: Always DISABLE whatever is attached to a DMA before reusing the DMA for something else. We may need a way to track what's using a DMA.
void DeInitAllowedSoftOutputs(void)
{

	uint32_t actuatorNumOutput;
	uint32_t outputNumber;

	for (actuatorNumOutput = 0; actuatorNumOutput < MAX_MOTOR_NUMBER; actuatorNumOutput++)
	{
		outputNumber = mainConfig.mixerConfig.motorOutput[actuatorNumOutput];
		switch (board.motors[outputNumber].enabled)
		{

			case ENUM_ACTUATOR_TYPE_WS2812:
			case ENUM_ACTUATOR_TYPE_SPORT:

				DeInitDmaOutputForSoftSerial(board.motors[outputNumber]); //disable DMA for motor if it's active and set to one of the above cases. This assume the DMA doesn't get deactivated somehow.

				break;

			default:
				break;

		}

	}

}

//TODO: make sure EXTIs don't conflict
/*
void InitAllowedSoftOutputs(void)
{

	uint32_t actuatorNumOutput;
	uint32_t actuatorNumCheck;
	uint32_t okayToEnable;
	uint32_t outputNumber;

	okayToEnable = 1;

	//TODO: We need more actuators, no more max motor number, instead we use max_actuator number.
	for (actuatorNumOutput = 0; actuatorNumOutput < MAX_MOTOR_NUMBER; actuatorNumOutput++) {
		outputNumber = mainConfig.mixerConfig.motorOutput[actuatorNumOutput];
		switch (board.motors[outputNumber].enabled) {
			case ENUM_ACTUATOR_TYPE_WS2812:
			case ENUM_ACTUATOR_TYPE_SPORT:
				for (actuatorNumCheck = 0; actuatorNumCheck < MAX_MOTOR_NUMBER; actuatorNumCheck++) { //make sure soft sport and soft ws2812 don't interfer with active motor configuration

					if (!DoesDmaConflictWithActiveDmas(board.motors[outputNumber])) {
						okayToEnable = 0;
					}

				}
				if (okayToEnable) {

					if (board.motors[outputNumber].enabled == ENUM_ACTUATOR_TYPE_SPORT) {
						//TODO: make telemetry and soft serial setup smarter
						softSerialEnabled = 1;
						telemEnabled      = 1;
						uint32_t currentTime = Micros();

						__disable_irq();
				    	//prepare soft serial buffer and index
						softSerialLastByteProcessedLocation = 0;
						softSerialCurBuf = 0;
						softSerialInd[softSerialCurBuf] = 0;
						softSerialBuf[softSerialCurBuf][softSerialInd[softSerialCurBuf]++] = currentTime;
						//prepare soft serial buffer and index
						__enable_irq();

						SetActiveDmaToActuatorDma(board.motors[outputNumber]);
						InitDmaOutputForSoftSerial(board.motors[outputNumber].enabled, board.motors[outputNumber]);
					}

				}
				break;
			default:
				break;
		}

	}

}
*/

void DeInitDmaOutputForSoftSerial(motor_type actuator)
{

	//TODO, check the EXTI doesn't conflict with gyro, output only serial doesn't need this

	if (actuator.timer != ENUM_ACTUATOR_TYPE_WS2812)
		EXTI_Deinit(ports[actuator.port], actuator.pin, actuator.EXTIn); //disable EXTI

	//HAL_GPIO_DeInit(ports[actuator.port], actuator.pin);
	//HAL_TIM_PWM_DeInit(&pwmTimers[actuator.actuatorArrayNum]);
	//HAL_TIM_Base_DeInit(&pwmTimers[actuator.actuatorArrayNum]);

	if (pwmTimers[actuator.actuatorArrayNum].hdma[actuator.CcDmaHandle] != 0)
	{
		//keep timer and GPIO settings as they were.

		HAL_DMA_DeInit(pwmTimers[actuator.actuatorArrayNum].hdma[actuator.CcDmaHandle]); //disable DMA
		board.dmasActive[actuator.Dma].enabled = 0;
	}

}

void InitDmaOutputForSoftSerial(uint32_t usedFor, motor_type actuator)
{

	uint32_t timerHz;
	uint32_t pwmHz;
	uint32_t normalPulse;
	uint32_t alonePulse;
	uint32_t endPulse;
	uint32_t loPulse;
	uint32_t inverted;

	if (usedFor == DMA_OUTPUT_WS2812_LEDS)
	{
		timerHz     = 24000000;
		pwmHz       = 800000;
		//onePulse  = 17;
		//zeroPulse = 8;
		normalPulse = 17;
		alonePulse  = 17;
		endPulse    = 17;
		loPulse     = 8;

		inverted    = 1;
	}
	else if (usedFor == DMA_OUTPUT_ESC_1WIRE)
	{

		timerHz     = 48000000; //48 MHz frequency
		pwmHz       = 19200;    //baudrate

		normalPulse = 2500;     //2500 max
		alonePulse  = 2490;
		endPulse    = 1;
		loPulse     = 0;

		inverted    = 0;

	}
	else if (usedFor == DMA_OUTPUT_SPORT)
	{

		timerHz     = 48000000; //48 MHz frequency is okay for 57600 Baud, but actually runs at 57623, full pulse is 833
		pwmHz       = 57600;    //baudrate

		normalPulse = 833;      //833 max, but we can't fill the CCR
		alonePulse  = 820;
		endPulse    = 1;
		loPulse     = 0;

		inverted    = 1;

	}

	alonePulseWidth[actuator.actuatorArrayNum+1]  = alonePulse;
	normalPulseWidth[actuator.actuatorArrayNum+1] = normalPulse;
	endPulseWidth[actuator.actuatorArrayNum+1]    = endPulse;
	loPulseWidth[actuator.actuatorArrayNum+1]     = loPulse;

	InitOutputForDma(actuator, pwmHz, timerHz, inverted);

	//17 / 24 = 0.708 us
    // note that the timer is running at 24 MHZ, with a period of 30 cycles
    // a "1" must be high for ~700 ns, which corresponds to roughly 17 timer cycles
    // a "0" must be high for ~350 ns, which corresponds to roughly 8 timer cycles
	//At 09600, bit time is 104.166666666666 microseconds. 104166 ns
	//At 19200, bit time is 52.083333333333 microseconds. 52083 ns
	//0.0416666666666667 us per tick with timerHz at 24000000 and pwmHz at 800000 with
	//1 MHz timer is 1us per tick. :)
	//52 ticks is a 1 and 0 ticks is a 0 for serial

	//19200KBAUD
	//timerHz   = 48000000; //48 MHz frequency is perfectly fine for 19200 Baud.
	//pwmHz     = 19200;   //baudrate
	//onePulse  = 1;
	//zeroPulse = 2490; //2500 max, but we can't fill the CCR

	//57600KBAUD
	//timerHz   = 48000000; //48 MHz frequency is okay for 57600 Baud, but actually runs at 57623, full pulse is 833
	//pwmHz     = 57600;   //baudrate
	//onePulse  = 0;
	//zeroPulse = 832; //833 max, but we can't fill the CCR
	//inverted  = 1;

	//100KBAUD
	//timerHz   = 48000000; //48 MHz frequency is perfectly fine for 19200 Baud.
	//pwmHz     = 100000;   //baudrate
	//onePulse  = 1;
	//zeroPulse = 470; //480 max, but we can't fill the CCR

	//200KBAUD
	//timerHz   = 48000000; //48 MHz frequency is perfectly fine for 19200 Baud.
	//pwmHz     = 200000;   //baudrate
	//onePulse  = 1;
	//zeroPulse = 230; //240 max, but we can't fill the CCR
}

void InitDshotOutputOnMotors(uint32_t usedFor)
{

	uint32_t timerHz;
	uint32_t pwmHz;
	uint32_t loPulse;
	uint32_t endPulse;
	uint32_t normalPulse;
	uint32_t alonePulse;
	uint32_t inverted;
	uint32_t outputNumber;

	if (usedFor == ESC_DDSHOT)
	{
		//32 steps per nibble
		//first half data, second half spacing
		//20.83333 ns per count cycle * 8 count cycles per step]
		//166.66664 ns per step * 32 steps
		//5.333333248 us per nibble * 4 nibbles
		//21.33333299 us per packet
		//at 31.25 cycle time that gives about 10us for packet sync
		//48MHz timer, broken down into 256 steps is 187500 Hz
		//bit rate is 187.5 KHz Baud rate is 751.168 Khz

		timerHz     = 48000000;
		pwmHz       = 187500;
		normalPulse = 15;
		alonePulse  = 15;
		endPulse    = 15;
		loPulse     = 30;
		inverted    = 1;

	}
	if (usedFor == ESC_DSHOT1200)
	{

		timerHz     = 48000000;
		pwmHz       = 1200000;
		normalPulse = 15;
		alonePulse  = 15;
		endPulse    = 15;
		loPulse     = 30;
		inverted    = 1;

	}
	else if (usedFor == ESC_DSHOT600)
	{
		//timerHz     = 48000000;
		//pwmHz       = 600000;
		//normalPulse = 30;
		//alonePulse  = 30;
		//endPulse    = 30;
		//loPulse     = 60;
		//inverted    = 1;
		timerHz     = 24000000;
		pwmHz       = 300000;
		normalPulse = 30;
		alonePulse  = 30;
		endPulse    = 30;
		loPulse     = 60;
		inverted    = 1;

	}
	else if (usedFor == ESC_DSHOT300)
	{

		timerHz     = 24000000;
		pwmHz       = 300000;
		normalPulse = 30;
		alonePulse  = 30;
		endPulse    = 30;
		loPulse     = 60;
		inverted    = 1;

	}
	else if (usedFor == ESC_DSHOT150)
	{

		timerHz     = 24000000;
		pwmHz       = 150000;
		normalPulse = 60;
		alonePulse  = 60;
		endPulse    = 60;
		loPulse     = 120;
		inverted    = 1;

	}

	for (uint32_t motorNum = 0; motorNum < MAX_MOTOR_NUMBER; motorNum++)
	{
		outputNumber = mainConfig.mixerConfig.motorOutput[motorNum];
		if ( (board.motors[outputNumber].enabled == ENUM_ACTUATOR_TYPE_MOTOR) && (board.dmasMotor[board.motors[outputNumber].actuatorArrayNum].enabled) )
		{
			SetActiveDmaToActuatorDma(board.motors[outputNumber]);
			alonePulseWidth[board.motors[outputNumber].actuatorArrayNum+1]  = alonePulse;
			normalPulseWidth[board.motors[outputNumber].actuatorArrayNum+1] = normalPulse;
			endPulseWidth[board.motors[outputNumber].actuatorArrayNum+1]    = endPulse;
			loPulseWidth[board.motors[outputNumber].actuatorArrayNum+1]     = loPulse;
			InitOutputForDma(board.motors[outputNumber], pwmHz, timerHz, inverted);
		}
	}

	//pwmHz     = 600000;
	//onePulse  = 30;
	//zeroPulse = 15;
	//24,000,000 / 600,000 = 40 ticks per cycle
	//1/24 = 0.04166666 us
	//0.04166666 * 40 = 1.66666 us cycles
	//(1/24)*x = 1.250; x=30;
	//(1/24)*x = 0.625; x=15;
}

static void InitOutputForDma(motor_type actuator, uint32_t pwmHz, uint32_t timerHz, uint32_t inverted)
{

    GPIO_InitTypeDef GPIO_InitStructure;
	uint16_t timerPrescaler;
	TIM_TypeDef *timer;

    //Timer Init
    timer = timers[actuator.timer];

	timerPrescaler = (uint16_t)(SystemCoreClock / TimerPrescalerDivisor(actuator.timer) / timerHz) - 1;

	//HAL_TIM_PWM_DeInit(&pwmTimers[actuator.actuatorArrayNum]);
	pwmTimers[actuator.actuatorArrayNum].Instance           	= timer;
	pwmTimers[actuator.actuatorArrayNum].Init.Prescaler     	= timerPrescaler;
	pwmTimers[actuator.actuatorArrayNum].Init.CounterMode   	= TIM_COUNTERMODE_UP;
	pwmTimers[actuator.actuatorArrayNum].Init.Period        	= (timerHz/pwmHz)-1;
	pwmTimers[actuator.actuatorArrayNum].Init.ClockDivision 	= TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_Base_Init(&pwmTimers[actuator.actuatorArrayNum]);

	HAL_TIM_PWM_Init(&pwmTimers[actuator.actuatorArrayNum]);

	sConfigOCHandles[actuator.actuatorArrayNum].OCMode       = TIM_OCMODE_PWM1;
	sConfigOCHandles[actuator.actuatorArrayNum].OCFastMode   = TIM_OCFAST_DISABLE;
	sConfigOCHandles[actuator.actuatorArrayNum].Pulse        = 0;

	if(actuator.isNChannel)
	{
		sConfigOCHandles[actuator.actuatorArrayNum].OCIdleState  = TIM_OCIDLESTATE_RESET;
		sConfigOCHandles[actuator.actuatorArrayNum].OCPolarity   = (inverted) ? TIM_OCPOLARITY_HIGH : TIM_OCPOLARITY_LOW;
		sConfigOCHandles[actuator.actuatorArrayNum].OCNIdleState = TIM_OCNIDLESTATE_RESET;
		sConfigOCHandles[actuator.actuatorArrayNum].OCNPolarity  = (inverted) ? TIM_OCNPOLARITY_HIGH : TIM_OCNPOLARITY_LOW;
	}
	else
	{
		sConfigOCHandles[actuator.actuatorArrayNum].OCIdleState  = TIM_OCIDLESTATE_SET;
		sConfigOCHandles[actuator.actuatorArrayNum].OCPolarity   = (inverted) ? TIM_OCPOLARITY_HIGH : TIM_OCPOLARITY_LOW;
		sConfigOCHandles[actuator.actuatorArrayNum].OCNIdleState = TIM_OCNIDLESTATE_SET;
		sConfigOCHandles[actuator.actuatorArrayNum].OCNPolarity  = (inverted) ? TIM_OCNPOLARITY_HIGH : TIM_OCNPOLARITY_LOW;
	}

	//if(actuator.isNChannel)
	//{
	//	//sConfigOCHandles[actuator.actuatorArrayNum].OCIdleState  = TIM_OCIDLESTATE_RESET;
	//	sConfigOCHandles[actuator.actuatorArrayNum].OCNIdleState = TIM_OCNIDLESTATE_RESET;
	//	sConfigOCHandles[actuator.actuatorArrayNum].OCNPolarity  = inverted ? TIM_OCNPOLARITY_LOW : TIM_OCNPOLARITY_HIGH; //High polarity if inverted, low if not
	//	
	//}
	//else
	//{
	//	sConfigOCHandles[actuator.actuatorArrayNum].OCIdleState  = TIM_OCIDLESTATE_SET;
	//	//sConfigOCHandles[actuator.actuatorArrayNum].OCNIdleState = TIM_OCNIDLESTATE_SET;
	//	sConfigOCHandles[actuator.actuatorArrayNum].OCPolarity   = inverted ? TIM_OCPOLARITY_HIGH : TIM_OCPOLARITY_LOW; //High polarity if inverted, low if not
	//	//sConfigOCHandles[actuator.actuatorArrayNum].OCIdleState  = inverted ? TIM_OCIDLESTATE_RESET : TIM_OCIDLESTATE_SET; //Reset if inverted, set if not
	//}


	HAL_TIM_PWM_ConfigChannel(&pwmTimers[actuator.actuatorArrayNum], &sConfigOCHandles[actuator.actuatorArrayNum], actuator.timChannel);

	//DMA INIT
	TimDmaInit(&pwmTimers[actuator.actuatorArrayNum], actuator.CcDmaHandle, board.dmasActive[actuator.Dma]);

    HAL_NVIC_SetPriority(actuator.timerIRQn, 3, 0);
    HAL_NVIC_EnableIRQ(actuator.timerIRQn);

	HAL_TIM_Base_Start(&pwmTimers[actuator.actuatorArrayNum]);
	
	//HAL_GPIO_DeInit(ports[actuator.port], actuator.pin);
	
	GPIO_InitStructure.Pin       = actuator.pin;
	GPIO_InitStructure.Mode      = GPIO_MODE_AF_PP; //GPIO_MODE_AF_PP
	GPIO_InitStructure.Pull      = inverted ? GPIO_PULLDOWN : GPIO_PULLUP; //pull down for inverted, pull up for non inverted
	GPIO_InitStructure.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStructure.Alternate = actuator.AF;
	
	HAL_GPIO_Init(ports[actuator.port], &GPIO_InitStructure);
	
}

static void TimDmaInit(TIM_HandleTypeDef *htim, uint32_t handlerIndex, board_dma actuatorDma)
{

	if (htim->hdma[handlerIndex] != 0)
	{
		HAL_DMA_DeInit(htim->hdma[handlerIndex]);
		board.dmasActive[actuatorDma.dmaHandle].enabled = 0;
	}
	//dmaHandles[board.dmasActive[actuatorDma.dmaHandle].dmaHandle].Instance = dmaStream[actuatorDma.dmaStream];
	dmaHandles[actuatorDma.dmaHandle].Instance                 = dmaStream[actuatorDma.dmaStream];
	dmaHandles[actuatorDma.dmaHandle].Init.Channel             = actuatorDma.dmaChannel;
	dmaHandles[actuatorDma.dmaHandle].Init.Direction           = actuatorDma.dmaDirection;
	dmaHandles[actuatorDma.dmaHandle].Init.PeriphInc           = actuatorDma.dmaPeriphInc;
	dmaHandles[actuatorDma.dmaHandle].Init.MemInc              = actuatorDma.dmaMemInc;
	dmaHandles[actuatorDma.dmaHandle].Init.PeriphDataAlignment = actuatorDma.dmaPeriphAlignment;
	dmaHandles[actuatorDma.dmaHandle].Init.MemDataAlignment    = actuatorDma.dmaMemAlignment;
	dmaHandles[actuatorDma.dmaHandle].Init.Mode                = actuatorDma.dmaMode;
	dmaHandles[actuatorDma.dmaHandle].Init.Priority            = actuatorDma.dmaPriority;
	dmaHandles[actuatorDma.dmaHandle].Init.FIFOMode            = actuatorDma.fifoMode;
	dmaHandles[actuatorDma.dmaHandle].Init.FIFOThreshold       = actuatorDma.fifoThreshold;
	dmaHandles[actuatorDma.dmaHandle].Init.MemBurst     	   = actuatorDma.MemBurst;
	dmaHandles[actuatorDma.dmaHandle].Init.PeriphBurst         = actuatorDma.PeriphBurst;

	HAL_DMA_UnRegisterCallback(&dmaHandles[actuatorDma.dmaHandle], HAL_DMA_XFER_ALL_CB_ID);

	/* Associate the initialized DMA handle to the TIM handle */
	__HAL_LINKDMA(htim, hdma[handlerIndex], dmaHandles[actuatorDma.dmaHandle]);

	if (HAL_DMA_Init(&dmaHandles[actuatorDma.dmaHandle]) != HAL_OK) {
		ErrorHandler(WS2812_LED_INIT_FAILIURE);
	} else {
		board.dmasActive[actuatorDma.dmaHandle].enabled = 1;
	}

	HAL_NVIC_SetPriority(actuatorDma.dmaIRQn, actuatorDma.priority, 3);
	HAL_NVIC_EnableIRQ(actuatorDma.dmaIRQn);

}
