#include "includes.h"

uint32_t softSerialRxTimeBuffer;

static const uint16_t bitLookup[] = {0x0000, 0x0001, 0x0003, 0x0007, 0x000F, 0x001F, 0x003F, 0x007F, 0x00FF, 0x01FF, 0x03FF, 0x07FF, 0x0FFF, 0x1FFF, 0x3FFF, 0x7FFF, 0xFFFF};

TIM_HandleTypeDef softSerialTimer;
uint32_t onOffBitArray[11];
int32_t  onOffBitArrayIdx = 0;

soft_serial_record softSerialRecord;

#define SOFT_SERIAL_BUF_SIZE 256
#define SOFT_SERIAL_TIME_BUFFER_SIZE (SOFT_SERIAL_BUF_SIZE * 10) //This is huge! Can probably do smaller than this size safely since the chance that nothing but 0xAA is sent is next to 0;

volatile uint32_t softSerialRxTimerBuffer[SOFT_SERIAL_TIME_BUFFER_SIZE];
volatile uint32_t softSerialRxTimerBufferIdx = 0;


volatile uint32_t sendingData = 0;

static void     NumberOfBits(uint32_t time2, uint32_t time1, uint32_t bitsInByte, float bitWidth, uint16_t *numberOfBits, uint32_t workingOnByte);
static float    FindSoftSerialBitWidth(uint32_t baudRate);
static float    FindSoftSerialByteWidth(float bitWidth, uint32_t bitsPerByte);
static float    FindSoftSerialLineIdleTime(float byteWidth);
static uint32_t IsSoftSerialLineIdle(void);
static void     CleanupTimer(void);

static void     InitSoftSerialTimer(uint32_t pwmHz, uint32_t timerHz);

static void InitSoftSerialTimer(uint32_t pwmHz, uint32_t timerHz)
{
	uint16_t timerPrescaler = 0;

	timerPrescaler = (uint16_t)(SystemCoreClock / TimerPrescalerDivisor(board.generalTimer[1].timer) / timerHz) - 1;

	// Initialize timer
	softSerialTimer.Instance           = timers[board.generalTimer[1].timer];
	softSerialTimer.Init.Prescaler     = timerPrescaler;
	softSerialTimer.Init.CounterMode   = TIM_COUNTERMODE_UP;
	softSerialTimer.Init.Period        = (timerHz / pwmHz) - 1;
	softSerialTimer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_Base_Init(&softSerialTimer);
	HAL_TIM_Base_Start_IT(&softSerialTimer);

    HAL_NVIC_SetPriority(board.generalTimer[1].timerIRQn, 0, 1);
    HAL_NVIC_EnableIRQ(board.generalTimer[1].timerIRQn);
}

static void CleanupTimer(void)
{
	if (softSerialRecord.inverted)
		inlineDigitalLo(ports[softSerialRecord.gpio], softSerialRecord.pin);
	else
		inlineDigitalHi(ports[softSerialRecord.gpio], softSerialRecord.pin);

	sendingData = 0;
}

void NewSoftSerialTimerCallback(uint32_t callbackNumber)
{

	(void)(callbackNumber);

    if (__HAL_TIM_GET_FLAG(&softSerialTimer, TIM_FLAG_UPDATE) != RESET)      //In case other interrupts are also running
    {

        if (__HAL_TIM_GET_ITSTATUS(&softSerialTimer, TIM_IT_UPDATE) != RESET)
        {

            __HAL_TIM_CLEAR_FLAG(&softSerialTimer, TIM_FLAG_UPDATE);

        	if (sendingData == 2)
        	{
        		if (onOffBitArrayIdx == -1)
        		{
        			CleanupTimer();
        		}
        		else if (onOffBitArray[onOffBitArrayIdx--])
        		{
        			if (softSerialRecord.inverted)
        				inlineDigitalLo(ports[softSerialRecord.gpio], softSerialRecord.pin);
        			else
        				inlineDigitalHi(ports[softSerialRecord.gpio], softSerialRecord.pin);
        		}
        		else
        		{
        			if (softSerialRecord.inverted)
        				inlineDigitalHi(ports[softSerialRecord.gpio], softSerialRecord.pin);
        			else
        				inlineDigitalLo(ports[softSerialRecord.gpio], softSerialRecord.pin);

        		}

        	}
        	else if (sendingData == 1)
        	{
        		sendingData = 2;
        	}

        }

    }

}

uint32_t SendSoftSerialByteBlocking(uint8_t byte, uint32_t timeoutMs)
{
	//start bit is a zero, end bit is a 1
	//baudrate, timer rate, 48MHz is good on f4s running at 192 MHz

	uint32_t x;

	SKIP_GYRO = 1;
	onOffBitArrayIdx = 0;
	timeoutMs += InlineMillis();

	//fill timer array, stored in buffer inverted
	switch(softSerialRecord.stopBits)
	{
		case SERIAL_STOP_BITS_1_0:
			onOffBitArray[onOffBitArrayIdx++] = 1;
			break;
		case SERIAL_STOP_BITS_1_5:
		case SERIAL_STOP_BITS_2_0:
			onOffBitArray[onOffBitArrayIdx++] = 1;
			onOffBitArray[onOffBitArrayIdx++] = 1;
			break;
	}

	for (x=0;x<8;x++)
	{
		if (BIT_CHECK(byte, x))
		{
			onOffBitArray[onOffBitArrayIdx++] = 1;
		}
		else
		{
			onOffBitArray[onOffBitArrayIdx++] = 0;
		}
	}

	if (softSerialRecord.startBit)
		onOffBitArray[onOffBitArrayIdx++] = 0;

	//start timer and start sending bits:
	sendingData = 2;
	onOffBitArrayIdx -= 1;
	while (sendingData)
	{
		if (InlineMillis() > timeoutMs)
		{
			CleanupTimer();
			SKIP_GYRO = 0;
			return(0);
		}
	}
	SKIP_GYRO = 0;
	return(1);
}

void InitBlockingSoftSerialPort( uint32_t baudrate, uint32_t inverted, uint32_t stopBits, uint32_t startBit, uint32_t port, uint32_t pin, uint32_t msb, uint32_t tbsHandling )
{

	softSerialRecord.init        = 1;
	softSerialRecord.baud        = baudrate;
	softSerialRecord.bitWidthUs  = FindSoftSerialBitWidth(baudrate);
	softSerialRecord.gpio        = port;
	softSerialRecord.pin         = pin;
	softSerialRecord.msb         = msb;
	softSerialRecord.stopBits    = stopBits;
	softSerialRecord.startBit    = startBit;
	softSerialRecord.inverted    = inverted;
	softSerialRecord.tbsHandling = tbsHandling;

	//check EXTI collisions, if there's a collision we take the gyro off the EXTI and put it on the fake EXTI
	if ( board.gyros[0].extiIRQn == GetExtinFromPin(softSerialRecord.pin) )
		InitFakeGyroExti();

//	if ( board.gyros[0].extiIRQn == GetExtinFromPin(softSerialRecord.pin) )
//  hdfghdf InitFakeGyroExti();

	if (softSerialRecord.stopBits == SERIAL_STOP_BITS_1_0)
	{
		if (softSerialRecord.startBit)
			softSerialRecord.bitsPerByte = 10;
		else
			softSerialRecord.bitsPerByte = 9;
	}
	else
	{
		if (softSerialRecord.startBit)
			softSerialRecord.bitsPerByte = 11;
		else
			softSerialRecord.bitsPerByte = 10;
	}

	softSerialRecord.byteWidthUs = softSerialRecord.bitWidthUs * (float)softSerialRecord.bitsPerByte;

	//Set the callback functions
	callbackFunctionArray[GetTimerCallbackFromTimerEnum(board.generalTimer[1].timer)] = NewSoftSerialTimerCallback;
	callbackFunctionArray[GetExtiCallbackFromPin(softSerialRecord.pin)]               = NewSoftSerialExtiCallback;
}

uint32_t DeInitBlockingSoftSerialPort(void)
{
	if (softSerialRecord.init)
	{
		HAL_TIM_Base_Stop_IT(&softSerialTimer);
		DeInitializeGpio(ports[softSerialRecord.gpio], softSerialRecord.pin);
		HAL_TIM_Base_DeInit(&softSerialTimer);
		softSerialRecord.init = 0;

		//check EXTI collisions, if there's a collision we took the gyro off the EXTI and put it on the fake EXTI, now we put it back
		if ( board.gyros[0].extiIRQn == GetExtinFromPin(softSerialRecord.pin) )
			DeInitFakeGyroExti();

		callbackFunctionArray[GetTimerCallbackFromTimerEnum(board.generalTimer[1].timer)] = 0;
		callbackFunctionArray[GetExtiCallbackFromPin(softSerialRecord.pin)]               = 0;

		return(1);
	}
	return(0);
}

uint32_t ReceiveSoftSerialBlocking(uint8_t rxBuffer[], uint32_t *rxBufferCount, uint32_t timeoutMs)
{

	timeoutMs += InlineMillis();

	SKIP_GYRO = 1;
	//Init the EXTI
    EXTI_Init(ports[softSerialRecord.gpio], softSerialRecord.pin, GetExtinFromPin(softSerialRecord.pin), 0, 1, GPIO_MODE_IT_RISING_FALLING, GPIO_NOPULL); //pulldown if inverted, pullup if normal serial

    //reset reception buffer index.
    softSerialRxTimerBufferIdx = 0;

    //block and wait for line to go idle after receiving data or wait for timeout
    while(!IsSoftSerialLineIdle())
   	{
		if (InlineMillis() > timeoutMs)
		{
		    EXTI_Deinit(ports[softSerialRecord.gpio], softSerialRecord.pin, GetExtinFromPin(softSerialRecord.pin));
			SKIP_GYRO = 0;
			return(0);
		}
   	}
    EXTI_Deinit(ports[softSerialRecord.gpio], softSerialRecord.pin, GetExtinFromPin(softSerialRecord.pin));

    SKIP_GYRO = 0;
	return(NewProcessSoftSerialBits(softSerialRxTimerBuffer, &softSerialRxTimerBufferIdx, rxBuffer, &(*rxBufferCount), softSerialRecord.bitWidthUs, softSerialRecord.bitsPerByte, softSerialRecord.tbsHandling));

}

void NewSoftSerialExtiCallback(uint32_t callbackNumber)
{
	(void)(callbackNumber);
	// EXTI line interrupt detected
	if(__HAL_GPIO_EXTI_GET_IT(softSerialRecord.pin) != RESET)
	{
		//record time of IRQ in microseconds
		softSerialRxTimerBuffer[softSerialRxTimerBufferIdx++] = Micros();
		if (softSerialRxTimerBufferIdx == SOFT_SERIAL_TIME_BUFFER_SIZE)
		{
			softSerialRxTimerBufferIdx = 0;
		}
		__HAL_GPIO_EXTI_CLEAR_IT(softSerialRecord.pin);
	}
}

uint32_t SendSoftSerialBlocking(uint8_t byteArray[], uint32_t numBytesToSend, uint32_t timeoutMs)
{
	uint32_t x;

	//init GPIO, default high if not inverted
	CleanupTimer();
	InitializeGpio( ports[softSerialRecord.gpio], softSerialRecord.pin, (softSerialRecord.inverted ? 1 : 0) );

	//baudrate, timer rate, 48MHz is good on f4s running at 192 MHz
	InitSoftSerialTimer(softSerialRecord.baud, 48000000);

	sendingData = 0;

	//to send
	timeoutMs = (timeoutMs / numBytesToSend) + 2;

	//fill the buffer that will switch the GPIO
	for(x=0;x<numBytesToSend;x++)
	{
		if (softSerialRecord.msb)
			SendSoftSerialByteBlocking(byteArray[x], timeoutMs);
		else
			SendSoftSerialByteBlocking(BitReverse8(byteArray[x]), timeoutMs);
	}


	return(1);
}

static uint32_t IsSoftSerialLineIdle(void)
{
	volatile float timeNow;
	timeNow = (float)Micros();
	if ( (timeNow - (float)softSerialRxTimerBuffer[softSerialRxTimerBufferIdx-1]) > (softSerialRecord.byteWidthUs))
	{
		if (softSerialRxTimerBufferIdx > 1)
			return(1);
	}
	return(0);
}

uint32_t NewProcessSoftSerialBits(volatile uint32_t timerBuffer[], volatile uint32_t *timerBufferIndex, volatile uint8_t serialBuffer[], volatile uint32_t *serialBufferIndex, float bitWidthUs, uint32_t bitsInByte, uint32_t tbsHandling)
{

	uint32_t xStart;
	uint32_t x;
	uint32_t fails;
	uint16_t bits;
	uint32_t currentBit;
	volatile uint32_t byte;
	uint16_t totalBitsFound;
	uint32_t bytesFound;

	bytesFound     = 0;

	xStart         = 1;
	fails          = 0;
	byte           = 0;
	currentBit     = 1;

	//at higher baud rates, the last bit doesn't always calculate correctly, so we hadd a fake byte to the end of the times and remove iut after calculations
	//put in last time so we can get the last byte. We need the last byte to calculate the frame.
	//put in a fake byte at the end and drop it after we calculate
	//last good byte time is timerBuffer[(*timerBufferIndex) - 1]
	//fake byte will have a stop bit which is timerBuffer[(*timerBufferIndex)] = timerBuffer[(*timerBufferIndex) - 1] + lrintf(bitWidthUs);
	//then put in start bit + 8 zeros which is lrintf(bitWidthUs * 9)

	//timerBuffer[(*timerBufferIndex)] = timerBuffer[(*timerBufferIndex) - 1] + lrintf(bitWidthUs * 10); //fake start bit at least 10 bits away
	//(*timerBufferIndex)++; //increment
	//timerBuffer[(*timerBufferIndex)] = timerBuffer[(*timerBufferIndex) - 1] + lrintf(bitWidthUs * 9); //fake zero byte
	//(*timerBufferIndex)++; //increment

	totalBitsFound = 0;
	currentBit     = 1;

	//need to ignore first interrupt
	if (tbsHandling)
	{
		xStart = 3;
	}
	else
	{
		xStart = 1;
	}

	for (x = xStart; x < (*timerBufferIndex); x++)
	{

		if (currentBit)
		{

			NumberOfBits(timerBuffer[x], timerBuffer[x-1], bitsInByte, bitWidthUs, &bits, totalBitsFound);

			if ( (bits > 0) )
			{
				if ( x== ( (*timerBufferIndex)-1) )
					bits = (bitsInByte-totalBitsFound);
				else
					bits = CONSTRAIN(bits,1,(bitsInByte-totalBitsFound));
				byte &= ~(bitLookup[ bits ] << totalBitsFound);
				totalBitsFound += bits;
				currentBit=0;
				bits = 0;
			}
			else
			{
				//ignore this time as a corruption
				fails++;
			}
		}
		else
		{

			NumberOfBits(timerBuffer[x], timerBuffer[x-1], bitsInByte, bitWidthUs, &bits, totalBitsFound);

			if ( (bits > 0) )
			{
				if ( x== ( (*timerBufferIndex)-1) )
					bits = (bitsInByte-totalBitsFound);
				else
					bits = CONSTRAIN(bits,1,(bitsInByte-totalBitsFound));
				byte |= (bitLookup[ bits ] << totalBitsFound);
				totalBitsFound += bits;
				currentBit=1;
				bits = 0;
			}
			else
			{
				//ignore this time as a corruption
				fails++;
			}
		}

		if(fails > 20)
		{
			totalBitsFound = 0;
			break;
		}

		if (totalBitsFound >= bitsInByte)
		{
			//trim off frames
			serialBuffer[(*serialBufferIndex)++] = (uint8_t)( (byte >> 1) & 0xFF );
			totalBitsFound = 0;
			fails = 0;

			bytesFound++;
		}

	}

	(*timerBufferIndex) = 0; //set time buffer index to zero
	(*serialBufferIndex) = 0; //set rx buffer index to zero
	return(bytesFound); //return number of bytes found
}

inline static void NumberOfBits(uint32_t time2, uint32_t time1, uint32_t bitsInByte, float bitWidth, uint16_t *numberOfBits, uint32_t workingOnByte)
{
	float timeD;
	float maxWidthPossible;
	*numberOfBits = 0;
	if (time2 <= time1) //no bits
	{
		return;
	}
	timeD = (float)(time2 - time1);
	maxWidthPossible = (bitWidth * (float)bitsInByte);
	if ( workingOnByte || (timeD < maxWidthPossible) ) //working on byte, so return line idle, or bits exist and this is a new byte
	{
		*numberOfBits = lrintf(round(timeD/(float)bitWidth));
	}
	return;
}

inline static float FindSoftSerialBitWidth(uint32_t baudRate)
{
	return ( (1.0 / (float)baudRate) * 1000.0 * 1000.0 );
}

inline static float FindSoftSerialByteWidth(float bitWidth, uint32_t bitsPerByte)
{
	return (bitWidth * (float)bitsPerByte);
}

inline static float FindSoftSerialLineIdleTime(float byteWidth)
{
	return (byteWidth * 1.50);
}
