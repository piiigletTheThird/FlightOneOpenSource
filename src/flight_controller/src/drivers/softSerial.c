#include "includes.h"

enum {
	SS_IDLE                    = 0,
	SS_PREPARING_ACTUATOR      = 1,
	SS_ACTUATOR_READY_TO_SEND  = 2,
	SS_SENDING_DATA            = 3,
	SS_RECEIVING_DATA          = 4,
	SS_ERROR_TIME_IN_BUFFER_OF = 5,
};

typedef struct {
	motor_type	      currentActuator;
	volatile uint32_t actuatorState; //can be changed by an ISR, so it's volatile.
	float             bitWidth;
	float             byteWidth;
	float		      lineIdleTime;
	uint32_t          buadRate;
	uint32_t          bitsPerByte; //including frame bits
	uint32_t          inverted;
	volatile uint32_t dataInBuffer;
	volatile uint32_t timeOfActivation;   //Anything time related should be made volatile.
	volatile uint32_t timeOfLastActivity; //do we need this? //can be changed by an ISR, so it's volatile if we need it.
	volatile uint32_t softSerialState;
} soft_serial_status;

#define SOFT_SERIAL_BUF_SIZE 256
#define SOFT_SERIAL_TIME_BUFFER_SIZE (SOFT_SERIAL_BUF_SIZE * 10) //This is huge! Can probably do smaller than this size safely since the chance that nothing but 0xAA is sent is next to 0;

soft_serial_status softSerialStatus;
static const uint16_t bitLookup[] = {0x0000, 0x0001, 0x0003, 0x0007, 0x000F, 0x001F, 0x003F, 0x007F, 0x00FF, 0x01FF, 0x03FF, 0x07FF, 0x0FFF, 0x1FFF, 0x3FFF, 0x7FFF, 0xFFFF};


volatile          softserial_function_pointer softserialCallbackFunctionArray[1];
uint8_t           serialOutBuffer[SOFT_SERIAL_BUF_SIZE];
uint8_t           serialInBuffer[SOFT_SERIAL_BUF_SIZE];
uint32_t          serialInBufferIdx=0;
volatile uint32_t timeInBuffer[SOFT_SERIAL_TIME_BUFFER_SIZE];
volatile uint32_t timeInBufferIdx = 0;


static   uint32_t IsSoftSerialLineIdle(void);
static   void     ProcessSoftSerialLineIdle(uint32_t useCallback);
static   void     PutSoftSerialActuatorInSendState(motor_type actuator);
static   void     PutSoftSerialActuatorInReceiveState(motor_type actuator);
static   void	  NumberOfBits(uint32_t time2, uint32_t time1, uint32_t bitsInByte, float bitWidth, uint16_t *numberOfBits, uint32_t workingOnByte);
static   uint32_t ProcessSoftSerialBits(void);
static   float    FindSoftSerialBitWidth(uint32_t baudRate);
static   float    FindSoftSerialByteWidth(float bitWidth, uint32_t bitsPerByte);
static   float    FindSoftSerialLineIdleTime(float byteWidth);
static   uint32_t HandleSoftSerial(void) __attribute__ ((unused));


void SoftSerialExtiCallback(uint32_t callbackNumber)
{
	(void)(callbackNumber);
	// EXTI line interrupt detected
	if(__HAL_GPIO_EXTI_GET_IT(softSerialStatus.currentActuator.pin) != RESET)
	{
		//record time of IRQ in microseconds
		timeInBuffer[timeInBufferIdx++] = Micros();
		if (timeInBufferIdx == SOFT_SERIAL_TIME_BUFFER_SIZE) {
			timeInBufferIdx = 0;
			softSerialStatus.softSerialState = SS_ERROR_TIME_IN_BUFFER_OF;
		}
		__HAL_GPIO_EXTI_CLEAR_IT(softSerialStatus.currentActuator.pin);
	}
}

void SoftSerialDmaCallback(uint32_t callbackNumber)
{
	(void)(callbackNumber);
	 if (dmaHandles[softSerialStatus.currentActuator.Dma].State == HAL_DMA_STATE_READY) {
			//DMA is done sending, let's switch GPIO to EXTI mode
			PutSoftSerialActuatorInReceiveState(softSerialStatus.currentActuator);
	 }
}

//TODO: Make this non blocking function 100% non blocking. 2ms wait is not safe during flight.
uint32_t SoftSerialSendNonBlocking(uint8_t serialOutBuffer[], uint32_t serialOutBufferLength, motor_type actuator)
{
	(void)(serialOutBuffer);
	(void)(serialOutBufferLength);
	(void)(actuator);
	return(0);
	//put actuator in send state. Only one actuator at a time can do this currently.
	PutSoftSerialActuatorInSendState(actuator);
	DelayMs(2); //need to put the next step in the scheduler.
}
uint32_t SoftSerialReceiveNonBlocking(uint8_t inBuffer[], uint32_t *inBufferIdx, motor_type actuator)
{
	(void)(inBuffer);
	(void)(inBufferIdx);
	(void)(actuator);
	return(0);
	softSerialStatus.softSerialState = SS_PREPARING_ACTUATOR;
	PutSoftSerialActuatorInSendState(actuator);
}

uint32_t SoftSerialSendReceiveBlocking(uint8_t serialOutBuffer[], uint32_t serialOutBufferLength, uint8_t inBuffer[], motor_type actuator, uint32_t timeoutMs)
{
	uint32_t returnNumber;
	volatile uint32_t timeout;
	timeout = timeoutMs + InlineMillis();
	//put actuator in send state. Only one actuator at a time can do this currently.
	PutSoftSerialActuatorInSendState(actuator);
	DelayMs(2); //2ms for timer to stabilize. We can do this in blocking mode in this function
	//set softSerialStatus to SENDING_DATA state
	softSerialStatus.softSerialState = SS_SENDING_DATA;
	//set the data to be sent and trigger the DMA to start sending.
	OutputSerialDmaByte(serialOutBuffer, serialOutBufferLength, actuator, 0, 1, 0); //send outbuffer, xx bytes, this actuator, 0=LSB, 1=serial frame
	//SS state is in SS_SENDING_DATA. Once data output completes it goes to SS_RECEIVING_DATA. While SS_SENDING_DATA we check to see if timeout time has passed.
	while (softSerialStatus.softSerialState == SS_SENDING_DATA) {
		//FeedTheDog(); //feed the dog while blocking
		if (InlineMillis() > timeout) {
			return (0); //timeout occurred, return failure
		}
	}
	//When in RECEIVING we wait for a line idle to occur. We allow up to timeout for this to happen.
	while (softSerialStatus.softSerialState == SS_RECEIVING_DATA) {
		//FeedTheDog(); //feed the dog while blocking
		if (InlineMillis() > timeout) {
			return (0); //timeout occurred, return failure
		}
		if ( IsSoftSerialLineIdle() ) {
			softSerialStatus.softSerialState = SS_IDLE;
			ProcessSoftSerialLineIdle(0); //proccess
			memcpy(inBuffer, serialInBuffer, serialInBufferIdx);
			returnNumber = serialInBufferIdx;
			timeInBufferIdx   = 0;
			serialInBufferIdx = 0;//reset buffer idx
			return (returnNumber);
		}
	}
	return (0);
}

static uint32_t IsSoftSerialLineIdle()
{
	volatile float timeNow;
	timeNow = (float)Micros();
	if ( (timeNow - (float)timeInBuffer[timeInBufferIdx-1]) > softSerialStatus.lineIdleTime) {
		if (timeInBufferIdx > 3)
			return(1);
	}
	return(0);
}

void ProcessSoftSerialLineIdle(uint32_t useCallback)
{
	timeInBuffer[timeInBufferIdx] = timeInBuffer[timeInBufferIdx - 1] + lrintf(softSerialStatus.bitWidth*2); //put in last time so we can get the last byte. We need the last byte to calculate the frame.
	timeInBufferIdx++;
	//Process the serial data received and disabled the IRQ if there's a line idle sensed AFTER data has been received
	if ( ProcessSoftSerialBits() )
	{
		//data exit and we're in a line idle
		//Set Soft Serial Status to IDLE Set input buffers back to zero state and process the received data
		//softSerialStatus.softSerialState = SS_IDLE;
		//process the received data
		if ( (useCallback) && (softserialCallbackFunctionArray[0]) )
			softserialCallbackFunctionArray[0](serialInBuffer, serialInBufferIdx);
	}
	else
	{
		//line idle exits, but no data found, how to handle this issue?
		//do not DeInit actuator and only error out if time since activation > greater than XX time
	}
}

static void PutSoftSerialActuatorInReceiveState(motor_type actuator)
{
	//Put softSerialDevice into RX mode
	softSerialStatus.softSerialState = SS_RECEIVING_DATA;
    //Set the IRQ callback functions
    callbackFunctionArray[actuator.EXTICallback] = SoftSerialExtiCallback;
    callbackFunctionArray[actuator.DmaCallback]  = SoftSerialDmaCallback;
	//Init the EXTI
    EXTI_Init(ports[actuator.port], actuator.pin, actuator.EXTIn, 1, 2, GPIO_MODE_IT_RISING_FALLING, softSerialStatus.inverted ? GPIO_PULLDOWN : GPIO_PULLUP); //pulldown if inverted, pullup if normal serial
	//reset reception buffer index.
	timeInBufferIdx = 0;
}

static void PutSoftSerialActuatorInSendState(motor_type actuator)
{
	//Will always assume this actuator has the right away to a DMA. Need to check this for some items, not needed to check for 1wire.
	//TODO: If current tim base is already enabled there's no need to reinit.
//	static uint32_t once = 0;
//	GPIO_InitTypeDef GPIO_InitStructure;
//
//	if (once) {
//		HAL_GPIO_DeInit(ports[actuator.port], actuator.pin);
//
//		//Set pin to timer now that IRQ has occurred.
//		GPIO_InitStructure.Pin       = actuator.pin;
//		GPIO_InitStructure.Mode      = GPIO_MODE_AF_PP; //GPIO_MODE_AF_PP
//		GPIO_InitStructure.Pull      = GPIO_PULLUP; //GPIO_PULLUP //pull up for non inverted, pull down for inverted
//		GPIO_InitStructure.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
//		GPIO_InitStructure.Alternate = actuator.AF;
//
//		HAL_GPIO_Init(ports[actuator.port], &GPIO_InitStructure);
//	}
//	once = 1;
	//Prepares the soft serial actuator. Places it into SS_ACTUATOR_READY_TO_SEND state.
	//set current actuator
	softSerialStatus.currentActuator    = actuator;
	softSerialStatus.timeOfActivation   = Micros();
	softSerialStatus.softSerialState    = SS_PREPARING_ACTUATOR;
	softSerialStatus.buadRate           = 19200; //baud rate
	softSerialStatus.bitsPerByte        = 10;    //including frame bits
	softSerialStatus.inverted           = 0;     //including frame bits
	//calculate these values now and store the results. Stored as floats.
	softSerialStatus.bitWidth           = FindSoftSerialBitWidth(softSerialStatus.buadRate); //bit length in us
	softSerialStatus.byteWidth          = FindSoftSerialByteWidth(softSerialStatus.bitWidth, softSerialStatus.bitsPerByte);
	softSerialStatus.lineIdleTime       = FindSoftSerialLineIdleTime(softSerialStatus.byteWidth);
    //Set DMA callback function to the SoftSerialDmaCallback //TODO: Allow more than one soft serial at a time.
    callbackFunctionArray[actuator.DmaCallback]  = SoftSerialDmaCallback;
    //set EXTI callback to disabled since we're in TX mode now.
	callbackFunctionArray[actuator.EXTICallback] = SoftSerialExtiCallback;
	//activate DMA output on actuator
	SetActiveDmaToActuatorDma(actuator);
	//Put actuator into Output state.
	InitDmaOutputForSoftSerial(DMA_OUTPUT_ESC_1WIRE, actuator);
}

static void NumberOfBits(uint32_t time2, uint32_t time1, uint32_t bitsInByte, float bitWidth, uint16_t *numberOfBits, uint32_t workingOnByte)
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

static uint32_t ProcessSoftSerialBits(void)
{
	uint32_t x;
	uint32_t fails;
	uint16_t bits;
	uint32_t currentBit;
	uint32_t bitsInByte;
	volatile uint32_t byte;
	uint16_t totalBitsFound;
	float    bitWidth;
	uint32_t byteFound;
	volatile uint32_t timeNow = Micros();
	byteFound  = 0;
	bitWidth   = 51.45;
	bitsInByte = 10;
	//bit time for 19200 is 51.45 us
	//max byte time including frames is 514.45 us. Call it 535 to be safe.
	//max byte size is
	fails=0;
	byte = 0;
	currentBit = 1;

	for (x = 0; x < timeInBufferIdx; x++)
	{
		NumberOfBits(timeInBuffer[x+1], timeInBuffer[x], bitsInByte, bitWidth, &bits, totalBitsFound);
		totalBitsFound = 0;
		if (totalBitsFound == 0)
		{ //starting new byte string from line idle.
			currentBit = 1;
			while (totalBitsFound < bitsInByte)
			{
				if (currentBit)
				{
					if (!bits)
					{
						x++;
						NumberOfBits(timeInBuffer[x+1], timeInBuffer[x], bitsInByte, bitWidth, &bits, totalBitsFound);
					}
					if (bits > 0)
					{
						bits = CONSTRAIN(bits,1,(bitsInByte-totalBitsFound));
						//byte |= (bitLookup[ bits ] << totalBitsFound);
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
					if (!bits)
					{
						x++;
						NumberOfBits(timeInBuffer[x+1], timeInBuffer[x], bitsInByte, bitWidth, &bits, totalBitsFound);
					}
					if (bits > 0)
					{
						bits = CONSTRAIN(bits,1,(bitsInByte-totalBitsFound));
						//byte &= ~(bitLookup[ bits ] << totalBitsFound);
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
				if(fails > 10)
				{
					totalBitsFound = 0;
					break;
				}
				if (totalBitsFound >= 10)
				{
					//trim off frames
					serialInBuffer[serialInBufferIdx++] = (uint8_t)( (byte >> 1) & 0xFF );
					if (timeNow > 1)
					{
						totalBitsFound = 0;
					}
					byteFound = 1;
					break;
				}
			}
		}
	}
	return(byteFound);
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

static uint32_t HandleSoftSerial(void) {
	//return 1 to run this function again, return 0 to move onto the next actuator or finish up.
	//I think we need a timeout check here.
	FeedTheDog(); //don't allow a watchdog reboot to happen.
	switch (softSerialStatus.softSerialState) {
		case SS_IDLE:                  //Actuator is Idle
			return (0);
			break;
		case SS_PREPARING_ACTUATOR:    //Actuator is being prepared, after 200 us has passed (allow time for actuator to stabilize. We can send data.
			if (Micros() - softSerialStatus.timeOfActivation > 20)
				softSerialStatus.softSerialState = SS_ACTUATOR_READY_TO_SEND;
			return (1);
			break;
		case SS_ACTUATOR_READY_TO_SEND: //Actuator is prepared and ready to send data. Let's send init then return 1 to keep checking.
			return (1);
			break;
		case SS_RECEIVING_DATA:         //Actuator is in the reception state. We keep checking until we sense a line idle.
			if ( IsSoftSerialLineIdle() ) {
				ProcessSoftSerialLineIdle(1);
			}
			return (1);
			break;
		case SS_SENDING_DATA:           //Actuator is in the sending state. We keep checking until DMA does a callback or until error time elapses
			return (1);
			break;
		case SS_ERROR_TIME_IN_BUFFER_OF:
			//We received more IRQs than we can deal with, buffer is too small or there's a problem.
			//TODO: better handle this situation.
			//DeInitDmaOutputForSoftSerial(softSerialStatus.currentActuator);
			return (0);
			break;
		default:
			return (1);
	}
}
