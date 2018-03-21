#include "includes.h"

volatile float throttleVelocity        = 0.0f;
volatile float smoothCurvedThrottle0_1 = 0.0f;
volatile float trueCurvedThrottle0_1   = 0.0f;
float trueRcCommandF[MAXCHANNELS];     //4 sticks. range is -1 to 1, directly related to stick position
float curvedRcCommandF[MAXCHANNELS];   //4 sticks. range is -1 to 1, this is the rcCommand after the curve is applied
float smoothedRcCommandF[MAXCHANNELS]; //4 sticks. range is -1 to 1, this is the smoothed rcCommand
volatile unsigned char isRxDataNew;
volatile uint32_t disarmCount = 0, latchFirstArm = 0;
volatile SPM_VTX_DATA vtxData;

volatile arming_structure armingStructure;

volatile uint32_t throttleIsSafe = 0;
static uint32_t packetTime = 11;
static float packetTimeInv = (11.0f / 1000.0f);

uint32_t veryFirstArm              = 1;
uint32_t PreArmFilterCheck         = 0;
uint32_t activeFailsafe            = 0;
uint32_t failsafeHappend           = 0;
volatile uint32_t armBoardAt       = 0;
uint32_t rxDataRaw[MAXCHANNELS]    = {0,};
uint32_t rxData[MAXCHANNELS]       = {0,};
volatile float maxFlopRate[3]      = {0.0f,};
volatile float maxKissRate[3]      = {0.0f,};
uint32_t skipRxMap                 = 0;
uint32_t progTimer                 = 0;
uint32_t ppmPin                    = 99;
volatile uint32_t progMode         = 0;
volatile uint32_t armCheckLatch    = 0;
int32_t  smoothingInterval         = 1;
int32_t  smoothingIntervalThrottle = 1;

#define PPM_SYNC_MINIMUM_US 4000
#define PPM_BUFFER_SIZE 25
#define PPM_CHANNELS 8
uint32_t ppmBufferIdx               = 0;
uint32_t ppmBuffer[PPM_BUFFER_SIZE] = {0,};
uint32_t ppmData[PPM_CHANNELS]      = {0,};

// 2048 resolution
#define SPEKTRUM_FRAME_SIZE 16
uint32_t spektrumChannelShift = 3;
uint32_t spektrumChannelMask  = 0x07;
uint32_t rxUpdateCount = 0;

static rx_calibration_records rxCalibrationRecords[3];
static void ProcessPpmPacket(uint32_t ppmBuffer2[], uint32_t *ppmBufferIdx);

static void checkRxPreArmCalibration(void);
static float GetKissMaxRates(float rcCommand, uint32_t axis);
static float GetFlopMaxRates(float rcCommand, uint32_t axis);

//check
static void checkRxPreArmCalibration(void)
{
	uint32_t axis;
	uint32_t y;
	uint32_t z;
	uint32_t highestCount;

	for (axis = 0;axis<3;axis++)
	{
		highestCount=0;
		//if pitch is near center, we add it to array
		//mainConfig.rcControlsConfig.midRc[ChannelMap(axis)]
		//cruiser
		if ( ABS((int32_t)rxData[axis] - (int32_t)mainConfig.rcControlsConfig.midRc[axis]) < 50 )
		{
			z = 0;
			//check each value for
			for (y=0;y<RX_CHECK_AMOUNT;y++)
			{
				//
				if (rxCalibrationRecords[axis].rxCalibrationRecord[y].dataValue == rxData[axis])
				{
					highestCount = rxCalibrationRecords[axis].rxCalibrationRecord[y].timesOccurred;
					rxCalibrationRecords[axis].rxCalibrationRecord[y].timesOccurred++;
					z++;
				}

				//find and record the highest value
				if (highestCount < rxCalibrationRecords[axis].rxCalibrationRecord[y].timesOccurred)
				{
					//set the dataValue with the
					highestCount = rxCalibrationRecords[axis].rxCalibrationRecord[y].timesOccurred;
					rxCalibrationRecords[axis].highestDataValue = rxCalibrationRecords[axis].rxCalibrationRecord[y].dataValue;
				}
			}
			if (!z)
			{
				for (y=0;y<RX_CHECK_AMOUNT;y++)
				{
					if (!rxCalibrationRecords[axis].rxCalibrationRecord[y].dataValue)
					{
						rxCalibrationRecords[axis].rxCalibrationRecord[y].dataValue     = rxData[axis];
						rxCalibrationRecords[axis].rxCalibrationRecord[y].timesOccurred = 1;
						break;
					}
				}
			}
		}
	}
}

#define SBUS_FRAME_SIZE 25
#define SBUS_FRAME_LOSS_FLAG (1 << 2)
#define SBUS_FAILSAFE_FLAG (1 << 3)
#define SBUS_STARTBYTE         0x0f
#define SBUS_ENDBYTE           0x00

typedef struct {
	uint8_t syncByte;
	unsigned int chan0  : 11;
	unsigned int chan1  : 11;
	unsigned int chan2  : 11;
	unsigned int chan3  : 11;
	unsigned int chan4  : 11;
	unsigned int chan5  : 11;
	unsigned int chan6  : 11;
	unsigned int chan7  : 11;
	unsigned int chan8  : 11;
	unsigned int chan9  : 11;
	unsigned int chan10 : 11;
	unsigned int chan11 : 11;
	unsigned int chan12 : 11;
	unsigned int chan13 : 11;
	unsigned int chan14 : 11;
	unsigned int chan15 : 11;
	uint8_t flags;
	uint8_t endByte;
} __attribute__ ((__packed__)) sbusFrame_t;

typedef struct {
	uint8_t syncByte;
	uint8_t lengthByte;
	uint8_t typeByte;
	unsigned int chan0  : 11;
	unsigned int chan1  : 11;
	unsigned int chan2  : 11;
	unsigned int chan3  : 11;
	unsigned int chan4  : 11;
	unsigned int chan5  : 11;
	unsigned int chan6  : 11;
	unsigned int chan7  : 11;
	unsigned int chan8  : 11;
	unsigned int chan9  : 11;
	unsigned int chan10 : 11;
	unsigned int chan11 : 11;
	unsigned int chan12 : 11;
	unsigned int chan13 : 11;
	unsigned int chan14 : 11;
	unsigned int chan15 : 11;
	uint8_t crc;
} __attribute__ ((__packed__)) crsfRcFrame_t;

//uint32_t tempData[MAXCHANNELS];

unsigned char copiedBufferData[RXBUFFERSIZE];

volatile uint32_t rx_timeout=0;
uint32_t spekPhase=1;
uint32_t ignoreEcho = 0;

static uint16_t CRC16(uint16_t crc, uint8_t value);




#define CRC_POLYNOME 0x1021
/*******************************************************************************
* Function Name : CRC16
* Description : crc calculation, adds a 8 bit unsigned to 16 bit crc
*******************************************************************************/
static uint16_t CRC16(uint16_t crc, uint8_t value)
{
	uint8_t i;
	crc = crc ^ (int16_t)value<<8;

	for(i=0; i<8; i++)
	{
		if (crc & 0x8000)
			crc = (crc << 1) ^ CRC_POLYNOME;
		else
			crc = (crc << 1);
	}

	return crc;
}




inline void CheckFailsafe(void)
{
	rx_timeout++;

	FeedTheDog(); //resets IWDG time to 0. This tells the timer the board is running.

	if (( (rx_timeout > loopSpeed.fsCount) || (ModeActive(M_FAILSAFE)) ))
		activeFailsafe = 1;
	else
		activeFailsafe = 0;

	if ((boardArmed) && ( (rx_timeout > loopSpeed.fsCount) || (ModeActive(M_FAILSAFE)) ) )
	{
		failsafeHappend = 1;
		buzzerStatus.status = STATE_BUZZER_FAILSAFE;
		DisarmBoard();
		ZeroActuators(32000); //immediately set actuators to disarmed position.
	}

	//make sure buzzer mode doesn't overwrite failsafe buzzer
	if ( ModeActive(M_BUZZER) && (buzzerStatus.status != STATE_BUZZER_FAILSAFE) )
	{
		buzzerStatus.status = STATE_BUZZER_ON;
	}
	else if ( ModeSet(M_BUZZER) && (buzzerStatus.status == STATE_BUZZER_ON) )
	{
		buzzerStatus.status = STATE_BUZZER_OFF;
	}

}

void ProcessArmingStructure(void)
{
	if (boardArmed)
		armingStructure.boardArmed = 1;
	else
		armingStructure.boardArmed = 0;

	armingStructure.latchFirstArm   = latchFirstArm;
	armingStructure.armModeSet      = ModeSet(M_ARMED);
	armingStructure.armModeActive   = ModeActive(M_ARMED);
	armingStructure.rcCalibrated    = mainConfig.rcControlsConfig.rcCalibrated;
	armingStructure.boardCalibrated = mainConfig.gyroConfig.boardCalibrated;
	armingStructure.progMode        = !progMode;
	armingStructure.throttleIsSafe  = throttleIsSafe;
	armingStructure.rxTimeout       = rx_timeout;
	armingStructure.failsafeHappend = !failsafeHappend;
	armingStructure.activeFailsafe  = !activeFailsafe;

}

inline void CheckThrottleSafe(void)
{
	uint32_t trueRangedThrottleU; //set

	trueRangedThrottleU = lrintf( InlineChangeRangef(trueRcCommandF[THROTTLE], 1.0f, -1.0f, 1023.0f, 0.0f) );

	//find the trueCurvedThrottle0_1 which is 0.0 to 1.0f
	trueCurvedThrottle0_1  = throttleLookup[trueRangedThrottleU];

	if ( (!threeDeeMode) && (trueRcCommandF[THROTTLE] < -0.85f) && (trueCurvedThrottle0_1 < 0.075f) )
	{
		throttleIsSafe = 1;
	}
	else if ( (threeDeeMode) && (trueRcCommandF[THROTTLE] > -0.10f) && (trueRcCommandF[THROTTLE] < 0.10f) && (trueCurvedThrottle0_1 > 0.45f) && (trueCurvedThrottle0_1 < 0.55f) )
	{
		throttleIsSafe = 1;
	}
	else
	{
		throttleIsSafe = 0;
	}
}

inline void RxUpdate(void) // hook for when rx updates
{

	 //get current flight modes

	//if(trueRcCommandF[5] > 0.9)
	//{
	//	inlineDigitalLo(ports[ENUM_PORTB], GPIO_PIN_0);
	//}
	//else
	//{
	//	inlineDigitalHi(ports[ENUM_PORTB], GPIO_PIN_0);
	//}

	CheckRxToModes();

	CheckThrottleSafe();

	//throttle must be low and board must be set to not armed before we allow an arming
	if (!ModeActive(M_ARMED) &&  throttleIsSafe && (rxUpdateCount++ > 25 ))
		armCheckLatch = 1;

	if ( (veryFirstArm) && (rxUpdateCount > 25 ) )
		checkRxPreArmCalibration(); //collect rx data if not armed yet

	if (armCheckLatch)
	{
		if ( (latchFirstArm == 0) && (!boardArmed) && (ModeActive(M_ARMED)) )
		{
			if (veryFirstArm)
			{
				PreArmFilterCheck = 1;
				ResetGyroCalibration();
			}

			veryFirstArm  = 0;
			latchFirstArm = 1;
			buzzerStatus.status = STATE_BUZZER_ARMING;

			if( mainConfig.telemConfig.telemSmartAudio && !ModeSet(M_VTXON) && (vtxRecord.vtxDevice !=  VTX_DEVICE_NONE))
			{
				turnOnVtxNow = 1;
			}
			if( mainConfig.telemConfig.telemTramp && !ModeSet(M_VTXON) && (vtxRecord.vtxDevice !=  VTX_DEVICE_NONE))
			{
				turnOnVtxNow = 1;
			}

		}
		else if ( (mainConfig.rcControlsConfig.rcCalibrated) && (latchFirstArm == 2) && (!boardArmed) && (ModeActive(M_ARMED)) && (mainConfig.gyroConfig.boardCalibrated) && throttleIsSafe && !progMode)
		{ //TODO: make uncalibrated board buzz

			if (mainConfig.rcControlsConfig.armMethod == ARM_DOUBLE_SINGLE)
				latchFirstArm = 1; //1 is double single single single, 0 is double double double double
			else if (mainConfig.rcControlsConfig.armMethod == ARM_DOUBLE_DOUBLE)
				latchFirstArm = 0; //1 is double single single single, 0 is double double double double

			disarmCount   = 0;

			if ( !(RtcReadBackupRegister(FC_STATUS_REG) == FC_STATUS_INFLIGHT) ) {
				//fc crashed during flight
				RtcWriteBackupRegister(FC_STATUS_REG,FC_STATUS_INFLIGHT);
				buzzerStatus.status = STATE_BUZZER_ARMING;
			}

			if ( ABS((int32_t)rxCalibrationRecords[PITCH].highestDataValue - (int32_t)mainConfig.rcControlsConfig.midRc[PITCH]) < 30 )
				mainConfig.rcControlsConfig.midRc[PITCH] = rxCalibrationRecords[PITCH].highestDataValue;
			if ( ABS((int32_t)rxCalibrationRecords[ROLL].highestDataValue - (int32_t)mainConfig.rcControlsConfig.midRc[ROLL]) < 30 )
				mainConfig.rcControlsConfig.midRc[ROLL] = rxCalibrationRecords[ROLL].highestDataValue;
			if ( ABS((int32_t)rxCalibrationRecords[YAW].highestDataValue - (int32_t)mainConfig.rcControlsConfig.midRc[YAW]) < 30 )
				mainConfig.rcControlsConfig.midRc[YAW] = rxCalibrationRecords[YAW].highestDataValue;

			if( (mainConfig.telemConfig.telemSmartAudio && !ModeSet(M_VTXON) ) || (mainConfig.telemConfig.telemTramp && !ModeSet(M_VTXON) ) )
			{
				armBoardAt = InlineMillis();
			}
			else
			{
				ArmBoard();
			}

		}
		else if ( !ModeActive(M_ARMED) )
		{
			if (disarmCount++ > 3)
			{
				if (latchFirstArm==1)
				{
					latchFirstArm = 2;
				}
				DisarmBoard();
				RtcWriteBackupRegister(FC_STATUS_REG,FC_STATUS_IDLE);
			}
		}
	}

	if ( (!boardArmed) && (trueRcCommandF[YAW] > 0.95f) && (trueRcCommandF[THROTTLE] < -0.95f) && (trueRcCommandF[PITCH] < -0.95f) && (trueRcCommandF[ROLL] < -0.95f) )
	{
		if ( (InlineMillis() - progTimer) > 2000 )
			progMode = 1;
	}
	else
	{
		progTimer = InlineMillis();
	}


}

void SpektrumBind(uint32_t bindNumber)
{

	uint32_t i            = 0;
	uint32_t serialNumber = 0;

	if (!bindNumber)
		return;

	for (serialNumber = 0; serialNumber<MAX_USARTS;serialNumber++)
	{
		if (board.serials[serialNumber].enabled)
		{
			InitializeGpio(ports[board.serials[serialNumber].TXPort], board.serials[serialNumber].TXPin, 1);
			InitializeGpio(ports[board.serials[serialNumber].RXPort], board.serials[serialNumber].RXPin, 1);
		}
	}

	DelayMs(2);

	if (!bindNumber)
		bindNumber = 9;

	for (i=0; i < bindNumber; i++) {

		for (serialNumber = 0; serialNumber<MAX_USARTS;serialNumber++)
		{
			if (board.serials[serialNumber].enabled)
			{
				inlineDigitalLo(ports[board.serials[serialNumber].TXPort], board.serials[serialNumber].TXPin);
				inlineDigitalLo(ports[board.serials[serialNumber].RXPort], board.serials[serialNumber].RXPin);
			}
		}
		DelayMs(2);

		for (serialNumber = 0; serialNumber<MAX_USARTS;serialNumber++)
		{
			if (board.serials[serialNumber].enabled)
			{
				inlineDigitalHi(ports[board.serials[serialNumber].TXPort], board.serials[serialNumber].TXPin);
				inlineDigitalHi(ports[board.serials[serialNumber].RXPort], board.serials[serialNumber].RXPin);
			}
		}
		DelayMs(2);

	}


    if (mainConfig.rcControlsConfig.bind)
    {
    	mainConfig.rcControlsConfig.bind = 0;
    	SaveConfig(ADDRESS_CONFIG_START);
    }


}

inline uint32_t ChannelMap(uint32_t inChannel)
{
	volatile uint32_t outChannel;

	if ( (!skipRxMap) && (mainConfig.rcControlsConfig.channelMap[inChannel] <= MAXCHANNELS) )
	{
		outChannel =mainConfig.rcControlsConfig.channelMap[inChannel];
	}
	else if ( mainConfig.rcControlsConfig.rcCalibrated != 1)
	{
		outChannel = inChannel;//not calibrated and no need to skip, send default
	}
	else
	{
		outChannel = 15; //else dump to junk channel
	}

	return(outChannel);
}

void ProcessCrsfPacket(uint8_t serialRxBuffer[], uint32_t frameSize)
{

	int chkSum; //set
	int rxSum;  //set
	int y;      //set

	//<​Device address​ or Sync Byte>
	//<Frame length>
	//<​Type​>
	//<Payload>
	//<​CRC​> 

															// Make sure this is very first thing done in function, and its called first on interrupt
	memcpy(copiedBufferData, serialRxBuffer, frameSize);    // we do this to make sure we don't have a race condition, we copy before it has a chance to be written by dma
															// We know since we are highest priority interrupt, nothing can interrupt us, and copy happens so quick, we will alwyas be guaranteed to get it
	crsfRcFrame_t *crsfRc = (crsfRcFrame_t*)copiedBufferData;

	if (copiedBufferData[0] == CRSF_SYNC_BYTE)
	{
		switch(copiedBufferData[2]) //frame type is thirdbyte
		{
			case CRSF_TYPE_RC:
				rxDataRaw[0]  = crsfRc->chan0;
				rxDataRaw[1]  = crsfRc->chan1;
				rxDataRaw[2]  = crsfRc->chan2;
				rxDataRaw[3]  = crsfRc->chan3;
				rxDataRaw[4]  = crsfRc->chan4;
				rxDataRaw[5]  = crsfRc->chan5;
				rxDataRaw[6]  = crsfRc->chan6;
				rxDataRaw[7]  = crsfRc->chan7;
				rxDataRaw[8]  = crsfRc->chan8;
				rxDataRaw[9]  = crsfRc->chan9;
				rxDataRaw[10] = crsfRc->chan10;
				rxDataRaw[11] = crsfRc->chan11;
				rxDataRaw[12] = crsfRc->chan12;
				rxDataRaw[13] = crsfRc->chan13;
				rxDataRaw[14] = crsfRc->chan14;
				rxDataRaw[15] = crsfRc->chan15;
				rxSum         = crsfRc->crc;

				chkSum = CrsfCrc8(copiedBufferData+2, frameSize-2);

				(void)(chkSum);
				(void)(rxSum);
				//if (chkSum == rxSum)
				if (1 == 1)
				{
					for (y=MAXCHANNELS-1;y>-1;y--)
					{
						rxData[y] = rxDataRaw[ChannelMap(y)];
					}

					packetTime = 10;
					packetTimeInv = ((float)packetTime / 1000.0f);
					rx_timeout = 0;

					if (buzzerStatus.status == STATE_BUZZER_FAILSAFE)
						buzzerStatus.status = STATE_BUZZER_OFF;

					InlineCollectRcCommand();
					RxUpdate();
					static int everyTen = 0;
					everyTen++;
					if (everyTen==10)
					{
						sendCrsfTelemtryAt = InlineMillis() + 1;
						everyTen = 0;
					}
				}
				break;
			default:
				return;
				break;
			
		}
	}
}


void ProcessSpektrumPacket(uint32_t serialNumber)
{
	static uint32_t timeSinceLastPacket = 0;
	volatile uint32_t spektrumChannel;
	uint32_t x;
	int32_t y;
	uint32_t value;
	uint16_t channelIdMask;
	uint16_t servoPosMask;
	uint32_t bitShift;

	if ( (InlineMillis() - timeSinceLastPacket)  < 5)
		return;

	channelIdMask = 0x7800;
	servoPosMask  = 0x07FF;
	bitShift      = 11;
	//DSM2 is 10 bits and the others are 11 bits of data for the RX
	if ((board.serials[serialNumber].Protocol == USING_DSM2_T) || (board.serials[serialNumber].Protocol == USING_DSM2_R))
	{
		channelIdMask = 0xFC00;
		servoPosMask  = 0x03FF;
		bitShift      = 10;
	}
																													// Make sure this is very first thing done in function, and its called first on interrupt
	memcpy(copiedBufferData, serialRxBuffer[board.serials[serialNumber].serialRxBuffer-1], SPEKTRUM_FRAME_SIZE);    // we do this to make sure we don't have a race condition, we copy before it has a chance to be written by dma
															   	   	   	   	   	   	   	   	   	   	   	   	   	   	// We know since we are highest priority interrupt, nothing can interrupt us, and copy happens so quick, we will alwyas be guaranteed to get it

	for (x = 2; x < 16; x += 2)
	{
		value = (copiedBufferData[x] << 8) + (copiedBufferData[x+1]);
		spektrumChannel = (value & channelIdMask) >> bitShift;
		if (spektrumChannel < MAXCHANNELS)
		{
			rxDataRaw[spektrumChannel] = (value & servoPosMask);
			rx_timeout = 0;
			if (buzzerStatus.status == STATE_BUZZER_FAILSAFE)
				buzzerStatus.status = STATE_BUZZER_OFF;
		}
		else
		{
			rx_timeout++;
		}
	}

	for (y=MAXCHANNELS-1;y>-1;y--)
	{
		rxData[y] = rxDataRaw[ChannelMap(y)];
	}

	timeSinceLastPacket = InlineMillis();
	if ( !(board.serials[serialNumber].Protocol == USING_DSM2_T) && !(board.serials[serialNumber].Protocol == USING_DSM2_R) )
	{
		spekPhase = copiedBufferData[2] & 0x80;

		//Check for vtx data
		if (copiedBufferData[12] == 0xE0)
		{
			if (mainConfig.telemConfig.telemSmartAudio || mainConfig.telemConfig.telemTramp)
			{
				vtxRequested.vtxBandChannel = VtxSpektrumBandAndChannelToVtxBandChannel( (copiedBufferData[13] >> 5) & 0x07, (copiedBufferData[13] & 0x0F) + 1);
				VtxChannelToBandAndChannel(vtxRequested.vtxBandChannel, &vtxRequested.vtxBand, &vtxRequested.vtxChannel);
				vtxRequested.vtxFrequency = VtxBandChannelToFrequency(vtxRequested.vtxBandChannel);
				//vtxData.vtxChannel = (copiedBufferData[13] & 0x0F) + 1;
				//vtxData.vtxBand    = (copiedBufferData[13] >> 5) & 0x07;
			}
		}

			  //Check channel slot 7 for vtx power, pit, and region data
		if (copiedBufferData[14] == 0xE0)
		{
			if (mainConfig.telemConfig.telemSmartAudio || mainConfig.telemConfig.telemTramp)
			{
				//julian = (int)(copiedBufferData[15] & 0x03);
				vtxRequested.vtxPower  = (uint32_t)(copiedBufferData[15] & 0x03);
				vtxRequested.vtxRegion = (uint32_t)((copiedBufferData[15] >> 3) & 0x01);
				vtxRequested.vtxPit    = (uint32_t)((copiedBufferData[15] >> 4) & 0x01);

				if (vtxRequested.vtxPit == SPEK_VTX_ACTIVE)
					vtxRequested.vtxPit = VTX_MODE_ACTIVE;
				else
					vtxRequested.vtxPit = VTX_MODE_PIT;

				//vtxData.vtxPower  = copiedBufferData[15] & 0x03;
				//vtxData.vtxRegion = (copiedBufferData[15] >> 3) & 0x01;
				//vtxData.vtxPit    = (copiedBufferData[15] >> 4) & 0x01;
			}
		}

		static int everyOtherSpekTelem = 0;
		if (!spekPhase && mainConfig.telemConfig.telemSpek)
		{
			if (everyOtherSpekTelem++ == 1)
			{
				everyOtherSpekTelem = 0;
				sendSpektrumTelemtryAt = InlineMillis() + 2;
			}
		}
		else
		{
			sendSpektrumTelemtryAt = 0;
		}

		packetTime = 11;
		packetTimeInv = ((float)packetTime / 1000.0f);
	}
	else
	{
		packetTime = 22;
		packetTimeInv = ((float)packetTime / 1000.0f);
	}

	InlineCollectRcCommand();
	RxUpdate();
}

void PowerInveter(uint32_t port, uint32_t pin, uint32_t direction)
{
	if (direction)
		InitializeGpio(ports[port], pin, direction-1);
}

void ProcessSbusPacket(uint32_t serialNumber)
{
	static uint32_t outOfSync = 0, inSync = 0;
	int32_t y;

	sbusFrame_t *frame = (sbusFrame_t*)copiedBufferData;

	memcpy(copiedBufferData, serialRxBuffer[board.serials[serialNumber].serialRxBuffer-1], SBUS_FRAME_SIZE);

	// do we need to hook these into rxData[ChannelMap(i)] ?
	//if ( (frame->syncByte == SBUS_STARTBYTE) && (frame->endByte == SBUS_ENDBYTE) )
	if (frame->syncByte == SBUS_STARTBYTE)
	{
		if ( !(frame->flags & (SBUS_FAILSAFE_FLAG) ) )
		{

			rx_timeout = 0;
			if (buzzerStatus.status == STATE_BUZZER_FAILSAFE)
				buzzerStatus.status = STATE_BUZZER_OFF;

			rxDataRaw[0]  = frame->chan0;
			rxDataRaw[1]  = frame->chan1;
			rxDataRaw[2]  = frame->chan2;
			rxDataRaw[3]  = frame->chan3;
			rxDataRaw[4]  = frame->chan4;
			rxDataRaw[5]  = frame->chan5;
			rxDataRaw[6]  = frame->chan6;
			rxDataRaw[7]  = frame->chan7;
			rxDataRaw[8]  = frame->chan8;
			rxDataRaw[9]  = frame->chan9;
			rxDataRaw[10] = frame->chan10;
			rxDataRaw[11] = frame->chan11;
			rxDataRaw[12] = frame->chan12;
			rxDataRaw[13] = frame->chan13;
			rxDataRaw[14] = frame->chan14;
			rxDataRaw[15] = frame->chan15;

			for (y=MAXCHANNELS-1;y>-1;y--)
			{
				rxData[y] = rxDataRaw[ChannelMap(y)];
			}

			inSync++;
			// TODO: is this best way to deal with failsafe stuff?
			//if (!(frame->flags & (SBUS_FRAME_LOSS_FLAG | SBUS_FAILSAFE_FLAG))) {
			//	rx_timeout = 0;
			//}
			// TODO: No, we should only look at SBUS_FAILSAFE_FLAG for failsafe.

			packetTime = 9;
			packetTimeInv = ((float)packetTime / 1000.0f);
			InlineCollectRcCommand();
			RxUpdate();
		}
	} else {
		outOfSync++;
	}

}

void ProcessSumdPacket(uint8_t serialRxBuffer[], uint32_t frameSize)
{

	uint32_t x              = 0;
	int32_t  y              = 0;
	uint16_t value          = 0;
	uint16_t numOfChannels  = 0;
	uint16_t calculatedCrc  = 0;
	//uint16_t receivedCrc;
																													// Make sure this is very first thing done in function, and its called first on interrupt
	memcpy(copiedBufferData, serialRxBuffer, frameSize);    // we do this to make sure we don't have a race condition, we copy before it has a chance to be written by dma
															   	   	   	   	   	   	   	   	   	   	   	   	   	   	// We know since we are highest priority interrupt, nothing can interrupt us, and copy happens so quick, we will alwyas be guaranteed to get it
	calculatedCrc = 0;

	if ( (copiedBufferData[0] == 0xA8) && (copiedBufferData[1] == 0x01) ) { //0 is graupner, //valid and live header 1 is 0x01, failsafe is 0x81, any other value is invalid

		numOfChannels = copiedBufferData[2];

		//check CRC
		calculatedCrc = CRC16(calculatedCrc, copiedBufferData[0]);
		calculatedCrc = CRC16(calculatedCrc, copiedBufferData[1]);
		calculatedCrc = CRC16(calculatedCrc, copiedBufferData[2]);

		if ( (numOfChannels < 0x20) && (numOfChannels > 0x01) )
		{

			//receivedCrc = (uint32_t)((copiedBufferData[(numOfChannels + 1) * 2 + 1] << 8) & 0x0000FF00); //crc high byte
			//receivedCrc = (uint32_t)(copiedBufferData[(numOfChannels + 1) * 2 + 2] & 0x000000FF);        //crc low byte
			for (x=0;x<(numOfChannels);x++)
			{
				calculatedCrc = CRC16(calculatedCrc, copiedBufferData[x * 2 + 1]);
				calculatedCrc = CRC16(calculatedCrc, copiedBufferData[x * 2 + 2]);
			}

		}

		//if (receivedCrc == calculatedCrc)
		if (1 == 1)
		{
			if ( (numOfChannels < 0x20) && (numOfChannels > 0x01) )
			{

				for (x=0;x<numOfChannels;x++)
				{

					value = (uint32_t)( (uint16_t)((copiedBufferData[3 + x * 2 + 0] << 8) & 0x0000FF00) | (uint8_t)(copiedBufferData[3 + x * 2 + 1] & 0x000000FF) );
					rxDataRaw[x] = value; //high byte

				}

				packetTime = 10;
				packetTimeInv = ((float)packetTime / 1000.0f);
				rx_timeout = 0;
				if (buzzerStatus.status == STATE_BUZZER_FAILSAFE)
					buzzerStatus.status = STATE_BUZZER_OFF;
				InlineCollectRcCommand();
				RxUpdate();

			}

			for (y=MAXCHANNELS-1;y>-1;y--)
			{
				rxData[y] = rxDataRaw[ChannelMap(y)];
			}
		}

	}

}

void ProcessIbusPacket(uint8_t serialRxBuffer[], uint32_t frameSize)
{

	uint32_t i      = 0;
	int32_t  y      = 0;
	uint16_t chkSum = 0;
	uint16_t rxSum  = 0;

															// Make sure this is very first thing done in function, and its called first on interrupt
	memcpy(copiedBufferData, serialRxBuffer, frameSize);    // we do this to make sure we don't have a race condition, we copy before it has a chance to be written by dma
															// We know since we are highest priority interrupt, nothing can interrupt us, and copy happens so quick, we will alwyas be guaranteed to get it
	chkSum = 0xFFFF;
	for (i = 0; i < 30; i++)
		chkSum -= copiedBufferData[i];

	rxSum = copiedBufferData[30] + (copiedBufferData[31] << 8);

	 if (chkSum == rxSum)
	 {
		rxDataRaw[0] = (copiedBufferData[ 3] << 8) + copiedBufferData[ 2];
		rxDataRaw[1] = (copiedBufferData[ 5] << 8) + copiedBufferData[ 4];
		rxDataRaw[2] = (copiedBufferData[ 7] << 8) + copiedBufferData[ 6];
		rxDataRaw[3] = (copiedBufferData[ 9] << 8) + copiedBufferData[ 8];
		rxDataRaw[4] = (copiedBufferData[11] << 8) + copiedBufferData[10];
		rxDataRaw[5] = (copiedBufferData[13] << 8) + copiedBufferData[12];
		rxDataRaw[6] = (copiedBufferData[15] << 8) + copiedBufferData[14];
		rxDataRaw[7] = (copiedBufferData[17] << 8) + copiedBufferData[16];
		rxDataRaw[8] = (copiedBufferData[19] << 8) + copiedBufferData[18];
		rxDataRaw[9] = (copiedBufferData[21] << 8) + copiedBufferData[20];

		for (y=MAXCHANNELS-1;y>-1;y--)
		{
			rxData[y] = rxDataRaw[ChannelMap(y)];
		}

		packetTime = 10;
		packetTimeInv = ((float)packetTime / 1000.0f);
		rx_timeout = 0;

		if (buzzerStatus.status == STATE_BUZZER_FAILSAFE)
			buzzerStatus.status = STATE_BUZZER_OFF;

		InlineCollectRcCommand();
		RxUpdate();
	}

}

void ProcessPpmPacket(uint32_t ppmBuffer2[], uint32_t *ppmBufferIdx)
{
//	ppmBuffer[*ppmBufferIdx]

	uint32_t x;
	//make sure sync is correct:
	//we have at least two times, make sure they are at least the PPM_SYNC_MINIMUM_US apart
	if ( (*ppmBufferIdx > 17) && ( (ppmBuffer2[1] - ppmBuffer[0]) >  PPM_SYNC_MINIMUM_US ) )
	{
		//sync looks good, where 0 is the end of the last pulse and 1 is the beginning of the new pulse
		//we have at least 8 channels which is 18 interrupts

		ppmData[0] = (ppmBuffer[ 3] - ppmBuffer[ 2]);
		ppmData[1] = (ppmBuffer[ 5] - ppmBuffer[ 4]);
		ppmData[2] = (ppmBuffer[ 7] - ppmBuffer[ 6]);
		ppmData[3] = (ppmBuffer[ 9] - ppmBuffer[ 8]);
		ppmData[4] = (ppmBuffer[11] - ppmBuffer[10]);
		ppmData[5] = (ppmBuffer[13] - ppmBuffer[12]);
		ppmData[6] = (ppmBuffer[15] - ppmBuffer[14]);
		ppmData[7] = (ppmBuffer[17] - ppmBuffer[16]);

		//memcpy(rxDataRaw, ppmData, 8);

		rxDataRaw[0] = ppmData[0];
		rxDataRaw[1] = ppmData[1];
		rxDataRaw[2] = ppmData[2];
		rxDataRaw[3] = ppmData[3];
		rxDataRaw[4] = ppmData[4];
		rxDataRaw[5] = ppmData[5];
		rxDataRaw[6] = ppmData[6];
		rxDataRaw[7] = ppmData[7];

		for (x=0;x<8;x++)
		{
			if ( (rxDataRaw[x] < 2200) && (rxDataRaw[x] > 500) )
			{
				rxData[x] = rxDataRaw[ChannelMap(x)];
			}
		}

		packetTime = (ppmBuffer[17] - ppmBuffer[0]);
		packetTimeInv = ((float)packetTime / 1000.0f);
		rx_timeout = 0;

		if (buzzerStatus.status == STATE_BUZZER_FAILSAFE)
			buzzerStatus.status = STATE_BUZZER_OFF;

		InlineCollectRcCommand();
		RxUpdate();

		ppmBuffer[0] = ppmBuffer[*ppmBufferIdx-1];
		*ppmBufferIdx = 1;
	}
	else if ( (*ppmBufferIdx > 1) && (ppmBuffer[1] - ppmBuffer[0]) <  PPM_SYNC_MINIMUM_US ) //we have at least two times and the first two aren't apart enough, so we reset the sync
	{
		//reset sync until we see a sync pulse
		ppmBuffer[0] = ppmBuffer[*ppmBufferIdx-1];
		*ppmBufferIdx = 1;
	}

}


void InitRcData(void)
{
	int32_t axis  = 0;

	//very first arm
	veryFirstArm = 1;
	rxUpdateCount = 0;

	bzero(rxCalibrationRecords, sizeof(rxCalibrationRecords));

	throttleVelocity = 0.0f;

	//precalculate max angles for RC Curves for the three stick axis
	for (axis = 3; axis >= 0; axis--)
	{
		maxKissRate[axis] = GetKissMaxRates(0.99f, axis);
		maxFlopRate[axis] = GetFlopMaxRates(0.99f, axis);
	}

	bzero(trueRcCommandF, MAXCHANNELS);
	bzero(curvedRcCommandF, MAXCHANNELS);
	bzero(smoothedRcCommandF, MAXCHANNELS);
	bzero(ppmBuffer, sizeof(ppmBuffer));
	ppmBufferIdx = 0;

	isRxDataNew = 0;

	vtxData.vtxChannel = 0;
	vtxData.vtxBand    = 0;
	vtxData.vtxPower   = 0;
	vtxData.vtxRegion  = 0;
	vtxData.vtxPit     = 0;

	activeFailsafe     = 0;
	failsafeHappend    = 0;

	armingStructure.boardArmed      = 0;
	armingStructure.latchFirstArm   = 0;
	armingStructure.armModeSet      = 0;
	armingStructure.armModeActive   = 0;
	armingStructure.rcCalibrated    = 0;
	armingStructure.boardCalibrated = 0;
	armingStructure.progMode        = 0;
	armingStructure.throttleIsSafe  = 0;
	armingStructure.rxTimeout       = 0;
	armingStructure.failsafeHappend = 0;
	armingStructure.activeFailsafe  = 0;

	//88  for spektrum at  8 KHz loop time
	//264 for spektrum at 24 KHz loop time
	//352 for spektrum at 32 KHz loop time
}


void PpmExtiCallback(uint32_t callbackNumber)
{
	(void)(callbackNumber);
	// EXTI line interrupt detected
	if(__HAL_GPIO_EXTI_GET_IT(ppmPin) != RESET)
	{
		//record time of IRQ in microseconds
		ppmBuffer[ppmBufferIdx++] = Micros();
		ProcessPpmPacket(ppmBuffer, &ppmBufferIdx);
		if (ppmBufferIdx == PPM_BUFFER_SIZE)
		{
			ppmBufferIdx = 0;
		}
		__HAL_GPIO_EXTI_CLEAR_IT(ppmPin);
	}
}


inline void InlineCollectRcCommand (void)
{

	uint32_t axis  = 0;
	float rangedRx = 0.0f;
	float deadBandToUse = 0.0f;

	isRxDataNew = 1; //this function is to be called by reception of vali RX data, so we know we have new RX data now


	//calculate main controls.
	//rc data is taken from RX and using the map is put into the correct "axis"
	for (axis = 0; axis < MAXCHANNELS; axis++)
	{
		deadBandToUse = mainConfig.rcControlsConfig.deadBand[axis];
		
		if(	ModeActive(M_LEARN) )
			deadBandToUse = 0.05f;

		if (rxData[axis] < mainConfig.rcControlsConfig.midRc[axis])  //negative  range
			rangedRx = InlineChangeRangef(rxData[axis], mainConfig.rcControlsConfig.midRc[(axis)], mainConfig.rcControlsConfig.minRc[(axis)], 0.00f, -1.0f); //-1 to 0
		else if ( (axis == THROTTLE) && (!mainConfig.rcControlsConfig.shortThrow) ) //add 5% deadband to top of throttle
			rangedRx = InlineChangeRangef(rxData[axis], mainConfig.rcControlsConfig.maxRc[(axis)], mainConfig.rcControlsConfig.midRc[(axis)], 1.05f, 0.0f); //0 to +1.05
		else if ( (axis == THROTTLE) ) //add 10% deadband to top of throttle
			rangedRx = InlineChangeRangef(rxData[axis], mainConfig.rcControlsConfig.maxRc[(axis)], mainConfig.rcControlsConfig.midRc[(axis)], 1.10f, 0.0f); //0 to +1.10
		else //positive range
			rangedRx = InlineChangeRangef(rxData[axis], mainConfig.rcControlsConfig.maxRc[(axis)], mainConfig.rcControlsConfig.midRc[(axis)], 1.00f, 0.0f); //0 to +1

		//do we want to apply deadband to trueRcCommandF? right now I think yes
		if (ABS(rangedRx) >= deadBandToUse)
		{
			//range with deadband in mind to produce smooth movement
			if(axis != THROTTLE)
			{
				if (rangedRx > 0)
				{
					rangedRx = InlineChangeRangef(rangedRx, 1.0f, deadBandToUse, 1.0f, 0.0f);
				}
				else
				{
					rangedRx = InlineChangeRangef(rangedRx, -deadBandToUse, -1.0f, 0.0f, -1.0f);
				}
			}

			trueRcCommandF[axis]   = InlineConstrainf( rangedRx, -1.0f, 1.0f);
			curvedRcCommandF[axis] = InlineApplyRcCommandCurve(trueRcCommandF[axis], mainConfig.tuneProfile[activeProfile].rcRates.useCurve, mainConfig.tuneProfile[activeProfile].rcRates.curveExpo[axis], axis);
		}
		else
		{
			// no need to calculate if movement is below deadband
			trueRcCommandF[axis]   = 0.0f;
			curvedRcCommandF[axis] = 0.0f;
		}

	}
	
}

static float GetKissMaxRates(float rcCommand, uint32_t axis)
{
	uint32_t negative        = 0;
	float    kissSetpoint    = 0.0f;
	float    kissRate        = 0.0f;
	float    kissGRate       = 0.0f;
	float    kissUseCurve    = 0.0f;
	float    kissTempCurve   = 0.0f;
	float    kissRpyUseRates = 0.0f;
	float    kissRxRaw       = 0.0f;
	float    kissAngle       = 0.0f;

	if (rcCommand<0.0f)
	{
		rcCommand = -rcCommand;
		negative  = 1;
	}

	kissUseCurve = (mainConfig.tuneProfile[activeProfile].rcRates.curveExpo[axis]);
	kissRate     = (mainConfig.tuneProfile[activeProfile].rcRates.acroPlus[axis] * 1.1f);
	kissGRate    = (mainConfig.tuneProfile[activeProfile].rcRates.rates[axis]);
	kissSetpoint = rcCommand;

	kissRpyUseRates = 1.0f - ABS(rcCommand) * kissGRate;
	kissRxRaw       = rcCommand * 1000.0f;
	kissTempCurve   = (kissRxRaw * kissRxRaw / 1000000.0f);
	kissSetpoint    = ((kissSetpoint * kissTempCurve) * kissUseCurve + kissSetpoint * (1.0f - kissUseCurve)) * (kissRate / 10.0f);
	kissAngle       = ((2000.0f * (1.0f / kissRpyUseRates)) * kissSetpoint); //setpoint is calculated directly here

	if (negative)
		return(-kissAngle);
	return(kissAngle);
}

static float GetFlopMaxRates(float rcCommand, uint32_t axis)
{
	uint32_t negative      = 0;
	float    flopSuperRate = 0.0f;
	float    flopRcRate    = 0.0f;
	float    flopExpo      = 0.0f;
	float    flopFactor    = 0.0f;
	float    flopAngle     = 0.0f;

	if (rcCommand<0.0f)
	{
		rcCommand = -rcCommand;
		negative  = 1;
	}

	flopExpo      = (mainConfig.tuneProfile[activeProfile].rcRates.curveExpo[axis]);
	flopRcRate    = (mainConfig.tuneProfile[activeProfile].rcRates.acroPlus[axis]);
	flopSuperRate = (mainConfig.tuneProfile[activeProfile].rcRates.rates[axis]);

	if (flopRcRate > 2.0f)
		flopRcRate = flopRcRate + (14.55f * (flopRcRate - 2.0f));

	if (flopExpo != 0.0f)
		rcCommand = rcCommand * Powerf(ABS(rcCommand), 3) * flopExpo + rcCommand * (1.0f-flopExpo);

	flopAngle = 200.0f * flopRcRate * rcCommand;

	if (flopSuperRate != 0.0f)
	{
		flopFactor = 1.0f / (InlineConstrainf(1.0f - (ABS(rcCommand) * (flopSuperRate)), 0.01f, 1.00f));
		flopAngle *= flopFactor; ; //setpoint is calculated directly here
	}
	if (negative)
		return(-flopAngle);
	return(flopAngle);
}

//return curved RC command as a percentage of max rate (-1.0 to 1.0)
 float InlineApplyRcCommandCurve(float rcCommand, uint32_t curveToUse, float expo, uint32_t axis)
{

	uint32_t negative        = 0;
	float    returnValue     = 0.0f;
	float    flopSuperRate   = 0.0f;
	float    flopRcRate      = 0.0f;
	float    flopExpo        = 0.0f;
	float    flopFactor      = 0.0f;
	float    flopAngle       = 0.0f;
	float    kissSetpoint    = 0.0f;
	float    kissRate        = 0.0f;
	float    kissGRate       = 0.0f;
	float    kissUseCurve    = 0.0f;
	float    kissTempCurve   = 0.0f;
	float    kissRpyUseRates = 0.0f;
	float    kissRxRaw       = 0.0f;
	float    kissAngle       = 0.0f;
	float    maxOutput       = 1.00f;
	float    maxOutputMod    = 0.01f;

	maxOutput    = 1.0f;
	maxOutputMod = 0.01f;

	switch (curveToUse)
	{

		case SKITZO_EXPO:
			returnValue = ((maxOutput + maxOutputMod * expo * (rcCommand * rcCommand - 1.0f)) * rcCommand * rcCommand);
			if (rcCommand < 0.0f) {
				returnValue = -returnValue;
			}
			return (returnValue);
			break;
		case BETAFLOP_EXPO:
			if (rcCommand < 0)
			{
				rcCommand = -rcCommand;
				negative  = 1;
			}
			if (rcCommand>0.98f)
				rcCommand = 0.98f;

			flopExpo      = (mainConfig.tuneProfile[activeProfile].rcRates.curveExpo[axis]);
			flopRcRate    = (mainConfig.tuneProfile[activeProfile].rcRates.acroPlus[axis]);
			flopSuperRate = (mainConfig.tuneProfile[activeProfile].rcRates.rates[axis]);

			if (flopRcRate > 2.0f)
				flopRcRate = flopRcRate + (14.55f * (flopRcRate - 2.0f));

			if (flopExpo != 0.0f)
				rcCommand = rcCommand * Powerf(ABS(rcCommand), 3) * flopExpo + rcCommand * (1.0f-flopExpo);

			flopAngle = 200.0f * flopRcRate * rcCommand;

			if (flopSuperRate != 0.0f)
			{
				flopFactor = 1.0f / (InlineConstrainf(1.0f - (ABS(rcCommand) * (flopSuperRate)), 0.01f, 1.00f));
				flopAngle *= flopFactor; ; //setpoint is calculated directly here
			}
			returnValue = (flopAngle / maxFlopRate[axis]); //get curved stick position based on percentage of setpoint
			if (negative)
				return(-returnValue);
			return(returnValue);
			break;
		case TARANIS_EXPO:
		case ACRO_PLUS:
		case FAST_EXPO:
			return ((maxOutput + maxOutputMod * expo * (rcCommand * rcCommand - 1.0f)) * rcCommand);
			break;
		case KISS_EXPO:
		case KISS_EXPO2:
			if (rcCommand>0.98f)
				rcCommand = 0.98f;

			kissUseCurve = (mainConfig.tuneProfile[activeProfile].rcRates.curveExpo[axis]);
			kissRate     = (mainConfig.tuneProfile[activeProfile].rcRates.acroPlus[axis] * 1.1f);
			kissGRate    = (mainConfig.tuneProfile[activeProfile].rcRates.rates[axis]);
			kissSetpoint = rcCommand;

			kissRpyUseRates = 1.0f - ABS(rcCommand) * kissGRate;
			kissRxRaw       = rcCommand * 1000.0f;
			kissTempCurve   = (kissRxRaw * kissRxRaw / 1000000.0f);
			kissSetpoint    = ((kissSetpoint * kissTempCurve) * kissUseCurve + kissSetpoint * (1.0f - kissUseCurve)) * (kissRate / 10.0f);
			kissAngle       = ((2000.0f * (1.0f / kissRpyUseRates)) * kissSetpoint); //setpoint is calculated directly here
			returnValue     = (kissAngle / maxKissRate[axis]); //get curved stick position based on percentage of setpoint
			return(returnValue);
			break;
		case NO_EXPO:
		default:
			return(rcCommand); //same as default for now.
			break;

	}
}

//runs every 10 ms
int CalculateThrottleVelocity(void)
{
	static volatile uint32_t throttleBufferIdx = 0;
	static volatile float throttleBuffer[10] = {0.0f,};

	throttleBuffer[throttleBufferIdx++] = smoothCurvedThrottle0_1;

	if(throttleBufferIdx == 10)
		throttleBufferIdx = 0;

	throttleVelocity = ABS(smoothCurvedThrottle0_1 - throttleBuffer[throttleBufferIdx]);

	return(0);
}

inline void InlineRcSmoothing(float curvedRcCommandF[], float smoothedRcCommandF[])
{
	static float   lastThrottleCommand = 0.0f;
	static float   lastThrottleDelta   = 0.0f;
    static float   lastCommand[4]      = {0.0f,};
    static float   deltaRC[4]          = {0.0f,};
    static int32_t factor              = 0;
    int32_t        channel             = 0;
	float          smoothToUse         = mainConfig.tuneProfile[activeProfile].rcRates.rcSmoothingFactor;

	//direct mode is smoothing 0
	if( ModeActive(M_DIRECT) )
		smoothToUse = 0.0f;

	//smoothing always 1 when learning
	if(	ModeActive(M_LEARN) )
		smoothToUse = 1.0f;
	
	if ( ( smoothToUse < 0.1f) || (mainConfig.tuneProfile[activeProfile].rcRates.useCurve == BETAFLOP_EXPO) || (mainConfig.tuneProfile[activeProfile].rcRates.useCurve == KISS_EXPO) )
	{
		for (channel=3; channel >= 0; channel--)
		{
			smoothedRcCommandF[channel] = curvedRcCommandF[channel];
		}
		smoothCurvedThrottle0_1 = trueCurvedThrottle0_1;
		//calculate Throttle veoclity
		ImuUpdateCommandQuat(curvedRcCommandF[ROLL], curvedRcCommandF[PITCH], curvedRcCommandF[YAW], (float)packetTimeInv * 0.5f );
		return;
	}

	smoothingInterval = lrintf((float)loopSpeed.khzDivider * (float)packetTime * smoothToUse ); //todo: calculate this number to be number of loops between PID loops
	//smoothingIntervalThrottle = lrintf((float)loopSpeed.khzDivider * (float)packetTime ); //todo: calculate this number to be number of loops between PID loops

    if (isRxDataNew)
    {

    	//set command quat here
    	ImuUpdateCommandQuat(curvedRcCommandF[ROLL], curvedRcCommandF[PITCH], curvedRcCommandF[YAW], (float)packetTimeInv * 0.5 );

        for (channel=3; channel >= 0; channel--)
        {
			deltaRC[channel]     = curvedRcCommandF[channel] - (lastCommand[channel] - ((deltaRC[channel] * (float)factor) / (float)smoothingInterval));
            lastCommand[channel] = curvedRcCommandF[channel];
        }
		lastThrottleDelta   = trueCurvedThrottle0_1 - (lastThrottleCommand - ((lastThrottleDelta * (float)factor) / (float)smoothingInterval));
		lastThrottleCommand = trueCurvedThrottle0_1;
        factor = smoothingInterval - 1;
        isRxDataNew = false;
    }
    else
    {
    	factor--;
    }

    if (factor > 0)
    {
    	for (channel=3; channel >= 0; channel--)
    	{
    		smoothedRcCommandF[channel] = (lastCommand[channel] - ( (deltaRC[channel] * (float)factor) / (float)smoothingInterval));
    	}
		smoothCurvedThrottle0_1 = (lastThrottleCommand - ( (lastThrottleDelta * (float)factor) / (float)smoothingInterval));
    }
    else
    {
    	factor = 0;
    }

}


void SetRxDefaults(uint32_t rxProtocol, uint32_t usart)
{

	mainConfig.rcControlsConfig.rxUsart   = usart;
	mainConfig.rcControlsConfig.rxProtcol = rxProtocol;

	//set some sane RC defaults for the protocol chosen
	switch(rxProtocol)
	{
		case USING_CPPM_R:
		case USING_CPPM_T:
			mainConfig.rcControlsConfig.shortThrow           = 1;
			mainConfig.rcControlsConfig.midRc[PITCH]         = 1100;
			mainConfig.rcControlsConfig.midRc[ROLL]          = 1100;
			mainConfig.rcControlsConfig.midRc[YAW]           = 1100;
			mainConfig.rcControlsConfig.midRc[THROTTLE]      = 1100;
			mainConfig.rcControlsConfig.midRc[AUX1]          = 1100;
			mainConfig.rcControlsConfig.midRc[AUX2]          = 1100;
			mainConfig.rcControlsConfig.midRc[AUX3]          = 1100;
			mainConfig.rcControlsConfig.midRc[AUX4]          = 1100;

			mainConfig.rcControlsConfig.minRc[PITCH]         = 800;
			mainConfig.rcControlsConfig.minRc[ROLL]          = 800;
			mainConfig.rcControlsConfig.minRc[YAW]           = 800;
			mainConfig.rcControlsConfig.minRc[THROTTLE]      = 800;
			mainConfig.rcControlsConfig.minRc[AUX1]          = 800;
			mainConfig.rcControlsConfig.minRc[AUX2]          = 800;
			mainConfig.rcControlsConfig.minRc[AUX3]          = 800;
			mainConfig.rcControlsConfig.minRc[AUX4]          = 800;

			mainConfig.rcControlsConfig.maxRc[PITCH]         = 1400;
			mainConfig.rcControlsConfig.maxRc[ROLL]          = 1400;
			mainConfig.rcControlsConfig.maxRc[YAW]           = 1400;
			mainConfig.rcControlsConfig.maxRc[THROTTLE]      = 1400;
			mainConfig.rcControlsConfig.maxRc[AUX1]          = 1400;
			mainConfig.rcControlsConfig.maxRc[AUX2]          = 1400;
			mainConfig.rcControlsConfig.maxRc[AUX3]          = 1400;
			mainConfig.rcControlsConfig.maxRc[AUX4]          = 1400;

			mainConfig.rcControlsConfig.channelMap[PITCH]    = 1;
			mainConfig.rcControlsConfig.channelMap[ROLL]     = 0;
			mainConfig.rcControlsConfig.channelMap[YAW]      = 3;
			mainConfig.rcControlsConfig.channelMap[THROTTLE] = 2;
			mainConfig.rcControlsConfig.channelMap[AUX1]     = 4;
			mainConfig.rcControlsConfig.channelMap[AUX2]     = 5;
			mainConfig.rcControlsConfig.channelMap[AUX3]     = 6;
			mainConfig.rcControlsConfig.channelMap[AUX4]     = 7;
			mainConfig.rcControlsConfig.channelMap[AUX5]     = 8;
			mainConfig.rcControlsConfig.channelMap[AUX6]     = 1000;
			mainConfig.rcControlsConfig.channelMap[AUX7]     = 1000;
			mainConfig.rcControlsConfig.channelMap[AUX8]     = 1000;
			mainConfig.rcControlsConfig.channelMap[AUX9]     = 1000;
			mainConfig.rcControlsConfig.channelMap[AUX10]    = 1000;
			mainConfig.rcControlsConfig.channelMap[AUX11]    = 1000;
			mainConfig.rcControlsConfig.channelMap[AUX12]    = 1000; //junk channel
			break;
		case USING_IBUS_R:
		case USING_IBUS_T:
			mainConfig.rcControlsConfig.shortThrow           = 1;
			mainConfig.rcControlsConfig.midRc[PITCH]         = 1500;
			mainConfig.rcControlsConfig.midRc[ROLL]          = 1500;
			mainConfig.rcControlsConfig.midRc[YAW]           = 1500;
			mainConfig.rcControlsConfig.midRc[THROTTLE]      = 1500;
			mainConfig.rcControlsConfig.midRc[AUX1]          = 1500;
			mainConfig.rcControlsConfig.midRc[AUX2]          = 1500;
			mainConfig.rcControlsConfig.midRc[AUX3]          = 1500;
			mainConfig.rcControlsConfig.midRc[AUX4]          = 1500;

			mainConfig.rcControlsConfig.minRc[PITCH]         = 1000;
			mainConfig.rcControlsConfig.minRc[ROLL]          = 1000;
			mainConfig.rcControlsConfig.minRc[YAW]           = 1000;
			mainConfig.rcControlsConfig.minRc[THROTTLE]      = 1000;
			mainConfig.rcControlsConfig.minRc[AUX1]          = 1000;
			mainConfig.rcControlsConfig.minRc[AUX2]          = 1000;
			mainConfig.rcControlsConfig.minRc[AUX3]          = 1000;
			mainConfig.rcControlsConfig.minRc[AUX4]          = 1000;

			mainConfig.rcControlsConfig.maxRc[PITCH]         = 2000;
			mainConfig.rcControlsConfig.maxRc[ROLL]          = 2000;
			mainConfig.rcControlsConfig.maxRc[YAW]           = 2000;
			mainConfig.rcControlsConfig.maxRc[THROTTLE]      = 2000;
			mainConfig.rcControlsConfig.maxRc[AUX1]          = 2000;
			mainConfig.rcControlsConfig.maxRc[AUX2]          = 2000;
			mainConfig.rcControlsConfig.maxRc[AUX3]          = 2000;
			mainConfig.rcControlsConfig.maxRc[AUX4]          = 2000;

			mainConfig.rcControlsConfig.channelMap[PITCH]    = 2;
			mainConfig.rcControlsConfig.channelMap[ROLL]     = 1;
			mainConfig.rcControlsConfig.channelMap[YAW]      = 3;
			mainConfig.rcControlsConfig.channelMap[THROTTLE] = 0;
			mainConfig.rcControlsConfig.channelMap[AUX1]     = 4;
			mainConfig.rcControlsConfig.channelMap[AUX2]     = 5;
			mainConfig.rcControlsConfig.channelMap[AUX3]     = 6;
			mainConfig.rcControlsConfig.channelMap[AUX4]     = 7;
			mainConfig.rcControlsConfig.channelMap[AUX5]     = 8;
			mainConfig.rcControlsConfig.channelMap[AUX6]     = 9;
			mainConfig.rcControlsConfig.channelMap[AUX7]     = 10;
			mainConfig.rcControlsConfig.channelMap[AUX8]     = 11;
			mainConfig.rcControlsConfig.channelMap[AUX9]     = 12;
			mainConfig.rcControlsConfig.channelMap[AUX10]    = 13;
			mainConfig.rcControlsConfig.channelMap[AUX11]    = 14;
			mainConfig.rcControlsConfig.channelMap[AUX12]    = 15; //junk channel
			break;
		case USING_SUMD_R:
		case USING_SUMD_T:
			mainConfig.rcControlsConfig.shortThrow           = 1;
			mainConfig.rcControlsConfig.midRc[PITCH]         = 12000;
			mainConfig.rcControlsConfig.midRc[ROLL]          = 12000;
			mainConfig.rcControlsConfig.midRc[YAW]           = 12000;
			mainConfig.rcControlsConfig.midRc[THROTTLE]      = 12000;
			mainConfig.rcControlsConfig.midRc[AUX1]          = 12000;
			mainConfig.rcControlsConfig.midRc[AUX2]          = 12000;
			mainConfig.rcControlsConfig.midRc[AUX3]          = 12000;
			mainConfig.rcControlsConfig.midRc[AUX4]          = 12000;

			mainConfig.rcControlsConfig.minRc[PITCH]         = 8000;
			mainConfig.rcControlsConfig.minRc[ROLL]          = 16000;
			mainConfig.rcControlsConfig.minRc[YAW]           = 16000;
			mainConfig.rcControlsConfig.minRc[THROTTLE]      = 16000;
			mainConfig.rcControlsConfig.minRc[AUX1]          = 8000;
			mainConfig.rcControlsConfig.minRc[AUX2]          = 8000;
			mainConfig.rcControlsConfig.minRc[AUX3]          = 8000;
			mainConfig.rcControlsConfig.minRc[AUX4]          = 8000;

			mainConfig.rcControlsConfig.maxRc[PITCH]         = 16000;
			mainConfig.rcControlsConfig.maxRc[ROLL]          = 8000;
			mainConfig.rcControlsConfig.maxRc[YAW]           = 8000;
			mainConfig.rcControlsConfig.maxRc[THROTTLE]      = 8000;
			mainConfig.rcControlsConfig.maxRc[AUX1]          = 16000;
			mainConfig.rcControlsConfig.maxRc[AUX2]          = 16000;
			mainConfig.rcControlsConfig.maxRc[AUX3]          = 16000;
			mainConfig.rcControlsConfig.maxRc[AUX4]          = 16000;

			mainConfig.rcControlsConfig.channelMap[PITCH]    = 2;
			mainConfig.rcControlsConfig.channelMap[ROLL]     = 1;
			mainConfig.rcControlsConfig.channelMap[YAW]      = 3;
			mainConfig.rcControlsConfig.channelMap[THROTTLE] = 0;
			mainConfig.rcControlsConfig.channelMap[AUX1]     = 4;
			mainConfig.rcControlsConfig.channelMap[AUX2]     = 5;
			mainConfig.rcControlsConfig.channelMap[AUX3]     = 6;
			mainConfig.rcControlsConfig.channelMap[AUX4]     = 7;
			mainConfig.rcControlsConfig.channelMap[AUX5]     = 8;
			mainConfig.rcControlsConfig.channelMap[AUX6]     = 9;
			mainConfig.rcControlsConfig.channelMap[AUX7]     = 10;
			mainConfig.rcControlsConfig.channelMap[AUX8]     = 11;
			mainConfig.rcControlsConfig.channelMap[AUX9]     = 12;
			mainConfig.rcControlsConfig.channelMap[AUX10]    = 13;
			mainConfig.rcControlsConfig.channelMap[AUX11]    = 14;
			mainConfig.rcControlsConfig.channelMap[AUX12]    = 15; //junk channel
			break;
		case USING_DSM2_R:
		case USING_DSM2_T:
			mainConfig.rcControlsConfig.shortThrow           = 1;
			mainConfig.rcControlsConfig.midRc[PITCH]         = 512;
			mainConfig.rcControlsConfig.midRc[ROLL]          = 512;
			mainConfig.rcControlsConfig.midRc[YAW]           = 512;
			mainConfig.rcControlsConfig.midRc[THROTTLE]      = 512;
			mainConfig.rcControlsConfig.midRc[AUX1]          = 512;
			mainConfig.rcControlsConfig.midRc[AUX2]          = 512;
			mainConfig.rcControlsConfig.midRc[AUX3]          = 512;
			mainConfig.rcControlsConfig.midRc[AUX4]          = 512;

			mainConfig.rcControlsConfig.minRc[PITCH]         = 12;
			mainConfig.rcControlsConfig.minRc[ROLL]          = 12;
			mainConfig.rcControlsConfig.minRc[YAW]           = 12;
			mainConfig.rcControlsConfig.minRc[THROTTLE]      = 12;
			mainConfig.rcControlsConfig.minRc[AUX1]          = 12;
			mainConfig.rcControlsConfig.minRc[AUX2]          = 12;
			mainConfig.rcControlsConfig.minRc[AUX3]          = 12;
			mainConfig.rcControlsConfig.minRc[AUX4]          = 12;

			mainConfig.rcControlsConfig.maxRc[PITCH]         = 1024;
			mainConfig.rcControlsConfig.maxRc[ROLL]          = 1024;
			mainConfig.rcControlsConfig.maxRc[YAW]           = 1024;
			mainConfig.rcControlsConfig.maxRc[THROTTLE]      = 1024;
			mainConfig.rcControlsConfig.maxRc[AUX1]          = 1024;
			mainConfig.rcControlsConfig.maxRc[AUX2]          = 1024;
			mainConfig.rcControlsConfig.maxRc[AUX3]          = 1024;
			mainConfig.rcControlsConfig.maxRc[AUX4]          = 1024;

			mainConfig.rcControlsConfig.channelMap[PITCH]    = 2;
			mainConfig.rcControlsConfig.channelMap[ROLL]     = 1;
			mainConfig.rcControlsConfig.channelMap[YAW]      = 3;
			mainConfig.rcControlsConfig.channelMap[THROTTLE] = 0;
			mainConfig.rcControlsConfig.channelMap[AUX1]     = 4;
			mainConfig.rcControlsConfig.channelMap[AUX2]     = 5;
			mainConfig.rcControlsConfig.channelMap[AUX3]     = 6;
			mainConfig.rcControlsConfig.channelMap[AUX4]     = 7;
			mainConfig.rcControlsConfig.channelMap[AUX5]     = 8;
			mainConfig.rcControlsConfig.channelMap[AUX6]     = 9;
			mainConfig.rcControlsConfig.channelMap[AUX7]     = 10;
			mainConfig.rcControlsConfig.channelMap[AUX8]     = 11;
			mainConfig.rcControlsConfig.channelMap[AUX9]     = 12;
			mainConfig.rcControlsConfig.channelMap[AUX10]    = 13;
			mainConfig.rcControlsConfig.channelMap[AUX11]    = 14;
			mainConfig.rcControlsConfig.channelMap[AUX12]    = 15; //junk channel
			break;
		case USING_SPEK_R:
		case USING_SPEK_T:
			mainConfig.rcControlsConfig.shortThrow           = 1;
			mainConfig.rcControlsConfig.midRc[PITCH]         = 1024;
			mainConfig.rcControlsConfig.midRc[ROLL]          = 1024;
			mainConfig.rcControlsConfig.midRc[YAW]           = 1024;
			mainConfig.rcControlsConfig.midRc[THROTTLE]      = 1024;
			mainConfig.rcControlsConfig.midRc[AUX1]          = 1024;
			mainConfig.rcControlsConfig.midRc[AUX2]          = 1024;
			mainConfig.rcControlsConfig.midRc[AUX3]          = 1024;
			mainConfig.rcControlsConfig.midRc[AUX4]          = 1024;

			mainConfig.rcControlsConfig.minRc[PITCH]         = 22;
			mainConfig.rcControlsConfig.minRc[ROLL]          = 22;
			mainConfig.rcControlsConfig.minRc[YAW]           = 22;
			mainConfig.rcControlsConfig.minRc[THROTTLE]      = 22;
			mainConfig.rcControlsConfig.minRc[AUX1]          = 342;
			mainConfig.rcControlsConfig.minRc[AUX2]          = 342;
			mainConfig.rcControlsConfig.minRc[AUX3]          = 342;
			mainConfig.rcControlsConfig.minRc[AUX4]          = 342;

			mainConfig.rcControlsConfig.maxRc[PITCH]         = 2025;
			mainConfig.rcControlsConfig.maxRc[ROLL]          = 2025;
			mainConfig.rcControlsConfig.maxRc[YAW]           = 2025;
			mainConfig.rcControlsConfig.maxRc[THROTTLE]      = 2025;
			mainConfig.rcControlsConfig.maxRc[AUX1]          = 1706;
			mainConfig.rcControlsConfig.maxRc[AUX2]          = 1706;
			mainConfig.rcControlsConfig.maxRc[AUX3]          = 1706;
			mainConfig.rcControlsConfig.maxRc[AUX4]          = 1706;

			mainConfig.rcControlsConfig.channelMap[PITCH]    = 2;
			mainConfig.rcControlsConfig.channelMap[ROLL]     = 1;
			mainConfig.rcControlsConfig.channelMap[YAW]      = 3;
			mainConfig.rcControlsConfig.channelMap[THROTTLE] = 0;
			mainConfig.rcControlsConfig.channelMap[AUX1]     = 4;
			mainConfig.rcControlsConfig.channelMap[AUX2]     = 5;
			mainConfig.rcControlsConfig.channelMap[AUX3]     = 6;
			mainConfig.rcControlsConfig.channelMap[AUX4]     = 7;
			mainConfig.rcControlsConfig.channelMap[AUX5]     = 8;
			mainConfig.rcControlsConfig.channelMap[AUX6]     = 9;
			mainConfig.rcControlsConfig.channelMap[AUX7]     = 10;
			mainConfig.rcControlsConfig.channelMap[AUX8]     = 11;
			mainConfig.rcControlsConfig.channelMap[AUX9]     = 12;
			mainConfig.rcControlsConfig.channelMap[AUX10]    = 13;
			mainConfig.rcControlsConfig.channelMap[AUX11]    = 14;
			mainConfig.rcControlsConfig.channelMap[AUX12]    = 15; //junk channel
			break;
		case USING_SBUS_R:
		case USING_SBUS_T:
			mainConfig.rcControlsConfig.shortThrow           = 1;
			mainConfig.rcControlsConfig.midRc[PITCH]         = 990;
			mainConfig.rcControlsConfig.midRc[ROLL]          = 990;
			mainConfig.rcControlsConfig.midRc[YAW]           = 990;
			mainConfig.rcControlsConfig.midRc[THROTTLE]      = 990;
			mainConfig.rcControlsConfig.midRc[AUX1]          = 990;
			mainConfig.rcControlsConfig.midRc[AUX2]          = 990;
			mainConfig.rcControlsConfig.midRc[AUX3]          = 990;
			mainConfig.rcControlsConfig.midRc[AUX4]          = 990;

			mainConfig.rcControlsConfig.minRc[PITCH]         = 170;
			mainConfig.rcControlsConfig.minRc[ROLL]          = 170;
			mainConfig.rcControlsConfig.minRc[YAW]           = 170;
			mainConfig.rcControlsConfig.minRc[THROTTLE]      = 170;
			mainConfig.rcControlsConfig.minRc[AUX1]          = 170;
			mainConfig.rcControlsConfig.minRc[AUX2]          = 170;
			mainConfig.rcControlsConfig.minRc[AUX3]          = 170;
			mainConfig.rcControlsConfig.minRc[AUX4]          = 170;

			mainConfig.rcControlsConfig.maxRc[PITCH]         = 1810;
			mainConfig.rcControlsConfig.maxRc[ROLL]          = 1810;
			mainConfig.rcControlsConfig.maxRc[YAW]           = 1810;
			mainConfig.rcControlsConfig.maxRc[THROTTLE]      = 1810;
			mainConfig.rcControlsConfig.maxRc[AUX1]          = 1810;
			mainConfig.rcControlsConfig.maxRc[AUX2]          = 1810;
			mainConfig.rcControlsConfig.maxRc[AUX3]          = 1810;
			mainConfig.rcControlsConfig.maxRc[AUX4]          = 1810;

			mainConfig.rcControlsConfig.channelMap[PITCH]    = 2;
			mainConfig.rcControlsConfig.channelMap[ROLL]     = 1;
			mainConfig.rcControlsConfig.channelMap[YAW]      = 3;
			mainConfig.rcControlsConfig.channelMap[THROTTLE] = 0;
			mainConfig.rcControlsConfig.channelMap[AUX1]     = 4;
			mainConfig.rcControlsConfig.channelMap[AUX2]     = 5;
			mainConfig.rcControlsConfig.channelMap[AUX3]     = 6;
			mainConfig.rcControlsConfig.channelMap[AUX4]     = 7;
			mainConfig.rcControlsConfig.channelMap[AUX5]     = 8;
			mainConfig.rcControlsConfig.channelMap[AUX6]     = 9;
			mainConfig.rcControlsConfig.channelMap[AUX7]     = 10;
			mainConfig.rcControlsConfig.channelMap[AUX8]     = 11;
			mainConfig.rcControlsConfig.channelMap[AUX9]     = 12;
			mainConfig.rcControlsConfig.channelMap[AUX10]    = 13;
			mainConfig.rcControlsConfig.channelMap[AUX11]    = 14;
			mainConfig.rcControlsConfig.channelMap[AUX12]    = 15; //junk channel
			break;
		case USING_CRSF_R:
		case USING_CRSF_T:
		case USING_CRSF_B:
			mainConfig.rcControlsConfig.shortThrow           = 1;
			mainConfig.rcControlsConfig.midRc[PITCH]         = 990;
			mainConfig.rcControlsConfig.midRc[ROLL]          = 990;
			mainConfig.rcControlsConfig.midRc[YAW]           = 990;
			mainConfig.rcControlsConfig.midRc[THROTTLE]      = 990;
			mainConfig.rcControlsConfig.midRc[AUX1]          = 990;
			mainConfig.rcControlsConfig.midRc[AUX2]          = 990;
			mainConfig.rcControlsConfig.midRc[AUX3]          = 990;
			mainConfig.rcControlsConfig.midRc[AUX4]          = 990;

			mainConfig.rcControlsConfig.minRc[PITCH]         = 170;
			mainConfig.rcControlsConfig.minRc[ROLL]          = 170;
			mainConfig.rcControlsConfig.minRc[YAW]           = 170;
			mainConfig.rcControlsConfig.minRc[THROTTLE]      = 170;
			mainConfig.rcControlsConfig.minRc[AUX1]          = 170;
			mainConfig.rcControlsConfig.minRc[AUX2]          = 170;
			mainConfig.rcControlsConfig.minRc[AUX3]          = 170;
			mainConfig.rcControlsConfig.minRc[AUX4]          = 170;

			mainConfig.rcControlsConfig.maxRc[PITCH]         = 1810;
			mainConfig.rcControlsConfig.maxRc[ROLL]          = 1810;
			mainConfig.rcControlsConfig.maxRc[YAW]           = 1810;
			mainConfig.rcControlsConfig.maxRc[THROTTLE]      = 1810;
			mainConfig.rcControlsConfig.maxRc[AUX1]          = 1810;
			mainConfig.rcControlsConfig.maxRc[AUX2]          = 1810;
			mainConfig.rcControlsConfig.maxRc[AUX3]          = 1810;
			mainConfig.rcControlsConfig.maxRc[AUX4]          = 1810;

			mainConfig.rcControlsConfig.channelMap[PITCH]    = 2;
			mainConfig.rcControlsConfig.channelMap[ROLL]     = 1;
			mainConfig.rcControlsConfig.channelMap[YAW]      = 3;
			mainConfig.rcControlsConfig.channelMap[THROTTLE] = 0;
			mainConfig.rcControlsConfig.channelMap[AUX1]     = 4;
			mainConfig.rcControlsConfig.channelMap[AUX2]     = 5;
			mainConfig.rcControlsConfig.channelMap[AUX3]     = 6;
			mainConfig.rcControlsConfig.channelMap[AUX4]     = 7;
			mainConfig.rcControlsConfig.channelMap[AUX5]     = 8;
			mainConfig.rcControlsConfig.channelMap[AUX6]     = 9;
			mainConfig.rcControlsConfig.channelMap[AUX7]     = 10;
			mainConfig.rcControlsConfig.channelMap[AUX8]     = 11;
			mainConfig.rcControlsConfig.channelMap[AUX9]     = 12;
			mainConfig.rcControlsConfig.channelMap[AUX10]    = 13;
			mainConfig.rcControlsConfig.channelMap[AUX11]    = 14;
			mainConfig.rcControlsConfig.channelMap[AUX12]    = 15; //junk channel
			break;
		default:
			return; //fail
	}


}