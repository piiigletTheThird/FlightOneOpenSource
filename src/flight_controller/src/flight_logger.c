#include "includes.h"

#define UPDATE_BB_CHAR_STRING_SIZE 10 //good for 100 days of logging
#define UPDATE_BB_TOTAL_HEADER_SIZE 256 //good for 100 days of logging
#define UPDATE_BB_DATA_SIZE 32 //good for 100 days of logging

#define BB_HEADER1 \
"H Product:Blackbox flight data recorder by Nicholas Sherlock\n" \
"H Data version:2\n" \
"H I interval:1\n\0"

#define BB_HEADER2 \
"H Field S name:flightModeFlags,stateFlags,failsafePhase,rxSignalReceived,rxFlightChannelsValid\n" \
"H Field S signed:0,0,0,0,0\n" \
"H Field S predictor:0,0,0,0,0\n" \
"H Field S encoding:1,1,7,7,7\n" \
"H Firmware type:Raceflight\n" \
"H Firmware revision:One\n" \
"H Firmware date:Oct 31 2017 22:44:00\n" \
"P interval:1/1\n" \
"H rcRate:100\n" \
"H minthrottle:1100\n" \
"H maxthrottle:2000\n" \
"H gyro.scale:0x41600000\n" \
"H acc_1G:1\n\0"

volatile int logMe;
volatile uint32_t lastProgramTime;
uint32_t LoggingEnabled;
uint32_t firstLogging;
uint32_t flashAlign;
uint32_t logTime;
uint32_t logItteration;
int32_t  logItterationCounter;
uint32_t logStartMillis;
uint32_t flashCountdownFake;
float currThrottleVelocity;

pid_output lastFlightPids[AXIS_NUMBER];
float      lastFlightSetPoints[AXIS_NUMBER];
float      lastFilteredGyroData[AXIS_NUMBER];
float      lastDpsGyroArray[AXIS_NUMBER];
float      lastFilteredAccData[AXIS_NUMBER];
float      lastMotorOutput[AXIS_NUMBER];

pid_output currFlightPids[AXIS_NUMBER];
float      currFlightSetPoints[AXIS_NUMBER];
float      currFilteredGyroData[AXIS_NUMBER];
float      currDpsGyroArray[AXIS_NUMBER];
float      currFilteredAccData[AXIS_NUMBER];
float      currKiTrim[AXIS_NUMBER];
float      currMotorOutput[4];
float      currRcCommandF[4];
float      currIteration;
float      currTime;

#define MAX_BB_VALUES 32

typedef struct
{
	const char *iName;
	const char *iSigned;
	const char *iPredictor;
	const char *iEncoding;
	const char *pPredictor;
	const char *pEncoding;
	float dataMultiplier;
	uint32_t dataType;
	void *data;
} __attribute__ ((__packed__)) bb_ip_value;

typedef struct
{
	int bbValuesTotal;
	const bb_ip_value *bbPtr[MAX_BB_VALUES];

} __attribute__ ((__packed__)) bb_values_used;

bb_values_used bbValues;

const bb_ip_value bb_data[MAX_BB_VALUES] =
{
		{ "H Field I name:", "H Field I signed:", "H Field I predictor:", "H Field I encoding:", "H Field P predictor:", "H Field P encoding:", 0, 0, 0},
		{ "loopIteration,", "0,", "0,", "1,", "6,", "9,", 1,     typeUINT,  &logItteration},
		{ "time,",          "0,", "0,", "1,", "1,", "0,", 1,     typeUINT,  &logTime},
		{ "axisP[2],",      "1,", "0,", "0,", "1,", "0,", 1000,  typeFLOAT, &currFlightPids[0].kp },
		{ "axisP[0],",      "1,", "0,", "0,", "1,", "0,", 1000,  typeFLOAT, &currFlightPids[1].kp },
		{ "axisP[1],",      "1,", "0,", "0,", "1,", "0,", 1000,  typeFLOAT, &currFlightPids[2].kp },
		{ "axisI[2],",      "1,", "0,", "0,", "1,", "0,", 1000,  typeFLOAT, &currFlightPids[0].ki },
		{ "axisI[0],",      "1,", "0,", "0,", "1,", "0,", 1000,  typeFLOAT, &currFlightPids[1].ki },
		{ "axisI[1],",      "1,", "0,", "0,", "1,", "0,", 1000,  typeFLOAT, &currFlightPids[2].ki },
		{ "axisD[2],",      "1,", "0,", "0,", "1,", "0,", 1000,  typeFLOAT, &currFlightPids[0].kd },
		{ "axisD[0],",      "1,", "0,", "0,", "1,", "0,", 1000,  typeFLOAT, &currFlightPids[1].kd },
		{ "axisD[1],",      "1,", "0,", "0,", "1,", "0,", 1000,  typeFLOAT, &currFlightPids[2].kd },

		{ "rcCommand[2],",  "1,", "0,", "0,", "1,", "0,", 500,   typeFLOAT, &currRcCommandF[YAW]},
		{ "rcCommand[0],",  "1,", "0,", "0,", "1,", "0,", 500,   typeFLOAT, &currRcCommandF[ROLL]},
		{ "rcCommand[1],",  "1,", "0,", "0,", "1,", "0,", 500,   typeFLOAT, &currRcCommandF[PITCH]},
		{ "rcCommand[3],",  "1,", "0,", "0,", "1,", "0,", 1,     typeFLOAT, &currRcCommandF[THROTTLE]},

		//{ "debug[2],",      "1,", "0,", "0,", "1,", "0,", 16.4,  typeFLOAT, &currFlightSetPoints[YAW]},
		//{ "debug[0],",      "1,", "0,", "0,", "1,", "0,", 16.4,  typeFLOAT, &currFlightSetPoints[ROLL]},
		//{ "debug[1],",      "1,", "0,", "0,", "1,", "0,", -16.4, typeFLOAT, &currFlightSetPoints[PITCH]},

		{ "debug[2],",      "1,", "0,", "0,", "1,", "0,", 1000,  typeFLOAT, &currFlightSetPoints[YAW]},
		{ "debug[0],",      "1,", "0,", "0,", "1,", "0,", 1000,  typeFLOAT, &currFlightSetPoints[ROLL]},
		{ "debug[1],",      "1,", "0,", "0,", "1,", "0,", 1000,  typeFLOAT, &currFlightSetPoints[PITCH]},

		{ "ugyroADC[2],",   "1,", "0,", "0,", "1,", "0,", 16.4,  typeFLOAT, &currFilteredGyroData[YAW]},
		{ "ugyroADC[0],",   "1,", "0,", "0,", "1,", "0,", 16.4,  typeFLOAT, &currFilteredGyroData[ROLL]},
		{ "ugyroADC[1],",   "1,", "0,", "0,", "1,", "0,", 16.4,  typeFLOAT, &currFilteredGyroData[PITCH]},

		{ "accSmooth[2],",  "1,", "0,", "0,", "1,", "0,", 2048,  typeFLOAT, &currFilteredAccData[ACCX]},
		{ "accSmooth[0],",  "1,", "0,", "0,", "1,", "0,", 2048,  typeFLOAT, &currFilteredAccData[ACCY]},
		{ "accSmooth[1],",  "1,", "0,", "0,", "1,", "0,", 2048,  typeFLOAT, &currFilteredAccData[ACCZ]},

		{ "motor[0],",      "1,", "0,", "0,", "1,", "0,", 1000,  typeFLOAT, &currMotorOutput[0]},
		{ "motor[1],",      "1,", "0,", "0,", "1,", "0,", 1000,  typeFLOAT, &currMotorOutput[1]},
		{ "motor[2],",      "1,", "0,", "0,", "1,", "0,", 1000,  typeFLOAT, &currMotorOutput[2]},
		{ "motor[3],",      "1,", "0,", "0,", "1,", "0,", 1000,  typeFLOAT, &currMotorOutput[3]},

		{ "voltageRaw,",    "1,", "0,", "0,", "1,", "0,", 1000,  typeFLOAT, &adcVoltage},
		{ "voltageAvg,",    "1,", "0,", "0,", "1,", "0,", 1000,  typeFLOAT, &averageVoltage},
		{ "current,",       "1,", "0,", "0,", "1,", "0,", 1000,  typeFLOAT, &adcCurrent},

		//{ "filty,",    "1,", "0,", "0,", "1,", "0,", 1000000,  typeFLOAT, &pafGyroStates[YAW].k},
		//{ "filtr,",    "1,", "0,", "0,", "1,", "0,", 1000000,  typeFLOAT, &pafGyroStates[ROLL].k},
		//{ "filtp,",     "1,", "0,", "0,", "1,", "0,", 1000000,  typeFLOAT, &pafGyroStates[PITCH].k},
		

};

static int PrintLogHeaderToBuffer(void);
static int SetupLog(void);

static int PrintLogHeaderToBuffer(void)
{

	int x;

	if(rfCustomReplyBufferPointer) //buffer in use, don't do anything
	{
		//don't write a header. HID buffer is in use.
		headerWritten=0;
		headerToWrite=0;
		return(0);
	}

	headerWritten=0;
	headerToWrite=0;

	snprintf(rfCustomSendBuffer+rfCustomReplyBufferPointer, strlen(BB_HEADER1)+1, "%s", BB_HEADER1);
	headerToWrite += strlen(BB_HEADER1);
	//headerToWrite += strlen(BB_HEADER1)+rfCustomReplyBufferPointer;
	
	for(x=0;x<bbValues.bbValuesTotal;x++)
	{
		snprintf(rfCustomSendBuffer+headerToWrite, strlen(bbValues.bbPtr[x]->iName)+1, "%s", bbValues.bbPtr[x]->iName);
		headerToWrite+=strlen(bbValues.bbPtr[x]->iName);
	}
	snprintf(rfCustomSendBuffer+headerToWrite-1, 2, "\n");

	for(x=0;x<bbValues.bbValuesTotal;x++)
	{
		snprintf(rfCustomSendBuffer+headerToWrite, strlen(bbValues.bbPtr[x]->iSigned)+1, "%s", bbValues.bbPtr[x]->iSigned);
		headerToWrite+=strlen(bbValues.bbPtr[x]->iSigned);
	}
	snprintf(rfCustomSendBuffer+headerToWrite-1, 2, "\n");

	for(x=0;x<bbValues.bbValuesTotal;x++)
	{
		snprintf(rfCustomSendBuffer+headerToWrite, strlen(bbValues.bbPtr[x]->iPredictor)+1, "%s", bbValues.bbPtr[x]->iPredictor);
		headerToWrite+=strlen(bbValues.bbPtr[x]->iPredictor);
	}
	snprintf(rfCustomSendBuffer+headerToWrite-1, 2, "\n");

	for(x=0;x<bbValues.bbValuesTotal;x++)
	{
		snprintf(rfCustomSendBuffer+headerToWrite, strlen(bbValues.bbPtr[x]->iEncoding)+1, "%s", bbValues.bbPtr[x]->iEncoding);
		headerToWrite+=strlen(bbValues.bbPtr[x]->iEncoding);
	}
	snprintf(rfCustomSendBuffer+headerToWrite-1, 2, "\n");

	for(x=0;x<bbValues.bbValuesTotal;x++)
	{
		snprintf(rfCustomSendBuffer+headerToWrite, strlen(bbValues.bbPtr[x]->pPredictor)+1, "%s", bbValues.bbPtr[x]->pPredictor);
		headerToWrite+=strlen(bbValues.bbPtr[x]->pPredictor);
	}
	snprintf(rfCustomSendBuffer+headerToWrite-1, 2, "\n");

	for(x=0;x<bbValues.bbValuesTotal;x++)
	{
		snprintf(rfCustomSendBuffer+headerToWrite, strlen(bbValues.bbPtr[x]->pEncoding)+1, "%s", bbValues.bbPtr[x]->pEncoding);
		headerToWrite+=strlen(bbValues.bbPtr[x]->pEncoding);
	}
	//volatile int asdfasf=33333;
	snprintf(rfCustomSendBuffer+headerToWrite-1, 2, "\n");


	snprintf(rfCustomSendBuffer+headerToWrite, strlen(BB_HEADER2)+1, "%s\n", BB_HEADER2);
	headerToWrite += strlen(BB_HEADER2);


	rfCustomReplyBufferPointer = headerToWrite;
	rfCustomReplyBufferPointerSent = 0;

	//fill buffer with config dump
	RfCustomReplyBuffer("H RF1 Dump\n");
	RfCustomReplyBuffer(FULL_VERSION_STRING);
	snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#fc HARDWARE:%s\n", FC_NAME);
	RfCustomReplyBuffer(rf_custom_out_buffer);
	PrintModes();
	PrintTpaCurves();
	for (x=0;x<(int)configSize;x++)
		OutputVarSet(x, 0);

	headerToWrite += rfCustomReplyBufferPointer;
	rfCustomReplyBufferPointer = 0; //done with use of this pointer. HID can overwrite the buffer if needed now.
	rfCustomReplyBufferPointerSent = 0;

	return(0);

}

static int SetupLog(void)
{
	int x;

	bbValues.bbValuesTotal = 0;
	for(x=0;x<MAX_BB_VALUES;x++)
	{
		if(x < 32)
		{
			//if(mainConfig.telemConfig.logMask1 & (1 << x))
			bbValues.bbPtr[bbValues.bbValuesTotal++] = &bb_data[x];
		}
		else
		{
			//if(mainConfig.telemConfig.logMask2 & (1 << (x-32)))
				//bbValues.bbPtr[bbValues.bbValuesTotal++] = &bb_data[x];
			bbValues.bbPtr[bbValues.bbValuesTotal++] = &bb_data[x];
		}
	}
	return(0);
}

int InitFlightLogger(void)
{

	logMe              = 0;
	flashCountdownFake = 0;
	logItterationCounter = 1;
	LoggingEnabled = 0;
	firstLogging   = 1;
	flashAlign     = 0;
	logItteration  = 0;
	logStartMillis = 0;
	bzero(lastFlightPids, sizeof(pid_output));
	bzero(lastFlightSetPoints, sizeof(lastFlightSetPoints));
	bzero(lastFilteredGyroData, sizeof(lastFilteredGyroData));
	bzero(lastDpsGyroArray, sizeof(lastDpsGyroArray));
	bzero(lastFilteredAccData, sizeof(lastFilteredAccData));
	bzero(lastMotorOutput, sizeof(lastMotorOutput));

	SetupLog();

	return (1);

}

void EnableLogging(void)
{
	LoggingEnabled = 1;
}

void DisableLogging(void)
{
	LoggingEnabled = 0;
}


void FinishPage(void)
{
	uint32_t remaingBytes = (flashInfo.pageSize - (flashInfo.buffer[flashInfo.bufferNum].txBufferPtr - FLASH_CHIP_BUFFER_WRITE_DATA_START) );
	for (uint32_t x=0;x<remaingBytes;x++)
	{
		WriteByteToFlash('\0');
	}
}

void FinishBlock(uint32_t count)
{
	uint32_t finishY = ((flashInfo.currentWriteAddress + (flashInfo.buffer[flashInfo.bufferNum].txBufferPtr - FLASH_CHIP_BUFFER_WRITE_DATA_START)) % count);
	if (finishY != 0)
	{
		for (uint32_t x=0;x<(count - finishY);x++)
		{
			WriteByteToFlash('\0');
		}
	}
}

void InlineWrite16To8 (int16_t data)
{
	WriteByteToFlash(  (uint8_t)( data >> 8 ) );
	WriteByteToFlash(  (uint8_t)(data & 0xff) );
}

void WriteByteToFlash (uint8_t data)
{

	buffer_record *buffer = &flashInfo.buffer[flashInfo.bufferNum];

	buffer->txBuffer[buffer->txBufferPtr++] = data;

	if (buffer->txBufferPtr > FLASH_CHIP_BUFFER_WRITE_DATA_END)
	{
		if (flashInfo.bufferNum == 0)
		{
			flashInfo.bufferNum = 1;
		}
		else
		{
			flashInfo.bufferNum = 0;
		}
		flashInfo.buffer[flashInfo.bufferNum].txBufferPtr = FLASH_CHIP_BUFFER_WRITE_DATA_START;

		#ifdef OLD_LOG
		if (flashInfo.status != DMA_DATA_WRITE_IN_PROGRESS)
		{
			//only write and increment write address is flash chip s not busy to prevent blocks of FFFFFFF
			//M25p16BlockingWritePage(flashInfo.currentWriteAddress, buffer->txBuffer, buffer->rxBuffer);
			M25p16DmaWritePage(flashInfo.currentWriteAddress, buffer->txBuffer, buffer->rxBuffer); //write buffer to flash using DMA
			flashInfo.currentWriteAddress += FLASH_CHIP_BUFFER_WRITE_DATA_SIZE; //add pointer to address
		}
		#else
		//if flash not being written to we set the variables the scheduler uses to write to flash
		if(flashTxBuffer == NULL)
		{
			flashWriteAddress = flashInfo.currentWriteAddress;
			flashTxBuffer     = (uint8_t *)buffer->txBuffer;
			flashRxBuffer     = (uint8_t *)buffer->rxBuffer;
			flashInfo.currentWriteAddress += FLASH_CHIP_BUFFER_WRITE_DATA_SIZE; //add pointer to address
		}
		#endif

		//if (flashInfo.currentWriteAddress >= flashInfo.totalSize)
		if (flashInfo.currentWriteAddress >= persistance.start1) //last two sectors are for persistance
			flashInfo.enabled = FLASH_FULL; //check if flash is full. Disable flash if it is full

	}

}

int DumbWriteString(char *string, int sizeOfString)
{
	for (int x=0; x < sizeOfString; x++)
		WriteByteToFlash( string[x] );

	return sizeOfString;
}

#define STARTLOG "STARTLOG"
#define ITERATION "iteration"

uint32_t ZigzagEncode(int32_t value)
{
    return ( (uint32_t)((value << 1) ^ (value >> 31)) );
}

void BlackboxWriteUnsignedVB(uint32_t value)
{
	while (value > 127)
	{ // one acceptable use of while loop, because 3 shifts is 0, guaranteeed P.G.
    	WriteByteToFlash((uint8_t) (value | 0x80)); // Set the high bit to mean "more bytes follow"
        value >>= 7;
    }
    WriteByteToFlash( (uint8_t)value );
}

void BlackboxWriteSignedVB(int32_t value)
{
    //ZigZag encode to make the value always positive
    BlackboxWriteUnsignedVB(ZigzagEncode(value));
}

void UpdateBlackbox(pid_output flightPids[], float flightSetPoints[], float dpsGyroArray[], float filteredGyroData[], float filteredAccData[] )
{

	uint32_t        finishX;
	uint32_t        recordJunkData = 0;
	static uint32_t loggingStartedLatch = 0;
	static int32_t  disarmLast = 0;
	volatile int toWrite;
	int x;

	#ifdef OLD_LOG
	if (IsDshotEnabled())
		return;
	#endif
#ifndef LOG32

#else
	(void)(flightPids);
	(void)(flightSetPoints);
	(void)(filteredAccData);
#endif

	if (
		(logMe) ||
		( (mainConfig.rcControlsConfig.rcCalibrated) && (boardArmed) && (ModeActive(M_LOGGING)) && (flashInfo.enabled == STAT_FLASH_ENABLED) ) ||
		( (mainConfig.rcControlsConfig.rcCalibrated) && (boardArmed) && (!ModeSet(M_LOGGING)) && (flashInfo.enabled == STAT_FLASH_ENABLED) )
	)
	{
		ledStatus.status    = LEDS_FASTER_BLINK;
		LoggingEnabled      = 1;
		loggingStartedLatch = 1;
		disarmLast          = 25;
		recordJunkData      = 0;
	}
	else if ( flashCountdownFake > 1 )
	{
		flashCountdownFake--;
		ledStatus.status    = LEDS_FASTER_BLINK;
		LoggingEnabled      = 1;
		loggingStartedLatch = 1;
		disarmLast          = 25;
		recordJunkData      = 0;
	}
	else
	{
		if (boardArmed)
			ledStatus.status = LEDS_MED_BLINK;
		else
			ledStatus.status = LEDS_SLOW_BLINK;

		if (disarmLast-- < 1)
		{
			disarmLast = -1;
			LoggingEnabled = 0;
			firstLogging = 1;
			if (loggingStartedLatch)
			{
				loggingStartedLatch = 0;
				FinishPage();
			}
		}
		recordJunkData = 1;
	}


	if ( (LoggingEnabled) && (flashInfo.enabled == STAT_FLASH_ENABLED) )
	{
		logItteration++;

		logTime = (InlineMillis() * 1000);

		if (firstLogging)
		{
			PrintLogHeaderToBuffer();
			logStartMillis = InlineMillis();

			flashInfo.buffer[0].txBufferPtr = FLASH_CHIP_BUFFER_WRITE_DATA_START;
			flashInfo.buffer[1].txBufferPtr = FLASH_CHIP_BUFFER_WRITE_DATA_START;
			flashInfo.buffer[0].rxBufferPtr = 0;
			flashInfo.buffer[1].rxBufferPtr = 0;
			flashInfo.bufferNum             = 0;

			finishX = (flashInfo.currentWriteAddress % UPDATE_BB_DATA_SIZE);
			if (finishX != 0)
			{
				flashInfo.currentWriteAddress += (UPDATE_BB_DATA_SIZE - finishX);
			}

			
			if(headerWritten < headerToWrite)
			{
				toWrite = CONSTRAIN(headerToWrite - headerWritten, (int)0, (int)flashInfo.pageSize);
				DumbWriteString(rfCustomSendBuffer+headerWritten, toWrite);
				headerWritten=headerWritten+toWrite;
			}

			lastProgramTime = InlineMillis();

			firstLogging = 0;
			logItterationCounter = 1;

		}
		else
		{

			if(--logItterationCounter == 0)
			{
				if(usedSkunk == 2)
					logItterationCounter = 4;	//TODO make this configurable value. Capture rate = 1khz/value
				else
					logItterationCounter = 2;	//TODO make this configurable value. Capture rate = 1khz/value
				
				if (IsDshotEnabled())
					logItterationCounter = 4;

				//no logging until header is written
				if(headerWritten < headerToWrite)
				{
					//don't try to right the header until the flash is ready
					if (
						( (InlineMillis() - lastProgramTime) > 6) &&
						(flashInfo.status == READ_ANDOR_WRITE_COMPLETE)
					)
					{
							toWrite = CONSTRAIN(headerToWrite - headerWritten, (int)0, (int)flashInfo.pageSize);
							DumbWriteString(rfCustomSendBuffer+headerWritten, toWrite);
							headerWritten+=toWrite;
							lastProgramTime = InlineMillis();
							return;
					}
					return;
				}

#ifndef LOG32
				//average all values
				for (finishX = 0; finishX < AXIS_NUMBER; finishX++)
				{
					currFlightPids[finishX].kp    = ( (flightPids[finishX].kp + lastFlightPids[finishX].kp) * 0.5);
					currFlightPids[finishX].ki    = ( (flightPids[finishX].ki + lastFlightPids[finishX].ki) * 0.5);
					currFlightPids[finishX].kd    = ( (flightPids[finishX].kd + lastFlightPids[finishX].kd) * 0.5);
					currFlightSetPoints[finishX]  = ( (flightSetPoints[finishX] + lastFlightSetPoints[finishX]) * 0.5);
					currFilteredGyroData[finishX] = ( (filteredGyroData[finishX] + lastFilteredGyroData[finishX]) * 0.5);
					currDpsGyroArray[finishX]     = ( (dpsGyroArray[finishX] + lastDpsGyroArray[finishX]) * 0.5);
					currFilteredAccData[finishX]  = ( (filteredAccData[finishX] + lastFilteredAccData[finishX]) * 0.5);
				}

				/*
				static float stdDevKp[40] = {0.0f,};
				static float stdDevKd[40] = {0.0f,};
				static int   stdDevCtr    = 0;
				static int   stdDevAxis   = 0;
				static int   axisOverflow = 0;
				stdDevAxis = 1;
				
				stdDevKp[stdDevCtr]   = flightPids[stdDevAxis].kp;
				stdDevKd[stdDevCtr++] = flightPids[stdDevAxis].kd;

				//std dev of last 10
				if(stdDevCtr == 40)
				{
					stdDevCtr = 0;
					//axisOverflow++;
				}
				float differenceHp = 0.0f;
				float differenceLp = 0.0f;
				float differenceHd = 0.0f;
				float differenceLd = 0.0f;
				
				//running std dev
				for(int z=0; z<40; z++)
				{
					if(differenceHp < stdDevKp[z])
						differenceHp = stdDevKp[z];
					if(differenceHd < stdDevKd[z])
						differenceHd = stdDevKd[z];

					if(differenceLp > stdDevKp[z])
						differenceLp = stdDevKp[z];
					if(differenceLd > stdDevKd[z])
						differenceLd = stdDevKd[z];
				}

				currFlightSetPoints[0]  = differenceHp - differenceLp; //CalculateSDSize(stdDevKp, 10) * 10.0f;
				currFlightSetPoints[1]  = differenceHd - differenceLd; //CalculateSDSize(stdDevKd, 10) * 10.0f;
				currFlightSetPoints[2]  = currFlightSetPoints[0] / currFlightSetPoints[1];

				//changed  axis every 100 times
				//if(axisOverflow == 10)
				//{
				//	axisOverflow = 0;
				//	stdDevAxis++;
				//	if(stdDevAxis == 3)
				//		stdDevAxis = 0;
				//}
				*/
				
				currMotorOutput[0] = ( (motorOutput[0] + lastMotorOutput[0]) * 0.5f) + 1.000f;
				currMotorOutput[1] = ( (motorOutput[1] + lastMotorOutput[1]) * 0.5f) + 1.000f;
				currMotorOutput[2] = ( (motorOutput[2] + lastMotorOutput[2]) * 0.5f) + 1.000f;
				currMotorOutput[3] = ( (motorOutput[3] + lastMotorOutput[3]) * 0.5f) + 1.000f;

				//currKiTrim[0] = kiTrim[0];
				//currKiTrim[1] = kiTrim[1];
				//currKiTrim[2] = kiTrim[2];
				
				//if(currMotorOutput[0] > 1.95f)
				//	currMotorOutput[0] = 2.0f;
				
				//if(currMotorOutput[1] > 1.95f)
				//	currMotorOutput[1] = 2.0f;
				
				//if(currMotorOutput[2] > 1.95f)
				//	currMotorOutput[2] = 2.0f;
				
				//if(currMotorOutput[3] > 1.95f)
				//	currMotorOutput[3] = 2.0f;

				//currThrottleVelocity = throttleVelocity;
				//copy current value to last values.
				memcpy(lastFlightPids, flightPids, sizeof(pid_output));
				memcpy(lastFlightSetPoints, flightSetPoints, sizeof(lastFlightSetPoints));
				memcpy(lastFilteredGyroData, filteredGyroData, sizeof(lastFilteredGyroData));
				memcpy(lastDpsGyroArray, dpsGyroArray, sizeof(lastDpsGyroArray));
				memcpy(lastFilteredAccData, filteredAccData, sizeof(lastFilteredAccData));

				lastMotorOutput[0] = motorOutput[0];
				lastMotorOutput[1] = motorOutput[1];
				lastMotorOutput[2] = motorOutput[2];
				lastMotorOutput[3] = motorOutput[3];

				currRcCommandF[YAW]   = smoothedRcCommandF[YAW];
				currRcCommandF[ROLL]  = smoothedRcCommandF[ROLL];
				currRcCommandF[PITCH] = smoothedRcCommandF[PITCH];
				//1000 to 1850
				currRcCommandF[THROTTLE] = ( ((smoothedRcCommandF[THROTTLE] + 1) * 500) + 1000) ;

#endif

				//write iframe
				WriteByteToFlash( 'I' );

				if (recordJunkData)
				{
					toWrite = 7;
					//BlackboxWriteSignedVB( (int)999 );
					//BlackboxWriteSignedVB( (int)999 );
					BlackboxWriteSignedVB( (int)999 );
					BlackboxWriteSignedVB( (int)rx_timeout );
					BlackboxWriteSignedVB( (int)deviceWhoAmI );
					BlackboxWriteSignedVB( (int)errorMask );
					BlackboxWriteSignedVB( (int)failsafeHappend );
					BlackboxWriteSignedVB( (int)activeModes );
				}
				else
				{
					toWrite = 1;
				}

				uint32_t temp1;
				int temp2;
				float temp3;
				for(x=toWrite;x<bbValues.bbValuesTotal;x++)
				{					
					switch(bbValues.bbPtr[x]->dataType)
					{
						case typeUINT:
							temp1 = (*(uint32_t *)bbValues.bbPtr[x]->data);
							temp1 *= (uint32_t)bbValues.bbPtr[x]->dataMultiplier;
							BlackboxWriteUnsignedVB( (uint32_t)( temp1 ));
							break;
						case typeINT:
							temp2 = (*(int *)bbValues.bbPtr[x]->data);
							temp2 *= (int)bbValues.bbPtr[x]->dataMultiplier;
							BlackboxWriteSignedVB( (int)( temp2 ));
							break;
						case typeFLOAT:
							temp3 = (*(float *)bbValues.bbPtr[x]->data);
							temp3 *= (float)bbValues.bbPtr[x]->dataMultiplier;
							BlackboxWriteSignedVB( (int)( temp3 ));
							break;
						case typeFLOATu:
							temp3 = (*(float *)bbValues.bbPtr[x]->data);
							temp3 *= (float)bbValues.bbPtr[x]->dataMultiplier;
							BlackboxWriteUnsignedVB( (uint32_t)( temp3 ));
							break;
					}
				}

#ifndef LOG32

				(void)(dpsGyroArray);
				(void)(currDpsGyroArray);
#else
				(void)(recordJunkData);
				BlackboxWriteSignedVB( (int32_t)((float)(filteredGyroData[YAW])   * 16.4) );
				BlackboxWriteSignedVB( (int32_t)((float)(dpsGyroArray[YAW])       * 16.4) );
#endif

			}

		}

	}
	else
	{
		ledStatus.status = LEDS_SLOW_BLINK;
	}
}
