#include "includes.h"

#define TRAMP_RETRIES 3
#define TRAMP_BUFFER_SIZE 16

static uint8_t  trampIoBuffer[TRAMP_BUFFER_SIZE];     //set

static uint8_t TrampChecksum(uint8_t trampBuffer[]);
static void    TrampSendFreq(int frequency);
static void    TrampSendRfPower(int power);
static int     TrampSendCommand(uint8_t cmd, uint16_t param, int waitForResponse);
static void    TrampResetInfoRecord(void);
static void    TrampSetPitMode(int on);

typedef enum
{
	TRAMP_ERROR  = -1,
	TRAMP_OFF    =  0,
	TRAMP_ON     =  1,
	TRAMP_SET    =  2,
	TRAMP_CHECK  =  3,
} tramp_status_record;

typedef struct
{
    tramp_status_record trampStatus;
    uint32_t trampRfFreqMin;
    uint32_t trampRfFreqMax;
    uint32_t trampRfPowerMax;
    uint32_t trampCurFreq;     //current frequency
    uint32_t trampReqFreq;     //requested frequency
    uint32_t trampRetFreq;     //retries for setting frequency
    uint32_t trampBand;
    uint32_t trampChannel;
    uint32_t trampCurPower;    //current power
    uint32_t trampReqPower;    //requested power
    uint32_t trampRetPower;    //retries for setting power
    int      trampTemp;
    uint32_t trampPitMode;
} tramp_info_record;

tramp_info_record trampInfo;

static void TrampResetInfoRecord(void)
{
    trampInfo.trampStatus     = TRAMP_OFF;
    trampInfo.trampRfFreqMin  = 0;
    trampInfo.trampRfFreqMax  = 0;
    trampInfo.trampRfPowerMax = 0;
    trampInfo.trampCurFreq    = 0;
    trampInfo.trampReqFreq    = 0;
    trampInfo.trampRetFreq    = 0;
    trampInfo.trampBand       = 0;
    trampInfo.trampChannel    = 0;
    trampInfo.trampCurPower   = 0;
    trampInfo.trampReqPower   = 0;
    trampInfo.trampRetPower   = 0;
    trampInfo.trampTemp       = 0;
    trampInfo.trampPitMode    = 0;
}

int InitTrampTelemetry(int usartNumber)
{

    TrampResetInfoRecord();

    board.serials[usartNumber].Protocol = USING_TRAMP;
    //not using DMA
	board.dmasSerial[board.serials[usartNumber].TXDma].enabled  = 0;
	board.dmasSerial[board.serials[usartNumber].RXDma].enabled  = 0;

	UsartDeInit(usartNumber); //deinits serial and associated pins and DMAs
	UsartInit(usartNumber);   //inits serial and associated pins and DMAs if used. Serial settings are set in serial.c

    return(1);
}

int TrampHandleResponse(uint8_t trampBuffer[])
{
    //uint8_t volatile crc1 = TrampChecksum(trampBuffer);
    //uint8_t volatile crc2 = trampBuffer[13];
    //uint8_t volatile crc3 = trampBuffer[14];
    //uint8_t volatile crc4 = trampBuffer[15];
    //uint8_t volatile crc5 = trampBuffer[15];
    //uint8_t volatile crc6 = trampBuffer[0];
    //uint8_t volatile crc7 = trampBuffer[1];
    //uint8_t volatile crc8 = trampBuffer[2];
    //uint8_t volatile crc9 = trampBuffer[3];
    //is crc valid
    //if (trampBuffer[14+1] == trampBuffer[14+1])
    //if (trampBuffer[14+1] == TrampChecksum(trampBuffer+1))
    if(1)
    {
        switch(trampBuffer[1+1])
        {
            case 'r':
                trampInfo.trampRfFreqMin = ( trampBuffer[2+1] |(trampBuffer[3+1] << 8) );
                if(trampInfo.trampRfFreqMin)
                {
                    trampInfo.trampRfFreqMax  = ( trampBuffer[4+1] |(trampBuffer[5+1] << 8) );
                    trampInfo.trampRfPowerMax = ( trampBuffer[6+1] |(trampBuffer[7+1] << 8) );
                    return(1);
                }
                break;
            case 's':
                trampInfo.trampTemp = (int16_t)( trampBuffer[6+1] |(trampBuffer[7+1] << 8) );
                return(1);
                break;
            case 'v':
                trampInfo.trampCurFreq = ( trampBuffer[2+1] |(trampBuffer[3+1] << 8) );
                if(trampInfo.trampCurFreq)
                {
                    trampInfo.trampCurPower = ( trampBuffer[4+1] |(trampBuffer[5+1] << 8) );
                    trampInfo.trampPitMode  = trampBuffer[7+1];
                    trampInfo.trampReqPower = ( trampBuffer[8+1] |(trampBuffer[9+1] << 8) );
                    //vtx58_Freq2Bandchan(trampCurFreq, &trampBand, &trampChannel);
                    //if(trampConfFreq == 0)  trampConfFreq  = trampCurFreq;
                    //if(trampConfPower == 0) trampConfPower = trampPower;
                    //return 'v';
                    return(1);
                }
                break;
        }
    }

    //bad crc or nonsensical data, return 0
    return(0);

}

int TrampGetSettings(void)
{

	if (boardArmed)
		return(0);

    //TrampResetInfoRecord();

	if (TrampSendCommand('s', 0, 1))
    {
        if (!TrampHandleResponse(trampIoBuffer))
            return(0);
    }
    else
    {
        return(0);
    }

    DelayMs(25);
	if (TrampSendCommand('r', 0, 1))
    {
        if (!TrampHandleResponse(trampIoBuffer))
            return(0);
    }
    else
    {
        return(0);
    }

    DelayMs(25);
	if (TrampSendCommand('v', 0, 1))
    {
        if (!TrampHandleResponse(trampIoBuffer))
            return(0);
    }
    else
    {
        return(0);
    }

    //all tramp querries were successful, fill vtx record here
    vtxRecord.vtxDevice      = VTX_DEVICE_TRAMP;
    vtxRecord.vtxTemp        = trampInfo.trampTemp;
    vtxRecord.vtxFrequency   = trampInfo.trampCurFreq;
    vtxRecord.vtxRegion      = VTX_REGION_US;
    vtxRecord.vtxBandChannel = VtxFrequencyToBandChannel(trampInfo.trampCurFreq);
    VtxChannelToBandAndChannel(vtxRecord.vtxBandChannel, &vtxRecord.vtxBand, &vtxRecord.vtxChannel);

    switch(trampInfo.trampReqPower)
    {
        case 25:
            vtxRecord.vtxPower = 0;
            break;
        case 100:
            vtxRecord.vtxPower = 1;
            break;
        case 200:
            vtxRecord.vtxPower = 2;
            break;
        case 400:
            vtxRecord.vtxPower = 3;
            break;
        case 600:
            vtxRecord.vtxPower = 4;
            break;
        default:
            vtxRecord.vtxPower = 0;
            break;
    }
/*
    switch(trampInfo.trampReqPower)
    {
        case 0:
            vtxRecord.vtxPower = VTX_POWER_025MW;
            break;
        case 1:
            vtxRecord.vtxPower = VTX_POWER_100MW;
            break;
        case 2:
            vtxRecord.vtxPower = VTX_POWER_200MW;
            break;
        case 3:
            vtxRecord.vtxPower = VTX_POWER_400MW;
            break;
        case 4:
            vtxRecord.vtxPower = VTX_POWER_600MW;
            break;
        default:
            vtxRecord.vtxPower = VTX_POWER_UN;
            break;
    }
*/

    if (trampInfo.trampPitMode)
    {
        vtxRecord.vtxPit = VTX_MODE_PIT;
    }
    else
    {
        vtxRecord.vtxPit = VTX_MODE_ACTIVE;
    }

    //only set first time
    if (vtxRequested.vtxDevice == VTX_DEVICE_NONE)
    {
        vtxRequested.vtxDevice      = vtxRecord.vtxDevice;
        vtxRequested.vtxBand        = vtxRecord.vtxBand;
        vtxRequested.vtxChannel     = vtxRecord.vtxChannel;
        vtxRequested.vtxBandChannel = vtxRecord.vtxBandChannel;
        vtxRequested.vtxPower       = vtxRecord.vtxPower;
        vtxRequested.vtxPit         = vtxRecord.vtxPit;
        vtxRequested.vtxRegion      = vtxRecord.vtxRegion;
        vtxRequested.vtxFrequency   = vtxRecord.vtxFrequency;
    }

	return(1);

/*
	//fill buffer
	rxBufferCount = 5;
	smartAudioTxRxBuffer[0] = SM_START_CODE1;
	smartAudioTxRxBuffer[1] = SM_START_CODE2;
	smartAudioTxRxBuffer[2] = ShiftSmartAudioCommand(SM_GET_SETTINGS);
	smartAudioTxRxBuffer[3] = 0x00;
	smartAudioTxRxBuffer[4] = SmCrc8(smartAudioTxRxBuffer, 4);
	SendSoftSerialBlocking(smartAudioTxRxBuffer, rxBufferCount, 50);
	DelayMs(2);
	rxBufferCount = 0;
	rxBufferCount = ReceiveSoftSerialBlocking(smartAudioTxRxBuffer, &rxBufferCount, 150);
	for (tries=4;tries>0;tries--)
	{
		if ( CheckSmartAudioRxCrc(smartAudioTxRxBuffer, rxBufferCount) )
		{
			if (smartAudioTxRxBuffer[2] == SM_VERSION_1)
			{
				vtxRecord.vtxDevice = VTX_DEVICE_SMARTV1;
			}
			else if (smartAudioTxRxBuffer[2] == SM_VERSION_2)
			{
				vtxRecord.vtxDevice = VTX_DEVICE_SMARTV2;
			}
			//set vtxBandChannel (0 through 39)
			vtxRecord.vtxBandChannel = smartAudioTxRxBuffer[4];
			//set vtxBand and vtxChannel (A,1 through R,8)
			VtxChannelToBandAndChannel(vtxRecord.vtxBandChannel, &vtxRecord.vtxBand, &vtxRecord.vtxChannel);
			//set vtxFreqency from band and channel
			vtxRecord.vtxFrequency = VtxBandChannelToFrequency(vtxRecord.vtxBandChannel);
			//set vtx power
			vtxRecord.vtxPower    = smartAudioTxRxBuffer[5];
			//no region info, assume US:
			vtxRecord.vtxRegion   = VTX_REGION_US;
			//VTX in pit mode or active?
			if ( BITMASK_CHECK(smartAudioTxRxBuffer[6], SM_OPMODE_PM) )
			{
				vtxRecord.vtxPit = VTX_MODE_ACTIVE;
			}
			else
			{
				vtxRecord.vtxPit = VTX_MODE_PIT;
			}
			//not used right now, set frequency dependant from bands
			//vtxRecord.vtxFrequency = ((smartAudioTxRxBuffer[8] << 8) | smartAudioTxRxBuffer[9]);
			//only set first time
			if (vtxRequested.vtxDevice == VTX_DEVICE_NONE)
			{
				vtxRequested.vtxDevice      = vtxRecord.vtxDevice;
				vtxRequested.vtxBand        = vtxRecord.vtxBand;
				vtxRequested.vtxChannel     = vtxRecord.vtxChannel;
				vtxRequested.vtxBandChannel = vtxRecord.vtxBandChannel;
				vtxRequested.vtxPower       = vtxRecord.vtxPower;
				vtxRequested.vtxPit         = vtxRecord.vtxPit;
				vtxRequested.vtxRegion      = vtxRecord.vtxRegion;
				vtxRequested.vtxFrequency   = vtxRecord.vtxFrequency;
			}
			return(1);
		}
	}
	return(0);
*/
}

void DeInitTrampTelemetry(void)
{

    int serialNumber; //set
 
    for (serialNumber = 0;serialNumber<MAX_USARTS;serialNumber++)
    {
        if ( (board.serials[serialNumber].enabled) && (mainConfig.telemConfig.telemTramp) )
        {
            if (board.serials[serialNumber].Protocol == USING_TRAMP)
            {
                UsartDeInit(serialNumber);
                return;
            }
        }
    }

}

static uint8_t TrampChecksum(uint8_t trampBuffer[])
{
    uint8_t checksum = 0;
    int32_t x; //set

    for (x=1;x<14;x++)
        checksum += trampBuffer[x];

    return(checksum);
}

static int TrampSendCommand(uint8_t cmd, uint16_t param, int waitForResponse)
{

    int x;            //set
    int serialNumber; //set
    int responseBack; //set

    bzero(trampIoBuffer, sizeof(trampIoBuffer));

    trampIoBuffer[0]  = 15;
    trampIoBuffer[1]  = cmd;
    trampIoBuffer[2]  = param & 0xff;
    trampIoBuffer[3]  = (param >> 8) & 0xff;
    trampIoBuffer[14] = TrampChecksum(trampIoBuffer);

    for (x=TRAMP_RETRIES;x>=0;x--)
    {
        for (serialNumber = 0;serialNumber<MAX_USARTS;serialNumber++)
        {
            if ( (board.serials[serialNumber].enabled) && (mainConfig.telemConfig.telemTramp) )
            {
                if (board.serials[serialNumber].Protocol == USING_TRAMP)
                {

                    responseBack = HAL_UART_Transmit(&uartHandles[board.serials[serialNumber].usartHandle], (uint8_t *)trampIoBuffer,TRAMP_BUFFER_SIZE, 45);
                    bzero(trampIoBuffer, sizeof(trampIoBuffer));

                    if (responseBack == HAL_OK)
                    {
                        if (!waitForResponse)
                        {
                            return(1);
                        }
                        else
                        {
                            DelayMs(5);
                            responseBack = HAL_UART_Receive(&uartHandles[board.serials[serialNumber].usartHandle], (uint8_t *)trampIoBuffer, TRAMP_BUFFER_SIZE, 200);
                        }
                    }

                    if (responseBack != HAL_OK)
                        return(0);
                    else
                        return(1);

                }
            }
        }
    }

	return(0);

}

static void TrampSetPitMode(int on)
{
    if (on)
        TrampSendCommand('I', 0, 0);
    else
        TrampSendCommand('I', 1, 0);
}

static void TrampSendFreq(int frequency)
{
    TrampSendCommand('F', (uint16_t)frequency, 0);
}

static void TrampSendRfPower(int power)
{
    TrampSendCommand('P', (uint16_t)power, 0);
}

int TrampSetPit(int pit)
{
    int x;
    for (x=TRAMP_RETRIES;x>=0;x--)
    {
        if (pit == VTX_MODE_PIT)
        {
            TrampSetPitMode(1);
        }
        else
        {
            TrampSetPitMode(0);
        }
        if(TrampGetSettings())
        {
            if (vtxRecord.vtxPit == pit)
                return(1);
        }
    }
    return(0);
}

int TrampSetBandChannel(int bandChannel)
{
    int x;
    for (x=TRAMP_RETRIES;x>=0;x--)
    {
        TrampSendFreq(VtxBandChannelToFrequency(bandChannel));
        if(TrampGetSettings())
        {
            if (vtxRecord.vtxBandChannel == bandChannel)
                return(1);
        }
    }
    return(0);
}

int TrampSetPower(int power)
{
    int x;           //set
    int powerNumber; //set

    switch(power)
    {
        case 0:
            powerNumber = 25;
            break;
        case 1:
            powerNumber = 100;
            break;
        case 2:
            powerNumber = 200;
            break;
        case 3:
            powerNumber = 400;
            break;
        case 4:
            powerNumber = 600;
            break;
        default:
            powerNumber = 25;
            break;
    }

    for (x=TRAMP_RETRIES;x>=0;x--)
    {
        TrampSendRfPower(powerNumber);
        if(TrampGetSettings())
        {
            if (vtxRecord.vtxPower == power)
                return(1);
        }
    }
    return(0);
}