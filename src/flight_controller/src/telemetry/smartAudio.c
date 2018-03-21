#include "includes.h"

#define SMART_AUDIO_BUFFER_SIZE 12

uint8_t  smartAudioTxRxBuffer[SMART_AUDIO_BUFFER_SIZE];
uint32_t smartAudioUsartSerialNumber = 999;


static uint8_t  ShiftSmartAudioCommand(uint8_t data);


uint32_t InitSmartAudio(void)
{

	uint32_t usartNumber   = 0;
	uint32_t usartPinType  = 0;
	uint32_t returnValue   = 0;
	uint32_t baudrate      = 0;
	int32_t  x             = 0;

	if (boardArmed)
		return(0);

#ifdef SPMFC400
	if (mainConfig.telemConfig.telemSmartAudio)
		mainConfig.telemConfig.telemSmartAudio = TELEM_SS1W_USART4T;
#endif
	switch(mainConfig.telemConfig.telemSmartAudio)
	{
		case TELEM_SS1W_USART1R:
			usartNumber  = ENUM_USART1;
			usartPinType = ENUM_USART_RX_PIN;
			break;
		case TELEM_SS1W_USART2R:
			usartNumber  = ENUM_USART2;
			usartPinType = ENUM_USART_RX_PIN;
			break;
		case TELEM_SS1W_USART3R:
			usartNumber  = ENUM_USART3;
			usartPinType = ENUM_USART_RX_PIN;
			break;
		case TELEM_SS1W_USART4R:
			usartNumber  = ENUM_USART4;
			usartPinType = ENUM_USART_RX_PIN;
			break;
		case TELEM_SS1W_USART5R:
			usartNumber  = ENUM_USART5;
			usartPinType = ENUM_USART_RX_PIN;
			break;
		case TELEM_SS1W_USART6R:
			usartNumber  = ENUM_USART6;
			usartPinType = ENUM_USART_RX_PIN;
			break;
		case TELEM_SS1W_USART1T:
			usartNumber  = ENUM_USART1;
			usartPinType = ENUM_USART_TX_PIN;
			break;
		case TELEM_SS1W_USART2T:
			usartNumber  = ENUM_USART2;
			usartPinType = ENUM_USART_TX_PIN;
			break;
		case TELEM_SS1W_USART3T:
			usartNumber  = ENUM_USART3;
			usartPinType = ENUM_USART_TX_PIN;
			break;
		case TELEM_SS1W_USART4T:
			usartNumber  = ENUM_USART4;
			usartPinType = ENUM_USART_TX_PIN;
			break;
		case TELEM_SS1W_USART5T:
			usartNumber  = ENUM_USART5;
			usartPinType = ENUM_USART_TX_PIN;
			break;
		case TELEM_SS1W_USART6T:
			usartNumber  = ENUM_USART6;
			usartPinType = ENUM_USART_TX_PIN;
			break;
		default:
			return(0);
			break;
	}

	if (mainConfig.telemConfig.telemSport)
		DeInitSoftSport();

    baudrate = 4900;

    for (x=3;x>=0;x--)
    {
        if (usartPinType == ENUM_USART_RX_PIN)
		    InitBlockingSoftSerialPort( baudrate, SERIAL_NORMAL, SERIAL_STOP_BITS_2_0, SERIAL_START_BIT_ON, board.serials[usartNumber].RXPort, board.serials[usartNumber].RXPin, SERIAL_LSB, TBS_HANDLING_ON );
	    else
		    InitBlockingSoftSerialPort( baudrate, SERIAL_NORMAL, SERIAL_STOP_BITS_2_0, SERIAL_START_BIT_ON, board.serials[usartNumber].TXPort, board.serials[usartNumber].TXPin, SERIAL_LSB, TBS_HANDLING_ON );

        returnValue = SmartAudioGetSettings();

		DelayMs(50);

		if (returnValue)
            return( returnValue );
		
		baudrate -=50;

    }

    return( returnValue );

}

void DeInitSmartAudio(void)
{
	DeInitBlockingSoftSerialPort();
	if (mainConfig.telemConfig.telemSport)
		InitAllSport();
}

uint32_t CheckSmartAudioRxCrc(uint8_t buffer[], uint32_t bufferSize)
{
	uint32_t returnLength;
	uint32_t crcReturned;

	if(bufferSize)
	{
		if (bufferSize > 3)
			returnLength = buffer[3];
		else
			return(0);

		if (bufferSize > (3+returnLength))
			crcReturned = buffer[3+returnLength];
		else
			return(0);

	}
	else
	{
		return(0);
	}

	if ( SmCrc8(&buffer[2], 1+returnLength) == crcReturned )
		return(1);
	else
		return(0);
}

uint32_t SmartAudioVtxTurnOn(void)
{

	uint32_t rxBufferCount;
	uint32_t tries;

	//add retry
	if (!vtxRecord.vtxDevice)
	{
		if (!SmartAudioGetSettings())
			return(0);
	}

	//fill buffer
	smartAudioTxRxBuffer[0] = SM_START_CODE1;
	smartAudioTxRxBuffer[1] = SM_START_CODE2;
	smartAudioTxRxBuffer[2] = ShiftSmartAudioCommand(SM_SET_OPERATION_MODE);
	smartAudioTxRxBuffer[3] = 0x01;

	if (mainConfig.telemConfig.vtxPitmodeType == 0)
		smartAudioTxRxBuffer[4] = (uint8_t)SM_SET_OPMODE_PM;
	else if (mainConfig.telemConfig.vtxPitmodeType == 1)
		smartAudioTxRxBuffer[4] = (uint8_t)SM_SET_OPMODE_DIS_PMOR;

	smartAudioTxRxBuffer[5] = SmCrc8(smartAudioTxRxBuffer, 5);

	//warning blocking code
	for (tries=3;tries>0;tries--)
	{
		SendSoftSerialBlocking(smartAudioTxRxBuffer, 6, 50);
		DelayMs(2); //important
		rxBufferCount = 0;
		rxBufferCount = ReceiveSoftSerialBlocking(smartAudioTxRxBuffer, &rxBufferCount, 150);

		if ( CheckSmartAudioRxCrc(smartAudioTxRxBuffer, rxBufferCount) )
		{
			vtxRecord.vtxPit = VTX_MODE_ACTIVE;
			return(1);
		}
	}

	return(0);
}

uint32_t SmartAudioVtxTurnPit(void)
{

	uint32_t rxBufferCount;
	uint32_t tries;

	if (!vtxRecord.vtxDevice)
	{
		if (!SmartAudioGetSettings())
			return(0);
	}

	//fill buffer
	smartAudioTxRxBuffer[0] = SM_START_CODE1;
	smartAudioTxRxBuffer[1] = SM_START_CODE2;
	smartAudioTxRxBuffer[2] = ShiftSmartAudioCommand(SM_SET_OPERATION_MODE);
	smartAudioTxRxBuffer[3] = 0x01;
	smartAudioTxRxBuffer[4] = (uint8_t)SM_SET_OPMODE_DIS_PMOR;
	smartAudioTxRxBuffer[5] = SmCrc8(smartAudioTxRxBuffer, 5);

	//warning blocking code
	for (tries=3;tries>0;tries--)
	{
		SendSoftSerialBlocking(smartAudioTxRxBuffer, 6, 50);
		DelayMs(2); //important
		rxBufferCount = 0;
		rxBufferCount = ReceiveSoftSerialBlocking(smartAudioTxRxBuffer, &rxBufferCount, 150);

		if ( CheckSmartAudioRxCrc(smartAudioTxRxBuffer, rxBufferCount) )
		{
			vtxRecord.vtxPit = VTX_MODE_ACTIVE;
			return(1);
		}
	}

	return(0);
}

static uint8_t ShiftSmartAudioCommand(uint8_t data)
{
	return( ((data) << 1) | 1 );
}


uint32_t SmartAudioGetSettings(void)
{

	uint32_t rxBufferCount;
	uint32_t tries;

	if (boardArmed)
		return(0);


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
			vtxRecord.vtxPower     = smartAudioTxRxBuffer[5];
			if(vtxRecord.vtxDevice == VTX_DEVICE_SMARTV1)
			{
				switch(smartAudioTxRxBuffer[5])
				{
					case 7:
						vtxRecord.vtxPower = 0;
						break;
					case 16:
						vtxRecord.vtxPower = 1;
						break;
					case 25:
						vtxRecord.vtxPower = 2;
						break;
					case 40:
						vtxRecord.vtxPower = 3;
						break;
					default:
						vtxRecord.vtxPower = 0;
						break;
				}
			}
			/*
			switch(vtxRecord.vtxDevice)
			{
				case VTX_DEVICE_SMARTV1:
					switch(smartAudioTxRxBuffer[5])
					{
						case 7:
							vtxRecord.vtxPower = VTX_POWER_025MW;
							break;
						case 16:
							vtxRecord.vtxPower = VTX_POWER_200MW;
							break;
						case 25:
							vtxRecord.vtxPower = VTX_POWER_500MW;
							break;
						case 40:
							vtxRecord.vtxPower = VTX_POWER_800MW;
							break;
						default:
							vtxRecord.vtxPower = VTX_POWER_UN;
							break;
					}
					break;
				case VTX_DEVICE_SMARTV2:
					switch(smartAudioTxRxBuffer[5])
					{
						case 0:
							vtxRecord.vtxPower = VTX_POWER_025MW;
							break;
						case 1:
							vtxRecord.vtxPower = VTX_POWER_200MW;
							break;
						case 2:
							vtxRecord.vtxPower = VTX_POWER_500MW;
							break;
						case 3:
							vtxRecord.vtxPower = VTX_POWER_800MW;
							break;
						default:
							vtxRecord.vtxPower = VTX_POWER_UN;
							break;
					}
					break;
			}
			*/

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
}

uint32_t SmartAudioVtxPower(uint32_t powerLevel)
{
	uint32_t rxBufferCount;
	uint32_t powerNumber;
	uint32_t tries;

	if (!vtxRecord.vtxDevice)
	{
		if (!SmartAudioGetSettings())
			return(0);
	}

	powerNumber = 99;
	/*
	switch(vtxRecord.vtxDevice)
	{
		case VTX_DEVICE_SMARTV1:
			switch(powerLevel)
			{
				case VTX_POWER_025MW:
				case VTX_POWER_050MW:
				case VTX_POWER_100MW:
					powerNumber = 7;
					break;
				case VTX_POWER_200MW:
				case VTX_POWER_400MW:
					powerNumber = 16;
					break;
				case VTX_POWER_500MW:
					powerNumber = 25;
					break;
				case VTX_POWER_600MW:
				case VTX_POWER_800MW:
					powerNumber = 40;
					break;
				default:
            		powerNumber = 7;
            		break;
			}
			break;
		case VTX_DEVICE_SMARTV2:
			switch(powerLevel)
			{
				case VTX_POWER_025MW:
				case VTX_POWER_050MW:
				case VTX_POWER_100MW:
					powerNumber = 0;
					break;
				case VTX_POWER_200MW:
				case VTX_POWER_400MW:
					powerNumber = 1;
					break;
				case VTX_POWER_500MW:
					powerNumber = 2;
					break;
				case VTX_POWER_600MW:
				case VTX_POWER_800MW:
					powerNumber = 3;
					break;
				default:
           			powerNumber = 0;
            		break;
			}
			break;
	}
	*/

	switch(vtxRecord.vtxDevice)
	{
		case VTX_DEVICE_SMARTV1:
			switch(powerLevel)
			{
				case 0:
					powerNumber = 7;
					break;
				case 1:
					powerNumber = 16;
					break;
				case 2:
					powerNumber = 25;
					break;
				case 3:
					powerNumber = 40;
					break;
				default:
            		powerNumber = 7;
            		break;
			}
			break;
		case VTX_DEVICE_SMARTV2:
			switch(powerLevel)
			{
				case 0:
					powerNumber = 0;
					break;
				case 1:
					powerNumber = 1;
					break;
				case 2:
					powerNumber = 2;
					break;
				case 3:
					powerNumber = 3;
					break;
				default:
           			powerNumber = 0;
            		break;
			}
			break;
	}

	if (powerNumber == 99)
	{
		return(0);
	}

	//fill buffer
	smartAudioTxRxBuffer[0] = SM_START_CODE1;
	smartAudioTxRxBuffer[1] = SM_START_CODE2;
	smartAudioTxRxBuffer[2] = ShiftSmartAudioCommand(SM_SET_POWER);
	smartAudioTxRxBuffer[3] = 0x01;
	smartAudioTxRxBuffer[4] = (uint8_t)powerNumber;
	smartAudioTxRxBuffer[5] = SmCrc8(smartAudioTxRxBuffer, 5);

	//warning blocking code
	for (tries=4;tries>0;tries--)
	{
		SendSoftSerialBlocking(smartAudioTxRxBuffer, 6, 50);
		DelayMs(2); //important
		rxBufferCount = 0;
		rxBufferCount = ReceiveSoftSerialBlocking(smartAudioTxRxBuffer, &rxBufferCount, 150);

		if ( CheckSmartAudioRxCrc(smartAudioTxRxBuffer, rxBufferCount) )
		{
			vtxRecord.vtxPower = powerLevel;
			return(1);
		}
	}
	return(0);

}

uint32_t SmartAudioVtxBandChannel(uint32_t bandChannel)
{

	uint32_t rxBufferCount;
	uint32_t tries;

	if (!vtxRecord.vtxDevice)
	{
		if (!SmartAudioGetSettings())
			return(0);
	}

	//fill buffer
	smartAudioTxRxBuffer[0] = SM_START_CODE1;
	smartAudioTxRxBuffer[1] = SM_START_CODE2;
	smartAudioTxRxBuffer[2] = ShiftSmartAudioCommand(SM_SET_CHANNEL);
	smartAudioTxRxBuffer[3] = 0x01;
	smartAudioTxRxBuffer[4] = (uint8_t)bandChannel;
	smartAudioTxRxBuffer[5] = SmCrc8(smartAudioTxRxBuffer, 5);

	//warning blocking code
	for (tries=4;tries>0;tries--)
	{
		SendSoftSerialBlocking(smartAudioTxRxBuffer, 6, 50);
		DelayMs(2); //important
		rxBufferCount = 0;
		rxBufferCount = ReceiveSoftSerialBlocking(smartAudioTxRxBuffer, &rxBufferCount, 150);

		if ( CheckSmartAudioRxCrc(smartAudioTxRxBuffer, rxBufferCount) )
		{
			vtxRecord.vtxBandChannel = bandChannel;
			VtxChannelToBandAndChannel(vtxRecord.vtxBandChannel, &vtxRecord.vtxBand, &vtxRecord.vtxChannel);
			vtxRecord.vtxFrequency = VtxBandChannelToFrequency(vtxRecord.vtxBandChannel);
			return(1);
		}
	}

	return(0);

}

/*
	//use manual protocol to setup s.port.
	board.serials[usartNumber].enabled    = 1;
	board.serials[usartNumber].Protocol   = USING_SMARTAUDIO;

	board.serials[usartNumber].BaudRate   = 4900;//should be 4800bps�1�Start�bit�and�2�Stop�bit, but the VTX drifts a lot
	board.serials[usartNumber].WordLength = UART_WORDLENGTH_8B;
	board.serials[usartNumber].StopBits   = UART_STOPBITS_2;
	board.serials[usartNumber].Parity     = UART_PARITY_NONE;
	board.serials[usartNumber].HwFlowCtl  = UART_HWCONTROL_NONE;
	board.serials[usartNumber].Mode       = UART_MODE_TX_RX;

	board.serials[usartNumber].serialTxInverted = 0;
	board.serials[usartNumber].serialRxInverted = 0;
	board.serials[usartNumber].FrameSize = 4;
	board.serials[usartNumber].Pull = GPIO_PULLDOWN;

	board.dmasSerial[board.serials[usartNumber].TXDma].enabled  = 0;
	board.dmasSerial[board.serials[usartNumber].RXDma].enabled  = 0;

	board.serials[usartNumber].RXPin  = board.serials[usartNumber].TXPin;
	board.serials[usartNumber].RXPort = board.serials[usartNumber].TXPort;

	UsartDeInit(usartNumber); //deinits serial and associated pins and DMAs
	UsartInit(usartNumber);   //inits serial and associated pins and DMAs
*/
