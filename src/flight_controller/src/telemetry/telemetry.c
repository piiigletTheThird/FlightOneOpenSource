#include "includes.h"


volatile uint32_t sendMspAt = 0;
volatile uint32_t sendRfOsdAt = 0;
volatile uint32_t sendSmartPortAt = 0;
volatile uint32_t sendSmartPortLuaAt = 0;
volatile uint32_t sendCrsfTelemtryAt = 0;
volatile uint32_t sendSpektrumTelemtryAt = 0;

volatile uint32_t telemEnabled = 1;

volatile vtx_record vtxRequested;
volatile vtx_record vtxRecord;

static const int vtxBandChannelToFrequencyLookup[] =
{
	5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725, //Boscam A
	5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866, //Boscam B
	5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945, //Boscam E
	5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880, //FatShark
	5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917, //RaceBand
};

void ProcessTelemtry(void)
{
	static uint32_t mspCase = 0;

	if (!telemEnabled)
		return;
	//This function is run by the task manager quite often.
	//is telemetry bidirectional or unidirectional

	//if bidirectional, which serial do we listen to?
	//if line is idle, check the data in it's RX buffer
	//check if it's time and what we need to send

	//if unidirectional, check if it's safe to send
	//if it's time to send figure out what to send and send it.

	//SoftSerialReceiveBlocking(uint8_t inBuffer[], motor_type actuator, uint32_t timeoutMs, uint32_t baudRate, uint32_t bitLength, uint32_t inverted);
	if (mainConfig.telemConfig.telemMsp)
	{
			//send at 10 ms intervals
			if ( (sendMspAt) && ( InlineMillis() > sendMspAt ) )
			{
				//set msp to run in 10 ms
				sendMspAt = InlineMillis() + 10;
				//Send MSP
				//SendMspAttitude();
				switch (mspCase++)
				{
					case 0:
						SendMspBoxIds();
						break;
					case 1:
						SendMspStatus();
						break;
					case 2:
						SendMspAnalog();
						break;
					case 3:
						SendMspAttitude();
						mspCase = 0;
						break;
				}
			}
			else if (!sendMspAt) 
			{
				//if not set then we set msp to run in 10 ms.
				sendMspAt = InlineMillis() + 10;
			}
	}

	if (mainConfig.telemConfig.telemRfOsd)
	{
		if ( (sendRfOsdAt) && ( InlineMillis() > sendRfOsdAt ) )
		{
			//set msp to run in 10 ms
			sendRfOsdAt = InlineMillis() + 10;
			HandleRfOsd();
		}
		else if (!sendRfOsdAt) 
		{
			//if not set then we set msp to run in 10 ms.
			sendRfOsdAt = InlineMillis() + 10;
		}
	}

	if (mainConfig.telemConfig.telemSport)
	{
		if ( (sendSmartPortAt) && ( sendSmartPortAt >= InlineMillis() ) )
		{
			sendSmartPortAt = 0; //reset send time to 0 which disables it
			SendSmartPort();     //send the data. Blind of soft or hard s.port

		}
		else if ( (sendSmartPortLuaAt) && ( sendSmartPortLuaAt >= InlineMillis() ) )
		{
			sendSmartPortLuaAt = 0; //reset send time to 0 which disables it
			SendSmartPortLua();     //send the data. Blind of soft or hard s.port

		}
		else
		{
			CheckIfSportReadyToSend(); //sets sendSmartPortAt if it needs to.
		}
	}
	else if (mainConfig.telemConfig.telemSpek)
	{
		if ( (sendSpektrumTelemtryAt) && ( sendSpektrumTelemtryAt >= InlineMillis() ) )
		{
			sendSpektrumTelemtryAt = 0; //reset send time to 0 which disables it
			sendSpektrumTelem();     //send the data. Blind of soft or hard s.port
		}
	}
	else if (mainConfig.telemConfig.telemCrsf)
	{
		if ( (sendCrsfTelemtryAt) && ( sendCrsfTelemtryAt >= InlineMillis() ) )
		{
			sendCrsfTelemtryAt = 0; //reset send time to 0 which disables it
			SendCrsfTelem();     //send the data. Blind of soft or hard s.port
		}
	}
}

void InitMavlink(uint32_t serialPort)
{
	(void)(serialPort);
}

int VtxFrequencyToBandChannel(int frequency)
{
	//if frequency is not in table then -1 is returned which is unknown frequency
	int x;
	for (x=sizeof(vtxBandChannelToFrequencyLookup)-1;x>=0;x--)
	{
		if(vtxBandChannelToFrequencyLookup[x] == frequency)
			return(x);
	}
	return(-1);
}

int VtxBandChannelToFrequency(int bandChannel)
{
	return(vtxBandChannelToFrequencyLookup[bandChannel]);
}

void VtxChannelToBandAndChannel(int inChannel, volatile int *vtxBand, volatile int *channel)
{

	int bandMultiplier;

	bandMultiplier = (inChannel) / 8;

	(*vtxBand) = bandMultiplier;
	(*channel) = inChannel - (bandMultiplier * 8);

}

int VtxBandAndChannelToBandChannel(volatile int vtxBand, volatile int channel)
{

	return ( (channel + (8 * vtxBand)) );

}


int VtxTurnOn(void)
{
	int returnValue;

	static int mutex = 0;

	if (mutex)
		return(0);

	mutex = 1;

	switch(vtxRecord.vtxDevice)
	{
		case VTX_DEVICE_SMARTV1:
		case VTX_DEVICE_SMARTV2:
			InitSmartAudio();
			returnValue = SmartAudioVtxTurnOn();
			DeInitSmartAudio();
			mutex = 0;
			return( returnValue );
			break;
		case VTX_DEVICE_TRAMP:
			returnValue = TrampSetPit(VTX_MODE_ACTIVE);
			mutex = 0;
			return( returnValue );
			break;
		case VTX_DEVICE_NONE:
		default:
			mutex = 0;
			return(0);
			break;
	}

	mutex = 0;
	return(0);

}

int VtxTurnPit(void)
{
	uint32_t returnValue;
	static uint32_t mutex = 0;

	if (mutex)
		return(0);

	mutex = 1;

	switch(vtxRecord.vtxDevice)
	{
		case VTX_DEVICE_SMARTV1:
		case VTX_DEVICE_SMARTV2:
			InitSmartAudio();
			returnValue = SmartAudioVtxTurnPit();
			DeInitSmartAudio();
			mutex = 0;
			return( returnValue );
			break;
		case VTX_DEVICE_TRAMP:
			returnValue = TrampSetPit(VTX_MODE_PIT);
			mutex = 0;
			return( returnValue );
			break;
		case VTX_DEVICE_NONE:
		default:
			mutex = 0;
			return(0);
			break;
	}

	mutex = 0;

	return(0);

}

int VtxBandChannel(int bandChannel)
{
	uint32_t returnValue;
	static uint32_t mutex = 0;

	if (mutex)
		return(0);

	mutex = 1;

	switch(vtxRecord.vtxDevice)
	{
		case VTX_DEVICE_SMARTV1:
		case VTX_DEVICE_SMARTV2:
			InitSmartAudio();
			returnValue = SmartAudioVtxBandChannel(bandChannel);
			DeInitSmartAudio();
			mutex = 0;
			return( returnValue );
			break;
		case VTX_DEVICE_TRAMP:
			returnValue = TrampSetBandChannel(bandChannel);
			mutex = 0;
			return( returnValue );
			break;
		case VTX_DEVICE_NONE:
		default:
			mutex = 0;
			return(0);
			break;
	}

	mutex = 0;

	return(0);

}

int VtxPower(int power)
{
	int returnValue;
	static int mutex = 0;

	if (mutex)
		return(0);

	mutex = 1;

	switch(vtxRecord.vtxDevice)
	{
		case VTX_DEVICE_SMARTV1:
		case VTX_DEVICE_SMARTV2:
			InitSmartAudio();
			returnValue = SmartAudioVtxPower(power);
			DeInitSmartAudio();
			mutex = 0;
			return( returnValue );
			break;
		case VTX_DEVICE_TRAMP:
			returnValue = TrampSetPower(power);
			mutex = 0;
			return( returnValue );
			break;
		case VTX_DEVICE_NONE:
		default:
			mutex = 0;
			return(0);
			break;
	}

	mutex = 0;
	return(0);

}

void InitTelemtry(void)
{

	static int firstTimeInit = 1;
	vtxRecord.vtxDevice      = VTX_DEVICE_NONE;
	vtxRequested.vtxDevice   = VTX_DEVICE_NONE;

	//try twice to init smart audi if it's enabled
	InitSmartAudio();
    if(mainConfig.telemConfig.telemSmartAudio && !vtxRecord.vtxDevice)
    {
		if(firstTimeInit)
		{
    		DelayMs(1500);
    		InitSmartAudio();
		}
    }

	InitAllSport();

	switch(mainConfig.telemConfig.telemMsp)
	{
		//soft msp not supported right now
		case TELEM_ACTUATOR1:
		case TELEM_ACTUATOR2:
		case TELEM_ACTUATOR3:
		case TELEM_ACTUATOR4:
		case TELEM_ACTUATOR5:
		case TELEM_ACTUATOR6:
		case TELEM_ACTUATOR7:
		case TELEM_ACTUATOR8:
			break;
		case TELEM_USART1:
			InitMsp(ENUM_USART1);
			break;
		case TELEM_USART2:
			InitMsp(ENUM_USART2);
			break;
		case TELEM_USART3:
			InitMsp(ENUM_USART3);
			break;
		case TELEM_USART4:
			InitMsp(ENUM_USART4);
			break;
		case TELEM_USART5:
			InitMsp(ENUM_USART5);
			break;
		case TELEM_USART6:
			InitMsp(ENUM_USART6);
			break;
		default:
			break;
	}

	switch(mainConfig.telemConfig.telemRfOsd)
	{
		//soft msp not supported right now
		case TELEM_ACTUATOR1:
		case TELEM_ACTUATOR2:
		case TELEM_ACTUATOR3:
		case TELEM_ACTUATOR4:
		case TELEM_ACTUATOR5:
		case TELEM_ACTUATOR6:
		case TELEM_ACTUATOR7:
		case TELEM_ACTUATOR8:
			break;
		case TELEM_USART1:
			InitRfOsd(ENUM_USART1);
			break;
		case TELEM_USART2:
			InitRfOsd(ENUM_USART2);
			break;
		case TELEM_USART3:
			InitRfOsd(ENUM_USART3);
			break;
		case TELEM_USART4:
			InitRfOsd(ENUM_USART4);
			break;
		case TELEM_USART5:
			InitRfOsd(ENUM_USART5);
			break;
		case TELEM_USART6:
			InitRfOsd(ENUM_USART6);
			break;
		default:
			break;
	}

	switch(mainConfig.telemConfig.telemMav)
	{
		//soft msp not supported right now
		case TELEM_ACTUATOR1:
		case TELEM_ACTUATOR2:
		case TELEM_ACTUATOR3:
		case TELEM_ACTUATOR4:
		case TELEM_ACTUATOR5:
		case TELEM_ACTUATOR6:
		case TELEM_ACTUATOR7:
		case TELEM_ACTUATOR8:
			break;
		case TELEM_USART1:
			InitMavlink(ENUM_USART1);
			break;
		case TELEM_USART2:
			InitMavlink(ENUM_USART2);
			break;
		case TELEM_USART3:
			InitMavlink(ENUM_USART3);
			break;
		case TELEM_USART4:
			InitMavlink(ENUM_USART4);
			break;
		case TELEM_USART5:
			InitMavlink(ENUM_USART5);
			break;
		case TELEM_USART6:
			InitMavlink(ENUM_USART6);
			break;
		default:
			break;
	}

	if(mainConfig.telemConfig.telemTramp)
	{
		switch(mainConfig.telemConfig.telemTramp)
		{
			//soft trsmp not supported right now
			case TELEM_ACTUATOR1:
			case TELEM_ACTUATOR2:
			case TELEM_ACTUATOR3:
			case TELEM_ACTUATOR4:
			case TELEM_ACTUATOR5:
			case TELEM_ACTUATOR6:
			case TELEM_ACTUATOR7:
			case TELEM_ACTUATOR8:
				break;
			case TELEM_USART1:
				InitTrampTelemetry(ENUM_USART1);
				break;
			case TELEM_USART2:
				InitTrampTelemetry(ENUM_USART2);
				break;
			case TELEM_USART3:
				InitTrampTelemetry(ENUM_USART3);
				break;
			case TELEM_USART4:
				InitTrampTelemetry(ENUM_USART4);
				break;
			case TELEM_USART5:
				InitTrampTelemetry(ENUM_USART5);
				break;
			case TELEM_USART6:
				InitTrampTelemetry(ENUM_USART6);
				break;
			default:
				break;
		}
	}

	if(mainConfig.telemConfig.telemCrsf)
	{
		switch(mainConfig.telemConfig.telemCrsf)
		{
			//soft trsmp not supported right now
			case TELEM_ACTUATOR1:
			case TELEM_ACTUATOR2:
			case TELEM_ACTUATOR3:
			case TELEM_ACTUATOR4:
			case TELEM_ACTUATOR5:
			case TELEM_ACTUATOR6:
			case TELEM_ACTUATOR7:
			case TELEM_ACTUATOR8:
				break;
			case TELEM_USART1:
				InitCrsfTelemetry(ENUM_USART1);
				break;
			case TELEM_USART2:
				InitCrsfTelemetry(ENUM_USART2);
				break;
			case TELEM_USART3:
				InitCrsfTelemetry(ENUM_USART3);
				break;
			case TELEM_USART4:
				InitCrsfTelemetry(ENUM_USART4);
				break;
			case TELEM_USART5:
				InitCrsfTelemetry(ENUM_USART5);
				break;
			case TELEM_USART6:
				InitCrsfTelemetry(ENUM_USART6);
				break;
			default:
				break;
		}
	}

	if (mainConfig.telemConfig.telemSpek)
		InitSpektrumTelemetry();

	firstTimeInit = 0;
}


