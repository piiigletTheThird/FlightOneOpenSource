#include "includes.h"

uint32_t activeModes;
uint32_t setModes;

//also modify the enumeration in modes.h
string_modes_rec stringModes[] =
{
	{"ARMED",       0,  M_ARMED },
	{"LEVEL",       1,  M_ATTITUDE },
	{"ACROLEVEL",   2,  M_HORIZON },
	{"FAILSAFE",    3,  M_FAILSAFE },
	{"LOGGING",     4,  M_LOGGING },
	{"BUZZER",      5,  M_BUZZER },
	{"LEDMODE",     6,  M_LEDMODE },
	{"LEDCOLOR",    7,  M_LEDCOLOR },
	{"DIRECT",      8,  M_DIRECT },
	{"VTXON",       9,  M_VTXON },
	{"BRAINDRAIN",  10, M_BRAINDRAIN },
	{"PROFILE2",    11, M_PROFILE2 },
	{"PROFILE3",    12, M_PROFILE3 },
	{"QUOPA",       13, M_QUOPA },
	{"LEARN",       14, M_LEARN },
	{"BEEP",        15, M_BEEP },
	{"TURKEY",      16, M_TURKEY },
	//		{"CATMODE",    11,  M_CATMODE },
};



void InitModes(void)
{
	setModes = 0;
	activeModes = 0;
}

inline void EnableMode(uint32_t modeMask)
{
	activeModes |= (modeMask);
}

inline void DisableMode(uint32_t modeMask)
{
	activeModes &= ~(modeMask);
}

inline uint32_t ModeActive(uint32_t modeMask)
{
	return (activeModes & modeMask);
}

inline uint32_t ModeSet(uint32_t modeMask)
{
	return (setModes & modeMask);
}

void CheckRxToModes(void)
{
	uint32_t mode = 1;
	uint16_t x;
	volatile uint16_t channel;
	float rcMin;
	float rcMax;

	for (x=0;x<FLIGHT_MODE_ARRAY_SIZE;x=x+3)
	{

		channel = (uint16_t)mainConfig.flightModeArray[x];
		rcMin   = (float)mainConfig.flightModeArray[x+1] * 0.01;
		rcMax   = (float)mainConfig.flightModeArray[x+2] * 0.01;
		if ((channel > 3) && (channel < MAXCHANNELS)) //first four channels are not to be used for flight modes
		{
			setModes |= (mode); //this mode is currently assigned.
			if ( (trueRcCommandF[channel] >= rcMin) && (trueRcCommandF[channel] <= rcMax) )
			{
				EnableMode(mode);
			}
			else
			{
				DisableMode(mode);
			}
		}
		else
		{
			setModes &= ~(mode); //this mode is not currently assigned.
		}
		mode *= 2;
	}

}

void PrintModes(void)
{
	uint32_t x;
	uint32_t channel;


	snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE-1, "#me modes active: %lu\n", activeModes );
	RfCustomReplyBuffer(rf_custom_out_buffer);
	for (x=0;x<(sizeof(stringModes)/sizeof(string_modes_rec));x++)
	{
		//display corrected output using the channel variable
		channel = mainConfig.flightModeArray[x*3+0];

		if (channel < 4)
			channel = 0;
		else
			channel += 1;

		snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE-1, "modes %s=%lu=%i=%i\n", stringModes[x].modeString, channel, mainConfig.flightModeArray[x*3+1], mainConfig.flightModeArray[x*3+2] );
		RfCustomReplyBuffer(rf_custom_out_buffer);
		//snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "modes %s=%d=%d=%d", stringModes[x].modeString, channel, mainConfig.flightModeArray[x*3+1], mainConfig.flightModeArray[x*3+2] );
		//RfCustomReply(rf_custom_out_buffer);
	}

}

/*
void SplitString(char *inString, char *inString2, char token)
{
	uint32_t x;
	uint32_t stringLength;

	stringLength = strlen(inString);

	for (x = 0; x < stringLength; x++)
	{
		if (inString[x] == token)
			break;
	}

	if ( stringLength > x )
	{
		inString2 = inString + x + 1; //put everything after the token into inString2
	}

	inString[x] = 0; //set modstrging to modeSting
}
*/

void SetMode(uint32_t modeMask, uint16_t channel, int16_t minRc, int16_t maxRc)
{

	uint32_t x;

	for (x=0;x<(sizeof(stringModes)/sizeof(string_modes_rec));x++)
	{
		if ( stringModes[x].modeMask == modeMask)
		{
			mainConfig.flightModeArray[x*3+0] = channel;
			mainConfig.flightModeArray[x*3+1] = minRc;
			mainConfig.flightModeArray[x*3+2] = maxRc;
			bzero(rf_custom_out_buffer,RF_BUFFER_SIZE);

			//display corrected output using the channel variable
			if (channel < 4)
				channel = 0;
			else
				channel += 1;

			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "%s set to channel %d and range %d to %d\n", stringModes[x].modeString, channel, mainConfig.flightModeArray[x*3+1], mainConfig.flightModeArray[x*3+2] );
			RfCustomReplyBuffer(rf_custom_out_buffer);
			resetBoard = 1;
		}
	}
}

void SetupModes(char *modString)
{

	uint32_t x;
	char *channelString = NULL;
	char *minRcString   = NULL;
	char *maxRcString   = NULL;
	int16_t channel;
	int16_t minRc;
	int16_t maxRc;
	uint32_t stringLength;

	if (!strcmp("list", modString))
	{
		PrintModes();
	}
	else
	{
		//look for "MODE=CHANEL=MINRX=MAXRC"
		StripSpaces(modString);

		stringLength = strlen(modString);

		for (x = 0; x < stringLength; x++)
		{
			if (modString[x] == '=')
				break;
		}

		if ( stringLength > x )
		{
			channelString = modString + x + 1; //put everything after the token into inString2
		}

		modString[x] = 0; //set modstrging to modeSting

		stringLength = strlen(channelString);

		for (x = 0; x < stringLength; x++)
		{
			if (channelString[x] == '=')
				break;
		}

		if ( stringLength > x )
		{
			minRcString = channelString + x + 1; //put everything after the token into inString2
		}

		channelString[x] = 0; //set modstrging to modeSting




		stringLength = strlen(minRcString);

		for (x = 0; x < stringLength; x++)
		{
			if (minRcString[x] == '=')
				break;
		}

		if ( stringLength > x )
		{
			maxRcString = minRcString + x + 1; //put everything after the token into inString2
		}

		minRcString[x] = 0; //set modstring to modeSting

		//SplitString(modeString,    channelString, '=');
		//SplitString(channelString, minRcString,   '=');
		//SplitString(minRcString,   maxRcString,   '=');

		channel = atoi(channelString) - 1;
		minRc   = atoi(minRcString);
		maxRc   = atoi(maxRcString);

		//add logic since people can set this manually.
		if (minRc < -100)
			minRc = -100;
		if (minRc > 100)
			minRc = 100;

		if (maxRc < -100)
			maxRc = -100;
		if (maxRc > 100)
			maxRc = 100;

		if (minRc > maxRc)
			minRc = maxRc;

		if (maxRc < minRc)
			maxRc = minRc;
		//
		if (channel < 4)
			channel = 0;

		if (channel >= MAXCHANNELS)
			channel = 0;

		for (x=0;x<(sizeof(stringModes)/sizeof(string_modes_rec));x++)
		{
			if (!strcmp(stringModes[x].modeString, modString))
			{
				mainConfig.flightModeArray[x*3+0] = channel;
				mainConfig.flightModeArray[x*3+1] = minRc;
				mainConfig.flightModeArray[x*3+2] = maxRc;

				//display corrected output using the channel variable
				if (channel < 4)
					channel = 0;
				else
					channel += 1;

				bzero(rf_custom_out_buffer,RF_BUFFER_SIZE);
				snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "%s set to channel %d and range %d to %d\n", stringModes[x].modeString, channel, mainConfig.flightModeArray[x*3+1], mainConfig.flightModeArray[x*3+2] );
				RfCustomReplyBuffer(rf_custom_out_buffer);
				resetBoard = 1;
			}
		}
	}
}
