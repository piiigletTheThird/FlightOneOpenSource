#include "includes.h"

const actuator_mixer CONST_MIXER_X1234[MAX_MOTOR_NUMBER] =  {
	//yaw, roll, pitch, throttle, aux1, aux2, aux3, aux4
	{-1.0f,  1.0f,  1.0f,  1.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 1
	{ 1.0f, -1.0f,  1.0f,  1.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 2
	{-1.0f, -1.0f, -1.0f,  1.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 3
	{ 1.0f,  1.0f, -1.0f,  1.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 4
	{ 0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 5
	{ 0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 6
	{ 0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 7
	{ 0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 8
};

const actuator_mixer CONST_MIXER_X1234RY[MAX_MOTOR_NUMBER] =  {
	//yaw, roll, pitch, throttle, aux1, aux2, aux3, aux4
	{ 1.0f,  1.0f,  1.0f,  1.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 1
	{-1.0f, -1.0f,  1.0f,  1.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 2
	{ 1.0f, -1.0f, -1.0f,  1.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 3
	{-1.0f,  1.0f, -1.0f,  1.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 4
	{ 0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 5
	{ 0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 6
	{ 0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 7
	{ 0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 8
};

const actuator_mixer CONST_MIXER_X1234RYT[MAX_MOTOR_NUMBER] =  {
	//yaw, roll, pitch, throttle, aux1, aux2, aux3, aux4
	{-1.0f, -1.0f, -1.0f,  1.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 1
	{ 1.0f,  1.0f, -1.0f,  1.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 2
	{-1.0f,  1.0f,  1.0f,  1.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 3
	{ 1.0f, -1.0f,  1.0f,  1.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 4
	{ 0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 5
	{ 0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 6
	{ 0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 7
	{ 0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 8
};

const actuator_mixer CONST_MIXER_X4213[MAX_MOTOR_NUMBER] =  {
	//yaw, roll, pitch, throttle, aux1, aux2, aux3, aux4
	{-1.0f, -1.0f, -1.0f,  1.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 3
	{ 1.0f, -1.0f,  1.0f,  1.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 2
	{ 1.0f,  1.0f, -1.0f,  1.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 4
	{-1.0f,  1.0f,  1.0f,  1.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 1
	{ 0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 5
	{ 0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 6
	{ 0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 7
	{ 0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 8
};

const actuator_mixer CONST_MIXER_X4213RY[MAX_MOTOR_NUMBER] =  {
	//yaw, roll, pitch, throttle, aux1, aux2, aux3, aux4
	{ 1.0f, -1.0f, -1.0f,  1.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 3
	{-1.0f, -1.0f,  1.0f,  1.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 2
	{-1.0f,  1.0f, -1.0f,  1.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 4
	{ 1.0f,  1.0f,  1.0f,  1.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 1
	{ 0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 5
	{ 0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 6
	{ 0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 7
	{ 0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 8
};

const actuator_mixer CONST_MIXER_PLUS1234[MAX_MOTOR_NUMBER] =  {
	//yaw, roll, pitch, throttle, aux1, aux2, aux3, aux4
	{ 1.0f,  0.0f,  1.0f,  1.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 1 counter clockwise
	{-1.0f, -1.0f,  0.0f,  1.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 2 clockwise
	{ 1.0f,  0.0f, -1.0f,  1.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 3 counter clockwise
	{-1.0f,  1.0f,  0.0f,  1.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 4 clockwise
	{ 0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 5
	{ 0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 6
	{ 0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 7
	{ 0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 8
};

const actuator_mixer CONST_MIXER_PLUS1234RY[MAX_MOTOR_NUMBER] =  {
	//yaw, roll, pitch, throttle, aux1, aux2, aux3, aux4
	{-1.0f,  0.0f,  1.0f,  1.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 1
	{ 1.0f, -1.0f,  0.0f,  1.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 2
	{-1.0f,  0.0f, -1.0f,  1.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 3
	{ 1.0f,  1.0f,  0.0f,  1.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 4
	{ 0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 5
	{ 0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 6
	{ 0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 7
	{ 0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 8
};

actuator_mixer motorMixer[MAX_MOTOR_NUMBER] =  {
	//yaw, roll, pitch, throttle, aux1, aux2, aux3, aux4
	{-1.0f,  1.0f,  1.0f,  1.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 1
	{ 1.0f, -1.0f,  1.0f,  1.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 2
	{-1.0f, -1.0f, -1.0f,  1.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 3
	{ 1.0f,  1.0f, -1.0f,  1.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 4
	{ 0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 5
	{ 0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 6
	{ 0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 7
	{ 0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 8
};

actuator_mixer motorMixerT[MAX_MOTOR_NUMBER] =  {
	//yaw, roll, pitch, throttle, aux1, aux2, aux3, aux4
	{-1.0f,  1.0f,  1.0f,  1.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 1
	{ 1.0f, -1.0f,  1.0f,  1.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 2
	{-1.0f, -1.0f, -1.0f,  1.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 3
	{ 1.0f,  1.0f, -1.0f,  1.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 4
	{ 0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 5
	{ 0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 6
	{ 0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 7
	{ 0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //motor 8
};

actuator_mixer servoMixer[MAX_SERVO_NUMBER] =  {
	//yaw, roll, pitch, throttle, aux1, aux2, aux3, aux4
	{ 0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //servo 1
	{ 0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //servo 2
	{ 0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //servo 3
	{ 0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //servo 4
	{ 0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //servo 5
	{ 0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //servo 6
	{ 0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //servo 7
	{ 0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f}, //servo 8
};

int   threeDeeMode;
float throttleLookup[1024];
float throttleLookupKp[1024];
float throttleLookupKi[1024];
float throttleLookupKd[1024];
float stabilizerAttenuation;
int   activeMotorCounter = -1; //number of active motors minus 1

volatile float motorOutput[MAX_MOTOR_NUMBER];
volatile float servoOutput[MAX_SERVO_NUMBER];



//static float kiAttenuationCurve[ATTENUATION_CURVE_SIZE] = {1.40, 1.35, 1.28, 1.20, 1.15, 1.10, 1.05, 1.00, 0.95};
//static float kpAttenuationCurve[ATTENUATION_CURVE_SIZE] = {1.35, 1.15, 1.00, 1.00, 0.90, 0.90, 0.85, 0.80, 0.75};
//static float kdAttenuationCurve[ATTENUATION_CURVE_SIZE] = {1.35, 1.15, 1.00, 1.00, 0.90, 0.90, 0.85, 0.80, 0.75};

static float throttleCurve[ATTENUATION_CURVE_SIZE]      = {0.000f, 0.125f, 0.250f, 0.375f, 0.500f, 0.625f, 0.750f, 0.875f, 1.000f};
static float kpAttenuationCurve[ATTENUATION_CURVE_SIZE] = {1.05f, 1.05f, 1.00f, 1.00f, 1.00f, 1.00f, 1.00f, 1.05f, 1.05f};
static float kiAttenuationCurve[ATTENUATION_CURVE_SIZE] = {1.05f, 1.00f, 0.95f, 0.95f, 0.95f, 0.95f, 0.95f, 1.00f, 1.05f};
static float kdAttenuationCurve[ATTENUATION_CURVE_SIZE] = {1.10f, 1.05f, 1.00f, 0.95f, 0.95f, 0.95f, 0.95f, 1.10f, 1.10f};

static void  PrintCurve(char *outText, float curve[]);

static void  BuildThrottleLookupTable(void);
static void  BuildThrottleLookupTableKp(void);
static void  BuildThrottleLookupTableKi(void);
static void  BuildThrottleLookupTableKd(void);

static float ApplyAttenuationCurve (float input, float curve[], uint32_t curveSize);
static float ApplyAttenuationOldCurve (float input, float curve[], uint32_t curveSize);

static void BuildThrottleLookupTable(void)
{
	int x;
	for (x=1023;x>=0;x--)
	{
		//range throttle 0 through 1023
		throttleLookup[x] = ApplyAttenuationCurve( ((float)x / 1023.0f), mainConfig.tuneProfile[activeProfile].filterConfig[0].throttleCurve, ATTENUATION_CURVE_SIZE );
	}
}

static void BuildThrottleLookupTableKp(void)
{
	int x;
	for (x=1023;x>=0;x--)
	{
		//range throttle 0 through 1023
		if (mainConfig.tuneProfile[activeProfile].filterConfig[0].tpaKpCurveType)
		{
			throttleLookupKp[x] = ApplyAttenuationOldCurve( ((float)x / 1023.0f), mainConfig.tuneProfile[activeProfile].filterConfig[0].tpaKpCurve, ATTENUATION_CURVE_SIZE );
		}
		else
		{
			throttleLookupKp[x] = ApplyAttenuationCurve( ((float)x / 1023.0f), mainConfig.tuneProfile[activeProfile].filterConfig[0].tpaKpCurve, ATTENUATION_CURVE_SIZE );
		}
	}
}

static void BuildThrottleLookupTableKi(void)
{
	int x;
	for (x=1023;x>=0;x--)
	{
		//range throttle 0 through 1023
		if (mainConfig.tuneProfile[activeProfile].filterConfig[0].tpaKiCurveType)
		{
			throttleLookupKi[x] = ApplyAttenuationOldCurve( ((float)x / 1023.0f), mainConfig.tuneProfile[activeProfile].filterConfig[0].tpaKiCurve, ATTENUATION_CURVE_SIZE );
		}
		else
		{
			throttleLookupKi[x] = ApplyAttenuationCurve( ((float)x / 1023.0f), mainConfig.tuneProfile[activeProfile].filterConfig[0].tpaKiCurve, ATTENUATION_CURVE_SIZE );
		}
	}
}

static void BuildThrottleLookupTableKd(void)
{
	int x;
	for (x=1023;x>=0;x--)
	{
		//range throttle 0 through 1023
		if (mainConfig.tuneProfile[activeProfile].filterConfig[0].tpaKdCurveType)
		{
			throttleLookupKd[x] = ApplyAttenuationOldCurve( ((float)x / 1023.0f), mainConfig.tuneProfile[activeProfile].filterConfig[0].tpaKdCurve, ATTENUATION_CURVE_SIZE );
		}
		else
		{
			throttleLookupKd[x] = ApplyAttenuationCurve( ((float)x / 1023.0f), mainConfig.tuneProfile[activeProfile].filterConfig[0].tpaKdCurve, ATTENUATION_CURVE_SIZE );
		}
	}
}

void ResetTpaCurves(void)
{
	memcpy(mainConfig.tuneProfile[0].filterConfig[0].throttleCurve, throttleCurve,      sizeof(throttleCurve));
	memcpy(mainConfig.tuneProfile[0].filterConfig[0].tpaKpCurve,    kpAttenuationCurve, sizeof(kpAttenuationCurve));
	memcpy(mainConfig.tuneProfile[0].filterConfig[0].tpaKiCurve,    kiAttenuationCurve, sizeof(kiAttenuationCurve));
	memcpy(mainConfig.tuneProfile[0].filterConfig[0].tpaKdCurve,    kdAttenuationCurve, sizeof(kdAttenuationCurve));

	memcpy(mainConfig.tuneProfile[1].filterConfig[0].throttleCurve, throttleCurve,      sizeof(throttleCurve));
	memcpy(mainConfig.tuneProfile[1].filterConfig[0].tpaKpCurve,    kpAttenuationCurve, sizeof(kpAttenuationCurve));
	memcpy(mainConfig.tuneProfile[1].filterConfig[0].tpaKiCurve,    kiAttenuationCurve, sizeof(kiAttenuationCurve));
	memcpy(mainConfig.tuneProfile[1].filterConfig[0].tpaKdCurve,    kdAttenuationCurve, sizeof(kdAttenuationCurve));

	memcpy(mainConfig.tuneProfile[2].filterConfig[0].throttleCurve, throttleCurve,      sizeof(throttleCurve));
	memcpy(mainConfig.tuneProfile[2].filterConfig[0].tpaKpCurve,    kpAttenuationCurve, sizeof(kpAttenuationCurve));
	memcpy(mainConfig.tuneProfile[2].filterConfig[0].tpaKiCurve,    kiAttenuationCurve, sizeof(kiAttenuationCurve));
	memcpy(mainConfig.tuneProfile[2].filterConfig[0].tpaKdCurve,    kdAttenuationCurve, sizeof(kdAttenuationCurve));
}

void AdjustTpa(char *modString, char *outText, float inCurve[])
{
	char     tempString[8];
	float    tempCurve[9] = {0.0f,};
	float    tempFloat = 0.0f;
	uint32_t x = 0;
	for ( char *p = modString, *q = modString; p != NULL; p = q )
    {
        q = strchr( p, '=' );
        if ( q )
        {
			sprintf(tempString, "%*.*s", ( int )(q - p ), ( int )( q - p ), p );
			tempFloat = (float)((float)atoi( tempString ) / 100.0f);
			if ( (tempFloat >= 0.0f) && (tempFloat <= 2.0f) )
			{
            	tempCurve[x++] = tempFloat;
			}
			else
			{
				x = 0;
			}
            ++q;
        }
		else
        {
			sprintf(tempString, "%s", p );
            tempFloat = (float)((float)atoi( tempString ) / 100.0f);
			if ( (tempFloat >= 0.0f) && (tempFloat <= 2.0f) )
			{
            	tempCurve[x++] = tempFloat;
			}
			else
			{
				x = 0;
			}
        }
    }
	if (x == 9)
	{
		for (x=0;x<9;x++)
			inCurve[x] = tempCurve[x];
		RfCustomReplyBuffer("#me New Curve Set\n");
	}
	else
	{
		RfCustomReplyBuffer("#me Curve Not Changed\n");
	}
	PrintCurve(outText, inCurve);
}

static void PrintCurve(char *outText, float curve[])
{
	uint32_t i;
	RfCustomReplyBuffer(outText);
	for (i=0;i<ATTENUATION_CURVE_SIZE;i++)
	{
		if (i == ATTENUATION_CURVE_SIZE - 1)
			sprintf(rf_custom_out_buffer, "%lu", (uint32_t)(curve[i]*100) );
		else
			sprintf(rf_custom_out_buffer, "%lu=", (uint32_t)(curve[i]*100) );
		RfCustomReplyBuffer(rf_custom_out_buffer);
	}
	RfCustomReplyBuffer("\n");
}

void PrintTpaCurves(void)
{
	PrintCurve("throttlecurve1 ", mainConfig.tuneProfile[0].filterConfig[0].throttleCurve);
	PrintCurve("tpakp1 ", mainConfig.tuneProfile[0].filterConfig[0].tpaKpCurve);
	PrintCurve("tpaki1 ", mainConfig.tuneProfile[0].filterConfig[0].tpaKiCurve);
	PrintCurve("tpakd1 ", mainConfig.tuneProfile[0].filterConfig[0].tpaKdCurve);
	PrintCurve("throttlecurve2 ", mainConfig.tuneProfile[1].filterConfig[0].throttleCurve);
	PrintCurve("tpakp2 ", mainConfig.tuneProfile[1].filterConfig[0].tpaKpCurve);
	PrintCurve("tpaki2 ", mainConfig.tuneProfile[1].filterConfig[0].tpaKiCurve);
	PrintCurve("tpakd2 ", mainConfig.tuneProfile[1].filterConfig[0].tpaKdCurve);
	PrintCurve("throttlecurve3 ", mainConfig.tuneProfile[2].filterConfig[0].throttleCurve);
	PrintCurve("tpakp3 ", mainConfig.tuneProfile[2].filterConfig[0].tpaKpCurve);
	PrintCurve("tpaki3 ", mainConfig.tuneProfile[2].filterConfig[0].tpaKiCurve);
	PrintCurve("tpakd3 ", mainConfig.tuneProfile[2].filterConfig[0].tpaKdCurve);
}

void InitMixer(void) {
	int32_t i;

	threeDeeMode = 0;
	stabilizerAttenuation = 0;

	BuildThrottleLookupTable();
	BuildThrottleLookupTableKp();
	BuildThrottleLookupTableKi();
	BuildThrottleLookupTableKd();
	for (i=0;i<MAX_MOTOR_NUMBER;i++)
		motorOutput[i]=0.0f;

	for (i=0;i<MAX_SERVO_NUMBER;i++)
		servoOutput[i]=0.0f;

	switch (mainConfig.mixerConfig.mixerType) {
		case MIXER_X4213_3D:
			threeDeeMode = 1;
		case MIXER_X4213:
			memcpy(motorMixer, CONST_MIXER_X4213, sizeof(motorMixer));
			break;
		case MIXER_X4213RY_3D:
			threeDeeMode = 1;
		case MIXER_X4213RY:
			memcpy(motorMixer, CONST_MIXER_X4213RY, sizeof(motorMixer));
			break;
		case MIXER_PLUS1234:
			memcpy(motorMixer, CONST_MIXER_PLUS1234, sizeof(motorMixer));
			break;
		case MIXER_PLUS1234RY:
			memcpy(motorMixer, CONST_MIXER_PLUS1234RY, sizeof(motorMixer));
			break;
		case MIXER_CUSTOM:
			//fill mixer customer here, for now it defaults to MIXER_X1234
		case MIXER_X1234RY_3D:
			threeDeeMode = 1;
		case MIXER_X1234RY:
			memcpy(motorMixer, CONST_MIXER_X1234RY, sizeof(motorMixer));
			memcpy(motorMixerT, CONST_MIXER_X1234RYT, sizeof(motorMixerT));
			break;
		case MIXER_X1234_3D:
			threeDeeMode = 1;
		case MIXER_X1234:
		default:
			memcpy(motorMixer, CONST_MIXER_X1234, sizeof(motorMixer));
			break;
	}

	//number of active moters starting from 0.
	//quad is 3, hex is 5, octo is 7
	//todo: need to set this based on mixer
	for (i = activeMotorCounter; i >= 0; i--) {
		//yaw, roll, pitch, throttle, aux1, aux2, aux3, aux4
		if ( (motorMixer[i].yaw != 0 ) || (motorMixer[i].roll != 0 ) || (motorMixer[i].pitch != 0 ) || (motorMixer[i].throttle != 0 ) )
		{
			activeMotorCounter++; //increment active motor counter
		}
	}

	activeMotorCounter=3;
}

static float ApplyAttenuationCurve (float inputAttn, float curve[], uint32_t curveSize)
{
	float attenuationValue = (inputAttn * (curveSize - 1)); 
	float remainder = (float)((float)attenuationValue - (int)attenuationValue); 
	uint32_t position = (int)attenuationValue; 

	if (inputAttn == 1) 
		return(curve[curveSize-1]);
	else
		return(curve[position] + (((curve[position+1] - curve[position]) * remainder)));
}


static float ApplyAttenuationOldCurve (float inputAttn, float curve[], uint32_t curveSize)
{
    uint32_t indexAttn;
    float remainderAttn;
#ifdef STM32F446xx
    return(1.0f);
#endif
    remainderAttn = (float)((float)inputAttn * (float)curveSize);
    indexAttn =(int)remainderAttn;
    if (indexAttn == 0)
        return (curve[0]);
    else
    {
        remainderAttn = remainderAttn - (float)indexAttn;
        return (curve[indexAttn-1] + (curve[indexAttn] * remainderAttn));
    }
}


//just like the standard mixer, but optimized for speed since it runs at a much higher speed than normal servos
inline float InlineApplyMotorMixer3dUpright(pid_output pids[], float throttleIn)
{
	float idleNum1;
	float idleNum2;
	float highestMotor  = -100.0f;
	float lowestMotor   =  100.0f;
	float actuatorRange =  0.0f;
	volatile float throttle;
	float throttleOffset;
	int32_t i           = 0;

	//static int32_t threeDeeThrottleLatch = 1;

	for (i = activeMotorCounter; i >= 0; i--)
	{

		//-1 to 1
		motorOutput[i] = (
			(
				(pids[YAW].kp * 1 ) +
				(pids[YAW].kd * 1 ) +
				(pids[YAW].ki * 1 )
			) * motorMixer[i].yaw * -1.0f +
			(
				(pids[ROLL].kp * 1 ) +
				(pids[ROLL].kd * 1 ) +
				(pids[ROLL].ki * 1 )
			) * motorMixer[i].roll +
			(
				(pids[PITCH].kp * 1 ) +
				(pids[PITCH].kd * 1 ) +
				(pids[PITCH].ki * 1 )
			) * motorMixer[i].pitch
		);
		motorOutput[i] = InlineChangeRangef(motorOutput[i], 1.0, -1.0, 1.0, 0.0);
		if (motorOutput[i] > highestMotor) { highestMotor = motorOutput[i]; }
		if (motorOutput[i] < lowestMotor)  { lowestMotor  = motorOutput[i]; }
	}

	actuatorRange = highestMotor - lowestMotor;

	if (actuatorRange > 1.0f)
	{
		for (i = activeMotorCounter; i >= 0; i--)
		{
			motorOutput[i] /= actuatorRange;
		}
		throttle = 0.0f;
	}
	else
	{
		//put throttle range to same range as actuators. 0 to 1 from -1 to 1
		throttleOffset = actuatorRange / 2.0f;
		throttle = InlineConstrainf(throttleIn, throttleOffset, 1.0f - throttleOffset) - 0.5f;

	}

	idleNum1 = (0.5f - (mainConfig.mixerConfig.idlePercent * 0.01f));
	idleNum2 = (1 - (0.5f - (mainConfig.mixerConfig.idlePercent * 0.01f)));
	for(i=7; i>=0; i--)
	{
		//motorOutput[i] = InlineChangeRangef(motorOutput[i]+throttle, 1.0f, 0.0f, 1.0f, 0.55f);
		//motorOutput[i] = InlineConstrainf(motorOutput[i], 1.0f, 0.55f);
		motorOutput[i] = InlineConstrainf(motorOutput[i]+throttle,0.0f,1.0f) * idleNum1 + idleNum2;
	}

	return(actuatorRange);

}

inline float InlineApplyMotorMixer3dInverted(pid_output pids[], float throttleIn)
{
	float idleNum;
	float highestMotor  = -100.0f;
	float lowestMotor   =  100.0f;
	float actuatorRange =  0.0f;
	volatile float throttle;
	float throttleOffset;
	int32_t i           = 0;

	//static int32_t threeDeeThrottleLatch = 1;

	for (i = activeMotorCounter; i >= 0; i--)
	{

		//-1 to 1
		motorOutput[i] = (
			(
				(pids[YAW].kp * 1 ) +
				(pids[YAW].kd * 1 ) +
				(pids[YAW].ki * 1 )
			) * motorMixer[i].yaw * -1.0f +
			(
				(pids[ROLL].kp * 1 ) +
				(pids[ROLL].kd * 1 ) +
				(pids[ROLL].ki * 1 )
			) * motorMixer[i].roll +
			(
				(pids[PITCH].kp * 1 ) +
				(pids[PITCH].kd * 1 ) +
				(pids[PITCH].ki * 1 )
			) * motorMixer[i].pitch
		);
		motorOutput[i] = InlineChangeRangef(motorOutput[i], 1.0, -1.0, 1.0, 0.0);
		if (motorOutput[i] > highestMotor) { highestMotor = motorOutput[i]; }
		if (motorOutput[i] < lowestMotor)  { lowestMotor  = motorOutput[i]; }
	}

	actuatorRange = highestMotor - lowestMotor;

	if (actuatorRange > 1.0f)
	{
		for (i = activeMotorCounter; i >= 0; i--)
		{
			motorOutput[i] /= actuatorRange;
		}
		throttle = 0.0f;
	}
	else
	{
		//put throttle range to same range as actuators. 0 to 1 from -1 to 1
		throttleOffset = actuatorRange / 2.0f;
		throttle = InlineConstrainf(throttleIn, throttleOffset, 1.0f - throttleOffset) - 0.5f;

	}

	idleNum = (0.5f - (mainConfig.mixerConfig.idlePercentInverted * 0.01f));

	for(i=7; i>=0; i--)
	{
		motorOutput[i] = InlineConstrainf(motorOutput[i]+throttle,0.0f,1.0f) * idleNum;
	}

	return(actuatorRange);
}

inline float InlineApplyMotorMixer3dNeutral(pid_output pids[], float throttleIn)
{
	(void)(pids);
	(void)(throttleIn);
	int32_t i;
	for(i=7; i>=0; i--)
	{
		motorOutput[i] = 0.50f;
	}
	return(0.0f);
}

inline float ForeAftMixerFixer(float motorOutputFloat, float throttleFloat, uint32_t motorNumber)
{

	float usedFamx = (float)mainConfig.mixerConfig.foreAftMixerFixer * 0.01f;

	if (usedFamx == 1.0f)
	{
		//famx not active
		return(InlineConstrainf(motorOutputFloat+throttleFloat,0.0f,1.0f));
	}

	if (activeMotorCounter == 3)
	{
		//quad mixer. Motor 0 is always front left. Motor 1 is always front right. Motor 2 is always rear right. Motor 3 is always rear left.
		switch (motorNumber)
		{
			case 0:
			case 1:
				//front motors
				if (usedFamx < 1.0f)
				{
					//reduce front motors by mixer fixer
					throttleFloat = InlineChangeRangef(throttleFloat, motorMixer[motorNumber].throttle, 0.0f, (motorMixer[motorNumber].throttle * usedFamx), 0.0f);
				}
				break;
			case 2:
			case 3:
				//rear motors
				if (usedFamx > 1.0f)
				{
					//reduce rear motors by difference between mixer fixer and normal value (1.05 will reduce rear numbers to 95% at full throttle
					throttleFloat = InlineChangeRangef(throttleFloat, motorMixer[motorNumber].throttle, 0.0f, (motorMixer[motorNumber].throttle * ( 1.0f - (usedFamx - 1.0f ) ) ), 0.0f);
				}
				break;
		}
	}

	return(InlineConstrainf(motorOutputFloat+throttleFloat,0.0f,1.0f));
}

inline float InlineApplyMotorMixer(pid_output pids[], float throttleIn)
{

	float highestMotor = -100.0f;
	float lowestMotor  =  100.0f;
	int32_t i;                  //set
	float actuatorRange;        //set
	float throttle;             //set
 	float throttleOffset;       //set
	uint32_t motorOutput0_1023; //set

	//static int32_t threeDeeThrottleLatch = 1;

	for (i = activeMotorCounter; i >= 0; i--)
	{

		motorOutput0_1023 = lrintf( InlineChangeRangef(motorOutput[i], 1.0f, 0.0f, 1023.0f, 0.0f) );
		//-1 to 1
		if(quopaState == QUOPA_ACTIVE)
		{
			throttleIn = -1.0f;
			motorOutput[i] = (
				(
					(pids[YAW].kp * throttleLookupKp[motorOutput0_1023] * 0.1f ) +
					(pids[YAW].kd * throttleLookupKd[motorOutput0_1023] * 0.1f ) +
					(pids[YAW].ki * throttleLookupKi[motorOutput0_1023] * 0.1f ) +
					kiTrim[YAW]
				) * motorMixerT[i].yaw +
				(
					(pids[ROLL].kp * throttleLookupKp[motorOutput0_1023] * 0.66f ) +
					(pids[ROLL].kd * throttleLookupKd[motorOutput0_1023] * 0.66f ) +
					(pids[ROLL].ki * throttleLookupKi[motorOutput0_1023] * 0.66f ) +
					kiTrim[ROLL]
				) * motorMixerT[i].roll +
				(
					(pids[PITCH].kp * throttleLookupKp[motorOutput0_1023] * 0.66f ) +
					(pids[PITCH].kd * throttleLookupKd[motorOutput0_1023] * 0.66f ) +
					(pids[PITCH].ki * throttleLookupKi[motorOutput0_1023] * 0.66f ) +
					kiTrim[PITCH]
				) * motorMixerT[i].pitch
			);
		}
		else
		{
			motorOutput[i] = (
				(
					(pids[YAW].kp * throttleLookupKp[motorOutput0_1023] ) +
					(pids[YAW].kd * throttleLookupKd[motorOutput0_1023] ) +
					(pids[YAW].ki * throttleLookupKi[motorOutput0_1023] ) +
					kiTrim[YAW]
				) * motorMixer[i].yaw +
				(
					(pids[ROLL].kp * throttleLookupKp[motorOutput0_1023] ) +
					(pids[ROLL].kd * throttleLookupKd[motorOutput0_1023] ) +
					(pids[ROLL].ki * throttleLookupKi[motorOutput0_1023] ) +
					kiTrim[ROLL]
				) * motorMixer[i].roll +
				(
					(pids[PITCH].kp * throttleLookupKp[motorOutput0_1023] ) +
					(pids[PITCH].kd * throttleLookupKd[motorOutput0_1023] ) +
					(pids[PITCH].ki * throttleLookupKi[motorOutput0_1023] ) +
					kiTrim[PITCH]
				) * motorMixer[i].pitch
			);
		}
		motorOutput[i] = InlineChangeRangef(motorOutput[i], 1.0, -1.0, 1.0, 0.0);
		if (motorOutput[i] > highestMotor) { highestMotor = motorOutput[i]; }
		if (motorOutput[i] < lowestMotor)  { lowestMotor  = motorOutput[i]; }
	}

	actuatorRange = highestMotor - lowestMotor;

	if (actuatorRange > 1.0f)
	{
		for (i = activeMotorCounter; i >= 0; i--)
		{
			motorOutput[i] /= actuatorRange;
		}
		throttle = 0.0f;
	}
	else
	{
		//put throttle range to same range as actuators. 0 to 1 from -1 to 1
		throttleOffset  = actuatorRange / 2.0f;
		throttle        = InlineConstrainf(throttleIn, throttleOffset, 1.0f - throttleOffset) - 0.5f;
	}

	for(i=7; i>=0; i--)
	{
		if(mainConfig.mixerConfig.foreAftMixerFixer > 89)
			motorOutput[i] = ForeAftMixerFixer( motorOutput[i], throttle, i);
		else
			motorOutput[i] = InlineConstrainf(motorOutput[i]+throttle,0.0f,1.0f);
		//if(ModeSet(M_LEARN) && !ModeActive(M_LEARN))
		//	motorOutput[i] = InlineConstrainf(motorOutput[i] * persistance.data.motorTrim[i], 0.0f, 1.0f);
			
	}

	return(actuatorRange);

}

inline float InlineApplyMotorMixer1(pid_output pids[], float throttleIn)
{

	float highestMotor  = -100.0f;
	float lowestMotor   =  100.0f;
	int32_t i;                  //set
	float actuatorRange;        //set
	float throttle;             //set
 	float throttleOffset;       //set
	uint32_t motorOutput0_1023; //set

	for (i = activeMotorCounter; i >= 0; i--)
	{
		motorOutput0_1023 = lrintf( InlineChangeRangef(motorOutput[i], 1.0f, 0.0f, 1023.0f, 0.0f) );
 		//-1 to 1
 		motorOutput[i] = (
			(
				(pids[YAW].kp * throttleLookupKp[motorOutput0_1023] ) +
				(pids[YAW].kd * throttleLookupKd[motorOutput0_1023] ) +
				(pids[YAW].ki * throttleLookupKi[motorOutput0_1023] )
			) * motorMixer[i].yaw +
			(
				(pids[ROLL].kp * throttleLookupKp[motorOutput0_1023] ) +
				(pids[ROLL].kd * throttleLookupKd[motorOutput0_1023] ) +
				(pids[ROLL].ki * throttleLookupKi[motorOutput0_1023] )
			) * motorMixer[i].roll +
			(
				(pids[PITCH].kp * throttleLookupKp[motorOutput0_1023] ) +
				(pids[PITCH].kd * throttleLookupKd[motorOutput0_1023] ) +
				(pids[PITCH].ki * throttleLookupKi[motorOutput0_1023] )
			) * motorMixer[i].pitch
 		);
 		if (motorOutput[i] > highestMotor) { highestMotor = motorOutput[i]; }
 		if (motorOutput[i] < lowestMotor)  { lowestMotor  = motorOutput[i]; }
 	}
 	actuatorRange = highestMotor - lowestMotor;
 	if (actuatorRange > 1.0f)
 	{
 		for (i = activeMotorCounter; i >= 0; i--)
 		{
 			motorOutput[i] /= actuatorRange;
 		}
 		throttle = 0.0f;
 	}
 	else
 	{
 		//put throttle range to same range as actuators. 0 to 1 from -1 to 1
		throttleOffset  = actuatorRange / 2.0f;
		throttle        = InlineConstrainf(throttleIn, throttleOffset, 1.0f - throttleOffset);

 	}
 	for(i=7; i>=0; i--)
 	{
			motorOutput[i] = InlineConstrainf(motorOutput[i]+throttle,0.0f,1.0f);
			//if(ModeSet(M_LEARN) && !ModeActive(M_LEARN))
			//	motorOutput[i] = InlineConstrainf(motorOutput[i] * persistance.data.motorTrim[i], 0.0f, 1.0f);
	}
 	return(actuatorRange);
 }

inline void InlineApplyMixer(pid_output pids[], float curvedRcCommandF[])
{
	(void)(pids);
	(void)(curvedRcCommandF);
}
