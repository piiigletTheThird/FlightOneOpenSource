#include "includes.h"

//0.1 to -0.1 fits within uint8_t 
#define KI_LEARN_MULTIPLIER   0.00078740157186985015869140625f
#define KI_LEARN_MULTIPLIER_I 1270.0f

//for 20 value table:
#define KI_TRIM_TABLE_SIZE 20
//for 20 value table:
#define X_LEARNING_AVERAGE 52.5f
//for 20 steps
#define X_STEP             0.05f

volatile learned_ki_model learnedKiModel[AXIS_NUMBER];

int motorTrimHasHappened;
int motorTrimCounter;
int kiTrimCounter;

static int GenerateTheKiTables(volatile int8_t kiTrim[], volatile float *mTemp, volatile float *bTemp);
static int BuildKiModel(volatile int8_t kiTrim[], volatile uint32_t confidenceLevel[], volatile learned_ki_model *learnedKiModel);

int BuildLearnedKiModel(void)
{
    BuildKiModel(persistance.data.yawKiTrim8, persistance.data.rememberence, &learnedKiModel[YAW]);
    BuildKiModel(persistance.data.rollKiTrim8, persistance.data.rememberence, &learnedKiModel[ROLL]);
    BuildKiModel(persistance.data.pitchKiTrim8, persistance.data.rememberence, &learnedKiModel[PITCH]);
    return(0);
}

inline float ApplyLearningModelToKi(float throttle, uint32_t axis)
{
	if( (mainConfig.mixerConfig.foreAftMixerFixer == 0) || (mainConfig.mixerConfig.foreAftMixerFixer > 2) )
		return(0.0f);
	else
		return( ConvertInt8ToFloatForKi( (throttle * learnedKiModel[axis].m) + learnedKiModel[axis].b ) );
	//return( ConvertInt8ToFloatForKi( (throttle * learnedKiModel[axis].m) ) );
}

int LearningInit(void)
{
    for (int x=(AXIS_NUMBER-1); x>=0; x--)
    {
        learnedKiModel[x].b = 0;
        learnedKiModel[x].m = 0;
    }

    motorTrimHasHappened = 0;
	motorTrimCounter = 0;
    kiTrimCounter = 0;

    return( 0 );
}

int LearningModelInit(void)
{
    return( BuildLearnedKiModel() );
}

inline int8_t ConvertFloatToInt8ForKi(float kiNumber)
{
	//clamp the values to fit in an int8
	if(kiNumber > 0.1f)
		kiNumber = 0.1f;

	if(kiNumber < -0.1f)
		kiNumber = -0.1f;

	return((int8_t)(kiNumber * KI_LEARN_MULTIPLIER_I));
}

inline float ConvertInt8ToFloatForKi(int8_t kiNumber)
{
	return((float)((float)kiNumber * KI_LEARN_MULTIPLIER));
}

int TrimKi(pid_output flightPids[])
{

	if(!fullKiLatched)
		return(0);

	if ( !(mainConfig.mixerConfig.foreAftMixerFixer == 2) )
		if (!ModeSet(M_LEARN))
        	return(0);
    
    if( (mainConfig.mixerConfig.foreAftMixerFixer == 0) || (mainConfig.mixerConfig.foreAftMixerFixer > 2) )
		return(0);

	if(!boardArmed)
		return(0);

    if(
        //(motorTrimHasHappened) &&
        (smoothedRcCommandF[PITCH] == 0.0f) &&
        (smoothedRcCommandF[ROLL] == 0.0f) &&
        (smoothedRcCommandF[YAW] == 0.0f)
    )
    {
        //need at least 20ms to trim motors, so 19+ will work
        kiTrimCounter++;
    }
    else
    {
        kiTrimCounter = 0;
    }

	uint32_t position = lrintf(smoothCurvedThrottle0_1*19);

	if (
		( (mainConfig.mixerConfig.foreAftMixerFixer == 2) ||
		ModeActive(M_LEARN) ) &&
		(kiTrimCounter > 19)
	)
	{
		persistance.data.yawKiTrim8[position] = (persistance.data.yawKiTrim8[position] * 0.95f) + (ConvertFloatToInt8ForKi(flightPids[YAW].ki + kiTrim[YAW]) * 0.05f);
		persistance.data.rollKiTrim8[position] = (persistance.data.rollKiTrim8[position] * 0.95f) + (ConvertFloatToInt8ForKi(flightPids[ROLL].ki + kiTrim[ROLL]) * 0.05f);
		persistance.data.pitchKiTrim8[position] = (persistance.data.pitchKiTrim8[position] * 0.95f) + (ConvertFloatToInt8ForKi(flightPids[PITCH].ki + kiTrim[PITCH]) * 0.05f);
		persistance.data.geeForce[position] = lrintf( (float)(persistance.data.geeForce[position] * 0.95f) + (float)((geeForceZ * 10.0f) * 0.05f));
		persistance.data.rememberence[position]++;
	}

	return(1);
}

int TrimMotors(void)
{
	if(!fullKiLatched)
		return(0);

	if ( !(mainConfig.mixerConfig.foreAftMixerFixer == 2) )
		if (!ModeSet(M_LEARN))
			return(0);

	if( (mainConfig.mixerConfig.foreAftMixerFixer == 0) || (mainConfig.mixerConfig.foreAftMixerFixer > 2) )
		return(0);

	if(!boardArmed)
		return(0);

	if(smoothCurvedThrottle0_1 < 0.05)
	{
		//motorTrimHasHappened = 0;
	}

	//throttle is 100% and constrols in deadband range
	if(
		( (mainConfig.mixerConfig.foreAftMixerFixer == 2) || ModeActive(M_LEARN) ) &&
		//(motorTrimHasHappened == 0) &&
		(smoothCurvedThrottle0_1 > 0.99f) &&
		(smoothedRcCommandF[PITCH] == 0.0f) &&
		(smoothedRcCommandF[ROLL] == 0.0f) &&
		(smoothedRcCommandF[YAW] == 0.0f)
	)
	{
		//need at least 50ms to trim motors, so 49+ will work
		motorTrimCounter++;
	}
	else
	{
		motorTrimCounter = 0;
		if(motorTrimHasHappened == 1)
			motorTrimHasHappened = 2;
	}

	if(motorTrimCounter>49)
	{
		if(motorTrimCounter>=99)
			motorTrimCounter = 0;

		motorTrimHasHappened = 1;
		float tallestMotor = 0.0f;
		//gradually trim motors
		//only check active motors
		//activeMotorCounter in mixer.c
		//for now quad only
		for(int xxx = 0; xxx < 4; xxx++)
		{
			persistance.data.motorTrim[xxx] = CONSTRAIN( (persistance.data.motorTrim[xxx] * 0.95f) + (motorOutput[xxx] * 0.05f), 0.85f, 1.0f);
			if(tallestMotor < persistance.data.motorTrim[xxx])
				tallestMotor = persistance.data.motorTrim[xxx];
		}

		for(int xxx = 0; xxx < 4; xxx++)
		{
			persistance.data.motorTrim[xxx] += (1.0f - tallestMotor);
		}

	}

	return(0);
}

static int GenerateTheKiTables(volatile int8_t kiTrim[], volatile float *mTemp, volatile float *bTemp)
{
	int   x;
	float xStep = 1.0f / (float)KI_TRIM_TABLE_SIZE;
	float xTemp = 0.0f;
	float xAverage = 0;
	float yAverage = 0;
	float xixaTable[KI_TRIM_TABLE_SIZE];
	float yiyaTable[KI_TRIM_TABLE_SIZE];
	float xTxSum = 0.0f;
	float xTySum = 0.0f;

	//get average of x values
	for (x=0;x<KI_TRIM_TABLE_SIZE;x++)
	{
		xAverage += xTemp;
		xTemp += xStep;
	}
	xAverage /= KI_TRIM_TABLE_SIZE;

	//get average of y values
	for (x=0;x<KI_TRIM_TABLE_SIZE;x++)
	{
		yAverage += (float)(kiTrim[x]);
	}
	yAverage /= KI_TRIM_TABLE_SIZE;

	//calculate table of Xi - Xa
	xTemp = 0;
	for (x=0;x<KI_TRIM_TABLE_SIZE;x++)
	{
		xixaTable[x] = xTemp - xAverage;
		xTemp += xStep;
	}

	//calculate table of Yi - Ya
	for (x=0;x<KI_TRIM_TABLE_SIZE;x++)
	{
		yiyaTable[x] = kiTrim[x] - yAverage;
	}

	//build x*y and x*x table sums
	for (x=0;x<KI_TRIM_TABLE_SIZE;x++)
	{
		xTySum += (xixaTable[x] * yiyaTable[x]);
		xTxSum += (xixaTable[x] * xixaTable[x]);
	}

	//set m and b
	(*mTemp) = xTySum/xTxSum;
	(*bTemp) = (yAverage - ((*mTemp) * xAverage) );

    return(0);
}

static int BuildKiModel(volatile int8_t kiTrim[], volatile uint32_t confidenceLevel[], volatile learned_ki_model *learnedKiModel)
{
    //for 20 steps
	volatile int8_t tempKiTrim[KI_TRIM_TABLE_SIZE];
	volatile float mTemp;
	volatile float bTemp;
	int   x;
	float xStep = 1.0f / (float)KI_TRIM_TABLE_SIZE;
	float xTemp = 0.0f;

	//only build model if confidence level is high enough
	if( 
		(confidenceLevel[KI_TRIM_TABLE_SIZE-1] > 50)
		//(confidenceLevel[KI_TRIM_TABLE_SIZE-2] > 50) &&
		//(confidenceLevel[KI_TRIM_TABLE_SIZE-3] > 50) &&
		//(confidenceLevel[KI_TRIM_TABLE_SIZE-4] > 50) &&
		//(confidenceLevel[3] > 50) &&
		//(confidenceLevel[2] > 50) &&
		//(confidenceLevel[1] > 50) &&
		//(confidenceLevel[0] > 50)
	)
	{
		//nothing
	}
	else
	{
		return(0);
	}
	//generate the first set
	GenerateTheKiTables(kiTrim, &mTemp, &bTemp);

	//take care of outliers
	xTemp = 0;
	for (x=0;x<KI_TRIM_TABLE_SIZE;x++)
	{
		if (x==0)
		{
			tempKiTrim[x] = kiTrim[x];  //ignore this one
		}
		else if( (x<4) && ABS(kiTrim[x]-lrintf(mTemp*xTemp + bTemp)) > 15)
		{
			tempKiTrim[x] = lrintf(mTemp*xTemp + bTemp);
		}
		else if( ABS(kiTrim[x]-lrintf(mTemp*xTemp + bTemp)) > 25)
		{
			tempKiTrim[x] = lrintf(mTemp*xTemp + bTemp);
		}
		else
		{
			tempKiTrim[x] = kiTrim[x];
		}
		xTemp += xStep;
	}

	//generate the second set with fixed outliers
	GenerateTheKiTables(tempKiTrim, &(learnedKiModel->m), &(learnedKiModel->b) );

    return(0);    
}