#include "includes.h"

#define GYRO_AVERAGE_MAX_SUM 33

pid_output   flightPids[AXIS_NUMBER];
biquad_state lpfFilterState[AXIS_NUMBER];
biquad_state lpfFilterStateKd[AXIS_NUMBER];
biquad_state lpfFilterStateAcc[AXIS_NUMBER];
biquad_state hpfFilterStateAcc[6];

biquad_state geeForceZFilter;
volatile float geeForceZ;

float gyroFiltUsed[AXIS_NUMBER];
float accNoise[6];
float averagedGyroData[AXIS_NUMBER][GYRO_AVERAGE_MAX_SUM];
uint32_t averagedGyroDataPointer[AXIS_NUMBER];
float averagedGyroDataPointerMultiplier[AXIS_NUMBER];
float filteredGyroData[AXIS_NUMBER];
float filteredAccData[VECTOR_NUMBER];
paf_state pafGyroStates[AXIS_NUMBER];
paf_state pafKdStates[AXIS_NUMBER];
paf_state pafAccStates[AXIS_NUMBER];
float actuatorRange;
float flightSetPoints[AXIS_NUMBER];
volatile uint32_t boardArmed, calibrateMotors, fullKiLatched;
volatile float flightcodeTime;
float pitchAttitude = 0, rollAttitude = 0, yawAttitude = 0;
uint32_t boardOrientation1 = 0;
uint32_t RfblDisasterPreventionCheck = 1;
uint32_t timeSinceSelfLevelActivated;
float slpUsed;
float sliUsed;
float sldUsed;
int   usedSkunk;
volatile uint32_t armedTime;
volatile uint32_t armedTimeSincePower = 0;

uint32_t khzLoopCounter                 = 0;
uint32_t khzLoopCounterPhase            = 0;
int32_t gyroFlightLoopCounter           = 0;
volatile uint32_t SKIP_GYRO             = 0;
volatile float yawAttitudeError         = 0.0f;
volatile float rollAttitudeError        = 0.0f;
volatile float pitchAttitudeError       = 0.0f;
volatile float yawAttitudeErrorKi       = 0.0f;
volatile float rollAttitudeErrorKi      = 0.0f;
volatile float pitchAttitudeErrorKi     = 0.0f;

volatile float yawAttitudeErrorKdelta   = 0.0f;
volatile float lastYawAttitudeError     = 0.0f;
volatile float rollAttitudeErrorKdelta  = 0.0f;
volatile float lastRollAttitudeError    = 0.0f;
volatile float pitchAttitudeErrorKdelta = 0.0f;
volatile float lastPitchAttitudeError   = 0.0f;

volatile uint32_t usedGa[AXIS_NUMBER]   = {0,};

//these numbers change based on loop_control
volatile loop_speed_record loopSpeed;

static float BoostModify(volatile float throttleIn);
void  InlineInitGyroFilters(void);
void  InlineInitKdFilters(void);
void  InlineInitSpectrumNoiseFilter(void);
void  InlineInitAccFilters(void);
float InlineGetSetPoint(float curvedRcCommandF, uint32_t curveToUse, float rates, float acroPlus, uint32_t axis);
float AverageGyroADCbuffer(uint32_t axis, volatile float currentData);

void ArmBoard(void)
{
	//no arming when dshot beep happening
	if(dshotBeepState != QUOPA_INACTIVE)
		return;

	//no arming when ModeActive M_BEEP
	if(ModeActive(M_BEEP))
		return;

	//no arming when dshot commands happening
	if(dshotCommandHandler.dshotCommandState != DSC_MODE_INACTIVE)
		return;

	if (board.flash[0].enabled)
	{
		LearningModelInit(); //needs flash
	}

	InitWatchdog(WATCHDOG_TIMEOUT_30MS);
	boardArmed = 1;
	timeSinceSelfLevelActivated = 0;
}

void DisarmBoard(void)
{
	if (boardArmed)
	{
		boardArmed = 0;
		ZeroActuators(10000); //make sure hardware PWM is zeroed
		InitWatchdog(WATCHDOG_TIMEOUT_32S);
	}
}

int SetCalibrate1(void)
{

	if ( ABS(filteredAccData[ACCZ]) > (ABS(filteredAccData[ACCX]) + ABS(filteredAccData[ACCY])) )
	{
		// is king
		if (filteredAccData[ACCZ] < -0.8) //ACCZ negative
		{
			//board inverted
			return (boardOrientation1 = CALIBRATE_BOARD_INVERTED);
		}
		else if (filteredAccData[ACCZ] > 0.8) //ACCZ positive
		{
			//board upright
			return (boardOrientation1 = CALIBRATE_BOARD_UPRIGHT);
		}

	}

	return (CALIBRATE_BOARD_FAILED); //check sanity and return 0 if result not sane.

}

int SetCalibrate2(void)
{

	if (!boardOrientation1)
	{ //make sure step one completed successfully.
		return (0);
	}

	if (boardOrientation1 == CALIBRATE_BOARD_UPRIGHT)
	{
		if (filteredAccData[ACCX] < -0.9)
		{
			boardOrientation1 = 0; //done calibrating
			mainConfig.gyroConfig.boardCalibrated = 1; //set board as calibrated
			mainConfig.gyroConfig.gyroRotation = CW0;  //set proper rotation
		}
		else if (filteredAccData[ACCY] < -0.9)
		{
			boardOrientation1 = 0; //done calibrating
			mainConfig.gyroConfig.boardCalibrated = 1; //set board as calibrated
			mainConfig.gyroConfig.gyroRotation = CW90;  //set proper rotation
		}
		else if (filteredAccData[ACCX] > 0.9)
		{
			boardOrientation1 = 0; //done calibrating
			mainConfig.gyroConfig.boardCalibrated = 1; //set board as calibrated
			mainConfig.gyroConfig.gyroRotation = CW180;  //set proper rotation
		}
		else if (filteredAccData[ACCY] > 0.9)
		{
			boardOrientation1 = 0; //done calibrating
			mainConfig.gyroConfig.boardCalibrated = 1; //set board as calibrated
			mainConfig.gyroConfig.gyroRotation = CW270;  //set proper rotation
		}
		else if ((filteredAccData[ACCX] < -0.6) && (filteredAccData[ACCY] < -0.6))
		{
			boardOrientation1 = 0; //done calibrating
			mainConfig.gyroConfig.boardCalibrated = 1; //set board as calibrated
			mainConfig.gyroConfig.gyroRotation = CW45;  //set proper rotation
		}
		else if ((filteredAccData[ACCX] > 0.6) && (filteredAccData[ACCY] < -0.6))
		{
			boardOrientation1 = 0; //done calibrating
			mainConfig.gyroConfig.boardCalibrated = 1; //set board as calibrated
			mainConfig.gyroConfig.gyroRotation = CW135;  //set proper rotation
		}
		else if ((filteredAccData[ACCX] > 0.6) && (filteredAccData[ACCY] > 0.6))
		{
			boardOrientation1 = 0; //done calibrating
			mainConfig.gyroConfig.boardCalibrated = 1; //set board as calibrated
			mainConfig.gyroConfig.gyroRotation = CW225;  //set proper rotation
		}
		else if ((filteredAccData[ACCX] < -0.6) && (filteredAccData[ACCY] > 0.6))
		{
			boardOrientation1 = 0; //done calibrating
			mainConfig.gyroConfig.boardCalibrated = 1; //set board as calibrated
			mainConfig.gyroConfig.gyroRotation = CW315;  //set proper rotation
		}
		else
		{
			boardOrientation1 = 0;
			return (CALIBRATE_BOARD_FAILED);
		}
	}
	else
	{
		if (filteredAccData[ACCX] < -0.9)
		{
			boardOrientation1 = 0; //done calibrating
			mainConfig.gyroConfig.boardCalibrated = 1; //set board as calibrated
			mainConfig.gyroConfig.gyroRotation = CW180_INV;  //set proper rotation
		}
		else if (filteredAccData[ACCY] < -0.9)
		{
			boardOrientation1 = 0; //done calibrating
			mainConfig.gyroConfig.boardCalibrated = 1; //set board as calibrated
			mainConfig.gyroConfig.gyroRotation = CW90_INV;  //set proper rotation
		}
		else if (filteredAccData[ACCX] > 0.9)
		{
			boardOrientation1 = 0; //done calibrating
			mainConfig.gyroConfig.boardCalibrated = 1; //set board as calibrated
			mainConfig.gyroConfig.gyroRotation = CW0_INV;  //set proper rotation
		}
		else if (filteredAccData[ACCY] > 0.9)
		{
			boardOrientation1 = 0; //done calibrating
			mainConfig.gyroConfig.boardCalibrated = 1; //set board as calibrated
			mainConfig.gyroConfig.gyroRotation = CW270_INV;  //set proper rotation
		}
		else if ((filteredAccData[ACCX] < -0.6) && (filteredAccData[ACCY] < -0.6))
		{
			boardOrientation1 = 0; //done calibrating
			mainConfig.gyroConfig.boardCalibrated = 1; //set board as calibrated
			mainConfig.gyroConfig.gyroRotation = CW315_INV;  //set proper rotation
		}
		else if ((filteredAccData[ACCX] > 0.6) && (filteredAccData[ACCY] < -0.6))
		{
			boardOrientation1 = 0; //done calibrating
			mainConfig.gyroConfig.boardCalibrated = 1; //set board as calibrated
			mainConfig.gyroConfig.gyroRotation = CW225_INV;  //set proper rotation
		}
		else if ((filteredAccData[ACCX] > 0.6) && (filteredAccData[ACCY] > 0.6))
		{
			boardOrientation1 = 0; //done calibrating
			mainConfig.gyroConfig.boardCalibrated = 1; //set board as calibrated
			mainConfig.gyroConfig.gyroRotation = CW135_INV;  //set proper rotation
		}
		else if ((filteredAccData[ACCX] < -0.6) && (filteredAccData[ACCY] > 0.6))
		{
			boardOrientation1 = 0; //done calibrating
			mainConfig.gyroConfig.boardCalibrated = 1; //set board as calibrated
			mainConfig.gyroConfig.gyroRotation = CW45_INV;  //set proper rotation
		}
		else
		{
			boardOrientation1 = 0;
			return (CALIBRATE_BOARD_FAILED);
		}
	}

	return(1);

}

float AverageGyroADCbuffer(uint32_t axis, volatile float currentData)
{
	float returnData;
	if (usedGa[axis] > 1)
	{
		averagedGyroData[axis][averagedGyroDataPointer[axis]++] = currentData;
		averagedGyroData[axis][GYRO_AVERAGE_MAX_SUM-1] += currentData;

		if (averagedGyroDataPointer[axis] == usedGa[axis])
			averagedGyroDataPointer[axis] = 0;

		returnData = (averagedGyroData[axis][GYRO_AVERAGE_MAX_SUM-1] * averagedGyroDataPointerMultiplier[axis]);
		averagedGyroData[axis][GYRO_AVERAGE_MAX_SUM-1] -= averagedGyroData[axis][averagedGyroDataPointer[axis]];
		return(returnData);
	}
	return currentData;
}

void InitFlightCode(uint32_t loopUsed)
{

	SKIP_GYRO = 0;
	armedTime = 0;

	bzero(lpfFilterState,          sizeof(lpfFilterState));
	bzero(lpfFilterStateKd,        sizeof(lpfFilterStateKd));
	bzero(averagedGyroData,        sizeof(averagedGyroData));
	bzero(averagedGyroDataPointer, sizeof(averagedGyroDataPointer));
	bzero(filteredGyroData,        sizeof(filteredGyroData));
	bzero(&flightPids,             sizeof(flightPids));

	timeSinceSelfLevelActivated = 0;
	slpUsed = 0.0f;
	sliUsed = 0.0f;
	sldUsed = 0.0f;

	averagedGyroDataPointerMultiplier[YAW]   = (1.0 / (float)mainConfig.tuneProfile[activeProfile].filterConfig[YAW].ga);
	averagedGyroDataPointerMultiplier[ROLL]  = (1.0 / (float)mainConfig.tuneProfile[activeProfile].filterConfig[ROLL].ga);
	averagedGyroDataPointerMultiplier[PITCH] = (1.0 / (float)mainConfig.tuneProfile[activeProfile].filterConfig[PITCH].ga);

	usedGa[0] = mainConfig.tuneProfile[activeProfile].filterConfig[0].ga;
	usedGa[1] = mainConfig.tuneProfile[activeProfile].filterConfig[1].ga;
	usedGa[2] = mainConfig.tuneProfile[activeProfile].filterConfig[2].ga;

	//set loopSpeed variables based on requested option
	switch (loopUsed)
	{
		case LOOP_UH32:
		case LOOP_H32:
			loopSpeed.gyrodT      = 0.00003125f;
			loopSpeed.dT          = 0.00003125f;
			loopSpeed.accdT       = 0.00100000f;
			loopSpeed.uhohNumber  = 24000;
			loopSpeed.gyroDivider = 0;
			loopSpeed.khzDivider  = 32;
			loopSpeed.gyroAccDiv  = 8; //gyro and acc still run at full speed
			loopSpeed.fsCount     = 500; //failsafe count for khzdivider
			break;
		case LOOP_UH16:
		case LOOP_H16:
			loopSpeed.gyrodT      = 0.00003125f;
			loopSpeed.dT          = 0.00006250f;
			loopSpeed.accdT       = 0.00100000f;
			loopSpeed.uhohNumber  = 12000;
			loopSpeed.gyroDivider = 1;
			loopSpeed.khzDivider  = 16;
			loopSpeed.gyroAccDiv  = 8; //gyro and acc still run at full speed
			loopSpeed.fsCount     = 500; //failsafe count for khzdivider
			break;
		case LOOP_UH8:
			loopSpeed.gyrodT      = 0.00003125f;
			loopSpeed.dT          = 0.00012500f;
			loopSpeed.accdT       = 0.00100000f;
			loopSpeed.uhohNumber  = 6000;
			loopSpeed.gyroDivider = 3;
			loopSpeed.khzDivider  = 8;
			loopSpeed.gyroAccDiv  = 8; //gyro and acc still run at full speed
			loopSpeed.fsCount     = 500; //failsafe count for khzdivider
			break;
		case LOOP_H8:
		case LOOP_M8:
			loopSpeed.gyrodT      = 0.00012500f;
			loopSpeed.dT          = 0.00012500f;
			loopSpeed.accdT       = 0.00100000f;
			loopSpeed.uhohNumber  = 6000;
			loopSpeed.gyroDivider = 0;
			loopSpeed.khzDivider  = 8;
			loopSpeed.gyroAccDiv  = 2;
			loopSpeed.fsCount     = 500; //failsafe count for khzdivider
			break;
		case LOOP_UH4:
			loopSpeed.gyrodT      = 0.00003125f;
			loopSpeed.dT          = 0.00025000f;
			loopSpeed.accdT       = 0.00100000f;
			loopSpeed.uhohNumber  = 3000;
			loopSpeed.gyroDivider = 7;
			loopSpeed.khzDivider  = 4;
			loopSpeed.gyroAccDiv  = 8;
			loopSpeed.fsCount     = 500; //failsafe count for khzdivider
			break;
		case LOOP_H4:
		case LOOP_M4:
			loopSpeed.gyrodT      = 0.00012500f;
			loopSpeed.dT          = 0.00025000f;
			loopSpeed.accdT       = 0.00100000f;
			loopSpeed.uhohNumber  = 3000;
			loopSpeed.gyroDivider = 1;
			loopSpeed.khzDivider  = 4;
			loopSpeed.gyroAccDiv  = 2;
			loopSpeed.fsCount     = 500; //failsafe count for khzdivider
			break;
		case LOOP_UH2:
			loopSpeed.gyrodT      = 0.00003125f;
			loopSpeed.dT          = 0.00050000f;
			loopSpeed.accdT       = 0.00100000f;
			loopSpeed.uhohNumber  = 1500;
			loopSpeed.gyroDivider = 15;
			loopSpeed.khzDivider  = 2;
			loopSpeed.gyroAccDiv  = 8;
			loopSpeed.fsCount     = 500; //failsafe count for khzdivider
		case LOOP_H2:
		case LOOP_M2:
			loopSpeed.gyrodT      = 0.00012500f;
			loopSpeed.dT          = 0.00050000f;
			loopSpeed.accdT       = 0.00100000f;
			loopSpeed.uhohNumber  = 1500;
			loopSpeed.gyroDivider = 3;
			loopSpeed.khzDivider  = 2;
			loopSpeed.gyroAccDiv  = 2;
			loopSpeed.fsCount     = 500; //failsafe count for khzdivider
			break;
		case LOOP_UH1:
			loopSpeed.gyrodT      = 0.00003125f;
			loopSpeed.dT          = 0.00100000f;
			loopSpeed.accdT       = 0.00100000f;
			loopSpeed.uhohNumber  = 750;
			loopSpeed.gyroDivider = 31;
			loopSpeed.khzDivider  = 1;
			loopSpeed.gyroAccDiv  = 8;
			loopSpeed.fsCount     = 500; //failsafe count for khzdivider
			break;
		case LOOP_UH_500:
			loopSpeed.gyrodT      = 0.00003125f;
			loopSpeed.dT          = 0.00200000f;
			loopSpeed.accdT       = 0.00200000f;
			loopSpeed.uhohNumber  = 375;
			loopSpeed.gyroDivider = 63;
			loopSpeed.khzDivider  = 1;
			loopSpeed.gyroAccDiv  = 8;
			loopSpeed.fsCount     = 250; //failsafe count for khzdivider
			break;
		case LOOP_UH_250:
			loopSpeed.gyrodT      = 0.00003125f;
			loopSpeed.dT          = 0.00400000f;
			loopSpeed.accdT       = 0.00400000f;
			loopSpeed.uhohNumber  = 187;
			loopSpeed.gyroDivider = 127;
			loopSpeed.khzDivider  = 1;
			loopSpeed.gyroAccDiv  = 8;
			loopSpeed.fsCount     = 125; //failsafe count for khzdivider
			break;
		case LOOP_UH_062:
			loopSpeed.gyrodT      = 0.00003125f;
			loopSpeed.dT          = 0.01600000f;
			loopSpeed.accdT       = 0.01600000f;
			loopSpeed.uhohNumber  = 94;
			loopSpeed.gyroDivider = 511;
			loopSpeed.khzDivider  = 1;
			loopSpeed.gyroAccDiv  = 8;
			loopSpeed.fsCount     = 62; //failsafe count for khzdivider
			break;
		case LOOP_H1:
		case LOOP_M1:
			loopSpeed.gyrodT      = 0.00012500f;
			loopSpeed.dT          = 0.00100000f;
			loopSpeed.accdT       = 0.00100000f;
			loopSpeed.uhohNumber  = 750;
			loopSpeed.gyroDivider = 7;
			loopSpeed.khzDivider  = 1;
			loopSpeed.gyroAccDiv  = 2;
			loopSpeed.fsCount     = 500; //failsafe count for khzdivider
			break;
		case LOOP_L1:
		default:
			loopSpeed.gyrodT      = 0.00100000f;
			loopSpeed.dT          = 0.00100000f;
			loopSpeed.accdT       = 0.00100000f;
			loopSpeed.uhohNumber  = 750;
			loopSpeed.gyroDivider = 0;
			loopSpeed.khzDivider  = 1;
			loopSpeed.gyroAccDiv  = 1;
			loopSpeed.fsCount     = 500; //failsafe count for khzdivider
			break;
	}

	//TODO: gyroConfig.accDenom is not set until after gyro is running.
	//loopSpeed.accdT     = loopSpeed.gyrodT * gyroConfig.accDenom;
	loopSpeed.halfGyrodT         = loopSpeed.gyrodT * 0.5f;
	loopSpeed.halfGyrodTSquared  = loopSpeed.gyrodT * loopSpeed.gyrodT * 0.5f;
	loopSpeed.halfGyrodTSquaredI = 1.0f/loopSpeed.halfGyrodTSquared;
	//loopSpeed.accdT              = loopSpeed.gyrodT * (float)loopSpeed.gyroAccDiv;
	loopSpeed.InversedT          = (1/loopSpeed.dT);
	loopSpeed.truedT             = loopSpeed.dT * 2.0f;

	actuatorRange       = 0;
	boardArmed          = 0;
	calibrateMotors     = 0;
	fullKiLatched       = 0;
	flightcodeTime      = 0.0f;

	for (uint32_t y=0; y<3; y++)
	{
		//if(mainConfig.tuneProfile[activeProfile].filterConfig[y].gyro.q < 100.0f)
			gyroFiltUsed[y] = (100.0f - mainConfig.tuneProfile[activeProfile].filterConfig[y].gyro.q);
		//else
		//	gyroFiltUsed[y] = mainConfig.tuneProfile[activeProfile].filterConfig[y].gyro.q;
	}

	InlineInitGyroFilters();
	//InlineInitKdFilters();
	InlineInitSpectrumNoiseFilter();
	InlineInitAccFilters();
	InitImu();
	//InitKalman();
}

void InlineInitGyroFilters(void)
{
	//first time init
	int32_t axis;

	geeForceZ = 0.0f;
	bzero(&geeForceZFilter, sizeof(geeForceZFilter));
	//here
	InitBiquad(50, &geeForceZFilter, loopSpeed.accdT, FILTER_TYPE_LOWPASS, &geeForceZFilter, 1.0f);	

	for (axis = 2; axis >= 0; --axis)
	{
		if ( mainConfig.tuneProfile[activeProfile].filterConfig[0].filterType == 0 )
		{
			OldInitPaf( &pafGyroStates[axis], gyroFiltUsed[axis], mainConfig.tuneProfile[activeProfile].filterConfig[axis].gyro.r, 0.0f, filteredGyroData[axis]);
		}
		else 
		{
			InitPaf( &pafGyroStates[axis], mainConfig.tuneProfile[activeProfile].filterConfig[axis].gyro.q, mainConfig.tuneProfile[activeProfile].filterConfig[axis].gyro.r, 0.0f, filteredGyroData[axis]);
		}

		InitBiquad(240, &lpfFilterState[axis], loopSpeed.gyrodT, FILTER_TYPE_LOWPASS, &lpfFilterState[axis], 1.98f);		

	}

}

void InlineInitKdFilters(void)
{

	//int32_t axis;

	//for (axis = 2; axis >= 0; --axis)
	//	InitBiquad(kdFiltUsed[axis], &lpfFilterStateKd[axis], loopSpeed.gyrodT, FILTER_TYPE_LOWPASS, &lpfFilterStateKd[axis], 1.99f);

}

void InlineInitSpectrumNoiseFilter(void)
{

	//InitBiquad(075, &lpfFilterStateNoise[0], loopSpeed.accdT, FILTER_TYPE_PEEK, &lpfFilterStateNoise[0], 0.33333333333f);
	//InitBiquad(125, &lpfFilterStateNoise[1], loopSpeed.accdT, FILTER_TYPE_PEEK, &lpfFilterStateNoise[1], 0.20000000000f);
	//InitBiquad(175, &lpfFilterStateNoise[2], loopSpeed.accdT, FILTER_TYPE_PEEK, &lpfFilterStateNoise[2], 0.14285714292f);
	//InitBiquad(225, &lpfFilterStateNoise[3], loopSpeed.accdT, FILTER_TYPE_PEEK, &lpfFilterStateNoise[3], 0.11111111111f);
	//InitBiquad(275, &lpfFilterStateNoise[4], loopSpeed.accdT, FILTER_TYPE_PEEK, &lpfFilterStateNoise[4], 0.09090909091f);
	//InitBiquad(325, &lpfFilterStateNoise[5], loopSpeed.accdT, FILTER_TYPE_PEEK, &lpfFilterStateNoise[5], 0.07692307692f);

}

void InlineInitAccFilters(void)
{

	int32_t vector;

	for (vector = 2; vector >= 0; --vector)
		InitBiquad(mainConfig.tuneProfile[activeProfile].filterConfig[vector].acc.r, &lpfFilterStateAcc[vector], loopSpeed.accdT, FILTER_TYPE_LOWPASS, &lpfFilterStateAcc[vector], mainConfig.tuneProfile[activeProfile].filterConfig[vector].acc.q);

}

void InlineUpdateAttitude(float geeForceAccArray[])
{

	//update gyro filter
	//PafUpdate(&pafAccStates[ACCX], geeForceAccArray[ACCX]);
	//PafUpdate(&pafAccStates[ACCY], geeForceAccArray[ACCY]);
	//PafUpdate(&pafAccStates[ACCZ], geeForceAccArray[ACCZ]);

	//filteredAccData[ACCX] = pafAccStates[ACCX].output;
	//filteredAccData[ACCY] = pafAccStates[ACCY].output;
	//filteredAccData[ACCZ] = pafAccStates[ACCZ].output;

	//if (!boardArmed)
	//{
		//inlineDigitalHi(ports[ENUM_PORTB], GPIO_PIN_1);
		//filteredAccData[ACCX] = BiquadUpdate(geeForceAccArray[ACCX], &lpfFilterStateAcc[ACCX]);
		//filteredAccData[ACCY] = BiquadUpdate(geeForceAccArray[ACCY], &lpfFilterStateAcc[ACCY]);
		//filteredAccDataXACCZ] = BiquadUpdate(geeForceAccArray[ACCZ], &lpfFilterStateAcc[ACCZ]);
	filteredAccData[ACCX] = geeForceAccArray[ACCX];
	filteredAccData[ACCY] = geeForceAccArray[ACCY];
	filteredAccData[ACCZ] = geeForceAccArray[ACCZ];
		//inlineDigitalLo(ports[ENUM_PORTB], GPIO_PIN_1);
	//}

	//accNoise[0] = BiquadUpdate(geeForceAccArray[ACCZ], &lpfFilterStateNoise[0]);
	//accNoise[1] = BiquadUpdate(geeForceAccArray[ACCZ], &lpfFilterStateNoise[1]);
	//accNoise[2] = BiquadUpdate(geeForceAccArray[ACCZ], &lpfFilterStateNoise[2]);
	//accNoise[3] = BiquadUpdate(geeForceAccArray[ACCZ], &lpfFilterStateNoise[3]);
	//accNoise[4] = BiquadUpdate(geeForceAccArray[ACCZ], &lpfFilterStateNoise[4]);
	//accNoise[5] = BiquadUpdate(geeForceAccArray[ACCZ], &lpfFilterStateNoise[5]);

}

void InlineFlightCode(float dpsGyroArray[])
{

	//used for IMU
	static float gyroAdder[3] = {0.0f,0.0f,0.0f};
	static uint32_t gyroAverager = 0;

	static uint32_t gyroStdDeviationLatch = 0;
	int32_t axis;
	volatile float averagedGyro;

//	inlineDigitalHi(ports[ENUM_PORTB], GPIO_PIN_0);

	//Gyro routine:
	//gyro interrupts
	//gyro read using DMA
	//updateGyro is called after the read is complete.
	//gyro and board rotation is applied to gyro data in updateGyro
	//updateGyro will call InlineFlightCode.

	//InlineFlightCode:
	//gyro filter applied
	//rc smoothing and acro+ is applied to arrive at setPoint. actuatorRange is used to limit setPoint.
	//pid controller is run using setpoint and smoothed gyro data
	//mixer is applied and outputs it's status as actuatorRange
	//output to motors

	//SKIP_GYRO = 1;
	if (SKIP_GYRO)
	{
		FeedTheDog();
		ledStatus.status = LEDS_FAST_BLINK;
		return;
	}

	//if (!ModeActive(M_GLUE) || !boardArmed) //if not M_GLUE mode or armed we reset the requesteds values.
	//{
	//	ImuResetCommandQuat();
	//}
	if (!boardArmed) //if not M_GLUE mode or armed we reset the requesteds values.
	{
		ImuResetCommandQuat();
	}

	//update gyro filter, every time there's an interrupt
	for (axis = 2; axis >= 0; --axis)
	{

		/* this is now optimized for speed, preference is always what should be checked first since its most likely outcome */
		//here
//		filteredGyroData[axis] = BiquadUpdate(filteredGyroData[axis], &lpfFilterState[axis]);
		
		if  (mainConfig.tuneProfile[activeProfile].filterConfig[0].filterType == 1)
		{
			PafUpdate(&pafGyroStates[axis], dpsGyroArray[axis] );
			//here
			filteredGyroData[axis] = BiquadUpdate((float)pafGyroStates[axis].x, &lpfFilterState[axis]);
		}
	
		else
		{
//			averagedGyro = AverageGyroADCbuffer(axis, dpsGyroArray[axis]);
			OldPafUpdate(&pafGyroStates[axis], AverageGyroADCbuffer(axis, dpsGyroArray[axis]));
			filteredGyroData[axis] = BiquadUpdate((float)pafGyroStates[axis].output, &lpfFilterState[axis]);			
//			filteredGyroData[axis] = (float)pafGyroStates[axis].output;
				
		}
	

	}

	if (gyroFlightLoopCounter-- <= 0) // code triggers every time, or every 4 time etc, depending on esc output
	{

		static int failTimes[4] = {0,};

		gyroFlightLoopCounter = loopSpeed.gyroDivider; // when sync'd its 0, otherwise, its how often you run this

		//smooth the rx data between rx signals
		InlineRcSmoothing(curvedRcCommandF, smoothedRcCommandF);
		
		//1st, find request rates regardless of modes
		flightSetPoints[YAW]   = InlineGetSetPoint(smoothedRcCommandF[YAW], mainConfig.tuneProfile[activeProfile].rcRates.useCurve, mainConfig.tuneProfile[activeProfile].rcRates.rates[YAW], mainConfig.tuneProfile[activeProfile].rcRates.acroPlus[YAW] * 0.01, YAW); //yaw is backwards for some reason
		flightSetPoints[ROLL]  = InlineGetSetPoint(smoothedRcCommandF[ROLL], mainConfig.tuneProfile[activeProfile].rcRates.useCurve, mainConfig.tuneProfile[activeProfile].rcRates.rates[ROLL], mainConfig.tuneProfile[activeProfile].rcRates.acroPlus[ROLL] * 0.01, ROLL);
		flightSetPoints[PITCH] = -InlineGetSetPoint(smoothedRcCommandF[PITCH], mainConfig.tuneProfile[activeProfile].rcRates.useCurve, mainConfig.tuneProfile[activeProfile].rcRates.rates[PITCH], mainConfig.tuneProfile[activeProfile].rcRates.acroPlus[PITCH] * 0.01, PITCH);

		//get setpoint for PIDC for self level modes.
		//TODO: move these to its own function in the IMU
		//if (ModeActive(M_CATMODE))



		/* this is terrible, we can't run these modes all at once, so we should have one variable set to define them */
		if (ModeActive(M_ATTITUDE) || ModeActive(M_HORIZON)  ||
			(
				(quopaState == QUOPA_ACTIVE) &&
				(
					(mainConfig.mixerConfig.quopaStyle == QUOPA_STYLE_AUTO)       ||
					(mainConfig.mixerConfig.quopaStyle == QUOPA_STYLE_SEMIAUTO)   ||
					(mainConfig.mixerConfig.quopaStyle == QUOPA_STYLE_AUTO_2)     ||
					(mainConfig.mixerConfig.quopaStyle == QUOPA_STYLE_SEMIAUTO_2)
				)
			)
		) //we're in a self level mode, let's find the set point based on angle of sticks and angle of craft
		{
			if (!timeSinceSelfLevelActivated)
				timeSinceSelfLevelActivated = InlineMillis();

			//todo move this check to the 1khz section
			//slowly ramp up self leveling over 100 milliseconds
			if ((InlineMillis() - timeSinceSelfLevelActivated) < 300)
			{
				slpUsed = mainConfig.tuneProfile[activeProfile].pidConfig[PITCH].slp * (float)(InlineMillis() - timeSinceSelfLevelActivated) * 0.003333f;
				sliUsed = mainConfig.tuneProfile[activeProfile].pidConfig[PITCH].sli * (float)(InlineMillis() - timeSinceSelfLevelActivated) * 0.003333f;
				sldUsed = mainConfig.tuneProfile[activeProfile].pidConfig[PITCH].sld * (float)(InlineMillis() - timeSinceSelfLevelActivated) * 0.003333f;
			}
			else
			{
				slpUsed = mainConfig.tuneProfile[activeProfile].pidConfig[PITCH].slp;
				sliUsed = mainConfig.tuneProfile[activeProfile].pidConfig[PITCH].sli;
				sldUsed = mainConfig.tuneProfile[activeProfile].pidConfig[PITCH].sld;
			}

			rollAttitudeError        = ( (trueRcCommandF[ROLL]  *  mainConfig.tuneProfile[activeProfile].pidConfig[PITCH].sla ) - rollAttitude  );
			pitchAttitudeError       = ( (trueRcCommandF[PITCH] * -mainConfig.tuneProfile[activeProfile].pidConfig[PITCH].sla ) - pitchAttitude );

			rollAttitudeErrorKi      = (rollAttitudeErrorKi  + rollAttitudeError  * sliUsed * loopSpeed.truedT);
			pitchAttitudeErrorKi     = (pitchAttitudeErrorKi + pitchAttitudeError * sliUsed * loopSpeed.truedT);

			rollAttitudeErrorKdelta  = -(rollAttitudeError  - lastRollAttitudeError);
			lastRollAttitudeError    = rollAttitudeError;
			pitchAttitudeErrorKdelta = -(pitchAttitudeError - lastPitchAttitudeError);
			lastPitchAttitudeError   = pitchAttitudeError;

			if (ModeActive(M_ATTITUDE) ||
			   (
					quopaState == QUOPA_ACTIVE &&
					(mainConfig.mixerConfig.quopaStyle == QUOPA_STYLE_AUTO || mainConfig.mixerConfig.quopaStyle == QUOPA_STYLE_AUTO_2 )
				)
			) //if M_ATTITUDE mode
			{
				//roll and pitch are set directly from self level mode
				flightSetPoints[ROLL]    = InlineConstrainf( (rollAttitudeError * slpUsed) + rollAttitudeErrorKi + (rollAttitudeErrorKdelta / loopSpeed.truedT * sldUsed), -300.0, 300.0);
				flightSetPoints[PITCH]   = InlineConstrainf( (pitchAttitudeError * slpUsed) + pitchAttitudeErrorKi + (pitchAttitudeErrorKdelta / loopSpeed.truedT * sldUsed), -300.0, 300.0);
			}
			else if (
				ModeActive(M_HORIZON) ||
				(
					quopaState == QUOPA_ACTIVE &&
					(mainConfig.mixerConfig.quopaStyle == QUOPA_STYLE_SEMIAUTO || mainConfig.mixerConfig.quopaStyle == QUOPA_STYLE_SEMIAUTO_2 )
				)
			
			) //if M_HORIZON mode
			{
				//roll and pitch and modified by stick angle proportionally to stick angle
				if ( (ABS(trueRcCommandF[PITCH]) < 0.75f) && (ABS(trueRcCommandF[ROLL]) < 0.75f) && ABS(pitchAttitude) < 75.0f ) //prevent gimbal lock since PIDc uses euler angles
				{
					flightSetPoints[ROLL]    += InlineConstrainf( (rollAttitudeError * slpUsed) + rollAttitudeErrorKi + (rollAttitudeErrorKdelta / loopSpeed.truedT * sldUsed), -300.0f, 300.0f) * (1.0f - ABS(trueRcCommandF[ROLL]) );
				}
				if ( (ABS(trueRcCommandF[PITCH]) < 0.75f)  && (ABS(trueRcCommandF[ROLL]) < 0.75f) )
					flightSetPoints[PITCH]   += InlineConstrainf( (pitchAttitudeError * slpUsed) + pitchAttitudeErrorKi + (pitchAttitudeErrorKdelta / loopSpeed.truedT * sldUsed), -300.0f, 300.0f) * (1.0f - ABS(trueRcCommandF[PITCH]) );
			}
		}
		else //no auto level modes active, we're in rate mode
		{
			if (timeSinceSelfLevelActivated)
				timeSinceSelfLevelActivated = 0;
		}

		if(fullKiLatched)
		{
			kiTrim[YAW]   = ApplyLearningModelToKi(smoothCurvedThrottle0_1, YAW);
			kiTrim[ROLL]  = ApplyLearningModelToKi(smoothCurvedThrottle0_1, ROLL);
			kiTrim[PITCH] = ApplyLearningModelToKi(smoothCurvedThrottle0_1, PITCH);
		}
		else
		{
			kiTrim[YAW]   = 0;
			kiTrim[ROLL]  = 0;
			kiTrim[PITCH] = 0;
		}

		//Run PIDC
		InlinePidController(filteredGyroData, flightSetPoints, flightPids, actuatorRange);

		if ( boardArmed || PreArmFilterCheck )
		{
			if (gyroCalibrationCycles != 0)
			{
			   return;
			}
			else if (!gyroStdDeviationLatch)
			{
				gyroStdDeviationLatch = 1;
				PreArmFilterCheck = 0;
				return;
			}
			/*
			//todo: move to a single mixer. This is a mess
			if (threeDeeMode)
			{
				static uint32_t actuatorToUse = 0;
				if (smoothCurvedThrottle0_1 > 0.6) //throttle above neutral latch
				{
					actuatorToUse = 1;
				}
				else if (smoothCurvedThrottle0_1 <= 0.4) //throttle below neutral latch
				{
					actuatorToUse = 2;
				}

				if (actuatorToUse == 2)
				{
					actuatorRange = InlineApplyMotorMixer3dInverted(flightPids, CONSTRAIN(smoothCurvedThrottle0_1, 0.0f, 0.45f)); //put in PIDs and Throttle or passthru
				}
				else
				{
					actuatorRange = InlineApplyMotorMixer3dUpright(flightPids, CONSTRAIN(smoothCurvedThrottle0_1, 0.55f, 1.0f)); //put in PIDs and Throttle or passthru
				}

			}
			else
			{
				*/

				//volatile float throttleToUse = BoostModify(smoothCurvedThrottle0_1);
				if (mainConfig.mixerConfig.mixerStyle == 1) //race mixer
					actuatorRange = InlineApplyMotorMixer1(flightPids, BoostModify(smoothCurvedThrottle0_1)); //put in PIDs and Throttle or passthru
				else //freestyle mixer
					actuatorRange = InlineApplyMotorMixer(flightPids, BoostModify(smoothCurvedThrottle0_1)); //put in PIDs and Throttle or passthru
	
	/*
				}
*/
		}
		else
		{
			//otherwise we keep Ki zeroed.
			flightPids[YAW].ki   = 0;
			flightPids[ROLL].ki  = 0;
			flightPids[PITCH].ki = 0;
			yawAttitudeErrorKi   = 0;
			rollAttitudeErrorKi  = 0;
			pitchAttitudeErrorKi = 0;
			fullKiLatched        = 0;
		}

		//output to actuators. This is the end of the flight code for this iteration.
		OutputActuators(motorOutput, servoOutput);


		//this code is less important that stabilization, so it happens AFTER stabilization.
		//Anything changed here can happen after the next iteration without any drawbacks. Stabilization above all else!

		/*
#ifdef LOG32
		//update blackbox here
		UpdateBlackbox(flightPids, flightSetPoints, dpsGyroArray, filteredGyroData, geeForceAccArray);
#endif
*/


		switch (khzLoopCounterPhase++)
		{
			case 6: 
				//here2 
				UpdateBlackbox(flightPids, flightSetPoints, dpsGyroArray, filteredGyroData, geeForceAccArray);
				khzLoopCounterPhase = 0;
				/*
				#ifndef LOG32
				//update blackbox here
					UpdateBlackbox(flightPids, flightSetPoints, dpsGyroArray, filteredGyroData, geeForceAccArray);
				#endif
				*/
			break;
			
			case 5: 
			break;
			
			case 4: 
				if(armedTime > 3000)
					TrimKi(flightPids);			
			break;
			
			case 3: 
			break;
			
			case 2: 
				TrimMotors();						
			break;

			case 1: 
			break;

			case 0: 
			break;
			
		}


		if (khzLoopCounter-- == 0)
		{

			static int everyTenth = 0;
			
			khzLoopCounterPhase = 0;

			everyTenth++;
			if(everyTenth == 10)
			{
				CalculateThrottleVelocity();
				everyTenth=0;
			}

			//runs at 1KHz or loopspeed, whatever is slower
			khzLoopCounter=loopSpeed.khzDivider;

			CheckFailsafe();
			if(boardArmed)
			{
				armedTime++; //armed time in ms
				armedTimeSincePower++;
			}
			else
			{
				armedTime = 0;
			}

			if( (quopaState == QUOPA_ACTIVE) && armedTime > QUOPA_FLIGHT_LIMIT )
			{
				DisarmBoard();
			}

			//check for fullKiLatched here
			if ( (boardArmed) && (smoothCurvedThrottle0_1 > 0.15f) )
			{
				fullKiLatched = 1;
			}

			if (RfblDisasterPreventionCheck)
			{
				if (InlineMillis() > 5000)
				{
					RfblDisasterPreventionCheck = 0;
					HandleRfblDisasterPrevention();
				}
			}

		}

	}

	if(usedSkunk == 2)
		return;

	gyroAdder[ROLL]  += filteredGyroData[ROLL];
	gyroAdder[PITCH] += filteredGyroData[PITCH];
	gyroAdder[YAW]   += filteredGyroData[YAW];

	geeForceZ = BiquadUpdate(filteredAccData[ACCZ], &geeForceZFilter);

	if (gyroAverager++ ==  loopSpeed.gyroAccDiv)
	{
		gyroAverager = 0;

		UpdateImu(filteredAccData[ACCX], filteredAccData[ACCY], filteredAccData[ACCZ], gyroAdder[ROLL], gyroAdder[PITCH], gyroAdder[YAW]);

		gyroAdder[ROLL]  = 0.0f;
		gyroAdder[PITCH] = 0.0f;
		gyroAdder[YAW]   = 0.0f;
	}
	//inlineDigitalLo(ports[ENUM_PORTB], GPIO_PIN_0);

}

//return setpoint in degrees per second, this is after stick smoothing
float InlineGetSetPoint(float rcCommandF, uint32_t curveToUse, float rates, float acroPlus, uint32_t axis)
{
	float returnValue;

	//betaflop and kiss set points are calculated in rx.c during curve calculation
	switch(curveToUse)
	{
		case ACRO_PLUS:
			returnValue = (rcCommandF * ( rates + ( ABS(rcCommandF) * rates * acroPlus ) ) );
			break;
		case BETAFLOP_EXPO:
			returnValue = (rcCommandF * maxFlopRate[axis]);
			break;
		case KISS_EXPO:
		case KISS_EXPO2:
			returnValue = (rcCommandF * maxKissRate[axis]);
			break;
		case TARANIS_EXPO:
		case SKITZO_EXPO:
		case FAST_EXPO:
		case NO_EXPO:
		default:
			returnValue = (rcCommandF * (rates + (rates * acroPlus) ) );
			break;

	}

	if (deviceWhoAmI == ICM20601_WHO_AM_I)
		returnValue = InlineConstrainf(returnValue,-3600,3600);
	else
		returnValue = InlineConstrainf(returnValue,-1800,1800);
	
	return (returnValue);
}




//these functions really don't belong here



static void InitVbusSensing(void);
static uint32_t IsUsbConnected(void);

static void InitVbusSensing(void)
{
#ifdef VBUS_SENSING
    GPIO_InitTypeDef GPIO_InitStructure;

    HAL_GPIO_DeInit(ports[VBUS_SENSING_GPIO], VBUS_SENSING_PIN);

    GPIO_InitStructure.Pin   = VBUS_SENSING_PIN;
    GPIO_InitStructure.Mode  = GPIO_MODE_INPUT;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStructure.Pull  = GPIO_NOPULL;
    HAL_GPIO_Init(ports[VBUS_SENSING_GPIO], &GPIO_InitStructure);
#endif
}

static uint32_t IsUsbConnected(void)
{
#ifdef VBUS_SENSING
	return(!inlineIsPinStatusHi(ports[VBUS_SENSING_GPIO], VBUS_SENSING_PIN));
#else
	return(0);
#endif

}

void DeinitFlight(void)
{
	DisarmBoard();              //sets WD to 32S
	AccGyroDeinit();            //takes about 200ms maybe to run, this will also stop the flight code from running so no reason to stop that.
	DeInitBoardUsarts();        //deinit all the USARTs.
	DeInitActuators();          //deinit all the Actuators.
	DeInitAllowedSoftOutputs(); //deinit all the soft outputs
}

#ifdef STM32F446xx
uint32_t used1Wire = 0;
#endif
//init the board

int InitFlight(uint32_t escProtocol, uint32_t escFrequency)
{

	volatile int retValChk;
	uint32_t loopUsed;

    //TODO: move the check into the init functions.
	InitOrientation();
	CheckRxToModes(); //check which modes are set whether or not they're enabled

	retValChk = InitSoftPwm(); //initiial init

	loopUsed = SanityCheckEscProtocolAndFrequency(&escProtocol, &escFrequency);

	DeInitAllowedSoftOutputs();

	retValChk = LearningInit();

	if (board.flash[0].enabled)
    {
		//persistance data stored here
    	InitFlashChip();
		InitFlightLogger();
		retValChk = LearningModelInit(); //needs flash
	}
	retValChk = InitMaxOsd();
	
    InitVbusSensing();
    InitRcData();
    InitMixer();              //init mixers
    InitFlightCode(loopUsed); //flight code before PID code is a must since flight.c contains loop time settings the pid.c uses.
    InitPid();                //Relies on InitFlightCode for proper activations.
    //InitActuators();        //Actuator init should happen after soft serial init.
    //ZeroActuators(1000);    //output actuators to idle after timers are stable;

	InitAdc();                //init ADC functions
    InitModes();              //set flight modes mask to zero.
    InitBoardUsarts();        //most important thing is activated last, the ability to control the craft.


#ifdef STM32F446xx
	if (used1Wire == 0)
	{
#endif
		if (AccGyroInit(mainConfig.gyroConfig.loopCtrl) < 1)
		{
			ErrorHandler(GYRO_INIT_FAILIURE);
		}
#ifdef STM32F446xx
	}
#endif

	//make sure gyro is interrupting and init scheduler
	InitScheduler();

#ifdef STM32F446xx
	InitLaptimer();
#endif

	if ( mainConfig.ledConfig.ledOnWithUsb )
	{
		retValChk = InitWs2812();
	}
	else if( !IsUsbConnected() )
	{
		retValChk = InitWs2812();
	}

    DeInitActuators();    //Deinit before Init is a shotgun startup
	InitActuators(escProtocol, escFrequency); //Actuator init should happen after soft serial init.
    ZeroActuators(500);     //output actuators to idle after timers are stable;

	//init telemtry, if there's a gyro EXTI and soft serial collision then the fake EXTI will be used in place of the actual gyro EXTI
	InitTelemtry();

	//InitTransponderTimer();
	DelayMs(2);

	if(retValChk)
		retValChk = retValChk;

	return(0);
}

static float BoostModify(volatile float throttleIn)
{
	volatile float throttleOut;
	//adcCurrent; //current current consumption
	//adcMAh;     //current MAh used
	throttleOut = CONSTRAIN(throttleIn, 0.0f, 1.0f);
	return( throttleOut );
}

uint32_t SanityCheckEscProtocolAndFrequency(uint32_t *escProtocol, uint32_t *escFrequency)
{
	uint32_t loopUsed = mainConfig.gyroConfig.loopCtrl;
	int validLoopConfig = 0;
	uint32_t finalEscFrequency = (*escFrequency);
	if ( (!validLoopConfig) && ((*escFrequency) >= 32000) )
	{
		finalEscFrequency = 32000;
		//mainConfig.gyroConfig.loopCtrl            = LOOP_UH32;
		loopUsed = LOOP_UH32;
		if ( ((*escProtocol) == ESC_MULTISHOT) || ((*escProtocol) == ESC_MULTISHOT25)  || ((*escProtocol) == ESC_MULTISHOT125) )
		{
			validLoopConfig = 1;
		}
	}
	if ( (!validLoopConfig) && ((*escFrequency) >= 16000) )
	{
		finalEscFrequency = 16000; 
		loopUsed = LOOP_UH16;
		if ( ((*escProtocol) == ESC_DSHOT600) || ((*escProtocol) == ESC_MULTISHOT)  || ((*escProtocol) == ESC_MULTISHOT25)  || ((*escProtocol) == ESC_MULTISHOT125) )
		{
			validLoopConfig = 1;
		}
	}
	if ( (!validLoopConfig) && ((*escFrequency) >= 8000) )
	{
		finalEscFrequency = 8000;
		loopUsed = LOOP_UH8;
		if ( ((*escProtocol) == ESC_DDSHOT) || ((*escProtocol) == ESC_ONESHOT42) || ((*escProtocol) == ESC_DSHOT300) || ((*escProtocol) == ESC_DSHOT600) || ((*escProtocol) == ESC_DSHOT1200) || ((*escProtocol) == ESC_MULTISHOT)  || ((*escProtocol) == ESC_MULTISHOT25)  || ((*escProtocol) == ESC_MULTISHOT125) )
		{
			validLoopConfig = 1;
			if ((*escProtocol) == ESC_DSHOT600)
			{
				if ((*escFrequency) > 16000)
				{
					(*escFrequency) = 16000;
				}
			}
		}
	}
	if ( (!validLoopConfig) && ((*escFrequency) >= 4000) )
	{
		finalEscFrequency = 4000;
		loopUsed = LOOP_UH4;
		if ( ((*escProtocol) == ESC_DDSHOT) || ((*escProtocol) == ESC_ONESHOT) || ((*escProtocol) == ESC_ONESHOT42) || ((*escProtocol) == ESC_DSHOT150) || ((*escProtocol) == ESC_DSHOT300) || ((*escProtocol) == ESC_DSHOT600) || ((*escProtocol) == ESC_DSHOT1200) || ((*escProtocol) == ESC_MULTISHOT)  || ((*escProtocol) == ESC_MULTISHOT25)  || ((*escProtocol) == ESC_MULTISHOT125) )
		{
			validLoopConfig = 1;
		}
	}
	if ( (!validLoopConfig) && ((*escFrequency) >= 2000) )
	{
		finalEscFrequency = 2000;
		loopUsed = LOOP_UH2;
		if ( ((*escProtocol) == ESC_DDSHOT) || ((*escProtocol) == ESC_ONESHOT) || ((*escProtocol) == ESC_ONESHOT42) || ((*escProtocol) == ESC_DSHOT150) || ((*escProtocol) == ESC_DSHOT300) || ((*escProtocol) == ESC_DSHOT600) || ((*escProtocol) == ESC_DSHOT1200) || ((*escProtocol) == ESC_MULTISHOT)  || ((*escProtocol) == ESC_MULTISHOT25)  || ((*escProtocol) == ESC_MULTISHOT125) )
		{
			validLoopConfig = 1;
		}
	}
	if ( (!validLoopConfig) && ((*escFrequency) >= 1000) )
	{
		finalEscFrequency = 1000;
		loopUsed = LOOP_UH1;
		if ( ((*escProtocol) == ESC_DDSHOT) || ((*escProtocol) == ESC_ONESHOT) || ((*escProtocol) == ESC_ONESHOT42) || ((*escProtocol) == ESC_DSHOT150) || ((*escProtocol) == ESC_DSHOT300) || ((*escProtocol) == ESC_DSHOT600) || ((*escProtocol) == ESC_DSHOT1200) || ((*escProtocol) == ESC_MULTISHOT)  || ((*escProtocol) == ESC_MULTISHOT25)  || ((*escProtocol) == ESC_MULTISHOT125) )
		{
			validLoopConfig = 1;
		}
	}
	if ( (!validLoopConfig) && ((*escFrequency) >= 500) )
	{
		finalEscFrequency = 500;
		loopUsed = LOOP_UH_500;
		if ( ((*escProtocol) == ESC_DDSHOT) || ((*escProtocol) == ESC_PWM) || ((*escProtocol) == ESC_ONESHOT) || ((*escProtocol) == ESC_ONESHOT42) || ((*escProtocol) == ESC_DSHOT150) || ((*escProtocol) == ESC_DSHOT300) || ((*escProtocol) == ESC_DSHOT600) || ((*escProtocol) == ESC_DSHOT1200) || ((*escProtocol) == ESC_MULTISHOT)  || ((*escProtocol) == ESC_MULTISHOT25)  || ((*escProtocol) == ESC_MULTISHOT125) )
		{
			validLoopConfig = 1;
		}
	}
	if ( (!validLoopConfig) && ((*escFrequency) >= 250) )
	{
		finalEscFrequency = 250;
		loopUsed = LOOP_UH_250;
		if ( ((*escProtocol) == ESC_DDSHOT) || ((*escProtocol) == ESC_PWM) || ((*escProtocol) == ESC_ONESHOT) || ((*escProtocol) == ESC_ONESHOT42) || ((*escProtocol) == ESC_DSHOT150) || ((*escProtocol) == ESC_DSHOT300) || ((*escProtocol) == ESC_DSHOT600) || ((*escProtocol) == ESC_DSHOT1200) || ((*escProtocol) == ESC_MULTISHOT)  || ((*escProtocol) == ESC_MULTISHOT25)  || ((*escProtocol) == ESC_MULTISHOT125) )
		{
			validLoopConfig = 1;
		}
	}
	if ( (!validLoopConfig) && ((*escFrequency) >= 62) )
	{
		finalEscFrequency = 62;
		loopUsed = LOOP_UH_062;
		if ( ((*escProtocol) == ESC_DDSHOT) || ((*escProtocol) == ESC_PWM) || ((*escProtocol) == ESC_ONESHOT) || ((*escProtocol) == ESC_ONESHOT42) || ((*escProtocol) == ESC_DSHOT150) || ((*escProtocol) == ESC_DSHOT300) || ((*escProtocol) == ESC_DSHOT600) || ((*escProtocol) == ESC_DSHOT1200) || ((*escProtocol) == ESC_MULTISHOT)  || ((*escProtocol) == ESC_MULTISHOT25)  || ((*escProtocol) == ESC_MULTISHOT125) )
		{
			validLoopConfig = 1;
		}
	}

	(*escFrequency) = finalEscFrequency;

	usedSkunk = mainConfig.gyroConfig.skunk;
	#ifdef DEBUG
		usedSkunk =  0;
	#endif
	if (!FULL_32)
	{
		//16 KHz on F4s if quaternions are needed.
		if ( 
			//(mainConfig.mixerConfig.foreAftMixerFixer == 2) ||
			ModeSet(M_ATTITUDE) ||
			ModeSet(M_HORIZON)  ||
			ModeSet(M_QUOPA)
			//ModeSet(M_HORIZON)  || 
			//ModeSet(M_GLUE)     ||
			//ModeSet(M_CATMODE)
		)
		{
			//set skunk to 0 which is 16 KHz w/ACC if ACC mode is needed
			usedSkunk = 0; 
		}
	}

	//skunk below 0.5f will set 32 KHz to 16 KHz operation
	if(usedSkunk == 0)
	{
		//true 32Khz
		if (loopUsed == LOOP_UH32)
		{
			loopUsed = LOOP_UH16;
		}
		else if (loopUsed == LOOP_H32)
		{
			loopUsed = LOOP_H16;
		}

	}
	
	return(loopUsed);
}