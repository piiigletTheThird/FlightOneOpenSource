#include "includes.h"


uint32_t checkRxData[MAXCHANNELS];
uint32_t inputChannelMapped[MAXCHANNELS];
rc_control_config tempRc;
wizard_record wizardStatus;

static void     ResetChannelCheck(void);
static uint32_t CheckSafeMotors(uint32_t time, uint32_t deviationAllowed);
static void     PrepareRcWizard(void);
static void     WizRcCheckMinMax(void);
static void     WizRcCheckCenter(void);
static void     WizRcCheckAndSendDirection(void);
static int32_t  WizRcWhichInChannelChange(void);
static int32_t  WizRcSetChannelMapAndDirection(uint32_t inChannel, uint32_t outChannel);
static void     WizRcSetRestOfMap(void);
static void     HandleWizRx(void);
static uint32_t WizRxCheckRxDataLooksValid(void);
static void     WizRcCheckArmSwitchDisarmed(void);
static uint32_t WizRcCheckArmSwitchArmed(void);

static void ResetChannelCheck(void)
{
	memcpy(checkRxData, rxDataRaw, sizeof(checkRxData));
}


//Using standard deviation of ACC to make sure motors aren't running.
static uint32_t CheckSafeMotors(uint32_t time, uint32_t deviationAllowed)
{

	float stdDeviation[10];
	bzero(stdDeviation, sizeof(stdDeviation));
	uint32_t simpleCouter = 0;

	float strdDeviationCheck;

	time = time / 10;

	for (uint32_t timeCounter = 0;timeCounter < time;timeCounter++)
	{

		for (simpleCouter=0;simpleCouter < 10;simpleCouter++)
		{
			stdDeviation[simpleCouter]   = ABS(geeForceAccArray[ACCZ]);
			DelayMs(1);
		}
		FeedTheDog();
		simpleCouter = 0;
		strdDeviationCheck = CalculateSD(stdDeviation) * 100000;

		if (strdDeviationCheck > deviationAllowed)
		{
			calibrateMotors = 0;
			DisarmBoard();
			ZeroActuators(32000);
			return(0);
		}

	}
	return(1);
}

void MixerWizard(char *inString)
{

	if (!strcmp("mixera", inString))
	{
		DisarmBoard();
		SKIP_GYRO=1;
		RfCustomReplyBuffer("spinningmotor1\n");
		DelayMs(10);
		IdleActuator(0);
		DelayMs(10);
	}
	else if (!strcmp("mixerb", inString))
	{
		DisarmBoard();
		SKIP_GYRO=0;
	}
	else if (!strcmp("mixerc", inString))
	{
		DisarmBoard();
		SKIP_GYRO=0;
	}
}

void WizRcSetRestOfMap(void)
{

	uint32_t x;
	uint32_t y;

	uint32_t alreadyMapped[MAXCHANNELS];

	bzero(alreadyMapped, sizeof(alreadyMapped));

	//channels that are already mapped get set to 1 in this array
	for (x = 0;x<MAXCHANNELS;x++)
	{
		if (mainConfig.rcControlsConfig.channelMap[x] < MAXCHANNELS)
			alreadyMapped[mainConfig.rcControlsConfig.channelMap[x]]=1;
	}

	for (x = 0;x<MAXCHANNELS;x++)
	{
		if (alreadyMapped[x] == 0) //x is not assigned to a channel so we assign it to the first channel mapped to 1000
		{
			for (y = 0;y<MAXCHANNELS;y++)
			{
				if (mainConfig.rcControlsConfig.channelMap[y] == 1000) //this channel is not assigned a map, so we assign it X and set min max to the mapped channel 0
				{
					mainConfig.rcControlsConfig.channelMap[y] = x;
					mainConfig.rcControlsConfig.minRc[y] = mainConfig.rcControlsConfig.minRc[0];
					mainConfig.rcControlsConfig.midRc[y] = mainConfig.rcControlsConfig.midRc[0];
					mainConfig.rcControlsConfig.maxRc[y] = mainConfig.rcControlsConfig.maxRc[0];
					break; //break out of inner loop
				}
			}
		}
	}
}

uint32_t WizRxCheckRxDataLooksValid(void)
{
	uint32_t correct = 0;

	//if processed data is within 5% of center on 3 axis it's safe to assume it's valid data
	if ( (trueRcCommandF[YAW] < 0.1) && (trueRcCommandF[YAW] > - 0.1) )
		correct ++;
	if ( (trueRcCommandF[ROLL] < 0.1) && (trueRcCommandF[ROLL] > - 0.1) )
		correct ++;
	if ( (trueRcCommandF[PITCH] < 0.1) && (trueRcCommandF[PITCH] > - 0.1) )
		correct ++;
	if ( (trueRcCommandF[THROTTLE] < 0.1) && (trueRcCommandF[THROTTLE] > - 0.1) )
		correct ++;

	if (correct > 2)
		return(1);
	else
		return(0);
}


void PrepareRcWizard(void)
{
	//set wizard structure
	wizardStatus.currentWizard = WIZ_RC;
	wizardStatus.currentStep   = 1;
	//reset rcCalibration config
	mainConfig.rcControlsConfig.rcCalibrated = 0;

	bzero(&tempRc, sizeof(rc_control_config));

	//reset channelMap config and prepare tempRc
	for (uint32_t x = 0;x<MAXCHANNELS;x++)
	{
		mainConfig.rcControlsConfig.channelMap[x] = 1000;
		tempRc.maxRc[x] = 0; //high will never be this low so we set it to high
		tempRc.minRc[x] = 1000000; //nothing will go this high so we set it to min
		tempRc.channelMap[x] = 1000; //disable the channel
	}

}

//only returns unassigned channels
//returns channel with largest change that's unassigned
static int32_t WizRcWhichInChannelChange(void)
{

	int32_t inChannelChanged  = -1;
	uint32_t changeValue      = 0;
	float currentChannelRange = 0;
	float percentFromMax      = 0;
	float percentFromMin      = 0;
	volatile float diffFloat  = 0;
	float closestToEndPoint   = .5;

	for (uint32_t x = 0;x<MAXCHANNELS;x++) {

		changeValue = ABS((int32_t)rxDataRaw[x] - (int32_t)checkRxData[x]);

		if ( changeValue > 200 )
		{
			if (inputChannelMapped[x] == 1000)
			{
				//				mainConfig.rcControlsConfig.midRc[x] = tempRc.midRc[0];
				currentChannelRange = ABS((float)tempRc.maxRc[x] - (float)tempRc.minRc[x]); //1000    //0  //1
				diffFloat      = (float)rxDataRaw[x] -  (float)tempRc.maxRc[x];
				percentFromMax = (float)( ABS(diffFloat) / (float)currentChannelRange);
				diffFloat      = (float)rxDataRaw[x] -  (float)tempRc.minRc[x];
				percentFromMin = (float)( ABS(diffFloat) / (float)currentChannelRange);
				if (percentFromMax > percentFromMin)
				{ //we're near min
					if (percentFromMax > closestToEndPoint )
					{ //if current channel is closer to endpoint and it's changed at least 200 points, this is our channel
						closestToEndPoint = percentFromMin;
						//inChannelChanged = x;
					}
				}
				else
				{ //we're closer to max or we're in the middle
					if (percentFromMin > closestToEndPoint )
					{ //if current channel is closer to endpoint and it's changed at least 200 points, this is our channel
						closestToEndPoint = percentFromMax;
						//inChannelChanged = x;
					}
				}
				if (closestToEndPoint < 0.05)
				{ //at least within 5% of endpoint
					inputChannelMapped[x] = 1;
					inChannelChanged = x;
				}

			}

		}

	}

	return(inChannelChanged);

}

void WizRcCheckMinMax(void)
{
	uint32_t x;

	for (x = 0;x<4;x++)
	{
		//for each channel we check if the current value is smaller than the set min value
		if (rxDataRaw[x] < tempRc.minRc[x])
		{
			tempRc.minRc[x] = rxDataRaw[x];
		}

		//for each channel we check if the current value is larger than the set max value
		if (rxDataRaw[x] > tempRc.maxRc[x])
		{
			tempRc.maxRc[x] = rxDataRaw[x];
		}

		//we just brute force set these for later use.
		tempRc.midRc[x] = rxDataRaw[x];
		tempRc.channelMap[x] = 1000;
		inputChannelMapped[x] = 1000;
	}
}

void WizRcCheckCenter(void)
{
	uint32_t x;

	for (x = 0;x<MAXCHANNELS;x++)
	{

		if ( tempRc.maxRc[x] != 1000000 )
		{
			tempRc.midRc[x] = rxDataRaw[x];
			//mainConfig.rcControlsConfig.midRc[x] = tempRc.midRc[x];
			//mainConfig.rcControlsConfig.minRc[x] = tempRc.minRc[x];
			//mainConfig.rcControlsConfig.maxRc[x] = tempRc.maxRc[x];

			//set 0 through 3 are set normally, others re set based on 0 (switches)
			if (x >= 4){
				tempRc.midRc[x] = tempRc.midRc[0];
				tempRc.minRc[x] = tempRc.minRc[0];
				tempRc.maxRc[x] = tempRc.maxRc[0];
			}
			//if ( ( ABS((int32_t)mainConfig.rcControlsConfig.midRc[x] - (int32_t)mainConfig.rcControlsConfig.minRc[x]) < 10 ) )
			//{ //looks like switch
			//	mainConfig.rcControlsConfig.midRc[x] = mainConfig.rcControlsConfig.minRc[x]; //set center to min RC
			//}
			//else if ( ( ABS((int32_t)mainConfig.rcControlsConfig.midRc[x] - (int32_t)mainConfig.rcControlsConfig.maxRc[x]) < 10 ) )
			//{ //looks like switch
			//	mainConfig.rcControlsConfig.midRc[x] = mainConfig.rcControlsConfig.minRc[x]; //set center to min RC
			//}

		}

	}

}

static int32_t WizRcSetChannelMapAndDirection(uint32_t inChannel, uint32_t outChannel)
{

	int32_t channelCheck = ( rxDataRaw[inChannel] < (tempRc.maxRc[inChannel] - 400) ); //channel is reversed

	if (mainConfig.rcControlsConfig.channelMap[outChannel] == 1000)
	{ //if channelMap for the inChannel is 50 than it's waiting to be assigned.

		mainConfig.rcControlsConfig.channelMap[outChannel] = inChannel; //set channel map

		if ( channelCheck )
		{ //min is higher so channel is reversed, reverse if needed
			mainConfig.rcControlsConfig.minRc[outChannel] = tempRc.maxRc[inChannel];
			mainConfig.rcControlsConfig.midRc[outChannel] = tempRc.midRc[inChannel];
			mainConfig.rcControlsConfig.maxRc[outChannel] = tempRc.minRc[inChannel];
		}
		else
		{
			mainConfig.rcControlsConfig.maxRc[outChannel] = tempRc.maxRc[inChannel];
			mainConfig.rcControlsConfig.midRc[outChannel] = tempRc.midRc[inChannel];
			mainConfig.rcControlsConfig.minRc[outChannel] = tempRc.minRc[inChannel];
		}
		return(1);

	}
	return(-1);

}

int32_t WizRcCheckAndSetChannel(uint32_t outChannel)
{
	int32_t changedInChannel = -1;
	int32_t throttleFix = 0;

	changedInChannel = WizRcWhichInChannelChange();

	if (changedInChannel > -1)
	{
		if (WizRcSetChannelMapAndDirection(changedInChannel, outChannel))
		{
			if ( (outChannel == THROTTLE) || (outChannel == AUX1) || (outChannel == AUX2) || (outChannel == AUX3) || (outChannel == AUX4) )
			{ //set mid point at center between extremes
				throttleFix = (int32_t)(((int32_t)mainConfig.rcControlsConfig.maxRc[outChannel] - (int32_t)mainConfig.rcControlsConfig.minRc[outChannel]) / 2);
				mainConfig.rcControlsConfig.midRc[outChannel] = (uint32_t)( throttleFix + (int32_t)mainConfig.rcControlsConfig.minRc[outChannel]);
			}
			return(1);
		}
		else
		{
			return(-1);
		}
	}
	else
	{
		return(-1);
	}
}

void WizRcCheckAndSendDirection(void)
{
	switch(wizardStatus.wicRcCheckDirection)
	{
		case WIZ_RC_THROTTLE_UP:
			if (WizRcCheckAndSetChannel(THROTTLE) > -1)
			{
				RfCustomReplyBuffer("#wiz Throttle Set\n");
				RfCustomReplyBuffer("#wiz Push Yaw Stick To Right\n");
				wizardStatus.wicRcCheckDirection = WIZ_RC_YAW_RIGHT;
			}
			else
			{
				RfCustomReplyBuffer("#wiz Push Throttle Stick To Top\n");
			}
			break;
		case WIZ_RC_YAW_RIGHT:
			if (WizRcCheckAndSetChannel(YAW) > -1)
			{
				RfCustomReplyBuffer("#wiz Yaw Set\n");
				RfCustomReplyBuffer("#wiz Push Pitch Stick To Top\n");
				wizardStatus.wicRcCheckDirection = WIZ_RC_PITCH_UP;
			}
			else
			{
				RfCustomReplyBuffer("#wiz Push Yaw Stick To Right\n");
			}
			break;
		case WIZ_RC_PITCH_UP:
			if (WizRcCheckAndSetChannel(PITCH) > -1)
			{
				RfCustomReplyBuffer("#wiz Pitch Set\n");
				RfCustomReplyBuffer("#wiz Push Roll Stick To Right\n");
				wizardStatus.wicRcCheckDirection = WIZ_RC_ROLL_RIGHT;
			}
			else
			{
				RfCustomReplyBuffer("#wiz Push Pitch Stick To Top\n");
			}
			break;
		case WIZ_RC_ROLL_RIGHT:
			if (WizRcCheckAndSetChannel(ROLL) > -1)
			{
				WizRcSetRestOfMap();
				RfCustomReplyBuffer("#wiz Roll Set\n");
				RfCustomReplyBuffer("#wiz Set arm switch to DISARMED and please run wiz rc4\n");
			}
			else
			{
				RfCustomReplyBuffer("#wiz Set Roll To Right\n");
			}
			break;

	}
}

static void WizRcCheckArmSwitchDisarmed(void)
{
	memcpy(checkRxData, rxData, sizeof(checkRxData));
}

static uint32_t WizRcCheckArmSwitchArmed(void)
{
	uint32_t x;
	for (x=4;x<MAXCHANNELS;x++)
	{
		//we don't look at the first 4 channels as they are sricks
		if ( ABS(rxData[x] - checkRxData[x]) > 300)
		{
			//found the arm switch
			SetMode(M_ARMED, x, (int16_t)(InlineConstrainf((trueRcCommandF[x] - 0.25),-1.0f,1.0f) * 100), (int16_t)(InlineConstrainf((trueRcCommandF[x] + 0.25),-1.0f,1.0f) * 100));
			return(1);
		}
	}
	return(0);
}

void HandleWizRc(void)
{

	DisarmBoard();
	switch(wizardStatus.currentStep)
	{
		case 0:
			PrepareRcWizard(); //set's current step to 1 after setting things up.
			//send reply that wizard has started
			RfCustomReplyBuffer("#wiz Move Sticks to extremes, center sticks, place throttle idle, then run wiz rc2\n");
			break;
		case 1:
			//Step 1 is running, record min max values every time this is run by the scheduler
			WizRcCheckMinMax();
			//todo: a report message here might be good
			break;
		case 2:
			//min max values are set, let's save them and now check centers
			WizRcCheckCenter();
			RfCustomReplyBuffer("#wiz Stick centers set, throttle idle set, run wiz rc3\n");
			ResetChannelCheck();
			break;
		case 3:
			//min max values are set, let's save them and now check centers
			WizRcCheckAndSendDirection();
			break;
		case 4:
			//min max values are set, let's save them and now check centers
			WizRcCheckArmSwitchDisarmed();
			RfCustomReplyBuffer("#wiz Disarm Set. Please run wiz rc5\n");
			break;
		case 5:
			if (WizRcCheckArmSwitchArmed())
			{
				RfCustomReplyBuffer("#wiz Wiz RC Successful\n");
				mainConfig.rcControlsConfig.rcCalibrated = 1;
				bzero(&wizardStatus, sizeof(wizardStatus)); //all done
				SaveAndSend();
				telemEnabled = 1;
			}
			else
			{
				RfCustomReplyBuffer("#wiz Wiz RC Failed\n");
			}

			break;
		default:
			RfCustomReplyBuffer("#wiz Unknown Step, Wiz RC\n");
			break;

	}
	DisarmBoard();
}

uint32_t WizRxCheckProtocol(uint32_t rxProtocol, uint32_t usart)
{
	FeedTheDog();
	DisarmBoard();
	DeInitBoardUsarts();
	DelayMs(2);
	SetRxDefaults(rxProtocol,usart);
	DelayMs(2);
	InitFlight(mainConfig.mixerConfig.escProtocol, mainConfig.mixerConfig.escUpdateFrequency);
	bzero(rxDataRaw, sizeof(rxDataRaw));
	bzero(rxData, sizeof(rxData));
	trueRcCommandF[0] = -1.1;
	trueRcCommandF[1] = -1.1;
	trueRcCommandF[2] = -1.1;
	trueRcCommandF[3] = -1.1;
	DisarmBoard();
	DelayMs(60);
	DisarmBoard();
	if (WizRxCheckRxDataLooksValid())
	{
		bzero(rxDataRaw, sizeof(rxDataRaw));
		bzero(rxData, sizeof(rxData));
		trueRcCommandF[0] = -1.1;
		trueRcCommandF[1] = -1.1;
		trueRcCommandF[2] = -1.1;
		trueRcCommandF[3] = -1.1;
		DelayMs(60);
		//check twice after a reset of data checks
		if (WizRxCheckRxDataLooksValid())
		{
			bzero(rxDataRaw, sizeof(rxDataRaw));
			bzero(rxData, sizeof(rxData));
			trueRcCommandF[0] = -1.1;
			trueRcCommandF[1] = -1.1;
			trueRcCommandF[2] = -1.1;
			trueRcCommandF[3] = -1.1;
			DelayMs(80);
			//check thrice after a reset of data checks
			if (WizRxCheckRxDataLooksValid())
			{
				//three times it looks good, most likely good data then
				return(1);
			}
		}
	}
	DelayMs(2);
	DisarmBoard();
	return(0);
}

void HandleWizRx(void)
{
	/*
#define USING_MANUAL           0
#define USING_SPEK_R           1
#define USING_SPEK_T           2
#define USING_SBUS_R           3
#define USING_SBUS_T           4
#define USING_SUMD_R           5
#define USING_SUMD_T           6
#define USING_IBUS_R           7
#define USING_IBUS_T           8
#define USING_DSM2_R           9
#define USING_DSM2_T           10
#define USING_CPPM_R           11
#define USING_CPPM_T           12
#define USING_SPORT            13
#define USING_MSP              14
#define USING_RFVTX            15
#define USING_SMARTAUDIO       16
#define USING_RFOSD            17
#define USING_TRAMP            18
#define USING_CRSF_R           19
#define USING_CRSF_T           20
#define USING_CRSF_B           21
#define USING_CRSF_TELEM       22
#define USING_RX_END           23
	 */
	uint32_t x, y, useThis, z = 0;


	for (x=1;x<USING_SPORT;x++)
	{

		if(x == USING_CPPM_R)
			useThis = USING_CRSF_R;
		else if(x == USING_CPPM_T)
			useThis = USING_CRSF_T;
		else
			useThis = x;

		for (y=0;y<MAX_USARTS;y++)
		{
			
			#if defined(REVOLT)
			//only check usart 1 and usart 3
			if( y==0 )
			{

			}
			else if( y==2 )
			{

			}
			else
			{
				continue;
			}
			//if( !(y==0 || y==2) )
			//	continue;
			#endif

			if (z == wizardStatus.currentStep)
			{
				wizardStatus.currentStep++;
				if (WizRxCheckProtocol(useThis,y))
				{
					snprintf( rf_custom_out_buffer, RF_BUFFER_SIZE, "#me RX Protocol %lu Found on Usart %lu\n", useThis, y+1 );
					RfCustomReplyBuffer(rf_custom_out_buffer);
					wizardStatus.currentWizard = 0;
					wizardStatus.currentStep = 0;
					resetBoard = 1;
					SetMode(M_ARMED, 0, 0, 0);
					mainConfig.rcControlsConfig.rcCalibrated = 0;
					SaveAndSend();
					return;
				}
				else
				{
					snprintf( rf_custom_out_buffer, RF_BUFFER_SIZE, "#me Scanning for RX... %lu\n",z+1 );
					RfCustomReplyBuffer(rf_custom_out_buffer);
					return;
				}
			}
			z++;
		}
	}
	snprintf( rf_custom_out_buffer, RF_BUFFER_SIZE, "#me RX Not Found\n" );
	RfCustomReplyBuffer(rf_custom_out_buffer);
	wizardStatus.currentWizard = 0;
	wizardStatus.currentStep = 0;
}

void SetupWizard(char *inString)
{

	if (!strcmp("rx", inString))
	{
		if (mainConfig.telemConfig.telemSmartAudio || mainConfig.telemConfig.telemSport)
		{
			DisarmBoard();
			mainConfig.telemConfig.telemSmartAudio = 0;
			mainConfig.telemConfig.telemSport = 0;
			InitFlight(mainConfig.mixerConfig.escProtocol, mainConfig.mixerConfig.escUpdateFrequency);
		}
		if (wizardStatus.currentWizard != WIZ_RX)
		{
			DeInitBoardUsarts();
			wizardStatus.currentWizard = WIZ_RX;
			wizardStatus.currentStep = 0;
			DelayMs(5);
		}
		HandleWizRx();
		return;
	}
	else if (!strcmp("mixera", inString))
	{
		MixerWizard(inString);
	}
	else if (!strcmp("mixerb", inString))
	{
		MixerWizard(inString);
	}
	else if (!strcmp("mixerc", inString))
	{
		MixerWizard(inString);
	}
	else if (!strcmp("cala", inString))
	{
		MassEraseDataFlash(0);
		mainConfig.gyroConfig.boardCalibrated = 0;
		mainConfig.gyroConfig.gyroRotation = CW0;
		DelayMs(200); //need to reset calibration and give ACC data time to refresh
		if (SetCalibrate1()) {
			RfCustomReplyBuffer("calibrate1finished\n");
		} else {
			RfCustomReplyBuffer("calibrationfailed\n");
		}
		return;
	}
	else if (!strcmp("calb", inString))
	{

		if (SetCalibrate2())
		{
			RfCustomReplyBuffer("calibrate2finished\n");
			SaveAndSend();
			return;
		} else {
			RfCustomReplyBuffer("calibrationfailed\n");
			return;
		}

	}
	else if (!strcmp("rc1", inString))
	{
		telemEnabled = 0;
		bzero(&wizardStatus, sizeof(wizardStatus));
		HandleWizRc();
		return;
	}
	else if (!strcmp("rc2", inString))
	{
		if (!mainConfig.rcControlsConfig.rcCalibrated)
		{
			DelayMs(30);
			wizardStatus.currentStep = 2;
			HandleWizRc();
			DelayMs(30);
		}
		else
		{
			RfCustomReplyBuffer("#wiz please run wiz rc1 first.\n");
		}
		return;
	}
	else if (!strcmp("rc3", inString))
	{
		if (!mainConfig.rcControlsConfig.rcCalibrated)
		{
			wizardStatus.currentStep = 3;
			HandleWizRc();
		}
		else
		{
			RfCustomReplyBuffer("#wiz please run wiz rc1 first.\n");
		}
		return;
	}
	else if (!strcmp("rc4", inString))
	{
		if (!mainConfig.rcControlsConfig.rcCalibrated)
		{
			wizardStatus.currentStep = 4;
			HandleWizRc();
		}
		else
		{
			RfCustomReplyBuffer("#wiz please run wiz rc1 first.\n");
		}
		return;
	}
	else if (!strcmp("rc5", inString))
	{
		if (!mainConfig.rcControlsConfig.rcCalibrated)
		{
			wizardStatus.currentStep = 5;
			HandleWizRc();
		}
		else
		{
			RfCustomReplyBuffer("#wiz please run wiz rc1 first.\n");
		}
		return;
	}
	else if (!strcmp("mot1", inString))
	{

		calibrateMotors = 1;
		SKIP_GYRO=1;
		DisarmBoard();
		motorOutput[0] = 1.0;
		OutputActuators(motorOutput, servoOutput);
		if (CheckSafeMotors(1000, 13000))
		{ //check for safe motors for 3 seconds, 10000 standard deviation allowed
			RfCustomReplyBuffer("#me Plug in battery, run wiz mot2 when tones finish\n");
			return;
		}
		else
		{
			RfCustomReplyBuffer("#me Motor calibration failed\n");
			motorOutput[0] = 0;
			motorOutput[1] = 0;
			motorOutput[2] = 0;
			motorOutput[3] = 0;
			motorOutput[4] = 0;
			motorOutput[5] = 0;
			motorOutput[6] = 0;
			motorOutput[7] = 0;
			motorOutput[8] = 0;
			OutputActuators(motorOutput, servoOutput);
			SKIP_GYRO=0;
			calibrateMotors = 0;
			return;
		}

	}
	else if (!strcmp("mot2", inString))
	{

		motorOutput[0] = 0;
		motorOutput[1] = 0;
		motorOutput[2] = 0;
		motorOutput[3] = 0;
		motorOutput[4] = 0;
		motorOutput[5] = 0;
		motorOutput[6] = 0;
		motorOutput[7] = 0;
		motorOutput[8] = 0;
		OutputActuators(motorOutput, servoOutput);
		SKIP_GYRO=0;
		calibrateMotors = 0;

		RfCustomReplyBuffer("#me Motor Calibration Success\n");
		return;

	}
	else
	{
		bzero(rf_custom_out_buffer,RF_BUFFER_SIZE);
		snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me Unknown Argument:%s\n", inString);
		RfCustomReplyBuffer(rf_custom_out_buffer);
	}
}






/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////ONE WIRE/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void OneWire(char *inString) {

	esc_hex_location escHexLocation;
	const oneWireParameter_t *parameter;
	int16_t value, bytesWritten = 0;
	uint32_t idx;
	uint32_t x;
	uint32_t y;
	char *modString = NULL;
	char *args = NULL;
	uint32_t motorNumber;
	uint32_t maxMotors;
	uint32_t modStringLength;
	uint32_t outputNumber;
	uint32_t somethingHappened = 0;
	uint32_t verbose = 1;
	uint32_t doingAuto = 0;
	uint32_t doingSettings = 0;

	if (!strcmp("start", inString) || !strcmp("read", inString) || !strcmp("check", inString) || !strcmp("settings", inString) || !strcmp("auto", inString) || !strcmp("autoa", inString))
	{

		if (!strcmp("check", inString))
		{
			verbose = 0;
		}
		else if (!strcmp("autoa", inString))
		{
			doingAuto = 1;
			doingAutoA = 1;
		}
		else if (!strcmp("auto", inString))
		{
			doingAuto = 1;
		}
		else if (!strcmp("settings", inString))
		{
			doingSettings = 1;
		}

		oneWireActive = 1;
		if (verbose)
		{
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "Reading ESCs...\n");
			RfCustomReplyBuffer(rf_custom_out_buffer);
			DelayMs(5);
		}
		if (OneWireInit() == 0)
		{
			if (verbose)
			{
				snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "No ESCs detected. Is your battery connected?\n");
				RfCustomReplyBuffer(rf_custom_out_buffer);
				DelayMs(5);
				return;
			}

		}
		else
		{
			for (x = 0; x < MAX_MOTOR_NUMBER; x++)
			{
				outputNumber = mainConfig.mixerConfig.motorOutput[x];
				if (board.motors[outputNumber].enabled == ENUM_ACTUATOR_TYPE_MOTOR)
				{
					if (escOneWireStatus[board.motors[outputNumber].actuatorArrayNum].enabled)
					{
						if (verbose)
						{
							snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "Motor %lu: %u.%u, %s, %s, %s\n", x, (uint8_t)( (escOneWireStatus[board.motors[outputNumber].actuatorArrayNum].version >> 8) & 0xFF), (uint8_t)(escOneWireStatus[board.motors[outputNumber].actuatorArrayNum].version & 0xFF), escOneWireStatus[board.motors[outputNumber].actuatorArrayNum].nameStr, escOneWireStatus[board.motors[outputNumber].actuatorArrayNum].fwStr, escOneWireStatus[board.motors[outputNumber].actuatorArrayNum].versionStr);
							RfCustomReplyBuffer(rf_custom_out_buffer);
						}
						if (escOneWireStatus[board.motors[outputNumber].actuatorArrayNum].escHexLocation.version > escOneWireStatus[board.motors[outputNumber].actuatorArrayNum].version) {
							if (verbose)
							{
								snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "Motor %lu: Upgrade to version %u.%u is available\n", x, (uint8_t)( (escOneWireStatus[board.motors[outputNumber].actuatorArrayNum].escHexLocation.version >> 8) & 0xFF), (uint8_t)( escOneWireStatus[board.motors[outputNumber].actuatorArrayNum].escHexLocation.version & 0xFF) );
								RfCustomReplyBuffer(rf_custom_out_buffer);
							}
							else
							{
								snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "upgrade available\n" );
								RfCustomReplyBuffer(rf_custom_out_buffer);
								return;
							}
						}
					}
					else
					{
						if (verbose)
						{
							snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "Motor %lu Unreadable\n", x);
							RfCustomReplyBuffer(rf_custom_out_buffer);
						}
					}
				}
				else
				{
					if (verbose)
					{
						snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "Motor %lu Disabled\n", x);
						RfCustomReplyBuffer(rf_custom_out_buffer);
					}
				}

			}

			if (!verbose)
			{
				snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "no upgrade available\n");
				RfCustomReplyBuffer(rf_custom_out_buffer);
				return;
			}
			if ( doingSettings || doingAuto )
			{
				for (x = 0; x < MAX_MOTOR_NUMBER; x++)
				{
					outputNumber = mainConfig.mixerConfig.motorOutput[x];
					if (board.motors[outputNumber].enabled == ENUM_ACTUATOR_TYPE_MOTOR)
					{
						//snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "Saving Motor %lu Config.\n", x);
						//RfCustomReplyBuffer(rf_custom_out_buffer);

						if (OneWireSaveConfig(board.motors[outputNumber]))
						{
							snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "Motor %lu Config Read.\n", x);
							RfCustomReplyBuffer(rf_custom_out_buffer);
						}
						else
						{
							snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "Error Motor %lu Config.\n", x);
							RfCustomReplyBuffer(rf_custom_out_buffer);
						}

					}

				}
			}
			if (doingAuto)
			{
				inString = "ma=upgrade";
				OneWire(inString);
			}

			snprintf(rf_custom_out_buffer+bytesWritten, RF_BUFFER_SIZE-bytesWritten, "1wiredumpstart\n");
			RfCustomReplyBuffer(rf_custom_out_buffer);
			bytesWritten = 0;


			for (x = 0; x < MAX_MOTOR_NUMBER; x++)
			{

				outputNumber = mainConfig.mixerConfig.motorOutput[x];
				if ( (board.motors[outputNumber].enabled == ENUM_ACTUATOR_TYPE_MOTOR) && (escOneWireStatus[board.motors[outputNumber].actuatorArrayNum].enabled) )
				{

					for (idx = 0; oneWireParameters[idx] != NULL; idx++)
					{

						parameter = oneWireParameters[idx];
						bytesWritten += snprintf(rf_custom_out_buffer+bytesWritten, RF_BUFFER_SIZE-bytesWritten, "m%lu=%s=", x, parameter->name);
						value = Esc1WireParameterFromDump(board.motors[outputNumber], parameter, escOneWireStatus[board.motors[outputNumber].actuatorArrayNum].config);
						// make sure the value is valid

						if (value == 0xFF)
						{
							bytesWritten += snprintf(rf_custom_out_buffer+bytesWritten, RF_BUFFER_SIZE-bytesWritten, "NONE\n");
							RfCustomReplyBuffer(rf_custom_out_buffer);
							bytesWritten = 0;
							continue;
						}

						// add the readable form of the parameter value to the buffer
						if (parameter->parameterNamed)
						{
							bytesWritten += snprintf(rf_custom_out_buffer+bytesWritten, RF_BUFFER_SIZE-bytesWritten, "%s\n", OneWireParameterValueToName(parameter->parameterNamed, value));
							RfCustomReplyBuffer(rf_custom_out_buffer);
							bytesWritten = 0;
						}
						else
						{
							bytesWritten += snprintf(rf_custom_out_buffer+bytesWritten, RF_BUFFER_SIZE-bytesWritten, "%d\n", OneWireParameterValueToNumber(parameter->parameterNumerical, value));
							RfCustomReplyBuffer(rf_custom_out_buffer);
							bytesWritten = 0;
						}

					}

				}

			}

			snprintf(rf_custom_out_buffer+bytesWritten, RF_BUFFER_SIZE-bytesWritten, "1wiredumpcomplete\n");
			RfCustomReplyBuffer(rf_custom_out_buffer);
			bytesWritten = 0;

		}
		if (doingAuto)
		{
			oneWireActive = 0;
			OneWireDeinit();
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "1Wire Session Ended.\n");
			RfCustomReplyBuffer(rf_custom_out_buffer);
		}

	}
	else if (!strcmp("list", inString))
	{
		if (!ListAllEscHexesInFlash())
		{
			snprintf(rf_custom_out_buffer+bytesWritten, RF_BUFFER_SIZE-bytesWritten, "No ESC Hexes found in flash\n");
			RfCustomReplyBuffer(rf_custom_out_buffer);
		}
	}
	else if (!strcmp("config", inString))
	{

		snprintf(rf_custom_out_buffer+bytesWritten, RF_BUFFER_SIZE-bytesWritten, "1wiredumpstart\n");
		RfCustomReplyBuffer(rf_custom_out_buffer);
		bytesWritten = 0;

		for (x = 0; x < MAX_MOTOR_NUMBER; x++)
		{

			outputNumber = mainConfig.mixerConfig.motorOutput[x];
			if ( (board.motors[outputNumber].enabled == ENUM_ACTUATOR_TYPE_MOTOR) && (escOneWireStatus[board.motors[outputNumber].actuatorArrayNum].enabled) )
			{

				for (idx = 0; oneWireParameters[idx] != NULL; idx++)
				{

					parameter = oneWireParameters[idx];
					bytesWritten += snprintf(rf_custom_out_buffer+bytesWritten, RF_BUFFER_SIZE-bytesWritten, "m%lu=%s=", x, parameter->name);
					value = Esc1WireParameterFromDump(board.motors[outputNumber], parameter, escOneWireStatus[board.motors[outputNumber].actuatorArrayNum].config);
					// make sure the value is valid

					if (value == 0xFF)
					{
						bytesWritten += snprintf(rf_custom_out_buffer+bytesWritten, RF_BUFFER_SIZE-bytesWritten, "NONE\n");
						RfCustomReplyBuffer(rf_custom_out_buffer);
						bytesWritten = 0;
						continue;
					}

					// add the readable form of the parameter value to the buffer
					if (parameter->parameterNamed)
					{
						bytesWritten += snprintf(rf_custom_out_buffer+bytesWritten, RF_BUFFER_SIZE-bytesWritten, "%s\n", OneWireParameterValueToName(parameter->parameterNamed, value));
						RfCustomReplyBuffer(rf_custom_out_buffer);
						bytesWritten = 0;
					}
					else
					{
						bytesWritten += snprintf(rf_custom_out_buffer+bytesWritten, RF_BUFFER_SIZE-bytesWritten, "%d\n", OneWireParameterValueToNumber(parameter->parameterNumerical, value));
						RfCustomReplyBuffer(rf_custom_out_buffer);
						bytesWritten = 0;
					}

				}

			}

		}

		snprintf(rf_custom_out_buffer+bytesWritten, RF_BUFFER_SIZE-bytesWritten, "1wiredumpcomplete\n");
		RfCustomReplyBuffer(rf_custom_out_buffer);
		bytesWritten = 0;

	}
	else if (!strcmp("save", inString))
	{

		for (x = 0; x < MAX_MOTOR_NUMBER; x++)
		{
			outputNumber = mainConfig.mixerConfig.motorOutput[x];
			if (board.motors[outputNumber].enabled == ENUM_ACTUATOR_TYPE_MOTOR)
			{
				snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "Saving Motor %lu Config.\n", x);
				RfCustomReplyBuffer(rf_custom_out_buffer);

				if (OneWireSaveConfig(board.motors[outputNumber]))
				{
					snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "Motor %lu Config Saved.\n", x);
					RfCustomReplyBuffer(rf_custom_out_buffer);
				}
				else
				{
					snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "Error Saving Motor %lu Config.\n", x);
					RfCustomReplyBuffer(rf_custom_out_buffer);
				}

			}

		}

	}
	else if (!strcmp("stop", inString))
	{
		oneWireActive = 0;
		OneWireDeinit();
		snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "1Wire Session Ended.\n");
		RfCustomReplyBuffer(rf_custom_out_buffer);
	}
	else
	{
		uint32_t forceUpgrade = 0;
		uint32_t normalUpgrade = 0;

		if ( (inString[0] == 'm') && (inString[2] == '=') ) {

			if (inString[1] == 'a' )
			{
				y=0;
				maxMotors=4;
			}
			else
			{
				y=atoi(&inString[1]);
				maxMotors=y+1;
			}

			modString = inString+3;

			StripSpaces(modString);

			modStringLength = strlen(modString);

			for (x = 0; x < modStringLength; x++)
			{
				if (modString[x] == '=')
					break;
			}

			if (modStringLength > x)
			{
				args = modString + x + 1;
			}

			modString[x] = 0;

			for (x = 0; x < strlen(modString); x++)
				modString[x] = tolower((unsigned char)modString[x]);

			for (x = 0; x < strlen(args); x++)
				args[x] = tolower((unsigned char)args[x]);

			if ( (inString[1] == 'a' ) && !strcmp("forceupgrade", modString) )
			{
				snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "Automatic Force Upgrading Not Allowed\n");
				RfCustomReplyBuffer(rf_custom_out_buffer);
				return;
			}
			else
			{
				if (!strcmp("upgrade", modString))
				{
					normalUpgrade = 1;
				}
				else if (!strcmp("forceupgrade", modString))
				{
					forceUpgrade = 1;
				}
			}

			for (motorNumber=y;motorNumber<maxMotors;motorNumber++)
			{
				outputNumber = mainConfig.mixerConfig.motorOutput[motorNumber];


				if ( normalUpgrade || forceUpgrade)
				{
					if (forceUpgrade)
					{
						if ( (atoi(args)) && (atoi(args) < 51) )
						{
							memcpy(&escHexLocation, &escHexByPosition[atoi(args)], sizeof(esc_hex_location));
						}
						else
						{
							snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "ESC %lu upgrade will not progress. ESC Hex not found.\n", motorNumber);
							RfCustomReplyBuffer(rf_custom_out_buffer);
							somethingHappened=1;
							continue;
						}
					}
					else
					{
						memcpy(&escHexLocation, &escOneWireStatus[board.motors[outputNumber].actuatorArrayNum].escHexLocation, sizeof(esc_hex_location));
					}

					if ( (board.motors[outputNumber].enabled == ENUM_ACTUATOR_TYPE_MOTOR) && (escOneWireStatus[board.motors[outputNumber].actuatorArrayNum].enabled) )
					{
						snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "Upgrading %lu...\n", motorNumber);
						RfCustomReplyBuffer(rf_custom_out_buffer);
						somethingHappened=1;

						if ( forceUpgrade || (escHexLocation.version > escOneWireStatus[board.motors[outputNumber].actuatorArrayNum].version) )
						{
							if ( BuiltInUpgradeSiLabsBLHeli(board.motors[outputNumber], escHexLocation) )
							{
								snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "ESC %lu upgrade complete.\n", motorNumber);
								RfCustomReplyBuffer(rf_custom_out_buffer);
							}
							else
							{
								snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "ESC %lu not upgraded.\n", motorNumber);
								RfCustomReplyBuffer(rf_custom_out_buffer);
							}
						}
						else
						{
							snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "ESC %lu version up to date.\n", motorNumber);
							RfCustomReplyBuffer(rf_custom_out_buffer);
						}
					}
					else
					{
						snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "No upgrade available for ESC %lu\n", motorNumber);
						RfCustomReplyBuffer(rf_custom_out_buffer);
						somethingHappened=1;
					}
				}
				else
				{
					if ( (board.motors[outputNumber].enabled == ENUM_ACTUATOR_TYPE_MOTOR) && (escOneWireStatus[board.motors[outputNumber].actuatorArrayNum].enabled) )
					{
						for (idx = 0; oneWireParameters[idx] != NULL; idx++)
						{
							parameter = oneWireParameters[idx];
							if (!strcmp(parameter->name, modString)) //found the proper parameter, now let's get the proper value based on the string args
							{
								if (parameter->parameterNamed) //is the value a string?
								{
									value = OneWireParameterNameToValue(parameter->parameterNamed, args);
								}
								else //then it's an int.
								{
									value = OneWireParameterNumberToValue(parameter->parameterNumerical, (int16_t)atoi(args));
								}
								if (value < 0)
								{
									snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "ERROR=Unknown parameter value\n");
									RfCustomReplyBuffer(rf_custom_out_buffer);
									somethingHappened=1;
									return;
								}
								else
								{
									if ( Esc1WireSetParameter(board.motors[outputNumber], parameter, escOneWireStatus[board.motors[outputNumber].actuatorArrayNum].config, value) )
									{
										snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "ESC %lu Value Set! Please Save Changes!\n", motorNumber);
										RfCustomReplyBuffer(rf_custom_out_buffer);
										somethingHappened=1;
										continue;
									}
									else
									{
										snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "ERROR=Unknown ESC Layout\n");
										RfCustomReplyBuffer(rf_custom_out_buffer);
										somethingHappened=1;
										continue;
									}

								}

							}

						}

					}

				}

			}

		}
		if (!somethingHappened)
		{
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "ERROR=Unknown Command\n");
			RfCustomReplyBuffer(rf_custom_out_buffer);
		}

	}

}
