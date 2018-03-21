#include "includes.h"

volatile uint32_t safeLoopCounter;
//this doesn't really belong here
volatile uint8_t tInBuffer[HID_EPIN_SIZE], tOutBuffer[HID_EPOUT_SIZE-1];

uint32_t skipTaskHandlePcComm = 0;
volatile uint32_t errorMask   = 0;

//scheduler timer
TIM_HandleTypeDef schedulerTimer;

volatile int taskIdleActuators[8];
volatile int taskDdsActuators;
volatile int taskDshotActuators;

//soft serial buffer handling. TODO: make a structure
volatile uint32_t softSerialEnabled = 0;
volatile uint32_t softSerialBuf[2][SOFT_SERIAL_BIT_TIME_ARRAY_SIZE];
volatile uint32_t softSerialInd[2];
volatile uint32_t softSerialCurBuf;
volatile uint32_t softSerialLastByteProcessedLocation;
volatile uint32_t softSerialSwitchBuffer;
volatile uint32_t turnOnVtxNow = 0;

uint8_t  proccesedSoftSerial[25]; //25 byte buffer enough?
uint32_t proccesedSoftSerialIdx   = 0;
uint32_t softSerialLineIdleSensed = 0;
uint32_t lastBitFound             = 0;

static void TaskProcessSoftSerial(void);
static void TaskTelemtry(void);
static void TaskQuopa(void);
static void TaskPersistanceAndFlash(void);
static void TaskWizard(void);
static void TaskHandlePcComm(void);
static void TaskLed(void);
static void TaskBuzzer(void);
static void TaskIdleActuators(void);
static void TaskAdc(void);
static void TaskCheckVtx(void);
static void TaskCheckDelayedArming(void);
static void TaskProcessArmingStructure(void);
static void InitGeneralInterruptTimer(uint32_t pwmHz, uint32_t timerHz);
static void DeInitGeneralInterruptTimer(void);
static void TaskeSafeLoopCounter(void);

static void DeInitGeneralInterruptTimer(void)
{
	HAL_TIM_Base_Stop_IT(&schedulerTimer);
	callbackFunctionArray[GetTimerCallbackFromTimerEnum(board.generalTimer[0].timer)] = 0;
}

static void InitGeneralInterruptTimer(uint32_t pwmHz, uint32_t timerHz)
{
	uint16_t timerPrescaler = 0;

	callbackFunctionArray[GetTimerCallbackFromTimerEnum(board.generalTimer[0].timer)] = GeneralInterruptTimerCallback;

	timerPrescaler = (uint16_t)(SystemCoreClock / TimerPrescalerDivisor(board.generalTimer[0].timer) / timerHz) - 1;

	// Initialize timer
	schedulerTimer.Instance           = timers[board.generalTimer[0].timer];
	schedulerTimer.Init.Prescaler     = timerPrescaler;
	schedulerTimer.Init.CounterMode   = TIM_COUNTERMODE_UP;
	schedulerTimer.Init.Period        = (timerHz / pwmHz) - 1;
	schedulerTimer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_Base_Init(&schedulerTimer);
	HAL_TIM_Base_Start_IT(&schedulerTimer);

    HAL_NVIC_SetPriority(board.generalTimer[0].timerIRQn, 2, 0);
    HAL_NVIC_EnableIRQ(board.generalTimer[0].timerIRQn);
}

//TODO: Don't hardcode to timer 6, use a callback instead
void GeneralInterruptTimerCallback(uint32_t callbackNumber)
{

	(void)(callbackNumber);

    if (__HAL_TIM_GET_FLAG(&schedulerTimer, TIM_FLAG_UPDATE) != RESET)      //In case other interrupts are also running
    {

        if (__HAL_TIM_GET_ITSTATUS(&schedulerTimer, TIM_IT_UPDATE) != RESET)
        {

            __HAL_TIM_CLEAR_FLAG(&schedulerTimer, TIM_FLAG_UPDATE);

            //triggers DMA read which will trigger flight code
            GyroExtiCallback(0);

        }

    }

}

//DeInit the fake Gyro EXTI and attempt to reenable the actual EXTI, this checks that the gyro is interrupting and will init the fake EXTI if necessary.
void DeInitFakeGyroExti(void)
{
	DeInitGeneralInterruptTimer();
	InitGyroExti();
	DelayMs(2);
	gyroInterrupting = 0;
	DelayMs(2);
	if(!gyroInterrupting)
	{
		InitFakeGyroExti();
	}
}

//Deinit the real gyro EXTI and Init the fake gyro EXTI
void InitFakeGyroExti(void)
{
	//TODO: Don't just hardcode for 32KHz
	DeInitGyroExti();
	InitGeneralInterruptTimer(32000, 48000000);
}

void InitScheduler(void)
{
	safeLoopCounter  = 0;
	taskDdsActuators = 0;
	taskDshotActuators = 0;
	taskIdleActuators[0]=0;
	taskIdleActuators[1]=0;
	taskIdleActuators[2]=0;
	taskIdleActuators[3]=0;
	taskIdleActuators[4]=0;
	taskIdleActuators[5]=0;
	taskIdleActuators[6]=0;
	taskIdleActuators[7]=0;
	//DeInit the fake Gyro EXTI and attempt to reenable the actual EXTI, this checks that the gyro is interrupting and will init the fake EXTI if necessary.
	DelayMs(2);
	gyroInterrupting = 0;
	DelayMs(2);
	if(!gyroInterrupting)
	{
		InitFakeGyroExti();
	}
	turnOnVtxNow = 0;

	return;
}

void Scheduler(int32_t count)
{

	switch (count) {

		case 0:
			TaskHandlePcComm();
			break;
		case 1:
			TaskLed();
			break;
		case 2:
			TaskBuzzer();
			break;
		case 3:
			TaskAdc();
			break;
		case 4:
			TaskProcessSoftSerial();
			break;
		case 5:
			TaskTelemtry();
			break;
		case 6:
			TaskWizard();
			break;
		case 7:
			TaskCheckVtx();
			break;
		case 8:
			TaskCheckDelayedArming();
			break;
		case 9:
			TaskProcessArmingStructure();
			break;
		case 10:
			TaskIdleActuators();
			break;
		case 11:
			TaskeSafeLoopCounter();
			break;
		case 12:
			TaskQuopa();
			break;
		case 13:
			TaskPersistanceAndFlash();
			break;
		case 14:
			break;
		case 15:
			break;
		case 16:
			break;
		default:
			break;

	}

}

static void TaskPersistanceAndFlash(void)
{

	static uint32_t lastPersistance = 0;

	#ifdef OLD_LOG
	return;
	#endif
	//is SPI busy?, doesn't send spi commands
	if(CheckIfFlashSpiBusy())
		return;

	//telem or flash for now, not both
	if(mainConfig.telemConfig.telemRfOsd == TELEM_INTERNAL_OSD)
	{
		switch(maxOsdRecord.osdUpdateStatus)
		{
			case OSD_STATUS_UPDATE_IDLE:
				break;
			case OSD_STATUS_REQUEST_UPDATE:
			maxOsdRecord.osdUpdateStatus = OSD_STATUS_UPDATE_APROVED;
				return;
			case OSD_STATUS_UPDATING:
			case OSD_STATUS_UPDATE_APROVED:
				return;		
				break;
		}
	}

	//if(InlineMillis() < 20000)
	//	return;

	////is flash chip busy?, blocking, but only 4 bytes
	//if(CheckIfFlashBusy())
	//	return;

	//nothing busy so we fall through to here
	//save persistance if it's enabled, due and flash is imediately availible, board is armed when this happens
	//if ( (armedTime > 4000) && (persistance.enabled) && (InlineMillis() - lastPersistance > 3000) )
	if ( (persistance.enabled) && (InlineMillis() - lastPersistance > 3000) )
	{
		//persistance overrides logging
		lastPersistance = InlineMillis();
		SavePersistance(); //uses blocking or irq as needed, both functions will set flashstatus back to free when avalible
	}
	else if(flashTxBuffer == NULL) //nothing to write
	{
		return;
	}
	else
	{ //something to write
		//M25p16BlockingWritePage(flashWriteAddress, flashTxBuffer);
		if ( IsDshotEnabled() || (quopaState == QUOPA_ACTIVE) )
		{
			M25p16IrqWritePage(flashWriteAddress, flashTxBuffer);
		}
		else
		{
			M25p16DmaWritePage(flashWriteAddress, flashTxBuffer, flashRxBuffer);
		}
		flashTxBuffer = NULL;
		
	}

}

static void TaskeSafeLoopCounter(void)
{
	static int failiureLatch = 0;
	static uint32_t lastTimeMs = 0;
	static uint32_t currTimeMs = 0;

	currTimeMs = InlineMillis();

	safeLoopCounter++;

	if (failiureLatch)
	{
		//ledStatus.status = LEDS_FAST_BLINK;
		if (!boardArmed)
		{
			//SKIP_GYRO = 1;
		}
	}

	//don't start checking until after 5 seconds
	if (currTimeMs > 5000)
	{
		//one second since last check has happened
		if (currTimeMs - lastTimeMs >= 1000)
		{
			if(safeLoopCounter < 32000)
			{
				//CPU load is too high
				failiureLatch = 1;
			}
			safeLoopCounter = 0;
			lastTimeMs = currTimeMs;
		}
	}

}

void TaskProcessArmingStructure(void)
{
	static uint32_t lastSwitch = 5000;

	ProcessArmingStructure();
	if (!boardArmed)
 	{
		if ( (InlineMillis() - lastSwitch) < 1000 )
			return;

 		if (ModeActive(M_PROFILE3)) // is profile 3 active?
 		{
 			if (activeProfile != PROFILE3 )
 			{
 				DeinitFlight();
 				activeProfile = PROFILE3;
 				InitFlight(mainConfig.mixerConfig.escProtocol, mainConfig.mixerConfig.escUpdateFrequency);
 				activeProfile = PROFILE3;
				lastSwitch = InlineMillis();
 			}
 		}
 		else if (ModeActive(M_PROFILE2))  // is profile 2 active?
 		{
 			if (activeProfile != PROFILE2 )
 			{
 				DeinitFlight();
 				activeProfile = PROFILE2;
 				InitFlight(mainConfig.mixerConfig.escProtocol, mainConfig.mixerConfig.escUpdateFrequency);
 				activeProfile = PROFILE2;
				lastSwitch = InlineMillis();
 			}
 		}
 		else  // then profile 1 is active?
 		{
 			if (activeProfile != PROFILE1 )
 			{
 				DeinitFlight();
 				activeProfile = PROFILE1;
 				InitFlight(mainConfig.mixerConfig.escProtocol, mainConfig.mixerConfig.escUpdateFrequency);
 				activeProfile = PROFILE1;
				lastSwitch = InlineMillis();
 			}
 		}
 	}
}

void TaskCheckDelayedArming(void)
{

 	//handles VTX enabling as well
 	if(armBoardAt)
 	{
 		//delayed arming window of 150 ms.
 		if ( (InlineMillis() > armBoardAt) && ((InlineMillis() - armBoardAt) < 250) )
 		{
 			if( mainConfig.telemConfig.telemSmartAudio && !ModeSet(M_VTXON) )
 			{
 				//only try turning on the VTX once per arming
 				turnOnVtxNow = 0;
 				VtxTurnOn();
 			}
 			armBoardAt = 0;
 			ArmBoard();
 		}
 	}
}

void TaskCheckVtx(void)
{

	static uint32_t modeLatch = 0;

	if ( (!boardArmed) && (!progMode) && ((mainConfig.telemConfig.telemSmartAudio) || (mainConfig.telemConfig.telemTramp)) )
	{

		if (mainConfig.telemConfig.telemTramp && vtxRecord.vtxDevice != VTX_DEVICE_TRAMP)
		{
			//check every two seconds for VTX if there's supposed to be one
			if (InlineMillis() % 2000 == 0)
			{
				TrampGetSettings();
			}
		}
		if (ModeSet(M_VTXON) && ModeActive(M_VTXON) && !modeLatch)
		{
			turnOnVtxNow = 1;
			modeLatch = 1;
		}
		else if (ModeSet(M_VTXON) && !ModeActive(M_VTXON))
		{
			modeLatch = 0;
		}

		if (turnOnVtxNow)
		{
			turnOnVtxNow = 0;
			VtxTurnOn(); //blocking of scheduler during send and receive
		}

		if (vtxRequested.vtxBandChannel != vtxRecord.vtxBandChannel)
		{
			VtxBandChannel(vtxRequested.vtxBandChannel);
		}

		if (vtxRequested.vtxPit != vtxRecord.vtxPit)
		{
			//if (vtxRecord.vtxPit == VTX_MODE_ACTIVE)
			//	VtxTurnOn();
			//else
			//	VtxTurnPit();
		}

		if (vtxRequested.vtxPower != vtxRecord.vtxPower)
		{
			VtxPower(vtxRequested.vtxPower);
		}
	}
}

void TaskAdc(void)
{
	if (progMode)
		return;
	PollAdc();
	CheckBatteryCellCount();
}

void TaskProcessSoftSerial(void)
{
	//feed the dog if oneWire is active to prevent WD restart
	if (oneWireActive)
		FeedTheDog();

	//feed the dog if dshot commanding is active to prevent WD restart
	if(dshotCommandHandler.dshotCommandState == DSC_MODE_ACTIVE)
		FeedTheDog();

	HandleDshotCommands();
}

void TaskWizard(void)
{
	switch(wizardStatus.currentWizard)
	{

		case WIZ_RC:
			if (wizardStatus.currentStep == 1) //step three needs to be polled by user/gui
				HandleWizRc();
			break;
		case 0:
		default:
			return;
	}
}

void TaskTelemtry(void)
{
	ProcessTelemtry();
	HandleMaxOsd();
}

void TaskHandlePcComm(void)
{
	uint32_t x;

	if (skipTaskHandlePcComm)
		return;

	if (tOutBuffer[0]==2)
	{ //we have a usb report
		ProcessCommand((char *)tOutBuffer);
		for (x=0;x<(HID_EPOUT_SIZE-1);x++)
			tOutBuffer[x] = 0;
	}

}

void TaskLed(void)
{
	UpdateLeds(); //update status LEDs
	UpdateWs2812Leds();
}

void TaskIdleActuators(void)
{
	int x;
	static uint32_t runEveryMs = 0;

	if (boardArmed)
		return;

	//if(runEveryUs == 0)
	//	ZeroActuators(500);

	if (InlineMillis() > runEveryMs)
	{
		runEveryMs = InlineMillis();
		if (taskDdsActuators)
		{
			OutputDDShotDma(board.motors[0], 0, taskDdsActuators);
			OutputDDShotDma(board.motors[1], 0, taskDdsActuators);
			OutputDDShotDma(board.motors[2], 0, taskDdsActuators);
			OutputDDShotDma(board.motors[3], 0, taskDdsActuators);
		}
		else if (taskDshotActuators)
		{
			uint8_t serialOutBuffer[2];
			uint32_t outputNumber;
			for (x=0;x<4;x++)
			{
				outputNumber = mainConfig.mixerConfig.motorOutput[x];
				if(taskIdleActuators[x] == 1)
				{
					ThrottleToDshot(serialOutBuffer, 0.001f, mainConfig.mixerConfig.idlePercent, mainConfig.mixerConfig.bitReverseEsc[x]);
					OutputSerialDmaByte(serialOutBuffer, 2, board.motors[outputNumber], 1, 0, 1); //buffer with data, number of bytes, actuator to output on, msb, no serial frame
				}
				else
				{
					ThrottleToDshot(serialOutBuffer, 0, 0, 0);
					OutputSerialDmaByte(serialOutBuffer, 2, board.motors[outputNumber], 1, 0, 1); //buffer with data, number of bytes, actuator to output on, msb, no serial frame
				}
			}	
		}
		else
		{
			for (x=0;x<8;x++)
			{
				if(taskIdleActuators[x] == 1)
					IdleActuator(x);

			}
		}
	}
}

void TaskBuzzer(void)
{
	UpdateBuzzer();
}

void ErrorHandler(uint32_t error)
{
	errorMask |= (error);
	int slowBlink = 2;
	int counter, temp; //set

	switch (error)
	{
		case BAD_TELEMETRY_SETUP:
		case TIMER_INPUT_INIT_FAILIURE:
		case ADC_INIT_FAILIURE:
		case ADC_DMA_INIT_FAILIURE:
		case MSP_DMA_GYRO_RX_INIT_FAILIURE:
		case MSP_DMA_GYRO_TX_INIT_FAILIURE:
		case MSP_DMA_SPI_RX_INIT_FAILIURE:
		case MSP_DMA_SPI_TX_INIT_FAILIURE:
			//ping warning to user here, may not a valid reason to crash the board though
			return;
			break;
		case SERIAL_HALF_DUPLEX_INIT_FAILURE:
			//ping warning to user here, not a valid reason to crash the board though
			return;
			break;
		case SERIAL_INIT_FAILURE:
			//ping warning to user here, not a valid reason to crash the board though
			return;
			break;
		case FLASH_SPI_INIT_FAILIURE:
			//ping warning to user here, not a valid reason to crash the board though
			return;
			break;
		case WS2812_LED_INIT_FAILIURE:
			//ping warning to user here, not a valid reason to crash the board though
			return;
			break;
		case GYRO_SPI_INIT_FAILIURE: //gyro failed to init. Can't fly like this.
		case GYRO_INIT_FAILIURE: //gyro failed to init. Can't fly like this.
		case GYRO_SETUP_COMMUNICATION_FAILIURE: //gyro init success, but setting up register failed. Can't fly like this.
			return;
			break;
		case HARD_FAULT:  //hard fault is bad, if we're in flight we should setup a restart, for now we crash the board
			slowBlink = 4;
			break;
		case MEM_FAULT:   //hard fault is bad, if we're in flight we should setup a restart, for now we crash the board
			slowBlink = 6;
			break;
		case BUS_FAULT:   //hard fault is bad, if we're in flight we should setup a restart, for now we crash the board
			slowBlink = 8;
			break;
		case USAGE_FAULT: //hard fault is bad, if we're in flight we should setup a restart, for now we crash the board
			slowBlink = 10;
			break;
		default:
			break;
	}

	//bad errors will fall through here
	ZeroActuators(32000);

	counter = 0;
	temp = slowBlink;
    while (1)
    {
		if(counter++ > 20)
		{
			while(temp-->0)
			{
				DoLed(1, 0);
				DoLed(0, 1);
				simpleDelay_ASM(1000000);
				DoLed(1, 0);
				DoLed(0, 0);
				simpleDelay_ASM(1000000);
			}
			temp = slowBlink;
			counter = 0;
		}
		DoLed(0, 1);
		DoLed(1, 0);
		simpleDelay_ASM(50000);
		DoLed(0, 0);
		DoLed(1, 1);
		simpleDelay_ASM(75000);
    	ZeroActuators(10);
    }

}

static void TaskQuopa(void)
{
	//set's the quopa state machine
	HandleQuopaMode();
	HandleDshotBeep();
}