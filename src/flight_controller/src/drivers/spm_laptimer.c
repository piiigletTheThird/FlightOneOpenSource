#include "includes.h"

STRU_TELE_LAPTIMER lap_timer;
uint32_t lastTimeLap;
uint32_t lastTimegate;
uint32_t currentTime;
uint32_t captureTime;
uint32_t pulseIndex;
uint32_t pulseDuration[8];
uint8_t pulseCode;
uint32_t hits;
uint8_t lapDelay = 0;

motor_type actuator;

void InitLaptimer(void)
{
	lap_timer.identifier = TELE_DEVICE_LAPTIMER;
	lap_timer.sID = 0x00;
	lap_timer.lapNumber = 0;
	lap_timer.gateNumber = 0;
	lap_timer.lastLapTime = 0;
	lap_timer.gateTime = 0;

	uint8_t actuatorNumOutput;
	

	for (actuatorNumOutput = 0; actuatorNumOutput < MAX_MOTOR_NUMBER; actuatorNumOutput++)
	{

		if (board.motors[actuatorNumOutput].enabled == ENUM_ACTUATOR_TYPE_SPMLAPTIMER)
		{
			actuator = board.motors[actuatorNumOutput];
			break;

		}

	}

	GPIO_InitTypeDef GPIO_InitStruct;
	TIM_IC_InitTypeDef TIM_ICInitStruct;

	uint16_t prescalerValue;
	__HAL_TIM_ENABLE_IT(&pwmTimers[actuator.actuatorArrayNum], actuator.timChannel);

	// Initialize GPIO
	HAL_GPIO_DeInit(ports[actuator.port], actuator.pin);

	GPIO_InitStruct.Pin       = actuator.pin;
	GPIO_InitStruct.Pull      = actuator.polarity;
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
	GPIO_InitStruct.Alternate = actuator.AF;

	HAL_GPIO_Init(ports[actuator.port], &GPIO_InitStruct);


	    // Compute the prescaler value 
	prescalerValue = (uint16_t)(SystemCoreClock / TimerPrescalerDivisor(actuator.timer)/ 1000000) - 1;
	// Time base configuration 
	pwmTimers[actuator.actuatorArrayNum].Instance           = timers[actuator.timer];
	pwmTimers[actuator.actuatorArrayNum].Init.Prescaler     = prescalerValue;
	pwmTimers[actuator.actuatorArrayNum].Init.CounterMode   = TIM_COUNTERMODE_UP;
	pwmTimers[actuator.actuatorArrayNum].Init.Period        = SEQUENCE_TIMEOUT;
	pwmTimers[actuator.actuatorArrayNum].Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_Base_Init(&pwmTimers[actuator.actuatorArrayNum]);
	HAL_TIM_IC_Init(&pwmTimers[actuator.actuatorArrayNum]);

	// PWM1 Mode configuration: Channel1
	TIM_ICInitStruct.ICPolarity = TIM_ICPOLARITY_FALLING;
	TIM_ICInitStruct.ICSelection = TIM_ICSELECTION_DIRECTTI;
	TIM_ICInitStruct.ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStruct.ICFilter = 0x00;
	
	if (HAL_TIM_IC_ConfigChannel(&pwmTimers[actuator.actuatorArrayNum], &TIM_ICInitStruct, actuator.timChannel) != HAL_OK)
	{
		/* Configuration Error */
		ErrorHandler(TIMER_INPUT_INIT_FAILIURE);
	}

	callbackFunctionArray[GetTimerCallbackFromTimerEnum(actuator.timer)] = SpmLaptimerCallback;

	//if timer has global interrupt
	if (actuator.timerIRQn)
	{
		HAL_NVIC_SetPriority(actuator.timerIRQn, 0, 0);
		HAL_NVIC_EnableIRQ(actuator.timerIRQn);
	}
	//Otherwise, it has dedicated interrupts
	else	
	{
		HAL_NVIC_SetPriority(actuator.timerIRQn_CC, 0, 0);
		HAL_NVIC_EnableIRQ(actuator.timerIRQn_CC);

		HAL_NVIC_SetPriority(actuator.timerIRQn_UP, 8, 8);
		HAL_NVIC_EnableIRQ(actuator.timerIRQn_UP);
	}
	


	if (HAL_TIM_IC_Start_IT(&pwmTimers[actuator.actuatorArrayNum], actuator.timChannel) != HAL_OK)
	{
		/* Starting Error */
		ErrorHandler(TIMER_INPUT_INIT_FAILIURE);
	}

	

}


uint8_t gpioStatus;
uint8_t fallingEdges;
uint8_t risingEdges;
uint8_t pulseCodeDebug;
LAP_STATE lapState = READING_PULSES;

void SpmLaptimerCallback(uint32_t callbackNumber)
{
//TIM CC Handler
	if (callbackNumber == TIM_CC)
	{
		timers[actuator.timer]->CNT = 0;
		__HAL_TIM_DISABLE_IT(&pwmTimers[actuator.actuatorArrayNum], actuator.timChannel);
		captureTime = HAL_TIM_ReadCapturedValue(&pwmTimers[actuator.actuatorArrayNum], actuator.timChannel);
	
		if (pulseIndex >= CODE_SIZE)
		{
			pulseIndex = 0;
			pulseCode = 0;
			__HAL_TIM_CLEAR_IT(&pwmTimers[actuator.actuatorArrayNum], actuator.timChannel);
			__HAL_TIM_ENABLE_IT(&pwmTimers[actuator.actuatorArrayNum], actuator.timChannel);
			return;
		}
		if (pulseIndex == 0)
		{
			__HAL_TIM_ENABLE_IT(&pwmTimers[actuator.actuatorArrayNum], TIM_IT_UPDATE);
			pulseCode = 0;
			memset(pulseDuration, 0, 32);
		}

		if ((captureTime > SHORT_MIN_PULSE + OFF_DURATION) && (captureTime < SHORT_MAX_PULSE + OFF_DURATION))	//0
		{
			pulseDuration[pulseIndex] = captureTime;
			pulseCode &= ~(1 << pulseIndex);
			pulseIndex++;
		}
		else if ((captureTime >= LONG_MIN_PULSE + OFF_DURATION) && (captureTime < LONG_MAX_PULSE + OFF_DURATION)) //1
		{
			pulseDuration[pulseIndex] = captureTime;
			pulseCode |= 1 << pulseIndex;
			pulseIndex++;
		}

		__HAL_TIM_CLEAR_IT(&pwmTimers[actuator.actuatorArrayNum], actuator.timChannel);
		__HAL_TIM_ENABLE_IT(&pwmTimers[actuator.actuatorArrayNum], actuator.timChannel);
	}

// TIM UP HANDLER
	else if (callbackNumber == TIM_UP)
	{
		__HAL_TIM_DISABLE_IT(&pwmTimers[actuator.actuatorArrayNum], TIM_IT_UPDATE);
	
	
		if (pulseIndex == CODE_SIZE 
#ifdef START_PULSES
		            && (pulseCode & 3) == 1 
#endif
#ifdef END_PULSES
		                        && (pulseCode >> (CODE_SIZE - 2) & 3) == 1
#endif
		)
		{
			//LED0_ON;
			lapDelay = 0;
			pulseCodeDebug = pulseCode;
			currentTime = InlineMillis();
			if ((pulseCode >> PULSE_SHIFT & 0x0F) == 0)	//lap captured. Disable CC and wait for enough update interrupts 
			{
				lap_timer.lapNumber++;
				lap_timer.lastLapTime = currentTime;
				lap_timer.gateNumber = (pulseCode >> PULSE_SHIFT & 0x0F); 
				lap_timer.gateTime = currentTime;
				lastTimeLap = currentTime;

			}
			else	//no lap. Disable update interrupt until a starting pulse is detected once again
			{
				lap_timer.gateNumber = (pulseCode >> PULSE_SHIFT & 0x0F); 
				lap_timer.gateTime = currentTime;
			}
		
	
			__HAL_TIM_CLEAR_IT(&pwmTimers[actuator.actuatorArrayNum], actuator.timChannel);
			__HAL_TIM_CLEAR_IT(&pwmTimers[actuator.actuatorArrayNum], TIM_IT_UPDATE);
			__HAL_TIM_ENABLE_IT(&pwmTimers[actuator.actuatorArrayNum], TIM_IT_UPDATE);
			__HAL_TIM_DISABLE_IT(&pwmTimers[actuator.actuatorArrayNum], actuator.timChannel);
			timers[actuator.timer]->ARR = DISABLE_PERIOD;
			lapState = GATE_PASSED;
		}
		else if (lapDelay > DISABLE_TIME)
		{
			//LED0_OFF;
			lapDelay = 0;
			timers[actuator.timer]->ARR = SEQUENCE_TIMEOUT;
			__HAL_TIM_CLEAR_IT(&pwmTimers[actuator.actuatorArrayNum], actuator.timChannel);
			__HAL_TIM_CLEAR_IT(&pwmTimers[actuator.actuatorArrayNum], TIM_IT_UPDATE);
			__HAL_TIM_DISABLE_IT(&pwmTimers[actuator.actuatorArrayNum], TIM_IT_UPDATE);
			__HAL_TIM_ENABLE_IT(&pwmTimers[actuator.actuatorArrayNum], actuator.timChannel);
			lapState = READING_PULSES;
			pulseIndex = 0;
			pulseCode = 0;
			return;
		}
		else if (lapState == GATE_PASSED)
		{
			lapDelay++;
			__HAL_TIM_CLEAR_IT(&pwmTimers[actuator.actuatorArrayNum], TIM_IT_UPDATE);
			__HAL_TIM_ENABLE_IT(&pwmTimers[actuator.actuatorArrayNum], TIM_IT_UPDATE);
		}
		else
		{
			__HAL_TIM_CLEAR_IT(&pwmTimers[actuator.actuatorArrayNum], TIM_IT_UPDATE);
			__HAL_TIM_DISABLE_IT(&pwmTimers[actuator.actuatorArrayNum], TIM_IT_UPDATE);
		}

		pulseIndex = 0;
		pulseCode = 0;
	}
}

