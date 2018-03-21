#include "includes.h"

volatile uint32_t disarmPulseValue3d;
volatile uint32_t disarmPulseValue;
volatile uint32_t calibratePulseValue;
volatile uint32_t calibrateWalledPulseValue;
volatile uint32_t idlePulseValue;
volatile uint32_t pulseValueRange;
volatile uint32_t boostedIdlePulseValue;
volatile uint32_t boostedPulseValueRange;
volatile uint32_t usedEscProtocol; //currently active ESC protocol

static void InitActuatorTimer(motor_type actuator, uint32_t pwmHz, uint32_t timerHz);
static uint32_t ThrottleToDDshot(float throttle, float idle);
static int IsDshotCurrentlyActive(void);
volatile float previousMotorOutput[MAX_MOTOR_NUMBER];


//motor_output_array motorOutputArray[MAX_MOTOR_NUMBER];

static int IsDshotCurrentlyActive(void)
{
	switch(usedEscProtocol)
	{
		case ESC_DSHOT1200:
		case ESC_DSHOT600:
		case ESC_DSHOT300:
		case ESC_DSHOT150:
			return(1);
			break;
		default:
			return(0);
			break;
	}
}

void DeInitActuators(void)
{
	uint32_t motorNum;
	uint32_t outputNumber;
	for (motorNum = 0; motorNum < MAX_MOTOR_NUMBER; motorNum++)
	{
		outputNumber = mainConfig.mixerConfig.motorOutput[motorNum];

		if (board.motors[outputNumber].enabled == ENUM_ACTUATOR_TYPE_MOTOR)
		{
			HAL_GPIO_DeInit(ports[board.motors[outputNumber].port], board.motors[outputNumber].pin);
			HAL_TIM_PWM_DeInit(&pwmTimers[board.motors[outputNumber].actuatorArrayNum]);
			HAL_TIM_Base_DeInit(&pwmTimers[board.motors[outputNumber].actuatorArrayNum]);
		}
	}
}

void InitActuators(uint32_t escProtocol, uint32_t escFrequency)
{
	float disarmUs;    // shortest pulse width (disarmed)
	float disarmUs3d;  // shortest pulse width (disarmed)
	float calibrateUs; // shortest pulse width (calibrate)
	float calibrateWalledUs;
	float walledUs;    // longest pulse width (full throttle)
	float idleUs;      // idle pulse width (armed, zero throttle)
	float boostedIdleUs;      // idle pulse width during high speed manuver (armed, zero throttle)
	uint32_t motorNum;
	uint32_t outputNumber;

	usedEscProtocol = escProtocol;
	// timerHz *must* be a proper divisor of the timer frequency
	// REVOLT - 48 MHz (overclocked from 42 MHz)
	uint32_t timerHz; // frequency of the timer
	uint32_t pwmHz;   // max update frequency for protocol
	uint32_t walledPulseValue;


	for (uint32_t x=0;x<MAX_MOTOR_NUMBER;x++)
		previousMotorOutput[x] = 0;
	


	switch (escProtocol)
	{
		case ESC_PWM:
			disarmUs3d  = 1500;
			disarmUs    = 990;
			calibrateUs = 1000;
			walledUs    = 2000;
			calibrateWalledUs = 1950;
			pwmHz       = 490;
			timerHz     = 1000000;
			break;
		case ESC_ONESHOT:
			disarmUs3d  = 187.5;
			disarmUs    = 120;
			calibrateUs = 125;
			walledUs    = 250;
			calibrateWalledUs = 240;
			pwmHz       = 3900;
			timerHz     = 8000000;
			break;
		case ESC_ONESHOT42:
			disarmUs    = 40;
			calibrateUs = 41.667;
			walledUs    = 83.334; // round up for int math
			calibrateWalledUs = 82.0;
			pwmHz       = 11500;
			timerHz     = 24000000;
			break;
		case ESC_DSHOT150:
		case ESC_DSHOT300:
		case ESC_DSHOT600:
		case ESC_DSHOT1200:
		case ESC_DDSHOT:
			disarmUs    = 0;
			calibrateUs = 0;
			walledUs    = 2000;
			calibrateWalledUs = 2000;
			idleUs      = 48;
			pwmHz       = 0;
			timerHz     = 0;
			disarmPulseValue = 0;
			idlePulseValue   = 48;
			walledPulseValue = 2048;
			pulseValueRange  = 2000;
			InitDshotOutputOnMotors(escProtocol);
			return;
			break;
		case ESC_MULTISHOT25:
			disarmUs3d  = 6.25;
			disarmUs    = 2.400;
			calibrateUs = 2.500;
			walledUs    = 10.00;
			calibrateWalledUs = 9.5;
			pwmHz       = 32000;
			timerHz     = 48000000; // full resolution
			//timerHz   = 12000000; // 1/4 resolution
			break;
		case ESC_MULTISHOT125:
			disarmUs3d  = 3.125;
			disarmUs    = 1.220;
			calibrateUs = 1.250;
			walledUs    = 5.000;
			calibrateWalledUs = 4.80;
			pwmHz       = 32000;
			timerHz     = 48000000; // full resolution
			//timerHz   = 12000000; // 1/4 resolution
			break;
		case ESC_MULTISHOT:
		default:
			disarmUs3d  = 13.75;
			disarmUs    = 4.900;
			calibrateUs = 5.000;
			walledUs    = 22.54;
			calibrateWalledUs = 22.50;
			pwmHz       = 32000;
			timerHz     = 48000000; // full resolution
			//timerHz     = 54000000; // full resolution
			//timerHz   = 12000000; // 1/4 resolution
			break;
	}


	// constrain motor update frequency between the 50Hz and the max allowed for the ESC protocol
	pwmHz = CONSTRAIN(escFrequency, 50, pwmHz);
	// compute idle PWM width from idlePercent
	idleUs = ((walledUs - disarmUs) * (mainConfig.mixerConfig.idlePercent * 0.01) ) + disarmUs;
	boostedIdleUs = ((walledUs - disarmUs) * (mainConfig.mixerConfig.idlePercent * 0.02) ) + disarmUs;
	
	disarmPulseValue3d         = ((uint32_t)(disarmUs3d * timerHz))        / 1000000;
	disarmPulseValue           = ((uint32_t)(disarmUs * timerHz))          / 1000000;
	calibratePulseValue        = ((uint32_t)(calibrateUs * timerHz))       / 1000000;
	idlePulseValue             = ((uint32_t)(idleUs * timerHz))            / 1000000;
	boostedIdlePulseValue      = ((uint32_t)(boostedIdleUs * timerHz))     / 1000000;
	walledPulseValue           = ((uint32_t)(walledUs * timerHz))          / 1000000;
	calibrateWalledPulseValue  = ((uint32_t)(calibrateWalledUs * timerHz)) / 1000000;

	pulseValueRange            = walledPulseValue - idlePulseValue;        //throttle for motor output is float motorThrottle * pulseValueRange + idlePulseValue;
	boostedPulseValueRange     = walledPulseValue - boostedIdlePulseValue; //throttle for motor output is float motorThrottle * pulseValueRange + idlePulseValue;
	
	for (motorNum = 0; motorNum < MAX_MOTOR_NUMBER; motorNum++)
	{
		outputNumber = mainConfig.mixerConfig.motorOutput[motorNum];
		if (board.motors[outputNumber].enabled == ENUM_ACTUATOR_TYPE_MOTOR)
		{
			InitActuatorTimer(board.motors[outputNumber], pwmHz, timerHz);
		}
	}

	DelayMs(2); //give timer time to stabilize.

	// Start the timers
	for (motorNum = 0; motorNum < MAX_MOTOR_NUMBER; motorNum++)
	{
		outputNumber = mainConfig.mixerConfig.motorOutput[motorNum];
		if (board.motors[outputNumber].enabled == ENUM_ACTUATOR_TYPE_MOTOR)
		{
			HAL_TIM_Base_Start(&pwmTimers[board.motors[outputNumber].actuatorArrayNum]);

			if(board.motors[outputNumber].isNChannel)
				HAL_TIMEx_PWMN_Start(&pwmTimers[board.motors[outputNumber].actuatorArrayNum], board.motors[outputNumber].timChannel);
			else
				HAL_TIM_PWM_Start(&pwmTimers[board.motors[outputNumber].actuatorArrayNum], board.motors[outputNumber].timChannel);
			
		}
	}

	DelayMs(2); //give timer time to stabilize.

}

void InitActuatorTimer(motor_type actuator, uint32_t pwmHz, uint32_t timerHz)
{
	uint16_t timerPrescaler = 0;

	GPIO_InitTypeDef GPIO_InitStruct;
	TIM_OC_InitTypeDef sConfigOC;

	timerPrescaler = (uint16_t)(SystemCoreClock / TimerPrescalerDivisor(actuator.timer) / timerHz) - 1;

	// Initialize GPIO
	HAL_GPIO_DeInit(ports[actuator.port], actuator.pin);

	GPIO_InitStruct.Pin       = actuator.pin;
	GPIO_InitStruct.Pull      = (actuator.polarity == TIM_OCPOLARITY_LOW) ? GPIO_PULLDOWN : GPIO_PULLUP;
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
	GPIO_InitStruct.Alternate = actuator.AF;

	HAL_GPIO_Init(ports[actuator.port], &GPIO_InitStruct);

	// Initialize timer
	pwmTimers[actuator.actuatorArrayNum].Instance           = timers[actuator.timer];
	pwmTimers[actuator.actuatorArrayNum].Init.Prescaler     = timerPrescaler;
	pwmTimers[actuator.actuatorArrayNum].Init.CounterMode   = TIM_COUNTERMODE_UP;
	pwmTimers[actuator.actuatorArrayNum].Init.Period        = (timerHz / pwmHz) - 1;
	pwmTimers[actuator.actuatorArrayNum].Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_Base_Init(&pwmTimers[actuator.actuatorArrayNum]);

	HAL_TIM_PWM_Init(&pwmTimers[actuator.actuatorArrayNum]);

	// Initialize timer pwm channel
	sConfigOC.OCMode      = TIM_OCMODE_PWM1;
	sConfigOC.Pulse       = 0;
	sConfigOC.OCFastMode  = TIM_OCFAST_DISABLE;

	if(actuator.isNChannel)
	{
		sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
		sConfigOC.OCNPolarity  = (actuator.polarity == TIM_OCPOLARITY_LOW) ? TIM_OCNPOLARITY_HIGH : TIM_OCNPOLARITY_LOW;
		sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
		sConfigOC.OCPolarity  = (actuator.polarity == TIM_OCPOLARITY_LOW) ? TIM_OCPOLARITY_HIGH : TIM_OCPOLARITY_LOW;
	}
	else
	{
		sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
		sConfigOC.OCPolarity   = (actuator.polarity == TIM_OCPOLARITY_LOW) ? TIM_OCPOLARITY_HIGH : TIM_OCPOLARITY_LOW;
		sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
		sConfigOC.OCNPolarity  = (actuator.polarity == TIM_OCPOLARITY_LOW) ? TIM_OCNPOLARITY_HIGH : TIM_OCNPOLARITY_LOW;
	}

	HAL_TIM_PWM_ConfigChannel(&pwmTimers[actuator.actuatorArrayNum], &sConfigOC, actuator.timChannel);

}

static uint32_t ThrottleToDDshot(float throttle, float idle)
{

	if (idle > 0)
	{
		return(lrintf(InlineChangeRangef(throttle, 1.0, 0.0, 1023.0f, (1023.0f * idle * 0.01f))));
	}
	else
	{
		return(0);
	}

}

void OutputActuators(volatile float motorOutput[], volatile float servoOutput[])
{
	(void)servoOutput;

	uint32_t outputNumber;
	uint32_t motorNum;
	uint8_t  serialOutBuffer[2];
	float    tempOutputF;
	float difference;


	if (mainConfig.tuneProfile[activeProfile].filterConfig[0].resRedux)
	{

		/* crap
		for (motorNum = 0; motorNum < MAX_MOTOR_NUMBER; motorNum++)
		{
			difference = abs(motorOutput[motorNum] - previousMotorOutput[motorNum]);
			if (difference <= .005)
			{
				motorOutput[motorNum] = previousMotorOutput[motorNum];
			}
			else
			{
				if (difference > .025)
				{
					if (motorOutput[motorNum] > previousMotorOutput[motorNum])
					{
						previousMotorOutput[motorNum] = motorOutput[motorNum] = motorOutput[motorNum] + .025; 
					}
					else
					{
						previousMotorOutput[motorNum] = motorOutput[motorNum] = motorOutput[motorNum] - .025; 
					}

				} 
				else
				{
					previousMotorOutput[motorNum] = MotorOutput[motorNum];
				}
			}

		}*/


		for (int motorNum = 0; motorNum < MAX_MOTOR_NUMBER; motorNum++)
		{
			difference = fabsf(motorOutput[motorNum] - previousMotorOutput[motorNum]);
			if (difference <= .0025)
			{
				motorOutput[motorNum] = previousMotorOutput[motorNum];
			}
			else
			{
				if (difference > .00050)
				{
					if (motorOutput[motorNum] > previousMotorOutput[motorNum])
					{
						motorOutput[motorNum] = previousMotorOutput[motorNum] + .00050; /* increase by max 25 */
						previousMotorOutput[motorNum] = motorOutput[motorNum];
					}
					else
					{
						motorOutput[motorNum] = previousMotorOutput[motorNum] - .00050; /* increase by max 25 */
						previousMotorOutput[motorNum] = motorOutput[motorNum];
					}
	
				} 
				else
				{
					previousMotorOutput[motorNum] = motorOutput[motorNum];
				}
			}
	
		}
		

/*
		for (motorNum = 0; motorNum < MAX_MOTOR_NUMBER; motorNum++)
		{
			//range 0 - 1 to 0 - 255 and round it to provide 256 bits of resolution
			tempOutputF = roundf(motorOutput[motorNum] * 255.0f);
			//change range back to 0 - 1
			motorOutput[motorNum] = InlineChangeRangef(tempOutputF, 255.0f, 0.0f, 1.0f, 0.0f);
		}


		*/
	}
	if (boardArmed)
	{
		for (motorNum = 0; motorNum < MAX_MOTOR_NUMBER; motorNum++)
		{

			outputNumber = mainConfig.mixerConfig.motorOutput[motorNum];
			if (board.motors[outputNumber].enabled == ENUM_ACTUATOR_TYPE_MOTOR) 
			{
				if ( usedEscProtocol == ESC_DDSHOT )
				{
					OutputDDShotDma(board.motors[outputNumber], mainConfig.mixerConfig.bitReverseEsc[motorNum], ThrottleToDDshot(motorOutput[motorNum], mainConfig.mixerConfig.idlePercent) );
						
				} 
				else if ( IsDshotCurrentlyActive() )				
				{
					ThrottleToDshot(serialOutBuffer, motorOutput[motorNum], mainConfig.mixerConfig.idlePercent, mainConfig.mixerConfig.bitReverseEsc[motorNum]);

					OutputSerialDmaByte(serialOutBuffer, 2, board.motors[outputNumber], 1, 0, 1);
				}
				else
				{
					*ccr[board.motors[outputNumber].timCCR] = (uint16_t)(motorOutput[motorNum] * (float)pulseValueRange) + idlePulseValue;
				}
			}
		}
	}
	else if (calibrateMotors)
	{
		if (motorOutput[0] < 0.1)
		{
			for (motorNum = 0; motorNum < MAX_MOTOR_NUMBER; motorNum++)
			{
				outputNumber = mainConfig.mixerConfig.motorOutput[motorNum];
				if (board.motors[outputNumber].enabled == ENUM_ACTUATOR_TYPE_MOTOR)
				{
					if ( !IsDshotCurrentlyActive() )
					{
						*ccr[board.motors[outputNumber].timCCR] = calibratePulseValue;
					}
				}
			}
		}
		else
		{
			for (motorNum = 0; motorNum < MAX_MOTOR_NUMBER; motorNum++)
			{
				outputNumber = mainConfig.mixerConfig.motorOutput[motorNum];
				if (board.motors[outputNumber].enabled == ENUM_ACTUATOR_TYPE_MOTOR) 
				{
					if ( !IsDshotCurrentlyActive() )
					{
						*ccr[board.motors[outputNumber].timCCR] = calibrateWalledPulseValue;
					}
				}
			}
		}
	}
	else
	{
		for (motorNum = 0; motorNum < MAX_MOTOR_NUMBER; motorNum++)
		{
			outputNumber = mainConfig.mixerConfig.motorOutput[motorNum];
			if (board.motors[outputNumber].enabled == ENUM_ACTUATOR_TYPE_MOTOR)
			{
				if ( usedEscProtocol == ESC_DDSHOT )
				{
					OutputDDShotDma(board.motors[outputNumber], mainConfig.mixerConfig.bitReverseEsc[motorNum], ThrottleToDDshot(0, 0) );
				} 
				else if ( IsDshotCurrentlyActive() )
				{
					ThrottleToDshot(serialOutBuffer, 0, 0, 0);
					OutputSerialDmaByte(serialOutBuffer, 2, board.motors[outputNumber], 1, 0, 1);
				}
				else
				{
					if (threeDeeMode)
						*ccr[board.motors[outputNumber].timCCR] = disarmPulseValue3d;
					else
						*ccr[board.motors[outputNumber].timCCR] = disarmPulseValue;
				}
			}
		}
	}
}


void ZeroActuators(uint32_t delayUs)
{

	uint32_t outputNumber;
	uint8_t  serialOutBuffer[2];

	if ( (usedEscProtocol != ESC_DDSHOT) && (usedEscProtocol != ESC_DSHOT1200) && (usedEscProtocol != ESC_DSHOT600) && (usedEscProtocol != ESC_DSHOT300) && (usedEscProtocol != ESC_DSHOT150) )
		__disable_irq();

	for (uint32_t motorNum=0; motorNum < MAX_MOTOR_NUMBER; motorNum++)
	{
		outputNumber = mainConfig.mixerConfig.motorOutput[motorNum];
		if (board.motors[outputNumber].enabled == ENUM_ACTUATOR_TYPE_MOTOR)
		{
			if ( usedEscProtocol == ESC_DDSHOT )
			{
				OutputDDShotDma(board.motors[outputNumber], mainConfig.mixerConfig.bitReverseEsc[motorNum], ThrottleToDDshot(0, 0) );
			} 
			else if ( IsDshotCurrentlyActive() )
			{
				ThrottleToDshot(serialOutBuffer, 0, 0, 0);
				OutputSerialDmaByte(serialOutBuffer, 2, board.motors[outputNumber], 1, 0, 1); //buffer with data, number of bytes, actuator to output on, msb, no serial frame
			}
			else
			{
				if (threeDeeMode)
					*ccr[board.motors[outputNumber].timCCR] = disarmPulseValue3d;
				else
					*ccr[board.motors[outputNumber].timCCR] = disarmPulseValue;
			}
		}
	}

	if (delayUs)
		simpleDelay_ASM(delayUs);

	if ( (usedEscProtocol != ESC_DDSHOT) && (usedEscProtocol != ESC_DSHOT1200) && (usedEscProtocol != ESC_DSHOT600) && (usedEscProtocol != ESC_DSHOT300) && (usedEscProtocol != ESC_DSHOT150) )
		__enable_irq();

}

void IdleActuator(uint32_t motorNum)
{

	uint32_t outputNumber;
	uint8_t serialOutBuffer[2];

	outputNumber = mainConfig.mixerConfig.motorOutput[motorNum];

	if (board.motors[outputNumber].enabled == ENUM_ACTUATOR_TYPE_MOTOR)
	{
		if ( usedEscProtocol == ESC_DDSHOT )
		{
			OutputDDShotDma(board.motors[outputNumber], mainConfig.mixerConfig.bitReverseEsc[motorNum], ThrottleToDDshot(0.001f, mainConfig.mixerConfig.idlePercent) );
		} 
		else if ( IsDshotCurrentlyActive() )
		{
			ThrottleToDshot(serialOutBuffer, 0.001, mainConfig.mixerConfig.idlePercent, mainConfig.mixerConfig.bitReverseEsc[motorNum]);
			OutputSerialDmaByte(serialOutBuffer, 2, board.motors[outputNumber], 1, 0, 1);
		}
		else
		{
			if (threeDeeMode)
				*ccr[board.motors[outputNumber].timCCR] = (uint16_t)disarmPulseValue3d; //idle is handled by mixer in 3D mode
			else
				*ccr[board.motors[outputNumber].timCCR] = (uint16_t)idlePulseValue;
		}
	}

}

//
void DirectActuator(uint32_t motorNum, float throttle)
{

	uint32_t outputNumber;
	uint8_t serialOutBuffer[2];

	outputNumber = mainConfig.mixerConfig.motorOutput[motorNum];

	if (board.motors[outputNumber].enabled == ENUM_ACTUATOR_TYPE_MOTOR)
	{
		if ( usedEscProtocol == ESC_DDSHOT )
		{
			if (throttle < 0.0f)
			{
				//zero
				OutputDDShotDma(board.motors[outputNumber], mainConfig.mixerConfig.bitReverseEsc[motorNum], ThrottleToDDshot(0.0f, 0.0f) );
			}
			else if (throttle >= 0.001f)
			{
				//throttle
				OutputDDShotDma(board.motors[outputNumber], mainConfig.mixerConfig.bitReverseEsc[motorNum], ThrottleToDDshot(throttle, mainConfig.mixerConfig.idlePercent) );
			}
			else
			{
				//idle
				OutputDDShotDma(board.motors[outputNumber], mainConfig.mixerConfig.bitReverseEsc[motorNum], ThrottleToDDshot(0.001f, mainConfig.mixerConfig.idlePercent) );
			}
		} 
		else if ( IsDshotCurrentlyActive() )
		{
			if (throttle < 0.0f)
			{
				//zero
				ThrottleToDshot(serialOutBuffer, 0, 0, 0);
				OutputSerialDmaByte(serialOutBuffer, 2, board.motors[outputNumber], 1, 0, 1); //buffer with data, number of bytes, actuator to output on, msb, no serial frame
			}
			else if (throttle >= 0.001f)
			{
				//throttle
				ThrottleToDshot(serialOutBuffer, throttle, mainConfig.mixerConfig.idlePercent, mainConfig.mixerConfig.bitReverseEsc[motorNum]);
				OutputSerialDmaByte(serialOutBuffer, 2, board.motors[outputNumber], 1, 0, 1); //buffer with data, number of bytes, actuator to output on, msb, no serial frame
			}
			else
			{
				//idle
				ThrottleToDshot(serialOutBuffer, 0.001f, mainConfig.mixerConfig.idlePercent, mainConfig.mixerConfig.bitReverseEsc[motorNum]);
				OutputSerialDmaByte(serialOutBuffer, 2, board.motors[outputNumber], 1, 0, 1); //buffer with data, number of bytes, actuator to output on, msb, no serial frame
			}
		}
		else
		{
			if (throttle < 0.0f)
			{
				//zero
				*ccr[board.motors[outputNumber].timCCR] = (uint16_t)disarmPulseValue;
			}
			else if (throttle >= 0.001f)
			{
				//throttle
				*ccr[board.motors[outputNumber].timCCR] = (uint16_t)(throttle * (float)pulseValueRange) + idlePulseValue;
			}
			else
			{
				//idle
				*ccr[board.motors[outputNumber].timCCR] = (uint16_t)idlePulseValue;
			}
		}
	}
}