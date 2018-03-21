#include "includes.h"

void InitTransponderTimer(void)
{

	uint32_t pwmHz = 460000;
	uint32_t timerHz = 48000000;
	uint16_t timerPrescaler = 0;

	uint32_t transponderTimer    = BUZZER_TIM;
	uint32_t transponderTimCh    = BUZZER_TIM_CH;
	uint32_t transponderPort     = BUZZER_GPIO_Port;
	uint32_t transponderPin      = BUZZER_GPIO_Pin;
	uint32_t transponderPolarity = BUZZER_POLARITY;
	uint32_t transponderAf       = BUZZER_ALTERNATE;
	uint32_t transponderArrayNum = 15;

	//TIM3_CH1
	//PB4
/*
#define BUZZER_GPIO_Port        _PORTB
#define BUZZER_GPIO_Pin         GPIO_PIN_4
#define BUZZER_TIM				ENUMTIM3
#define BUZZER_ALTERNATE		GPIO_AF1_TIM3
#define BUZZER_TIM_CH			TIM_CHANNEL_1
#define BUZZER_TIM_CCR			TIM3CCR1
#define BUZZER_POLARITY			TIM_OCPOLARITY_LOW
*/
	//board.buzzerPort
	//board.buzzerPin

	GPIO_InitTypeDef GPIO_InitStruct;
	TIM_OC_InitTypeDef sConfigOC;
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_ClockConfigTypeDef sClockSourceConfig;

	timerPrescaler = (uint16_t)(SystemCoreClock / TimerPrescalerDivisor(transponderTimer) / timerHz) - 1;

	// Initialize GPIO
	HAL_GPIO_DeInit(ports[transponderPort], transponderPin);

	GPIO_InitStruct.Pin       = transponderPin;
	GPIO_InitStruct.Pull      = (transponderPolarity == TIM_OCPOLARITY_LOW) ? GPIO_PULLDOWN : GPIO_PULLUP;
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
	GPIO_InitStruct.Alternate = transponderAf;

	HAL_GPIO_Init(ports[transponderPort], &GPIO_InitStruct);

	// Initialize timer
	pwmTimers[transponderArrayNum].Instance           = timers[transponderTimer];
	pwmTimers[transponderArrayNum].Init.Prescaler     = timerPrescaler;
	pwmTimers[transponderArrayNum].Init.CounterMode   = TIM_COUNTERMODE_UP;
	pwmTimers[transponderArrayNum].Init.Period        = (timerHz / pwmHz) - 1;
	pwmTimers[transponderArrayNum].Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_Base_Init(&pwmTimers[transponderArrayNum]);

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	HAL_TIM_ConfigClockSource(&pwmTimers[transponderArrayNum], &sClockSourceConfig);

	HAL_TIM_PWM_Init(&pwmTimers[transponderArrayNum]);

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&pwmTimers[transponderArrayNum], &sMasterConfig);

	// Initialize timer pwm channel
	sConfigOC.OCMode      = TIM_OCMODE_PWM2;
	sConfigOC.Pulse       = 10;
	sConfigOC.OCPolarity  = transponderPolarity;
	sConfigOC.OCFastMode  = TIM_OCFAST_ENABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;

	HAL_TIM_PWM_ConfigChannel(&pwmTimers[transponderArrayNum], &sConfigOC, transponderTimCh);

	HAL_TIM_Base_Start(&pwmTimers[transponderArrayNum]);
	HAL_TIM_PWM_Start(&pwmTimers[transponderArrayNum], transponderTimCh);
}
