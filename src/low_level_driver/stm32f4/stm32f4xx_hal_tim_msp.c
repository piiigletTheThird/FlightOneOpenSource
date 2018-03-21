#include "includes.h"

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
	if (htim_base->Instance == TIM2) {
		/* Peripheral clock enable */
		__HAL_RCC_TIM2_CLK_ENABLE();

		/* Peripheral interrupt init */
		//HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
		//HAL_NVIC_EnableIRQ(TIM2_IRQn);
	} else if (htim_base->Instance == TIM3) {
		/* Peripheral clock enable */
		__HAL_RCC_TIM3_CLK_ENABLE();

		/* Peripheral interrupt init */
		//HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
		//HAL_NVIC_EnableIRQ(TIM3_IRQn);
	}
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{
	if (htim_base->Instance == TIM2) {
		/* Peripheral clock disable */
		__HAL_RCC_TIM2_CLK_DISABLE();

		/* Peripheral interrupt DeInit*/
		//HAL_NVIC_DisableIRQ(TIM2_IRQn);
	} else if (htim_base->Instance == TIM3) {
		/* Peripheral clock disable */
		__HAL_RCC_TIM3_CLK_DISABLE();

		/* Peripheral interrupt DeInit */
		//HAL_NVIC_DisableIRQ(TIM3_IRQn);
	}
}
