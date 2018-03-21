#include "includes.h"

void EXTI_Init(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority, uint32_t gpioModeIt, uint32_t gpioPull)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.Pin  = GPIO_Pin;
    GPIO_InitStructure.Mode = gpioModeIt;
    GPIO_InitStructure.Pull = gpioPull;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStructure);

    /* EXTI interrupt init */
    HAL_NVIC_SetPriority(IRQn, PreemptPriority, SubPriority);
    HAL_NVIC_EnableIRQ(IRQn);
}

void EXTI_Deinit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, IRQn_Type IRQn)
{
    HAL_GPIO_DeInit(GPIOx, GPIO_Pin);
    HAL_NVIC_DisableIRQ(IRQn);
}
