#pragma once

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_hal_conf.h"
#include "flash.h"

extern int FULL_32;
extern uint32_t TimerPrescalerDivisor(uint32_t timer);
extern int     VectorIrqInit(uint32_t address);

//asm delay used for not so accurate delays needed before the board is initialized
#define simpleDelay_ASM(us) do {\
	asm volatile (	"MOV R0,%[loops]\n\t"\
			"1: \n\t"\
			"SUB R0, #1\n\t"\
			"CMP R0, #0\n\t"\
			"BNE 1b \n\t" : : [loops] "r" (16*us) : "memory"\
		      );\
} while(0)

#define NothingFunction() return;

#define STM32_UUID ((uint32_t *)0x1FFF7A10)

#if defined(stm32f411xe) || defined(STM32F411xE)

#include <stm32f411xe.h>

#define ResetReleaseUsart1() __USART1_FORCE_RESET();__USART1_RELEASE_RESET();
#define ResetReleaseUsart2() __USART2_FORCE_RESET();__USART2_RELEASE_RESET();	
#define ResetReleaseUsart3() NothingFunction();
#define ResetReleaseUsart4() NothingFunction();
#define ResetReleaseUsart5() NothingFunction();
#define ResetReleaseUsart6() NothingFunction();

#define _PORTA 0
#define _PORTB 1
#define _PORTC 2
#define _PORTD 3
#define _PORTE 4
#define _PORTF 5
#define _PORTG 6
#define _PORTH 7
#define _PORTI 8

#define _ADC1  ADC1
#define _ADC2  NULL
#define _ADC3  NULL

#define _TIM1  TIM1
#define _TIM2  TIM2
#define _TIM3  TIM3
#define _TIM4  TIM4
#define _TIM5  TIM5
#define _TIM9  TIM9
#define _TIM10 TIM10
#define _TIM11 TIM11
#define _TIM6  NULL
#define _TIM7  NULL
#define _TIM8  NULL
#define _TIM12 NULL
#define _TIM13 NULL
#define _TIM14 NULL

#define _GPIOA GPIOA
#define _GPIOB GPIOB
#define _GPIOC GPIOC
#define _GPIOD GPIOD
#define _GPIOE GPIOE
#define _GPIOF NULL
#define _GPIOG NULL
#define _GPIOH NULL
#define _GPIOI NULL
#define _GPIOJ NULL
#define _GPIOK NULL

#define _USART1 USART1
#define _USART1s 1
#define _USART2 USART2
#define _USART2s 1
#define _USART3 NULL
#define _USART3s 0
#define _USART4 NULL
#define _USART4s 0
#define _USART5 NULL
#define _USART5s 0
#define _USART6 USART6
#define _USART6s 1
#define _USART7 NULL
#define _USART7s 0

//STM32F4 UID address
#define DEVICE_ID1					0x1FFF7A10
#define DEVICE_ID2					0x1FFF7A14
#define DEVICE_ID3					0x1FFF7A18

#define USE_RFBL 
#define ADDRESS_FLASH_START		(uint32_t)(0x08000000)
#define ADDRESS_RECOVERY_START	(uint32_t)(0x08000000)
#define ADDRESS_RFBL_START		(uint32_t)(0x08008000)
#define ADDRESS_CONFIG_START	(uint32_t)(0x08010000)
#define ADDRESS_RFFW_START		(uint32_t)(0x08020000)
#define ADDRESS_ESC_START		(uint32_t)(0x08060000)
#define ADDRESS_FLASH_END		(uint32_t)(0x080FFFF0)

#endif

#if defined(stm32f405xx) || defined(STM32F405xx)

#include <stm32f405xx.h>

#define ResetReleaseUsart1() __USART1_FORCE_RESET();__USART1_RELEASE_RESET();
#define ResetReleaseUsart2() __USART2_FORCE_RESET();__USART2_RELEASE_RESET();
#define ResetReleaseUsart3() __USART3_FORCE_RESET();__USART3_RELEASE_RESET();
#define ResetReleaseUsart4() __USART4_FORCE_RESET();__USART4_RELEASE_RESET();
#define ResetReleaseUsart5() __USART5_FORCE_RESET();__USART5_RELEASE_RESET();
#define ResetReleaseUsart6() __USART6_FORCE_RESET();__USART6_RELEASE_RESET();

#define _PORTA 0
#define _PORTB 1
#define _PORTC 2
#define _PORTD 3
#define _PORTE 4
#define _PORTF 5
#define _PORTG 6
#define _PORTH 7
#define _PORTI 8

#define _ADC1  ADC1
#define _ADC2  ADC2
#define _ADC3  ADC3

#define _TIM1  TIM1
#define _TIM2  TIM2
#define _TIM3  TIM3
#define _TIM4  TIM4
#define _TIM5  TIM5
#define _TIM6  TIM6
#define _TIM7  TIM7
#define _TIM8  TIM8
#define _TIM9  TIM9
#define _TIM10 TIM10
#define _TIM11 TIM11
#define _TIM12 TIM12
#define _TIM13 TIM13
#define _TIM14 TIM14

#define _GPIOA GPIOA
#define _GPIOB GPIOB
#define _GPIOC GPIOC
#define _GPIOD GPIOD
#define _GPIOE GPIOE
#define _GPIOF GPIOF
#define _GPIOG GPIOG
#define _GPIOH GPIOH
#define _GPIOI GPIOI
#define _GPIOJ NULL
#define _GPIOK NULL

#define _USART1  USART1
#define _USART1s 1
#define _USART2  USART2
#define _USART2s 1
#define _USART3  USART3
#define _USART3s 1
#define _USART4  UART4
#define _USART4s 0
#define _USART5  UART5
#define _USART5s 0
#define _USART6  USART6
#define _USART6s 1
#define _USART7  USART7
#define _USART7s 1

//STM32F4 UID address
#define DEVICE_ID1					0x1FFF7A10
#define DEVICE_ID2					0x1FFF7A14
#define DEVICE_ID3					0x1FFF7A18

#define USE_RFBL 
#define ADDRESS_FLASH_START		(uint32_t)(0x08000000)
#define ADDRESS_RECOVERY_START	(uint32_t)(0x08000000)
#define ADDRESS_RFBL_START		(uint32_t)(0x08008000)
#define ADDRESS_CONFIG_START	(uint32_t)(0x08010000)
#define ADDRESS_RFFW_START		(uint32_t)(0x08020000)
#define ADDRESS_ESC_START		(uint32_t)(0x08070000)
#define ADDRESS_FLASH_END		(uint32_t)(0x080FFFF0)

#endif

#if defined(stm32f446xx) || defined(STM32F446xx)

#include <stm32f446xx.h>

#define ResetReleaseUsart1() __USART1_FORCE_RESET();__USART1_RELEASE_RESET();
#define ResetReleaseUsart2() __USART2_FORCE_RESET();__USART2_RELEASE_RESET();
#define ResetReleaseUsart3() __USART3_FORCE_RESET();__USART3_RELEASE_RESET();
#define ResetReleaseUsart4() __USART4_FORCE_RESET();__USART4_RELEASE_RESET();
#define ResetReleaseUsart5() __USART5_FORCE_RESET();__USART5_RELEASE_RESET();
#define ResetReleaseUsart6() __USART6_FORCE_RESET();__USART6_RELEASE_RESET();

#define _PORTA 0
#define _PORTB 1
#define _PORTC 2
#define _PORTD 3
#define _PORTE 4
#define _PORTF 5
#define _PORTG 6
#define _PORTH 7
#define _PORTI 8

#define _ADC1  ADC1
#define _ADC2  ADC2
#define _ADC3  ADC3

#define _GPIOA GPIOA
#define _GPIOB GPIOB
#define _GPIOC GPIOC
#define _GPIOD GPIOD
#define _GPIOE GPIOE
#define _GPIOF GPIOF
#define _GPIOG NULL
#define _GPIOH NULL
#define _GPIOI NULL
#define _GPIOJ NULL
#define _GPIOK NULL

#define _TIM1  TIM1
#define _TIM2  TIM2
#define _TIM3  TIM3
#define _TIM4  TIM4
#define _TIM5  TIM5
#define _TIM6  TIM6
#define _TIM7  TIM7
#define _TIM8  TIM8
#define _TIM9  TIM9
#define _TIM10 TIM10
#define _TIM11 TIM11
#define _TIM12 TIM12
#define _TIM13 TIM13
#define _TIM14 TIM14

#define _USART1  USART1
#define _USART1s 0
#define _USART2  USART2
#define _USART2s 1
#define _USART3  USART3
#define _USART3s 1
#define _USART4  UART4
#define _USART4s 1
#define _USART5  UART5
#define _USART5s 1
#define _USART6  USART6
#define _USART6s 0
#define _USART7  USART7
#define _USART7s 0

//STM32F4 UID address
#define DEVICE_ID1					0x1FFF7A10
#define DEVICE_ID2					0x1FFF7A14
#define DEVICE_ID3					0x1FFF7A18

#define USE_RFBL 
#define ADDRESS_FLASH_START		(uint32_t)(0x08000000)
#define ADDRESS_RECOVERY_START	(uint32_t)(0x08000000)
#define ADDRESS_RFBL_START		(uint32_t)(0x08008000)
#define ADDRESS_CONFIG_START	(uint32_t)(0x08010000)
#define ADDRESS_RFFW_START		(uint32_t)(0x08020000)
#define ADDRESS_ESC_START		(uint32_t)(0x08058000)
#define ADDRESS_FLASH_END		(uint32_t)(0x0807FFF0)

#endif