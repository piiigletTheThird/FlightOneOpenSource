#include "includes.h"

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 192000000
  *            HCLK(Hz)                       = 192000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 25
  *            PLL_N                          = 336
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */

int FULL_32 = 0;

static void StartSystemClocks(void);

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /* Enable Power Control clock */

    /* The voltage scaling allows optimizing the power consumption when the device is
       clocked below the maximum system frequency, to update the voltage scaling value
       regarding system frequency refer to product datasheet.  */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* Configure RCC Oscillators: All parameters can be changed according to user’s needs */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM       = FC_PLLM;
	RCC_OscInitStruct.PLL.PLLN       = FC_PLLN;
	RCC_OscInitStruct.PLL.PLLP       = FC_PLLP;
	RCC_OscInitStruct.PLL.PLLQ       = FC_PLLQ;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        while(1);
    }

    /* RCC Clocks: All parameters can be changed according to user’s needs */
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_SYSCLK |RCC_CLOCKTYPE_HCLK |RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    //HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        while(1);
    }

    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(SysTick_IRQn);

    //HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
    //HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
    //HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
    systemUsTicks = (HAL_RCC_GetHCLKFreq()/1000000);    
}

int VectorIrqInit(uint32_t address)
{
	SCB->VTOR = address; //set vector register to firmware start
	__enable_irq(); // enable interrupts

	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    return(0);
}

int BoardInit(void)
{
	HAL_Init();
    SystemClock_Config();
    StartSystemClocks();
    return(0);
}

int USBInit(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /**USB_OTG_FS GPIO Configuration
    PA9     ------> USB_OTG_FS_VBUS
    PA11     ------> USB_OTG_FS_DM
    PA12     ------> USB_OTG_FS_DP
    */
    /*
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    */

    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_USB_OTG_FS_CLK_ENABLE();

    /* Peripheral interrupt init */
    HAL_NVIC_SetPriority(OTG_FS_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
    return(0);
}

int USBDeInit(void)
{
    /* Peripheral clock disable */
    __HAL_RCC_USB_OTG_FS_CLK_DISABLE();

    /**USB_OTG_FS GPIO Configuration
    PA9     ------> USB_OTG_FS_VBUS
    PA11     ------> USB_OTG_FS_DM
    PA12     ------> USB_OTG_FS_DP
    */
    //HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_12);
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(OTG_FS_IRQn);
    return(0);
}

uint32_t TimerPrescalerDivisor(uint32_t timer)
{
	switch (timer) {
		case ENUM_TIM1:
		case ENUM_TIM8:
		case ENUM_TIM9:
		case ENUM_TIM10:
		case ENUM_TIM11:
			return(1);
			break;
		default:
			return(2);
			break;
	}
}


#if defined(STM32F405xx) || defined(stm32f405xx) || defined(STM32F446xx) || defined(stm32f446xx)

static void StartSystemClocks(void)
{
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_ADC2_CLK_ENABLE();
    __HAL_RCC_ADC3_CLK_ENABLE();

    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    __USART1_CLK_ENABLE();
    __USART2_CLK_ENABLE();
    __USART3_CLK_ENABLE();
    __UART4_CLK_ENABLE();
    __UART5_CLK_ENABLE();
    __USART6_CLK_ENABLE();

    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_TIM5_CLK_ENABLE();
    __HAL_RCC_TIM6_CLK_ENABLE();
    __HAL_RCC_TIM7_CLK_ENABLE();
    __HAL_RCC_TIM8_CLK_ENABLE();
    __HAL_RCC_TIM9_CLK_ENABLE();
    __HAL_RCC_TIM10_CLK_ENABLE();
    __HAL_RCC_TIM11_CLK_ENABLE();
    __HAL_RCC_TIM12_CLK_ENABLE();
    __HAL_RCC_TIM13_CLK_ENABLE();
    __HAL_RCC_TIM14_CLK_ENABLE();

}

#endif


#if defined(STM32F411xE) || defined(stm32f411xe)

static void StartSystemClocks(void)
{
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    __HAL_RCC_ADC1_CLK_ENABLE();

    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    __USART1_CLK_ENABLE();
    __USART2_CLK_ENABLE();

    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_TIM5_CLK_ENABLE();
    __HAL_RCC_TIM9_CLK_ENABLE();
    __HAL_RCC_TIM10_CLK_ENABLE();
    __HAL_RCC_TIM11_CLK_ENABLE();
}

#endif