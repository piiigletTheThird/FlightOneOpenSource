#include "includes.h"

#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
	ErrorHandler(HARD_FAULT);
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
	ErrorHandler(MEM_FAULT);
}

/**
* @brief This function handles Pre-fetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
	ErrorHandler(BUS_FAULT);
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
	ErrorHandler(USAGE_FAULT);
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();
    InlineUpdateMillisClock();
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles USB On The Go FS global interrupt.
*/
void OTG_FS_IRQHandler(void)
{
    HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
}



//void TIM1_CC_IRQHandler(void)
//{
//	//HAL_TIM_IRQHandler(&TimHandle);
//}


//void TIM2_IRQHandler(void)
//{
//	//HAL_TIM_IRQHandler(&TimHandle);
//}
//
//void TIM3_IRQHandler(void)
//{
//	HAL_TIM_IRQHandler(&pwmTimers[board.motors[0].actuatorArrayNum]);
//}

//void TIM8_UP_TIM13_IRQHandler(void)
//{
//	HAL_TIM_IRQHandler(&softSerialClockTimer);
//	if (inlineIsPinStatusHi(ports[board.motors[0].port], board.motors[0].pin))
//		inlineDigitalHi(ports[board.motors[0].port], board.motors[0].pin);
//	else
//		inlineDigitalLo(ports[board.motors[0].port], board.motors[0].pin);
//}


//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	(void)(htim);
//	//if(htim->Instance == TIM8)
//	//{
//		//HAL_TIM_IRQHandler(&softSerialClockTimer);
//		//if (inlineIsPinStatusHi(ports[board.motors[0].port], board.motors[0].pin))
//		//	inlineDigitalHi(ports[board.motors[0].port], board.motors[0].pin);
//		//else
//		//	inlineDigitalLo(ports[board.motors[0].port], board.motors[0].pin);
//		//HAL_TIM_Base_Stop_IT(htim);
//		//HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,GPIO_PIN_RESET);
//    //}
//
//}


void TIM1_UP_TIM10_IRQHandler(void)
{
	if (callbackFunctionArray[FP_TIM1])
		callbackFunctionArray[FP_TIM1](TIM_UP);
}

void TIM1_CC_IRQHandler(void)
{
	if (callbackFunctionArray[FP_TIM1])
		callbackFunctionArray[FP_TIM1](TIM_CC);
}

void TIM5_IRQHandler(void)
{
	if (callbackFunctionArray[FP_TIM5])
		callbackFunctionArray[FP_TIM5](FP_TIM5);

}

void TIM6_DAC_IRQHandler(void)
{
	if (callbackFunctionArray[FP_TIM6])
		callbackFunctionArray[FP_TIM6](FP_TIM6);

}

void TIM7_IRQHandler(void)
{
	if (callbackFunctionArray[FP_TIM7])
		callbackFunctionArray[FP_TIM7](FP_TIM7);
}

void TIM1_BRK_TIM9_IRQHandler(void)
{
	if (callbackFunctionArray[FP_TIM9])
		callbackFunctionArray[FP_TIM9](FP_TIM9);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if(htim->Instance == _TIM1)
	{
		if (callbackFunctionArray[FP_TIM1])
			callbackFunctionArray[FP_TIM1](FP_TIM1);
	}
	else if(htim->Instance == _TIM2)
	{
		if (callbackFunctionArray[FP_TIM2])
			callbackFunctionArray[FP_TIM2](FP_TIM2);
	}
	else if(htim->Instance == _TIM3)
	{
		if (callbackFunctionArray[FP_TIM3])
			callbackFunctionArray[FP_TIM3](FP_TIM3);
	}
	else if(htim->Instance == _TIM4)
	{
		if (callbackFunctionArray[FP_TIM4])
			callbackFunctionArray[FP_TIM4](FP_TIM4);
	}
	else if(htim->Instance == _TIM5)
	{
		if (callbackFunctionArray[FP_TIM5])
			callbackFunctionArray[FP_TIM5](FP_TIM5);
	}
	else if(htim->Instance == _TIM6)
	{
		if (callbackFunctionArray[FP_TIM6])
			callbackFunctionArray[FP_TIM6](FP_TIM6);
	}
	else if(htim->Instance == _TIM7)
	{
		if (callbackFunctionArray[FP_TIM7])
			callbackFunctionArray[FP_TIM7](FP_TIM7);
	}
	else if(htim->Instance == _TIM8)
	{
		if (callbackFunctionArray[FP_TIM8])
			callbackFunctionArray[FP_TIM8](FP_TIM8);
	}
	else if(htim->Instance == _TIM9)
	{
		if (callbackFunctionArray[FP_TIM9])
			callbackFunctionArray[FP_TIM9](FP_TIM9);
	}
	else if(htim->Instance == _TIM10)
	{
		if (callbackFunctionArray[FP_TIM10])
			callbackFunctionArray[FP_TIM10](FP_TIM10);
	}
	else if(htim->Instance == _TIM11)
	{
		if (callbackFunctionArray[FP_TIM11])
			callbackFunctionArray[FP_TIM11](FP_TIM11);
	}
	else if(htim->Instance == _TIM12)
	{
		if (callbackFunctionArray[FP_TIM12])
			callbackFunctionArray[FP_TIM12](FP_TIM12);
	}
	else if(htim->Instance == _TIM13)
	{
		if (callbackFunctionArray[FP_TIM13])
			callbackFunctionArray[FP_TIM13](FP_TIM13);
	}
	else if(htim->Instance == _TIM14)
	{
		if (callbackFunctionArray[FP_TIM14])
			callbackFunctionArray[FP_TIM14](FP_TIM14);
	}

}



void DMA1_Stream0_IRQHandler(void) {
	HAL_NVIC_ClearPendingIRQ(DMA1_Stream0_IRQn);
	HAL_DMA_IRQHandler(&dmaHandles[ENUM_DMA1_STREAM_0]);
	if (callbackFunctionArray[FP_DMA1_S0])
		callbackFunctionArray[FP_DMA1_S0](FP_DMA1_S0);
}

void DMA1_Stream1_IRQHandler(void) {
	HAL_NVIC_ClearPendingIRQ(DMA1_Stream1_IRQn);
	HAL_DMA_IRQHandler(&dmaHandles[ENUM_DMA1_STREAM_1]);
	if (callbackFunctionArray[FP_DMA1_S1])
		callbackFunctionArray[FP_DMA1_S1](FP_DMA1_S1);
}

void DMA1_Stream2_IRQHandler(void) {
	HAL_NVIC_ClearPendingIRQ(DMA1_Stream2_IRQn);
	HAL_DMA_IRQHandler(&dmaHandles[ENUM_DMA1_STREAM_2]);
	if (callbackFunctionArray[FP_DMA1_S2])
		callbackFunctionArray[FP_DMA1_S2](FP_DMA1_S2);
}

void DMA1_Stream3_IRQHandler(void) {
	HAL_NVIC_ClearPendingIRQ(DMA1_Stream3_IRQn);
	HAL_DMA_IRQHandler(&dmaHandles[ENUM_DMA1_STREAM_3]);
	if (callbackFunctionArray[FP_DMA1_S3])
		callbackFunctionArray[FP_DMA1_S3](FP_DMA1_S3);
}

void DMA1_Stream4_IRQHandler(void) {
	HAL_NVIC_ClearPendingIRQ(DMA1_Stream4_IRQn);
	HAL_DMA_IRQHandler(&dmaHandles[ENUM_DMA1_STREAM_4]);
	if (callbackFunctionArray[FP_DMA1_S4])
		callbackFunctionArray[FP_DMA1_S4](FP_DMA1_S4);
}

void DMA1_Stream5_IRQHandler(void) {
	HAL_NVIC_ClearPendingIRQ(DMA1_Stream5_IRQn);
	HAL_DMA_IRQHandler(&dmaHandles[ENUM_DMA1_STREAM_5]);
	if (callbackFunctionArray[FP_DMA1_S5])
		callbackFunctionArray[FP_DMA1_S5](FP_DMA1_S5);
}

void DMA1_Stream6_IRQHandler(void) {
	HAL_NVIC_ClearPendingIRQ(DMA1_Stream6_IRQn);
	HAL_DMA_IRQHandler(&dmaHandles[ENUM_DMA1_STREAM_6]);
	if (callbackFunctionArray[FP_DMA1_S6])
		callbackFunctionArray[FP_DMA1_S6](FP_DMA1_S6);
}

void DMA1_Stream7_IRQHandler(void) {
	HAL_NVIC_ClearPendingIRQ(DMA1_Stream7_IRQn);
	HAL_DMA_IRQHandler(&dmaHandles[ENUM_DMA1_STREAM_7]);
	if (callbackFunctionArray[FP_DMA1_S7])
		callbackFunctionArray[FP_DMA1_S7](FP_DMA1_S7);
}

void DMA2_Stream0_IRQHandler(void) {
    HAL_NVIC_ClearPendingIRQ(DMA2_Stream0_IRQn);
    HAL_DMA_IRQHandler(&dmaHandles[ENUM_DMA2_STREAM_0]);
    if (callbackFunctionArray[FP_DMA2_S0])
   		callbackFunctionArray[FP_DMA2_S0](FP_DMA2_S0);
}

void DMA2_Stream1_IRQHandler(void) {
    HAL_NVIC_ClearPendingIRQ(DMA2_Stream1_IRQn);
    HAL_DMA_IRQHandler(&dmaHandles[ENUM_DMA2_STREAM_1]);
    if (callbackFunctionArray[FP_DMA2_S1])
		callbackFunctionArray[FP_DMA2_S1](FP_DMA2_S1);
}

void DMA2_Stream2_IRQHandler(void) {
    HAL_NVIC_ClearPendingIRQ(DMA2_Stream2_IRQn);
    HAL_DMA_IRQHandler(&dmaHandles[ENUM_DMA2_STREAM_2]);
    if (callbackFunctionArray[FP_DMA2_S2])
		callbackFunctionArray[FP_DMA2_S2](FP_DMA2_S2);
}

void DMA2_Stream3_IRQHandler(void)
{
    HAL_NVIC_ClearPendingIRQ(DMA2_Stream3_IRQn);
    HAL_DMA_IRQHandler(&dmaHandles[ENUM_DMA2_STREAM_3]);
    if (callbackFunctionArray[FP_DMA2_S3])
		callbackFunctionArray[FP_DMA2_S3](FP_DMA2_S3);
}

void DMA2_Stream4_IRQHandler(void)
{
    HAL_NVIC_ClearPendingIRQ(DMA2_Stream4_IRQn);
    HAL_DMA_IRQHandler(&dmaHandles[ENUM_DMA2_STREAM_4]);
    if (callbackFunctionArray[FP_DMA2_S4])
		callbackFunctionArray[FP_DMA2_S4](FP_DMA2_S4);
}

void DMA2_Stream5_IRQHandler(void)
{
    HAL_NVIC_ClearPendingIRQ(DMA2_Stream5_IRQn);
    HAL_DMA_IRQHandler(&dmaHandles[ENUM_DMA2_STREAM_5]);
    if (callbackFunctionArray[FP_DMA2_S5])
		callbackFunctionArray[FP_DMA2_S5](FP_DMA2_S5);
}

void DMA2_Stream6_IRQHandler(void) {
    HAL_NVIC_ClearPendingIRQ(DMA2_Stream6_IRQn);
    HAL_DMA_IRQHandler(&dmaHandles[ENUM_DMA2_STREAM_6]);
    if (callbackFunctionArray[FP_DMA2_S6])
		callbackFunctionArray[FP_DMA2_S6](FP_DMA2_S6);
}

void DMA2_Stream7_IRQHandler(void) {
    HAL_NVIC_ClearPendingIRQ(DMA2_Stream7_IRQn);
    HAL_DMA_IRQHandler(&dmaHandles[ENUM_DMA2_STREAM_7]);
    if (callbackFunctionArray[FP_DMA2_S7])
		callbackFunctionArray[FP_DMA2_S7](FP_DMA2_S7);
}





void EXTI0_IRQHandler(void)
{
	if (callbackFunctionArray[FP_EXTI0])
		callbackFunctionArray[FP_EXTI0](FP_EXTI0);
}

void EXTI1_IRQHandler(void)
{
	if (callbackFunctionArray[FP_EXTI1])
		callbackFunctionArray[FP_EXTI1](FP_EXTI1);
}

void EXTI2_IRQHandler(void)
{
	if (callbackFunctionArray[FP_EXTI2])
		callbackFunctionArray[FP_EXTI2](FP_EXTI2);
}

void EXTI3_IRQHandler(void)
{
	if (callbackFunctionArray[FP_EXTI3])
		callbackFunctionArray[FP_EXTI3](FP_EXTI3);
}

void EXTI4_IRQHandler(void)
{
	if (callbackFunctionArray[FP_EXTI4])
		callbackFunctionArray[FP_EXTI4](FP_EXTI4);
}

void EXTI9_5_IRQHandler(void)
{
	if (callbackFunctionArray[FP_EXTI9_5])
		callbackFunctionArray[FP_EXTI9_5](FP_EXTI9_5);
}

void EXTI15_10_IRQHandler(void)
{
	if (callbackFunctionArray[FP_EXTI15_10])
		callbackFunctionArray[FP_EXTI15_10](FP_EXTI15_10);
}



void ADC_IRQHandler(void)
{
	//only need to use one ADC handle for now
	HAL_ADC_IRQHandler(&adcHandle[board.boardADC[1].adcHandle]);
}


void SPI1_IRQHandler(void)
{
  	HAL_SPI_IRQHandler(&spiHandles[ENUM_SPI1]);
}

void SPI2_IRQHandler(void)
{
 	HAL_SPI_IRQHandler(&spiHandles[ENUM_SPI2]);
}

void SPI3_IRQHandler(void)
{
  	HAL_SPI_IRQHandler(&spiHandles[ENUM_SPI3]);
}