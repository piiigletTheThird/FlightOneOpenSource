#include "includes.h"

void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{

    GPIO_InitTypeDef GPIO_InitStruct;
    uint32_t spiInx;

    if (hspi->Instance == SPI1)
	{
		spiInx = ENUM_SPI1;
		__HAL_RCC_SPI1_CLK_ENABLE();
	}
	else if (hspi->Instance == SPI2)
	{
		spiInx = ENUM_SPI2;
		__HAL_RCC_SPI2_CLK_ENABLE();
	}
	else if (hspi->Instance == SPI3)
	{
		spiInx = ENUM_SPI3;
		__HAL_RCC_SPI3_CLK_ENABLE();
	}
	else
	{
		return;
	}


	HAL_GPIO_DeInit(ports[board.spis[spiInx].NSSPort],  board.spis[spiInx].NSSPin);
	HAL_GPIO_DeInit(ports[board.spis[spiInx].SCKPort],  board.spis[spiInx].SCKPin);
	HAL_GPIO_DeInit(ports[board.spis[spiInx].MISOPort], board.spis[spiInx].MISOPin);
	HAL_GPIO_DeInit(ports[board.spis[spiInx].MOSIPort], board.spis[spiInx].MOSIPin);

	GPIO_InitStruct.Pin   = board.spis[spiInx].NSSPin;
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull  = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(ports[board.spis[spiInx].NSSPort], &GPIO_InitStruct);

	GPIO_InitStruct.Pin   = board.spis[spiInx].SCKPin;
	GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull  = board.spis[spiInx].SCKPull;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = board.spis[spiInx].SCKAlternate;
	HAL_GPIO_Init(ports[board.spis[spiInx].SCKPort], &GPIO_InitStruct);

	GPIO_InitStruct.Pin   = board.spis[spiInx].MISOPin;
	GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull  = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = board.spis[spiInx].MISOAlternate;
	HAL_GPIO_Init(ports[board.spis[spiInx].MISOPort], &GPIO_InitStruct);

	GPIO_InitStruct.Pin   = board.spis[spiInx].MOSIPin;
	GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull  = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = board.spis[spiInx].MOSIAlternate;
	HAL_GPIO_Init(ports[board.spis[spiInx].MOSIPort], &GPIO_InitStruct);

	if (board.dmasSpi[board.spis[spiInx].TXDma].enabled)
	{

		dmaHandles[board.dmasActive[board.spis[spiInx].RXDma].dmaHandle].Instance                 = dmaStream[board.dmasActive[board.spis[spiInx].RXDma].dmaStream];
		dmaHandles[board.dmasActive[board.spis[spiInx].RXDma].dmaHandle].Init.Channel             = board.dmasActive[board.spis[spiInx].RXDma].dmaChannel;
		dmaHandles[board.dmasActive[board.spis[spiInx].RXDma].dmaHandle].Init.Direction           = board.dmasActive[board.spis[spiInx].RXDma].dmaDirection;
		dmaHandles[board.dmasActive[board.spis[spiInx].RXDma].dmaHandle].Init.PeriphInc           = board.dmasActive[board.spis[spiInx].RXDma].dmaPeriphInc;
		dmaHandles[board.dmasActive[board.spis[spiInx].RXDma].dmaHandle].Init.MemInc              = board.dmasActive[board.spis[spiInx].RXDma].dmaMemInc;
		dmaHandles[board.dmasActive[board.spis[spiInx].RXDma].dmaHandle].Init.PeriphDataAlignment = board.dmasActive[board.spis[spiInx].RXDma].dmaPeriphAlignment;
		dmaHandles[board.dmasActive[board.spis[spiInx].RXDma].dmaHandle].Init.MemDataAlignment    = board.dmasActive[board.spis[spiInx].RXDma].dmaMemAlignment;
		dmaHandles[board.dmasActive[board.spis[spiInx].RXDma].dmaHandle].Init.Mode                = board.dmasActive[board.spis[spiInx].RXDma].dmaMode;
		dmaHandles[board.dmasActive[board.spis[spiInx].RXDma].dmaHandle].Init.Priority            = board.dmasActive[board.spis[spiInx].RXDma].dmaPriority;
		dmaHandles[board.dmasActive[board.spis[spiInx].RXDma].dmaHandle].Init.FIFOMode            = board.dmasActive[board.spis[spiInx].RXDma].fifoMode;
		
		HAL_DMA_UnRegisterCallback(&dmaHandles[board.dmasActive[board.spis[spiInx].RXDma].dmaHandle], HAL_DMA_XFER_ALL_CB_ID);
		
		if (HAL_DMA_Init(&dmaHandles[board.dmasActive[board.spis[spiInx].RXDma].dmaHandle]) != HAL_OK)
		{
			ErrorHandler(MSP_DMA_SPI_RX_INIT_FAILIURE);
		}

		__HAL_LINKDMA(hspi, hdmarx, dmaHandles[board.dmasActive[board.spis[spiInx].RXDma].dmaHandle]);


		dmaHandles[board.dmasActive[board.spis[spiInx].TXDma].dmaHandle].Instance                 = dmaStream[board.dmasActive[board.spis[spiInx].TXDma].dmaStream];
		dmaHandles[board.dmasActive[board.spis[spiInx].TXDma].dmaHandle].Init.Channel             = board.dmasActive[board.spis[spiInx].TXDma].dmaChannel;
		dmaHandles[board.dmasActive[board.spis[spiInx].TXDma].dmaHandle].Init.Direction           = board.dmasActive[board.spis[spiInx].TXDma].dmaDirection;
		dmaHandles[board.dmasActive[board.spis[spiInx].TXDma].dmaHandle].Init.PeriphInc           = board.dmasActive[board.spis[spiInx].TXDma].dmaPeriphInc;
		dmaHandles[board.dmasActive[board.spis[spiInx].TXDma].dmaHandle].Init.MemInc              = board.dmasActive[board.spis[spiInx].TXDma].dmaMemInc;
		dmaHandles[board.dmasActive[board.spis[spiInx].TXDma].dmaHandle].Init.PeriphDataAlignment = board.dmasActive[board.spis[spiInx].TXDma].dmaPeriphAlignment;
		dmaHandles[board.dmasActive[board.spis[spiInx].TXDma].dmaHandle].Init.MemDataAlignment    = board.dmasActive[board.spis[spiInx].TXDma].dmaMemAlignment;
		dmaHandles[board.dmasActive[board.spis[spiInx].TXDma].dmaHandle].Init.Mode                = board.dmasActive[board.spis[spiInx].TXDma].dmaMode;
		dmaHandles[board.dmasActive[board.spis[spiInx].TXDma].dmaHandle].Init.Priority            = board.dmasActive[board.spis[spiInx].TXDma].dmaPriority;
		dmaHandles[board.dmasActive[board.spis[spiInx].TXDma].dmaHandle].Init.FIFOMode            = board.dmasActive[board.spis[spiInx].TXDma].fifoMode;
		
		HAL_DMA_UnRegisterCallback(&dmaHandles[board.dmasActive[board.spis[spiInx].TXDma].dmaHandle], HAL_DMA_XFER_ALL_CB_ID);
		
		if (HAL_DMA_Init(&dmaHandles[board.dmasActive[board.spis[spiInx].TXDma].dmaHandle]) != HAL_OK)
		{
			ErrorHandler(MSP_DMA_SPI_TX_INIT_FAILIURE);
		}

		__HAL_LINKDMA(hspi, hdmatx, dmaHandles[board.dmasActive[board.spis[spiInx].TXDma].dmaHandle]);

		HAL_NVIC_SetPriority(board.dmasActive[board.spis[spiInx].TXDma].dmaIRQn, board.dmasActive[board.spis[spiInx].TXDma].priority, 0);
		HAL_NVIC_EnableIRQ(board.dmasActive[board.spis[spiInx].TXDma].dmaIRQn);

		HAL_NVIC_SetPriority(board.dmasActive[board.spis[spiInx].RXDma].dmaIRQn, board.dmasActive[board.spis[spiInx].RXDma].priority, 0);
		HAL_NVIC_EnableIRQ(board.dmasActive[board.spis[spiInx].RXDma].dmaIRQn);

		board.dmasActive[board.spis[spiInx].TXDma].enabled = 1;
		board.dmasActive[board.spis[spiInx].RXDma].enabled = 1;
	}

	// Peripheral interrupt init
	HAL_NVIC_SetPriority(board.spis[spiInx].SPI_IRQn, board.spis[spiInx].priority, 0);
	HAL_NVIC_EnableIRQ(board.spis[spiInx].SPI_IRQn);

}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi)
{

	uint32_t spiInx;

	if (hspi->Instance == SPI1)
	{
		spiInx = ENUM_SPI1;
		__HAL_RCC_SPI1_CLK_DISABLE();
	}
	else if (hspi->Instance == SPI2)
	{
		spiInx = ENUM_SPI2;
		__HAL_RCC_SPI2_CLK_DISABLE();
	}
	else if (hspi->Instance == SPI3)
	{
		spiInx = ENUM_SPI3;
		__HAL_RCC_SPI3_CLK_DISABLE();
	}
	else
	{
		return;
	}


    HAL_GPIO_DeInit(ports[board.spis[spiInx].NSSPort],  board.spis[spiInx].NSSPin);
    HAL_GPIO_DeInit(ports[board.spis[spiInx].SCKPort],  board.spis[spiInx].SCKPin);
    HAL_GPIO_DeInit(ports[board.spis[spiInx].MISOPort], board.spis[spiInx].MISOPin);
    HAL_GPIO_DeInit(ports[board.spis[spiInx].MOSIPort], board.spis[spiInx].MOSIPin);

	if (board.dmasSpi[board.spis[spiInx].TXDma].enabled)
	{
		/* Peripheral DMA DeInit*/
		HAL_DMA_DeInit(hspi->hdmarx);
		HAL_DMA_DeInit(hspi->hdmatx);
		HAL_NVIC_DisableIRQ(board.dmasActive[board.spis[spiInx].TXDma].dmaIRQn);
		HAL_NVIC_DisableIRQ(board.dmasActive[board.spis[spiInx].RXDma].dmaIRQn);

		board.dmasActive[board.spis[spiInx].TXDma].enabled = 0;
		board.dmasActive[board.spis[spiInx].RXDma].enabled = 0;
	}

    /* Peripheral interrupt DeInit*/
    HAL_NVIC_DisableIRQ(board.spis[spiInx].SPI_IRQn);


}
