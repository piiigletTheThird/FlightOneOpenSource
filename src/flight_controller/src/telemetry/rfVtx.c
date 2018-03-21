#include "includes.h"

volatile uint8_t rfVtxRxBuffer[4];
volatile uint8_t vtxOutPacket[4];

void InitRfVtx(uint32_t usartNumber)
{
	uint32_t x;

	for (x=0;x<4;x++)
		rfVtxRxBuffer[x]=0;

	//use manual protocol to setup s.port.
	board.serials[usartNumber].enabled   = 1;
	board.serials[usartNumber].Protocol  = USING_RFVTX;

	board.serials[usartNumber].BaudRate   = 9600;
	board.serials[usartNumber].WordLength = UART_WORDLENGTH_8B;
	board.serials[usartNumber].StopBits   = UART_STOPBITS_1;
	board.serials[usartNumber].Parity     = UART_PARITY_NONE;
	board.serials[usartNumber].HwFlowCtl  = UART_HWCONTROL_NONE;
	board.serials[usartNumber].Mode       = UART_MODE_TX_RX;

	board.serials[usartNumber].serialTxInverted = 0;
	board.serials[usartNumber].serialRxInverted = 0;
	board.serials[usartNumber].FrameSize = 4;

	board.dmasSerial[board.serials[usartNumber].TXDma].enabled  = 1;
	board.dmasSerial[board.serials[usartNumber].RXDma].enabled  = 1;

	board.dmasSerial[board.serials[usartNumber].TXDma].dmaDirection       = DMA_MEMORY_TO_PERIPH;
	board.dmasSerial[board.serials[usartNumber].TXDma].dmaPeriphInc       = DMA_PINC_DISABLE;
	board.dmasSerial[board.serials[usartNumber].TXDma].dmaMemInc          = DMA_MINC_ENABLE;
	board.dmasSerial[board.serials[usartNumber].TXDma].dmaPeriphAlignment = DMA_PDATAALIGN_BYTE;
	board.dmasSerial[board.serials[usartNumber].TXDma].dmaMemAlignment    = DMA_MDATAALIGN_BYTE;
	board.dmasSerial[board.serials[usartNumber].TXDma].dmaMode            = DMA_NORMAL;
	board.dmasSerial[board.serials[usartNumber].TXDma].dmaPriority        = DMA_PRIORITY_MEDIUM;
	board.dmasSerial[board.serials[usartNumber].TXDma].fifoMode           = DMA_FIFOMODE_DISABLE;
	memcpy( &board.dmasActive[board.serials[usartNumber].TXDma], &board.dmasSerial[board.serials[usartNumber].TXDma], sizeof(board_dma) ); //TODO: Add dmasUsart

	board.dmasSerial[board.serials[usartNumber].RXDma].dmaDirection       = DMA_PERIPH_TO_MEMORY;
	board.dmasSerial[board.serials[usartNumber].RXDma].dmaPeriphInc       = DMA_PINC_DISABLE;
	board.dmasSerial[board.serials[usartNumber].RXDma].dmaMemInc          = DMA_MINC_DISABLE;
	board.dmasSerial[board.serials[usartNumber].RXDma].dmaPeriphAlignment = DMA_PDATAALIGN_BYTE;
	board.dmasSerial[board.serials[usartNumber].RXDma].dmaMemAlignment    = DMA_MDATAALIGN_BYTE;
	board.dmasSerial[board.serials[usartNumber].RXDma].dmaMode            = DMA_CIRCULAR;
	board.dmasSerial[board.serials[usartNumber].RXDma].dmaPriority        = DMA_PRIORITY_MEDIUM;
	board.dmasSerial[board.serials[usartNumber].RXDma].fifoMode           = DMA_FIFOMODE_DISABLE;
	memcpy( &board.dmasActive[board.serials[usartNumber].RXDma], &board.dmasSerial[board.serials[usartNumber].RXDma], sizeof(board_dma) );

	UsartDeInit(usartNumber); //deinits serial and associated pins and DMAs
	UsartInit(usartNumber); //inits serial and associated pins and DMAs
}

uint32_t RfVtxOff(void)
{
	vtxOutPacket[0] = 0x55;
	vtxOutPacket[1] = 0xAA;
	vtxOutPacket[2] = 0x03;
	vtxOutPacket[3] = 0x00;
	for (uint32_t serialNumber = 0;serialNumber<MAX_USARTS;serialNumber++)
	{
		if ( board.serials[serialNumber].enabled )
		{
			if (board.serials[serialNumber].Protocol == USING_RFVTX)
			{
				HAL_UART_Transmit_DMA(&uartHandles[board.serials[serialNumber].usartHandle], (uint8_t *)vtxOutPacket, 4);
				return(1);
			}
		}
	}
	return(0);
}

uint32_t RfVtxBaud(void)
{
	vtxOutPacket[0] = 0x55;
	vtxOutPacket[1] = 0xAA;
	vtxOutPacket[2] = 0x04;
	vtxOutPacket[3] = 0x01;
	for (uint32_t serialNumber = 0;serialNumber<MAX_USARTS;serialNumber++)
	{
		if ( board.serials[serialNumber].enabled )
		{
			if (board.serials[serialNumber].Protocol == USING_RFVTX)
			{
				HAL_UART_Transmit_DMA(&uartHandles[board.serials[serialNumber].usartHandle], (uint8_t *)vtxOutPacket, 4);
				return(1);
			}
		}
	}
	return(0);
}


uint32_t RfVtxOn25(void)
{
	vtxOutPacket[0] = 0x55;
	vtxOutPacket[1] = 0xAA;
	vtxOutPacket[2] = 0x03;
	vtxOutPacket[3] = 0x01;
	for (uint32_t serialNumber = 0;serialNumber<MAX_USARTS;serialNumber++)
	{
		if ( board.serials[serialNumber].enabled )
		{
			if (board.serials[serialNumber].Protocol == USING_RFVTX)
			{
				HAL_UART_Transmit_DMA(&uartHandles[board.serials[serialNumber].usartHandle], (uint8_t *)vtxOutPacket, 4);
				return(1);
			}
		}
	}
	return(0);
}

uint32_t RfVtxOn200(void)
{
	vtxOutPacket[0] = 0x55;
	vtxOutPacket[1] = 0xAA;
	vtxOutPacket[2] = 0x03;
	vtxOutPacket[3] = 0x02;
	for (uint32_t serialNumber = 0;serialNumber<MAX_USARTS;serialNumber++)
	{
		if ( board.serials[serialNumber].enabled )
		{
			if (board.serials[serialNumber].Protocol == USING_RFVTX)
			{
				HAL_UART_Transmit_DMA(&uartHandles[board.serials[serialNumber].usartHandle], (uint8_t *)vtxOutPacket, 4);
				return(1);
			}
		}
	}
	return(0);
}


uint32_t RfVtxBand(uint32_t band)
{
	vtxOutPacket[0] = 0x55;
	vtxOutPacket[1] = 0xAA;
	vtxOutPacket[2] = 0x01;
	vtxOutPacket[3] = (uint8_t)band;
	for (uint32_t serialNumber = 0;serialNumber<MAX_USARTS;serialNumber++)
	{
		if ( board.serials[serialNumber].enabled )
		{
			if (board.serials[serialNumber].Protocol == USING_RFVTX)
			{
				HAL_UART_Transmit_DMA(&uartHandles[board.serials[serialNumber].usartHandle], (uint8_t *)vtxOutPacket, 4);
				return(1);
			}
		}
	}
	return(0);
}

uint32_t RfVtxChannel(uint32_t channel)
{
	vtxOutPacket[0] = 0x55;
	vtxOutPacket[1] = 0xAA;
	vtxOutPacket[2] = 0x02;
	vtxOutPacket[3] = (uint8_t)channel;
	for (uint32_t serialNumber = 0;serialNumber<MAX_USARTS;serialNumber++)
	{
		if ( board.serials[serialNumber].enabled )
		{
			if (board.serials[serialNumber].Protocol == USING_RFVTX)
			{
				HAL_UART_Transmit_DMA(&uartHandles[board.serials[serialNumber].usartHandle], (uint8_t *)vtxOutPacket, 4);
				return(1);
			}
		}
	}
	return(0);
}
