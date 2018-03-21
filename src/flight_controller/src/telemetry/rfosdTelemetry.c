#include "includes.h"

#define RFOSD_PACKET_SIZE 60
#define RFOSD_LINE_SIZE 30

char rfOsdLineBuffer[RFOSD_LINE_SIZE];
char rfOsdTxBuffer[RFOSD_PACKET_SIZE];
static uint32_t currentLine;

static void FillLine(void);

void InitRfOsd(uint32_t usartNumber)
{
    currentLine = 0;
	bzero(rfOsdTxBuffer, sizeof(rfOsdTxBuffer));
	bzero(rfOsdLineBuffer, sizeof(rfOsdLineBuffer));

	//use manual protocol to setup s.port.
	board.serials[usartNumber].enabled   = 1;
	board.serials[usartNumber].Protocol  = USING_RFOSD;

	board.serials[usartNumber].BaudRate   = 115200;
	board.serials[usartNumber].WordLength = UART_WORDLENGTH_8B;
	board.serials[usartNumber].StopBits   = UART_STOPBITS_1;
	board.serials[usartNumber].Parity     = UART_PARITY_NONE;
	board.serials[usartNumber].HwFlowCtl  = UART_HWCONTROL_NONE;
	board.serials[usartNumber].Mode       = UART_MODE_TX;
	//board.serials[usartNumber].Mode       = UART_MODE_TX_RX;
	//can do tx and rx, but for now we send blindly

	board.serials[usartNumber].serialTxInverted = 0;
	board.serials[usartNumber].serialRxInverted = 0;
	board.serials[usartNumber].FrameSize = 10; //variable

	board.dmasSerial[board.serials[usartNumber].TXDma].enabled  = 1;
	board.dmasSerial[board.serials[usartNumber].RXDma].enabled  = 0;

	board.dmasSerial[board.serials[usartNumber].TXDma].dmaDirection       = DMA_MEMORY_TO_PERIPH;
	board.dmasSerial[board.serials[usartNumber].TXDma].dmaPeriphInc       = DMA_PINC_DISABLE;
	board.dmasSerial[board.serials[usartNumber].TXDma].dmaMemInc          = DMA_MINC_ENABLE;
	board.dmasSerial[board.serials[usartNumber].TXDma].dmaPeriphAlignment = DMA_PDATAALIGN_BYTE;
	board.dmasSerial[board.serials[usartNumber].TXDma].dmaMemAlignment    = DMA_MDATAALIGN_BYTE;
	board.dmasSerial[board.serials[usartNumber].TXDma].dmaMode            = DMA_NORMAL;
	board.dmasSerial[board.serials[usartNumber].TXDma].dmaPriority        = DMA_PRIORITY_MEDIUM;
	board.dmasSerial[board.serials[usartNumber].TXDma].fifoMode           = DMA_FIFOMODE_DISABLE;
	memcpy( &board.dmasActive[board.serials[usartNumber].TXDma], &board.dmasSerial[board.serials[usartNumber].TXDma], sizeof(board_dma) ); //TODO: Add dmasUsart

	//board.dmasSerial[board.serials[usartNumber].RXDma].dmaDirection       = DMA_PERIPH_TO_MEMORY;
	//board.dmasSerial[board.serials[usartNumber].RXDma].dmaPeriphInc       = DMA_PINC_DISABLE;
	//board.dmasSerial[board.serials[usartNumber].RXDma].dmaMemInc          = DMA_MINC_DISABLE;
	//board.dmasSerial[board.serials[usartNumber].RXDma].dmaPeriphAlignment = DMA_PDATAALIGN_BYTE;
	//board.dmasSerial[board.serials[usartNumber].RXDma].dmaMemAlignment    = DMA_MDATAALIGN_BYTE;
	//board.dmasSerial[board.serials[usartNumber].RXDma].dmaMode            = DMA_CIRCULAR;
	//board.dmasSerial[board.serials[usartNumber].RXDma].dmaPriority        = DMA_PRIORITY_MEDIUM;
	//board.dmasSerial[board.serials[usartNumber].RXDma].fifoMode           = DMA_FIFOMODE_DISABLE;
	//memcpy( &board.dmasActive[board.serials[usartNumber].RXDma], &board.dmasSerial[board.serials[usartNumber].RXDma], sizeof(board_dma) );

	UsartDeInit(usartNumber); //deinits serial and associated pins and DMAs
	UsartInit(usartNumber); //inits serial and associated pins and DMAs
}

static void FillLine(void)
{
    //0000 0000   0000 0000
    //vertical (y) = A & 31   (0001 1111 0000 0000)
    //horizontal (x) = A & 224 >> 4 (1110 0000 0000 0000)
    int32_t  x;

    for (x = (RFOSD_LINE_SIZE-1);x>(-1);x--)
    {
        //x and y coordinates ( 4 MSBs of X coordinate are stored in the 4 MSBs of the "A" bytes) (All 4 bytes of the Y coordinate are stored in the 4 LSBs of the "A" byte)
        rfOsdTxBuffer[ (x * 2) ] = (char)(( x << 4) | (currentLine & 15)) ;
        //x coordinate LSB and charector (LSB of X coordinate is LSB of the "B" byte)
        rfOsdTxBuffer[ (x * 2) + 1 ] = ( (rfOsdLineBuffer[x] & 127) << 1) | ( (x >> 4) & 1 ) ;
        // 
    }
}

void HandleRfOsd(void)
{
    uint32_t serialNumber;

	if(mainConfig.telemConfig.telemRfOsd == TELEM_INTERNAL_OSD)
		return;

    if (!telemEnabled)
		return;

	if (currentLine == 12)
		currentLine = 0;

	bzero(rfOsdTxBuffer, sizeof(rfOsdTxBuffer));
	bzero(rfOsdLineBuffer, sizeof(rfOsdLineBuffer));

	uint32_t temp;
	uint32_t remainderUsed;

    switch(currentLine++)
    {
        case 0:
            snprintf(rfOsdLineBuffer, RFOSD_LINE_SIZE, " RF1 FT %lu          ", (armedTime / 1000));
            break;
		case 1:
            sprintf(rfOsdLineBuffer, "                              ");
            break;
		case 2:
            sprintf(rfOsdLineBuffer, "                              ");
            break;
		case 3:
            sprintf(rfOsdLineBuffer, "                              ");
            break;
		case 4:
            sprintf(rfOsdLineBuffer, "                              ");
            break;
		case 5:
            sprintf(rfOsdLineBuffer, "                              ");
            break;
		case 6:
            sprintf(rfOsdLineBuffer, "                              ");
            break;
		case 7:
            sprintf(rfOsdLineBuffer, "                              ");
            break;
		case 8:
            sprintf(rfOsdLineBuffer, "                              ");
            break;
		case 9:
            sprintf(rfOsdLineBuffer, "                              ");
            break;
		case 10:
			if (averageVoltage < 0.5)
			{
				temp = 0;
				remainderUsed = 0;
			}
			else
			{
				temp = (uint32_t)(averageVoltage);
				remainderUsed = (uint32_t)((averageVoltage - (float)temp) * 100);
			}
            snprintf(rfOsdLineBuffer, RFOSD_LINE_SIZE, " Volt: %lu.%lu     ", temp, remainderUsed );
            break;
        case 11:
			if (adcCurrent < 0.5)
			{
				temp = 0;
				remainderUsed = 0;
			}
			else
			{
				temp = (uint32_t)(adcCurrent);
				remainderUsed = (uint32_t)((adcCurrent - (float)temp) * 100);
			}
            snprintf(rfOsdLineBuffer, RFOSD_LINE_SIZE, " Curr: %lu.%lu     ", temp, remainderUsed );
            break;
        case 12:
            sprintf(rfOsdLineBuffer, "                              ");
            break;
        case 13:
            sprintf(rfOsdLineBuffer, "                              ");
            break;
        default:
            currentLine = 0;
            break;

    }
    FillLine();

	for (serialNumber = 0;serialNumber<MAX_USARTS;serialNumber++)
	{
		if ( (board.serials[serialNumber].enabled) && (mainConfig.telemConfig.telemRfOsd) )
		{
			if (board.serials[serialNumber].Protocol == USING_RFOSD)
				HAL_UART_Transmit_DMA(&uartHandles[board.serials[serialNumber].usartHandle], (uint8_t *)rfOsdTxBuffer, RFOSD_PACKET_SIZE);
		}
	}

}