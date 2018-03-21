#include "includes.h"

volatile uint8_t mspRxBuffer[25];
volatile uint8_t mspTxBuffer[25];


void SendMspAttitude(void)
{
	uint8_t crc = 0;
	uint32_t packetSize = 12;
	//<preamble>,<direction>,<size>,<command>,,<crc>
	mspTxBuffer[0] =  '$';
	mspTxBuffer[1] =  'M';
	mspTxBuffer[2] =  '>';
	mspTxBuffer[3] =  6;
	mspTxBuffer[4] =  MSP_ATTITUDE;
	mspTxBuffer[5] =  (uint8_t)((int16_t)(rollAttitude * 10.0f) & 0xFF);
	mspTxBuffer[6] =  (uint8_t)(((int16_t)(rollAttitude * 10.0f) >> 8) & 0xFF);;
	mspTxBuffer[7] =  (uint8_t)((int16_t)(pitchAttitude * 10.0f) & 0xFF);
	mspTxBuffer[8] =  (uint8_t)(((int16_t)(pitchAttitude * 10.0f) >> 8) & 0xFF);;
	mspTxBuffer[9] =  (uint8_t)((int16_t)(yawAttitude * 10.0f) & 0xFF);
	mspTxBuffer[10] =  (uint8_t)(((int16_t)(yawAttitude * 10.0f) >> 8) & 0xFF);;

	crc ^= mspTxBuffer[3];
	crc ^= mspTxBuffer[4];
	crc ^= mspTxBuffer[5];
	crc ^= mspTxBuffer[6];
	crc ^= mspTxBuffer[7];
	crc ^= mspTxBuffer[8];
	crc ^= mspTxBuffer[9];
	crc ^= mspTxBuffer[10];

	mspTxBuffer[11] = crc;

	if (!telemEnabled)
			return;

	for (uint32_t serialNumber = 0;serialNumber<MAX_USARTS;serialNumber++)
	{
		if ( (board.serials[serialNumber].enabled) && (mainConfig.telemConfig.telemMsp) )
		{
			if (board.serials[serialNumber].Protocol == USING_MSP)
				HAL_UART_Transmit_DMA(&uartHandles[board.serials[serialNumber].usartHandle], (uint8_t *)mspTxBuffer, packetSize);
		}
	}

/*
	//this isn't calculated in glue mode'
		rollAttitude  =  InlineRadiansToDegrees( Atan2fast(attitudeFrameQuat.y * attitudeFrameQuat.z + attitudeFrameQuat.w * attitudeFrameQuat.x, 0.5f - (attitudeFrameQuat.x * attitudeFrameQuat.x + attitudeFrameQuat.y * attitudeFrameQuat.y)) );
		pitchAttitude =  InlineRadiansToDegrees( arm_sin_f32(2.0f * (attitudeFrameQuat.x * attitudeFrameQuat.z - attitudeFrameQuat.w * attitudeFrameQuat.y)) );
		yawAttitude   = -InlineRadiansToDegrees( Atan2fast(attitudeFrameQuat.x * attitudeFrameQuat.y + attitudeFrameQuat.w * attitudeFrameQuat.z, 0.5f - (attitudeFrameQuat.y * attitudeFrameQuat.y + attitudeFrameQuat.z * attitudeFrameQuat.z)) );
*/

/*
	angx	INT 16	Range [-1800;1800] (unit: 1/10 degree)
	angy	INT 16	Range [-900;900] (unit: 1/10 degree)
	heading	INT 16	Range [-180;180]
*/
}

void SendMspAnalog(void)
{
	uint8_t crc = 0;
	uint32_t packetSize = 13;
	//<preamble>,<direction>,<size>,<command>,,<crc>
	mspTxBuffer[0] =  '$';
	mspTxBuffer[1] =  'M';
	mspTxBuffer[2] =  '>';
	mspTxBuffer[3] =  7;
	mspTxBuffer[4] =  MSP_ANALOG;
	mspTxBuffer[5] =  (uint8_t)(averageVoltage * 10.0f); //	unit: 1/10 volt
	mspTxBuffer[6] =  (uint8_t)((uint16_t)(adcMAh) & 0xFF); //mAh drawn
	mspTxBuffer[7] =  (uint8_t)(((uint16_t)(adcMAh) >> 8) & 0xFF); //mAh drawn
	mspTxBuffer[8] =  (uint8_t)((uint16_t)(0) & 0xFF);         //RSSI range: [0;1023]
	mspTxBuffer[9] =  (uint8_t)(((uint16_t)(0) >> 8) & 0xFF); //RSSI range: [0;1023]
	mspTxBuffer[10] = (uint8_t)((uint16_t)(adcCurrent * 10.0f) & 0xFF); //current in 1/10 amp ?
	mspTxBuffer[11] = (uint8_t)(((uint16_t)(adcCurrent * 10.0f) >> 8) & 0xFF); //current in 1/10 amp ?

	crc ^= mspTxBuffer[3];
	crc ^= mspTxBuffer[4];
	crc ^= mspTxBuffer[5];
	crc ^= mspTxBuffer[6];
	crc ^= mspTxBuffer[7];
	crc ^= mspTxBuffer[8];
	crc ^= mspTxBuffer[9];
	crc ^= mspTxBuffer[10];
	crc ^= mspTxBuffer[11];

	mspTxBuffer[12] = crc;

	if (!telemEnabled)
			return;

	for (uint32_t serialNumber = 0;serialNumber<MAX_USARTS;serialNumber++)
	{
		if ( (board.serials[serialNumber].enabled) && (mainConfig.telemConfig.telemMsp) )
		{
			if (board.serials[serialNumber].Protocol == USING_MSP)
				HAL_UART_Transmit_DMA(&uartHandles[board.serials[serialNumber].usartHandle], (uint8_t *)mspTxBuffer, packetSize);
		}
	}
}

void SendMspStatus(void)
{
	uint32_t mspFlag = 0;
	uint8_t crc = 0;
	uint32_t packetSize = 16;

/*
// Mode bits
struct {
  uint8_t armed;
  uint8_t stable;
  uint8_t horizon;
  uint8_t baro;
  uint8_t mag;
  uint16_t camstab;
  uint16_t gpshome;
  uint16_t gpshold;
  uint16_t passthru;
  uint32_t air;
  uint32_t acroplus;
  uint32_t osd_switch;
  uint32_t llights;
  uint32_t gpsmission;
  uint32_t gpsland;
}mode;

	M_ARMED      = (1 << 0),
	M_ATTITUDE   = (1 << 1),
	M_HORIZON    = (1 << 2),
	M_FAILSAFE   = (1 << 3),
	M_LOGGING    = (1 << 4),
	M_BUZZER     = (1 << 5),
	M_LEDMODE    = (1 << 6),
	M_LEDCOLOR   = (1 << 7),
	M_DIRECT     = (1 << 8),
	M_VTXON      = (1 << 9),
	M_GLUE       = (1 << 10),
	M_BRAINDRAIN = (1 << 11)

*/


	mspFlag |= (boardArmed);
	mspFlag |= ModeActive(M_ATTITUDE) << 1;
	mspFlag |= ModeActive(M_HORIZON) << 2;

	//<preamble>,<direction>,<size>,<command>,,<crc>
	mspTxBuffer[0]  =  '$';
	mspTxBuffer[1]  =  'M';
	mspTxBuffer[2]  =  '>';
	mspTxBuffer[3]  =  11;
	mspTxBuffer[4]  =  MSP_STATUS;
	mspTxBuffer[5]  =  0;
	mspTxBuffer[6]  =  31; //cycle time is a 16 bit variable... HAH!
	mspTxBuffer[7]  =  0;
	mspTxBuffer[8]  =  0;  //I2C errors, none ever because we don't use sucky I2C
	mspTxBuffer[9]  =  0xff;
	mspTxBuffer[10] =  0xff;  //ACC<<0|BARO<<1|MAG<<2|GPS<<3|SONAR<<4|
	mspTxBuffer[11] =  (uint8_t)((uint32_t)(mspFlag) & 0xFF);
	mspTxBuffer[12] =  (uint8_t)(((uint32_t)(mspFlag) >> 8) & 0xFF);
	mspTxBuffer[13] =  (uint8_t)(((uint32_t)(mspFlag) >> 16) & 0xFF);
	mspTxBuffer[14] =  (uint8_t)(((uint32_t)(mspFlag) >> 24) & 0xFF);
	mspTxBuffer[15] =  0;

	crc ^= mspTxBuffer[3];
	crc ^= mspTxBuffer[4];
	crc ^= mspTxBuffer[5];
	crc ^= mspTxBuffer[6];
	crc ^= mspTxBuffer[7];
	crc ^= mspTxBuffer[8];
	crc ^= mspTxBuffer[9];
	crc ^= mspTxBuffer[10];
	crc ^= mspTxBuffer[11];
	crc ^= mspTxBuffer[12];
	crc ^= mspTxBuffer[13];
	crc ^= mspTxBuffer[14];
	crc ^= mspTxBuffer[15];

	mspTxBuffer[16] = crc;

	if (!telemEnabled)
			return;

	for (uint32_t serialNumber = 0;serialNumber<MAX_USARTS;serialNumber++)
	{
		if ( (board.serials[serialNumber].enabled) && (mainConfig.telemConfig.telemMsp) )
		{
			if (board.serials[serialNumber].Protocol == USING_MSP)
				HAL_UART_Transmit_DMA(&uartHandles[board.serials[serialNumber].usartHandle], (uint8_t *)mspTxBuffer, packetSize);
		}
	}
}

void SendMspBoxIds(void)
{
	uint8_t  crc = 0;
	uint32_t packetSize = 9;

/*
// Mode bits
each BOX (used or not) have a unique ID.
In order to retrieve the number of BOX and which BOX are in used, this request can be used. It is more efficient than retrieving BOX names if you know what BOX function is behing the ID. See enum MultiWii.cpp (0: ARM, 1 ANGLE, 2 HORIZON, …)
*/

	//<preamble>,<direction>,<size>,<command>,,<crc>
	mspTxBuffer[0]  =  '$';
	mspTxBuffer[1]  =  'M';
	mspTxBuffer[2]  =  '>';
	mspTxBuffer[3]  =  3;
	mspTxBuffer[4]  =  MSP_BOXIDS;
	mspTxBuffer[5]  =  0;
	mspTxBuffer[6]  =  1;
	mspTxBuffer[7]  =  2;


	crc ^= mspTxBuffer[3];
	crc ^= mspTxBuffer[4];
	crc ^= mspTxBuffer[5];
	crc ^= mspTxBuffer[6];
	crc ^= mspTxBuffer[7];

	mspTxBuffer[8] = crc;

	if (!telemEnabled)
			return;

	for (uint32_t serialNumber = 0;serialNumber<MAX_USARTS;serialNumber++)
	{
		if ( (board.serials[serialNumber].enabled) && (mainConfig.telemConfig.telemMsp) )
		{
			if (board.serials[serialNumber].Protocol == USING_MSP)
				HAL_UART_Transmit_DMA(&uartHandles[board.serials[serialNumber].usartHandle], (uint8_t *)mspTxBuffer, packetSize);
		}
	}
}

void InitMsp(uint32_t usartNumber)
{
	uint32_t x;

	for (x=0;x<4;x++)
		rfVtxRxBuffer[x]=0;

	//use manual protocol to setup s.port.
	board.serials[usartNumber].enabled   = 1;
	board.serials[usartNumber].Protocol  = USING_MSP;

	board.serials[usartNumber].BaudRate   = 115200;
	board.serials[usartNumber].WordLength = UART_WORDLENGTH_8B;
	board.serials[usartNumber].StopBits   = UART_STOPBITS_1;
	board.serials[usartNumber].Parity     = UART_PARITY_NONE;
	board.serials[usartNumber].HwFlowCtl  = UART_HWCONTROL_NONE;
	//board.serials[usartNumber].Mode       = UART_MODE_TX_RX;
	//can do tx and rx, but for now we send blindly
	board.serials[usartNumber].Mode       = UART_MODE_TX;

	board.serials[usartNumber].serialTxInverted = 0;
	board.serials[usartNumber].serialRxInverted = 0;
	board.serials[usartNumber].FrameSize = 10; //variable

	board.dmasSerial[board.serials[usartNumber].TXDma].enabled  = 1;
	//board.dmasSerial[board.serials[usartNumber].RXDma].enabled  = 1;
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

void SendMsp(uint8_t charArray, uint32_t size)
{

	(void)(charArray);
	(void)(size);
	//uint8_t sPortPacket[4];

	//bzero(rfVtxRxBuffer, sizeof(rfVtxRxBuffer));
}
