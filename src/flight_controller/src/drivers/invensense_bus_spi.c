#include "includes.h"


uint32_t skipGyro     = 1;
volatile uint32_t gyroInterrupting = 0;


static void SPI_Init(uint32_t baudRatePrescaler)
{

	spiHandles[board.spis[board.gyros[0].spiNumber].spiHandle].Instance               = spiInstance[board.gyros[0].spiNumber];
    HAL_SPI_DeInit(&spiHandles[board.spis[board.gyros[0].spiNumber].spiHandle]);

    spiHandles[board.spis[board.gyros[0].spiNumber].spiHandle].Init.Mode              = SPI_MODE_MASTER;
    spiHandles[board.spis[board.gyros[0].spiNumber].spiHandle].Init.Direction         = SPI_DIRECTION_2LINES;
    spiHandles[board.spis[board.gyros[0].spiNumber].spiHandle].Init.DataSize          = SPI_DATASIZE_8BIT;
    spiHandles[board.spis[board.gyros[0].spiNumber].spiHandle].Init.CLKPolarity       = SPI_POLARITY_HIGH;
    spiHandles[board.spis[board.gyros[0].spiNumber].spiHandle].Init.CLKPhase          = SPI_PHASE_2EDGE;
    spiHandles[board.spis[board.gyros[0].spiNumber].spiHandle].Init.NSS               = SPI_NSS_SOFT;
    spiHandles[board.spis[board.gyros[0].spiNumber].spiHandle].Init.BaudRatePrescaler = baudRatePrescaler;
    spiHandles[board.spis[board.gyros[0].spiNumber].spiHandle].Init.FirstBit          = SPI_FIRSTBIT_MSB;
    spiHandles[board.spis[board.gyros[0].spiNumber].spiHandle].Init.TIMode            = SPI_TIMODE_DISABLE;
    spiHandles[board.spis[board.gyros[0].spiNumber].spiHandle].Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    spiHandles[board.spis[board.gyros[0].spiNumber].spiHandle].Init.CRCPolynomial     = 7;

    if (HAL_SPI_Init(&spiHandles[board.spis[board.gyros[0].spiNumber].spiHandle]) != HAL_OK) {
        ErrorHandler(GYRO_SPI_INIT_FAILIURE);
    }

    inlineDigitalHi(ports[board.gyros[0].csPort], board.gyros[0].csPin);
}

void DeInitGyroExti(void)
{
	EXTI_Deinit(ports[board.gyros[0].extiPort], board.gyros[0].extiPin, board.gyros[0].extiIRQn);
}

void InitGyroExti(void)
{
	EXTI_Init(ports[board.gyros[0].extiPort], board.gyros[0].extiPin, board.gyros[0].extiIRQn, 2, 0, GPIO_MODE_IT_RISING, GPIO_PULLDOWN);
}

void AccGyroDeinit(void) {

	//reset the gyro if it's connected and talking to us.
	//will timeout if gyro isn't initialized, but this is okay.
	AccGyroWriteRegister(INVENS_RM_PWR_MGMT_1, INVENS_CONST_H_RESET);
    DelayMs(80);

	//TODO: get rid of these defines
    // ensure the interrupt is not running
	HAL_NVIC_DisableIRQ(board.gyros[0].extiIRQn);

	//set CS high
    inlineDigitalHi(ports[board.gyros[0].csPort], board.gyros[0].csPin);

    //SPI DeInit will disable the GPIOs, DMAs, IRQs and SPIs attached to this SPI handle
    HAL_SPI_DeInit(&spiHandles[board.spis[board.gyros[0].spiNumber].spiHandle]); //TODO: Remove all HAL and place these functions in the stm32.c file so we can support other MCU families.

	//Deinit the EXTI
    DeInitGyroExti();
}

uint32_t AccGyroInit(loopCtrl_e loopCtrl)
{

	if (board.dmasSpi[board.spis[board.gyros[0].spiNumber].RXDma].enabled) {
		memcpy( &board.dmasActive[board.spis[board.gyros[0].spiNumber].RXDma], &board.dmasSpi[board.spis[board.gyros[0].spiNumber].RXDma], sizeof(board_dma) );
	}
	if (board.dmasSpi[board.spis[board.gyros[0].spiNumber].TXDma].enabled) {
		memcpy( &board.dmasActive[board.spis[board.gyros[0].spiNumber].TXDma], &board.dmasSpi[board.spis[board.gyros[0].spiNumber].TXDma], sizeof(board_dma) );
	}

	//callbackFunctionArray[GetExtiCallbackFromPin(board.gyros[0].extiPin)] = GyroExtiCallback;
	//callbackFunctionArray[GetDmaCallbackFromDmaStream(board.spis[board.gyros[0].spiNumber].RXDma)]    = GyroRxDmaCallback;

	AccGyroDeinit();

    // read and write settings at slow speed
	//TODO: All this needs to go into the board record.
	SPI_Init(board.gyros[0].spiSlowBaud);
    DelayMs(5);

    if (!AccGyroDeviceDetect()) {
        return 0;
    }

    DelayMs(5);

    if (!AccGyroDeviceInit(loopCtrl)) {
        return 0;
    }

    // reinitialize at full speed
	SPI_Init(board.gyros[0].spiFastBaud);

    // after the gyro is started, start up the interrupt
	InitGyroExti();

    skipGyro = 0;

    return 1;
}

//uint32_t popcorn = 0;
void GyroExtiCallback(uint32_t callbackNumber)
{
//    popcorn = Micros();
	static int gyroLoopCounter = 0;

	HAL_GPIO_EXTI_IRQHandler(board.gyros[0].extiPin);

	(void)(callbackNumber);

	gyroInterrupting = 1;
    if (!skipGyro)
    {
        //update ACC after the rest of the flight code upon the proper denom
        
    	if (gyroLoopCounter--==0) {
    		gyroLoopCounter = gyroConfig.accDenom;
        	accgyroDeviceReadAccGyro();
        } else {
        	accgyroDeviceReadGyro();
        }
    }
}

void GyroRxDmaCallback(uint32_t callbackNumber)
{
    volatile HAL_DMA_StateTypeDef pop;
	(void)(callbackNumber);
//    volatile static int count1 = 0;
//    volatile static int count2 = 0;
//    volatile static int count3 = 0;

    

//    count1++;
//    if (count1 > 256000)
//    {
//        count1 = 0;
//    }
    pop = HAL_DMA_GetState(&dmaHandles[board.dmasActive[board.spis[board.gyros[0].spiNumber].RXDma].dmaHandle]);
    //while(HAL_DMA_GetState(&dmaHandles[board.dmasActive[board.spis[board.gyros[0].spiNumber].RXDma].dmaHandle]) == HAL_DMA_STATE_BUSY);
    if (pop == HAL_DMA_STATE_READY) {
        // reset chip select line
//        volatile uint32_t popcornTime = Micros() - popcorn;
	    inlineDigitalHi(ports[board.gyros[0].csPort], board.gyros[0].csPin);

//        count2++;
        // run callback for completed gyro read
        accgyroDeviceReadComplete();
    }
    else
    {
//        count3++;
        //inlineDigitalHi(ports[board.gyros[0].csPort], board.gyros[0].csPin);
    }
}

// TODO: get rid of this? only need read/write register and read/write data w/DMA or interrupt
uint32_t AccGyroWriteData(uint8_t *data, uint8_t length)
{
    // poll until SPI is ready in case of ongoing DMA
    while (HAL_SPI_GetState(&spiHandles[board.spis[board.gyros[0].spiNumber].spiHandle]) != HAL_SPI_STATE_READY);

    inlineDigitalLo(ports[board.gyros[0].csPort], board.gyros[0].csPin);
    DelayMs(2);
    HAL_SPI_Transmit(&spiHandles[board.spis[board.gyros[0].spiNumber].spiHandle], data, length, 100);
    inlineDigitalHi(ports[board.gyros[0].csPort], board.gyros[0].csPin);
    DelayMs(2);

    return 1;
}

uint32_t AccGyroWriteRegister(uint8_t reg, uint8_t data)
{
	uint32_t timeout;

    // poll until SPI is ready in case of ongoing DMA
	for (timeout = 0;timeout<10;timeout++) {
		if (HAL_SPI_GetState(&spiHandles[board.spis[board.gyros[0].spiNumber].spiHandle]) == HAL_SPI_STATE_READY)
			break;
		DelayMs(1);
	}
	if (timeout == 10) {
		return 0;
	}

    inlineDigitalLo(ports[board.gyros[0].csPort], board.gyros[0].csPin);
    DelayMs(2);

    // TODO: what should these timeouts be?
    HAL_SPI_Transmit(&spiHandles[board.spis[board.gyros[0].spiNumber].spiHandle], &reg, 1, 25);
    HAL_SPI_Transmit(&spiHandles[board.spis[board.gyros[0].spiNumber].spiHandle], &data, 1, 25);

    inlineDigitalHi(ports[board.gyros[0].csPort], board.gyros[0].csPin);
    DelayMs(2);

    return 1;
}

uint32_t AccGyroVerifyWriteRegister(uint8_t reg, uint8_t data)
{
    uint8_t attempt, data_verify;

    for (attempt = 0; attempt < 20; attempt++)
    {
    	AccGyroWriteRegister(reg, data);
        DelayMs(2);

        AccGyroSlowReadData(reg, &data_verify, 1);
        if (data_verify == data) {
            return 1;
        }
    }

    ErrorHandler(GYRO_SETUP_COMMUNICATION_FAILIURE);

    return 0;  // this is never reached
}

uint32_t AccGyroReadData(uint8_t reg, uint8_t *data, uint8_t length)
{
    reg |= 0x80;

    // poll until SPI is ready in case of ongoing DMA
    while (HAL_SPI_GetState(&spiHandles[board.spis[board.gyros[0].spiNumber].spiHandle]) != HAL_SPI_STATE_READY);

    inlineDigitalLo(ports[board.gyros[0].csPort], board.gyros[0].csPin);

    HAL_SPI_Transmit(&spiHandles[board.spis[board.gyros[0].spiNumber].spiHandle], &reg, 1, 100);
    HAL_SPI_Receive(&spiHandles[board.spis[board.gyros[0].spiNumber].spiHandle], data, length, 100);

    inlineDigitalHi(ports[board.gyros[0].csPort], board.gyros[0].csPin);

    return 1;
}

uint32_t AccGyroSlowReadData(uint8_t reg, uint8_t *data, uint8_t length)
{
    reg |= 0x80;

    // poll until SPI is ready in case of ongoing DMA
    while (HAL_SPI_GetState(&spiHandles[board.spis[board.gyros[0].spiNumber].spiHandle]) != HAL_SPI_STATE_READY);

    inlineDigitalLo(ports[board.gyros[0].csPort], board.gyros[0].csPin);
    DelayMs(1);

    HAL_SPI_Transmit(&spiHandles[board.spis[board.gyros[0].spiNumber].spiHandle], &reg, 1, 100);
    HAL_SPI_Receive(&spiHandles[board.spis[board.gyros[0].spiNumber].spiHandle], data, length, 100);

    inlineDigitalHi(ports[board.gyros[0].csPort], board.gyros[0].csPin);
    DelayMs(1);

    return 1;
}

uint32_t AccGyroDMAReadWriteData(uint8_t *txData, uint8_t *rxData, uint8_t length)
{
    volatile HAL_DMA_StateTypeDef dmaState = HAL_DMA_GetState(&dmaHandles[board.dmasActive[board.spis[board.gyros[0].spiNumber].RXDma].dmaHandle]);
    volatile HAL_SPI_StateTypeDef spiState = HAL_SPI_GetState(&spiHandles[board.spis[board.gyros[0].spiNumber].spiHandle]);
    // ensure that both SPI and DMA resources are available, but don't block if they are not
    if (dmaState == HAL_DMA_STATE_READY && spiState == HAL_SPI_STATE_READY) {
    //if (1 == 1 && 1 == 1) {
    	inlineDigitalLo(ports[board.gyros[0].csPort], board.gyros[0].csPin);

        HAL_SPI_TransmitReceive_DMA(&spiHandles[board.spis[board.gyros[0].spiNumber].spiHandle], txData, rxData, length);

        return 1;
    } else {
        return 0;
    }
}
