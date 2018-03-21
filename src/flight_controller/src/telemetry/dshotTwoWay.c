#include "includes.h"

#define DSHOT_OUT_BUFFER_SIZE 48 //16 * 3 bytes
uint32_t dshotOutBuffer[DSHOT_OUT_BUFFER_SIZE];  
uint32_t dshotInBuffer[DSHOT_OUT_BUFFER_SIZE*8];  


//static void DshotTimerInit(uint32_t timer, uint32_t pwmHz, uint32_t timerHz); //make general function?
//static void DshotDmaInit(void);
//static void DshotTransferComplete(uint32_t callbackNumber);
//static void FillDshotBuffer(uint16_t data, uint32_t gpio);

//DSHOT timer
TIM_HandleTypeDef dshotTimer;

volatile dshot_command_handler dshotCommandHandler;

int InitDshotCommandState(void)
{
    dshotCommandHandler.dshotCommandState   = DSC_MODE_INACTIVE;
    dshotCommandHandler.requestActivation   = 0; //this enabled the state. Set by external things like modes or config commands
    dshotCommandHandler.timeSinceLastAction = 0; //the time an action last occurred
    dshotCommandHandler.commandToSend       = 0; //the command we sent
    dshotCommandHandler.commandReceived     = 0; //the command we received
    dshotCommandHandler.motorCommMask       = 0; //what motors do we communicate with?
    return(0);
}

int HandleDshotCommands(void)
{
    uint8_t serialOutBuffer[2];

    //nothing allowed here if board is armed
    if(boardArmed)
        return(1);

    //request to use the commands is off, and mode is active. Let's disable the mode
    if(
        (!dshotCommandHandler.requestActivation) &&
        (dshotCommandHandler.dshotCommandState != DSC_MODE_INACTIVE)
    )
    {
        //if not using dshot, go back to other protocol
        if ( !IsDshotEnabled() )
        {
            //dshot is not on, go back to ms
            SKIP_GYRO=1;
            DeinitFlight();
            DelayMs(5); //let MCU stabilize 
            InitFlight(mainConfig.mixerConfig.escProtocol, mainConfig.mixerConfig.escUpdateFrequency);
            DelayMs(5); //let MCU stabilize 
            SKIP_GYRO=0;
            DelayMs(5); //let MCU stabilize 
        }
        dshotCommandHandler.dshotCommandState = DSC_MODE_INACTIVE;
    }

    //init handler, no latch needed
    if( 
        (dshotCommandHandler.requestActivation) &&
        (dshotCommandHandler.dshotCommandState == DSC_MODE_INACTIVE)
    )
    {
        //mode has been latched and all conditions for mode have been met
        dshotCommandHandler.dshotCommandState = DSC_MODE_INIT;
    }        

    //send command?
    if(dshotCommandHandler.dshotCommandState == DSC_MODE_SEND)
    {
        SKIP_GYRO=1;
        dshotCommandHandler.dshotCommandState = DSC_MODE_SENDING;

        //allow throttle or not?
        dshotCommandHandler.commandToSend = CONSTRAIN(dshotCommandHandler.commandToSend, 0, DSHOT_CMD_MAX);
        CommandToDshot(serialOutBuffer, dshotCommandHandler.commandToSend);
        for(uint32_t x=0;x<60;x++)
        {
            OutputSerialDmaByte(serialOutBuffer, 2, board.motors[0], 1, 0, 1);
            OutputSerialDmaByte(serialOutBuffer, 2, board.motors[1], 1, 0, 1);
            OutputSerialDmaByte(serialOutBuffer, 2, board.motors[2], 1, 0, 1);
            OutputSerialDmaByte(serialOutBuffer, 2, board.motors[3], 1, 0, 1);
            DelayMs(3); //let MCU stabilize 
        }
        CommandToDshot(serialOutBuffer, dshotCommandHandler.commandToSend);
        for(uint32_t x=0;x<60;x++)
        {
            OutputSerialDmaByte(serialOutBuffer, 2, board.motors[0], 1, 0, 1);
            OutputSerialDmaByte(serialOutBuffer, 2, board.motors[1], 1, 0, 1);
            OutputSerialDmaByte(serialOutBuffer, 2, board.motors[2], 1, 0, 1);
            OutputSerialDmaByte(serialOutBuffer, 2, board.motors[3], 1, 0, 1);
            DelayMs(3); //let MCU stabilize 
        }
        dshotCommandHandler.dshotCommandState = DSC_MODE_ACTIVE;
    }

    //init
    if(dshotCommandHandler.dshotCommandState == DSC_MODE_INIT)
    {
        if ( !IsDshotEnabled() )
        {
            //dshot not used, let's put fc into dshot mode
            //SKIP_GYRO=1;
            DeinitFlight();
            DelayMs(10); //let MCU stabilize 
            InitFlight(ESC_DSHOT300, 4000);
            DelayMs(5); //let MCU stabilize 
            SKIP_GYRO=1;

            CommandToDshot(serialOutBuffer, 0);
            for(uint32_t x=0;x<3500;x++)
            {
                OutputSerialDmaByte(serialOutBuffer, 2, board.motors[0], 1, 0, 1);
                OutputSerialDmaByte(serialOutBuffer, 2, board.motors[1], 1, 0, 1);
                OutputSerialDmaByte(serialOutBuffer, 2, board.motors[2], 1, 0, 1);
                OutputSerialDmaByte(serialOutBuffer, 2, board.motors[3], 1, 0, 1);
                delayUs(500); //let MCU stabilize 
            }
        }

        SKIP_GYRO=1;
        DelayMs(2);
        CommandToDshot(serialOutBuffer, DSHOT_CMD_BEEP2);
        for(uint32_t x=0;x<60;x++)
        {
            OutputSerialDmaByte(serialOutBuffer, 2, board.motors[0], 1, 0, 1);
            OutputSerialDmaByte(serialOutBuffer, 2, board.motors[1], 1, 0, 1);
            OutputSerialDmaByte(serialOutBuffer, 2, board.motors[2], 1, 0, 1);
            OutputSerialDmaByte(serialOutBuffer, 2, board.motors[3], 1, 0, 1);
            DelayMs(3); //let MCU stabilize 
        }
        CommandToDshot(serialOutBuffer, DSHOT_CMD_BEEP4);
        for(uint32_t x=0;x<60;x++)
        {
            OutputSerialDmaByte(serialOutBuffer, 2, board.motors[0], 1, 0, 1);
            OutputSerialDmaByte(serialOutBuffer, 2, board.motors[1], 1, 0, 1);
            OutputSerialDmaByte(serialOutBuffer, 2, board.motors[2], 1, 0, 1);
            OutputSerialDmaByte(serialOutBuffer, 2, board.motors[3], 1, 0, 1);
            DelayMs(3); //let MCU stabilize 
        }
        SKIP_GYRO=1;
        dshotCommandHandler.dshotCommandState = DSC_MODE_ACTIVE;
    }
    return(0);
}

void ThrottleToDshot(uint8_t *serialOutBuffer, float throttle, float idle, int reverse)
{
	uint16_t currThrottle;
    
	(void)(reverse);

    //limit motor spinning
	if( (quopaState == QUOPA_ACTIVE) && mainConfig.mixerConfig.quopaStyle > 2)
        if(throttle < 0.15f)
            throttle = 0.0f;

	if (idle > 0.0f)
	{
        //lrintf is faster than typecasting
		currThrottle = lrintf(InlineChangeRangef(throttle, 1.0f, 0.0f, 2047.0f, (2000.0f * idle * 0.01f)+48.0f));
        CommandToDshot(serialOutBuffer, currThrottle);
	}
	else
	{
        CommandToDshot(serialOutBuffer, 0);
	}
    
}

void CommandToDshot(uint8_t *serialOutBuffer, uint16_t command)
{
	uint16_t digitalThrottle;

	digitalThrottle = ( ( command << 1 ) | 0); //0 is no telem request, 1 is telem request

    // append checksum
    digitalThrottle = ( (digitalThrottle << 4) | CRC4_12(digitalThrottle) );

    serialOutBuffer[0] = (uint8_t)(digitalThrottle >> 8);
    serialOutBuffer[1] = (uint8_t)(digitalThrottle & 0x00ff);
}






///////////////////////////GPIO Manipulation method:

/*
void FillDshotBuffer(uint16_t data, uint32_t gpio)
{
    int bitIndex;

    #define BITS_PER_DSHOT_PACKET 16

    for (bitIndex = (BITS_PER_DSHOT_PACKET - 1); bitIndex >= 0; bitIndex--)
    {
        if (data & (1 << bitIndex))
        {
            dshotOutBuffer[bitIndex*3]   |= gpio;
            dshotOutBuffer[bitIndex*3+1] |= gpio;
            dshotOutBuffer[bitIndex*3+2] &= ~(gpio);
        }
        else
        {
            dshotOutBuffer[bitIndex*3]   |= gpio;
            dshotOutBuffer[bitIndex*3+1] &= ~(gpio);
            dshotOutBuffer[bitIndex*3+2] &= ~(gpio);
        }
    }

}

void DshotInit(int offlineMode)
{

    //todo change from single buffer method maybe
    //int outputNumber;   //set, motor output based on board orrientation, from quad's POV: 0 is top left, 1 is top right, etc...
    int motorNum;       //set, motor output based on board arrat, from board's POV: 0 is top left, 1 is top right, etc...

    //deinit flight and treat motors independantly, make this compatible with old board
    if(offlineMode)
    {
        dShotFeedTheDog = 1; //feed the dog in the scheduler
        //DeinitFlight();      //disable gyro, acc, motor outputs, serial, and soft serial
    }

    //init timer
    DshotTimerInit(ENUM_TIM1, 15, 24000000);
    //DshotTimerInit(ENUM_TIM1, 512, 1024); //slow way down for visual test
    DshotDmaInit();
    if ((HAL_TIM_Base_Start(&dshotTimer)) != HAL_OK)
    {
      while(1);
    }

    bzero(dshotOutBuffer, sizeof(dshotOutBuffer));
    for(motorNum=0;motorNum<MAX_MOTOR_NUMBER;motorNum++)
    {
        
        //outputNumber = mainConfig.mixerConfig.motorOutput[motorNum]; //get output number from quad's POV
        //if(board.motors[outputNumber].enabled == ENUM_ACTUATOR_TYPE_MOTOR)
        //{
        //    InitializeGpio(ports[board.motors[outputNumber].port], board.motors[outputNumber].pin, 0);
        //    //todo this can be changed to a constant lookup table
        //    FillDshotBuffer(0xAAAA, board.motors[outputNumber].pin);

        //    //source is dshot buffer, destination is motor GPIO (LED for first testing)
        //    if (HAL_DMA_Start_IT(&dmaHandles[ENUM_DMA2_STREAM_5], (uint32_t)&dshotOutBuffer, (uint32_t)&ports[board.motors[outputNumber].port]->ODR, DSHOT_OUT_BUFFER_SIZE) != HAL_OK)
        //    {
        //        while(1);
        //    }
        //}

    }
    InitializeGpio(ports[ENUM_PORTA], GPIO_PIN_2, 1); //use B5 to test output on LED
    FillDshotBuffer(0xAAAA, GPIO_PIN_2);
    //if (HAL_DMA_Start_IT(&dmaHandles[ENUM_DMA2_STREAM_5], (uint32_t)&dshotOutBuffer, (uint32_t)&GPIOA->ODR, DSHOT_OUT_BUFFER_SIZE) != HAL_OK)
    //{
    //    while(1);
    //}    
    if (HAL_DMA_Start_IT(&dmaHandles[ENUM_DMA2_STREAM_5], (uint32_t)&dshotOutBuffer, (uint32_t)&GPIOA->ODR, DSHOT_OUT_BUFFER_SIZE) != HAL_OK)
    {
        while(1);
    }
    TIM1->DIER |=  0x00000100;
    //hdma->Instance->CR  &= ~(DMA_IT_TC);
    //TIM1->DIER = TIM_DIER_UDE;
    //set motor outputs as GPIOs
    //motorNum = 0; //test motor 1
    //outputNumber = mainConfig.mixerConfig.motorOutput[motorNum]; //get output number from quad's POV
    //InitializeGpio(ports[board.motors[outputNumber].port], board.motors[outputNumber].pin, 0); //init GPIO and set output to low
    //InitializeGpio(ports[ENUM_PORTB], GPIO_PIN_2, 0); //use B5 to test output on LED


    //init timer
    //DshotTimerInit(TIM1, 15, 24000000);
    //DshotTimerInit(ENUM_TIM1, 15000000, 24000000); //slow way down for visual test
    //DshotDmaInit();

    //start the timer
    //if ((HAL_TIM_Base_Start(&dshotTimer)) != HAL_OK)
    //{
    //  while(1);
    //}

    //TESTING, output to GPIOA for testing.
    //start the transfer:
    DelayMs(6000);
}

static void DshotTransferComplete(uint32_t callbackNumber)
{
    static int fisherton = 0;
    (void)(callbackNumber);
    TIM1->DIER &= ~(0x00000100);
    volatile int transferCompleteDetected = 1;
    //1.55us from completion to this beingtriggered
    //inlineDigitalHi(ports[ENUM_PORTA], GPIO_PIN_2);
    if(fisherton)
    {
        fisherton = 0;
        FillDshotBuffer(0xAAAA, GPIO_PIN_2);
    }
    else
    {
        fisherton = 1;
        FillDshotBuffer(0x5555, GPIO_PIN_2);        
    }
    (void)(transferCompleteDetected);
    if (HAL_DMA_Start_IT(&dmaHandles[ENUM_DMA2_STREAM_5], (uint32_t)&dshotOutBuffer, (uint32_t)&GPIOA->ODR, DSHOT_OUT_BUFFER_SIZE) != HAL_OK)
    {
        while(1);
    }
    //inlineDigitalHi(ports[ENUM_PORTA], GPIO_PIN_2);
    TIM1->DIER |= 0x00000100;
}

static void DshotDmaInit(void) //make general function?
{
    //init DMA
    //for testing we hard code DMA2 Stream 5
    dmaHandles[ENUM_DMA2_STREAM_5].Instance                 = DMA2_Stream5;
    dmaHandles[ENUM_DMA2_STREAM_5].Init.Channel             = DMA_CHANNEL_6;
    dmaHandles[ENUM_DMA2_STREAM_5].Init.Direction           = DMA_MEMORY_TO_PERIPH;
    dmaHandles[ENUM_DMA2_STREAM_5].Init.PeriphInc           = DMA_PINC_DISABLE;
    dmaHandles[ENUM_DMA2_STREAM_5].Init.MemInc              = DMA_MINC_ENABLE;
    dmaHandles[ENUM_DMA2_STREAM_5].Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    dmaHandles[ENUM_DMA2_STREAM_5].Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    dmaHandles[ENUM_DMA2_STREAM_5].Init.Mode                = DMA_NORMAL;
    dmaHandles[ENUM_DMA2_STREAM_5].Init.Priority            = DMA_PRIORITY_HIGH;
    dmaHandles[ENUM_DMA2_STREAM_5].Init.FIFOMode            = DMA_FIFOMODE_DISABLE;

    HAL_DMA_UnRegisterCallback(&dmaHandles[ENUM_DMA2_STREAM_5], HAL_DMA_XFER_ALL_CB_ID);
    
    //only need the first one of these?
    __HAL_LINKDMA(&dshotTimer,hdma[TIM_DMA_ID_UPDATE],dmaHandles[ENUM_DMA2_STREAM_5]);
    __HAL_LINKDMA(&dshotTimer,hdma[TIM_DMA_ID_CC3],dmaHandles[ENUM_DMA2_STREAM_5]);


    if (HAL_DMA_Init(&dmaHandles[ENUM_DMA2_STREAM_5]) != HAL_OK)
    {
        while(1);
    }

    HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 0, 2);
    HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);

    //register the callback function
    callbackFunctionArray[FP_DMA2_S5] = DshotTransferComplete;
}

static void DshotTimerInit(uint32_t timer, uint32_t pwmHz, uint32_t timerHz) //make general function?
{

    TIM_ClockConfigTypeDef  sClockSourceConfig; //set, timer variable
    TIM_MasterConfigTypeDef sMasterConfig;      //set, timer variable

    (void)(pwmHz);
    //run at 24 MHz, need a bitrate of 3 bits per byte:
    //three types of "bytes"
    //one, zero, nothing:
    //one is XXx.
    //zero is Xxx,
    //nothing is xxx,
    //Bit length (total timing period) is 1.67 microseconds
    //For a bit to be 0, the pulse width is 625 nanoseconds
    //For a bit to be 1, the pulse width is 1250 nanoseconds
    //For DSHOT 600:
    //15 and 24000000 MHz will generate a timer event ever 625 ns when the timer is active
    //(15 / 24,000,000) = 0.000,000,625 seconds
    //3 timer events per bit, 000 = nothing, 100 = zero, 110 = one

    dshotTimer.Instance           = timers[timer];
    dshotTimer.Init.Prescaler     = (uint16_t)(SystemCoreClock / TimerPrescalerDivisor(timer) / timerHz) - 1;
    //dshotTimer.Init.CounterMode   = TIM_COUNTERMODE_UP;
    //dshotTimer.Init.Period        = (timerHz / pwmHz) - 1;;
    //dshotTimer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    //dshotTimer.Init.Prescaler     = 1024;
    dshotTimer.Init.CounterMode   = TIM_COUNTERMODE_UP;
    dshotTimer.Init.Period        = 3;
    dshotTimer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    if (HAL_TIM_Base_Init(&dshotTimer) != HAL_OK)
    {
        while(1);
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&dshotTimer, &sClockSourceConfig) != HAL_OK)
    {
        while(1);
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&dshotTimer, &sMasterConfig) != HAL_OK)
    {
        while(1);
    }

}

void BlockingReadGPIO()
{
}
*/