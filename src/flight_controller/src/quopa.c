#include "includes.h"

#define QUAD_HAS_FLOWN_MS_LIMIT 1000
#define QUOPA_PATTERN_HOLD_MS 250

volatile quopa_state quopaState;
volatile quopa_state dshotBeepState;


int InitDshotBeep(void)
{
    dshotBeepState = QUOPA_INACTIVE;
    return(0);
}

int InitQuopaMode(void)
{
    quopaState = QUOPA_INACTIVE;
    return(0);
}

int HandleQuopaMode(void)
{
    static int quopaModeLatch = 0;
    uint8_t serialOutBuffer[3];

    if(boardArmed)
        return(0);

    //latch handling for init
    if( 
        !quopaModeLatch &&
        (armedTimeSincePower > QUAD_HAS_FLOWN_MS_LIMIT) &&
        !ModeActive(M_QUOPA) &&
        quopaState == QUOPA_INACTIVE
    )
    {
        //switch is inactive and is allowed to be active
        quopaModeLatch = 1;
    }

    //mode is active but requested not to be. turn it off
    if(!ModeActive(M_QUOPA) && quopaState == QUOPA_ACTIVE)
    {
        SKIP_GYRO=1;
        DelayMs(2); //let MCU stabilize 
        CommandToDshot(serialOutBuffer, DSHOT_CMD_SPIN_DIRECTION_NORMAL);
        for(uint32_t x=0;x<70;x++)
        {
            OutputSerialDmaByte(serialOutBuffer, 2, board.motors[0], 1, 0, 1);
            OutputSerialDmaByte(serialOutBuffer, 2, board.motors[1], 1, 0, 1);
            OutputSerialDmaByte(serialOutBuffer, 2, board.motors[2], 1, 0, 1);
            OutputSerialDmaByte(serialOutBuffer, 2, board.motors[3], 1, 0, 1);
            DelayMs(3); //let MCU stabilize 
        }
        CommandToDshot(serialOutBuffer, DSHOT_CMD_BEEP1);
        for(uint32_t x=0;x<70;x++)
        {
            OutputSerialDmaByte(serialOutBuffer, 2, board.motors[0], 1, 0, 1);
            OutputSerialDmaByte(serialOutBuffer, 2, board.motors[1], 1, 0, 1);
            OutputSerialDmaByte(serialOutBuffer, 2, board.motors[2], 1, 0, 1);
            OutputSerialDmaByte(serialOutBuffer, 2, board.motors[3], 1, 0, 1);
            DelayMs(3); //let MCU stabilize 
        }
        SKIP_GYRO=0;
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
        quopaState = QUOPA_INACTIVE;
    }

    //init handling, uses latch
    if( 
        quopaModeLatch &&
        (armedTimeSincePower > QUAD_HAS_FLOWN_MS_LIMIT) &&
        ModeActive(M_QUOPA) &&
        quopaState == QUOPA_INACTIVE
    )
    {
        //quopa mode has been latched and all conditions for quopa mode have been met
        quopaState = QUOPA_INIT;
        quopaModeLatch = 0;
    }

    //init
    if(quopaState == QUOPA_INIT)
    {
        if ( !IsDshotEnabled() )
        {
            //dshot not used, let's put fc into dshot mode
            //SKIP_GYRO=1;
            DeinitFlight();
            DelayMs(10); //let MCU stabilize 
            InitFlight(ESC_DSHOT600, 16000);
            DelayMs(5); //let MCU stabilize 
            SKIP_GYRO=1;

            CommandToDshot(serialOutBuffer, 0);
            for(uint32_t x=0;x<4500;x++)
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
        CommandToDshot(serialOutBuffer, DSHOT_CMD_SPIN_DIRECTION_REVERSED);
        for(uint32_t x=0;x<70;x++)
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

        SKIP_GYRO=0;
        quopaState = QUOPA_ACTIVE;

    }
    return(0);
}

int HandleDshotBeep(void)
{
    static int dshotBeepLatch = 0;
    static uint32_t lastDshotBeep = 0;
    uint8_t serialOutBuffer[3];

    if(boardArmed)
        return(0);

    //latch handling for init
    if( 
        !dshotBeepLatch &&
        !ModeActive(M_BEEP) &&
        dshotBeepState == QUOPA_INACTIVE
    )
    {
        //switch is inactive and is allowed to be active
        dshotBeepLatch = 1;
    }

    //mode is active but requested not to be. turn it off
    if(!ModeActive(M_BEEP) && dshotBeepState == QUOPA_ACTIVE)
    {
        SKIP_GYRO=1;
        DelayMs(2); //let MCU stabilize 
        CommandToDshot(serialOutBuffer, DSHOT_CMD_SPIN_DIRECTION_NORMAL);
        for(uint32_t x=0;x<60;x++)
        {
            OutputSerialDmaByte(serialOutBuffer, 2, board.motors[0], 1, 0, 1);
            OutputSerialDmaByte(serialOutBuffer, 2, board.motors[1], 1, 0, 1);
            OutputSerialDmaByte(serialOutBuffer, 2, board.motors[2], 1, 0, 1);
            OutputSerialDmaByte(serialOutBuffer, 2, board.motors[3], 1, 0, 1);
            DelayMs(3); //let MCU stabilize 
        }
        CommandToDshot(serialOutBuffer, DSHOT_CMD_BEEP1);
        for(uint32_t x=0;x<60;x++)
        {
            OutputSerialDmaByte(serialOutBuffer, 2, board.motors[0], 1, 0, 1);
            OutputSerialDmaByte(serialOutBuffer, 2, board.motors[1], 1, 0, 1);
            OutputSerialDmaByte(serialOutBuffer, 2, board.motors[2], 1, 0, 1);
            OutputSerialDmaByte(serialOutBuffer, 2, board.motors[3], 1, 0, 1);
            DelayMs(3); //let MCU stabilize 
        }

        SKIP_GYRO=0;

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
        dshotBeepState = QUOPA_INACTIVE;
    }

    if( 
        dshotBeepLatch &&
        ModeActive(M_BEEP) &&
        dshotBeepState == QUOPA_INACTIVE
    )
    {
        //quopa mode has been latched and all conditions for quopa mode have been met
        dshotBeepState = QUOPA_INIT;
        dshotBeepLatch = 0;
    }        

    //send a beep
    if( (InlineMillis() - lastDshotBeep > 4000) && (dshotBeepState == QUOPA_ACTIVE) && ModeActive(M_BEEP))
    {
        lastDshotBeep = InlineMillis();
        SKIP_GYRO=1;
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
    }

    //mode init
    if(dshotBeepState == QUOPA_INIT)
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
            for(uint32_t x=0;x<4500;x++)
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
        CommandToDshot(serialOutBuffer, DSHOT_CMD_BEEP1);
        for(uint32_t x=0;x<60;x++)
        {
            OutputSerialDmaByte(serialOutBuffer, 2, board.motors[0], 1, 0, 1);
            OutputSerialDmaByte(serialOutBuffer, 2, board.motors[1], 1, 0, 1);
            OutputSerialDmaByte(serialOutBuffer, 2, board.motors[2], 1, 0, 1);
            OutputSerialDmaByte(serialOutBuffer, 2, board.motors[3], 1, 0, 1);
            DelayMs(3); //let MCU stabilize 
        }
        SKIP_GYRO=1;
        dshotBeepState = QUOPA_ACTIVE;
    }
    return(0);
}