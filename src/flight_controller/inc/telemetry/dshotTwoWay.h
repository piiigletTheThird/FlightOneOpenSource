#pragma once

enum
{
    DSHOT_CMD_MOTOR_STOP              = 0,
    DSHOT_CMD_BEEP1                   = 1,	// Wait length of beep plus 100ms before next command
    DSHOT_CMD_BEEP2                   = 2,	// Wait length of beep plus 100ms before next command
    DSHOT_CMD_BEEP3                   = 3,	// Wait length of beep plus 100ms before next command
    DSHOT_CMD_BEEP4                   = 4,	// Wait length of beep plus 100ms before next command
    DSHOT_CMD_BEEP5                   = 5,	// Wait length of beep plus 100ms before next command
    DSHOT_CMD_ESC_INFO                = 6,	// Currently not implemented
    DSHOT_CMD_SPIN_DIRECTION_1        = 7,	// Need 9x
    DSHOT_CMD_SPIN_DIRECTION_2        = 8,	// Need 9x
    DSHOT_CMD_3D_MODE_OFF             = 9,	// Need 9x
    DSHOT_CMD_3D_MODE_ON              = 10, // Need 9x
    DSHOT_CMD_SETTINGS_REQUEST        = 11, // Currently not implemented
    DSHOT_CMD_SAVE_SETTINGS           = 12, // Need 9x
    DSHOT_CMD_SPIN_DIRECTION_NORMAL   = 20,	// Need 9x
    DSHOT_CMD_SPIN_DIRECTION_REVERSED = 21,	// Need 9x
    DSHOT_CMD_LED0_ON                 = 22,	// Currently not implemented
    DSHOT_CMD_LED1_ON                 = 23,	// Currently not implemented
    DSHOT_CMD_LED2_ON                 = 24,	// Currently not implemented
    DSHOT_CMD_LED3_ON                 = 25,	// Currently not implemented
    DSHOT_CMD_LED0_OFF                = 26,	// Currently not implemented
    DSHOT_CMD_LED1_OFF                = 27,	// Currently not implemented
    DSHOT_CMD_LED2_OFF                = 28,	// Currently not implemented
    DSHOT_CMD_LED3_OFF                = 29,	// Currently not implemented
    DSHOT_CMD_MAX                     = 47,
};

typedef enum dshot_command_state_t
{
    DSC_MODE_INACTIVE  = 0, //not in use
    DSC_MODE_ACTIVE    = 1, //ready to do something
    DSC_MODE_INIT      = 2, //in init phase
    DSC_MODE_SEND      = 3, //send a command
    DSC_MODE_SENDING   = 4, //sending a command
    DSC_MODE_SENT      = 5, //command was send
    DSC_MODE_RECEIVE   = 6, //receive and wait
    DSC_MODE_RECEIVING = 7, //currently receiving
    DSC_MODE_RECEIVED  = 8, //command received
} dshot_command_state;

typedef struct
{
	dshot_command_state dshotCommandState;
	uint32_t            requestActivation;
	uint32_t            timeSinceLastAction;
	uint16_t            commandToSend;
    uint16_t            commandReceived;
    uint32_t            motorCommMask;
} dshot_command_handler;

extern volatile dshot_command_handler dshotCommandHandler;

extern void DshotInit(int offlineMode);
extern int  InitDshotCommandState(void);
extern int  HandleDshotCommands(void);
extern void ThrottleToDshot(uint8_t *serialOutBuffer, float throttle, float idle, int reverse);
extern void CommandToDshot(uint8_t *serialOutBuffer, uint16_t command);
