#pragma once


//buffer size, 256 plus two bytes for CRC
#define ESC_BUF_SIZE 256+2

// Bootloader commands
#define RestartBootloader  0
#define ExitBootloader     1

#define BLHELI_END_DATA 112

#define NO_CMD             0xFF
#define CMD_RUN            0x00
#define CMD_PROG_FLASH     0x01
#define CMD_ERASE_FLASH    0x02
#define CMD_READ_FLASH_SIL 0x03
#define CMD_VERIFY_FLASH   0x03
#define CMD_READ_EEPROM    0x04
#define CMD_PROG_EEPROM    0x05
#define CMD_READ_SRAM      0x06
#define CMD_READ_FLASH_ATM 0x07
#define CMD_KEEP_ALIVE     0xFD
#define CMD_SET_ADDRESS    0xFF
#define CMD_SET_BUFFER     0xFE
#define CMD_BOOTINIT       0x07
#define CMD_BOOTSIGN       0x08

// Bootloader result codes
#define RET_SUCCESS        0x30
#define RET_ERRORCOMMAND   0xC1
#define RET_ERRORCRC       0xC2
#define RET_NONE           0xFF

#define BLHELI_EEPROM_HEAD 3

typedef enum {
    BLHBLM_BLHELI_SILABS = 1,
	BLHBLM_BLHELI_ATMEL  = 2,
	BLHBLM_SIMONK_ATMEL  = 3,
} esc_bootloader_mode;

typedef struct {
	uint32_t startAddress;
	uint32_t endAddress;
	uint16_t version;
} esc_hex_location;

extern esc_hex_location escHexByPosition[];
extern uint32_t oneWireActive;

typedef struct BLHeli_EEprom {
    uint8_t BL_GOV_P_GAIN;
    uint8_t BL_GOV_I_GAIN;
    uint8_t BL_GOV_MODE;
    uint8_t BL_MOT_GAIN;
    uint8_t BL_STARTUP_PWR;
    uint8_t BL_PWM_FREQ;
    uint8_t BL_DIRECTION;
    uint8_t BL_INPUT_POL;
    uint8_t BL_INIT_L;
    uint8_t BL_INIT_H;
    uint8_t BL_ENABLE_TX;
    uint8_t BL_COMM_TIMING;
    uint8_t BL_PPM_MIN_THROTLE;
    uint8_t BL_PPM_MAX_THROTLE;
    uint8_t BL_BEEP_STRENGTH;
    uint8_t BL_BEACON_STRENGTH;
    uint8_t BL_BEACON_DELAY;
    uint8_t BL_DEMAG_COMP;
    uint8_t BL_BEC_VOLTAGE_HIGH;
    uint8_t BL_PPM_CENTER;
    uint8_t BL_TEMP_PROTECTION;
    uint8_t BL_ENABLE_POWER_PROT;
    uint8_t BL_ENABLE_PWM_INPUT;
    uint8_t BL_PWM_DITHER;
    uint8_t BL_BRAKE_ON_STOP;
    uint8_t BL_LED_CONTROL;
} BLHeli_EEprom_t;


typedef struct {
    uint32_t (*Disconnect)(motor_type actuator, uint32_t timeout);
//    uint32_t (*PollReadReady)(void);
    uint32_t (*ReadFlash)(motor_type actuator, uint8_t inBuffer[], uint16_t address, uint16_t length, uint32_t timeout);
    uint32_t (*WriteFlash)(motor_type actuator, uint8_t outBuffer[], uint16_t address, uint16_t length, uint32_t timeout);
    uint32_t (*ReadEEprom)(motor_type actuator, uint8_t inBuffer[], uint32_t timeout);
    uint32_t (*WriteEEprom)(motor_type actuator, uint8_t outBuffer[], uint16_t length, uint32_t timeout);
    uint32_t (*PageErase)(motor_type actuator, uint16_t address, uint32_t timeout);
    uint32_t (*EepromErase)(motor_type actuator, uint32_t timeout);
} esc1WireProtocol_t;

typedef struct {
    uint8_t min;
    uint8_t max;
    int16_t step;
    int16_t offset;
} oneWireParameterNumerical_t;

typedef struct {
    uint8_t value;
    const char *name;
} oneWireParameterValue_t;

typedef struct {
    const char* name;
    const oneWireParameterValue_t *parameterNamed;
    const oneWireParameterNumerical_t *parameterNumerical;
    uint32_t offset;
} oneWireParameter_t;

typedef struct {
	uint8_t processed;
    uint8_t txprogramming;
    uint8_t beacondelay;
    uint8_t beaconstrength;
    uint8_t beepstrength;
    uint8_t brakeonstop;
    uint8_t demag;
    uint8_t direction;
    uint8_t frequency;
    uint8_t maxthrottle;
    uint8_t minthrottle;
    uint8_t midthrottle;
    uint8_t startuppower;
    uint8_t tempprotection;
    uint8_t timing;
} oneWireCurrentValues_t;

typedef struct {

	uint32_t                  escSignature;
	uint32_t                  bootVersion;
	uint32_t                  bootPages;
	esc_bootloader_mode       escBootloaderMode;
	const esc1WireProtocol_t *esc1WireProtocol;
	const BLHeli_EEprom_t    *layout;
	uint8_t                   nameStr[16];
	uint8_t                   fwStr[16];
	uint8_t                   versionStr[16];
	uint16_t                  version;
	uint32_t                  enabled;
	uint8_t                   config[128];
	const BLHeli_EEprom_t    *BLHeliEEpromLayout;
	esc_hex_location          escHexLocation;
	oneWireCurrentValues_t    oneWireCurrentValues;

	enum {
		OW_IDLE                   = 0,
		OW_AWAITING_BOOT_MESSAGE  = 1,
		OW_AWAITING_READ_EEPROM   = 2,
		OW_PREPARING_ACTUATOR     = 3,
		OW_ACTUATOR_READY_TO_SEND = 4,
		OW_SENDING_DATA           = 5,
		OW_RECEIVING_DATA         = 6,
		OW_ERROR_UNKNOWN_BOOT_MSG = 7,
		OW_CONNECT_TO_BOOTLOADER  = 8,
	} oneWireState;

} esc_one_wire_status;


extern uint32_t doingAutoA;
extern uint32_t oneWireHasRun;

extern const oneWireParameter_t motorBeaconDelayParameter;
extern const oneWireParameter_t motorBeaconStrengthParameter;
extern const oneWireParameter_t motorBeepStrengthParameter;
extern const oneWireParameter_t motorBrakeOnStopParameter;
extern const oneWireParameter_t motorDemagParameter;
extern const oneWireParameter_t motorDirectionParameter;
extern const oneWireParameter_t motorEnableTxParameter;
extern const oneWireParameter_t motorFrequencyParameter;
extern const oneWireParameter_t motorMaxThrottleParameter;
extern const oneWireParameter_t motorMinThrottleParameter;
extern const oneWireParameter_t motorMidThrottleParameter;
extern const oneWireParameter_t motorStartupPowerParameter;
extern const oneWireParameter_t motorTempProtectionParameter;
extern const oneWireParameter_t motorTimingParameter;


extern const oneWireParameter_t* oneWireParameters[];
extern uint32_t ListAllEscHexesInFlash(void);

extern int16_t     Esc1WireSetParameter(motor_type actuator, const oneWireParameter_t *parameter, uint8_t buf[], int16_t value);
extern int16_t     Esc1WireParameterFromDump(motor_type actuator, const oneWireParameter_t *parameter, uint8_t buf[]);
extern const char* OneWireParameterValueToName(const oneWireParameterValue_t *valuesList, uint8_t value);
extern int16_t     OneWireParameterNameToValue(const oneWireParameterValue_t *valuesList, const char *name);
extern int16_t     OneWireParameterValueToNumber(const oneWireParameterNumerical_t *numerical, uint8_t value);
extern uint8_t     OneWireParameterNumberToValue(const oneWireParameterNumerical_t *numerical, int16_t value);
extern uint32_t    BuiltInUpgradeSiLabsBLHeli(motor_type actuator, esc_hex_location escHexLocation);
extern void        FindEscHexInFlashByName(uint8_t escStringName[], esc_hex_location *escHexLocation, uint32_t escNameStringSize);

//int16_t esc1WireGetParameter(uint8_t escIndex, const oneWireParameter_t *layout);
//uint8_t esc1WireSetParameter(uint8_t escIndex, const oneWireParameter_t *layout, uint8_t value);
//int16_t esc1WireParameterFromDump(uint8_t escIndex, const oneWireParameter_t *parameter, uint8_t *buf);


extern esc_one_wire_status escOneWireStatus[];
extern volatile uint32_t oneWireOngoing;

extern uint32_t OneWireInit(void);
extern void     OneWireDeinit(void);
extern uint32_t OneWireSaveConfig(motor_type actuator);
