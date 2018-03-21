#pragma once

#define FIRMWARE_VERSION_INT	(uint16_t)(380U)
#define CONFIG_VERSION			(uint8_t)(140U)
#define CONFIG_VERSION_STR		"140"
#define FIRMWARE_VERSION		"0.380.140 BETA" //RC19 is next
#define FIRMWARE_NAME			"RaceFlight One"
#define FULL_VERSION_STRING		"#vr NAME:" FIRMWARE_NAME ";VERSION:" FIRMWARE_VERSION ";CONFIG:" CONFIG_VERSION_STR "\n\0"

#define RF_BUFFER_SIZE HID_EPIN_SIZE-1
#define FLIGHT_MODE_ARRAY_SIZE 96

#define PROFILE_COUNT 3

enum
{
	PROFILE1 = 0,
	PROFILE2 = 1,
	PROFILE3 = 2,
	PROFILE4 = 3,
	PROFILE5 = 4,
	PROFILE6 = 5,
	PROFILE7 = 6,
};

typedef struct
{
	char			  profileName[16];
	rc_rate           rcRates;
	pid_terms         pidConfig[AXIS_NUMBER];
	filter_device     filterConfig[AXIS_NUMBER];
} tune_profile;

typedef struct
{
	uint32_t		  marker1;
	uint32_t		  marker2;
	char			  craftName[16];
	uint32_t		  marker3;
	tune_profile      tuneProfile[PROFILE_COUNT];
	rc_control_config rcControlsConfig;
	gyro_config       gyroConfig;
	mixer_config      mixerConfig;
	led_config		  ledConfig;
	telem_config	  telemConfig;
	int16_t           flightModeArray[96];
	uint8_t           version;
	uint16_t          size;
	uint8_t           czechsum;
} main_config;

enum
{
	typeINT    = 0,
	typeUINT   = 1,
	typeFLOAT  = 2,
	typeSTRING = 3,
	typeFLOATu = 4,
};

typedef struct
{
    const char *name;
    const uint32_t type;
    const char *group;
    void  *ptr;
    float Min;
    float Max;
    float Default;
    const char *strDefault;
} config_variables_rec;

typedef struct
{
    const char *valueString;
    const int32_t valueInt;
} string_comp_rec;

extern char rf_custom_out_buffer[];
extern char rfCustomSendBuffer[];

extern char *StripSpaces(char *inString);
extern char *CleanupString(char *inString);

extern volatile uint32_t          configSize;
extern volatile int               activeProfile;
extern uint32_t                   resetBoard;
extern main_config                mainConfig;
extern const config_variables_rec valueTable[];
extern volatile uint32_t rfCustomReplyBufferPointer;
extern volatile uint32_t rfCustomReplyBufferPointerSent;


extern volatile int headerToWrite;
extern volatile int headerWritten;
extern volatile int julian;

extern char   *CleanupNumberString(char *inString);
extern void    SaveConfig (uint32_t addresConfigStart);
extern int     LoadConfig (uint32_t addresConfigStart);
extern void    GenerateConfig(void);
extern void    ValidateConfigSettings(void);
extern void    ProcessCommand(char *inString);
extern int     RfCustomReply(char *rf_custom_out_buffer);
extern void    SendStatusReport(char *inString);
extern void    SaveAndSend(void);

extern int     RfCustomReplyBuffer(char *rfCustomSendBufferAdder);
extern int     SendRfCustomReplyBuffer(void);
extern void    OutputVarSet(uint32_t position, int doDiff);