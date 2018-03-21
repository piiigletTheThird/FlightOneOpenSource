#pragma once

#define MAXCHANNELS 16

#define MINRC 1000 //1000 to 2000 is old standard for RC channels. midRc needs to be adjustable.
#define MAXRC 2000 //1000 to 2000 is old standard for RC channels. midRc needs to be adjustable.

//config structure which is loaded by config
typedef struct {
    float    deadBand[MAXCHANNELS];
    uint32_t midRc[MAXCHANNELS];
    uint32_t minRc[MAXCHANNELS];
    uint32_t maxRc[MAXCHANNELS];
    uint32_t channelMap[MAXCHANNELS];
    uint32_t rcCalibrated;
    uint32_t rxProtcol;
    uint32_t rxUsart;
    uint32_t rxInvertPin;
    uint32_t rxInvertPort;
    uint32_t rxInvertDirection;
    uint32_t bind;
	uint32_t shortThrow;
    uint32_t armMethod;
} rc_control_config;

typedef struct
{
	int   useCurve;
	float rcSmoothingFactor;
	float rates[3];
	float acroPlus[3];
	float curveExpo[3];
} rc_rate;

typedef struct
{
	uint32_t dataValue;
	uint32_t timesOccurred;
} rx_calibraation_record;

typedef struct
{
	uint32_t boardArmed;
	uint32_t latchFirstArm;
	uint32_t armModeSet;
	uint32_t armModeActive;
	uint32_t rcCalibrated;
	uint32_t boardCalibrated;
	uint32_t progMode;
	uint32_t throttleIsSafe;
	uint32_t rxTimeout;
	uint32_t failsafeHappend;
	uint32_t activeFailsafe;
} arming_structure;

extern volatile float throttleVelocity;
extern volatile uint32_t throttleIsSafe;
extern volatile arming_structure armingStructure;

#define RX_CHECK_AMOUNT 24

typedef struct
{
	rx_calibraation_record rxCalibrationRecord[RX_CHECK_AMOUNT];
	uint32_t highestDataValue;
} rx_calibration_records;

enum {
	ARM_DOUBLE_SINGLE = 0,
	ARM_DOUBLE_DOUBLE = 1,
};

//Enumerate the different channels in code. The TX map is not affected by this. This is for internal code only.
enum {
	YAW      = 0,
	ROLL     = 1,
	PITCH    = 2,
	THROTTLE = 3,
	AUX1     = 4,
	AUX2     = 5,
	AUX3     = 6,
	AUX4     = 7,
	AUX5     = 8,
	AUX6     = 9,
	AUX7     = 10,
	AUX8     = 11,
	AUX9     = 12,
	AUX10    = 13,
	AUX11    = 14,
	AUX12    = 15,
};

enum {
	ACCX = 0,
	ACCY = 1,
	ACCZ = 2,
};

//We can use different styles of curves
//used in config.c string table
enum
{
	NO_EXPO        = 0,
	SKITZO_EXPO    = 1,
	TARANIS_EXPO   = 2,
	FAST_EXPO      = 3,
	ACRO_PLUS      = 4,
	KISS_EXPO      = 5,
	KISS_EXPO2     = 6,
	BETAFLOP_EXPO  = 7,
	EXPO_CURVE_END = 8,
};

//used in config.c string table
#define USING_MANUAL           0
#define USING_SPEK_R           1
#define USING_SPEK_T           2
#define USING_SBUS_R           3
#define USING_SBUS_T           4
#define USING_SUMD_R           5
#define USING_SUMD_T           6
#define USING_IBUS_R           7
#define USING_IBUS_T           8
#define USING_DSM2_R           9
#define USING_DSM2_T           10
#define USING_CPPM_R           11
#define USING_CPPM_T           12
#define USING_SPORT            13
#define USING_MSP              14
#define USING_RFVTX            15
#define USING_SMARTAUDIO       16
#define USING_RFOSD            17
#define USING_TRAMP            18
#define USING_CRSF_R           19
#define USING_CRSF_T           20
#define USING_CRSF_B           21
#define USING_CRSF_TELEM       22
#define USING_RX_END           23

extern volatile float smoothCurvedThrottle0_1;
extern volatile float trueCurvedThrottle0_1;
extern uint32_t ppmPin;
extern volatile float maxFlopRate[];
extern volatile float maxKissRate[];
extern volatile uint32_t armBoardAt;

extern volatile uint32_t progMode;
extern volatile SPM_VTX_DATA vtxData;
extern volatile uint32_t rx_timeout;
extern float trueRcCommandF[MAXCHANNELS];     //4 sticks. range is -1 to 1, directly related to stick position
extern float curvedRcCommandF[MAXCHANNELS];   //4 sticks. range is -1 to 1, this is the rcCommand after the curve is applied
extern float smoothedRcCommandF[MAXCHANNELS]; //4 sticks. range is -1 to 1, this is the smoothed rcCommand
extern uint32_t rxDataRaw[MAXCHANNELS];
extern uint32_t rxData[MAXCHANNELS];
extern volatile unsigned char isRxDataNew;
extern uint32_t skipRxMap;
extern uint32_t PreArmFilterCheck;
extern uint32_t activeFailsafe;
extern uint32_t failsafeHappend;


extern void CheckThrottleSafe(void);
extern void SpektrumBind (uint32_t bindNumber);

extern void InitRcData(void);
extern void InlineCollectRcCommand(void);
extern float InlineApplyRcCommandCurve(float rcCommand, uint32_t curveToUse, float expo, uint32_t axis);
extern void InlineRcSmoothing(float curvedRcCommandF[], float smoothedRcCommandF[]);

extern void ProcessArmingStructure(void);
extern void ProcessSpektrumPacket(uint32_t serialNumber);
extern void PowerInveter(uint32_t port, uint32_t pin, uint32_t direction);
extern void ProcessSbusPacket(uint32_t serialNumber);
extern void ProcessSumdPacket(uint8_t serialRxBuffer[], uint32_t frameSize);
extern void ProcessIbusPacket(uint8_t serialRxBuffer[], uint32_t frameSize);
extern void ProcessCrsfPacket(uint8_t serialRxBuffer[], uint32_t frameSize);

extern void RxUpdate(void);
extern void CheckFailsafe(void);
extern uint32_t SpektrumChannelMap(uint32_t inChannel);
extern uint32_t ChannelMap(uint32_t inChannel);

extern void PpmExtiCallback(uint32_t callbackNumber);
extern void SetRxDefaults(uint32_t rxProtocol, uint32_t usart);

extern int CalculateThrottleVelocity(void);
