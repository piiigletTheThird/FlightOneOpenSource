#pragma once

enum
{
	TELEM_OFF          = 0,
	TELEM_USART1       = 1,
	TELEM_USART2       = 2,
	TELEM_USART3       = 3,
	TELEM_USART4       = 4,
	TELEM_USART5       = 5,
	TELEM_USART6       = 6,
	TELEM_ACTUATOR1    = 7,
	TELEM_ACTUATOR2    = 8,
	TELEM_ACTUATOR3    = 9,
	TELEM_ACTUATOR4    = 10,
	TELEM_ACTUATOR5    = 11,
	TELEM_ACTUATOR6    = 12,
	TELEM_ACTUATOR7    = 13,
	TELEM_ACTUATOR8    = 14,
	TELEM_SS1W_USART1R = 15,
	TELEM_SS1W_USART2R = 16,
	TELEM_SS1W_USART3R = 17,
	TELEM_SS1W_USART4R = 18,
	TELEM_SS1W_USART5R = 19,
	TELEM_SS1W_USART6R = 20,
	TELEM_SS1W_USART1T = 21,
	TELEM_SS1W_USART2T = 22,
	TELEM_SS1W_USART3T = 23,
	TELEM_SS1W_USART4T = 24,
	TELEM_SS1W_USART5T = 25,
	TELEM_SS1W_USART6T = 26,
	TELEM_INTERNAL_OSD = 27,
	TELEM_NUM          = 28,
};

enum
{
	VTX_POWER_UN    = -1,
	VTX_POWER_025MW =  0,
	VTX_POWER_200MW =  1,
	VTX_POWER_500MW =  2,
	VTX_POWER_800MW =  3,
	VTX_POWER_END   =  4,
};

/*
enum
{
	VTX_POWER_UN    = -1,
	VTX_POWER_025MW =  0,
	VTX_POWER_050MW =  1,
	VTX_POWER_100MW =  2,
	VTX_POWER_200MW =  3,
	VTX_POWER_400MW =  4,
	VTX_POWER_500MW =  5,
	VTX_POWER_600MW =  6,
	VTX_POWER_800MW =  7, 
	VTX_POWER_END   =  8,
};
*/

enum
{
	VTX_DEVICE_NONE    = 0,
	VTX_DEVICE_SMARTV1 = 1,
	VTX_DEVICE_SMARTV2 = 2,
	VTX_DEVICE_TRAMP   = 3,
	VTX_DEVICE_RF      = 4,
};

enum
{
	VTX_MODE_PIT    = 0,
	VTX_MODE_ACTIVE = 1,
};

enum
{
	VTX_REGION_UNKNOWN = -1,
	VTX_REGION_US      = 0,
	VTX_REGION_EU      = 1,
};

enum
{
	VTX_BAND_A   = 0,
	VTX_BAND_B   = 1,
	VTX_BAND_E   = 2,
	VTX_BAND_F   = 3,
	VTX_BAND_R   = 4,
	VTX_BAND_END = 5,
};

enum
{
	VTX_CHANNEL_1   = 0,
	VTX_CHANNEL_2   = 1,
	VTX_CHANNEL_3   = 2,
	VTX_CHANNEL_4   = 3,
	VTX_CHANNEL_5   = 4,
	VTX_CHANNEL_6   = 5,
	VTX_CHANNEL_7   = 6,
	VTX_CHANNEL_8   = 7,
	VTX_CHANNEL_END = 8,
};

enum
{
	VTX_CH_UN  = -1,
	VTX_CH_A1  = 0,
	VTX_CH_A2  = 1,
	VTX_CH_A3  = 2,
	VTX_CH_A4  = 3,
	VTX_CH_A5  = 4,
	VTX_CH_A6  = 5,
	VTX_CH_A7  = 6,
	VTX_CH_A8  = 7,
	VTX_CH_B1  = 8,
	VTX_CH_B2  = 9,
	VTX_CH_B3  = 10,
	VTX_CH_B4  = 11,
	VTX_CH_B5  = 12,
	VTX_CH_B6  = 13,
	VTX_CH_B7  = 14,
	VTX_CH_B8  = 15,
	VTX_CH_E1  = 16,
	VTX_CH_E2  = 17,
	VTX_CH_E3  = 18,
	VTX_CH_E4  = 19,
	VTX_CH_E5  = 20,
	VTX_CH_E6  = 21,
	VTX_CH_E7  = 22,
	VTX_CH_E8  = 23,
	VTX_CH_F1  = 24,
	VTX_CH_F2  = 25,
	VTX_CH_F3  = 26,
	VTX_CH_F4  = 27,
	VTX_CH_F5  = 28,
	VTX_CH_F6  = 29,
	VTX_CH_F7  = 30,
	VTX_CH_F8  = 31,
	VTX_CH_R1  = 32,
	VTX_CH_R2  = 33,
	VTX_CH_R3  = 34,
	VTX_CH_R4  = 35,
	VTX_CH_R5  = 36,
	VTX_CH_R6  = 37,
	VTX_CH_R7  = 38,
	VTX_CH_R8  = 39,
	VTX_CH_END = 40,
};

typedef struct
{
	uint32_t telemSmartAudio;
    uint32_t telemSport;
    uint32_t telemSpek;
	uint32_t telemCrsf;
    uint32_t telemMsp;
    uint32_t telemMav;
    uint32_t telemTramp;
	uint32_t telemRfOsd;
    float    adcCurrFactor;
	uint32_t vtxPitmodeType;
	uint32_t vbatbuzzer;
	float    vbatCutoff;
	uint32_t batSize;
	uint32_t crsfOtxCurHack;
	uint32_t logMask1;
	uint32_t logMask2;
} telem_config;

typedef struct
{
	int vtxDevice;
	int vtxBand;
	int vtxChannel;
	int vtxBandChannel;
	int vtxPower;
	int vtxPit;
	int vtxRegion;
	int vtxFrequency;
	int vtxTemp;
} vtx_record;

extern volatile vtx_record vtxRequested;
extern volatile vtx_record vtxRecord;
extern volatile uint32_t   sendSmartPortAt;
extern volatile uint32_t   sendSmartPortLuaAt;
extern volatile uint32_t   sendCrsfTelemtryAt;
extern volatile uint32_t   sendSpektrumTelemtryAt;

extern volatile uint32_t telemEnabled;
extern volatile uint32_t lastTimeSPort;
extern volatile uint32_t okToSendSPort;
extern volatile uint32_t sPortExtiSet;

extern void InitTelemtry(void);
extern void ProcessTelemtry(void);
extern void TelemtryRxCallback(uint8_t serialBuffer[], uint32_t outputLength);
extern void TelemtryTxCallback(uint8_t serialBuffer[], uint32_t outputLength);
extern void SportSoftSerialExtiCallback(uint32_t callbackNumber);
extern void SportSoftSerialDmaCallback(uint32_t callbackNumber);
extern int  VtxTurnOn(void);
extern int  VtxTurnPit(void);
extern int  VtxBandChannel(int bandChannel);
extern int  VtxPower(int power);
extern int  VtxBandChannelToFrequency(int bandChannel);
extern void VtxChannelToBandAndChannel(int inChannel, volatile int *vtxBand, volatile int *channel);
extern int  VtxBandAndChannelToBandChannel(volatile int vtxBand, volatile int channel);
extern int  VtxFrequencyToBandChannel(int frequency);