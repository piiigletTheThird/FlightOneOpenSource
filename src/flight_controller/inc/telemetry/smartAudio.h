#pragma once

enum
{
	SM_GET_SETTINGS       = 0x01,
	SM_SET_POWER          = 0x02,
	SM_SET_CHANNEL        = 0x03,
	SM_SET_FREQUENCY      = 0x04,
	SM_SET_OPERATION_MODE = 0x05,
	SM_START_CODE1        = 0xAA,
	SM_START_CODE2        = 0x55,
};

enum
{
	SM_OPMODE_PMIR        = 1,
	SM_OPMODE_PMOR        = 2,
	SM_OPMODE_PM          = 4,
	SM_OPMODE_LOCKED      = 8,

};

enum
{
	SM_SET_OPMODE_PMIR     = 0x01,
	SM_SET_OPMODE_PMOR     = 0x02,
	SM_SET_OPMODE_PM       = 0x04,
	SM_SET_OPMODE_DIS_PMIR = 0x05,
	SM_SET_OPMODE_DIS_PMOR = 0x06,
};

enum
{
	SM_VERSION_1 = 0x01,
	SM_VERSION_2 = 0x09,
};

extern uint8_t  smartAudioRxBuffer[];
extern uint8_t  smartAudioTxBuffer[];

extern uint32_t SmartAudioVtxTurnOn(void);
extern uint32_t SmartAudioVtxTurnPit(void);
extern uint32_t InitSmartAudio(void);
extern void     DeInitSmartAudio(void);
extern uint32_t SmartAudioGetSettings(void);
extern uint32_t SmartAudioVtxPower(uint32_t powerLevel);
extern uint32_t SmartAudioVtxBandChannel(uint32_t bandChannel);

/*
extern volatile uint8_t rfVtxRxBuffer[];

extern void     InitRfVtx(uint32_t usartNumber);
extern uint32_t RfVtxOff(void);
extern uint32_t RfVtxBaud(void);
extern uint32_t RfVtxOn25(void);
extern uint32_t RfVtxOn200(void);
extern uint32_t RfVtxBand(uint32_t band);
extern uint32_t RfVtxChannel(uint32_t channel);
*/
