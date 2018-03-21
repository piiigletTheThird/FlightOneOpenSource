#pragma once

enum
{
    LEDS_OFF           = 0,
    LEDS_ON            = 1,
    LEDS_SLOW_BLINK    = 2,
    LEDS_MED_BLINK     = 3,
    LEDS_FAST_BLINK    = 4,
    LEDS_FASTER_BLINK  = 5,
    LEDS_FASTEST_BLINK = 6,
    LEDS_ERROR         = 7,
};

enum
{
	LED_MODE_OFF              = 0,
	LED_MODE_ON               = 1,
	LED_MODE_DISCO_FAST       = 2,
	LED_MODE_DISCO_SLOW       = 3,
	LED_MODE_PARTY_FAST       = 4,
	LED_MODE_PARTY_SLOW       = 5,
	LED_MODE_GYRO_MOTION      = 6,
	LED_MODE_COLOR_PULSE      = 7,
	LED_MODE_MULTI_DISCO_FAST = 8,
	LED_MODE_MULTI_DISCO_SLOW = 9,
	LED_MODE_MULTI_PARTY_FAST = 10,
	LED_MODE_MULTI_PARTY_SLOW = 11,
	LED_MODE_KNIGHT_RIDER     = 12,
	LED_MODE_BATTERY_LEVEL    = 13,
	LED_MODE_END              = 14,
};

#define MAX_LED_MODES 15
#define COLOR_CHART_SIZE 7



typedef struct ledStatus_t
{
    uint8_t status;
    uint8_t lastStatus;
    bool on;
    uint32_t timeStart;
    uint32_t timeStop;
} ledStatus_t;


extern uint8_t colorChart[COLOR_CHART_SIZE][3];
extern ledStatus_t ledStatus;

extern void DoLed(uint32_t number, uint32_t on);
extern int InitLeds (void);
extern void UpdateLeds(void);
extern void BlinkAllLeds(uint32_t timeNow, uint16_t time1, uint16_t time2);
extern void UpdateWs2812Leds(void);
extern void CoolLedEffect(uint32_t pwmPeriod, uint32_t dutyNumber, uint32_t ledNumber);
