#pragma once

extern uint32_t activeModes;
extern uint32_t setModes;

typedef struct {
    const char *modeString;
    const uint32_t modeNumber;
    const uint32_t modeMask;
} string_modes_rec;

//also modify stringModes in modes.c
enum {
	M_ARMED      = (1 << 0),
	M_ATTITUDE   = (1 << 1),
	M_HORIZON    = (1 << 2),
	M_FAILSAFE   = (1 << 3),
	M_LOGGING    = (1 << 4),
	M_BUZZER     = (1 << 5),
	M_LEDMODE    = (1 << 6),
	M_LEDCOLOR   = (1 << 7),
	M_DIRECT     = (1 << 8),
	M_VTXON      = (1 << 9),
	M_BRAINDRAIN = (1 << 10),
	M_PROFILE2   = (1 << 11),
	M_PROFILE3   = (1 << 12),
	M_QUOPA      = (1 << 13),
	M_LEARN      = (1 << 14),
	M_BEEP       = (1 << 15),
	M_TURKEY     = (1 << 16),
	//M_CATMODE    = (1 << 11),
};

extern string_modes_rec stringModes[];

extern void PrintModes(void);
extern void InitModes(void);
extern void EnableMode(uint32_t modeMask);
extern void DisableMode(uint32_t modeMask);
extern uint32_t ModeActive(uint32_t modeMask);
extern uint32_t ModeSet(uint32_t modeMask);
extern void SetMode(uint32_t modeMask, uint16_t channel, int16_t minRc, int16_t maxRc);
extern void SetupModes(char *modString);
extern void CheckRxToModes(void);
