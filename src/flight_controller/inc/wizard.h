#pragma once
#include "includes.h"

enum {
		WIZ_NONE = 0,
		WIZ_RC = 1,
		WIZ_RX = 2,
};

enum {
	WIZ_RC_THROTTLE_UP = 0,
	WIZ_RC_YAW_RIGHT   = 1,
	WIZ_RC_PITCH_UP    = 2,
	WIZ_RC_ROLL_RIGHT  = 3,
	WIZ_RC_DONE        = 4,
};

typedef struct {
	uint32_t currentWizard;
    uint32_t currentStep;
    uint32_t wicRcCheckDirection;
} wizard_record;

extern wizard_record wizardStatus;

extern void OneWire(char *inString);
extern void SetupWizard(char *inString);
extern void WizMixer(char *inString);

extern void HandleWizRc(void);
