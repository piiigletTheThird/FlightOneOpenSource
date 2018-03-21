#include "includes.h"


//todo: will need to move this to MCU based
int InitWatchdog(watchdog_timeout timeout)
{
	int result = 1;
	uint16_t reload = 0;

	if (RCC->CSR & RCC_CSR_WWDGRSTF)
	{

		// Reset by IWDG
		result = 0;

		// Clear reset flags
		RCC->CSR |= RCC_CSR_RMVF;
	}

	// Enable write access to IWDG_PR and IWDG_RLR registers
	IWDG->KR = 0x5555;
	// Set proper clock depending on timeout user select
	if (timeout >= WATCHDOG_TIMEOUT_8S) {
		// IWDG counter clock: LSI/256 = 128Hz
		IWDG->PR = 0x07;
	}
	else {
		// IWDG counter clock: LSI/32 = 1024Hz
		IWDG->PR = 0x03;
	}

	// Set counter reload value
	if (timeout == WATCHDOG_TIMEOUT_5MS) {
		reload = 5; // 1024 Hz IWDG ticking
	}
	else if (timeout == WATCHDOG_TIMEOUT_10MS) {
		reload = 10; // 1024 Hz IWDG ticking
	}
	else if (timeout == WATCHDOG_TIMEOUT_15MS) {
		reload = 15; // 1024 Hz IWDG ticking
	}
	else if (timeout == WATCHDOG_TIMEOUT_30MS) {
		reload = 31; // 1024 Hz IWDG ticking
	}
	else if (timeout == WATCHDOG_TIMEOUT_60MS) {
		reload = 61; // 1024 Hz IWDG ticking
	}
	else if (timeout == WATCHDOG_TIMEOUT_120MS) {
		reload = 123; // 1024 Hz IWDG ticking
	}
	else if (timeout == WATCHDOG_TIMEOUT_250MS) {
		reload = 255; // 1024 Hz IWDG ticking
	}
	else if (timeout == WATCHDOG_TIMEOUT_500MS) {
		reload = 511; // 1024 Hz IWDG ticking
	}
	else if (timeout == WATCHDOG_TIMEOUT_1S) {
		reload = 1023; // 1024 Hz IWDG ticking
	}
	else if (timeout == WATCHDOG_TIMEOUT_2S) {
		reload = 2047; // 1024 Hz IWDG ticking
	}
	else if (timeout == WATCHDOG_TIMEOUT_4S) {
		reload = 4095; // 1024 Hz IWDG ticking
	}
	else if (timeout == WATCHDOG_TIMEOUT_8S) {
		reload = 1023; // 128 Hz IWDG ticking
	}
	else if (timeout == WATCHDOG_TIMEOUT_16S) {
		reload = 2047; // 128 Hz IWDG ticking
	}
	else if (timeout == WATCHDOG_TIMEOUT_32S) {
		reload = 4095; // 128 Hz IWDG ticking
	}

	// Set reload
	IWDG->RLR = reload;
	// Reload IWDG counter
	IWDG->KR = 0xAAAA;
	// Enable IWDG (the LSI oscillator will be enabled by hardware)
	IWDG->KR = 0xCCCC;

	/* Return status */
	return(result);
}


inline void FeedTheDog(void) {
	IWDG->KR = 0xAAAA;
}
