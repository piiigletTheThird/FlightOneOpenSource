#pragma once

#include "includes.h"


typedef enum {
	WATCHDOG_TIMEOUT_5MS   = 0x00,  /*!< System reset called every 5ms   */
	WATCHDOG_TIMEOUT_10MS  = 0x01,  /*!< System reset called every 10ms  */
	WATCHDOG_TIMEOUT_15MS  = 0x02,  /*!< System reset called every 15ms  */
	WATCHDOG_TIMEOUT_30MS  = 0x03,  /*!< System reset called every 30ms  */
	WATCHDOG_TIMEOUT_60MS  = 0x04,  /*!< System reset called every 60ms  */
	WATCHDOG_TIMEOUT_120MS = 0x05,  /*!< System reset called every 120ms */
	WATCHDOG_TIMEOUT_250MS = 0x06,  /*!< System reset called every 250ms */
	WATCHDOG_TIMEOUT_500MS = 0x07,  /*!< System reset called every 500ms */
	WATCHDOG_TIMEOUT_1S    = 0x08,  /*!< System reset called every 1s    */
	WATCHDOG_TIMEOUT_2S    = 0x09,  /*!< System reset called every 2s    */
	WATCHDOG_TIMEOUT_4S    = 0x0A,  /*!< System reset called every 4s    */
	WATCHDOG_TIMEOUT_8S    = 0x0B,  /*!< System reset called every 8s    */
	WATCHDOG_TIMEOUT_16S   = 0x0C,  /*!< System reset called every 16s   */
	WATCHDOG_TIMEOUT_32S   = 0x0D   /*!< System reset called every 32s. This is maximum value allowed with IWDG timer */
} watchdog_timeout;


extern int InitWatchdog(watchdog_timeout timeout);
extern void FeedTheDog(void);
