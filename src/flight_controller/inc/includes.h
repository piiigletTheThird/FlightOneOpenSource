#pragma once

extern volatile int retValChk;

//enable or disable new or old logging method
#define OLD_LOG

//#define LOG32
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stddef.h>
#include <ctype.h>
#include <limits.h>

//input
#include "input/gyro.h"
#include "usbd_hid.h"
#include "usb_device.h"

//low level drivers and driver based functions
#include "../../low_level_driver/boarddef.h"
#include "mcu_include.h"
#include "general_functions.h"
#include "exti.h"

//general includes
#include "imu.h"
#include "quopa.h"
#include "pid.h"
#include "mixer.h"
#include "modes.h"
#include "debug.h"
#include "flight.h"
#include "filter.h"
#include "wizard.h"
#include "watchdog.h"
#include "learning.h"
#include "scheduler.h"
#include "MaxOsdCharmap.h"
#include "newSoftSerial.h"
#include "flight_logger.h"
#include "rfbl_functions.h"

//telemetry includes
#include "telemetry/crossfire.h"
#include "telemetry/dshotTwoWay.h"
#include "telemetry/sPortTelemetry.h"
#include "telemetry/telemetry.h"
#include "telemetry/mspTelemetry.h"
#include "telemetry/spektrumTelemetry.h"
#include "telemetry/smartAudio.h"
#include "telemetry/rfVtx.h"
#include "telemetry/rfosdTelemetry.h"
#include "telemetry/trampTelemetry.h"

//all drivers
#include "drivers/softPwm.h"
#include "drivers/adc.h"
#include "drivers/esc_1wire.h"
#include "drivers/softSerial.h"
#include "drivers/transponder.h"
#include "drivers/serial.h"
#include "drivers/leds.h"
#include "drivers/buzzer.h"
#include "drivers/rx.h"
#include "drivers/flash_chip.h"
#include "drivers/maxOsd.h"
#include "drivers/actuator_output.h"
#include "drivers/invensense_bus.h"
#include "drivers/invensense_device.h"
#include "drivers/invensense_register_map.h"
#include "drivers/dmaShenanigans.h"
#include "drivers/spm_laptimer.h"


//config
#include "config.h"
#include "rf_math.h"

#include <arm_math.h>

extern int BoardInit(void);
