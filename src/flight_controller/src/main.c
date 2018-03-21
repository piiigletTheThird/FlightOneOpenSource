//#define STM32F405xx
#include "includes.h"

volatile int retValChk;

int main(void)
{
	//Absolutely no MCU specific code to go here.

    int count = 16;

    //TODO: Make automatic
	retValChk = VectorIrqInit(ADDRESS_RFFW_START);

	//TODO Needs to pull parameters from flash here. For now we use defines
	retValChk = GetBoardHardwareDefs();

    retValChk = InitializeMCUSettings();

    retValChk = BoardInit();

    //DshotInit(1);
    
    retValChk = HandleRfbl();

    retValChk = LoadConfig(ADDRESS_CONFIG_START);

    SpektrumBind(mainConfig.rcControlsConfig.bind);

    retValChk = HandleFcStartupReg();

    retValChk = InitBuzzer();
    retValChk = InitLeds();

    retValChk = InitUsb();

    retValChk = InitFlight(mainConfig.mixerConfig.escProtocol, mainConfig.mixerConfig.escUpdateFrequency);

    //what happens if you do "save" when these modes are active?
    retValChk = InitQuopaMode();
    retValChk = InitDshotBeep();
    retValChk = InitDshotCommandState(); // send dshot commands and listen back if needed
    
    retValChk = InitWatchdog(WATCHDOG_TIMEOUT_32S);

    buzzerStatus.status = STATE_BUZZER_STARTUP;
    ledStatus.status    = LEDS_SLOW_BLINK;

    //DeInitActuators();
    //DelayMs(10);
    //InitializeGpio(ports[ACTUATOR2_GPIO], ACTUATOR2_PIN, 0);
    //inlineDigitalHi(ports[ACTUATOR2_GPIO], ACTUATOR2_PIN);
    //InitializeGpio(ports[ENUM_PORTB], GPIO_PIN_0, 0);

    while (1)
    {
        inlineDigitalHi(ports[ACTUATOR2_GPIO], ACTUATOR2_PIN);
    	Scheduler(count--);

    	if (count == -1)
    		count = 16;

		//If 1wire is run, the gyro is disabled for the SPMFCF400.  This prevents the watchdog from ever resetting, causing the board to reset.
	    if (oneWireHasRun)
		    FeedTheDog();
    }

}