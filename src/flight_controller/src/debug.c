#include "includes.h"

uint32_t debugU32[8];
float    debugF[8];

extern void InitDebugGpio(void)
{
	//PC6,
	//PC7
	InitializeGpio(ports[_PORTB], GPIO_PIN_6, 0);
	InitializeGpio(ports[_PORTB], GPIO_PIN_7, 0);
}

extern void DebugGpioOn(void)
{
	inlineDigitalHi(ports[_PORTB], GPIO_PIN_6);
}

extern void DebugGpioOff(void)
{
	inlineDigitalLo(ports[_PORTB], GPIO_PIN_6);
}
