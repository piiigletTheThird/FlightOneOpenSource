
#define START_PULSES
#define END_PULSES
#ifdef START_PULSES
#define PULSE_SHIFT     2
#else
#define PULSE_SHIFT     0
#endif

#define SEQUENCE_TIMEOUT	1200		//Max 0XFFFF OR 65535
#define DISABLE_PERIOD		0XFFFF	//Period to be used when the update interrupt is being used as a counter for disable time
#define DISABLE_TIME		15		//Time for lap capture to be deactivated after passing a gate. Value = (Desired delay in seconds)/.065536
#define CODE_SIZE			8		//Number of bits expected in IR code

//0 Pulse ranges
#define SHORT_MIN_PULSE         150
#define SHORT_MAX_PULSE         420

//1 Pulse ranges
#define LONG_MIN_PULSE          440
#define LONG_MAX_PULSE          710

//Duration of Off pulse (IR off in between 0 and 1 pulses)
#define OFF_DURATION            300

typedef enum
{
	READING_PULSES = 0,
	GATE_PASSED
  
} LAP_STATE;


void InitLaptimer(void);
void SpmLaptimerCallback(uint32_t callbackNumber);
