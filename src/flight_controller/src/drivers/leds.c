#include "includes.h"

static uint32_t updateInterval = 100;
uint8_t greenTemp = 0;
uint8_t redTemp = 0;
uint8_t blueTemp = 0;
static uint8_t currentLedPulse = 0;
static uint8_t colorPulse = 0;
static uint32_t currentColorChart = 0;
uint32_t ledArraySize = 0;
int32_t adding = 1;
int32_t pixel = 0;


ledStatus_t ledStatus;

uint8_t colorChart[COLOR_CHART_SIZE][3] = {
		{255, 255, 255},
		{255, 255, 000},
		{255, 000, 000},
		{000, 255, 000},
		{000, 255, 255},
		{000, 000, 255},
		{255, 000, 255},
};
uint8_t rgbArray[WS2812_MAX_LEDS*3];

static void HandleCoolLeds(uint32_t heartbeatMs, uint32_t heartbeatMsHalf);
static inline uint8_t LowerLed(uint8_t input, uint8_t sub);
static inline void SetPixel(uint8_t pixel, uint8_t red, uint8_t green, uint8_t blue);
static inline void LedModeOff(uint32_t speed);
static inline void LedModeColor();
static inline void LedModeOn(uint32_t speed);
static inline void LedModeDisco(uint8_t speed);
static inline void LedModeParty(uint8_t speed);
static inline void LedModeGyroMotion(uint8_t speed);
static inline void LedModeColorPulse(uint8_t speed);
static inline void LedModeMultiDisco(uint8_t speed);
static inline void LedModeMultiParty(uint8_t speed);
static inline void LedModeKnightRider(uint8_t speed, uint8_t red, uint8_t green, uint8_t blue);
static inline void LedModeBatteryLevel(void);

static inline uint8_t LowerLed(uint8_t input, uint8_t sub)
{
	if ((input - sub) >= 0)
	{
		return(input - sub);
	}
	else
	{
		return(0);
	}
}

static inline void SetPixel(uint8_t pixel, uint8_t red, uint8_t green, uint8_t blue)
{
	//sets led in array, wont be need with matrix
	rgbArray[( ((pixel-1)*3))] = ~(uint8_t)green; //green is the first set, nothing needs to be added
	rgbArray[( ((pixel-1)*3) + 1)] = ~(uint8_t)red;
	rgbArray[( ((pixel-1)*3) + 2)] = ~(uint8_t)blue;
}

static inline void LedModeOff(uint32_t speed)
{
	//setup & math
	updateInterval = speed;
	uint32_t x = 0;

	//set colors
	for (x=0;x < mainConfig.ledConfig.ledCount+1;x++)
	{
		SetPixel(x,0,0,0);
	}
}

static inline void LedModeColor()
{
	//setup & math
	uint32_t x = 0;
	if (currentColorChart >= COLOR_CHART_SIZE)
	{
		currentColorChart = 0;
	}
	greenTemp = colorChart[currentColorChart++][1];
	redTemp = colorChart[currentColorChart++][0];
	blueTemp = colorChart[currentColorChart++][2];

	for (x=0;x < mainConfig.ledConfig.ledCount+1;x++)
	{
		SetPixel(x,redTemp,greenTemp,blueTemp);
	}
}

static inline void LedModeOn(uint32_t speed)
{
	//setup & math
	updateInterval = speed;
	uint32_t x = 0;


	for (x=0;x < mainConfig.ledConfig.ledCount+1;x++)
	{
		SetPixel(x,mainConfig.ledConfig.ledRed,mainConfig.ledConfig.ledGreen,mainConfig.ledConfig.ledBlue);
	}
}


static inline void LedModeDisco(uint8_t speed)
{
	//setup & math
	updateInterval=speed;
	uint32_t x = 0;
	greenTemp = ((rand() & 0xF0) + 1);
	redTemp = ((rand() & 0xF0) + 1);
	blueTemp = ((rand() & 0xF0) + 1);

	for (x=0;x < mainConfig.ledConfig.ledCount+1;x++)
	{
		SetPixel(x,redTemp,greenTemp,blueTemp);
	}
}


static inline void LedModeParty(uint8_t speed)
{
	//setup & math
	redTemp =  (rand() & 0xF0) + 1;
	greenTemp = (rand() & 0xF0) + 1;
	blueTemp = (rand() & 0xF0) + 1;
	updateInterval=speed;
	uint32_t x = 0;

	if (redTemp > greenTemp)
		greenTemp = 0;
	if (redTemp > blueTemp)
		blueTemp = 0;
	if (blueTemp > greenTemp)
		greenTemp = 0;
	if (blueTemp > redTemp)
		redTemp = 0;
	if (greenTemp > blueTemp)
		blueTemp = 0;
	if (greenTemp > redTemp)
		blueTemp = 0;

	for (x=0;x < mainConfig.ledConfig.ledCount+1;x++)
	{
		SetPixel(x,redTemp,greenTemp,blueTemp);
	}

}

static inline void LedModeGyroMotion(uint8_t speed)
{
	 //setup & math
	updateInterval=speed;
	uint32_t x = 0;

	for (x=0;x < mainConfig.ledConfig.ledCount+1;x++)
	{
		SetPixel(x,(uint8_t)CONSTRAIN(ABS(filteredGyroData[YAW]),0,254)+1,(uint8_t)CONSTRAIN(ABS(filteredGyroData[ROLL]),0,254)+1,(uint8_t)CONSTRAIN(ABS(filteredGyroData[PITCH]),0,254)+1);
	}
}

static inline void LedModeColorPulse(uint8_t speed)
{
	//setup & math
	updateInterval=speed;
	colorPulse +=1;
	uint32_t x = 0;

	if (colorPulse > 254)
	{
		currentLedPulse++;
		colorPulse = 0;
	}
	redTemp= 0;
	blueTemp= 0;
	greenTemp=0;



	//for loop for setting the led, dont change any math in here if you want all to update at once
	for (x=0;x < mainConfig.ledConfig.ledCount+1;x++)
	{
		switch (currentLedPulse)
		{
			case 3:
				currentLedPulse=0;
			case 0:
				SetPixel(x,colorPulse,0,0);
				break;
			case 1:
				SetPixel(x,0,colorPulse,0);
				break;
			case 2:
				SetPixel(x,0,0,colorPulse);
				break;
		}
	}
}

static inline void LedModeMultiDisco(uint8_t speed)
{
	//setup & math
	updateInterval=speed;
	uint32_t x = 0;

	for (x=0;x < mainConfig.ledConfig.ledCount+1;x++)
	{
		greenTemp = ((rand() & 0xF0) + 1);
		redTemp = ((rand() & 0xF0) + 1);
		blueTemp = ((rand() & 0xF0) + 1);

		SetPixel(x,redTemp,greenTemp,blueTemp);
	}
}

static inline void LedModeMultiParty(uint8_t speed)
{
	//setup & math
	redTemp = (rand() & 0xF0) + 1;
	greenTemp = (rand() & 0xF0) + 1;
	blueTemp = (rand() & 0xF0) + 1;
	updateInterval=speed;
	uint32_t x = 0;


	for (x=0;x < mainConfig.ledConfig.ledCount+1;x++)
	{
		redTemp = (rand() & 0xF0) + 1;
		greenTemp = (rand() & 0xF0) + 1;
		blueTemp = (rand() & 0xF0) + 1;
		if (redTemp > greenTemp)
			greenTemp = 0;
		if (redTemp > blueTemp)
			blueTemp = 0;
		if (blueTemp > greenTemp)
			greenTemp = 0;
		if (blueTemp > redTemp)
			redTemp = 0;
		if (greenTemp > blueTemp)
			blueTemp = 0;
		if (greenTemp > redTemp)
			blueTemp = 0;

		SetPixel(x,redTemp,blueTemp,greenTemp);
	}
}

static inline void LedModeKnightRider(uint8_t speed, uint8_t red, uint8_t green, uint8_t blue)
{
	//setup & math
	updateInterval=speed;

	if (adding) {
		pixel = pixel + 1;
		if (pixel == ((int32_t)mainConfig.ledConfig.ledCount+1))
		{
			adding=0;
			pixel -= 2;
		}
	}
	else
	{
		pixel = pixel - 1;
		if (pixel == 0)
		{
			adding=1;
			pixel+=2;
		}
	}


	SetPixel(pixel, red, green, blue);

	if (adding)
	{
		if ((pixel - 1) >= 0 )
		{
			SetPixel(pixel - 1, LowerLed(red, 200), LowerLed(green, 200), LowerLed(blue, 200));
		}
		if ((pixel - 2) >= 0 )
		{
			SetPixel(pixel - 2, LowerLed(red, 250), LowerLed(green, 250), LowerLed(blue, 250));
		}
		if ((pixel - 3) >= 0 )
		{
			SetPixel(pixel - 3, LowerLed(red, 255), LowerLed(green, 255), LowerLed(blue, 255));
		}
	}

	else
	{
		if ((pixel + 1) <= 7 )
		{
			SetPixel(pixel + 1, LowerLed(red, 200), LowerLed(green, 200), LowerLed(blue, 200));
		}
		if ((pixel + 2) <= 7 )
		{
			SetPixel(pixel + 2, LowerLed(red, 250), LowerLed(green, 250), LowerLed(blue, 250));
		}
		if ((pixel + 3) <= 7 )
		{
			SetPixel(pixel + 3, LowerLed(red, 255), LowerLed(green, 255), LowerLed(blue, 255));
		}
	}

}

static inline void LedModeBatteryLevel()
{
	if (averageVoltage >= fullVoltage)
	{
		LedModeKnightRider(60, 0, 255, 0);
	}
	if (averageVoltage > runningVoltage && averageVoltage<fullVoltage)
	{
		LedModeKnightRider(80, 0, 100, 150);
	}
	if (averageVoltage > lowVoltage && averageVoltage<=runningVoltage)
	{
		LedModeKnightRider(100, 150, 255, 0);
	}
	if (averageVoltage <= lowVoltage)
	{
		LedModeKnightRider(120, 255, 0, 0);
	}
}


int InitLeds (void)
{
	int x;

	for (x=0;x<3;x++)
	{
		if (board.internalLeds[x].enabled)
		{
			InitializeGpio(ports[board.internalLeds[x].port], board.internalLeds[x].pin, 0);
		}
	}
	ledStatus.status = LEDS_OFF;
	return(0);
}


void DoLed(uint32_t number, uint32_t on)
{
	if (on)
	{
		if (board.internalLeds[number].enabled)
			HAL_GPIO_WritePin(ports[board.internalLeds[number].port], board.internalLeds[number].pin, GPIO_PIN_RESET);
	}
	else
	{
		if (board.internalLeds[number].enabled)
			HAL_GPIO_WritePin(ports[board.internalLeds[number].port], board.internalLeds[number].pin, GPIO_PIN_SET);
	}
}

void UpdateLeds(void)
{
	uint32_t timeNow = InlineMillis();

	if (ledStatus.status != ledStatus.lastStatus)
	{
		ledStatus.timeStart = timeNow;
	}
	ledStatus.lastStatus = ledStatus.status;

	switch (ledStatus.status)
	{
		case LEDS_OFF:
			DoLed(0, 0);
			DoLed(1, 0);
			DoLed(2, 0);
			break;
		case LEDS_ON:
			DoLed(0, 1);
			DoLed(1, 1);
			DoLed(2, 1);
			break;
		case LEDS_SLOW_BLINK:
			HandleCoolLeds(2000, 1000);
			break;
		case LEDS_MED_BLINK:
			HandleCoolLeds(1500, 500);
			break;
		case LEDS_FAST_BLINK:
			HandleCoolLeds(1000, 250);
			break;
		case LEDS_FASTER_BLINK:
			HandleCoolLeds(300, 150);
			break;
		case LEDS_FASTEST_BLINK:
			HandleCoolLeds(150, 75);
			break;
		default:
			DoLed(0, 0);
			DoLed(1, 0);
			DoLed(2, 0);
			break;
	}
}

void HandleCoolLeds(uint32_t heartbeatMs, uint32_t heartbeatMsHalf)
{
	uint32_t dutyNumber;

	dutyNumber = (InlineMillis() % heartbeatMs);

	if (dutyNumber > heartbeatMsHalf)
	{
		dutyNumber = (heartbeatMs - dutyNumber) * 2;
	}
	else
	{
		dutyNumber *= 2;
	}

	//update 1000 times faster
	CoolLedEffect(heartbeatMs, dutyNumber, 0);

}

void BlinkAllLeds(uint32_t timeNow, uint16_t time1, uint16_t time2)
{
	if (((timeNow - ledStatus.timeStart) < time1) && (ledStatus.on) )
	{
		DoLed(0, 0);
		DoLed(1, 0);
		DoLed(2, 0);
		ledStatus.on = false;
	} else if (((timeNow - ledStatus.timeStart) > time1) && ((timeNow - ledStatus.timeStart) < time2) && (!ledStatus.on) )
	{
		DoLed(0, 1);
		DoLed(1, 1);
		DoLed(2, 1);
		ledStatus.on = true;
	} else if ((timeNow - ledStatus.timeStart) > time2 )
	{
		ledStatus.timeStart = timeNow;
	}
}

void CoolLedEffect(uint32_t pwmPeriod, uint32_t dutyNumber, uint32_t ledNumber)
{
	//pwm an LED based on Micros()
	uint32_t currentTime;
	uint32_t currentDutyIn;
	uint32_t pwmOffAt;

	currentTime   = Micros();
	currentDutyIn = (currentTime % pwmPeriod);
	pwmOffAt      = currentTime - currentDutyIn + dutyNumber;

	if (currentTime > pwmOffAt)
		DoLed(ledNumber, 1);
	else
		DoLed(ledNumber, 0);

}

inline void UpdateWs2812Leds(void)
{

	static uint32_t currentLedMode = 0;
	static uint32_t colorLatch = 0;
	static uint32_t modeLatch  = 0;
	static uint32_t onceDone = 0;
	static uint32_t lastUpdate = 0;


	if (!ws2812LedRecord.enabled)
		return;

	//check color and mode change modes
	if ( ModeActive(M_LEDCOLOR) && !colorLatch )
	{
		colorLatch = 1;
		LedModeColor();

	}
	else if (!ModeActive(M_LEDCOLOR))
	{
		colorLatch = 0;
	}

	if ( ModeActive(M_LEDMODE) && !modeLatch )
	{
		modeLatch = 1;
		currentLedMode ++;
		if (currentLedMode >= LED_MODE_END)
		{
			currentLedMode = 0;
		}
		mainConfig.ledConfig.ledMode = currentLedMode;
	}
	else if (!ModeActive(M_LEDMODE))
	{
		modeLatch = 0;
	}

	if ( ModeSet(M_BUZZER) && !onceDone)
	{
		onceDone = 1;
		currentLedMode = 0;
	}


	//Update WS2812 LEDs
	if ( ( InlineMillis() - lastUpdate ) > updateInterval )
	{
		lastUpdate = InlineMillis();

		mainConfig.ledConfig.ledCount = CONSTRAIN(mainConfig.ledConfig.ledCount, 1, WS2812_MAX_LEDS);
		ledArraySize = mainConfig.ledConfig.ledCount*3;

		switch (mainConfig.ledConfig.ledMode)
		{
			case LED_MODE_OFF:
				//leds off
				LedModeOff(5000);
				break;
			case LED_MODE_ON:
				//leds on
				LedModeOn(5000);
				break;
			case LED_MODE_DISCO_FAST:
				//Disco Fast
				LedModeDisco(20);
				break;
			case LED_MODE_DISCO_SLOW:
				//Disco Slow
				LedModeDisco(100);
				break;
			case LED_MODE_PARTY_FAST:
				//Party Fast
				LedModeParty(20);
				break;
			case LED_MODE_PARTY_SLOW:
				//Party Slow
				LedModeParty(100);
				break;
			case LED_MODE_GYRO_MOTION:
				//Gyro motion Slow
				LedModeGyroMotion(5);
				break;
			case LED_MODE_COLOR_PULSE:
				//Color Pulse
				LedModeColorPulse(20);
				break;
			case LED_MODE_MULTI_DISCO_FAST:
				//Disco Fast
				LedModeMultiDisco(20);
				break;
			case LED_MODE_MULTI_DISCO_SLOW:
				//Disco Slow
				LedModeMultiDisco(100);
				break;
			case LED_MODE_MULTI_PARTY_FAST:
				//Party Fast
				LedModeMultiParty(20);
				break;
			case LED_MODE_MULTI_PARTY_SLOW:
				//Party Slow
				LedModeMultiParty(100);
				break;

			case LED_MODE_KNIGHT_RIDER:
				//Change color based on battery level
				LedModeKnightRider(120, 255, 0, 0);
				break;
			case LED_MODE_BATTERY_LEVEL:
				LedModeBatteryLevel();
				break;
		}
		//updates the led
	OutputSerialDmaByte(rgbArray,ledArraySize, ws2812LedRecord.ws2812Actuator, 1, 0, 0);
	}
}
