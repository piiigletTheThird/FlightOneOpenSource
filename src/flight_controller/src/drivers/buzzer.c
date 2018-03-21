#include "includes.h"


buzzerStatus_t buzzerStatus;


uint32_t startBuzzArray[]    = {225,225,200,200,175,175,150,150,125,125,100,100,75,75,50,50,25,25,0};
uint32_t lostBuzzArray[]     = {500,100,500,150,500,1};
uint32_t failsafeBuzzArray[] = {500,100,500,150,500,1};
uint32_t errorBuzzArray[]    = {100,200,300,400,500,600,700,1};
uint32_t armingBuzzArray[]   = {100,100,100,100,100,0};
//uint32_t lowBatteryBuzzArray[]  = {200,0,0,0,0,0,0,0,0,0,0,0,0,0};
uint32_t deadBatteryBuzzArray[]  = {200,50,100,50,200,50,100,0};
uint32_t switchBuzzArray[]   = {200,50,50,50,50,50,200,0};

int InitBuzzer(void)
{
	InitializeBuzzerPin(ports[board.buzzerPort], board.buzzerPin);
	return(0);
}


void InitializeBuzzerPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    HAL_GPIO_DeInit(GPIOx, GPIO_Pin);

    GPIO_InitStructure.Pin = GPIO_Pin;

    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStructure.Pull = GPIO_NOPULL;

	if (board.buzzerPolarity == TIM_OCPOLARITY_HIGH)
	{
		GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
		GPIO_InitStructure.Pull = GPIO_NOPULL;
	}
		
    HAL_GPIO_Init(GPIOx, &GPIO_InitStructure);

	if (board.buzzerPolarity == TIM_OCPOLARITY_HIGH)
		HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);

    buzzerStatus.status = STATE_BUZZER_OFF;

}


void DoBuzz(int on)
{
	if (on)
	{
		if (board.buzzerPolarity == TIM_OCPOLARITY_HIGH)
			HAL_GPIO_WritePin(ports[board.buzzerPort], board.buzzerPin, GPIO_PIN_SET);
		else
    		HAL_GPIO_WritePin(ports[board.buzzerPort], board.buzzerPin, GPIO_PIN_RESET);
	}
	else
	{
		if (board.buzzerPolarity == TIM_OCPOLARITY_HIGH)
			HAL_GPIO_WritePin(ports[board.buzzerPort], board.buzzerPin, GPIO_PIN_RESET);
		else
			HAL_GPIO_WritePin(ports[board.buzzerPort], board.buzzerPin, GPIO_PIN_SET);
	}
}


void UpdateBuzzer(void)
{
	uint32_t timeNow = InlineMillis();
	//checking if time needs to be updated
	//if (buzzerStatus.status != buzzerStatus.lastStatus)
	//{
	//	buzzerStatus.timeStart = timeNow;
	//}
	//buzzerStatus.lastStatus = buzzerStatus.status;

	//different states for buzzer
    switch(buzzerStatus.status)
    {
		default:
		case STATE_BUZZER_DEBUG:
			//do nothing
			break;
		case STATE_BUZZER_OFF:
			DoBuzz(0);
			break;
		case STATE_BUZZER_ON:
			ComplexBuzz(timeNow, switchBuzzArray);
			break;
		case STATE_BUZZER_LOST:
			ComplexBuzz(timeNow, lostBuzzArray);
			break;
		case STATE_BUZZER_ARMING:
			ComplexBuzz(timeNow, armingBuzzArray);
			break;
		case STATE_BUZZER_FAILSAFE:
			ComplexBuzz(timeNow, failsafeBuzzArray);
			break;
		case STATE_BUZZER_STARTUP:
			ComplexBuzz(timeNow, startBuzzArray);
			break;
		case STATE_BUZZER_DEADBAT:
			ComplexBuzz(timeNow, deadBatteryBuzzArray);
			break;
//		case STATE_BUZZER_LOWBAT:
//			ComplexBuzz(timeNow, lowBatteryBuzzArray);
			break;
		case STATE_BUZZER_ERROR:
			ComplexBuzz(timeNow, errorBuzzArray);
     }

}


void ComplexBuzz(uint32_t timeNow, uint32_t buzzArray[])
{

	uint32_t buzzLength;

	if (buzzerStatus.arrayIndex == 0)
	{
		buzzerStatus.timeStart = timeNow;
		buzzerStatus.timeStop  = timeNow + buzzArray[buzzerStatus.arrayIndex++];
		buzzerStatus.on = 1;
		DoBuzz(buzzerStatus.on);
		return;
	}

	buzzLength = buzzArray[buzzerStatus.arrayIndex];

	switch(buzzLength)
	{
		case 0:
			//end of array has been reached, 0 means we turn off buzzer
			buzzerStatus.status = STATE_BUZZER_OFF;
			buzzerStatus.arrayIndex = 0;
			buzzerStatus.on = 0;
			DoBuzz(buzzerStatus.on);
			break;
		case 1:
			buzzerStatus.arrayIndex = 0;
			buzzerStatus.on = 0;
			DoBuzz(buzzerStatus.on);
			break;
		default:
			//wait until time has expired to do anything.
			if (timeNow > buzzerStatus.timeStop)
			{
				//buzz on or off time has expired, increment the index and do the next buzz
				buzzerStatus.arrayIndex += 1;
				buzzerStatus.timeStart = timeNow;
				buzzerStatus.timeStop  = timeNow + buzzArray[buzzerStatus.arrayIndex];
				if (buzzerStatus.on)
				{
					buzzerStatus.on = 0;
					DoBuzz(buzzerStatus.on);
				}
				else
				{
					buzzerStatus.on = 1;
					DoBuzz(buzzerStatus.on);
					DoBuzz(1);
				}
			}
			break;
	}


}


void Buzz(uint32_t timeNow, uint16_t time1, uint16_t time2) //function to make the buzzer buzz
{
	//does this for the amount of time for time1
	if (((timeNow - buzzerStatus.timeStart) < time1) && (!buzzerStatus.on) )
	{
		DoBuzz(1);
		buzzerStatus.on = 1;
	}
	//does this for the amount of time2
	else if (((timeNow - buzzerStatus.timeStart) > time1) && ((timeNow - buzzerStatus.timeStart) < time2) && (buzzerStatus.on) )
	{
		DoBuzz(0);
		buzzerStatus.on = 0;
	}
	//if greater than time time2 reset timestart
	else if ((timeNow - buzzerStatus.timeStart) > time2 )
	{
		buzzerStatus.timeStart = timeNow;
	}
}


void BuzzTest()
{
	UpdateBuzzer();
	buzzerStatus.status = STATE_BUZZER_ERROR;
}
