#include "includes.h"

//0.02% error is acceptable
#define SOFT_PWM_MAX_BUMPS  12
#define SOFT_PWM_PWM_HZ     11000
#define SOFT_PWM_TIM_HZ     48000000
#define CAM_KEY_PWM_TIM_MS  180


volatile software_pwm_record softPwmRecord;
TIM_HandleTypeDef softPwmTimer;

static const int cameraResistanceValues[] =
{
    45000,
    27000,
    15000,
    6810,
    0
};

static void DeInitSoftPwmTimer(void)
{
    /*
    if(softPwmRecord.softwarePwmSatus == SPT_ENABLED)
    {
        inlineDigitalLo(ports[softPwmRecord.port], softPwmRecord.pin);
        HAL_GPIO_DeInit(ports[softPwmRecord.port], softPwmRecord.pin);
    }

    softPwmRecord.softwarePwmSatus = SPT_DISABLED;
	HAL_TIM_Base_Stop_IT(&softPwmTimer);
    callbackFunctionArray[GetTimerCallbackFromTimerEnum(board.generalTimer[2].timer)] = 0;
    */
}

static void InitSoftPwmTimer(uint32_t pwmHz, uint32_t timerHz)
{
    /*
    GPIO_InitTypeDef GPIO_InitStruct;
	uint16_t timerPrescaler = 0;

    HAL_GPIO_DeInit(ports[softPwmRecord.port], softPwmRecord.pin);
    inlineDigitalLo(ports[softPwmRecord.port], softPwmRecord.pin); //default state is low
    GPIO_InitStruct.Pin   = softPwmRecord.pin;
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(ports[softPwmRecord.port], &GPIO_InitStruct);

	callbackFunctionArray[GetTimerCallbackFromTimerEnum(board.generalTimer[2].timer)] = SoftPwmTimerCallback;

	timerPrescaler = (uint16_t)(SystemCoreClock / TimerPrescalerDivisor(board.generalTimer[2].timer) / timerHz) - 1;

	// Initialize timer
	softPwmTimer.Instance           = timers[board.generalTimer[2].timer];
	softPwmTimer.Init.Prescaler     = timerPrescaler;
	softPwmTimer.Init.CounterMode   = TIM_COUNTERMODE_UP;
	softPwmTimer.Init.Period        = (timerHz / pwmHz) - 1;
	softPwmTimer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_Base_Init(&softPwmTimer);
	HAL_TIM_Base_Start_IT(&softPwmTimer);

    HAL_NVIC_SetPriority(board.generalTimer[2].timerIRQn, 1, 0);
    HAL_NVIC_EnableIRQ(board.generalTimer[2].timerIRQn);
    */
}

void SoftPwmTimerCallback(uint32_t callbackNumber)
{
    /*

	(void)(callbackNumber);

    if (__HAL_TIM_GET_FLAG(&softPwmTimer, TIM_FLAG_UPDATE) != RESET)      //In case other interrupts are also running
    {

        if (__HAL_TIM_GET_ITSTATUS(&softPwmTimer, TIM_IT_UPDATE) != RESET)
        {

            __HAL_TIM_CLEAR_FLAG(&softPwmTimer, TIM_FLAG_UPDATE);

            if(InlineMillis() >= softPwmRecord.pwmStopTime)
            {
                DeInitSoftPwmTimer();
            }
            else
            {
                if(softPwmRecord.bumpCount++ < softPwmRecord.softwarePwmVoltage)
                    inlineDigitalHi(ports[softPwmRecord.port], softPwmRecord.pin);
                else
                    inlineDigitalLo(ports[softPwmRecord.port], softPwmRecord.pin);

                if(softPwmRecord.bumpCount == SOFT_PWM_MAX_BUMPS)
                    softPwmRecord.bumpCount = 0;
            }

        }

    }
*/
}

//initial init from scratch
int InitSoftPwm(void)
{
    /*
    DeInitSoftPwmTimer();
    softPwmRecord.softwarePwmVoltage = SPT_0_0;
    softPwmRecord.softwarePwmSatus   = SPT_DISABLED;
    softPwmRecord.bumpCount          = 0;
    */
    return(0);
}

int SendCamKey(uint32_t port, uint32_t pin, cam_key_t camKeyToSend)
{
    /*
    softPwmRecord.cameraKeyPressed = camKeyToSend;
    softPwmRecord.pwmStopTime = CAM_KEY_PWM_TIM_MS;
    SoftPwmVoltage(uint32_t port, uint32_t pin, software_pwm_voltage_t voltage)
    return(0);
    */
}

//program blind of init
int SoftPwmVoltage(uint32_t port, uint32_t pin, software_pwm_voltage_t voltage)
{
    /*
    //set voltage
    softPwmRecord.softwarePwmVoltage = voltage;
    //init if needed
    if(softPwmRecord.softwarePwmSatus == SPT_DISABLED)
    {
        softPwmRecord.port = port;
        softPwmRecord.pin  = pin;
        softPwmRecord.softwarePwmSatus = SPT_ENABLED;
        InitSoftPwmTimer(SOFT_PWM_PWM_HZ, SOFT_PWM_TIM_HZ);
    }
    return(0);
    */
}

int StopSoftPwm(void)
{
    /*
    if(softPwmRecord.softwarePwmSatus == SPT_ENABLED)
    {
        DeInitSoftPwmTimer();
        return(0);
    }
    else
    {
        return(1);
    }
    */
}