#include <stdbool.h>

#include "includes.h"

#include "input/gyro.h"

#ifdef GYRO_EXTI
#include "exti.h"
#endif

I2C_HandleTypeDef gyro_i2c;
uint8_t i2cGyroDmaTx;
uint8_t i2cGyroDmaRx[RXBUFFERSIZE];

static void I2cInit()
{
	//TODO: check all these
	gyro_i2c.Instance             = I2C1;//TODO: looks like this can either be 1 or 2, may need to configure differntly depedning

	gyro_i2c.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
	gyro_i2c.Init.ClockSpeed      = 400000;// looks like this could go to 1000 Khz
	gyro_i2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	gyro_i2c.Init.DutyCycle       = I2C_DUTYCYCLE_16_9;
	gyro_i2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	gyro_i2c.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
	gyro_i2c.Init.OwnAddress1     = 0x00;//TODO: this is probbaly not right
    gyro_i2c.Init.OwnAddress2     = 0xFE;
}


bool I2cAccGyroWrite()
{
	while(HAL_I2C_Master_Transmit_DMA(&gyro_i2c, (uint16_t)gyro_i2c.Init.OwnAddress1, (uint8_t*)i2cGyroDmaTx, TXBUFFERSIZE)!= HAL_OK);
}

bool I2cAccGyroRead()
{
	while(HAL_I2C_Master_Receive_DMA(&gyro_i2c, (uint16_t)gyro_i2c.Init.OwnAddress1, (uint8_t *)i2cGyroDmaRx, RXBUFFERSIZE) != HAL_OK);
}

bool I2cAccGyroState()
{
	while (HAL_I2C_GetState(&gyro_i2c) != HAL_I2C_STATE_READY);
}
