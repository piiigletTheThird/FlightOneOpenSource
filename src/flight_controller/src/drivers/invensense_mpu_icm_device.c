#include <stdbool.h>

#include "includes.h"

// Product ID Description for MPU6000
// Product Name Product Revision
#define MPU6000ES_REV_C4    0x14
#define MPU6000ES_REV_C5    0x15
#define MPU6000ES_REV_D6    0x16
#define MPU6000ES_REV_D7    0x17
#define MPU6000ES_REV_D8    0x18
#define MPU6000_REV_C4      0x54
#define MPU6000_REV_C5      0x55
#define MPU6000_REV_D6      0x56
#define MPU6000_REV_D7      0x57
#define MPU6000_REV_D8      0x58
#define MPU6000_REV_D9      0x59
#define MPU6000_REV_D10     0x5A

typedef struct __attribute__((__packed__))
{
    uint8_t accAddress;  // needed to start rx/tx transfer when sending address
    uint8_t accelX_H;
    uint8_t accelX_L;
    uint8_t accelY_H;
    uint8_t accelY_L;
    uint8_t accelZ_H;
    uint8_t accelZ_L;
    uint8_t dummy;        // otherwise TEMP_H
    uint8_t gyroAddress;  // otherwise TEMP_L
    uint8_t gyroX_H;
    uint8_t gyroX_L;
    uint8_t gyroY_H;
    uint8_t gyroY_L;
    uint8_t gyroZ_H;
    uint8_t gyroZ_L;
} gyroFrame_t;

static gyroFrame_t gyroRxFrame;
static gyroFrame_t gyroTxFrame;
int32_t deviceWhoAmI = 0;


// TODO: check what the acc actually updates at when sample rate divider != 0
// we have assumed that sample divider rate effects the accelerometer update rate
// currently: L1 uses 1khz base rate for gyro and acc, gyro updates same as
//            acc update
//            everything else uses 8khz or 32khz gyro rate, so set acc to 4khz
//            acc update, get acc update every 2nd or 8th gyro update
static const gyro_device_config mpu6500GyroConfig[] =
{
    [LOOP_L1] = {1, INVENS_CONST_GYRO_DLPF_188, INVENS_CONST_GYRO_FCB_DISABLE, INVENS_CONST_ACC_DLPF_460, INVENS_CONST_ACC_FCB_DISABLE, 1},
    [LOOP_M1] = {8, INVENS_CONST_GYRO_DLPF_256, INVENS_CONST_GYRO_FCB_DISABLE, 0, INVENS_CONST_ACC_FCB_ENABLE, 2},
    [LOOP_M2] = {4, INVENS_CONST_GYRO_DLPF_256, INVENS_CONST_GYRO_FCB_DISABLE, 0, INVENS_CONST_ACC_FCB_ENABLE, 2},
    [LOOP_M4] = {2, INVENS_CONST_GYRO_DLPF_256, INVENS_CONST_GYRO_FCB_DISABLE, 0, INVENS_CONST_ACC_FCB_ENABLE, 2},
    [LOOP_M8] = {1, INVENS_CONST_GYRO_DLPF_256, INVENS_CONST_GYRO_FCB_DISABLE, 0, INVENS_CONST_ACC_FCB_ENABLE, 2},
    [LOOP_H1] = {8, INVENS_CONST_GYRO_DLPF_3600, INVENS_CONST_GYRO_FCB_DISABLE, 0, INVENS_CONST_ACC_FCB_ENABLE, 2},
    [LOOP_H2] = {4, INVENS_CONST_GYRO_DLPF_3600, INVENS_CONST_GYRO_FCB_DISABLE, 0, INVENS_CONST_ACC_FCB_ENABLE, 2},
    [LOOP_H4] = {2, INVENS_CONST_GYRO_DLPF_3600, INVENS_CONST_GYRO_FCB_DISABLE, 0, INVENS_CONST_ACC_FCB_ENABLE, 2},
    [LOOP_H8] = {1, INVENS_CONST_GYRO_DLPF_3600, INVENS_CONST_GYRO_FCB_DISABLE, 0, INVENS_CONST_ACC_FCB_ENABLE, 2},
    [LOOP_H16] = {2, 0, INVENS_CONST_GYRO_FCB_32_3600, 0, INVENS_CONST_ACC_FCB_ENABLE, 8},
    [LOOP_H32] = {1, 0, INVENS_CONST_GYRO_FCB_32_3600, 0, INVENS_CONST_ACC_FCB_ENABLE, 8},
    [LOOP_UH1] = {32, 0, INVENS_CONST_GYRO_FCB_32_8800, 0, INVENS_CONST_ACC_FCB_ENABLE, 8},
    [LOOP_UH2] = {16, 0, INVENS_CONST_GYRO_FCB_32_8800, 0, INVENS_CONST_ACC_FCB_ENABLE, 8},
    [LOOP_UH4] = {8, 0, INVENS_CONST_GYRO_FCB_32_8800, 0, INVENS_CONST_ACC_FCB_ENABLE, 8},
    [LOOP_UH8] = {4, 0, INVENS_CONST_GYRO_FCB_32_8800, 0, INVENS_CONST_ACC_FCB_ENABLE, 8},
    [LOOP_UH16] = {2, 0, INVENS_CONST_GYRO_FCB_32_8800, 0, INVENS_CONST_ACC_FCB_ENABLE, 8},
    [LOOP_UH32] = {1, 0, INVENS_CONST_GYRO_FCB_32_8800, 0, INVENS_CONST_ACC_FCB_ENABLE, 8},
};

static const gyro_device_config mpu6000GyroConfig[] =
{
    [LOOP_L1] = {1, INVENS_CONST_GYRO_DLPF_188, 0, 0, 0, 1},
    [LOOP_M1] = {8, INVENS_CONST_GYRO_DLPF_256, 0, 0, 0, 1},
    [LOOP_M2] = {4, INVENS_CONST_GYRO_DLPF_256, 0, 0, 0, 2},
    [LOOP_M4] = {2, INVENS_CONST_GYRO_DLPF_256, 0, 0, 0, 4},
    [LOOP_M8] = {1, INVENS_CONST_GYRO_DLPF_256, 0, 0, 0, 8},
    [LOOP_H1] = {8, INVENS_CONST_GYRO_DLPF_3600, 0, 0, 0, 1},
    [LOOP_H2] = {4, INVENS_CONST_GYRO_DLPF_3600, 0, 0, 0, 2},
    [LOOP_H4] = {2, INVENS_CONST_GYRO_DLPF_3600, 0, 0, 0, 4},
    [LOOP_H8] = {1, INVENS_CONST_GYRO_DLPF_3600, 0, 0, 0, 8},
};

static bool accelUpdate = false;
static int32_t accelData[3];
//static int32_t accelCal[3];
static int32_t gyroData[3];
static int32_t gyroCal[3];
gyro_device_config gyroConfig;

int AccGyroDeviceInit(loopCtrl_e gyroLoop)
{

    // the mpu6000 caps out at 8khz
    if (deviceWhoAmI == MPU6000_WHO_AM_I)
    {
        if (gyroLoop > LOOP_H8)
        {
			gyroLoop = LOOP_H8;
		}
		gyroConfig = mpu6000GyroConfig[gyroLoop];
    }
    else
    {
	    // don't overflow array
        if (gyroLoop > LOOP_UH32)
        {
	        gyroLoop = LOOP_UH32;
	    }
	    gyroConfig = mpu6500GyroConfig[gyroLoop];
	}

    // reset gyro
	AccGyroWriteRegister(INVENS_RM_PWR_MGMT_1, INVENS_CONST_H_RESET);
	DelayMs(80);

    // set gyro clock to Z axis gyro
    AccGyroVerifyWriteRegister(INVENS_RM_PWR_MGMT_1, INVENS_CONST_CLK_Z);

    // clear low power states
    AccGyroWriteRegister(INVENS_RM_PWR_MGMT_2, 0);

    // disable I2C Interface, clear fifo, and reset sensor signal paths
    // TODO: shouldn't disable i2c on non-spi
    AccGyroWriteRegister(INVENS_RM_USER_CTRL, INVENS_CONST_I2C_IF_DIS | INVENS_CONST_FIFO_RESET | INVENS_CONST_SIG_COND_RESET);

    // set gyro sample divider rate
    AccGyroVerifyWriteRegister(INVENS_RM_SMPLRT_DIV, gyroConfig.rateDiv - 1);

    // gyro DLPF config
    AccGyroVerifyWriteRegister(INVENS_RM_CONFIG, gyroConfig.gyroDlpf);

    // set gyro full scale to +/- 2000 deg / sec
    AccGyroVerifyWriteRegister(INVENS_RM_GYRO_CONFIG, INVENS_CONST_GYRO_FSR_2000DPS << 3 | gyroConfig.gyroDlpfBypass);

    // set accel full scale to +/- 16g
    AccGyroVerifyWriteRegister(INVENS_RM_ACCEL_CONFIG, INVENS_CONST_ACC_FSR_16G << 3);

    if (deviceWhoAmI != MPU6000_WHO_AM_I) { //6000 is only gyro not to have this function
    	// set the accelerometer dlpf
    	AccGyroVerifyWriteRegister(INVENS_RM_ACCEL_CONFIG2, gyroConfig.accDlpfBypass << 3 | gyroConfig.accDlpf);
    	//this function varies between 6000 and 6500+ family
    	// set interrupt pin PP, 50uS pulse, status cleared on INT_STATUS read
    	AccGyroVerifyWriteRegister(INVENS_RM_INT_PIN_CFG, INVENS_CONST_INT_RD_CLEAR | INVENS_CONST_BYPASS_EN);
    } else {
        // set interrupt pin PP, 50uS pulse, status cleared on INT_STATUS read
    	AccGyroVerifyWriteRegister(INVENS_RM_INT_PIN_CFG, INVENS_CONST_INT_RD_CLEAR);
    }

    // enable data ready interrupt
    AccGyroVerifyWriteRegister(INVENS_RM_INT_ENABLE, INVENS_CONST_DATA_RDY_EN);

    return(1);
}

int AccGyroDeviceDetect(void)
{
    uint8_t attempt, data;

    // reset gyro
    AccGyroWriteRegister(INVENS_RM_PWR_MGMT_1, INVENS_CONST_H_RESET);
    DelayMs(80);
    AccGyroWriteRegister(INVENS_RM_PWR_MGMT_1, INVENS_CONST_H_RESET);

    // poll for the who am i register while device resets
    for (attempt = 0; attempt < 100; attempt++)
    {
        DelayMs(80);

        AccGyroReadData(INVENS_RM_WHO_AM_I, &data, 1);
        switch (data)
        {
        	case MPU6000_WHO_AM_I:
        		//deviceWhoAmI = data;
				//return data;
				//break;
            case MPU6555_WHO_AM_I:
			case MPU9250_WHO_AM_I:
            case ICM20689_WHO_AM_I:
            //case ICM20608G_WHO_AM_I:
#if defined(RVTF7)
            	deviceWhoAmI = data;
                return data;
				break;
#endif
				return -1;
				break;
			case MPU6500_WHO_AM_I:
#ifdef SPMFC400
            	deviceWhoAmI = data;
                return data;
				break;
#else
				return -2;
				break;
#endif
            case ICM20602_WHO_AM_I:
            case ICM20601_WHO_AM_I:
            case ICM20608G_WHO_AM_I:
#if defined(REVOLT) || defined(REVOLTF7) || defined(MICROVOLT)
            	deviceWhoAmI = data;
                return data;
                break;
#else
				return -3;
				break;
#endif
        }

    }

    if (attempt == 100)
    {
        return (0);
    }

    return (0);
    /* No need to do this
    // read the product id
    AccGyroReadData(INVENS_RM_PRODUCT_ID, &data, 1);

    // if who am i and id match, return true
    switch (data) {
        case MPU6000ES_REV_C4:
        case MPU6000ES_REV_C5:
        case MPU6000_REV_C4:
        case MPU6000_REV_C5:
        case MPU6000ES_REV_D6:
        case MPU6000ES_REV_D7:
        case MPU6000ES_REV_D8:
        case MPU6000_REV_D6:
        case MPU6000_REV_D7:
        case MPU6000_REV_D8:
        case MPU6000_REV_D9:
        case MPU6000_REV_D10:
            return true;
    }

    return 0;
    */

}

void accgyroDeviceReadAccGyro(void)
{
    // start read from accel, set high bit to read
    gyroTxFrame.accAddress = INVENS_RM_ACCEL_XOUT_H | 0x80;

    accelUpdate = true;
    AccGyroDMAReadWriteData(&gyroTxFrame.accAddress, &gyroRxFrame.accAddress, 15);
}

void accgyroDeviceReadGyro(void)
{
    // start read from gyro, set high bit to read
    gyroTxFrame.gyroAddress = INVENS_RM_GYRO_XOUT_H | 0x80;

    accelUpdate = false;
    AccGyroDMAReadWriteData(&gyroTxFrame.gyroAddress, &gyroRxFrame.gyroAddress, 7);
}

void accgyroDeviceReadComplete(void)
{
	gyroData[0] = (int32_t)(int16_t)((gyroRxFrame.gyroX_H << 8) | gyroRxFrame.gyroX_L);
	gyroData[1] = (int32_t)(int16_t)((gyroRxFrame.gyroY_H << 8) | gyroRxFrame.gyroY_L);
	gyroData[2] = (int32_t)(int16_t)((gyroRxFrame.gyroZ_H << 8) | gyroRxFrame.gyroZ_L);

    if (deviceWhoAmI == ICM20601_WHO_AM_I)
        InlineUpdateGyro( gyroData, 0.1219512195121951f ); // 1/8.2 is 0.1219512195121951
    else
        InlineUpdateGyro( gyroData, 0.060975609756098f ); // 1/16.4 is 0.060975609756098

        //32,767
    if (accelUpdate)
    {
		accelData[0] = (int32_t)(int16_t)((gyroRxFrame.accelX_H << 8) | gyroRxFrame.accelX_L);
		accelData[1] = (int32_t)(int16_t)((gyroRxFrame.accelY_H << 8) | gyroRxFrame.accelY_L);
		accelData[2] = (int32_t)(int16_t)((gyroRxFrame.accelZ_H << 8) | gyroRxFrame.accelZ_L);

        if (deviceWhoAmI == ICM20601_WHO_AM_I)
            InlineUpdateAcc( accelData, 0.0009765625f); //  1/1024 is 0.0009765625f
        else
            InlineUpdateAcc( accelData, 0.00048828125f); //  1/2048 is 0.00048828125f

	}
}

void accgyroDeviceCalibrate(int32_t *gyroData)
{
    uint8_t idx;

    for (idx = 0; idx < 3; idx++)
    {
        gyroCal[idx] = gyroData[idx];
    }
    /*//mpu 6500+ has gyro offset. Faster than using the old fashioned way above
    skipGyro = 1;

    AccGyroVerifyWriteRegister(INVENS_RM_XG_OFFSET_H, (uint8_t)(gyroData[0] >> 8));
    AccGyroVerifyWriteRegister(INVENS_RM_XG_OFFSET_L, (uint8_t)(gyroData[0] & 0xFF));
    AccGyroVerifyWriteRegister(INVENS_RM_YG_OFFSET_H, (uint8_t)(gyroData[1] >> 8));
    AccGyroVerifyWriteRegister(INVENS_RM_YG_OFFSET_L, (uint8_t)(gyroData[1] & 0xFF));
    AccGyroVerifyWriteRegister(INVENS_RM_ZG_OFFSET_H, (uint8_t)(gyroData[2] >> 8));
    AccGyroVerifyWriteRegister(INVENS_RM_ZG_OFFSET_L, (uint8_t)(gyroData[2] & 0xFF));

    skipGyro = 0;
     */
}

void accgyroDeviceApplyCalibration(int32_t *gyroData)
{
    uint8_t idx;

    for (idx = 0; idx < 3; idx++) {
        gyroData[idx] += gyroCal[idx];
    }
    /*//mpu 6500+ has gyro offset. Faster than using the old fashioned way above
    (void)gyroData;
    */
}
