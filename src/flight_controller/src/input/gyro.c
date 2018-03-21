//#include "includes.h"

#include <stdbool.h>
#include <stdint.h>

#include "includes.h"

void BuildRotationMatrix(int x, int y, int z);

float dpsGyroArray[3] = {0.0f, 0.0f, 0.0f};
float geeForceAccArray[3] = {0.0f, 0.0f, 0.0f};
static int32_t gyroSum[3] = {0, 0, 0};

volatile uint32_t gyroCalibrationCycles = GYRO_CALIBRATION_CYCLES * 1;

void ResetGyroCalibration(void)
{

	bzero(gyroSum,sizeof(gyroSum));
	gyroCalibrationCycles = GYRO_CALIBRATION_CYCLES * 1;

}

static void InlineUpdateCalibration(int32_t *rawGyro)
{

	int32_t gyroCalibration[3];
	int32_t axis;

    if (gyroCalibrationCycles <= GYRO_CALIBRATION_CYCLES) {
        for (axis = 2; axis >= 0; axis--) {
            gyroSum[axis] += rawGyro[axis];
        }
    }

    if (--gyroCalibrationCycles == 0) {
        for (axis = 2; axis >= 0; axis--) {
            // add what comes out to be 1/2 to improve rounding
            gyroCalibration[axis] = (int32_t)(-(gyroSum[axis] + (GYRO_CALIBRATION_CYCLES / 2)) / GYRO_CALIBRATION_CYCLES);
        }

        accgyroDeviceCalibrate(gyroCalibration);
    }
}

void InlineUpdateAcc(int32_t rawAcc[], float scale)
{

    InlineApplyGyroAccRotationAndScale(rawAcc, geeForceAccArray, scale);

    InlineUpdateAttitude(geeForceAccArray);
}

void InlineUpdateGyro(int32_t rawGyro[], float scale)
{

	float swapArray[3];

    if (gyroCalibrationCycles != 0)
    {
        InlineUpdateCalibration(rawGyro);
        return;
    }

    accgyroDeviceApplyCalibration(rawGyro);

    InlineApplyGyroAccRotationAndScale(rawGyro, swapArray, scale);

    dpsGyroArray[PITCH] = -swapArray[Y];
    dpsGyroArray[ROLL]  =  swapArray[X];
    dpsGyroArray[YAW]   = -swapArray[Z];
    //x, y, z: to yaw, roll, pitch. need to swap 0 and 2

    InlineFlightCode(dpsGyroArray);
}

static float rotationMatrix[3][3];
static int   matrixFormed = -1;

void BuildRotationMatrix(int x, int y, int z)
{
    float cosx, sinx, cosy, siny, cosz, sinz;
    float coszcosx, sinzcosx, coszsinx, sinzsinx;
    float xRadians, yRadians, zRadians;

    zRadians = ((float)z * PIf * I180);
    yRadians = ((float)y * PIf * I180);
    xRadians = ((float)x * PIf * I180);

    cosz = cosf(zRadians);
    sinz = sinf(zRadians);
    cosy = cosf(yRadians);
    siny = sinf(yRadians);
    cosx = cosf(xRadians);
    sinx = sinf(xRadians);

    coszcosx = cosz * cosx;
    sinzcosx = sinz * cosx;
    coszsinx = sinx * cosz;
    sinzsinx = sinx * sinz;

    rotationMatrix[0][X] = cosz * cosy;
    rotationMatrix[0][Y] = -cosy * sinz;
    rotationMatrix[0][Z] = siny;
    rotationMatrix[1][X] = sinzcosx + (coszsinx * siny);
    rotationMatrix[1][Y] = coszcosx - (sinzsinx * siny);
    rotationMatrix[1][Z] = -sinx * cosy;
    rotationMatrix[2][X] = (sinzsinx) - (coszcosx * siny);
    rotationMatrix[2][Y] = (coszsinx) + (sinzcosx * siny);
    rotationMatrix[2][Z] = cosy * cosx;
}

void InitOrientation(void)
{
	matrixFormed =-1;
}

void InlineApplyGyroAccRotationAndScale (int32_t rawData[], float dataArray[], float scale )
{

	uint32_t nonNinety;
	//from gyro, x, y, z (0, 1, 2)
	// x is roll, y is pitch, z is yaw

	int x = 0;
	int y = 0;
	int z = 0;
	x = -lrintf(mainConfig.gyroConfig.minorBoardRotation[X]);
	y = lrintf(mainConfig.gyroConfig.minorBoardRotation[Y]);
	z = lrintf(mainConfig.gyroConfig.minorBoardRotation[Z]);
	nonNinety = 0;
    switch (mainConfig.gyroConfig.gyroRotation)
    {

		case CW0:
	    	if (x || y || z)
	    	{
				if (matrixFormed != CW0) {
					matrixFormed = CW0;
					BuildRotationMatrix(x,y,z); //x, y, z, pitch, roll, yaw
				}
				nonNinety = 1;
	    	}
	    	else
	    	{
				dataArray[X] = ((float)rawData[X] * scale);
				dataArray[Y] = ((float)rawData[Y] * scale);
				dataArray[Z] = ((float)rawData[Z] * scale);
	    	}
			break;
        case CW90:
        	dataArray[X] = ((float)rawData[Y] * scale);
        	dataArray[Y] = ((float)rawData[X] * -scale);
        	dataArray[Z] = ((float)rawData[Z] * scale);
            break;
        case CW180:
        	dataArray[X] = ((float)rawData[X] * -scale);
        	dataArray[Y] = ((float)rawData[Y] * -scale);
        	dataArray[Z] = ((float)rawData[Z] * scale);
            break;
        case CW270:
        	dataArray[X] = ((float)rawData[Y] * -scale);
        	dataArray[Y] = ((float)rawData[X] * scale);
        	dataArray[Z] = ((float)rawData[Z] * scale);
            break;
        case CW0_INV:
        	dataArray[X] = ((float)rawData[X] * -scale);
        	dataArray[Y] = ((float)rawData[Y] * scale);
        	dataArray[Z] = ((float)rawData[Z] * -scale);
            break;
        case CW90_INV:
        	dataArray[X] = ((float)rawData[Y] * scale);
        	dataArray[Y] = ((float)rawData[X] * scale);
        	dataArray[Z] = ((float)rawData[Z] * -scale);
            break;
        case CW180_INV:
        	dataArray[X] = ((float)rawData[X] * scale);
        	dataArray[Y] = ((float)rawData[Y] * -scale);
        	dataArray[Z] = ((float)rawData[Z] * -scale);
            break;
        case CW270_INV:
        	dataArray[X] = ((float)rawData[Y] * -scale);
        	dataArray[Y] = ((float)rawData[X] * -scale);
        	dataArray[Z] = ((float)rawData[Z] * -scale);
            break;
    	case CW45:
    		if (matrixFormed != CW45) {
    			matrixFormed = CW45;
    			BuildRotationMatrix(0,0,45); //x, y, z, pitch, roll, yaw
    		}
    		nonNinety = 1;
       		break;
    	case CW135:
    		if (matrixFormed != CW135) {
				matrixFormed = CW135;
				BuildRotationMatrix(0,0,135); //x, y, z, pitch, roll, yaw
			}
    		nonNinety = 1;
    	    break;
    	case CW225:
    		if (matrixFormed != CW225) {
				matrixFormed = CW225;
				BuildRotationMatrix(0,0,225); //x, y, z, pitch, roll, yaw
			}
    		nonNinety = 1;
    	    break;
    	case CW315:
    		if (matrixFormed != CW315) {
				matrixFormed = CW315;
				BuildRotationMatrix(0,0,315); //x, y, z, pitch, roll, yaw
			}
    		nonNinety = 1;
    	    break;
    	case CW45_INV:
    		if (matrixFormed != CW45_INV) {
				matrixFormed = CW45_INV;
				BuildRotationMatrix(180,0,45); //x, y, z, pitch, roll, yaw
			}
    		nonNinety = 1;
    	    break;
    	case CW135_INV:
    		if (matrixFormed != CW135_INV) {
				matrixFormed = CW135_INV;
				BuildRotationMatrix(180,0,135); //x, y, z, pitch, roll, yaw
			}
    		nonNinety = 1;
    	    break;
    	case CW225_INV:
    		if (matrixFormed != CW225_INV) {
				matrixFormed = CW225_INV;
				BuildRotationMatrix(180,0,225); //x, y, z, pitch, roll, yaw
			}
    		nonNinety = 1;
    	    break;
    	case CW315_INV:
    		if (matrixFormed != CW315_INV) {
				matrixFormed = CW315_INV;
				BuildRotationMatrix(180,0,315); //x, y, z, pitch, roll, yaw
			}
    		nonNinety = 1;
    	    break;
    }

    if (nonNinety)
    {
		dataArray[X] = (rotationMatrix[0][X] * (float)rawData[X] * scale + rotationMatrix[1][X] * (float)rawData[Y] * scale + rotationMatrix[2][X] * (float)rawData[Z] * scale);
		dataArray[Y] = (rotationMatrix[0][Y] * (float)rawData[X] * scale + rotationMatrix[1][Y] * (float)rawData[Y] * scale + rotationMatrix[2][Y] * (float)rawData[Z] * scale);
		dataArray[Z] = (rotationMatrix[0][Z] * (float)rawData[X] * scale + rotationMatrix[1][Z] * (float)rawData[Y] * scale + rotationMatrix[2][Z] * (float)rawData[Z] * scale);
    }

}
