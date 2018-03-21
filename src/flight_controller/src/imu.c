#include "includes.h"

volatile int quadInverted = 0;
volatile quaternion_record commandQuat;
volatile quaternion_record accQuat;
volatile quaternion_record gyroQuat;
volatile vector_record gyroVector;
volatile vector_record accBodyVector;
volatile vector_record accWorldVector;
volatile vector_record errorWorldVector;
volatile vector_record errorBodyVector;
volatile vector_record verticalVector;
volatile quaternion_record accBodyQuat;
volatile quaternion_record accWorldQuat;
volatile quaternion_record rotationQuat;
volatile quaternion_record errorQuat;
volatile quaternion_record attitudeFrameQuat;
volatile quaternion_record inertialFrameQuat;

volatile float currentSpinRate = 0.0f;
volatile float rotationalMatrix[3][3];
volatile float requestedDegrees[3];

static void QuaternionZeroRotation(volatile quaternion_record *quaternion);
static void VectorZeroVector(volatile vector_record *vector);
static void QuaternionNormalize (volatile quaternion_record *out);
static void QuaternionMultiply(volatile quaternion_record *out, volatile quaternion_record *q1, volatile quaternion_record *q2);
static quaternion_record QuaternionFromEuler (float halfBankRads, float halfAttitudeRads, float halfHeadingRads);
//static void QuaternionToEuler(volatile quaternion_record *inQuat, volatile float rates[]);
static void UpdateRotationMatrix(void);
//static void QuaternionMultiplyDps(volatile quaternion_record *quatToUpdate, float gyroRollDiffRads, float gyroPitchDiffRads, float gyroYawDiffRads);
//static void QuaternionDifference(volatile quaternion_record *q1, volatile quaternion_record *q2);

quaternion_record MultiplyQuaternionByQuaternion(volatile quaternion_record q1, volatile quaternion_record q2);
void              GenerateQuaternionFromGyroVector(volatile quaternion_record *quatOut, vector_record vectorIn, float halfdT);
quaternion_record QuaternionConjugate (volatile quaternion_record *out);
void              VectorToQuat(quaternion_record *outQuad, float x, float y, float z);
quaternion_record MultiplyQuatAndVector(volatile quaternion_record quatIn, volatile vector_record vectorIn);
quaternion_record MultiplyQuatAndQuat(volatile quaternion_record quatIn1, volatile quaternion_record quatIn2);
void              EulerToVector(volatile vector_record *outVector, float x, float y, float z);
void              VectorAddVector(volatile vector_record *vectorOut, vector_record vectorIn, float trust);

//quats are defined like this: X is roll, Y is pitch, Z is yaw.
//Positive X is a roll to the right which matches our gyro
//Positive Y is a pitch down which is opposite of our gyro
//Positive Z is a yaw to the left which is opposite of our gyro
//we feed the quad negative yaw and pitch values to make it match our gyro

static void QuaternionZeroRotation(volatile quaternion_record *quaternion)
{
	quaternion->w = 1.0f;
	quaternion->x = 0.0f;
	quaternion->y = 0.0f;
	quaternion->z = 0.0f;
}

static void VectorZeroVector(volatile vector_record *vector)
{
	vector->x = 0.0f;
	vector->y = 0.0f;
	vector->z = 0.0f;
}

static void QuaternionNormalize (volatile quaternion_record *out)
{
	float norm;
	arm_sqrt_f32( (out->w * out->w + out->x * out->x + out->y * out->y + out->z * out->z), &norm);
	norm = 1.0f/norm;
	out->w *= norm;
	out->x *= norm;
	out->y *= norm;
	out->z *= norm;
}

inline quaternion_record QuaternionConjugate (volatile quaternion_record *out)
{
	quaternion_record outQuat;
	outQuat.w =  out->w;
	outQuat.x = -out->x;
	outQuat.y = -out->y;
	outQuat.z = -out->z;
	return(outQuat);
}

void ImuResetCommandQuat(void)
{
	commandQuat.w = attitudeFrameQuat.w;
	commandQuat.x = attitudeFrameQuat.x;
	commandQuat.y = attitudeFrameQuat.y;
	commandQuat.z = attitudeFrameQuat.z;
}

void ImuUpdateCommandQuat(float rollDps, float pitchDps, float yawDps, float halfdT)
{

	vector_record     commandVector;
	quaternion_record commandQuatChange;

	//if (ModeActive(M_GLUE))
	if (0)
	{
		EulerToVector( &commandVector, ( rollDps ), ( pitchDps ), ( -yawDps ) );
		GenerateQuaternionFromGyroVector(&commandQuatChange, commandVector, halfdT * 16.0f);
		commandQuat = MultiplyQuaternionByQuaternion(commandQuatChange, commandQuat);   //update attitudeFrameQuat quaternion

		QuaternionNormalize(&commandQuat);
	}


	/*
	//this is setting up the command quad for glue mode
	if (ModeActive(M_GLUE))
	{
		EulerToVector( &commandVector, ( rollDps ), ( pitchDps ), ( -yawDps ) );
		GenerateQuaternionFromGyroVector(&commandQuatChange, commandVector, halfdT * 16.0f);
		commandQuat = MultiplyQuaternionByQuaternion(commandQuatChange, commandQuat);   //update attitudeFrameQuat quaternion

		QuaternionNormalize(&commandQuat);
	}
	else //for the other self level modes we do this
	{
		//get satick position
		// we want this angle (trueRcCommandF[ROLL]  *  mainConfig.pidConfig[PITCH].sla )
		// we want this angle (trueRcCommandF[PITCH] *  mainConfig.pidConfig[PITCH].sla )
		// find yaw difference

		vector_record     commandedYaw;
		vector_record     commandedAttitude;
		EulerToVector(&commandedYaw, 0, 0, gyroYaw);                          //set gyro vector from gyro readings
		EulerToVector(&accBodyVector, accX, accY, accZ);                                   //set ACC vector from ACC readings
		accWorldVector = RotateVectorByQuaternionQV(attitudeFrameQuat, accBodyVector);     //rotate acc body frame to world frame using attitudeFrameQuat quatenrion
		VectorCrossProduct(&errorWorldVector, accWorldVector, verticalVector);             //find correction error from ACC readings
		errorBodyVector = RotateVectorByQuaternionVQ(errorWorldVector, attitudeFrameQuat); //rotate error world frame to world body using attitudeFrameQuat quatenrion
		VectorAddVector(&gyroVector, errorBodyVector, accTrust);                           //apply ACC correction to Gyro Vector with trust modifier
		GenerateQuaternionFromGyroVector(&gyroQuat, gyroVector, loopSpeed.halfGyrodT);     //generate gyro quaternion from modified gyro vector
		attitudeFrameQuat = MultiplyQuaternionByQuaternion(gyroQuat, attitudeFrameQuat);   //update attitudeFrameQuat quaternion

	}
*/

	//for attitude and horizon mode we do this:

	//put requested DPS into command quat.
//	QuaternionMultiplyDps(&commandQuat, rollDps, pitchDps, yawDps);
	//find difference between command quad and attitude quat and get euler numbers back that the PIDC can use
	//fills requestedDegrees
//	QuaternionDifference(&commandQuat, &attitudeFrameQuat);
}

/*
static void QuaternionDifference(volatile quaternion_record *q1, volatile quaternion_record *q2)
{
	volatile quaternion_record differenceQuaternion;
	volatile quaternion_record inverseQ1;

	//Conjugate q1 into tempQuaternion
	inverseQ1.w =  q1->w;
	inverseQ1.x = -q1->x;
	inverseQ1.y = -q1->y;
	inverseQ1.z = -q1->z;

	QuaternionMultiply(&inverseQ1, &inverseQ1, q2);
	QuaternionMultiply(&differenceQuaternion, q2, &inverseQ1);

	QuaternionToEuler(&differenceQuaternion, requestedDegrees);
}


static void QuaternionToEuler(volatile quaternion_record *inQuat, volatile float rates[])
{

	float test;
	float heading;
	float attitude;
	float bank;

	test = (inQuat->x * inQuat->y + inQuat->z * inQuat->w);

	if (test > 0.499)
	{ // singularity at north pole
		heading      = 2.0f * atan2f(inQuat->x,inQuat->w);
		attitude     = HALF_PI_F;
		bank         = 0.0f;
		rates[ROLL]  = bank;
		rates[PITCH] = attitude;
		rates[YAW]   = -heading;
		return;
	}
	if (test < -0.499)
	{ // singularity at south pole
		heading      = -2.0f * atan2f(inQuat->x,inQuat->w);
		attitude     = - HALF_PI_F;
		bank         = 0.0f;
		rates[ROLL]  = bank;
		rates[PITCH] = attitude;
		rates[YAW]   = -heading;
		return;
	}

    //sqx = inQuat->x * inQuat->x;
    //sqy = inQuat->y * inQuat->y;
    //sqz = inQuat->z * inQuat->z;

	rates[ROLL]  =  InlineRadiansToDegrees( atan2f(attitudeFrameQuat.y * attitudeFrameQuat.z + attitudeFrameQuat.w * attitudeFrameQuat.x, 0.5f - (attitudeFrameQuat.x * attitudeFrameQuat.x + attitudeFrameQuat.y * attitudeFrameQuat.y)) );  
	rates[PITCH] =  InlineRadiansToDegrees( asinf(2.0f * (attitudeFrameQuat.x * attitudeFrameQuat.z - attitudeFrameQuat.w * attitudeFrameQuat.y)) );
	rates[YAW]   = -InlineRadiansToDegrees( atan2f(attitudeFrameQuat.x * attitudeFrameQuat.y + attitudeFrameQuat.w * attitudeFrameQuat.z, 0.5f - (attitudeFrameQuat.y * attitudeFrameQuat.y + attitudeFrameQuat.z * attitudeFrameQuat.z)) );

    //heading  = atan2f(2.0f * inQuat->y * inQuat->w - 2.0f * inQuat->x * inQuat->z , 1.0f - 2.0f * sqy - 2.0f * sqz);
	//bank     = -asinf(2.0f * test);
	//attitude = -atan2f(2.0f * inQuat->x * inQuat->w - 2.0f * inQuat->y * inQuat->z , 1.0f - 2.0f * sqx - 2.0f * sqz);

	//rates[YAW]   = InlineRadiansToDegrees(heading);
	//rates[PITCH] = InlineRadiansToDegrees(attitude);
	//rates[ROLL]  = InlineRadiansToDegrees(bank);
}
*/

static void QuaternionMultiply (volatile quaternion_record *out, volatile quaternion_record *q1, volatile quaternion_record *q2)
{

	out->x =  q1->x * q2->w + q1->y * q2->z - q1->z * q2->y + q1->w * q2->x;
	out->y = -q1->x * q2->z + q1->y * q2->w + q1->z * q2->x + q1->w * q2->y;
	out->z =  q1->x * q2->y - q1->y * q2->x + q1->z * q2->w + q1->w * q2->z;
	out->w = -q1->x * q2->x - q1->y * q2->y - q1->z * q2->z + q1->w * q2->w;

}


/*
static float QuaternionDotProduct (volatile quaternion_record *a, volatile quaternion_record *b)
{
	return(a->w * b->w + a->x * b->x + a->y * b->y + a->z * b->z);
}
*/

static quaternion_record QuaternionFromEuler (float halfBankRads, float halfAttitudeRads, float halfHeadingRads)
{
	quaternion_record tempQuaternion;
	float c2, c1, c3;
	float s2, s1, s3;

	c1 = arm_cos_f32(halfHeadingRads);
	c2 = arm_cos_f32(halfAttitudeRads);
	c3 = arm_cos_f32(halfBankRads);
	s1 = arm_sin_f32(halfHeadingRads);
	s2 = arm_sin_f32(halfAttitudeRads);
	s3 = arm_sin_f32(halfBankRads);

	tempQuaternion.w = (c1 * c2 * c3 - s1 * s2 * s3);
	tempQuaternion.x = (s1 * s2 * c3 + c1 * c2 * s3);
	tempQuaternion.y = (s1 * c2 * c3 + c1 * s2 * s3);
	tempQuaternion.z = (c1 * s2 * c3 - s1 * c2 * s3);

	return(tempQuaternion);
}

/*
static void QuaternionToEuler(volatile quaternion_record *inQuat, float *roll, float *pitch, float *yaw)
{

	float test;
	float heading;
	float attitude;
	float bank;
	float sqx;
	float sqy;
	float sqz;

	test = (inQuat->x * inQuat->y + inQuat->z * inQuat->w);

	if (test > 0.499)
	{ // singularity at north pole
		heading  = 2.0f * atan2f(inQuat->x,inQuat->w);
		attitude = HALF_PI_F;
		bank     = 0.0f;
		return;
	}
	if (test < -0.499)
	{ // singularity at south pole
		heading  = -2.0f * atan2f(inQuat->x,inQuat->w);
		attitude = - HALF_PI_F;
		bank     = 0.0f;
		return;
	}

    sqx = inQuat->x * inQuat->x;
    sqy = inQuat->y * inQuat->y;
    sqz = inQuat->z * inQuat->z;

    heading  = atan2f(2.0f * inQuat->y * inQuat->w - 2.0f * inQuat->x * inQuat->z , 1.0f - 2.0f * sqy - 2.0f * sqz);
	attitude = asinf(2.0f * test);
	bank     = atan2f(2.0f * inQuat->x * inQuat->w - 2.0f * inQuat->y * inQuat->z , 1.0f - 2.0f * sqx - 2.0f * sqz);

	*yaw   = InlineRadiansToDegrees(heading);
	*pitch = InlineRadiansToDegrees(attitude);
	*roll  = InlineRadiansToDegrees(bank);
}
*/

static void UpdateRotationMatrix(void)
{
    float qxqx = (attitudeFrameQuat.x * attitudeFrameQuat.x);
    float qyqy = (attitudeFrameQuat.y * attitudeFrameQuat.y);
    float qzqz = (attitudeFrameQuat.z * attitudeFrameQuat.z);

    float qwqx = (attitudeFrameQuat.w * attitudeFrameQuat.x);
    float qwqy = (attitudeFrameQuat.w * attitudeFrameQuat.y);
    float qwqz = (attitudeFrameQuat.w * attitudeFrameQuat.z);
    float qxqy = (attitudeFrameQuat.x * attitudeFrameQuat.y);
    float qxqz = (attitudeFrameQuat.x * attitudeFrameQuat.z);
    float qyqz = (attitudeFrameQuat.y * attitudeFrameQuat.z);

    rotationalMatrix[0][0] = (1.0f - 2.0f * qyqy - 2.0f * qzqz);
    rotationalMatrix[0][1] = (2.0f * (qxqy - qwqz));
    rotationalMatrix[0][2] = (2.0f * (qxqz + qwqy));

    rotationalMatrix[1][0] = (2.0f * (qxqy + qwqz));
    rotationalMatrix[1][1] = (1.0f - 2.0f * qxqx - 2.0f * qzqz);
    rotationalMatrix[1][2] = (2.0f * (qyqz - qwqx));

    rotationalMatrix[2][0] = (2.0f * (qxqz - qwqy));
    rotationalMatrix[2][1] = (2.0f * (qyqz + qwqx));
    rotationalMatrix[2][2] = (1.0f - 2.0f * qxqx - 2.0f * qyqy);
}

void InitImu(void)
{
	uint32_t x, y;

	for (x = 0; x < 3; x++)
	{
		for (y = 0; y < 3; y++)
		{
			rotationalMatrix[x][y] = 0.0f;
		}
	}

	VectorZeroVector(&gyroVector);
	VectorZeroVector(&accBodyVector);
	VectorZeroVector(&accWorldVector);
	VectorZeroVector(&errorWorldVector);
	VectorZeroVector(&errorBodyVector);

	//set vertical vector to normallized vertical values
	verticalVector.x = 0.0f;
	verticalVector.y = 0.0f;
	verticalVector.z = 1.0f;

	QuaternionZeroRotation(&gyroQuat);
	QuaternionZeroRotation(&attitudeFrameQuat);
	QuaternionZeroRotation(&inertialFrameQuat);
	QuaternionZeroRotation(&rotationQuat);
	QuaternionZeroRotation(&commandQuat);

	UpdateRotationMatrix();
	ImuResetCommandQuat();
	requestedDegrees[0] = 0.0f;
	requestedDegrees[1] = 0.0f;
	requestedDegrees[2] = 0.0f;
}

/*
static void QuaternionMultiplyDps(volatile quaternion_record *quatToUpdate, float gyroRollDiffDps, float gyroPitchDiffDps, float gyroYawDiffDps)
{
	quaternion_record tempQuat;

	tempQuat.w = quatToUpdate->w;
	tempQuat.x = quatToUpdate->x;
	tempQuat.y = quatToUpdate->y;
	tempQuat.z = quatToUpdate->z;

	//change DPS to Half Radians
	gyroRollDiffDps  = InlineDegreesToRadians( gyroRollDiffDps )  * loopSpeed.halfGyrodT;
	gyroPitchDiffDps = InlineDegreesToRadians( gyroPitchDiffDps ) * loopSpeed.halfGyrodT;
	gyroYawDiffDps   = InlineDegreesToRadians( gyroYawDiffDps )   * loopSpeed.halfGyrodT;

	//sanity check
	if (isnan(gyroRollDiffDps) || isnan(gyroPitchDiffDps) || isnan(gyroYawDiffDps))
	{
		return;
	}

	//multiply the quaternion
	quatToUpdate->w += (-tempQuat.x * gyroRollDiffDps  - tempQuat.y * gyroPitchDiffDps - tempQuat.z * gyroYawDiffDps);
	quatToUpdate->x += (tempQuat.w  * gyroRollDiffDps  + tempQuat.y * gyroYawDiffDps   - tempQuat.z * gyroPitchDiffDps);
	quatToUpdate->y += (tempQuat.w  * gyroPitchDiffDps - tempQuat.x * gyroYawDiffDps   + tempQuat.z * gyroRollDiffDps);
	quatToUpdate->z += (tempQuat.w  * gyroYawDiffDps   + tempQuat.x * gyroPitchDiffDps - tempQuat.y * gyroRollDiffDps);

	QuaternionNormalize(quatToUpdate);
}
*/

void UpdateAttitudeFrameQuat(float gyroRollDiffRads, float gyroPitchDiffRads, float gyroYawDiffRads)
{
	quaternion_record tempQuat;

	tempQuat.w = attitudeFrameQuat.w;
	tempQuat.x = attitudeFrameQuat.x;
	tempQuat.y = attitudeFrameQuat.y;
	tempQuat.z = attitudeFrameQuat.z;

	gyroRollDiffRads  = InlineDegreesToRadians( gyroRollDiffRads )  * loopSpeed.halfGyrodT;
	gyroPitchDiffRads = InlineDegreesToRadians( gyroPitchDiffRads ) * loopSpeed.halfGyrodT;
	gyroYawDiffRads   = InlineDegreesToRadians( gyroYawDiffRads )   * loopSpeed.halfGyrodT;

	if (isnan(gyroRollDiffRads) || isnan(gyroPitchDiffRads) || isnan(gyroYawDiffRads))
	{
		return;
	}

	attitudeFrameQuat.w += (-tempQuat.x * gyroRollDiffRads  - tempQuat.y * gyroPitchDiffRads - tempQuat.z * gyroYawDiffRads);
	attitudeFrameQuat.x += (tempQuat.w  * gyroRollDiffRads  + tempQuat.y * gyroYawDiffRads   - tempQuat.z * gyroPitchDiffRads);
	attitudeFrameQuat.y += (tempQuat.w  * gyroPitchDiffRads - tempQuat.x * gyroYawDiffRads   + tempQuat.z * gyroRollDiffRads);
	attitudeFrameQuat.z += (tempQuat.w  * gyroYawDiffRads   + tempQuat.x * gyroPitchDiffRads - tempQuat.y * gyroRollDiffRads);

	QuaternionNormalize(&attitudeFrameQuat);
}

inline quaternion_record MultiplyQuatAndVector(volatile quaternion_record quatIn, volatile vector_record vectorIn)
{
	quaternion_record quatOut;
    quatOut.w = -quatIn.x * vectorIn.x - quatIn.y * vectorIn.y - quatIn.z * vectorIn.z;
    quatOut.x =  quatIn.w * vectorIn.x + quatIn.z * vectorIn.y - quatIn.y * vectorIn.z;  
    quatOut.y =  quatIn.w * vectorIn.y + quatIn.x * vectorIn.z - quatIn.z * vectorIn.x;
    quatOut.z =  quatIn.y * vectorIn.x - quatIn.x * vectorIn.y + quatIn.w * vectorIn.z;
    return(quatOut);
}

inline quaternion_record MultiplyQuatAndQuat(volatile quaternion_record quatIn1, volatile quaternion_record quatIn2)
{
	quaternion_record quatOut;
    quatOut.x = quatIn1.w * quatIn2.x + quatIn1.z * quatIn2.y - quatIn1.y * quatIn2.z + quatIn1.x * quatIn2.w;  
    quatOut.y = quatIn1.w * quatIn2.y + quatIn1.x * quatIn2.z + quatIn1.y * quatIn2.w - quatIn1.z * quatIn2.x;
    quatOut.z = quatIn1.y * quatIn2.x - quatIn1.x * quatIn2.y + quatIn1.w * quatIn2.z + quatIn1.z * quatIn2.w;
    quatOut.w = quatIn1.w * quatIn2.w - quatIn1.x * quatIn2.x - quatIn1.y * quatIn2.y - quatIn1.z * quatIn2.z;
    return(quatOut);
}

inline vector_record QuaternionToVector(volatile quaternion_record quatIn)
{
    vector_record vectorOut;
    vectorOut.x = quatIn.x;
    vectorOut.y = quatIn.y;
    vectorOut.z = quatIn.z;
    return(vectorOut);
}

inline vector_record RotateVectorByQuaternionQV(volatile quaternion_record quatIn, volatile vector_record vectorIn)
{
	quaternion_record tempQuat;

	tempQuat = MultiplyQuatAndQuat(
		MultiplyQuatAndVector( QuaternionConjugate(&quatIn), vectorIn ),
		quatIn
	);

	return( QuaternionToVector(tempQuat) );
}

inline vector_record RotateVectorByQuaternionVQ(volatile vector_record vectorIn, volatile quaternion_record quatIn)
{
	quaternion_record tempQuat;

	tempQuat = MultiplyQuatAndQuat(
		MultiplyQuatAndVector(quatIn, vectorIn),
		QuaternionConjugate(&quatIn)
	);

	return( QuaternionToVector(tempQuat) );
}

inline void VectorCrossProduct(volatile vector_record *vectorOut, volatile vector_record vectorIn1, volatile vector_record vectorIn2)
{ 
    vectorOut->x = vectorIn1.y * vectorIn2.z - vectorIn1.z * vectorIn2.y;
    vectorOut->y = vectorIn1.z * vectorIn2.x - vectorIn1.x * vectorIn2.z;
    vectorOut->z = vectorIn1.x * vectorIn2.y - vectorIn1.y * vectorIn2.x;
}

inline void EulerToVector(volatile vector_record *outVector, float x, float y, float z)
{
	outVector->x = x;
	outVector->y = y;
	outVector->z = z;
}

inline void VectorAddVector(volatile vector_record *vectorOut, vector_record vectorIn, float trust)
{
    vectorOut->x += vectorIn.x * trust;
    vectorOut->y += vectorIn.y * trust;
    vectorOut->z += vectorIn.z * trust;
}

inline void GenerateQuaternionFromGyroVector(volatile quaternion_record *quatOut, vector_record vectorIn, float halfdT)
{  
    
    quatOut->x  = vectorIn.x * halfdT;
    quatOut->y  = vectorIn.y * halfdT;
    quatOut->z  = vectorIn.z * halfdT;
    quatOut->w  = 1.0f - 0.5f * ( SQUARE(quatOut->x) + SQUARE(quatOut->y) + SQUARE(quatOut->z) );
}

inline quaternion_record MultiplyQuaternionByQuaternion(volatile quaternion_record q1, volatile quaternion_record q2)
{
	quaternion_record returnQuat;
    returnQuat.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
	returnQuat.x = q1.w * q2.x + q1.z * q2.y - q1.y * q2.z + q1.x * q2.w;  
    returnQuat.y = q1.w * q2.y + q1.x * q2.z + q1.y * q2.w - q1.z * q2.x;
    returnQuat.z = q1.y * q2.x - q1.x * q2.y + q1.w * q2.z + q1.z * q2.w;
	return(returnQuat);
}

void UpdateImu(float accX, float accY, float accZ, float gyroRoll, float gyroPitch, float gyroYaw)
{

	float accTrust = 0.01;
	float norm;
	float accToGyroError[3] = {0.0f, 0.0f, 0.0f};

	static float accTrustKiStorage[3] = {0.0f, 0.0f, 0.0f};
	//static uint32_t gyroToAccDivisorCounter = 0;

	if (boardArmed)
	{
		if(quopaState == QUOPA_ACTIVE)
		{
			accTrust  = 10.0f;
		}
		else
		{
			accTrust  = 1.0f;
		}
	}
	else
	{
		accTrust  = 20.0f;
	}

	//we use radians
	gyroPitch = -InlineDegreesToRadians( gyroPitch );
	gyroRoll  =  InlineDegreesToRadians( gyroRoll );
	gyroYaw   = -InlineDegreesToRadians( gyroYaw );

	if( (gyroRoll < -100) || (gyroRoll > 100) )
		quadInverted = 1;
	else
		quadInverted = 0;
	////normallize the ACC readings
	//arm_sqrt_f32( (accX * accX + accY * accY + accZ * accZ), &norm);
	//norm = 1.0f/norm;
	//accX *= norm;
	//accY *= norm;
	//accZ *= norm;

	//this gives us attitude
	//vector_record commandVector = verticalVector;


	//this block takes 6.23 us to run
	EulerToVector(&gyroVector, gyroRoll, gyroPitch, gyroYaw);                          //set gyro vector from gyro readings
	EulerToVector(&accBodyVector, accX, accY, accZ);                                   //set ACC vector from ACC readings
	accWorldVector = RotateVectorByQuaternionQV(attitudeFrameQuat, accBodyVector);     //rotate acc body frame to world frame using attitudeFrameQuat quatenrion
	VectorCrossProduct(&errorWorldVector, accWorldVector, verticalVector);             //find correction error from ACC readings
	errorBodyVector = RotateVectorByQuaternionVQ(errorWorldVector, attitudeFrameQuat); //rotate error world frame to world body using attitudeFrameQuat quatenrion
	VectorAddVector(&gyroVector, errorBodyVector, accTrust);                           //apply ACC correction to Gyro Vector with trust modifier
	GenerateQuaternionFromGyroVector(&gyroQuat, gyroVector, loopSpeed.halfGyrodT);     //generate gyro quaternion from modified gyro vector
	attitudeFrameQuat = MultiplyQuaternionByQuaternion(gyroQuat, attitudeFrameQuat);   //update attitudeFrameQuat quaternion

	//inlineDigitalHi(ports[ENUM_PORTB], GPIO_PIN_1);
	//it takes 1.41 us to multiply a quaternion

/*
	if ( 
			ModeSet(M_ATTITUDE) || 
			ModeSet(M_HORIZON)  || 
			ModeSet(M_QUOPA)    || 
			ModeSet(M_GLUE)     ||
			ModeSet(M_CATMODE)
		)
	{
*/
	//if (ModeActive(M_GLUE))
	if (0)
	{
		//the command quat is used to find the difference between the attitude quad and where we want the qwuad to be
		errorQuat = MultiplyQuaternionByQuaternion( commandQuat, QuaternionConjugate(&attitudeFrameQuat) );
		//inlineDigitalLo(ports[ENUM_PORTB], GPIO_PIN_1);

		//atan2f takes 4.18 us to run... atan2fast takes 2.75 us to run and you only lose 0.5% accuracy... add in arm_sin_f32 and time is at 1.99 us to run
		requestedDegrees[ROLL]  =  InlineRadiansToDegrees( Atan2fast(errorQuat.y * errorQuat.z + errorQuat.w * errorQuat.x, 0.5f - (errorQuat.x * errorQuat.x + errorQuat.y * errorQuat.y)) );  
		requestedDegrees[PITCH] =  InlineRadiansToDegrees( arm_sin_f32(2.0f * (errorQuat.x * errorQuat.z - errorQuat.w * errorQuat.y)) );
		requestedDegrees[YAW]   = -InlineRadiansToDegrees( Atan2fast(errorQuat.x * errorQuat.y + errorQuat.w * errorQuat.z, 0.5f - (errorQuat.y * errorQuat.y + errorQuat.z * errorQuat.z)) );
	}
	else
	{
		rollAttitude  =  InlineRadiansToDegrees( Atan2fast(attitudeFrameQuat.y * attitudeFrameQuat.z + attitudeFrameQuat.w * attitudeFrameQuat.x, 0.5f - (attitudeFrameQuat.x * attitudeFrameQuat.x + attitudeFrameQuat.y * attitudeFrameQuat.y)) );
		pitchAttitude =  InlineRadiansToDegrees( arm_sin_f32(2.0f * (attitudeFrameQuat.x * attitudeFrameQuat.z - attitudeFrameQuat.w * attitudeFrameQuat.y)) );
		yawAttitude   = -InlineRadiansToDegrees( Atan2fast(attitudeFrameQuat.x * attitudeFrameQuat.y + attitudeFrameQuat.w * attitudeFrameQuat.z, 0.5f - (attitudeFrameQuat.y * attitudeFrameQuat.y + attitudeFrameQuat.z * attitudeFrameQuat.z)) );
	}
	////error from the vertical
	//rollAttitude  =  InlineRadiansToDegrees( Atan2fast(attitudeFrameQuat.y * attitudeFrameQuat.z + attitudeFrameQuat.w * attitudeFrameQuat.x, 0.5f - (attitudeFrameQuat.x * attitudeFrameQuat.x + attitudeFrameQuat.y * attitudeFrameQuat.y)) );  ;
	//pitchAttitude =  InlineRadiansToDegrees( arm_sin_f32(2.0f * (attitudeFrameQuat.x * attitudeFrameQuat.z - attitudeFrameQuat.w * attitudeFrameQuat.y)) );
	//yawAttitude   = -InlineRadiansToDegrees( Atan2fast(attitudeFrameQuat.x * attitudeFrameQuat.y + attitudeFrameQuat.w * attitudeFrameQuat.z, 0.5f - (attitudeFrameQuat.y * attitudeFrameQuat.y + attitudeFrameQuat.z * attitudeFrameQuat.z)) );


	static uint32_t forcedUntilBelow = 0;
	static uint32_t forcedUntilAbove = 0;
	if ( (forcedUntilBelow) || (pitchAttitude > 88.0f) )
	{
		forcedUntilBelow = 1;
		if ( (pitchAttitude < 88.0f) && (pitchAttitude > 1.0f) )
		{
			forcedUntilBelow = 0;
		}
		else
		{
			pitchAttitude = 90.0f;
		}
	}

	if ( (forcedUntilAbove) || (pitchAttitude < -88.0f) )
	{
		forcedUntilAbove = 1;
		if ( (pitchAttitude > -88.0f) && (pitchAttitude < -1.0f) )
		{
			forcedUntilAbove = 0;
		}
		else
		{
			pitchAttitude = -90.0f;
		}
	}

	//requestedDegrees[ROLL] = 0.0f;
	//requestedDegrees[PITCH] = 0.0f;
	//requestedDegrees[YAW] = 0.0f;
	//how much error is there between the command vector and the attitudeFrameQuat
	//requestedDegrees[ROLL]  =  InlineRadiansToDegrees( atan2f(errorQuat.y * errorQuat.z + errorQuat.w * errorQuat.x, 0.5f - (errorQuat.x * errorQuat.x + errorQuat.y * errorQuat.y)) );  
	//requestedDegrees[PITCH] =  InlineRadiansToDegrees( asinf(2.0f * (errorQuat.x * errorQuat.z - errorQuat.w * errorQuat.y)) );
	//requestedDegrees[YAW]   = -InlineRadiansToDegrees( atan2f(errorQuat.x * errorQuat.y + errorQuat.w * errorQuat.z, 0.5f - (errorQuat.y * errorQuat.y + errorQuat.z * errorQuat.z)) );

//	static float lastPitch = 0.0f;

	//rates[ROLL]  =  InlineRadiansToDegrees( atan2f(attitudeFrameQuat.y * attitudeFrameQuat.z + attitudeFrameQuat.w * attitudeFrameQuat.x, 0.5f - (attitudeFrameQuat.x * attitudeFrameQuat.x + attitudeFrameQuat.y * attitudeFrameQuat.y)) );  
	//rates[PITCH] =  InlineRadiansToDegrees( asinf(2.0f * (attitudeFrameQuat.x * attitudeFrameQuat.z - attitudeFrameQuat.w * attitudeFrameQuat.y)) );
	//rates[YAW]   = -InlineRadiansToDegrees( atan2f(attitudeFrameQuat.x * attitudeFrameQuat.y + attitudeFrameQuat.w * attitudeFrameQuat.z, 0.5f - (attitudeFrameQuat.y * attitudeFrameQuat.y + attitudeFrameQuat.z * attitudeFrameQuat.z)) );

	//rollAttitude  =  InlineRadiansToDegrees( atan2f(attitudeFrameQuat.y * attitudeFrameQuat.z + attitudeFrameQuat.w * attitudeFrameQuat.x, 0.5f - (attitudeFrameQuat.x * attitudeFrameQuat.x + attitudeFrameQuat.y * attitudeFrameQuat.y)) );  
	//pitchAttitude =  InlineRadiansToDegrees( asinf(2.0f * (attitudeFrameQuat.x * attitudeFrameQuat.z - attitudeFrameQuat.w * attitudeFrameQuat.y)) );
	//yawAttitude   = -InlineRadiansToDegrees( atan2f(attitudeFrameQuat.x * attitudeFrameQuat.y + attitudeFrameQuat.w * attitudeFrameQuat.z, 0.5f - (attitudeFrameQuat.y * attitudeFrameQuat.y + attitudeFrameQuat.z * attitudeFrameQuat.z)) );

//	QuaternionToEuler(&attitudeFrameQuat, rates);
//	rollAttitude  =  rates[ROLL];
//	pitchAttitude =  rates[PITCH];
//	yawAttitude   =  rates[YAW];

	//QuaternionToEuler(&commandQuat, rates);
	//QuaternionToEuler(&attitudeFrameQuat, rates);
	//rollAttitude  = rates[ROLL];
	//pitchAttitude = -rates[PITCH];
	//yawAttitude   = -rates[YAW];
	return;










	//calculate current spin rate in DPS
	//arm_sqrt_f32( SQUARE(gyroRoll) + SQUARE(gyroPitch) + SQUARE(gyroYaw), &norm);
	//currentSpinRate = norm;

	//use ACC to fix Gyro drift here. Only needs to be done every eigth iteration at 32 KHz.
	if( !( (accX == 0.0f) && (accY == 0.0f) && (accZ == 0.0f) ) )
	{

		//gyroToAccDivisorCounter++;

		//if (gyroToAccDivisorCounter == loopSpeed.gyroAccDiv)
		//{

		//	gyroToAccDivisorCounter = 0;

			//normalize the acc readings
			arm_sqrt_f32( (accX * accX + accY * accY + accZ * accZ), &norm);
			norm = 1.0f/norm;
			accX *= norm;
			accY *= norm;
			accZ *= norm;

			//if (currentSpinRate < MAX_SPIN_RATE_RAD)
			//{
			//	accTrustKiStorage[ACCX] += accTrustKi * accToGyroError[ACCX] * loopSpeed.gyrodT;
			//	accTrustKiStorage[ACCY] += accTrustKi * accToGyroError[ACCY] * loopSpeed.gyrodT;
			//	accTrustKiStorage[ACCZ] += accTrustKi * accToGyroError[ACCZ] * loopSpeed.gyrodT;
			//}


			//accToGyroError[ACCX] += (accY * rotationalMatrix[2][2] - accZ * rotationalMatrix[2][1]);
			//accToGyroError[ACCY] += (accZ * rotationalMatrix[2][0] - accX * rotationalMatrix[2][2]);
			//accToGyroError[ACCZ] += (accX * rotationalMatrix[2][1] - accY * rotationalMatrix[2][0]);

			//accToGyroError[ACCX] += (accY - accZ);
			//accToGyroError[ACCY] += (accZ - accX);
			//accToGyroError[ACCZ] += (accX - accY);
			//accToGyroError[ACCX] =
			//accToGyroError[ACCY] =
			//accToGyroError[ACCZ] =
			//trust ACCs more when the quad is disamred.
			if (boardArmed)
			{
				accTrust  = 100.13000f;
			}
			else
			{
				accTrust  = 4000.3000f;
				accTrust  = 100.3000f;
			}

			//float pitchAcc = (atan2f( (float)filteredAccData[ACCX], (float)filteredAccData[ACCZ]) + PIf) * (180.0 * IPIf) - 180.0; //multiplying by the inverse of Pi is faster than dividing by Pi
			// Turning around the Y axis results in a vector on the X-axis
			//float rollAcc  = (atan2f((float)filteredAccData[ACCY], (float)filteredAccData[ACCZ]) + PIf) * (180.0 * IPIf) - 180.0;
			//float yawAcc   = (atan2f((float)filteredAccData[ACCX], (float)filteredAccData[ACCY]) + PIf) * (180.0 * IPIf) - 180.0;
			//accToGyroError[ACCX] = rollAttitude - rollAcc;
			//accToGyroError[ACCY] = pitchAttitude - pitchAcc;
			//accToGyroError[ACCZ] = yawAttitude - yawAcc;

			
			gyroRoll  += accTrust * accToGyroError[ACCX] + accTrustKiStorage[ACCX];
			gyroPitch += accTrust * accToGyroError[ACCY] + accTrustKiStorage[ACCY];
			gyroYaw   += accTrust * accToGyroError[ACCZ] + accTrustKiStorage[ACCZ];


		//}

	}

	
	//gyroQuat = QuaternionFromEuler (
	//	InlineDegreesToRadians( gyroRoll )  * loopSpeed.halfGyrodT,
	//	InlineDegreesToRadians( gyroPitch )  * loopSpeed.halfGyrodT,
	//	InlineDegreesToRadians( gyroYaw ) * loopSpeed.halfGyrodT
	//);

	//convert gyro readins into difference quaternion which is then multiplied by the attitudeFrame quaternion
	gyroQuat = QuaternionFromEuler (
		InlineDegreesToRadians( gyroPitch )  * loopSpeed.halfGyrodT,
		InlineDegreesToRadians( gyroRoll )  * loopSpeed.halfGyrodT,
		InlineDegreesToRadians( gyroYaw ) * loopSpeed.halfGyrodT
	);
	QuaternionNormalize(&gyroQuat);
	QuaternionMultiply(&attitudeFrameQuat, &attitudeFrameQuat, &gyroQuat);

	////only need to do this to get p,r,y, which shouldn't be needed for flight
	//volatile float rates[3];
	////QuaternionToEuler(&commandQuat, rates);
	//QuaternionToEuler(&attitudeFrameQuat, rates);
	//rollAttitude  = rates[ROLL];
	//pitchAttitude = -rates[PITCH];
	//yawAttitude   = -rates[YAW];

	//QuaternionMultiplyDps(&attitudeFrameQuat, gyroRoll, gyroPitch, gyroYaw);
	//UpdateRotationMatrix();

	//rollAttitude  = InlineRadiansToDegrees(atan2f(rotationalMatrix[2][1], rotationalMatrix[2][2]));
	//pitchAttitude = -InlineRadiansToDegrees(HALF_PI_F - acosf(-rotationalMatrix[2][0]));
	//yawAttitude   = -InlineRadiansToDegrees(atan2f(rotationalMatrix[1][0], rotationalMatrix[0][0]));

}
