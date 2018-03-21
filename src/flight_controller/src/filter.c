#include "includes.h"


/*
kalman_state kalmanState;

arm_matrix_instance_f32 xOld; //state matrix
arm_matrix_instance_f32 x; //state matrix
arm_matrix_instance_f32 z; //state matrix
arm_matrix_instance_f32 a; //a matrix
arm_matrix_instance_f32 aTranspose; //a transpose matrix
arm_matrix_instance_f32 u; //control variable matrix
arm_matrix_instance_f32 b; //b matrix
arm_matrix_instance_f32 w; //w matrix
arm_matrix_instance_f32 p; //Process variance/covariance matrix
arm_matrix_instance_f32 pTemp; //Process variance/covariance matrix, temporary
arm_matrix_instance_f32 k; //Kalman gain matrix
arm_matrix_instance_f32 h; //h matrix
arm_matrix_instance_f32 q; //h matrix
arm_matrix_instance_f32 r; //h matrix
arm_matrix_instance_f32 y; //h matrix
arm_matrix_instance_f32 aTx; //a times x matrix
arm_matrix_instance_f32 aTp; //a times x matrix
arm_matrix_instance_f32 bTu; //a times x matrix

#define VARIANCE_AMOUNT 10
#define P_M_X 0
#define P_M_Y 4
#define P_M_Z 8

uint32_t measurementPtr[3] = {VARIANCE_AMOUNT,VARIANCE_AMOUNT,VARIANCE_AMOUNT};
float    measurementData[3][VARIANCE_AMOUNT] = {0};
float    measurementVariance[3] = {0};


void KalmanTestUpdate(float measurement[])
{


	static float lastMeasurement[3] = {0.0f, 0.0f, 0.0f};
	static float lastPosition[3]    = {0.0f, 0.0f, 0.0f};
	static float position[3]        = {0.0f, 0.0f, 0.0f};

	int32_t axis;

	// (1) update prediction
	arm_mat_mult_f32(&a, &x, &aTx);
	arm_mat_mult_f32(&b, &u, &bTu);
	arm_mat_add_f32(&aTx, &bTu, &x);

	// (2) project covariance
	arm_mat_mult_f32(&a, &p, &aTp);
	arm_mat_mult_f32(&aTp, &aTranspose, &pTemp);
	arm_mat_add_f32(&pTemp, &q, &p);

	// (3) compute the kalman gain
	arm_mat_add_f32(&p, &r, &pTemp);
	arm_mat_inverse_f32(&pTemp, &bTu);
	arm_mat_mult_f32(&p, &bTu, &k);

	// (4) import new observation
	//Yk = CYk + Zk (0)

	//calculate position and acceleration
	for (axis = 2; axis >= 0; --axis)
	{
		//position
		position[axis]       = (lastPosition[axis] + (lastMeasurement[axis] * loopSpeed.gyrodT));
		//acceleration
		kalmanState.u[axis] = ((measurement[axis] - lastMeasurement[axis]) * loopSpeed.gyrodT);
	}
	kalmanState.y[0] = position[0];
	kalmanState.y[1] = position[1];
	kalmanState.y[2] = position[2];
	kalmanState.y[3] = measurement[0];
	kalmanState.y[4] = measurement[1];
	kalmanState.y[5] = measurement[2];

	// (5) compute current state
	arm_mat_sub_f32(&y, &x, &z);
	arm_mat_mult_f32(&k, &z, &y);
	arm_mat_add_f32(&xOld, &y, &x);
	memcpy(&kalmanState.xOld, &kalmanState.x, sizeof(kalmanState.x));
	
}

void InitKalmanTest(void)
{
	uint32_t srcRows;
	uint32_t srcColumns;

	float m_x[6] =
	{
		0.0f,
		0.0f,
		0.0f,
		0.0f,
		0.0f,
		0.0f,
	};
	memcpy(kalmanState.xOld, m_x, sizeof(m_x));
	memcpy(kalmanState.x, m_x, sizeof(m_x));
	memcpy(kalmanState.aTx_matrix, m_x, sizeof(m_x));
	memcpy(kalmanState.y, m_x, sizeof(m_x));
	memcpy(kalmanState.z, m_x, sizeof(m_x));
	srcRows    = 1;
	srcColumns = 6;
	arm_mat_init_f32(&xOld, srcRows, srcColumns, (float *)kalmanState.x);
	arm_mat_init_f32(&x, srcRows, srcColumns, (float *)kalmanState.x);
	arm_mat_init_f32(&aTx, srcRows, srcColumns, (float *)kalmanState.aTx_matrix);
	arm_mat_init_f32(&y, srcRows, srcColumns, (float *)kalmanState.y);
	arm_mat_init_f32(&z, srcRows, srcColumns, (float *)kalmanState.z);



	float m_u[3] =
	{
		0.0f,
		0.0f,
		0.0f,
	};
	memcpy(kalmanState.u, m_u, sizeof(m_u));
	srcRows    = 1;
	srcColumns = 3;
	arm_mat_init_f32(&u, srcRows, srcColumns, (float *)kalmanState.u);



	float m_a_matrix[36] =
	{
		1.0f, 0.0f, 0.0f, loopSpeed.gyrodT,             0.0f,             0.0f,
		0.0f, 1.0f, 0.0f,             0.0f, loopSpeed.gyrodT,             0.0f,
		0.0f, 0.0f, 1.0f,             0.0f,             0.0f, loopSpeed.gyrodT,
		0.0f, 0.0f, 0.0f,             1.0f,             0.0f,             0.0f,
		0.0f, 0.0f, 0.0f,             0.0f,             1.0f,             0.0f,
		0.0f, 0.0f, 0.0f,             0.0f,             0.0f,             1.0f,
	};
	memcpy(kalmanState.a_matrix, m_a_matrix, sizeof(m_a_matrix));
	memcpy(kalmanState.aTp_matrix, m_a_matrix, sizeof(m_a_matrix));
	memcpy(kalmanState.aTranpose_matrix, m_a_matrix, sizeof(m_a_matrix));
	srcRows    = 6;
	srcColumns = 6;
	arm_mat_init_f32(&a, srcRows, srcColumns, (float *)kalmanState.a_matrix);
	arm_mat_init_f32(&aTp, srcRows, srcColumns, (float *)kalmanState.aTp_matrix);
	arm_mat_init_f32(&aTranspose, srcRows, srcColumns, (float *)kalmanState.aTranpose_matrix);
	arm_mat_trans_f32(&a, &aTranspose);


	float m_b_matrix[18] =
	{
		loopSpeed.halfGyrodTSquared, 0.0f,                        0.0f,
		0.0f,                        loopSpeed.halfGyrodTSquared, 0.0f,
		0.0f,                        0.0f,                        loopSpeed.halfGyrodTSquared,
		loopSpeed.gyrodT,            0.0f,                        0.0f,
		0.0f,                        loopSpeed.gyrodT,            0.0f,
		0.0f,                        0.0f,                        loopSpeed.gyrodT,
	};
	memcpy(kalmanState.b_matrix, m_b_matrix, sizeof(m_b_matrix));
	memcpy(kalmanState.bTu_matrix, m_b_matrix, sizeof(m_b_matrix));
	srcRows    = 3;
	srcColumns = 6;
	arm_mat_init_f32(&b, srcRows, srcColumns, (float *)kalmanState.b_matrix);
	arm_mat_init_f32(&bTu, srcRows, srcColumns, (float *)kalmanState.bTu_matrix);


	//Pk
	//assuming error of 1.5 dps for gyro and 
	float m_p_matrix[36] =
	{
		0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
	};
	memcpy(kalmanState.p_matrix, m_p_matrix, sizeof(m_p_matrix));
	srcRows    = 6;
	srcColumns = 6;
	arm_mat_init_f32(&p, srcRows, srcColumns, (float *)kalmanState.p_matrix);
	arm_mat_init_f32(&pTemp, srcRows, srcColumns, (float *)kalmanState.p_matrix_temp);



	float m_q_matrix[36] =
	{
		0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
	};
	memcpy(kalmanState.q_matrix, m_q_matrix, sizeof(m_q_matrix));
	srcRows    = 6;
	srcColumns = 6;
	arm_mat_init_f32(&q, srcRows, srcColumns, (float *)kalmanState.q_matrix);



	float m_r_matrix[36] =
	{
		0.0000390625f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 0.0000390625f, 0.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 0.0000390625f, 0.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 0.0f,          2.3f, 0.0f, 0.0f,
		0.0f, 0.0f, 0.0f,          0.0f, 2.3f, 0.0f,
		0.0f, 0.0f, 0.0f,          0.0f, 0.0f, 2.3f,
	};
	memcpy(kalmanState.r_matrix, m_r_matrix, sizeof(m_r_matrix));
	srcRows    = 6;
	srcColumns = 6;
	arm_mat_init_f32(&r, srcRows, srcColumns, (float *)kalmanState.r_matrix);



	float m_k_matrix[36] =
	{
		0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
	};
	memcpy(kalmanState.k_matrix, m_k_matrix, sizeof(m_k_matrix));
	srcRows    = 6;
	srcColumns = 6;
	arm_mat_init_f32(&k, srcRows, srcColumns, (float *)kalmanState.k_matrix);

}
*/

void InitKdFilter(kd_filter *kdFilter)
{
	kdFilter->r = 1.0f / ( 2.0f * PIf * 50.0f );
	kdFilter->k = loopSpeed.gyrodT / (kdFilter->r + loopSpeed.gyrodT);
	kdFilter->x = 0.0f;
}

void KdFilterUpdate(kd_filter *kdFilter, float measurement)
{
	kdFilter->x = kdFilter->x + kdFilter->k * (measurement - kdFilter->x);
}

void OldInitPaf(paf_state *state, float q, float r, float p, float intial_value)
{
	state->q = q * 0.000001f;
	state->r = r * 0.001f;
	state->p = p * 0.001f;
	state->x = intial_value * 164.0f;
	state->output = intial_value;
}

void OldPafUpdate(paf_state *state, float measurement)
{
	//prediction update
	state->p = state->p + state->q;
	//measurement update
	state->k = state->p / (state->p + state->r);
	state->x = state->x + state->k * (measurement * 164.0f - state->x);
	state->p = (1.0f - state->k) * state->p;
	state->output = (state->x * 0.006097560975f);
}

void InitPaf(paf_state *state, float q, float r, float p, float intial_value)
{
	(void)(p);
	if (mainConfig.tuneProfile[activeProfile].filterConfig[0].filterType)
	{
		state->q *= 6.0f;
	}
	state->q = (float)q * 0.000001f;
	//state->r = r * 0.001;
	state->r = (float)r;
	state->p = 0.0f;
	state->x = (float)intial_value;
	state->lastX = (float)0.0f;
}

void PafUpdate(paf_state *state, float measurement)
{

	float acceleration;
	double accelerationD;

	switch (mainConfig.tuneProfile[activeProfile].filterConfig[0].filterType)
	{
		case 1:
			//float w; //process noise
			//float v; //measurment noise
			acceleration = (float)( ((float)state->x - (float)state->lastX) * (float)loopSpeed.InversedT );
			//project the state ahead using average acceleration

			state->x = state->x + (float)((float)acceleration * (float)loopSpeed.gyrodT);
			//state->x = state->x + (state->lastX - state->x) * loopSpeed.gyrodT;
			state->lastX = state->x;
			break;
		case 2:
			//float w; //process noise
			//float v; //measurment noise
			accelerationD = (double)( ((double)state->x - (double)state->lastX) * (double)loopSpeed.InversedT );
			//project the state ahead using average acceleration

			state->x = state->x + (float)((double)accelerationD * (double)loopSpeed.gyrodT);
			//state->x = state->x + (state->lastX - state->x) * loopSpeed.gyrodT;
			state->lastX = state->x;
			break;
		case 3:
			//only calculater acceleration if we are above 150 degrees per second rotation
			if(ABS(state->x) > 150.0f)
			{
				acceleration = (float)( ((float)state->x - (float)state->lastX) * (float)loopSpeed.InversedT );

				state->x = state->x + (float)((float)acceleration * (float)loopSpeed.gyrodT);

				state->lastX = state->x;
			}
			break;
	}

	//calculate acceleration change and average it over 4 cycles
	//state->accelAvgTotal -= state->accelAvgBuff[--state->accelAvgPtr];
	//state->accelAvgBuff[state->accelAvgPtr] = (measurement - state->x) * loopSpeed.gyrodT;
	//state->accelAvgTotal += state->accelAvgBuff[state->accelAvgPtr];
	//state->accelAvg = state->accelAvgTotal * 0.25f;

	//handle pointer
	//if (!state->accelAvgPtr)
	//	state->accelAvgPtr = 4;

	//prediction update
	state->p = state->p + state->q;
	//measurement update
	state->k = state->p / (state->p + state->r);
	state->x = state->x + (float)state->k * ((float)measurement - (float)state->x);
	state->p = (1.0f - state->k) * state->p;
}
/*
void InitKalman(kalman_state *kalmanState, float q, float r, float p, float intial_value)
{

	kalmanState->lastMeasurment = 0.0f;

	//temp 4x4 matrix
	bzero(kalmanState->temp4x4a, 4 * sizeof(float));
	arm_mat_init_f32(&kalmanState->armMatTemp4x4a, 2, 2, (float *)kalmanState->temp4x4a);
	bzero(kalmanState->temp4x4b, 4 * sizeof(float));
	arm_mat_init_f32(&kalmanState->armMatTemp4x4b, 2, 2, (float *)kalmanState->temp4x4b);
	bzero(kalmanState->temp2x1a, 2 * sizeof(float));
	arm_mat_init_f32(&kalmanState->armMatTemp2x1a, 2, 1, (float *)kalmanState->temp2x1a);
	bzero(kalmanState->temp2x1b, 2 * sizeof(float));
	arm_mat_init_f32(&kalmanState->armMatTemp2x1b, 2, 1, (float *)kalmanState->temp2x1b);

	//identity matrix
	kalmanState->i[0] = 1.0f;
	kalmanState->i[1] = 0.0f;
	kalmanState->i[2] = 0.0f;
	kalmanState->i[3] = 1.0f;
	arm_mat_init_f32(&kalmanState->armMatI, 2, 2, (float *)kalmanState->i);

	//k matrix
	bzero(kalmanState->k, 4 * sizeof(float));
	arm_mat_init_f32(&kalmanState->armMatK, 2, 2, (float *)kalmanState->k);

	//q process noise covariance
	bzero(kalmanState->q, 4 * sizeof(float));
	arm_mat_init_f32(&kalmanState->armMatQ, 2, 2, (float *)kalmanState->q);

	//x matrix
	bzero(kalmanState->x, 2 * sizeof(float));
	arm_mat_init_f32(&kalmanState->armMatX, 2, 1, (float *)kalmanState->x);
	bzero(kalmanState->lastX, 2 * sizeof(float));
	arm_mat_init_f32(&kalmanState->armMatLastX, 2, 1, (float *)kalmanState->lastX);

	//y matrix
	bzero(kalmanState->y, 2 * sizeof(float));
	arm_mat_init_f32(&kalmanState->armMatY, 2, 1, (float *)kalmanState->y);

	//r measurement noise covariance
	kalmanState->r[0] = (1.5f * 1.5f);
	kalmanState->r[1] = 0.0f;
	kalmanState->r[2] = 0.0f;
	kalmanState->r[3] = (( (1.5f * 1.5f) + (1.5f * 1.5f) ) * loopSpeed.gyrodT) * (( (1.5f * 1.5f) + (1.5f * 1.5f) ) * loopSpeed.gyrodT);
	arm_mat_init_f32(&kalmanState->armMatR, 2, 2, (float *)kalmanState->r);

	//p covarianve matrix
	bzero(kalmanState->p, 4 * sizeof(float));
	arm_mat_init_f32(&kalmanState->armMatP, 2, 2, (float *)kalmanState->p);
	bzero(kalmanState->lastP, 4 * sizeof(float));
	arm_mat_init_f32(&kalmanState->armMatLastP, 2, 2, (float *)kalmanState->lastP);

	//a matrix
	kalmanState->a[0] = loopSpeed.gyrodT;
	kalmanState->a[1] = loopSpeed.halfGyrodTSquared;
	kalmanState->a[2] = 0.0f;
	kalmanState->a[3] = loopSpeed.gyrodT;
	arm_mat_init_f32(&kalmanState->armMatA, 2, 2, (float *)kalmanState->a);

	//a transpose matrix
	kalmanState->aT[0] = loopSpeed.gyrodT;
	kalmanState->aT[1] = 0.0f;
	kalmanState->aT[2] = loopSpeed.halfGyrodTSquared;
	kalmanState->aT[3] = loopSpeed.gyrodT;
	arm_mat_init_f32(&kalmanState->armMatAT, 2, 2, (float *)kalmanState->aT);

}

void KalmanUpdate(kalman_state *state, float measurement)
{

		//predict covariance
		//Pk = APk-1A^t
		//state->p = state->p + state->q;
		arm_mat_mult_f32(&state->armMatA, &state->armMatP, &state->armMatTemp4x4a);
		arm_mat_mult_f32(&state->armMatTemp4x4a, &state->armMatAT, &state->armMatTemp4x4b);
		arm_mat_add_f32(&state->armMatTemp4x4b, &state->armMatQ, &state->armMatP);

		//compute kalman gain
		//state->k = state->p / (state->p + state->r);
		arm_mat_add_f32(&state->armMatP, &state->armMatR, &state->armMatTemp4x4a);
		arm_mat_inverse_f32(&state->armMatTemp4x4a, &state->armMatTemp4x4b);
		arm_mat_add_f32(&state->armMatP, &state->armMatTemp4x4b, &state->armMatK);
		memcpy(&state->lastP, &state->p, 2 * sizeof(float)); //todo in right spot?

		//Yk = CYkm + Zk //add Z?
		//put in velocity and acceleration
		state->y[0] = measurement;
		state->y[1] = (measurement - state->lastMeasurment) * loopSpeed.gyrodT; //use last state? average?
		state->lastMeasurment = measurement;
		//state->x = state->x + state->k * (measurement - state->x);
		arm_mat_sub_f32(&state->armMatY, &state->armMatLastX, &state->armMatTemp2x1a);
		arm_mat_mult_f32(&state->armMatK, &state->armMatTemp2x1a, &state->armMatTemp2x1b);
		arm_mat_add_f32(&state->armMatLastX, &state->armMatTemp2x1b, &state->armMatX);
		memcpy(&state->lastX, &state->x, 2 * sizeof(float));

		//update the covariance matrix
		//state->p = (1 - state->k) * state->p;
		arm_mat_sub_f32(&state->armMatI, &state->armMatK, &state->armMatTemp4x4a);
		arm_mat_mult_f32(&state->armMatTemp4x4a, &state->armMatLastP, &state->armMatP);
		state->output = state->x[0];

}

*/

void InitBiquad(float filterCutFreq, biquad_state *newState, float refreshRateSeconds, uint32_t filterType, biquad_state *oldState, float bandwidth)
{

	float samplingRate;
    float bigA, omega, sn, cs, alpha, beta;
    float a0, a1, a2, b0, b1, b2;

    float dbGain = 4.0;

    samplingRate = (1 / refreshRateSeconds);

	omega = 2 * (float)M_PI_FLOAT * (float) filterCutFreq / samplingRate;
	sn    = (float)sinf((float)omega);
	cs    = (float)cosf((float)omega);
	alpha = sn * (float)sinf( (float)((float)M_LN2_FLOAT / 2 * (float)bandwidth * (omega / sn)) );

	(void)(beta);
    //bigA  = powf(10, dbGain /40);
	//beta  = arm_sqrt_f32(bigA + bigA);

	switch (filterType)
	{
		case FILTER_TYPE_LOWPASS:
			b0 = (1 - cs) /2;
			b1 = 1 - cs;
			b2 = (1 - cs) /2;
			a0 = 1 + alpha;
			a1 = -2 * cs;
			a2 = 1 - alpha;
			break;
		case FILTER_TYPE_NOTCH:
			b0 = 1;
			b1 = -2 * cs;
			b2 = 1;
			a0 = 1 + alpha;
			a1 = -2 * cs;
			a2 = 1 - alpha;
			break;
		case FILTER_TYPE_PEEK:
		    bigA = powf(10, dbGain /40);
			b0   = 1 + (alpha * bigA);
			b1   = -2 * cs;
			b2   = 1 - (alpha * bigA);
			a0   = 1 + (alpha / bigA);
			a1   = -2 * cs;
			a2   = 1 - (alpha / bigA);
			break;
		 case FILTER_TYPE_HIGHPASS:
			b0 = (1 + cs) /2;
			b1 = -(1 + cs);
			b2 = (1 + cs) /2;
			a0 = 1 + alpha;
			a1 = -2 * cs;
			a2 = 1 - alpha;
			break;
	}

    // precompute the coefficients
    newState->a0 = b0 / a0;
    newState->a1 = b1 / a0;
    newState->a2 = b2 / a0;
    newState->a3 = a1 / a0;
    newState->a4 = a2 / a0;

    // zero initial samples
    //todo: make updateable on the fly
    newState->x1 =  oldState->x1;
    newState->x2 =  oldState->x2;
    newState->y1 =  oldState->y1;
    newState->y2 =  oldState->y1;

}

void LpfInit(lpf_state *filter, float frequencyCut, float refreshRateSeconds)
{
	filter->dT = refreshRateSeconds;
	filter->rC = 1.0f / ( 2.0f * M_PI * frequencyCut );
	filter->state = 0.0;
}

float LpfUpdate(float input, lpf_state *filter)
{

	filter->state = filter->state + filter->dT / (filter->rC + filter->dT) * (input - filter->state);

	return (filter->state);
}

float BiquadUpdate(float sample, biquad_state *state)
{
    float result;

    /* compute result */
    result = state->a0 * (float)sample + state->a1 * state->x1 + state->a2 * state->x2 -
            state->a3 * state->y1 - state->a4 * state->y2;

    /* shift x1 to x2, sample to x1 */
    state->x2 = state->x1;
    state->x1 = (float)sample;
    /* shift y1 to y2, result to y1 */
    state->y2 = state->y1;
    state->y1 = result;

    return (float)result;

}
