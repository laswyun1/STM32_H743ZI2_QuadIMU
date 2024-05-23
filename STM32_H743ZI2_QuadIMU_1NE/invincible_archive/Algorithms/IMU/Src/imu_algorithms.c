/*
 * imu_algorithms.c
 *
 *  Created on: Mar 29, 2024
 *      Author: INVINCIBLE
 */


#include "imu_algorithms.h"

/**
 *-----------------------------------------------------------
 *              TYPE DEFINITIONS AND ENUMERATIONS
 *-----------------------------------------------------------
 * @brief Enumerated types and structures central to this module.
 */




/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Variables accessible throughout the application.
 */




/**
 *------------------------------------------------------------
 *                      STATIC VARIABLES
 *------------------------------------------------------------
 * @brief Variables local to this module.
 */




/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */


/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

/* ------------------- SAVE PREVIOUS VALUE ------------------- */
void IMU_UpdateBuffer(IMU_SensorData_t* sensorData, IMU_AngleData_t* angleData, IMU_GaitData_t* gaitData, IMU_NormData_t* normData)
{
	sensorData->accX[1] = sensorData->accX[0];
	sensorData->accY[1] = sensorData->accY[0];
	sensorData->gyrZ[1] = sensorData->gyrZ[0];

	angleData->velRaw[1] = angleData->velRaw[0];
	angleData->degTvcf[1] = angleData->degTvcf[0];

	angleData->degLPF1st[1] = angleData->degLPF1st[0];
	angleData->degLPF2nd[1] = angleData->degLPF2nd[0];
	angleData->velLPF1st[1] = angleData->velLPF1st[0];
	angleData->velLPF2nd[1] = angleData->velLPF2nd[0];

	normData->degOriLPF[1] = normData->degOriLPF[0];
	normData->velOriLPF[1] = normData->velOriLPF[0];
	normData->ampDegLPF[1] = normData->ampDegLPF[0];
	normData->ampVelLPF[1] = normData->ampVelLPF[0];

	gaitData->gaitPhasePre = gaitData->gaitPhase;
}

/* ------------------- FUZZY LOGIC ------------------- */
float IMU_SquareRootSum(float x, float y)
{
	return sqrt(pow(x, 2) + pow(y, 2));
}

float IMU_AbsoluteValue(float value)
{
	return fabs(value);
}

float IMU_Derivative(float currentVal, float previousVal)
{
	return (currentVal - previousVal) / IMU_CONTROL_PERIOD;
}

void IMU_CalculateMu(float fuzzyVar, float fuzzyInput, float* mu)
{
	/* fuzzyVar is measurement value (initially set value in Set_Init_Parameters) */
	float xoi = 3 * fuzzyVar;									// Threshold Value (maybe middle value)
	float si = log(3) / fuzzyVar;								// Sensor Sensitivity (natural logarithm)
	float xbar = 0.5 * (1 + tanh(si * (fuzzyInput - xoi)));		// Fuzzy Logic Relational Expressions
	*mu *= (1 - xbar);											// Update mu for TVCF cutoff frequency(wc)
}

/*
*Generate Fuzzy Logic Input (Acc, Jerk, Angular Velocity, Angular Accerleration)
*/
void IMU_CalculateFuzzyInput(IMU_SensorData_t* sensorData, IMU_FuzzyData_t* fuzzyData)
{
	float jerkX = IMU_Derivative(sensorData->accX[0], sensorData->accX[1]);
	float jerkY = IMU_Derivative(sensorData->accY[0], sensorData->accY[1]);
	float wdotZ = IMU_Derivative(sensorData->gyrZ[0], sensorData->gyrZ[1]);

	// absolute ACC
	fuzzyData->fuzzyInput[0] = IMU_SquareRootSum(sensorData->accX[0], sensorData->accY[0]);
	// absolute Jerk
	fuzzyData->fuzzyInput[1] = IMU_SquareRootSum(jerkX, jerkY);
	// absolute Gyr(Angular Velocity)
	fuzzyData->fuzzyInput[2] = IMU_AbsoluteValue(sensorData->gyrZ[0]);
	// absolute Wdot(Angular Acceleration)
	fuzzyData->fuzzyInput[3] = IMU_AbsoluteValue(wdotZ);
}

/*
*Calculate Wc(CutOff Frequency)
*/
float IMU_CalculateFuzzyWc(IMU_FuzzyData_t* fuzzyData)
{
	fuzzyData->wc = 0;
	float mu = 1;

	/* Perform calculations for each fuzzy input (Acc, Jerk, Angular Velocity, Angular Accerleration) */
	for (int i = 0; i < 4; i++) {
		IMU_CalculateMu(fuzzyData->var[i], fuzzyData->fuzzyInput[i], &mu);
	}

	fuzzyData->wc = mu * (fuzzyData->wh) + (1 - mu) * (fuzzyData->wl);

	return fuzzyData->wc;
}


/* ------------------- FILTERS ------------------- */
/*
*Low pass filtering
*/
float IMU_LPF(float currAngle, float filteredAnglePrev, float cutoffFreq, float samplingPeriod)
{
	float filteredAngle = (cutoffFreq * samplingPeriod * currAngle + filteredAnglePrev) / (cutoffFreq * samplingPeriod + 1);
	return filteredAngle;
}

/*
*High pass filtering
*/
float IMU_HPF(float currAngle, float filteredAnglePrev, float cutoffFreq, float samplingPeriod)
{
	float filteredAngle = (currAngle * samplingPeriod + filteredAnglePrev) / (cutoffFreq * samplingPeriod + 1);
	return filteredAngle;
}


/*
*Function to apply a Time Variant Complementary Filter (TVCF) to an angle
*/
void IMU_RunTVCF(IMU_SensorData_t* sensorData, IMU_AngleData_t* angleData, float cutoffFreq, float samplingPeriod, IMU_AttachCase_t attachCase)
{
	/* Calculate the angle using accelerometer measurements and convert it to degrees */
    /* Thigh Angle Degree */
	float degAcc = 0.0;
	float degAccFilteredUpdate = 0.0;
	float degGyrFilteredUpdate = 0.0;
	float degTvcf = 0.0;

	degAcc = IMU_AttachCaseSetting(sensorData, attachCase);

	/* Apply Low Pass Filter (LPF) on accelerometer angle */
	degAccFilteredUpdate = IMU_LPF(degAcc, angleData->degAccFiltered, cutoffFreq, samplingPeriod);

	/* Apply High Pass Filter (HPF) on gyroscope measurements */
	degGyrFilteredUpdate = IMU_HPF(sensorData->gyrZ[0], angleData->degGyrFiltered, cutoffFreq, samplingPeriod);

	/* Combine filtered accelerometer and gyroscope measurements */
	degTvcf = degAccFilteredUpdate + degGyrFilteredUpdate;

	angleData->degAccFiltered 	= degAccFilteredUpdate;
	angleData->degGyrFiltered 	= degGyrFilteredUpdate;
	angleData->degTvcfFiltered 	= degTvcf;
}


/* ------------------- GAIT FUNCTION ------------------- */
/*
*Get Max or Min Value between two variables for Normalization
*/
float IMU_GetMaxValue(float x, float y)
{
	return (x > y) ? x : y;
}

float IMU_GetMinValue(float x, float y)
{
	return (x < y) ? x : y;
}

void IMU_1stHalfGaitCycle(IMU_NormData_t* normData, IMU_GaitData_t* gaitData)
{
	normData->degOri = normData->sumDeg / normData->sumIter;
	normData->velOri = normData->sumVel / normData->sumIter;

	gaitData->gaitPeriod = normData->sumIter;

	normData->ampDeg = (normData->degMax - normData->degMin) / 2;
	normData->ampVel = (normData->velMax - normData->velMin) / 2;
	normData->sumIter = 0;
	normData->sumDeg = 0;
	normData->sumVel = 0;
	normData->degMax = 0;
	normData->velMax = 0;
	normData->degMin = 0;
	normData->velMin = 0;
}

void IMU_2ndHalfGaitCycle(IMU_NormData_t* normData, IMU_AngleData_t* angleData)
{
	normData->sumIter++;
	normData->sumDeg += angleData->degLPF2nd[0];
	normData->sumVel += angleData->velLPF2nd[0];
	normData->degMax = IMU_GetMaxValue(angleData->degLPF2nd[0], normData->degMax);
	normData->degMin = IMU_GetMinValue(angleData->degLPF2nd[0], normData->degMin);
	normData->velMax = IMU_GetMaxValue(angleData->velLPF2nd[0], normData->velMax);
	normData->velMin = IMU_GetMinValue(angleData->velLPF2nd[0], normData->velMin);
}

/*
*Function to Prepare for Circular Normalization
*/
void IMU_Normalization(IMU_AngleData_t* angleData, IMU_NormData_t* normData, IMU_GaitData_t* gaitData)
{
	if (angleData->velLPF2nd[0] < 0 && angleData->velLPF2nd[1] > 0
		&& normData->sumIter > (gaitData->gaitPeriod)*0.5) {
		IMU_1stHalfGaitCycle(normData, gaitData);
	}
	else{
		IMU_2ndHalfGaitCycle(normData, angleData);
	}
}

/*
*Function to calculate the current phase of the gait (0~100%)
*/
float IMU_GetGaitPhase(IMU_NormData_t* normData, IMU_GaitData_t* gaitData)
{
	/* Calculate initial phase using atan function */
	float gaitPhase = atan((-1) * (normData->velNorm) / (normData->degNorm));

	/* Adjust phase based on the value of normalized degree */
    if (normData->degNorm < 0){
        gaitPhase += IMU_PI;
    }
    else if (normData->degNorm > 0 && normData->velNorm > 0){
        gaitPhase += 2 * IMU_PI;
    }

	/* Convert phase from radians to custom scale */
    gaitPhase = gaitPhase * 50.0f / IMU_PI;
//    gaitPhase -= 12.11;
    gaitPhase -= 5.6;

    /* Adjust phase if it falls outside the range 0-100 */
    if (gaitPhase < 0 && gaitPhase != -100){
        gaitPhase += 100;
    }

    /* Compare phase with the stored gait phase in gaitInfo */
    if (gaitPhase > 5 && gaitPhase < 95){
        gaitPhase = IMU_GetMaxValue(gaitPhase, gaitData->gaitPhase);
	}

    /* If the walking state is 0 or 1, set the gait phase to -100 */	// Added for fluctuation of gaitPhase
    if (gaitData->walkingState == IMU_STOP || gaitData->walkingState == IMU_WALKING_START){
    	gaitPhase = -100;
    }

	return gaitPhase;	// 0 ~ 100%
}

/*
*Function to find degAcc according to the "IMU Attach Case"
*/
float IMU_AttachCaseSetting(IMU_SensorData_t* sensorData, IMU_AttachCase_t attachCase)
{
	float estimatedAngle = 0.0;

	switch (attachCase)
	{
		case (IMU_LEFT_SAGITAL):
			estimatedAngle = atan2((sensorData->accX[0])*(-1), sensorData->accY[0]) * (180 / IMU_PI);		// arctan(-x/y) : Left Sagital case
			return estimatedAngle;

		case (IMU_RIGHT_SAGITAL):
			estimatedAngle = atan2(sensorData->accX[0], sensorData->accY[0]) * (180 / IMU_PI);				// arctan(x/y) : Right Sagital case
			return estimatedAngle;

		default:
			return estimatedAngle;
	}
}
