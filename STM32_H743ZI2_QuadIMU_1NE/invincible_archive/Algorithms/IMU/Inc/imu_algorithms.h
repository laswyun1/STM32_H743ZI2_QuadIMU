/*
 * imu_algorithms.h
 *
 *  Created on: Mar 29, 2024
 *      Author: INVINCIBLE
 */

#ifndef ALGORITHMS_IMU_INC_IMU_ALGORITHMS_H_
#define ALGORITHMS_IMU_INC_IMU_ALGORITHMS_H_

#include <stdint.h>
#include <math.h>
#include <string.h>

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define IMU_CONTROL_PERIOD		0.001
#define IMU_PI					3.141592653589793


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

/**
 * @brief Enumeration of walking state
 */
typedef enum _IMU_WalkingState_t {
	IMU_STOP,
	IMU_WALKING_START,
	IMU_WALKING_HALF_CYCLE,
	IMU_WALKING_ONE_CYCLE
} IMU_WalkingState_t;

/**
 * @brief Enumeration of IMU attachment cases
 */
typedef enum _IMU_AttachCase_t {
	IMU_LEFT_SAGITAL,
	IMU_RIGHT_SAGITAL
} IMU_AttachCase_t;


/**
 * @brief Structure to save (current & previous) IMU sensor data
 */
typedef struct _IMU_SensorData_t {
	float accX[2];			// [0] is current value, [1] is previous value (Accelerometer X axis)
	float accY[2];
	float gyrZ[2];			// [0] is current value, [1] is previous value (Gyroscope Z axis)
} IMU_SensorData_t;

/**
 * @brief Structure to hold (Angle & Angular velocity) data
 */
typedef struct _IMU_AngleData_t {
	float initAngle; 			// Initial angle(degree) will be set maybe 0

	float initAngleIMU;
	float initAngleABS;

	float degABS;
	float velABS;

	float degAcc;				// Angle using Accelerometer measurements
	float degGyr;				// Angle using Gyroscope measurements

	float degFinal;
	float velFinal;

	float degAccFiltered;		// Angle through LPF(Acc)
	float degGyrFiltered;		// Angle through HPF(Gyro)
	float degTvcfFiltered;		// Angle through TVCF

	float velRaw[2];			// [0] is current value, [1] is previous value (Angular velocity)
	float degTvcf[2];			// [0] is current value, [1] is previous value (Angel(degree) through TVCF)
	float degLPF1st[2];			// [0] is current value, [1] is previous value (Angle(degree) through 1st order LPF)
	float degLPF2nd[2];			// [0] is current value, [1] is previous value (Angle(degree) through 2nd order LPF)
	float velLPF1st[2];			// [0] is current value, [1] is previous value (Angular velocity through 1st order LPF)
	float velLPF2nd[2];			// [0] is current value, [1] is previous value (Angular velocity through 2nd order LPF)
} IMU_AngleData_t;

/**
 * @brief Structure to hold Fuzzy Logic parameters
 */
typedef struct _IMU_FuzzyData_t {
	float fuzzyInput[4];	// Acc, Jerk, Gyro, Wdot(measurement value)
	float wc;				// Cut off Frequency
	float wl;				// Low Frequency
	float wh;				// High Frequency
	float var[4];			// Acc, Jerk, Gyro, Wdot-variance(initially set velue)
} IMU_FuzzyData_t;

/**
 * @brief Structure to hold Threshold values(Start/Stop) of (angle & angular velocity)
 */
typedef struct _IMU_ThresData_t {
	float degThStart;			// Angle Threshold (Gait Phase Start)
	float velThStart;			// Angular Velocity Threshold (Gait Phase Stop)
	float degThStop;			// Angle Threshold (Gait Phase Start)
	float velThStop;			// Angular Velocity Threshold (Gait Phase Stop)
} IMU_ThresData_t;

/**
 * @brief Structure to execute the Normalization of gait phase graph
 */
typedef struct  _IMU_NormData_t {
	float degOri;			// Center point location of elliptical graph before normalization
	float velOri;			// Center point location of elliptical graph before normalization
	float degOriLPF[2];
	float velOriLPF[2];

	float sumDeg;			// Sum of angle for calculating deg_o
	float sumVel;			// Sum of angular velocity for calculating vel_o

	float degMax;			// Max of angle in elliptical plot
	float degMin;			// Min of angle in elliptical plot
	float velMax;			// Max of angular velocity in elliptical plot
	float velMin;			// Min of angular velocity in elliptical plot

	float degNorm;			// Current angle value on circle after normalization
	float velNorm;			// Current angular velocity value on circle after normalization

	float ampDeg;			// Amplitude of angle of an elliptical graph before normalization
	float ampVel;			// Amplitude of angular velocity of an elliptical graph before normalization
	float ampDegLPF[2];
	float ampVelLPF[2];

	uint16_t sumIter;		// Sum of number(gait phase 50%) for calculating deg_o, vel_o
} IMU_NormData_t;

/**
 * @brief Structure to save (gait period & gait phase)
 */
typedef struct _IMU_GaitData_t {
	float gaitPhase;			// Current Gait Phase 0 ~ 100%
	float gaitPhasePre;			// Previous Gait Phase
	uint16_t gaitPeriod;		// Gait Period (ms) < 2000ms
	uint8_t	walkingState;
} IMU_GaitData_t;


/**
 *------------------------------------------------------------
 *                      GLOBAL VARIABLES
 *------------------------------------------------------------
 * @brief Extern declarations for global variables.
 */


/**
 *------------------------------------------------------------
 *                     FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Function prototypes declaration for this module.
 */
void IMU_UpdateBuffer(IMU_SensorData_t* sensorData, IMU_AngleData_t* angleData, IMU_GaitData_t* gaitData, IMU_NormData_t* normData);
float IMU_SquareRootSum(float x, float y);
float IMU_AbsoluteValue(float value);
float IMU_Derivative(float currentVal, float previousVal);
void IMU_CalculateMu(float fuzzyVar, float fuzzyInput, float* mu);
void IMU_CalculateFuzzyInput(IMU_SensorData_t* sensorData, IMU_FuzzyData_t* fuzzyData);
float IMU_CalculateFuzzyWc(IMU_FuzzyData_t* fuzzyData);
float IMU_LPF(float currAngle, float filteredAnglePrev, float cutoffFreq, float samplingPeriod);
float IMU_HPF(float currAngle, float filteredAnglePrev, float cutoffFreq, float samplingPeriod);
void IMU_RunTVCF(IMU_SensorData_t* sensorData, IMU_AngleData_t* angleData, float cutoffFreq, float samplingPeriod, IMU_AttachCase_t attachCase);
float IMU_GetMaxValue(float x, float y);
float IMU_GetMinValue(float x, float y);
void IMU_1stHalfGaitCycle(IMU_NormData_t* normData, IMU_GaitData_t* gaitData);
void IMU_2ndHalfGaitCycle(IMU_NormData_t* normData, IMU_AngleData_t* angleData);
void IMU_Normalization(IMU_AngleData_t* angleData, IMU_NormData_t* normData, IMU_GaitData_t* gaitData);
float IMU_GetGaitPhase(IMU_NormData_t* normData, IMU_GaitData_t* gaitData);
float IMU_AttachCaseSetting(IMU_SensorData_t* sensorData, IMU_AttachCase_t attachCase);


#endif /* ALGORITHMS_IMU_INC_IMU_ALGORITHMS_H_ */
