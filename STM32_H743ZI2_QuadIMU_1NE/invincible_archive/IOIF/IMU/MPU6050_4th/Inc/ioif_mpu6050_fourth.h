/**
 *-----------------------------------------------------------
 *                 6AXIS ACC & GYRO IMU DRIVER
 *-----------------------------------------------------------
 * @file ioif_mpu6050.h
 * @date Created on: Mar 28, 2023
 * @author INVINCIBLE 1NE
 * @brief Driver code for the MPU6050 accelerometer and gyroscope.
 *
 * This header file provides functionality to interface
 * with the MPU6050 accelerometer and gyroscope, including initialization,
 * data retrieval, and control register configurations.
 * 
 * Refer to the MPU6050 datasheet and related documents for more information.
 *
 * @ref MPU6050 Datasheet
 */

#ifndef MPU6050_FOURTH_INC_IOIF_MPU6050_FOURTH_H_
#define MPU6050_FOURTH_INC_IOIF_MPU6050_FOURTH_H_


/** @defgroup I2C I2C
  * @brief I2C MPU6050 module driver
  * @{
  */

#include <string.h>

#include "mpu6050.h"
#include "main.h"
#include "i2c.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define IOIF_MPU6050_FOURTH_BUFF_SIZE        32

#define IOIF_MPU6050_FOURTH_TRIALS           10
#define IOIF_MPU6050_FOURTH_STRAT_UP_DELAY   10
#define IOIF_MPU6050_FOURTH_TIMEOUT          1


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

/**
 * @brief Enumeration to describe the state of the 6-axis IMU.
 */
typedef enum _IOIF_6AxisState4_t {
    IOIF_IMU6AXIS_FOURTH_STATUS_OK = 0,
    IOIF_IMU6AXIS_FOURTH_STATUS_ERROR,
    IOIF_IMU6AXIS_FOURTH_STATUS_BUSY,
    IOIF_IMU6AXIS_FOURTH_STATUS_TIMEOUT,
} IOIF_6AxisState4_t;

/**
 * @brief Structure to hold the data from the 6-axis IMU.
 */
typedef struct _IOIF_6AxisData4_t {
    float accX;
    float accY;
    float accZ;

    float temp;

    float gyrX;
    float gyrY;
    float gyrZ;
} IOIF_6AxisData4_t;


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

IOIF_6AxisState4_t IOIF_Init6Axis4(I2C_HandleTypeDef* i2c);
IOIF_6AxisState4_t IOIF_Get6AxisValue4(IOIF_6AxisData4_t* imuData);


#endif /* MPU6050_FOURTH_INC_IOIF_MPU6050_FOURTH_H_ */
