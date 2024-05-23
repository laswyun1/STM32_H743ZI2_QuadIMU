
/**
 *-----------------------------------------------------------
 *    MPU6050 ACCELEROMETER AND GYROSCOPE DRIVER HEADER
 *-----------------------------------------------------------
 * @file MPU6050.h
 * @date Created on: Mar 28, 2023
 * @author INVINCIBLE 1NE
 * @brief Main header file for the MPU6050 IMU sensor.
 * 
 * This header file includes macros, type declarations, global variables,
 * and function prototypes necessary for interfacing and working with
 * the MPU6050 sensor.
 * 
 * Refer to the MPU6050 datasheet and related documents for more information.
 *
 * @ref MPU6050 Datasheet
 */

#ifndef MPU6050_INC_MPU6050_H_
#define MPU6050_INC_MPU6050_H_

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "mpu6050_regmap.h"

/**
 *-----------------------------------------------------------
 *              MACROS AND PREPROCESSOR DIRECTIVES
 *-----------------------------------------------------------
 * @brief Directives and macros for readability and efficiency.
 */

#define MPU6050_DATA_BUFF_SIZE    32
#define MPU6050_READ_DATA_SIZE    14

#define MPU6050_TOTAL_SENSORS     7

#define MPU6050_TRIALS            10
#define MPU6050_START_UP_DELAY    10
#define MPU6050_TIMEOUT           1

#define MPU6050_ACCEL_SCALE_FACTOR_2g			16384.0f	// 16384 LSB/g
#define MPU6050_ACCEL_SCALE_FACTOR_4g    		8192.0f		// 8192  LSB/g
#define MPU6050_ACCEL_SCALE_FACTOR_8g			4096.0f		// 4096	 LSB/g
#define MPU6050_ACCEL_SCALE_FACTOR_16g			2048.0f		// 2048  LSB/g

#define MPU6050_GYRO_SCALE_FACTOR_250dps		131.0f		// 131  LSB/dps
#define MPU6050_GYRO_SCALE_FACTOR_500dps     	65.5f		// 65.5 LSB/dps
#define MPU6050_GYRO_SCALE_FACTOR_1000dps		32.8f		// 32.8 LSB/dps
#define MPU6050_GYRO_SCALE_FACTOR_2000dps		16.4f		// 16.4 LSB/dps

#define MPU6050_TEMP_SCALE_FACTOR     			340.0f		// Temperature(Celsius) = (Temp register value) / 340.0 + 36.53
#define MPU6050_TEMP_OFFSET      				36.53f


/**
 *------------------------------------------------------------
 *                     TYPE DECLARATIONS
 *------------------------------------------------------------
 * @brief Custom data types and structures for the module.
 */

// Function pointer types for initialization, de-initialization, readiness, etc.
typedef uint8_t (*MPU6050_InitFunc)       (void);
typedef uint8_t (*MPU6050_DeInitFunc)     (void);
typedef uint8_t (*MPU6050_IsDevReadyFunc) (uint16_t);                                 // DevAddr
typedef uint8_t (*MPU6050_ReadRegFunc)    (uint16_t, uint16_t, uint8_t*, uint16_t);   // DevAddr, RegAddr, Data, DataSize
typedef uint8_t (*MPU6050_WriteRegFunc)   (uint16_t, uint16_t, uint8_t*, uint16_t);   // DevAddr, RegAddr, Data, DataSize
typedef uint8_t (*MPU6050_WaitFunc)       (uint32_t);			                      // ms to Wait

// Enumeration for MPU6050 status codes.
typedef enum _MPU6050State_t {
    MPU6050_STATUS_OK = 0,
    MPU6050_STATUS_ERROR,
    MPU6050_STATUS_BUSY,
    MPU6050_STATUS_TIMEOUT,
} MPU6050State_t;

// Structure for IO context
typedef struct _MPU6050IOctx_t {
    MPU6050_InitFunc       Init;
    MPU6050_DeInitFunc     DeInit;
    MPU6050_IsDevReadyFunc IsDevReady;
    MPU6050_ReadRegFunc    ReadReg;
    MPU6050_WriteRegFunc   WriteReg;
    MPU6050_WaitFunc       Wait;
} MPU6050IOctx_t;

// Structure for sensor values
typedef struct _MPU6050Data_t {
    float accX;
    float accY;
    float accZ;

    float temp;

    float gyrX;
    float gyrY;
    float gyrZ;
} MPU6050Data_t;

// Object handle structure
typedef struct _MPU6050Obj_t {
    uint16_t devAddr;               // Device address
    uint8_t isInit;                 // Initialization status
    uint8_t* dataBuff;              // Data Buffer
    MPU6050Data_t IMUData;          // Acc, Gyro, Temp Data
    MPU6050IOctx_t io;              // IO context
} MPU6050Obj_t;


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

// Set the I/O context for the MPU6050 object.
uint8_t MPU6050_Init(MPU6050Obj_t* mpu6050_obj);

// Initialize the MPU6050 object.
uint8_t MPU6050_SetIoctx(MPU6050Obj_t* mpu6050_obj, MPU6050IOctx_t* mpu6050_ioctx);

// Retrieve the acclerometer and gyroscope data values from the MPU6050 object.
uint8_t MPU6050_GetValue(MPU6050Obj_t* mpu6050_obj);


#endif /* MPU6050_INC_MPU6050_H_ */
