/**
 *-----------------------------------------------------------
 *       MPU6050 ACCELEROMETER AND GYROSCOPE DRIVER
 *-----------------------------------------------------------
 * @file mpu6050.c
 * @date Created on: Mar 28, 2023
 * @author INVINCIBLE 1NE
 * @brief Driver code for the MPU6050 accelerometer and gyroscope.
 *
 * This source file provides functionality to interface
 * with the MPU6050 accelerometer and gyroscope, including initialization,
 * data retrieval, and control register configurations.
 * 
 * Refer to the MPU6050 datasheet and related documents for more information.
 *
 * @ref MPU6050 Datasheet
 */

#include "mpu6050.h"

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

// static uint8_t ReadReg(MPU6050Obj_t* mpu6050Obj, uint16_t reg, uint8_t* pData, uint32_t size);
static uint8_t WriteReg(MPU6050Obj_t* mpu6050Obj, uint16_t reg, uint8_t* pData, uint32_t size);
static uint8_t SetReg(MPU6050Obj_t* mpu6050Obj, uint16_t reg, uint8_t* pData, uint32_t size);

static void CalValues(MPU6050Obj_t* mpu6050Obj);


/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

/**
 * @brief Set the I/O context for the MPU6050 object.
 * @param mpu6050Obj Pointer to the MPU6050 object.
 * @param mpu6050IOctx Pointer to the I/O context structure.
 * @return MPU6050 status code.
 */
uint8_t MPU6050_SetIoctx(MPU6050Obj_t* mpu6050Obj, MPU6050IOctx_t* mpu6050IOctx)
{
    if (!mpu6050Obj || !mpu6050IOctx->IsDevReady || !mpu6050IOctx->ReadReg || !mpu6050IOctx->WriteReg) {
        return MPU6050_STATUS_ERROR; // Return error if any pointer is NULL or essential functions are missing
    }

    // Assign IO context functions
    mpu6050Obj->io.Init       = mpu6050IOctx->Init;
    mpu6050Obj->io.DeInit     = mpu6050IOctx->DeInit;
    mpu6050Obj->io.IsDevReady = mpu6050IOctx->IsDevReady;
    mpu6050Obj->io.ReadReg    = mpu6050IOctx->ReadReg;
    mpu6050Obj->io.WriteReg   = mpu6050IOctx->WriteReg;
    mpu6050Obj->io.Wait       = mpu6050IOctx->Wait;

    return MPU6050_STATUS_OK;
}

/**
 * @brief Initialize the MPU6050 object by setting control registers.
 * @param mpu6050Obj Pointer to the MPU6050 object.
 * @return MPU6050 status code.
 */
uint8_t MPU6050_Init(MPU6050Obj_t* mpu6050Obj)
{
    if (!mpu6050Obj) {
        return MPU6050_STATUS_ERROR; // Return error if any pointer is NULL or essential functions are missing
    }

    uint8_t status = MPU6050_STATUS_ERROR;
    mpu6050Obj->devAddr = MPU6050_DEV_ADDR;

    // Check device readiness
    if (mpu6050Obj->io.IsDevReady(mpu6050Obj->devAddr) == 0){
        status = MPU6050_STATUS_OK;
    }

    // Configure control registers if the device is ready
    if (status == MPU6050_STATUS_OK){
        uint8_t conf_1 = MPU6050_PWR_MGMT_1_DATA;
        status = SetReg(mpu6050Obj, MPU6050_PWR_MGMT_1, &conf_1, MPU6050_CONTROL_SIZE);
        uint8_t conf_2 = MPU6050_PWR_MGMT_2_DATA;
        status = SetReg(mpu6050Obj, MPU6050_PWR_MGMT_2, &conf_2, MPU6050_CONTROL_SIZE);
        uint8_t conf_3 = MPU6050_GYR_CONFIG_500dps;
        status = SetReg(mpu6050Obj, MPU6050_GYRO_CONFIG, &conf_3, MPU6050_CONTROL_SIZE);
        uint8_t conf_4 = MPU6050_ACC_CONFIG_4g;
        status = SetReg(mpu6050Obj, MPU6050_ACCEL_CONFIG, &conf_4, MPU6050_CONTROL_SIZE);
    }

    return status;
}

/**
 * @brief Retrieves the raw data from the MPU6050 sensor and converts it into scaled acceleration and gyroscope values.
 *
 * This function reads the raw data bytes from the MPU6050 sensor using the provided ReadReg function pointer.
 * The raw data is then passed to the CalValues function to be converted into scaled values for acceleration
 * and gyroscope, which are assigned to the corresponding members of the MPU6050 object.
 *
 * @param mpu6050Obj Pointer to the MPU6050 object containing the configuration, IO context, and data structure.
 * @return MPU6050_STATUS_OK if the operation is successful; MPU6050_STATUS_ERROR otherwise.
 */
uint8_t MPU6050_GetValue(MPU6050Obj_t* mpu6050Obj)
{
    // Check for NULL pointer
    if (!mpu6050Obj) {
        return MPU6050_STATUS_ERROR; // Return error if any pointer is NULL or essential functions are missing
    }

    // Read the raw data from the sensor
    uint8_t status = mpu6050Obj->io.ReadReg(mpu6050Obj->devAddr, MPU6050_ACCEL_XOUT_H, mpu6050Obj->dataBuff, MPU6050_READ_DATA_SIZE);

    // If the read operation was successful, scale and assign the data
    if (status == MPU6050_STATUS_OK) {
        CalValues(mpu6050Obj);
    }

    return status;
}


/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

/**
 * @brief Read data from the specified register of the MPU6050 device.
 * @param mpu6050Obj Pointer to the MPU6050 object.
 * @param reg Register devAddress to read.
 * @param data Pointer to store the read data.
 * @param size Size of the data to be read.
 * @return MPU6050 status code.
 */
// static uint8_t ReadReg(MPU6050Obj_t* mpu6050Obj, uint16_t reg, uint8_t* pData, uint32_t size)
// {
//    return mpu6050Obj->io.ReadReg(mpu6050Obj->devAddr, reg, pData, size);
// }

/**
 * @brief Write data to the specified register of the MPU6050 device.
 * @param mpu6050Obj Pointer to the MPU6050 object.
 * @param reg Register devAddress to write.
 * @param data Pointer to the data to write.
 * @param size Size of the data to be written.
 * @return MPU6050 status code.
 */
static uint8_t WriteReg(MPU6050Obj_t* mpu6050Obj, uint16_t reg, uint8_t* pData, uint32_t size)
{
    return mpu6050Obj->io.WriteReg(mpu6050Obj->devAddr, reg, pData, size);
}

/**
 * @brief Set and verify the specified register of the MPU6050 device.
 * @param mpu6050Obj Pointer to the MPU6050 object.
 * @param reg Register devAddress to set.
 * @param data Pointer to the data to set.
 * @param size Size of the data to be set.
 * @return MPU6050 status code.
 */
static uint8_t SetReg(MPU6050Obj_t* mpu6050Obj, uint16_t reg, uint8_t* pData, uint32_t size)
{
	uint8_t status = MPU6050_STATUS_OK;

    status = WriteReg(mpu6050Obj, reg, pData, size);

    return status;
}

/**
 * @brief Scales the raw data from the MPU6050 sensor and assigns it to the appropriate structure members.
 * 
 * This function takes the raw data bytes from the MPU6050 sensor, converts them to 16-bit signed integers,
 * scales them according to the defined scale factors for acceleration and gyroscope, and assigns them
 * to the corresponding members of the MPU6050 object.
 * 
 * @param mpu6050Obj Pointer to the MPU6050 object containing the data structure to be updated.
 * @param dataBuff Pointer to the buffer containing the raw data bytes from the sensor.
 */
static void CalValues(MPU6050Obj_t* mpu6050Obj)
{
    // Array to hold the raw 16-bit values for each sensor reading
    int16_t rawValues[MPU6050_TOTAL_SENSORS] = {0};

    // Convert the raw bytes to 16-bit signed integers
    for (int i = 0; i < MPU6050_TOTAL_SENSORS; i++) {
        rawValues[i] = (int16_t)(mpu6050Obj->dataBuff[i * 2] << 8 | mpu6050Obj->dataBuff[i * 2 + 1]);
    }

    // Scale the raw values and assign them to the appropriate structure members
    mpu6050Obj->IMUData.accX = (float)(rawValues[0] / MPU6050_ACCEL_SCALE_FACTOR_4g);
    mpu6050Obj->IMUData.accY = (float)(rawValues[1] / MPU6050_ACCEL_SCALE_FACTOR_4g);
    mpu6050Obj->IMUData.accZ = (float)(rawValues[2] / MPU6050_ACCEL_SCALE_FACTOR_4g);
    mpu6050Obj->IMUData.temp = (float)(rawValues[3] / MPU6050_TEMP_SCALE_FACTOR) + MPU6050_TEMP_OFFSET;
    mpu6050Obj->IMUData.gyrX = (float)(rawValues[4] / MPU6050_GYRO_SCALE_FACTOR_500dps);
    mpu6050Obj->IMUData.gyrY = (float)(rawValues[5] / MPU6050_GYRO_SCALE_FACTOR_500dps);
    mpu6050Obj->IMUData.gyrZ = (float)(rawValues[6] / MPU6050_GYRO_SCALE_FACTOR_500dps);
}
