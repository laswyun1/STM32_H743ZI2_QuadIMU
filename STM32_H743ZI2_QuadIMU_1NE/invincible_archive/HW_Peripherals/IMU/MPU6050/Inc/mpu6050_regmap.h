/**
 *-----------------------------------------------------------
 * MPU6050 ACCELEROMETER AND GYROSCOPE DRIVER REGISTER MAP
 *-----------------------------------------------------------
 * @file MPU6050_REGMAP.h
 * @date Created on: Mar 28, 2024
 * @author INVINCIBLE 1NE
 * @brief Register map definitions for MPU6050 IMU sensor.
 * 
 * This header file contains definitions for device addresses, register addresses,
 * data sizes, and configurations related to the MPU6050 sensor.
 * 
 * Refer to the MPU6050 datasheet and related documents for more information.
 *
 * @ref MPU6050 Datasheet
 */

#ifndef MPU6050_INC_MPU6050_REGMAP_H_
#define MPU6050_INC_MPU6050_REGMAP_H_

/* Device Address */
// Device address configurations for MPU6050
#define MPU6050_DEV_ADDR						0xD0U

// Specific configurations used by a particular application or mode
#define MPU6050_PWR_MGMT_1_DATA				    0b00000000		// For 0x6B register
#define MPU6050_PWR_MGMT_2_DATA				    0b00000000		// For 0x6C register
#define MPU6050_GYR_CONFIG_250dps				0b00000000		// For 0x1B register
#define MPU6050_GYR_CONFIG_500dps				0b00001000		// For 0x1B register
#define MPU6050_GYR_CONFIG_1000dps				0b00010000		// For 0x1B register
#define MPU6050_GYR_CONFIG_2000dps				0b00011000		// For 0x1B register
#define MPU6050_ACC_CONFIG_2g				   	0b00000000		// For 0x1C register
#define MPU6050_ACC_CONFIG_4g				    0b00001000		// For 0x1C register
#define MPU6050_ACC_CONFIG_8g				    0b00010000		// For 0x1C register
#define MPU6050_ACC_CONFIG_16g				    0b00011000		// For 0x1C register
#define MPU6050_CONTROL_SIZE					1U

/* Register Address */
// Register address definitions for MPU6050
// Includes offset, configuration, status, data, and control registers
#define MPU6050_GYRO_CONFIG						0x1BU
#define MPU6050_ACCEL_CONFIG					0x1CU

#define MPU6050_ACCEL_XOUT_H					0x3BU	// Information MSB
#define MPU6050_ACCEL_XOUT_L					0x3CU	// Information LSB
#define MPU6050_ACCEL_YOUT_H					0x3DU	// Information MSB
#define MPU6050_ACCEL_YOUT_L					0x3EU	// Information LSB
#define MPU6050_ACCEL_ZOUT_H					0x3FU	// Information MSB
#define MPU6050_ACCEL_ZOUT_L					0x40U	// Information LSB

#define MPU6050_TEMP_OUT_H						0x41U	// Information MSB
#define MPU6050_TEMP_OUT_L						0x42U	// Information LSB

#define MPU6050_GYRO_XOUT_H						0x43U	// Information MSB
#define MPU6050_GYRO_XOUT_L						0x44U	// Information LSB
#define MPU6050_GYRO_YOUT_H						0x45U	// Information MSB
#define MPU6050_GYRO_YOUT_L						0x46U	// Information LSB
#define MPU6050_GYRO_ZOUT_H						0x47U	// Information MSB
#define MPU6050_GYRO_ZOUT_L						0x48U	// Information LSB

#define MPU6050_PWR_MGMT_1						0x6BU
#define MPU6050_PWR_MGMT_2						0x6CU

/* Data Size */
// Data size definitions for accelerometer, gyroscope, and configuration registers
#define MPU6050_ACC_DATA_SIZE					6U
#define MPU6050_GYR_DATA_SIZE					6U
#define MPU6050_ACC_CONFIG1_SIZE				1U
#define MPU6050_ACC_CONFIG2_SIZE				1U
#define MPU6050_GYR_CONFIG_SIZE					1U

/* ACCEL Configuration */
// Accelerometer configuration values for different ranges and scales
#define MPU6050_ACC_CONFIG_RANGE_2G          	0b00U	// Scale : 16384
#define MPU6050_ACC_CONFIG_RANGE_4G           	0b01U	// Scale : 8192 (General)
#define MPU6050_ACC_CONFIG_RANGE_8G           	0b10U	// Scale : 4096
#define MPU6050_ACC_CONFIG_RANGE_16G          	0b11U	// Scale : 2048

/* GYRO Configuration */
// Gyroscope configuration values for different ranges and scales
#define MPU6050_GYR_CONFIG_RANGE_250DPS        	0b00U	// Scale : 131
#define MPU6050_GYR_CONFIG_RANGE_500DPS        	0b01U	// Scale : 65.5	(General)
#define MPU6050_GYR_CONFIG_RANGE_1000DPS        0b10U	// Scale : 32.8
#define MPU6050_GYR_CONFIG_RANGE_2000DPS        0b11U	// Scale : 16.4

#endif /* MPU6050_INC_MPU6050_REGMAP_H_ */
