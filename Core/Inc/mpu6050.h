/*
 * mpu6050.h
 *
 *  Created on: May 28, 2025
 *      Author: Admin
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#define MPU_I2C_ADDR (0x68 << 1)

#define PWR_MGMT_1				0x6B

#define GYRO_CONFIG				0x1B
#define MPU6050_GYRO_FS_250DPS   (0 << 3)
#define MPU6050_GYRO_FS_500DPS   (1 << 3)
#define MPU6050_GYRO_FS_1000DPS  (2 << 3)
#define MPU6050_GYRO_FS_2000DPS  (3 << 3)


#define ACCEL_CONFIG			0x1C
#define MPU6050_ACCEL_FS_2G  	(0 << 3)
#define MPU6050_ACCEL_FS_4G   	(1 << 3)
#define MPU6050_ACCEL_FS_8G 	(2 << 3)
#define MPU6050_ACCEL_FS_16G	(3 << 3)

#define ACCEL_XOUT_H	0x3B
#define ACCEL_XOUT_L	0x3C
#define ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define ACCEL_ZOUT_H	0x3F
#define ACCEL_ZOUT_L	0x40

#define TEMP_OUT_H 		0x41
#define TEMP_OUT_L		0x42

#define GYRO_XOUT_H		0x43
#define GYRO_XOUT_L		0x44
#define GYRO_YOUT_H		0x45
#define GYRO_YOUT_L		0x46
#define GYRO_ZOUT_H		0x47
#define GYRO_ZOUT_L		0x48

typedef struct{
	I2C_HandleTypeDef *i2cHandle;
	float acc[3];
	float gyro[3];
	float temp;
	uint8_t dma_buffer[14];
	volatile bool data_ready_flag;

} MPU6050;

void MPU6050_Init(MPU6050 *dev, I2C_HandleTypeDef *i2cHandle);
void MPU6050_Start_DMA(MPU6050 *dev);
void MPU6050_Process_DMA(MPU6050 *dev);
HAL_StatusTypeDef MPU6050_ReadAcc(MPU6050 *dev);
HAL_StatusTypeDef MPU6050_ReadRegister(MPU6050 *dev, uint8_t reg, uint8_t* data);
HAL_StatusTypeDef MPU6050_ReadRegisters(MPU6050 *dev, uint8_t reg, uint8_t* data, uint8_t len);
HAL_StatusTypeDef MPU6050_WriteRegister(MPU6050 *dev, uint8_t reg, uint8_t data_value);
#endif /* INC_MPU6050_H_ */
