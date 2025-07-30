/*
 * mpu6050.c
 *
 *  Created on: May 28, 2025
 *      Author: Admin
 */
#include "mpu6050.h"
void MPU6050_Init(MPU6050 *dev, I2C_HandleTypeDef *i2cHandle){
	dev->i2cHandle = i2cHandle;

	dev->acc[0] = 0.0f;
	dev->acc[1] = 0.0f;
	dev->acc[2] = 0.0f;

	//wake up the mpu
	MPU6050_WriteRegister(dev, PWR_MGMT_1, 0x00);
	//Set the mpu acc range
	MPU6050_WriteRegister(dev, ACCEL_CONFIG, MPU6050_ACCEL_FS_4G);
	//Set the mpu gyro range
	MPU6050_WriteRegister(dev, GYRO_CONFIG, MPU6050_GYRO_FS_500DPS);

}

void MPU6050_Start_DMA(MPU6050 *dev){
	HAL_I2C_Mem_Read_DMA(dev->i2cHandle, MPU_I2C_ADDR, ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, dev->dma_buffer, 14);
}

void MPU6050_Process_DMA(MPU6050 *dev) {
    int16_t raw_acc_x = (dev->dma_buffer[0] << 8) | dev->dma_buffer[1];
    int16_t raw_acc_y = (dev->dma_buffer[2] << 8) | dev->dma_buffer[3];
    int16_t raw_acc_z = (dev->dma_buffer[4] << 8) | dev->dma_buffer[5];

    int16_t raw_temp  = (dev->dma_buffer[6] << 8) | dev->dma_buffer[7];

    int16_t raw_gyro_x = (dev->dma_buffer[8] << 8) | dev->dma_buffer[9];
    int16_t raw_gyro_y = (dev->dma_buffer[10] << 8) | dev->dma_buffer[11];
    int16_t raw_gyro_z = (dev->dma_buffer[12] << 8) | dev->dma_buffer[13];

    dev->acc[0] = (float)raw_acc_x / 8192.0f;   // ±4g scale
    dev->acc[1] = (float)raw_acc_y / 8192.0f;
    dev->acc[2] = (float)raw_acc_z / 8192.0f;

    dev->temp = ((float)raw_temp) / 340.0f + 36.53f;

    dev->gyro[0] = (float)raw_gyro_x / 65.5f;   // ±500°/s
    dev->gyro[1] = (float)raw_gyro_y / 65.5f;
    dev->gyro[2] = (float)raw_gyro_z / 65.5f;

    dev->data_ready_flag = 0; // Reset after processing
}

HAL_StatusTypeDef MPU6050_ReadAcc(MPU6050 *dev){
    uint8_t raw_data[6]; // To store high and low bytes for X, Y, Z

    // Read 6 bytes starting from ACCEL_XOUT_H (0x3B)
    HAL_StatusTypeDef status = MPU6050_ReadRegisters(dev, ACCEL_XOUT_H, raw_data, 6);

    if (status == HAL_OK) {
        // Combine high and low bytes to get 16-bit signed integers
        int16_t acc_x = (int16_t)(raw_data[0] << 8 | raw_data[1]);
        int16_t acc_y = (int16_t)(raw_data[2] << 8 | raw_data[3]);
        int16_t acc_z = (int16_t)(raw_data[4] << 8 | raw_data[5]);

        float accel_scale_factor = 8192.0f;
        dev->acc[0] = (float)acc_x / accel_scale_factor;
        dev->acc[1] = (float)acc_y / accel_scale_factor;
        dev->acc[2] = (float)acc_z / accel_scale_factor;


    }

    return status;
}



HAL_StatusTypeDef MPU6050_ReadRegister(MPU6050 *dev, uint8_t reg, uint8_t* data){
	return HAL_I2C_Mem_Read(dev->i2cHandle, MPU_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}
HAL_StatusTypeDef MPU6050_ReadRegisters(MPU6050 *dev, uint8_t reg, uint8_t* data, uint8_t len){
	return HAL_I2C_Mem_Read(dev->i2cHandle, MPU_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY);
}
HAL_StatusTypeDef MPU6050_WriteRegister(MPU6050 *dev, uint8_t reg, uint8_t data_value){
    return HAL_I2C_Mem_Write(dev->i2cHandle, MPU_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &data_value, 1, HAL_MAX_DELAY);
}
