/*
 * MLX90614.c
 *
 *  Created on: Jan 11, 2025
 *      Author: bhagy
 */


#include "mlx90614.h"
//#include "stm32f4xx_hal_i2c.h"







HAL_StatusTypeDef MLX90614_initialise(MLX90614 *dev, I2C_HandleTypeDef *i2cHandle){

	dev->i2cHandle = i2cHandle;

	dev->temp_data_c = 0.0;

	return HAL_I2C_IsDeviceReady(dev->i2cHandle, MLX90614_ADDR, 100, HAL_MAX_DELAY);


}

HAL_StatusTypeDef MLX90614_ReadTemperature(MLX90614 *dev){


	uint8_t temp_buff[2];

	HAL_StatusTypeDef ret = MLX90614_ReadRegisters(dev, MLX90614_RAM_TOBJ1, temp_buff, 2);

    int16_t temp_raw = ((temp_buff[1] << 8 )| temp_buff[0]);



	//Convert to readable temperature

	dev->temp_data_c = (temp_raw*0.02) - 273.15;



	return ret;


}



HAL_StatusTypeDef MLX90614_ReadRegister(MLX90614 *dev, uint8_t reg, uint8_t *data){

	return HAL_I2C_Mem_Read(dev->i2cHandle, MLX90614_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);

}



HAL_StatusTypeDef MLX90614_ReadRegisters(MLX90614 *dev, uint8_t reg, uint8_t *data, uint8_t length){

	return HAL_I2C_Mem_Read(dev->i2cHandle, MLX90614_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);

}





