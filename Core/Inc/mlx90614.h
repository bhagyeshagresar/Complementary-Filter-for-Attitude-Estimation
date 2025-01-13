/*
 * mlx90614.h
 * MLX90614 Driver Header File
 *
 *  Created on: Jan 11, 2025
 *      Author: Bhagyesh Agresar
 */

#ifndef INC_MLX90614_H_
#define INC_MLX90614_H_


#include "stm32f4xx_hal.h"

//Note: In I2C the address of slave device is 7bits and stm32 hardware requires we << 1 to accomodate the R/W bit.
//The R/W bit setting is taken care of by the HAL
#define MLX90614_ADDR (0x5A << 1) //Refer Table 4 on Pg. 12 of datasheet

//Define the RAM registers for read access on the MLX90614. Refer Table 10 on Pg.17 of datasheet
#define MLX90614_RAM_AMBIENT_TEMP 0x03
#define MLX90614_RAM_IR_DATA_01 0x04
#define MLX90614_RAM_IR_DATA_02 0x05
#define MLX90614_RAM_TOBJ1 0x07

#define MLX90614_BUFFER_LEN 14

typedef struct{
	I2C_HandleTypeDef *i2cHandle;
	int16_t temp_data_c; //The range of 0x07 will go from 0x27AD to 0x7FFF. Refer Pg.20 in Datasheet


}MLX90614;





//Function to initialize the chip
HAL_StatusTypeDef MLX90614_initialise(MLX90614 *dev, I2C_HandleTypeDef *i2cHandle);
HAL_StatusTypeDef MLX90614_ReadTemperature(MLX90614 *dev);


//Low Level Functions
HAL_StatusTypeDef MLX90614_ReadRegister(MLX90614 *dev, uint8_t reg, uint8_t *data);
HAL_StatusTypeDef MLX90614_ReadRegisters(MLX90614 *dev, uint8_t reg, uint8_t *data, uint8_t length);

#endif /* INC_MLX90614_H_ */
