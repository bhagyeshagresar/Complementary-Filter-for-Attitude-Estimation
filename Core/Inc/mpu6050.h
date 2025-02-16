/*
 * mpu6050.h
 *
 *  Created on: Feb 15, 2025
 *      Author: bhagy
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_


#include "stm32f4xx_hal.h"



#define MPU6050_INT_PIN GPIO_PIN_0



//extern volatile int mpu6050_data_ready;


static const uint8_t MPU6050_ADDR = 0x68<<1;
static const uint8_t WHO_AM_I = 0x75;
static const uint8_t ACCEL_ZOUT_L = 0x40;
static const uint8_t ACCEL_ZOUT_H = 0x3F;
static const uint8_t ACCEL_XOUT_L = 0x3C;
static const uint8_t ACCEL_XOUT_H = 0x3B;
static const uint8_t ACCEL_CONFIG = 0x1C;
static const uint8_t MPU_6050_INT_EN_REG = 0x38;


//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);


#endif /* INC_MPU6050_H_ */
