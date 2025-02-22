/*
 * mpu6050.h
 *
 *  Created on: Feb 15, 2025
 *      Author: bhagy
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_


#include "stm32f4xx_hal.h"







//extern volatile int mpu6050_data_ready;

#define MPU6050_INT_PIN GPIO_PIN_0
#define MPU6050_ADDR (0x68<<1);
#define WHO_AM_I 0x75;
#define ACCEL_ZOUT_L 0x40;
#define ACCEL_ZOUT_H 0x3F;
#define ACCEL_XOUT_L 0x3C;
#define ACCEL_XOUT_H 0x3B;
#define ACCEL_CONFIG 0x1C;
#define MPU_6050_INT_EN_REG 0x38;


//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);


#endif /* INC_MPU6050_H_ */
