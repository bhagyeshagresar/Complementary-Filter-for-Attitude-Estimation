/*
 * epaper.h
 *
 *  Created on: Dec 24, 2024
 *      Author: bhagy
 */

#ifndef INC_SSD1680_H_
#define INC_SSD1680_H_


#include "stm32f4xx_hal.h"






//Define the Registers for the chip
#define SSD1680_DRVR_OUT_CTRL		0x01  // Default Settings: 296 MUX, GD = 0, SM = 0, TB = 0, Pg no. 20
#define SSD1680_DEEP_SLEEP_MODE		0x10  // Default Settings: 00 Normal Mode, Pg no. 23
#define SSD1680_SW_RST 				0x12  // Reset to Software Default except Sleep mode Pg. 23







#endif /* INC_SSD1680_H_ */
