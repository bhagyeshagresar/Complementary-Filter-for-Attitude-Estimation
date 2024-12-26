/*
 * epaper_display.c
 *
 *  Created on: Dec 24, 2024
 *      Author: bhagy
 */

#include "ssd1680.h"


void SSD1680_init(SSD1680* driver, SPI_HandleTypeDef* spiHandle){

	//Initialise the SPI interface
	driver->spiHandle = spiHandle;

	//TODO : HW Reset using GPIO

	//SW Reset
	SSD1680_send_command(&driver, SSD1680_SW_RST);









}



void SSD1680_send_command(SSD1680* driver, uint8_t command){



}
