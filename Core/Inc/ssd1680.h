/*
 * epaper.h
 *
 *  Created on: Dec 24, 2024
 *      Author: bhagy
 */

#ifndef INC_SSD1680_H_
#define INC_SSD1680_H_


#include "stm32f4xx_hal.h"


//All the information is sent to the chip as 'commands'
//Some commands are followed by multi-byte data.
//When sending 8bit command, D/C# pin is low
//When sending 8bit data, D/C# pin is high







//Cap the number of source and Gate outputs based on the display resolution
#define SSD1680_SOURCE_OUTPUTS 122
#define SSD1680_GATE_OUTPUTS   250
#define SSD1680_BUFFER_LEN (SSD1680_SOURCE_OUTPUTS*SSD1680_GATE_OUTPUTS) //Display Size



//Define the Commands and Registers for the chip
#define SSD1680_DRVR_OUT_CTRL		0x01  // POR: 296 MUX, GD = 0, SM = 0, TB = 0, Pg no. 20
#define SSD1680_DEEP_SLEEP_MODE		0x10  // POR: 00 Normal Mode, Pg no. 23
#define SSD1680_SW_RST 				0x12  // Reset to Software Default except Sleep mode, Pg. 23
#define SSD1680_TMP_CTRL 			0x18  // POR: 0x48, external temperature sensor, Pg. 24
#define SSD1680_TMP_CTRL_INTERNAL 	0x1A  // POR: 0x7FF, write to temperature register, Pg. 24
#define SSD1680_TMP_CTRL_R			0x1B  // Pg. 24
#define SSD1680_TMO_CTRL_EXT		0x1C  // POR: 0x00, Pg. 25
#define SSD1680_MSTR_ACT			0x20  // Pg. 25
#define SSD1680_RAM_ADDR_CNTR		0x11  // POR: X increment, Y increment, Update in X direction, Pg. 23
#define SSD1680_Border_Signal		0x3C  // POR: A[7:0] = 0xC0
#define SSD1680_DSPLY_UPDATE_CTRL2	0x22  // POR: A[7:0] = 0xFF, Pg. 26
#define SSD1680_DSPLY_UPDATE_CTRL1	0x21  // POR: A[7:0] = 0x00, B[7:0] = 0x00 Pg. 26
#define SSD1680_DSPLY_MSTR_UPDATE 	0x20  // Pg. 25
#define SSD1680_SET_X_RANGE	 		0x44  // POR: XStart = 0x00, XEnd = 0x15
#define SSD1680_SET_Y_RANGE			0x45  // POR: Ystart = 0x00, YEnd = 0x127
#define SSD1680_RAM_X_ADDR_CNTR 	0x4E  // POR: 0x00
#define SSD1680_RAM_Y_ADDR_CNTR 	0x4F  // POR: 0x00
#define SSD1680_WRITE_RAM_BW		0x24
#define SSD1680_WRITE_RAM_RED		0x26
#define SSD1680_SOFT_START 			0x0C


//Declare the SSD1680 Display to index through each element of the buffer
extern uint8_t SSD1680_frame_buffer[SD1680_BUFFER_LEN];

typedef struct{

	SPI_HandleTypeDef *spiHandle;


}SSD1680;






void SSD1680_init(SSD1680* driver, SPI_HandleTypeDef* spiHandle);
void SSD1680_send_command(SSD1680* driver, uint8_t command);
void SSD1680_send_data(SSD1680* driver);
void SSD1680_load_waveform_lut();
void SSD1680_update_display();
void SSD1680_clear_display();




#endif /* INC_SSD1680_H_ */
