/*
 * OLED_SSD1327.h
 *
 *  The MIT License.
 *  Created on: 16.07.2020
 *      Author: Mateusz Salamon
 *      www.msalamon.pl
 *      mateusz@msalamon.pl
 */

#ifndef OLED_SSD1327_H_
#define OLED_SSD1327_H_

/*
 *
 *    SETTINGS
 *
 *    Please set only one interface. It won't work with both one time.
 *
 */
//#define SSD1327_SPI_CONTROL
#define SSD1327_SPI_CONTROL

#ifdef SSD1327_I2C_CONTROL
//#define SSD1327_I2C_DMA_ENABLE
#define SSD1327_I2C_ADDRESS   0x78 // original file

#endif
#ifdef SSD1327_SPI_CONTROL
#define SSD1327_RESET_USE
#define SSD1327_SPI_DMA_ENABLE
//#define SPI_CS_HARDWARE_CONTROL 0
#endif

//
// Resolution
//
#define SSD1327_LCDWIDTH                  128
#define SSD1327_LCDHEIGHT                 128

/*
 * 		Please set what functionality you want to use.
 * 		Some functions need other functionalities. It should works automatically.
 *
 * 		1 - will be compiled
 * 		0 - won't be compiled
 */
#define GRAPHIC_ACCELERATION_COMMANDS 0

/****************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <gpiod.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>
#include <linux/input.h>
#include <poll.h>
#include <linux/input-event-codes.h>
#include <sys/epoll.h>
#include <string.h>
#include <errno.h>
//
// Commands
//
#define SSD1327_SETCOLUMNADDRESS	0x15
#define SSD1327_SETROWADDRESS		0x75
#define SSD1327_SETCONTRASTCURRENT 	0x81
#define SSD1327_NOP					0x84
#define SSD1327_SEGREMAP			0xA0
#define SSD1327_SETDISPLAYSTARTLINE	0xA1
#define SSD1327_SETDISPLAYOFFSET	0xA2
#define SSD1327_DISPLAYALLON_RESUME 0xA4
#define SSD1327_DISPLAYALLON 		0xA5
#define SSD1327_NORMALDISPLAY 		0xA6
#define SSD1327_INVERTDISPLAY 		0xA7
#define SSD1327_SETMULTIPLEX		0xA8
#define SSD1327_FUNCTIONSELECTIONA	0xAB
#define SSD1327_DISPLAYOFF			0xAE
#define SSD1327_DISPLAYON			0xAF
#define SSD1327_SETPHASELENGTH		0xB1
#define SSD1327_SETFRONTCLOCKDIVIDER_OSCILLATORFREQUENCY 0xB3
#define SSD1327_SETGPIO				0xB5
#define SSD1327_SETSECONDPRECHARGEPERTIOD	0xB6
#define SSD1327_SETGRAYSCALETABLE	0xB8
#define SSD1327_SELECTDEFAULTLINEARGRAYSCALETABLE	0xB9
#define SSD1327_SETPRECHARGEVOLTAGE	0xBC
#define SSD1327_SETSETVCOMVOLTAGE	0xBE
#define SSD1327_FUNCTIONSELECTIONB	0xD5
#define SSD1327_SETCOMMANDLOCK		0xFD

//
// Scrolling #defines
//
#define SSD1327_ACTIVATE_SCROLL			0x2F
#define SSD1327_DEACTIVATE_SCROLL		0x2E
#define SSD1327_RIGHT_HORIZONTAL_SCROLL	0x26
#define SSD1327_LEFT_HORIZONTAL_SCROLL	0x27


#define GPIO_DEVICE4 "/dev/gpiochip3"
#define GPIO_LINE_LCD_RST 1
#define GPIO_LINE_LCD_DC 4
#define GPIO_LINE_LCD_LED 10
#define GPIO_LINE_LCD_CS 13

#define SPI_DEVICE "/dev/spidev1.0"
#define SPI_DEFAULT_FREQ  15000000 // 8 MHz


//
// Colors
//
#define BLACK 0
// Grays between
#define WHITE 15
#define GRAY  7
//
// Scrolling enums
//
typedef enum
{
	SCROLL_EVERY_5_FRAMES,
	SCROLL_EVERY_64_FRAMES,
	SCROLL_EVERY_128_FRAMES,
	SCROLL_EVERY_256_FRAMES,
	SCROLL_EVERY_3_FRAMES,
	SCROLL_EVERY_4_FRAMES,
	SCROLL_EVERY_25_FRAMES,
	SCROLL_EVERY_2_FRAMES,
	
} scroll_horizontal_speed;
//
// Functions
//


void SSD1327_SpiInit();

//
// Configuration
//
void SSD1327_DisplayON(uint8_t On);
void SSD1327_InvertColors(uint8_t Invert);
void SSD1327_RotateDisplay(uint8_t Rotate);
void SSD1327_SetContrast(uint8_t Contrast);

//
// Drawing
//
void SSD1327_DrawPixel(int16_t x, int16_t y, uint8_t Color);
void SSD1327_Clear(uint8_t Color);
void SSD1327_Display(void);
void SSD1327_Bitmap(uint8_t *bitmap);

#if GRAPHIC_ACCELERATION_COMMANDS == 1
//
// Graphic Acceleration Commands
//
void SSD1327_StartScrollRight(uint8_t StartPage, uint8_t EndPage, scroll_horizontal_speed Speed);
void SSD1327_StartScrollLeft(uint8_t StartPage, uint8_t EndPage, scroll_horizontal_speed Speed);
void SSD1327_StopScroll(void);
#endif


#endif /* OLED_SSD1327_H_ */
