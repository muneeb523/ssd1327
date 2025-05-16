/*
 * OLED_SSD1327.c
 *
 *  The MIT License.
 *  Created on: 16.07.2020
 *      Author: Mateusz Salamon
 *      www.msalamon.pl
 *      mateusz@msalamon.pl
 */



#include "ssd1327_oled.h"

#include <linux/spi/spidev.h>
#include <sys/ioctl.h>       // for ioctl()
#include <fcntl.h>           // for open()
#include <unistd.h>          // for close(), write(), read()

typedef struct {
    int spi_fd;
    struct gpiod_line_request *rst_req;
    struct gpiod_line_request *cs_req;
    struct gpiod_line_request *dc_req;
    struct gpiod_line_request *led_req;
    uint32_t _freq;
} ssd1327;

 ssd1327 SSD1327;  // or global if needed


#define SSD1327_BUFFERSIZE (SSD1327_LCDHEIGHT * SSD1327_LCDWIDTH / 2)
static uint8_t buffer[SSD1327_BUFFERSIZE];

// Function to simulate the requestOutputLine (you should implement it according to your system)
struct gpiod_line_request *requestOutputLine(const char *chip_path, unsigned int offset, const char *consumer)
{
	struct gpiod_request_config *req_cfg = gpiod_request_config_new();
	struct gpiod_line_settings *settings = gpiod_line_settings_new();
	struct gpiod_line_config *line_cfg = gpiod_line_config_new();

	if (!req_cfg || !settings || !line_cfg)
	{
		printf("Failed to allocate GPIO settings");
	}

	gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_OUTPUT);
	gpiod_line_settings_set_drive(settings, GPIOD_LINE_DRIVE_PUSH_PULL);
	gpiod_line_settings_set_bias(settings, GPIOD_LINE_BIAS_PULL_UP);
	gpiod_line_settings_set_output_value(settings, GPIOD_LINE_VALUE_INACTIVE);
	gpiod_line_config_add_line_settings(line_cfg, &offset, 1, settings);
	gpiod_request_config_set_consumer(req_cfg, consumer);

	struct gpiod_chip *chip = gpiod_chip_open(chip_path);
	if (!chip)
	{
		printf("Failed to open GPIO chip");
	}

	struct gpiod_line_request *request = gpiod_chip_request_lines(chip, req_cfg, line_cfg);

	gpiod_chip_close(chip);
	gpiod_line_config_free(line_cfg);
	gpiod_line_settings_free(settings);
	gpiod_request_config_free(req_cfg);

	if (!request)
	{
		printf("Failed to request GPIO line %d", offset);
	}

	return request;
}

void setLineValue(struct gpiod_line_request *request, unsigned int line_offset, enum gpiod_line_value value)
{
	// std::cout << "line_offset: " << line_offset
	//           << ", rst_line_offset: " << rst_line_offset
	//           << ", dc_offset: " << dc_line_offset << std::endl;

	if (!request || gpiod_line_request_set_value(request, line_offset, value) < 0)
	{
		printf("Failed to set GPIO line value\r\n");
	}
}
void Pin_RES_High(void) 
{
    setLineValue(SSD1327.rst_req, GPIO_LINE_LCD_RST, GPIOD_LINE_VALUE_ACTIVE);
    printf("Pin_RES_High\r\n");
}

void Pin_RES_Low(void) 
{
    setLineValue(SSD1327.rst_req, GPIO_LINE_LCD_RST, GPIOD_LINE_VALUE_INACTIVE);
    printf("Pin_RES_Low\r\n");
}

void Pin_DC_High(void) 
{
    setLineValue(SSD1327.dc_req, GPIO_LINE_LCD_DC, GPIOD_LINE_VALUE_ACTIVE);
}

void Pin_DC_Low(void) 
{
   setLineValue(SSD1327.dc_req, GPIO_LINE_LCD_DC, GPIOD_LINE_VALUE_INACTIVE); // Command mode
}

void SPI_send(uint16_t len, uint8_t *data) 
{

	struct spi_ioc_transfer tr = {};
    tr.speed_hz = SPI_DEFAULT_FREQ;
    tr.bits_per_word = 8;
    tr.tx_buf = (uintptr_t)data; // Pointing to data array
    tr.len = len; // Length to send
    tr.cs_change = true;


    // If the length is greater than 1 byte, send in chunks if needed
    while (len > 0) {
        // If the length to send is greater than what can be handled in one transfer, adjust
        uint16_t chunk_size = (len > 256) ? 256 : len; // Adjust the chunk size, here assuming 256 bytes is safe

        tr.len = chunk_size;

        // Perform the SPI transfer
        if (ioctl(SSD1327.spi_fd, SPI_IOC_MESSAGE(1), &tr) < 0) {
            printf("failed spi");
            return;
        }

        // Update data pointer and length for the next chunk
        data += chunk_size;
        len -= chunk_size;
    }

    // End transmission: Pull CS high
}
void SSD1327_init_peripherals(uint32_t freq)
{
	// Default values if freq is not provided
	if (freq == 0)
	{
		freq = SPI_DEFAULT_FREQ;
	}

	SSD1327.rst_req = requestOutputLine(GPIO_DEVICE4, GPIO_LINE_LCD_RST, "LCD_RST");
	// SSD1327.cs_req  = requestOutputLine(GPIO_DEVICE5, GPIO_LINE_LCD_CS,  "LCD_CS");
	SSD1327.dc_req = requestOutputLine(GPIO_DEVICE4, GPIO_LINE_LCD_DC, "LCD_DC");
	SSD1327.led_req = requestOutputLine(GPIO_DEVICE4, GPIO_LINE_LCD_LED, "LCD_LED");

	// Open the SPI device
	SSD1327.spi_fd = open(SPI_DEVICE, O_RDWR);
	if (SSD1327.spi_fd < 0)
	{
		perror("Failed to open SPI device");
		exit(1);
	}

	// Set the SPI mode to SPI_MODE_0
	uint8_t mode = SPI_MODE_0;
	if (ioctl(SSD1327.spi_fd, SPI_IOC_WR_MODE, &mode) < 0)
	{
		perror("SPI mode");
		close(SSD1327.spi_fd);
		exit(1);
	}

	// Set the SPI speed (frequency)
	if (ioctl(SSD1327.spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &freq) < 0)
	{
		perror("Failed to set SPI frequency");
		close(SSD1327.spi_fd);
		exit(1);
	}
}

void SSD1327_Command(uint8_t com)
{
	Pin_DC_Low();
    usleep(2000);
    SPI_send(1, &com);

}

void SSD1327_Data(uint8_t dat)
{
	Pin_DC_High();
	usleep(2000);
	SPI_send(1, &dat);
}


void SSD1327_Reset(void)
{
	setLineValue(SSD1327.rst_req, GPIO_LINE_LCD_RST, GPIOD_LINE_VALUE_INACTIVE);
	usleep(2000);
	setLineValue(SSD1327.rst_req, GPIO_LINE_LCD_RST, GPIOD_LINE_VALUE_ACTIVE);
}

//
// Configuration functions
//
void SSD1327_InvertColors(uint8_t Invert)
{

	SSD1327_Command(Invert ? SSD1327_INVERTDISPLAY : SSD1327_NORMALDISPLAY);
}

void SSD1327_RotateDisplay(uint8_t Rotate)
{
	if (Rotate > 1)
		Rotate = 1;

	SSD1327_Command(0xA0 | (0x01 & Rotate)); // Set Segment Re-Map Default
											 // 0xA0 (0x00) => column Address 0 mapped to 127
											 // 0xA1 (0x01) => Column Address 127 mapped to 0

	SSD1327_Command(0xC0 | (0x08 & (Rotate << 3))); // Set COM Output Scan Direction
													// 0xC0	(0x00) => normal mode (RESET) Scan from COM0 to COM[N-1];Where N is the Multiplex ratio.
													// 0xC8	(0xC8) => remapped mode. Scan from COM[N-1] to COM0;;Where N is the Multiplex ratio.
}

void SSD1327_DisplayON(uint8_t On)
{
	SSD1327_Command(On ? SSD1327_DISPLAYON : SSD1327_DISPLAYOFF);
}

void SSD1327_SetContrast(uint8_t Contrast)
{
	SSD1327_Command(SSD1327_SETCONTRASTCURRENT); // Set Contrast Control
	SSD1327_Command(Contrast);
}

#if GRAPHIC_ACCELERATION_COMMANDS == 1
//
// Graphic Acceleration Command
//
void SSD1327_StartScrollRight(uint8_t StartPage, uint8_t EndPage, uint8_t Speed)
{
	SSD1327_Command(SSD1327_RIGHT_HORIZONTAL_SCROLL);
	SSD1327_Command(0x00);
	SSD1327_Command(StartPage);
	SSD1327_Command(Speed);
	SSD1327_Command(EndPage);
	SSD1327_Command(SSD1327_ACTIVATE_SCROLL);
}

void SSD1327_StartScrollLeft(uint8_t StartPage, uint8_t EndPage, uint8_t Speed)
{
	SSD1327_Command(SSD1327_LEFT_HORIZONTAL_SCROLL);
	SSD1327_Command(0x00);
	SSD1327_Command(StartPage);
	SSD1327_Command(Speed);
	SSD1327_Command(EndPage);
	SSD1327_Command(SSD1327_ACTIVATE_SCROLL);
}

void SSD1327_StopScroll(void)
{
	SSD1327_Command(SSD1327_DEACTIVATE_SCROLL);
}
#endif

//
// Initialization
//
void SSD1327_Init(void)
{
	SSD1327_Command(SSD1327_DISPLAYOFF); // Turn off the display during initialization

	SSD1327_Command(SSD1327_SETMULTIPLEX); // Set multiplex ratio
	SSD1327_Command(0x5F);				   // Configure multiplex ratio to 96 (0x5F)

	SSD1327_Command(SSD1327_SETDISPLAYSTARTLINE); // Set the starting line for the display
	SSD1327_Command(0x00);						  // Starting line is set to 0

	SSD1327_Command(SSD1327_SETDISPLAYOFFSET); // Set the vertical offset of the display
	SSD1327_Command(0x20);					   // Vertical offset set to 32

	SSD1327_Command(SSD1327_SEGREMAP); // Configure segment remapping
	SSD1327_Command(0x51);			   // Enable remapping and configure COM/SEG direction

	SSD1327_SetContrast(0x7F); // Set display contrast to 127 (mid-level)

	SSD1327_Command(SSD1327_SETPHASELENGTH); // Set phase length for display timing
	SSD1327_Command(0x22);					 // Set phase lengths (higher nibble = 2, lower nibble = 2)

	SSD1327_Command(SSD1327_SETFRONTCLOCKDIVIDER_OSCILLATORFREQUENCY); // Set the display clock divider and oscillator frequency
	SSD1327_Command(0x50);											   // Clock divide ratio = 1, oscillator frequency set

	SSD1327_Command(SSD1327_SELECTDEFAULTLINEARGRAYSCALETABLE); // Select the default linear grayscale table

	SSD1327_Command(SSD1327_SETPRECHARGEVOLTAGE); // Set the pre-charge voltage
	SSD1327_Command(0x10);						  // Pre-charge voltage set to 0.6 × Vcc

	SSD1327_Command(SSD1327_SETSETVCOMVOLTAGE); // Set the VCOMH voltage
	SSD1327_Command(0x05);						// VCOMH voltage set to 0.77 × Vcc

	SSD1327_Command(SSD1327_SETSECONDPRECHARGEPERTIOD); // Set the second pre-charge period
	SSD1327_Command(0x0a);								// Second pre-charge period set to 10 DCLKs

	SSD1327_Command(SSD1327_FUNCTIONSELECTIONB); // Enable external VSL
	SSD1327_Command(0x62);						 // Configure VSL to external mode

	SSD1327_Command(SSD1327_SETCOLUMNADDRESS); // Set the column address range
	SSD1327_Command(0x00);					   // Start column address = 0
	SSD1327_Command(0x7F);					   // End column address = 127

	SSD1327_Command(SSD1327_SETROWADDRESS); // Set the row address range
	SSD1327_Command(0x00);					// Start row address = 0
	SSD1327_Command(0x7F);					//////< End row address = 127

	SSD1327_Command(SSD1327_NORMALDISPLAY); //////< Set normal display mode (non-inverted colors)

	SSD1327_Command(SSD1327_DISPLAYALLON_RESUME); /////< Resume display from all-on mode

#if GRAPHIC_ACCELERATION_COMMANDS == 1

	SSD1327_StopScroll(); // Stop any scrolling operation if graphic acceleration is enabled

#endif

	SSD1327_DisplayON(1); // Turn on the display
}

#ifdef SSD1327_I2C_CONTROL
void SSD1327_I2cInit(I2C_HandleTypeDef *i2c)
{
	ssd1337_i2c = i2c;

	SSD1327_Init();
}
#endif

#ifdef SSD1327_SPI_CONTROL
void SSD1327_SpiInit()
{
	SSD1327_init_peripherals(SPI_DEFAULT_FREQ);
	SSD1327_Reset();
	SSD1327_Init();
}
#endif

//
// Draw pixel in the buffer
//
void SSD1327_DrawPixel(int16_t x, int16_t y, uint8_t Color)
{
	if ((x < 0) || (x >= SSD1327_LCDWIDTH) || (y < 0) || (y >= SSD1327_LCDHEIGHT))
		return;

	uint8_t SelectedCell = buffer[x / 2 + y * (SSD1327_LCDWIDTH / 2)];

	if (x % 2)
	{
		SelectedCell &= ~(0x0F);
		SelectedCell |= (0x0F & Color);
	}
	else
	{
		SelectedCell &= ~(0xF0);
		SelectedCell |= (0xF0 & (Color << 4));
	}

	buffer[x / 2 + y * (SSD1327_LCDWIDTH / 2)] = SelectedCell;
}

//
// Clear the buffer
//
void SSD1327_Clear(uint8_t Color)
{
	if (Color > WHITE)
		Color = WHITE;
	memset(buffer, (Color << 4 | Color), SSD1327_BUFFERSIZE);
}

//
// Send buffer to OLDE GCRAM
//
void SSD1327_Display(void)
{
	SSD1327_Command(SSD1327_SETCOLUMNADDRESS);
	SSD1327_Command(0x00);
	SSD1327_Command(0x7F); // 127
	SSD1327_Command(SSD1327_SETROWADDRESS);
	SSD1327_Command(0x00);
	SSD1327_Command(0x7F); // 127
	Pin_DC_High();
	usleep(1000);
	SPI_send(SSD1327_BUFFERSIZE, (uint8_t *)&buffer);

}

//
// Display Bitmap directly on screen
//
void SSD1327_Bitmap(uint8_t *bitmap)
{
	SSD1327_Command(0x22);
	SSD1327_Command(0x00);
	SSD1327_Command(0x07);
	Pin_DC_High();
	usleep(1000);
	SPI_send((SSD1327_LCDHEIGHT * SSD1327_LCDWIDTH / 8),bitmap);
}

