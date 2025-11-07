/*
 * gmt020-02.c
 *
 *  Created on: May 24, 2025
 *      Author: Hamid
 */


#include <ST7789_2x240x240_12bit.h>
#include "fonts.h"
#include "main.h"

#define RESET do { \
	HAL_GPIO_WritePin(ST7789_GPIO_Port, (1U << RST_PIN_INDEX), 1); \
	HAL_Delay(150); \
	HAL_GPIO_WritePin(ST7789_GPIO_Port, (1U << RST_PIN_INDEX), 0); \
	HAL_Delay(150); \
} while (0)
	//	ST7789_GPIO_Port->BSRR = RST_PIN_LOW;
	//	ST7789_GPIO_Port->BSRR = RST_PIN_LOW;
#define BACKLIGHT_ON (ST7789_GPIO_Port->BSRR = TFT_BLK_HIGH)
#define BACKLIGHT_OFF (ST7789_GPIO_Port->BSRR = TFT_BLK_LOW)

uint16_t colors[] = { BLACK, WHITE, RED, GREEN, BLUE, CYAN, YELLOW, PURPLE, PINK };
//uint8_t addr[] = {ST7789_CASET, ST7789_RASET, ST7789_RAMWR };


/*void ST7789_init()
{
	uint8_t init_addr[] = {ST7789_SWRST, ST7789_SLPOUT, ST7789_COLMOD, ST7789_MADCTL, ST7789_CASET, ST7789_RASET, ST7789_INVON, ST7789_NORON, ST7789_DISPON };
	uint8_t data[] = {0x55, 0x00};
	uint16_t data_16bit[] = {ST7789_XSTART, 0x00EF, ST7789_YSTART, 0x00EF};

//	DATA_MODE(1);
//	DATA_MODE(2);

//	RESET;

	HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, 0);
	HAL_Delay(150);
	HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, 1);
	HAL_Delay(150);

	ST7789_write_command();
//	COMMAND_MODE(1);
//	COMMAND_MODE(2);
	HAL_SPI_Transmit(&hspi1, &init_addr[0], 1, 500); //ST7789_SWRST
	HAL_Delay(150);
	HAL_SPI_Transmit(&hspi1, &init_addr[1], 1, 500); //ST7789_SLPOUT
	HAL_Delay(500);

	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin); //
	HAL_SPI_Transmit(&hspi1, &init_addr[2], 1, 500); //ST7789_COLMOD
	HAL_Delay(50); //
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin); //

	ST7789_write_data();
//	DATA_MODE(1);
//	DATA_MODE(2);
	HAL_SPI_Transmit(&hspi1, &data[0], 1, 500); //0x55 - 16bit; 0x03 - 12bit
	HAL_Delay(50);

	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin); //
	ST7789_write_command();
//	COMMAND_MODE(1);
//	COMMAND_MODE(2);
	HAL_SPI_Transmit(&hspi1, &init_addr[3], 1, 500); //ST7789_MADCTL
	HAL_Delay(50); //
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin); //

	ST7789_write_data();
//	DATA_MODE(1);
//	DATA_MODE(2);
	HAL_SPI_Transmit(&hspi1, &data[1], 1, 500); // Try different values: 0x00, 0xC0, 0xA0, 0x60
	HAL_Delay(50); //
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin); //


	// Writing 2 x 16 bit data streams as 4 x 8 bits
	// Sending 0x0000 as 0x00 0x00
	// Sending 239, 0x00EF as 0x00 0xEF

	ST7789_write_command();
//	COMMAND_MODE(1);
//	COMMAND_MODE(2);
	HAL_SPI_Transmit(&hspi1, &init_addr[4], 1, 500); //ST7789_CASET
	HAL_Delay(50); //
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin); //
	ST7789_write_data();
//	DATA_MODE(1);
//	DATA_MODE(2);
	ST7789_write_16bit(ST7789_XSTART);
	ST7789_write_16bit(&data_16bit[1]); //239
	HAL_Delay(50); //
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin); //

	// Writing 2 x 16 bit data streams as 4 x 8 bits
	// Sending 0x0000 as 0x00 0x00
	// Sending 239, 0x00EF as 0x00 0xEF
	ST7789_write_command();
//	COMMAND_MODE(1);
//	COMMAND_MODE(2);
	HAL_SPI_Transmit(&hspi1, &init_addr[5], 1, 500); //ST7789_RASET
	HAL_Delay(50); //
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin); //
	ST7789_write_data();
//	DATA_MODE(1);
//	DATA_MODE(2);
	ST7789_write_16bit(ST7789_YSTART);
	ST7789_write_16bit(&data_16bit[3]); //239
	HAL_Delay(50); //
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin); //

	ST7789_write_command();
//	COMMAND_MODE(1);
//	COMMAND_MODE(2);
	HAL_SPI_Transmit(&hspi1, &init_addr[6], 1, 500);
	HAL_Delay(10);
	HAL_SPI_Transmit(&hspi1, &init_addr[7], 1, 500);
	HAL_Delay(10);
	HAL_SPI_Transmit(&hspi1, &init_addr[8], 1, 500);
	HAL_Delay(20);
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin); //
	ST7789_end_write();
//	COMM_END(1);
//	COMM_END(2);

//	BACKLIGHT_ON;
}*/

void ST7789_init()
{

	uint8_t data[] = {0x55, 0x00};
	uint16_t data_16bit[] = {ST7789_XSTART, 0x00EF, ST7789_YSTART, 0x00EF};
	uint8_t addr[] = {ST7789_SWRST, ST7789_SLPOUT, ST7789_COLMOD, ST7789_MADCTL, ST7789_CASET, ST7789_RASET, ST7789_INVON, ST7789_NORON, ST7789_DISPON};

	COMMAND_MODE(CS1_PIN_INDEX);
	COMMAND_MODE(CS2_PIN_INDEX);
//ST7789_write_command();

	HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, 0);
	HAL_Delay(150);
	HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, 1);
	HAL_Delay(150);

	COMMAND_MODE(CS1_PIN_INDEX);
	COMMAND_MODE(CS2_PIN_INDEX);
//	ST7789_write_command();
	HAL_SPI_Transmit(&hspi1, &addr[0], 1, 500); //ST7789_SWRST
	HAL_Delay(150);
	HAL_SPI_Transmit(&hspi1, &addr[1], 1, 500); //ST7789_SLPOUT
	HAL_Delay(500);

	HAL_SPI_Transmit(&hspi1, &addr[2], 1, 500); //ST7789_COLMOD
	DATA_MODE(CS1_PIN_INDEX);
	DATA_MODE(CS2_PIN_INDEX);
//	HAL_Delay(10000);
//	ST7789_write_data();
	HAL_SPI_Transmit(&hspi1, &data[0], 1, 500); //0x55
	HAL_Delay(50);

	COMMAND_MODE(CS1_PIN_INDEX);
	COMMAND_MODE(CS2_PIN_INDEX);
//	ST7789_write_command();
	HAL_SPI_Transmit(&hspi1, &addr[3], 1, 500); //ST7789_MADCTL
	DATA_MODE(CS1_PIN_INDEX);
	DATA_MODE(CS2_PIN_INDEX);
//	ST7789_write_data();
	HAL_SPI_Transmit(&hspi1, &data[1], 1, 500); // Try different values: 0x00, 0xC0, 0xA0, 0x60


	// Writing 2 x 16 bit data streams as 4 x 8 bits
	// Sending 0x0000 as 0x00 0x00
	// Sending 239, 0x00EF as 0x00 0xEF
	COMMAND_MODE(CS1_PIN_INDEX);
	COMMAND_MODE(CS2_PIN_INDEX);
//	ST7789_write_command();
	HAL_SPI_Transmit(&hspi1, &addr[4], 1, 500); //ST7789_CASET
	DATA_MODE(CS1_PIN_INDEX);
	DATA_MODE(CS2_PIN_INDEX);
//	ST7789_write_data();
	ST7789_write_16bit(&data_16bit[0]);
	ST7789_write_16bit(&data_16bit[1]);

	// Writing 2 x 16 bit data streams as 4 x 8 bits
	// Sending 0x0000 as 0x00 0x00
	// Sending 239, 0x00EF as 0x00 0xEF
	COMMAND_MODE(CS1_PIN_INDEX);
	COMMAND_MODE(CS2_PIN_INDEX);
//	ST7789_write_command();
	HAL_SPI_Transmit(&hspi1, &addr[5], 1, 500); //ST7789_RASET
	DATA_MODE(CS1_PIN_INDEX);
	DATA_MODE(CS2_PIN_INDEX);
//	ST7789_write_data();
	ST7789_write_16bit(&data_16bit[2]);
	ST7789_write_16bit(&data_16bit[3]);


	COMMAND_MODE(CS1_PIN_INDEX);
	COMMAND_MODE(CS2_PIN_INDEX);
	//	ST7789_write_command();
	HAL_SPI_Transmit(&hspi1, &addr[6], 1, 500); //ST7789_INVON
	HAL_Delay(10);
	HAL_SPI_Transmit(&hspi1, &addr[7], 1, 500); //ST7789_NORON
	HAL_Delay(10);
	HAL_SPI_Transmit(&hspi1, &addr[8], 1, 500); //ST7789_DISPON
	HAL_Delay(20);
//	ST7789_end_write();
	COMM_END(CS1_PIN_INDEX);
	COMM_END(CS2_PIN_INDEX);
}

void ST7789_write_data()
{
	DATA_MODE(CS1_PIN_INDEX);
	DATA_MODE(CS2_PIN_INDEX);

//	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 0);
//	HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, 1);
}

void ST7789_write_command()
{
	COMMAND_MODE(CS1_PIN_INDEX);
	COMMAND_MODE(CS2_PIN_INDEX);
//	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 0);
//	HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, 0);
}

void ST7789_end_write()
{
	COMM_END(CS1_PIN_INDEX);
	COMM_END(CS2_PIN_INDEX);
//	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, 1);
}


void ST7789_write_16bit(uint16_t *data)
{
	uint8_t bufferA = (*data) >> 8;;
	uint8_t bufferB = (*data) & 0xFF;

	HAL_SPI_Transmit(&hspi1, &bufferA, 1, 500);
	HAL_SPI_Transmit(&hspi1, &bufferB, 1, 500);
/*	while(!(SPI1->SR & SPI_SR_TXE));  // Wait for TX buffer empty
    SPI1->DR = (*data) >> 8;
	while(SPI1->SR & SPI_SR_BSY);
    SPI1->DR = (*data) & 0xFF;
	while(SPI1->SR & SPI_SR_BSY); 	// Wait for SPI not busy
*/}

//not completed
void ST7789_write_row_sr(uint16_t *data)
{
   	while(!(SPI1->SR & SPI_SR_TXE));  // Wait for TX buffer empty
    SPI1->DR = (*data) >> 8;
	while(SPI1->SR & SPI_SR_BSY);
    SPI1->DR = (*data) & 0xFF;
	while(SPI1->SR & SPI_SR_BSY); 	// Wait for SPI not busy
}

void ST7789_set_addr(uint16_t x0, uint16_t x1, uint16_t y0, uint16_t y1)
{
	uint16_t data;
	uint8_t addr[] = {ST7789_CASET, ST7789_RASET, ST7789_RAMWR};

	ST7789_write_command();
	HAL_SPI_Transmit(&hspi1, &addr[0], 1, 500);
	ST7789_write_data();
	data = x0 + ST7789_XSTART;
	ST7789_write_16bit(&data);
	data = x1 + ST7789_XSTART;
	ST7789_write_16bit(&data);

	ST7789_write_command();
	HAL_SPI_Transmit(&hspi1, &addr[1], 1, 500);
	ST7789_write_data();
	data = y0 + ST7789_XSTART;
	ST7789_write_16bit(&data);

	data = y1 + ST7789_XSTART;
	ST7789_write_16bit(&data);


	ST7789_write_command();
	HAL_SPI_Transmit(&hspi1, &addr[2], 1, 500);
	ST7789_end_write();
}

void ST7789_draw_pixel(uint16_t x, uint16_t y, uint16_t colour)
{
	uint16_t data;
	ST7789_set_addr(x, x+1, y, y+1);
	ST7789_write_data();

	data = colour;
	ST7789_write_16bit(&data);
	ST7789_end_write();
}

void ST7789_clear_screen(uint8_t pin_index, uint16_t colour)
{
	uint32_t i;
	uint8_t pixel1 = colour >> 8;
	uint8_t pixel2 = colour & 0xFF;

	ST7789_set_addr(0, LCD_WIDTH-1, 0, LCD_HEIGHT-1);
	DATA_MODE(pin_index);
	for (i = LCD_SIZE; i > 0; i--){
		while(!(SPI1->SR & SPI_SR_TXE));  // Wait for TX buffer empty
		SPI1->DR = pixel1;
		while(!(SPI1->SR & SPI_SR_TXE));  // Wait for TX buffer empty
		SPI1->DR = pixel2;
	}
	while(SPI1->SR & SPI_SR_BSY); 	// Wait for SPI not busy
	COMM_END(pin_index);
}

void ST7789_draw_char(uint16_t x, uint16_t y, uint16_t colour, char letter)
{
    if (letter < 32 || letter > 127) return;

    int col, row;

    for (col = 0; col < 5; col++) {
        uint8_t line = ascii[letter - 32][col];

        for (row = 0; row < 8; row++) {
            if (line & (1 << row)) {
                ST7789_draw_pixel(x + col, y + row, colour);
            }
        }
    }
}

void ST7789_draw_string(uint16_t x, uint16_t y, uint16_t colour, char* string)
{
    int i;
    uint16_t x_pos = x;
    uint16_t y_pos = y;

    for (i = 0; string[i] != '\0'; i++) {
        if(x_pos + 6 > LCD_WIDTH){
            y_pos += 8;
            x_pos = 0;

            if(y_pos + 8 > LCD_HEIGHT){
                y_pos = 0;
            }

        }

        ST7789_draw_char(x_pos, y_pos, colour, string[i]);
        x_pos += 6;
    }
}


