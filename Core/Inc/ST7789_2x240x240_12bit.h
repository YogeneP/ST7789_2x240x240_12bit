/*
 * gmt020-02.h
 *
 *  Created on: May 24, 2025
 *      Author: Hamid
 */

#ifndef INC_ST7789_2x240x240_12BIT_H_
#define INC_ST7789_2x240x240_12BIT_H_

#include "stm32f1xx_hal.h"
#include "fonts.h"
#include <stdint.h>
#include <string.h>

#define ST7789_GPIO_Port GPIOA
#define CS1_PIN_INDEX 3
#define CS2_PIN_INDEX 4
#define DC_PIN_INDEX 6
#define RST_PIN_INDEX 2
#define TFT_BLK_INDEX 1

#define CS1_PIN_HIGH (1U << CS1_PIN_INDEX)
#define CS2_PIN_HIGH (1U << CS2_PIN_INDEX)
#define DC_PIN_HIGH (1U << DC_PIN_INDEX)
#define RST_PIN_HIGH (1U << RST_PIN_INDEX)
#define TFT_BLK_HIGH (1U << TFT_PIN_INDEX)

#define CS1_PIN_LOW (1U << (CS1_PIN_INDEX + 16))
#define CS2_PIN_LOW (1U << (CS2_PIN_INDEX + 16))
#define DC_PIN_LOW (1U << (DC_PIN_INDEX + 16))
#define RST_PIN_LOW (1U << (RST_PIN_INDEX + 16))
#define TFT_BLK_LOW (1U << (TFT_PIN_INDEX + 16))

/*DEBUG defines
 */
#define CS_Pin TFT1_CS_Pin
#define CS_GPIO_Port GPIOA
#define DC_Pin TFT_DC_Pin
#define DC_GPIO_Port GPIOA
#define RST_Pin TFT_RST_Pin
#define RST_GPIO_Port GPIOA

// RGB565 Colors
/*#define BLACK   0x0000
#define WHITE   0xFFFF
#define RED     0xF800
#define GREEN   0x07E0
#define BLUE    0x001F
#define CYAN    0x07FF
#define YELLOW  0xFFC0
#define PURPLE  0xF81F
#define PINK    0xD252
*/
// LCD Command Definitions
#define ST7789_SWRST  0x01
#define ST7789_SLPOUT 0x11
#define ST7789_COLMOD 0x3A
#define ST7789_MADCTL 0x36
#define ST7789_CASET  0x2A
#define ST7789_RASET  0x2B
#define ST7789_INVON  0x21
#define ST7789_NORON  0x13
#define ST7789_RAMWR  0x2C
#define ST7789_DISPON 0x29

#define ST7789_XSTART 0x0000
#define ST7789_YSTART 0x0000

#define LCD_WIDTH     ((uint16_t)240)
#define LCD_HEIGHT    ((uint16_t)240)
#define LCD_SIZE      ((uint32_t)57600) //LCD_WIDTH * LCD_HEIGHT

//RGB444 Colors
#define BLACK   0x000
#define WHITE   0xFFF
#define RED     0xF00
#define GREEN   0x0F0
#define BLUE    0x00F
#define CYAN    0x0FF
#define YELLOW  0xFF0
#define PURPLE  0xF0F

extern SPI_HandleTypeDef hspi1;

extern uint16_t colors[];

// LCD Driver API
static inline void DATA_MODE(uint8_t pin_index) {
	HAL_GPIO_WritePin(ST7789_GPIO_Port, (1U << pin_index), 0); //CS##s##_PIN_INDEX
	HAL_GPIO_WritePin(ST7789_GPIO_Port, (1U << DC_PIN_INDEX), 1);
} //(ST7789_GPIO_Port->BSRR = CS##s##_PIN_LOW | DC_PIN_HIGH)
static inline void COMMAND_MODE(uint8_t pin_index) {
	HAL_GPIO_WritePin(ST7789_GPIO_Port, (1U << pin_index), 0); //CS##s##_PIN_INDEX
	HAL_GPIO_WritePin(ST7789_GPIO_Port, (1U << DC_PIN_INDEX), 0);
} //(ST7789_GPIO_Port->BSRR = CS##s##_PIN_LOW | DC_PIN_LOW)
static inline void COMM_END(uint8_t pin_index) {HAL_GPIO_WritePin(ST7789_GPIO_Port, (1U << pin_index), 1);} //(ST7789_GPIO_Port->BSRR = CS##s##_PIN_HIGH)


void ST7789_init(void);
void ST7789_write(uint8_t data);
void ST7789_write_16bit(uint16_t *data);

void ST7789_write_command(void);
void ST7789_write_data(void);
void ST7789_end_write(void);

void ST7789_set_addr(uint16_t x0, uint16_t x1, uint16_t y0, uint16_t y1);
void ST7789_draw_pixel(uint16_t x, uint16_t y, uint16_t colour);
void ST7789_draw_char(uint16_t x, uint16_t y, uint16_t colour, char letter);
void ST7789_draw_string(uint16_t x, uint16_t y, uint16_t colour, char* m);
void ST7789_clear_screen(uint8_t pin_index,uint16_t colour);

#endif /* INC_GMT020_02_H_ */
