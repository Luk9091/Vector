#ifndef __LCD_H
#define __LCD_H

#include <stdio.h>
#include <pico/stdlib.h>

#include <hardware/spi.h>
#include <hardware/pwm.h>

#define LCD_HEIGHT 240
#define LCD_WIDTH 240

#define ANALOG_BRIGHTNESS false

#define LCD_WIDTH_Byte 240

#define LCD_SPI_FREQUENCY 10000 * 1000

#define LCD_RST_PIN 12
#define LCD_DC_PIN  8
#define LCD_BL_PIN	13

#define LCD_CS_PIN   9
#define LCD_CLK_PIN  10
#define LCD_MOSI_PIN 11

#define HORIZONTAL 0
#define VERTICAL   1



void LCD_Init(uint8_t Scan_dir);

void LCD_brightness(uint value);

void LCD_Clear(uint8_t Color);
void LCD_Display();
void LCD_DisplayWindows(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t *Image);
void LCD_DisplayPoint(uint8_t X, uint8_t Y, uint8_t Color);

void Handler_1in54_LCD(int signo);
#endif
