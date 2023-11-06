#include <stdio.h>

#include "pico/stdlib.h"
#include <pico/multicore.h>
#include <hardware/gpio.h>
#include <hardware/spi.h>

#include "lcd_display.h"

#define LED_PIN 16

void blink();


int main(){
    stdio_init_all();
    multicore_launch_core1(blink);

    spi_init(spi1, LCD_SPI_FREQUENCY);
    gpio_set_function(LCD_CLK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(LCD_MOSI_PIN, GPIO_FUNC_SPI);

    LCD_Init(HORIZONTAL);
    LCD_Clear(0x00);
    LCD_brightness(1);

    
    while (1){
        sleep_ms(250);
    }
    
    return 0;
}



void blink(){
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    while (1){
        gpio_put(LED_PIN, !gpio_get(LED_PIN));
        sleep_ms(250);
    }
}
