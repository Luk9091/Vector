#include <stdio.h>

#include "pico/stdlib.h"
#include <pico/multicore.h>
#include <hardware/gpio.h>

#define LED_PIN 0

void blink(){
    multicore_lockout_victim_init();
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    // uint stop = 0;

    while (1){
        gpio_put(LED_PIN, !gpio_get(LED_PIN));
        sleep_ms(250);
    }
}


int main(){
    stdio_init_all();
    multicore_launch_core1(blink);

    while (1){
        sleep_ms(500);
    }
    
    return 0;
}