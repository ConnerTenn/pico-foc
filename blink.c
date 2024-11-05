
// #include <pico/stdlib.h>
#include "pico/stdlib.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <hardware/gpio.h>

#define LED_PIN PICO_DEFAULT_LED_PIN
#define LED_DELAY_MS 250

int main()
{
    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    while (true)
    {
        printf("Pico!");
        gpio_put(PICO_DEFAULT_LED_PIN, true);
        sleep_ms(LED_DELAY_MS);
        gpio_put(PICO_DEFAULT_LED_PIN, false);
        sleep_ms(LED_DELAY_MS);
    }

    return 0;
}

