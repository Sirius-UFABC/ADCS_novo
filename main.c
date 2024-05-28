#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/gpio.h"

#define PIN 2 #pino interno da esp32


void app_main(void)
{
     esp_rom_gpio_pad_select_gpio(PIN);
    gpio_set_direction(PIN, GPIO_MODE_OUTPUT);

    bool status = 0;

    while(1){
        status = !status;
        gpio_set_level(PIN, status);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}