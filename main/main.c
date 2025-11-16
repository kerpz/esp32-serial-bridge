#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void app_main(void) {
    while(1) {
        printf("Hello from ESP-IDF GitHub Actions!\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}