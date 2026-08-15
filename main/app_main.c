#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "GSM_VALVE";

void app_main(void)
{
    ESP_LOGI(TAG, "GSM-VALVE firmware starting");

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}