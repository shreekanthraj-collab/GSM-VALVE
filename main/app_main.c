#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "hal_nvs.h"


/* -------------------------------------------------------------------------- */
/* Application                                                                */
/* -------------------------------------------------------------------------- */

static const char *TAG = "GSM_VALVE";


/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

static esp_err_t app_init(void)
{
    esp_err_t err;

    /*
     * Initialize persistent storage first.
     */
    err = hal_nvs_init();

    if (err != ESP_OK) {
        ESP_LOGE(
            TAG,
            "NVS initialization failed: %s",
            esp_err_to_name(err));

        return err;
    }

    ESP_LOGI(
        TAG,
        "NVS initialized");

    /*
     * Additional hardware and managers will be
     * initialized here in their dependency order.
     *
     * I2C and VBAT ADC initialization are intentionally
     * deferred until the board-specific GPIO mapping is
     * finalized.
     */

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Application entry                                                          */
/* -------------------------------------------------------------------------- */

void app_main(void)
{
    ESP_LOGI(
        TAG,
        "GSM-VALVE firmware starting");

    esp_err_t err = app_init();

    if (err != ESP_OK) {
        ESP_LOGE(
            TAG,
            "Application initialization failed: %s",
            esp_err_to_name(err));

        /*
         * Do not continue into normal application
         * runtime after mandatory initialization fails.
         */
        while (1) {
            vTaskDelay(
                pdMS_TO_TICKS(1000));
        }
    }

    ESP_LOGI(
        TAG,
        "Application initialization complete");

    while (1) {
        vTaskDelay(
            pdMS_TO_TICKS(1000));
    }
}