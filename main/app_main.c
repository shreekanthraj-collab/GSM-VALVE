#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "hal_i2c.h"
#include "hal_nvs.h"
#include "rtc_manager.h"


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
     * Initialize the I2C bus.
     *
     * Board GPIO mapping:
     *   SDA = GPIO 8
     *   SCL = GPIO 9
     *
     * The I2C HAL owns the physical bus.
     */
    const hal_i2c_config_t i2c_config = {
        .frequency_hz = 100000U,
        .timeout_ms = 1000U
    };

    err = hal_i2c_init(&i2c_config);

    if (err != ESP_OK) {
        ESP_LOGE(
            TAG,
            "I2C initialization failed: %s",
            esp_err_to_name(err));

        return err;
    }

    ESP_LOGI(
        TAG,
        "I2C initialized: 100 kHz");

    /*
     * Initialize the RTC manager.
     *
     * The RTC manager depends on the already initialized
     * I2C HAL.
     */
    err = rtc_manager_init();

    if (err != ESP_OK) {
        ESP_LOGE(
            TAG,
            "RTC initialization failed: %s",
            esp_err_to_name(err));

        return err;
    }

    ESP_LOGI(
        TAG,
        "RTC manager initialized");

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