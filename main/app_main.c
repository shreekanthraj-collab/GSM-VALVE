#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "hal_i2c.h"
#include "hal_nvs.h"
#include "hal_adc.h"
#include "rtc_manager.h"

#include "drv_as5600.h"

#include "encoder_manager.h"
#include "encoder_persistence_manager.h"
#include "encoder_position_manager.h"

#include "battery_manager.h"


/* -------------------------------------------------------------------------- */
/* Application                                                                */
/* -------------------------------------------------------------------------- */

static const char *TAG = "GSM_VALVE";


/* -------------------------------------------------------------------------- */
/* Battery configuration                                                      */
/* -------------------------------------------------------------------------- */

/*
 * GSM-VALVE hardware:
 *
 * VBAT ADC input:
 *   GPIO1 -> ADC1 channel 0
 *
 * Battery divider:
 *   100 kOhm / 10 kOhm
 *   ratio = (100k + 10k) / 10k = 11.0
 *
 * Battery policy:
 *   critical < 11.2 V
 *   cut      < 11.6 V
 *   low      < 12.0 V
 *   reset    = 12.0 V
 *   high     > 12.4 V
 *
 * The reset threshold is retained as part of the battery policy
 * configuration for the later motor/bypass manager.
 */
static const battery_manager_config_t battery_config = {
    .unit = ADC_UNIT_1,
    .channel = ADC_CHANNEL_0,
    .attenuation = ADC_ATTEN_DB_12,
    .bitwidth = ADC_BITWIDTH_DEFAULT,
    .divider_ratio = 11.0f,
    .critical_voltage_v = 11.2f,
    .cut_voltage_v = 11.6f,
    .low_voltage_v = 12.0f,
    .reset_voltage_v = 12.0f,
    .high_voltage_v = 12.4f
};


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

    /*
     * Initialize the AS5600 driver.
     *
     * The AS5600 uses the shared I2C bus at address 0x36.
     */
    const drv_as5600_config_t as5600_config = {
        .i2c_address = DRV_AS5600_I2C_ADDRESS
    };

    err = drv_as5600_init(&as5600_config);

    if (err != ESP_OK) {
        ESP_LOGE(
            TAG,
            "AS5600 initialization failed: %s",
            esp_err_to_name(err));

        return err;
    }

    ESP_LOGI(
        TAG,
        "AS5600 initialized");

    /*
     * Initialize the wrap-aware encoder manager.
     */
    err = encoder_manager_init();

    if (err != ESP_OK) {
        ESP_LOGE(
            TAG,
            "Encoder manager initialization failed: %s",
            esp_err_to_name(err));

        return err;
    }

    ESP_LOGI(
        TAG,
        "Encoder manager initialized");

    /*
     * Initialize encoder persistence.
     *
     * NVS has already been initialized above.
     */
    err = encoder_persistence_init();

    if (err != ESP_OK) {
        ESP_LOGE(
            TAG,
            "Encoder persistence initialization failed: %s",
            esp_err_to_name(err));

        return err;
    }

    ESP_LOGI(
        TAG,
        "Encoder persistence initialized");

    /*
     * Initialize absolute encoder position management.
     *
     * This depends on:
     *   - AS5600 driver
     *   - encoder manager
     *   - encoder persistence
     */
    err = encoder_position_init();

    if (err != ESP_OK) {
        ESP_LOGE(
            TAG,
            "Encoder position initialization failed: %s",
            esp_err_to_name(err));

        return err;
    }

    ESP_LOGI(
        TAG,
        "Encoder position manager initialized");

    /*
     * Restore the previously persisted absolute position.
     *
     * ESP_ERR_NOT_FOUND is a normal first-boot condition.
     */
    err = encoder_position_restore();

    if (err == ESP_ERR_NOT_FOUND) {

        ESP_LOGI(
            TAG,
            "No persisted encoder position found");

    } else if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "Encoder position restore failed: %s",
            esp_err_to_name(err));

        return err;

    } else {

        ESP_LOGI(
            TAG,
            "Encoder position restored");

    }

    /*
     * Read the first live AS5600 angle.
     *
     * encoder_position_update() deliberately treats this
     * first sample as the new live reference and therefore
     * does not create movement relative to the persisted
     * last_angle.
     */
    uint16_t angle = 0U;

    err = drv_as5600_read_angle(&angle);

    if (err != ESP_OK) {
        ESP_LOGE(
            TAG,
            "Initial AS5600 read failed: %s",
            esp_err_to_name(err));

        return err;
    }

    err = encoder_position_update(angle);

    if (err != ESP_OK) {
        ESP_LOGE(
            TAG,
            "Initial encoder position update failed: %s",
            esp_err_to_name(err));

        return err;
    }

    /*
     * Report the resulting position for diagnostics.
     */
    encoder_position_state_t position = {0};

    err = encoder_position_get_state(&position);

    if (err != ESP_OK) {
        ESP_LOGE(
            TAG,
            "Encoder position state read failed: %s",
            esp_err_to_name(err));

        return err;
    }

    ESP_LOGI(
        TAG,
        "Encoder position: angle=%u total_angle=%lld turns=%.4f restored=%s",
        (unsigned)position.angle,
        (long long)position.total_angle,
        (double)position.total_turns,
        position.restored ? "yes" : "no");

    /*
     * Initialize the battery manager.
     *
     * Hardware mapping:
     *   VBAT = GPIO1 = ADC1 channel 0
     *
     * The battery manager owns this ADC channel.
     */
    err = battery_manager_init(&battery_config);

    if (err != ESP_OK) {
        ESP_LOGE(
            TAG,
            "Battery manager initialization failed: %s",
            esp_err_to_name(err));

        return err;
    }

    ESP_LOGI(
        TAG,
        "Battery manager initialized: GPIO1 / ADC1_CH0");

    /*
     * Take an initial battery measurement.
     *
     * This validates the ADC path during application startup
     * without yet applying motor-control battery policy.
     */
    battery_manager_reading_t battery = {0};

    err = battery_manager_read(&battery);

    if (err != ESP_OK) {
        ESP_LOGE(
            TAG,
            "Initial battery read failed: %s",
            esp_err_to_name(err));

        return err;
    }

    ESP_LOGI(
        TAG,
        "Battery: ADC=%.3f V VBAT=%.3f V state=%d",
        (double)battery.adc_voltage_v,
        (double)battery.battery_voltage_v,
        (int)battery.state);

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
