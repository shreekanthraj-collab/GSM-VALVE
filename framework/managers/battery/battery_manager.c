#include "battery_manager.h"

#include <stddef.h>

#include "hal_adc.h"


/* -------------------------------------------------------------------------- */
/* Internal state                                                             */
/* -------------------------------------------------------------------------- */

static bool s_initialized = false;
static bool s_valid = false;

static battery_manager_config_t s_config = {0};

static battery_manager_reading_t s_last_reading = {0};


/* -------------------------------------------------------------------------- */
/* Validation                                                                 */
/* -------------------------------------------------------------------------- */

static bool valid_config(
    const battery_manager_config_t *config)
{
    if (config == NULL) {
        return false;
    }

    if (config->divider_ratio <= 0.0f) {
        return false;
    }

    if (config->critical_voltage_v < 0.0f) {
        return false;
    }

    if (config->cut_voltage_v <
        config->critical_voltage_v) {
        return false;
    }

    if (config->low_voltage_v <
        config->cut_voltage_v) {
        return false;
    }

    if (config->reset_voltage_v <
        config->low_voltage_v) {
        return false;
    }

    if (config->high_voltage_v <
        config->reset_voltage_v) {
        return false;
    }

    return true;
}


/* -------------------------------------------------------------------------- */
/* State classification                                                       */
/* -------------------------------------------------------------------------- */

static battery_manager_state_t classify_voltage(
    float voltage_v)
{
    if (voltage_v < s_config.critical_voltage_v) {
        return BATTERY_STATE_CRITICAL;
    }

    if (voltage_v < s_config.cut_voltage_v) {
        return BATTERY_STATE_CUT;
    }

    if (voltage_v < s_config.low_voltage_v) {
        return BATTERY_STATE_LOW;
    }

    if (voltage_v > s_config.high_voltage_v) {
        return BATTERY_STATE_HIGH;
    }

    return BATTERY_STATE_NORMAL;
}


/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

esp_err_t battery_manager_init(
    const battery_manager_config_t *config)
{
    if (s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!valid_config(config)) {
        return ESP_ERR_INVALID_ARG;
    }

    hal_adc_config_t adc_config = {
        .unit = config->unit,
        .channel = config->channel,
        .attenuation = config->attenuation,
        .bitwidth = config->bitwidth
    };

    esp_err_t err =
        hal_adc_init(&adc_config);

    if (err != ESP_OK) {
        s_initialized = false;
        s_valid = false;

        return err;
    }

    s_config = *config;

    s_last_reading.adc_voltage_v = 0.0f;
    s_last_reading.battery_voltage_v = 0.0f;
    s_last_reading.state = BATTERY_STATE_UNKNOWN;
    s_last_reading.valid = false;

    s_initialized = true;
    s_valid = false;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Reading                                                                    */
/* -------------------------------------------------------------------------- */

esp_err_t battery_manager_read(
    battery_manager_reading_t *reading)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (reading == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int millivolts = 0;

    esp_err_t err =
        hal_adc_read_mv(
            s_config.unit,
            s_config.channel,
            &millivolts);

    if (err != ESP_OK) {
        s_valid = false;
        s_last_reading.valid = false;

        return err;
    }

    if (millivolts < 0) {
        s_valid = false;
        s_last_reading.valid = false;

        return ESP_ERR_INVALID_RESPONSE;
    }

    float adc_voltage_v =
        (float)millivolts / 1000.0f;

    float battery_voltage_v =
        adc_voltage_v *
        s_config.divider_ratio;

    reading->adc_voltage_v =
        adc_voltage_v;

    reading->battery_voltage_v =
        battery_voltage_v;

    reading->state =
        classify_voltage(battery_voltage_v);

    reading->valid = true;

    s_last_reading =
        *reading;

    s_valid = true;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Voltage                                                                    */
/* -------------------------------------------------------------------------- */

esp_err_t battery_manager_get_voltage(
    float *voltage_v)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (voltage_v == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    battery_manager_reading_t reading = {0};

    esp_err_t err =
        battery_manager_read(&reading);

    if (err != ESP_OK) {
        return err;
    }

    *voltage_v =
        reading.battery_voltage_v;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* State                                                                      */
/* -------------------------------------------------------------------------- */

esp_err_t battery_manager_get_state(
    battery_manager_state_t *state)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    battery_manager_reading_t reading = {0};

    esp_err_t err =
        battery_manager_read(&reading);

    if (err != ESP_OK) {
        return err;
    }

    *state =
        reading.state;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Status                                                                     */
/* -------------------------------------------------------------------------- */

bool battery_manager_is_initialized(void)
{
    return s_initialized;
}


bool battery_manager_is_valid(void)
{
    return s_valid;
}
