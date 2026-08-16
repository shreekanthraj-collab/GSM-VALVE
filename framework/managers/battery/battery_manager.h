#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    BATTERY_STATE_UNKNOWN = 0,
    BATTERY_STATE_CRITICAL,
    BATTERY_STATE_CUT,
    BATTERY_STATE_LOW,
    BATTERY_STATE_NORMAL,
    BATTERY_STATE_HIGH
} battery_manager_state_t;

typedef struct
{
    adc_unit_t unit;
    adc_channel_t channel;

    adc_atten_t attenuation;
    adc_bitwidth_t bitwidth;

    /*
     * Battery voltage = ADC input voltage * divider_ratio.
     *
     * Example:
     * 10:1 divider -> 10.0f
     */
    float divider_ratio;

    float critical_voltage_v;
    float cut_voltage_v;
    float low_voltage_v;
    float reset_voltage_v;
    float high_voltage_v;
} battery_manager_config_t;

typedef struct
{
    float adc_voltage_v;
    float battery_voltage_v;

    battery_manager_state_t state;

    bool valid;
} battery_manager_reading_t;

/*
 * Initialize the battery manager.
 *
 * The supplied ADC configuration is passed to the
 * frozen WPK ADC HAL.
 */
esp_err_t battery_manager_init(
    const battery_manager_config_t *config);

/*
 * Read the current battery voltage and state.
 */
esp_err_t battery_manager_read(
    battery_manager_reading_t *reading);

/*
 * Read only the battery voltage.
 */
esp_err_t battery_manager_get_voltage(
    float *voltage_v);

/*
 * Get the current battery state.
 *
 * Performs a fresh ADC reading.
 */
esp_err_t battery_manager_get_state(
    battery_manager_state_t *state);

/*
 * Check whether the manager is initialized.
 */
bool battery_manager_is_initialized(void);

/*
 * Check whether the most recent battery reading is valid.
 */
bool battery_manager_is_valid(void);

#ifdef __cplusplus
}
#endif
