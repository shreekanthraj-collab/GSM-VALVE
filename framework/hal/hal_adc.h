#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    adc_unit_t unit;
    adc_channel_t channel;

    adc_atten_t attenuation;
    adc_bitwidth_t bitwidth;
} hal_adc_config_t;


/*
 * Initialize one ADC channel.
 *
 * One HAL owner is permitted per ADC unit/channel pair.
 */
esp_err_t hal_adc_init(
    const hal_adc_config_t *config);


/*
 * Deinitialize an ADC unit.
 *
 * All channels owned by this HAL unit must be released
 * before the ADC unit is deinitialized.
 */
esp_err_t hal_adc_deinit(
    adc_unit_t unit);


/*
 * Read the raw ADC conversion result.
 */
esp_err_t hal_adc_read_raw(
    adc_unit_t unit,
    adc_channel_t channel,
    int *raw_value);


/*
 * Read the calibrated ADC voltage in millivolts.
 *
 * Returns ESP_ERR_NOT_SUPPORTED when the selected
 * calibration scheme is unavailable.
 */
esp_err_t hal_adc_read_mv(
    adc_unit_t unit,
    adc_channel_t channel,
    int *millivolts);


/*
 * Check whether an ADC channel is initialized.
 */
bool hal_adc_is_initialized(
    adc_unit_t unit,
    adc_channel_t channel);

#ifdef __cplusplus
}
#endif
