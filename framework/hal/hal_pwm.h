#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/ledc.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    ledc_mode_t speed_mode;
    ledc_channel_t channel;
    int gpio_num;
    ledc_timer_t timer;
    ledc_timer_bit_t duty_resolution;
    uint32_t frequency_hz;
} hal_pwm_config_t;

/*
 * Initialize one PWM channel.
 *
 * One HAL owner is permitted per PWM channel.
 *
 * A timer may be shared by multiple channels when the
 * frequency and duty resolution are identical.
 */
esp_err_t hal_pwm_init(
    const hal_pwm_config_t *config);

/*
 * Deinitialize a PWM channel.
 *
 * The output is forced low before HAL ownership is released.
 */
esp_err_t hal_pwm_deinit(
    ledc_channel_t channel);

/*
 * Set the configured PWM duty cycle.
 *
 * duty_percent must be between 0.0 and 100.0.
 *
 * The requested duty is retained while the channel is disabled.
 */
esp_err_t hal_pwm_set_duty(
    ledc_channel_t channel,
    float duty_percent);

/*
 * Get the configured PWM duty cycle.
 */
esp_err_t hal_pwm_get_duty(
    ledc_channel_t channel,
    float *duty_percent);

/*
 * Enable PWM output using the configured duty.
 */
esp_err_t hal_pwm_enable(
    ledc_channel_t channel);

/*
 * Disable PWM output.
 *
 * Hardware duty is forced to zero.
 * The configured duty is retained.
 */
esp_err_t hal_pwm_disable(
    ledc_channel_t channel);

/*
 * Check whether a PWM channel is initialized.
 */
bool hal_pwm_is_initialized(
    ledc_channel_t channel);

#ifdef __cplusplus
}
#endif