#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "hal_rtc.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    hal_rtc_time_t time;
    uint32_t timestamp;
    bool valid;
} rtc_manager_state_t;

/*
 * Initialize the RTC manager.
 *
 * The underlying HAL I2C bus must already be initialized.
 */
esp_err_t rtc_manager_init(void);

/*
 * Read the current RTC date/time.
 */
esp_err_t rtc_manager_get_time(
    hal_rtc_time_t *time);

/*
 * Set the RTC date/time.
 */
esp_err_t rtc_manager_set_time(
    const hal_rtc_time_t *time);

/*
 * Read the current RTC Unix timestamp.
 */
esp_err_t rtc_manager_get_timestamp(
    uint32_t *timestamp);

/*
 * Set the RTC from a Unix timestamp.
 */
esp_err_t rtc_manager_set_timestamp(
    uint32_t timestamp);

/*
 * Read the current RTC state.
 */
esp_err_t rtc_manager_get_state(
    rtc_manager_state_t *state);

/*
 * Read the configured hardware alarm.
 */
esp_err_t rtc_manager_get_alarm(
    hal_rtc_alarm_t *alarm);

/*
 * Configure the hardware alarm.
 */
esp_err_t rtc_manager_set_alarm(
    const hal_rtc_alarm_t *alarm);

/*
 * Disable the hardware alarm and clear its flag.
 */
esp_err_t rtc_manager_clear_alarm(void);

/*
 * Check whether the RTC manager is initialized.
 */
bool rtc_manager_is_initialized(void);

/*
 * Check whether the RTC currently provides a valid time.
 */
bool rtc_manager_is_valid(void);

#ifdef __cplusplus
}
#endif