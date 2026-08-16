#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif


/* -------------------------------------------------------------------------- */
/* Weekday                                                                    */
/* -------------------------------------------------------------------------- */

#define HAL_RTC_WEEKDAY_SUNDAY       0
#define HAL_RTC_WEEKDAY_MONDAY       1
#define HAL_RTC_WEEKDAY_TUESDAY      2
#define HAL_RTC_WEEKDAY_WEDNESDAY    3
#define HAL_RTC_WEEKDAY_THURSDAY     4
#define HAL_RTC_WEEKDAY_FRIDAY       5
#define HAL_RTC_WEEKDAY_SATURDAY     6


/* -------------------------------------------------------------------------- */
/* Date and time                                                              */
/* -------------------------------------------------------------------------- */

typedef struct
{
    uint8_t second;
    uint8_t minute;
    uint8_t hour;

    uint8_t day;
    uint8_t month;
    uint16_t year;

    uint8_t weekday;

} hal_rtc_time_t;


/* -------------------------------------------------------------------------- */
/* Hardware alarm                                                             */
/* -------------------------------------------------------------------------- */

/*
 * PCF8563 hardware alarm.
 *
 * The PCF8563 provides alarm matching for:
 * minute, hour, day and weekday.
 *
 * The PCF8563 does not provide a seconds alarm comparator.
 */
typedef struct
{
    uint8_t minute;
    uint8_t hour;
    uint8_t day;
    uint8_t weekday;

    bool minute_enabled;
    bool hour_enabled;
    bool day_enabled;
    bool weekday_enabled;

    bool enabled;

} hal_rtc_alarm_t;


/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

/*
 * Initialize the PCF8563 RTC.
 *
 * The underlying HAL I2C bus must already be initialized.
 */
esp_err_t hal_rtc_init(void);


/*
 * Deinitialize the RTC HAL.
 *
 * The underlying I2C bus is not deinitialized.
 */
esp_err_t hal_rtc_deinit(void);


/* -------------------------------------------------------------------------- */
/* Time                                                                       */
/* -------------------------------------------------------------------------- */

/*
 * Read the complete RTC date/time.
 */
esp_err_t hal_rtc_get_time(
    hal_rtc_time_t *time);


/*
 * Set the complete RTC date/time.
 */
esp_err_t hal_rtc_set_time(
    const hal_rtc_time_t *time);


/* -------------------------------------------------------------------------- */
/* Timestamp                                                                  */
/* -------------------------------------------------------------------------- */

/*
 * Read the current RTC time as a Unix timestamp.
 */
esp_err_t hal_rtc_get_timestamp(
    uint32_t *timestamp);


/*
 * Set the RTC from a Unix timestamp.
 */
esp_err_t hal_rtc_set_timestamp(
    uint32_t timestamp);


/* -------------------------------------------------------------------------- */
/* Alarm                                                                      */
/* -------------------------------------------------------------------------- */

/*
 * Read the configured hardware alarm.
 */
esp_err_t hal_rtc_get_alarm(
    hal_rtc_alarm_t *alarm);


/*
 * Configure the hardware alarm.
 */
esp_err_t hal_rtc_set_alarm(
    const hal_rtc_alarm_t *alarm);


/*
 * Disable the hardware alarm and clear its alarm flag.
 */
esp_err_t hal_rtc_clear_alarm(void);


/* -------------------------------------------------------------------------- */
/* State                                                                      */
/* -------------------------------------------------------------------------- */

/*
 * Check whether the RTC HAL is initialized.
 */
bool hal_rtc_is_initialized(void);


#ifdef __cplusplus
}
#endif