#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "hal_rtc.h"

#ifdef __cplusplus
extern "C" {
#endif


/* -------------------------------------------------------------------------- */
/* Configuration                                                              */
/* -------------------------------------------------------------------------- */

#define SCHEDULING_MAX_SLOTS        8U

#define SCHEDULING_MIN_HOUR         0U
#define SCHEDULING_MAX_HOUR         23U

#define SCHEDULING_MIN_MINUTE       0U
#define SCHEDULING_MAX_MINUTE       59U

/*
 * Weekday mask:
 *
 * bit 0 = Sunday
 * bit 1 = Monday
 * bit 2 = Tuesday
 * bit 3 = Wednesday
 * bit 4 = Thursday
 * bit 5 = Friday
 * bit 6 = Saturday
 */
#define SCHEDULING_WEEKDAY_SUNDAY      (1U << HAL_RTC_WEEKDAY_SUNDAY)
#define SCHEDULING_WEEKDAY_MONDAY      (1U << HAL_RTC_WEEKDAY_MONDAY)
#define SCHEDULING_WEEKDAY_TUESDAY     (1U << HAL_RTC_WEEKDAY_TUESDAY)
#define SCHEDULING_WEEKDAY_WEDNESDAY   (1U << HAL_RTC_WEEKDAY_WEDNESDAY)
#define SCHEDULING_WEEKDAY_THURSDAY    (1U << HAL_RTC_WEEKDAY_THURSDAY)
#define SCHEDULING_WEEKDAY_FRIDAY      (1U << HAL_RTC_WEEKDAY_FRIDAY)
#define SCHEDULING_WEEKDAY_SATURDAY    (1U << HAL_RTC_WEEKDAY_SATURDAY)

#define SCHEDULING_WEEKDAY_ALL         0x7FU


/* -------------------------------------------------------------------------- */
/* Schedule action                                                            */
/* -------------------------------------------------------------------------- */

typedef enum
{
    SCHEDULING_ACTION_OPEN = 0,
    SCHEDULING_ACTION_CLOSE

} scheduling_action_t;


/* -------------------------------------------------------------------------- */
/* Schedule mode                                                              */
/* -------------------------------------------------------------------------- */

typedef enum
{
    /*
     * Execute when the configured weekday and
     * hour/minute match the current RTC time.
     */
    SCHEDULING_MODE_WEEKLY = 0,

    /*
     * Execute once on the configured calendar date
     * at the configured hour/minute.
     */
    SCHEDULING_MODE_ONCE

} scheduling_mode_t;


/* -------------------------------------------------------------------------- */
/* Schedule configuration                                                     */
/* -------------------------------------------------------------------------- */

typedef struct
{
    bool enabled;

    scheduling_action_t action;

    scheduling_mode_t mode;

    uint8_t hour;
    uint8_t minute;

    /*
     * Used by WEEKLY mode.
     *
     * Ignored by ONCE mode.
     */
    uint8_t weekday_mask;

    /*
     * Used by ONCE mode.
     *
     * Ignored by WEEKLY mode.
     */
    uint16_t year;
    uint8_t month;
    uint8_t day;

} scheduling_config_t;


/* -------------------------------------------------------------------------- */
/* Schedule runtime state                                                     */
/* -------------------------------------------------------------------------- */

typedef struct
{
    bool valid;

    bool due;

    esp_err_t last_error;

    uint32_t last_execution_timestamp;

    uint32_t execution_count;

} scheduling_state_t;


/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

/*
 * Initialize the Scheduling Manager.
 *
 * The RTC Manager must already be initialized.
 *
 * All schedule slots are reset to disabled state.
 */
esp_err_t scheduling_manager_init(void);


/*
 * Deinitialize the Scheduling Manager.
 *
 * All schedule configuration and runtime state are cleared.
 */
esp_err_t scheduling_manager_deinit(void);


/* -------------------------------------------------------------------------- */
/* Configuration                                                              */
/* -------------------------------------------------------------------------- */

/*
 * Configure one schedule slot.
 *
 * Valid slots are 0..SCHEDULING_MAX_SLOTS-1.
 *
 * A new configuration clears the previous runtime
 * execution state for that slot.
 */
esp_err_t scheduling_configure(
    uint8_t slot,
    const scheduling_config_t *config);


/*
 * Disable one schedule slot.
 *
 * The existing configuration and runtime state are
 * retained until the slot is reconfigured or the
 * manager is deinitialized.
 */
esp_err_t scheduling_disable(
    uint8_t slot);


/*
 * Get the current configuration of one schedule slot.
 */
esp_err_t scheduling_get_config(
    uint8_t slot,
    scheduling_config_t *config);


/*
 * Check whether one schedule slot is enabled.
 */
bool scheduling_is_enabled(
    uint8_t slot);


/* -------------------------------------------------------------------------- */
/* Scheduling                                                                 */
/* -------------------------------------------------------------------------- */

/*
 * Evaluate one schedule slot against the current RTC time.
 *
 * If the configured schedule is due, the returned action
 * identifies the action that the application layer should
 * perform.
 *
 * This function does NOT control the motor, relay, or actuator.
 */
esp_err_t scheduling_check(
    uint8_t slot,
    scheduling_action_t *action,
    bool *due);


/*
 * Evaluate all enabled schedule slots.
 *
 * The first due schedule is returned.
 *
 * This function does NOT control the motor, relay, or actuator.
 */
esp_err_t scheduling_check_all(
    uint8_t *slot,
    scheduling_action_t *action);


/*
 * Mark a schedule as executed.
 *
 * The caller must invoke this only after the application
 * layer has accepted/started the scheduled action.
 */
esp_err_t scheduling_mark_executed(
    uint8_t slot);


/* -------------------------------------------------------------------------- */
/* State                                                                      */
/* -------------------------------------------------------------------------- */

/*
 * Get the runtime state of one schedule slot.
 */
esp_err_t scheduling_get_state(
    uint8_t slot,
    scheduling_state_t *state);


/*
 * Check whether the Scheduling Manager is initialized.
 */
bool scheduling_manager_is_initialized(void);


/*
 * Check whether the RTC currently provides a valid
 * scheduling time.
 */
bool scheduling_manager_is_time_valid(void);


#ifdef __cplusplus
}
#endif