#include "scheduling_manager.h"

#include <stddef.h>
#include <string.h>

#include "rtc_manager.h"


/* -------------------------------------------------------------------------- */
/* Internal state                                                             */
/* -------------------------------------------------------------------------- */

typedef struct
{
    scheduling_config_t config;
    scheduling_state_t state;

} scheduling_slot_t;


static bool s_initialized = false;

static scheduling_slot_t s_slots[
    SCHEDULING_MAX_SLOTS
];


/* -------------------------------------------------------------------------- */
/* Validation                                                                 */
/* -------------------------------------------------------------------------- */

static bool valid_slot(
    uint8_t slot)
{
    return slot < SCHEDULING_MAX_SLOTS;
}


static bool valid_action(
    scheduling_action_t action)
{
    return action == SCHEDULING_ACTION_OPEN ||
           action == SCHEDULING_ACTION_CLOSE;
}


static bool valid_mode(
    scheduling_mode_t mode)
{
    return mode == SCHEDULING_MODE_WEEKLY ||
           mode == SCHEDULING_MODE_ONCE;
}


static bool valid_weekday_mask(
    uint8_t weekday_mask)
{
    return weekday_mask != 0U &&
           (weekday_mask & (uint8_t)~SCHEDULING_WEEKDAY_ALL) == 0U;
}


static bool valid_date(
    uint16_t year,
    uint8_t month,
    uint8_t day)
{
    if (year < 2000U ||
        year > 2099U) {
        return false;
    }

    if (month < 1U ||
        month > 12U) {
        return false;
    }

    static const uint8_t days_in_month[] = {
        31U,
        28U,
        31U,
        30U,
        31U,
        30U,
        31U,
        31U,
        30U,
        31U,
        30U,
        31U
    };

    uint8_t max_day =
        days_in_month[month - 1U];

    if (month == 2U &&
        (year % 4U) == 0U &&
        ((year % 100U) != 0U ||
         (year % 400U) == 0U)) {

        max_day = 29U;
    }

    return day >= 1U &&
           day <= max_day;
}


static bool valid_config(
    const scheduling_config_t *config)
{
    if (config == NULL) {
        return false;
    }

    if (!valid_action(config->action)) {
        return false;
    }

    if (!valid_mode(config->mode)) {
        return false;
    }

    if (config->hour > SCHEDULING_MAX_HOUR) {
        return false;
    }

    if (config->minute > SCHEDULING_MAX_MINUTE) {
        return false;
    }

    if (config->mode == SCHEDULING_MODE_WEEKLY) {

        if (!valid_weekday_mask(
                config->weekday_mask)) {

            return false;
        }

    } else {

        if (!valid_date(
                config->year,
                config->month,
                config->day)) {

            return false;
        }
    }

    return true;
}


/* -------------------------------------------------------------------------- */
/* Time matching                                                              */
/* -------------------------------------------------------------------------- */

static bool weekly_match(
    const scheduling_config_t *config,
    const hal_rtc_time_t *time)
{
    if (config == NULL ||
        time == NULL) {
        return false;
    }

    if (time->weekday > HAL_RTC_WEEKDAY_SATURDAY) {
        return false;
    }

    uint8_t weekday_bit =
        (uint8_t)(1U << time->weekday);

    if ((config->weekday_mask & weekday_bit) == 0U) {
        return false;
    }

    return time->hour == config->hour &&
           time->minute == config->minute;
}


static bool once_match(
    const scheduling_config_t *config,
    const hal_rtc_time_t *time)
{
    if (config == NULL ||
        time == NULL) {
        return false;
    }

    return time->year == config->year &&
           time->month == config->month &&
           time->day == config->day &&
           time->hour == config->hour &&
           time->minute == config->minute;
}


/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

esp_err_t scheduling_manager_init(void)
{
    if (s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!rtc_manager_is_initialized()) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(
        s_slots,
        0,
        sizeof(s_slots));

    for (uint8_t slot = 0U;
         slot < SCHEDULING_MAX_SLOTS;
         slot++) {

        s_slots[slot].config.enabled = false;

        s_slots[slot].config.action =
            SCHEDULING_ACTION_CLOSE;

        s_slots[slot].config.mode =
            SCHEDULING_MODE_WEEKLY;

        s_slots[slot].config.hour = 0U;
        s_slots[slot].config.minute = 0U;

        s_slots[slot].config.weekday_mask =
            SCHEDULING_WEEKDAY_ALL;

        s_slots[slot].state.valid = false;
        s_slots[slot].state.due = false;
        s_slots[slot].state.last_error = ESP_OK;
        s_slots[slot].state.last_execution_timestamp = 0U;
        s_slots[slot].state.execution_count = 0U;
    }

    s_initialized = true;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Deinitialization                                                           */
/* -------------------------------------------------------------------------- */

esp_err_t scheduling_manager_deinit(void)
{
    if (!s_initialized) {
        return ESP_OK;
    }

    memset(
        s_slots,
        0,
        sizeof(s_slots));

    s_initialized = false;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Configuration                                                              */
/* -------------------------------------------------------------------------- */

esp_err_t scheduling_configure(
    uint8_t slot,
    const scheduling_config_t *config)
{
    if (!valid_slot(slot)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!valid_config(config)) {
        return ESP_ERR_INVALID_ARG;
    }

    s_slots[slot].config = *config;

    s_slots[slot].state.valid = false;
    s_slots[slot].state.due = false;
    s_slots[slot].state.last_error = ESP_OK;
    s_slots[slot].state.last_execution_timestamp = 0U;
    s_slots[slot].state.execution_count = 0U;

    return ESP_OK;
}


esp_err_t scheduling_disable(
    uint8_t slot)
{
    if (!valid_slot(slot)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    s_slots[slot].config.enabled = false;

    s_slots[slot].state.due = false;
    s_slots[slot].state.last_error = ESP_OK;

    return ESP_OK;
}


esp_err_t scheduling_get_config(
    uint8_t slot,
    scheduling_config_t *config)
{
    if (!valid_slot(slot) ||
        config == NULL) {

        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    *config = s_slots[slot].config;

    return ESP_OK;
}


bool scheduling_is_enabled(
    uint8_t slot)
{
    if (!valid_slot(slot)) {
        return false;
    }

    if (!s_initialized) {
        return false;
    }

    return s_slots[slot].config.enabled;
}


/* -------------------------------------------------------------------------- */
/* Scheduling                                                                 */
/* -------------------------------------------------------------------------- */

esp_err_t scheduling_check(
    uint8_t slot,
    scheduling_action_t *action,
    bool *due)
{
    if (!valid_slot(slot)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (action == NULL ||
        due == NULL) {

        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    *action = SCHEDULING_ACTION_CLOSE;
    *due = false;

    scheduling_slot_t *schedule =
        &s_slots[slot];

    schedule->state.due = false;

    if (!schedule->config.enabled) {
        schedule->state.valid = true;
        schedule->state.last_error = ESP_OK;

        return ESP_OK;
    }

    if (!rtc_manager_is_valid()) {
        schedule->state.valid = false;
        schedule->state.last_error =
            ESP_ERR_INVALID_STATE;

        return ESP_ERR_INVALID_STATE;
    }

    hal_rtc_time_t time = {0};

    esp_err_t err =
        rtc_manager_get_time(&time);

    if (err != ESP_OK) {
        schedule->state.valid = false;
        schedule->state.last_error = err;

        return err;
    }

    bool matched = false;

    if (schedule->config.mode ==
        SCHEDULING_MODE_WEEKLY) {

        matched =
            weekly_match(
                &schedule->config,
                &time);

    } else {

        matched =
            once_match(
                &schedule->config,
                &time);
    }

    /*
     * Prevent a schedule from firing repeatedly during
     * multiple scheduler checks within the same minute.
     */
    uint32_t timestamp = 0U;

    err =
        rtc_manager_get_timestamp(&timestamp);

    if (err != ESP_OK) {
        schedule->state.valid = false;
        schedule->state.last_error = err;

        return err;
    }

    if (matched &&
        schedule->state.last_execution_timestamp != 0U) {

        uint32_t last_timestamp =
            schedule->state.last_execution_timestamp;

        if (timestamp >= last_timestamp &&
            (timestamp - last_timestamp) < 60U) {

            matched = false;
        }
    }

    /*
     * A one-shot schedule is permanently consumed after
     * successful execution.
     */
    if (schedule->config.mode ==
            SCHEDULING_MODE_ONCE &&
        schedule->state.execution_count > 0U) {

        matched = false;
    }

    schedule->state.valid = true;
    schedule->state.due = matched;
    schedule->state.last_error = ESP_OK;

    if (matched) {
        *action = schedule->config.action;
        *due = true;
    }

    return ESP_OK;
}


esp_err_t scheduling_check_all(
    uint8_t *slot,
    scheduling_action_t *action)
{
    if (slot == NULL ||
        action == NULL) {

        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    *slot = 0U;
    *action = SCHEDULING_ACTION_CLOSE;

    for (uint8_t index = 0U;
         index < SCHEDULING_MAX_SLOTS;
         index++) {

        if (!s_slots[index].config.enabled) {
            continue;
        }

        bool due = false;

        esp_err_t err =
            scheduling_check(
                index,
                action,
                &due);

        if (err != ESP_OK) {
            return err;
        }

        if (due) {
            *slot = index;
            return ESP_OK;
        }
    }

    return ESP_OK;
}


esp_err_t scheduling_mark_executed(
    uint8_t slot)
{
    if (!valid_slot(slot)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    scheduling_slot_t *schedule =
        &s_slots[slot];

    if (!schedule->config.enabled) {
        return ESP_ERR_INVALID_STATE;
    }

    uint32_t timestamp = 0U;

    esp_err_t err =
        rtc_manager_get_timestamp(&timestamp);

    if (err != ESP_OK) {
        schedule->state.last_error = err;
        return err;
    }

    schedule->state.last_execution_timestamp =
        timestamp;

    schedule->state.execution_count++;

    schedule->state.due = false;
    schedule->state.valid = true;
    schedule->state.last_error = ESP_OK;

    /*
     * ONCE schedules are automatically disabled after
     * successful execution.
     */
    if (schedule->config.mode ==
        SCHEDULING_MODE_ONCE) {

        schedule->config.enabled = false;
    }

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* State                                                                      */
/* -------------------------------------------------------------------------- */

/*
 * Get the runtime state of one schedule slot.
 */
esp_err_t scheduling_get_state(
    uint8_t slot,
    scheduling_state_t *state)
{
    if (!valid_slot(slot) ||
        state == NULL) {

        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    *state = s_slots[slot].state;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Manager state                                                              */
/* -------------------------------------------------------------------------- */

bool scheduling_manager_is_initialized(void)
{
    return s_initialized;
}


bool scheduling_manager_is_time_valid(void)
{
    if (!s_initialized) {
        return false;
    }

    return rtc_manager_is_valid();
}