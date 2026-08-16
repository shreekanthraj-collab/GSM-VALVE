#include "rtc_manager.h"

#include <stddef.h>


/* -------------------------------------------------------------------------- */
/* Internal state                                                             */
/* -------------------------------------------------------------------------- */

static bool s_initialized = false;
static bool s_valid = false;


/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

esp_err_t rtc_manager_init(void)
{
    if (s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err =
        hal_rtc_init();

    if (err != ESP_OK) {
        s_initialized = false;
        s_valid = false;

        return err;
    }

    s_initialized = true;
    s_valid = false;

    /*
     * Validate that the RTC can provide a coherent
     * date/time immediately after initialization.
     */
    hal_rtc_time_t time = {0};

    err =
        hal_rtc_get_time(&time);

    if (err != ESP_OK) {
        s_valid = false;
        return ESP_OK;
    }

    s_valid = true;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Time                                                                       */
/* -------------------------------------------------------------------------- */

esp_err_t rtc_manager_get_time(
    hal_rtc_time_t *time)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (time == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err =
        hal_rtc_get_time(time);

    if (err != ESP_OK) {
        s_valid = false;
        return err;
    }

    s_valid = true;

    return ESP_OK;
}


esp_err_t rtc_manager_set_time(
    const hal_rtc_time_t *time)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (time == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err =
        hal_rtc_set_time(time);

    if (err != ESP_OK) {
        s_valid = false;
        return err;
    }

    s_valid = true;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Timestamp                                                                  */
/* -------------------------------------------------------------------------- */

esp_err_t rtc_manager_get_timestamp(
    uint32_t *timestamp)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (timestamp == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err =
        hal_rtc_get_timestamp(timestamp);

    if (err != ESP_OK) {
        s_valid = false;
        return err;
    }

    s_valid = true;

    return ESP_OK;
}


esp_err_t rtc_manager_set_timestamp(
    uint32_t timestamp)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err =
        hal_rtc_set_timestamp(timestamp);

    if (err != ESP_OK) {
        s_valid = false;
        return err;
    }

    s_valid = true;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* State                                                                      */
/* -------------------------------------------------------------------------- */

esp_err_t rtc_manager_get_state(
    rtc_manager_state_t *state)
{
    if (state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    state->valid = false;
    state->timestamp = 0U;

    hal_rtc_time_t time = {0};

    esp_err_t err =
        hal_rtc_get_time(&time);

    if (err != ESP_OK) {
        s_valid = false;
        return err;
    }

    uint32_t timestamp = 0U;

    err =
        hal_rtc_get_timestamp(&timestamp);

    if (err != ESP_OK) {
        s_valid = false;
        return err;
    }

    state->time = time;
    state->timestamp = timestamp;
    state->valid = true;

    s_valid = true;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Alarm                                                                      */
/* -------------------------------------------------------------------------- */

esp_err_t rtc_manager_get_alarm(
    hal_rtc_alarm_t *alarm)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (alarm == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return hal_rtc_get_alarm(alarm);
}


esp_err_t rtc_manager_set_alarm(
    const hal_rtc_alarm_t *alarm)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (alarm == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return hal_rtc_set_alarm(alarm);
}


esp_err_t rtc_manager_clear_alarm(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    return hal_rtc_clear_alarm();
}


/* -------------------------------------------------------------------------- */
/* State                                                                      */
/* -------------------------------------------------------------------------- */

bool rtc_manager_is_initialized(void)
{
    return s_initialized;
}


bool rtc_manager_is_valid(void)
{
    return s_valid;
}