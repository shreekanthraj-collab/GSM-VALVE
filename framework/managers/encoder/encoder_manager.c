#include "encoder_manager.h"

#include "drv_as5600.h"


static bool s_initialized = false;
static bool s_valid = false;

static uint16_t s_previous_angle = 0U;
static uint16_t s_current_angle = 0U;

static int32_t s_rotation_count = 0;
static int64_t s_total_angle = 0;


/* -------------------------------------------------------------------------- */
/* Validation                                                                 */
/* -------------------------------------------------------------------------- */

static bool valid_angle(uint16_t angle)
{
    return angle <= ENCODER_MANAGER_ANGLE_MAX;
}


/* -------------------------------------------------------------------------- */
/* Wrap-aware delta                                                           */
/* -------------------------------------------------------------------------- */

static int32_t calculate_delta(
    uint16_t previous,
    uint16_t current)
{
    int32_t delta =
        (int32_t)current - (int32_t)previous;

    if (delta > (int32_t)ENCODER_MANAGER_HALF_RANGE) {
        delta -= (int32_t)(ENCODER_MANAGER_ANGLE_MAX + 1U);
    }
    else if (delta < -(int32_t)ENCODER_MANAGER_HALF_RANGE) {
        delta += (int32_t)(ENCODER_MANAGER_ANGLE_MAX + 1U);
    }

    return delta;
}


/* -------------------------------------------------------------------------- */
/* Initialization                                                              */
/* -------------------------------------------------------------------------- */

esp_err_t encoder_manager_init(void)
{
    if (!drv_as5600_is_initialized()) {
        return ESP_ERR_INVALID_STATE;
    }

    s_initialized = true;
    s_valid = false;

    s_previous_angle = 0U;
    s_current_angle = 0U;

    s_rotation_count = 0;
    s_total_angle = 0;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Update                                                                      */
/* -------------------------------------------------------------------------- */

esp_err_t encoder_manager_update(
    uint16_t angle)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!valid_angle(angle)) {
        return ESP_ERR_INVALID_ARG;
    }

    /*
     * First sample establishes the reference.
     * No movement is accumulated from the initial sample.
     */
    if (!s_valid) {
        s_previous_angle = angle;
        s_current_angle = angle;
        s_valid = true;

        return ESP_OK;
    }

    int32_t delta =
        calculate_delta(
            s_previous_angle,
            angle);

    s_previous_angle = angle;
    s_current_angle = angle;

    s_total_angle += (int64_t)delta;

    /*
     * One complete revolution is 4096 encoder counts.
     */
    s_rotation_count =
        (int32_t)(
            s_total_angle /
            (int64_t)(ENCODER_MANAGER_ANGLE_MAX + 1U));

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* State                                                                       */
/* -------------------------------------------------------------------------- */

esp_err_t encoder_manager_get_state(
    encoder_manager_state_t *state)
{
    if (state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    state->angle = s_current_angle;
    state->rotation_count = s_rotation_count;
    state->total_angle = s_total_angle;

    state->total_turns =
        (float)s_total_angle /
        (float)(ENCODER_MANAGER_ANGLE_MAX + 1U);

    state->valid = s_valid;

    return ESP_OK;
}


bool encoder_manager_is_valid(void)
{
    return s_valid;
}


/* -------------------------------------------------------------------------- */
/* Reset                                                                       */
/* -------------------------------------------------------------------------- */

esp_err_t encoder_manager_reset(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    s_valid = false;

    s_previous_angle = 0U;
    s_current_angle = 0U;

    s_rotation_count = 0;
    s_total_angle = 0;

    return ESP_OK;
}
