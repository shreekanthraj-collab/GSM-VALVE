#include "encoder_position_manager.h"

#include <stddef.h>

#include "drv_as5600.h"
#include "encoder_manager.h"
#include "encoder_persistence_manager.h"


static bool s_initialized = false;
static bool s_valid = false;
static bool s_restored = false;

static uint16_t s_current_angle = 0U;

static int64_t s_base_total_angle = 0;
static int64_t s_live_total_angle = 0;


/* -------------------------------------------------------------------------- */
/* Validation                                                                 */
/* -------------------------------------------------------------------------- */

static bool valid_angle(uint16_t angle)
{
    return angle <= ENCODER_POSITION_ANGLE_MAX;
}


/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

esp_err_t encoder_position_init(void)
{
    if (!drv_as5600_is_initialized()) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!encoder_persistence_is_initialized()) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err =
        encoder_manager_init();

    if (err != ESP_OK) {
        return err;
    }

    s_initialized = true;
    s_valid = false;
    s_restored = false;

    s_current_angle = 0U;
    s_base_total_angle = 0;
    s_live_total_angle = 0;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Restore                                                                    */
/* -------------------------------------------------------------------------- */

esp_err_t encoder_position_restore(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    /*
     * Always invalidate the live encoder reference first.
     *
     * The next live AS5600 angle must become the new
     * WPK-017 reference and must not create movement.
     */
    esp_err_t err =
        encoder_manager_reset();

    if (err != ESP_OK) {
        return err;
    }

    encoder_persistence_record_t record = {0};

    err =
        encoder_persistence_load(&record);

    if (err != ESP_OK) {
        s_restored = false;
        s_valid = false;

        s_current_angle = 0U;
        s_base_total_angle = 0;
        s_live_total_angle = 0;

        if (err == ESP_ERR_NOT_FOUND) {
            return ESP_ERR_NOT_FOUND;
        }

        return err;
    }

    /*
     * encoder_persistence_load() has already validated:
     *
     * - record size
     * - version
     * - last_angle
     * - valid flag
     * - checksum
     *
     * Restore only the accumulated absolute position.
     *
     * The persisted last_angle is deliberately NOT compared
     * with the first live post-boot AS5600 sample.
     */
    s_base_total_angle =
        record.total_angle;

    s_live_total_angle = 0;

    s_current_angle =
        record.last_angle;

    s_restored = true;
    s_valid = false;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Update                                                                     */
/* -------------------------------------------------------------------------- */

esp_err_t encoder_position_update(
    uint16_t angle)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!valid_angle(angle)) {
        return ESP_ERR_INVALID_ARG;
    }

    /*
     * The first live sample establishes the WPK-017
     * reference. No movement is accumulated.
     */
    if (!s_valid) {
        esp_err_t err =
            encoder_manager_update(angle);

        if (err != ESP_OK) {
            return err;
        }

        s_current_angle = angle;
        s_live_total_angle = 0;
        s_valid = true;

        return ESP_OK;
    }

    /*
     * Obtain WPK-017 state before the update.
     */
    encoder_manager_state_t before = {0};

    esp_err_t err =
        encoder_manager_get_state(&before);

    if (err != ESP_OK) {
        return err;
    }

    /*
     * Update the frozen WPK-017 live tracker.
     */
    err =
        encoder_manager_update(angle);

    if (err != ESP_OK) {
        return err;
    }

    /*
     * Obtain WPK-017 state after the update.
     */
    encoder_manager_state_t after = {0};

    err =
        encoder_manager_get_state(&after);

    if (err != ESP_OK) {
        return err;
    }

    /*
     * Only the movement detected by WPK-017 is added to
     * the restored absolute position.
     */
    s_live_total_angle +=
        after.total_angle - before.total_angle;

    s_current_angle = angle;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* State                                                                      */
/* -------------------------------------------------------------------------- */

esp_err_t encoder_position_get_state(
    encoder_position_state_t *state)
{
    if (state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    state->angle =
        s_current_angle;

    state->total_angle =
        s_base_total_angle +
        s_live_total_angle;

    state->rotation_count =
        (int32_t)(
            state->total_angle /
            (int64_t)ENCODER_POSITION_FULL_RANGE);

    state->total_turns =
        (float)state->total_angle /
        (float)ENCODER_POSITION_FULL_RANGE;

    state->restored =
        s_restored;

    state->valid =
        s_valid;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Save                                                                      */
/* -------------------------------------------------------------------------- */

esp_err_t encoder_position_save(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!s_valid) {
        return ESP_ERR_INVALID_STATE;
    }

    encoder_position_state_t state = {0};

    esp_err_t err =
        encoder_position_get_state(&state);

    if (err != ESP_OK) {
        return err;
    }

    encoder_persistence_record_t record = {0};

    record.last_angle =
        state.angle;

    record.rotation_count =
        state.rotation_count;

    record.total_angle =
        state.total_angle;

    record.valid = 1U;

    err =
        encoder_persistence_save(&record);

    if (err != ESP_OK) {
        return err;
    }

    /*
     * The current position now exists as a valid
     * persisted position.
     */
    s_restored = true;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Reset                                                                      */
/* -------------------------------------------------------------------------- */

esp_err_t encoder_position_reset(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err =
        encoder_manager_reset();

    if (err != ESP_OK) {
        return err;
    }

    err =
        encoder_persistence_erase();

    if (err != ESP_OK) {
        return err;
    }

    s_current_angle = 0U;
    s_base_total_angle = 0;
    s_live_total_angle = 0;

    s_restored = false;
    s_valid = false;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* State                                                                      */
/* -------------------------------------------------------------------------- */

bool encoder_position_is_initialized(void)
{
    return s_initialized;
}


bool encoder_position_is_valid(void)
{
    return s_valid;
}


bool encoder_position_was_restored(void)
{
    return s_restored;
}