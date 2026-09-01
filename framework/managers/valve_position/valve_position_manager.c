#include "valve_position_manager.h"

#include "actuator_manager.h"
#include "encoder_position_manager.h"
#include "valve_position_persistence.h"


/* -------------------------------------------------------------------------- */
/* Internal state                                                             */
/* -------------------------------------------------------------------------- */

static bool s_initialized = false;
static bool s_calibrated = false;
static bool s_closed_calibration_pending = false;

static int64_t s_closed_total_angle = 0;
static int64_t s_open_total_angle = 0;

static valve_position_status_t s_status;


/* -------------------------------------------------------------------------- */
/* Constants                                                                  */
/* -------------------------------------------------------------------------- */

/*
 * Encoder-angle tolerance used when deciding that a target has been reached.
 *
 * AS5600 resolution is 4096 counts per revolution. A small tolerance avoids
 * requiring an exact integer encoder position at the target.
 */
#define VALVE_POSITION_TARGET_TOLERANCE_ANGLE 2


/* -------------------------------------------------------------------------- */
/* Internal helpers                                                           */
/* -------------------------------------------------------------------------- */

static bool valid_position_percent(
    valve_position_percent_t percent);

static int64_t calculate_target_total_angle(
    valve_position_percent_t percent);


/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

esp_err_t valve_position_manager_init(void)
{
    if (s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!encoder_position_is_initialized()) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!valve_position_persistence_is_initialized()) {
        return ESP_ERR_INVALID_STATE;
    }

    s_status =
        (valve_position_status_t){0};

    s_status.target_percent =
        VALVE_POSITION_0_PERCENT;

    s_initialized = true;
    s_calibrated = false;
    s_closed_calibration_pending = false;

    s_closed_total_angle = 0;
    s_open_total_angle = 0;

    valve_position_persistence_record_t record =
        {0};

    esp_err_t err =
        valve_position_persistence_load(
            &record);

    if (err == ESP_OK) {

        s_closed_total_angle =
            record.closed_total_angle;

        s_open_total_angle =
            record.open_total_angle;

        s_calibrated =
            record.valid != 0U;

        s_closed_calibration_pending =
            false;

        return ESP_OK;
    }

    if (err == ESP_ERR_NOT_FOUND) {
        return ESP_OK;
    }

    /*
     * A corrupt or incompatible calibration record
     * does not make the manager unusable.
     *
     * The actuator simply remains uncalibrated.
     */
    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Calibration                                                                */
/* -------------------------------------------------------------------------- */

esp_err_t valve_position_calibrate_closed(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    /*
     * Calibration is only permitted while the actuator is stopped.
     */
    if (actuator_manager_is_running()) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!encoder_position_is_valid()) {
        return ESP_ERR_INVALID_STATE;
    }

    encoder_position_state_t position =
        {0};

    esp_err_t err =
        encoder_position_get_state(
            &position);

    if (err != ESP_OK) {
        return err;
    }

    /*
     * Keep the new CLOSED point in RAM only.
     *
     * The persisted calibration is not modified until a valid
     * OPEN point is subsequently recorded.
     */
    s_closed_total_angle =
        position.total_angle;

    s_closed_calibration_pending =
        true;

    s_calibrated =
        false;

    /*
     * A new calibration sequence invalidates any active
     * position target.
     */
    s_status.target_active =
        false;

    s_status.moving =
        false;

    s_status.valid =
        false;

    s_status.current_total_angle =
        position.total_angle;

    return ESP_OK;
}


esp_err_t valve_position_calibrate_open(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    /*
     * OPEN calibration must be the second step of a fresh
     * CLOSED -> OPEN calibration sequence.
     */
    if (!s_closed_calibration_pending) {
        return ESP_ERR_INVALID_STATE;
    }

    /*
     * Calibration is only permitted while the actuator is stopped.
     */
    if (actuator_manager_is_running()) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!encoder_position_is_valid()) {
        return ESP_ERR_INVALID_STATE;
    }

    encoder_position_state_t position =
        {0};

    esp_err_t err =
        encoder_position_get_state(
            &position);

    if (err != ESP_OK) {
        return err;
    }

    const int64_t open_total_angle =
        position.total_angle;

    /*
     * OPEN must be mechanically above CLOSED in absolute
     * encoder space because target calculation assumes this
     * orientation.
     */
    if (open_total_angle <=
        s_closed_total_angle) {

        return ESP_ERR_INVALID_STATE;
    }

    valve_position_persistence_record_t record =
        {0};

    record.version =
        VALVE_POSITION_PERSISTENCE_VERSION;

    record.closed_total_angle =
        s_closed_total_angle;

    record.open_total_angle =
        open_total_angle;

    record.valid =
        1U;

    /*
     * The persistence layer calculates and verifies the checksum.
     */
    err =
        valve_position_persistence_save(
            &record);

    if (err != ESP_OK) {
        return err;
    }

    /*
     * Only mark the calibration active after persistence
     * succeeds.
     */
    s_open_total_angle =
        open_total_angle;

    s_calibrated =
        true;

    s_closed_calibration_pending =
        false;

    s_status.target_active =
        false;

    s_status.moving =
        false;

    s_status.valid =
        true;

    s_status.current_total_angle =
        position.total_angle;

    return ESP_OK;
}


esp_err_t valve_position_calibration_clear(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    /*
     * Calibration must never modify an active motor operation.
     */
    if (actuator_manager_is_running()) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err =
        valve_position_persistence_erase();

    if (err != ESP_OK) {
        return err;
    }

    s_closed_total_angle =
        0;

    s_open_total_angle =
        0;

    s_calibrated =
        false;

    s_closed_calibration_pending =
        false;

    s_status =
        (valve_position_status_t){0};

    s_status.target_percent =
        VALVE_POSITION_0_PERCENT;

    return ESP_OK;
}


bool valve_position_manager_is_calibrated(void)
{
    return
        s_initialized &&
        s_calibrated;
}


/* -------------------------------------------------------------------------- */
/* Position command                                                           */
/* -------------------------------------------------------------------------- */

esp_err_t valve_position_set_target(
    valve_position_percent_t percent,
    uint32_t now_ms)
{
    (void)now_ms;

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!s_calibrated) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!valid_position_percent(percent)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!encoder_position_is_valid()) {
        return ESP_ERR_INVALID_STATE;
    }

    /*
     * Obtain the current absolute encoder position.
     */
    encoder_position_state_t position =
        {0};

    esp_err_t err =
        encoder_position_get_state(
            &position);

    if (err != ESP_OK) {
        return err;
    }

    /*
     * Calculate the absolute encoder target
     * from the persisted CLOSED/OPEN calibration.
     */
    s_status.target_percent =
        percent;

    s_status.target_total_angle =
        calculate_target_total_angle(
            percent);

    s_status.current_total_angle =
        position.total_angle;

    s_status.target_active =
        true;

    s_status.moving =
        false;

    s_status.valid =
        true;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Runtime processing                                                         */
/* -------------------------------------------------------------------------- */

esp_err_t valve_position_manager_update(
    uint32_t now_ms)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!s_calibrated) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!s_status.valid) {
        return ESP_ERR_INVALID_STATE;
    }

    /*
     * Nothing to process when there is no active target.
     */
    if (!s_status.target_active) {
        s_status.moving = false;
        return ESP_OK;
    }

    /*
     * The encoder position is the authoritative position source.
     */
    if (!encoder_position_is_valid()) {
        return ESP_ERR_INVALID_STATE;
    }

    encoder_position_state_t position =
        {0};

    esp_err_t err =
        encoder_position_get_state(
            &position);

    if (err != ESP_OK) {
        return err;
    }

    s_status.current_total_angle =
        position.total_angle;

    /*
     * Obtain actuator state.
     */
    actuator_manager_status_t actuator =
        {0};

    err =
        actuator_manager_get_status(
            &actuator);

    if (err != ESP_OK) {
        return err;
    }

    /*
     * Calculate signed position error.
     */
    const int64_t position_error =
        position.total_angle -
        s_status.target_total_angle;

    /*
     * Target reached.
     *
     * A small tolerance is used instead of requiring exact
     * encoder equality.
     */
    if (position_error >=
            -(int64_t)
                VALVE_POSITION_TARGET_TOLERANCE_ANGLE &&
        position_error <=
            (int64_t)
                VALVE_POSITION_TARGET_TOLERANCE_ANGLE) {

        if (actuator.motor_running) {

            err =
                actuator_manager_stop(
                    now_ms);

            if (err != ESP_OK) {
                return err;
            }
        }

        s_status.target_active =
            false;

        s_status.moving =
            false;

        return ESP_OK;
    }

    /*
     * Target is above the current position.
     *
     * Move OPEN.
     */
    if (position.total_angle <
        s_status.target_total_angle) {

        /*
         * Motor already moving OPEN.
         */
        if (actuator.motor_running &&
            actuator.direction ==
                ACTUATOR_DIRECTION_OPEN) {

            s_status.moving =
                true;

            return ESP_OK;
        }

        /*
         * Never reverse direction while running.
         * Stop first and wait for the next update.
         */
        if (actuator.motor_running) {

            err =
                actuator_manager_stop(
                    now_ms);

            if (err != ESP_OK) {
                return err;
            }

            s_status.moving =
                false;

            return ESP_OK;
        }

        err =
            actuator_manager_open(
                now_ms);

        if (err != ESP_OK) {
            return err;
        }

        s_status.moving =
            true;

        return ESP_OK;
    }

    /*
     * Target is below the current position.
     *
     * Move CLOSE.
     */
    if (actuator.motor_running &&
        actuator.direction ==
            ACTUATOR_DIRECTION_CLOSE) {

        s_status.moving =
            true;

        return ESP_OK;
    }

    /*
     * Never reverse direction while running.
     * Stop first and wait for the next update.
     */
    if (actuator.motor_running) {

        err =
            actuator_manager_stop(
                now_ms);

        if (err != ESP_OK) {
            return err;
        }

        s_status.moving =
            false;

        return ESP_OK;
    }

    err =
        actuator_manager_close(
            now_ms);

    if (err != ESP_OK) {
        return err;
    }

    s_status.moving =
        true;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Status                                                                     */
/* -------------------------------------------------------------------------- */

esp_err_t valve_position_manager_get_status(
    valve_position_status_t *status)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /*
     * Refresh the current encoder position when available.
     */
    if (encoder_position_is_valid()) {

        encoder_position_state_t position =
            {0};

        esp_err_t err =
            encoder_position_get_state(
                &position);

        if (err != ESP_OK) {
            return err;
        }

        s_status.current_total_angle =
            position.total_angle;
    }

    *status =
        s_status;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* State queries                                                              */
/* -------------------------------------------------------------------------- */

bool valve_position_manager_is_initialized(void)
{
    return s_initialized;
}


bool valve_position_manager_is_target_active(void)
{
    return s_status.target_active;
}


/* -------------------------------------------------------------------------- */
/* Cancellation                                                               */
/* -------------------------------------------------------------------------- */

esp_err_t valve_position_manager_cancel(
    uint32_t now_ms)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    /*
     * Cancel the position target first.
     */
    s_status.target_active =
        false;

    s_status.moving =
        false;

    /*
     * If the actuator is physically running, stop it
     * through the Actuator Manager.
     *
     * Do not bypass the actuator/safety layers.
     */
    if (actuator_manager_is_running()) {

        esp_err_t err =
            actuator_manager_stop(
                now_ms);

        if (err != ESP_OK) {
            return err;
        }
    }

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Target calculation                                                         */
/* -------------------------------------------------------------------------- */

static bool valid_position_percent(
    valve_position_percent_t percent)
{
    return
        percent == VALVE_POSITION_0_PERCENT ||
        percent == VALVE_POSITION_25_PERCENT ||
        percent == VALVE_POSITION_50_PERCENT ||
        percent == VALVE_POSITION_75_PERCENT ||
        percent == VALVE_POSITION_100_PERCENT;
}


static int64_t calculate_target_total_angle(
    valve_position_percent_t percent)
{
    const int64_t span =
        s_open_total_angle -
        s_closed_total_angle;

    return
        s_closed_total_angle +
        (span * (int64_t)percent) / 100;
}