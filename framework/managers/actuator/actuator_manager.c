#include "actuator_manager.h"

#include <string.h>

#include "hal_actuator.h"


/* -------------------------------------------------------------------------- */
/* Internal state                                                             */
/* -------------------------------------------------------------------------- */

static bool s_initialized = false;

static actuator_manager_config_t s_config = {0};

static actuator_manager_status_t s_status = {
    .state =
        ACTUATOR_STATE_IDLE,

    .direction =
        ACTUATOR_DIRECTION_NONE,

    .motor_running =
        false,

    .safety_blocked =
        false,

    .runtime_ms =
        0U,

    .operation_count =
        0U,

    .last_command =
        ACTUATOR_DIRECTION_NONE,

    .valid =
        false
};

static uint32_t s_operation_start_ms = 0U;


/* -------------------------------------------------------------------------- */
/* Configuration validation                                                   */
/* -------------------------------------------------------------------------- */

static bool valid_config(
    const actuator_manager_config_t *config)
{
    if (config == NULL) {
        return false;
    }

    /*
     * Zero runtime is not a valid configured safety limit.
     */
    if (config->max_runtime_ms == 0U) {
        return false;
    }

    /*
     * PWM duty must remain inside the HAL-supported range.
     */
    if (config->motor_duty_percent < 0.0f ||
        config->motor_duty_percent > 100.0f) {

        return false;
    }

    return true;
}


/* -------------------------------------------------------------------------- */
/* Internal helpers                                                           */
/* -------------------------------------------------------------------------- */

/*
 * Clear the active-operation state.
 *
 * This does not modify:
 *
 *   - safety_blocked
 *   - operation_count
 *   - last_command
 */
static void clear_active_operation(void)
{
    s_status.motor_running =
        false;

    s_status.direction =
        ACTUATOR_DIRECTION_NONE;

    s_status.runtime_ms =
        0U;

    s_operation_start_ms =
        0U;
}





/*
 * Record completion of one accepted motor operation.
 */
static void mark_operation_stopped(void)
{
    clear_active_operation();

    s_status.operation_count++;
}


/*
 * Start the physical motor operation.
 *
 * IMPORTANT:
 * Manager state is updated only after the HAL
 * successfully accepts the hardware command.
 */
static esp_err_t start_operation(
    actuator_manager_direction_t direction,
    uint32_t now_ms)
{
    esp_err_t err;

    if (direction ==
        ACTUATOR_DIRECTION_OPEN) {

        err =
            hal_actuator_start_open(
                s_config.motor_duty_percent);
    }
    else if (direction ==
             ACTUATOR_DIRECTION_CLOSE) {

        err =
            hal_actuator_start_close(
                s_config.motor_duty_percent);
    }
    else {

        return ESP_ERR_INVALID_ARG;
    }

    /*
     * Never claim that the motor is running when
     * the hardware start operation failed.
     */
    if (err != ESP_OK) {
        return err;
    }

    s_status.direction =
        direction;

    s_status.last_command =
        direction;

    s_status.motor_running =
        true;

    s_status.runtime_ms =
        0U;

    s_operation_start_ms =
        now_ms;

    if (direction ==
        ACTUATOR_DIRECTION_OPEN) {

        s_status.state =
            ACTUATOR_STATE_OPENING;
    }
    else {

        s_status.state =
            ACTUATOR_STATE_CLOSING;
    }

    return ESP_OK;
}


/*
 * Stop the physical motor.
 *
 * The manager enters STOPPING before requesting the HAL stop.
 * If the HAL stop fails, the manager remains in STOPPING rather
 * than falsely reporting IDLE.
 */
static esp_err_t stop_operation(void)
{
    s_status.state =
        ACTUATOR_STATE_STOPPING;

    esp_err_t err =
        hal_actuator_stop();

    if (err != ESP_OK) {
        return err;
    }

    mark_operation_stopped();

    s_status.state =
        ACTUATOR_STATE_IDLE;

    return ESP_OK;
}

/*
 * Force the physical actuator into the hardware-safe state.
 *
 * This is used for faults and safety-enforced shutdown.
 */
static esp_err_t force_safe_state(void)
{
    esp_err_t err =
        hal_actuator_force_safe();

    if (err != ESP_OK) {
        /*
         * Hardware safety operation itself failed.
         *
         * Keep the manager in FAULT so the application
         * cannot accidentally treat the actuator as normal.
         */
        s_status.state =
            ACTUATOR_STATE_FAULT;

        s_status.motor_running =
            false;

        s_status.direction =
            ACTUATOR_DIRECTION_NONE;

        return err;
    }

    /*
     * Hardware is confirmed safe.
     */
    clear_active_operation();

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

esp_err_t actuator_manager_init(
    const actuator_manager_config_t *config)
{
    if (s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!valid_config(config)) {
        return ESP_ERR_INVALID_ARG;
    }

    /*
     * Actuator manager requires the hardware HAL to already
     * be initialized.
     */
    if (!hal_actuator_is_initialized()) {
        return ESP_ERR_INVALID_STATE;
    }

    s_config =
        *config;

    s_status =
        (actuator_manager_status_t){

            .state =
                ACTUATOR_STATE_IDLE,

            .direction =
                ACTUATOR_DIRECTION_NONE,

            .motor_running =
                false,

            .safety_blocked =
                false,

            .runtime_ms =
                0U,

            .operation_count =
                0U,

            .last_command =
                ACTUATOR_DIRECTION_NONE,

            .valid =
                true
        };

    s_operation_start_ms =
        0U;

    /*
     * Initialization must begin from a known hardware-safe state.
     *
     * Do not silently continue if this fails.
     */
    esp_err_t err =
        hal_actuator_force_safe();

    if (err != ESP_OK) {

        memset(
            &s_config,
            0,
            sizeof(s_config));

        s_status.valid =
            false;

        return err;
    }

    s_initialized =
        true;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* OPEN                                                                       */
/* -------------------------------------------------------------------------- */

esp_err_t actuator_manager_open(
    uint32_t now_ms)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!s_status.valid) {
        return ESP_ERR_INVALID_STATE;
    }

    /*
     * A latched FAULT must be explicitly cleared.
     */
    if (s_status.state ==
        ACTUATOR_STATE_FAULT) {

        return ESP_ERR_INVALID_STATE;
    }

    /*
     * Safety block prevents new motor commands.
     */
    if (s_status.safety_blocked) {
        return ESP_ERR_INVALID_STATE;
    }

    /*
     * Do not permit a second command while a motor
     * operation is already active.
     */
    if (s_status.motor_running) {
        return ESP_ERR_INVALID_STATE;
    }

    return start_operation(
        ACTUATOR_DIRECTION_OPEN,
        now_ms);
}


/* -------------------------------------------------------------------------- */
/* CLOSE                                                                      */
/* -------------------------------------------------------------------------- */

esp_err_t actuator_manager_close(
    uint32_t now_ms)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!s_status.valid) {
        return ESP_ERR_INVALID_STATE;
    }

    /*
     * A latched FAULT must be explicitly cleared.
     */
    if (s_status.state ==
        ACTUATOR_STATE_FAULT) {

        return ESP_ERR_INVALID_STATE;
    }

    /*
     * Safety block prevents normal new commands.
     */
    if (s_status.safety_blocked) {
        return ESP_ERR_INVALID_STATE;
    }

    /*
     * Do not permit a second command while the motor
     * is already running.
     */
    if (s_status.motor_running) {
        return ESP_ERR_INVALID_STATE;
    }

    return start_operation(
        ACTUATOR_DIRECTION_CLOSE,
        now_ms);
}


/* -------------------------------------------------------------------------- */
/* STOP                                                                       */
/* -------------------------------------------------------------------------- */

esp_err_t actuator_manager_stop(
    uint32_t now_ms)
{
    (void)now_ms;

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!s_status.valid) {
        return ESP_ERR_INVALID_STATE;
    }

    /*
     * STOP is allowed even when safety_blocked.
     *
     * Safety must always be able to stop the actuator.
     */
    if (!s_status.motor_running) {

        /*
         * If already stopped, ensure the hardware remains
         * safe without creating a fake completed operation.
         */
        esp_err_t err =
            hal_actuator_force_safe();

        if (err != ESP_OK) {
            s_status.state =
                ACTUATOR_STATE_FAULT;

            return err;
        }

        if (s_status.state !=
            ACTUATOR_STATE_FAULT) {

            s_status.state =
                ACTUATOR_STATE_IDLE;
        }

        return ESP_OK;
    }

    return stop_operation();
}


/* -------------------------------------------------------------------------- */
/* Safety application                                                         */
/* -------------------------------------------------------------------------- */

esp_err_t actuator_manager_apply_safety(
    bool allow_motor_start,
    bool allow_motor_run,
    bool request_motor_stop,
    bool request_motor_close,
    bool fault,
    uint32_t now_ms)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!s_status.valid) {
        return ESP_ERR_INVALID_STATE;
    }

    /*
     * --------------------------------------------------------------
     * 1. FAULT has highest priority.
     * --------------------------------------------------------------
     *
     * A fault always forces the physical actuator safe and latches
     * the manager into FAULT.
     */
    if (fault) {

        s_status.safety_blocked =
            true;

        esp_err_t err =
            force_safe_state();

        s_status.state =
            ACTUATOR_STATE_FAULT;

        if (err != ESP_OK) {
            return err;
        }

        return ESP_OK;
    }


    /*
     * --------------------------------------------------------------
     * 2. Explicit safety STOP.
     * --------------------------------------------------------------
     *
     * This has priority over a requested safety CLOSE.
     */
    if (request_motor_stop) {

        esp_err_t err =
            actuator_manager_stop(
                now_ms);

        if (err != ESP_OK) {
            return err;
        }
    }


    /*
     * --------------------------------------------------------------
     * 3. Runtime operation permission.
     * --------------------------------------------------------------
     *
     * If the safety layer says the motor may no longer run,
     * immediately stop an active operation.
     */
    if (!allow_motor_run &&
        s_status.motor_running) {

        esp_err_t err =
            actuator_manager_stop(
                now_ms);

        if (err != ESP_OK) {
            return err;
        }
    }


    /*
     * --------------------------------------------------------------
     * 4. New motor-start permission.
     * --------------------------------------------------------------
     *
     * A false allow_motor_start means normal OPEN/CLOSE requests
     * must remain blocked.
     */
    if (!allow_motor_start) {

        s_status.safety_blocked =
            true;
    }
    else {

        /*
         * Safety is currently permitting new starts.
         *
         * Do not clear a FAULT here. Fault clearing is explicit.
         */
        if (s_status.state !=
            ACTUATOR_STATE_FAULT) {

            s_status.safety_blocked =
                false;
        }
    }


    /*
     * --------------------------------------------------------------
     * 5. Safety-requested CLOSE.
     * --------------------------------------------------------------
     *
     * This is an explicit safety-layer instruction.
     *
     * It is only performed when:
     *
     *   - there is no fault,
     *   - the actuator is stopped,
     *   - the manager is not latched FAULT.
     *
     * The safety layer has explicitly requested CLOSE, therefore
     * this operation does not go through the normal external
     * OPEN/CLOSE command path.
     */
    if (request_motor_close) {

        if (s_status.state ==
            ACTUATOR_STATE_FAULT) {

            return ESP_ERR_INVALID_STATE;
        }

        if (s_status.motor_running) {

            return ESP_ERR_INVALID_STATE;
        }

        /*
         * Safety-requested CLOSE is permitted only when the
         * safety layer has explicitly allowed the motor to run.
         */
        if (!allow_motor_run) {

            return ESP_ERR_INVALID_STATE;
        }

        /*
         * Directly start the close operation because this is a
         * safety-layer command rather than a normal user command.
         */
        esp_err_t err =
            start_operation(
                ACTUATOR_DIRECTION_CLOSE,
                now_ms);

        if (err != ESP_OK) {
            return err;
        }
    }

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Periodic processing                                                        */
/* -------------------------------------------------------------------------- */

esp_err_t actuator_manager_update(
    uint32_t now_ms)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!s_status.valid) {
        return ESP_ERR_INVALID_STATE;
    }

    /*
     * Nothing to update when the actuator is not running.
     */
    if (!s_status.motor_running) {
        return ESP_OK;
    }

    /*
     * Calculate runtime using unsigned subtraction.
     *
     * uint32_t subtraction naturally handles normal millisecond
     * tick wrap-around.
     */
    s_status.runtime_ms =
        now_ms -
        s_operation_start_ms;

    /*
     * The manager tracks runtime for status/telemetry.
     *
     * Safety Manager remains the authoritative safety layer for
     * the maximum motor runtime. Therefore this manager does NOT
     * independently create a second safety decision here.
     */
    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Fault control                                                              */
/* -------------------------------------------------------------------------- */

esp_err_t actuator_manager_clear_fault(
    void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!s_status.valid) {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_status.state !=
        ACTUATOR_STATE_FAULT) {

        return ESP_ERR_INVALID_STATE;
    }

    /*
     * Hardware must be confirmed safe before clearing
     * the software fault latch.
     */
    esp_err_t err =
        hal_actuator_force_safe();

    if (err != ESP_OK) {
        return err;
    }

    clear_active_operation();

    s_status.state =
        ACTUATOR_STATE_IDLE;

    /*
     * Clearing the fault does not automatically authorize
     * motor operation. The safety layer must provide permission
     * again.
     */
    s_status.safety_blocked =
        true;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Cycle control                                                              */
/* -------------------------------------------------------------------------- */

esp_err_t actuator_manager_reset_cycle(
    void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!s_status.valid) {
        return ESP_ERR_INVALID_STATE;
    }

    /*
     * A running motor cannot have its cycle reset.
     */
    if (s_status.motor_running) {
        return ESP_ERR_INVALID_STATE;
    }

    /*
     * Do not reset a latched FAULT here.
     *
     * Fault recovery must use actuator_manager_clear_fault().
     */
    if (s_status.state ==
        ACTUATOR_STATE_FAULT) {

        return ESP_ERR_INVALID_STATE;
    }

    s_status.runtime_ms =
        0U;

    s_operation_start_ms =
        0U;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Status                                                                     */
/* -------------------------------------------------------------------------- */

esp_err_t actuator_manager_get_status(
    actuator_manager_status_t *status)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *status =
        s_status;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* State queries                                                              */
/* -------------------------------------------------------------------------- */

bool actuator_manager_is_initialized(
    void)
{
    return s_initialized;
}


bool actuator_manager_is_running(
    void)
{
    return s_initialized &&
           s_status.valid &&
           s_status.motor_running;
}


bool actuator_manager_is_fault(
    void)
{
    return s_initialized &&
           s_status.valid &&
           s_status.state ==
               ACTUATOR_STATE_FAULT;
}


actuator_manager_state_t actuator_manager_get_state(
    void)
{
    if (!s_initialized ||
        !s_status.valid) {

        return ACTUATOR_STATE_FAULT;
    }

    return s_status.state;
}


actuator_manager_direction_t actuator_manager_get_direction(
    void)
{
    if (!s_initialized ||
        !s_status.valid) {

        return ACTUATOR_DIRECTION_NONE;
    }

    return s_status.direction;
}