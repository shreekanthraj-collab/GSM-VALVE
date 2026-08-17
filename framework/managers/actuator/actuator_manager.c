#include "actuator_manager.h"

/* -------------------------------------------------------------------------- */
/* Internal state                                                             */
/* -------------------------------------------------------------------------- */

static bool s_initialized = false;

static actuator_manager_config_t s_config = {0};

static actuator_manager_status_t s_status = {
    .state = ACTUATOR_STATE_IDLE,
    .direction = ACTUATOR_DIRECTION_NONE,
    .motor_running = false,
    .safety_blocked = false,
    .runtime_ms = 0U,
    .operation_count = 0U,
    .last_command = ACTUATOR_DIRECTION_NONE,
    .valid = false
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

    if (config->max_runtime_ms == 0U) {
        return false;
    }

    return true;
}


/* -------------------------------------------------------------------------- */
/* Internal helpers                                                           */
/* -------------------------------------------------------------------------- */

static void set_idle_state(void)
{
    s_status.state =
        ACTUATOR_STATE_IDLE;

    s_status.direction =
        ACTUATOR_DIRECTION_NONE;

    s_status.motor_running =
        false;

    s_status.runtime_ms =
        0U;
}


static void start_operation(
    actuator_manager_direction_t direction,
    uint32_t now_ms)
{
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

    s_config = *config;

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

    if (s_status.safety_blocked) {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_status.motor_running) {
        return ESP_ERR_INVALID_STATE;
    }

    start_operation(
        ACTUATOR_DIRECTION_OPEN,
        now_ms);

    return ESP_OK;
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

    if (s_status.safety_blocked) {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_status.motor_running) {
        return ESP_ERR_INVALID_STATE;
    }

    start_operation(
        ACTUATOR_DIRECTION_CLOSE,
        now_ms);

    return ESP_OK;
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

    if (!s_status.motor_running) {

        set_idle_state();

        return ESP_OK;
    }

    s_status.state =
        ACTUATOR_STATE_STOPPING;

    s_status.motor_running =
        false;

    s_status.direction =
        ACTUATOR_DIRECTION_NONE;

    s_status.runtime_ms =
        0U;

    s_status.operation_count++;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Apply safety decision                                                      */
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

    /*
     * A fault always has priority.
     */
    if (fault) {

        if (s_status.motor_running) {

            s_status.state =
                ACTUATOR_STATE_STOPPING;

            s_status.motor_running =
                false;

            s_status.direction =
                ACTUATOR_DIRECTION_NONE;

            s_status.operation_count++;
        }

        s_status.safety_blocked =
            true;

        s_status.state =
            ACTUATOR_STATE_FAULT;

        return ESP_OK;
    }


    /*
     * Explicit safety stop has priority over normal operation.
     */
    if (request_motor_stop) {

        if (s_status.motor_running) {

            esp_err_t err =
                actuator_manager_stop(now_ms);

            if (err != ESP_OK) {
                return err;
            }
        }

        s_status.safety_blocked =
            !allow_motor_start;
    }


    /*
     * Safety may request a close after stopping the motor.
     *
     * This is intentionally handled only after the stop.
     */
    if (request_motor_close &&
        allow_motor_start &&
        !s_status.motor_running &&
        !s_status.safety_blocked) {

        return actuator_manager_close(now_ms);
    }


    /*
     * Prevent an already-running motor from continuing when
     * the safety layer withdraws run permission.
     */
    if (s_status.motor_running &&
        !allow_motor_run) {

        return actuator_manager_stop(now_ms);
    }


    /*
     * If the safety layer grants permission again, remove the
     * temporary safety block.
     */
    if (allow_motor_start &&
        allow_motor_run) {

        s_status.safety_blocked =
            false;
    }

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Periodic update                                                            */
/* -------------------------------------------------------------------------- */

esp_err_t actuator_manager_update(
    uint32_t now_ms)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!s_status.motor_running) {
        return ESP_OK;
    }

    s_status.runtime_ms =
        now_ms -
        s_operation_start_ms;

    /*
     * The Safety Manager is authoritative for motor runtime
     * safety. This manager only tracks the runtime.
     *
     * max_runtime_ms is retained in the actuator configuration
     * for future hardware-specific enforcement.
     */
    if (s_status.runtime_ms >=
        s_config.max_runtime_ms) {

        s_status.state =
            ACTUATOR_STATE_STOPPING;

        s_status.motor_running =
            false;

        s_status.direction =
            ACTUATOR_DIRECTION_NONE;

        s_status.operation_count++;

        s_status.safety_blocked =
            true;

        s_status.state =
            ACTUATOR_STATE_FAULT;
    }

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Clear fault                                                                */
/* -------------------------------------------------------------------------- */

esp_err_t actuator_manager_clear_fault(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    s_status.safety_blocked =
        false;

    set_idle_state();

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Reset cycle                                                                */
/* -------------------------------------------------------------------------- */

esp_err_t actuator_manager_reset_cycle(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_status.motor_running) {
        return ESP_ERR_INVALID_STATE;
    }

    s_status.runtime_ms =
        0U;

    s_operation_start_ms =
        0U;

    s_status.last_command =
        ACTUATOR_DIRECTION_NONE;

    if (s_status.state !=
        ACTUATOR_STATE_FAULT) {

        s_status.state =
            ACTUATOR_STATE_IDLE;
    }

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


bool actuator_manager_is_initialized(
    void)
{
    return s_initialized;
}


bool actuator_manager_is_running(
    void)
{
    return s_initialized &&
           s_status.motor_running;
}


bool actuator_manager_is_fault(
    void)
{
    return s_initialized &&
           s_status.state ==
               ACTUATOR_STATE_FAULT;
}


actuator_manager_state_t actuator_manager_get_state(
    void)
{
    if (!s_initialized) {
        return ACTUATOR_STATE_FAULT;
    }

    return s_status.state;
}


actuator_manager_direction_t actuator_manager_get_direction(
    void)
{
    if (!s_initialized) {
        return ACTUATOR_DIRECTION_NONE;
    }

    return s_status.direction;
}
