#include "safety_manager.h"


/* -------------------------------------------------------------------------- */
/* Internal state                                                             */
/* -------------------------------------------------------------------------- */

static bool s_initialized = false;
static bool s_fault_latched = false;

static safety_manager_config_t s_config = {0};

static safety_manager_state_t s_state =
    SAFETY_STATE_NORMAL;

static uint32_t s_motor_start_ms = 0U;
static bool s_motor_timing_active = false;


/* -------------------------------------------------------------------------- */
/* Configuration validation                                                   */
/* -------------------------------------------------------------------------- */

static bool valid_config(
    const safety_manager_config_t *config)
{
    if (config == NULL) {
        return false;
    }

    if (config->critical_voltage_v < 0.0f) {
        return false;
    }

    if (config->cut_voltage_v <
        config->critical_voltage_v) {
        return false;
    }

    if (config->overcurrent_trip_a <= 0.0f) {
        return false;
    }

    if (config->battery_bypass_timeout_ms == 0U) {
        return false;
    }

    if (config->motor_max_runtime_ms == 0U) {
        return false;
    }

    return true;
}


/* -------------------------------------------------------------------------- */
/* Timing                                                                     */
/* -------------------------------------------------------------------------- */

static uint32_t elapsed_ms(
    uint32_t now_ms,
    uint32_t start_ms)
{
    return now_ms - start_ms;
}


static void clear_motor_timing(void)
{
    s_motor_start_ms = 0U;
    s_motor_timing_active = false;
}


/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

esp_err_t safety_manager_init(
    const safety_manager_config_t *config)
{
    if (s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!valid_config(config)) {
        return ESP_ERR_INVALID_ARG;
    }

    s_config = *config;

    s_fault_latched = false;

    s_state =
        SAFETY_STATE_NORMAL;

    clear_motor_timing();

    s_initialized = true;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Evaluation                                                                 */
/* -------------------------------------------------------------------------- */

esp_err_t safety_manager_evaluate(
    const safety_manager_inputs_t *inputs,
    uint32_t now_ms,
    safety_manager_output_t *output)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (inputs == NULL ||
        output == NULL) {

        return ESP_ERR_INVALID_ARG;
    }

    *output =
        (safety_manager_output_t){0};

    output->valid = true;


    /* ---------------------------------------------------------------------- */
    /* Latched fault                                                           */
    /* ---------------------------------------------------------------------- */

    if (s_fault_latched) {

        s_state =
            SAFETY_STATE_FAULT;

        output->state =
            s_state;

        output->request_motor_stop =
            inputs->motor_running;

        output->fault = true;

        return ESP_OK;
    }


    /* ---------------------------------------------------------------------- */
    /* Motor runtime                                                           */
    /* ---------------------------------------------------------------------- */

    if (inputs->motor_running) {

        if (!s_motor_timing_active) {

            s_motor_start_ms =
                now_ms;

            s_motor_timing_active =
                true;
        }

        output->motor_runtime_ms =
            elapsed_ms(
                now_ms,
                s_motor_start_ms);
    }
    else {

        clear_motor_timing();
    }


    /* ---------------------------------------------------------------------- */
    /* Measurement validity                                                    */
    /* ---------------------------------------------------------------------- */

    /*
     * The active safety path uses Condition Monitor data.
     *
     * No ADC/VBAT measurement is performed here.
     */
    if (!inputs->voltage_valid) {

        s_fault_latched = true;

        s_state =
            SAFETY_STATE_FAULT;

        output->state =
            s_state;

        output->request_motor_stop =
            inputs->motor_running;

        output->fault = true;

        return ESP_OK;
    }


    /* ---------------------------------------------------------------------- */
    /* Critical voltage                                                        */
    /* ---------------------------------------------------------------------- */

    if (inputs->voltage_locked ||
        inputs->bus_voltage_v <
            s_config.critical_voltage_v) {

        s_fault_latched = true;

        s_state =
            SAFETY_STATE_FAULT;

        output->state =
            s_state;

        output->request_motor_stop =
            inputs->motor_running;

        output->request_motor_close =
            inputs->motor_running;

        output->fault = true;

        return ESP_OK;
    }


    /* ---------------------------------------------------------------------- */
    /* Over-current                                                            */
    /* ---------------------------------------------------------------------- */

    if (inputs->overcurrent_locked ||
        (inputs->overcurrent &&
         inputs->motor_running)) {

        if (inputs->overcurrent_locked) {

            s_state =
                SAFETY_STATE_OVERCURRENT_LOCKED;

            s_fault_latched = true;

        }
        else {

            s_state =
                SAFETY_STATE_OVERCURRENT;
        }

        output->state =
            s_state;

        output->request_motor_stop = true;

        output->fault =
            inputs->overcurrent_locked;

        return ESP_OK;
    }


    /*
     * Fallback current check.
     *
     * Condition Monitor remains the primary source of OC state.
     * This additional check protects against an input integration
     * mistake where the numerical current is already above the
     * configured threshold but the OC flag has not propagated.
     */
    if (inputs->motor_running &&
        inputs->current_valid &&
        inputs->motor_current_a >=
            s_config.overcurrent_trip_a) {

        s_state =
            SAFETY_STATE_OVERCURRENT;

        output->state =
            s_state;

        output->request_motor_stop = true;

        return ESP_OK;
    }


    /* ---------------------------------------------------------------------- */
    /* Motor runtime timeout                                                   */
    /* ---------------------------------------------------------------------- */

    if (inputs->motor_running &&
        s_motor_timing_active &&
        output->motor_runtime_ms >=
            s_config.motor_max_runtime_ms) {

        s_fault_latched = true;

        s_state =
            SAFETY_STATE_MOTOR_TIMEOUT;

        output->state =
            s_state;

        output->request_motor_stop = true;
        output->request_motor_close = true;
        output->fault = true;

        return ESP_OK;
    }


    /* ---------------------------------------------------------------------- */
    /* Low-voltage bypass                                                      */
    /* ---------------------------------------------------------------------- */

    if (inputs->bypass_required) {

        output->bypass_required = true;

        /*
         * Condition Monitor owns the bypass timer/state.
         *
         * Safety Manager therefore does not start a second
         * independent timer. The Condition Monitor state is
         * consumed through the input flags.
         */
        if (inputs->bypass_acknowledged) {

            s_state =
                SAFETY_STATE_LOW_BATTERY;

            output->state =
                s_state;

            output->allow_motor_run =
                true;

            return ESP_OK;
        }

        /*
         * The Condition Monitor has already determined that
         * bypass handling is required. The safety layer keeps
         * the motor running only while the higher-level bypass
         * policy permits it.
         *
         * Timeout/automatic close is handled by the caller
         * using the Condition Monitor timing state.
         */
        s_state =
            SAFETY_STATE_BATTERY_CUT_PENDING;

        output->state =
            s_state;

        output->allow_motor_run =
            true;

        return ESP_OK;
    }


    /* ---------------------------------------------------------------------- */
    /* Normal operation                                                       */
    /* ---------------------------------------------------------------------- */

    s_state =
        SAFETY_STATE_NORMAL;

    output->state =
        s_state;

    output->allow_motor_start = true;
    output->allow_motor_run = true;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Fault control                                                              */
/* -------------------------------------------------------------------------- */

esp_err_t safety_manager_clear_fault(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    s_fault_latched = false;

    s_state =
        SAFETY_STATE_NORMAL;

    clear_motor_timing();

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Cycle control                                                              */
/* -------------------------------------------------------------------------- */

esp_err_t safety_manager_reset_cycle(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    clear_motor_timing();

    if (!s_fault_latched) {

        s_state =
            SAFETY_STATE_NORMAL;
    }

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Status                                                                     */
/* -------------------------------------------------------------------------- */

bool safety_manager_is_initialized(void)
{
    return s_initialized;
}


bool safety_manager_is_fault_latched(void)
{
    return s_fault_latched;
}


safety_manager_state_t safety_manager_get_state(void)
{
    return s_state;
}