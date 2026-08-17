#include "condition_monitor_manager.h"

#include <stddef.h>

#include "drv_ina226.h"


/* -------------------------------------------------------------------------- */
/* Internal state                                                             */
/* -------------------------------------------------------------------------- */

static bool s_initialized = false;

static condition_monitor_config_t s_config = {0};

static condition_monitor_reading_t s_reading = {0};

static condition_monitor_state_t s_state = {
    .voltage_state = CONDITION_VOLTAGE_UNKNOWN,
    .current_state = CONDITION_CURRENT_NORMAL,
    .bypass_required = false,
    .bypass_acknowledged = false,
    .overcurrent_locked = false,
    .overcurrent_attempts = 0U,
    .bypass_start_ms = 0U,
    .bypass_elapsed_ms = 0U,
    .successful_reads = 0U,
    .failed_reads = 0U,
    .last_error = ESP_OK
};


/* -------------------------------------------------------------------------- */
/* Configuration validation                                                   */
/* -------------------------------------------------------------------------- */

static bool valid_config(
    const condition_monitor_config_t *config)
{
    if (config == NULL) {
        return false;
    }

    /*
     * Voltage relationship:
     *
     * critical <= cut <= low warning <= reset
     *
     * There is deliberately no high-voltage warning threshold.
     */

    if (config->critical_voltage_v < 0.0f) {
        return false;
    }

    if (config->cut_voltage_v <
        config->critical_voltage_v) {
        return false;
    }

    if (config->warn_voltage_low_v <
        config->cut_voltage_v) {
        return false;
    }

    if (config->reset_voltage_v <
        config->warn_voltage_low_v) {
        return false;
    }

    /*
     * Over-current threshold must be positive.
     *
     * The actual production value is runtime configurable
     * through the higher configuration/AWS layer.
     */
    if (config->overcurrent_trip_a <= 0.0f) {
        return false;
    }

    /*
     * At least one confirmed OC trip must be permitted.
     */
    if (config->overcurrent_max_attempts == 0U) {
        return false;
    }

    /*
     * Bypass acknowledgement timeout must be non-zero.
     */
    if (config->battery_bypass_timeout_ms == 0U) {
        return false;
    }

    return true;
}


/* -------------------------------------------------------------------------- */
/* Voltage evaluation                                                         */
/* -------------------------------------------------------------------------- */

static condition_monitor_voltage_state_t evaluate_voltage_state(
    float voltage_v,
    bool motor_running)
{
    /*
     * Critical voltage:
     *
     * This is a hard voltage-lock condition.
     */
    if (voltage_v <
        s_config.critical_voltage_v) {

        return CONDITION_VOLTAGE_LOCKED;
    }

    /*
     * Motor running below the cut threshold:
     *
     * Motor operation must be stopped and bypass handling
     * becomes required.
     */
    if (motor_running &&
        voltage_v <
            s_config.cut_voltage_v) {

        return CONDITION_VOLTAGE_WAIT_BYPASS;
    }

    /*
     * Low-voltage warning region.
     */
    if (voltage_v <
        s_config.warn_voltage_low_v) {

        return CONDITION_VOLTAGE_WARNING;
    }

    /*
     * All other voltage conditions are considered normal.
     *
     * There is intentionally no high-voltage warning state.
     */
    return CONDITION_VOLTAGE_NORMAL;
}


/* -------------------------------------------------------------------------- */
/* Bypass state handling                                                      */
/* -------------------------------------------------------------------------- */

static void update_bypass_state(
    uint32_t now_ms,
    bool motor_running)
{
    /*
     * Critical voltage does not use the bypass timer.
     */
    if (s_state.voltage_state ==
        CONDITION_VOLTAGE_LOCKED) {

        s_state.bypass_required = false;
        s_state.bypass_acknowledged = false;
        s_state.bypass_start_ms = 0U;
        s_state.bypass_elapsed_ms = 0U;

        return;
    }

    /*
     * A low-voltage motor cut requires bypass handling.
     */
    if (s_state.voltage_state ==
        CONDITION_VOLTAGE_WAIT_BYPASS) {

        s_state.bypass_required = true;

        if (!s_state.bypass_acknowledged) {

            if (s_state.bypass_start_ms == 0U) {

                s_state.bypass_start_ms =
                    now_ms;

                s_state.bypass_elapsed_ms =
                    0U;

            } else {

                s_state.bypass_elapsed_ms =
                    now_ms -
                    s_state.bypass_start_ms;
            }
        }

        return;
    }

    /*
     * Once bypass has been acknowledged, retain the
     * acknowledgement while the system remains in the
     * bypass-active state.
     */
    if (s_state.bypass_acknowledged) {

        s_state.voltage_state =
            CONDITION_VOLTAGE_BYPASS_ACTIVE;

        s_state.bypass_required = true;

        if (s_state.bypass_start_ms != 0U) {

            s_state.bypass_elapsed_ms =
                now_ms -
                s_state.bypass_start_ms;
        }

        return;
    }

    /*
     * Normal voltage condition.
     */
    s_state.bypass_required = false;
    s_state.bypass_start_ms = 0U;
    s_state.bypass_elapsed_ms = 0U;

    if (s_state.voltage_state !=
        CONDITION_VOLTAGE_LOCKED) {

        s_state.voltage_state =
            CONDITION_VOLTAGE_NORMAL;
    }

    /*
     * motor_running is intentionally not used here beyond
     * the voltage-state evaluation above.
     */
    (void)motor_running;
}


/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

esp_err_t condition_monitor_manager_init(
    const condition_monitor_config_t *config)
{
    if (s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!valid_config(config)) {
        return ESP_ERR_INVALID_ARG;
    }

    /*
     * INA226 must already be initialized by the lower driver layer.
     */
    if (!drv_ina226_is_initialized()) {
        return ESP_ERR_INVALID_STATE;
    }

    s_config = *config;

    s_reading =
        (condition_monitor_reading_t){0};

    s_state =
        (condition_monitor_state_t){
            .voltage_state =
                CONDITION_VOLTAGE_UNKNOWN,

            .current_state =
                CONDITION_CURRENT_NORMAL,

            .bypass_required =
                false,

            .bypass_acknowledged =
                false,

            .overcurrent_locked =
                false,

            .overcurrent_attempts =
                0U,

            .bypass_start_ms =
                0U,

            .bypass_elapsed_ms =
                0U,

            .successful_reads =
                0U,

            .failed_reads =
                0U,

            .last_error =
                ESP_OK
        };

    s_initialized = true;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Deinitialization                                                           */
/* -------------------------------------------------------------------------- */

esp_err_t condition_monitor_manager_deinit(
    void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    s_initialized = false;

    s_config =
        (condition_monitor_config_t){0};

    s_reading =
        (condition_monitor_reading_t){0};

    s_state =
        (condition_monitor_state_t){
            .voltage_state =
                CONDITION_VOLTAGE_UNKNOWN,

            .current_state =
                CONDITION_CURRENT_NORMAL,

            .bypass_required =
                false,

            .bypass_acknowledged =
                false,

            .overcurrent_locked =
                false,

            .overcurrent_attempts =
                0U,

            .bypass_start_ms =
                0U,

            .bypass_elapsed_ms =
                0U,

            .successful_reads =
                0U,

            .failed_reads =
                0U,

            .last_error =
                ESP_OK
        };

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Monitoring                                                                 */
/* -------------------------------------------------------------------------- */

esp_err_t condition_monitor_manager_update(
    uint32_t now_ms,
    bool motor_running)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    drv_ina226_reading_t ina_reading = {0};

    esp_err_t err =
        drv_ina226_read(
            &ina_reading);

    if (err != ESP_OK) {

        s_state.failed_reads++;

        s_state.last_error =
            err;

        s_reading.valid = false;

        return err;
    }

    s_state.successful_reads++;

    s_state.last_error =
        ESP_OK;

    /*
     * Copy the complete converted INA226 measurement.
     */
    s_reading.shunt_voltage_v =
        ina_reading.shunt_voltage_v;

    s_reading.bus_voltage_v =
        ina_reading.bus_voltage_v;

    s_reading.current_a =
        ina_reading.current_a;

    s_reading.power_w =
        ina_reading.power_w;

    s_reading.shunt_voltage_v =
        ina_reading.shunt_voltage_v;

    s_reading.voltage_valid = true;
    s_reading.current_valid = true;
    s_reading.power_valid = true;
    s_reading.valid = true;

    /*
     * Evaluate voltage condition.
     */
    s_state.voltage_state =
        evaluate_voltage_state(
            s_reading.bus_voltage_v,
            motor_running);

    /*
     * Update bypass state and timing.
     */
    update_bypass_state(
        now_ms,
        motor_running);

    /*
     * Current / over-current state.
     *
     * A measurement crossing the threshold identifies
     * an over-current condition.
     *
     * The actual OC attempt counter is incremented only
     * by condition_monitor_manager_record_overcurrent_trip()
     * after the higher actuator/safety layer confirms that
     * an actual motor trip occurred.
     */
    if (s_reading.current_valid &&
        s_reading.current_a >=
            s_config.overcurrent_trip_a) {

        s_state.current_state =
            CONDITION_CURRENT_OVERCURRENT;

    } else {

        if (!s_state.overcurrent_locked) {

            s_state.current_state =
                CONDITION_CURRENT_NORMAL;
        }
    }

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Bypass acknowledgement                                                      */
/* -------------------------------------------------------------------------- */

esp_err_t condition_monitor_manager_acknowledge_bypass(
    void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!s_state.bypass_required) {
        return ESP_ERR_INVALID_STATE;
    }

    s_state.bypass_acknowledged = true;

    s_state.voltage_state =
        CONDITION_VOLTAGE_BYPASS_ACTIVE;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Clear bypass                                                               */
/* -------------------------------------------------------------------------- */

esp_err_t condition_monitor_manager_clear_bypass(
    void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    s_state.bypass_required = false;

    s_state.bypass_acknowledged = false;

    s_state.bypass_start_ms = 0U;

    s_state.bypass_elapsed_ms = 0U;

    if (s_state.voltage_state ==
        CONDITION_VOLTAGE_BYPASS_ACTIVE) {

        s_state.voltage_state =
            CONDITION_VOLTAGE_NORMAL;
    }

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Reset cycle                                                                */
/* -------------------------------------------------------------------------- */

esp_err_t condition_monitor_manager_reset_cycle(
    void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    s_state.overcurrent_attempts = 0U;

    s_state.overcurrent_locked = false;

    if (s_state.current_state ==
        CONDITION_CURRENT_LOCKED) {

        s_state.current_state =
            CONDITION_CURRENT_NORMAL;
    }

    s_state.bypass_required = false;

    s_state.bypass_acknowledged = false;

    s_state.bypass_start_ms = 0U;

    s_state.bypass_elapsed_ms = 0U;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Record confirmed over-current trip                                         */
/* -------------------------------------------------------------------------- */

esp_err_t condition_monitor_manager_record_overcurrent_trip(
    void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_state.overcurrent_attempts <
        s_config.overcurrent_max_attempts) {

        s_state.overcurrent_attempts++;
    }

    s_state.current_state =
        CONDITION_CURRENT_OVERCURRENT;

    if (s_state.overcurrent_attempts >=
        s_config.overcurrent_max_attempts) {

        s_state.overcurrent_locked = true;

        s_state.current_state =
            CONDITION_CURRENT_LOCKED;
    }

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Reading                                                                    */
/* -------------------------------------------------------------------------- */

esp_err_t condition_monitor_manager_get_reading(
    condition_monitor_reading_t *reading)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (reading == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *reading = s_reading;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* State                                                                      */
/* -------------------------------------------------------------------------- */

esp_err_t condition_monitor_manager_get_state(
    condition_monitor_state_t *state)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *state = s_state;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Configuration                                                              */
/* -------------------------------------------------------------------------- */

esp_err_t condition_monitor_manager_get_config(
    condition_monitor_config_t *config)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *config = s_config;

    return ESP_OK;
}


esp_err_t condition_monitor_manager_set_config(
    const condition_monitor_config_t *config)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!valid_config(config)) {
        return ESP_ERR_INVALID_ARG;
    }

    s_config = *config;

    /*
     * Re-evaluate the current reading against the new thresholds.
     *
     * This keeps the manager ready for configuration supplied by
     * NVS/AWS in a higher layer.
     */
    if (s_reading.voltage_valid) {

        s_state.voltage_state =
            evaluate_voltage_state(
                s_reading.bus_voltage_v,
                false);

        update_bypass_state(
            0U,
            false);
    }

    if (s_reading.current_valid) {

        if (s_reading.current_a >=
            s_config.overcurrent_trip_a) {

            s_state.current_state =
                CONDITION_CURRENT_OVERCURRENT;

        } else if (!s_state.overcurrent_locked) {

            s_state.current_state =
                CONDITION_CURRENT_NORMAL;
        }
    }

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Convenience status                                                         */
/* -------------------------------------------------------------------------- */

bool condition_monitor_manager_is_voltage_safe_to_start(
    void)
{
    if (!s_initialized) {
        return false;
    }

    return s_state.voltage_state ==
               CONDITION_VOLTAGE_NORMAL ||
           s_state.voltage_state ==
               CONDITION_VOLTAGE_WARNING;
}


bool condition_monitor_manager_is_overcurrent(
    void)
{
    if (!s_initialized) {
        return false;
    }

    return s_state.current_state ==
               CONDITION_CURRENT_OVERCURRENT ||
           s_state.current_state ==
               CONDITION_CURRENT_LOCKED;
}


bool condition_monitor_manager_is_locked(
    void)
{
    if (!s_initialized) {
        return false;
    }

    return s_state.voltage_state ==
               CONDITION_VOLTAGE_LOCKED ||
           s_state.current_state ==
               CONDITION_CURRENT_LOCKED;
}


bool condition_monitor_manager_is_bypass_required(
    void)
{
    if (!s_initialized) {
        return false;
    }

    return s_state.bypass_required;
}


bool condition_monitor_manager_is_initialized(
    void)
{
    return s_initialized;
}