#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif


/* -------------------------------------------------------------------------- */
/* Voltage state                                                              */
/* -------------------------------------------------------------------------- */

typedef enum
{
    CONDITION_VOLTAGE_UNKNOWN = 0,

    CONDITION_VOLTAGE_NORMAL,

    CONDITION_VOLTAGE_WARNING,

    CONDITION_VOLTAGE_WAIT_BYPASS,

    CONDITION_VOLTAGE_BYPASS_ACTIVE,

    CONDITION_VOLTAGE_LOCKED

} condition_monitor_voltage_state_t;


/* -------------------------------------------------------------------------- */
/* Current state                                                              */
/* -------------------------------------------------------------------------- */

typedef enum
{
    CONDITION_CURRENT_NORMAL = 0,

    CONDITION_CURRENT_OVERCURRENT,

    CONDITION_CURRENT_LOCKED

} condition_monitor_current_state_t;


/* -------------------------------------------------------------------------- */
/* Configuration                                                              */
/* -------------------------------------------------------------------------- */

typedef struct
{
    /*
     * INA226 bus-voltage thresholds.
     *
     * warn_voltage_low_v:
     *     Low-voltage warning threshold.
     *
     * cut_voltage_v:
     *     Motor cut/bypass threshold.
     *
     * critical_voltage_v:
     *     Hard voltage-lock threshold.
     *
     * reset_voltage_v:
     *     Recovery threshold for locked/bypass voltage states.
     *
     * No high-voltage warning threshold is used.
     */
    float warn_voltage_low_v;
    float cut_voltage_v;
    float critical_voltage_v;
    float reset_voltage_v;

    /*
     * INA226 current threshold.
     */
    float overcurrent_trip_a;

    /*
     * Maximum number of OC attempts before locking.
     */
    uint8_t overcurrent_max_attempts;

    /*
     * Time allowed for bypass acknowledgement after
     * a low-voltage motor cut.
     */
    uint32_t battery_bypass_timeout_ms;

} condition_monitor_config_t;


/* -------------------------------------------------------------------------- */
/* INA226 reading                                                             */
/* -------------------------------------------------------------------------- */

typedef struct
{
    float bus_voltage_v;
    float current_a;
    float power_w;
    float shunt_voltage_v;

    bool voltage_valid;
    bool current_valid;
    bool power_valid;

    bool valid;

} condition_monitor_reading_t;


/* -------------------------------------------------------------------------- */
/* Runtime state                                                              */
/* -------------------------------------------------------------------------- */

typedef struct
{
    condition_monitor_voltage_state_t voltage_state;

    condition_monitor_current_state_t current_state;

    /*
     * True when a low-voltage motor cut requires bypass handling.
     */
    bool bypass_required;

    /*
     * True when bypass has been acknowledged.
     */
    bool bypass_acknowledged;

    /*
     * True when the current condition has reached the OC
     * lock limit.
     */
    bool overcurrent_locked;

    /*
     * Number of OC trips/attempts in the current cycle.
     */
    uint8_t overcurrent_attempts;

    /*
     * Time at which the battery-cut/bypass timer started.
     */
    uint32_t bypass_start_ms;

    /*
     * Last calculated bypass elapsed time.
     */
    uint32_t bypass_elapsed_ms;

    /*
     * Measurement diagnostics.
     */
    uint32_t successful_reads;
    uint32_t failed_reads;
    esp_err_t last_error;

} condition_monitor_state_t;


/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

/*
 * Initialize the condition monitoring manager.
 *
 * The INA226 driver must already be initialized.
 */
esp_err_t condition_monitor_manager_init(
    const condition_monitor_config_t *config);


/*
 * Deinitialize the condition monitoring manager.
 */
esp_err_t condition_monitor_manager_deinit(void);


/* -------------------------------------------------------------------------- */
/* Monitoring                                                                 */
/* -------------------------------------------------------------------------- */

/*
 * Update INA226 measurements and evaluate electrical conditions.
 *
 * now_ms:
 *     Monotonic millisecond timestamp supplied by the caller.
 *
 * motor_running:
 *     Current motor-running state.
 */
esp_err_t condition_monitor_manager_update(
    uint32_t now_ms,
    bool motor_running);


/* -------------------------------------------------------------------------- */
/* Voltage / bypass control                                                   */
/* -------------------------------------------------------------------------- */

/*
 * Inform the manager that bypass has been acknowledged.
 *
 * The acknowledgement may originate from local or remote
 * control logic. MQTT handling does not belong in this manager.
 */
esp_err_t condition_monitor_manager_acknowledge_bypass(void);


/*
 * Clear the current bypass request after the safety/control
 * layer has completed the required action.
 */
esp_err_t condition_monitor_manager_clear_bypass(void);


/*
 * Reset the OC attempt counter for a new motor cycle.
 */
esp_err_t condition_monitor_manager_reset_cycle(void);


/*
 * Record one confirmed over-current trip.
 *
 * The Safety/Actuator layer calls this after an OC event has
 * actually caused a motor stop.
 */
esp_err_t condition_monitor_manager_record_overcurrent_trip(void);


/* -------------------------------------------------------------------------- */
/* Reading                                                                    */
/* -------------------------------------------------------------------------- */

esp_err_t condition_monitor_manager_get_reading(
    condition_monitor_reading_t *reading);


/* -------------------------------------------------------------------------- */
/* State                                                                      */
/* -------------------------------------------------------------------------- */

esp_err_t condition_monitor_manager_get_state(
    condition_monitor_state_t *state);


/* -------------------------------------------------------------------------- */
/* Configuration                                                              */
/* -------------------------------------------------------------------------- */

esp_err_t condition_monitor_manager_get_config(
    condition_monitor_config_t *config);


/*
 * Update runtime configuration.
 *
 * Persistence and MQTT transport are handled by higher layers.
 *
 * The voltage thresholds in condition_monitor_config_t are therefore
 * intentionally exposed to the higher configuration layer.
 */
esp_err_t condition_monitor_manager_set_config(
    const condition_monitor_config_t *config);


/* -------------------------------------------------------------------------- */
/* Convenience status                                                         */
/* -------------------------------------------------------------------------- */

bool condition_monitor_manager_is_voltage_safe_to_start(
    void);


bool condition_monitor_manager_is_overcurrent(
    void);


bool condition_monitor_manager_is_locked(
    void);


bool condition_monitor_manager_is_bypass_required(
    void);


bool condition_monitor_manager_is_initialized(
    void);


#ifdef __cplusplus
}
#endif