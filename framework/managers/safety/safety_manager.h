#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif


/* -------------------------------------------------------------------------- */
/* Safety state                                                               */
/* -------------------------------------------------------------------------- */

typedef enum
{
    SAFETY_STATE_NORMAL = 0,

    SAFETY_STATE_LOW_BATTERY,

    SAFETY_STATE_BATTERY_CUT_PENDING,

    SAFETY_STATE_BATTERY_CUT,

    SAFETY_STATE_OVERCURRENT,

    SAFETY_STATE_OVERCURRENT_LOCKED,

    SAFETY_STATE_MOTOR_TIMEOUT,

    SAFETY_STATE_FAULT

} safety_manager_state_t;


/* -------------------------------------------------------------------------- */
/* Inputs                                                                      */
/* -------------------------------------------------------------------------- */

typedef struct
{
    /*
     * Current actuator state.
     */
    bool motor_running;

    /*
     * Condition Monitor voltage information.
     */
    float bus_voltage_v;
    bool voltage_valid;

    /*
     * Condition Monitor current information.
     */
    float motor_current_a;
    bool current_valid;

    /*
     * Condition Monitor state flags.
     */
    bool voltage_locked;
    bool overcurrent;
    bool overcurrent_locked;

    /*
     * Low-voltage bypass state.
     */
    bool bypass_required;
    bool bypass_acknowledged;

} safety_manager_inputs_t;


/* -------------------------------------------------------------------------- */
/* Configuration                                                               */
/* -------------------------------------------------------------------------- */

typedef struct
{
    /*
     * These values mirror the Condition Monitor configuration.
     *
     * MQTT/NVS persistence belongs to a higher layer.
     */
    float cut_voltage_v;
    float critical_voltage_v;

    float overcurrent_trip_a;

    uint32_t battery_bypass_timeout_ms;

    /*
     * Maximum allowed continuous motor runtime.
     */
    uint32_t motor_max_runtime_ms;

} safety_manager_config_t;


/* -------------------------------------------------------------------------- */
/* Output                                                                      */
/* -------------------------------------------------------------------------- */

typedef struct
{
    safety_manager_state_t state;

    /*
     * Motor permission.
     */
    bool allow_motor_start;
    bool allow_motor_run;

    /*
     * Safety actions requested by this manager.
     */
    bool request_motor_stop;
    bool request_motor_close;

    /*
     * Bypass handling.
     */
    bool bypass_required;

    /*
     * General fault indication.
     */
    bool fault;

    /*
     * Diagnostics/timing.
     */
    uint32_t battery_cut_elapsed_ms;
    uint32_t motor_runtime_ms;

    /*
     * Output validity.
     */
    bool valid;

} safety_manager_output_t;


/* -------------------------------------------------------------------------- */
/* Initialization                                                              */
/* -------------------------------------------------------------------------- */

esp_err_t safety_manager_init(
    const safety_manager_config_t *config);


/* -------------------------------------------------------------------------- */
/* Evaluation                                                                  */
/* -------------------------------------------------------------------------- */

/*
 * Evaluate all active safety conditions.
 *
 * The safety manager consumes Condition Monitor results.
 * It does not read INA226 directly and does not use ADC/VBAT.
 */
esp_err_t safety_manager_evaluate(
    const safety_manager_inputs_t *inputs,
    uint32_t now_ms,
    safety_manager_output_t *output);


/* -------------------------------------------------------------------------- */
/* Fault / cycle control                                                       */
/* -------------------------------------------------------------------------- */

/*
 * Clear a latched safety fault.
 *
 * The caller is responsible for ensuring the underlying
 * fault condition has been resolved.
 */
esp_err_t safety_manager_clear_fault(void);


/*
 * Reset per-motor-cycle timing state.
 */
esp_err_t safety_manager_reset_cycle(void);


/* -------------------------------------------------------------------------- */
/* Status                                                                      */
/* -------------------------------------------------------------------------- */

bool safety_manager_is_initialized(void);

bool safety_manager_is_fault_latched(void);

safety_manager_state_t safety_manager_get_state(void);


#ifdef __cplusplus
}
#endif