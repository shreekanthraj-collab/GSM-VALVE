#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif


/* -------------------------------------------------------------------------- */
/* Actuator state                                                             */
/* -------------------------------------------------------------------------- */

typedef enum
{
    ACTUATOR_STATE_IDLE = 0,

    ACTUATOR_STATE_OPENING,

    ACTUATOR_STATE_CLOSING,

    ACTUATOR_STATE_STOPPING,

    ACTUATOR_STATE_FAULT

} actuator_manager_state_t;


/* -------------------------------------------------------------------------- */
/* Motor direction                                                            */
/* -------------------------------------------------------------------------- */

typedef enum
{
    ACTUATOR_DIRECTION_NONE = 0,

    ACTUATOR_DIRECTION_OPEN,

    ACTUATOR_DIRECTION_CLOSE

} actuator_manager_direction_t;


/* -------------------------------------------------------------------------- */
/* Configuration                                                              */
/* -------------------------------------------------------------------------- */

typedef struct
{
    /*
     * Maximum permitted continuous motor runtime.
     *
     * Safety Manager remains the authoritative safety
     * layer for this limit.
     */
    uint32_t max_runtime_ms;

    /*
     * Normal motor PWM duty cycle.
     *
     * Valid range: 0.0 to 100.0 percent.
     */
    float motor_duty_percent;

} actuator_manager_config_t;


/* -------------------------------------------------------------------------- */
/* Runtime state                                                              */
/* -------------------------------------------------------------------------- */

typedef struct
{
    actuator_manager_state_t state;

    actuator_manager_direction_t direction;

    /*
     * True while the actuator is physically commanded to run.
     */
    bool motor_running;

    /*
     * True when a safety layer has prevented operation.
     */
    bool safety_blocked;

    /*
     * Runtime of the current motor operation.
     */
    uint32_t runtime_ms;

    /*
     * Number of completed motor operations.
     */
    uint32_t operation_count;

    /*
     * Last motor command accepted by the manager.
     */
    actuator_manager_direction_t last_command;

    /*
     * State validity.
     */
    bool valid;

} actuator_manager_status_t;


/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

esp_err_t actuator_manager_init(
    const actuator_manager_config_t *config);


/* -------------------------------------------------------------------------- */
/* Motor commands                                                             */
/* -------------------------------------------------------------------------- */

/*
 * Request actuator OPEN.
 *
 * The command is accepted only when the actuator is not
 * safety-blocked.
 */
esp_err_t actuator_manager_open(
    uint32_t now_ms);


/*
 * Request actuator CLOSE.
 *
 * The command is accepted only when the actuator is not
 * safety-blocked.
 */
esp_err_t actuator_manager_close(
    uint32_t now_ms);


/*
 * Request actuator STOP.
 */
esp_err_t actuator_manager_stop(
    uint32_t now_ms);


/*
 * Apply a safety decision to the actuator manager.
 *
 * request_motor_stop:
 *     Stop the motor immediately.
 *
 * request_motor_close:
 *     Request a close operation after the safety stop.
 *
 * fault:
 *     Latch the actuator into FAULT state.
 */
esp_err_t actuator_manager_apply_safety(
    bool allow_motor_start,
    bool allow_motor_run,
    bool request_motor_stop,
    bool request_motor_close,
    bool fault,
    uint32_t now_ms);


/* -------------------------------------------------------------------------- */
/* Periodic processing                                                        */
/* -------------------------------------------------------------------------- */

/*
 * Process actuator runtime/state transitions.
 *
 * Hardware-specific GPIO/PWM work remains below this manager.
 */
esp_err_t actuator_manager_update(
    uint32_t now_ms);


/* -------------------------------------------------------------------------- */
/* Fault / cycle control                                                      */
/* -------------------------------------------------------------------------- */

/*
 * Clear a latched actuator fault.
 */
esp_err_t actuator_manager_clear_fault(void);


/*
 * Reset per-operation runtime tracking.
 */
esp_err_t actuator_manager_reset_cycle(void);


/* -------------------------------------------------------------------------- */
/* Status                                                                     */
/* -------------------------------------------------------------------------- */

esp_err_t actuator_manager_get_status(
    actuator_manager_status_t *status);


bool actuator_manager_is_initialized(
    void);


bool actuator_manager_is_running(
    void);


bool actuator_manager_is_fault(
    void);


actuator_manager_state_t actuator_manager_get_state(
    void);


actuator_manager_direction_t actuator_manager_get_direction(
    void);


#ifdef __cplusplus
}
#endif
