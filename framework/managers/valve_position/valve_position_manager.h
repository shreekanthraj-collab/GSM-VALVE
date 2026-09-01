#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif


/* -------------------------------------------------------------------------- */
/* Valve position                                                             */
/* -------------------------------------------------------------------------- */

typedef enum
{
    VALVE_POSITION_0_PERCENT = 0,

    VALVE_POSITION_25_PERCENT = 25,

    VALVE_POSITION_50_PERCENT = 50,

    VALVE_POSITION_75_PERCENT = 75,

    VALVE_POSITION_100_PERCENT = 100

} valve_position_percent_t;


/* -------------------------------------------------------------------------- */
/* Runtime state                                                              */
/* -------------------------------------------------------------------------- */

typedef struct
{
    valve_position_percent_t target_percent;

    int64_t target_total_angle;

    int64_t current_total_angle;

    bool target_active;

    bool moving;

    bool valid;

} valve_position_status_t;


/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

esp_err_t valve_position_manager_init(void);


/* -------------------------------------------------------------------------- */
/* Calibration                                                                */
/* -------------------------------------------------------------------------- */

/*
 * Record the current encoder position as CLOSED.
 *
 * The actuator must be stopped and the encoder position must
 * be valid.
 *
 * This does not start motor movement.
 */
esp_err_t valve_position_calibrate_closed(void);


/*
 * Record the current encoder position as OPEN.
 *
 * The actuator must be stopped and the encoder position must
 * be valid.
 *
 * OPEN calibration is accepted only when it is different from
 * the previously recorded CLOSED position.
 *
 * Successful OPEN calibration persists the complete calibration
 * record and makes position control available.
 */
esp_err_t valve_position_calibrate_open(void);


/*
 * Clear the persisted CLOSED/OPEN calibration.
 *
 * Position control becomes unavailable until calibration is
 * performed again.
 */
esp_err_t valve_position_calibration_clear(void);


/*
 * Return whether a valid CLOSED/OPEN calibration is active.
 */
bool valve_position_manager_is_calibrated(void);


/* -------------------------------------------------------------------------- */
/* Position command                                                           */
/* -------------------------------------------------------------------------- */

/*
 * Request a calibrated valve position.
 *
 * Supported targets:
 *
 *     0   = CLOSED
 *     25  = 25%
 *     50  = 50%
 *     75  = 75%
 *     100 = OPEN
 *
 * The manager does not bypass the Safety Manager.
 */
esp_err_t valve_position_set_target(
    valve_position_percent_t percent,
    uint32_t now_ms);


/* -------------------------------------------------------------------------- */
/* Runtime processing                                                         */
/* -------------------------------------------------------------------------- */

/*
 * Process the active position command.
 *
 * The manager observes the encoder position and actuator state.
 * It requests OPEN/CLOSE/STOP through the Actuator Manager.
 */
esp_err_t valve_position_manager_update(
    uint32_t now_ms);


/* -------------------------------------------------------------------------- */
/* Status                                                                     */
/* -------------------------------------------------------------------------- */

esp_err_t valve_position_manager_get_status(
    valve_position_status_t *status);

bool valve_position_manager_is_initialized(void);

bool valve_position_manager_is_target_active(void);


/* -------------------------------------------------------------------------- */
/* Emergency / cancellation                                                   */
/* -------------------------------------------------------------------------- */

/*
 * Cancel the active position command.
 *
 * This does not clear an Emergency Stop or Safety fault.
 */
esp_err_t valve_position_manager_cancel(
    uint32_t now_ms);


#ifdef __cplusplus
}
#endif