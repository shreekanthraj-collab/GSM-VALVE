#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif


/* -------------------------------------------------------------------------- */
/* Configuration                                                              */
/* -------------------------------------------------------------------------- */

#define MODBUS_POLL_DEFAULT_INTERVAL_MS    5000U


/* -------------------------------------------------------------------------- */
/* Configuration                                                              */
/* -------------------------------------------------------------------------- */

typedef struct
{
    /*
     * Minimum delay between polling cycles.
     *
     * One cycle checks all eight slots once.
     */
    uint32_t poll_interval_ms;

} modbus_poll_manager_config_t;


/* -------------------------------------------------------------------------- */
/* Runtime status                                                             */
/* -------------------------------------------------------------------------- */

typedef struct
{
    bool running;

    uint8_t current_slot;

    uint32_t total_polls;

    uint32_t successful_polls;

    uint32_t failed_polls;

} modbus_poll_manager_status_t;


/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

/*
 * Initialize the Modbus polling manager.
 *
 * This does not start polling.
 *
 * The Modbus device manager must already be initialized.
 */
esp_err_t modbus_poll_manager_init(
    const modbus_poll_manager_config_t *config);


/*
 * Start the polling task.
 */
esp_err_t modbus_poll_manager_start(void);


/*
 * Stop the polling task.
 */
esp_err_t modbus_poll_manager_stop(void);


/*
 * Deinitialize the polling manager.
 *
 * The polling task must be stopped first.
 */
esp_err_t modbus_poll_manager_deinit(void);


/* -------------------------------------------------------------------------- */
/* Status                                                                     */
/* -------------------------------------------------------------------------- */

/*
 * Get the current polling status.
 */
esp_err_t modbus_poll_manager_get_status(
    modbus_poll_manager_status_t *status);


/*
 * Check whether polling is currently running.
 */
bool modbus_poll_manager_is_running(void);


#ifdef __cplusplus
}
#endif
