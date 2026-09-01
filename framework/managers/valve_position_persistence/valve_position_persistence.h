#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif


/* -------------------------------------------------------------------------- */
/* Persistence schema                                                         */
/* -------------------------------------------------------------------------- */

#define VALVE_POSITION_PERSISTENCE_VERSION 1U


/* -------------------------------------------------------------------------- */
/* Persisted calibration                                                      */
/* -------------------------------------------------------------------------- */

typedef struct
{
    /*
     * Persistence schema version.
     */
    uint32_t version;

    /*
     * Absolute encoder position corresponding to CLOSED.
     */
    int64_t closed_total_angle;

    /*
     * Absolute encoder position corresponding to OPEN.
     */
    int64_t open_total_angle;

    /*
     * Calibration validity.
     */
    uint8_t valid;

    /*
     * Record integrity checksum.
     */
    uint32_t checksum;

} valve_position_persistence_record_t;


/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

esp_err_t valve_position_persistence_init(void);


/* -------------------------------------------------------------------------- */
/* Save / load                                                                */
/* -------------------------------------------------------------------------- */

/*
 * Save the calibrated CLOSED and OPEN encoder positions.
 */
esp_err_t valve_position_persistence_save(
    const valve_position_persistence_record_t *record);


/*
 * Load the calibrated valve-position record.
 *
 * Returns ESP_ERR_NOT_FOUND when no valid calibration exists.
 */
esp_err_t valve_position_persistence_load(
    valve_position_persistence_record_t *record);


/* -------------------------------------------------------------------------- */
/* Erase                                                                      */
/* -------------------------------------------------------------------------- */

esp_err_t valve_position_persistence_erase(void);


/* -------------------------------------------------------------------------- */
/* Status                                                                     */
/* -------------------------------------------------------------------------- */

bool valve_position_persistence_is_initialized(void);

bool valve_position_persistence_has_valid_record(void);


#ifdef __cplusplus
}
#endif