#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ENCODER_PERSISTENCE_VERSION 1U

typedef struct
{
    uint32_t version;
    uint16_t last_angle;
    int32_t rotation_count;
    int64_t total_angle;
    uint8_t valid;
    uint32_t checksum;
} encoder_persistence_record_t;

/*
 * Initialize the encoder persistence manager.
 *
 * The underlying NVS subsystem must already be initialized.
 */
esp_err_t encoder_persistence_init(void);

/*
 * Save an encoder position record to NVS.
 */
esp_err_t encoder_persistence_save(
    const encoder_persistence_record_t *record);

/*
 * Load the previously saved encoder position.
 *
 * Returns ESP_ERR_NOT_FOUND when no valid record exists.
 */
esp_err_t encoder_persistence_load(
    encoder_persistence_record_t *record);

/*
 * Erase the persisted encoder position.
 */
esp_err_t encoder_persistence_erase(void);

/*
 * Check whether the persistence manager has been initialized.
 */
bool encoder_persistence_is_initialized(void);

/*
 * Check whether a valid persisted encoder record exists.
 */
bool encoder_persistence_has_valid_record(void);

#ifdef __cplusplus
}
#endif