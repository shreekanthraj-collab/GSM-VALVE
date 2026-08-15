#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ENCODER_POSITION_ANGLE_MAX  4095U
#define ENCODER_POSITION_FULL_RANGE 4096U

typedef struct
{
    uint16_t angle;
    int32_t rotation_count;
    int64_t total_angle;
    float total_turns;
    bool restored;
    bool valid;
} encoder_position_state_t;

/*
 * Initialize the encoder position manager.
 *
 * The AS5600 driver and encoder manager must already be initialized.
 * The NVS subsystem must already be initialized through the persistence
 * manager.
 */
esp_err_t encoder_position_init(void);

/*
 * Restore the previously persisted absolute position.
 *
 * The first live AS5600 sample after restore establishes the new
 * tracking reference and does not create movement.
 *
 * Returns ESP_ERR_NOT_FOUND when no valid persisted record exists.
 */
esp_err_t encoder_position_restore(void);

/*
 * Update the position with a new AS5600 angle.
 *
 * The first sample after initialization or restore establishes the
 * reference and produces zero movement.
 */
esp_err_t encoder_position_update(
    uint16_t angle);

/*
 * Get the current absolute encoder position.
 */
esp_err_t encoder_position_get_state(
    encoder_position_state_t *state);

/*
 * Save the current absolute encoder position to NVS.
 */
esp_err_t encoder_position_save(void);

/*
 * Reset the accumulated absolute position to zero.
 *
 * The next angle sample becomes the new reference.
 */
esp_err_t encoder_position_reset(void);

/*
 * Check whether the manager has been initialized.
 */
bool encoder_position_is_initialized(void);

/*
 * Check whether a valid live position is currently available.
 */
bool encoder_position_is_valid(void);

/*
 * Check whether the current position originated from
 * a valid persisted record.
 */
bool encoder_position_was_restored(void);

#ifdef __cplusplus
}
#endif