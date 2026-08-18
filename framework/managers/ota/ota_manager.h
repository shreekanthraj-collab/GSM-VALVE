#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif


/* -------------------------------------------------------------------------- */
/* OTA state                                                                  */
/* -------------------------------------------------------------------------- */

typedef enum
{
    OTA_MANAGER_STATE_IDLE = 0,

    OTA_MANAGER_STATE_PREPARING,

    OTA_MANAGER_STATE_DOWNLOADING,

    OTA_MANAGER_STATE_VALIDATING,

    OTA_MANAGER_STATE_READY_TO_REBOOT,

    OTA_MANAGER_STATE_SUCCESS,

    OTA_MANAGER_STATE_ERROR

} ota_manager_state_t;


/* -------------------------------------------------------------------------- */
/* OTA error                                                                  */
/* -------------------------------------------------------------------------- */

typedef enum
{
    OTA_MANAGER_ERROR_NONE = 0,

    OTA_MANAGER_ERROR_NOT_INITIALIZED,

    OTA_MANAGER_ERROR_INVALID_ARGUMENT,

    OTA_MANAGER_ERROR_ALREADY_RUNNING,

    OTA_MANAGER_ERROR_MOTOR_RUNNING,

    OTA_MANAGER_ERROR_SAFETY_FAULT,

    OTA_MANAGER_ERROR_BATTERY_UNSAFE,

    OTA_MANAGER_ERROR_INVALID_URL,

    OTA_MANAGER_ERROR_DOWNLOAD_FAILED,

    OTA_MANAGER_ERROR_WRITE_FAILED,

    OTA_MANAGER_ERROR_VALIDATION_FAILED,

    OTA_MANAGER_ERROR_BOOT_PARTITION_FAILED,

    OTA_MANAGER_ERROR_ROLLBACK_FAILED,

    OTA_MANAGER_ERROR_INTERNAL

} ota_manager_error_t;


/* -------------------------------------------------------------------------- */
/* OTA configuration                                                          */
/* -------------------------------------------------------------------------- */

typedef struct
{
    /*
     * Maximum accepted firmware image size.
     *
     * Current GSM-VALVE OTA partitions are 1 MB.
     */
    size_t max_image_size;

    /*
     * Require HTTPS URL.
     *
     * Production configuration should keep this enabled.
     */
    bool require_https;

} ota_manager_config_t;


/* -------------------------------------------------------------------------- */
/* OTA status                                                                 */
/* -------------------------------------------------------------------------- */

typedef struct
{
    ota_manager_state_t state;

    ota_manager_error_t error;

    /*
     * Download progress.
     *
     * 0..100 percent.
     */
    uint8_t progress_percent;

    /*
     * Bytes downloaded so far.
     */
    size_t bytes_downloaded;

    /*
     * Total image size when known.
     *
     * Zero means unknown.
     */
    size_t image_size;

    /*
     * Current firmware version.
     */
    char current_version[32];

    /*
     * Firmware version supplied for the pending update.
     */
    char target_version[32];

    /*
     * True when OTA is currently active.
     */
    bool active;

    /*
     * True when reboot is required to activate
     * the downloaded image.
     */
    bool reboot_required;

    /*
     * True when the running application has been
     * accepted by the bootloader.
     */
    bool running_image_valid;

} ota_manager_status_t;


/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

esp_err_t ota_manager_init(
    const ota_manager_config_t *config);


/* -------------------------------------------------------------------------- */
/* OTA execution                                                              */
/* -------------------------------------------------------------------------- */

/*
 * Start OTA from an HTTPS URL.
 *
 * This function performs the OTA download and writes the
 * inactive OTA application partition.
 *
 * The caller must ensure:
 *
 *   - motor is stopped
 *   - actuator is safe
 *   - sufficient power is available
 *   - network connection is available
 *
 * The manager itself also maintains its own OTA state and
 * rejects concurrent OTA operations.
 */
esp_err_t ota_manager_start(
    const char *url,
    const char *target_version);


/*
 * Abort an active OTA operation.
 */
esp_err_t ota_manager_abort(void);


/*
 * Reboot into the newly installed firmware.
 *
 * OTA must already have completed successfully.
 */
esp_err_t ota_manager_reboot(void);


/* -------------------------------------------------------------------------- */
/* Status                                                                     */
/* -------------------------------------------------------------------------- */

esp_err_t ota_manager_get_status(
    ota_manager_status_t *status);

ota_manager_state_t ota_manager_get_state(void);

ota_manager_error_t ota_manager_get_error(void);

bool ota_manager_is_initialized(void);

bool ota_manager_is_active(void);


/* -------------------------------------------------------------------------- */
/* Boot / rollback validation                                                 */
/* -------------------------------------------------------------------------- */

/*
 * Validate the currently running OTA image.
 *
 * When rollback support is enabled, the application should
 * call this after successful startup once the essential
 * hardware/software initialization has completed.
 */
esp_err_t ota_manager_mark_running_image_valid(void);


/*
 * Check whether the current image is pending verification.
 */
bool ota_manager_is_image_pending_verify(void);


/* -------------------------------------------------------------------------- */
/* Safety interlock                                                           */
/* -------------------------------------------------------------------------- */

/*
 * Update the OTA safety interlock.
 *
 * OTA is permitted only when:
 *
 *   motor_running == false
 *   safety_fault == false
 *   battery_safe == true
 */
esp_err_t ota_manager_set_safety_state(
    bool motor_running,
    bool safety_fault,
    bool battery_safe);


#ifdef __cplusplus
}
#endif