#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#include "eol_manager.h"

#ifdef __cplusplus
extern "C" {
#endif


/* -------------------------------------------------------------------------- */
/* Persistence constants                                                      */
/* -------------------------------------------------------------------------- */

/*
 * Dedicated NVS namespace for EOL persistence.
 */
#define EOL_PERSISTENCE_NAMESPACE              "eol"

/*
 * NVS key used for the persisted EOL result record.
 */
#define EOL_PERSISTENCE_RECORD_KEY             "eol_record"

/*
 * NVS key used for the persisted factory operating configuration.
 */
#define EOL_PERSISTENCE_FACTORY_CONFIG_KEY     "factory_cfg"

/*
 * Persistent record magic.
 *
 * "EOL1" in hexadecimal.
 */
#define EOL_PERSISTENCE_MAGIC                  0x454F4C31UL

/*
 * Persistent record format version.
 *
 * Increment when the eol_persisted_record_t binary layout changes.
 */
#define EOL_PERSISTENCE_RECORD_VERSION         1U

/*
 * Compatibility version.
 *
 * Increment when a firmware/hardware change requires factory
 * EOL re-verification.
 */
#define EOL_PERSISTENCE_COMPATIBILITY_VERSION  1U

/*
 * Maximum firmware-version string length including terminating NUL.
 */
#define EOL_PERSISTENCE_FIRMWARE_VERSION_MAX_LEN  32U


/* -------------------------------------------------------------------------- */
/* EOL validity                                                               */
/* -------------------------------------------------------------------------- */

typedef enum
{
    EOL_PERSISTENCE_NOT_TESTED = 0,

    EOL_PERSISTENCE_VALID,

    EOL_PERSISTENCE_REVERIFICATION_REQUIRED,

    EOL_PERSISTENCE_FAILED

} eol_persistence_validity_t;


/* -------------------------------------------------------------------------- */
/* Persisted EOL record                                                       */
/* -------------------------------------------------------------------------- */

/*
 * Factory EOL result stored in NVS.
 *
 * This structure follows the current eol_persistence.c implementation.
 *
 * The previous record is retained when identity or compatibility
 * changes. A new record replaces it only after a completed EOL result
 * is explicitly saved.
 */
typedef struct
{
    /*
     * Persistent record identity.
     */
    uint32_t magic;

    uint32_t record_version;

    uint32_t compatibility_version;


    /*
     * Device identity.
     */
    uint32_t hardware_fingerprint;

    uint32_t firmware_fingerprint;


    /*
     * Firmware version associated with this EOL record.
     */
    char firmware_version[
        EOL_PERSISTENCE_FIRMWARE_VERSION_MAX_LEN];


    /*
     * Overall EOL result.
     */
    eol_overall_result_t overall_result;


    /*
     * EOL execution timing.
     */
    uint32_t started_ms;

    uint32_t completed_ms;


    /*
     * Test counters.
     */
    uint32_t tests_passed;

    uint32_t tests_failed;

    uint32_t tests_skipped;

    uint32_t tests_not_run;


    /*
     * Individual test results.
     */
    uint8_t test_result[
        EOL_TEST_COUNT];


    /*
     * Diagnostic values.
     */
    uint16_t encoder_angle;

    float battery_voltage_v;

    float ina226_bus_voltage_v;

    float ina226_current_a;

    float ina226_power_w;


} eol_persisted_record_t;


/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

/*
 * Initialize the EOL persistence subsystem.
 *
 * Opens the dedicated NVS namespace and restores any existing
 * factory EOL record/configuration.
 */
esp_err_t eol_persistence_init(void);


/*
 * Deinitialize the EOL persistence subsystem.
 */
esp_err_t eol_persistence_deinit(void);


/*
 * Return whether the persistence subsystem is initialized.
 */
bool eol_persistence_is_initialized(void);


/* -------------------------------------------------------------------------- */
/* Factory operating configuration                                            */
/* -------------------------------------------------------------------------- */

/*
 * Save the factory operating configuration.
 *
 * The configuration must already have passed
 * eol_manager_validate_factory_config().
 */
esp_err_t eol_persistence_save_factory_config(
    const eol_factory_config_t *config);


/*
 * Load the persisted factory operating configuration.
 *
 * ESP_ERR_NOT_FOUND means no configuration exists.
 */
esp_err_t eol_persistence_load_factory_config(
    eol_factory_config_t *config);


/*
 * Check whether a factory operating configuration exists.
 *
 * The result is returned through 'present'.
 */
esp_err_t eol_persistence_has_factory_config(
    bool *present);


/*
 * Clear only the factory operating configuration.
 *
 * The EOL result record is not affected.
 */
esp_err_t eol_persistence_clear_factory_config(void);


/* -------------------------------------------------------------------------- */
/* Current identity                                                           */
/* -------------------------------------------------------------------------- */

/*
 * Set the current hardware identity fingerprint.
 */
esp_err_t eol_persistence_set_hardware_fingerprint(
    uint32_t fingerprint);


/*
 * Set the current firmware identity fingerprint.
 */
esp_err_t eol_persistence_set_firmware_fingerprint(
    uint32_t fingerprint);


/*
 * Set the firmware version associated with the current device.
 */
esp_err_t eol_persistence_set_firmware_version(
    const char *version);


/*
 * Get current hardware fingerprint.
 */
uint32_t eol_persistence_get_hardware_fingerprint(void);


/*
 * Get current firmware fingerprint.
 */
uint32_t eol_persistence_get_firmware_fingerprint(void);


/* -------------------------------------------------------------------------- */
/* EOL record                                                                 */
/* -------------------------------------------------------------------------- */

/*
 * Load the persisted EOL record.
 *
 * ESP_ERR_NOT_FOUND means no EOL record exists.
 */
esp_err_t eol_persistence_load(
    eol_persisted_record_t *record);


/*
 * Save a completed EOL result as the factory record.
 */
esp_err_t eol_persistence_save_result(
    const eol_status_t *status);


/*
 * Clear the persisted EOL result record.
 */
esp_err_t eol_persistence_clear(void);


/*
 * Get the currently cached EOL record.
 */
esp_err_t eol_persistence_get_record(
    eol_persisted_record_t *record);


/* -------------------------------------------------------------------------- */
/* Validity                                                                   */
/* -------------------------------------------------------------------------- */

/*
 * Determine validity of the stored EOL record against the current
 * device identity and compatibility version.
 */
eol_persistence_validity_t eol_persistence_get_validity(void);


/*
 * Return true when a valid factory EOL PASS exists.
 */
bool eol_persistence_is_valid(void);


/*
 * Return true when EOL re-verification is required.
 */
bool eol_persistence_is_reverification_required(void);


/*
 * Return true when no factory EOL result exists.
 */
bool eol_persistence_is_not_tested(void);


/*
 * Return true when the stored EOL result is FAIL.
 */
bool eol_persistence_is_failed(void);


/* -------------------------------------------------------------------------- */
/* Stored identity                                                            */
/* -------------------------------------------------------------------------- */

/*
 * Get stored hardware fingerprint.
 */
esp_err_t eol_persistence_get_stored_hardware_fingerprint(
    uint32_t *fingerprint);


/*
 * Get stored firmware fingerprint.
 */
esp_err_t eol_persistence_get_stored_firmware_fingerprint(
    uint32_t *fingerprint);


/*
 * Get stored compatibility version.
 *
 * The current implementation returns the value directly.
 */
uint32_t eol_persistence_get_stored_compatibility_version(void);


/* -------------------------------------------------------------------------- */
/* Re-verification                                                            */
/* -------------------------------------------------------------------------- */

/*
 * Evaluate the stored record against the current identity.
 *
 * Does not erase the stored record.
 */
esp_err_t eol_persistence_validate_current_identity(void);


/*
 * Mark the current device as requiring EOL re-verification.
 *
 * The previous record remains stored.
 */
esp_err_t eol_persistence_require_reverification(void);


/* -------------------------------------------------------------------------- */
/* Compatibility                                                              */
/* -------------------------------------------------------------------------- */

/*
 * Return the firmware's current EOL compatibility version.
 */
uint32_t eol_persistence_get_compatibility_version(void);


/* -------------------------------------------------------------------------- */
/* Utility                                                                     */
/* -------------------------------------------------------------------------- */

/*
 * Calculate a deterministic fingerprint from a byte buffer.
 */
uint32_t eol_persistence_calculate_fingerprint(
    const void *data,
    uint32_t length);


#ifdef __cplusplus
}
#endif