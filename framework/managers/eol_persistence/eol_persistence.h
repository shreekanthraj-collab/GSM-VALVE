#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#include "eol_manager.h"

#ifdef __cplusplus
extern "C" {
#endif


/* -------------------------------------------------------------------------- */
/* EOL persistence constants                                                  */
/* -------------------------------------------------------------------------- */

/*
 * NVS namespace used exclusively for the factory EOL record.
 */
#define EOL_PERSISTENCE_NAMESPACE          "eol"

/*
 * Persistent-record magic.
 *
 * "EOL1" in hexadecimal.
 */
#define EOL_PERSISTENCE_MAGIC              0x454F4C31UL

/*
 * Persistent record schema.
 *
 * Increment this when the stored EOL record format changes.
 */
#define EOL_PERSISTENCE_SCHEMA_VERSION    1U

/*
 * EOL compatibility version.
 *
 * Increment this when a firmware/hardware change requires the
 * actuator to undergo EOL re-verification.
 *
 * This is deliberately independent from the firmware version.
 */
#define EOL_COMPATIBILITY_VERSION          1U


/* -------------------------------------------------------------------------- */
/* EOL validity                                                               */
/* -------------------------------------------------------------------------- */

typedef enum
{
    /*
     * No valid factory EOL record exists.
     */
    EOL_PERSISTENCE_NOT_TESTED = 0,

    /*
     * A completed EOL record exists and its identity still matches
     * the currently installed hardware/configuration.
     */
    EOL_PERSISTENCE_VALID,

    /*
     * A previous EOL record exists, but its identity or compatibility
     * information no longer matches the current device.
     */
    EOL_PERSISTENCE_REVERIFICATION_REQUIRED,

    /*
     * A completed EOL record exists and the recorded EOL result was FAIL.
     */
    EOL_PERSISTENCE_FAILED

} eol_persistence_validity_t;


/* -------------------------------------------------------------------------- */
/* Persisted EOL record                                                       */
/* -------------------------------------------------------------------------- */

/*
 * This structure represents the factory EOL result stored in NVS.
 *
 * IMPORTANT:
 *
 * The previous record is retained when a fingerprint mismatch is
 * detected. A new record replaces it only after a new EOL run has
 * completed and the result has been explicitly saved.
 */
typedef struct
{
    uint32_t magic;

    uint16_t schema_version;

    uint16_t compatibility_version;


    /* ---------------------------------------------------------------------- */
    /* Factory result                                                         */
    /* ---------------------------------------------------------------------- */

    bool completed;

    bool passed;


    /* ---------------------------------------------------------------------- */
    /* EOL execution information                                             */
    /* ---------------------------------------------------------------------- */

    uint32_t completed_timestamp;

    uint32_t tests_passed;

    uint32_t tests_failed;

    uint32_t tests_skipped;

    uint32_t tests_not_run;


    /* ---------------------------------------------------------------------- */
    /* Identity                                                                */
    /* ---------------------------------------------------------------------- */

    /*
     * Hardware identity fingerprint.
     *
     * This represents the physical hardware/configuration that was
     * verified during the EOL run.
     */
    uint32_t hardware_fingerprint;

    /*
     * Firmware/build identity fingerprint.
     *
     * This is separate from the hardware fingerprint.
     */
    uint32_t firmware_fingerprint;

    /*
     * EOL compatibility version used when the test was performed.
     */
    uint32_t eol_compatibility_version;


    /* ---------------------------------------------------------------------- */
    /* Individual test results                                                */
    /* ---------------------------------------------------------------------- */

    uint8_t test_result[EOL_TEST_COUNT];


    /* ---------------------------------------------------------------------- */
    /* Diagnostic values                                                      */
    /* ---------------------------------------------------------------------- */

    uint16_t encoder_angle;

    float battery_voltage_v;

    float ina226_bus_voltage_v;

    float ina226_current_a;

    float ina226_power_w;


    /* ---------------------------------------------------------------------- */
    /* Firmware identification                                                */
    /* ---------------------------------------------------------------------- */

    char firmware_version[32];


} eol_persisted_record_t;


/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

/*
 * Initialize the EOL persistence subsystem.
 *
 * This opens the dedicated "eol" NVS namespace and loads the
 * previously stored factory result if one exists.
 *
 * This function does NOT erase an existing record.
 */
esp_err_t eol_persistence_init(void);


/*
 * Deinitialize the EOL persistence subsystem.
 */
esp_err_t eol_persistence_deinit(void);


/* -------------------------------------------------------------------------- */
/* Current identity                                                           */
/* -------------------------------------------------------------------------- */

/*
 * Set the current hardware identity fingerprint.
 *
 * The fingerprint represents the hardware/configuration currently
 * installed in the device.
 */
esp_err_t eol_persistence_set_hardware_fingerprint(
    uint32_t fingerprint);


/*
 * Set the current firmware identity fingerprint.
 */
esp_err_t eol_persistence_set_firmware_fingerprint(
    uint32_t fingerprint);


/*
 * Set the firmware version string associated with the current device.
 */
esp_err_t eol_persistence_set_firmware_version(
    const char *version);


/*
 * Get the currently configured hardware fingerprint.
 */
uint32_t eol_persistence_get_hardware_fingerprint(void);


/*
 * Get the currently configured firmware fingerprint.
 */
uint32_t eol_persistence_get_firmware_fingerprint(void);


/* -------------------------------------------------------------------------- */
/* Factory EOL record                                                         */
/* -------------------------------------------------------------------------- */

/*
 * Load the saved factory EOL record.
 *
 * Returns ESP_ERR_NOT_FOUND when no factory EOL record exists.
 */
esp_err_t eol_persistence_load(
    eol_persisted_record_t *record);


/*
 * Save a completed EOL result as the new factory record.
 *
 * The supplied result must represent a completed EOL run.
 *
 * This operation replaces the previous factory record only after
 * the new result has been successfully written and committed.
 */
esp_err_t eol_persistence_save_result(
    const eol_status_t *status);


/*
 * Clear the stored factory EOL record.
 *
 * This is an explicit maintenance operation and is NOT performed
 * automatically when identity changes.
 */
esp_err_t eol_persistence_clear(void);


/* -------------------------------------------------------------------------- */
/* Validity                                                                   */
/* -------------------------------------------------------------------------- */

/*
 * Determine the validity of the stored factory EOL result against
 * the currently configured hardware/firmware identity.
 */
eol_persistence_validity_t eol_persistence_get_validity(void);


/*
 * Return true when a valid factory EOL PASS exists for the current
 * hardware/configuration/compatibility identity.
 */
bool eol_persistence_is_valid(void);


/*
 * Return true when a new EOL run is required before the device can
 * be considered factory-verified.
 */
bool eol_persistence_is_reverification_required(void);


/*
 * Return true when no factory EOL result has ever been stored.
 */
bool eol_persistence_is_not_tested(void);


/*
 * Return true when the stored factory EOL result itself was FAIL.
 */
bool eol_persistence_is_failed(void);


/* -------------------------------------------------------------------------- */
/* Status                                                                      */
/* -------------------------------------------------------------------------- */

/*
 * Check whether the persistence manager is initialized.
 */
bool eol_persistence_is_initialized(void);


/*
 * Get the currently cached factory EOL record.
 *
 * Returns ESP_ERR_NOT_FOUND if no record exists.
 */
esp_err_t eol_persistence_get_record(
    eol_persisted_record_t *record);


/*
 * Get the stored hardware fingerprint.
 */
esp_err_t eol_persistence_get_stored_hardware_fingerprint(
    uint32_t *fingerprint);


/*
 * Get the stored firmware fingerprint.
 */
esp_err_t eol_persistence_get_stored_firmware_fingerprint(
    uint32_t *fingerprint);


/*
 * Get the stored EOL compatibility version.
 */
esp_err_t eol_persistence_get_stored_compatibility_version(
    uint32_t *version);


/* -------------------------------------------------------------------------- */
/* Re-verification                                                            */
/* -------------------------------------------------------------------------- */

/*
 * Evaluate the stored EOL record against the current identity.
 *
 * This function does not modify or erase the stored record.
 */
esp_err_t eol_persistence_validate_current_identity(void);


/*
 * Mark the current device as requiring EOL re-verification.
 *
 * This changes the in-memory validity state only.
 *
 * The previous factory result remains stored for traceability.
 */
esp_err_t eol_persistence_require_reverification(void);


/* -------------------------------------------------------------------------- */
/* Utility                                                                     */
/* -------------------------------------------------------------------------- */

/*
 * Calculate a deterministic fingerprint from a byte buffer.
 *
 * This is provided so the application can construct hardware and
 * configuration fingerprints without accessing NVS directly.
 */
uint32_t eol_persistence_calculate_fingerprint(
    const void *data,
    uint32_t length);


/*
 * Return the fixed EOL compatibility version used by this firmware.
 */
uint32_t eol_persistence_get_compatibility_version(void);


#ifdef __cplusplus
}
#endif