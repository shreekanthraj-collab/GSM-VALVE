#include "eol_persistence.h"

#include <inttypes.h>
#include <string.h>

#include "esp_log.h"

#include "hal_nvs.h"


/* -------------------------------------------------------------------------- */
/* Logging                                                                    */
/* -------------------------------------------------------------------------- */

static const char *TAG = "EOL_PERSIST";


/* -------------------------------------------------------------------------- */
/* NVS keys                                                                   */
/* -------------------------------------------------------------------------- */

#define EOL_NVS_KEY_RECORD        "record"


/* -------------------------------------------------------------------------- */
/* Runtime context                                                            */
/* -------------------------------------------------------------------------- */

typedef struct
{
    bool initialized;

    nvs_handle_t nvs_handle;

    bool record_loaded;

    eol_persisted_record_t record;

    uint32_t current_hardware_fingerprint;

    uint32_t current_firmware_fingerprint;

    char current_firmware_version[32];

    eol_persistence_validity_t validity;

} eol_persistence_context_t;


static eol_persistence_context_t s_ctx =
{
    .initialized = false,

    .nvs_handle = 0,

    .record_loaded = false,

    .record = {0},

    .current_hardware_fingerprint = 0U,

    .current_firmware_fingerprint = 0U,

    .current_firmware_version = {0},

    .validity = EOL_PERSISTENCE_NOT_TESTED
};


/* -------------------------------------------------------------------------- */
/* Internal helpers                                                           */
/* -------------------------------------------------------------------------- */

static void reset_cached_record(void)
{
    memset(
        &s_ctx.record,
        0,
        sizeof(s_ctx.record));

    s_ctx.record_loaded = false;
}


/* -------------------------------------------------------------------------- */

static bool record_is_structurally_valid(
    const eol_persisted_record_t *record)
{
    if (record == NULL) {
        return false;
    }

    if (record->magic != EOL_PERSISTENCE_MAGIC) {
        return false;
    }

    if (record->schema_version !=
        EOL_PERSISTENCE_SCHEMA_VERSION) {

        return false;
    }

    if (record->eol_compatibility_version == 0U) {
        return false;
    }

    if (record->eol_compatibility_version >
        EOL_COMPATIBILITY_VERSION) {

        return false;
    }

    return true;
}


/* -------------------------------------------------------------------------- */

static bool current_identity_matches_record(void)
{
    if (!s_ctx.record_loaded) {
        return false;
    }


    if (s_ctx.record.hardware_fingerprint !=
        s_ctx.current_hardware_fingerprint) {

        ESP_LOGW(
            TAG,
            "Hardware fingerprint mismatch: "
            "stored=0x%08" PRIX32
            " current=0x%08" PRIX32,
            s_ctx.record.hardware_fingerprint,
            s_ctx.current_hardware_fingerprint);

        return false;
    }


    if (s_ctx.record.firmware_fingerprint !=
        s_ctx.current_firmware_fingerprint) {

        ESP_LOGW(
            TAG,
            "Firmware fingerprint mismatch: "
            "stored=0x%08" PRIX32
            " current=0x%08" PRIX32,
            s_ctx.record.firmware_fingerprint,
            s_ctx.current_firmware_fingerprint);

        return false;
    }


    if (s_ctx.record.eol_compatibility_version !=
        EOL_COMPATIBILITY_VERSION) {

        ESP_LOGW(
            TAG,
            "EOL compatibility mismatch: "
            "stored=%" PRIu32
            " current=%u",
            s_ctx.record.eol_compatibility_version,
            EOL_COMPATIBILITY_VERSION);

        return false;
    }


    return true;
}


/* -------------------------------------------------------------------------- */

static void update_validity(void)
{
    if (!s_ctx.record_loaded) {

        s_ctx.validity =
            EOL_PERSISTENCE_NOT_TESTED;

        return;
    }


    if (!record_is_structurally_valid(
            &s_ctx.record)) {

        /*
         * Preserve the old NVS data.
         *
         * A malformed or incompatible record is never
         * automatically erased.
         */
        s_ctx.validity =
            EOL_PERSISTENCE_REVERIFICATION_REQUIRED;

        return;
    }


    if (!s_ctx.record.completed) {

        s_ctx.validity =
            EOL_PERSISTENCE_REVERIFICATION_REQUIRED;

        return;
    }


    if (!s_ctx.record.passed) {

        s_ctx.validity =
            EOL_PERSISTENCE_FAILED;

        return;
    }


    if (!current_identity_matches_record()) {

        s_ctx.validity =
            EOL_PERSISTENCE_REVERIFICATION_REQUIRED;

        return;
    }


       s_ctx.validity =
        EOL_PERSISTENCE_VALID;
}


/* -------------------------------------------------------------------------- */

static void copy_test_results(
    eol_persisted_record_t *record,
    const eol_status_t *status)
{
    if (record == NULL || status == NULL) {
        return;
    }


    for (size_t i = 0U;
         i < EOL_TEST_COUNT;
         ++i) {

        record->test_result[i] =
            (uint8_t)status->tests[i].result;
    }


    /*
     * Reset diagnostic values before copying.
     */
    record->encoder_angle =
        0U;

    record->battery_voltage_v =
        0.0f;

    record->ina226_bus_voltage_v =
        0.0f;

    record->ina226_current_a =
        0.0f;

    record->ina226_power_w =
        0.0f;


    /*
     * The current EOL result interface exposes one numeric
     * value per test.
     *
     * Encoder:
     *     value is treated as the reported angle.
     */
    if (status->tests[EOL_TEST_ENCODER].result !=
        EOL_RESULT_NOT_RUN) {

        float value =
            status->tests[EOL_TEST_ENCODER].value;

        if (value >= 0.0f &&
            value <= 65535.0f) {

            record->encoder_angle =
                (uint16_t)value;
        }
    }


    /*
     * Battery:
     *     value is treated as battery voltage.
     */
    if (status->tests[EOL_TEST_BATTERY].result !=
        EOL_RESULT_NOT_RUN) {

        record->battery_voltage_v =
            status->tests[EOL_TEST_BATTERY].value;
    }


    /*
     * INA226:
     *
     * The current EOL result structure only provides one
     * numeric value for each test. Therefore we store the
     * reported INA226 value as bus voltage for now.
     *
     * We do NOT invent current or power values.
     */
    if (status->tests[EOL_TEST_INA226].result !=
        EOL_RESULT_NOT_RUN) {

        record->ina226_bus_voltage_v =
            status->tests[EOL_TEST_INA226].value;
    }
}


/* -------------------------------------------------------------------------- */

static esp_err_t write_record(
    const eol_persisted_record_t *record)
{
    if (!s_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (record == NULL) {
        return ESP_ERR_INVALID_ARG;
    }


    esp_err_t err =
        hal_nvs_set_blob(
            s_ctx.nvs_handle,
            EOL_NVS_KEY_RECORD,
            record,
            sizeof(*record));

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "Failed to write EOL record: %s",
            esp_err_to_name(err));

        return err;
    }


    err =
        hal_nvs_commit(
            s_ctx.nvs_handle);

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "Failed to commit EOL record: %s",
            esp_err_to_name(err));

        return err;
    }


    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

esp_err_t eol_persistence_init(void)
{
    if (s_ctx.initialized) {
        return ESP_OK;
    }


    esp_err_t err =
        hal_nvs_open(
            EOL_PERSISTENCE_NAMESPACE,
            NVS_READWRITE,
            &s_ctx.nvs_handle);

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "NVS namespace open failed: %s",
            esp_err_to_name(err));

        return err;
    }


    s_ctx.initialized =
        true;

    reset_cached_record();


    /*
     * Load the existing factory EOL record.
     *
     * No record is a normal condition for a new device.
     */
    size_t size =
        sizeof(s_ctx.record);

    err =
        hal_nvs_get_blob(
            s_ctx.nvs_handle,
            EOL_NVS_KEY_RECORD,
            &s_ctx.record,
            &size);


    if (err == ESP_ERR_NVS_NOT_FOUND) {

        s_ctx.validity =
            EOL_PERSISTENCE_NOT_TESTED;

        ESP_LOGI(
            TAG,
            "No factory EOL record found");

        return ESP_OK;
    }


    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "Failed to read EOL record: %s",
            esp_err_to_name(err));

        hal_nvs_close(
            s_ctx.nvs_handle);

        s_ctx.nvs_handle =
            0;

        s_ctx.initialized =
            false;

        reset_cached_record();

        return err;
    }


    /*
     * Protect against an older/different record layout.
     *
     * Do not erase it.
     */
    if (size != sizeof(s_ctx.record)) {

        ESP_LOGW(
            TAG,
            "EOL record size mismatch: "
            "stored=%u expected=%u",
            (unsigned)size,
            (unsigned)sizeof(s_ctx.record));

        reset_cached_record();

        s_ctx.validity =
            EOL_PERSISTENCE_REVERIFICATION_REQUIRED;

        return ESP_OK;
    }


    s_ctx.record_loaded =
        true;


    update_validity();


    ESP_LOGI(
        TAG,
        "Factory EOL record loaded: "
        "completed=%s "
        "passed=%s "
        "validity=%d",
        s_ctx.record.completed ? "yes" : "no",
        s_ctx.record.passed ? "yes" : "no",
        (int)s_ctx.validity);


    return ESP_OK;
}


/* -------------------------------------------------------------------------- */

esp_err_t eol_persistence_deinit(void)
{
    if (!s_ctx.initialized) {
        return ESP_OK;
    }


    hal_nvs_close(
        s_ctx.nvs_handle);


    s_ctx.nvs_handle =
        0;

    s_ctx.initialized =
        false;


    reset_cached_record();


    s_ctx.current_hardware_fingerprint =
        0U;

    s_ctx.current_firmware_fingerprint =
        0U;


    memset(
        s_ctx.current_firmware_version,
        0,
        sizeof(s_ctx.current_firmware_version));


    s_ctx.validity =
        EOL_PERSISTENCE_NOT_TESTED;


    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Current identity                                                           */
/* -------------------------------------------------------------------------- */

esp_err_t eol_persistence_set_hardware_fingerprint(
    uint32_t fingerprint)
{
    s_ctx.current_hardware_fingerprint =
        fingerprint;

    update_validity();

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */

esp_err_t eol_persistence_set_firmware_fingerprint(
    uint32_t fingerprint)
{
    s_ctx.current_firmware_fingerprint =
        fingerprint;

    update_validity();

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */

esp_err_t eol_persistence_set_firmware_version(
    const char *version)
{
    if (version == NULL) {
        return ESP_ERR_INVALID_ARG;
    }


    memset(
        s_ctx.current_firmware_version,
        0,
        sizeof(s_ctx.current_firmware_version));


    strncpy(
        s_ctx.current_firmware_version,
        version,
        sizeof(s_ctx.current_firmware_version) - 1U);


    return ESP_OK;
}


/* -------------------------------------------------------------------------- */

uint32_t eol_persistence_get_hardware_fingerprint(void)
{
    return s_ctx.current_hardware_fingerprint;
}


/* -------------------------------------------------------------------------- */

uint32_t eol_persistence_get_firmware_fingerprint(void)
{
    return s_ctx.current_firmware_fingerprint;
}


/* -------------------------------------------------------------------------- */
/* Factory EOL record                                                         */
/* -------------------------------------------------------------------------- */

esp_err_t eol_persistence_load(
    eol_persisted_record_t *record)
{
    if (record == NULL) {
        return ESP_ERR_INVALID_ARG;
    }


    if (!s_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }


    if (!s_ctx.record_loaded) {
        return ESP_ERR_NOT_FOUND;
    }


    memcpy(
        record,
        &s_ctx.record,
        sizeof(*record));


    return ESP_OK;
}


/* -------------------------------------------------------------------------- */

esp_err_t eol_persistence_save_result(
    const eol_status_t *status)
{
    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }


    if (!s_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }


    if (status->state !=
        EOL_STATE_COMPLETE) {

        ESP_LOGE(
            TAG,
            "Cannot save EOL result: "
            "state=%d is not COMPLETE",
            (int)status->state);

        return ESP_ERR_INVALID_STATE;
    }


    if (status->overall_result ==
        EOL_OVERALL_NOT_RUN) {

        return ESP_ERR_INVALID_STATE;
    }


    /*
     * Construct the new record completely in RAM first.
     *
     * The previous NVS record is untouched until the
     * complete new record is successfully committed.
     */
    eol_persisted_record_t new_record;

    memset(
        &new_record,
        0,
        sizeof(new_record));


    new_record.magic =
        EOL_PERSISTENCE_MAGIC;


    new_record.schema_version =
        EOL_PERSISTENCE_SCHEMA_VERSION;


    new_record.compatibility_version =
        (uint16_t)EOL_COMPATIBILITY_VERSION;


    new_record.completed =
        true;


    new_record.passed =
        status->overall_result ==
        EOL_OVERALL_PASS;


    new_record.completed_timestamp =
        status->completed_ms;


    new_record.tests_passed =
        status->tests_passed;


    new_record.tests_failed =
        status->tests_failed;


    new_record.tests_skipped =
        status->tests_skipped;


    new_record.tests_not_run =
        status->tests_not_run;


    new_record.hardware_fingerprint =
        s_ctx.current_hardware_fingerprint;


    new_record.firmware_fingerprint =
        s_ctx.current_firmware_fingerprint;


    new_record.eol_compatibility_version =
        EOL_COMPATIBILITY_VERSION;


    copy_test_results(
        &new_record,
        status);


    strncpy(
        new_record.firmware_version,
        s_ctx.current_firmware_version,
        sizeof(new_record.firmware_version) - 1U);


    /*
     * Write and commit the complete new factory record.
     */
    esp_err_t err =
        write_record(
            &new_record);

    if (err != ESP_OK) {
        return err;
    }


    /*
     * Update the cached record only after the NVS
     * transaction has completed successfully.
     */
    memcpy(
        &s_ctx.record,
        &new_record,
        sizeof(new_record));


    s_ctx.record_loaded =
        true;


    update_validity();


    ESP_LOGI(
        TAG,
        "Factory EOL result saved: %s",
        new_record.passed ? "PASS" : "FAIL");


    return ESP_OK;
}


/* -------------------------------------------------------------------------- */

esp_err_t eol_persistence_clear(void)
{
    if (!s_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }


    esp_err_t err =
        hal_nvs_erase_key(
            s_ctx.nvs_handle,
            EOL_NVS_KEY_RECORD);


    if (err == ESP_ERR_NVS_NOT_FOUND) {

        reset_cached_record();

        s_ctx.validity =
            EOL_PERSISTENCE_NOT_TESTED;

        return ESP_OK;
    }


    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "Failed to erase EOL record: %s",
            esp_err_to_name(err));

        return err;
    }


    err =
        hal_nvs_commit(
            s_ctx.nvs_handle);


    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "Failed to commit EOL record erase: %s",
            esp_err_to_name(err));

        return err;
    }


    reset_cached_record();


    s_ctx.validity =
        EOL_PERSISTENCE_NOT_TESTED;


    ESP_LOGI(
        TAG,
        "Factory EOL record cleared");


    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Validity                                                                   */
/* -------------------------------------------------------------------------- */

eol_persistence_validity_t
eol_persistence_get_validity(void)
{
    update_validity();

    return s_ctx.validity;
}


/* -------------------------------------------------------------------------- */

bool eol_persistence_is_valid(void)
{
    return
        eol_persistence_get_validity() ==
        EOL_PERSISTENCE_VALID;
}


/* -------------------------------------------------------------------------- */

bool eol_persistence_is_reverification_required(void)
{
    return
        eol_persistence_get_validity() ==
        EOL_PERSISTENCE_REVERIFICATION_REQUIRED;
}


/* -------------------------------------------------------------------------- */

bool eol_persistence_is_not_tested(void)
{
    return
        eol_persistence_get_validity() ==
        EOL_PERSISTENCE_NOT_TESTED;
}


/* -------------------------------------------------------------------------- */

bool eol_persistence_is_failed(void)
{
    return
        eol_persistence_get_validity() ==
        EOL_PERSISTENCE_FAILED;
}


/* -------------------------------------------------------------------------- */
/* Status                                                                     */
/* -------------------------------------------------------------------------- */

bool eol_persistence_is_initialized(void)
{
    return s_ctx.initialized;
}


/* -------------------------------------------------------------------------- */

esp_err_t eol_persistence_get_record(
    eol_persisted_record_t *record)
{
    return
        eol_persistence_load(
            record);
}


/* -------------------------------------------------------------------------- */

esp_err_t eol_persistence_get_stored_hardware_fingerprint(
    uint32_t *fingerprint)
{
    if (fingerprint == NULL) {
        return ESP_ERR_INVALID_ARG;
    }


    if (!s_ctx.record_loaded) {
        return ESP_ERR_NOT_FOUND;
    }


    *fingerprint =
        s_ctx.record.hardware_fingerprint;


    return ESP_OK;
}


/* -------------------------------------------------------------------------- */

esp_err_t eol_persistence_get_stored_firmware_fingerprint(
    uint32_t *fingerprint)
{
    if (fingerprint == NULL) {
        return ESP_ERR_INVALID_ARG;
    }


    if (!s_ctx.record_loaded) {
        return ESP_ERR_NOT_FOUND;
    }


    *fingerprint =
        s_ctx.record.firmware_fingerprint;


    return ESP_OK;
}


/* -------------------------------------------------------------------------- */

esp_err_t eol_persistence_get_stored_compatibility_version(
    uint32_t *version)
{
    if (version == NULL) {
        return ESP_ERR_INVALID_ARG;
    }


    if (!s_ctx.record_loaded) {
        return ESP_ERR_NOT_FOUND;
    }


    *version =
        s_ctx.record.eol_compatibility_version;


    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Re-verification                                                            */
/* -------------------------------------------------------------------------- */

esp_err_t eol_persistence_validate_current_identity(void)
{
    if (!s_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }


    update_validity();


    return ESP_OK;
}


/* -------------------------------------------------------------------------- */

esp_err_t eol_persistence_require_reverification(void)
{
    if (!s_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }


    /*
     * Deliberately do not alter or erase the NVS record.
     *
     * The previous factory result remains available for
     * manufacturing traceability.
     */
    if (s_ctx.record_loaded) {

        s_ctx.validity =
            EOL_PERSISTENCE_REVERIFICATION_REQUIRED;

    }
    else {

        s_ctx.validity =
            EOL_PERSISTENCE_NOT_TESTED;
    }


    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Fingerprint                                                                */
/* -------------------------------------------------------------------------- */

uint32_t eol_persistence_calculate_fingerprint(
    const void *data,
    uint32_t length)
{
    /*
     * FNV-1a 32-bit.
     *
     * Deterministic and lightweight.
     *
     * This is an identity/change detector, not a security
     * or cryptographic authentication mechanism.
     */
    const uint8_t *bytes =
        (const uint8_t *)data;


    uint32_t hash =
        2166136261UL;


    if (bytes == NULL &&
        length != 0U) {

        return 0U;
    }


    for (uint32_t i = 0U;
         i < length;
         ++i) {

        hash ^=
            bytes[i];

        hash *=
            16777619UL;
    }


    return hash;
}


/* -------------------------------------------------------------------------- */

uint32_t eol_persistence_get_compatibility_version(void)
{
    return
        EOL_COMPATIBILITY_VERSION;
}