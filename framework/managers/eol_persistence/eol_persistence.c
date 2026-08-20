#include "eol_persistence.h"

#include <string.h>

#include "esp_log.h"
#include "nvs.h"

#include "hal_nvs.h"
#include "eol_manager.h"


/* -------------------------------------------------------------------------- */
/* Logging                                                                    */
/* -------------------------------------------------------------------------- */

static const char *TAG = "EOL_PERSIST";


/* -------------------------------------------------------------------------- */
/* NVS                                                                         */
/* -------------------------------------------------------------------------- */

#define EOL_PERSISTENCE_NAMESPACE       "eol"

#define EOL_NVS_KEY_RECORD              "record"

#define EOL_NVS_KEY_FACTORY_CONFIG     "factory_cfg"
#define EOL_NVS_KEY_FACTORY_CONFIG_VER "factory_cfg_ver"

#define EOL_FACTORY_CONFIG_VERSION     1U


/* -------------------------------------------------------------------------- */
/* Internal context                                                           */
/* -------------------------------------------------------------------------- */

typedef struct
{
    bool initialized;

    nvs_handle_t nvs_handle;

    eol_persisted_record_t record;

    eol_persistence_validity_t validity;

    uint32_t hardware_fingerprint;

    uint32_t firmware_fingerprint;

    char firmware_version[
        EOL_PERSISTENCE_FIRMWARE_VERSION_MAX_LEN];

} eol_persistence_context_t;


static eol_persistence_context_t s_ctx;


/* -------------------------------------------------------------------------- */
/* Internal helpers                                                           */
/* -------------------------------------------------------------------------- */

static void reset_cached_record(void)
{
    memset(
        &s_ctx.record,
        0,
        sizeof(s_ctx.record));

    s_ctx.validity =
        EOL_PERSISTENCE_NOT_TESTED;
}


static bool valid_record(
    const eol_persisted_record_t *record)
{
    if (record == NULL) {
        return false;
    }

    /*
     * A factory record is considered structurally valid when:
     *
     *   - it contains the expected record version
     *   - it represents a completed EOL run
     *   - its test counts are internally consistent
     */
    if (record->record_version !=
        EOL_PERSISTENCE_RECORD_VERSION) {
        return false;
    }

    if (record->overall_result !=
        EOL_OVERALL_PASS &&
        record->overall_result !=
        EOL_OVERALL_FAIL) {
        return false;
    }

    if (record->tests_passed +
        record->tests_failed +
        record->tests_skipped +
        record->tests_not_run >
        EOL_TEST_COUNT) {
        return false;
    }

    return true;
}


static bool identity_matches(
    const eol_persisted_record_t *record)
{
    if (record == NULL) {
        return false;
    }

    if (record->hardware_fingerprint !=
        s_ctx.hardware_fingerprint) {
        return false;
    }

    if (record->firmware_fingerprint !=
        s_ctx.firmware_fingerprint) {
        return false;
    }

    if (record->compatibility_version !=
        EOL_PERSISTENCE_COMPATIBILITY_VERSION) {
        return false;
    }

    return true;
}


static void update_validity(void)
{
    if (!valid_record(&s_ctx.record)) {

        s_ctx.validity =
            EOL_PERSISTENCE_NOT_TESTED;

        return;
    }

    if (s_ctx.record.overall_result !=
        EOL_OVERALL_PASS) {

        s_ctx.validity =
            EOL_PERSISTENCE_FAILED;

        return;
    }

    if (!identity_matches(&s_ctx.record)) {

        s_ctx.validity =
            EOL_PERSISTENCE_REVERIFICATION_REQUIRED;

        return;
    }

    s_ctx.validity =
        EOL_PERSISTENCE_VALID;
}


/* -------------------------------------------------------------------------- */
/* Record creation                                                            */
/* -------------------------------------------------------------------------- */

static void populate_record_from_status(
    eol_persisted_record_t *record,
    const eol_status_t *status)
{
    if (record == NULL ||
        status == NULL) {
        return;
    }

    memset(
        record,
        0,
        sizeof(*record));

    record->record_version =
        EOL_PERSISTENCE_RECORD_VERSION;

    record->compatibility_version =
        EOL_PERSISTENCE_COMPATIBILITY_VERSION;

    record->hardware_fingerprint =
        s_ctx.hardware_fingerprint;

    record->firmware_fingerprint =
        s_ctx.firmware_fingerprint;

    strncpy(
        record->firmware_version,
        s_ctx.firmware_version,
        sizeof(record->firmware_version) - 1U);

    record->firmware_version[
        sizeof(record->firmware_version) - 1U] =
        '\0';

    record->overall_result =
        status->overall_result;

    record->started_ms =
        status->started_ms;

    record->completed_ms =
        status->completed_ms;

    record->tests_passed =
        status->tests_passed;

    record->tests_failed =
        status->tests_failed;

    record->tests_skipped =
        status->tests_skipped;

    record->tests_not_run =
        status->tests_not_run;


    /*
     * Encoder:
     *
     * The EOL result numeric value is treated as
     * the reported encoder angle.
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
     *
     * The EOL result numeric value is treated
     * as battery voltage.
     */
    if (status->tests[EOL_TEST_BATTERY].result !=
        EOL_RESULT_NOT_RUN) {

        record->battery_voltage_v =
            status->tests[EOL_TEST_BATTERY].value;
    }


    /*
     * INA226:
     *
     * Current EOL result structure provides only
     * one numeric value for this test.
     *
     * Preserve existing behavior and store it
     * as bus voltage.
     *
     * Do not invent current or power values.
     */
    if (status->tests[EOL_TEST_INA226].result !=
        EOL_RESULT_NOT_RUN) {

        record->ina226_bus_voltage_v =
            status->tests[EOL_TEST_INA226].value;
    }
}


/* -------------------------------------------------------------------------- */
/* Record write                                                               */
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
/* Factory configuration persistence                                          */
/* -------------------------------------------------------------------------- */

esp_err_t eol_persistence_save_factory_config(
    const eol_factory_config_t *config)
{
    if (!s_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }


    /*
     * The EOL manager owns the safety validation.
     *
     * Persistence must never allow an invalid configuration
     * to reach NVS.
     */
    esp_err_t err =
        eol_manager_validate_factory_config(config);

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "Factory configuration rejected: %s",
            esp_err_to_name(err));

        return err;
    }


    /*
     * Write the configuration blob first.
     */
    err =
        hal_nvs_set_blob(
            s_ctx.nvs_handle,
            EOL_NVS_KEY_FACTORY_CONFIG,
            config,
            sizeof(*config));

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "Failed to write factory configuration: %s",
            esp_err_to_name(err));

        return err;
    }


    /*
     * Write the configuration schema/version.
     */
    err =
        hal_nvs_set_u32(
            s_ctx.nvs_handle,
            EOL_NVS_KEY_FACTORY_CONFIG_VER,
            EOL_FACTORY_CONFIG_VERSION);

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "Failed to write factory configuration version: %s",
            esp_err_to_name(err));

        return err;
    }


    /*
     * Commit both values together.
     */
    err =
        hal_nvs_commit(
            s_ctx.nvs_handle);

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "Failed to commit factory configuration: %s",
            esp_err_to_name(err));

        return err;
    }


    ESP_LOGI(
        TAG,
        "Factory configuration saved");

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */

esp_err_t eol_persistence_load_factory_config(
    eol_factory_config_t *config)
{
    if (!s_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }


    memset(
        config,
        0,
        sizeof(*config));


    uint32_t version = 0U;

    esp_err_t err =
        hal_nvs_get_u32(
            s_ctx.nvs_handle,
            EOL_NVS_KEY_FACTORY_CONFIG_VER,
            &version);

    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_ERR_NOT_FOUND;
    }

    if (err != ESP_OK) {
        return err;
    }


    /*
     * Reject configuration formats that this firmware
     * does not understand.
     */
    if (version != EOL_FACTORY_CONFIG_VERSION) {

        ESP_LOGW(
            TAG,
            "Unsupported factory configuration version: %lu",
            (unsigned long)version);

        return ESP_ERR_INVALID_VERSION;
    }


    size_t size =
        sizeof(*config);

    err =
        hal_nvs_get_blob(
            s_ctx.nvs_handle,
            EOL_NVS_KEY_FACTORY_CONFIG,
            config,
            &size);

    if (err == ESP_ERR_NVS_NOT_FOUND) {

        memset(
            config,
            0,
            sizeof(*config));

        return ESP_ERR_NOT_FOUND;
    }

    if (err != ESP_OK) {

        memset(
            config,
            0,
            sizeof(*config));

        return err;
    }


    /*
     * Protect against a blob created by another
     * structure/version.
     */
    if (size != sizeof(*config)) {

        ESP_LOGE(
            TAG,
            "Factory configuration size mismatch: "
            "stored=%u expected=%u",
            (unsigned)size,
            (unsigned)sizeof(*config));

        memset(
            config,
            0,
            sizeof(*config));

        return ESP_ERR_INVALID_SIZE;
    }


    /*
     * Always validate data after loading from NVS.
     *
     * This protects against:
     *
     *   - corrupted NVS data
     *   - obsolete configuration
     *   - invalid values
     *   - values outside firmware safety limits
     */
    err =
        eol_manager_validate_factory_config(config);

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "Stored factory configuration failed validation");

        memset(
            config,
            0,
            sizeof(*config));

        return ESP_ERR_INVALID_STATE;
    }


    return ESP_OK;
}


/* -------------------------------------------------------------------------- */

esp_err_t eol_persistence_has_factory_config(
    bool *present)
{
    if (!s_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (present == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *present = false;


    uint32_t version = 0U;

    esp_err_t err =
        hal_nvs_get_u32(
            s_ctx.nvs_handle,
            EOL_NVS_KEY_FACTORY_CONFIG_VER,
            &version);

    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_OK;
    }

    if (err != ESP_OK) {
        return err;
    }


    if (version != EOL_FACTORY_CONFIG_VERSION) {
        return ESP_OK;
    }


    size_t size = 0U;

    err =
        hal_nvs_get_blob(
            s_ctx.nvs_handle,
            EOL_NVS_KEY_FACTORY_CONFIG,
            NULL,
            &size);

    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_OK;
    }

    if (err != ESP_OK) {
        return err;
    }


    if (size == sizeof(eol_factory_config_t)) {
        *present = true;
    }


    return ESP_OK;
}


/* -------------------------------------------------------------------------- */

esp_err_t eol_persistence_clear_factory_config(void)
{
    if (!s_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }


    esp_err_t err =
        hal_nvs_erase_key(
            s_ctx.nvs_handle,
            EOL_NVS_KEY_FACTORY_CONFIG);

    if (err != ESP_OK &&
        err != ESP_ERR_NVS_NOT_FOUND) {

        return err;
    }


    err =
        hal_nvs_erase_key(
            s_ctx.nvs_handle,
            EOL_NVS_KEY_FACTORY_CONFIG_VER);

    if (err != ESP_OK &&
        err != ESP_ERR_NVS_NOT_FOUND) {

        return err;
    }


    return hal_nvs_commit(
        s_ctx.nvs_handle);
}


/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

esp_err_t eol_persistence_init(void)
{
    if (s_ctx.initialized) {
        return ESP_OK;
    }


    memset(
        &s_ctx,
        0,
        sizeof(s_ctx));

    s_ctx.nvs_handle = 0;


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
     * No record is normal on a new device.
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

        reset_cached_record();

        return err;
    }


    if (size != sizeof(s_ctx.record)) {

        ESP_LOGE(
            TAG,
            "EOL record size mismatch: stored=%u expected=%u",
            (unsigned)size,
            (unsigned)sizeof(s_ctx.record));

        reset_cached_record();

        return ESP_ERR_INVALID_SIZE;
    }


    if (!valid_record(&s_ctx.record)) {

        ESP_LOGW(
            TAG,
            "Stored EOL record is structurally invalid");

        s_ctx.validity =
            EOL_PERSISTENCE_FAILED;

        return ESP_OK;
    }


    update_validity();


    ESP_LOGI(
        TAG,
        "Factory EOL record loaded");

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Deinitialization                                                           */
/* -------------------------------------------------------------------------- */

esp_err_t eol_persistence_deinit(void)
{
    if (!s_ctx.initialized) {
        return ESP_OK;
    }


    hal_nvs_close(
        s_ctx.nvs_handle);


    memset(
        &s_ctx,
        0,
        sizeof(s_ctx));


    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Current identity                                                           */
/* -------------------------------------------------------------------------- */

esp_err_t eol_persistence_set_hardware_fingerprint(
    uint32_t fingerprint)
{
    if (!s_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }


    s_ctx.hardware_fingerprint =
        fingerprint;

    update_validity();

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */

esp_err_t eol_persistence_set_firmware_fingerprint(
    uint32_t fingerprint)
{
    if (!s_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }


    s_ctx.firmware_fingerprint =
        fingerprint;

    update_validity();

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */

esp_err_t eol_persistence_set_firmware_version(
    const char *version)
{
    if (!s_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (version == NULL) {
        return ESP_ERR_INVALID_ARG;
    }


    strncpy(
        s_ctx.firmware_version,
        version,
        sizeof(s_ctx.firmware_version) - 1U);

    s_ctx.firmware_version[
        sizeof(s_ctx.firmware_version) - 1U] =
        '\0';


    update_validity();

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */

uint32_t eol_persistence_get_hardware_fingerprint(void)
{
    return s_ctx.hardware_fingerprint;
}


/* -------------------------------------------------------------------------- */

uint32_t eol_persistence_get_firmware_fingerprint(void)
{
    return s_ctx.firmware_fingerprint;
}


/* -------------------------------------------------------------------------- */
/* Current identity validation                                                */
/* -------------------------------------------------------------------------- */

esp_err_t eol_persistence_validate_current_identity(void)
{
    if (!s_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!valid_record(&s_ctx.record)) {
        return ESP_ERR_NOT_FOUND;
    }

    if (!identity_matches(&s_ctx.record)) {
        return ESP_ERR_INVALID_STATE;
    }

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Factory EOL record                                                         */
/* -------------------------------------------------------------------------- */

esp_err_t eol_persistence_load(
    eol_persisted_record_t *record)
{
    if (!s_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (record == NULL) {
        return ESP_ERR_INVALID_ARG;
    }


    if (s_ctx.validity ==
        EOL_PERSISTENCE_NOT_TESTED) {

        return ESP_ERR_NOT_FOUND;
    }


    *record =
        s_ctx.record;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */

esp_err_t eol_persistence_save_result(
    const eol_status_t *status)
{
    if (!s_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }


    if (status->state !=
        EOL_STATE_COMPLETE) {

        ESP_LOGE(
            TAG,
            "Cannot save incomplete EOL result");

        return ESP_ERR_INVALID_STATE;
    }


    if (status->overall_result !=
        EOL_OVERALL_PASS &&
        status->overall_result !=
        EOL_OVERALL_FAIL) {

        ESP_LOGE(
            TAG,
            "Invalid EOL overall result");

        return ESP_ERR_INVALID_ARG;
    }


    eol_persisted_record_t record;

    populate_record_from_status(
        &record,
        status);


    if (!valid_record(&record)) {

        ESP_LOGE(
            TAG,
            "Generated EOL record is invalid");

        return ESP_ERR_INVALID_ARG;
    }


    /*
     * Write first.
     *
     * Cached state is changed only after NVS write
     * and commit succeed.
     */
    esp_err_t err =
        write_record(&record);

    if (err != ESP_OK) {
        return err;
    }


    s_ctx.record =
        record;

    update_validity();


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
        err = ESP_OK;
    }


    if (err != ESP_OK) {
        return err;
    }


    err =
        hal_nvs_commit(
            s_ctx.nvs_handle);

    if (err != ESP_OK) {
        return err;
    }


    reset_cached_record();


    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Validity                                                                   */
/* -------------------------------------------------------------------------- */

eol_persistence_validity_t
eol_persistence_get_validity(void)
{
    return s_ctx.validity;
}


/* -------------------------------------------------------------------------- */

bool eol_persistence_is_valid(void)
{
    return s_ctx.validity ==
           EOL_PERSISTENCE_VALID;
}


/* -------------------------------------------------------------------------- */

bool eol_persistence_is_reverification_required(void)
{
    return s_ctx.validity ==
           EOL_PERSISTENCE_REVERIFICATION_REQUIRED;
}


/* -------------------------------------------------------------------------- */

bool eol_persistence_is_not_tested(void)
{
    return s_ctx.validity ==
           EOL_PERSISTENCE_NOT_TESTED;
}


/* -------------------------------------------------------------------------- */

bool eol_persistence_is_failed(void)
{
    return s_ctx.validity ==
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
    if (!s_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (record == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_ctx.validity ==
        EOL_PERSISTENCE_NOT_TESTED) {

        return ESP_ERR_NOT_FOUND;
    }


    *record =
        s_ctx.record;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */

esp_err_t eol_persistence_get_stored_hardware_fingerprint(
    uint32_t *fingerprint)
{
    if (!s_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (fingerprint == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_ctx.validity ==
        EOL_PERSISTENCE_NOT_TESTED) {

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
    if (!s_ctx.initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (fingerprint == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_ctx.validity ==
        EOL_PERSISTENCE_NOT_TESTED) {

        return ESP_ERR_NOT_FOUND;
    }


    *fingerprint =
        s_ctx.record.firmware_fingerprint;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */

uint32_t eol_persistence_get_stored_compatibility_version(void)
{
    if (s_ctx.validity ==
        EOL_PERSISTENCE_NOT_TESTED) {

        return 0U;
    }


    return s_ctx.record.compatibility_version;
}


/* -------------------------------------------------------------------------- */

const char *eol_persistence_get_stored_firmware_version(void)
{
    if (s_ctx.validity ==
        EOL_PERSISTENCE_NOT_TESTED) {

        return NULL;
    }


    return s_ctx.record.firmware_version;
}