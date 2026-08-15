#include "encoder_persistence_manager.h"

#include <stddef.h>
#include <string.h>

#include "hal_nvs.h"


#define ENCODER_PERSISTENCE_NAMESPACE "encoder"
#define ENCODER_PERSISTENCE_KEY       "position"


static bool s_initialized = false;
static bool s_has_valid_record = false;


/* -------------------------------------------------------------------------- */
/* Checksum                                                                   */
/* -------------------------------------------------------------------------- */

static uint32_t calculate_checksum(
    const encoder_persistence_record_t *record)
{
    uint32_t checksum = 2166136261U;

    const uint8_t *data;
    size_t size;

    data = (const uint8_t *)&record->version;
    size = sizeof(record->version);

    for (size_t i = 0U; i < size; ++i) {
        checksum ^= data[i];
        checksum *= 16777619U;
    }

    data = (const uint8_t *)&record->last_angle;
    size = sizeof(record->last_angle);

    for (size_t i = 0U; i < size; ++i) {
        checksum ^= data[i];
        checksum *= 16777619U;
    }

    data = (const uint8_t *)&record->rotation_count;
    size = sizeof(record->rotation_count);

    for (size_t i = 0U; i < size; ++i) {
        checksum ^= data[i];
        checksum *= 16777619U;
    }

    data = (const uint8_t *)&record->total_angle;
    size = sizeof(record->total_angle);

    for (size_t i = 0U; i < size; ++i) {
        checksum ^= data[i];
        checksum *= 16777619U;
    }

    data = (const uint8_t *)&record->valid;
    size = sizeof(record->valid);

    for (size_t i = 0U; i < size; ++i) {
        checksum ^= data[i];
        checksum *= 16777619U;
    }

    return checksum;
}


/* -------------------------------------------------------------------------- */
/* Validation                                                                 */
/* -------------------------------------------------------------------------- */

static bool valid_record(
    const encoder_persistence_record_t *record)
{
    if (record == NULL) {
        return false;
    }

    if (record->version !=
        ENCODER_PERSISTENCE_VERSION) {
        return false;
    }

    if (record->last_angle > 4095U) {
        return false;
    }

    if (record->valid > 1U) {
        return false;
    }

    return record->checksum ==
           calculate_checksum(record);
}


/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

esp_err_t encoder_persistence_init(void)
{
    esp_err_t err = hal_nvs_init();

    if (err != ESP_OK) {
        s_initialized = false;
        s_has_valid_record = false;

        return err;
    }

    s_initialized = true;
    s_has_valid_record = false;

    /*
     * Determine whether a valid persistent record
     * already exists.
     */
    encoder_persistence_record_t record = {0};

    err = encoder_persistence_load(&record);

    if (err == ESP_OK) {
        s_has_valid_record = true;
        return ESP_OK;
    }

    if (err == ESP_ERR_NOT_FOUND) {
        return ESP_OK;
    }

    /*
     * A corrupt or incompatible record does not make
     * the persistence subsystem itself unusable.
     */
    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Save                                                                       */
/* -------------------------------------------------------------------------- */

esp_err_t encoder_persistence_save(
    const encoder_persistence_record_t *record)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (record == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    encoder_persistence_record_t stored =
        *record;

    stored.version =
        ENCODER_PERSISTENCE_VERSION;

    if (stored.last_angle > 4095U) {
        return ESP_ERR_INVALID_ARG;
    }

    if (stored.valid > 1U) {
        return ESP_ERR_INVALID_ARG;
    }

    stored.checksum =
        calculate_checksum(&stored);

    nvs_handle_t handle = 0;

    esp_err_t err =
        hal_nvs_open(
            ENCODER_PERSISTENCE_NAMESPACE,
            NVS_READWRITE,
            &handle);

    if (err != ESP_OK) {
        return err;
    }

    err = hal_nvs_set_blob(
        handle,
        ENCODER_PERSISTENCE_KEY,
        &stored,
        sizeof(stored));

    if (err == ESP_OK) {
        err = hal_nvs_commit(handle);
    }

    hal_nvs_close(handle);

    if (err != ESP_OK) {
        return err;
    }

    s_has_valid_record = true;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Load                                                                       */
/* -------------------------------------------------------------------------- */

esp_err_t encoder_persistence_load(
    encoder_persistence_record_t *record)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (record == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(
        record,
        0,
        sizeof(*record));

    nvs_handle_t handle = 0;

    esp_err_t err =
        hal_nvs_open(
            ENCODER_PERSISTENCE_NAMESPACE,
            NVS_READONLY,
            &handle);

    if (err != ESP_OK) {
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            return ESP_ERR_NOT_FOUND;
        }

        return err;
    }

    size_t size =
        sizeof(*record);

    err = hal_nvs_get_blob(
        handle,
        ENCODER_PERSISTENCE_KEY,
        record,
        &size);

    hal_nvs_close(handle);

    if (err == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_ERR_NOT_FOUND;
    }

    if (err != ESP_OK) {
        return err;
    }

    if (size != sizeof(*record)) {
        memset(
            record,
            0,
            sizeof(*record));

        return ESP_ERR_INVALID_SIZE;
    }

    if (!valid_record(record)) {
        memset(
            record,
            0,
            sizeof(*record));

        return ESP_ERR_INVALID_RESPONSE;
    }

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Erase                                                                      */
/* -------------------------------------------------------------------------- */

esp_err_t encoder_persistence_erase(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    nvs_handle_t handle = 0;

    esp_err_t err =
        hal_nvs_open(
            ENCODER_PERSISTENCE_NAMESPACE,
            NVS_READWRITE,
            &handle);

    if (err != ESP_OK) {
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            s_has_valid_record = false;
            return ESP_OK;
        }

        return err;
    }

    err = hal_nvs_erase_key(
        handle,
        ENCODER_PERSISTENCE_KEY);

    if (err == ESP_ERR_NVS_NOT_FOUND) {
        err = ESP_OK;
    }

    if (err == ESP_OK) {
        err = hal_nvs_commit(handle);
    }

    hal_nvs_close(handle);

    if (err != ESP_OK) {
        return err;
    }

    s_has_valid_record = false;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* State                                                                      */
/* -------------------------------------------------------------------------- */

bool encoder_persistence_is_initialized(void)
{
    return s_initialized;
}


bool encoder_persistence_has_valid_record(void)
{
    return s_has_valid_record;
}