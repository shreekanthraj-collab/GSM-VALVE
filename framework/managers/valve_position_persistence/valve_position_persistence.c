#include "valve_position_persistence.h"

#include <stddef.h>
#include <string.h>

#include "hal_nvs.h"


/* -------------------------------------------------------------------------- */
/* NVS                                                                        */
/* -------------------------------------------------------------------------- */

#define VALVE_POSITION_PERSISTENCE_NAMESPACE \
    "valve_pos"

#define VALVE_POSITION_PERSISTENCE_KEY \
    "calibration"


/* -------------------------------------------------------------------------- */
/* Internal state                                                             */
/* -------------------------------------------------------------------------- */

static bool s_initialized = false;
static bool s_has_valid_record = false;


/* -------------------------------------------------------------------------- */
/* Checksum                                                                   */
/* -------------------------------------------------------------------------- */

static uint32_t calculate_checksum(
    const valve_position_persistence_record_t *record)
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

    data = (const uint8_t *)&record->closed_total_angle;
    size = sizeof(record->closed_total_angle);

    for (size_t i = 0U; i < size; ++i) {
        checksum ^= data[i];
        checksum *= 16777619U;
    }

    data = (const uint8_t *)&record->open_total_angle;
    size = sizeof(record->open_total_angle);

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
    const valve_position_persistence_record_t *record)
{
    if (record == NULL) {
        return false;
    }

    if (record->version !=
        VALVE_POSITION_PERSISTENCE_VERSION) {
        return false;
    }

    if (record->valid > 1U) {
        return false;
    }

    /*
     * A valid calibration must contain distinct
     * CLOSED and OPEN absolute positions.
     */
    if (record->valid != 0U &&
        record->closed_total_angle ==
            record->open_total_angle) {
        return false;
    }

    return record->checksum ==
           calculate_checksum(record);
}


/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

esp_err_t valve_position_persistence_init(void)
{
    esp_err_t err =
        hal_nvs_init();

    if (err != ESP_OK) {
        s_initialized = false;
        s_has_valid_record = false;

        return err;
    }

    s_initialized = true;
    s_has_valid_record = false;

    valve_position_persistence_record_t record =
        {0};

    err =
        valve_position_persistence_load(
            &record);

    if (err == ESP_OK) {
        s_has_valid_record = true;
        return ESP_OK;
    }

    if (err == ESP_ERR_NOT_FOUND) {
        return ESP_OK;
    }

    /*
     * A corrupt or incompatible record does not
     * make the persistence subsystem unusable.
     */
    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Save                                                                       */
/* -------------------------------------------------------------------------- */

esp_err_t valve_position_persistence_save(
    const valve_position_persistence_record_t *record)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (record == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    valve_position_persistence_record_t stored =
        *record;

    stored.version =
        VALVE_POSITION_PERSISTENCE_VERSION;

    if (stored.valid > 1U) {
        return ESP_ERR_INVALID_ARG;
    }

    if (stored.valid != 0U &&
        stored.closed_total_angle ==
            stored.open_total_angle) {
        return ESP_ERR_INVALID_ARG;
    }

    stored.checksum =
        calculate_checksum(&stored);

    nvs_handle_t handle = 0;

    esp_err_t err =
        hal_nvs_open(
            VALVE_POSITION_PERSISTENCE_NAMESPACE,
            NVS_READWRITE,
            &handle);

    if (err != ESP_OK) {
        return err;
    }

    err =
        hal_nvs_set_blob(
            handle,
            VALVE_POSITION_PERSISTENCE_KEY,
            &stored,
            sizeof(stored));

    if (err == ESP_OK) {
        err =
            hal_nvs_commit(handle);
    }

    hal_nvs_close(handle);

    if (err != ESP_OK) {
        return err;
    }

    s_has_valid_record =
        stored.valid != 0U;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Load                                                                       */
/* -------------------------------------------------------------------------- */

esp_err_t valve_position_persistence_load(
    valve_position_persistence_record_t *record)
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
            VALVE_POSITION_PERSISTENCE_NAMESPACE,
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

    err =
        hal_nvs_get_blob(
            handle,
            VALVE_POSITION_PERSISTENCE_KEY,
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

esp_err_t valve_position_persistence_erase(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    nvs_handle_t handle = 0;

    esp_err_t err =
        hal_nvs_open(
            VALVE_POSITION_PERSISTENCE_NAMESPACE,
            NVS_READWRITE,
            &handle);

    if (err != ESP_OK) {
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            s_has_valid_record = false;
            return ESP_OK;
        }

        return err;
    }

    err =
        hal_nvs_erase_key(
            handle,
            VALVE_POSITION_PERSISTENCE_KEY);

    if (err == ESP_ERR_NVS_NOT_FOUND) {
        err = ESP_OK;
    }

    if (err == ESP_OK) {
        err =
            hal_nvs_commit(handle);
    }

    hal_nvs_close(handle);

    if (err != ESP_OK) {
        return err;
    }

    s_has_valid_record = false;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Status                                                                     */
/* -------------------------------------------------------------------------- */

bool valve_position_persistence_is_initialized(void)
{
    return s_initialized;
}


bool valve_position_persistence_has_valid_record(void)
{
    return s_has_valid_record;
}