#include "hal_nvs.h"

#include "nvs_flash.h"


/* -------------------------------------------------------------------------- */
/* Validation                                                                 */
/* -------------------------------------------------------------------------- */

static bool valid_handle(nvs_handle_t handle)
{
    return handle != 0;
}


static bool valid_key(const char *key)
{
    return key != NULL &&
           key[0] != '\0';
}


static bool valid_namespace(const char *namespace_name)
{
    return namespace_name != NULL &&
           namespace_name[0] != '\0';
}


/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

esp_err_t hal_nvs_init(void)
{
    esp_err_t err = nvs_flash_init();

    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {

        err = nvs_flash_erase();

        if (err != ESP_OK) {
            return err;
        }

        err = nvs_flash_init();
    }

    return err;
}


/* -------------------------------------------------------------------------- */
/* Namespace                                                                  */
/* -------------------------------------------------------------------------- */

esp_err_t hal_nvs_open(
    const char *namespace_name,
    nvs_open_mode_t mode,
    nvs_handle_t *handle)
{
    if (!valid_namespace(namespace_name) ||
        handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *handle = 0;

    return nvs_open(
        namespace_name,
        mode,
        handle);
}


void hal_nvs_close(
    nvs_handle_t handle)
{
    if (!valid_handle(handle)) {
        return;
    }

    nvs_close(handle);
}


/* -------------------------------------------------------------------------- */
/* Commit                                                                     */
/* -------------------------------------------------------------------------- */

esp_err_t hal_nvs_commit(
    nvs_handle_t handle)
{
    if (!valid_handle(handle)) {
        return ESP_ERR_INVALID_ARG;
    }

    return nvs_commit(handle);
}


/* -------------------------------------------------------------------------- */
/* uint32                                                                     */
/* -------------------------------------------------------------------------- */

esp_err_t hal_nvs_get_u32(
    nvs_handle_t handle,
    const char *key,
    uint32_t *value)
{
    if (!valid_handle(handle) ||
        !valid_key(key) ||
        value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return nvs_get_u32(
        handle,
        key,
        value);
}


esp_err_t hal_nvs_set_u32(
    nvs_handle_t handle,
    const char *key,
    uint32_t value)
{
    if (!valid_handle(handle) ||
        !valid_key(key)) {
        return ESP_ERR_INVALID_ARG;
    }

    return nvs_set_u32(
        handle,
        key,
        value);
}


/* -------------------------------------------------------------------------- */
/* int32                                                                      */
/* -------------------------------------------------------------------------- */

esp_err_t hal_nvs_get_i32(
    nvs_handle_t handle,
    const char *key,
    int32_t *value)
{
    if (!valid_handle(handle) ||
        !valid_key(key) ||
        value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return nvs_get_i32(
        handle,
        key,
        value);
}


esp_err_t hal_nvs_set_i32(
    nvs_handle_t handle,
    const char *key,
    int32_t value)
{
    if (!valid_handle(handle) ||
        !valid_key(key)) {
        return ESP_ERR_INVALID_ARG;
    }

    return nvs_set_i32(
        handle,
        key,
        value);
}


/* -------------------------------------------------------------------------- */
/* float                                                                      */
/* -------------------------------------------------------------------------- */

esp_err_t hal_nvs_get_float(
    nvs_handle_t handle,
    const char *key,
    float *value)
{
    if (!valid_handle(handle) ||
        !valid_key(key) ||
        value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t size = sizeof(*value);

    return nvs_get_blob(
        handle,
        key,
        value,
        &size);
}


esp_err_t hal_nvs_set_float(
    nvs_handle_t handle,
    const char *key,
    float value)
{
    if (!valid_handle(handle) ||
        !valid_key(key)) {
        return ESP_ERR_INVALID_ARG;
    }

    return nvs_set_blob(
        handle,
        key,
        &value,
        sizeof(value));
}


/* -------------------------------------------------------------------------- */
/* String                                                                     */
/* -------------------------------------------------------------------------- */

esp_err_t hal_nvs_get_string(
    nvs_handle_t handle,
    const char *key,
    char *value,
    size_t value_size)
{
    if (!valid_handle(handle) ||
        !valid_key(key) ||
        value == NULL ||
        value_size == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t size = value_size;

    return nvs_get_str(
        handle,
        key,
        value,
        &size);
}


esp_err_t hal_nvs_set_string(
    nvs_handle_t handle,
    const char *key,
    const char *value)
{
    if (!valid_handle(handle) ||
        !valid_key(key) ||
        value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return nvs_set_str(
        handle,
        key,
        value);
}


/* -------------------------------------------------------------------------- */
/* Blob                                                                       */
/* -------------------------------------------------------------------------- */

esp_err_t hal_nvs_get_blob(
    nvs_handle_t handle,
    const char *key,
    void *data,
    size_t *size)
{
    if (!valid_handle(handle) ||
        !valid_key(key) ||
        size == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return nvs_get_blob(
        handle,
        key,
        data,
        size);
}


esp_err_t hal_nvs_set_blob(
    nvs_handle_t handle,
    const char *key,
    const void *data,
    size_t size)
{
    if (!valid_handle(handle) ||
        !valid_key(key) ||
        (data == NULL && size != 0)) {
        return ESP_ERR_INVALID_ARG;
    }

    return nvs_set_blob(
        handle,
        key,
        data,
        size);
}


/* -------------------------------------------------------------------------- */
/* Erase                                                                      */
/* -------------------------------------------------------------------------- */

esp_err_t hal_nvs_erase_key(
    nvs_handle_t handle,
    const char *key)
{
    if (!valid_handle(handle) ||
        !valid_key(key)) {
        return ESP_ERR_INVALID_ARG;
    }

    return nvs_erase_key(
        handle,
        key);
}


esp_err_t hal_nvs_erase_all(
    nvs_handle_t handle)
{
    if (!valid_handle(handle)) {
        return ESP_ERR_INVALID_ARG;
    }

    return nvs_erase_all(handle);
}