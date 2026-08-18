#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "nvs.h"

#ifdef __cplusplus
extern "C" {
#endif


/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

/*
 * Initialize the NVS subsystem.
 *
 * Handles the standard ESP-IDF NVS initialization
 * and recovery from a full/new partition when required.
 */
esp_err_t hal_nvs_init(void);


/* -------------------------------------------------------------------------- */
/* Namespace                                                                  */
/* -------------------------------------------------------------------------- */

/*
 * Open an NVS namespace.
 *
 * The returned handle must be closed with
 * hal_nvs_close().
 */
esp_err_t hal_nvs_open(
    const char *namespace_name,
    nvs_open_mode_t mode,
    nvs_handle_t *handle);


/*
 * Close an NVS handle.
 */
void hal_nvs_close(
    nvs_handle_t handle);


/*
 * Commit pending changes for an NVS handle.
 */
esp_err_t hal_nvs_commit(
    nvs_handle_t handle);


/* -------------------------------------------------------------------------- */
/* Unsigned 32-bit value                                                      */
/* -------------------------------------------------------------------------- */

esp_err_t hal_nvs_get_u32(
    nvs_handle_t handle,
    const char *key,
    uint32_t *value);

esp_err_t hal_nvs_set_u32(
    nvs_handle_t handle,
    const char *key,
    uint32_t value);


/* -------------------------------------------------------------------------- */
/* Signed 32-bit value                                                        */
/* -------------------------------------------------------------------------- */

esp_err_t hal_nvs_get_i32(
    nvs_handle_t handle,
    const char *key,
    int32_t *value);

esp_err_t hal_nvs_set_i32(
    nvs_handle_t handle,
    const char *key,
    int32_t value);


/* -------------------------------------------------------------------------- */
/* Floating-point value                                                       */
/* -------------------------------------------------------------------------- */

esp_err_t hal_nvs_get_float(
    nvs_handle_t handle,
    const char *key,
    float *value);

esp_err_t hal_nvs_set_float(
    nvs_handle_t handle,
    const char *key,
    float value);


/* -------------------------------------------------------------------------- */
/* String value                                                               */
/* -------------------------------------------------------------------------- */

/*
 * hal_nvs_get_string() requires the caller to provide
 * the destination buffer and its size.
 */
esp_err_t hal_nvs_get_string(
    nvs_handle_t handle,
    const char *key,
    char *value,
    size_t value_size);

esp_err_t hal_nvs_set_string(
    nvs_handle_t handle,
    const char *key,
    const char *value);


/* -------------------------------------------------------------------------- */
/* Binary blob                                                                 */
/* -------------------------------------------------------------------------- */

/*
 * Read a binary blob from NVS.
 *
 * The caller supplies the destination buffer and its size.
 *
 * On input:
 *
 *     *size = size of destination buffer
 *
 * On success:
 *
 *     *size = number of bytes actually read
 */
esp_err_t hal_nvs_get_blob(
    nvs_handle_t handle,
    const char *key,
    void *data,
    size_t *size);


/*
 * Write a binary blob to NVS.
 */
esp_err_t hal_nvs_set_blob(
    nvs_handle_t handle,
    const char *key,
    const void *data,
    size_t size);


/* -------------------------------------------------------------------------- */
/* Key management                                                             */
/* -------------------------------------------------------------------------- */

/*
 * Erase one key.
 */
esp_err_t hal_nvs_erase_key(
    nvs_handle_t handle,
    const char *key);


/*
 * Erase all keys in the currently opened namespace.
 */
esp_err_t hal_nvs_erase_all(
    nvs_handle_t handle);


#ifdef __cplusplus
}
#endif