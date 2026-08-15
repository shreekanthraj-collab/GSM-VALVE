#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t frequency_hz;
    uint32_t timeout_ms;
} hal_i2c_config_t;

esp_err_t hal_i2c_init(const hal_i2c_config_t *config);
esp_err_t hal_i2c_deinit(void);

esp_err_t hal_i2c_write(
    uint8_t address,
    const uint8_t *data,
    size_t length);

esp_err_t hal_i2c_read(
    uint8_t address,
    uint8_t *data,
    size_t length);

esp_err_t hal_i2c_write_read(
    uint8_t address,
    const uint8_t *write_data,
    size_t write_length,
    uint8_t *read_data,
    size_t read_length);

esp_err_t hal_i2c_probe(uint8_t address);

esp_err_t hal_i2c_recover(void);

bool hal_i2c_is_initialized(void);

#ifdef __cplusplus
}
#endif