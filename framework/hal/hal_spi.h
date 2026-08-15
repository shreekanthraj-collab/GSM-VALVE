#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/spi_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    spi_host_device_t host;

    int sclk_gpio;
    int mosi_gpio;
    int miso_gpio;

    int max_transfer_sz;

    int dma_chan;
} hal_spi_bus_config_t;

typedef struct {
    spi_host_device_t host;

    int cs_gpio;

    int clock_speed_hz;

    uint8_t mode;

    int queue_size;


} hal_spi_device_config_t;


/*
 * Initialize an SPI bus.
 *
 * One HAL owner is permitted per SPI host.
 */
esp_err_t hal_spi_bus_init(
    const hal_spi_bus_config_t *config);


/*
 * Deinitialize an SPI bus.
 *
 * The bus is released only after successful
 * SPI driver removal.
 *
 * All devices must be removed first.
 */
esp_err_t hal_spi_bus_deinit(
    spi_host_device_t host);


/*
 * Attach an SPI device to an initialized bus.
 *
 * The resulting device handle is returned through
 * the handle argument.
 */
esp_err_t hal_spi_device_add(
    const hal_spi_device_config_t *config,
    spi_device_handle_t *handle);


/*
 * Remove an SPI device.
 */
esp_err_t hal_spi_device_remove(
    spi_device_handle_t handle);


/*
 * Perform a blocking SPI transaction.
 *
 * length_bytes is the number of bytes transmitted
 * and/or received.
 */
esp_err_t hal_spi_transmit(
    spi_device_handle_t handle,
    const void *tx_data,
    void *rx_data,
    size_t length_bytes);


/*
 * Check whether an SPI host is initialized.
 */
bool hal_spi_bus_is_initialized(
    spi_host_device_t host);

#ifdef __cplusplus
}
#endif