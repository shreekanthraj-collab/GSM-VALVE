#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/uart.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif


/* -------------------------------------------------------------------------- */
/* Configuration                                                              */
/* -------------------------------------------------------------------------- */

typedef struct
{
    uart_port_t port;

    uint32_t baud_rate;

    uart_word_length_t data_bits;
    uart_parity_t parity;
    uart_stop_bits_t stop_bits;

    uint32_t rx_buffer_size;
    uint32_t tx_buffer_size;

    uint32_t timeout_ms;
} drv_rs485_config_t;


/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

/*
 * Initialize the RS485 physical-layer driver.
 *
 * The underlying UART HAL must not already be owned by another component.
 *
 * RS485 direction control is handled internally through
 * HAL_GPIO_SIGNAL_RS485_DE.
 */
esp_err_t drv_rs485_init(
    const drv_rs485_config_t *config);


/*
 * Deinitialize the RS485 driver.
 *
 * The UART HAL is released after successful deinitialization.
 */
esp_err_t drv_rs485_deinit(void);


/* -------------------------------------------------------------------------- */
/* Data transfer                                                              */
/* -------------------------------------------------------------------------- */

/*
 * Transmit raw bytes over RS485.
 *
 * The driver:
 *   1. switches the transceiver to transmit mode,
 *   2. writes the bytes,
 *   3. waits for physical TX completion,
 *   4. switches the transceiver back to receive mode.
 */
esp_err_t drv_rs485_write(
    const uint8_t *data,
    size_t length,
    size_t *bytes_written);


/*
 * Receive raw bytes from RS485.
 *
 * The transceiver remains in receive mode.
 *
 * A receive timeout with zero bytes available is returned
 * as ESP_OK with bytes_read == 0.
 */
esp_err_t drv_rs485_read(
    uint8_t *data,
    size_t length,
    size_t *bytes_read);


/*
 * Flush all pending bytes from the RS485 receive buffer.
 */
esp_err_t drv_rs485_flush_rx(void);


/* -------------------------------------------------------------------------- */
/* State                                                                      */
/* -------------------------------------------------------------------------- */

/*
 * Check whether the RS485 driver is initialized.
 */
bool drv_rs485_is_initialized(void);


#ifdef __cplusplus
}
#endif