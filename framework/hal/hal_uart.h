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
/* UART configuration                                                         */
/* -------------------------------------------------------------------------- */

typedef struct
{
    uart_port_t port;

    int tx_gpio;
    int rx_gpio;

    uint32_t baud_rate;

    uart_word_length_t data_bits;

    uart_parity_t parity;

    uart_stop_bits_t stop_bits;

    uart_hw_flowcontrol_t flow_ctrl;

    uint8_t rx_flow_ctrl_thresh;

    uint32_t rx_buffer_size;

    uint32_t tx_buffer_size;

    uint32_t timeout_ms;

} hal_uart_config_t;


/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

/*
 * Initialize one UART port.
 *
 * Ownership rule:
 *
 *   One HAL owner is permitted per UART port.
 *
 * A second initialization attempt for the same UART port
 * is rejected.
 */
esp_err_t hal_uart_init(
    const hal_uart_config_t *config);


/* -------------------------------------------------------------------------- */
/* Deinitialization                                                           */
/* -------------------------------------------------------------------------- */

/*
 * Deinitialize the UART port.
 *
 * The HAL handle/state is detached only after
 * uart_driver_delete() succeeds.
 */
esp_err_t hal_uart_deinit(
    uart_port_t port);


/* -------------------------------------------------------------------------- */
/* Transmit                                                                   */
/* -------------------------------------------------------------------------- */

/*
 * Transmit bytes.
 *
 * The function is protected by the HAL TX mutex.
 *
 * bytes_written may be NULL when the caller does not
 * require the number of bytes accepted by the UART driver.
 */
esp_err_t hal_uart_write(
    uart_port_t port,
    const uint8_t *data,
    size_t length,
    size_t *bytes_written);


/* -------------------------------------------------------------------------- */
/* Receive                                                                    */
/* -------------------------------------------------------------------------- */

/*
 * Receive bytes.
 *
 * Blocking receive with timeout specified by the
 * UART HAL configuration.
 *
 * length specifies the maximum number of bytes that
 * may be received.
 *
 * bytes_read receives the actual number of bytes read.
 *
 * A timeout with zero bytes available is not considered
 * a driver failure. In that case ESP_OK is returned and
 * *bytes_read is zero.
 */
esp_err_t hal_uart_read(
    uart_port_t port,
    uint8_t *data,
    size_t length,
    size_t *bytes_read);


/* -------------------------------------------------------------------------- */
/* Receive buffer                                                             */
/* -------------------------------------------------------------------------- */

/*
 * Flush the UART receive buffer.
 */
esp_err_t hal_uart_flush_rx(
    uart_port_t port);


/* -------------------------------------------------------------------------- */
/* Transmit completion                                                        */
/* -------------------------------------------------------------------------- */

/*
 * Wait until all queued TX data has physically completed.
 *
 * timeout_ms specifies the maximum wait time.
 */
esp_err_t hal_uart_wait_tx_done(
    uart_port_t port,
    uint32_t timeout_ms);


/* -------------------------------------------------------------------------- */
/* State                                                                      */
/* -------------------------------------------------------------------------- */

/*
 * Check whether a UART port is currently owned and
 * initialized by this HAL.
 */
bool hal_uart_is_initialized(
    uart_port_t port);


#ifdef __cplusplus
}
#endif