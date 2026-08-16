#include "drv_rs485.h"

#include <stddef.h>

#include "board_config.h"
#include "hal_gpio.h"
#include "hal_uart.h"


/* -------------------------------------------------------------------------- */
/* Constants                                                                  */
/* -------------------------------------------------------------------------- */

#define DRV_RS485_DE_TX      HAL_GPIO_LEVEL_HIGH
#define DRV_RS485_DE_RX      HAL_GPIO_LEVEL_LOW


/* -------------------------------------------------------------------------- */
/* Internal state                                                             */
/* -------------------------------------------------------------------------- */

static bool s_initialized = false;

static drv_rs485_config_t s_config = {0};


/* -------------------------------------------------------------------------- */
/* Validation                                                                 */
/* -------------------------------------------------------------------------- */

static bool valid_config(
    const drv_rs485_config_t *config)
{
    if (config == NULL) {
        return false;
    }

    if (config->port < UART_NUM_0 ||
        config->port >= UART_NUM_MAX) {
        return false;
    }

    if (config->baud_rate == 0U) {
        return false;
    }

    if (config->rx_buffer_size == 0U ||
        config->tx_buffer_size == 0U) {
        return false;
    }

    if (config->data_bits != UART_DATA_5_BITS &&
        config->data_bits != UART_DATA_6_BITS &&
        config->data_bits != UART_DATA_7_BITS &&
        config->data_bits != UART_DATA_8_BITS) {
        return false;
    }

    return true;
}


/* -------------------------------------------------------------------------- */
/* Direction control                                                          */
/* -------------------------------------------------------------------------- */

static esp_err_t set_receive_mode(void)
{
    return hal_gpio_write(
        HAL_GPIO_SIGNAL_RS485_DE,
        DRV_RS485_DE_RX);
}


static esp_err_t set_transmit_mode(void)
{
    return hal_gpio_write(
        HAL_GPIO_SIGNAL_RS485_DE,
        DRV_RS485_DE_TX);
}


/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

esp_err_t drv_rs485_init(
    const drv_rs485_config_t *config)
{
    if (s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!valid_config(config)) {
        return ESP_ERR_INVALID_ARG;
    }

    /*
     * Configure the RS485 transceiver direction pin.
     */
    esp_err_t err =
        hal_gpio_configure(
            HAL_GPIO_SIGNAL_RS485_DE,
            HAL_GPIO_MODE_OUTPUT,
            HAL_GPIO_PULL_NONE);

    if (err != ESP_OK) {
        return err;
    }

    /*
     * Always enter receive mode before UART ownership
     * is established.
     */
    err =
        set_receive_mode();

    if (err != ESP_OK) {
        return err;
    }

    /*
     * UART configuration is delegated to the frozen
     * UART HAL.
     */
    hal_uart_config_t uart_config = {
        .port = config->port,
        .tx_gpio = BOARD_RS485_TX_GPIO,
        .rx_gpio = BOARD_RS485_RX_GPIO,
        .baud_rate = config->baud_rate,
        .data_bits = config->data_bits,
        .parity = config->parity,
        .stop_bits = config->stop_bits,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0U,
        .rx_buffer_size = config->rx_buffer_size,
        .tx_buffer_size = config->tx_buffer_size,
        .timeout_ms = config->timeout_ms
    };

    err =
        hal_uart_init(&uart_config);

    if (err != ESP_OK) {
        (void)set_receive_mode();
        return err;
    }

    s_config = *config;
    s_initialized = true;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Deinitialization                                                           */
/* -------------------------------------------------------------------------- */

esp_err_t drv_rs485_deinit(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    /*
     * Never leave the transceiver in transmit mode.
     */
    esp_err_t err =
        set_receive_mode();

    if (err != ESP_OK) {
        return err;
    }

    err =
        hal_uart_deinit(s_config.port);

    if (err != ESP_OK) {
        return err;
    }

    s_config = (drv_rs485_config_t){0};
    s_initialized = false;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Write                                                                      */
/* -------------------------------------------------------------------------- */

esp_err_t drv_rs485_write(
    const uint8_t *data,
    size_t length,
    size_t *bytes_written)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (data == NULL || length == 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    if (bytes_written != NULL) {
        *bytes_written = 0U;
    }

    /*
     * Enable RS485 transmitter.
     */
    esp_err_t err =
        set_transmit_mode();

    if (err != ESP_OK) {
        return err;
    }

    size_t written = 0U;

    /*
     * Write through the UART HAL.
     */
    err =
        hal_uart_write(
            s_config.port,
            data,
            length,
            &written);

    if (err != ESP_OK) {
        /*
         * Safety: return to receive mode on failure.
         */
        (void)set_receive_mode();
        return err;
    }

    /*
     * Wait until the UART hardware has physically
     * transmitted the complete frame.
     */
    err =
        hal_uart_wait_tx_done(
            s_config.port,
            s_config.timeout_ms);

    if (err != ESP_OK) {
        /*
         * Safety: return to receive mode on failure.
         */
        (void)set_receive_mode();
        return err;
    }

    /*
     * Only switch back after physical TX completion.
     */
    err =
        set_receive_mode();

    if (err != ESP_OK) {
        return err;
    }

    if (bytes_written != NULL) {
        *bytes_written = written;
    }

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Read                                                                       */
/* -------------------------------------------------------------------------- */

esp_err_t drv_rs485_read(
    uint8_t *data,
    size_t length,
    size_t *bytes_read)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (data == NULL || length == 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    if (bytes_read != NULL) {
        *bytes_read = 0U;
    }

    /*
     * Ensure the transceiver is in receive mode.
     */
    esp_err_t err =
        set_receive_mode();

    if (err != ESP_OK) {
        return err;
    }

    /*
     * Receive through the UART HAL.
     *
     * Zero bytes on timeout is returned as ESP_OK
     * by the UART HAL.
     */
    return hal_uart_read(
        s_config.port,
        data,
        length,
        bytes_read);
}


/* -------------------------------------------------------------------------- */
/* Flush RX                                                                   */
/* -------------------------------------------------------------------------- */

esp_err_t drv_rs485_flush_rx(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err =
        set_receive_mode();

    if (err != ESP_OK) {
        return err;
    }

    return hal_uart_flush_rx(
        s_config.port);
}


/* -------------------------------------------------------------------------- */
/* State                                                                      */
/* -------------------------------------------------------------------------- */

bool drv_rs485_is_initialized(void)
{
    return s_initialized;
}