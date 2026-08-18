#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif


/* -------------------------------------------------------------------------- */
/* Modem state                                                                */
/* -------------------------------------------------------------------------- */

typedef enum
{
    DRV_MODEM_STATE_UNINITIALIZED = 0,

    DRV_MODEM_STATE_INITIALIZED,

    DRV_MODEM_STATE_READY,

    DRV_MODEM_STATE_NETWORK_SEARCH,

    DRV_MODEM_STATE_REGISTERED,

    DRV_MODEM_STATE_DATA_READY,

    DRV_MODEM_STATE_ERROR

} drv_modem_state_t;


/* -------------------------------------------------------------------------- */
/* Modem configuration                                                        */
/* -------------------------------------------------------------------------- */

typedef struct
{
    /*
     * UART used by the modem.
     */
    uart_port_t uart_port;

    /*
     * Physical UART pins.
     */
    int tx_gpio;
    int rx_gpio;

    /*
     * Optional modem control GPIOs.
     *
     * GPIO_NUM_NC disables the corresponding signal.
     */
    gpio_num_t pwrkey_gpio;
    gpio_num_t reset_gpio;
    gpio_num_t status_gpio;

    /*
     * UART parameters.
     */
    uint32_t baud_rate;

    uint32_t uart_rx_buffer_size;
    uint32_t uart_tx_buffer_size;

    uint32_t uart_timeout_ms;

    /*
     * AT command timeout.
     */
    uint32_t command_timeout_ms;

    /*
     * Maximum command/response buffer.
     */
    size_t response_buffer_size;

} drv_modem_config_t;


/* -------------------------------------------------------------------------- */
/* Modem information                                                          */
/* -------------------------------------------------------------------------- */

typedef struct
{
    char manufacturer[32];

    char model[64];

    char revision[64];

    char imei[32];

    char iccid[32];

    char operator_name[64];

} drv_modem_info_t;


/* -------------------------------------------------------------------------- */
/* Network status                                                             */
/* -------------------------------------------------------------------------- */

typedef struct
{
    bool registered;

    bool roaming;

    int registration_status;

    int signal_quality;

    bool packet_domain_attached;

    bool data_ready;

} drv_modem_network_status_t;


/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

esp_err_t drv_modem_init(
    const drv_modem_config_t *config);


/* -------------------------------------------------------------------------- */
/* Deinitialization                                                           */
/* -------------------------------------------------------------------------- */

esp_err_t drv_modem_deinit(void);


/* -------------------------------------------------------------------------- */
/* State                                                                      */
/* -------------------------------------------------------------------------- */

drv_modem_state_t drv_modem_get_state(void);

bool drv_modem_is_initialized(void);


/* -------------------------------------------------------------------------- */
/* AT command                                                                 */
/* -------------------------------------------------------------------------- */

/*
 * Send one AT command and wait for a response.
 *
 * command:
 *     Command without trailing CR/LF.
 *
 * response:
 *     Caller-provided response buffer.
 *
 * response_length:
 *     Size of response buffer.
 *
 * response_used:
 *     Actual number of bytes received.
 */
esp_err_t drv_modem_command(
    const char *command,
    char *response,
    size_t response_length,
    size_t *response_used,
    uint32_t timeout_ms);


/* -------------------------------------------------------------------------- */
/* Basic modem control                                                        */
/* -------------------------------------------------------------------------- */

/*
 * Send AT and verify that the modem responds.
 */
esp_err_t drv_modem_ping(void);


/*
 * Power-key operation.
 *
 * pulse_ms specifies the GPIO pulse duration.
 */
esp_err_t drv_modem_power_key(
    uint32_t pulse_ms);


/*
 * Hardware reset.
 */
esp_err_t drv_modem_reset(
    uint32_t pulse_ms);


/* -------------------------------------------------------------------------- */
/* Modem information                                                          */
/* -------------------------------------------------------------------------- */

esp_err_t drv_modem_get_info(
    drv_modem_info_t *info);


/* -------------------------------------------------------------------------- */
/* Network                                                                    */
/* -------------------------------------------------------------------------- */

esp_err_t drv_modem_get_network_status(
    drv_modem_network_status_t *status);


/*
 * Wait for packet-data/network registration.
 */
esp_err_t drv_modem_wait_for_network(
    uint32_t timeout_ms);


/* -------------------------------------------------------------------------- */
/* Data connection                                                            */
/* -------------------------------------------------------------------------- */

/*
 * Configure the modem APN.
 *
 * APN is copied into the modem driver only for the duration
 * of the command operation; persistent credential storage
 * belongs to the configuration/NVS layer.
 */
esp_err_t drv_modem_set_apn(
    const char *apn);


/*
 * Establish packet-data connection.
 */
esp_err_t drv_modem_connect_data(void);


/*
 * Disconnect packet-data connection.
 */
esp_err_t drv_modem_disconnect_data(void);


/*
 * Check whether packet data is currently available.
 */
bool drv_modem_is_data_ready(void);


/* -------------------------------------------------------------------------- */
/* Raw UART                                                                   */
/* -------------------------------------------------------------------------- */

/*
 * Send raw modem data.
 *
 * This is intended for higher-level modem protocols such as
 * HTTP/HTTPS streaming.
 */
esp_err_t drv_modem_write(
    const uint8_t *data,
    size_t length,
    size_t *bytes_written);


/*
 * Receive raw modem data.
 */
esp_err_t drv_modem_read(
    uint8_t *data,
    size_t length,
    size_t *bytes_read);


/*
 * Flush pending modem RX data.
 */
esp_err_t drv_modem_flush_rx(void);


#ifdef __cplusplus
}
#endif
