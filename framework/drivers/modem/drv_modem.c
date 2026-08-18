#include "drv_modem.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "hal_uart.h"


/* -------------------------------------------------------------------------- */
/* Private definitions                                                        */
/* -------------------------------------------------------------------------- */

static const char *TAG = "DRV_MODEM";

#define MODEM_DEFAULT_BAUD_RATE              115200U
#define MODEM_DEFAULT_RX_BUFFER_SIZE         4096U
#define MODEM_DEFAULT_TX_BUFFER_SIZE         2048U
#define MODEM_DEFAULT_UART_TIMEOUT_MS        1000U
#define MODEM_DEFAULT_COMMAND_TIMEOUT_MS     3000U
#define MODEM_DEFAULT_RESPONSE_BUFFER_SIZE   1024U

#define MODEM_CR                            '\r'
#define MODEM_LF                            '\n'

#define MODEM_RESPONSE_OK                  "OK"
#define MODEM_RESPONSE_ERROR               "ERROR"


/* -------------------------------------------------------------------------- */
/* Private state                                                              */
/* -------------------------------------------------------------------------- */

static bool s_initialized = false;

static drv_modem_state_t s_state =
    DRV_MODEM_STATE_UNINITIALIZED;

static drv_modem_config_t s_config = {
    .uart_port = UART_NUM_2,

    .tx_gpio = -1,
    .rx_gpio = -1,

    .pwrkey_gpio = GPIO_NUM_NC,
    .reset_gpio = GPIO_NUM_NC,
    .status_gpio = GPIO_NUM_NC,

    .baud_rate = MODEM_DEFAULT_BAUD_RATE,

    .uart_rx_buffer_size =
        MODEM_DEFAULT_RX_BUFFER_SIZE,

    .uart_tx_buffer_size =
        MODEM_DEFAULT_TX_BUFFER_SIZE,

    .uart_timeout_ms =
        MODEM_DEFAULT_UART_TIMEOUT_MS,

    .command_timeout_ms =
        MODEM_DEFAULT_COMMAND_TIMEOUT_MS,

    .response_buffer_size =
        MODEM_DEFAULT_RESPONSE_BUFFER_SIZE
};


/* -------------------------------------------------------------------------- */
/* Private helpers                                                            */
/* -------------------------------------------------------------------------- */

static bool valid_config(
    const drv_modem_config_t *config)
{
    if (config == NULL) {
        return false;
    }

    if (config->tx_gpio < 0 ||
        config->rx_gpio < 0) {

        return false;
    }

    if (config->baud_rate == 0U) {
        return false;
    }

    if (config->uart_rx_buffer_size == 0U ||
        config->uart_tx_buffer_size == 0U) {

        return false;
    }

    if (config->uart_timeout_ms == 0U ||
        config->command_timeout_ms == 0U) {

        return false;
    }

    if (config->response_buffer_size < 16U) {
        return false;
    }

    return true;
}


static bool response_contains_ok(
    const char *response)
{
    if (response == NULL) {
        return false;
    }

    return strstr(
        response,
        MODEM_RESPONSE_OK) != NULL;
}


static bool response_contains_error(
    const char *response)
{
    if (response == NULL) {
        return false;
    }

    return strstr(
        response,
        MODEM_RESPONSE_ERROR) != NULL;
}


static esp_err_t modem_configure_control_gpio(
    gpio_num_t gpio,
    bool output)
{
    if (gpio == GPIO_NUM_NC) {
        return ESP_OK;
    }

    if (output) {

        return gpio_config(
            &(gpio_config_t){
                .pin_bit_mask =
                    (1ULL << gpio),

                .mode =
                    GPIO_MODE_OUTPUT,

                .pull_up_en =
                    GPIO_PULLUP_DISABLE,

                .pull_down_en =
                    GPIO_PULLDOWN_DISABLE,

                .intr_type =
                    GPIO_INTR_DISABLE
            });
    }

    return gpio_config(
        &(gpio_config_t){
            .pin_bit_mask =
                (1ULL << gpio),

            .mode =
                GPIO_MODE_INPUT,

            .pull_up_en =
                GPIO_PULLUP_DISABLE,

            .pull_down_en =
                GPIO_PULLDOWN_DISABLE,

            .intr_type =
                GPIO_INTR_DISABLE
        });
}


static esp_err_t modem_send_command_raw(
    const char *command)
{
    if (command == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t length =
        strlen(command);

    size_t written = 0U;

    esp_err_t err =
        hal_uart_write(
            s_config.uart_port,
            (const uint8_t *)command,
            length,
            &written);

    if (err != ESP_OK) {
        return err;
    }

    if (written != length) {
        return ESP_FAIL;
    }

    static const uint8_t crlf[] = {
        MODEM_CR,
        MODEM_LF
    };

    written = 0U;

    err =
        hal_uart_write(
            s_config.uart_port,
            crlf,
            sizeof(crlf),
            &written);

    if (err != ESP_OK) {
        return err;
    }

    if (written != sizeof(crlf)) {
        return ESP_FAIL;
    }

    return hal_uart_wait_tx_done(
        s_config.uart_port,
        s_config.uart_timeout_ms);
}


static esp_err_t modem_read_response(
    char *response,
    size_t response_length,
    size_t *response_used,
    uint32_t timeout_ms)
{
    if (response == NULL ||
        response_length == 0U) {

        return ESP_ERR_INVALID_ARG;
    }

    if (response_used != NULL) {
        *response_used = 0U;
    }

    response[0] = '\0';

    uint32_t elapsed_ms = 0U;

    size_t used = 0U;

    while (elapsed_ms < timeout_ms) {

        if (used + 1U >= response_length) {
            break;
        }

        size_t bytes_read = 0U;

        esp_err_t err =
            hal_uart_read(
                s_config.uart_port,
                (uint8_t *)&response[used],
                response_length - used - 1U,
                &bytes_read);

        if (err != ESP_OK) {
            return err;
        }

        if (bytes_read > 0U) {

            used += bytes_read;

            response[used] =
                '\0';

            if (response_contains_ok(response) ||
                response_contains_error(response)) {

                break;
            }
        }

        /*
         * hal_uart_read() uses the configured UART timeout.
         * We therefore advance the software timeout in
         * that same interval.
         */
        if (s_config.uart_timeout_ms >=
            timeout_ms - elapsed_ms) {

            elapsed_ms = timeout_ms;

        } else {

            elapsed_ms +=
                s_config.uart_timeout_ms;
        }

        if (bytes_read == 0U) {
            vTaskDelay(
                pdMS_TO_TICKS(10U));

            if (elapsed_ms + 10U >= timeout_ms) {
                elapsed_ms = timeout_ms;
            } else {
                elapsed_ms += 10U;
            }
        }
    }

    if (response_used != NULL) {
        *response_used = used;
    }

    if (used == 0U) {
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}


static esp_err_t modem_command_expect_ok(
    const char *command,
    char *response,
    size_t response_length,
    uint32_t timeout_ms)
{
    size_t response_used = 0U;

    esp_err_t err =
        drv_modem_command(
            command,
            response,
            response_length,
            &response_used,
            timeout_ms);

    if (err != ESP_OK) {
        return err;
    }

    if (response_contains_error(response)) {
        return ESP_FAIL;
    }

    if (!response_contains_ok(response)) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    return ESP_OK;
}


static bool parse_csq(
    const char *response,
    int *signal_quality)
{
    if (response == NULL ||
        signal_quality == NULL) {

        return false;
    }

    const char *p =
        strstr(response, "+CSQ:");

    if (p == NULL) {
        return false;
    }

    int rssi = 0;
    int ber = 0;

    if (sscanf(
            p,
            "+CSQ: %d,%d",
            &rssi,
            &ber) != 2) {

        return false;
    }

    *signal_quality =
        rssi;

    return true;
}


static bool parse_creg(
    const char *response,
    int *registration_status)
{
    if (response == NULL ||
        registration_status == NULL) {

        return false;
    }

    const char *p =
        strstr(response, "+CREG:");

    if (p == NULL) {
        return false;
    }

    int mode = 0;
    int stat = 0;

    if (sscanf(
            p,
            "+CREG: %d,%d",
            &mode,
            &stat) == 2) {

        *registration_status =
            stat;

        return true;
    }

    if (sscanf(
            p,
            "+CREG: %d",
            &stat) == 1) {

        *registration_status =
            stat;

        return true;
    }

    return false;
}


/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

esp_err_t drv_modem_init(
    const drv_modem_config_t *config)
{
    if (s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!valid_config(config)) {

        ESP_LOGE(
            TAG,
            "Invalid modem configuration");

        return ESP_ERR_INVALID_ARG;
    }

    s_config =
        *config;

    /*
     * Configure optional modem control GPIOs.
     */
    esp_err_t err =
        modem_configure_control_gpio(
            s_config.pwrkey_gpio,
            true);

    if (err != ESP_OK) {
        return err;
    }

    err =
        modem_configure_control_gpio(
            s_config.reset_gpio,
            true);

    if (err != ESP_OK) {
        return err;
    }

    err =
        modem_configure_control_gpio(
            s_config.status_gpio,
            false);

    if (err != ESP_OK) {
        return err;
    }

    /*
     * Keep PWRKEY and RESET inactive.
     *
     * Actual polarity is modem-specific and will be
     * defined by the selected modem profile.
     */
    if (s_config.pwrkey_gpio != GPIO_NUM_NC) {

        gpio_set_level(
            s_config.pwrkey_gpio,
            1);
    }

    if (s_config.reset_gpio != GPIO_NUM_NC) {

        gpio_set_level(
            s_config.reset_gpio,
            1);
    }

    /*
     * Initialize the UART through the existing HAL.
     *
     * The modem driver does not directly install or delete
     * the ESP-IDF UART driver.
     */
    const hal_uart_config_t uart_config = {

        .port =
            s_config.uart_port,

        .tx_gpio =
            s_config.tx_gpio,

        .rx_gpio =
            s_config.rx_gpio,

        .baud_rate =
            s_config.baud_rate,

        .data_bits =
            UART_DATA_8_BITS,

        .parity =
            UART_PARITY_DISABLE,

        .stop_bits =
            UART_STOP_BITS_1,

        .flow_ctrl =
            UART_HW_FLOWCTRL_DISABLE,

        .rx_flow_ctrl_thresh =
            0U,

        .rx_buffer_size =
            s_config.uart_rx_buffer_size,

        .tx_buffer_size =
            s_config.uart_tx_buffer_size,

        .timeout_ms =
            s_config.uart_timeout_ms
    };

    err =
        hal_uart_init(
            &uart_config);

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "UART initialization failed: %s",
            esp_err_to_name(err));

        return err;
    }

    s_state =
        DRV_MODEM_STATE_INITIALIZED;

    s_initialized =
        true;

    ESP_LOGI(
        TAG,
        "Modem driver initialized");

    ESP_LOGI(
        TAG,
        "UART=%d TX=%d RX=%d baud=%lu",
        (int)s_config.uart_port,
        s_config.tx_gpio,
        s_config.rx_gpio,
        (unsigned long)s_config.baud_rate);

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Deinitialization                                                           */
/* -------------------------------------------------------------------------- */

esp_err_t drv_modem_deinit(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err =
        hal_uart_deinit(
            s_config.uart_port);

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "UART deinitialization failed: %s",
            esp_err_to_name(err));

        return err;
    }

    s_initialized =
        false;

    s_state =
        DRV_MODEM_STATE_UNINITIALIZED;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* State                                                                      */
/* -------------------------------------------------------------------------- */

drv_modem_state_t drv_modem_get_state(void)
{
    return s_state;
}


bool drv_modem_is_initialized(void)
{
    return s_initialized;
}


/* -------------------------------------------------------------------------- */
/* AT command                                                                 */
/* -------------------------------------------------------------------------- */

esp_err_t drv_modem_command(
    const char *command,
    char *response,
    size_t response_length,
    size_t *response_used,
    uint32_t timeout_ms)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (command == NULL ||
        response == NULL ||
        response_length == 0U) {

        return ESP_ERR_INVALID_ARG;
    }

    if (timeout_ms == 0U) {
        timeout_ms =
            s_config.command_timeout_ms;
    }

    if (response_used != NULL) {
        *response_used = 0U;
    }

    /*
     * Remove stale response bytes before sending
     * the next AT command.
     */
    esp_err_t err =
        hal_uart_flush_rx(
            s_config.uart_port);

    if (err != ESP_OK) {
        return err;
    }

    err =
        modem_send_command_raw(
            command);

    if (err != ESP_OK) {

        s_state =
            DRV_MODEM_STATE_ERROR;

        return err;
    }

    err =
        modem_read_response(
            response,
            response_length,
            response_used,
            timeout_ms);

    if (err != ESP_OK) {

        s_state =
            DRV_MODEM_STATE_ERROR;

        return err;
    }

    if (response_contains_error(response)) {

        return ESP_FAIL;
    }

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Basic modem control                                                        */
/* -------------------------------------------------------------------------- */

esp_err_t drv_modem_ping(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    char response[128];

    esp_err_t err =
        modem_command_expect_ok(
            "AT",
            response,
            sizeof(response),
            s_config.command_timeout_ms);

    if (err != ESP_OK) {

        s_state =
            DRV_MODEM_STATE_ERROR;

        ESP_LOGE(
            TAG,
            "Modem AT ping failed");

        return err;
    }

    s_state =
        DRV_MODEM_STATE_READY;

    ESP_LOGI(
        TAG,
        "Modem responded to AT");

    return ESP_OK;
}


esp_err_t drv_modem_power_key(
    uint32_t pulse_ms)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_config.pwrkey_gpio ==
        GPIO_NUM_NC) {

        return ESP_ERR_NOT_SUPPORTED;
    }

    if (pulse_ms == 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    /*
     * This is intentionally a generic pulse.
     *
     * The exact active level must be selected by the
     * modem-specific board/profile configuration.
     */
    gpio_set_level(
        s_config.pwrkey_gpio,
        0);

    vTaskDelay(
        pdMS_TO_TICKS(pulse_ms));

    gpio_set_level(
        s_config.pwrkey_gpio,
        1);

    return ESP_OK;
}


esp_err_t drv_modem_reset(
    uint32_t pulse_ms)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_config.reset_gpio ==
        GPIO_NUM_NC) {

        return ESP_ERR_NOT_SUPPORTED;
    }

    if (pulse_ms == 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    gpio_set_level(
        s_config.reset_gpio,
        0);

    vTaskDelay(
        pdMS_TO_TICKS(pulse_ms));

    gpio_set_level(
        s_config.reset_gpio,
        1);

    s_state =
        DRV_MODEM_STATE_INITIALIZED;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Modem information                                                          */
/* -------------------------------------------------------------------------- */

esp_err_t drv_modem_get_info(
    drv_modem_info_t *info)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (info == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *info =
        (drv_modem_info_t){0};

    char response[512];

    esp_err_t err;

    /*
     * Manufacturer.
     */
    err =
        drv_modem_command(
            "AT+CGMI",
            response,
            sizeof(response),
            NULL,
            s_config.command_timeout_ms);

    if (err != ESP_OK) {
        return err;
    }

    sscanf(
        response,
        "%*[^ \r\n]%*[ \r\n]%31[^\r\n]",
        info->manufacturer);

    /*
     * Model.
     */
    err =
        drv_modem_command(
            "AT+CGMM",
            response,
            sizeof(response),
            NULL,
            s_config.command_timeout_ms);

    if (err != ESP_OK) {
        return err;
    }

    sscanf(
        response,
        "%*[^ \r\n]%*[ \r\n]%63[^\r\n]",
        info->model);

    /*
     * Firmware revision.
     */
    err =
        drv_modem_command(
            "AT+CGMR",
            response,
            sizeof(response),
            NULL,
            s_config.command_timeout_ms);

    if (err != ESP_OK) {
        return err;
    }

    sscanf(
        response,
        "%*[^ \r\n]%*[ \r\n]%63[^\r\n]",
        info->revision);

    /*
     * IMEI.
     */
    err =
        drv_modem_command(
            "AT+CGSN",
            response,
            sizeof(response),
            NULL,
            s_config.command_timeout_ms);

    if (err != ESP_OK) {
        return err;
    }

    sscanf(
        response,
        "%31[0-9]",
        info->imei);

    /*
     * ICCID.
     *
     * This command is not guaranteed to be identical across
     * all modem families. It is therefore best-effort here.
     */
    err =
        drv_modem_command(
            "AT+CCID",
            response,
            sizeof(response),
            NULL,
            s_config.command_timeout_ms);

    if (err == ESP_OK) {

        const char *p =
            strstr(response, "CCID:");

        if (p != NULL) {

            p += 5;

            while (*p == ' ' ||
                   *p == '\t') {
                p++;
            }

            sscanf(
                p,
                "%31[0-9]",
                info->iccid);
        }
    }

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Network                                                                    */
/* -------------------------------------------------------------------------- */

esp_err_t drv_modem_get_network_status(
    drv_modem_network_status_t *status)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *status =
        (drv_modem_network_status_t){0};

    char response[512];

    /*
     * Signal quality.
     */
    esp_err_t err =
        drv_modem_command(
            "AT+CSQ",
            response,
            sizeof(response),
            NULL,
            s_config.command_timeout_ms);

    if (err != ESP_OK) {
        return err;
    }

    parse_csq(
        response,
        &status->signal_quality);

    /*
     * Circuit-switched registration.
     */
    err =
        drv_modem_command(
            "AT+CREG?",
            response,
            sizeof(response),
            NULL,
            s_config.command_timeout_ms);

    if (err != ESP_OK) {
        return err;
    }

    if (!parse_creg(
            response,
            &status->registration_status)) {

        return ESP_FAIL;
    }

    status->registered =
        (status->registration_status == 1 ||
         status->registration_status == 5);

    status->roaming =
        (status->registration_status == 5);

    /*
     * Packet-domain attachment.
     */
    err =
        drv_modem_command(
            "AT+CGATT?",
            response,
            sizeof(response),
            NULL,
            s_config.command_timeout_ms);

    if (err == ESP_OK) {

        if (strstr(
                response,
                "+CGATT: 1") != NULL) {

            status->packet_domain_attached =
                true;
        }
    }

    status->data_ready =
        status->registered &&
        status->packet_domain_attached;

    if (status->data_ready) {

        s_state =
            DRV_MODEM_STATE_DATA_READY;

    } else if (status->registered) {

        s_state =
            DRV_MODEM_STATE_REGISTERED;

    } else {

        s_state =
            DRV_MODEM_STATE_NETWORK_SEARCH;
    }

    return ESP_OK;
}


esp_err_t drv_modem_wait_for_network(
    uint32_t timeout_ms)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (timeout_ms == 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    s_state =
        DRV_MODEM_STATE_NETWORK_SEARCH;

    uint32_t elapsed_ms = 0U;

    while (elapsed_ms < timeout_ms) {

        drv_modem_network_status_t status;

        esp_err_t err =
            drv_modem_get_network_status(
                &status);

        if (err == ESP_OK &&
            status.registered) {

            ESP_LOGI(
                TAG,
                "Network registered: stat=%d RSSI=%d",
                status.registration_status,
                status.signal_quality);

            return ESP_OK;
        }

        vTaskDelay(
            pdMS_TO_TICKS(1000U));

        if (timeout_ms - elapsed_ms <= 1000U) {
            elapsed_ms = timeout_ms;
        } else {
            elapsed_ms += 1000U;
        }
    }

    s_state =
        DRV_MODEM_STATE_ERROR;

    return ESP_ERR_TIMEOUT;
}


/* -------------------------------------------------------------------------- */
/* Data connection                                                            */
/* -------------------------------------------------------------------------- */

esp_err_t drv_modem_set_apn(
    const char *apn)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (apn == NULL ||
        apn[0] == '\0') {

        return ESP_ERR_INVALID_ARG;
    }

    /*
     * Generic PDP context configuration.
     *
     * CID 1 is used for the default data context.
     *
     * The modem-specific profile may later override this
     * command for A7670C/EC200 variants.
     */
    char command[192];

    int written =
        snprintf(
            command,
            sizeof(command),
            "AT+CGDCONT=1,\"IP\",\"%s\"",
            apn);

    if (written < 0 ||
        (size_t)written >= sizeof(command)) {

        return ESP_ERR_INVALID_ARG;
    }

    char response[256];

    return modem_command_expect_ok(
        command,
        response,
        sizeof(response),
        s_config.command_timeout_ms);
}


esp_err_t drv_modem_connect_data(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    /*
     * Verify network registration first.
     */
    drv_modem_network_status_t status;

    esp_err_t err =
        drv_modem_get_network_status(
            &status);

    if (err != ESP_OK) {
        return err;
    }

    if (!status.registered) {

        return ESP_ERR_INVALID_STATE;
    }

    /*
     * Attach packet domain if necessary.
     */
    if (!status.packet_domain_attached) {

        char response[256];

        err =
            modem_command_expect_ok(
                "AT+CGATT=1",
                response,
                sizeof(response),
                10000U);

        if (err != ESP_OK) {
            return err;
        }
    }

    /*
     * Generic PDP activation.
     */
    {
        char response[256];

        err =
            modem_command_expect_ok(
                "AT+CGACT=1,1",
                response,
                sizeof(response),
                30000U);

        if (err != ESP_OK) {
            return err;
        }
    }

    s_state =
        DRV_MODEM_STATE_DATA_READY;

    ESP_LOGI(
        TAG,
        "Packet-data context activated");

    return ESP_OK;
}


esp_err_t drv_modem_disconnect_data(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    char response[256];

    esp_err_t err =
        modem_command_expect_ok(
            "AT+CGACT=0,1",
            response,
            sizeof(response),
            10000U);

    if (err != ESP_OK) {
        return err;
    }

    s_state =
        DRV_MODEM_STATE_REGISTERED;

    return ESP_OK;
}


bool drv_modem_is_data_ready(void)
{
    return s_initialized &&
           s_state ==
               DRV_MODEM_STATE_DATA_READY;
}


/* -------------------------------------------------------------------------- */
/* Raw UART                                                                   */
/* -------------------------------------------------------------------------- */

esp_err_t drv_modem_write(
    const uint8_t *data,
    size_t length,
    size_t *bytes_written)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (data == NULL ||
        length == 0U) {

        return ESP_ERR_INVALID_ARG;
    }

    return hal_uart_write(
        s_config.uart_port,
        data,
        length,
        bytes_written);
}


esp_err_t drv_modem_read(
    uint8_t *data,
    size_t length,
    size_t *bytes_read)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (data == NULL ||
        length == 0U) {

        return ESP_ERR_INVALID_ARG;
    }

    return hal_uart_read(
        s_config.uart_port,
        data,
        length,
        bytes_read);
}


esp_err_t drv_modem_flush_rx(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    return hal_uart_flush_rx(
        s_config.uart_port);
}
