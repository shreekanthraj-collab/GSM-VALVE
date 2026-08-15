#include "hal_uart.h"

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define HAL_UART_MAX_PORTS    UART_NUM_MAX

typedef struct {
    bool initialized;

    uart_port_t port;

    SemaphoreHandle_t tx_mutex;
    SemaphoreHandle_t rx_mutex;

    uint32_t timeout_ms;
} hal_uart_state_t;

static hal_uart_state_t s_uart_state[HAL_UART_MAX_PORTS] = {
    {0},
};


/* -------------------------------------------------------------------------- */
/* Port validation                                                            */
/* -------------------------------------------------------------------------- */

static bool valid_port(uart_port_t port)
{
    return port >= UART_NUM_0 &&
           port < HAL_UART_MAX_PORTS;
}

static hal_uart_state_t *get_state(uart_port_t port)
{
    if (!valid_port(port)) {
        return NULL;
    }

    return &s_uart_state[port];
}


/* -------------------------------------------------------------------------- */
/* Configuration validation                                                   */
/* -------------------------------------------------------------------------- */

static esp_err_t validate_data_bits(
    uart_word_length_t data_bits)
{
    switch (data_bits) {
        case UART_DATA_5_BITS:
        case UART_DATA_6_BITS:
        case UART_DATA_7_BITS:
        case UART_DATA_8_BITS:
            return ESP_OK;

        default:
            return ESP_ERR_INVALID_ARG;
    }
}

static esp_err_t validate_flow_control(
    uart_hw_flowcontrol_t flow_ctrl,
    uint8_t rx_flow_ctrl_thresh)
{
    switch (flow_ctrl) {
        case UART_HW_FLOWCTRL_DISABLE:
            return ESP_OK;

        case UART_HW_FLOWCTRL_RTS:
        case UART_HW_FLOWCTRL_CTS:
        case UART_HW_FLOWCTRL_CTS_RTS:
            if (rx_flow_ctrl_thresh == 0) {
                return ESP_ERR_INVALID_ARG;
            }

            return ESP_OK;

        default:
            return ESP_ERR_INVALID_ARG;
    }
}

static esp_err_t validate_config(
    const hal_uart_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!valid_port(config->port)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (config->tx_gpio < 0 ||
        config->tx_gpio >= GPIO_NUM_MAX ||
        config->rx_gpio < 0 ||
        config->rx_gpio >= GPIO_NUM_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    if (config->baud_rate == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = validate_data_bits(config->data_bits);

    if (err != ESP_OK) {
        return err;
    }

    err = validate_flow_control(
        config->flow_ctrl,
        config->rx_flow_ctrl_thresh);

    if (err != ESP_OK) {
        return err;
    }

    if (config->rx_buffer_size == 0 ||
        config->tx_buffer_size == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Mutex helper                                                               */
/* -------------------------------------------------------------------------- */

static esp_err_t lock_mutex(
    SemaphoreHandle_t mutex,
    uint32_t timeout_ms)
{
    if (mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    TickType_t ticks = pdMS_TO_TICKS(timeout_ms);

    if (ticks == 0) {
        ticks = 1;
    }

    if (xSemaphoreTake(mutex, ticks) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

esp_err_t hal_uart_init(
    const hal_uart_config_t *config)
{
    esp_err_t err = validate_config(config);

    if (err != ESP_OK) {
        return err;
    }

    hal_uart_state_t *state = get_state(config->port);

    if (state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /*
     * One HAL owner per UART port.
     */
    if (state->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    uart_config_t uart_config = {
        .baud_rate = config->baud_rate,
        .data_bits = config->data_bits,
        .parity = config->parity,
        .stop_bits = config->stop_bits,
        .flow_ctrl = config->flow_ctrl,
        .rx_flow_ctrl_thresh = config->rx_flow_ctrl_thresh,
        .source_clk = UART_SCLK_DEFAULT,
    };

    err = uart_param_config(
        config->port,
        &uart_config);

    if (err != ESP_OK) {
        return err;
    }

    err = uart_set_pin(
        config->port,
        config->tx_gpio,
        config->rx_gpio,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE);

    if (err != ESP_OK) {
        return err;
    }

    err = uart_driver_install(
        config->port,
        config->rx_buffer_size,
        config->tx_buffer_size,
        0,
        NULL,
        0);

    if (err != ESP_OK) {
        return err;
    }

    /*
     * Create TX mutex.
     */
    state->tx_mutex = xSemaphoreCreateMutex();

    if (state->tx_mutex == NULL) {
        (void)uart_driver_delete(config->port);
        return ESP_ERR_NO_MEM;
    }

    /*
     * Create RX mutex.
     */
    state->rx_mutex = xSemaphoreCreateMutex();

    if (state->rx_mutex == NULL) {
        vSemaphoreDelete(state->tx_mutex);
        state->tx_mutex = NULL;

        (void)uart_driver_delete(config->port);

        return ESP_ERR_NO_MEM;
    }

    state->port = config->port;

    state->timeout_ms = config->timeout_ms;

    if (state->timeout_ms == 0) {
        state->timeout_ms = 100;
    }

    state->initialized = true;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Deinitialization                                                           */
/* -------------------------------------------------------------------------- */

esp_err_t hal_uart_deinit(
    uart_port_t port)
{
    hal_uart_state_t *state = get_state(port);

    if (state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!state->initialized) {
        return ESP_OK;
    }

    /*
     * IMPORTANT:
     *
     * HAL state is detached only after successful
     * uart_driver_delete().
     */
    esp_err_t err = uart_driver_delete(port);

    if (err != ESP_OK) {
        return err;
    }

    state->initialized = false;

    if (state->tx_mutex != NULL) {
        vSemaphoreDelete(state->tx_mutex);
        state->tx_mutex = NULL;
    }

    if (state->rx_mutex != NULL) {
        vSemaphoreDelete(state->rx_mutex);
        state->rx_mutex = NULL;
    }

    state->timeout_ms = 0;
    state->port = UART_NUM_MAX;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Write                                                                      */
/* -------------------------------------------------------------------------- */

esp_err_t hal_uart_write(
    uart_port_t port,
    const uint8_t *data,
    size_t length,
    size_t *bytes_written)
{
    if (data == NULL || length == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (bytes_written != NULL) {
        *bytes_written = 0;
    }

    hal_uart_state_t *state = get_state(port);

    if (state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!state->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = lock_mutex(
        state->tx_mutex,
        state->timeout_ms);

    if (err != ESP_OK) {
        return err;
    }

    int written = uart_write_bytes(
        port,
        data,
        length);

    xSemaphoreGive(state->tx_mutex);

    if (written < 0) {
        return ESP_FAIL;
    }

    if (bytes_written != NULL) {
        *bytes_written = (size_t)written;
    }

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Read                                                                       */
/* -------------------------------------------------------------------------- */

esp_err_t hal_uart_read(
    uart_port_t port,
    uint8_t *data,
    size_t length,
    size_t *bytes_read)
{
    if (data == NULL || length == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (bytes_read != NULL) {
        *bytes_read = 0;
    }

    hal_uart_state_t *state = get_state(port);

    if (state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!state->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = lock_mutex(
        state->rx_mutex,
        state->timeout_ms);

    if (err != ESP_OK) {
        return err;
    }

    int received = uart_read_bytes(
        port,
        data,
        length,
        pdMS_TO_TICKS(state->timeout_ms));

    xSemaphoreGive(state->rx_mutex);

    if (received < 0) {
        return ESP_FAIL;
    }

    if (bytes_read != NULL) {
        *bytes_read = (size_t)received;
    }

    /*
     * Zero bytes means the receive timeout expired.
     * This is not treated as a driver failure.
     */
    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Flush RX                                                                   */
/* -------------------------------------------------------------------------- */

esp_err_t hal_uart_flush_rx(
    uart_port_t port)
{
    hal_uart_state_t *state = get_state(port);

    if (state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!state->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = lock_mutex(
        state->rx_mutex,
        state->timeout_ms);

    if (err != ESP_OK) {
        return err;
    }

    err = uart_flush_input(port);

    xSemaphoreGive(state->rx_mutex);

    return err;
}


/* -------------------------------------------------------------------------- */
/* Wait for TX completion                                                     */
/* -------------------------------------------------------------------------- */

esp_err_t hal_uart_wait_tx_done(
    uart_port_t port,
    uint32_t timeout_ms)
{
    hal_uart_state_t *state = get_state(port);

    if (state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!state->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = lock_mutex(
        state->tx_mutex,
        state->timeout_ms);

    if (err != ESP_OK) {
        return err;
    }

    TickType_t ticks = pdMS_TO_TICKS(timeout_ms);

    if (ticks == 0) {
        ticks = 1;
    }

    err = uart_wait_tx_done(
        port,
        ticks);

    xSemaphoreGive(state->tx_mutex);

    return err;
}


/* -------------------------------------------------------------------------- */
/* State                                                                      */
/* -------------------------------------------------------------------------- */

bool hal_uart_is_initialized(
    uart_port_t port)
{
    hal_uart_state_t *state = get_state(port);

    if (state == NULL) {
        return false;
    }

    return state->initialized;
}