#include "hal_spi.h"

#include "driver/gpio.h"


#define HAL_SPI_MAX_HOSTS    SPI_HOST_MAX


typedef struct {
    bool initialized;
    spi_host_device_t host;
} hal_spi_bus_state_t;


static hal_spi_bus_state_t s_bus_state[HAL_SPI_MAX_HOSTS] = {
    {0},
};


/* -------------------------------------------------------------------------- */
/* Validation                                                                 */
/* -------------------------------------------------------------------------- */

static bool valid_host(spi_host_device_t host)
{
    return host >= SPI1_HOST &&
           host < HAL_SPI_MAX_HOSTS;
}


static hal_spi_bus_state_t *get_bus_state(
    spi_host_device_t host)
{
    if (!valid_host(host)) {
        return NULL;
    }

    return &s_bus_state[host];
}


static bool valid_gpio(int gpio)
{
    return gpio >= 0 &&
           gpio < GPIO_NUM_MAX;
}


static esp_err_t validate_bus_config(
    const hal_spi_bus_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!valid_host(config->host)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!valid_gpio(config->sclk_gpio) ||
        !valid_gpio(config->mosi_gpio) ||
        !valid_gpio(config->miso_gpio)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (config->max_transfer_sz <= 0) {
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}


static esp_err_t validate_device_config(
    const hal_spi_device_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!valid_host(config->host)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!valid_gpio(config->cs_gpio)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (config->clock_speed_hz <= 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (config->mode > 3) {
        return ESP_ERR_INVALID_ARG;
    }

    if (config->queue_size <= 0) {
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Bus initialization                                                         */
/* -------------------------------------------------------------------------- */

esp_err_t hal_spi_bus_init(
    const hal_spi_bus_config_t *config)
{
    esp_err_t err = validate_bus_config(config);

    if (err != ESP_OK) {
        return err;
    }

    hal_spi_bus_state_t *state =
        get_bus_state(config->host);

    if (state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /*
     * One HAL owner per SPI host.
     */
    if (state->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    spi_bus_config_t bus_config = {
        .mosi_io_num = config->mosi_gpio,
        .miso_io_num = config->miso_gpio,
        .sclk_io_num = config->sclk_gpio,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = config->max_transfer_sz,
    };

    err = spi_bus_initialize(
        config->host,
        &bus_config,
        config->dma_chan);

    if (err != ESP_OK) {
        return err;
    }

    state->host = config->host;
    state->initialized = true;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Bus deinitialization                                                       */
/* -------------------------------------------------------------------------- */

esp_err_t hal_spi_bus_deinit(
    spi_host_device_t host)
{
    hal_spi_bus_state_t *state =
        get_bus_state(host);

    if (state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!state->initialized) {
        return ESP_OK;
    }

    /*
     * All SPI devices must have been removed by the caller
     * before the bus is deinitialized.
     */
    esp_err_t err = spi_bus_free(host);

    if (err != ESP_OK) {
        return err;
    }

    /*
     * Detach HAL state only after successful bus removal.
     */
    state->initialized = false;
    state->host = SPI_HOST_MAX;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Device management                                                          */
/* -------------------------------------------------------------------------- */

esp_err_t hal_spi_device_add(
    const hal_spi_device_config_t *config,
    spi_device_handle_t *handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *handle = NULL;

    esp_err_t err = validate_device_config(config);

    if (err != ESP_OK) {
        return err;
    }

    hal_spi_bus_state_t *state =
        get_bus_state(config->host);

    if (state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!state->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    spi_device_interface_config_t device_config = {
        .clock_speed_hz = config->clock_speed_hz,
        .mode = config->mode,
        .spics_io_num = config->cs_gpio,
        .queue_size = config->queue_size,
        .flags = 0,
    };

    return spi_bus_add_device(
        config->host,
        &device_config,
        handle);
}


/* -------------------------------------------------------------------------- */
/* Device removal                                                             */
/* -------------------------------------------------------------------------- */

esp_err_t hal_spi_device_remove(
    spi_device_handle_t handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return spi_bus_remove_device(handle);
}


/* -------------------------------------------------------------------------- */
/* Transaction                                                                */
/* -------------------------------------------------------------------------- */

esp_err_t hal_spi_transmit(
    spi_device_handle_t handle,
    const void *tx_data,
    void *rx_data,
    size_t length_bytes)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (length_bytes == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (tx_data == NULL &&
        rx_data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    spi_transaction_t transaction = {
        .length = length_bytes * 8,
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
    };

    return spi_device_transmit(
        handle,
        &transaction);
}


/* -------------------------------------------------------------------------- */
/* State                                                                      */
/* -------------------------------------------------------------------------- */

bool hal_spi_bus_is_initialized(
    spi_host_device_t host)
{
    hal_spi_bus_state_t *state =
        get_bus_state(host);

    if (state == NULL) {
        return false;
    }

    return state->initialized;
}