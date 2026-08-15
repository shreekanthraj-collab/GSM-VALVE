#include "hal_i2c.h"

#include "board_config.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static SemaphoreHandle_t s_i2c_mutex = NULL;
static bool s_initialized = false;

static uint32_t s_timeout_ms = 100;
static uint32_t s_frequency_hz = 100000;


/* -------------------------------------------------------------------------- */
/* Board mapping                                                              */
/* -------------------------------------------------------------------------- */

static esp_err_t resolve_pins(gpio_num_t *sda, gpio_num_t *scl)
{
    if (sda == NULL || scl == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (BOARD_I2C_SDA_GPIO < 0 ||
        BOARD_I2C_SCL_GPIO < 0 ||
        BOARD_I2C_SDA_GPIO >= GPIO_NUM_MAX ||
        BOARD_I2C_SCL_GPIO >= GPIO_NUM_MAX) {
        return ESP_ERR_NOT_FOUND;
    }

    *sda = (gpio_num_t)BOARD_I2C_SDA_GPIO;
    *scl = (gpio_num_t)BOARD_I2C_SCL_GPIO;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Bus locking                                                                */
/* -------------------------------------------------------------------------- */

static esp_err_t lock_bus(void)
{
    if (s_i2c_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    TickType_t ticks = pdMS_TO_TICKS(s_timeout_ms);

    if (ticks == 0) {
        ticks = 1;
    }

    if (xSemaphoreTake(s_i2c_mutex, ticks) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}


static void unlock_bus(void)
{
    if (s_i2c_mutex != NULL) {
        xSemaphoreGive(s_i2c_mutex);
    }
}


/* -------------------------------------------------------------------------- */
/* I2C driver installation                                                     */
/* -------------------------------------------------------------------------- */

static esp_err_t install_i2c_driver(
    gpio_num_t sda,
    gpio_num_t scl,
    uint32_t frequency_hz)
{
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = frequency_hz,
        .clk_flags = 0,
    };

    esp_err_t err = i2c_param_config(
        I2C_NUM_0,
        &i2c_config);

    if (err != ESP_OK) {
        return err;
    }

    return i2c_driver_install(
        I2C_NUM_0,
        I2C_MODE_MASTER,
        0,
        0,
        0);
}


/* -------------------------------------------------------------------------- */
/* Initialization                                                              */
/* -------------------------------------------------------------------------- */

esp_err_t hal_i2c_init(const hal_i2c_config_t *config)
{
    if (config == NULL || config->frequency_hz == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_initialized) {
        return ESP_OK;
    }

    gpio_num_t sda;
    gpio_num_t scl;

    esp_err_t err = resolve_pins(&sda, &scl);

    if (err != ESP_OK) {
        return err;
    }

    s_timeout_ms = config->timeout_ms;

    if (s_timeout_ms == 0) {
        s_timeout_ms = 100;
    }

    s_frequency_hz = config->frequency_hz;

    s_i2c_mutex = xSemaphoreCreateMutex();

    if (s_i2c_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }

    err = install_i2c_driver(
        sda,
        scl,
        s_frequency_hz);

    if (err != ESP_OK) {
        vSemaphoreDelete(s_i2c_mutex);
        s_i2c_mutex = NULL;
        return err;
    }

    s_initialized = true;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Deinitialization                                                            */
/* -------------------------------------------------------------------------- */

esp_err_t hal_i2c_deinit(void)
{
    if (!s_initialized) {
        return ESP_OK;
    }

    esp_err_t err = i2c_driver_delete(I2C_NUM_0);

    if (err != ESP_OK) {
        return err;
    }

    s_initialized = false;

    if (s_i2c_mutex != NULL) {
        vSemaphoreDelete(s_i2c_mutex);
        s_i2c_mutex = NULL;
    }

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Write                                                                       */
/* -------------------------------------------------------------------------- */

esp_err_t hal_i2c_write(
    uint8_t address,
    const uint8_t *data,
    size_t length)
{
    if (data == NULL ||
        length == 0 ||
        address > 0x7F) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = lock_bus();

    if (err != ESP_OK) {
        return err;
    }

    err = i2c_master_write_to_device(
        I2C_NUM_0,
        address,
        data,
        length,
        pdMS_TO_TICKS(s_timeout_ms));

    unlock_bus();

    return err;
}


/* -------------------------------------------------------------------------- */
/* Read                                                                        */
/* -------------------------------------------------------------------------- */

esp_err_t hal_i2c_read(
    uint8_t address,
    uint8_t *data,
    size_t length)
{
    if (data == NULL ||
        length == 0 ||
        address > 0x7F) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = lock_bus();

    if (err != ESP_OK) {
        return err;
    }

    err = i2c_master_read_from_device(
        I2C_NUM_0,
        address,
        data,
        length,
        pdMS_TO_TICKS(s_timeout_ms));

    unlock_bus();

    return err;
}


/* -------------------------------------------------------------------------- */
/* Write + Read                                                                */
/* -------------------------------------------------------------------------- */

esp_err_t hal_i2c_write_read(
    uint8_t address,
    const uint8_t *write_data,
    size_t write_length,
    uint8_t *read_data,
    size_t read_length)
{
    if (write_data == NULL ||
        read_data == NULL ||
        write_length == 0 ||
        read_length == 0 ||
        address > 0x7F) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = lock_bus();

    if (err != ESP_OK) {
        return err;
    }

    err = i2c_master_write_read_device(
        I2C_NUM_0,
        address,
        write_data,
        write_length,
        read_data,
        read_length,
        pdMS_TO_TICKS(s_timeout_ms));

    unlock_bus();

    return err;
}


/* -------------------------------------------------------------------------- */
/* Probe                                                                       */
/* -------------------------------------------------------------------------- */

esp_err_t hal_i2c_probe(uint8_t address)
{
    if (address > 0x7F) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = lock_bus();

    if (err != ESP_OK) {
        return err;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    if (cmd == NULL) {
        unlock_bus();
        return ESP_ERR_NO_MEM;
    }

    err = i2c_master_start(cmd);

    if (err == ESP_OK) {
        err = i2c_master_write_byte(
            cmd,
            (address << 1) | I2C_MASTER_WRITE,
            true);
    }

    if (err == ESP_OK) {
        err = i2c_master_stop(cmd);
    }

    if (err == ESP_OK) {
        err = i2c_master_cmd_begin(
            I2C_NUM_0,
            cmd,
            pdMS_TO_TICKS(s_timeout_ms));
    }

    i2c_cmd_link_delete(cmd);

    unlock_bus();

    return err;
}


/* -------------------------------------------------------------------------- */
/* Bus recovery                                                                */
/* -------------------------------------------------------------------------- */

esp_err_t hal_i2c_recover(void)
{
    gpio_num_t sda;
    gpio_num_t scl;

    esp_err_t err = resolve_pins(&sda, &scl);

    if (err != ESP_OK) {
        return err;
    }

    if (s_i2c_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    err = lock_bus();

    if (err != ESP_OK) {
        return err;
    }

    /*
     * Stop the I2C peripheral before manipulating SDA/SCL directly.
     */
    if (s_initialized) {
        err = i2c_driver_delete(I2C_NUM_0);

        if (err != ESP_OK) {
            unlock_bus();
            return err;
        }

        s_initialized = false;
    }

    /*
     * Configure SCL as open-drain output and SDA as input.
     */
    gpio_config_t scl_cfg = {
        .pin_bit_mask = 1ULL << scl,
        .mode = GPIO_MODE_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    gpio_config_t sda_cfg = {
        .pin_bit_mask = 1ULL << sda,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    err = gpio_config(&scl_cfg);

    if (err == ESP_OK) {
        err = gpio_config(&sda_cfg);
    }

    /*
     * Clock SCL up to nine times while SDA remains low.
     */
    if (err == ESP_OK) {
        gpio_set_level(scl, 1);

        for (int i = 0;
             i < 9 && gpio_get_level(sda) == 0;
             ++i) {

            gpio_set_level(scl, 0);
            vTaskDelay(pdMS_TO_TICKS(1));

            gpio_set_level(scl, 1);
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    /*
     * Generate a STOP condition:
     * SDA low -> SCL high -> SDA high.
     */
    if (err == ESP_OK) {
        gpio_config_t sda_output = {
            .pin_bit_mask = 1ULL << sda,
            .mode = GPIO_MODE_OUTPUT_OD,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };

        err = gpio_config(&sda_output);

        if (err == ESP_OK) {
            gpio_set_level(sda, 0);
            vTaskDelay(pdMS_TO_TICKS(1));

            gpio_set_level(scl, 1);
            vTaskDelay(pdMS_TO_TICKS(1));

            gpio_set_level(sda, 1);
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    /*
     * Restore the pins to a neutral state before reinstalling
     * the I2C peripheral.
     */
    gpio_reset_pin(sda);
    gpio_reset_pin(scl);

    /*
     * Reinstall the I2C peripheral if recovery itself succeeded.
     */
    if (err == ESP_OK) {
        err = install_i2c_driver(
            sda,
            scl,
            s_frequency_hz);

        if (err == ESP_OK) {
            s_initialized = true;
        }
    }

    unlock_bus();

    return err;
}


/* -------------------------------------------------------------------------- */
/* State                                                                        */
/* -------------------------------------------------------------------------- */

bool hal_i2c_is_initialized(void)
{
    return s_initialized;
}