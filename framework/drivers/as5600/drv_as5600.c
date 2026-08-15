#include "drv_as5600.h"

#include <stddef.h>

#include "hal_i2c.h"


static bool s_initialized = false;
static uint8_t s_i2c_address = DRV_AS5600_I2C_ADDRESS;


/* -------------------------------------------------------------------------- */
/* Validation                                                                 */
/* -------------------------------------------------------------------------- */

static bool valid_address(uint8_t address)
{
    return address >= 0x08U &&
           address <= 0x77U;
}


static bool valid_raw_angle(uint16_t value)
{
    return value <= DRV_AS5600_RAW_ANGLE_MAX;
}


static bool valid_angle(uint16_t value)
{
    return value <= DRV_AS5600_ANGLE_MAX;
}


/* -------------------------------------------------------------------------- */
/* Register access                                                             */
/* -------------------------------------------------------------------------- */

static esp_err_t read_register(
    uint8_t reg,
    uint8_t *data,
    size_t length)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (data == NULL || length == 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    return hal_i2c_write_read(
        s_i2c_address,
        &reg,
        1U,
        data,
        length);
}


/* -------------------------------------------------------------------------- */
/* Initialization                                                              */
/* -------------------------------------------------------------------------- */

esp_err_t drv_as5600_init(
    const drv_as5600_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!valid_address(config->i2c_address)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!hal_i2c_is_initialized()) {
        return ESP_ERR_INVALID_STATE;
    }

    /*
     * Do not claim initialization until the device has
     * successfully responded on the I2C bus.
     */
    esp_err_t err = hal_i2c_probe(config->i2c_address);

    if (err != ESP_OK) {
        return err;
    }

    s_i2c_address = config->i2c_address;
    s_initialized = true;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Raw angle                                                                   */
/* -------------------------------------------------------------------------- */

esp_err_t drv_as5600_read_raw_angle(
    uint16_t *raw_angle)
{
    if (raw_angle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t data[2] = {0};

    esp_err_t err = read_register(
        DRV_AS5600_REG_RAW_ANGLE,
        data,
        sizeof(data));

    if (err != ESP_OK) {
        return err;
    }

    uint16_t value =
        ((uint16_t)data[0] << 8) |
        (uint16_t)data[1];

    value &= 0x0FFFU;

    if (!valid_raw_angle(value)) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    *raw_angle = value;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Angle                                                                       */
/* -------------------------------------------------------------------------- */

esp_err_t drv_as5600_read_angle(
    uint16_t *angle)
{
    if (angle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t data[2] = {0};

    esp_err_t err = read_register(
        DRV_AS5600_REG_ANGLE,
        data,
        sizeof(data));

    if (err != ESP_OK) {
        return err;
    }

    uint16_t value =
        ((uint16_t)data[0] << 8) |
        (uint16_t)data[1];

    value &= 0x0FFFU;

    if (!valid_angle(value)) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    *angle = value;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Combined reading                                                            */
/* -------------------------------------------------------------------------- */

esp_err_t drv_as5600_read(
    drv_as5600_reading_t *reading)
{
    if (reading == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t raw_data[2] = {0};
    uint8_t angle_data[2] = {0};

    esp_err_t err = read_register(
        DRV_AS5600_REG_RAW_ANGLE,
        raw_data,
        sizeof(raw_data));

    if (err != ESP_OK) {
        return err;
    }

    err = read_register(
        DRV_AS5600_REG_ANGLE,
        angle_data,
        sizeof(angle_data));

    if (err != ESP_OK) {
        return err;
    }

    uint16_t raw =
        (((uint16_t)raw_data[0] << 8) |
         (uint16_t)raw_data[1]) & 0x0FFFU;

    uint16_t angle =
        (((uint16_t)angle_data[0] << 8) |
         (uint16_t)angle_data[1]) & 0x0FFFU;

    if (!valid_raw_angle(raw) ||
        !valid_angle(angle)) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    reading->raw_angle = raw;
    reading->angle = angle;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Probe                                                                       */
/* -------------------------------------------------------------------------- */

esp_err_t drv_as5600_probe(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    return hal_i2c_probe(s_i2c_address);
}


/* -------------------------------------------------------------------------- */
/* State                                                                       */
/* -------------------------------------------------------------------------- */

bool drv_as5600_is_initialized(void)
{
    return s_initialized;
}
