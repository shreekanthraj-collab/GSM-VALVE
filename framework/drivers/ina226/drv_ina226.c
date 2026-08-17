#include "drv_ina226.h"

#include <stddef.h>

#include "hal_i2c.h"


/* -------------------------------------------------------------------------- */
/* Internal state                                                             */
/* -------------------------------------------------------------------------- */

static bool s_initialized = false;

static uint8_t s_i2c_address =
    DRV_INA226_I2C_ADDRESS_DEFAULT;

static float s_shunt_resistance_ohms = 0.0f;
static float s_current_lsb_a = 0.0f;

static uint16_t s_config_register = 0U;


/* -------------------------------------------------------------------------- */
/* Validation                                                                 */
/* -------------------------------------------------------------------------- */

static bool valid_address(
    uint8_t address)
{
    return address >= 0x08U &&
           address <= 0x77U;
}


static bool valid_config(
    const drv_ina226_config_t *config)
{
    if (config == NULL) {
        return false;
    }

    if (!valid_address(config->i2c_address)) {
        return false;
    }

    if (config->shunt_resistance_ohms <= 0.0f) {
        return false;
    }

    if (config->current_lsb_a <= 0.0f) {
        return false;
    }

    return true;
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


static esp_err_t write_register(
    uint8_t reg,
    uint16_t value)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t data[3];

    data[0] = reg;
    data[1] = (uint8_t)(value >> 8);
    data[2] = (uint8_t)(value & 0xFFU);

    return hal_i2c_write(
        s_i2c_address,
        data,
        sizeof(data));
}


/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

esp_err_t drv_ina226_init(
    const drv_ina226_config_t *config)
{
    if (s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!valid_config(config)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!hal_i2c_is_initialized()) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err =
        hal_i2c_probe(
            config->i2c_address);

    if (err != ESP_OK) {
        return err;
    }

    /*
     * INA226 calibration:
     *
     * CAL = 0.00512 / (Current_LSB * R_SHUNT)
     */
    float calibration =
        0.00512f /
        (config->current_lsb_a *
         config->shunt_resistance_ohms);

    if (calibration < 1.0f ||
        calibration > 65535.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t calibration_register =
        (uint16_t)calibration;

    uint16_t config_register =
        config->config_register;

    /*
     * Default configuration:
     *
     * Continuous shunt conversion
     * Continuous bus conversion
     * 1.1 ms conversion time
     * 16 sample averaging
     */
    if (config_register == 0U) {
        config_register = 0x4527U;
    }

    /*
     * Store configuration before register access because
     * the register helpers require the driver to be marked
     * initialized.
     */
    s_i2c_address =
        config->i2c_address;

    s_shunt_resistance_ohms =
        config->shunt_resistance_ohms;

    s_current_lsb_a =
        config->current_lsb_a;

    s_config_register =
        config_register;

    s_initialized = true;

    err =
        write_register(
            DRV_INA226_REG_CONFIG,
            config_register);

    if (err != ESP_OK) {
        s_initialized = false;
        return err;
    }

    err =
        write_register(
            DRV_INA226_REG_CALIBRATION,
            calibration_register);

    if (err != ESP_OK) {
        s_initialized = false;
        return err;
    }

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Raw shunt voltage                                                           */
/* -------------------------------------------------------------------------- */

esp_err_t drv_ina226_read_shunt_voltage_raw(
    int16_t *value)
{
    if (value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t data[2] = {0};

    esp_err_t err =
        read_register(
            DRV_INA226_REG_SHUNT_VOLTAGE,
            data,
            sizeof(data));

    if (err != ESP_OK) {
        return err;
    }

    uint16_t raw =
        ((uint16_t)data[0] << 8) |
        (uint16_t)data[1];

    *value = (int16_t)raw;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Raw bus voltage                                                             */
/* -------------------------------------------------------------------------- */

esp_err_t drv_ina226_read_bus_voltage_raw(
    uint16_t *value)
{
    if (value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t data[2] = {0};

    esp_err_t err =
        read_register(
            DRV_INA226_REG_BUS_VOLTAGE,
            data,
            sizeof(data));

    if (err != ESP_OK) {
        return err;
    }

    *value =
        ((uint16_t)data[0] << 8) |
        (uint16_t)data[1];

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Raw current                                                                 */
/* -------------------------------------------------------------------------- */

esp_err_t drv_ina226_read_current_raw(
    int16_t *value)
{
    if (value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t data[2] = {0};

    esp_err_t err =
        read_register(
            DRV_INA226_REG_CURRENT,
            data,
            sizeof(data));

    if (err != ESP_OK) {
        return err;
    }

    uint16_t raw =
        ((uint16_t)data[0] << 8) |
        (uint16_t)data[1];

    *value = (int16_t)raw;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Raw power                                                                   */
/* -------------------------------------------------------------------------- */

esp_err_t drv_ina226_read_power_raw(
    uint16_t *value)
{
    if (value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t data[2] = {0};

    esp_err_t err =
        read_register(
            DRV_INA226_REG_POWER,
            data,
            sizeof(data));

    if (err != ESP_OK) {
        return err;
    }

    *value =
        ((uint16_t)data[0] << 8) |
        (uint16_t)data[1];

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Converted shunt voltage                                                     */
/* -------------------------------------------------------------------------- */

esp_err_t drv_ina226_read_shunt_voltage(
    float *voltage_v)
{
    if (voltage_v == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int16_t raw = 0;

    esp_err_t err =
        drv_ina226_read_shunt_voltage_raw(
            &raw);

    if (err != ESP_OK) {
        return err;
    }

    *voltage_v =
        (float)raw *
        DRV_INA226_SHUNT_VOLTAGE_LSB_V;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Converted bus voltage                                                       */
/* -------------------------------------------------------------------------- */

esp_err_t drv_ina226_read_bus_voltage(
    float *voltage_v)
{
    if (voltage_v == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t raw = 0U;

    esp_err_t err =
        drv_ina226_read_bus_voltage_raw(
            &raw);

    if (err != ESP_OK) {
        return err;
    }

    *voltage_v =
        (float)raw *
        DRV_INA226_BUS_VOLTAGE_LSB_V;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Converted current                                                           */
/* -------------------------------------------------------------------------- */

esp_err_t drv_ina226_read_current(
    float *current_a)
{
    if (current_a == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int16_t raw = 0;

    esp_err_t err =
        drv_ina226_read_current_raw(
            &raw);

    if (err != ESP_OK) {
        return err;
    }

    /*
     * INA226 current register:
     *
     * Current[A] = Current_Register * Current_LSB
     */
    *current_a =
        (float)raw *
        s_current_lsb_a;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Converted power                                                             */
/* -------------------------------------------------------------------------- */

esp_err_t drv_ina226_read_power(
    float *power_w)
{
    if (power_w == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t raw = 0U;

    esp_err_t err =
        drv_ina226_read_power_raw(
            &raw);

    if (err != ESP_OK) {
        return err;
    }

    /*
     * INA226 power LSB is 25 * Current_LSB.
     */
    float power_lsb_w =
        25.0f *
        s_current_lsb_a;

    *power_w =
        (float)raw *
        power_lsb_w;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Combined measurement                                                        */
/* -------------------------------------------------------------------------- */

esp_err_t drv_ina226_read(
    drv_ina226_reading_t *reading)
{
    if (reading == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    int16_t shunt_voltage_raw = 0;
    uint16_t bus_voltage_raw = 0U;
    int16_t current_raw = 0;
    uint16_t power_raw = 0U;

    esp_err_t err;

    err =
        drv_ina226_read_shunt_voltage_raw(
            &shunt_voltage_raw);

    if (err != ESP_OK) {
        return err;
    }

    err =
        drv_ina226_read_bus_voltage_raw(
            &bus_voltage_raw);

    if (err != ESP_OK) {
        return err;
    }

    err =
        drv_ina226_read_current_raw(
            &current_raw);

    if (err != ESP_OK) {
        return err;
    }

    err =
        drv_ina226_read_power_raw(
            &power_raw);

    if (err != ESP_OK) {
        return err;
    }

    reading->shunt_voltage_raw =
        shunt_voltage_raw;

    reading->bus_voltage_raw =
        bus_voltage_raw;

    reading->current_raw =
        current_raw;

    reading->power_raw =
        power_raw;

    reading->shunt_voltage_v =
        (float)shunt_voltage_raw *
        DRV_INA226_SHUNT_VOLTAGE_LSB_V;

    reading->bus_voltage_v =
        (float)bus_voltage_raw *
        DRV_INA226_BUS_VOLTAGE_LSB_V;

    reading->current_a =
        (float)current_raw *
        s_current_lsb_a;

    reading->power_w =
        (float)power_raw *
        (25.0f * s_current_lsb_a);

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Manufacturer ID                                                             */
/* -------------------------------------------------------------------------- */

esp_err_t drv_ina226_read_manufacturer_id(
    uint16_t *manufacturer_id)
{
    if (manufacturer_id == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t data[2] = {0};

    esp_err_t err =
        read_register(
            DRV_INA226_REG_MANUFACTURER_ID,
            data,
            sizeof(data));

    if (err != ESP_OK) {
        return err;
    }

    *manufacturer_id =
        ((uint16_t)data[0] << 8) |
        (uint16_t)data[1];

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Die ID                                                                      */
/* -------------------------------------------------------------------------- */

esp_err_t drv_ina226_read_die_id(
    uint16_t *die_id)
{
    if (die_id == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t data[2] = {0};

    esp_err_t err =
        read_register(
            DRV_INA226_REG_DIE_ID,
            data,
            sizeof(data));

    if (err != ESP_OK) {
        return err;
    }

    *die_id =
        ((uint16_t)data[0] << 8) |
        (uint16_t)data[1];

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Probe                                                                       */
/* -------------------------------------------------------------------------- */

esp_err_t drv_ina226_probe(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    return hal_i2c_probe(
        s_i2c_address);
}


/* -------------------------------------------------------------------------- */
/* Status                                                                      */
/* -------------------------------------------------------------------------- */

bool drv_ina226_is_initialized(void)
{
    return s_initialized;
}