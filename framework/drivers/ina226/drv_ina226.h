#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DRV_INA226_I2C_ADDRESS_DEFAULT  0x40U

#define DRV_INA226_REG_CONFIG           0x00U
#define DRV_INA226_REG_SHUNT_VOLTAGE    0x01U
#define DRV_INA226_REG_BUS_VOLTAGE      0x02U
#define DRV_INA226_REG_POWER            0x03U
#define DRV_INA226_REG_CURRENT          0x04U
#define DRV_INA226_REG_CALIBRATION      0x05U
#define DRV_INA226_REG_MASK_ENABLE      0x06U
#define DRV_INA226_REG_ALERT_LIMIT      0x07U
#define DRV_INA226_REG_MANUFACTURER_ID  0xFEU
#define DRV_INA226_REG_DIE_ID           0xFFU

#define DRV_INA226_MANUFACTURER_ID      0x5449U

#define DRV_INA226_BUS_VOLTAGE_LSB_V    0.00125f
#define DRV_INA226_SHUNT_VOLTAGE_LSB_V  0.0000025f

typedef struct
{
    uint8_t i2c_address;
    float shunt_resistance_ohms;
    float current_lsb_a;
    uint16_t config_register;
} drv_ina226_config_t;

typedef struct
{
    int16_t shunt_voltage_raw;
    uint16_t bus_voltage_raw;
    int16_t current_raw;
    uint16_t power_raw;

    float shunt_voltage_v;
    float bus_voltage_v;
    float current_a;
    float power_w;
} drv_ina226_reading_t;

/*
 * Initialize the INA226 driver.
 *
 * The HAL I2C bus must already be initialized.
 */
esp_err_t drv_ina226_init(
    const drv_ina226_config_t *config);

/*
 * Read raw shunt voltage.
 */
esp_err_t drv_ina226_read_shunt_voltage_raw(
    int16_t *value);

/*
 * Read raw bus voltage.
 */
esp_err_t drv_ina226_read_bus_voltage_raw(
    uint16_t *value);

/*
 * Read raw current.
 */
esp_err_t drv_ina226_read_current_raw(
    int16_t *value);

/*
 * Read raw power.
 */
esp_err_t drv_ina226_read_power_raw(
    uint16_t *value);

/*
 * Read shunt voltage in volts.
 */
esp_err_t drv_ina226_read_shunt_voltage(
    float *voltage_v);

/*
 * Read bus voltage in volts.
 */
esp_err_t drv_ina226_read_bus_voltage(
    float *voltage_v);

/*
 * Read current in amperes.
 */
esp_err_t drv_ina226_read_current(
    float *current_a);

/*
 * Read power in watts.
 */
esp_err_t drv_ina226_read_power(
    float *power_w);

/*
 * Read all primary measurements.
 */
esp_err_t drv_ina226_read(
    drv_ina226_reading_t *reading);

/*
 * Read manufacturer ID.
 */
esp_err_t drv_ina226_read_manufacturer_id(
    uint16_t *manufacturer_id);

/*
 * Read die ID.
 */
esp_err_t drv_ina226_read_die_id(
    uint16_t *die_id);

/*
 * Probe the configured INA226.
 */
esp_err_t drv_ina226_probe(void);

/*
 * Check whether the driver is initialized.
 */
bool drv_ina226_is_initialized(void);

#ifdef __cplusplus
}
#endif