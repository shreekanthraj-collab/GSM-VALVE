#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif


/* -------------------------------------------------------------------------- */
/* Device                                                                      */
/* -------------------------------------------------------------------------- */

#define DRV_INA226_I2C_ADDRESS_DEFAULT  0x40U


/* -------------------------------------------------------------------------- */
/* Register map                                                                */
/* -------------------------------------------------------------------------- */

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


/* -------------------------------------------------------------------------- */
/* Device identification                                                       */
/* -------------------------------------------------------------------------- */

#define DRV_INA226_MANUFACTURER_ID      0x5449U


/* -------------------------------------------------------------------------- */
/* Conversion constants                                                        */
/* -------------------------------------------------------------------------- */

#define DRV_INA226_BUS_VOLTAGE_LSB_V    0.00125f
#define DRV_INA226_SHUNT_VOLTAGE_LSB_V  0.0000025f


/* -------------------------------------------------------------------------- */
/* Configuration                                                              */
/* -------------------------------------------------------------------------- */

typedef struct
{
    uint8_t i2c_address;

    /*
     * Physical shunt resistance used by the INA226
     * calibration calculation.
     */
    float shunt_resistance_ohms;

    /*
     * Current represented by one LSB of the INA226
     * current register.
     */
    float current_lsb_a;

    /*
     * INA226 configuration register.
     *
     * Set to 0 to use the driver's default continuous
     * shunt + bus conversion configuration.
     */
    uint16_t config_register;

} drv_ina226_config_t;


/* -------------------------------------------------------------------------- */
/* Complete measurement                                                       */
/* -------------------------------------------------------------------------- */

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


/* -------------------------------------------------------------------------- */
/* Initialization                                                              */
/* -------------------------------------------------------------------------- */

/*
 * Initialize the INA226 driver.
 *
 * The HAL I2C bus must already be initialized.
 *
 * The driver:
 *   - validates the supplied configuration
 *   - probes the configured I2C address
 *   - calculates the INA226 calibration register
 *   - programs the configuration register
 *   - programs the calibration register
 */
esp_err_t drv_ina226_init(
    const drv_ina226_config_t *config);


/* -------------------------------------------------------------------------- */
/* Raw measurements                                                            */
/* -------------------------------------------------------------------------- */

/*
 * Read raw shunt voltage register.
 */
esp_err_t drv_ina226_read_shunt_voltage_raw(
    int16_t *value);


/*
 * Read raw bus voltage register.
 */
esp_err_t drv_ina226_read_bus_voltage_raw(
    uint16_t *value);


/*
 * Read raw current register.
 */
esp_err_t drv_ina226_read_current_raw(
    int16_t *value);


/*
 * Read raw power register.
 */
esp_err_t drv_ina226_read_power_raw(
    uint16_t *value);


/* -------------------------------------------------------------------------- */
/* Converted measurements                                                       */
/* -------------------------------------------------------------------------- */

/*
 * Read shunt voltage in volts.
 */
esp_err_t drv_ina226_read_shunt_voltage(
    float *voltage_v);


/*
 * Read bus voltage in volts.
 *
 * This is the INA226 monitored load/battery-side bus
 * voltage used by higher layers for voltage monitoring.
 */
esp_err_t drv_ina226_read_bus_voltage(
    float *voltage_v);


/*
 * Read current in amperes.
 *
 * The conversion uses the configured current_lsb_a.
 */
esp_err_t drv_ina226_read_current(
    float *current_a);


/*
 * Read power in watts.
 *
 * The conversion uses the INA226 power register and
 * the configured current_lsb_a.
 */
esp_err_t drv_ina226_read_power(
    float *power_w);


/* -------------------------------------------------------------------------- */
/* Combined measurement                                                        */
/* -------------------------------------------------------------------------- */

/*
 * Read all primary INA226 measurements.
 *
 * Returns:
 *   - raw shunt voltage
 *   - raw bus voltage
 *   - raw current
 *   - raw power
 *   - converted shunt voltage
 *   - converted bus voltage
 *   - converted current
 *   - converted power
 */
esp_err_t drv_ina226_read(
    drv_ina226_reading_t *reading);


/* -------------------------------------------------------------------------- */
/* Device identification                                                       */
/* -------------------------------------------------------------------------- */

/*
 * Read INA226 manufacturer ID.
 */
esp_err_t drv_ina226_read_manufacturer_id(
    uint16_t *manufacturer_id);


/*
 * Read INA226 die ID.
 */
esp_err_t drv_ina226_read_die_id(
    uint16_t *die_id);


/*
 * Probe the configured INA226.
 *
 * This verifies that the configured I2C device responds.
 */
esp_err_t drv_ina226_probe(void);


/* -------------------------------------------------------------------------- */
/* Status                                                                      */
/* -------------------------------------------------------------------------- */

/*
 * Check whether the INA226 driver is initialized.
 */
bool drv_ina226_is_initialized(void);


#ifdef __cplusplus
}
#endif