#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DRV_AS5600_I2C_ADDRESS   0x36U

#define DRV_AS5600_REG_ZMCO      0x00U
#define DRV_AS5600_REG_ZPOS      0x01U
#define DRV_AS5600_REG_MPOS      0x03U
#define DRV_AS5600_REG_MANG      0x05U
#define DRV_AS5600_REG_CONF      0x07U
#define DRV_AS5600_REG_STATUS    0x0BU
#define DRV_AS5600_REG_RAW_ANGLE 0x0CU
#define DRV_AS5600_REG_ANGLE     0x0EU
#define DRV_AS5600_REG_AGC       0x1AU
#define DRV_AS5600_REG_MAGNITUDE 0x1BU

#define DRV_AS5600_RAW_ANGLE_MAX 4095U
#define DRV_AS5600_ANGLE_MAX     4095U

typedef struct
{
    uint8_t i2c_address;
} drv_as5600_config_t;

typedef struct
{
    uint16_t raw_angle;
    uint16_t angle;
} drv_as5600_reading_t;

/*
 * Initialize the AS5600 driver.
 *
 * The underlying HAL I2C bus must already be initialized.
 */
esp_err_t drv_as5600_init(
    const drv_as5600_config_t *config);

/*
 * Read the raw 12-bit AS5600 angle.
 *
 * Returns the unscaled 0..4095 sensor value.
 */
esp_err_t drv_as5600_read_angle(
    uint16_t *angle);
/*
 * Read the processed 12-bit AS5600 angle register.
 *
 * Returns the 0..4095 sensor value.
 */
esp_err_t drv_as5600_read_angle(
    uint16_t *angle);

/*
 * Read both raw and processed angle values.
 */
esp_err_t drv_as5600_read(
    drv_as5600_reading_t *reading);

/*
 * Probe the configured AS5600 device.
 */
esp_err_t drv_as5600_probe(void);

/*
 * Check whether the driver has been initialized.
 */
bool drv_as5600_is_initialized(void);

#ifdef __cplusplus
}
#endif
