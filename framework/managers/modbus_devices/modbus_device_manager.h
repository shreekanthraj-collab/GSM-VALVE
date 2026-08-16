#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif


/* -------------------------------------------------------------------------- */
/* Configuration                                                              */
/* -------------------------------------------------------------------------- */

#define MODBUS_DEVICE_MAX_SLOTS          8U
#define MODBUS_DEVICE_MAX_REGISTERS      125U
#define MODBUS_DEVICE_MIN_SLOT           0U
#define MODBUS_DEVICE_MAX_SLAVE_ADDRESS  247U


/* -------------------------------------------------------------------------- */
/* Register type                                                              */
/* -------------------------------------------------------------------------- */

typedef enum
{
    MODBUS_DEVICE_REGISTER_HOLDING = 0,
    MODBUS_DEVICE_REGISTER_INPUT
} modbus_device_register_type_t;


/* -------------------------------------------------------------------------- */
/* Device configuration                                                       */
/* -------------------------------------------------------------------------- */

typedef struct
{
    bool enabled;

    uint8_t slave_address;

    uint16_t start_register;

    uint16_t register_count;

    modbus_device_register_type_t register_type;

} modbus_device_config_t;


/* -------------------------------------------------------------------------- */
/* Device state                                                               */
/* -------------------------------------------------------------------------- */

typedef struct
{
    bool valid;

    esp_err_t last_error;

    uint16_t register_count;

    uint16_t registers[
        MODBUS_DEVICE_MAX_REGISTERS
    ];

    uint32_t successful_polls;

    uint32_t failed_polls;

} modbus_device_state_t;


/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

/*
 * Initialize the Modbus device manager.
 *
 * The underlying Modbus master must already be initialized.
 *
 * All eight device slots are reset to disabled state.
 */
esp_err_t modbus_device_manager_init(void);


/*
 * Deinitialize the Modbus device manager.
 *
 * All slot configuration and runtime state are cleared.
 */
esp_err_t modbus_device_manager_deinit(void);


/* -------------------------------------------------------------------------- */
/* Configuration                                                              */
/* -------------------------------------------------------------------------- */

/*
 * Configure one Modbus device slot.
 *
 * The slot must be in the range 0..7.
 *
 * A new configuration invalidates the previous
 * measurement state for that slot.
 */
esp_err_t modbus_device_configure(
    uint8_t slot,
    const modbus_device_config_t *config);


/*
 * Disable one Modbus device slot.
 *
 * The existing state is retained until the slot is
 * reconfigured or the manager is deinitialized.
 */
esp_err_t modbus_device_disable(
    uint8_t slot);


/*
 * Get the current configuration of one slot.
 */
esp_err_t modbus_device_get_config(
    uint8_t slot,
    modbus_device_config_t *config);


/*
 * Check whether one slot is currently enabled.
 */
bool modbus_device_is_enabled(
    uint8_t slot);


/* -------------------------------------------------------------------------- */
/* Polling                                                                    */
/* -------------------------------------------------------------------------- */

/*
 * Perform one immediate poll of one enabled device.
 *
 * The manager selects Modbus function 0x03 or 0x04
 * according to the configured register type.
 */
esp_err_t modbus_device_poll(
    uint8_t slot);


/* -------------------------------------------------------------------------- */
/* State                                                                      */
/* -------------------------------------------------------------------------- */

/*
 * Get the current runtime state of one device slot.
 */
esp_err_t modbus_device_get_state(
    uint8_t slot,
    modbus_device_state_t *state);


/*
 * Check whether the device manager is initialized.
 */
bool modbus_device_manager_is_initialized(void);


#ifdef __cplusplus
}
#endif