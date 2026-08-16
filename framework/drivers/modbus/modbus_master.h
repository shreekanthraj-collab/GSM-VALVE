#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MODBUS_MAX_BYTE_COUNT    250U
#define MODBUS_MAX_FRAME         256U

#define MODBUS_DEFAULT_SLAVE     1U

typedef struct
{
    uint8_t slave_address;

    uint32_t timeout_ms;

    uint16_t max_registers;
} modbus_master_config_t;


/*
 * Initialize the Modbus RTU master.
 *
 * The underlying RS485 driver must already be initialized.
 */
esp_err_t modbus_master_init(
    const modbus_master_config_t *config);


/*
 * Deinitialize the Modbus RTU master.
 */
esp_err_t modbus_master_deinit(void);


/*
 * Read holding registers.
 *
 * Function: 0x03
 */
esp_err_t modbus_read_holding_registers(
    uint8_t slave_address,
    uint16_t start_register,
    uint16_t register_count,
    uint16_t *registers);


/*
 * Read input registers.
 *
 * Function: 0x04
 */
esp_err_t modbus_read_input_registers(
    uint8_t slave_address,
    uint16_t start_register,
    uint16_t register_count,
    uint16_t *registers);


/*
 * Write a single holding register.
 *
 * Function: 0x06
 */
esp_err_t modbus_write_single_register(
    uint8_t slave_address,
    uint16_t register_address,
    uint16_t value);


/*
 * Write multiple holding registers.
 *
 * Function: 0x10
 */
esp_err_t modbus_write_multiple_registers(
    uint8_t slave_address,
    uint16_t start_register,
    uint16_t register_count,
    const uint16_t *registers);


/*
 * Check whether the Modbus master is initialized.
 */
bool modbus_master_is_initialized(void);


#ifdef __cplusplus
}
#endif