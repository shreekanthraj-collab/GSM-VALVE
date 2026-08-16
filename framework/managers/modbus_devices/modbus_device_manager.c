#include "modbus_device_manager.h"

#include <string.h>

#include "modbus_master.h"


/* -------------------------------------------------------------------------- */
/* Constants                                                                  */
/* -------------------------------------------------------------------------- */

#define MODBUS_DEVICE_MIN_SLOT             0U
#define MODBUS_DEVICE_MAX_SLAVE_ADDRESS    247U


/* -------------------------------------------------------------------------- */
/* Internal state                                                             */
/* -------------------------------------------------------------------------- */

typedef struct
{
    modbus_device_config_t config;
    modbus_device_state_t state;
} modbus_device_slot_t;

static bool s_initialized = false;

static modbus_device_slot_t s_slots[
    MODBUS_DEVICE_MAX_SLOTS
];


/* -------------------------------------------------------------------------- */
/* Validation                                                                 */
/* -------------------------------------------------------------------------- */

static bool valid_slot(
    uint8_t slot)
{
    return slot < MODBUS_DEVICE_MAX_SLOTS;
}


static bool valid_slave_address(
    uint8_t slave_address)
{
    return slave_address >= 1U &&
           slave_address <= MODBUS_DEVICE_MAX_SLAVE_ADDRESS;
}


static bool valid_register_type(
    modbus_device_register_type_t register_type)
{
    return register_type == MODBUS_DEVICE_REGISTER_HOLDING ||
           register_type == MODBUS_DEVICE_REGISTER_INPUT;
}


static bool valid_config(
    const modbus_device_config_t *config)
{
    if (config == NULL) {
        return false;
    }

    if (!valid_slave_address(config->slave_address)) {
        return false;
    }

    if (config->register_count == 0U ||
        config->register_count > MODBUS_DEVICE_MAX_REGISTERS) {
        return false;
    }

    /*
     * Modbus register addresses are 16-bit.
     *
     * Prevent the configured register block from extending
     * beyond the 0..65535 register address space.
     */
    if ((uint32_t)config->start_register +
        (uint32_t)config->register_count > 65536U) {
        return false;
    }

    if (!valid_register_type(config->register_type)) {
        return false;
    }

    return true;
}


/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

esp_err_t modbus_device_manager_init(void)
{
    if (s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!modbus_master_is_initialized()) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(
        s_slots,
        0,
        sizeof(s_slots));

    for (uint8_t slot = MODBUS_DEVICE_MIN_SLOT;
         slot < MODBUS_DEVICE_MAX_SLOTS;
         slot++) {

        s_slots[slot].config.enabled = false;

        s_slots[slot].config.slave_address =
            MODBUS_DEFAULT_SLAVE;

        s_slots[slot].config.start_register = 0U;

        s_slots[slot].config.register_count = 1U;

        s_slots[slot].config.register_type =
            MODBUS_DEVICE_REGISTER_HOLDING;

        s_slots[slot].state.valid = false;
        s_slots[slot].state.last_error = ESP_OK;
    }

    s_initialized = true;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Deinitialization                                                           */
/* -------------------------------------------------------------------------- */

esp_err_t modbus_device_manager_deinit(void)
{
    if (!s_initialized) {
        return ESP_OK;
    }

    memset(
        s_slots,
        0,
        sizeof(s_slots));

    s_initialized = false;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Configuration                                                              */
/* -------------------------------------------------------------------------- */

esp_err_t modbus_device_configure(
    uint8_t slot,
    const modbus_device_config_t *config)
{
    if (!valid_slot(slot)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!valid_config(config)) {
        return ESP_ERR_INVALID_ARG;
    }

    s_slots[slot].config = *config;

    /*
     * A new configuration invalidates the previous
     * measurement result.
     */
    s_slots[slot].state.valid = false;
    s_slots[slot].state.last_error = ESP_OK;
    s_slots[slot].state.register_count = 0U;

    memset(
        s_slots[slot].state.registers,
        0,
        sizeof(s_slots[slot].state.registers));

    return ESP_OK;
}


esp_err_t modbus_device_disable(
    uint8_t slot)
{
    if (!valid_slot(slot)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    s_slots[slot].config.enabled = false;

    return ESP_OK;
}


esp_err_t modbus_device_get_config(
    uint8_t slot,
    modbus_device_config_t *config)
{
    if (!valid_slot(slot) ||
        config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    *config = s_slots[slot].config;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Polling                                                                    */
/* -------------------------------------------------------------------------- */

esp_err_t modbus_device_poll(
    uint8_t slot)
{
    if (!valid_slot(slot)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    modbus_device_slot_t *device =
        &s_slots[slot];

    if (!device->config.enabled) {
        return ESP_ERR_INVALID_STATE;
    }

    uint16_t registers[
        MODBUS_DEVICE_MAX_REGISTERS
    ] = {0};

    esp_err_t err;

    if (device->config.register_type ==
        MODBUS_DEVICE_REGISTER_INPUT) {

        err = modbus_read_input_registers(
            device->config.slave_address,
            device->config.start_register,
            device->config.register_count,
            registers);

    } else {

        err = modbus_read_holding_registers(
            device->config.slave_address,
            device->config.start_register,
            device->config.register_count,
            registers);
    }

    if (err != ESP_OK) {
        device->state.last_error = err;
        device->state.failed_polls++;

        return err;
    }

    memcpy(
        device->state.registers,
        registers,
        device->config.register_count *
            sizeof(uint16_t));

    device->state.register_count =
        device->config.register_count;

    device->state.last_error = ESP_OK;
    device->state.successful_polls++;
    device->state.valid = true;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* State                                                                      */
/* -------------------------------------------------------------------------- */

esp_err_t modbus_device_get_state(
    uint8_t slot,
    modbus_device_state_t *state)
{
    if (!valid_slot(slot) ||
        state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    *state = s_slots[slot].state;

    return ESP_OK;
}


bool modbus_device_manager_is_initialized(void)
{
    return s_initialized;
}


bool modbus_device_is_enabled(
    uint8_t slot)
{
    if (!valid_slot(slot)) {
        return false;
    }

    if (!s_initialized) {
        return false;
    }

    return s_slots[slot].config.enabled;
}