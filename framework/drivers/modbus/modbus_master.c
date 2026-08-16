#include "modbus_master.h"

#include <stddef.h>

#include "drv_rs485.h"


/* -------------------------------------------------------------------------- */
/* Constants                                                                  */
/* -------------------------------------------------------------------------- */

#define MODBUS_FUNCTION_READ_HOLDING    0x03U
#define MODBUS_FUNCTION_READ_INPUT      0x04U
#define MODBUS_FUNCTION_WRITE_SINGLE    0x06U
#define MODBUS_FUNCTION_WRITE_MULTIPLE  0x10U

#define MODBUS_MIN_FRAME                4U
#define MODBUS_MAX_REGISTERS_DEFAULT    125U
#define MODBUS_MAX_REGISTERS_WRITE      123U

#define MODBUS_EXCEPTION_MASK           0x80U

#define MODBUS_EXCEPTION_ILLEGAL_FUNCTION    0x01U
#define MODBUS_EXCEPTION_ILLEGAL_ADDRESS     0x02U
#define MODBUS_EXCEPTION_ILLEGAL_VALUE       0x03U
#define MODBUS_EXCEPTION_DEVICE_FAILURE      0x04U
#define MODBUS_EXCEPTION_ACKNOWLEDGE         0x05U
#define MODBUS_EXCEPTION_DEVICE_BUSY        0x06U
#define MODBUS_EXCEPTION_NEGATIVE_ACK        0x07U
#define MODBUS_EXCEPTION_MEMORY_PARITY       0x08U


/* -------------------------------------------------------------------------- */
/* Internal state                                                             */
/* -------------------------------------------------------------------------- */

static bool s_initialized = false;

static modbus_master_config_t s_config = {
    .slave_address = MODBUS_DEFAULT_SLAVE,
    .timeout_ms = 1000U,
    .max_registers = MODBUS_MAX_REGISTERS_DEFAULT
};


/* -------------------------------------------------------------------------- */
/* Validation                                                                 */
/* -------------------------------------------------------------------------- */

static bool valid_slave_address(
    uint8_t slave_address)
{
    /*
     * Modbus RTU slave addresses 1..247 are valid.
     * 0 is the broadcast address and is not supported by
     * this request/response master API.
     */
    return slave_address >= 1U &&
           slave_address <= 247U;
}


static bool valid_config(
    const modbus_master_config_t *config)
{
    if (config == NULL) {
        return false;
    }

    if (!valid_slave_address(config->slave_address)) {
        return false;
    }

    if (config->timeout_ms == 0U) {
        return false;
    }

    if (config->max_registers == 0U ||
        config->max_registers > MODBUS_MAX_REGISTERS_DEFAULT) {
        return false;
    }

    return true;
}


static bool valid_read_request(
    uint8_t slave_address,
    uint16_t register_count)
{
    if (!valid_slave_address(slave_address)) {
        return false;
    }

    if (register_count == 0U ||
        register_count > s_config.max_registers) {
        return false;
    }

    return true;
}


static bool valid_write_multiple_request(
    uint8_t slave_address,
    uint16_t register_count)
{
    if (!valid_slave_address(slave_address)) {
        return false;
    }

    if (register_count == 0U ||
        register_count > MODBUS_MAX_REGISTERS_WRITE) {
        return false;
    }

    if (register_count > s_config.max_registers) {
        return false;
    }

    return true;
}


/* -------------------------------------------------------------------------- */
/* CRC                                                                        */
/* -------------------------------------------------------------------------- */

static uint16_t modbus_crc16(
    const uint8_t *data,
    size_t length)
{
    uint16_t crc = 0xFFFFU;

    if (data == NULL) {
        return crc;
    }

    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];

        for (uint8_t bit = 0; bit < 8U; ++bit) {
            if ((crc & 0x0001U) != 0U) {
                crc >>= 1U;
                crc ^= 0xA001U;
            } else {
                crc >>= 1U;
            }
        }
    }

    return crc;
}


static void append_crc(
    uint8_t *frame,
    size_t length_without_crc)
{
    uint16_t crc = modbus_crc16(
        frame,
        length_without_crc);

    /*
     * Modbus RTU transmits CRC low byte first.
     */
    frame[length_without_crc] =
        (uint8_t)(crc & 0xFFU);

    frame[length_without_crc + 1U] =
        (uint8_t)((crc >> 8U) & 0xFFU);
}


static bool valid_crc(
    const uint8_t *frame,
    size_t length)
{
    if (frame == NULL ||
        length < MODBUS_MIN_FRAME) {
        return false;
    }

    uint16_t received_crc =
        (uint16_t)frame[length - 2U] |
        ((uint16_t)frame[length - 1U] << 8U);

    uint16_t calculated_crc =
        modbus_crc16(
            frame,
            length - 2U);

    return received_crc == calculated_crc;
}


/* -------------------------------------------------------------------------- */
/* Error mapping                                                              */
/* -------------------------------------------------------------------------- */

static esp_err_t map_exception(
    uint8_t exception_code)
{
    switch (exception_code) {
        case MODBUS_EXCEPTION_ILLEGAL_FUNCTION:
            return ESP_ERR_NOT_SUPPORTED;

        case MODBUS_EXCEPTION_ILLEGAL_ADDRESS:
            return ESP_ERR_INVALID_ARG;

        case MODBUS_EXCEPTION_ILLEGAL_VALUE:
            return ESP_ERR_INVALID_ARG;

        case MODBUS_EXCEPTION_DEVICE_FAILURE:
            return ESP_FAIL;

        case MODBUS_EXCEPTION_ACKNOWLEDGE:
            return ESP_ERR_TIMEOUT;

        case MODBUS_EXCEPTION_DEVICE_BUSY:
            return ESP_ERR_TIMEOUT;

        case MODBUS_EXCEPTION_NEGATIVE_ACK:
            return ESP_FAIL;

        case MODBUS_EXCEPTION_MEMORY_PARITY:
            return ESP_FAIL;

        default:
            return ESP_ERR_INVALID_RESPONSE;
    }
}


/* -------------------------------------------------------------------------- */
/* Response handling                                                          */
/* -------------------------------------------------------------------------- */

static esp_err_t receive_response(
    uint8_t *response,
    size_t response_capacity,
    size_t *response_length)
{
    if (response == NULL ||
        response_length == NULL ||
        response_capacity < MODBUS_MIN_FRAME) {
        return ESP_ERR_INVALID_ARG;
    }

    *response_length = 0U;

    esp_err_t err =
        drv_rs485_read(
            response,
            response_capacity,
            response_length);

    if (err != ESP_OK) {
        return err;
    }

    if (*response_length < MODBUS_MIN_FRAME) {
        return ESP_ERR_TIMEOUT;
    }

    if (!valid_crc(
            response,
            *response_length)) {
        return ESP_ERR_INVALID_CRC;
    }

    if ((response[1] &
         MODBUS_EXCEPTION_MASK) != 0U) {

        if (*response_length != 5U) {
            return ESP_ERR_INVALID_RESPONSE;
        }

        return map_exception(
            response[2]);
    }

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Request/response exchange                                                  */
/* -------------------------------------------------------------------------- */

static esp_err_t exchange(
    const uint8_t *request,
    size_t request_length,
    uint8_t *response,
    size_t response_capacity,
    size_t *response_length)
{
    if (request == NULL ||
        request_length < MODBUS_MIN_FRAME ||
        request_length > MODBUS_MAX_FRAME) {
        return ESP_ERR_INVALID_ARG;
    }

    if (response == NULL ||
        response_length == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *response_length = 0U;

    esp_err_t err =
        drv_rs485_flush_rx();

    if (err != ESP_OK) {
        return err;
    }

    size_t bytes_written = 0U;

    err =
        drv_rs485_write(
            request,
            request_length,
            &bytes_written);

    if (err != ESP_OK) {
        return err;
    }

    if (bytes_written != request_length) {
        return ESP_ERR_INVALID_STATE;
    }

    return receive_response(
        response,
        response_capacity,
        response_length);
}


/* -------------------------------------------------------------------------- */
/* Read response                                                              */
/* -------------------------------------------------------------------------- */

static esp_err_t parse_read_response(
    uint8_t slave_address,
    uint8_t function,
    uint16_t register_count,
    const uint8_t *response,
    size_t response_length,
    uint16_t *registers)
{
    if (response == NULL ||
        registers == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t expected_byte_count =
        (size_t)register_count * 2U;

    size_t expected_length =
        expected_byte_count + 5U;

    if (expected_byte_count > MODBUS_MAX_BYTE_COUNT ||
        expected_length > MODBUS_MAX_FRAME) {
        return ESP_ERR_INVALID_SIZE;
    }

    if (response_length != expected_length) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    if (response[0] != slave_address) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    if (response[1] != function) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    if (response[2] !=
        (uint8_t)expected_byte_count) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    for (uint16_t i = 0;
         i < register_count;
         ++i) {

        size_t index =
            3U + ((size_t)i * 2U);

        registers[i] =
            ((uint16_t)response[index] << 8U) |
            (uint16_t)response[index + 1U];
    }

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

esp_err_t modbus_master_init(
    const modbus_master_config_t *config)
{
    if (s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!valid_config(config)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!drv_rs485_is_initialized()) {
        return ESP_ERR_INVALID_STATE;
    }

    s_config = *config;

    s_initialized = true;

    return ESP_OK;
}


esp_err_t modbus_master_deinit(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    s_initialized = false;

    s_config.slave_address =
        MODBUS_DEFAULT_SLAVE;

    s_config.timeout_ms = 1000U;

    s_config.max_registers =
        MODBUS_MAX_REGISTERS_DEFAULT;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Read holding registers                                                     */
/* -------------------------------------------------------------------------- */

esp_err_t modbus_read_holding_registers(
    uint8_t slave_address,
    uint16_t start_register,
    uint16_t register_count,
    uint16_t *registers)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (registers == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!valid_read_request(
            slave_address,
            register_count)) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t request[8] = {0};

    request[0] = slave_address;
    request[1] =
        MODBUS_FUNCTION_READ_HOLDING;

    request[2] =
        (uint8_t)(start_register >> 8U);

    request[3] =
        (uint8_t)(start_register & 0xFFU);

    request[4] =
        (uint8_t)(register_count >> 8U);

    request[5] =
        (uint8_t)(register_count & 0xFFU);

    append_crc(request, 6U);

    uint8_t response[MODBUS_MAX_FRAME] = {0};
    size_t response_length = 0U;

    esp_err_t err =
        exchange(
            request,
            sizeof(request),
            response,
            sizeof(response),
            &response_length);

    if (err != ESP_OK) {
        return err;
    }

    return parse_read_response(
        slave_address,
        MODBUS_FUNCTION_READ_HOLDING,
        register_count,
        response,
        response_length,
        registers);
}


/* -------------------------------------------------------------------------- */
/* Read input registers                                                       */
/* -------------------------------------------------------------------------- */

esp_err_t modbus_read_input_registers(
    uint8_t slave_address,
    uint16_t start_register,
    uint16_t register_count,
    uint16_t *registers)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (registers == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!valid_read_request(
            slave_address,
            register_count)) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t request[8] = {0};

    request[0] = slave_address;
    request[1] =
        MODBUS_FUNCTION_READ_INPUT;

    request[2] =
        (uint8_t)(start_register >> 8U);

    request[3] =
        (uint8_t)(start_register & 0xFFU);

    request[4] =
        (uint8_t)(register_count >> 8U);

    request[5] =
        (uint8_t)(register_count & 0xFFU);

    append_crc(request, 6U);

    uint8_t response[MODBUS_MAX_FRAME] = {0};
    size_t response_length = 0U;

    esp_err_t err =
        exchange(
            request,
            sizeof(request),
            response,
            sizeof(response),
            &response_length);

    if (err != ESP_OK) {
        return err;
    }

    return parse_read_response(
        slave_address,
        MODBUS_FUNCTION_READ_INPUT,
        register_count,
        response,
        response_length,
        registers);
}


/* -------------------------------------------------------------------------- */
/* Write single register                                                      */
/* -------------------------------------------------------------------------- */

esp_err_t modbus_write_single_register(
    uint8_t slave_address,
    uint16_t register_address,
    uint16_t value)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!valid_slave_address(slave_address)) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t request[8] = {0};

    request[0] = slave_address;
    request[1] =
        MODBUS_FUNCTION_WRITE_SINGLE;

    request[2] =
        (uint8_t)(register_address >> 8U);

    request[3] =
        (uint8_t)(register_address & 0xFFU);

    request[4] =
        (uint8_t)(value >> 8U);

    request[5] =
        (uint8_t)(value & 0xFFU);

    append_crc(request, 6U);

    uint8_t response[8] = {0};
    size_t response_length = 0U;

    esp_err_t err =
        exchange(
            request,
            sizeof(request),
            response,
            sizeof(response),
            &response_length);

    if (err != ESP_OK) {
        return err;
    }

    if (response_length != sizeof(request)) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    if (response[0] != slave_address ||
        response[1] != MODBUS_FUNCTION_WRITE_SINGLE) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    for (size_t i = 2U; i < 6U; ++i) {
        if (response[i] != request[i]) {
            return ESP_ERR_INVALID_RESPONSE;
        }
    }

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Write multiple registers                                                   */
/* -------------------------------------------------------------------------- */

esp_err_t modbus_write_multiple_registers(
    uint8_t slave_address,
    uint16_t start_register,
    uint16_t register_count,
    const uint16_t *registers)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (registers == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!valid_write_multiple_request(
            slave_address,
            register_count)) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t byte_count =
        (size_t)register_count * 2U;

    size_t frame_without_crc =
        7U + byte_count;

    size_t frame_length =
        frame_without_crc + 2U;

    if (byte_count > MODBUS_MAX_BYTE_COUNT ||
        frame_length > MODBUS_MAX_FRAME) {
        return ESP_ERR_INVALID_SIZE;
    }

    uint8_t request[MODBUS_MAX_FRAME] = {0};

    request[0] = slave_address;
    request[1] =
        MODBUS_FUNCTION_WRITE_MULTIPLE;

    request[2] =
        (uint8_t)(start_register >> 8U);

    request[3] =
        (uint8_t)(start_register & 0xFFU);

    request[4] =
        (uint8_t)(register_count >> 8U);

    request[5] =
        (uint8_t)(register_count & 0xFFU);

    request[6] =
        (uint8_t)byte_count;

    for (uint16_t i = 0;
         i < register_count;
         ++i) {

        size_t index =
            7U + ((size_t)i * 2U);

        request[index] =
            (uint8_t)(registers[i] >> 8U);

        request[index + 1U] =
            (uint8_t)(registers[i] & 0xFFU);
    }

    append_crc(
        request,
        frame_without_crc);

    uint8_t response[8] = {0};
    size_t response_length = 0U;

    esp_err_t err =
        exchange(
            request,
            frame_length,
            response,
            sizeof(response),
            &response_length);

    if (err != ESP_OK) {
        return err;
    }

    if (response_length != 8U) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    if (response[0] != slave_address ||
        response[1] != MODBUS_FUNCTION_WRITE_MULTIPLE) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    uint16_t response_address =
        ((uint16_t)response[2] << 8U) |
        (uint16_t)response[3];

    uint16_t response_count =
        ((uint16_t)response[4] << 8U) |
        (uint16_t)response[5];

    if (response_address != start_register ||
        response_count != register_count) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* State                                                                      */
/* -------------------------------------------------------------------------- */

bool modbus_master_is_initialized(void)
{
    return s_initialized;
}