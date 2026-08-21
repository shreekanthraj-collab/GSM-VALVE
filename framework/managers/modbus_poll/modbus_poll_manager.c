
#include "modbus_poll_manager.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "modbus_device_manager.h"


/* -------------------------------------------------------------------------- */
/* Constants                                                                  */
/* -------------------------------------------------------------------------- */

#define MODBUS_POLL_TASK_STACK_SIZE    4096U
#define MODBUS_POLL_TASK_PRIORITY      5U
#define MODBUS_POLL_SLOT_COUNT         8U


/* -------------------------------------------------------------------------- */
/* Internal state                                                             */
/* -------------------------------------------------------------------------- */

static bool s_initialized = false;

static TaskHandle_t s_task_handle = NULL;

static modbus_poll_manager_config_t s_config = {
    .poll_interval_ms =
        MODBUS_POLL_DEFAULT_INTERVAL_MS
};

static modbus_poll_manager_status_t s_status = {
    .running = false,
    .current_slot = 0U,
    .total_polls = 0U,
    .successful_polls = 0U,
    .failed_polls = 0U
};


/* -------------------------------------------------------------------------- */
/* Validation                                                                 */
/* -------------------------------------------------------------------------- */

static bool valid_config(
    const modbus_poll_manager_config_t *config)
{
    if (config == NULL) {
        return false;
    }

    if (config->poll_interval_ms == 0U) {
        return false;
    }

    return true;
}


/* -------------------------------------------------------------------------- */
/* Polling task                                                               */
/* -------------------------------------------------------------------------- */

static void modbus_poll_task(
    void *argument)
{
    (void)argument;

    s_status.running = true;

    while (true) {

        for (uint8_t slot = 0U;
             slot < MODBUS_POLL_SLOT_COUNT;
             ++slot) {

            s_status.current_slot = slot;

            if (!modbus_device_is_enabled(slot)) {
                continue;
            }

            s_status.total_polls++;

            esp_err_t err =
                modbus_device_poll(slot);

            if (err == ESP_OK) {

                s_status.successful_polls++;

            } else {

                s_status.failed_polls++;
            }
        }

        vTaskDelay(
            pdMS_TO_TICKS(
                s_config.poll_interval_ms));
    }
}


/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

esp_err_t modbus_poll_manager_init(
    const modbus_poll_manager_config_t *config)
{
    if (s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!valid_config(config)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!modbus_device_manager_is_initialized()) {
        return ESP_ERR_INVALID_STATE;
    }

    s_config = *config;

    s_status = (modbus_poll_manager_status_t){
        .running = false,
        .current_slot = 0U,
        .total_polls = 0U,
        .successful_polls = 0U,
        .failed_polls = 0U
    };

    s_task_handle = NULL;

    s_initialized = true;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Start                                                                      */
/* -------------------------------------------------------------------------- */

esp_err_t modbus_poll_manager_start(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_task_handle != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    BaseType_t result =
        xTaskCreate(
            modbus_poll_task,
            "modbus_poll",
            MODBUS_POLL_TASK_STACK_SIZE,
            NULL,
            MODBUS_POLL_TASK_PRIORITY,
            &s_task_handle);

    if (result != pdPASS) {
        s_task_handle = NULL;
        s_status.running = false;

        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Stop                                                                       */
/* -------------------------------------------------------------------------- */

esp_err_t modbus_poll_manager_stop(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_task_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    TaskHandle_t task =
        s_task_handle;

    s_task_handle = NULL;
    s_status.running = false;

    vTaskDelete(task);

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Deinitialization                                                           */
/* -------------------------------------------------------------------------- */

esp_err_t modbus_poll_manager_deinit(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_task_handle != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    s_config =
        (modbus_poll_manager_config_t){
            .poll_interval_ms =
                MODBUS_POLL_DEFAULT_INTERVAL_MS
        };

    s_status =
        (modbus_poll_manager_status_t){
            .running = false,
            .current_slot = 0U,
            .total_polls = 0U,
            .successful_polls = 0U,
            .failed_polls = 0U
        };

    s_initialized = false;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Status                                                                     */
/* -------------------------------------------------------------------------- */

esp_err_t modbus_poll_manager_get_status(
    modbus_poll_manager_status_t *status)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *status = s_status;

    return ESP_OK;
}


bool modbus_poll_manager_is_running(void)
{
    if (!s_initialized) {
        return false;
    }

    return s_task_handle != NULL;
}
