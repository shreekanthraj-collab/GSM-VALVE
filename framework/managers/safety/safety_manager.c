#include "safety_manager.h"

static bool s_initialized = false;
static bool s_fault_latched = false;
static safety_manager_config_t s_config = {0};
static safety_manager_state_t s_state = SAFETY_STATE_NORMAL;
static uint32_t s_motor_start_ms = 0U;
static bool s_motor_timing_active = false;
static uint32_t s_battery_cut_start_ms = 0U;
static bool s_battery_cut_timing_active = false;

static bool valid_config(const safety_manager_config_t *config)
{
    if (config == NULL) return false;
    if (config->battery_critical_voltage_v < 0.0f) return false;
    if (config->battery_cut_voltage_v < config->battery_critical_voltage_v) return false;
    if (config->overcurrent_trip_a <= 0.0f) return false;
    if (config->battery_bypass_timeout_ms == 0U) return false;
    if (config->motor_max_runtime_ms == 0U) return false;
    return true;
}

static uint32_t elapsed_ms(uint32_t now_ms, uint32_t start_ms)
{
    return now_ms - start_ms;
}

static void clear_timing_state(void)
{
    s_motor_start_ms = 0U;
    s_motor_timing_active = false;
    s_battery_cut_start_ms = 0U;
    s_battery_cut_timing_active = false;
}

esp_err_t safety_manager_init(const safety_manager_config_t *config)
{
    if (s_initialized) return ESP_ERR_INVALID_STATE;
    if (!valid_config(config)) return ESP_ERR_INVALID_ARG;
    s_config = *config;
    s_fault_latched = false;
    s_state = SAFETY_STATE_NORMAL;
    clear_timing_state();
    s_initialized = true;
    return ESP_OK;
}

esp_err_t safety_manager_evaluate(
    const safety_manager_inputs_t *inputs,
    uint32_t now_ms,
    safety_manager_output_t *output)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    if (inputs == NULL || output == NULL) return ESP_ERR_INVALID_ARG;

    *output = (safety_manager_output_t){0};
    output->valid = true;

    if (s_fault_latched) {
        s_state = SAFETY_STATE_FAULT;
        output->state = s_state;
        output->request_motor_stop = inputs->motor_running;
        output->fault = true;
        return ESP_OK;
    }

    if (inputs->motor_running) {
        if (!s_motor_timing_active) {
            s_motor_start_ms = now_ms;
            s_motor_timing_active = true;
        }
        output->motor_runtime_ms = elapsed_ms(now_ms, s_motor_start_ms);
    } else {
        s_motor_timing_active = false;
        s_motor_start_ms = 0U;
    }

    if (!inputs->battery_valid) {
        s_fault_latched = true;
        s_state = SAFETY_STATE_FAULT;
        output->state = s_state;
        output->request_motor_stop = inputs->motor_running;
        output->fault = true;
        return ESP_OK;
    }

    if (inputs->battery_voltage_v < s_config.battery_critical_voltage_v) {
        s_fault_latched = true;
        s_state = SAFETY_STATE_FAULT;
        output->state = s_state;
        output->request_motor_stop = inputs->motor_running;
        output->request_motor_close = inputs->motor_running;
        output->fault = true;
        return ESP_OK;
    }

    if (inputs->motor_running &&
        inputs->current_valid &&
        inputs->motor_current_a >= s_config.overcurrent_trip_a) {
        s_fault_latched = true;
        s_state = SAFETY_STATE_OVERCURRENT;
        output->state = s_state;
        output->request_motor_stop = true;
        output->request_motor_close = true;
        output->fault = true;
        return ESP_OK;
    }

    if (inputs->motor_running &&
        s_motor_timing_active &&
        output->motor_runtime_ms >= s_config.motor_max_runtime_ms) {
        s_fault_latched = true;
        s_state = SAFETY_STATE_MOTOR_TIMEOUT;
        output->state = s_state;
        output->request_motor_stop = true;
        output->request_motor_close = true;
        output->fault = true;
        return ESP_OK;
    }

    if (inputs->motor_running &&
        inputs->battery_voltage_v < s_config.battery_cut_voltage_v) {

        if (!s_battery_cut_timing_active) {
            s_battery_cut_start_ms = now_ms;
            s_battery_cut_timing_active = true;
        }

        output->battery_cut_elapsed_ms =
            elapsed_ms(now_ms, s_battery_cut_start_ms);

        if (inputs->bypass_acknowledged) {
            s_state = SAFETY_STATE_LOW_BATTERY;
            output->state = s_state;
            output->allow_motor_run = true;
            return ESP_OK;
        }

        if (output->battery_cut_elapsed_ms <
            s_config.battery_bypass_timeout_ms) {
            s_state = SAFETY_STATE_BATTERY_CUT_PENDING;
            output->state = s_state;
            output->allow_motor_run = true;
            output->bypass_required = true;
            return ESP_OK;
        }

        s_state = SAFETY_STATE_BATTERY_CUT;
        output->state = s_state;
        output->request_motor_stop = true;
        output->request_motor_close = true;
        return ESP_OK;
    }

    s_battery_cut_timing_active = false;
    s_battery_cut_start_ms = 0U;
    s_state = SAFETY_STATE_NORMAL;
    output->state = s_state;
    output->allow_motor_start = true;
    output->allow_motor_run = true;
    return ESP_OK;
}

esp_err_t safety_manager_clear_fault(void)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    s_fault_latched = false;
    s_state = SAFETY_STATE_NORMAL;
    clear_timing_state();
    return ESP_OK;
}

esp_err_t safety_manager_reset_cycle(void)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    clear_timing_state();
    if (!s_fault_latched) s_state = SAFETY_STATE_NORMAL;
    return ESP_OK;
}

bool safety_manager_is_initialized(void)
{
    return s_initialized;
}

safety_manager_state_t safety_manager_get_state(void)
{
    return s_state;
}