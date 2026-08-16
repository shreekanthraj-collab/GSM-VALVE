#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SAFETY_STATE_NORMAL = 0,
    SAFETY_STATE_LOW_BATTERY,
    SAFETY_STATE_BATTERY_CUT_PENDING,
    SAFETY_STATE_BATTERY_CUT,
    SAFETY_STATE_OVERCURRENT,
    SAFETY_STATE_MOTOR_TIMEOUT,
    SAFETY_STATE_FAULT
} safety_manager_state_t;

typedef struct {
    bool motor_running;
    float battery_voltage_v;
    bool battery_valid;
    float motor_current_a;
    bool current_valid;
    bool bypass_acknowledged;
} safety_manager_inputs_t;

typedef struct {
    float battery_cut_voltage_v;
    float battery_critical_voltage_v;
    float overcurrent_trip_a;
    uint32_t battery_bypass_timeout_ms;
    uint32_t motor_max_runtime_ms;
} safety_manager_config_t;

typedef struct {
    safety_manager_state_t state;
    bool allow_motor_start;
    bool allow_motor_run;
    bool request_motor_stop;
    bool request_motor_close;
    bool bypass_required;
    bool fault;
    uint32_t battery_cut_elapsed_ms;
    uint32_t motor_runtime_ms;
    bool valid;
} safety_manager_output_t;

esp_err_t safety_manager_init(const safety_manager_config_t *config);

esp_err_t safety_manager_evaluate(
    const safety_manager_inputs_t *inputs,
    uint32_t now_ms,
    safety_manager_output_t *output);

esp_err_t safety_manager_clear_fault(void);
esp_err_t safety_manager_reset_cycle(void);

bool safety_manager_is_initialized(void);
safety_manager_state_t safety_manager_get_state(void);

#ifdef __cplusplus
}
#endif
