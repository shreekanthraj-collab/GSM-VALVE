#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ENCODER_MANAGER_ANGLE_MAX  4095U
#define ENCODER_MANAGER_HALF_RANGE 2048U

typedef struct
{
    uint16_t angle;
    int32_t rotation_count;
    int64_t total_angle;
    float total_turns;
    bool valid;
} encoder_manager_state_t;

esp_err_t encoder_manager_init(void);

esp_err_t encoder_manager_update(
    uint16_t angle);

esp_err_t encoder_manager_get_state(
    encoder_manager_state_t *state);

bool encoder_manager_is_valid(void);

esp_err_t encoder_manager_reset(void);

#ifdef __cplusplus
}
#endif
