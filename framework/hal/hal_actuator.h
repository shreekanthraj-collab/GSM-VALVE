#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif


/* -------------------------------------------------------------------------- */
/* Actuator hardware state                                                    */
/* -------------------------------------------------------------------------- */

typedef enum
{
    HAL_ACTUATOR_STATE_IDLE = 0,

    HAL_ACTUATOR_STATE_OPENING,

    HAL_ACTUATOR_STATE_CLOSING,

    HAL_ACTUATOR_STATE_STOPPING,

    HAL_ACTUATOR_STATE_FAULT

} hal_actuator_state_t;


/* -------------------------------------------------------------------------- */
/* Actuator hardware configuration                                            */
/* -------------------------------------------------------------------------- */

typedef struct
{
    /*
     * GPIO assignments are deliberately supplied by the board layer.
     *
     * The HAL does not assume the final PCB pin map.
     */
    int power_gpio;
    int forward_gpio;
    int reverse_gpio;
    int pwm_gpio;

    /*
     * PWM configuration.
     */
    uint32_t pwm_frequency_hz;
    uint8_t pwm_resolution_bits;

} hal_actuator_config_t;


/* -------------------------------------------------------------------------- */
/* Initialization                                                              */
/* -------------------------------------------------------------------------- */

/*
 * Initialize the actuator hardware abstraction.
 *
 * No motor movement is performed by initialization.
 */
esp_err_t hal_actuator_init(
    const hal_actuator_config_t *config);


/*
 * Deinitialize actuator hardware.
 *
 * The actuator is forced to a safe stopped state before
 * hardware resources are released.
 */
esp_err_t hal_actuator_deinit(
    void);


/* -------------------------------------------------------------------------- */
/* Power control                                                               */
/* -------------------------------------------------------------------------- */

/*
 * Enable or disable actuator motor power.
 */
esp_err_t hal_actuator_set_power(
    bool enabled);


/* -------------------------------------------------------------------------- */
/* Direction control                                                           */
/* -------------------------------------------------------------------------- */

/*
 * Command forward/open direction.
 *
 * Direction pins are controlled by the HAL.
 */
esp_err_t hal_actuator_set_forward(
    bool enabled);


/*
 * Command reverse/close direction.
 *
 * Direction pins are controlled by the HAL.
 */
esp_err_t hal_actuator_set_reverse(
    bool enabled);


/* -------------------------------------------------------------------------- */
/* PWM control                                                                 */
/* -------------------------------------------------------------------------- */

/*
 * Set motor PWM duty cycle.
 *
 * duty_percent:
 *     0.0 to 100.0 percent.
 */
esp_err_t hal_actuator_set_pwm(
    float duty_percent);


/* -------------------------------------------------------------------------- */
/* Combined motor operations                                                   */
/* -------------------------------------------------------------------------- */

/*
 * Start motor in the forward/open direction.
 */
esp_err_t hal_actuator_start_open(
    float duty_percent);


/*
 * Start motor in the reverse/close direction.
 */
esp_err_t hal_actuator_start_close(
    float duty_percent);


/*
 * Stop the motor and remove direction drive.
 */
esp_err_t hal_actuator_stop(
    void);


/*
 * Force the actuator into a safe hardware state.
 *
 * This must remove motor power and direction drive.
 */
esp_err_t hal_actuator_force_safe(
    void);


/* -------------------------------------------------------------------------- */
/* Status                                                                     */
/* -------------------------------------------------------------------------- */

esp_err_t hal_actuator_get_state(
    hal_actuator_state_t *state);


bool hal_actuator_is_initialized(
    void);


bool hal_actuator_is_power_enabled(
    void);


bool hal_actuator_is_running(
    void);


#ifdef __cplusplus
}
#endif
