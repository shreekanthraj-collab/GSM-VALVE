#pragma once

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif


/* -------------------------------------------------------------------------- */
/* GPIO signals                                                               */
/* -------------------------------------------------------------------------- */

typedef enum
{
    HAL_GPIO_SIGNAL_MOTOR_POWER = 0,

    HAL_GPIO_SIGNAL_MOTOR_FORWARD,

    HAL_GPIO_SIGNAL_MOTOR_REVERSE,

    HAL_GPIO_SIGNAL_MOTOR_PWM,

    HAL_GPIO_SIGNAL_RS485_TX,

    HAL_GPIO_SIGNAL_RS485_RX,

    HAL_GPIO_SIGNAL_RS485_DE,

    HAL_GPIO_SIGNAL_LIMIT_SWITCH,

    HAL_GPIO_SIGNAL_RTC_INT,

    HAL_GPIO_SIGNAL_LTE_PWRKEY,

    HAL_GPIO_SIGNAL_LTE_RESET,

    HAL_GPIO_SIGNAL_LTE_STATUS,

    HAL_GPIO_SIGNAL_BUZZER,

    HAL_GPIO_SIGNAL_VBAT_ADC,

    HAL_GPIO_SIGNAL_COUNT

} hal_gpio_signal_t;


/* -------------------------------------------------------------------------- */
/* GPIO mode                                                                  */
/* -------------------------------------------------------------------------- */

typedef enum
{
    HAL_GPIO_MODE_INPUT = 0,

    HAL_GPIO_MODE_OUTPUT

} hal_gpio_mode_t;


/* -------------------------------------------------------------------------- */
/* GPIO level                                                                 */
/* -------------------------------------------------------------------------- */

typedef enum
{
    HAL_GPIO_LEVEL_LOW = 0,

    HAL_GPIO_LEVEL_HIGH

} hal_gpio_level_t;


/* -------------------------------------------------------------------------- */
/* GPIO pull configuration                                                     */
/* -------------------------------------------------------------------------- */

typedef enum
{
    HAL_GPIO_PULL_NONE = 0,

    HAL_GPIO_PULL_UP,

    HAL_GPIO_PULL_DOWN

} hal_gpio_pull_t;


/* -------------------------------------------------------------------------- */
/* Configuration                                                              */
/* -------------------------------------------------------------------------- */

/*
 * Configure a logical GPIO signal.
 *
 * Physical GPIO mapping is resolved by the board layer.
 */
esp_err_t hal_gpio_configure(
    hal_gpio_signal_t signal,
    hal_gpio_mode_t mode,
    hal_gpio_pull_t pull);


/* -------------------------------------------------------------------------- */
/* Output                                                                      */
/* -------------------------------------------------------------------------- */

/*
 * Write a logical GPIO signal.
 */
esp_err_t hal_gpio_write(
    hal_gpio_signal_t signal,
    hal_gpio_level_t level);


/* -------------------------------------------------------------------------- */
/* Input                                                                       */
/* -------------------------------------------------------------------------- */

/*
 * Read a logical GPIO signal.
 */
esp_err_t hal_gpio_read(
    hal_gpio_signal_t signal,
    hal_gpio_level_t *level);


#ifdef __cplusplus
}
#endif