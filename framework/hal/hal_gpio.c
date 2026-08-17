#include "hal_gpio.h"

#include "board_config.h"

#include "driver/gpio.h"


/* -------------------------------------------------------------------------- */
/* Internal helpers                                                           */
/* -------------------------------------------------------------------------- */

/*
 * Resolve a logical HAL GPIO signal to the physical board GPIO.
 *
 * The physical mapping belongs exclusively to board_config.h.
 */
static esp_err_t resolve_gpio(
    hal_gpio_signal_t signal,
    gpio_num_t *gpio)
{
    if (gpio == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    switch (signal) {

        case HAL_GPIO_SIGNAL_MOTOR_POWER:
            *gpio = BOARD_MOTOR_POWER_GPIO;
            break;

        case HAL_GPIO_SIGNAL_MOTOR_FORWARD:
            *gpio = BOARD_MOTOR_FORWARD_GPIO;
            break;

        case HAL_GPIO_SIGNAL_MOTOR_REVERSE:
            *gpio = BOARD_MOTOR_REVERSE_GPIO;
            break;

        case HAL_GPIO_SIGNAL_MOTOR_PWM:
            *gpio = BOARD_MOTOR_PWM_GPIO;
            break;

        case HAL_GPIO_SIGNAL_RS485_TX:
            *gpio = BOARD_RS485_TX_GPIO;
            break;

        case HAL_GPIO_SIGNAL_RS485_RX:
            *gpio = BOARD_RS485_RX_GPIO;
            break;

        case HAL_GPIO_SIGNAL_RS485_DE:
            *gpio = BOARD_RS485_DE_GPIO;
            break;

        case HAL_GPIO_SIGNAL_LIMIT_SWITCH:
            *gpio = BOARD_LIMIT_SWITCH_GPIO;
            break;

        case HAL_GPIO_SIGNAL_RTC_INT:
            *gpio = BOARD_RTC_INT_GPIO;
            break;

        case HAL_GPIO_SIGNAL_LTE_PWRKEY:
            *gpio = BOARD_LTE_PWRKEY_GPIO;
            break;

        case HAL_GPIO_SIGNAL_LTE_RESET:
            *gpio = BOARD_LTE_RESET_GPIO;
            break;

        case HAL_GPIO_SIGNAL_LTE_STATUS:
            *gpio = BOARD_LTE_STATUS_GPIO;
            break;

        case HAL_GPIO_SIGNAL_BUZZER:
            *gpio = BOARD_BUZZER_GPIO;
            break;

        case HAL_GPIO_SIGNAL_VBAT_ADC:
            *gpio = BOARD_VBAT_ADC_GPIO;
            break;

        default:
            return ESP_ERR_INVALID_ARG;
    }

    /*
     * Negative GPIO values mean that the board mapping
     * has not yet been defined.
     */
    if (*gpio < 0 ||
        *gpio >= GPIO_NUM_MAX) {

        return ESP_ERR_NOT_FOUND;
    }

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* GPIO configuration                                                         */
/* -------------------------------------------------------------------------- */

esp_err_t hal_gpio_configure(
    hal_gpio_signal_t signal,
    hal_gpio_mode_t mode,
    hal_gpio_pull_t pull)
{
    gpio_num_t gpio;

    esp_err_t err =
        resolve_gpio(
            signal,
            &gpio);

    if (err != ESP_OK) {
        return err;
    }

    gpio_config_t config = {
        .pin_bit_mask =
            1ULL << gpio,

        .mode =
            (mode == HAL_GPIO_MODE_OUTPUT)
                ? GPIO_MODE_OUTPUT
                : GPIO_MODE_INPUT,

        .pull_up_en =
            GPIO_PULLUP_DISABLE,

        .pull_down_en =
            GPIO_PULLDOWN_DISABLE,

        .intr_type =
            GPIO_INTR_DISABLE
    };

    switch (pull) {

        case HAL_GPIO_PULL_UP:

            config.pull_up_en =
                GPIO_PULLUP_ENABLE;

            break;

        case HAL_GPIO_PULL_DOWN:

            config.pull_down_en =
                GPIO_PULLDOWN_ENABLE;

            break;

        case HAL_GPIO_PULL_NONE:

        default:

            break;
    }

    return gpio_config(
        &config);
}


/* -------------------------------------------------------------------------- */
/* GPIO write                                                                 */
/* -------------------------------------------------------------------------- */

esp_err_t hal_gpio_write(
    hal_gpio_signal_t signal,
    hal_gpio_level_t level)
{
    gpio_num_t gpio;

    esp_err_t err =
        resolve_gpio(
            signal,
            &gpio);

    if (err != ESP_OK) {
        return err;
    }

    return gpio_set_level(
        gpio,
        level == HAL_GPIO_LEVEL_HIGH
            ? 1
            : 0);
}


/* -------------------------------------------------------------------------- */
/* GPIO read                                                                  */
/* -------------------------------------------------------------------------- */

esp_err_t hal_gpio_read(
    hal_gpio_signal_t signal,
    hal_gpio_level_t *level)
{
    if (level == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    gpio_num_t gpio;

    esp_err_t err =
        resolve_gpio(
            signal,
            &gpio);

    if (err != ESP_OK) {
        return err;
    }

    *level =
        gpio_get_level(gpio)
            ? HAL_GPIO_LEVEL_HIGH
            : HAL_GPIO_LEVEL_LOW;

    return ESP_OK;
}
