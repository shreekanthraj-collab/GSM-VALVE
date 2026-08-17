#include "hal_actuator.h"

#include "driver/ledc.h"

#include "hal_gpio.h"
#include "hal_pwm.h"


/* -------------------------------------------------------------------------- */
/* Internal state                                                             */
/* -------------------------------------------------------------------------- */

static bool s_initialized = false;

static hal_actuator_config_t s_config = {0};

static hal_actuator_state_t s_state =
    HAL_ACTUATOR_STATE_IDLE;

static bool s_power_enabled = false;
static bool s_running = false;


/* -------------------------------------------------------------------------- */
/* Configuration validation                                                   */
/* -------------------------------------------------------------------------- */

static bool valid_config(
    const hal_actuator_config_t *config)
{
    if (config == NULL) {
        return false;
    }

    if (config->power_gpio < 0 ||
        config->forward_gpio < 0 ||
        config->reverse_gpio < 0 ||
        config->pwm_gpio < 0) {
        return false;
    }

    if (config->pwm_frequency_hz == 0U) {
        return false;
    }

    if (config->pwm_resolution_bits == 0U ||
        config->pwm_resolution_bits > 20U) {
        return false;
    }

    return true;
}


/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

esp_err_t hal_actuator_init(
    const hal_actuator_config_t *config)
{
    if (s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!valid_config(config)) {
        return ESP_ERR_INVALID_ARG;
    }

    s_config = *config;

    /*
     * Configure motor control GPIO signals through the GPIO HAL.
     */
    esp_err_t err =
        hal_gpio_configure(
            HAL_GPIO_SIGNAL_MOTOR_POWER,
            HAL_GPIO_MODE_OUTPUT,
            HAL_GPIO_PULL_NONE);

    if (err != ESP_OK) {
        return err;
    }

    err =
        hal_gpio_configure(
            HAL_GPIO_SIGNAL_MOTOR_FORWARD,
            HAL_GPIO_MODE_OUTPUT,
            HAL_GPIO_PULL_NONE);

    if (err != ESP_OK) {
        return err;
    }

    err =
        hal_gpio_configure(
            HAL_GPIO_SIGNAL_MOTOR_REVERSE,
            HAL_GPIO_MODE_OUTPUT,
            HAL_GPIO_PULL_NONE);

    if (err != ESP_OK) {
        return err;
    }

    /*
     * PWM ownership remains inside HAL PWM.
     *
     * The board layer supplies the actual LEDC channel/timer
     * through the configuration.
     *
     * The GPIO number stored in hal_actuator_config_t is retained
     * for board-level documentation/validation, while HAL GPIO
     * owns the logical motor signals.
     */
    hal_pwm_config_t pwm_config = {
        .speed_mode =
            LEDC_LOW_SPEED_MODE,

        .channel =
            LEDC_CHANNEL_0,

        .gpio_num =
            config->pwm_gpio,

        .timer =
            LEDC_TIMER_0,

        .duty_resolution =
            (ledc_timer_bit_t)
                config->pwm_resolution_bits,

        .frequency_hz =
            config->pwm_frequency_hz
    };

    err =
        hal_pwm_init(&pwm_config);

    if (err != ESP_OK) {
        return err;
    }

    /*
     * Always enter the hardware in a safe state.
     */
    err =
        hal_gpio_write(
            HAL_GPIO_SIGNAL_MOTOR_FORWARD,
            HAL_GPIO_LEVEL_LOW);

    if (err != ESP_OK) {
        return err;
    }

    err =
        hal_gpio_write(
            HAL_GPIO_SIGNAL_MOTOR_REVERSE,
            HAL_GPIO_LEVEL_LOW);

    if (err != ESP_OK) {
        return err;
    }

    err =
        hal_gpio_write(
            HAL_GPIO_SIGNAL_MOTOR_POWER,
            HAL_GPIO_LEVEL_LOW);

    if (err != ESP_OK) {
        return err;
    }

    err =
        hal_pwm_disable(
            LEDC_CHANNEL_0);

    if (err != ESP_OK) {
        return err;
    }

    s_power_enabled = false;
    s_running = false;
    s_state = HAL_ACTUATOR_STATE_IDLE;

    s_initialized = true;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Deinitialization                                                           */
/* -------------------------------------------------------------------------- */

esp_err_t hal_actuator_deinit(
    void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err =
        hal_actuator_force_safe();

    if (err != ESP_OK) {
        return err;
    }

    err =
        hal_pwm_deinit(
            LEDC_CHANNEL_0);

    if (err != ESP_OK) {
        return err;
    }

    s_initialized = false;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Power control                                                              */
/* -------------------------------------------------------------------------- */

esp_err_t hal_actuator_set_power(
    bool enabled)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    /*
     * Motor power is controlled through GPIO HAL.
     */
    esp_err_t err =
        hal_gpio_write(
            HAL_GPIO_SIGNAL_MOTOR_POWER,
            enabled
                ? HAL_GPIO_LEVEL_HIGH
                : HAL_GPIO_LEVEL_LOW);

    if (err != ESP_OK) {
        return err;
    }

    s_power_enabled = enabled;

    if (!enabled) {
        s_running = false;

        s_state =
            HAL_ACTUATOR_STATE_IDLE;
    }

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Direction control                                                          */
/* -------------------------------------------------------------------------- */

esp_err_t hal_actuator_set_forward(
    bool enabled)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    /*
     * Never permit both directions simultaneously.
     */
    if (enabled) {

        esp_err_t err =
            hal_gpio_write(
                HAL_GPIO_SIGNAL_MOTOR_REVERSE,
                HAL_GPIO_LEVEL_LOW);

        if (err != ESP_OK) {
            return err;
        }
    }

    return hal_gpio_write(
        HAL_GPIO_SIGNAL_MOTOR_FORWARD,
        enabled
            ? HAL_GPIO_LEVEL_HIGH
            : HAL_GPIO_LEVEL_LOW);
}


esp_err_t hal_actuator_set_reverse(
    bool enabled)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    /*
     * Never permit both directions simultaneously.
     */
    if (enabled) {

        esp_err_t err =
            hal_gpio_write(
                HAL_GPIO_SIGNAL_MOTOR_FORWARD,
                HAL_GPIO_LEVEL_LOW);

        if (err != ESP_OK) {
            return err;
        }
    }

    return hal_gpio_write(
        HAL_GPIO_SIGNAL_MOTOR_REVERSE,
        enabled
            ? HAL_GPIO_LEVEL_HIGH
            : HAL_GPIO_LEVEL_LOW);
}


/* -------------------------------------------------------------------------- */
/* PWM                                                                        */
/* -------------------------------------------------------------------------- */

esp_err_t hal_actuator_set_pwm(
    float duty_percent)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    return hal_pwm_set_duty(
        LEDC_CHANNEL_0,
        duty_percent);
}


/* -------------------------------------------------------------------------- */
/* Start OPEN                                                                 */
/* -------------------------------------------------------------------------- */

esp_err_t hal_actuator_start_open(
    float duty_percent)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!s_power_enabled) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err =
        hal_actuator_set_reverse(false);

    if (err != ESP_OK) {
        return err;
    }

    err =
        hal_actuator_set_forward(true);

    if (err != ESP_OK) {
        return err;
    }

    err =
        hal_pwm_set_duty(
            LEDC_CHANNEL_0,
            duty_percent);

    if (err != ESP_OK) {
        return err;
    }

    err =
        hal_pwm_enable(
            LEDC_CHANNEL_0);

    if (err != ESP_OK) {
        return err;
    }

    s_running = true;

    s_state =
        HAL_ACTUATOR_STATE_OPENING;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Start CLOSE                                                                */
/* -------------------------------------------------------------------------- */

esp_err_t hal_actuator_start_close(
    float duty_percent)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!s_power_enabled) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err =
        hal_actuator_set_forward(false);

    if (err != ESP_OK) {
        return err;
    }

    err =
        hal_actuator_set_reverse(true);

    if (err != ESP_OK) {
        return err;
    }

    err =
        hal_pwm_set_duty(
            LEDC_CHANNEL_0,
            duty_percent);

    if (err != ESP_OK) {
        return err;
    }

    err =
        hal_pwm_enable(
            LEDC_CHANNEL_0);

    if (err != ESP_OK) {
        return err;
    }

    s_running = true;

    s_state =
        HAL_ACTUATOR_STATE_CLOSING;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Stop                                                                       */
/* -------------------------------------------------------------------------- */

esp_err_t hal_actuator_stop(
    void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    /*
     * Disable PWM first.
     */
    esp_err_t err =
        hal_pwm_disable(
            LEDC_CHANNEL_0);

    if (err != ESP_OK) {
        return err;
    }

    s_state =
        HAL_ACTUATOR_STATE_STOPPING;

    /*
     * Remove both direction commands.
     */
    err =
        hal_actuator_set_forward(false);

    if (err != ESP_OK) {
        return err;
    }

    err =
        hal_actuator_set_reverse(false);

    if (err != ESP_OK) {
        return err;
    }

    s_running = false;

    s_state =
        HAL_ACTUATOR_STATE_IDLE;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Force safe                                                                 */
/* -------------------------------------------------------------------------- */

esp_err_t hal_actuator_force_safe(
    void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    /*
     * PWM off first.
     */
    esp_err_t err =
        hal_pwm_disable(
            LEDC_CHANNEL_0);

    if (err != ESP_OK) {
        return err;
    }

    /*
     * Remove both direction outputs.
     */
    err =
        hal_gpio_write(
            HAL_GPIO_SIGNAL_MOTOR_FORWARD,
            HAL_GPIO_LEVEL_LOW);

    if (err != ESP_OK) {
        return err;
    }

    err =
        hal_gpio_write(
            HAL_GPIO_SIGNAL_MOTOR_REVERSE,
            HAL_GPIO_LEVEL_LOW);

    if (err != ESP_OK) {
        return err;
    }

    /*
     * Remove motor power last.
     */
    err =
        hal_gpio_write(
            HAL_GPIO_SIGNAL_MOTOR_POWER,
            HAL_GPIO_LEVEL_LOW);

    if (err != ESP_OK) {
        return err;
    }

    s_power_enabled = false;
    s_running = false;

    s_state =
        HAL_ACTUATOR_STATE_IDLE;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Status                                                                     */
/* -------------------------------------------------------------------------- */

esp_err_t hal_actuator_get_state(
    hal_actuator_state_t *state)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *state =
        s_state;

    return ESP_OK;
}


bool hal_actuator_is_initialized(
    void)
{
    return s_initialized;
}


bool hal_actuator_is_power_enabled(
    void)
{
    return s_initialized &&
           s_power_enabled;
}


bool hal_actuator_is_running(
    void)
{
    return s_initialized &&
           s_running;
}