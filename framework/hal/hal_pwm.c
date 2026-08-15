#include "hal_pwm.h"

#include "driver/gpio.h"

#define HAL_PWM_MAX_CHANNELS    LEDC_CHANNEL_MAX
#define HAL_PWM_MAX_TIMERS      LEDC_TIMER_MAX

typedef struct {
    bool initialized;
    bool enabled;

    ledc_mode_t speed_mode;
    ledc_channel_t channel;
    ledc_timer_t timer;

    int gpio_num;

    ledc_timer_bit_t duty_resolution;
    uint32_t frequency_hz;

    float duty_percent;
} hal_pwm_channel_state_t;

typedef struct {
    bool initialized;

    ledc_mode_t speed_mode;
    ledc_timer_t timer;

    ledc_timer_bit_t duty_resolution;
    uint32_t frequency_hz;

    uint32_t channel_users;
} hal_pwm_timer_state_t;

static hal_pwm_channel_state_t
s_channel_state[LEDC_SPEED_MODE_MAX][HAL_PWM_MAX_CHANNELS];

static hal_pwm_timer_state_t
s_timer_state[LEDC_SPEED_MODE_MAX][HAL_PWM_MAX_TIMERS];


/* -------------------------------------------------------------------------- */
/* Validation                                                                 */
/* -------------------------------------------------------------------------- */

static bool valid_speed_mode(ledc_mode_t speed_mode)
{
    return speed_mode == LEDC_LOW_SPEED_MODE;
}

static bool valid_channel(ledc_channel_t channel)
{
    return channel >= LEDC_CHANNEL_0 &&
           channel < LEDC_CHANNEL_MAX;
}

static bool valid_timer(ledc_timer_t timer)
{
    return timer >= LEDC_TIMER_0 &&
           timer < LEDC_TIMER_MAX;
}

static bool valid_gpio(int gpio_num)
{
    return gpio_num >= 0 &&
           gpio_num < GPIO_NUM_MAX;
}

static bool valid_duty_resolution(
    ledc_timer_bit_t resolution)
{
    return resolution >= LEDC_TIMER_1_BIT &&
           resolution < LEDC_TIMER_BIT_MAX;
}

static esp_err_t validate_config(
    const hal_pwm_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!valid_speed_mode(config->speed_mode)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!valid_channel(config->channel)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!valid_timer(config->timer)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!valid_gpio(config->gpio_num)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!valid_duty_resolution(
            config->duty_resolution)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (config->frequency_hz == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* State helpers                                                              */
/* -------------------------------------------------------------------------- */

static hal_pwm_channel_state_t *find_channel(
    ledc_channel_t channel)
{
    if (!valid_channel(channel)) {
        return NULL;
    }

    for (ledc_mode_t mode = LEDC_LOW_SPEED_MODE;
         mode < LEDC_SPEED_MODE_MAX;
         ++mode) {

        if (s_channel_state[mode][channel].initialized) {
            return &s_channel_state[mode][channel];
        }
    }

    return NULL;
}

static hal_pwm_timer_state_t *get_timer_state(
    ledc_mode_t speed_mode,
    ledc_timer_t timer)
{
    if (!valid_speed_mode(speed_mode) ||
        !valid_timer(timer)) {
        return NULL;
    }

    return &s_timer_state[speed_mode][timer];
}


/* -------------------------------------------------------------------------- */
/* Duty helpers                                                               */
/* -------------------------------------------------------------------------- */

static uint32_t duty_max(
    ledc_timer_bit_t resolution)
{
    return (uint32_t)((1ULL << resolution) - 1ULL);
}

static uint32_t duty_from_percent(
    ledc_timer_bit_t resolution,
    float duty_percent)
{
    uint32_t max_duty = duty_max(resolution);

    if (duty_percent <= 0.0f) {
        return 0;
    }

    if (duty_percent >= 100.0f) {
        return max_duty;
    }

    return (uint32_t)(
        ((double)max_duty *
         (double)duty_percent) /
        100.0
    );
}


/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

esp_err_t hal_pwm_init(
    const hal_pwm_config_t *config)
{
    esp_err_t err = validate_config(config);

    if (err != ESP_OK) {
        return err;
    }

    hal_pwm_channel_state_t *channel_state =
        &s_channel_state
            [config->speed_mode]
            [config->channel];

    hal_pwm_timer_state_t *timer_state =
        get_timer_state(
            config->speed_mode,
            config->timer);

    if (timer_state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (channel_state->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    /*
     * A timer may be shared, but its configuration cannot
     * be changed while another HAL channel is using it.
     */
    if (timer_state->initialized) {

        if (timer_state->duty_resolution !=
                config->duty_resolution ||
            timer_state->frequency_hz !=
                config->frequency_hz) {

            return ESP_ERR_INVALID_STATE;
        }

    } else {

        ledc_timer_config_t timer_config = {
            .speed_mode = config->speed_mode,
            .timer_num = config->timer,
            .duty_resolution =
                config->duty_resolution,
            .freq_hz = config->frequency_hz,
            .clk_cfg = LEDC_AUTO_CLK,
        };

        err = ledc_timer_config(&timer_config);

        if (err != ESP_OK) {
            return err;
        }

        timer_state->initialized = true;
        timer_state->speed_mode =
            config->speed_mode;
        timer_state->timer =
            config->timer;
        timer_state->duty_resolution =
            config->duty_resolution;
        timer_state->frequency_hz =
            config->frequency_hz;
        timer_state->channel_users = 0;
    }

    ledc_channel_config_t channel_config = {
        .gpio_num = config->gpio_num,
        .speed_mode = config->speed_mode,
        .channel = config->channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = config->timer,
        .duty = 0,
        .hpoint = 0,
    };

    err = ledc_channel_config(&channel_config);

    if (err != ESP_OK) {
        if (timer_state->channel_users == 0) {
            timer_state->initialized = false;
            timer_state->frequency_hz = 0;
        }

        return err;
    }

    channel_state->initialized = true;
    channel_state->enabled = false;
    channel_state->speed_mode =
        config->speed_mode;
    channel_state->channel =
        config->channel;
    channel_state->timer =
        config->timer;
    channel_state->gpio_num =
        config->gpio_num;
    channel_state->duty_resolution =
        config->duty_resolution;
    channel_state->frequency_hz =
        config->frequency_hz;
    channel_state->duty_percent = 0.0f;

    timer_state->channel_users++;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Deinitialization                                                           */
/* -------------------------------------------------------------------------- */

esp_err_t hal_pwm_deinit(
    ledc_channel_t channel)
{
    hal_pwm_channel_state_t *channel_state =
        find_channel(channel);

    if (channel_state == NULL) {
        return ESP_OK;
    }

    esp_err_t err = ledc_stop(
        channel_state->speed_mode,
        channel_state->channel,
        0);

    if (err != ESP_OK) {
        return err;
    }

    hal_pwm_timer_state_t *timer_state =
        get_timer_state(
            channel_state->speed_mode,
            channel_state->timer);

    if (timer_state != NULL &&
        timer_state->channel_users > 0) {

        timer_state->channel_users--;

        if (timer_state->channel_users == 0) {
            timer_state->initialized = false;
            timer_state->frequency_hz = 0;
            timer_state->channel_users = 0;
        }
    }

    channel_state->initialized = false;
    channel_state->enabled = false;
    channel_state->duty_percent = 0.0f;
    channel_state->gpio_num = -1;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Duty control                                                               */
/* -------------------------------------------------------------------------- */

esp_err_t hal_pwm_set_duty(
    ledc_channel_t channel,
    float duty_percent)
{
    if (!valid_channel(channel)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (duty_percent < 0.0f ||
        duty_percent > 100.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    hal_pwm_channel_state_t *state =
        find_channel(channel);

    if (state == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    state->duty_percent = duty_percent;

    if (!state->enabled) {
        return ESP_OK;
    }

    uint32_t duty = duty_from_percent(
        state->duty_resolution,
        duty_percent);

    return ledc_set_duty_and_update(
        state->speed_mode,
        state->channel,
        duty,
        0);
}


/* -------------------------------------------------------------------------- */
/* Duty readback                                                              */
/* -------------------------------------------------------------------------- */

esp_err_t hal_pwm_get_duty(
    ledc_channel_t channel,
    float *duty_percent)
{
    if (duty_percent == NULL ||
        !valid_channel(channel)) {
        return ESP_ERR_INVALID_ARG;
    }

    hal_pwm_channel_state_t *state =
        find_channel(channel);

    if (state == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    *duty_percent = state->duty_percent;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Enable                                                                     */
/* -------------------------------------------------------------------------- */

esp_err_t hal_pwm_enable(
    ledc_channel_t channel)
{
    if (!valid_channel(channel)) {
        return ESP_ERR_INVALID_ARG;
    }

    hal_pwm_channel_state_t *state =
        find_channel(channel);

    if (state == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    uint32_t duty = duty_from_percent(
        state->duty_resolution,
        state->duty_percent);

    esp_err_t err = ledc_set_duty_and_update(
        state->speed_mode,
        state->channel,
        duty,
        0);

    if (err != ESP_OK) {
        return err;
    }

    state->enabled = true;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Disable                                                                    */
/* -------------------------------------------------------------------------- */

esp_err_t hal_pwm_disable(
    ledc_channel_t channel)
{
    if (!valid_channel(channel)) {
        return ESP_ERR_INVALID_ARG;
    }

    hal_pwm_channel_state_t *state =
        find_channel(channel);

    if (state == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = ledc_set_duty_and_update(
        state->speed_mode,
        state->channel,
        0,
        0);

    if (err != ESP_OK) {
        return err;
    }

    state->enabled = false;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* State                                                                      */
/* -------------------------------------------------------------------------- */

bool hal_pwm_is_initialized(
    ledc_channel_t channel)
{
    if (!valid_channel(channel)) {
        return false;
    }

    return find_channel(channel) != NULL;
}