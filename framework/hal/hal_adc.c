#include "hal_adc.h"

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"


#define HAL_ADC_UNIT_COUNT       2
#include "soc/soc_caps.h"

#define HAL_ADC_MAX_CHANNELS     SOC_ADC_CHANNEL_NUM(0)     10


typedef struct {
    bool initialized;

    adc_unit_t unit;
    adc_channel_t channel;

    adc_atten_t attenuation;
    adc_bitwidth_t bitwidth;
} hal_adc_channel_state_t;


typedef struct {
    bool initialized;

    adc_unit_t unit;
    adc_oneshot_unit_handle_t handle;

    uint32_t channel_users;
} hal_adc_unit_state_t;


static hal_adc_channel_state_t
s_channel_state[HAL_ADC_UNIT_COUNT][HAL_ADC_MAX_CHANNELS];


static hal_adc_unit_state_t
s_unit_state[HAL_ADC_UNIT_COUNT];


/* -------------------------------------------------------------------------- */
/* Validation                                                                 */
/* -------------------------------------------------------------------------- */

static bool valid_unit(adc_unit_t unit)
{
    return unit == ADC_UNIT_1 ||
           unit == ADC_UNIT_2;
}


static bool valid_channel(
    adc_unit_t unit,
    adc_channel_t channel)
{
    if (!valid_unit(unit)) {
        return false;
    }

    return channel >= 0 &&
           channel < HAL_ADC_MAX_CHANNELS;
}


static bool valid_attenuation(
    adc_atten_t attenuation)
{
    return attenuation >= ADC_ATTEN_DB_0 &&
           attenuation <= ADC_ATTEN_DB_12;
}


static bool valid_bitwidth(
    adc_bitwidth_t bitwidth)
{
    return bitwidth == ADC_BITWIDTH_DEFAULT ||
           bitwidth == ADC_BITWIDTH_9 ||
           bitwidth == ADC_BITWIDTH_10 ||
           bitwidth == ADC_BITWIDTH_11 ||
           bitwidth == ADC_BITWIDTH_12;
}


static esp_err_t validate_config(
    const hal_adc_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!valid_unit(config->unit)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!valid_channel(
            config->unit,
            config->channel)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!valid_attenuation(
            config->attenuation)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!valid_bitwidth(
            config->bitwidth)) {
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* State helpers                                                              */
/* -------------------------------------------------------------------------- */

static uint32_t unit_index(adc_unit_t unit)
{
    return (uint32_t)(unit - ADC_UNIT_1);
}


static hal_adc_unit_state_t *get_unit_state(
    adc_unit_t unit)
{
    if (!valid_unit(unit)) {
        return NULL;
    }

    return &s_unit_state[unit_index(unit)];
}


static hal_adc_channel_state_t *get_channel_state(
    adc_unit_t unit,
    adc_channel_t channel)
{
    if (!valid_channel(unit, channel)) {
        return NULL;
    }

    return &s_channel_state
        [unit_index(unit)]
        [channel];
}


/* -------------------------------------------------------------------------- */
/* Calibration                                                                */
/* -------------------------------------------------------------------------- */

static esp_err_t create_calibration(
    const hal_adc_channel_state_t *channel_state,
    adc_cali_handle_t *handle)
{
    if (channel_state == NULL ||
        handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *handle = NULL;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED

    adc_cali_curve_fitting_config_t config = {
        .unit_id = channel_state->unit,
        .chan = channel_state->channel,
        .atten = channel_state->attenuation,
        .bitwidth = channel_state->bitwidth,
    };

    return adc_cali_create_scheme_curve_fitting(
        &config,
        handle);

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED

    adc_cali_line_fitting_config_t config = {
        .unit_id = channel_state->unit,
        .atten = channel_state->attenuation,
        .bitwidth = channel_state->bitwidth,
    };

    return adc_cali_create_scheme_line_fitting(
        &config,
        handle);

#else

    return ESP_ERR_NOT_SUPPORTED;

#endif
}


static esp_err_t delete_calibration(
    adc_cali_handle_t handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED

    return adc_cali_delete_scheme_curve_fitting(
        handle);

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED

    return adc_cali_delete_scheme_line_fitting(
        handle);

#else

    return ESP_ERR_NOT_SUPPORTED;

#endif
}


/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

esp_err_t hal_adc_init(
    const hal_adc_config_t *config)
{
    esp_err_t err = validate_config(config);

    if (err != ESP_OK) {
        return err;
    }

    hal_adc_unit_state_t *unit_state =
        get_unit_state(config->unit);

    hal_adc_channel_state_t *channel_state =
        get_channel_state(
            config->unit,
            config->channel);

    if (unit_state == NULL ||
        channel_state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /*
     * One HAL owner per ADC channel.
     */
    if (channel_state->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    /*
     * Create the ADC oneshot unit only once.
     */
    if (!unit_state->initialized) {

        adc_oneshot_unit_init_cfg_t unit_config = {
            .unit_id = config->unit,
            .ulp_mode = ADC_ULP_MODE_DISABLE,
        };

        err = adc_oneshot_new_unit(
            &unit_config,
            &unit_state->handle);

        if (err != ESP_OK) {
            return err;
        }

        unit_state->initialized = true;
        unit_state->unit = config->unit;
        unit_state->channel_users = 0;
    }

    adc_oneshot_chan_cfg_t channel_config = {
        .atten = config->attenuation,
        .bitwidth = config->bitwidth,
    };

    err = adc_oneshot_config_channel(
        unit_state->handle,
        config->channel,
        &channel_config);

    if (err != ESP_OK) {

        /*
         * If this was the first channel, remove the unit
         * created above so HAL state remains synchronized
         * with the ESP-IDF driver.
         */
        if (unit_state->channel_users == 0) {

            esp_err_t cleanup_err =
                adc_oneshot_del_unit(
                    unit_state->handle);

            if (cleanup_err == ESP_OK) {
                unit_state->handle = NULL;
                unit_state->initialized = false;
                unit_state->channel_users = 0;
            }
        }

        return err;
    }

    channel_state->initialized = true;
    channel_state->unit = config->unit;
    channel_state->channel = config->channel;
    channel_state->attenuation =
        config->attenuation;
    channel_state->bitwidth =
        config->bitwidth;

    unit_state->channel_users++;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Deinitialization                                                           */
/* -------------------------------------------------------------------------- */

esp_err_t hal_adc_deinit(
    adc_unit_t unit)
{
    hal_adc_unit_state_t *unit_state =
        get_unit_state(unit);

    if (unit_state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!unit_state->initialized) {
        return ESP_OK;
    }

    /*
     * All owned channels must be released first.
     */
    if (unit_state->channel_users != 0) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err =
        adc_oneshot_del_unit(
            unit_state->handle);

    if (err != ESP_OK) {
        return err;
    }

    unit_state->handle = NULL;
    unit_state->initialized = false;
    unit_state->channel_users = 0;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Raw conversion                                                             */
/* -------------------------------------------------------------------------- */

esp_err_t hal_adc_read_raw(
    adc_unit_t unit,
    adc_channel_t channel,
    int *raw_value)
{
    if (raw_value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    hal_adc_unit_state_t *unit_state =
        get_unit_state(unit);

    hal_adc_channel_state_t *channel_state =
        get_channel_state(unit, channel);

    if (unit_state == NULL ||
        channel_state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!unit_state->initialized ||
        !channel_state->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    return adc_oneshot_read(
        unit_state->handle,
        channel,
        raw_value);
}


/* -------------------------------------------------------------------------- */
/* Calibrated voltage                                                         */
/* -------------------------------------------------------------------------- */

esp_err_t hal_adc_read_mv(
    adc_unit_t unit,
    adc_channel_t channel,
    int *millivolts)
{
    if (millivolts == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    hal_adc_unit_state_t *unit_state =
        get_unit_state(unit);

    hal_adc_channel_state_t *channel_state =
        get_channel_state(unit, channel);

    if (unit_state == NULL ||
        channel_state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!unit_state->initialized ||
        !channel_state->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    int raw_value = 0;

    esp_err_t err =
        adc_oneshot_read(
            unit_state->handle,
            channel,
            &raw_value);

    if (err != ESP_OK) {
        return err;
    }

    adc_cali_handle_t cali_handle = NULL;

    err = create_calibration(
        channel_state,
        &cali_handle);

    if (err != ESP_OK) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    err = adc_cali_raw_to_voltage(
        cali_handle,
        raw_value,
        millivolts);

    esp_err_t delete_err =
        delete_calibration(cali_handle);

    if (err != ESP_OK) {
        return err;
    }

    return delete_err;
}


/* -------------------------------------------------------------------------- */
/* State                                                                      */
/* -------------------------------------------------------------------------- */

bool hal_adc_is_initialized(
    adc_unit_t unit,
    adc_channel_t channel)
{
    hal_adc_channel_state_t *channel_state =
        get_channel_state(unit, channel);

    if (channel_state == NULL) {
        return false;
    }

    return channel_state->initialized;
}