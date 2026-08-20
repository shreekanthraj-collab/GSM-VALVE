#include "eol_manager.h"

#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"

#include "hal_i2c.h"
#include "hal_rtc.h"

#include "drv_as5600.h"
#include "drv_ina226.h"

#include "rtc_manager.h"
#include "battery_manager.h"


/* -------------------------------------------------------------------------- */
/* Module                                                                     */
/* -------------------------------------------------------------------------- */

static const char *TAG = "EOL";


/* -------------------------------------------------------------------------- */
/* GSM-VALVE hardware                                                         */
/* -------------------------------------------------------------------------- */

#define EOL_I2C_AS5600_ADDRESS       0x36U
#define EOL_I2C_INA226_ADDRESS       0x40U
#define EOL_I2C_RTC_ADDRESS          0x51U

#define EOL_INA226_EXPECTED_MFG_ID   0x5449U

#define EOL_ENCODER_MAX_ANGLE        4095U

#define EOL_BATTERY_MIN_VALID_V      0.0f
#define EOL_BATTERY_MAX_VALID_V      20.0f


/* -------------------------------------------------------------------------- */
/* Factory default configuration                                              */
/* -------------------------------------------------------------------------- */

/*
 * These are factory fallback values only.
 *
 * They are NOT the firmware absolute safety limits.
 *
 * During EOL, these values may be replaced by the technician's
 * production configuration after validation.
 *
 * After EOL, the accepted configuration is persisted by the
 * EOL persistence manager.
 */
#define EOL_DEFAULT_VOLTAGE_WARN_LOW_V          12.0f
#define EOL_DEFAULT_VOLTAGE_WARN_HIGH_V         12.4f
#define EOL_DEFAULT_VOLTAGE_CUTOFF_V            11.6f
#define EOL_DEFAULT_VOLTAGE_CRITICAL_V          11.2f
#define EOL_DEFAULT_VOLTAGE_RESET_V             12.0f
#define EOL_DEFAULT_VOLTAGE_BYPASS_TIMEOUT_MS   120000U

#define EOL_DEFAULT_CURRENT_MIN_SAFE_A          1.0f
#define EOL_DEFAULT_CURRENT_MAX_SAFE_A          5.0f


/* -------------------------------------------------------------------------- */
/* Internal state                                                             */
/* -------------------------------------------------------------------------- */

static bool s_initialized = false;

static bool s_requested = false;

static eol_manager_config_t s_config = {
    .overall_timeout_ms = 300000U,
    .test_timeout_ms = 10000U,
    .motor_test_runtime_ms = 2000U,
    .motor_min_turns = 0.05f,
    .motor_max_current_a = 5.0f,

    .factory_config = {
        .voltage_warn_low_v =
            EOL_DEFAULT_VOLTAGE_WARN_LOW_V,

        .voltage_warn_high_v =
            EOL_DEFAULT_VOLTAGE_WARN_HIGH_V,

        .voltage_cutoff_v =
            EOL_DEFAULT_VOLTAGE_CUTOFF_V,

        .voltage_critical_v =
            EOL_DEFAULT_VOLTAGE_CRITICAL_V,

        .voltage_reset_v =
            EOL_DEFAULT_VOLTAGE_RESET_V,

        .voltage_bypass_timeout_ms =
            EOL_DEFAULT_VOLTAGE_BYPASS_TIMEOUT_MS,

        .current_min_safe_a =
            EOL_DEFAULT_CURRENT_MIN_SAFE_A,

        .current_max_safe_a =
            EOL_DEFAULT_CURRENT_MAX_SAFE_A
    }
};

static eol_status_t s_status;


/* -------------------------------------------------------------------------- */
/* Internal helpers                                                           */
/* -------------------------------------------------------------------------- */

static bool valid_test_id(
    eol_test_id_t test_id)
{
    return
        test_id >= EOL_TEST_I2C_SCAN &&
        test_id < EOL_TEST_COUNT;
}


static const char *test_name(
    eol_test_id_t test_id)
{
    switch (test_id) {

        case EOL_TEST_I2C_SCAN:
            return "I2C_SCAN";

        case EOL_TEST_RTC:
            return "RTC";

        case EOL_TEST_ENCODER:
            return "ENCODER";

        case EOL_TEST_INA226:
            return "INA226";

        case EOL_TEST_BATTERY:
            return "BATTERY";

        case EOL_TEST_LIMIT_SWITCH:
            return "LIMIT_SWITCH";

        case EOL_TEST_BUZZER:
            return "BUZZER";

        case EOL_TEST_MODEM:
            return "MODEM";

        case EOL_TEST_MOTOR_FORWARD:
            return "MOTOR_FORWARD";

        case EOL_TEST_MOTOR_REVERSE:
            return "MOTOR_REVERSE";

        default:
            return "UNKNOWN";
    }
}


static void clear_result(
    eol_test_result_info_t *result,
    eol_test_id_t test_id)
{
    if (result == NULL) {
        return;
    }

    memset(
        result,
        0,
        sizeof(*result));

    result->id =
        test_id;

    result->result =
        EOL_RESULT_NOT_RUN;
}


static void set_result(
    eol_test_id_t test_id,
    eol_test_result_t result,
    const char *message,
    float value)
{
    if (!valid_test_id(test_id)) {
        return;
    }

    eol_test_result_info_t *info =
        &s_status.tests[test_id];

    info->id =
        test_id;

    info->result =
        result;

    info->value =
        value;

    if (message != NULL) {

        (void)snprintf(
            info->message,
            sizeof(info->message),
            "%s",
            message);
    }
    else {

        info->message[0] =
            '\0';
    }
}


static void recalculate_counts(void)
{
    s_status.tests_passed =
        0U;

    s_status.tests_failed =
        0U;

    s_status.tests_skipped =
        0U;

    s_status.tests_not_run =
        0U;

    for (int i = 0;
         i < EOL_TEST_COUNT;
         ++i) {

        switch (
            s_status.tests[i].result) {

            case EOL_RESULT_PASS:
                s_status.tests_passed++;
                break;

            case EOL_RESULT_FAIL:
                s_status.tests_failed++;
                break;

            case EOL_RESULT_SKIP:
                s_status.tests_skipped++;
                break;

            case EOL_RESULT_NOT_RUN:
            case EOL_RESULT_RUNNING:
            default:
                s_status.tests_not_run++;
                break;
        }
    }
}


static void calculate_overall_result(void)
{
    recalculate_counts();

    if (s_status.tests_failed > 0U) {

        s_status.overall_result =
            EOL_OVERALL_FAIL;

        return;
    }

    if (s_status.state ==
        EOL_STATE_COMPLETE) {

        s_status.overall_result =
            EOL_OVERALL_PASS;

        return;
    }

    s_status.overall_result =
        EOL_OVERALL_NOT_RUN;
}


static bool elapsed_timeout(
    uint32_t now_ms,
    uint32_t start_ms,
    uint32_t timeout_ms)
{
    return
        (uint32_t)(now_ms - start_ms) >=
        timeout_ms;
}


static void reset_status(void)
{
    memset(
        &s_status,
        0,
        sizeof(s_status));

    s_status.state =
        EOL_STATE_IDLE;

    s_status.overall_result =
        EOL_OVERALL_NOT_RUN;

    for (int i = 0;
         i < EOL_TEST_COUNT;
         ++i) {

        clear_result(
            &s_status.tests[i],
            (eol_test_id_t)i);
    }
}


/* -------------------------------------------------------------------------- */
/* Factory configuration                                                      */
/* -------------------------------------------------------------------------- */

/*
 * Validate the factory operating configuration against the firmware's
 * absolute safety boundaries.
 *
 * IMPORTANT:
 *
 * This function does not write NVS.
 *
 * It is deliberately usable by both:
 *
 *     1. EOL/factory commissioning
 *     2. AWS/field configuration
 *
 * Therefore there is one common safety gate for configuration changes.
 */
esp_err_t eol_manager_validate_factory_config(
    const eol_factory_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }


    /* ---------------------------------------------------------------------- */
    /* Reject non-finite floating point values                               */
    /* ---------------------------------------------------------------------- */

    if (!isfinite(config->voltage_warn_low_v) ||
        !isfinite(config->voltage_warn_high_v) ||
        !isfinite(config->voltage_cutoff_v) ||
        !isfinite(config->voltage_critical_v) ||
        !isfinite(config->voltage_reset_v) ||
        !isfinite(config->current_min_safe_a) ||
        !isfinite(config->current_max_safe_a)) {

        ESP_LOGE(
            TAG,
            "Factory config rejected: non-finite value");

        return ESP_ERR_INVALID_ARG;
    }


    /* ---------------------------------------------------------------------- */
    /* Voltage absolute safety boundaries                                    */
    /* ---------------------------------------------------------------------- */

    if (config->voltage_warn_low_v <
            EOL_ABS_VOLTAGE_WARN_LOW_MIN_V ||
        config->voltage_warn_low_v >
            EOL_ABS_VOLTAGE_WARN_LOW_MAX_V) {

        ESP_LOGE(
            TAG,
            "Factory config rejected: warn-low %.3f V outside "
            "absolute limits",
            (double)config->voltage_warn_low_v);

        return ESP_ERR_INVALID_ARG;
    }


    if (config->voltage_warn_high_v <
            EOL_ABS_VOLTAGE_WARN_HIGH_MIN_V ||
        config->voltage_warn_high_v >
            EOL_ABS_VOLTAGE_WARN_HIGH_MAX_V) {

        ESP_LOGE(
            TAG,
            "Factory config rejected: warn-high %.3f V outside "
            "absolute limits",
            (double)config->voltage_warn_high_v);

        return ESP_ERR_INVALID_ARG;
    }


    if (config->voltage_cutoff_v <
            EOL_ABS_VOLTAGE_CUTOFF_MIN_V ||
        config->voltage_cutoff_v >
            EOL_ABS_VOLTAGE_CUTOFF_MAX_V) {

        ESP_LOGE(
            TAG,
            "Factory config rejected: cutoff %.3f V outside "
            "absolute limits",
            (double)config->voltage_cutoff_v);

        return ESP_ERR_INVALID_ARG;
    }


    if (config->voltage_critical_v <
            EOL_ABS_VOLTAGE_CRITICAL_MIN_V ||
        config->voltage_critical_v >
            EOL_ABS_VOLTAGE_CRITICAL_MAX_V) {

        ESP_LOGE(
            TAG,
            "Factory config rejected: critical %.3f V outside "
            "absolute limits",
            (double)config->voltage_critical_v);

        return ESP_ERR_INVALID_ARG;
    }


    if (config->voltage_reset_v <
            EOL_ABS_VOLTAGE_RESET_MIN_V ||
        config->voltage_reset_v >
            EOL_ABS_VOLTAGE_RESET_MAX_V) {

        ESP_LOGE(
            TAG,
            "Factory config rejected: reset %.3f V outside "
            "absolute limits",
            (double)config->voltage_reset_v);

        return ESP_ERR_INVALID_ARG;
    }


    /* ---------------------------------------------------------------------- */
    /* Voltage relationship validation                                        */
    /* ---------------------------------------------------------------------- */

    /*
     * Expected battery hierarchy:
     *
     *     WARN_HIGH
     *        >
     *     RESET
     *        >
     *     WARN_LOW
     *        >
     *     CUTOFF
     *        >
     *     CRITICAL
     *
     * This prevents a configuration that is numerically inside the
     * absolute boundaries but logically unsafe.
     */

    if (!(config->voltage_warn_high_v >
          config->voltage_reset_v)) {

        ESP_LOGE(
            TAG,
            "Factory config rejected: "
            "warn-high must be greater than reset");

        return ESP_ERR_INVALID_ARG;
    }


    if (!(config->voltage_reset_v >
          config->voltage_warn_low_v)) {

        ESP_LOGE(
            TAG,
            "Factory config rejected: "
            "reset must be greater than warn-low");

        return ESP_ERR_INVALID_ARG;
    }


    if (!(config->voltage_warn_low_v >
          config->voltage_cutoff_v)) {

        ESP_LOGE(
            TAG,
            "Factory config rejected: "
            "warn-low must be greater than cutoff");

        return ESP_ERR_INVALID_ARG;
    }


    if (!(config->voltage_cutoff_v >
          config->voltage_critical_v)) {

        ESP_LOGE(
            TAG,
            "Factory config rejected: "
            "cutoff must be greater than critical");

        return ESP_ERR_INVALID_ARG;
    }


    /* ---------------------------------------------------------------------- */
    /* Bypass timeout                                                         */
    /* ---------------------------------------------------------------------- */

    if (config->voltage_bypass_timeout_ms == 0U) {

        ESP_LOGE(
            TAG,
            "Factory config rejected: bypass timeout is zero");

        return ESP_ERR_INVALID_ARG;
    }


    /* ---------------------------------------------------------------------- */
    /* Current absolute safety boundaries                                    */
    /* ---------------------------------------------------------------------- */

    if (config->current_min_safe_a <
            EOL_ABS_CURRENT_MIN_SAFE_MIN_A ||
        config->current_min_safe_a >
            EOL_ABS_CURRENT_MIN_SAFE_MAX_A) {

        ESP_LOGE(
            TAG,
            "Factory config rejected: minimum current %.3f A "
            "outside absolute limits",
            (double)config->current_min_safe_a);

        return ESP_ERR_INVALID_ARG;
    }


    if (config->current_max_safe_a <
            EOL_ABS_CURRENT_MAX_SAFE_MIN_A ||
        config->current_max_safe_a >
            EOL_ABS_CURRENT_MAX_SAFE_MAX_A) {

        ESP_LOGE(
            TAG,
            "Factory config rejected: maximum current %.3f A "
            "outside absolute limits",
            (double)config->current_max_safe_a);

        return ESP_ERR_INVALID_ARG;
    }


    /* ---------------------------------------------------------------------- */
    /* Current relationship                                                   */
    /* ---------------------------------------------------------------------- */

    if (!(config->current_max_safe_a >=
          config->current_min_safe_a)) {

        ESP_LOGE(
            TAG,
            "Factory config rejected: "
            "maximum current %.3f A is below minimum %.3f A",
            (double)config->current_max_safe_a,
            (double)config->current_min_safe_a);

        return ESP_ERR_INVALID_ARG;
    }


    ESP_LOGI(
        TAG,
        "Factory configuration validated: "
        "VwarnLow=%.3f "
        "VwarnHigh=%.3f "
        "Vcut=%.3f "
        "Vcritical=%.3f "
        "Vreset=%.3f "
        "bypass=%" PRIu32 " ms "
        "Imin=%.3f A "
        "Imax=%.3f A",
        (double)config->voltage_warn_low_v,
        (double)config->voltage_warn_high_v,
        (double)config->voltage_cutoff_v,
        (double)config->voltage_critical_v,
        (double)config->voltage_reset_v,
        config->voltage_bypass_timeout_ms,
        (double)config->current_min_safe_a,
        (double)config->current_max_safe_a);

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Factory configuration get                                                  */
/* -------------------------------------------------------------------------- */

esp_err_t eol_manager_get_factory_config(
    eol_factory_config_t *config)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *config =
        s_config.factory_config;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Factory configuration set                                                  */
/* -------------------------------------------------------------------------- */

esp_err_t eol_manager_set_factory_config(
    const eol_factory_config_t *config)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err =
        eol_manager_validate_factory_config(
            config);

    if (err != ESP_OK) {

        ESP_LOGW(
            TAG,
            "Factory configuration rejected");

        return err;
    }

    /*
     * Important:
     *
     * No NVS write happens here.
     *
     * The caller/persistence layer is responsible for persistence.
     */
    s_config.factory_config =
        *config;

    ESP_LOGI(
        TAG,
        "Factory configuration accepted");

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Stage 2 hardware tests                                                     */
/* -------------------------------------------------------------------------- */

static esp_err_t run_i2c_scan_test(void)
{
    bool as5600_ok =
        hal_i2c_probe(
            EOL_I2C_AS5600_ADDRESS) == ESP_OK;

    bool ina226_ok =
        hal_i2c_probe(
            EOL_I2C_INA226_ADDRESS) == ESP_OK;

    bool rtc_ok =
        hal_i2c_probe(
            EOL_I2C_RTC_ADDRESS) == ESP_OK;

    ESP_LOGI(
        TAG,
        "I2C scan: AS5600=0x%02X %s, "
        "INA226=0x%02X %s, "
        "RTC=0x%02X %s",
        EOL_I2C_AS5600_ADDRESS,
        as5600_ok ? "FOUND" : "NOT_FOUND",
        EOL_I2C_INA226_ADDRESS,
        ina226_ok ? "FOUND" : "NOT_FOUND",
        EOL_I2C_RTC_ADDRESS,
        rtc_ok ? "FOUND" : "NOT_FOUND");

    if (!as5600_ok ||
        !ina226_ok ||
        !rtc_ok) {

        set_result(
            EOL_TEST_I2C_SCAN,
            EOL_RESULT_FAIL,
            "one or more required I2C devices missing",
            0.0f);

        return ESP_FAIL;
    }

    set_result(
        EOL_TEST_I2C_SCAN,
        EOL_RESULT_PASS,
        "all required I2C devices found",
        3.0f);

    return ESP_OK;
}


static esp_err_t run_rtc_test(void)
{
    if (!rtc_manager_is_initialized()) {

        set_result(
            EOL_TEST_RTC,
            EOL_RESULT_FAIL,
            "RTC manager not initialized",
            0.0f);

        return ESP_ERR_INVALID_STATE;
    }

    rtc_manager_state_t state = {0};

    esp_err_t err =
        rtc_manager_get_state(
            &state);

    if (err != ESP_OK) {

        set_result(
            EOL_TEST_RTC,
            EOL_RESULT_FAIL,
            "RTC state read failed",
            0.0f);

        return err;
    }

    if (!state.valid) {

        set_result(
            EOL_TEST_RTC,
            EOL_RESULT_FAIL,
            "RTC time invalid",
            0.0f);

        return ESP_FAIL;
    }

    uint32_t timestamp = 0U;

    err =
        rtc_manager_get_timestamp(
            &timestamp);

    if (err != ESP_OK) {

        set_result(
            EOL_TEST_RTC,
            EOL_RESULT_FAIL,
            "RTC timestamp read failed",
            0.0f);

        return err;
    }

    if (timestamp == 0U) {

        set_result(
            EOL_TEST_RTC,
            EOL_RESULT_FAIL,
            "RTC timestamp is zero",
            0.0f);

        return ESP_FAIL;
    }

    ESP_LOGI(
        TAG,
        "RTC test PASS: timestamp=%" PRIu32,
        timestamp);

    set_result(
        EOL_TEST_RTC,
        EOL_RESULT_PASS,
        "RTC read and timestamp valid",
        (float)timestamp);

    return ESP_OK;
}


static esp_err_t run_encoder_test(void)
{
    if (!drv_as5600_is_initialized()) {

        set_result(
            EOL_TEST_ENCODER,
            EOL_RESULT_FAIL,
            "AS5600 driver not initialized",
            0.0f);

        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err =
        drv_as5600_probe();

    if (err != ESP_OK) {

        set_result(
            EOL_TEST_ENCODER,
            EOL_RESULT_FAIL,
            "AS5600 probe failed",
            0.0f);

        return err;
    }

    drv_as5600_reading_t reading = {0};

    err =
        drv_as5600_read(
            &reading);

    if (err != ESP_OK) {

        set_result(
            EOL_TEST_ENCODER,
            EOL_RESULT_FAIL,
            "AS5600 angle read failed",
            0.0f);

        return err;
    }

    if (reading.raw_angle >
            EOL_ENCODER_MAX_ANGLE ||
        reading.angle >
            EOL_ENCODER_MAX_ANGLE) {

        set_result(
            EOL_TEST_ENCODER,
            EOL_RESULT_FAIL,
            "AS5600 angle outside valid range",
            (float)reading.angle);

        return ESP_FAIL;
    }

    ESP_LOGI(
        TAG,
        "AS5600 test PASS: raw=%u angle=%u",
        (unsigned)reading.raw_angle,
        (unsigned)reading.angle);

    set_result(
        EOL_TEST_ENCODER,
        EOL_RESULT_PASS,
        "AS5600 probe and angle valid",
        (float)reading.angle);

    return ESP_OK;
}


static esp_err_t run_ina226_test(void)
{
    if (!drv_ina226_is_initialized()) {

        set_result(
            EOL_TEST_INA226,
            EOL_RESULT_FAIL,
            "INA226 driver not initialized",
            0.0f);

        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err =
        drv_ina226_probe();

    if (err != ESP_OK) {

        set_result(
            EOL_TEST_INA226,
            EOL_RESULT_FAIL,
            "INA226 probe failed",
            0.0f);

        return err;
    }

    uint16_t manufacturer_id = 0U;

    err =
        drv_ina226_read_manufacturer_id(
            &manufacturer_id);

    if (err != ESP_OK) {

        set_result(
            EOL_TEST_INA226,
            EOL_RESULT_FAIL,
            "INA226 manufacturer ID read failed",
            0.0f);

        return err;
    }

    if (manufacturer_id !=
        EOL_INA226_EXPECTED_MFG_ID) {

        ESP_LOGE(
            TAG,
            "INA226 manufacturer ID mismatch: "
            "expected=0x%04X actual=0x%04X",
            EOL_INA226_EXPECTED_MFG_ID,
            manufacturer_id);

        set_result(
            EOL_TEST_INA226,
            EOL_RESULT_FAIL,
            "INA226 manufacturer ID mismatch",
            (float)manufacturer_id);

        return ESP_FAIL;
    }

    drv_ina226_reading_t reading = {0};

    err =
        drv_ina226_read(
            &reading);

    if (err != ESP_OK) {

        set_result(
            EOL_TEST_INA226,
            EOL_RESULT_FAIL,
            "INA226 measurement read failed",
            0.0f);

        return err;
    }

    ESP_LOGI(
        TAG,
        "INA226 test PASS: "
        "MFG=0x%04X "
        "V=%.3f V "
        "I=%.3f A "
        "P=%.3f W",
        manufacturer_id,
        (double)reading.bus_voltage_v,
        (double)reading.current_a,
        (double)reading.power_w);

    set_result(
        EOL_TEST_INA226,
        EOL_RESULT_PASS,
        "INA226 identity and measurement valid",
        reading.bus_voltage_v);

    return ESP_OK;
}


static esp_err_t run_battery_test(void)
{
    if (!battery_manager_is_initialized()) {

        set_result(
            EOL_TEST_BATTERY,
            EOL_RESULT_FAIL,
            "battery manager not initialized",
            0.0f);

        return ESP_ERR_INVALID_STATE;
    }

    battery_manager_reading_t reading = {0};

    esp_err_t err =
        battery_manager_read(
            &reading);

    if (err != ESP_OK) {

        set_result(
            EOL_TEST_BATTERY,
            EOL_RESULT_FAIL,
            "battery reading failed",
            0.0f);

        return err;
    }

    if (!reading.valid) {

        set_result(
            EOL_TEST_BATTERY,
            EOL_RESULT_FAIL,
            "battery reading invalid",
            reading.battery_voltage_v);

        return ESP_FAIL;
    }

    if (reading.battery_voltage_v <
            EOL_BATTERY_MIN_VALID_V ||
        reading.battery_voltage_v >
            EOL_BATTERY_MAX_VALID_V) {

        set_result(
            EOL_TEST_BATTERY,
            EOL_RESULT_FAIL,
            "battery voltage outside diagnostic range",
            reading.battery_voltage_v);

        return ESP_FAIL;
    }

    ESP_LOGI(
        TAG,
        "Battery test PASS: "
        "ADC=%.3f V "
        "VBAT=%.3f V "
        "state=%d",
        (double)reading.adc_voltage_v,
        (double)reading.battery_voltage_v,
        (int)reading.state);

    set_result(
        EOL_TEST_BATTERY,
        EOL_RESULT_PASS,
        "battery ADC and voltage valid",
        reading.battery_voltage_v);

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

esp_err_t eol_manager_init(
    const eol_manager_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (config->overall_timeout_ms == 0U ||
        config->test_timeout_ms == 0U) {

        return ESP_ERR_INVALID_ARG;
    }

    if (config->motor_test_runtime_ms == 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    if (config->motor_min_turns < 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!isfinite(config->motor_max_current_a) ||
        config->motor_max_current_a <= 0.0f) {

        return ESP_ERR_INVALID_ARG;
    }

    /*
     * Validate the factory operating configuration before the
     * manager becomes active.
     */
    esp_err_t err =
        eol_manager_validate_factory_config(
            &config->factory_config);

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "EOL manager initialization rejected: "
            "invalid factory configuration");

        return err;
    }

    s_config =
        *config;

    reset_status();

    s_requested =
        false;

    s_initialized =
        true;

    ESP_LOGI(
        TAG,
        "EOL manager initialized");

    ESP_LOGI(
        TAG,
        "Factory config: "
        "Imin=%.3f A Imax=%.3f A "
        "Vlow=%.3f V Vhigh=%.3f V "
        "Vcut=%.3f V Vcritical=%.3f V "
        "Vreset=%.3f V bypass=%" PRIu32 " ms",
        (double)s_config.factory_config.current_min_safe_a,
        (double)s_config.factory_config.current_max_safe_a,
        (double)s_config.factory_config.voltage_warn_low_v,
        (double)s_config.factory_config.voltage_warn_high_v,
        (double)s_config.factory_config.voltage_cutoff_v,
        (double)s_config.factory_config.voltage_critical_v,
        (double)s_config.factory_config.voltage_reset_v,
        s_config.factory_config.voltage_bypass_timeout_ms);

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* EOL entry                                                                  */
/* -------------------------------------------------------------------------- */

esp_err_t eol_manager_set_requested(
    bool requested)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    s_requested =
        requested;

    ESP_LOGI(
        TAG,
        "EOL requested=%s",
        requested ? "yes" : "no");

    return ESP_OK;
}


bool eol_manager_is_requested(void)
{
    return
        s_initialized &&
        s_requested;
}


/* -------------------------------------------------------------------------- */
/* EOL execution                                                              */
/* -------------------------------------------------------------------------- */

esp_err_t eol_manager_start(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_status.state ==
        EOL_STATE_RUNNING) {

        return ESP_ERR_INVALID_STATE;
    }

    reset_status();

    s_status.state =
        EOL_STATE_RUNNING;

    s_status.started_ms =
        0U;

    ESP_LOGI(
        TAG,
        "EOL sequence started");

    return ESP_OK;
}


esp_err_t eol_manager_abort(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_status.state !=
        EOL_STATE_RUNNING) {

        return ESP_OK;
    }

    esp_err_t err =
        eol_manager_force_safe_state();

    if (err != ESP_OK) {
        return err;
    }

    s_status.state =
        EOL_STATE_ABORTED;

    s_status.completed_ms =
        0U;

    calculate_overall_result();

    ESP_LOGW(
        TAG,
        "EOL sequence aborted");

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Individual tests                                                           */
/* -------------------------------------------------------------------------- */

esp_err_t eol_manager_run_test(
    eol_test_id_t test_id)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!valid_test_id(test_id)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_status.state !=
        EOL_STATE_RUNNING) {

        return ESP_ERR_INVALID_STATE;
    }

    set_result(
        test_id,
        EOL_RESULT_RUNNING,
        "test running",
        0.0f);

    switch (test_id) {

        case EOL_TEST_I2C_SCAN:
            return run_i2c_scan_test();

        case EOL_TEST_RTC:
            return run_rtc_test();

        case EOL_TEST_ENCODER:
            return run_encoder_test();

        case EOL_TEST_INA226:
            return run_ina226_test();

        case EOL_TEST_BATTERY:
            return run_battery_test();

        case EOL_TEST_LIMIT_SWITCH:

            set_result(
                test_id,
                EOL_RESULT_SKIP,
                "limit switch not configured",
                0.0f);

            return ESP_OK;

        case EOL_TEST_BUZZER:

            set_result(
                test_id,
                EOL_RESULT_SKIP,
                "buzzer not configured",
                0.0f);

            return ESP_OK;

        case EOL_TEST_MODEM:

            set_result(
                test_id,
                EOL_RESULT_SKIP,
                "modem EOL adapter pending",
                0.0f);

            return ESP_OK;

        case EOL_TEST_MOTOR_FORWARD:

            set_result(
                test_id,
                EOL_RESULT_SKIP,
                "motor EOL test pending safety integration",
                0.0f);

            return ESP_OK;

        case EOL_TEST_MOTOR_REVERSE:

            set_result(
                test_id,
                EOL_RESULT_SKIP,
                "motor EOL test pending safety integration",
                0.0f);

            return ESP_OK;

        default:

            set_result(
                test_id,
                EOL_RESULT_FAIL,
                "unknown EOL test",
                0.0f);

            return ESP_ERR_INVALID_ARG;
    }
}


/* -------------------------------------------------------------------------- */
/* EOL processing                                                              */
/* -------------------------------------------------------------------------- */

esp_err_t eol_manager_process(
    uint32_t now_ms)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_status.state !=
        EOL_STATE_RUNNING) {

        return ESP_OK;
    }

    if (s_status.started_ms == 0U) {

        s_status.started_ms =
            now_ms;
    }

    if (elapsed_timeout(
            now_ms,
            s_status.started_ms,
            s_config.overall_timeout_ms)) {

        ESP_LOGE(
            TAG,
            "EOL overall timeout");

        (void)eol_manager_force_safe_state();

        s_status.state =
            EOL_STATE_ABORTED;

        s_status.completed_ms =
            now_ms;

        calculate_overall_result();

        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Test result                                                                */
/* -------------------------------------------------------------------------- */

esp_err_t eol_manager_get_test_result(
    eol_test_id_t test_id,
    eol_test_result_info_t *result)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!valid_test_id(test_id)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (result == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *result =
        s_status.tests[test_id];

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Status                                                                     */
/* -------------------------------------------------------------------------- */

esp_err_t eol_manager_get_status(
    eol_status_t *status)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    recalculate_counts();

    *status =
        s_status;

    return ESP_OK;
}


eol_state_t eol_manager_get_state(void)
{
    if (!s_initialized) {
        return EOL_STATE_IDLE;
    }

    return
        s_status.state;
}


eol_overall_result_t eol_manager_get_overall_result(void)
{
    if (!s_initialized) {
        return EOL_OVERALL_NOT_RUN;
    }

    return
        s_status.overall_result;
}


bool eol_manager_is_initialized(void)
{
    return
        s_initialized;
}


bool eol_manager_is_active(void)
{
    return
        s_initialized &&
        s_status.state ==
            EOL_STATE_RUNNING;
}


bool eol_manager_is_complete(void)
{
    return
        s_initialized &&
        s_status.state ==
            EOL_STATE_COMPLETE;
}


/* -------------------------------------------------------------------------- */
/* Safe state                                                                 */
/* -------------------------------------------------------------------------- */

esp_err_t eol_manager_force_safe_state(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    /*
     * Motor control remains outside the EOL manager until
     * the actuator EOL safety integration stage.
     */
    ESP_LOGI(
        TAG,
        "EOL safe-state requested");

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Summary                                                                    */
/* -------------------------------------------------------------------------- */

esp_err_t eol_manager_log_summary(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    recalculate_counts();

    ESP_LOGI(
        TAG,
        "==================================================");

    ESP_LOGI(
        TAG,
        "EOL SUMMARY");

    ESP_LOGI(
        TAG,
        "State=%d Overall=%d",
        (int)s_status.state,
        (int)s_status.overall_result);

    ESP_LOGI(
        TAG,
        "PASS=%" PRIu32
        " FAIL=%" PRIu32
        " SKIP=%" PRIu32
        " NOT_RUN=%" PRIu32,
        s_status.tests_passed,
        s_status.tests_failed,
        s_status.tests_skipped,
        s_status.tests_not_run);

    for (int i = 0;
         i < EOL_TEST_COUNT;
         ++i) {

        const eol_test_result_info_t *test =
            &s_status.tests[i];

        ESP_LOGI(
            TAG,
            "%-16s result=%d value=%.4f msg=%s",
            test_name(test->id),
            (int)test->result,
            (double)test->value,
            test->message);
    }

    ESP_LOGI(
        TAG,
        "==================================================");

    return ESP_OK;
}