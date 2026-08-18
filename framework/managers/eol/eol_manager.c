#include "eol_manager.h"

#include <inttypes.h>
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
/* Internal state                                                             */
/* -------------------------------------------------------------------------- */

static bool s_initialized = false;

static bool s_requested = false;

static eol_manager_config_t s_config = {
    .overall_timeout_ms = 300000U,
    .test_timeout_ms = 10000U,
    .motor_test_runtime_ms = 2000U,
    .motor_min_turns = 0.05f,
    .motor_max_current_a = 6.0f
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

    /*
     * A successful timestamp read verifies that the PCF8563
     * is readable through the complete RTC manager path.
     */
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

    if (config->motor_max_current_a <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
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

        /*
         * These tests intentionally remain disconnected.
         *
         * They will be implemented only after their hardware
         * ownership and safety conditions are established.
         */
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