#include "eol_manager.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"


/* -------------------------------------------------------------------------- */
/* Module                                                                     */
/* -------------------------------------------------------------------------- */

static const char *TAG = "EOL";


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

    /*
     * SKIP is allowed for tests which are not populated or
     * cannot safely be performed at the current production
     * stage.
     */
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

    ESP_LOGI(
        TAG,
        "Config: overall_timeout=%" PRIu32
        " ms test_timeout=%" PRIu32
        " ms motor_runtime=%" PRIu32
        " ms min_turns=%.4f max_current=%.3f A",
        s_config.overall_timeout_ms,
        s_config.test_timeout_ms,
        s_config.motor_test_runtime_ms,
        (double)s_config.motor_min_turns,
        (double)s_config.motor_max_current_a);

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

    /*
     * Hardware-specific actuator shutdown will be connected
     * here during the actuator EOL integration stage.
     */
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

    /*
     * Hardware test adapters are intentionally connected in
     * the next EOL implementation stage.
     */
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
        "test adapter not connected",
        0.0f);

    ESP_LOGI(
        TAG,
        "EOL test requested: %s",
        test_name(test_id));

    return ESP_OK;
}


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
     * No direct GPIO manipulation is performed here yet.
     *
     * The actuator manager will become the owner of actual
     * motor shutdown when the EOL motor tests are integrated.
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