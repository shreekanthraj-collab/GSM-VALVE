#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif


/* -------------------------------------------------------------------------- */
/* EOL test identifiers                                                       */
/* -------------------------------------------------------------------------- */

typedef enum
{
    EOL_TEST_I2C_SCAN = 0,

    EOL_TEST_RTC,

    EOL_TEST_ENCODER,

    EOL_TEST_INA226,

    EOL_TEST_BATTERY,

    EOL_TEST_LIMIT_SWITCH,

    EOL_TEST_BUZZER,

    EOL_TEST_MODEM,

    EOL_TEST_MOTOR_FORWARD,

    EOL_TEST_MOTOR_REVERSE,

    EOL_TEST_COUNT

} eol_test_id_t;


/* -------------------------------------------------------------------------- */
/* EOL test result                                                            */
/* -------------------------------------------------------------------------- */

typedef enum
{
    EOL_RESULT_NOT_RUN = 0,

    EOL_RESULT_RUNNING,

    EOL_RESULT_PASS,

    EOL_RESULT_FAIL,

    EOL_RESULT_SKIP

} eol_test_result_t;


/* -------------------------------------------------------------------------- */
/* Overall EOL state                                                          */
/* -------------------------------------------------------------------------- */

typedef enum
{
    EOL_STATE_IDLE = 0,

    EOL_STATE_RUNNING,

    EOL_STATE_COMPLETE,

    EOL_STATE_ABORTED

} eol_state_t;


/* -------------------------------------------------------------------------- */
/* Overall EOL result                                                         */
/* -------------------------------------------------------------------------- */

typedef enum
{
    EOL_OVERALL_NOT_RUN = 0,

    EOL_OVERALL_PASS,

    EOL_OVERALL_FAIL

} eol_overall_result_t;


/* -------------------------------------------------------------------------- */
/* Individual test information                                                */
/* -------------------------------------------------------------------------- */

typedef struct
{
    eol_test_id_t id;

    eol_test_result_t result;

    /*
     * Short diagnostic text.
     *
     * This is intended for serial logging and later
     * local EOL reporting.
     */
    char message[96];

    /*
     * Optional numeric diagnostic value.
     *
     * Examples:
     *
     *   battery voltage
     *   encoder angle
     *   INA226 current
     *   modem signal value
     */
    float value;

} eol_test_result_info_t;


/* -------------------------------------------------------------------------- */
/* EOL status                                                                 */
/* -------------------------------------------------------------------------- */

typedef struct
{
    eol_state_t state;

    eol_overall_result_t overall_result;

    uint32_t started_ms;

    uint32_t completed_ms;

    uint32_t tests_passed;

    uint32_t tests_failed;

    uint32_t tests_skipped;

    uint32_t tests_not_run;

    eol_test_result_info_t tests[EOL_TEST_COUNT];

} eol_status_t;


/* -------------------------------------------------------------------------- */
/* EOL configuration                                                          */
/* -------------------------------------------------------------------------- */

typedef struct
{
    /*
     * Maximum time allowed for a complete EOL run.
     *
     * This is a safety limit for the EOL state machine.
     */
    uint32_t overall_timeout_ms;

    /*
     * Maximum time allowed for an individual test.
     */
    uint32_t test_timeout_ms;

    /*
     * Motor test duration.
     *
     * Motor movement is NEVER started automatically merely
     * because EOL mode was entered.
     */
    uint32_t motor_test_runtime_ms;

    /*
     * Minimum encoder movement expected during a motor test.
     */
    float motor_min_turns;

    /*
     * Maximum allowed motor current during EOL motor testing.
     */
    float motor_max_current_a;

} eol_manager_config_t;


/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

/*
 * Initialize the EOL manager.
 *
 * EOL manager initialization does not start any motor movement.
 */
esp_err_t eol_manager_init(
    const eol_manager_config_t *config);


/* -------------------------------------------------------------------------- */
/* EOL entry detection                                                        */
/* -------------------------------------------------------------------------- */

/*
 * Configure whether EOL mode has been requested.
 *
 * The actual physical EOL entry mechanism belongs to the board
 * adaptation layer.
 */
esp_err_t eol_manager_set_requested(
    bool requested);


/*
 * Return whether EOL mode was requested.
 */
bool eol_manager_is_requested(void);


/* -------------------------------------------------------------------------- */
/* EOL execution                                                              */
/* -------------------------------------------------------------------------- */

/*
 * Start the complete automatic EOL sequence.
 *
 * Safety:
 *
 *   - motor movement is NOT started unless the motor tests
 *     explicitly reach their execution stage.
 *   - the caller must place the actuator in a safe mechanical
 *     test condition before allowing motor tests.
 */
esp_err_t eol_manager_start(void);


/*
 * Abort an active EOL sequence.
 *
 * Any active motor operation must be stopped before returning.
 */
esp_err_t eol_manager_abort(void);


/*
 * Execute one EOL state-machine step.
 *
 * This is intentionally non-blocking at the manager level.
 */
esp_err_t eol_manager_process(
    uint32_t now_ms);


/* -------------------------------------------------------------------------- */
/* Individual tests                                                           */
/* -------------------------------------------------------------------------- */

/*
 * Run one individual EOL test.
 *
 * Motor tests require explicit authorization through the
 * corresponding manager implementation.
 */
esp_err_t eol_manager_run_test(
    eol_test_id_t test_id);


/*
 * Get the result of one test.
 */
esp_err_t eol_manager_get_test_result(
    eol_test_id_t test_id,
    eol_test_result_info_t *result);


/* -------------------------------------------------------------------------- */
/* Status                                                                     */
/* -------------------------------------------------------------------------- */

esp_err_t eol_manager_get_status(
    eol_status_t *status);

eol_state_t eol_manager_get_state(void);

eol_overall_result_t eol_manager_get_overall_result(void);

bool eol_manager_is_initialized(void);

bool eol_manager_is_active(void);

bool eol_manager_is_complete(void);


/* -------------------------------------------------------------------------- */
/* Safe-state control                                                         */
/* -------------------------------------------------------------------------- */

/*
 * Force EOL into a safe state.
 *
 * This must stop/disable actuator activity before EOL exits.
 */
esp_err_t eol_manager_force_safe_state(void);


/* -------------------------------------------------------------------------- */
/* EOL summary                                                                */
/* -------------------------------------------------------------------------- */

/*
 * Print a complete EOL summary to the ESP-IDF log.
 */
esp_err_t eol_manager_log_summary(void);


#ifdef __cplusplus
}
#endif