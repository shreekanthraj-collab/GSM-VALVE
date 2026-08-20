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
/* EOL factory operating configuration                                        */
/* -------------------------------------------------------------------------- */

/*
 * These values represent the production operating configuration
 * established during EOL/factory commissioning.
 *
 * They are NOT the absolute firmware safety limits.
 *
 * EOL values are validated against the firmware absolute limits
 * before being accepted as production configuration.
 *
 * After EOL, AWS may request changes to these values.
 *
 * The ESP32 must validate every AWS value before accepting it.
 */
typedef struct
{
    /* ---------------------------------------------------------------------- */
    /* Battery / voltage configuration                                        */
    /* ---------------------------------------------------------------------- */

    /*
     * Warning low battery voltage.
     */
    float voltage_warn_low_v;

    /*
     * Warning high battery voltage.
     */
    float voltage_warn_high_v;

    /*
     * Battery voltage at which motor operation requires bypass handling.
     */
    float voltage_cutoff_v;

    /*
     * Critical battery voltage.
     */
    float voltage_critical_v;

    /*
     * Voltage required before normal operation may resume.
     */
    float voltage_reset_v;

    /*
     * Maximum time allowed for the battery bypass acknowledgement.
     */
    uint32_t voltage_bypass_timeout_ms;


    /* ---------------------------------------------------------------------- */
    /* Motor / over-current configuration                                     */
    /* ---------------------------------------------------------------------- */

    /*
     * Minimum safe operating current.
     *
     * This is the starting current for the OC protection/escalation
     * algorithm.
     */
    float current_min_safe_a;

    /*
     * Maximum safe operating current.
     *
     * The OC escalation algorithm must NEVER exceed this value.
     */
    float current_max_safe_a;

} eol_factory_config_t;


/* -------------------------------------------------------------------------- */
/* Firmware absolute safety limits                                            */
/* -------------------------------------------------------------------------- */

/*
 * These limits are compiled into firmware.
 *
 * They represent the absolute safety boundary.
 *
 * Neither EOL nor AWS configuration may exceed these limits.
 *
 * Example:
 *
 *     firmware max current = 5.0 A
 *
 *     EOL 4.0 A -> ACCEPT
 *     AWS 4.5 A -> ACCEPT
 *     AWS 5.0 A -> ACCEPT
 *     AWS 5.5 A -> REJECT
 *
 * The ESP32 performs the validation.
 */


/* Voltage absolute limits */

#define EOL_ABS_VOLTAGE_WARN_LOW_MIN_V       0.0f
#define EOL_ABS_VOLTAGE_WARN_LOW_MAX_V       20.0f

#define EOL_ABS_VOLTAGE_WARN_HIGH_MIN_V      0.0f
#define EOL_ABS_VOLTAGE_WARN_HIGH_MAX_V      20.0f

#define EOL_ABS_VOLTAGE_CUTOFF_MIN_V         0.0f
#define EOL_ABS_VOLTAGE_CUTOFF_MAX_V         20.0f

#define EOL_ABS_VOLTAGE_CRITICAL_MIN_V       0.0f
#define EOL_ABS_VOLTAGE_CRITICAL_MAX_V       20.0f

#define EOL_ABS_VOLTAGE_RESET_MIN_V          0.0f
#define EOL_ABS_VOLTAGE_RESET_MAX_V          20.0f


/* Current absolute limits */

#define EOL_ABS_CURRENT_MIN_SAFE_MIN_A       0.10f
#define EOL_ABS_CURRENT_MIN_SAFE_MAX_A       5.00f

#define EOL_ABS_CURRENT_MAX_SAFE_MIN_A       0.10f
#define EOL_ABS_CURRENT_MAX_SAFE_MAX_A       5.00f


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
     * Intended for serial logging and local EOL reporting.
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
/* EOL manager configuration                                                  */
/* -------------------------------------------------------------------------- */

typedef struct
{
    /*
     * Maximum time allowed for a complete EOL run.
     *
     * Safety limit for the EOL state machine.
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
     * Maximum current permitted during the EOL motor test itself.
     *
     * This is a test safety limit and is separate from
     * current_max_safe_a in eol_factory_config_t.
     */
    float motor_max_current_a;

    /*
     * Factory production configuration.
     *
     * This is the configuration established by EOL and subsequently
     * stored in NVS.
     */
    eol_factory_config_t factory_config;

} eol_manager_config_t;


/* -------------------------------------------------------------------------- */
/* Factory configuration validation                                           */
/* -------------------------------------------------------------------------- */

/*
 * Validate a factory configuration against the firmware absolute
 * safety boundaries.
 *
 * No NVS write is performed here.
 *
 * Returns:
 *
 *     ESP_OK
 *         Configuration is safe and structurally valid.
 *
 *     ESP_ERR_INVALID_ARG
 *         Configuration exceeds an absolute firmware limit or
 *         contains an invalid relationship.
 */
esp_err_t eol_manager_validate_factory_config(
    const eol_factory_config_t *config);


/*
 * Get the currently configured factory/EOL values.
 */
esp_err_t eol_manager_get_factory_config(
    eol_factory_config_t *config);


/*
 * Set the factory/EOL configuration in the EOL manager.
 *
 * The configuration is validated against the firmware absolute
 * limits before being accepted.
 *
 * This function does NOT directly write NVS.
 *
 * NVS persistence remains the responsibility of the
 * EOL persistence manager.
 */
esp_err_t eol_manager_set_factory_config(
    const eol_factory_config_t *config);


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
 *   - caller must place the actuator in a safe mechanical
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
 * Intentionally non-blocking at manager level.
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
 * Must stop/disable actuator activity before EOL exits.
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