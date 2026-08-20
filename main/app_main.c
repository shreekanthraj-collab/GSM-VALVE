#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_event.h"
#include "esp_netif.h"

#include "hal_i2c.h"
#include "hal_nvs.h"
#include "hal_adc.h"
#include "rtc_manager.h"

#include "drv_as5600.h"
#include "drv_ina226.h"
#include "drv_modem.h"

#include "encoder_manager.h"
#include "encoder_persistence_manager.h"
#include "encoder_position_manager.h"

#include "battery_manager.h"

#include "condition_monitor_manager.h"
#include "safety_manager.h"

#include "eol_manager.h"
#include "eol_persistence.h"
#include "eol_web.h"

#include "board_config.h"


/* -------------------------------------------------------------------------- */
/* Application                                                                */
/* -------------------------------------------------------------------------- */

static const char *TAG = "GSM_VALVE";


/* -------------------------------------------------------------------------- */
/* EOL persistence identity                                                   */
/* -------------------------------------------------------------------------- */

static const uint32_t EOL_HARDWARE_FINGERPRINT =
    0x47535632UL;

static const uint32_t EOL_FIRMWARE_FINGERPRINT =
    0x00010001UL;

static const char *EOL_FIRMWARE_VERSION =
    "GSM-VALVE";


/* -------------------------------------------------------------------------- */
/* Battery configuration                                                      */
/* -------------------------------------------------------------------------- */

static const battery_manager_config_t battery_config = {

    .unit =
        ADC_UNIT_1,

    .channel =
        ADC_CHANNEL_0,

    .attenuation =
        ADC_ATTEN_DB_12,

    .bitwidth =
        ADC_BITWIDTH_DEFAULT,

    .divider_ratio =
        11.0f,

    .critical_voltage_v =
        11.2f,

    .cut_voltage_v =
        11.6f,

    .low_voltage_v =
        12.0f,

    .reset_voltage_v =
        12.0f,

    .high_voltage_v =
        12.4f
};


/* -------------------------------------------------------------------------- */
/* EOL factory operating configuration defaults                               */
/* -------------------------------------------------------------------------- */

static const eol_factory_config_t eol_factory_defaults = {

    .voltage_warn_low_v =
        12.0f,

    .voltage_warn_high_v =
        12.4f,

    .voltage_cutoff_v =
        11.6f,

    .voltage_critical_v =
        11.2f,

    .voltage_reset_v =
        12.0f,

    .voltage_bypass_timeout_ms =
        120000U,

    .current_min_safe_a =
        4.0f,

    .current_max_safe_a =
        5.0f
};


/* -------------------------------------------------------------------------- */
/* INA226 configuration                                                       */
/* -------------------------------------------------------------------------- */

static const drv_ina226_config_t ina226_config = {

    .i2c_address =
        DRV_INA226_I2C_ADDRESS_DEFAULT,

    .shunt_resistance_ohms =
        0.010f,

    .current_lsb_a =
        0.001f,

    .config_register =
        0U
};


/* -------------------------------------------------------------------------- */
/* Condition Monitor configuration                                            */
/* -------------------------------------------------------------------------- */

static const condition_monitor_config_t condition_monitor_config = {

    .warn_voltage_low_v =
        12.0f,

    .cut_voltage_v =
        11.6f,

    .critical_voltage_v =
        11.2f,

    .reset_voltage_v =
        12.0f,

    .overcurrent_trip_a =
        6.0f,

    .overcurrent_max_attempts =
        3U,

    .battery_bypass_timeout_ms =
        120000U
};


/* -------------------------------------------------------------------------- */
/* Safety Manager configuration                                               */
/* -------------------------------------------------------------------------- */

static const safety_manager_config_t safety_config = {

    .cut_voltage_v =
        11.6f,

    .critical_voltage_v =
        11.2f,

    .overcurrent_trip_a =
        6.0f,

    .battery_bypass_timeout_ms =
        120000U,

    .motor_max_runtime_ms =
        120000U
};


/* -------------------------------------------------------------------------- */
/* EOL Manager configuration                                                  */
/* -------------------------------------------------------------------------- */

static eol_manager_config_t eol_config = {

    .overall_timeout_ms =
        300000U,

    .test_timeout_ms =
        10000U,

    .motor_test_runtime_ms =
        2000U,

    .motor_min_turns =
        0.05f,

    .motor_max_current_a =
        5.0f,

    .factory_config =
        {
            .voltage_warn_low_v =
                12.0f,

            .voltage_warn_high_v =
                12.4f,

            .voltage_cutoff_v =
                11.6f,

            .voltage_critical_v =
                11.2f,

            .voltage_reset_v =
                12.0f,

            .voltage_bypass_timeout_ms =
                120000U,

            .current_min_safe_a =
                1.0f,

            .current_max_safe_a =
                5.0f
        }
};


/* -------------------------------------------------------------------------- */
/* EOL Web configuration                                                      */
/* -------------------------------------------------------------------------- */

/*
 * WPK-027A bench EOL web interface.
 *
 * SoftAP:
 *
 *     SSID     = GSM-VALVE-EOL
 *     Password = none
 *     Address  = 192.168.4.1
 *
 * This is intentionally an open local bench network.
 *
 * Production EOL security policy will be added later.
 */
static const eol_web_config_t eol_web_config = {

    .ssid =
        "GSM-VALVE-EOL",

    .password =
        "",

    .max_connections =
        1U,

    .start_http_server =
        true
};


/* -------------------------------------------------------------------------- */
/* A7670C modem configuration                                                 */
/* -------------------------------------------------------------------------- */

static const drv_modem_config_t modem_config = {

    .uart_port =
        BOARD_LTE_UART_PORT,

    .tx_gpio =
        BOARD_LTE_TX_GPIO,

    .rx_gpio =
        BOARD_LTE_RX_GPIO,

    .pwrkey_gpio =
        BOARD_LTE_PWRKEY_GPIO,

    .reset_gpio =
        BOARD_LTE_RESET_GPIO,

    .status_gpio =
        BOARD_LTE_STATUS_GPIO,

    .baud_rate =
        115200U,

    .uart_rx_buffer_size =
        4096U,

    .uart_tx_buffer_size =
        2048U,

    .uart_timeout_ms =
        1000U,

    .command_timeout_ms =
        3000U,

    .response_buffer_size =
        1024U
};


/* -------------------------------------------------------------------------- */
/* EOL Stage 2A runner                                                        */
/* -------------------------------------------------------------------------- */

static esp_err_t app_run_eol_stage2a(
    uint32_t now_ms)
{
    esp_err_t err;

    ESP_LOGI(
        TAG,
        "==================================================");

    ESP_LOGI(
        TAG,
        "EOL Stage 2A starting");

    ESP_LOGI(
        TAG,
        "Safe diagnostics only");

    ESP_LOGI(
        TAG,
        "I2C -> RTC -> AS5600 -> INA226 -> BATTERY");

    ESP_LOGI(
        TAG,
        "==================================================");

    err =
        eol_manager_start();

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "EOL start failed: %s",
            esp_err_to_name(err));

        return err;
    }

    ESP_LOGI(
        TAG,
        "EOL TEST: I2C scan");

    err =
        eol_manager_run_test(
            EOL_TEST_I2C_SCAN);

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "EOL I2C test failed: %s",
            esp_err_to_name(err));
    }

    ESP_LOGI(
        TAG,
        "EOL TEST: RTC");

    err =
        eol_manager_run_test(
            EOL_TEST_RTC);

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "EOL RTC test failed: %s",
            esp_err_to_name(err));
    }

    ESP_LOGI(
        TAG,
        "EOL TEST: AS5600 encoder");

    err =
        eol_manager_run_test(
            EOL_TEST_ENCODER);

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "EOL encoder test failed: %s",
            esp_err_to_name(err));
    }

    ESP_LOGI(
        TAG,
        "EOL TEST: INA226");

    err =
        eol_manager_run_test(
            EOL_TEST_INA226);

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "EOL INA226 test failed: %s",
            esp_err_to_name(err));
    }

    ESP_LOGI(
        TAG,
        "EOL TEST: Battery");

    err =
        eol_manager_run_test(
            EOL_TEST_BATTERY);

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "EOL battery test failed: %s",
            esp_err_to_name(err));
    }

    err =
        eol_manager_process(
            now_ms);

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "EOL processing failed: %s",
            esp_err_to_name(err));

        return err;
    }

    err =
        eol_manager_force_safe_state();

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "EOL safe-state operation failed: %s",
            esp_err_to_name(err));

        return err;
    }

    eol_status_t status = {0};

    err =
        eol_manager_get_status(
            &status);

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "EOL status read failed: %s",
            esp_err_to_name(err));

        return err;
    }

    ESP_LOGI(
        TAG,
        "EOL results: "
        "PASS=%lu "
        "FAIL=%lu "
        "SKIP=%lu "
        "NOT_RUN=%lu",
        (unsigned long)
            status.tests_passed,
        (unsigned long)
            status.tests_failed,
        (unsigned long)
            status.tests_skipped,
        (unsigned long)
            status.tests_not_run);

    if (status.state ==
        EOL_STATE_COMPLETE) {

        err =
            eol_persistence_save_result(
                &status);

        if (err != ESP_OK) {

            ESP_LOGE(
                TAG,
                "EOL result persistence failed: %s",
                esp_err_to_name(err));

            return err;
        }

        ESP_LOGI(
            TAG,
            "Factory EOL result saved to NVS");

        if (status.overall_result ==
            EOL_OVERALL_PASS) {

            ESP_LOGI(
                TAG,
                "FACTORY EOL RESULT: PASS");
        }
        else {

            ESP_LOGW(
                TAG,
                "FACTORY EOL RESULT: FAIL");
        }
    }
    else {

        ESP_LOGW(
            TAG,
            "EOL result was not COMPLETE; "
            "not saving factory record");
    }

    err =
        eol_manager_log_summary();

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "EOL summary failed: %s",
            esp_err_to_name(err));

        return err;
    }

    ESP_LOGI(
        TAG,
        "EOL Stage 2A complete");

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* EOL persistence startup status                                             */
/* -------------------------------------------------------------------------- */

static void app_log_eol_persistence_status(void)
{
    eol_persistence_validity_t validity =
        eol_persistence_get_validity();

    switch (validity) {

        case EOL_PERSISTENCE_VALID:

            ESP_LOGI(
                TAG,
                "==================================================");

            ESP_LOGI(
                TAG,
                "FACTORY EOL STATUS: VALID");

            ESP_LOGI(
                TAG,
                "Stored EOL record matches current identity");

            ESP_LOGI(
                TAG,
                "==================================================");

            break;

        case EOL_PERSISTENCE_REVERIFICATION_REQUIRED:

            ESP_LOGW(
                TAG,
                "==================================================");

            ESP_LOGW(
                TAG,
                "FACTORY EOL STATUS: "
                "RE-VERIFICATION REQUIRED");

            ESP_LOGW(
                TAG,
                "Previous EOL record is preserved in NVS");

            ESP_LOGW(
                TAG,
                "A new EOL verification is required");

            ESP_LOGW(
                TAG,
                "==================================================");

            break;

        case EOL_PERSISTENCE_FAILED:

            ESP_LOGW(
                TAG,
                "==================================================");

            ESP_LOGW(
                TAG,
                "FACTORY EOL STATUS: "
                "PREVIOUS RESULT FAILED");

            ESP_LOGW(
                TAG,
                "==================================================");

            break;

        case EOL_PERSISTENCE_NOT_TESTED:

        default:

            ESP_LOGI(
                TAG,
                "==================================================");

            ESP_LOGI(
                TAG,
                "FACTORY EOL STATUS: NOT TESTED");

            ESP_LOGI(
                TAG,
                "No completed factory EOL record exists");

            ESP_LOGI(
                TAG,
                "==================================================");

            break;
    }
}


/* -------------------------------------------------------------------------- */
/* Network initialization                                                     */
/* -------------------------------------------------------------------------- */

static esp_err_t app_init_network_stack(void)
{
    esp_err_t err;

    err =
        esp_netif_init();

    if (err != ESP_OK &&
        err != ESP_ERR_INVALID_STATE) {

        ESP_LOGE(
            TAG,
            "esp_netif_init failed: %s",
            esp_err_to_name(err));

        return err;
    }

    err =
        esp_event_loop_create_default();

    if (err != ESP_OK &&
        err != ESP_ERR_INVALID_STATE) {

        ESP_LOGE(
            TAG,
            "Default event loop initialization failed: %s",
            esp_err_to_name(err));

        return err;
    }

    ESP_LOGI(
        TAG,
        "Network/event infrastructure initialized");

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Application initialization                                                 */
/* -------------------------------------------------------------------------- */

static esp_err_t app_init(void)
{
    esp_err_t err;

    err =
        hal_nvs_init();

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "NVS initialization failed: %s",
            esp_err_to_name(err));

        return err;
    }

    ESP_LOGI(
        TAG,
        "NVS initialized");

    /* ---------------------------------------------------------------------- */
    /* EOL Persistence                                                         */
    /* ---------------------------------------------------------------------- */

    err =
        eol_persistence_init();

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "EOL persistence initialization failed: %s",
            esp_err_to_name(err));

        return err;
    }

    ESP_LOGI(
        TAG,
        "EOL persistence initialized");

    err =
        eol_persistence_set_hardware_fingerprint(
            EOL_HARDWARE_FINGERPRINT);

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "EOL hardware fingerprint setup failed: %s",
            esp_err_to_name(err));

        return err;
    }

    err =
        eol_persistence_set_firmware_fingerprint(
            EOL_FIRMWARE_FINGERPRINT);

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "EOL firmware fingerprint setup failed: %s",
            esp_err_to_name(err));

        return err;
    }

    err =
        eol_persistence_set_firmware_version(
            EOL_FIRMWARE_VERSION);

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "EOL firmware version setup failed: %s",
            esp_err_to_name(err));

        return err;
    }

    err =
        eol_persistence_validate_current_identity();

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "EOL identity validation failed: %s",
            esp_err_to_name(err));

        return err;
    }

    app_log_eol_persistence_status();

    /* ---------------------------------------------------------------------- */
    /* EOL Factory Operating Configuration                                     */
    /* ---------------------------------------------------------------------- */

    eol_factory_config_t active_factory_config = {0};
    err =
        eol_persistence_load_factory_config(
            &active_factory_config);

    if (err == ESP_ERR_NOT_FOUND) {

        active_factory_config =
            eol_factory_defaults;

        ESP_LOGI(
            TAG,
            "No EOL factory configuration found; "
            "using firmware defaults");

    } else if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "Failed to load EOL factory configuration: %s",
            esp_err_to_name(err));

        active_factory_config =
            eol_factory_defaults;

    }

    /*
     * Validate the configuration before making it active.
     *
     * This protects the runtime configuration even when
     * the stored NVS record is corrupted or outdated.
     */
    err =
        eol_manager_validate_factory_config(
            &active_factory_config);

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "EOL factory configuration rejected: %s; "
            "using firmware defaults",
            esp_err_to_name(err));

        active_factory_config =
            eol_factory_defaults;

        err =
            eol_manager_validate_factory_config(
                &active_factory_config);

        if (err != ESP_OK) {

            ESP_LOGE(
                TAG,
                "Firmware EOL factory defaults are invalid: %s",
                esp_err_to_name(err));

            return err;
        }
    }

        /*
     * Pass the validated factory configuration into the
     * EOL manager configuration before initialization.
     */
    eol_config.factory_config =
        active_factory_config;

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "Failed to apply EOL factory configuration: %s",
            esp_err_to_name(err));

        return err;
    }

    ESP_LOGI(
        TAG,
        "EOL factory configuration active: "
        "current_min=%.2f A, current_max=%.2f A",
        active_factory_config.current_min_safe_a,
        active_factory_config.current_max_safe_a);


    ESP_LOGI(
        TAG,
        "EOL configuration: "
        "Vlow=%.2f V "
        "Vhigh=%.2f V "
        "Vcut=%.2f V "
        "Vcritical=%.2f V "
        "Vreset=%.2f V "
        "bypass=%lu ms "
        "Imin=%.2f A "
        "Imax=%.2f A",
        (double)
            active_factory_config.voltage_warn_low_v,
        (double)
            active_factory_config.voltage_warn_high_v,
        (double)
            active_factory_config.voltage_cutoff_v,
        (double)
            active_factory_config.voltage_critical_v,
        (double)
            active_factory_config.voltage_reset_v,
        (unsigned long)
            active_factory_config.voltage_bypass_timeout_ms,
        (double)
            active_factory_config.current_min_safe_a,
        (double)
            active_factory_config.current_max_safe_a);

    /* ---------------------------------------------------------------------- */
    /* I2C                                                                     */
    /* ---------------------------------------------------------------------- */

    const hal_i2c_config_t i2c_config = {

        .frequency_hz =
            100000U,

        .timeout_ms =
            1000U
    };

    err =
        hal_i2c_init(
            &i2c_config);

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "I2C initialization failed: %s",
            esp_err_to_name(err));

        return err;
    }

    ESP_LOGI(
        TAG,
        "I2C initialized: 100 kHz");

    /* ---------------------------------------------------------------------- */
    /* RTC                                                                     */
    /* ---------------------------------------------------------------------- */

    err =
        rtc_manager_init();

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "RTC initialization failed: %s",
            esp_err_to_name(err));

        return err;
    }

    ESP_LOGI(
        TAG,
        "RTC manager initialized");

    /* ---------------------------------------------------------------------- */
    /* INA226                                                                  */
    /* ---------------------------------------------------------------------- */

    err =
        drv_ina226_init(
            &ina226_config);

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "INA226 initialization failed: %s",
            esp_err_to_name(err));

        return err;
    }

    ESP_LOGI(
        TAG,
        "INA226 initialized: "
        "addr=0x%02X "
        "shunt=%.3f Ohm "
        "current_lsb=%.3f A",
        ina226_config.i2c_address,
        (double)
            ina226_config.shunt_resistance_ohms,
        (double)
            ina226_config.current_lsb_a);

    /* ---------------------------------------------------------------------- */
    /* Condition Monitor                                                       */
    /* ---------------------------------------------------------------------- */
    condition_monitor_config_t runtime_condition_monitor_config =
        condition_monitor_config;

    runtime_condition_monitor_config.overcurrent_trip_a =
        active_factory_config.current_max_safe_a;
   err =
    condition_monitor_manager_init(
        &runtime_condition_monitor_config);
    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "Condition Monitor initialization failed: %s",
            esp_err_to_name(err));

        return err;
    }

    ESP_LOGI(
        TAG,
        "Condition Monitor initialized: OC=%.2f A",
        (double)
                     runtime_condition_monitor_config.overcurrent_trip_a);

    /* ---------------------------------------------------------------------- */
    /* Safety Manager                                                          */
    /* ---------------------------------------------------------------------- */
  safety_manager_config_t runtime_safety_config =
        safety_config;

    runtime_safety_config.overcurrent_trip_a =
        active_factory_config.current_max_safe_a;
    err =
        safety_manager_init(
            &runtime_safety_config);

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "Safety Manager initialization failed: %s",
            esp_err_to_name(err));

        return err;
    }

    ESP_LOGI(
        TAG,
        "Safety Manager initialized");

    /* ---------------------------------------------------------------------- */
    /* AS5600                                                                  */
    /* ---------------------------------------------------------------------- */

    const drv_as5600_config_t as5600_config = {

        .i2c_address =
            DRV_AS5600_I2C_ADDRESS
    };

    err =
        drv_as5600_init(
            &as5600_config);

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "AS5600 initialization failed: %s",
            esp_err_to_name(err));

        return err;
    }

    ESP_LOGI(
        TAG,
        "AS5600 initialized");

    /* ---------------------------------------------------------------------- */
    /* Encoder Manager                                                         */
    /* ---------------------------------------------------------------------- */

    err =
        encoder_manager_init();

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "Encoder manager initialization failed: %s",
            esp_err_to_name(err));

        return err;
    }

    ESP_LOGI(
        TAG,
        "Encoder manager initialized");

    /* ---------------------------------------------------------------------- */
    /* Encoder Persistence                                                     */
    /* ---------------------------------------------------------------------- */

    err =
        encoder_persistence_init();

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "Encoder persistence initialization failed: %s",
            esp_err_to_name(err));

        return err;
    }

    ESP_LOGI(
        TAG,
        "Encoder persistence initialized");

    /* ---------------------------------------------------------------------- */
    /* Encoder Position                                                        */
    /* ---------------------------------------------------------------------- */

    err =
        encoder_position_init();

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "Encoder position initialization failed: %s",
            esp_err_to_name(err));

        return err;
    }

    ESP_LOGI(
        TAG,
        "Encoder position manager initialized");

    /* ---------------------------------------------------------------------- */
    /* Restore Encoder Position                                                */
    /* ---------------------------------------------------------------------- */

    err =
        encoder_position_restore();

    if (err == ESP_ERR_NOT_FOUND) {

        ESP_LOGI(
            TAG,
            "No persisted encoder position found");

    }
    else if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "Encoder position restore failed: %s",
            esp_err_to_name(err));

        return err;
    }
    else {

        ESP_LOGI(
            TAG,
            "Encoder position restored");
    }

    /* ---------------------------------------------------------------------- */
    /* Initial Encoder Reading                                                */
    /* ---------------------------------------------------------------------- */

    uint16_t angle = 0U;

    err =
        drv_as5600_read_angle(
            &angle);

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "Initial AS5600 read failed: %s",
            esp_err_to_name(err));

        return err;
    }

    err =
        encoder_position_update(
            angle);

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "Initial encoder position update failed: %s",
            esp_err_to_name(err));

        return err;
    }

    /* ---------------------------------------------------------------------- */
    /* Encoder Diagnostic State                                               */
    /* ---------------------------------------------------------------------- */

    encoder_position_state_t position = {0};

    err =
        encoder_position_get_state(
            &position);

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "Encoder position state read failed: %s",
            esp_err_to_name(err));

        return err;
    }

    ESP_LOGI(
        TAG,
        "Encoder position: "
        "angle=%u "
        "total_angle=%lld "
        "turns=%.4f "
        "restored=%s",
        (unsigned)
            position.angle,
        (long long)
            position.total_angle,
        (double)
            position.total_turns,
        position.restored ? "yes" : "no");

    /* ---------------------------------------------------------------------- */
    /* Battery Manager                                                        */
    /* ---------------------------------------------------------------------- */

    err =
        battery_manager_init(
            &battery_config);

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "Battery manager initialization failed: %s",
            esp_err_to_name(err));

        return err;
    }

    ESP_LOGI(
        TAG,
        "Battery manager initialized: "
        "GPIO1 / ADC1_CH0");

    /* ---------------------------------------------------------------------- */
    /* Initial Battery Reading                                                */
    /* ---------------------------------------------------------------------- */

    battery_manager_reading_t battery = {0};

    err =
        battery_manager_read(
            &battery);

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "Initial battery read failed: %s",
            esp_err_to_name(err));

        return err;
    }

    ESP_LOGI(
        TAG,
        "Battery: "
        "ADC=%.3f V "
        "VBAT=%.3f V "
        "state=%d",
        (double)
            battery.adc_voltage_v,
        (double)
            battery.battery_voltage_v,
        (int)
            battery.state);

    /* ---------------------------------------------------------------------- */
    /* A7670C Modem                                                           */
    /* ---------------------------------------------------------------------- */

    err =
        drv_modem_init(
            &modem_config);

    if (err != ESP_OK) {

        ESP_LOGW(
            TAG,
            "A7670C modem initialization failed: %s",
            esp_err_to_name(err));
    }
    else {

        ESP_LOGI(
            TAG,
            "A7670C modem driver initialized");

        err =
            drv_modem_ping();

        if (err != ESP_OK) {

            ESP_LOGW(
                TAG,
                "A7670C AT ping failed: %s",
                esp_err_to_name(err));
        }
        else {

            ESP_LOGI(
                TAG,
                "A7670C AT ping passed");
        }
    }

    /* ---------------------------------------------------------------------- */
    /* EOL Manager                                                             */
    /* ---------------------------------------------------------------------- */

    err =
        eol_manager_init(
            &eol_config);

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "EOL manager initialization failed: %s",
            esp_err_to_name(err));

        return err;
    }

    ESP_LOGI(
        TAG,
        "EOL manager initialized");

    /* ---------------------------------------------------------------------- */
    /* Network infrastructure                                                  */
    /* ---------------------------------------------------------------------- */

    err =
        app_init_network_stack();

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "Network infrastructure initialization failed: %s",
            esp_err_to_name(err));

        return err;
    }

    /* ---------------------------------------------------------------------- */
    /* EOL Web                                                                 */
    /* ---------------------------------------------------------------------- */

    err =
        eol_web_init(
            &eol_web_config);

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "EOL web initialization failed: %s",
            esp_err_to_name(err));

        return err;
    }

    ESP_LOGI(
        TAG,
        "EOL web subsystem initialized");

    err =
        eol_web_start();

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "EOL web server start failed: %s",
            esp_err_to_name(err));

        return err;
    }

    ESP_LOGI(
        TAG,
        "==================================================");

    ESP_LOGI(
        TAG,
        "EOL WEB READY");

    ESP_LOGI(
        TAG,
        "SSID: %s",
        eol_web_get_ssid());

    ESP_LOGI(
        TAG,
        "Open: http://%s/",
        eol_web_get_ip_address());

    ESP_LOGI(
        TAG,
        "==================================================");

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Application entry                                                          */
/* -------------------------------------------------------------------------- */

void app_main(void)
{
    ESP_LOGI(
        TAG,
        "GSM-VALVE firmware starting");

    esp_err_t err =
        app_init();

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "Application initialization failed: %s",
            esp_err_to_name(err));

        while (1) {

            vTaskDelay(
                pdMS_TO_TICKS(1000));
        }
    }

    ESP_LOGI(
        TAG,
        "Application initialization complete");

    /* ---------------------------------------------------------------------- */
    /* EOL Stage 2A                                                           */
    /* ---------------------------------------------------------------------- */

    {
        uint32_t now_ms =
            (uint32_t)(
                esp_timer_get_time() /
                1000ULL);

        err =
            app_run_eol_stage2a(
                now_ms);

        if (err != ESP_OK) {

            ESP_LOGE(
                TAG,
                "EOL Stage 2A failed: %s",
                esp_err_to_name(err));
        }
        else {

            ESP_LOGI(
                TAG,
                "EOL Stage 2A finished");
        }
    }

    /* ---------------------------------------------------------------------- */
    /* Runtime Monitoring                                                     */
    /* ---------------------------------------------------------------------- */

    while (1) {

        uint32_t now_ms =
            (uint32_t)(
                esp_timer_get_time() /
                1000ULL);

        const bool motor_running =
            false;

        err =
            condition_monitor_manager_update(
                now_ms,
                motor_running);

        if (err != ESP_OK) {

            ESP_LOGE(
                TAG,
                "Condition Monitor update failed: %s",
                esp_err_to_name(err));

            vTaskDelay(
                pdMS_TO_TICKS(1000));

            continue;
        }

        condition_monitor_reading_t reading =
            {0};

        err =
            condition_monitor_manager_get_reading(
                &reading);

        if (err != ESP_OK) {

            ESP_LOGE(
                TAG,
                "Condition Monitor reading failed: %s",
                esp_err_to_name(err));

            vTaskDelay(
                pdMS_TO_TICKS(1000));

            continue;
        }

        condition_monitor_state_t condition_state =
            {0};

        err =
            condition_monitor_manager_get_state(
                &condition_state);

        if (err != ESP_OK) {

            ESP_LOGE(
                TAG,
                "Condition Monitor state failed: %s",
                esp_err_to_name(err));

            vTaskDelay(
                pdMS_TO_TICKS(1000));

            continue;
        }

        const safety_manager_inputs_t safety_inputs = {

            .motor_running =
                motor_running,

            .bus_voltage_v =
                reading.bus_voltage_v,

            .voltage_valid =
                reading.voltage_valid,

            .motor_current_a =
                reading.current_a,

            .current_valid =
                reading.current_valid,

            .voltage_locked =
                condition_state.voltage_state ==
                    CONDITION_VOLTAGE_LOCKED,

            .overcurrent =
                condition_state.current_state ==
                    CONDITION_CURRENT_OVERCURRENT,

            .overcurrent_locked =
                condition_state.current_state ==
                    CONDITION_CURRENT_LOCKED,

            .bypass_required =
                condition_state.bypass_required,

            .bypass_acknowledged =
                condition_state.bypass_acknowledged
        };

        safety_manager_output_t safety_output =
            {0};

        err =
            safety_manager_evaluate(
                &safety_inputs,
                now_ms,
                &safety_output);

        if (err != ESP_OK) {

            ESP_LOGE(
                TAG,
                "Safety Manager evaluation failed: %s",
                esp_err_to_name(err));

            vTaskDelay(
                pdMS_TO_TICKS(1000));

            continue;
        }

        ESP_LOGI(
            TAG,
            "MON: "
            "V=%.3f V "
            "I=%.3f A "
            "P=%.3f W "
            "VState=%d "
            "CState=%d "
            "Safety=%d "
            "AllowStart=%d "
            "AllowRun=%d "
            "Stop=%d "
            "Close=%d "
            "Bypass=%d "
            "Fault=%d",
            (double)
                reading.bus_voltage_v,
            (double)
                reading.current_a,
            (double)
                reading.power_w,
            (int)
                condition_state.voltage_state,
            (int)
                condition_state.current_state,
            (int)
                safety_output.state,
            safety_output.allow_motor_start,
            safety_output.allow_motor_run,
            safety_output.request_motor_stop,
            safety_output.request_motor_close,
            safety_output.bypass_required,
            safety_output.fault);

        vTaskDelay(
            pdMS_TO_TICKS(1000));
    }
}
