#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"

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

#include "board_config.h"


/* -------------------------------------------------------------------------- */
/* Application                                                                */
/* -------------------------------------------------------------------------- */

static const char *TAG = "GSM_VALVE";


/* -------------------------------------------------------------------------- */
/* EOL persistence identity                                                   */
/* -------------------------------------------------------------------------- */

/*
 * Stage 2A hardware/configuration identity.
 *
 * This is currently a deterministic configuration fingerprint.
 *
 * It is NOT a cryptographic identity and does not yet claim to
 * uniquely identify every replaceable physical component.
 *
 * The fingerprint will be extended later when component-specific
 * hardware identity information is exposed by the drivers.
 */
static const uint32_t EOL_HARDWARE_FINGERPRINT =
    0x47535632UL;


/*
 * Firmware/EOL compatibility identity.
 *
 * If the EOL compatibility requirements change, this value must
 * change. A mismatch causes the previous factory EOL result to
 * require re-verification.
 */
static const uint32_t EOL_FIRMWARE_FINGERPRINT =
    0x00010001UL;


/*
 * Firmware identification stored with the factory EOL record.
 */
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
/* INA226 configuration                                                       */
/* -------------------------------------------------------------------------- */

/*
 * GSM-VALVE INA226 hardware:
 *
 * I2C address:
 *   0x40
 *
 * Shunt:
 *   R010 = 0.010 Ohm
 *
 * Current LSB:
 *   1 mA / LSB
 *
 * Calibration:
 *
 *   CAL = 0.00512 / (Current_LSB * R_SHUNT)
 *       = 0.00512 / (0.001 * 0.010)
 *       = 512
 *
 * config_register = 0 selects the driver's default
 * continuous shunt + bus conversion configuration.
 */
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

/*
 * Electrical protection policy.
 *
 * Voltage:
 *
 *   critical = 11.2 V
 *   cut      = 11.6 V
 *   warning  = 12.0 V
 *   reset    = 12.0 V
 *
 * Current:
 *
 *   OC trip       = 6.0 A
 *   max attempts  = 3
 *
 * Bypass:
 *
 *   120 seconds.
 */
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

/*
 * Safety consumes Condition Monitor results.
 *
 * It does not directly access INA226.
 * It does not directly access ADC/VBAT.
 */
static const safety_manager_config_t safety_config = {

    .cut_voltage_v =
        11.6f,

    .critical_voltage_v =
        11.2f,

    .overcurrent_trip_a =
        6.0f,

    .battery_bypass_timeout_ms =
        120000U,

    /*
     * Temporary baseline motor runtime limit.
     */
    .motor_max_runtime_ms =
        120000U
};


/* -------------------------------------------------------------------------- */
/* EOL Manager configuration                                                  */
/* -------------------------------------------------------------------------- */

/*
 * EOL Stage 2A.
 *
 * Safe diagnostic tests only:
 *
 *   1. I2C scan
 *   2. RTC
 *   3. AS5600 encoder
 *   4. INA226
 *   5. Battery
 *
 * Motor tests remain disabled.
 *
 * No relay or motor output is activated by EOL.
 */
static const eol_manager_config_t eol_config = {

    .overall_timeout_ms =
        300000U,

    .test_timeout_ms =
        10000U,

    .motor_test_runtime_ms =
        2000U,

    .motor_min_turns =
        0.05f,

    .motor_max_current_a =
        6.0f
};


/* -------------------------------------------------------------------------- */
/* A7670C modem configuration                                                 */
/* -------------------------------------------------------------------------- */

/*
 * A7670C LTE modem.
 *
 * Initial UART baud:
 *
 *   115200
 *
 * IMPORTANT:
 *
 * 115200 is only the initial communication-test setting.
 * It has not yet been confirmed against the physical modem.
 *
 * The modem is not automatically power-cycled here.
 */
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

/*
 * Run the five safe EOL diagnostics.
 *
 * No motor movement is performed.
 */
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


    /* ---------------------------------------------------------------------- */
    /* Start EOL                                                              */
    /* ---------------------------------------------------------------------- */

    err =
        eol_manager_start();

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "EOL start failed: %s",
            esp_err_to_name(err));

        return err;
    }


    /* ---------------------------------------------------------------------- */
    /* I2C                                                                     */
    /* ---------------------------------------------------------------------- */

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


    /* ---------------------------------------------------------------------- */
    /* RTC                                                                     */
    /* ---------------------------------------------------------------------- */

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


    /* ---------------------------------------------------------------------- */
    /* Encoder                                                                 */
    /* ---------------------------------------------------------------------- */

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


    /* ---------------------------------------------------------------------- */
    /* INA226                                                                  */
    /* ---------------------------------------------------------------------- */

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


    /* ---------------------------------------------------------------------- */
    /* Battery                                                                 */
    /* ---------------------------------------------------------------------- */

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


    /* ---------------------------------------------------------------------- */
    /* Process EOL                                                             */
    /* ---------------------------------------------------------------------- */

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


    /* ---------------------------------------------------------------------- */
    /* Force safe state                                                        */
    /* ---------------------------------------------------------------------- */

    err =
        eol_manager_force_safe_state();

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "EOL safe-state operation failed: %s",
            esp_err_to_name(err));

        return err;
    }


    /* ---------------------------------------------------------------------- */
    /* Read EOL status                                                         */
    /* ---------------------------------------------------------------------- */

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


    /* ---------------------------------------------------------------------- */
    /* Persist completed EOL result                                           */
    /* ---------------------------------------------------------------------- */

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


    /* ---------------------------------------------------------------------- */
    /* EOL summary                                                             */
    /* ---------------------------------------------------------------------- */

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
/* Application initialization                                                 */
/* -------------------------------------------------------------------------- */

static esp_err_t app_init(void)
{
    esp_err_t err;


    /* ---------------------------------------------------------------------- */
    /* NVS                                                                      */
    /* ---------------------------------------------------------------------- */

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


    /*
     * Configure the identity currently represented by the
     * Stage 2A hardware/configuration baseline.
     */
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
    /* I2C                                                                     */
    /* ---------------------------------------------------------------------- */

    /*
     * Board mapping:
     *
     * SDA = GPIO8
     * SCL = GPIO9
     */
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

    err =
        condition_monitor_manager_init(
            &condition_monitor_config);

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
            condition_monitor_config.overcurrent_trip_a);


    /* ---------------------------------------------------------------------- */
    /* Safety Manager                                                          */
    /* ---------------------------------------------------------------------- */

    err =
        safety_manager_init(
            &safety_config);

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

    /*
     * Initial modem communication test.
     *
     * No power-cycle is performed.
     */
    err =
        drv_modem_init(
            &modem_config);

    if (err != ESP_OK) {

        ESP_LOGW(
            TAG,
            "A7670C modem initialization failed: %s",
            esp_err_to_name(err));

        /*
         * Modem is not required for Stage 2A safe
         * hardware diagnostics.
         */
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

    /*
     * Stage 2A runs once for the current bench-validation firmware.
     *
     * Safe tests:
     *
     *   I2C
     *   RTC
     *   AS5600
     *   INA226
     *   Battery
     *
     * Motor control remains disconnected.
     */
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

    /*
     * Motor control is not connected yet.
     *
     * Therefore motor_running remains false.
     */
    while (1) {

        uint32_t now_ms =
            (uint32_t)(
                esp_timer_get_time() /
                1000ULL);


        const bool motor_running =
            false;


        /* ------------------------------------------------------------------ */
        /* Condition Monitor update                                           */
        /* ------------------------------------------------------------------ */

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


        /* ------------------------------------------------------------------ */
        /* Condition Monitor reading                                          */
        /* ------------------------------------------------------------------ */

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


        /* ------------------------------------------------------------------ */
        /* Condition Monitor state                                            */
        /* ------------------------------------------------------------------ */

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


        /* ------------------------------------------------------------------ */
        /* Safety Manager inputs                                               */
        /* ------------------------------------------------------------------ */

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


        /* ------------------------------------------------------------------ */
        /* Safety Manager evaluation                                           */
        /* ------------------------------------------------------------------ */

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


        /* ------------------------------------------------------------------ */
        /* Runtime diagnostics                                                 */
        /* ------------------------------------------------------------------ */

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