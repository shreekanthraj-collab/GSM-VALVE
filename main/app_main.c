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

#include "encoder_manager.h"
#include "encoder_persistence_manager.h"
#include "encoder_position_manager.h"

#include "battery_manager.h"

#include "condition_monitor_manager.h"
#include "safety_manager.h"


/* -------------------------------------------------------------------------- */
/* Application                                                                */
/* -------------------------------------------------------------------------- */

static const char *TAG = "GSM_VALVE";


/* -------------------------------------------------------------------------- */
/* Battery configuration                                                      */
/* -------------------------------------------------------------------------- */

/*
 * GSM-VALVE hardware:
 *
 * VBAT ADC input:
 *   GPIO1 -> ADC1 channel 0
 *
 * Battery divider:
 *   100 kOhm / 10 kOhm
 *   ratio = (100k + 10k) / 10k = 11.0
 *
 * Battery policy:
 *   critical < 11.2 V
 *   cut      < 11.6 V
 *   low      < 12.0 V
 *   reset    = 12.0 V
 *   high     > 12.4 V
 *
 * The reset threshold is retained as part of the battery policy
 * configuration for the later motor/bypass manager.
 */
static const battery_manager_config_t battery_config = {
    .unit = ADC_UNIT_1,
    .channel = ADC_CHANNEL_0,
    .attenuation = ADC_ATTEN_DB_12,
    .bitwidth = ADC_BITWIDTH_DEFAULT,

    .divider_ratio = 11.0f,

    .critical_voltage_v = 11.2f,
    .cut_voltage_v = 11.6f,
    .low_voltage_v = 12.0f,
    .reset_voltage_v = 12.0f,
    .high_voltage_v = 12.4f
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
 * Expected motor current range:
 *   approximately 6 A to 7 A
 *
 * Shunt voltage:
 *   6 A -> 60 mV
 *   7 A -> 70 mV
 *
 * INA226 calibration:
 *
 *   CAL = 0.00512 / (Current_LSB * R_SHUNT)
 *
 *   CAL = 0.00512 / (0.001 * 0.010)
 *       = 512
 *
 * config_register = 0 selects the driver's default
 * continuous shunt + bus conversion configuration.
 */
static const drv_ina226_config_t ina226_config = {
    .i2c_address = DRV_INA226_I2C_ADDRESS_DEFAULT,

    .shunt_resistance_ohms = 0.010f,

    .current_lsb_a = 0.001f,

    .config_register = 0U
};


/* -------------------------------------------------------------------------- */
/* Condition Monitor configuration                                            */
/* -------------------------------------------------------------------------- */

/*
 * Electrical protection policy.
 *
 * Voltage:
 *   critical = 11.2 V
 *   cut      = 11.6 V
 *   warning  = 12.0 V
 *   reset    = 12.0 V
 *
 * Current:
 *   default OC trip = 6.0 A
 *   maximum attempts = 3
 *
 * The OC threshold is intentionally kept outside the board
 * configuration so it can later be made runtime configurable
 * through the higher configuration/NVS/AWS layer.
 *
 * Bypass acknowledgement timeout:
 *   120 seconds = 2 minutes
 */
static const condition_monitor_config_t condition_monitor_config = {
    .warn_voltage_low_v = 12.0f,

    .cut_voltage_v = 11.6f,

    .critical_voltage_v = 11.2f,

    .reset_voltage_v = 12.0f,

    .overcurrent_trip_a = 6.0f,

    .overcurrent_max_attempts = 3U,

    .battery_bypass_timeout_ms = 120000U
};


/* -------------------------------------------------------------------------- */
/* Safety Manager configuration                                               */
/* -------------------------------------------------------------------------- */

/*
 * Safety Manager consumes Condition Monitor results.
 *
 * It does not access INA226 directly.
 * It does not access ADC/VBAT directly.
 */
static const safety_manager_config_t safety_config = {
    .cut_voltage_v = 11.6f,

    .critical_voltage_v = 11.2f,

    .overcurrent_trip_a = 6.0f,

    .battery_bypass_timeout_ms = 120000U,

    /*
     * Temporary baseline motor runtime limit.
     *
     * This remains a configuration-layer parameter and can
     * later be made runtime configurable.
     */
    .motor_max_runtime_ms = 120000U
};


/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

static esp_err_t app_init(void)
{
    esp_err_t err;


    /* ---------------------------------------------------------------------- */
    /* NVS                                                                    */
    /* ---------------------------------------------------------------------- */

    /*
     * Initialize persistent storage first.
     */
    err = hal_nvs_init();

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
    /* I2C                                                                    */
    /* ---------------------------------------------------------------------- */

    /*
     * Initialize the I2C bus.
     *
     * Board GPIO mapping:
     *   SDA = GPIO 8
     *   SCL = GPIO 9
     *
     * The I2C HAL owns the physical bus.
     */
    const hal_i2c_config_t i2c_config = {
        .frequency_hz = 100000U,
        .timeout_ms = 1000U
    };

    err = hal_i2c_init(&i2c_config);

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
    /* RTC                                                                    */
    /* ---------------------------------------------------------------------- */

    /*
     * Initialize the RTC manager.
     *
     * The RTC manager depends on the already initialized
     * I2C HAL.
     */
    err = rtc_manager_init();

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
    /* INA226                                                                 */
    /* ---------------------------------------------------------------------- */

    /*
     * Initialize INA226 current / voltage monitoring.
     *
     * Hardware:
     *   INA226 address = 0x40
     *   Shunt          = R010 = 0.010 Ohm
     *   Current LSB    = 0.001 A
     *
     * The I2C HAL is already initialized.
     */
    err = drv_ina226_init(
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
        "addr=0x%02X shunt=%.3f Ohm current_lsb=%.3f A",
        ina226_config.i2c_address,
        (double)ina226_config.shunt_resistance_ohms,
        (double)ina226_config.current_lsb_a);


    /* ---------------------------------------------------------------------- */
    /* Condition Monitor                                                      */
    /* ---------------------------------------------------------------------- */

    /*
     * Initialize the Condition Monitor.
     *
     * The INA226 driver must already be initialized.
     */
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
        "Condition Monitor initialized: "
        "OC=%.2f A attempts=%u bypass=%lu ms",
        (double)condition_monitor_config.overcurrent_trip_a,
        (unsigned)condition_monitor_config.overcurrent_max_attempts,
        (unsigned long)
            condition_monitor_config.battery_bypass_timeout_ms);


    /* ---------------------------------------------------------------------- */
    /* Safety Manager                                                         */
    /* ---------------------------------------------------------------------- */

    /*
     * Initialize the Safety Manager.
     *
     * Safety Manager consumes Condition Monitor results.
     */
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
        "Safety Manager initialized: "
        "OC=%.2f A max_runtime=%lu ms",
        (double)safety_config.overcurrent_trip_a,
        (unsigned long)safety_config.motor_max_runtime_ms);


    /* ---------------------------------------------------------------------- */
    /* AS5600                                                                */
    /* ---------------------------------------------------------------------- */

    /*
     * Initialize the AS5600 driver.
     *
     * The AS5600 uses the shared I2C bus at address 0x36.
     */
    const drv_as5600_config_t as5600_config = {
        .i2c_address = DRV_AS5600_I2C_ADDRESS
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
    /* Encoder Manager                                                        */
    /* ---------------------------------------------------------------------- */

    /*
     * Initialize the wrap-aware encoder manager.
     */
    err = encoder_manager_init();

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
    /* Encoder Persistence                                                    */
    /* ---------------------------------------------------------------------- */

    /*
     * Initialize encoder persistence.
     *
     * NVS has already been initialized above.
     */
    err = encoder_persistence_init();

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
    /* Encoder Position                                                       */
    /* ---------------------------------------------------------------------- */

    /*
     * Initialize absolute encoder position management.
     *
     * This depends on:
     *   - AS5600 driver
     *   - encoder manager
     *   - encoder persistence
     */
    err = encoder_position_init();

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


    /*
     * Restore the previously persisted absolute position.
     *
     * ESP_ERR_NOT_FOUND is a normal first-boot condition.
     */
    err = encoder_position_restore();

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


    /*
     * Read the first live AS5600 angle.
     *
     * encoder_position_update() deliberately treats this
     * first sample as the new live reference and therefore
     * does not create movement relative to the persisted
     * last_angle.
     */
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


    /*
     * Report the resulting position for diagnostics.
     */
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
        "angle=%u total_angle=%lld turns=%.4f restored=%s",
        (unsigned)position.angle,
        (long long)position.total_angle,
        (double)position.total_turns,
        position.restored ? "yes" : "no");


    /* ---------------------------------------------------------------------- */
    /* Battery Manager                                                        */
    /* ---------------------------------------------------------------------- */

    /*
     * Initialize the battery manager.
     *
     * Hardware mapping:
     *   VBAT = GPIO1 = ADC1_CH0
     *
     * The battery manager owns this ADC channel.
     */
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


    /*
     * Take an initial battery measurement.
     *
     * This validates the ADC path during application startup
     * without yet applying motor-control battery policy.
     */
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
        "Battery: ADC=%.3f V "
        "VBAT=%.3f V state=%d",
        (double)battery.adc_voltage_v,
        (double)battery.battery_voltage_v,
        (int)battery.state);


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

        /*
         * Do not continue into normal application
         * runtime after mandatory initialization fails.
         */
        while (1) {

            vTaskDelay(
                pdMS_TO_TICKS(1000));
        }
    }


    ESP_LOGI(
        TAG,
        "Application initialization complete");


    /* ---------------------------------------------------------------------- */
    /* Runtime monitoring                                                     */
    /* ---------------------------------------------------------------------- */

    /*
     * Runtime monitoring loop.
     *
     * Motor control is intentionally NOT connected yet.
     *
     * Runtime sequence:
     *
     *   INA226
     *       |
     *       v
     *   Condition Monitor
     *       |
     *       v
     *   Safety Manager
     *       |
     *       v
     *   Diagnostics only
     *
     * motor_running is deliberately false until the actuator
     * control layer is integrated and validated.
     */
    while (1) {

        uint32_t now_ms =
            (uint32_t)(
                esp_timer_get_time() /
                1000ULL);

        /*
         * Motor is not connected to this runtime stage.
         */
        const bool motor_running = false;


        /* ------------------------------------------------------------------ */
        /* Condition Monitor update                                            */
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
        /* Condition Monitor reading                                           */
        /* ------------------------------------------------------------------ */

        condition_monitor_reading_t reading = {0};

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
        /* Condition Monitor state                                             */
        /* ------------------------------------------------------------------ */

        condition_monitor_state_t condition_state = {0};

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
        /* Safety Manager inputs                                                */
        /* ------------------------------------------------------------------ */

        /*
         * Safety Manager consumes Condition Monitor results.
         *
         * It does not read INA226 directly.
         */
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
        /* Safety Manager evaluation                                            */
        /* ------------------------------------------------------------------ */

        safety_manager_output_t safety_output = {0};

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

        /*
         * No actuator command is executed here.
         *
         * Safety requests are logged only.
         */
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
            (double)reading.bus_voltage_v,
            (double)reading.current_a,
            (double)reading.power_w,
            (int)condition_state.voltage_state,
            (int)condition_state.current_state,
            (int)safety_output.state,
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