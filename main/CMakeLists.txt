idf_component_register(
    SRCS
        "app_main.c"

        "../framework/hal/hal_adc.c"
        "../framework/hal/hal_actuator.c"
        "../framework/hal/hal_gpio.c"
        "../framework/hal/hal_i2c.c"
        "../framework/hal/hal_nvs.c"
        "../framework/hal/hal_pwm.c"
        "../framework/hal/hal_rtc.c"
        "../framework/hal/hal_spi.c"
        "../framework/hal/hal_uart.c"

        "../framework/drivers/as5600/drv_as5600.c"
        "../framework/drivers/ina226/drv_ina226.c"
        "../framework/drivers/rs485/drv_rs485.c"
        "../framework/drivers/modbus/modbus_master.c"
        "../framework/drivers/modem/drv_modem.c"

        "../framework/managers/encoder/encoder_manager.c"
        "../framework/managers/encoder_persistence/encoder_persistence_manager.c"
        "../framework/managers/encoder_position/encoder_position_manager.c"
"../framework/managers/valve_position_persistence/valve_position_persistence.c"
"../framework/managers/valve_position/valve_position_manager.c"
        "../framework/managers/rtc/rtc_manager.c"
        "../framework/managers/battery/battery_manager.c"

        "../framework/managers/modbus_devices/modbus_device_manager.c"
        "../framework/managers/modbus_poll/modbus_poll_manager.c"

        "../framework/managers/scheduling/scheduling_manager.c"
        "../framework/managers/condition_monitor/condition_monitor_manager.c"
        "../framework/managers/safety/safety_manager.c"
        "../framework/managers/actuator/actuator_manager.c"

        "../framework/managers/eol/eol_manager.c"
        "../framework/managers/eol_persistence/eol_persistence.c"
        "../framework/managers/eol_web/eol_web.c"

    INCLUDE_DIRS
        "."
        "../framework/hal"

        "../framework/drivers/as5600"
        "../framework/drivers/ina226"
        "../framework/drivers/rs485"
        "../framework/drivers/modbus"
        "../framework/drivers/modem"

        "../framework/managers/encoder"
        "../framework/managers/encoder_persistence"
        "../framework/managers/encoder_position"
"../framework/managers/valve_position_persistence"
"../framework/managers/valve_position"
        "../framework/managers/rtc"
        "../framework/managers/battery"

        "../framework/managers/modbus_devices"
        "../framework/managers/modbus_poll"

        "../framework/managers/scheduling"
        "../framework/managers/condition_monitor"
        "../framework/managers/safety"
        "../framework/managers/actuator"

        "../framework/managers/eol"
        "../framework/managers/eol_persistence"
        "../framework/managers/eol_web"

        "../boards/gsm_valve"
)