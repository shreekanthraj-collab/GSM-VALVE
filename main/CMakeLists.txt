idf_component_register(
    SRCS
        "app_main.c"

        "../framework/hal/hal_adc.c"
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

        "../framework/managers/encoder/encoder_manager.c"
        "../framework/managers/encoder_persistence/encoder_persistence_manager.c"
        "../framework/managers/encoder_position/encoder_position_manager.c"
        "../framework/managers/rtc/rtc_manager.c"
        "../framework/managers/battery/battery_manager.c"
        "../framework/managers/modbus_devices/modbus_device_manager.c"
        "../framework/managers/scheduling/scheduling_manager.c"

    INCLUDE_DIRS
        "."
        "../framework/hal"
        "../framework/drivers/as5600"
        "../framework/drivers/ina226"
        "../framework/drivers/rs485"
        "../framework/drivers/modbus"
        "../framework/managers/encoder"
        "../framework/managers/encoder_persistence"
        "../framework/managers/encoder_position"
        "../framework/managers/rtc"
        "../framework/managers/battery"
        "../framework/managers/modbus_devices"
        "../framework/managers/scheduling"
        "../boards/gsm_valve"
)
