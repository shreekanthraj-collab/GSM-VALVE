#pragma once

/*
 * GSM-VALVE board adaptation layer
 *
 * GPIO assignments are provisional.
 * Final PCB mapping can be changed here without
 * changing application/controller logic.
 */

#define BOARD_MOTOR_POWER_GPIO       (-1)
#define BOARD_MOTOR_FORWARD_GPIO     (-1)
#define BOARD_MOTOR_REVERSE_GPIO     (-1)
#define BOARD_MOTOR_PWM_GPIO         (-1)

#define BOARD_RS485_TX_GPIO          (-1)
#define BOARD_RS485_RX_GPIO          (-1)
#define BOARD_RS485_DE_GPIO          (-1)

#define BOARD_LIMIT_SWITCH_GPIO      (-1)
#define BOARD_RTC_INT_GPIO           (-1)
#define BOARD_LTE_PWRKEY_GPIO        (-1)
#define BOARD_LTE_RESET_GPIO         (-1)
#define BOARD_LTE_STATUS_GPIO        (-1)
#define BOARD_BUZZER_GPIO            (-1)
#define BOARD_VBAT_ADC_GPIO          (-1)
#define BOARD_I2C_SDA_GPIO           (-1)
#define BOARD_I2C_SCL_GPIO           (-1)