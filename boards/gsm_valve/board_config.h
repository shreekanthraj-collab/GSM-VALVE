#pragma once

/*
 * GSM-VALVE board adaptation layer
 *
 * Physical hardware mapping only.
 *
 * Runtime operating parameters such as over-current threshold,
 * battery limits, motor runtime and bypass timeout belong to
 * the configuration/manager layers and must not be placed here.
 */


/* -------------------------------------------------------------------------- */
/* RS485                                                                      */
/* -------------------------------------------------------------------------- */

#define BOARD_RS485_TX_GPIO          15
#define BOARD_RS485_RX_GPIO          16
#define BOARD_RS485_DE_GPIO          19


/* -------------------------------------------------------------------------- */
/* Motor actuator                                                             */
/* -------------------------------------------------------------------------- */

/*
 * Relay 1 = OPEN
 * Relay 2 = CLOSE
 * Relay 3 = MOTOR POWER
 *
 * PWM provides the motor soft-start / speed control.
 */

#define BOARD_MOTOR_FORWARD_GPIO     41
#define BOARD_MOTOR_REVERSE_GPIO     40
#define BOARD_MOTOR_POWER_GPIO       39
#define BOARD_MOTOR_PWM_GPIO         42


/* -------------------------------------------------------------------------- */
/* Other board signals                                                        */
/* -------------------------------------------------------------------------- */

#define BOARD_LIMIT_SWITCH_GPIO      (-1)
#define BOARD_RTC_INT_GPIO           (-1)

/*
 * SIMCom A7670C LTE modem.
 *
 * Hardware reference:
 *
 *   ESP32-S3 GPIO 17 -> UART1 TXD
 *   ESP32-S3 GPIO 18 -> UART1 RXD
 *   ESP32-S3 GPIO 6  -> LTE PWRKEY
 *   ESP32-S3 GPIO 7  -> LTE RESET
 *   ESP32-S3 GPIO 38 -> LTE STATUS
 *
 * UART baud rate is intentionally not defined here.
 * It belongs to the modem configuration layer and will
 * initially use the modem driver's 115200 default until
 * verified against the physical A7670C.
 */

#define BOARD_LTE_UART_PORT          UART_NUM_1
#define BOARD_LTE_TX_GPIO            17
#define BOARD_LTE_RX_GPIO            18
#define BOARD_LTE_PWRKEY_GPIO        6
#define BOARD_LTE_RESET_GPIO         7
#define BOARD_LTE_STATUS_GPIO        38

#define BOARD_BUZZER_GPIO            (-1)
#define BOARD_VBAT_ADC_GPIO          1


/* -------------------------------------------------------------------------- */
/* I2C                                                                        */
/* -------------------------------------------------------------------------- */

#define BOARD_I2C_SDA_GPIO           8
#define BOARD_I2C_SCL_GPIO           9