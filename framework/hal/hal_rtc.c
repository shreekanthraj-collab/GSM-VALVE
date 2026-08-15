#include "hal_rtc.h"

#include <stddef.h>
#include <stdint.h>

#include "hal_i2c.h"


/* -------------------------------------------------------------------------- */
/* PCF8563                                                                    */
/* -------------------------------------------------------------------------- */

#define HAL_RTC_I2C_ADDRESS        0x51

#define PCF8563_REG_CONTROL1       0x00
#define PCF8563_REG_CONTROL2       0x01
#define PCF8563_REG_SECONDS        0x02
#define PCF8563_REG_MINUTES        0x03
#define PCF8563_REG_HOURS          0x04
#define PCF8563_REG_DAYS           0x05
#define PCF8563_REG_WEEKDAYS       0x06
#define PCF8563_REG_MONTHS         0x07
#define PCF8563_REG_YEARS          0x08

#define PCF8563_REG_ALARM_MINUTE   0x09
#define PCF8563_REG_ALARM_HOUR     0x0A
#define PCF8563_REG_ALARM_DAY      0x0B
#define PCF8563_REG_ALARM_WEEKDAY  0x0C

#define PCF8563_CONTROL1_STOP      0x20
#define PCF8563_SECONDS_VL         0x80

#define PCF8563_CONTROL2_AF        0x08
#define PCF8563_CONTROL2_AIE       0x02

#define PCF8563_ALARM_DISABLE      0x80


/* -------------------------------------------------------------------------- */
/* Internal state                                                             */
/* -------------------------------------------------------------------------- */

static bool s_initialized = false;


/* -------------------------------------------------------------------------- */
/* BCD helpers                                                                */
/* -------------------------------------------------------------------------- */

static uint8_t bcd_to_bin(uint8_t value)
{
    return (uint8_t)(((value >> 4) * 10U) +
                     (value & 0x0FU));
}


static uint8_t bin_to_bcd(uint8_t value)
{
    return (uint8_t)(((value / 10U) << 4) |
                     (value % 10U));
}


/* -------------------------------------------------------------------------- */
/* Validation                                                                 */
/* -------------------------------------------------------------------------- */

static bool is_leap_year(uint16_t year)
{
    return ((year % 4U) == 0U &&
            ((year % 100U) != 0U ||
             (year % 400U) == 0U));
}


static uint8_t days_in_month(
    uint16_t year,
    uint8_t month)
{
    static const uint8_t days[] = {
        31,
        28,
        31,
        30,
        31,
        30,
        31,
        31,
        30,
        31,
        30,
        31
    };

    if (month == 2U && is_leap_year(year)) {
        return 29U;
    }

    return days[month - 1U];
}


static bool valid_time(
    const hal_rtc_time_t *time)
{
    if (time == NULL) {
        return false;
    }

    /*
     * This HAL supports the PCF8563 calendar range
     * represented by its two-digit year register.
     */
    if (time->year < 2000U ||
        time->year > 2099U) {
        return false;
    }

    if (time->month < 1U ||
        time->month > 12U) {
        return false;
    }

    if (time->day < 1U ||
        time->day > days_in_month(time->year, time->month)) {
        return false;
    }

    if (time->hour > 23U) {
        return false;
    }

    if (time->minute > 59U) {
        return false;
    }

    if (time->second > 59U) {
        return false;
    }

    if (time->weekday > HAL_RTC_WEEKDAY_SATURDAY) {
        return false;
    }

    return true;
}


static bool valid_alarm(
    const hal_rtc_alarm_t *alarm)
{
    if (alarm == NULL) {
        return false;
    }

    if (alarm->minute > 59U) {
        return false;
    }

    if (alarm->hour > 23U) {
        return false;
    }

    if (alarm->day < 1U ||
        alarm->day > 31U) {
        return false;
    }

    if (alarm->weekday > HAL_RTC_WEEKDAY_SATURDAY) {
        return false;
    }

    return true;
}


/* -------------------------------------------------------------------------- */
/* I2C register helpers                                                       */
/* -------------------------------------------------------------------------- */

static esp_err_t read_registers(
    uint8_t start_register,
    uint8_t *data,
    size_t length)
{
    if (data == NULL || length == 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    return hal_i2c_write_read(
        HAL_RTC_I2C_ADDRESS,
        &start_register,
        1U,
        data,
        length);
}


static esp_err_t write_registers(
    uint8_t start_register,
    const uint8_t *data,
    size_t length)
{
    if (data == NULL || length == 0U) {
        return ESP_ERR_INVALID_ARG;
    }

    /*
     * Maximum transaction here is small and fixed by the RTC
     * register blocks. Keep the write buffer on the stack.
     */
    uint8_t buffer[8];

    if (length > (sizeof(buffer) - 1U)) {
        return ESP_ERR_INVALID_SIZE;
    }

    buffer[0] = start_register;

    for (size_t i = 0; i < length; ++i) {
        buffer[i + 1U] = data[i];
    }

    return hal_i2c_write(
        HAL_RTC_I2C_ADDRESS,
        buffer,
        length + 1U);
}


/* -------------------------------------------------------------------------- */
/* Time conversion                                                            */
/* -------------------------------------------------------------------------- */

static uint8_t calculate_weekday(
    uint16_t year,
    uint8_t month,
    uint8_t day)
{
    /*
     * Sakamoto algorithm.
     *
     * Result:
     * 0 = Sunday
     * 1 = Monday
     * ...
     * 6 = Saturday
     */
    static const uint8_t table[] = {
        0,
        3,
        2,
        5,
        0,
        3,
        5,
        1,
        4,
        6,
        2,
        4
    };

    uint16_t adjusted_year = year;

    if (month < 3U) {
        adjusted_year--;
    }

    return (uint8_t)(
        (adjusted_year +
         adjusted_year / 4U -
         adjusted_year / 100U +
         adjusted_year / 400U +
         table[month - 1U] +
         day) % 7U);
}


static uint32_t days_before_year(
    uint16_t year)
{
    uint32_t days = 0U;

    for (uint16_t y = 1970U; y < year; ++y) {
        days += is_leap_year(y) ? 366U : 365U;
    }

    return days;
}


static uint32_t days_before_month(
    uint16_t year,
    uint8_t month)
{
    uint32_t days = 0U;

    for (uint8_t m = 1U; m < month; ++m) {
        days += days_in_month(year, m);
    }

    return days;
}


static uint32_t time_to_timestamp(
    const hal_rtc_time_t *time)
{
    uint32_t days =
        days_before_year(time->year) +
        days_before_month(time->year, time->month) +
        (uint32_t)(time->day - 1U);

    return days * 86400UL +
           (uint32_t)time->hour * 3600UL +
           (uint32_t)time->minute * 60UL +
           (uint32_t)time->second;
}


static bool timestamp_to_time(
    uint32_t timestamp,
    hal_rtc_time_t *time)
{
    if (time == NULL) {
        return false;
    }

    uint32_t days = timestamp / 86400UL;
    uint32_t seconds = timestamp % 86400UL;

    uint16_t year = 1970U;

    while (true) {
        uint32_t year_days =
            is_leap_year(year) ? 366U : 365U;

        if (days < year_days) {
            break;
        }

        days -= year_days;
        year++;

        if (year > 2099U) {
            return false;
        }
    }

    uint8_t month = 1U;

    while (true) {
        uint8_t month_days =
            days_in_month(year, month);

        if (days < month_days) {
            break;
        }

        days -= month_days;
        month++;

        if (month > 12U) {
            return false;
        }
    }

    time->year = year;
    time->month = month;
    time->day = (uint8_t)(days + 1U);

    time->hour = (uint8_t)(seconds / 3600UL);
    seconds %= 3600UL;

    time->minute = (uint8_t)(seconds / 60UL);
    time->second = (uint8_t)(seconds % 60UL);

    time->weekday =
        calculate_weekday(
            time->year,
            time->month,
            time->day);

    return true;
}


/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

esp_err_t hal_rtc_init(void)
{
    if (s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!hal_i2c_is_initialized()) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err =
        hal_i2c_probe(HAL_RTC_I2C_ADDRESS);

    if (err != ESP_OK) {
        return err;
    }

    /*
     * Clear STOP so the oscillator is running.
     *
     * Preserve all other CONTROL1 bits.
     */
    uint8_t control1 = 0U;

    err = read_registers(
        PCF8563_REG_CONTROL1,
        &control1,
        1U);

    if (err != ESP_OK) {
        return err;
    }

    control1 &= (uint8_t)~PCF8563_CONTROL1_STOP;

    err = write_registers(
        PCF8563_REG_CONTROL1,
        &control1,
        1U);

    if (err != ESP_OK) {
        return err;
    }

    s_initialized = true;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Deinitialization                                                           */
/* -------------------------------------------------------------------------- */

esp_err_t hal_rtc_deinit(void)
{
    if (!s_initialized) {
        return ESP_OK;
    }

    s_initialized = false;

    return ESP_OK;
}


/* -------------------------------------------------------------------------- */
/* Time                                                                       */
/* -------------------------------------------------------------------------- */

esp_err_t hal_rtc_get_time(
    hal_rtc_time_t *time)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (time == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /*
     * Read seconds through years in one transaction.
     * This is important for a coherent RTC snapshot.
     */
    uint8_t data[7];

    esp_err_t err =
        read_registers(
            PCF8563_REG_SECONDS,
            data,
            sizeof(data));

    if (err != ESP_OK) {
        return err;
    }

    /*
     * VL indicates that the integrity of the time/calendar
     * information cannot be guaranteed.
     */
    if ((data[0] & PCF8563_SECONDS_VL) != 0U) {
        return ESP_ERR_INVALID_STATE;
    }

    time->second =
        bcd_to_bin(data[0] & 0x7FU);

    time->minute =
        bcd_to_bin(data[1] & 0x7FU);

    time->hour =
        bcd_to_bin(data[2] & 0x3FU);

    time->day =
        bcd_to_bin(data[3] & 0x3FU);

    time->weekday =
        data[4] & 0x07U;

    time->month =
        bcd_to_bin(data[5] & 0x1FU);

    /*
     * PCF8563 century bit:
     *
     * C = 0 -> 20xx
     * C = 1 -> 19xx
     *
     * This HAL intentionally supports the 2000-2099
     * WPK operating range.
     */
    if ((data[5] & 0x80U) != 0U) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    time->year =
        (uint16_t)(2000U +
                   bcd_to_bin(data[6]));

    if (!valid_time(time)) {
        return ESP_ERR_INVALID_STATE;
    }

    return ESP_OK;
}


esp_err_t hal_rtc_set_time(
    const hal_rtc_time_t *time)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!valid_time(time)) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t data[7];

    data[0] = bin_to_bcd(time->second);
    data[1] = bin_to_bcd(time->minute);
    data[2] = bin_to_bcd(time->hour);
    data[3] = bin_to_bcd(time->day);
    data[4] = time->weekday & 0x07U;

    /*
     * Year is limited to 2000-2099, therefore the century
     * bit remains cleared.
     */
    data[5] = bin_to_bcd(time->month);
    data[6] = bin_to_bcd(
        (uint8_t)(time->year - 2000U));

    return write_registers(
        PCF8563_REG_SECONDS,
        data,
        sizeof(data));
}


/* -------------------------------------------------------------------------- */
/* Timestamp                                                                  */
/* -------------------------------------------------------------------------- */

esp_err_t hal_rtc_get_timestamp(
    uint32_t *timestamp)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (timestamp == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    hal_rtc_time_t time;

    esp_err_t err =
        hal_rtc_get_time(&time);

    if (err != ESP_OK) {
        return err;
    }

    *timestamp = time_to_timestamp(&time);

    return ESP_OK;
}


esp_err_t hal_rtc_set_timestamp(
    uint32_t timestamp)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    hal_rtc_time_t time;

    if (!timestamp_to_time(timestamp, &time)) {
        return ESP_ERR_INVALID_ARG;
    }

    return hal_rtc_set_time(&time);
}


/* -------------------------------------------------------------------------- */
/* Alarm                                                                      */
/* -------------------------------------------------------------------------- */

esp_err_t hal_rtc_get_alarm(
    hal_rtc_alarm_t *alarm)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (alarm == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t data[4];

    esp_err_t err =
        read_registers(
            PCF8563_REG_ALARM_MINUTE,
            data,
            sizeof(data));

    if (err != ESP_OK) {
        return err;
    }

    alarm->minute_enabled =
        (data[0] & PCF8563_ALARM_DISABLE) == 0U;

    alarm->hour_enabled =
        (data[1] & PCF8563_ALARM_DISABLE) == 0U;

    alarm->day_enabled =
        (data[2] & PCF8563_ALARM_DISABLE) == 0U;

    alarm->weekday_enabled =
        (data[3] & PCF8563_ALARM_DISABLE) == 0U;

    alarm->minute =
        bcd_to_bin(data[0] & 0x7FU);

    alarm->hour =
        bcd_to_bin(data[1] & 0x7FU);

    alarm->day =
        bcd_to_bin(data[2] & 0x7FU);

    alarm->weekday =
        data[3] & 0x07U;

    uint8_t control2 = 0U;

    err = read_registers(
        PCF8563_REG_CONTROL2,
        &control2,
        1U);

    if (err != ESP_OK) {
        return err;
    }

    alarm->enabled =
        (control2 & PCF8563_CONTROL2_AIE) != 0U;

    return ESP_OK;
}


esp_err_t hal_rtc_set_alarm(
    const hal_rtc_alarm_t *alarm)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!valid_alarm(alarm)) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t data[4];

    data[0] =
        bin_to_bcd(alarm->minute);

    data[1] =
        bin_to_bcd(alarm->hour);

    data[2] =
        bin_to_bcd(alarm->day);

    data[3] =
        alarm->weekday & 0x07U;

    if (!alarm->minute_enabled) {
        data[0] |= PCF8563_ALARM_DISABLE;
    }

    if (!alarm->hour_enabled) {
        data[1] |= PCF8563_ALARM_DISABLE;
    }

    if (!alarm->day_enabled) {
        data[2] |= PCF8563_ALARM_DISABLE;
    }

    if (!alarm->weekday_enabled) {
        data[3] |= PCF8563_ALARM_DISABLE;
    }

    esp_err_t err =
        write_registers(
            PCF8563_REG_ALARM_MINUTE,
            data,
            sizeof(data));

    if (err != ESP_OK) {
        return err;
    }

    /*
     * Preserve other CONTROL2 bits while:
     *
     * - clearing a stale alarm flag
     * - enabling/disabling the alarm interrupt
     */
    uint8_t control2 = 0U;

    err = read_registers(
        PCF8563_REG_CONTROL2,
        &control2,
        1U);

    if (err != ESP_OK) {
        return err;
    }

    control2 &= (uint8_t)~PCF8563_CONTROL2_AF;

    if (alarm->enabled) {
        control2 |= PCF8563_CONTROL2_AIE;
    } else {
        control2 &= (uint8_t)~PCF8563_CONTROL2_AIE;
    }

    return write_registers(
        PCF8563_REG_CONTROL2,
        &control2,
        1U);
}


esp_err_t hal_rtc_clear_alarm(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t control2 = 0U;

    esp_err_t err =
        read_registers(
            PCF8563_REG_CONTROL2,
            &control2,
            1U);

    if (err != ESP_OK) {
        return err;
    }

    control2 &= (uint8_t)~(
        PCF8563_CONTROL2_AF |
        PCF8563_CONTROL2_AIE);

    return write_registers(
        PCF8563_REG_CONTROL2,
        &control2,
        1U);
}


/* -------------------------------------------------------------------------- */
/* State                                                                      */
/* -------------------------------------------------------------------------- */

bool hal_rtc_is_initialized(void)
{
    return s_initialized;
}