#include "stm32l4xx_hal.h"
#include "grove_rtc_ds1307.h"
#include <stdint.h>

/* Meme I2C1 que les deux tests deja valides sur la carte. */
extern I2C_HandleTypeDef hi2c1;
#define DS1307_ADDR (0x68U << 1)

static unsigned bcd_to_dec(uint8_t v) { return (v >> 4) * 10U + (v & 15U); }
static uint8_t dec_to_bcd(unsigned v) { return (uint8_t)((v / 10U) * 16U + v % 10U); }
static bool valid_bcd(uint8_t v) { return (v >> 4) <= 9U && (v & 15U) <= 9U; }
static bool valid_datetime(const RTC_DateTime *dt)
{
    static const unsigned days[] = {31,28,31,30,31,30,31,31,30,31,30,31};
    if (!dt || dt->year < 2000 || dt->year > 2099 || dt->month < 1 || dt->month > 12 ||
        dt->dayOfWeek < 1 || dt->dayOfWeek > 7 || dt->hours > 23 ||
        dt->minutes > 59 || dt->seconds > 59) return false;
    unsigned max_day = days[dt->month-1] + (dt->month == 2 && dt->year % 4 == 0);
    return dt->day >= 1 && dt->day <= max_day;
}

bool GroveRTC_Init(void)
{
    /* Detection seulement : ne remet pas la date a zero. */
    return HAL_I2C_IsDeviceReady(&hi2c1, DS1307_ADDR, 3, 100) == HAL_OK;
}

bool GroveRTC_GetDateTime(RTC_DateTime *dt)
{
    if (!dt) return false;
    uint8_t r[7];
    if (HAL_I2C_Mem_Read(&hi2c1, DS1307_ADDR, 0, I2C_MEMADD_SIZE_8BIT, r, 7, 100) != HAL_OK)
        return false;
    if (r[0] & 0x80U) return false; /* CH=1 : horloge arretee. */
    uint8_t hour_bcd = r[2] & ((r[2] & 0x40U) ? 0x1FU : 0x3FU);
    if (!valid_bcd(r[0] & 0x7F) || !valid_bcd(r[1] & 0x7F) ||
        !valid_bcd(hour_bcd) || !valid_bcd(r[4] & 0x3F) ||
        !valid_bcd(r[5] & 0x1F) || !valid_bcd(r[6])) return false;
    RTC_DateTime next = {0};
    next.seconds = bcd_to_dec(r[0] & 0x7F);
    next.minutes = bcd_to_dec(r[1] & 0x7F);
    unsigned hour = bcd_to_dec(hour_bcd);
    if (r[2] & 0x40U) {
        if (hour < 1 || hour > 12) return false;
        hour = hour % 12 + ((r[2] & 0x20U) ? 12 : 0);
    }
    next.hours = hour;
    next.dayOfWeek = r[3];
    next.day = bcd_to_dec(r[4] & 0x3F);
    next.month = bcd_to_dec(r[5] & 0x1F);
    next.year = 2000 + bcd_to_dec(r[6]);
    if (!valid_datetime(&next)) return false;
    *dt = next;
    return true;
}

bool GroveRTC_SetDateTime(const RTC_DateTime *dt)
{
    if (!valid_datetime(dt)) return false;
    uint8_t r[7] = {
        dec_to_bcd(dt->seconds), /* CH=0 */
        dec_to_bcd(dt->minutes),
        dec_to_bcd(dt->hours), /* mode 24 h */
        (uint8_t)dt->dayOfWeek,
        dec_to_bcd(dt->day), dec_to_bcd(dt->month), dec_to_bcd(dt->year-2000)
    };
    return HAL_I2C_Mem_Write(&hi2c1, DS1307_ADDR, 0, I2C_MEMADD_SIZE_8BIT, r, 7, 100) == HAL_OK;
}

