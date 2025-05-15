#include "rx8130ce.h"

#include "i2c.h"
#include "gpio.h"

#include "stm32h7xx_hal_i2c.h"
#include "stm32h7xx_hal_i2c_ex.h"
#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_hal_gpio_ex.h"

#include "elog.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#define RTC_USE_IIC_BUS (hi2c4)
#define RTC_USE_IIC_SLAVE_ADDR (0x32)
#define RTC_IIC_XFER_TIMEOUT (2000)

const static char *TAG = "rx8130 rtc";

#define RTC_RETURN_ON_FALSE(x, ret, mtag, mmsg, ...) \
    do                                               \
    {                                                \
        if (!(x))                                    \
        {                                            \
            elog_e(mtag, mmsg, ##__VA_ARGS__);       \
            return ret;                              \
        }                                            \
    } while (0)

#define RTC_DELAY_MS(x) vTaskDelay(pdMS_TO_TICKS(x))

/*========================================||========================================*/

/*====================||====================*/

/*========================================|tools|========================================*/

/**
 * @brief 将0-99的二进制数转换为BCD码
 * @param bin 输入的二进制数(0-99)
 * @return 对应的BCD码
 */
static uint8_t bin_to_bcd(uint8_t bin)
{
    if (bin > 99)
    {
        // 输入超出范围，可以返回错误或处理，这里简单返回0
        elog_e(TAG, "bin2bcd week fail");
        return 0;
    }

    uint8_t tens = bin / 10;    // 十位数
    uint8_t units = bin % 10;   // 个位数
    return (tens << 4) | units; // 组合成BCD码
} // bin_to_bcd

/**
 * @brief 将BCD码转换为二进制数
 * @param bcd 输入的BCD码
 * @return 对应的二进制数(0-99)
 */
static uint8_t bcd_to_bin(uint8_t bcd)
{
    uint8_t tens = (bcd >> 4) & 0x0F; // 提取十位数
    uint8_t units = bcd & 0x0F;       // 提取个位数

    // 检查每个数字是否有效(0-9)
    if (tens > 9 || units > 9)
    {
        // 无效的BCD码，可以返回错误或处理，这里简单返回0
        elog_e(TAG, "bcd2bin fail");
        return 0;
    }

    return tens * 10 + units;
} // bcd_to_bin

static rx8130_err_t time_chk(rtc_time_t t)
{
    if (t.tm_sec > 59 || t.tm_min > 59 || t.tm_hour > 23 ||
        t.tm_mday > 31 || t.tm_mon > 12 || t.tm_year > 99 ||
        t.tm_wday > 6)
    {
        return RX8130_ERR_CODE;
    }
    return RX8130_OK;
}

static uint8_t match_week(uint8_t reg)
{
    for (uint8_t i = 0; i < 7; i++)
    {
        // bit[0,6]
        if (reg & (1 << i))
        {
            return i;
        }
    }
    elog_e(TAG, "match week fail");
    return 0;
} // match_week

static uint8_t build_week(uint8_t week)
{
    if (week > 6)
    {
        return 0;
        elog_e(TAG, "build week fail");
    }
    return 1 << week;
} // match_week

/*========================================|static|========================================*/
static HAL_StatusTypeDef rtc_read_reg(uint8_t reg, uint8_t *dat, uint16_t len)
{
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(&RTC_USE_IIC_BUS, (RTC_USE_IIC_SLAVE_ADDR << 1), reg, I2C_MEMADD_SIZE_8BIT, dat, len, RTC_IIC_XFER_TIMEOUT);

    if (ret != HAL_OK)
    {
        elog_e(TAG, "read reg err %d", ret);
    }
    return ret;

} // rtc_read_reg

static HAL_StatusTypeDef rtc_write_reg(uint8_t reg, uint8_t *dat, uint16_t len)
{
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Write(&RTC_USE_IIC_BUS, (RTC_USE_IIC_SLAVE_ADDR << 1), reg, I2C_MEMADD_SIZE_8BIT, dat, len, RTC_IIC_XFER_TIMEOUT);
    if (ret != HAL_OK)
    {
        elog_e(TAG, "write reg err %d", ret);
    }
    return ret;
} // rtc_write_reg

static void rtc_dummy_read()
{
    uint8_t tmp;
    // read sec reg with 1ms timeout
    HAL_I2C_Mem_Read(&RTC_USE_IIC_BUS, (RTC_USE_IIC_SLAVE_ADDR << 1), 0x10, I2C_MEMADD_SIZE_8BIT, &tmp, 1, 1);
}

static rx8130_err_t rtc_software_reset()
{

    /*
    1)  Power ON
    2)  Wait > 30 ms
    3)  Dummy Read
    4)  Write 00h Address 1Eh
    5)  Write 80h Address 1Eh
    6)  Write 6Ch Address 50h
    7)  Write 01h Address 53h
    8)  Write 03h Address 66h
    9)  Write 02h Address 6Bh
    10) Write 01h Address 6Bh
    11) ????
    12) Wait 125 ms
    Software Reset complete

    RST signal outputs Low 95ms Max from step10.
    TEST bit is cleared automatically in step10.
    It has possibility leak current occurs from step 6) to step 10).
    because all power switch are turned to ON while this.
    After step 5), please complete the process immediately.
    */
    elog_i(TAG, "start sw rst");
    HAL_StatusTypeDef io_ret = HAL_OK;
    // stable RTC
    RTC_DELAY_MS(30);
    // Dummy-read
    rtc_dummy_read();

    // process start
    io_ret = rtc_write_reg(0x1e, (uint8_t[]){0x00}, 1);
    RTC_RETURN_ON_FALSE((io_ret == HAL_OK), RX8130_ERR_IO, TAG, "io err %d", io_ret);
    io_ret = rtc_write_reg(0x1e, (uint8_t[]){0x80}, 1);
    RTC_RETURN_ON_FALSE((io_ret == HAL_OK), RX8130_ERR_IO, TAG, "io err %d", io_ret);
    io_ret = rtc_write_reg(0x50, (uint8_t[]){0x6c}, 1);
    RTC_RETURN_ON_FALSE((io_ret == HAL_OK), RX8130_ERR_IO, TAG, "io err %d", io_ret);
    io_ret = rtc_write_reg(0x53, (uint8_t[]){0x01}, 1);
    RTC_RETURN_ON_FALSE((io_ret == HAL_OK), RX8130_ERR_IO, TAG, "io err %d", io_ret);
    io_ret = rtc_write_reg(0x66, (uint8_t[]){0x03}, 1);
    RTC_RETURN_ON_FALSE((io_ret == HAL_OK), RX8130_ERR_IO, TAG, "io err %d", io_ret);
    io_ret = rtc_write_reg(0x6b, (uint8_t[]){0x02}, 1);
    RTC_RETURN_ON_FALSE((io_ret == HAL_OK), RX8130_ERR_IO, TAG, "io err %d", io_ret);
    io_ret = rtc_write_reg(0x6b, (uint8_t[]){0x01}, 1);
    RTC_RETURN_ON_FALSE((io_ret == HAL_OK), RX8130_ERR_IO, TAG, "io err %d", io_ret);

    // Wait >125 ms
    RTC_DELAY_MS(130);
    return RX8130_OK;
} // rtc_software_reset

/*========================================||========================================*/

uint8_t rtc_get_flag(uint8_t flag)
{
    uint8_t tmp = 0;
    rtc_read_reg(0x1d, &tmp, 1);
    return (tmp & flag) ? 1 : 0;
} // rtc_get_flag

rx8130_err_t rtc_clear_flag(uint8_t flag)
{
    uint8_t tmp = 0;
    HAL_StatusTypeDef io_ret = HAL_OK;
    io_ret = rtc_read_reg(0x1d, &tmp, 1);
    RTC_RETURN_ON_FALSE((io_ret == HAL_OK), RX8130_ERR_IO, TAG, "io err %d", io_ret);

    tmp = tmp & ~(flag);

    io_ret = rtc_write_reg(0x1d, &tmp, 1);
    RTC_RETURN_ON_FALSE((io_ret == HAL_OK), RX8130_ERR_IO, TAG, "io err %d", io_ret);

    return RX8130_OK;
} // rtc_set_flag

rx8130_err_t rx8130_read_counter(uint16_t *cnt)
{
    uint8_t tmp[2];
    HAL_StatusTypeDef io_ret = HAL_OK;
    io_ret = rtc_read_reg(0x1a, tmp, 2); // 0x1a,0x1b
    RTC_RETURN_ON_FALSE((io_ret == HAL_OK), RX8130_ERR_IO, TAG, "io err %d", io_ret);
    *cnt = (tmp[1] << 8) | tmp[0];

    return RX8130_OK;
} // rx8130_read_counter

rx8130_err_t rx8130_set_counter(uint16_t cnt)
{
    uint8_t tmp[2] = {cnt & 0xff, (cnt >> 8) & 0xff};
    HAL_StatusTypeDef io_ret = HAL_OK;
    io_ret = rtc_write_reg(0x1a, tmp, 2); // 0x1a,0x1b
    RTC_RETURN_ON_FALSE((io_ret == HAL_OK), RX8130_ERR_IO, TAG, "io err %d", io_ret);
    return RX8130_OK;
} // rx8130_set_counter

rx8130_err_t rx8130_read_time(rtc_time_t *tim)
{
    uint8_t tmp[7];
    HAL_StatusTypeDef io_ret = HAL_OK;
    io_ret = rtc_read_reg(0x10, tmp, 7); // 0x10,7 sec,min,hor,wek,day,mon,yar
    RTC_RETURN_ON_FALSE((io_ret == HAL_OK), RX8130_ERR_IO, TAG, "io err %d", io_ret);
    memset(tim, 0x0, sizeof(rtc_time_t));

    tim->tm_sec = bcd_to_bin(tmp[0]);
    tim->tm_min = bcd_to_bin(tmp[1]);
    tim->tm_hour = bcd_to_bin(tmp[2]);
    tim->tm_wday = match_week(tmp[3]);
    tim->tm_mday = bcd_to_bin(tmp[4]);
    tim->tm_mon = bcd_to_bin(tmp[5]);
    tim->tm_year = bcd_to_bin(tmp[6]);

    return RX8130_OK;
} // rx8130_read_time

rx8130_err_t rx8130_set_time(rtc_time_t tim)
{
    uint8_t tmp[7];
    HAL_StatusTypeDef io_ret = HAL_OK;

    if (time_chk(tim) != RX8130_OK)
    {
        elog_e(TAG, "time range err");
        return RX8130_ERR_CODE;
    }

    tmp[0] = bin_to_bcd(tim.tm_sec);
    tmp[1] = bin_to_bcd(tim.tm_min);
    tmp[2] = bin_to_bcd(tim.tm_hour);
    tmp[3] = build_week(tim.tm_wday);
    tmp[4] = bin_to_bcd(tim.tm_mday);
    tmp[5] = bin_to_bcd(tim.tm_mon);
    tmp[6] = bin_to_bcd(tim.tm_year);

    io_ret = rtc_write_reg(0x10, tmp, 7); // 0x10,7 sec,min,hor,wek,day,mon,yar
    RTC_RETURN_ON_FALSE((io_ret == HAL_OK), RX8130_ERR_IO, TAG, "io err %d", io_ret);
    return RX8130_OK;
} // rx8130_set_time

/**
 *
 * @note for day match(WADA=1)
 */
rx8130_err_t rx8130_read_alarm(uint8_t *min_enable, uint8_t *hor_enable, uint8_t *day_enable,
                               uint8_t *set_min, uint8_t *set_hor, uint8_t *set_day)
{
    uint8_t tmp[3];
    HAL_StatusTypeDef io_ret = HAL_OK;
    io_ret = rtc_read_reg(0x17, tmp, 3); // 0x17 alarm min hor day
    RTC_RETURN_ON_FALSE((io_ret == HAL_OK), RX8130_ERR_IO, TAG, "io err %d", io_ret);

    *min_enable = (tmp[0] & 0x80) ? 0 : 1;
    *hor_enable = (tmp[1] & 0x80) ? 0 : 1;
    *day_enable = (tmp[2] & 0x80) ? 0 : 1;

    *set_min = bcd_to_bin((tmp[0] & 0x7f));
    *set_hor = bcd_to_bin((tmp[1] & 0x7f));
    *set_day = bcd_to_bin((tmp[2] & 0x7f));

    return RX8130_OK;
} // rx8130_read_alarm

/**
 *
 * @note for day match(WADA=1)
 */
rx8130_err_t rx8130_set_alarm(uint8_t min_enable, uint8_t hor_enable, uint8_t day_enable,
                              uint8_t set_min, uint8_t set_hor, uint8_t set_day)
{
    uint8_t tmp[3];
    HAL_StatusTypeDef io_ret = HAL_OK;

    if (set_min > 59 || set_hor > 23 || set_day > 31)
    {
        elog_e(TAG, "alarm range err");
        return RX8130_ERR_CODE;
    }

    tmp[0] = ((min_enable ? 0 : 1) << 7) | (bin_to_bcd(set_min) & 0x7f);
    tmp[1] = ((hor_enable ? 0 : 1) << 7) | (bin_to_bcd(set_hor) & 0x7f);
    tmp[2] = ((day_enable ? 0 : 1) << 7) | (bin_to_bcd(set_day) & 0x7f);

    io_ret = rtc_write_reg(0x17, tmp, 3); // 0x17 alarm min hor day
    RTC_RETURN_ON_FALSE((io_ret == HAL_OK), RX8130_ERR_IO, TAG, "io err %d", io_ret);
    return RX8130_OK;
}

rx8130_err_t rx8130_read_config(rx8130_control_t *control)
{
    uint8_t tmp[4];
    HAL_StatusTypeDef io_ret = HAL_OK;
    io_ret = rtc_read_reg(0x1c, &tmp[0], 1);
    RTC_RETURN_ON_FALSE((io_ret == HAL_OK), RX8130_ERR_IO, TAG, "io err %d", io_ret);
    io_ret = rtc_read_reg(0x1e, &tmp[1], 1);
    RTC_RETURN_ON_FALSE((io_ret == HAL_OK), RX8130_ERR_IO, TAG, "io err %d", io_ret);
    io_ret = rtc_read_reg(0x1f, &tmp[2], 1);
    RTC_RETURN_ON_FALSE((io_ret == HAL_OK), RX8130_ERR_IO, TAG, "io err %d", io_ret);
    io_ret = rtc_read_reg(0x31, &tmp[3], 1);
    RTC_RETURN_ON_FALSE((io_ret == HAL_OK), RX8130_ERR_IO, TAG, "io err %d", io_ret);

    elog_hexdump("reg", 8, tmp, 4);
    memcpy(control, tmp, sizeof(tmp));

    return RX8130_OK;
} // rx8130_read_config

rx8130_err_t rx8130_set_config(rx8130_control_t control)
{
    HAL_StatusTypeDef io_ret = HAL_OK;
    uint8_t *tmp = (uint8_t *)&control;

    tmp[1] = tmp[1] & 0x7f; // force test 1'b0
    tmp[2] = tmp[2] & 0xf7; // force rsvd 1'b0
    tmp[3] = tmp[3] & 0x01; // force rsvd 7'b0

    io_ret = rtc_write_reg(0x1f, &tmp[2], 1);
    RTC_RETURN_ON_FALSE((io_ret == HAL_OK), RX8130_ERR_IO, TAG, "io err %d", io_ret);

    io_ret = rtc_write_reg(0x1c, &tmp[0], 1);
    RTC_RETURN_ON_FALSE((io_ret == HAL_OK), RX8130_ERR_IO, TAG, "io err %d", io_ret);
    io_ret = rtc_write_reg(0x1e, &tmp[1], 1);
    RTC_RETURN_ON_FALSE((io_ret == HAL_OK), RX8130_ERR_IO, TAG, "io err %d", io_ret);
    io_ret = rtc_write_reg(0x31, &tmp[3], 1);
    RTC_RETURN_ON_FALSE((io_ret == HAL_OK), RX8130_ERR_IO, TAG, "io err %d", io_ret);

    return RX8130_OK;
} // rx8130_set_config

rx8130_err_t rx8130_read_XTAL_offset(uint8_t *enable, int8_t *offset)
{
    uint8_t tmp;
    HAL_StatusTypeDef io_ret = HAL_OK;
    io_ret = rtc_read_reg(0x30, &tmp, 1); // 0x30 offset
    RTC_RETURN_ON_FALSE((io_ret == HAL_OK), RX8130_ERR_IO, TAG, "io err %d", io_ret);

    *enable = (tmp & 0x80) ? 1 : 0;

    *offset = (int8_t)((tmp & 0x7f) << 1);

    return RX8130_OK;
} // rx8130_read_XTAL_offset

rx8130_err_t rx8130_set_XTAL_offset(uint8_t enable, int8_t offset)
{
    uint8_t tmp = (enable) ? 0x80 : 0x00;
    HAL_StatusTypeDef io_ret = HAL_OK;

    tmp |= (uint8_t)(offset >> 1);

    io_ret = rtc_write_reg(0x30, &tmp, 1); // 0x30 offset
    RTC_RETURN_ON_FALSE((io_ret == HAL_OK), RX8130_ERR_IO, TAG, "io err %d", io_ret);
    return RX8130_OK;
} // rx8130_set_XTAL_offset

rx8130_err_t rx8130_read_user_ram(uint32_t *ram)
{
    uint8_t tmp[4];
    HAL_StatusTypeDef io_ret = HAL_OK;

    io_ret = rtc_read_reg(0x20, tmp, 4); // 0x20 user ram
    RTC_RETURN_ON_FALSE((io_ret == HAL_OK), RX8130_ERR_IO, TAG, "io err %d", io_ret);

    *ram = ((uint32_t)tmp[0] << 24) | ((uint32_t)tmp[1] << 16) | ((uint32_t)tmp[2] << 8) | ((uint32_t)tmp[3]);

    return RX8130_OK;
} // rx8130_read_user_ram

rx8130_err_t rx8130_write_user_ram(uint32_t ram)
{
    uint8_t tmp[4] = {(ram >> 24) & 0xff, (ram >> 16) & 0xff, (ram >> 8) & 0xff, (ram >> 24) & 0xff};
    HAL_StatusTypeDef io_ret = HAL_OK;

    io_ret = rtc_write_reg(0x20, tmp, 4); // 0x20 user ram
    RTC_RETURN_ON_FALSE((io_ret == HAL_OK), RX8130_ERR_IO, TAG, "io err %d", io_ret);
    return RX8130_OK;
} // rx8130_write_user_ram

void rx8130_log_config(rx8130_control_t c)
{
    elog_i(TAG, "FOUT_freq %x", c.FOUT_freq);
    elog_i(TAG, "sec_min_irq_select %x", c.sec_min_irq_select);
    elog_i(TAG, "wkup_timer_enable %x", c.wkup_timer_enable);
    elog_i(TAG, "alarm_use_day_or_week %x", c.alarm_use_day_or_week);
    elog_i(TAG, "wkup_timer_clk_src %x", c.wkup_timer_clk_src);
    elog_i(TAG, "tim_stop %x", c.tim_stop);
    elog_i(TAG, "sec_min_irq_enable %x", c.sec_min_irq_enable);
    elog_i(TAG, "wkup_timer_irq_enable %x", c.wkup_timer_irq_enable);
    elog_i(TAG, "alarm_irq_enable %x", c.alarm_irq_enable);
    elog_i(TAG, "wkup_timer_stop %x", c.wkup_timer_stop);
    elog_i(TAG, "timer_count_which_power %x", c.timer_count_which_power);
    elog_i(TAG, "timer_count_all_power %x", c.timer_count_all_power);
    elog_i(TAG, "voltage_detection_period %x", c.voltage_detection_period);
    elog_i(TAG, "bkup_bat_chg_enable %x", c.bkup_bat_chg_enable);
    elog_i(TAG, "intf_disable_in_power_down %x", c.intf_disable_in_power_down);
    elog_i(TAG, "low_power_voltage %x", c.low_power_voltage);
    elog_i(TAG, "bkup_bat_charge_limit_voltage %x", c.bkup_bat_charge_limit_voltage);
    elog_i(TAG, "bkup_bat_charge_full_det_enable %x", c.bkup_bat_charge_full_det_enable);
}

inline void rx8130_log_tim(rtc_time_t tim)
{
    elog_i(TAG, "time:%2d-%2d-%2d %2d:%2d:%2d %d",
           tim.tm_year, tim.tm_mon, tim.tm_mday,
           tim.tm_hour, tim.tm_min, tim.tm_sec,
           tim.tm_wday);
} // rx8130_log_tim

rx8130_err_t rx8130_init()
{
    /*
    • At least 30 ms wait time is needed.
    So that it is stable RTC.
    40 ms is not oscillation startup time

    •When the power supply conditions for which Power-On Reset is executed cannot be satisfied,
    then must be execute a Dummy-read.

    Dummy read is one time read access to a free address Ignore ACK / NACK from RX8130CE in Dummy-read.
    Judge RX8130CE returned from backup normally, or fail.
    When the power conditions for which the power-on reset is executed are not satisfied, execute a software reset.
    For [Software Reset], start from "4) of "18.2 Software Reset."

    VLF cannot be cleared to 0 until internal oscillation starts.
    •Set any waiting time.

    Set the maximum number of loops, for an trouble of crystal oscillation

    =========================================================================================
    Initialization same as Power-On Reset is performed by this processing.
    As for the register value after software reset,
    See 13.2.2. Register initial value.

    Notes.
    It has possibility leak current occurs from step 6) to step 10).
    because all power switch are turned to ON while this.

    After step 5), please complete the process immediately.
    3) In a dummy lead, ignore NACK/ACK from RTC.
    10) TEST bit is cleared automatically in step10.
    Both time and a calendar register are not initialized in any values
    by this Software Reset.
    Both a calendar and Time before reset are maintained.
    /RST signal outputs Low 95ms Max from step10.
    Please care /RST active signal.
    And a VLF bit is set to 1.
    Please clear VLF to 0.
    */

    rx8130_err_t io_ret = RX8130_OK;

    // https://download.epsondevice.com/td/pdf/app/RX8130CE_cn.pdf
    // page 55

    // stable RTC
    RTC_DELAY_MS(30);

    // Dummy-read
    rtc_dummy_read();

    // read Flag Register for VLF
    if (!(rtc_get_flag(RX8130_FLAG_VLF)))
    {
        elog_i(TAG, "VLF=0,no reset reg");
        return RX8130_OK;
    } // VLF=0

    io_ret = rtc_software_reset();
    RTC_RETURN_ON_FALSE((io_ret == RX8130_OK), RX8130_ERR_IO, TAG, "io err %d", io_ret);

    io_ret = rtc_clear_flag(RX8130_FLAG_VLF);
    RTC_RETURN_ON_FALSE((io_ret == RX8130_OK), RX8130_ERR_IO, TAG, "io err %d", io_ret);

    return RX8130_OK;
} // rx8130_init

rx8130_err_t rx8130_debug()
{
    rx8130_control_t d;
    memset(&d, 0x0, sizeof(d));
    rx8130_read_config(&d);
    elog_i(TAG, "============================init");
    rx8130_log_config(d);

    rx8130_control_t def = RX8130_DEFAULT_CONFIG();
    rx8130_set_config(def);

    elog_i(TAG, "============================cover");
    memset(&d, 0x0, sizeof(d));
    rx8130_read_config(&d);
    rx8130_log_config(d);

    elog_i(TAG, "============================wkup timer");
    uint16_t cnt;

    rx8130_read_counter(&cnt);
    elog_i(TAG, "pwr on wkup timer = %d", cnt);

    rx8130_set_counter(0);

    rx8130_read_counter(&cnt);
    elog_i(TAG, "after zero wkup timer = %d", cnt);

    elog_i(TAG, "============================user ram");
    uint32_t user;

    rx8130_read_user_ram(&user);
    elog_i(TAG, "pwr on user = %lx", user);

    rx8130_write_user_ram(0x23333333);

    rx8130_read_user_ram(&user);
    elog_i(TAG, "after 233 user = %lx", user);

    elog_i(TAG, "============================rtc");
    rtc_time_t rtc;

    rx8130_read_time(&rtc);
    rx8130_log_tim(rtc);

    rtc_time_t init_rtc = {
        .tm_sec = 0,
        .tm_min = 1,
        .tm_hour = 1,
        .tm_mday = 1,
        .tm_mon = 1,
        .tm_year = 25,
        .tm_wday = 3,
    };

    rx8130_set_time(init_rtc);

    rx8130_read_time(&rtc);
    rx8130_log_tim(rtc);

    return RX8130_OK;
} // rx8130_debug

// eof