#ifndef __RX8130_H__
#define __RX8130_H__

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
/*========================================||========================================*/

/*
13.1.功能概述
1） 时钟功能
此功能用于设置和读取秒、分钟、小时、天、月、年（到最后两位数字）和日期数据。
任何（两位数）为4的倍数的年份都被视为闰年，并自动计算到2099年。
在I2C通信开始时，时间和时钟计数停止（这会导致时间损失），时钟在I2C通信结束时再次自动启动。
2） 唤醒定时器中断功能
唤醒定时器中断功能在244.14之间的任何唤醒设置时定期生成中断事件s和65535小时。
当产生中断事件时，/IRQ引脚变为低电平，TF位设置为“1”，以报告发生了事件。
3） 长定时器功能
它能够使用唤醒定时器中断功能作为长定时器或使用计数器。
此功能测量主电源和备用电源的运行时间，并可以自动将其相加。
4） 报警中断功能
报警中断功能为日期、日期、小时和分钟设置等报警设置生成中断事件。
当发生中断事件时，AF位值设置为“1”，/IRQ引脚变为低电平，表示发生了事件。
5） 时间更新中断功能
时间更新中断功能以一秒或一分钟的间隔产生中断，与RTC的秒或分钟时间寄存器的更新同步。当产生中断事件时，/IRQ引脚变为低电平，“1”设置为UF位，以报告发生了事件。
6） 频率停止检测功能
此标志位指示时钟操作或内部数据的保留状态。当由于电源电压低而可能发生数据丢失时，其值从“0”变为“1”。
7） 时钟输出功能
可以从FOUT引脚输出与内置晶体谐振器具有相同频率（32.768kHz）的时钟。输出也可以是1 Hz或1024 Hz。
8） 用户RAM
RAM寄存器可对任何数据进行读/写访问。
9） 数字偏移功能
通过添加时间偏移可以提高时钟精度。
*/

typedef enum
{
    RX8130_OK,
    RX8130_ERR_IO,
    RX8130_ERR_CODE,
    RX8130_ERR_TIMEOUT,
    RX8130_ERR_MAX,
} rx8130_err_t;

typedef struct __attribute__((packed))
{
    // 1C Extension Register FSEL1 FSEL0 USEL TE WADA TSEL2 TSEL1 TSEL0
    uint8_t wkup_timer_clk_src : 3;    // TSEL[2...0] , The combination of these three bits is used to set the countdown period (source clock) for this function.
    uint8_t alarm_use_day_or_week : 1; // WADA , ( Week Alarm / Day Alarm Select )
    uint8_t wkup_timer_enable : 1;     // TE , ( Timer Enable )
    uint8_t sec_min_irq_select : 1;    // USEL , This bit is used to select "second" update or "minute" update as the timing for generation of time update interrupt events.
    uint8_t FOUT_freq : 2;             // FSEL[1..0] , FOUT Frequency selection

    // 1E Control Register0 TEST STOP UIE TIE AIE TSTP TBKON TBKE
    uint8_t timer_count_all_power : 1;   // TBKE , This setting counts normal mode and backup mode.
    uint8_t timer_count_which_power : 1; // TBKON , This function selects the operation time with the main power supply or the operation time with the backup power supply. The count value is added.
    uint8_t wkup_timer_stop : 1;         // TSTP , This bit is used to stop wakeup timer count down
    uint8_t alarm_irq_enable : 1;        // AIE , Alarm Interrupt Enable
    uint8_t wkup_timer_irq_enable : 1;   // TIE , wakeup timer interrupt enable
    uint8_t sec_min_irq_enable : 1;      // UIE , When a time update interrupt enable
    uint8_t tim_stop : 1;                // STOP , This bit is to stop a timekeeping operation. I
    uint8_t : 1;                         // test , always zero

    // 1F Control Register1 SMP_TSEL[1,0] CHG EN  INIEN  0x0 RS_VSEL BF_VSEL[1,0]
    uint8_t bkup_bat_charge_limit_voltage : 2; // BF_VSEL , Charge stop voltage (full charge)
    uint8_t low_power_voltage : 1;             // RS_VSEL , Setting of VDET1 voltage level. In case VDD drops below this level, the /RST-signal is output and the I/F and FOUT output are stopped (depending on INIEN-bit setting).
    uint8_t : 1;                               // rsvd , always zero
    uint8_t intf_disable_in_power_down : 1;    // INIEN , I2C and FOUT operate even if the VDD terminal voltage is VDET1 or less.
    uint8_t bkup_bat_chg_enable : 1;           // CHG EN , This bit has to be set to allow charging of a Re-chargeable battery connected to VBAT from VDD pin.
    uint8_t voltage_detection_period : 2;      // SMP_TSEL[1,0] , VDET3, VDET4 intermittent detection period

    // 31 Extension Register1 - - - - - - - VBLFE
    uint8_t bkup_bat_charge_full_det_enable : 1; // VBLFE , VBLF detection not available
    uint8_t : 7;                                 // rsvd , always zero

} rx8130_control_t;

typedef struct
{
    uint8_t tm_sec;  /* seconds after the minute, 0 to 60
                       (0 - 60 allows for the occasional leap second) */
    uint8_t tm_min;  /* minutes after the hour, 0 to 59 */
    uint8_t tm_hour; /* hours since midnight, 0 to 23 */

    uint8_t tm_mday; /* day of the month, 1 to 31 */
    uint8_t tm_mon;  /* months since January, 0 to 11 */
    uint8_t tm_year; /* years since 2000 */

    uint8_t tm_wday; /* days since Sunday, 0 to 6 */
} rtc_time_t;
/*========================================|event flags|========================================*/

#define RX8130_FLAG_VBLF (0x80) // Low VBAT detection
#define RX8130_FLAG_UF (0x20)   // Time update interrupt events
#define RX8130_FLAG_TF (0x10)   // Wakeup timer interrupt was occurred
#define RX8130_FLAG_AF (0x08)   // Alarm interrupt events
#define RX8130_FLAG_RSF (0x04)  // voltage drops below -VDET1 was detected
#define RX8130_FLAG_VLF (0x02)  // Oscillation stop is detected
#define RX8130_FLAG_VBFF (0x01) // Full charge of VBAT detected

/*========================================|config|========================================*/

#define RX8130_FOUT_32KHZ (0x0)
#define RX8130_FOUT_1KHZ (0x1)
#define RX8130_FOUT_1HZ (0x2)
#define RX8130_FOUT_OFF (0x3)

#define RX8130_MAIN_TIMER_UPDATE_EVENT_SELECT_SEC (0x0)
#define RX8130_MAIN_TIMER_UPDATE_EVENT_SELECT_MIN (0x1)

#define RX8130_ALARM_USE_WEEK (0x0)
#define RX8130_ALARM_USE_DAY (0x1)

#define RX8130_WKUP_TIMER_CLK_SRC_4KHZ (0x0)
#define RX8130_WKUP_TIMER_CLK_SRC_64HZ (0x1)
#define RX8130_WKUP_TIMER_CLK_SRC_1HZ (0x2)
#define RX8130_WKUP_TIMER_CLK_SRC_1DIV60HZ (0x3)
#define RX8130_WKUP_TIMER_CLK_SRC_1DIV3K6HZ (0x4)

#define RX8130_WKUP_TIMER_RUN_IN_NORMAL_MODE (0x0)
#define RX8130_WKUP_TIMER_RUN_IN_BACKUP_MODE (0x0)

#define RX8130_VOLTAGE_DET_PERID_2MS (0x0)
#define RX8130_VOLTAGE_DET_PERID_16MS (0x1)
#define RX8130_VOLTAGE_DET_PERID_128MS (0x2)
#define RX8130_VOLTAGE_DET_PERID_256MS (0x3)

#define RX8130_LOW_POWER_VOLTAGE_2V75 (0x0)
#define RX8130_LOW_POWER_VOLTAGE_2V70 (0x1)

#define RX8130_CHARGE_FULL_3V02 (0x0)
#define RX8130_CHARGE_FULL_3V08 (0x1)
#define RX8130_CHARGE_FULL_2V92 (0x2)
#define RX8130_CHARGE_FULL_UNLIMIT (0x3)

#define RX8130_DEFAULT_CONFIG() {                                    \
    .FOUT_freq = RX8130_FOUT_OFF,                                    \
    .sec_min_irq_select = RX8130_MAIN_TIMER_UPDATE_EVENT_SELECT_MIN, \
    .wkup_timer_enable = 1,                                          \
    .alarm_use_day_or_week = RX8130_ALARM_USE_DAY,                   \
    .wkup_timer_clk_src = RX8130_WKUP_TIMER_CLK_SRC_1HZ,             \
    .tim_stop = 0,                                                   \
    .sec_min_irq_enable = 0,                                         \
    .wkup_timer_irq_enable = 0,                                      \
    .alarm_irq_enable = 0,                                           \
    .wkup_timer_stop = 0,                                            \
    .timer_count_which_power = RX8130_WKUP_TIMER_RUN_IN_NORMAL_MODE, \
    .timer_count_all_power = 1,                                      \
    .voltage_detection_period = RX8130_VOLTAGE_DET_PERID_256MS,      \
    .bkup_bat_chg_enable = 0,                                        \
    .intf_disable_in_power_down = 1,                                 \
    .low_power_voltage = RX8130_LOW_POWER_VOLTAGE_2V70,              \
    .bkup_bat_charge_limit_voltage = RX8130_CHARGE_FULL_2V92,        \
    .bkup_bat_charge_full_det_enable = 0,                            \
}

/*========================================||========================================*/

uint8_t rtc_get_flag(uint8_t flag);
rx8130_err_t rtc_clear_flag(uint8_t flag);

rx8130_err_t rx8130_read_counter(uint16_t *cnt);
rx8130_err_t rx8130_set_counter(uint16_t cnt);

rx8130_err_t rx8130_read_time(rtc_time_t *tim);
rx8130_err_t rx8130_set_time(rtc_time_t tim);

rx8130_err_t rx8130_read_alarm(uint8_t *min_enable, uint8_t *hor_enable, uint8_t *day_enable,
                               uint8_t *set_min, uint8_t *set_hor, uint8_t *set_day);

rx8130_err_t rx8130_set_alarm(uint8_t min_enable, uint8_t hor_enable, uint8_t day_enable,
                              uint8_t set_min, uint8_t set_hor, uint8_t set_day);

rx8130_err_t rx8130_read_config(rx8130_control_t *control);
rx8130_err_t rx8130_set_config(rx8130_control_t control);

rx8130_err_t rx8130_read_XTAL_offset(uint8_t *enable, int8_t *offset);
rx8130_err_t rx8130_set_XTAL_offset(uint8_t enable, int8_t offset);

rx8130_err_t rx8130_read_user_ram(uint32_t *ram);
rx8130_err_t rx8130_write_user_ram(uint32_t ram);

void rx8130_log_tim(rtc_time_t tim);
void rx8130_log_config(rx8130_control_t c);

rx8130_err_t rx8130_init();
rx8130_err_t rx8130_debug();

#endif