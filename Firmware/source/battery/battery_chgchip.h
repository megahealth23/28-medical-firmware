#ifndef _BAT_CHARGE_H_  
#define _BAT_CHARGE_H_

#include <stdbool.h>
#include <stdint.h>
#include "app_error.h"

#include "BQ21080\BQ21080.h"

#include "app_timer.h"
#include "ring_bsp.h"
#include "ring_startup.h"
#include "ring_swtimer.h"

#include "I2C.h"
#include "mTask.h"

#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "system_clock.h"
#include "battery_adc.h"

#ifdef __cplusplus  
extern "C"
{
#endif

#define CHGCHIP_DELAY_TIME_MS   (500)
#define CHGCHIP_FEEDWDT_PERIOD  (20)

struct chgchip_signal_s
{
    uint16_t interrupt;
    uint16_t feedwdt;
};

bool chgchip_init(void);
bool charg_chip_ok();
bool chgchip_timer_init(void);
bool chgchip_timer_start(void);
void update_batteryInfo_isr(void);
bool chgchip_is_oncharger();
uint8_t chgchip_charg_status();
void charg_chip_wdt_handle();
bool chgchip_set_mode(ring_chgchip_mode_t mode);

#ifdef __cplusplus  
}
#endif  
#endif /* _RING_ACC_H */  

