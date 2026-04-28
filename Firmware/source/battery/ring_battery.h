/*********************************************************************************
 * Copyright (c) 2019 MegaHealth. All Rights Reserved.
 * ring_battery.h
 * AUTHOR:zhao mengshou
 * DATE:
 *
 */

#ifndef _RING_BATTERY_H_
#define _RING_BATTERY_H_

#ifdef __cplusplus  
extern "C"
{
#endif 

#include <stdbool.h>
#include <stdint.h>
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "battery_adc.h"
#include "battery_chgchip.h"

#define RING_BATT_MARKER "%d.%02d"
 
/**
 * @brief Macro for dissecting a float number into two numbers (integer and residuum).
 */
#define BATT_SHOW(val)      (int32_t)(val),                                       \
                           (int32_t)((((val) > 0) ? (val) - (int32_t)(val)       \
                                                : (int32_t)(val) - (val))*100)

enum batt_downsample_rate_e
{
/*  
*   In our project, the measurement period of temperature is setting to 1Hz
*	TMP_SAMPLE_TIME_MS  =  1s  
*   1s = 1000ms  
*   Thus, max(TMP_FREQUENCY) is about 1Hz.	
*/   
	BATT_DOWNSAMPLE_NONE = 0,
	BATT_DOWNSAMPLE_1_60_HZ = 60,
	BATT_DOWNSAMPLE_1_30_HZ = 30,
	BATT_DOWNSAMPLE_1_10_HZ = 10,
	BATT_DOWNSAMPLE_1_5_HZ = 5,
	BATT_DOWNSAMPLE_1_2_HZ = 2,
	BATT_DOWNSAMPLE_1HZ = 1,
	BATT_DOWNSAMPLE_INVALID = 61
};
//
struct batt_dev_info_s
{
//  sensor data  by downsampling per second	
	uint32_t app_sec;		/* uint32_t app_get_sec(void) */
	
	uint32_t chging_irq_cnt;
	uint32_t dischg_irq_cnt;

//  time  domain
	int32_t    smp_period;
	int32_t    smp_times;
};

struct ring_batt_s
{
//  factor test domain
	struct batt_dev_info_s        dev_info;
	
};

//#define BATT_CHG_CURRENT_CTRL_PIN NRF_GPIO_PIN_MAP(1, 9) //Large current charging control;CD-25120A must be set HIGH initially;When charging,MCU£¨BOOT£© can detect the BAT voltage3.0-3.1V,CD-25120A must be set LOW;When MCU works normally,CD-25120A must be set LOW.

extern volatile struct battery_st battery_manager;

//#define ADC_LOWPOWER_VOL			(3.15f)
//#define ADC_SHUTDOWN_VOL			(3.0f)

void  update_battery_uptime(uint8_t tick_step);
uint32_t get_battery_uptime(void);

void battery_init(void);
void battery_start_check(void);
void battery_uninit(void);
bool device_is_oncharger();
struct batt_dev_info_s show_batt_dev_info(void);
void batt_devinfo_reset(void);

void charger_large_current(void);
void charger_small_current(void);
#ifdef __cplusplus  
}
#endif  
#endif /* _RING_BATTERY_H_ */  

