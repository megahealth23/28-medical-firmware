/*
* Copyright (c) 2019 MegaHealth. All Rights Reserved.
* @file ring_handonled.h
* @author:li tianzheng
* @modifing:
* @date 2020.0.0
* @version
* @brief
*/
#ifndef _RING_HANDONLED_H_
#define _RING_HANDONLED_H_

#ifdef __cplusplus  
extern "C"
{
#endif 

#include "app_pwm.h"
#include "app_timer.h"
#include "nrf_drv_timer.h"
#include "app_error.h"
#include <nrf_drv_gpiote.h>
#include <nrfx_gpiote.h>
#include "nrf_log.h"

#include "system_clock.h"
#include "app_gpiote.h"

#include "ring_bsp.h"

#ifdef HW_V2_CONFIG_ENABLE
#include "ring_ft.h"
#endif

enum handonled_duty_e
{
/* Note that, V8's LED is driven by the low level */
	HANDONLED_DUTY_NONE = 0,

#ifndef HW_V2_CONFIG_ENABLE
/* V8's setting parameters is different from V6, because the LED is driven by low level. */
	HANDONLED_DUTY_INDICATE = 1,
	HANDONLED_DUTY_DETECTION = 50,
#else
	HANDONLED_DUTY_INDICATE_V2 =  1,
	HANDONLED_DUTY_DETECTION_V2 = 30,
	
	HANDONLED_DUTY_INDICATE_V3 = 1,
	HANDONLED_DUTY_DETECTION_V3 = 30,
#endif

};

enum handonled_flashing_period_e{
	HANDONLED_PERIOD_NONE   = 0,		
	HANDONLED_PERIOD_500MS  = 250,
	HANDONLED_PERIOD_1000MS = 500,
	HANDONLED_PERIOD_1500MS = 750,
	HANDONLED_PERIOD_2000MS = 1000,
	HANDONLED_PERIOD_2500MS = 1250,
};	

struct ring_handonled_s 
{
#define  HANDON_LED_ACTION_NONE    (0x00)
#define  HANDON_LED_ACTION_ON      (0x01)
#define  HANDON_LED_ACTION_OFF     (0x02)
	uint8_t visual_action:4;
	uint8_t physics_action:4;

#define HANDON_LED 	NRF_GPIO_PIN_MAP(0, 8) // LED-2mA  /* V3_HW */

	bool using_pwn;
	bool using_timer;

	bool win_10sec;
	bool auto_flashing;
	uint8_t pwm_duty;
	uint16_t flashing_period;
	uint32_t flashing_times;
	uint32_t run_tick_10sec;
};

bool ring_handonled_init(bool create_timer, enum handonled_duty_e duty);
bool ring_handonled_on(void);
void ring_handonled_off(void);
bool ring_handonled_flashing_start(bool win_10s, enum handonled_flashing_period_e flashing_period);
void ring_handonled_flashing_stop(void);

#ifdef __cplusplus  
}
#endif

#endif /* _RING_HANDONLED_H_ */
