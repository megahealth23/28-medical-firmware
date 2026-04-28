/*
* Copyright (c) 2019 MegaHealth. All Rights Reserved.
* @file
* @date 2020.07.29
* @version
* @brief
*/

#include "nrf.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_clock.h"
#include "app_error.h"
#include "system_clock.h"
#include "ring_utc.h"
#include "mTask.h"

#include "nrf_log.h"

#define RTC_PRESCALAR					(4095)//1/8秒 //(2047)	//1/16秒, 每隔2048次计数一次，32768 记录 32次
//#define RTC_PRESCALAR					(31)	//1/1024秒, 每隔32次计数一次，32768 记录 1024次
#define ONE_SEC_CC_VALUE				(32768/(RTC_PRESCALAR+1))	//每秒值

//协议栈启动期间，禁止使用RTC2做毫秒定时器
#ifdef MSEC_FUCNTION_ENABLE
#if (RTC_PRESCALAR == 31)
#error "When SoftDevice enable, rtc tick must be larger than 3ms!"
#endif
#endif

const nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(2);	/**< Declaring an instance of nrf_drv_rtc for RTC0. */

static uint32_t m_system_sec = 0;
static uint32_t m_system_ticks = 0;
static const uint32_t s_system_one_sec_evt = SYS_ONE_SECOND << 24;

//static uint32_t m_system_msec = 0;

/** @brief: Function for handling the RTC0 interrupts.
 * Triggered on TICK and COMPARE0 match.
 */
static void rtc_handler(nrf_drv_rtc_int_type_t int_type)
{
	uint32_t err_code;
	if (int_type == NRF_DRV_RTC_INT_COMPARE0)
	{
		//NRF_LOG_INFO("NRF_DRV_RTC_INT_COMPARE0 and system up %ds or %dms", m_system_sec, m_system_ticks);	
		err_code = nrf_drv_rtc_cc_set(&rtc, 0, ONE_SEC_CC_VALUE, true);
		APP_ERROR_CHECK(err_code);
		nrf_drv_rtc_counter_clear(&rtc);			//Clear the RTC counter to start count from zero
		
		m_system_sec++;
		UTC_clockUpdate();
		
		sEvent evt = { SYS_QID, &s_system_one_sec_evt};
		taskPush(&evt, false);
	}
	else if (int_type == NRF_DRV_RTC_INT_TICK)
	{
		//NRF_LOG_INFO("NRF_DRV_RTC_INT_TICK");
		m_system_ticks++;
		if (m_system_ticks == 0xFFFFFFFF) {
			m_system_ticks = 0;
		}
	}
}

/** @brief Function initialization and configuration of RTC driver instance.
 */
void rtc_start(void)
{
	uint32_t err_code;

	//Initialize RTC instance
	nrf_drv_rtc_config_t config = NRF_DRV_RTC_DEFAULT_CONFIG;
	config.prescaler = RTC_PRESCALAR;
	err_code = nrf_drv_rtc_init(&rtc, &config, rtc_handler);
	APP_ERROR_CHECK(err_code);

	//Enable tick event & interrupt
	//nrf_drv_rtc_tick_enable(&rtc,true);

	//Set compare channel to trigger interrupt after COMPARE_COUNTERTIME seconds
	err_code = nrf_drv_rtc_cc_set(&rtc, 0, ONE_SEC_CC_VALUE, true);
	APP_ERROR_CHECK(err_code);

	//Power on RTC instance
	nrf_drv_rtc_enable(&rtc);
}

void rtc_stop(void)
{
	nrfx_rtc_uninit(&rtc);
}

uint32_t app_get_sec(void)
{
	return UTC_getClock();
	//return m_system_sec;
}

void app_set_sec(uint32_t sec)
{
    UTC_setClock(sec);
	m_system_sec = sec;
}


uint32_t app_get_ticks(void)
{
	return m_system_ticks;
}

#ifdef MSEC_FUCNTION_ENABLE
uint32_t app_get_msec(void)
{
	return m_system_msec;
}
#endif
