/*
* Copyright (c) 2019 MegaHealth. All Rights Reserved.
* @file
* @date 2020.07.29
* @version
* @brief
*/

#include "ring_swtimer.h"
#include "protocol.h"
#include "ble_ring.h"


#define ERROR   0
#define INFO    1
#define LOG_FLASH(log_level, ...) \
do {                                     \
        if (log_level == ERROR)          \
        {                                \
            NRF_LOG_ERROR( __VA_ARGS__); \
        }                                \
        else if (log_level == INFO)      \
        {                                \
            NRF_LOG_INFO( __VA_ARGS__);  \
        }                                \
    } while (0); 


APP_TIMER_DEF(m_pair_timer_id);					/**< ble pair timer. */
APP_TIMER_DEF(m_report_time_id);


extern st_bleRsp_t bleRsp;

static void indi_timeout_handler(void * p_context)
{
	UNUSED_PARAMETER(p_context);

	//NRF_LOG_INFO("indi_timeout_handler");
	ring_indicate(bleRsp.data, 20);

	return;
}

extern void report_timeout_handler(void * p_context);

bool ring_swtimer_init(void)
{
	ret_code_t rslt;
	rslt = app_timer_create(&m_pair_timer_id, APP_TIMER_MODE_REPEATED, PairTimeout_handler);
	APP_ERROR_CHECK(rslt);

	
	rslt = app_timer_create(&m_report_time_id,
								APP_TIMER_MODE_SINGLE_SHOT,
								report_timeout_handler);
	APP_ERROR_CHECK(rslt);

	if(rslt != NRF_SUCCESS)
	{
		NRF_LOG_INFO("Create report timer failed.\n");
		return false;
	}
	return true;
}

/*********************************************************************
* @fn      startPairTimer
*
* @brief   start ble start timer
*
* @param   none
*
* @return  bool.
*/
bool startPairTimer(void)
{
	uint32_t rslt;
	rslt = app_timer_start(m_pair_timer_id, BLE_PAIR_INTERVAL, NULL);
	APP_ERROR_CHECK(rslt);
	if(rslt != NRF_SUCCESS)
	{
		NRF_LOG_INFO("start ble pair timer failed.\n");
		return false;
	}
	return true;
}

/*********************************************************************
* @fn      stopPairTimer
*
* @brief   stop ble pair timer
*
* @param   none
*
* @return  bool.
*/
bool stopPairTimer(void)
{
	uint32_t rslt;
	rslt = app_timer_stop(m_pair_timer_id);
	APP_ERROR_CHECK(rslt);
	if(rslt != NRF_SUCCESS)
	{
		NRF_LOG_INFO("stop ble pair timer failed.\n");
		return false;
	}
	return true;
}

bool startReportTimer(int ms)
{
	uint32_t rslt;
	rslt = app_timer_start(m_report_time_id, APP_TIMER_TICKS(ms), NULL);
	APP_ERROR_CHECK(rslt);
	if(rslt != NRF_SUCCESS)
	{
		NRF_LOG_INFO("start ble pair timer failed.\n");
		return false;
	}
	return true;
}

bool stopReportTimer(void)
{
	uint32_t rslt;
	rslt = app_timer_stop(m_report_time_id);
	APP_ERROR_CHECK(rslt);
	if(rslt != NRF_SUCCESS)
	{
		NRF_LOG_INFO("stop ble pair timer failed.\n");
		return false;
	}
	return true;
}

