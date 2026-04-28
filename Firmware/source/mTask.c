/*
* Copyright (c) 2019 MegaHealth. All Rights Reserved.
* @file
* @date 2020.07.29
* @version
* @brief
*/

#include "nrf_queue.h"
#include "nrf_log.h"
#include "mTask.h"

typedef struct _taskProc
{
	uint16_t event;
	asyncCallback cb;
} taskProc;

#define SYSTEM_EVT_POOL_SIZE		64
#define CRITICAL_EVT_POOL_SIZE		8

NRF_QUEUE_DEF(sEvent, m_system_pool, SYSTEM_EVT_POOL_SIZE, NRF_QUEUE_MODE_NO_OVERFLOW);
NRF_QUEUE_DEF(sEvent, m_critical_pool, CRITICAL_EVT_POOL_SIZE, NRF_QUEUE_MODE_NO_OVERFLOW);

static taskProc m_process[MAX_QID - 1] = { 0 };

void taskRegister(uint16_t queueID, asyncCallback taskProc)
{
	if (queueID != 0 && taskProc != NULL)
	{
		if (queueID <= MAX_QID)
		{
			m_process[queueID - 1].event = queueID;
			m_process[queueID - 1].cb = taskProc;
		}
	}
}

void taskUnregister(uint16_t queueID)
{
	if (queueID != 0 && queueID <= MAX_QID)
	{
		m_process[queueID - 1].event = 0;
		m_process[queueID - 1].cb = NULL;
	}
}

static bool task_inited = false;
void taskInit(void)
{
	nrf_queue_reset( &m_system_pool );
	nrf_queue_reset( &m_critical_pool );
	memset((void*)&m_process, 0, sizeof(m_process));
    task_inited = true;
}

ret_code_t taskPush(sEvent* buf, bool critical)
{
    if(task_inited != true){
        return NRF_ERROR_MODULE_NOT_INITIALIZED;
    }

	//NRF_LOG_INFO("TaskPush");
	if (critical) {
		return nrf_queue_push( &m_critical_pool, (void const*)buf );
	}
	else {
		return nrf_queue_push( &m_system_pool, (void const*)buf );
	}

    return NRF_SUCCESS;
}

ret_code_t taskPop(sEvent* buf, bool critical)
{
    if(task_inited != true){
        return NRF_ERROR_MODULE_NOT_INITIALIZED;
    }

	if (critical) {
		return nrf_queue_pop(&m_critical_pool, (void*)buf);
	}
	else {
		return nrf_queue_pop(&m_system_pool, (void*)buf);
	}

    return NRF_SUCCESS;
}

void taskProcess(void)
{
	sEvent handle;
	while (true) {
		/* critical evevt must be handle first */
		while (NRF_SUCCESS == taskPop(&handle, true)) {
            NRF_LOG_INFO("task process critical");
			if (handle.evt > 0) {
				if (m_process[handle.evt - 1].cb) {
					m_process[handle.evt - 1].cb(handle.payload);
				}
			}
		}

		if (NRF_SUCCESS == taskPop(&handle, false)) {
            NRF_LOG_INFO("task process system");
			if (handle.evt > 0) {
                NRF_LOG_INFO("task process system evt: %d", handle.evt);
				if (m_process[handle.evt - 1].cb) {
                    NRF_LOG_INFO("task process system evt: %d succ", handle.evt);
					m_process[handle.evt - 1].cb(handle.payload);
				}
			}
		}
		else {
			if (nrf_queue_is_empty(&m_critical_pool)) {
				break;
			}
		}

        
        wdt_feed();
        idle_state_handle();
	}
}

