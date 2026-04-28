/*
* Copyright (c) 2019 MegaHealth. All Rights Reserved.
* @file ring_startup.h
* @author:li tianzheng
* @modifing:
* @date 2020.10.15
* @version
* @brief
*/

#ifndef _RING_STARTUP_H  
#define _RING_STARTUP_H  

#include "ring_power.h"

#include "app_error.h"
#include "nrf_ble_lesc.h"
#include "nrf_pwr_mgmt.h"

#include "system_clock.h"
#include "mTask.h"
#include "ring_bsp.h"

#include "battery\ring_battery.h"
#include "acc\ring_acc.h"
#include "nrf_drv_clock.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "low_pwm.h"
#include "ring_swtimer.h"
#include "ble_rawdata.h"

#include "ring_tmp.h"
#include "ring_afe.h"

#include "I2C.h"
#include "ble_service.h"

#include "protocol.h"

#include <stdbool.h>
#include <stdint.h>
#include "app_error.h"

#include "ble_ring.h"

#include "source\algorithm\DataProcSPO2.h"
#include "source\algorithm\DataProcEHR.h"
#include "source\algorithm\DataProcPTT.h"

#include "HRI.h"

#ifdef __cplusplus  
extern "C"
{
#endif

	typedef uint16_t STATE_TYPE;
	typedef uint16_t STATE_EVT;
	
#define ACC_ISR_WAKE_EVT		0x0001
#define WRIST_ON_WAKE_EVT		0x0002
#define DAILY_RECORD_EVT		0x0004
#define BLE_RTCHECK_EVT			0x0008
#define MONITOR_RECORDS_EVT		0x0010
#define SPORT_RECORDS_EVT		0x0020
#define BP_MEASURE_EVT			0x0040	
#define PULSE_IMP_EVT			0x0080
#define PWR_CHGING_EVT			0x1000
	
	typedef enum {
		IDLE = 0,
		SPO2,
		EHR,
		DAILY,
		RTSHOW,
		BP,
		PULSE,
		TEMP,
		USER,	//8
		GLUCOSE,
		HRV,
		BREATHRATE,
		INVALID_MODE = 255
	} emStatus;

typedef enum {
	MSOFF,
	GLU
} emMultiState;
	
	typedef struct  {
		ring_afe_mode_t afe;
		ring_acc_mode_t imu;
	} mSet_t;

	typedef struct 
	{
		uint32_t	recordRunTime;
		uint32_t	recordUTCtime;
		uint32_t	TempRunTime;

		emStatus mode;
	} sys_state_t;
	
	extern sys_state_t sState;

	extern emMultiState msState;
	

	void app_startup(void);
	void idle_state_handle(void);

#ifdef __cplusplus  
}
#endif  
#endif /* _RING_STARTUP_H */

