/*
* Copyright (c) 2019 MegaHealth. All Rights Reserved.
* @file ring_power.h
* @author:li tianzheng
* @modifing:
* @date 2020.0.0
* @version
* @brief
*/
#ifndef _RING_POWER_H_
#define _RING_POWER_H_

#ifdef __cplusplus  
extern "C"
{
#endif 

#include "ring_battery.h"
#include "battery_adc.h"

#include "ring_tmp.h"
#include "ring_afe.h"
#include "acc\ring_acc.h"
#include "nrf_drv_gpiote.h"
#include "ring_bsp.h"
#include "I2C.h"
#include "nrf_pwr_mgmt.h"
#include "ring_handonled.h"

#include "ble_gap.h"
#include "ble_service.h"

#include "ble_rawdata.h"
#include "HRI.h"

// 800ms
#define STANDBY_BLE_RXTX_PARAM	{MSEC_TO_UNITS(100, UNIT_1_25_MS),	MSEC_TO_UNITS(800, UNIT_1_25_MS), 0, MSEC_TO_UNITS(5000, UNIT_10_MS)}
#define NORMAL_BLE_RXTX_PARAM		{MIN_CONN_INTERVAL,	MAX_CONN_INTERVAL, SLAVE_LATENCY, CONN_SUP_TIMEOUT}

void ring_systemoff_entry(void);
void ring_ultra_lowpower_entry(void);
void ring_startup_idle(void);
void ring_systemreset_entry(void);
void ring_systemlowpower_entry(void);
void ring_ultra_lowpower_entry(void);

void systemreset_offchip_peripheral(void);

#ifdef __cplusplus  
}
#endif
#endif /* _RING_POWER_H_ */
