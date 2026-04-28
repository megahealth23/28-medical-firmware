/*
* Copyright (c) 2019 MegaHealth. All Rights Reserved.
* @file
* @date 2020.07.29
* @version
* @brief
*/

#include "app_util_platform.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"

#include "ring_bsp.h"

#include "I2C.h"
#include "inner_flash_ram.h"


static uint8_t i2c_status = 0;
static nrf_drv_twi_t twi = NRF_DRV_TWI_INSTANCE(0);

static bool m_xfer_done = false;

#define I2C_RETRY			3
#define MACRO_MS_US(ms)		(ms*1000)

static bool wait_i2c_xfer_timeout(void)
{
	int us = 0;
	while (!m_xfer_done) {
		if (us > MACRO_MS_US(50)) {
			//超时2s刷新看门狗 ？
			if (NRF_WDT->RUNSTATUS & 0x01) {
				NRF_WDT->RR[0] = WDT_RR_RR_Reload;
			}
			return true;
		}
		us += 10;
		nrf_delay_us(10);
	}
	return false;
    return false;
}

/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const* p_event, void* p_context)
{
	switch (p_event->type)
	{
	case NRF_DRV_TWI_EVT_DONE:
		//if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_TX)
		m_xfer_done = true;
		break;

	default:
		break;
	}
}

extern productMacSn pms;
bool i2c_init(void)
{
	uint8_t retry = 0;
	if (0 == i2c_status) {
		ret_code_t rslt;

		nrf_drv_twi_config_t config = {
		   .scl = RING_SCL_PIN,
		   .sda = RING_SDA_PIN,
		   .frequency = NRF_DRV_TWI_FREQ_400K,
		   .interrupt_priority = APP_IRQ_PRIORITY_LOW
		};

        //if(pms.sn.VYM.reserved == 1){
        //    config.sda = RING_SDA_PIN_1;
        //}

		rslt = nrf_drv_twi_init(&twi, &config, twi_handler, NULL);
		while (rslt != NRF_SUCCESS)
		{
			if (retry++ >= I2C_RETRY) {
				return false;
			}
			rslt = nrf_drv_twi_init(&twi, &config, twi_handler, NULL);
		}

		nrf_drv_twi_enable(&twi);

		i2c_status = 1;
	}
	return true;
}


bool i2c_init_sync(void)
{
	uint8_t retry = 0;
	if (0 == i2c_status) {
		ret_code_t rslt;

		nrf_drv_twi_config_t const config = {
		   .scl = RING_SCL_PIN,
		   .sda = RING_SDA_PIN,
		   .frequency = NRF_DRV_TWI_FREQ_400K,
		   .interrupt_priority = APP_IRQ_PRIORITY_LOW
		};

		rslt = nrf_drv_twi_init(&twi, &config, NULL, NULL);
		while (rslt != NRF_SUCCESS)
		{
			if (retry++ >= I2C_RETRY) {
				return false;
			}
			rslt = nrf_drv_twi_init(&twi, &config, NULL, NULL);
		}

		nrf_drv_twi_enable(&twi);

		i2c_status = 1;
	}
	return true;
}

bool i2c_uninit(void)
{
	if (1 == i2c_status) {
		//TODO: release i2c resource
		nrf_drv_twi_uninit(&twi);
		i2c_status = 0;
	}

	return true;
}

//bool i2c_write(uint8_t device_address, uint8_t register_address, uint8_t* value, uint8_t bytes)
ret_code_t i2c_write(uint8_t device_address, uint8_t register_address, uint8_t* value, uint8_t bytes)
{
	uint8_t retry = 0;
	ret_code_t rslt = 1;

	if (i2c_status == 0)
		return rslt;

	m_xfer_done = false;
	//rslt = nrf_drv_twi_tx(&twi, (device_address), value, bytes, false);
	// address  address	Address of a specific slave device (only 7 LSB).

	rslt = nrf_drv_twi_tx(&twi, (device_address), value, bytes, false);
	//APP_ERROR_CHECK(rslt);
	while (rslt != NRF_SUCCESS)
	{
		if (retry++ >= I2C_RETRY)
			break;
		rslt = nrf_drv_twi_tx(&twi, (device_address), value, bytes, false);
	}

	if (rslt != NRF_SUCCESS || wait_i2c_xfer_timeout()) {
		//return false;
		return rslt;
	}

	//return true;
	return rslt;
}

//bool i2c_read(uint8_t device_address, uint8_t register_address, uint8_t* destination, uint8_t bytes)
ret_code_t i2c_read(uint8_t device_address, uint8_t register_address, uint8_t* destination, uint8_t bytes)

{
	uint8_t retry = 0;
	ret_code_t rslt= 1;

	if (i2c_status == 0) {
		return rslt;
	}

	m_xfer_done = false;

	rslt = nrf_drv_twi_tx(&twi, (device_address), &register_address, 1, true);
	//APP_ERROR_CHECK(rslt);
	while (rslt != NRF_SUCCESS)
	{
		if (retry++ >= I2C_RETRY)
			return rslt;
		rslt = nrf_drv_twi_tx(&twi, (device_address), &register_address, 1, true);
	}
	
	if (rslt != NRF_SUCCESS || wait_i2c_xfer_timeout()) {
		//return false;
		return rslt;
	}

	retry = 0;
	m_xfer_done = false;
	rslt = nrf_drv_twi_rx(&twi, (device_address), destination, bytes);
	//APP_ERROR_CHECK(rslt);
	while (rslt != NRF_SUCCESS)
	{
		if (retry++ >= I2C_RETRY) {
			//return false;
			return rslt;
		}
		rslt = nrf_drv_twi_rx(&twi, (device_address), destination, bytes);
	}

	if (rslt != NRF_SUCCESS || wait_i2c_xfer_timeout()) {
		//return false;
		return rslt;
	}
	//return true;
	return rslt;
}
