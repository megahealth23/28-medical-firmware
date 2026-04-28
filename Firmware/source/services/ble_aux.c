/*
* Copyright (c) 2019 MegaHealth. All Rights Reserved.
* @file
* @date 2020.07.29
* @version
* @brief
*/

#include "sdk_common.h"

#include "ble_aux.h"
#include <string.h>
#include "ble_srv_common.h"

#include "nrf_log.h"
#include "ble_rawdata.h"

#define OPCODE_LENGTH 1																/**< Length of opcode inside AUX packet. */
#define HANDLE_LENGTH 2																/**< Length of handle inside AUX packet. */
#define MAX_AUX_LEN	(NRF_SDH_BLE_GATT_MAX_MTU_SIZE - OPCODE_LENGTH - HANDLE_LENGTH)	/**< Maximum size of a transmitted AUX. */

static uint32_t ble_aux_ctrl_char_add(ble_aux_t* p_aux)
{
	ble_add_char_params_t add_char_params;

	memset(&add_char_params, 0, sizeof(add_char_params));
	add_char_params.uuid = BLE_AUX_CTRL_CHAR_UUID;
	add_char_params.uuid_type = p_aux->uuid_type;
	add_char_params.char_props.write = 1;
	add_char_params.is_var_len = true;
	add_char_params.max_len = MAX_AUX_LEN;

	add_char_params.write_access = SEC_OPEN;

	return characteristic_add(p_aux->service_handle, &add_char_params, &p_aux->ctrl_handles);
}

static uint32_t ble_aux_data_char_add(ble_aux_t* p_aux)
{
	ble_add_char_params_t add_char_params;

	memset(&add_char_params, 0, sizeof(add_char_params));
	add_char_params.uuid = BLE_AUX_DATA_CHAR_UUID;
	add_char_params.uuid_type = p_aux->uuid_type;
	add_char_params.char_props.notify = 1;
	add_char_params.is_var_len = true;
	add_char_params.max_len = MAX_AUX_LEN;

	add_char_params.cccd_write_access = SEC_OPEN;

	return characteristic_add(p_aux->service_handle, &add_char_params, &p_aux->data_handles);
}


/**@brief Function for handling the Connect event.
 *
 * @param[in]	 p_aux		 AUX structure.
 * @param[in]	 p_ble_evt	 Event received from the BLE stack.
 */
static void on_connect(ble_aux_t* p_aux, ble_evt_t const* p_ble_evt)
{
	p_aux->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]	 p_aux		 AUX structure.
 * @param[in]	 p_ble_evt	 Event received from the BLE stack.
 */
static void on_disconnect(ble_aux_t* p_aux, ble_evt_t const* p_ble_evt)
{
	UNUSED_PARAMETER(p_ble_evt);
	p_aux->conn_handle = BLE_CONN_HANDLE_INVALID;
}


/**@brief Function for handling the Write event.
 *
 * @param[in]	 p_aux		 AUX structure.
 * @param[in]	 p_ble_evt	 Event received from the BLE stack.
 */
static void on_write(ble_aux_t* p_aux, ble_evt_t const* p_ble_evt)
{
	ble_gatts_evt_write_t const* p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

	if (p_evt_write->handle == p_aux->data_handles.cccd_handle) {
		if (p_evt_write->len == 2) {
			p_aux->is_notify_enabled = ble_srv_is_notification_enabled(p_evt_write->data);
		}
		NRF_LOG_INFO("aux ctrl point indicate Rawdata Notify= %d \n", p_aux->is_notify_enabled);
//		rawdata_enable = p_aux->is_notify_enabled;
		if( p_aux->is_notify_enabled )
			ring_rawdata_start();
		else
			ring_rawdata_stop();
	}
}

uint32_t ble_aux_init(ble_aux_t* p_aux, ble_aux_init_t const* p_aux_init)
{
	uint32_t err_code;
	ble_uuid_t service_uuid;
	ble_uuid128_t aux_base_uuid = BLE_AUX_VENDOR_BASE_UUID;

	// Initialize service structure
	p_aux->evt_handler = p_aux_init->evt_handler;
	p_aux->is_notify_enabled = false;
	p_aux->conn_handle = BLE_CONN_HANDLE_INVALID;
	p_aux->max_aux_len = MAX_AUX_LEN;


	// Add vendor specific base UUID
	err_code = sd_ble_uuid_vs_add(&aux_base_uuid, &p_aux->uuid_type);
	if (err_code != NRF_SUCCESS) {
		return err_code;
	}

	// Add service
	//BLE_UUID_BLE_ASSIGN(service_uuid, BLE_AUX_SERVICE_UUID);
	service_uuid.type = p_aux->uuid_type;
	service_uuid.uuid = BLE_AUX_SERVICE_UUID;
	err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
		&service_uuid,
		&p_aux->service_handle);

	if (err_code != NRF_SUCCESS) {
		return err_code;
	}

	// Add the ctrl char.
	err_code = ble_aux_ctrl_char_add(p_aux);
	if (err_code != NRF_SUCCESS) {
		return err_code;
	}

	// Add the data char.
	err_code = ble_aux_data_char_add(p_aux);
	if (err_code != NRF_SUCCESS) {
		return err_code;
	}

	return NRF_SUCCESS;
}


void ble_aux_on_gatt_evt(ble_aux_t* p_aux, nrf_ble_gatt_evt_t const* p_gatt_evt)
{
	if ((p_aux->conn_handle == p_gatt_evt->conn_handle)
		&& (p_gatt_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
	{
		p_aux->max_aux_len = p_gatt_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
	}
}


void ble_aux_on_ble_evt(ble_evt_t const* p_ble_evt, void* p_context)
{
	ble_aux_t* p_aux = (ble_aux_t*)p_context;

	switch (p_ble_evt->header.evt_id)
	{
	case BLE_GAP_EVT_CONNECTED:
		on_connect(p_aux, p_ble_evt);
        set_phy_2mbps(p_ble_evt->evt.gap_evt.conn_handle);
		break;

	case BLE_GAP_EVT_DISCONNECTED:
		on_disconnect(p_aux, p_ble_evt);
		break;

	case BLE_GATTS_EVT_WRITE:
		on_write(p_aux, p_ble_evt);
		break;

	case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
		//on_rw_authorize_req(p_ble_evt);
		break;


	default:
		// No implementation needed.
		break;
	}
}

