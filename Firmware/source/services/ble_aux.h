/*********************************************************************************
 * Copyright (c) 2019 MegaHealth. All Rights Reserved.
 * ble_aux.h
 * AUTHOR:zhao mengshou
 * DATE:2019/6/12 18:28:29
 * http://www.megahealth.cn
 *
 */
#ifndef _BLE_AUX_H  
#define _BLE_AUX_H  

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"

#ifdef __cplusplus  
extern "C"
{
#endif 

	//UID('4999269a-8cfd-11e9-af28-80a589e0081a')

#define BLE_AUX_BLE_OBSERVER_PRIO 2

#define BLE_AUX_SERVICE_UUID            0xFEF0 

#define BLE_AUX_CTRL_CHAR_UUID			0xFEF1 
#define BLE_AUX_DATA_CHAR_UUID			0xFEF2 


/**@brief Nordic vendor-specific base UUID.
 */
#define BLE_AUX_VENDOR_BASE_UUID					\
{{                                                  \
    0x1A, 0x08, 0xE0, 0x89, 0xA5, 0x80, 0x28, 0xAF, \
    0xE9, 0x11, 0xFD, 0x8C, 0x00, 0x00, 0x99, 0x49  \
}}

 /**@brief   Macro for defining a ble_aux instance.
  *
  * @param   _name   Name of the instance.
  * @hideinitializer
  */
#define BLE_AUX_DEF(_name)                      \
static ble_aux_t _name;                         \
NRF_SDH_BLE_OBSERVER(_name ## _obs,             \
                     BLE_AUX_BLE_OBSERVER_PRIO, \
                     ble_aux_on_ble_evt, &_name)

	typedef enum
	{
		BLE_AUX_EVT_NOTIFICATION_ENABLED,		/**< notification enabled event. */
		BLE_AUX_EVT_NOTIFICATION_DISABLED,      /**< notification disabled event. */
	} ble_aux_evt_type_t;

	/**@brief AUX event. */
	typedef struct
	{
		ble_aux_evt_type_t evt_type;            /**< Type of event. */
	} ble_aux_evt_t;

	// Forward declaration of the ble_aux_t type.
	typedef struct ble_aux_s ble_aux_t;

	/**@brief AUX event handler type. */
	typedef void (*ble_aux_evt_handler_t) (ble_aux_t* p_aux, ble_aux_evt_t* p_evt);

	/**@brief AUX init structure. This contains all options and data needed for
	 *        initialization of the service. */
	typedef struct
	{
		ble_aux_evt_handler_t        evt_handler;			/**< Event handler to be called for handling events in the AUX. */
	} ble_aux_init_t;

	/**@brief AUX structure. This contains various status information for the service. */
	struct ble_aux_s
	{
		ble_aux_evt_handler_t        evt_handler;			/**< Event handler to be called for handling events in the AUX. */
		uint8_t                      uuid_type;				/**< UUID type for DFU UUID. */
		bool						 is_notify_enabled;
		uint16_t                     service_handle;
		ble_gatts_char_handles_t     ctrl_handles;			/** write ack character */
		ble_gatts_char_handles_t     data_handles;			/** otify character */
		uint16_t					 conn_handle;			/**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
		uint8_t                      max_aux_len;			/**< Current maximum length, adjusted according to the current ATT MTU. */
	};

	uint32_t	ble_aux_init(ble_aux_t* p_aux, ble_aux_init_t const* p_aux_init);
	void		ble_aux_on_gatt_evt(ble_aux_t* p_aux, nrf_ble_gatt_evt_t const* p_gatt_evt);
	void		ble_aux_on_ble_evt(ble_evt_t const* p_ble_evt, void* p_context);

#ifdef __cplusplus
}
#endif
#endif /* _BLE_AUX_H */


