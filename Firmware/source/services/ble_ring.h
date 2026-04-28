/*********************************************************************************
 * Copyright (c) 2019 MegaHealth. All Rights Reserved.
 * ble_ring.h
 * AUTHOR:zhao mengshou
 * DATE:2019/6/11 17:34:36
 * http://www.megahealth.cn
 *
 */

#ifndef _BLE_RING_H  
#define _BLE_RING_H  

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

//#define DFU_SUPPORT  //defined in 'Options for Target'


#define BLE_RING_BLE_OBSERVER_PRIO 2

#ifdef FDA_CHECK
#define BLE_RING_SERVICE_UUID           0xFAB1 

#define BLE_RING_CTRL_CHAR_UUID         0xFAB2 
#define BLE_RING_INDI_CHAR_UUID         0xFAB4
#define BLE_RING_DATA_CHAR_UUID         0xFAB5

#define BLE_RING_READ_CHAR_UUID         0xFAB6 

#else
#define BLE_RING_SERVICE_UUID           0xFEE0 

#define BLE_RING_CTRL_CHAR_UUID         0xFEE1 
#define BLE_RING_DATA_CHAR_UUID         0xFEE2 
#define BLE_RING_READ_CHAR_UUID         0xFEE3 
#endif

#pragma pack(1)

#define FIRMWARE_MAJOR_VERSION      6
#define FIRMWARE_MINOR_VERSION      1


/**@brief Nordic vendor-specific base UUID.
 */
//DFU 
#define BLE_RING_VENDOR_BASE_UUID                   \
{{                                                  \
    0x1A, 0x08, 0xE0, 0x89, 0xA5, 0x80, 0xEB, 0xB0, \
    0xE9, 0x11, 0x6A, 0x8B, 0x00, 0x00, 0x6A, 0xF8  \
}}


/**@brief   Macro for defining a ble_ring instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_RING_DEF(_name)                         \
	ble_ring_t _name;                            \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                 \
                     BLE_RING_BLE_OBSERVER_PRIO,    \
                     ble_ring_on_ble_evt, &_name)

typedef enum
{
    BLE_RING_EVT_NOTIFICATION_ENABLED,  /**< notification enabled event. */
    BLE_RING_EVT_NOTIFICATION_DISABLED, /**< notification disabled event. */
    BLE_RING_EVT_INDICATION_ENABLED,    /**< indication enabled event. */
    BLE_RING_EVT_INDICATION_DISABLED    /**< indication disabled event. */
} ble_ring_evt_type_t;

/**@brief RING event. */
typedef struct
{
    ble_ring_evt_type_t evt_type;       /**< Type of event. */
} ble_ring_evt_t;

// Forward declaration of the ble_ring_t type.
typedef struct ble_ring_s ble_ring_t;

/**@brief RING event handler type. */
typedef void (*ble_ring_evt_handler_t) (ble_ring_t * p_ring, ble_ring_evt_t * p_evt);

/**@brief RING init structure. This contains all options and data needed for
 *        initialization of the service. */
typedef struct
{
    ble_ring_evt_handler_t		evt_handler;			/**< Event handler to be called for handling events in the RING. */
} ble_ring_init_t;

/**@brief ring structure. This contains various status information for the service. */
struct ble_ring_s
{
	ble_ring_evt_handler_t		evt_handler;			/**< Event handler to be called for handling events in the ring. */
	uint8_t						uuid_type;				/**< UUID type for DFU UUID. */
	bool						is_notify_enabled;
	bool						is_indicate_enabled;
    uint16_t					service_handle;                                       
    ble_gatts_char_handles_t	ctrl_handles;			/** write + indicate ack character */
    ble_gatts_char_handles_t	data_handles;			/** write without response + notify character */ 
    ble_gatts_char_handles_t	read_handles;
	uint16_t					conn_handle;			/**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
	uint8_t						max_ring_len;			/**< Current maximum length, adjusted according to the current ATT MTU. */
};

uint32_t	ble_ring_init(ble_ring_t * p_ring, ble_ring_init_t const * p_ring_init);
void		ble_ring_on_gatt_evt(ble_ring_t * p_ring, nrf_ble_gatt_evt_t const * p_gatt_evt);
void		ble_ring_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);

void on_read(ble_ring_t * p_ring );

/* TMP117:AFE:GS:I2C */
#define 	I2C_DRV_OK 				(0x01<<0)
#define 	GSENSOR_DRV_OK			(0x01<<1)
#define 	AFE_DRV_OK				(0x01<<2)
#define 	TMP117_DRV_OK		(0x01<<3)
#define 	CHGCHIP_DRV_OK		(0x01<<4)
void ring_set_sensor_status(uint8_t status_mask, bool res);
uint8_t ring_get_sensor_status(void);

void ringPairStart(void);

void ringPairError(void);

uint8_t app_pair_rsp(uint8_t type, uint8_t *pValue);

#ifdef __cplusplus
}
#endif
#endif /* _BLE_RING_H */
