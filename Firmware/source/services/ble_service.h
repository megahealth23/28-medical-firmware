/*********************************************************************************
 * Copyright (c) 2019 MegaHealth. All Rights Reserved.
 * ble_service.h
 * AUTHOR:zhao mengshou
 * DATE:2019/6/11 17:34:58
 * http://www.megahealth.cn
 *
 */

#ifndef _BLE_SERVICE_H  
#define _BLE_SERVICE_H  
#include <stdbool.h>

#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"

#include "ble_dfu.h"
#include "nrf_power.h"
#include "nrf_bootloader_info.h"

#ifdef __cplusplus  
extern "C"
{
#endif 

// Service UUID
#define MRINGPROFILE_SERV_UUID						0xFAB1
		
// CHAR UUID
#define MRINGPROFILE_WRITE_CHAR_UUID				0xFAB2
#define MRINGPROFILE_WRITE_NORSP_CHAR_UUID			0xFAB3
#define MRINGPROFILE_INDICATE_CHAR_UUID				0xFAB4
#define MRINGPROFILE_NOTIFY_CHAR_UUID				0xFAB5
#define MRINGPROFILE_READ_CHAR_UUID					0xFAB6

#ifdef FDA_CHECK
#define DEVICE_NAME							"MRingV2"
#else
#define DEVICE_NAME							"C11E"							/**< Name of device. Will be included in the advertising data. */
#endif

#define MANUFACTURER_NAME					"MegaHealth"						/**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL					2500//4000	//500								/**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */

#define APP_ADV_DURATION					0// 18000							/**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define APP_BLE_CONN_CFG_TAG				1									/**< A tag identifying the SoftDevice BLE configuration. */
#define APP_BLE_OBSERVER_PRIO				3									/**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define ADV_RSP_DATA_LEN                    27
#define ADV_IND_SN_LEN                      6
#define ADV_IND_MAC_LEN                     6
#define ADV_IND_MODE_LEN                    1

#define ADV_RING_TYPE                       2
#define ADV_MENUFACTURER                    2
#define ADV_PROTOCOL_VERSION                2

#define MIN_CONN_INTERVAL	MSEC_TO_UNITS(/*400*/7.5, UNIT_1_25_MS)	/**< Minimum acceptable connection interval (0.4 seconds). */
#define MAX_CONN_INTERVAL	MSEC_TO_UNITS(650, UNIT_1_25_MS)           /**< Maximum acceptable connection interval (0.65 second). */
//#define MIN_CONN_INTERVAL	MSEC_TO_UNITS(8, UNIT_1_25_MS)
//#define MAX_CONN_INTERVAL	MSEC_TO_UNITS(12, UNIT_1_25_MS)

#define SLAVE_LATENCY						0									/**< Slave latency. */
#define CONN_SUP_TIMEOUT					MSEC_TO_UNITS(4000, UNIT_10_MS)		/**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY		APP_TIMER_TICKS(5000)				/**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY		APP_TIMER_TICKS(30000)				/**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT		3									/**< Number of attempts before giving up the connection parameter negotiation. */

//APP connect param.
#define ANDROID_BLE_RXTX_FAST_PARAM	{MSEC_TO_UNITS(8.75, UNIT_1_25_MS),	MSEC_TO_UNITS(12.5, UNIT_1_25_MS), 0, MSEC_TO_UNITS(5000, UNIT_10_MS)}
#define ANDROID_BLE_RXTX_MID_PARAM	{MSEC_TO_UNITS(40, UNIT_1_25_MS),	MSEC_TO_UNITS(60, UNIT_1_25_MS), 0, MSEC_TO_UNITS(5000, UNIT_10_MS)}
#define ANDROID_BLE_RXTX_SLOW_PARAM	{MSEC_TO_UNITS(300, UNIT_1_25_MS),	MSEC_TO_UNITS(500, UNIT_1_25_MS), 0, MSEC_TO_UNITS(5000, UNIT_10_MS)}

#define IOS_BLE_RXTX_FAST_PARAM		{MSEC_TO_UNITS(15, UNIT_1_25_MS),	MSEC_TO_UNITS(32.5, UNIT_1_25_MS), 0, MSEC_TO_UNITS(5000, UNIT_10_MS)}
#define IOS_BLE_RXTX_MID_PARAM		{MSEC_TO_UNITS(40, UNIT_1_25_MS),	MSEC_TO_UNITS(60, UNIT_1_25_MS), 0, MSEC_TO_UNITS(5000, UNIT_10_MS)}
#define IOS_BLE_RXTX_SLOW_PARAM		{MSEC_TO_UNITS(300, UNIT_1_25_MS),	MSEC_TO_UNITS(500, UNIT_1_25_MS), 0, MSEC_TO_UNITS(5000, UNIT_10_MS)}

enum mr_conn_speed_t
{
	CONN_NONE,
	CONN_SLOW,
	CONN_MID,
	CONN_IOS_FAST,
	CONN_ANDROID_FAST
};

extern enum mr_conn_speed_t mr_conn_speed;
extern ble_gap_conn_params_t  m_current_conn_params; 
#define LESC_DEBUG_MODE						0									/**< Set to 1 to use LESC debug keys, allows you to use a sniffer to inspect traffic. */

#define SEC_PARAM_BOND						1									/**< Perform bonding. */
#define SEC_PARAM_MITM						0									/**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC						1									/**< LE Secure Connections enabled. */
#define SEC_PARAM_KEYPRESS					0									/**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES			BLE_GAP_IO_CAPS_NONE				/**< No I/O capabilities. */
#define SEC_PARAM_OOB						0									/**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE				7									/**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE				16									/**< Maximum encryption key size. */

#define DEAD_BEEF							0xDEADBEEF				/**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

extern uint8_t ControlNounce[SOC_ECB_CLEARTEXT_LENGTH];
extern uint8_t PRIVATE_PAIR_AES_KEY[SOC_ECB_KEY_LENGTH];


#define MAX_BUFFER_ENTRIES                  20		/**< Number of elements that can be enqueued */
	
		/** Abstracts buffer element */
        #pragma pack(4)
		typedef struct indi_buffer
		{
			uint16_t 	 data_len;	  /**< Total length of data */
			uint8_t    * p_data;	  /**< Indicate? */
		} buffer_entry_t;
        #pragma pack()
		
		STATIC_ASSERT(sizeof(buffer_entry_t) % 4 == 0);
		
		/** Circular buffer list */
		typedef struct
		{
			buffer_entry_t buffer[MAX_BUFFER_ENTRIES]; /**< Maximum number of entries that can enqueued in the list */
			uint8_t 	   rp;						   /**< Index to the read location */
			uint8_t 	   wp;						   /**< Index to write location */
			uint8_t 	   count;					   /**< Number of elements in the list */
		} buffer_list_t;
	
	
		/**Buffer queue access macros
		 *
		 * @{ */
		/** Initialization of buffer list */
#define BUFFER_LIST_INIT()     \
			do						   \
			{						   \
				buffer_list.rp	  = 0; \
				buffer_list.wp	  = 0; \
				buffer_list.count = 0; \
			} while (0)
		
		/** Provide status of data list is full or not */
#define BUFFER_LIST_FULL() \
			((MAX_BUFFER_ENTRIES == buffer_list.count - 1) ? true : false)
		
		/** Provides status of buffer list is empty or not */
#define BUFFER_LIST_EMPTY() \
			((0 == buffer_list.count) ? true : false)
		
#define BUFFER_ELEMENT_INIT(i)                 \
			do										   \
			{										   \
				buffer_list.buffer[(i)].p_data = NULL; \
			} while (0)

	enum mr_pair_evt_t
	{
		MR_PAIR_NONE,
		MR_PAIR_START,
		MR_PAIR_WAITING,
		MR_PAIR_END,
	};

	typedef struct _connect_crtl_t
	{
		uint32_t pairStartTime;
		uint32_t pair_proc_timeout;
		uint8_t wait_isr;	// confirm when new app connect, man action
		uint8_t pair_proc;
		uint8_t isPaired;
		enum mr_pair_evt_t mr_pair_evt;		
	}connect_crtl_t;
	extern volatile connect_crtl_t connect_crtl;

	void ble_service_init(void);
	void advertising_start(bool erase_bonds);
    bool is_adv_stoped();
	uint8_t ring_max_len(void);
	uint8_t aux_max_len(void);
	uint32_t ring_notify(uint8_t* msg, uint16_t len);
	uint32_t aux_raw_notify(uint8_t* msg, uint16_t len);
	uint32_t ring_indicate(uint8_t *ind_data, uint16_t ind_len);
	bool ring_connparam_update(ble_gap_conn_params_t *p_new_params);
	void connpriority_request(enum mr_conn_speed_t conn_speed);
	bool ring_conn_status(void);
	bool ring_disconnect(void);
	void AdvUpdate(void);
	bool isBLEconnected(void);

#ifdef __cplusplus  
}
#endif  
#endif /* _BLE_SERVICE_H */  

