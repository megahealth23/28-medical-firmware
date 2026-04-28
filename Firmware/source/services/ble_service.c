/*
 * Copyright (c) 2019 MegaHealth. All Rights Reserved.
 * @file
 * @date 2020.07.29
 * @version
 * @brief
 */

#include "app_error.h"
#include "ble.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_sdm.h"
#include <stdint.h>
#include <string.h>

#include "app_timer.h"
#include "ble_aux.h"
#include "ble_conn_state.h"
#include "ble_ring.h"
#include "fds.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_lesc.h"
#include "nrf_ble_qwr.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"

#include "ble_service.h"
#include "ring_startup.h"
#include "ring_swtimer.h"
#include "system_clock.h"
#include "tools.h"

#include "nrf_log.h"
#include "inner_flash_ram.h"
#include "state_module.h"

NRF_BLE_GATT_DEF(m_gatt); /**< GATT module instance. */

BLE_ADVERTISING_DEF(m_advertising); /**< Advertising module instance. */
BLE_RING_DEF(m_ring);
BLE_AUX_DEF(m_aux);

// private pair parameters
uint8_t PRIVATE_PAIR_AES_KEY[SOC_ECB_KEY_LENGTH] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 'M', 'e', 'g', 'a', '*', '*', '*', '*', '*', '*'};

uint8_t ControlKey[SOC_ECB_KEY_LENGTH] = { // qinkeshi_huanghz
    0x71, 0x69, 0x6e, 0x6b, 0x65, 0x73, 0x68, 0x69, 0x5f, 0x68, 0x75, 0x61, 0x6e, 0x67, 0x68, 0x7a};
//{ 0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6, 0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c };

uint8_t ControlNounce[SOC_ECB_CLEARTEXT_LENGTH] = {0x6b, 0xc1, 0xbe, 0xe2, 0x2e, 0x40,
    0x9f, 0x96, 0xe9, 0x3d, 0x7e, 0x11, 0x73, 0x93, 0x17, 0x2a};

volatile connect_crtl_t connect_crtl;

enum mr_conn_speed_t mr_conn_speed = CONN_NONE;
ble_gap_conn_params_t  m_current_conn_params = {0};    /**< Connection parameters received in the most recent Connect event. */

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID; /**< Handle of the current connection. */

static ble_uuid_t m_adv_uuids[] = {{MRINGPROFILE_SERV_UUID, BLE_UUID_TYPE_BLE}}; /**< Universally unique service identifiers. */

static buffer_list_t buffer_list; /**< List to enqueue not just data to be sent.*/

/**@brief   Function for initializing the buffer queue used to key events that could not be
 *          transmitted
 *
 * @warning This handler is an example only. You need to analyze how you wish to buffer or buffer at
 *          all.
 *
 * @note    A temporary buffering could be employed to handle scenarios
 *          where were no transmit buffers.
 */
static void buffer_init(void) {
  uint32_t buffer_count;

  BUFFER_LIST_INIT();

  for (buffer_count = 0; buffer_count < MAX_BUFFER_ENTRIES; buffer_count++) {
    BUFFER_ELEMENT_INIT(buffer_count);
  }
}

static uint32_t buffer_enqueue(uint8_t *pData, uint8_t len) {
  buffer_entry_t *element;
  uint32_t err_code = NRF_SUCCESS;

  if (BUFFER_LIST_FULL()) {
    // Element cannot be buffered.
    err_code = NRF_ERROR_NO_MEM;
  } else {
    // Make entry of buffer element and copy data.
    element = &buffer_list.buffer[(buffer_list.wp)];
    element->p_data = pData;
    element->data_len = len;

    buffer_list.count++;
    buffer_list.wp++;

    if (buffer_list.wp == MAX_BUFFER_ENTRIES) {
      buffer_list.wp = 0;
    }
  }
  NRF_LOG_INFO("buffer_enqueue err_code=%2X, buffer_list.wp=%d", err_code, buffer_list.wp);

  return err_code;
}

/**@brief   Function to dequeue key scan patterns that could not be transmitted either completely of
 *          partially.
 *
 * @warning This handler is an example only. You need to analyze how you wish to send the key
 *          release.
 *
 * @param[in]  tx_flag   Indicative of whether the dequeue should result in transmission or not.
 * @note       A typical example when all keys are dequeued with transmission is when link is
 *             disconnected.
 *
 * @return     NRF_SUCCESS on success, else an error code indicating reason for failure.
 */
static uint32_t buffer_dequeue(bool tx_flag) {
  buffer_entry_t *p_element;
  ble_ring_t *p_ring = &m_ring;
  uint32_t err_code = NRF_SUCCESS;

  if (BUFFER_LIST_EMPTY()) {
    err_code = NRF_ERROR_NOT_FOUND;
  } else {
    bool remove_element = true; // false;

    p_element = &buffer_list.buffer[(buffer_list.rp)];

    if (tx_flag) {
      // err_code = ring_indicate(p_element->p_data, p_element->data_len);

      if ((m_ring.conn_handle != BLE_CONN_HANDLE_INVALID)) {
        ble_gatts_hvx_params_t hvx_params;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = m_ring.ctrl_handles.value_handle;
        hvx_params.type = BLE_GATT_HVX_INDICATION;
        hvx_params.offset = 0;
        hvx_params.p_len = &(p_element->data_len);
        hvx_params.p_data = p_element->p_data;

        do {
          err_code = sd_ble_gatts_hvx(p_ring->conn_handle, &hvx_params);
          if (err_code == NRF_SUCCESS) {
            remove_element = true;
          }
        } while (err_code == NRF_ERROR_BUSY);
      }
    }

    if (remove_element) {
      BUFFER_ELEMENT_INIT(buffer_list.rp);

      buffer_list.rp++;
      buffer_list.count--;

      if (buffer_list.rp == MAX_BUFFER_ENTRIES) {
        buffer_list.rp = 0;
      }
    }
  }

  return err_code;
}

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t *p_file_name) {
  app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void) {
  ret_code_t err_code;
  ble_gap_conn_params_t gap_conn_params;
  ble_gap_conn_sec_mode_t sec_mode;

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

/*
SKIP_FT is setting to true,
default: 0 <==>  (!SKIP_FT_STEP)
*/
//#if !SKIP_FT_STEP
#if (!SKIP_FT_STEP) && (false)
  if (!isFTdone())
    err_code = sd_ble_gap_device_name_set(&sec_mode,
        (const uint8_t *)("MR\xE4\xBA\xA7\xE6\xB5\x8B"),
        strlen("MR\xE4\xBA\xA7\xE6\xB5\x8B"));
  else
#endif
    err_code = sd_ble_gap_device_name_set(&sec_mode,
        (const uint8_t *)DEVICE_NAME,
        strlen(DEVICE_NAME));

  APP_ERROR_CHECK(err_code);

  err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_HEART_RATE_SENSOR_HEART_RATE_BELT);
  APP_ERROR_CHECK(err_code);

  memset(&gap_conn_params, 0, sizeof(gap_conn_params));

  gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
  gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
  gap_conn_params.slave_latency = SLAVE_LATENCY;
  gap_conn_params.conn_sup_timeout = CONN_SUP_TIMEOUT;

  err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
  APP_ERROR_CHECK(err_code);

#if 0
	//Enable connection event extension
	ble_opt_t  opt;
	memset(&opt, 0x00, sizeof(opt));
	opt.common_opt.conn_evt_ext.enable = true;
	err_code = sd_ble_opt_set(BLE_COMMON_OPT_CONN_EVT_EXT, &opt);
	APP_ERROR_CHECK(err_code);
#endif
}

/**@brief GATT module event handler.
 */
static void gatt_evt_handler(nrf_ble_gatt_t *p_gatt, nrf_ble_gatt_evt_t const *p_evt) {
  if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED) {
    NRF_LOG_INFO("GATT ATT MTU on connection 0x%x changed to %d.",
        p_evt->conn_handle,
        p_evt->params.att_mtu_effective);
  }

  ble_ring_on_gatt_evt(&m_ring, p_evt);
  ble_aux_on_gatt_evt(&m_aux, p_evt);
}

/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void) {
  ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
  APP_ERROR_CHECK(err_code);
}

static void ble_dfu_evt_handler(ble_dfu_buttonless_evt_type_t event) {
  switch (event) {
  case BLE_DFU_EVT_BOOTLOADER_ENTER_PREPARE:
    NRF_LOG_INFO("Device is preparing to enter bootloader mode.");
    // YOUR_JOB: Disconnect all bonded devices that currently are connected.
    //           This is required to receive a service changed indication
    //           on bootup after a successful (or aborted) Device Firmware Update.
    uint8_t *pFlag = "NIRA";
    extern NIRAM_t NIramCfg;
    bool rslt;
    uint32_t last_charge_time = NIramCfg.last_charge_time;
    uint8_t last_charge_percent = NIramCfg.last_charge_percent;
    uint8_t charged_percent = NIramCfg.charged_percent;
    memset(&NIramCfg, 0, sizeof(NIRAM_t));
    memcpy(&NIramCfg.NIflag[0], pFlag, 4);

    NIramCfg.systme_sec = app_get_sec();
    NIramCfg.utc_sec = UTC_getClock();
    NIramCfg.last_charge_time = last_charge_time;
    NIramCfg.last_charge_percent = last_charge_percent;
    NIramCfg.charged_percent = charged_percent;

    systemreset_offchip_peripheral();
    break;

  case BLE_DFU_EVT_BOOTLOADER_ENTER:
    // YOUR_JOB: Write app-specific unwritten data to FLASH, control finalization of this
    //           by delaying reset by reporting false in app_shutdown_handler
    NRF_LOG_INFO("Device will enter bootloader mode.");
    break;

  case BLE_DFU_EVT_BOOTLOADER_ENTER_FAILED:
    NRF_LOG_ERROR("Request to enter bootloader mode failed asynchroneously.");
    // YOUR_JOB: Take corrective measures to resolve the issue
    //           like calling APP_ERROR_CHECK to reset the device.
    break;

  case BLE_DFU_EVT_RESPONSE_SEND_ERROR:
    NRF_LOG_ERROR("Request to send a response to client failed.");
    // YOUR_JOB: Take corrective measures to resolve the issue
    //           like calling APP_ERROR_CHECK to reset the device.
    APP_ERROR_CHECK(false);
    break;

  default:
    NRF_LOG_ERROR("Unknown event from ble_dfu_buttonless.");
    break;
  }
}

/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the Heart Rate, Battery and Device Information services.
 */
void services_init(void) {
  ret_code_t err_code;
  ble_ring_init_t ring_init = {0};
  ble_aux_init_t aux_init = {0};

  memset(&ring_init, 0, sizeof(ring_init));

  ring_init.evt_handler = NULL;

  err_code = ble_ring_init(&m_ring, &ring_init);
  APP_ERROR_CHECK(err_code);

  memset(&aux_init, 0, sizeof(aux_init));

  aux_init.evt_handler = NULL;

  err_code = ble_aux_init(&m_aux, &aux_init);
  APP_ERROR_CHECK(err_code);

  ble_dfu_buttonless_init_t dfus_init = {0};
  // Initialize the async SVCI interface to bootloader.
  err_code = ble_dfu_buttonless_async_svci_init();

  if (NRF_SUCCESS == err_code) {
    dfus_init.evt_handler = ble_dfu_evt_handler;
    err_code = ble_dfu_buttonless_init(&dfus_init);
    APP_ERROR_CHECK(err_code);
  }

  buffer_init();
}

/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t *p_evt) {
  ret_code_t err_code;

  if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED) {
    err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
    APP_ERROR_CHECK(err_code);
  }
}

/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error) {
  APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for initializing the Connection Parameters module.
 */
void conn_params_init(void) {
  ret_code_t err_code;
  ble_conn_params_init_t cp_init;

  memset(&cp_init, 0, sizeof(cp_init));

  cp_init.p_conn_params = NULL;
  cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
  cp_init.next_conn_params_update_delay = NEXT_CONN_PARAMS_UPDATE_DELAY;
  cp_init.max_conn_params_update_count = MAX_CONN_PARAMS_UPDATE_COUNT;
  // cp_init.start_on_notify_cccd_handle    = m_hrs.hrm_handles.cccd_handle;
  cp_init.disconnect_on_fail = false;
  cp_init.evt_handler = on_conn_params_evt;
  cp_init.error_handler = conn_params_error_handler;

  err_code = ble_conn_params_init(&cp_init);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt) {
  // ret_code_t err_code;

  switch (ble_adv_evt) {
  case BLE_ADV_EVT_FAST:
    // NRF_LOG_INFO("Fast advertising.");
    break;
  case BLE_ADV_EVT_SLOW:
    NRF_LOG_INFO("Slow advertising.");
    break;

  case BLE_ADV_EVT_IDLE:
    // sleep_mode_enter();
    break;

  default:
    break;
  }
}

void ringPairStart() {
  memset((void *)&connect_crtl, 0, sizeof(connect_crtl));

  connect_crtl.mr_pair_evt = MR_PAIR_START;
  connect_crtl.pairStartTime = app_get_sec();
  NRF_LOG_INFO("Connect start, pairStartTime=%d \r\n", connect_crtl.pairStartTime);
  startPairTimer();
  connect_crtl.pair_proc_timeout = 30; // 10;
  connect_crtl.wait_isr = 0;

#ifdef FDA_CHECK
  connect_crtl.wait_isr = 1;
#endif

  connect_crtl.pair_proc = 0;
}
void ringPairError(void) {
  if (m_ring.conn_handle != BLE_CONN_HANDLE_INVALID) {
    uint32_t err_code = sd_ble_gap_disconnect(m_ring.conn_handle,
        BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    APP_ERROR_CHECK(err_code);
  }
  stopPairTimer();
  connect_crtl.mr_pair_evt = MR_PAIR_NONE;
}

#define RADIO_TX_POWER 8
typedef enum BLE_GAP_TX_POWER_ROLES eRoles;

static void tx_power_set(eRoles r, int power) {
  ret_code_t err_code;
  err_code = sd_ble_gap_tx_power_set(r /*BLE_GAP_TX_POWER_ROLE_ADV , BLE_GAP_TX_POWER_ROLE_CONN*/,
      m_advertising.adv_handle, power);
  //APP_ERROR_CHECK(err_code);
}

static volatile bool m_trans_queue_full = false;
extern uint32_t s_sys_connect_evt;

void set_phy_2mbps(uint16_t conn_handle) {
    ble_gap_phys_t phys = {0};
    phys.tx_phys = BLE_GAP_PHY_2MBPS;
    phys.rx_phys = BLE_GAP_PHY_2MBPS; 

    ret_code_t err_code = sd_ble_gap_phy_update(conn_handle, &phys);
    if (err_code != NRF_SUCCESS) {
        NRF_LOG_ERROR("Failed to set PHY: 0x%x", err_code);
    }
}

bool bStopAdv = false;

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const *p_ble_evt, void *p_context) {
    ret_code_t err_code;

    switch (p_ble_evt->header.evt_id) {
    case BLE_GAP_EVT_CONNECTED:{
        NRF_LOG_INFO("Connected.");
        m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
        tx_power_set(BLE_GAP_TX_POWER_ROLE_CONN, RADIO_TX_POWER);
        set_phy_2mbps(p_ble_evt->evt.gap_evt.conn_handle);

#ifdef FDA_CHECK
    on_read(&m_ring);
#else
    on_read(NULL);
#endif

        ringPairStart();
        connect_crtl.isPaired = false;

        s_sys_connect_evt = (SYS_MODULE << 24) | (SYS_BLE_CONNECT<<16);
        sEvent evt = { SYS_QID, &s_sys_connect_evt};
        taskPush(&evt, false);
    }break;

    case BLE_GAP_EVT_DISCONNECTED:{
        NRF_LOG_INFO("Disconnected, reason %2X.", p_ble_evt->evt.gap_evt.params.disconnected.reason);
        m_conn_handle = BLE_CONN_HANDLE_INVALID;

        extern void advertising_stop();
        if (bStopAdv == true) // Switch off BLE.
            advertising_stop();

        // Dequeue all data without transmission.
        (void)buffer_dequeue(false);

        s_sys_connect_evt = (SYS_MODULE << 24) | (SYS_BLE_DISCONNECT<<16);
        sEvent evt = { SYS_QID, &s_sys_connect_evt};
        taskPush(&evt, false);

        connect_crtl.isPaired = false;
    }break;

#if 0
    case BLE_GATTS_EVT_HVN_TX_COMPLETE: //BLE_GATTC_EVT_HVX: //
		// Send next indi event
		NRF_LOG_INFO("BLE_GATTS_EVT_HVN_TX_COMPLETE!");
		m_trans_queue_full = false;
	
		(void) buffer_dequeue(true);
        break;
#endif

    case BLE_GAP_EVT_PHY_UPDATE_REQUEST: {
        NRF_LOG_INFO("PHY update request.");
        ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
        err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
        APP_ERROR_CHECK(err_code);
    } break;

    case BLE_GATTC_EVT_TIMEOUT:
        // Disconnect on GATT Client timeout event.
        NRF_LOG_INFO("GATT Client Timeout.");
        err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
            BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
    break;

    case BLE_GATTS_EVT_TIMEOUT:
        // Disconnect on GATT Server timeout event.
        NRF_LOG_INFO("GATT Server Timeout.");
        err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
            BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
    break;

    case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
        NRF_LOG_INFO("BLE_GAP_EVT_SEC_PARAMS_REQUEST");
    break;

    case BLE_GAP_EVT_AUTH_KEY_REQUEST:
        NRF_LOG_INFO("BLE_GAP_EVT_AUTH_KEY_REQUEST");
    break;

    case BLE_GAP_EVT_LESC_DHKEY_REQUEST:
        NRF_LOG_INFO("BLE_GAP_EVT_LESC_DHKEY_REQUEST");
    break;

    case BLE_GAP_EVT_AUTH_STATUS:
        NRF_LOG_INFO("BLE_GAP_EVT_AUTH_STATUS: status=0x%x bond=0x%x lv4: %d kdist_own:0x%x kdist_peer:0x%x",
            p_ble_evt->evt.gap_evt.params.auth_status.auth_status,
            p_ble_evt->evt.gap_evt.params.auth_status.bonded,
            p_ble_evt->evt.gap_evt.params.auth_status.sm1_levels.lv4,
            *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_own),
            *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_peer));
    break;

    case BLE_GAP_EVT_CONN_PARAM_UPDATE:
        memcpy(&m_current_conn_params, &(p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params),sizeof(m_current_conn_params));

        NRF_LOG_INFO("BLE_GAP_EVT_CONN_PARAM_UPDATE: %d %d %d %d",
            m_current_conn_params.min_conn_interval,
            m_current_conn_params.max_conn_interval,
            m_current_conn_params.slave_latency,
            m_current_conn_params.conn_sup_timeout);
    break;
    // DLE
    case BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST: {
        NRF_LOG_INFO("DLE update request.");
        ble_gap_data_length_params_t dle_param;
        memset(&dle_param, 0, sizeof(ble_gap_data_length_params_t)); // 0 means auto select DLE
        err_code = sd_ble_gap_data_length_update(p_ble_evt->evt.gap_evt.conn_handle, &dle_param, NULL);
        APP_ERROR_CHECK(err_code);
    } break;

    default:
        // No implementation needed.
    break;
    }
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
#define MAX_LEN_DEVICE_NAME 12

static void ble_stack_init(void) {
    ret_code_t err_code;

    // extern productMacSn* g_pPMS;
    extern productMacSn pms;

    ble_gap_addr_t gap_addr;
    // Check if not written
    uint8_t empty_addr[BLE_GAP_ADDR_LEN];
    memset(empty_addr, 0x00, BLE_GAP_ADDR_LEN);
    uint8_t full_addr[BLE_GAP_ADDR_LEN];
    memset(full_addr, 0xFF, BLE_GAP_ADDR_LEN);

    gap_addr.addr_id_peer = 0;
    gap_addr.addr_type = BLE_GAP_ADDR_TYPE_PUBLIC;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // if(!memcmp( g_pPMS->mac, empty_addr, BLE_GAP_ADDR_LEN))
    if (!memcmp(pms.mac, empty_addr, BLE_GAP_ADDR_LEN) || !memcmp(pms.mac, full_addr, BLE_GAP_ADDR_LEN)) {
        NRF_LOG_WARNING("No BD address in flash, using chip default");
        err_code = sd_ble_gap_addr_get(&gap_addr);
        APP_ERROR_CHECK(err_code);
        NRF_LOG_HEXDUMP_INFO(&gap_addr.addr, 6);
    } else {
        // memcpy(&gap_addr.addr, mac_ntoh(g_pPMS->mac), BLE_GAP_ADDR_LEN);
        memcpy(&gap_addr.addr, mac_ntoh(&pms.mac[0]), BLE_GAP_ADDR_LEN);

        NRF_LOG_INFO("GAP Address");
        NRF_LOG_HEXDUMP_INFO(&gap_addr.addr, 6);
    }
    NRF_LOG_INFO("Setting ble address to: %02X:%02X:%02X:%02X:%02X:%02X",
        gap_addr.addr[5],
        gap_addr.addr[4],
        gap_addr.addr[3],
        gap_addr.addr[2],
        gap_addr.addr[1],
        gap_addr.addr[0]);

    err_code = sd_ble_gap_addr_set(&gap_addr);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

/**@brief Function for application main entry.
 */
void ble_service_init(void) {
    ble_stack_init();
    gap_params_init();
    gatt_init();

    // advertising_init();
    AdvUpdate();

    services_init();
    conn_params_init();
  //	peer_manager_init();
}

void ble_power_update(){
    tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, RADIO_TX_POWER);
    tx_power_set(BLE_GAP_TX_POWER_ROLE_CONN, RADIO_TX_POWER);
    tx_power_set(BLE_GAP_TX_POWER_ROLE_SCAN_INIT, RADIO_TX_POWER);
}

extern uint8_t air_plane_mode;
static ble_advdata_manuf_data_t adv_ind_manuf_data;
static ble_advdata_manuf_data_t adv_scanrsp_manuf_data;
void AdvUpdate() {
    NRF_LOG_INFO("AdvUpdate!!!");
    if(UTC_getClock() % 3 != 0)
        return;
    if(air_plane_mode == 1)
        return;
    if (bStopAdv == true)
        return;

    if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
        return;

    ret_code_t rslt;
    extern productMacSn pms;
    static ble_advertising_t *pBleAdv = &m_advertising;
    static uint8_t adv_mac_sn[ADV_IND_MAC_LEN+ADV_IND_SN_LEN+ADV_IND_MODE_LEN] = {0};
    ble_advertising_init_t init;
    static uint8_t scanrsp_manuf_data_data[ADV_RSP_DATA_LEN] = {0};
    static uint8_array_t scanrsp_manuf_data_array;



/******Advertising Indications Manufacturer Data Set****************/
    adv_ind_manuf_data.company_identifier = (ADV_RING_TYPE << 8) | ADV_MENUFACTURER;
    adv_ind_manuf_data.data.size = ADV_IND_MAC_LEN+ADV_IND_SN_LEN+ADV_IND_MODE_LEN;

    memcpy(adv_mac_sn, mac_ntoh(&pms.mac[0]), ADV_IND_MAC_LEN);
    memcpy(&adv_mac_sn[ADV_IND_MAC_LEN], &(pms.sn), ADV_IND_SN_LEN); // 6BYTE SN
    adv_mac_sn[ADV_IND_MAC_LEN+ADV_IND_SN_LEN] = NIramCfg.cur_module;

    adv_ind_manuf_data.data.p_data = adv_mac_sn;
/******Advertising Indications Manufacturer Data Set****************/

/******Advertising Scan Response Manufacturer Data Set****************/
    adv_scanrsp_manuf_data.company_identifier = (ADV_RING_TYPE << 8) | ADV_MENUFACTURER;

    uint32_t _now = UTC_getClock();
    scanrsp_manuf_data_data[0] = ADV_PROTOCOL_VERSION;
    scanrsp_manuf_data_data[1] = (_now >> 24) & 0xFF;
    scanrsp_manuf_data_data[2] = (_now >> 16) & 0xFF;
    scanrsp_manuf_data_data[3] = (_now >> 8) & 0xFF;
    scanrsp_manuf_data_data[4] = (_now >> 0) & 0xFF;
    scanrsp_manuf_data_data[5] = get_batt_power_status();
    if(cur_module_type() == e_CHARG){
        if(get_batt_power_status() != DEV_CHGFULL)
            scanrsp_manuf_data_data[5] = DEV_CHGING;
    }
    scanrsp_manuf_data_data[6] = get_batt_vol_percent();
    set_adv_scan_rsp_data(&scanrsp_manuf_data_data[7]);

    scanrsp_manuf_data_array.p_data = scanrsp_manuf_data_data;
    scanrsp_manuf_data_array.size = sizeof(scanrsp_manuf_data_data);

    adv_scanrsp_manuf_data.data = scanrsp_manuf_data_array;
/******Advertising Scan Response Manufacturer Data Set****************/

    if (m_advertising.initialized == true)
        sd_ble_gap_adv_stop(m_advertising.adv_handle);

    memset(&init, 0, sizeof(init));
    init.advdata.name_type = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids = m_adv_uuids;

    init.config.ble_adv_fast_enabled = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout = APP_ADV_DURATION;

    init.advdata.p_manuf_specific_data = &adv_ind_manuf_data;
    init.srdata.p_manuf_specific_data = &adv_scanrsp_manuf_data;
    init.evt_handler = on_adv_evt;

    m_advertising.adv_data.adv_data.len = BLE_GAP_ADV_SET_DATA_SIZE_MAX;
    rslt = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(rslt);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);

    rslt = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(rslt);

}

bool isBLEconnected(void) {
    if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
        return true;
    return false;
}

/**@brief Function for starting advertising.
 */
void advertising_start(bool erase_bonds) {
    bStopAdv = false;
    return;
}

bool is_adv_stoped(){
    return bStopAdv;
}

void advertising_stop() {
    ret_code_t rslt;
    if (m_advertising.initialized == true)
        rslt = sd_ble_gap_adv_stop(m_advertising.adv_handle);
    bStopAdv = true;
    NRF_LOG_INFO("sd_ble_gap_adv_stop=%d", rslt);
}

uint8_t ring_max_len(void) {
  if (m_ring.conn_handle != BLE_CONN_HANDLE_INVALID) {
    return m_ring.max_ring_len;
  } else {
    return 0;
  }
}

uint8_t aux_max_len(void) {
  if (m_aux.conn_handle != BLE_CONN_HANDLE_INVALID) {
    return m_aux.max_aux_len;
  } else {
    return 0;
  }
}

uint32_t ring_indicate(uint8_t *ind_data, uint16_t ind_len) {
  uint32_t err_code = NRF_ERROR_INVALID_STATE;
  ble_ring_t *p_ring = &m_ring;

  // Send value if connected and notifying.

  	NRF_LOG_INFO("ind_data[0]=0x%2X, ind_data[2]=0x%2X, ind_data[3]=0x%2X,  ind_data[4]=0x%2X",
  		ind_data[0], ind_data[2], ind_data[3], ind_data[4]);

  if ((p_ring->conn_handle != BLE_CONN_HANDLE_INVALID)) {

    ble_gatts_hvx_params_t hvx_params;

    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = p_ring->ctrl_handles.value_handle;
    hvx_params.type = BLE_GATT_HVX_INDICATION;
    hvx_params.offset = 0;
    hvx_params.p_len = &ind_len;
    hvx_params.p_data = ind_data;

    // err_code = sd_ble_gatts_hvx(p_ring->conn_handle, &hvx_params);
    //		NRF_LOG_INFO("ring_indicate err_code=%d, ind_data[0]=%2X, ind_data[2]=%2X, len=%d",
    //			err_code, ind_data[0], ind_data[2], *(hvx_params.p_len));

    do {
      err_code = sd_ble_gatts_hvx(p_ring->conn_handle, &hvx_params);
      // m_trans_queue_full = true;
      /*
      if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY)) {
              buffer_enqueue(ind_data, ind_len);
      }
      */
    } while (err_code == NRF_ERROR_BUSY);

  } else {
    err_code = NRF_ERROR_INVALID_STATE;
  }

  return err_code;
}

uint32_t ring_notify(uint8_t *msg, uint16_t len) {
  uint32_t err_code;

  ble_ring_t *p_ring = &m_ring;

  if ((p_ring->conn_handle != BLE_CONN_HANDLE_INVALID) && (p_ring->is_notify_enabled == true)) {

    uint16_t hvx_len;
    ble_gatts_hvx_params_t hvx_params;
    hvx_len = len;

    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = p_ring->data_handles.value_handle;
    hvx_params.type = BLE_GATT_HVX_NOTIFICATION;
    hvx_params.offset = 0;
    hvx_params.p_len = &hvx_len;
    hvx_params.p_data = msg;

    err_code = sd_ble_gatts_hvx(p_ring->conn_handle, &hvx_params);
    if ((err_code == NRF_SUCCESS) && (hvx_len != len)) {
      err_code = NRF_ERROR_DATA_SIZE;
    }
    if (err_code != NRF_SUCCESS) {
      // NRF_LOG_INFO("sd_ble_gatts_hvx err_code = %4x",err_code);
    }
  } else {
    err_code = NRF_ERROR_INVALID_STATE;
  }

  return err_code;
}

uint32_t aux_raw_notify(uint8_t *msg, uint16_t len) {
  uint32_t err_code;

  ble_aux_t *p_aux = &m_aux;

  if ((p_aux->conn_handle != BLE_CONN_HANDLE_INVALID) && (p_aux->is_notify_enabled == true)) {

    uint16_t hvx_len;
    ble_gatts_hvx_params_t hvx_params;
    hvx_len = len;

    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = p_aux->data_handles.value_handle;
    hvx_params.type = BLE_GATT_HVX_NOTIFICATION;
    hvx_params.offset = 0;
    hvx_params.p_len = &hvx_len;
    hvx_params.p_data = msg;

    err_code = sd_ble_gatts_hvx(p_aux->conn_handle, &hvx_params);
    if ((err_code == NRF_SUCCESS) && (hvx_len != len)) {
      err_code = NRF_ERROR_DATA_SIZE;
    }
  } else {
    err_code = NRF_ERROR_INVALID_STATE;
  }

  return err_code;
}

bool ring_connparam_update(ble_gap_conn_params_t *p_new_params) {
  if (BLE_CONN_HANDLE_INVALID == m_conn_handle)
    return false;
  if (NRF_SUCCESS == ble_conn_params_change_conn_params(m_conn_handle, p_new_params))
    return true;
  else
    return false;
}

void connpriority_request(enum mr_conn_speed_t conn_speed){

  static int8_t retry_time = 3;

  ble_gap_conn_params_t conn_param = {0};
  memcpy(&conn_param, &m_current_conn_params, sizeof(m_current_conn_params));
  NRF_LOG_INFO("old conn_params, max_interval:%d, min_interval:%d, slave_latency:%d, timeout:%d\r\n",
					conn_param.max_conn_interval, conn_param.min_conn_interval,
					conn_param.slave_latency, conn_param.conn_sup_timeout);

  switch (conn_speed)
  {
     case CONN_SLOW:
	if (conn_param.min_conn_interval < MSEC_TO_UNITS(300, UNIT_1_25_MS)
		|| conn_param.min_conn_interval > MSEC_TO_UNITS(500, UNIT_1_25_MS)) {
            NRF_LOG_INFO("new conn_params: CONN_SLOW");
            if (--retry_time >= 0) {
               ble_gap_conn_params_t gpt = ANDROID_BLE_RXTX_SLOW_PARAM;
               ring_connparam_update(&gpt);	
             }
         } else {
             retry_time = 3;
	 }
         break;

      case CONN_MID:
          if (conn_param.min_conn_interval < MSEC_TO_UNITS(40, UNIT_1_25_MS)
                        || conn_param.min_conn_interval > MSEC_TO_UNITS(60, UNIT_1_25_MS)) {
                                    NRF_LOG_INFO("new conn_params: CONN_MID");
	      if (--retry_time > 0) {
	        ble_gap_conn_params_t gpt = ANDROID_BLE_RXTX_MID_PARAM;
		ring_connparam_update(&gpt);	
	      }
	   } else {
		retry_time = 3;
	   }
	   break;

       case CONN_ANDROID_FAST:
	   if (conn_param.min_conn_interval < MSEC_TO_UNITS(10, UNIT_1_25_MS)
			  || conn_param.min_conn_interval > MSEC_TO_UNITS(12.5, UNIT_1_25_MS)) {
                                      NRF_LOG_INFO("new conn_params: CONN_ANDROID_FAST");
	       if (--retry_time > 0) {
		    ble_gap_conn_params_t gpt = ANDROID_BLE_RXTX_FAST_PARAM;
		    ring_connparam_update(&gpt);	
	       }					
	   } else {
	      retry_time = 3;
	  }
	  break;

      case CONN_IOS_FAST:
         if (conn_param.min_conn_interval < MSEC_TO_UNITS(15, UNIT_1_25_MS)
		|| conn_param.min_conn_interval > MSEC_TO_UNITS(32.5, UNIT_1_25_MS)) {
                            NRF_LOG_INFO("new conn_params: CONN_IOS_FAST");
             if (--retry_time > 0) {
		ble_gap_conn_params_t gpt = IOS_BLE_RXTX_FAST_PARAM;
		ring_connparam_update(&gpt);	
	    }					
	} else {
	    retry_time = 3;
	}
	break;

     default:
	retry_time = 3;
        break;
				
  }
  retry_time = (retry_time <= 0) ? 3 : retry_time;
}

bool ring_conn_status(void) {
  if (BLE_CONN_HANDLE_INVALID == m_conn_handle)
    return false;
  else
    return true;
}

bool ring_disconnect(void) {
  if (BLE_CONN_HANDLE_INVALID == m_conn_handle)
    return false;
  if (NRF_SUCCESS == sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE))
    return true;
  else
    return false;
}