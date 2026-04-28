/*
 * Copyright (c) 2019 MegaHealth. All Rights Reserved.
 * @file
 * @date 2020.07.29
 * @version
 * @brief
 */

#include "ble_ring.h"
#include "ble_service.h"
#include "ble_srv_common.h"
#include "nrf_drv_rng.h"
#include "sdk_common.h"
#include <string.h>
#include "inner_flash_ram.h"
#include "nrf_dfu.h"

#include "nrf_log.h"
#include "protocol.h"
#include "tools.h"

#define OPCODE_LENGTH 1                                                              /**< Length of opcode inside RING packet. */
#define HANDLE_LENGTH 2                                                              /**< Length of handle inside RING packet. */
#define MAX_RING_LEN (NRF_SDH_BLE_GATT_MAX_MTU_SIZE - OPCODE_LENGTH - HANDLE_LENGTH) /**< Maximum size of a transmitted RING. */

static uint32_t ble_ring_ctrl_char_add(ble_ring_t *p_ring) {
  ble_add_char_params_t add_char_params;

  memset(&add_char_params, 0, sizeof(add_char_params));
  add_char_params.uuid = BLE_RING_CTRL_CHAR_UUID;
  add_char_params.uuid_type = p_ring->uuid_type;

#ifndef FDA_CHECK
  add_char_params.char_props.indicate = 1;
#endif
  add_char_params.char_props.write = 1;
  // add_char_params.is_defered_write    = true;
  add_char_params.is_var_len = true;
  add_char_params.max_len = MAX_RING_LEN;

  add_char_params.cccd_write_access = SEC_OPEN;
  add_char_params.write_access = SEC_OPEN;

  return characteristic_add(p_ring->service_handle, &add_char_params, &p_ring->ctrl_handles);
}

#ifdef FDA_CHECK
static uint32_t ble_ring_indi_char_add(ble_ring_t *p_ring) {
  ble_add_char_params_t add_char_params;

  memset(&add_char_params, 0, sizeof(add_char_params));
  add_char_params.uuid = BLE_RING_INDI_CHAR_UUID;
  add_char_params.uuid_type = p_ring->uuid_type;
  add_char_params.char_props.indicate = 1;
  add_char_params.char_props.write = 0;
  add_char_params.is_var_len = true;
  add_char_params.max_len = MAX_RING_LEN;

  add_char_params.cccd_write_access = SEC_OPEN;
  add_char_params.write_access = SEC_OPEN;

  return characteristic_add(p_ring->service_handle, &add_char_params, &p_ring->ctrl_handles);
}
#endif

static uint32_t ble_ring_data_char_add(ble_ring_t *p_ring) {
  ble_add_char_params_t add_char_params;

  memset(&add_char_params, 0, sizeof(add_char_params));
  add_char_params.uuid = BLE_RING_DATA_CHAR_UUID;
  add_char_params.uuid_type = p_ring->uuid_type;
  add_char_params.char_props.notify = 1;
  add_char_params.char_props.write_wo_resp = 1;
  add_char_params.is_var_len = true;
  add_char_params.max_len = MAX_RING_LEN;

  add_char_params.cccd_write_access = SEC_OPEN;
  add_char_params.write_access = SEC_OPEN;

  return characteristic_add(p_ring->service_handle, &add_char_params, &p_ring->data_handles);
}

static uint32_t ble_ring_read_char_add(ble_ring_t *p_ring) {
  ble_add_char_params_t add_char_params;

  memset(&add_char_params, 0, sizeof(add_char_params));
  add_char_params.uuid = BLE_RING_READ_CHAR_UUID;
  add_char_params.uuid_type = p_ring->uuid_type;
  add_char_params.char_props.read = 1;
  // add_char_params.is_defered_read    = true;
  add_char_params.is_var_len = true;
  add_char_params.max_len = MAX_RING_LEN;

  add_char_params.read_access = SEC_OPEN;

  return characteristic_add(p_ring->service_handle, &add_char_params, &p_ring->read_handles);
}

/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_ring       RING structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_ring_t *p_ring, ble_evt_t const *p_ble_evt) {
  p_ring->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}

/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_ring       RING structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_ring_t *p_ring, ble_evt_t const *p_ble_evt) {
  UNUSED_PARAMETER(p_ble_evt);
  p_ring->conn_handle = BLE_CONN_HANDLE_INVALID;
}

static void readDeviceInfo(uint8_t *);

/**@brief Function for handling the Read.
 *
 * @param[in]   p_ring		RING structure.
 * @param[in]   p_ble_evt	Event received from the BLE stack.
 */
void on_read(ble_ring_t *p_ring) {
  uint32_t err_code;
  static uint8_t payload[20];
  ble_gatts_value_t gatts_value;
  memset(&gatts_value, 0, sizeof(gatts_value));

  if (p_ring != NULL) {
    readDeviceInfo(payload);
    NRF_LOG_INFO("on_read...readDeviceInfo");
  } else {
    extern uint32_t make_random_value(uint8_t * p_rnd_buf, uint32_t p_rnd_len);
    make_random_value(ControlNounce, SOC_ECB_CLEARTEXT_LENGTH);
    memcpy(payload, ControlNounce, SOC_ECB_CLEARTEXT_LENGTH);
  }

  gatts_value.len = sizeof(payload);
  gatts_value.offset = 0;
  gatts_value.p_value = payload;
  //	NRF_LOG_INFO("App read data ... Return device info. \n");

  // Save in READ channel
  err_code = sd_ble_gatts_value_set(p_ring->conn_handle, p_ring->read_handles.value_handle, &gatts_value);
  if (err_code == NRF_ERROR_INVALID_STATE) {
    NRF_LOG_INFO("Set READ failed.....");
  }
  return;
}

/**@brief Function for handling the Write event.
 *
 * @param[in]   p_ring       RING structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_ring_t *p_ring, ble_evt_t const *p_ble_evt) {
  // uint32_t err_code;
  protocol_error_t proError;
  extern ble_dfu_buttonless_t m_dfu;

  NRF_LOG_INFO("ble ring on write");
  ble_gatts_evt_write_t *p_evt_write = (ble_gatts_evt_write_t *)&p_ble_evt->evt.gatts_evt.params.write;

  // ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

  if (p_evt_write->handle == m_dfu.control_point_char.cccd_handle) {
    safeStop(1);
    NRF_LOG_INFO("Received indication state %d",
        m_dfu.is_ctrlpt_indication_enabled);
    return;
  }

#ifndef FDA_CHECK
  if (p_evt_write->handle == p_ring->ctrl_handles.value_handle) {
#else
  {
#endif
    message_source_t evt = BLUETOOTH_MSG;

    NRF_LOG_INFO("ble ring ctrl event : %0X, len: %d", p_evt_write->data[0], p_evt_write->len);
    proError = protocol_msg_push(evt, p_evt_write->data, p_evt_write->len); // Data from App
    if (PROTO_OK != proError)
      NRF_LOG_INFO("protocol_msg_push = %d", proError);

    if (p_ring->conn_handle != BLE_CONN_HANDLE_INVALID) // Read channel
    {
      on_read(p_ring); // Return device info.
    }
  }

  if (p_evt_write->handle == p_ring->ctrl_handles.cccd_handle) {
    if (p_evt_write->len == 2)
      p_ring->is_indicate_enabled = ble_srv_is_indication_enabled(p_evt_write->data);
  } else if (p_evt_write->handle == p_ring->data_handles.cccd_handle) {
    if (p_evt_write->len == 2)
      p_ring->is_notify_enabled = ble_srv_is_notification_enabled(p_evt_write->data);
  }
}

/**@brief Function for handling the HVC events.
 *
 * @details Handles HVC events from the BLE stack.
 *
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
static void on_hvc(ble_ring_t *p_ring, ble_evt_t const *p_ble_evt) {
  ble_gatts_evt_hvc_t const *p_hvc = &p_ble_evt->evt.gatts_evt.params.hvc;
}

#include "ble_dfu.h"

uint32_t ble_ring_init(ble_ring_t *p_ring, ble_ring_init_t const *p_ring_init) {
  uint32_t err_code;
  ble_uuid_t service_uuid;
  ble_uuid128_t ring_base_uuid = BLE_RING_VENDOR_BASE_UUID;

#ifndef FDA_CHECK
  // Initialize service structure
  p_ring->evt_handler = p_ring_init->evt_handler;
  p_ring->is_indicate_enabled = false;
  p_ring->is_notify_enabled = false;
  p_ring->conn_handle = BLE_CONN_HANDLE_INVALID;
  p_ring->max_ring_len = MAX_RING_LEN;

  // Add vendor specific base UUID to use with the Buttonless DFU characteristic.
  err_code = sd_ble_uuid_vs_add(&ring_base_uuid, &p_ring->uuid_type);
  if (err_code != NRF_SUCCESS) {
    return err_code;
  }
  // Add service
  service_uuid.type = p_ring->uuid_type;
  service_uuid.uuid = BLE_RING_SERVICE_UUID;
#else
  service_uuid.type = BLE_UUID_TYPE_BLE;
  service_uuid.uuid = MRINGPROFILE_SERV_UUID;
  p_ring->is_indicate_enabled = false;
  p_ring->is_notify_enabled = false;
  p_ring->conn_handle = BLE_CONN_HANDLE_INVALID;
  p_ring->max_ring_len = MAX_RING_LEN;
#endif

  err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
      &service_uuid,
      &p_ring->service_handle);

  if (err_code != NRF_SUCCESS) {
    return err_code;
  }

  // Add the ctrl char.
  err_code = ble_ring_ctrl_char_add(p_ring);
  if (err_code != NRF_SUCCESS) {
    return err_code;
  }

#ifdef FDA_CHECK
  ble_ring_indi_char_add(p_ring);
#endif

  // Add the data char.
  err_code = ble_ring_data_char_add(p_ring);
  if (err_code != NRF_SUCCESS) {
    return err_code;
  }

  // Add the read char.
  err_code = ble_ring_read_char_add(p_ring);
  if (err_code != NRF_SUCCESS) {
    return err_code;
  }

#ifdef FDA_CHECK
  // Initialize service structure
  p_ring->evt_handler = p_ring_init->evt_handler;
  p_ring->is_indicate_enabled = false;
  p_ring->is_notify_enabled = false;
  p_ring->conn_handle = BLE_CONN_HANDLE_INVALID;
  p_ring->max_ring_len = MAX_RING_LEN;

  // Add vendor specific base UUID to use with the Buttonless DFU characteristic.
  err_code = sd_ble_uuid_vs_add(&ring_base_uuid, &p_ring->uuid_type);
  if (err_code != NRF_SUCCESS) {
    return err_code;
  }

  // Add service
  service_uuid.type = p_ring->uuid_type;
  service_uuid.uuid = 0xFEE0;

  err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
      &service_uuid,
      &p_ring->service_handle);

#endif

  return NRF_SUCCESS;
}

void ble_ring_on_gatt_evt(ble_ring_t *p_ring, nrf_ble_gatt_evt_t const *p_gatt_evt) {
  if ((p_ring->conn_handle == p_gatt_evt->conn_handle) && (p_gatt_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)) {
    p_ring->max_ring_len = p_gatt_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
    NRF_LOG_INFO("p_ring->max_ring_len=%dbytes", p_ring->max_ring_len);
  }
}

void ble_ring_on_ble_evt(ble_evt_t const *p_ble_evt, void *p_context) {
  ble_ring_t *p_ring = (ble_ring_t *)p_context;
  //	uint32_t err_code;

  switch (p_ble_evt->header.evt_id) {
  case BLE_GAP_EVT_CONNECTED:
    on_connect(p_ring, p_ble_evt);
    set_phy_2mbps(p_ble_evt->evt.gap_evt.conn_handle);
    break;

  case BLE_GAP_EVT_DISCONNECTED:
    on_disconnect(p_ring, p_ble_evt);
    break;

  case BLE_GATTS_EVT_WRITE:
    on_write(p_ring, p_ble_evt);
    break;

  case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
    // on_rw_authorize_req(p_ble_evt);
    NRF_LOG_INFO("BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST.\r\n");
    break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

  case BLE_GATTS_EVT_HVC:
    on_hvc(p_ring, p_ble_evt);
    // NRF_LOG_INFO("BLE_GATTS_EVT_HVC.\r\n");
    break;

  case BLE_GATTS_EVT_HVN_TX_COMPLETE:
    NRF_LOG_INFO("BLE_GATTS_EVT_HVN_TX_COMPLETE.\r\n");
    break;

  default:
    // No implementation needed.
    break;
  }
}

uint8_t app_pair_rsp(uint8_t type, uint8_t *pValue) {
    uint8_t data[20];
    data[0] = APP_PAIR_CMD;
    data[1] = get_ble_msg_sn();
    data[2] = CMD_SUCCESS;
    data[3] = type;
    // Type == 0, User should knock ring to pair. When knocked, send token B4~B9. 6bytes.
    // Type == 1, Token matched, just go.

    // NRF_LOG_INFO("app_pair_rsp");
    #define VID(x) pValue[(x)]
    NRF_LOG_INFO("UpRead V0=%2X V1=%2X V2=%2X V3=%2X V4=%2X V5=%2X \n",
        VID(0), VID(1), VID(2), VID(3), VID(4), VID(5));

    if (type == 0) {
        memcpy(&(data[4]), pValue, 6);
        ring_indicate(data, 20);
    } else
        ring_indicate(data, 20);

    return 0;
}

/**@brief Function for Device information.
 *
 * @param[in]   payload		To fill the RING information.
 *
 */
RingInfo_t gRIval;
static void readDeviceInfo(uint8_t *payload) {
    extern int Patch;
    extern productMacSn pms;

    //	gRIval.sensorStatus = 0x0F; // Bitfield 1ok,0error. TMP117:AFE:GS:I2C
    gRIval.recordStatus = 0; // Obsolete.
    gRIval.power = 50;       // Percentage.

    memset(gRIval.padding, 0, 6);

    gRIval.ringSize = 1;

    memcpy(payload, (uint8_t *)&gRIval, 5);

    memcpy(payload + 5, &pms.sn, 6);

    *(payload + 11) = gRIval.sensorStatus; // Sensor
    *(payload + 13) = gRIval.power;        // Power
    //memcpy(payload + 14, &gRIval.padding, 6);
  
    *(payload + 14) = ADV_RING_TYPE;
    *(payload + 15) = ADV_MENUFACTURER;
    *(payload + 16) = ADV_PROTOCOL_VERSION;

  return;
}

void ring_set_sensor_status(uint8_t status_mask, bool res) {
  if (0 != gRIval.sensorStatus)
    gRIval.sensorStatus |= I2C_DRV_OK;

  if (res)
    gRIval.sensorStatus |= status_mask;
  else
    gRIval.sensorStatus &= ~status_mask;
}

uint8_t ring_get_sensor_status(void) {
  return gRIval.sensorStatus;
}