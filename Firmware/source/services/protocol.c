/*
 * Copyright (c) 2019 MegaHealth. All Rights Reserved.
 * @file
 * @date 2020.07.29
 * @version
 * @brief
 */
#include "hardfault.h"
#include "nrf_crypto_rng.h"
#include "nrf_delay.h"
#include "nrf_drv_rng.h"

#include "mTask.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_queue.h"
#include "protocol.h"

#include "ble_service.h"
#include "nrf_log.h"
#include "tools.h"

#include "ble_ring.h"
#include "ring_battery.h"
#include "ring_startup.h"
#include "ring_swtimer.h"
#include "ring_tmp.h"
#include "system_clock.h"

#include "md5.h"
#include "ring_power.h"
#include "ring_swtimer.h"
#include "ring_utc.h"
#include "state_module_manager.h"
#include "inner_flash_ram.h"

#define EXTEND_ENABLE true
#if EXTEND_ENABLE
#endif

#define MSG_BODY_ELEMENT_SIZE 20
#define MSG_BODY_ELEMENT_NUM 20

#define PROTOCOL_MSG_POOL_SIZE 20
#define PROTOCOL_TASK_NOTIFY_TIMES 3

#define LINE1(x) #x
#define LINE(x) LINE1(x)
#define TODO(msg) (__FILE__ "(" LINE(__LINE__) "): [TODO] " #msg)

extern productMacSn *g_pPMS;
extern productMacSn *f_pPMS;
extern user_descriptor* user_desp;

typedef struct _msg_body_t {
  uint8_t status;
  uint8_t len;
  uint8_t body[MSG_BODY_ELEMENT_SIZE];
} msg_body_t;

keepAlive_t keepAlive = {0};

NRF_QUEUE_DEF(protocol_message_t, m_protocol_pool, PROTOCOL_MSG_POOL_SIZE, NRF_QUEUE_MODE_NO_OVERFLOW);

static msg_body_t m_msg_body[MSG_BODY_ELEMENT_NUM] = {0};
static int syncIndiHeader(uint8_t *pLoad);

static uint8_t m_task_notify_times = 0;

st_bleRsp_t bleRsp;

#define CRC_LEN 2             // 2Bytes
#define SECTOR_SYNC_SIZE 2048 // BLE sync block size.

static uint32_t bp_uptime = 0;

typedef struct
{
  uint8_t pduSn;
  uint8_t gSn;
  uint8_t sn;
  uint8_t readFromFlash;
  uint16_t readLen;
  uint16_t txIndex;

  volatile uint8_t synStatus;
  uint32_t totalLen;
  uint16_t subLen;
  uint16_t recvLen;
  uint16_t crc;
  uint8_t rType;
  uint8_t err;
  uint8_t reportCmd;
  uint8_t packets[BLE_MAX_BUFFER_NUM][PDU_LENGTH]; // (107+1) [19]
} syncGlobalInfo_t;
static syncGlobalInfo_t gSyncInfo = {0};
static void clearSyncInfo() {
  uint8_t cmd = gSyncInfo.reportCmd;
  uint8_t rType = gSyncInfo.rType;
  memset(&gSyncInfo, 0, sizeof(gSyncInfo));
  gSyncInfo.reportCmd = cmd;
  gSyncInfo.rType = rType;
}

uint32_t ecb_block_encrypt(uint8_t *const p_dst_buf, uint8_t const *const p_src_buf, uint8_t const *const p_key_buf) {
  nrf_ecb_hal_data_t ecb_data;
  nrf_ecb_hal_data_t *p_ecb_data = &ecb_data;
  memcpy(ecb_data.key, p_key_buf, SOC_ECB_KEY_LENGTH);
  memcpy(ecb_data.cleartext, p_src_buf, SOC_ECB_CLEARTEXT_LENGTH);

  uint32_t err_code = NRF_SUCCESS;

  err_code = sd_ecb_block_encrypt(p_ecb_data);
  APP_ERROR_CHECK(err_code);

  memcpy(p_dst_buf, ecb_data.ciphertext, SOC_ECB_CIPHERTEXT_LENGTH);
  return err_code;
}

uint32_t make_random_value(uint8_t *p_rnd_buf, uint32_t p_rnd_len) {
  uint32_t err_code;
  //	err_code = nrf_drv_rng_init(NULL);
  //	APP_ERROR_CHECK(err_code);

  nrf_drv_rng_init(NULL);

  nrf_drv_rng_block_rand(p_rnd_buf, p_rnd_len);

  nrf_drv_rng_uninit();

  return err_code;
}

static msg_body_t *msg_bd_malloc(void) {
    NRF_LOG_INFO("msg_bd_malloc");
    for (int i = 0; i < MSG_BODY_ELEMENT_NUM; ++i) {
        if (m_msg_body[i].status == 0) {
            m_msg_body[i].status = 1;
            NRF_LOG_INFO("msg_bd_malloc select %d", i);
            return &m_msg_body[i];
        }
    }

    return NULL;
}

static void msg_bd_free(msg_body_t *bd) {
  bd->status = 0;
}

static void protocol_standard_handle(protocol_message_t *msg) {
  if (msg->source >= SPECIAL_DEFINE_MSG) {
    return;
  }

  msg_body_t *bd = (msg_body_t *)msg->message;
  protocol_cmd_t cmd = (protocol_cmd_t)bd->body[0];

  switch (cmd) {
  case SET_MONITOR:
    NRF_LOG_INFO("set monitor...");
    break;

  case QUERY_MONITOR:
    NRF_LOG_INFO("query monitor...");
    break;

  case RECV_MONITOR_DATA:
    NRF_LOG_INFO("recv monitor...");
    break;

  default:
    break;
  }
}

static void protocol_special_handle(void *payload) {
  // TODO: user define the payload handler
}

/**
 * @brief 内部定义可以缓存数据处理,长度控制在MSG_BODY_ELEMENT_SIZE以内,处理完会释放.
 */
protocol_error_t protocol_msg_push(message_source_t src, uint8_t *p_data, uint8_t length) {
  ret_code_t err_code;
  protocol_error_t ret;

  if (length > MSG_BODY_ELEMENT_SIZE) {
    return PROTO_ERROR_LENGTH;
  }

  CRITICAL_REGION_ENTER();

  protocol_message_t msg;
  msg.source = src;

  msg_body_t *bd = msg_bd_malloc();
  if (!bd) {
    NRF_LOG_INFO("11111111111111111111111");
    ret = PROTO_ERROR_MEMOUT;
    goto END;
  }
  bd->len = length;
  memcpy(bd->body, p_data, length);
  msg.message = (void *)bd;

  err_code = nrf_queue_push(&m_protocol_pool, (void const *)&msg);
    NRF_LOG_INFO("nrf_queue_push code : %d", err_code);
  if (err_code == NRF_SUCCESS) {
    /*防止过多的塞入系统事件，每次循环都会取净*/
    //if (m_task_notify_times < PROTOCOL_TASK_NOTIFY_TIMES) {
        NRF_LOG_INFO("m_task_notify_times : %d", m_task_notify_times);
      sEvent buf = {BLECmd_QID, NULL};
      err_code = taskPush(&buf, false);
      NRF_LOG_INFO("taskPush code : %d", err_code);
      //if (NRF_SUCCESS == err_code) {
      //  m_task_notify_times++;
      //}
    //}

    ret = PROTO_OK;
  } else if (err_code == NRF_ERROR_NO_MEM) {
    NRF_LOG_INFO("22222222222222222222222222222");
    ret = PROTO_ERROR_MEMOUT;
  } else {
    ret = PROTO_ERROR_OPERATION;
  }

END:
  CRITICAL_REGION_EXIT();
  return ret;
}

protocol_error_t protocol_user_msg_push(message_source_t src, void *usr_msg) {
  ret_code_t err_code;
  protocol_error_t ret;

  if (src < SPECIAL_DEFINE_MSG) {
    return PROTO_ERROR_OPERATION;
  }

  protocol_message_t msg;
  msg.source = src;
  msg.message = usr_msg;

  err_code = nrf_queue_push(&m_protocol_pool, (void const *)&msg);

  if (err_code == NRF_SUCCESS) {
    if (m_task_notify_times < PROTOCOL_TASK_NOTIFY_TIMES) {
      sEvent buf = {BLECmd_QID, NULL};
      if (NRF_SUCCESS == taskPush(&buf, false)) {
        m_task_notify_times++;
      }
    }

    ret = PROTO_OK;
  } else if (err_code == NRF_ERROR_NO_MEM) {
    ret = PROTO_ERROR_MEMOUT;
  } else {
    ret = PROTO_ERROR_OPERATION;
  }

  return ret;
}

protocol_error_t protocol_msg_pop(protocol_message_t *msg) {
  ret_code_t err_code;
  protocol_error_t ret;

  err_code = nrf_queue_pop(&m_protocol_pool, (void *)msg);
  //	NRF_LOG_INFO("protocol_msg_pop err_code=%d", err_code );

  if (err_code == NRF_SUCCESS) {
    ret = PROTO_OK;
  } else if (err_code == NRF_ERROR_NOT_FOUND) {
    m_task_notify_times = 0;
    ret = PROTO_ERROR_EMPTY;
  } else {
    ret = PROTO_ERROR_OPERATION;
  }

  return ret;
}

extern ble_ring_t m_ring;
void PairTimeout_handler(void *p_context) {
  UNUSED_PARAMETER(p_context);
  sEvent evt;

  if (m_ring.conn_handle == BLE_CONN_HANDLE_INVALID) {
    stopPairTimer();
    NRF_LOG_INFO("STEP1 error\r\n");
    return;
  }

  switch (connect_crtl.mr_pair_evt) {
  case MR_PAIR_START:
#ifdef FDA_CHECK
    connect_crtl.mr_pair_evt = MR_PAIR_WAITING;
#endif
    if ((app_get_sec() - connect_crtl.pairStartTime) > connect_crtl.pair_proc_timeout) // 10/120s timerout
    {
      NRF_LOG_INFO("STEP2 error,app_get_sec=%u, pairStartTime=%u,\n",
          app_get_sec(), connect_crtl.pairStartTime);
      ringPairError();
    }
    break;

  case MR_PAIR_WAITING:
    if (1 == connect_crtl.wait_isr) { // TODO: Flash on-hand led to indicate user shock the ring.
      connect_crtl.mr_pair_evt = MR_PAIR_END;
      NRF_LOG_INFO("MR_PAIR_END\n");
    } else if ((app_get_sec() - connect_crtl.pairStartTime) > APP_PAIR_TIMEOUT) // 30s timerout
    {
      NRF_LOG_INFO("STEP3 error \n");
      ringPairError();
      static uint16_t acc_pair_isr = MR_PAIR_ISR_TIMEOUT;
      evt.evt = I2C_QID;
      evt.payload = (void *)&acc_pair_isr;
      taskPush(&evt, false);
    }
    break;

  case MR_PAIR_END:
    NRF_LOG_WARNING("MR_PAIR_END 8");

    connect_crtl.isPaired = true;
    on_read(&m_ring);

#ifdef FDA_CHECK
    user_desp->pair_rnd_type = PAIR_USE_OLD_RND_PARAM;
#endif

    if (user_desp->pair_rnd_type == PAIR_USE_NEW_RND_PARAM) {
      app_pair_rsp(PAIR_USE_NEW_RND_PARAM, user_desp->random_id);
      NRF_LOG_INFO("== PAIR_USE_NEW_RND_PARAM pair end report.\r\n");

      update_NI_flash_CMD();
    } else {
      NRF_LOG_INFO("!= PAIR_USE_OLD_RND_PARAM pair end report.\r\n");

      app_pair_rsp(PAIR_USE_OLD_RND_PARAM, NULL);
    }

    stopPairTimer();

    break;

  default:
    NRF_LOG_INFO("STEP4 error\r\n");
    ringPairError();
    break;
  }
}

static uint32_t PairHandle(uint8_t *p_buf) {
  // extern productMacSn *g_pPMS;
  extern productMacSn pms;
  NRF_LOG_WARNING("PairHandle enter");

  sEvent tx_evt;
  static uint16_t acc_open_pair = MR_OPEN_PAIR_ISR;
  uint32_t err_code = NRF_SUCCESS;

  //printf("PairHandle: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X \n",
  //  p_buf[0], p_buf[1], p_buf[2], p_buf[3], p_buf[4], p_buf[5], p_buf[6], p_buf[7], p_buf[8], p_buf[9], 
  //  p_buf[10], p_buf[11], p_buf[12], p_buf[13], p_buf[14], p_buf[15], p_buf[16], p_buf[17], p_buf[18], p_buf[19]);

  if (p_buf[0] == 0xB0) {
    uint8_t master_words[16] = {0}; // md5_16digest. key
    md5(&p_buf[3], 12, master_words);
    
  //printf("master_words: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X \n",
  //  master_words[0], master_words[1], master_words[2], master_words[3], master_words[4], master_words[5], master_words[6], master_words[7], master_words[8], master_words[9], 
  //  master_words[10], master_words[11], master_words[12], master_words[13], master_words[14], master_words[15]);
    // PDU.  B0:cmd(0xB0), B1:0, B2:devType, B3~B14:uid(12bytes),
    // B15~B19:MACdigest(last 5 Bytes of md5_16(mac) )
    if (memcmp(p_buf + 15, &master_words[11], 5) == 0) // compare 5Bytes key.
    {
      user_desp->pair_rnd_type = PAIR_USE_OLD_RND_PARAM;
      connect_crtl.wait_isr = 1;
      NRF_LOG_WARNING("master_words PAIR_USE_OLD_RND_PARAM.  Set wait_isr=1");
      return NRF_SUCCESS;
    }

    //FDS_Read(USER);

    user_desp->pair_rnd_type = PAIR_USE_NEW_RND_PARAM;
    /*
                    uint8_t sec_v[16] = {g_pPMS->mac[5], g_pPMS->mac[4], g_pPMS->mac[3], g_pPMS->mac[2],
                                                             g_pPMS->mac[1], g_pPMS->mac[0], 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    */
    uint8_t sec_v[16] = {pms.mac[5], pms.mac[4], pms.mac[3], pms.mac[2],
        pms.mac[1], pms.mac[0], 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    memcpy(PRIVATE_PAIR_AES_KEY, sec_v, BLE_GAP_ADDR_LEN);

    if (NRF_SUCCESS != ecb_block_encrypt(sec_v, sec_v, PRIVATE_PAIR_AES_KEY))
      return NRF_ERROR_INVALID_PARAM;

    // if default(MAC) key not equal, compare the USERID in flash.
    if (memcmp(p_buf + 15, sec_v + 11, 5) != 0) { // not default(MAC) key
        memset(sec_v, 0, sizeof(sec_v));
        memcpy(sec_v, user_desp->random_id, BLE_GAP_ADDR_LEN);
        memcpy(PRIVATE_PAIR_AES_KEY, sec_v, BLE_GAP_ADDR_LEN);

        //printf("compare random_id: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X \n",
        //    RID(0), RID(1), RID(2), RID(3), RID(4), RID(5), RID(6), RID(7), RID(8), RID(9), 
        //    RID(10), RID(11), RID(12), RID(13), RID(14), RID(15));
        if (NRF_SUCCESS != ecb_block_encrypt(sec_v, sec_v, PRIVATE_PAIR_AES_KEY))
            return NRF_ERROR_INVALID_PARAM;

        if (memcmp(p_buf + 15, sec_v + 11, 5) == 0) {
            user_desp->pair_rnd_type = PAIR_USE_OLD_RND_PARAM;
            connect_crtl.wait_isr = 1;
            NRF_LOG_WARNING("111 master_words PAIR_USE_OLD_RND_PARAM.  Set wait_isr=1");
        }
    }

    if (user_desp->pair_rnd_type == PAIR_USE_NEW_RND_PARAM) {
    
        NRF_LOG_WARNING("requre pair");
        memcpy(user_desp->user_id, p_buf + 3, USRD_ID_LEN);
        make_random_value(user_desp->random_id, BLE_GAP_ADDR_LEN);

        //printf("make random_id: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X \n",
        //    RID(0), RID(1), RID(2), RID(3), RID(4), RID(5), RID(6), RID(7), RID(8), RID(9), 
        //    RID(10), RID(11), RID(12), RID(13), RID(14), RID(15));
        //NRF_LOG_INFO("PAIR R0=%2X R1=%2X R2=%2X R3=%2X R4=%2X R5=%2X \n",
        //    RID(0), RID(1), RID(2), RID(3), RID(4), RID(5));

        tx_evt.evt = I2C_QID;
        tx_evt.payload = (void *)&acc_open_pair;
        taskPush(&tx_evt, false);
        app_pair_rsp(PAIR_REP_USER_CLICK, NULL);
    }
  } else {
    return NRF_ERROR_INVALID_PARAM;
  }

  return err_code;
}
/*
*SNnumber：[typename][size][verYYmm][num]
----------------------------------------------------------------------------------------------------------------------------------------------
|               Byte0-Byte1              |                Byte2-Byte4            |                          Byte5                            |
----------------------------------------------------------------------------------------------------------------------------------------------
|            verYYmm(Big-Endian)         |               num(Big-Endian)         |                          reg_sn                           |
----------------------------------------------------------------------------------------------------------------------------------------------
|    verYYmm[12:7]   |    verYYmm[6:3]   |                 num[23:0]             |     reg_sn[7:5]    |   reg_sn[:4:]   |    reg_sn[3:0]     |
----------------------------------------------------------------------------------------------------------------------------------------------
|         yy         |         mm        |                   num                 |   typename_part2   |      size       |   typename_part1   |
----------------------------------------------------------------------------------------------------------------------------------------------
*
typename:
--------------------------------------------------------
|                |              typename               |
|    ringType    |--------------------------------------
|                |  typename_part1  |  typename_part2  |
--------------------------------------------------------
|    "C11E"      |      0x05        |       0x00       |
--------------------------------------------------------
|    "P11E"      |      0x05        |       0x01       | <=="P11E*"
--------------------------------------------------------
|    "P11F"      |      0x05        |       0x02       |
--------------------------------------------------------
|    "C11E"      |      0x01        |       ----       |
--------------------------------------------------------
|    "C11E"      |      0x0D        |       0x00       |
--------------------------------------------------------
*
*/
static uint8_t app_update_sn(uint8_t *pR) {
  //productMacSn pms;
  //memcpy(&pms, g_pPMS, sizeof(productMacSn));
  //NRF_LOG_INFO("pR[3:8]: %02X %02X %02X %02X 02X %02X", pR[3], pR[4], pR[5], pR[6], pR[7], pR[8]);

  ///* update_sn */
  //memcpy(&pms.sn, &pR[3], 6);
  //writePMStoUICR(&pms);

  //if (0 == (memcmp(&pms.sn, &(g_pPMS->sn), 6)))
  //  return true;
  //else
  //  return false;

  // return CMD_SUCCESS;
}

void proTaskProc(void *pMsg) {
    protocol_message_t msg;

    while (protocol_msg_pop(&msg) == PROTO_OK) {
        switch (msg.source) {
        case BLUETOOTH_MSG:
            bleProcess((uint8_t *)msg.message);
            break;
        case AT_COMMAND_MSG:
        case SYSTEM_INTERNAL_MSG:
            protocol_standard_handle(&msg);
            break;
        case SPECIAL_DEFINE_MSG:
            protocol_special_handle(msg.message);
            break;
        }

        msg_bd_free(msg.message);
    }
    return;
}

enum recordStatus rStatus = NO_RECORDING;

// Send notify Temperature / SpO2 / HR realtime per seconds.
#define NORMAL_NOTIFY_LEN 20

int sentCnt = 0;  // Successful counts.
int totalTry = 0; // Total counts of send.

void sendRawdata(uint8_t rawType, uint8_t *buf, uint8_t len) {
  uint32_t n;
  uint8_t max_len = aux_max_len();

  //	if (len > max_len)
  //		NRF_LOG_INFO("Should assemble some groups with MTU( mtu-3),len = %d",len);

  buf[0] = rawType;
  buf[1] = 1; // Protocol num of rawdata.

  totalTry++;

  n = htonl(totalTry);
  memcpy(buf + 2, (uint8_t *)&n, 4);

  if (aux_raw_notify(buf, max_len) == NRF_SUCCESS) {
    sentCnt++;
    // NRF_LOG_INFO("Raw sent count=%d\n", sentCnt);
  } else {
    NRF_LOG_ERROR("Raw sent error.")
  }
}

static uint8_t ble_msg_sn = 0;
uint8_t get_ble_msg_sn(){
    return ble_msg_sn;
}

extern struct st_dump dump;
void bleProcess(uint8_t *pM) {
    uint8_t *pR = pM + 2;   // 2 bytes are inner msgHeader.
    uint8_t cmd = *(pR);    // Response paired with the request.
    uint8_t sn = *(pR + 1); // Sn should be matched.

    sEvent tx_evt;
    uint8_t v;
    static uint8_t Indi[20] = {0};
    memset(Indi, 0, 20);

    static uint8_t idx = 0;
    uint8_t rStatus = CMD_SUCCESS;

    // uint8_t forceFlag = pR[3]; // If app set mode. Byte3 force flag.

    Indi[0] = cmd;
    Indi[1] = sn;
    Indi[2] = rStatus;
    idx = 3; // Content start index of indication.
    ble_msg_sn = sn;
    
    // NRF_LOG_INFO("bleProcess CMD=%2X",cmd );
    NRF_LOG_INFO("bleProcess 0x%02X 0x00 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
        pR[0], pR[2], pR[3], pR[4], pR[5], pR[6]);

    if(state_modules_BLE_process(pR, Indi))
        return;

    switch (cmd) {
    case APP_PAIR_CMD:
        if (connect_crtl.isPaired == false) {
            if (NRF_SUCCESS == PairHandle(pR)) {
                connect_crtl.mr_pair_evt = MR_PAIR_WAITING;
            } else
                ringPairError();
        } else {
            Indi[3] = PAIR_USE_OLD_RND_PARAM;
            connect_crtl.mr_pair_evt = MR_PAIR_END;
            break;
        }
    return; // No need to send indication again.
    case APP_SET_DEVICE_RESET_CMD: // 0xE2
    /*  Make it enable to quickly disconnect from APP.  */
        ring_disconnect();
        //stopMonitoring(STOP_RESET);
        nrf_delay_ms(100);
        ring_systemreset_entry();
    break;
    case APP_SET_DEVICE_SHUTDOWN:
    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    // Go to system off.
    //if (!device_is_oncharger()) {
    //    /*  Make it enable to quickly disconnect from APP.  */
        ring_disconnect();
        nrf_delay_ms(100);
        peripherals_power_off();
        nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
        //ring_systemoff_entry();
    //} else
    //    rStatus = OPERA_DISALLOW;
    break;
    case APP_HARD_RESET_RING_CMD:
        update_NI_flash_nvmc_4_initiactive_hard_reset();
        chg_hard_reset();
    break;
    case APP_GET_BATT_CMD:
        Indi[idx++] = get_batt_vol_percent();
        Indi[idx++] = chgchip_charg_status();
    break;
    case APP_SET_USER_INFO_CMD:
        memcpy((uint8_t *)&(user_desp->age), &pR[3], RECORDS_PERSON_DESP_LENGTH);
        memcpy(Indi + idx, (uint8_t *)&(user_desp->age), RECORDS_PERSON_DESP_LENGTH);
        idx += RECORDS_PERSON_DESP_LENGTH;
        if (user_desp->pair_rnd_type == PAIR_USE_NEW_RND_PARAM) {
            // update as old usr
            user_desp->pair_rnd_type = PAIR_USE_OLD_RND_PARAM;
            user_desp->time = UTC_getClock();
            update_NI_flash_CMD();
        }
    break;
    case APP_GET_USER_INFO_CMD:
        rStatus = CMD_SUCCESS;
        memcpy(Indi, user_desp->user_id, USRD_ID_LEN);
        idx += USRD_ID_LEN;
        memcpy(Indi + idx, (uint8_t *)&(user_desp->age), RECORDS_PERSON_DESP_LENGTH);
        idx += RECORDS_PERSON_DESP_LENGTH;
    break;
    case APP_GET_RING_CRASH_DATA:
    // Crash log was saved in NIRAM. When power off , it will be erased.
        if (pR[3] == 0) { // 0, read crash log.
            memcpy(Indi + idx, (uint8_t *)&dump, 16);
        idx += 16;
        } else { // 1, clear crash log.
            memset((uint8_t *)&dump, 0, sizeof(dump_t));
        }

    break;
    case APP_GET_SYSTEM_SEC:
        rStatus = CMD_SUCCESS;
        uint32_4bytes_t rds_time;
        rds_time.value.bytes = 0; // app_get_sec();
        Indi[idx++] = rds_time.value.field.byte3;
        Indi[idx++] = rds_time.value.field.byte2;
        Indi[idx++] = rds_time.value.field.byte1;
        Indi[idx++] = rds_time.value.field.byte0;
    break;
    default:
    ft_bleProcess_label:
    #if !SKIP_FT_STEP
        ft_bleProcess(pM);
        return;
    #endif
        break;
    }

    Indi[2] = rStatus;

    ring_indicate(Indi, 20);

    return;
}