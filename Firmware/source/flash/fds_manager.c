#include "fds_manager.h"
#include "fds.h"
#include "hardfault.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

/* Array to map FDS return values to strings. */
char const *fds_err_str[] =
    {
        "FDS_SUCCESS",
        "FDS_ERR_OPERATION_TIMEOUT",
        "FDS_ERR_NOT_INITIALIZED",
        "FDS_ERR_UNALIGNED_ADDR",
        "FDS_ERR_INVALID_ARG",
        "FDS_ERR_NULL_ARG",
        "FDS_ERR_NO_OPEN_RECORDS",
        "FDS_ERR_NO_SPACE_IN_FLASH",
        "FDS_ERR_NO_SPACE_IN_QUEUES",
        "FDS_ERR_RECORD_TOO_LARGE",
        "FDS_ERR_NOT_FOUND",
        "FDS_ERR_NO_PAGES",
        "FDS_ERR_USER_LIMIT_REACHED",
        "FDS_ERR_CRC_CHECK_FAILED",
        "FDS_ERR_BUSY",
        "FDS_ERR_INTERNAL",
};

static fds_m_handler s_handler = NULL;
static E_FDS_STATUS fds_status = e_fds_no_init;
static void fds_evt_handler(fds_evt_t const *p_evt) {
    switch (p_evt->id) {
        case FDS_EVT_INIT:{
            NRF_LOG_INFO("FDS initialize result (%s)", fds_err_str[p_evt->result]);
        } break;
        case FDS_EVT_WRITE: {
            NRF_LOG_INFO("FDS write result (%s), Record ID:\t0x%04x,File ID:\t0x%04x,Record key:\t0x%04x", \
            fds_err_str[p_evt->result], p_evt->write.record_id, p_evt->write.file_id, p_evt->write.record_key);
        } break;
        case FDS_EVT_DEL_FILE:{
            NRF_LOG_INFO("FDS del file result (%s), Record ID:\t0x%04x,File ID:\t0x%04x,Record key:\t0x%04x", \
            fds_err_str[p_evt->result], p_evt->del.record_id, p_evt->del.file_id, p_evt->del.record_key);
        }
        case FDS_EVT_DEL_RECORD: {
            NRF_LOG_INFO("FDS del record result (%s), Record ID:\t0x%04x, File ID:\t0x%04x, Record key:\t0x%04x", \
            fds_err_str[p_evt->result], p_evt->del.record_id, p_evt->del.file_id, p_evt->del.record_key);
        } break;
        case FDS_EVT_GC:
            NRF_LOG_INFO("FDS GC result (%s)!!!", fds_err_str[p_evt->result]);
        break;
        case FDS_EVT_UPDATE:{
            NRF_LOG_INFO("FDS update result (%s), Record ID:\t0x%04x,File ID:\t0x%04x,Record key:\t0x%04x", \
                fds_err_str[p_evt->result], p_evt->write.record_id, p_evt->write.file_id, p_evt->write.record_key);
            }
        break;

        default:
        break;
    }

    if(s_handler != NULL)
        s_handler(p_evt);

    fds_status = e_fds_idle;
}

static void wait_for_fds_ready(void) {
  while (fds_status != e_fds_idle) {
        if (NRF_LOG_PROCESS() == false) {
          nrf_pwr_mgmt_run();
        }
    }
}

uint32_t _word_len(uint32_t len){
    return (len + 3)/4;
}

ret_code_t fds_m_init(void) {
    ret_code_t rc;

    /* Register first to receive an event when initialization is complete. */
    (void)fds_register(fds_evt_handler);

    NRF_LOG_WARNING("fds_m_init!!!\n");

    rc = fds_init();
    APP_ERROR_CHECK(rc);

    /* Wait for fds to initialize. */
    wait_for_fds_ready();

    NRF_LOG_INFO("Reading flash usage statistics...");

    fds_status = e_fds_gcing;
    fds_gc();
    wait_for_fds_ready();

    fds_stat_t stat = {0};

    rc = fds_stat(&stat);
    APP_ERROR_CHECK(rc);

    NRF_LOG_INFO("Found %d valid records.", stat.valid_records);
    NRF_LOG_INFO("Found %d dirty records (ready to be garbage collected).", stat.dirty_records);

    NRF_LOG_INFO("largest_contig = %d", stat.largest_contig);

    //for(int id = 1; id < 256; id++){
    //    wait_for_fds_ready();
    //    fds_record_desc_t _desc = {0};
    //    fds_find_token_t _token = {0};
    //    ret_code_t ret = fds_record_find(511, id, &_desc, &_token);
    //    NRF_LOG_INFO("fds_m_find find record id 1, key %d, result is (%s)", id, fds_err_str[ret]);
    //    NRF_LOG_PROCESS();
    //}

    return FDS_SUCCESS;
}

uint32_t fds_m_free_space(){
    fds_stat_t stat = {0};
    fds_stat(&stat);

    return stat.largest_contig * 4;
}

ret_code_t fds_m_write(uint8_t* data, uint32_t length, uint16_t file_id, uint16_t key){
    wait_for_fds_ready();
    fds_record_desc_t _desc = {0};
    fds_find_token_t _token = {0};

    if(fds_record_find(file_id, key, &_desc, &_token) == NRF_SUCCESS){
        fds_status = e_fds_deleting;
        fds_record_delete(&_desc);
        wait_for_fds_ready();
    }

    fds_status = e_fds_writing;
    fds_record_t _record;
    _record.file_id = file_id;
    _record.key = key;
    _record.data.p_data = data;
    uint32_t _len = _word_len(length);
    _record.data.length_words = _len;
    ret_code_t ret = fds_record_write(&_desc, &_record);
    if(ret != FDS_SUCCESS){
        fds_status = e_fds_idle;
        NRF_LOG_INFO("fds_m_write result (%s)", fds_err_str[ret]);
        return ret;
    }

    wait_for_fds_ready();
    return FDS_SUCCESS;
}

ret_code_t fds_m_del_file(uint16_t file_id){
    wait_for_fds_ready();
    fds_status = e_fds_deleting;
    fds_record_desc_t _desc;
    fds_find_token_t _t;
    ret_code_t ret = fds_file_delete(file_id);
    if(ret != FDS_SUCCESS){
        fds_status = e_fds_idle;
        NRF_LOG_INFO("fds_m_del_file result (%s)", fds_err_str[ret]);
        return ret;
    }

    wait_for_fds_ready();
    return FDS_SUCCESS;
}

ret_code_t fds_m_read(uint16_t file_id, uint16_t key, uint8_t* pData, uint32_t* pLength){
    wait_for_fds_ready();
    fds_record_desc_t _desc = {0};
    fds_find_token_t _token = {0};
    ret_code_t ret = fds_record_find(file_id, key, &_desc, &_token);
    if(ret != NRF_SUCCESS){
        NRF_LOG_INFO("fds_m_read find record id %d, key %d, result is (%s)", file_id, key, fds_err_str[ret]);
        return ret;
    }

    fds_flash_record_t _config = {0};

    ret = fds_record_open(&_desc, &_config);
    if(ret != NRF_SUCCESS){
        NRF_LOG_ERROR("fds_m_read open report of report id %d key %d, result is (%s)", file_id, key, fds_err_str[ret]);
        return ret;
    }

    *pLength = (_config.p_header->length_words * 4) > (*pLength) ? (*pLength) : (_config.p_header->length_words * 4);
    memcpy(pData, _config.p_data, *pLength);
    //*ppData = _config.p_data;
    //*pLength = (_config.p_header->length_words * 4);

    fds_record_close(&_desc);

    return FDS_SUCCESS;
}

ret_code_t fds_m_find(uint16_t file_id, uint16_t key){
    wait_for_fds_ready();
    fds_record_desc_t _desc = {0};
    fds_find_token_t _token = {0};
    ret_code_t ret = fds_record_find(file_id, key, &_desc, &_token);
    NRF_LOG_INFO("fds_m_find find record id %d, key %d, result is (%s)", file_id, key, fds_err_str[ret]);
    return ret;
}

ret_code_t fds_m_gc(){
    wait_for_fds_ready();
    fds_status = e_fds_gcing;
    ret_code_t ret = fds_gc();
    if(ret != NRF_SUCCESS){
        fds_status = e_fds_idle;
        NRF_LOG_ERROR("fds_m_gc result is (%s)", fds_err_str[ret]);
        return ret;
    }

    return FDS_SUCCESS;
}

uint16_t fds_file_last_key(uint16_t file_id){
    uint16_t key = 0;

    while(++key){
        wait_for_fds_ready();
        fds_record_desc_t _desc = {0};
        fds_find_token_t _token = {0};
        ret_code_t ret = fds_record_find(file_id, key, &_desc, &_token);
        if(ret != NRF_SUCCESS){
            key--;
            break;
        }
    }

    return key;
}

void set_callback(fds_m_handler handler){
    s_handler = handler;
}