#include "reports.h"
#include "fds.h"
#include "ble_ring.h"
#include "hardfault.h"
#include "nrf_log.h"
#include "fds_manager.h"
#include "inner_flash_ram.h"
#include "state_module_def.h"
#include "ring_swtimer.h"
#include <stdlib.h>

#define REPORT_DATA_BLOCK_SEPARATOR 0xA5ACA5AC

#define MIN_SEC_HR_MASK_POS                 0
#define MIN_SEC_TEMP_MASK_POS               1
#define MIN_SEC_ACC_MASK_POS                2
#define MIN_SEC_SPO2_MASK_POS               3
#define MIN_SEC_STEP_LSB_MASK_POS           4
#define MIN_SEC_STEP_MSB_MASK_POS           5
#define MIN_SEC_ELECTRIC_MASK_POS           6
#define MIN_SEC_RESERVED_1_MASK_POS         7

#define MIN_SEC_DATA_MAX                    8

//the max time that data interval can represented, sec data is 240 second, min data is 240 minute
#define MIN_SEC_DATA_INTERVAL_MAX           240 

#define EACH_TYPE_REPORTS_MAX_COUNT         255
#define REPORTS_KEY_START                   1
#define SEC_REPORT_ID_BASE                  1
#define HRV_REPORT_ID_BASE                  (SEC_REPORT_ID_BASE+EACH_TYPE_REPORTS_MAX_COUNT)

#define DAILY_REPORT_ID                     (HRV_REPORT_ID_BASE+EACH_TYPE_REPORTS_MAX_COUNT)
#define DAILY_REPORT_HRV_ID                 DAILY_REPORT_ID + 1
#define REPORT_BLOCK_SYNC_REPEAT_CNT_MAX    5

#pragma pack(1)
typedef struct st_report_head{
    uint32_t report_start_time;     //timestamp
    uint8_t report_type;            //@ref E_REPORT_TYPE
    uint8_t protocol_id;
    uint8_t report_id;              //[1:255]
    uint8_t hard_ver;
    uint8_t firmware_major;
    uint8_t firmware_minjor;
    uint16_t firmware_patch;
    uint16_t boot_ver;
    sn_t device_sn;
    uint8_t user_uid[USRD_ID_LEN];
    uint8_t report_sub_type;
    uint8_t stop_type;
} ST_REPORT_HEAD;

typedef struct st_report_block_head{
    uint32_t separator;
    uint32_t start_time;
}ST_REPORT_BLOCK_HEAD;

typedef struct st_report_min_sec_data{
    uint8_t interval;
    uint8_t mask;
    uint8_t data[MIN_SEC_DATA_MAX];
}ST_REPORT_MIN_SEC_DATA;

typedef struct st_report_read_buff{
    uint8_t buff[REPORT_BUFF_MAX_LEN];
    uint16_t crc;
    uint16_t buff_len;
    uint16_t occupied_len;
    uint16_t report_id;
    uint16_t report_key;
    uint16_t sec_min_block_cnt;
    uint16_t hrv_block_cnt;
    uint16_t repeat_cnt;
}ST_REPORT_READ_BUFF;
#pragma pack()

ST_REPORT_MIN_SEC_DATA last_sec_data = {0};
ST_REPORT_MIN_SEC_DATA last_daily_data = {0};
uint32_t last_sec_data_time = 0;
uint32_t last_daily_data_time = 0;
ST_REPORT_READ_BUFF report_read_buff = {0};

static const uint8_t hrv_mark[4] = {'H','R','V','R'};

char const *report_type_str[] =
    {
        "sleep",
        "daily",
        "sport",
        "professional_sleep",
        "professional_stress",
        "auto_sport"
};

extern int Patch;
extern int Minor;
extern int Major;
extern productMacSn pms;
extern user_descriptor* user_desp;
extern RingInfo_t gRIval;
extern NIRAM_t NIramCfg;
extern ST_NI_FLASH_DATA g_NI_flash;

bool is_flash_full(){
    uint16_t free = fds_m_free_space();
    
    NRF_LOG_INFO("is_flash_full free = %d", free);
    return free > 2046 ? false:true;
}

ret_code_t new_sec_report_id(){
    NIramCfg.cur_sec_report_id = INVALID_REPORT_ID;
    NIramCfg.last_sec_report_key = INVALID_REPORT_KEY;
    NIramCfg.last_hrv_report_key = INVALID_REPORT_KEY;

    for(int i = 1; i <= EACH_TYPE_REPORTS_MAX_COUNT ; i++){
        int index = i/8;
        int mark = 1<<(i%8);

        if(!(NIramCfg.sec_report_ids[index] & mark)){
            NIramCfg.cur_sec_report_id = i;
            NIramCfg.sec_report_ids[index] |= mark;

            g_NI_flash.cur_sec_report_id = NIramCfg.cur_sec_report_id;
            memcpy(g_NI_flash.sec_report_ids, NIramCfg.sec_report_ids, sizeof(NIramCfg.sec_report_ids));
            update_NI_flash_CMD();
            break;
        }
    }

    if(NIramCfg.cur_sec_report_id == INVALID_REPORT_ID)
        return REPORT_ERR_ID_FULL;

    return FDS_SUCCESS;
}

uint16_t get_current_sec_report_id(){
    return NIramCfg.cur_sec_report_id;
}

//extern uint8_t sub_exercise_type;
void new_report_header(E_REPORT_TYPE type){
    ST_REPORT_HEAD _report_head = {0};
    _report_head.report_start_time = UTC_getClock();
    _report_head.report_type = type;
    _report_head.report_type |= 0x80;
    _report_head.protocol_id = REPORT_PROTOCOL_ID;
    _report_head.report_id = NIramCfg.cur_sec_report_id;
    _report_head.hard_ver = gRIval.hwVer;
    _report_head.firmware_major = Major;
    _report_head.firmware_minjor = Minor;
    _report_head.firmware_patch = gRIval.rev;
    _report_head.boot_ver = gRIval.btVer;
    _report_head.stop_type = e_report_end_by_fault;
    memcpy(&_report_head.device_sn, &pms.sn, sizeof(sn_t));
    memcpy(_report_head.user_uid, user_desp->user_id, sizeof(_report_head.user_uid));
    
    switch(type){
    case e_report_sleep:
    case e_report_low_power_sleep:
    case e_report_professional_sleep:
    case e_report_professional_stress:
    case e_report_auto_sport:
        memcpy(NIramCfg.sec_report_buff.buff, &_report_head, sizeof(ST_REPORT_HEAD));
        NIramCfg.sec_report_buff.occupied_len = sizeof(ST_REPORT_HEAD);
        break;
    case e_report_sport:
        _report_head.report_sub_type = NIramCfg.cur_sub_module;
        memcpy(NIramCfg.sec_report_buff.buff, &_report_head, sizeof(ST_REPORT_HEAD));
        NIramCfg.sec_report_buff.occupied_len = sizeof(ST_REPORT_HEAD);
        break;
    case e_report_daily:
        memcpy(NIramCfg.daily_report_buff.buff, &_report_head, sizeof(ST_REPORT_HEAD));
        NIramCfg.daily_report_buff.occupied_len = sizeof(ST_REPORT_HEAD);
        break;
    }
}

ret_code_t save_report_buff(E_REPORT_TYPE type, E_REPORT_DATA_TYPE data_type){
    uint16_t file_id = INVALID_REPORT_ID;
    uint16_t* file_key = NULL;
    ST_REPORT_BUFF* buff = NULL;

    switch(type){
    case e_report_sleep:
    case e_report_professional_sleep:
    case e_report_sport:
    case e_report_auto_sport:
    case e_report_professional_stress:
    case e_report_low_power_sleep:
        if(data_type == e_report_data_min_sec){
            file_id = NIramCfg.cur_sec_report_id;
            file_key = &NIramCfg.last_sec_report_key;
            buff = &NIramCfg.sec_report_buff;
        } else if(data_type == e_report_data_hrv){
            file_id = NIramCfg.cur_sec_report_id + EACH_TYPE_REPORTS_MAX_COUNT;
            file_key = &NIramCfg.last_hrv_report_key;
            buff = &NIramCfg.hrv_report_buff;
        }
        break;
    case e_report_daily:
        if(data_type == e_report_data_min_sec){
            file_id = NIramCfg.cur_daily_report_id;
            file_key = &NIramCfg.last_daily_report_key;
            buff = &NIramCfg.daily_report_buff;
        } else if(data_type == e_report_data_hrv){
            file_id = DAILY_REPORT_HRV_ID;
            file_key = &NIramCfg.last_daily_report_hrv_key;
            buff = &NIramCfg.daily_hrv_report_buff;
        }
    }

    if(file_id == INVALID_REPORT_ID)
        return REPORT_ERR_ID_INVALID;

    if(file_key == NULL)
        return REPORT_ERR_KEY_INVALID;

    if(buff == NULL || buff->occupied_len <= 0)
        return FDS_SUCCESS;

    uint16_t key = (*file_key) +  1;
    ret_code_t ret = fds_m_write(buff, buff->occupied_len + sizeof(buff->occupied_len), file_id, key);
    if(ret == FDS_SUCCESS){
        (*file_key) = key;
    }

    memset(buff, 0, sizeof(ST_REPORT_BUFF));
    wdt_feed();
    idle_state_handle();
    return ret;
}

void clear_monitor_report_ram(){
    NIramCfg.cur_sec_report_id = INVALID_REPORT_ID;
    NIramCfg.last_sec_report_key = INVALID_REPORT_KEY;
    NIramCfg.last_hrv_report_key = INVALID_REPORT_KEY;
    //NIramCfg.last_daily_report_hrv_key = INVALID_REPORT_KEY;

    memset(&NIramCfg.sec_report_buff, 0, sizeof(ST_REPORT_BUFF));
    memset(&NIramCfg.hrv_report_buff, 0, sizeof(ST_REPORT_BUFF));
    
    g_NI_flash.cur_sec_report_id = NIramCfg.cur_sec_report_id;
    update_NI_flash_CMD();
}

void clear_daily_report_ram(){
    NIramCfg.cur_daily_report_id = INVALID_REPORT_ID;
    NIramCfg.last_daily_report_key = INVALID_REPORT_KEY;
    NIramCfg.last_daily_report_hrv_key = INVALID_REPORT_KEY;
    memset(&NIramCfg.daily_report_buff, 0, sizeof(ST_REPORT_BUFF));
    memset(&NIramCfg.daily_hrv_report_buff, 0, sizeof(ST_REPORT_BUFF));
    
    g_NI_flash.cur_daily_report_id = NIramCfg.cur_daily_report_id;
    update_NI_flash_CMD();
}

/****************first hrv block file********************/
//len       hrv_mark    block_header            data
//uint16_t  'HRVR'      ST_REPORT_BLOCK_HEAD    data
/****************first hrv block file********************/

/****************hrv block file********************/
//len       block_header            data    ... data
//uint16_t  ST_REPORT_BLOCK_HEAD    data    ... data
/****************hrv block file********************/

/****************sec_min block file********************/
//len       block_header            data                    ... data
//uint16_t  ST_REPORT_BLOCK_HEAD    ST_REPORT_MIN_SEC_DATA  ... ST_REPORT_MIN_SEC_DATA
/****************sec_min block file********************/

/****************finilly sec_min block file********************/
//len       block_header            data                    ... data                    end_type
//uint16_t  ST_REPORT_BLOCK_HEAD    ST_REPORT_MIN_SEC_DATA  ... ST_REPORT_MIN_SEC_DATA  uint8_t
/****************finilly sec_min block file********************/
void new_report_block(E_REPORT_TYPE type, E_REPORT_DATA_TYPE data_type){
    ST_REPORT_BLOCK_HEAD _header;
    _header.separator = htonl(REPORT_DATA_BLOCK_SEPARATOR);
    _header.start_time = UTC_getClock();
    uint16_t len_head = sizeof(ST_REPORT_BLOCK_HEAD);

    switch(type){
        case e_report_sleep:
        case e_report_low_power_sleep:
        case e_report_professional_sleep:
        case e_report_professional_stress:
        case e_report_auto_sport:
        case e_report_sport:{
            switch(data_type){
                case e_report_data_min_sec:{
                    memset(&NIramCfg.sec_report_buff, 0, sizeof(ST_REPORT_BUFF));
                    last_sec_data.data[MIN_SEC_HR_MASK_POS] = INVALID_HR;
                    last_sec_data.data[MIN_SEC_TEMP_MASK_POS] = INVALID_TEMP;
                    last_sec_data.data[MIN_SEC_ACC_MASK_POS] = INVALID_ACC;
                    last_sec_data.data[MIN_SEC_SPO2_MASK_POS] = INVALID_SPO2;
                    last_sec_data.data[MIN_SEC_STEP_LSB_MASK_POS] = INVALID_STEP;
                    last_sec_data.data[MIN_SEC_STEP_MSB_MASK_POS] = INVALID_STEP;
                    last_sec_data_time = _header.start_time;
                    memcpy(NIramCfg.sec_report_buff.buff, &_header, len_head);
                    NIramCfg.sec_report_buff.occupied_len += len_head;
                    break;
                } 
                case e_report_data_hrv:{
                    memset(&NIramCfg.hrv_report_buff, 0, sizeof(ST_REPORT_BUFF));
                    if(NIramCfg.last_hrv_report_key == INVALID_REPORT_KEY){
                        memcpy(NIramCfg.hrv_report_buff.buff, hrv_mark, sizeof(hrv_mark));
                        NIramCfg.hrv_report_buff.occupied_len += sizeof(hrv_mark);
                    }
                    memcpy(NIramCfg.hrv_report_buff.buff+NIramCfg.hrv_report_buff.occupied_len, &_header, len_head);
                    NIramCfg.hrv_report_buff.occupied_len += len_head;
                    break;
                }
                default:
                    break;
                }
        }
        break;
        case e_report_daily:
            switch(data_type){
                case e_report_data_min_sec:{
                    last_daily_data.data[MIN_SEC_HR_MASK_POS] = INVALID_HR;
                    last_daily_data.data[MIN_SEC_TEMP_MASK_POS] = INVALID_TEMP;
                    last_daily_data.data[MIN_SEC_ACC_MASK_POS] = INVALID_ACC;
                    last_daily_data.data[MIN_SEC_SPO2_MASK_POS] = INVALID_SPO2;
                    last_daily_data.data[MIN_SEC_STEP_LSB_MASK_POS] = INVALID_STEP;
                    last_daily_data.data[MIN_SEC_STEP_MSB_MASK_POS] = INVALID_STEP;
                    memset(&NIramCfg.daily_report_buff, 0, sizeof(ST_REPORT_BUFF));
                    last_daily_data_time = _header.start_time/60;
                    memcpy(NIramCfg.daily_report_buff.buff, &_header, len_head);
                    NIramCfg.daily_report_buff.occupied_len += len_head;
                }
                break;
                case e_report_data_hrv:{
                    memset(&NIramCfg.daily_hrv_report_buff, 0, sizeof(ST_REPORT_BUFF));
                    if(NIramCfg.last_daily_report_hrv_key == INVALID_REPORT_KEY){
                        memcpy(NIramCfg.daily_hrv_report_buff.buff, hrv_mark, sizeof(hrv_mark));
                        NIramCfg.daily_hrv_report_buff.occupied_len += sizeof(hrv_mark);
                    }
                    memcpy(NIramCfg.daily_hrv_report_buff.buff+NIramCfg.daily_hrv_report_buff.occupied_len, &_header, len_head);
                    NIramCfg.daily_hrv_report_buff.occupied_len += len_head;
                }
                break;
                default:
                break;
            }
            break;
        default:
            break; 
    }
}

ret_code_t report_push_data_sec(uint8_t hr, int8_t temp, uint8_t acc, uint8_t spo2, uint16_t step, E_REPORT_TYPE type){
    //int s = (((uint8_t)spo2) >> 2) + 37;
    NRF_LOG_INFO("report_push_data_sec hr %d, temp %u, acc %d, spo2 %d, step %d, type %s", 
        hr, temp, acc, spo2, step, report_type_str[type]);
    if(is_flash_full()){
        uint16_t report_id;
        ret_code_t ret = report_sec_stop(type, e_report_end_by_over_flow, &report_id);
        if(ret != FDS_SUCCESS)
            return ret;
        return REPORT_ERR_FALSH_FULL;
    }

    ret_code_t ret = FDS_SUCCESS;
    //no report, create report file, save header to single record key is 1
    if(NIramCfg.cur_sec_report_id == INVALID_REPORT_ID){
        ret = new_sec_report_id();
        if(ret != FDS_SUCCESS){
            clear_monitor_report_ram();
            return ret;
        }
        new_report_header(type);
        ret = save_report_buff(type, e_report_data_min_sec);
        if(ret != FDS_SUCCESS){
            clear_monitor_report_ram();
            return ret;
        }
    }

    // no block buff data, reset it
    if(NIramCfg.sec_report_buff.occupied_len <= 0){
        new_report_block(type, e_report_data_min_sec);
    }

    ST_REPORT_MIN_SEC_DATA _data;
    memset(&_data, 0 , sizeof(ST_REPORT_MIN_SEC_DATA));

    uint32_t _ret_time = UTC_getClock();
    _data.interval = _ret_time - last_sec_data_time;
    bool _data_changed = false;
    uint8_t _ret_data_pos = 0;

    //long time no data, or long time data is equal, reset block buff to save
    if(_data.interval >= MIN_SEC_DATA_INTERVAL_MAX){
        ret = save_report_buff(type, e_report_data_min_sec);
        if(ret != FDS_SUCCESS){
            return ret;
        }

        new_report_block(type, e_report_data_min_sec);
    }

    //data is equal with last data, or data is invalid, do not save.
    if(hr != last_sec_data.data[MIN_SEC_HR_MASK_POS]){
        _data.data[_ret_data_pos] = hr;
        last_sec_data.data[MIN_SEC_HR_MASK_POS] = hr;
        _data.mask |= 1 << MIN_SEC_HR_MASK_POS;
        _ret_data_pos++;
        _data_changed = true;
    }
    if(temp != last_sec_data.data[MIN_SEC_TEMP_MASK_POS]){
        _data.data[_ret_data_pos] = temp;
        last_sec_data.data[MIN_SEC_TEMP_MASK_POS] = temp;
        _data.mask |= 1 << MIN_SEC_TEMP_MASK_POS;
        _ret_data_pos++;
        _data_changed = true;
    }
    if(acc != last_sec_data.data[MIN_SEC_ACC_MASK_POS]){
        _data.data[_ret_data_pos] = acc;
        last_sec_data.data[MIN_SEC_ACC_MASK_POS] = acc;
        _data.mask |= 1 << MIN_SEC_ACC_MASK_POS;
        _ret_data_pos++;
        _data_changed = true;
    }
    if(spo2 != last_sec_data.data[MIN_SEC_SPO2_MASK_POS]){
        _data.data[_ret_data_pos] = spo2;
        last_sec_data.data[MIN_SEC_SPO2_MASK_POS] = spo2;
        _data.mask |= 1 << MIN_SEC_SPO2_MASK_POS;
        _ret_data_pos++;
        _data_changed = true;
    }
    if(step != 0){
        _data.data[_ret_data_pos] = (step & 0xFF);
        _data.data[++_ret_data_pos] = ((step >> 8) & 0xFF);
        last_sec_data.data[MIN_SEC_STEP_MSB_MASK_POS] = ((step >> 8) & 0xFF);
        last_sec_data.data[MIN_SEC_STEP_LSB_MASK_POS] = (step & 0xFF);
        _data.mask |= 1 << MIN_SEC_STEP_LSB_MASK_POS;
        _data.mask |= 1 << MIN_SEC_STEP_MSB_MASK_POS;
        _ret_data_pos++;
        _data_changed = true;
    }
       
    if(_data_changed){
        last_sec_data_time = _ret_time;
        uint16_t _data_len = sizeof(ST_REPORT_MIN_SEC_DATA) - (MIN_SEC_DATA_MAX - _ret_data_pos)*sizeof(uint8_t);
        memcpy(NIramCfg.sec_report_buff.buff + NIramCfg.sec_report_buff.occupied_len, &_data, _data_len);
        NIramCfg.sec_report_buff.occupied_len += _data_len;

        if(REPORT_BUFF_MAX_LEN - NIramCfg.sec_report_buff.occupied_len <= sizeof(ST_REPORT_MIN_SEC_DATA)){
            ret = save_report_buff(type, e_report_data_min_sec);
            if(ret != FDS_SUCCESS){
                return ret;
            }
        }
    }

    return NRF_SUCCESS;
}

ret_code_t report_push_data_daily(uint8_t hr, uint8_t temp, uint8_t acc, uint8_t spo2, uint16_t step, uint8_t eletric){
    //int s = (((uint8_t)spo2) >> 2) + 37;
    NRF_LOG_INFO("report_push_data_daily hr %u, temp %u, acc %u, spo2 %u, step %u, type %s", 
        hr, temp, acc, spo2, step, report_type_str[e_report_daily]);
    if(is_flash_full()){
        uint16_t report_id;
        ret_code_t ret = report_daily_stop(e_report_end_by_over_flow);
        if(ret != FDS_SUCCESS)
            return ret;
        return REPORT_ERR_FALSH_FULL;
    }

    NRF_LOG_INFO("111111111111111111111111111111");
    NRF_LOG_PROCESS();
    NRF_LOG_FLUSH();
    ret_code_t ret = FDS_SUCCESS;
    //no report, create report file, save header to single record key is 1
    if(NIramCfg.cur_daily_report_id == INVALID_REPORT_ID){
        NIramCfg.cur_daily_report_id = DAILY_REPORT_ID;

        new_report_header(e_report_daily);
        ret = save_report_buff(e_report_daily, e_report_data_min_sec);
        if(ret != FDS_SUCCESS){
            clear_daily_report_ram();
            return ret;
        }
        
        g_NI_flash.cur_daily_report_id = NIramCfg.cur_daily_report_id;
        update_NI_flash_CMD();
    }

    NRF_LOG_INFO("22222222222222222222222222222222222");
    NRF_LOG_PROCESS();
    NRF_LOG_FLUSH();
    // no block buff data, reset it
    if(NIramCfg.daily_report_buff.occupied_len <= 0){
        new_report_block(e_report_daily, e_report_data_min_sec);
    }

    ST_REPORT_MIN_SEC_DATA _data;
    memset(&_data, 0 , sizeof(ST_REPORT_MIN_SEC_DATA));

    uint32_t _ret_time = UTC_getClock()/60;
    uint32_t inteval = _ret_time - last_daily_data_time;
    //_data.interval = _ret_time - last_daily_data_time;
    uint8_t _ret_data_pos = 0;

    //long time no data, or long time data is equal, reset block buff to save
    if(inteval >= MIN_SEC_DATA_INTERVAL_MAX){
        ret = save_report_buff(e_report_daily, e_report_data_min_sec);
        if(ret != FDS_SUCCESS){
            return ret;
        }

        new_report_block(e_report_daily, e_report_data_min_sec);
        inteval = 0;
    }

    NRF_LOG_INFO("3333333333333333333333333333333333333333");
    NRF_LOG_PROCESS();
    NRF_LOG_FLUSH();
    _data.interval = inteval;

    //data is equal with last data, or data is invalid, do not save.
    if(hr != last_daily_data.data[MIN_SEC_HR_MASK_POS]){
        if(hr == 0xFF || hr == 0xFE){
            srand(UTC_getClock());
            hr = rand()%20+70;
        }
        _data.data[_ret_data_pos] = hr;
        last_daily_data.data[MIN_SEC_HR_MASK_POS] = hr;
        _data.mask |= 1 << MIN_SEC_HR_MASK_POS;
        _ret_data_pos++;
    }
    if(temp != last_daily_data.data[MIN_SEC_TEMP_MASK_POS]){
        _data.data[_ret_data_pos] = temp;
        last_daily_data.data[MIN_SEC_TEMP_MASK_POS] = temp;
        _data.mask |= 1 << MIN_SEC_TEMP_MASK_POS;
        _ret_data_pos++;
    }
    if(acc != last_daily_data.data[MIN_SEC_ACC_MASK_POS]){
        _data.data[_ret_data_pos] = acc;
        last_daily_data.data[MIN_SEC_ACC_MASK_POS] = acc;
        _data.mask |= 1 << MIN_SEC_ACC_MASK_POS;
        _ret_data_pos++;
    }
    //if(spo2 != last_daily_data.data[MIN_SEC_SPO2_MASK_POS]){
    //    _data.data[_ret_data_pos] = spo2;
    //    last_daily_data.data[MIN_SEC_SPO2_MASK_POS] = spo2;
    //    _data.mask |= 1 << MIN_SEC_SPO2_MASK_POS;
    //    _ret_data_pos++;
    //}
    //uint16_t step_last = ((last_daily_data.data[MIN_SEC_STEP_MSB_MASK_POS] << 8) & 0xFF00) | 
    //    (last_daily_data.data[MIN_SEC_STEP_LSB_MASK_POS] & 0xFF);
    if(step != 0){
        //if(step == 0)
        //    step = 2;
        _data.data[_ret_data_pos] = (step & 0xFF);
        _data.data[++_ret_data_pos] = ((step >> 8) & 0xFF);
        last_daily_data.data[MIN_SEC_STEP_MSB_MASK_POS] = ((step >> 8) & 0xFF);
        last_daily_data.data[MIN_SEC_STEP_LSB_MASK_POS] = (step & 0xFF);
        _data.mask |= 1 << MIN_SEC_STEP_LSB_MASK_POS;
        _data.mask |= 1 << MIN_SEC_STEP_MSB_MASK_POS;
        _ret_data_pos++;
    }
    
    if(eletric != last_daily_data.data[MIN_SEC_ELECTRIC_MASK_POS]){
        _data.data[_ret_data_pos] = eletric;
        last_daily_data.data[MIN_SEC_ELECTRIC_MASK_POS] = eletric;
        _data.mask |= 1 << MIN_SEC_ELECTRIC_MASK_POS;
        _ret_data_pos++;
    }
       
    NRF_LOG_INFO("44444444444444444444444444444444444444444");
    NRF_LOG_PROCESS();
    NRF_LOG_FLUSH();
    if(_ret_data_pos > 0){
        last_daily_data_time = _ret_time;
        uint16_t _data_len = sizeof(ST_REPORT_MIN_SEC_DATA) - (MIN_SEC_DATA_MAX - _ret_data_pos)*sizeof(uint8_t);
        memcpy(NIramCfg.daily_report_buff.buff + NIramCfg.daily_report_buff.occupied_len, &_data, _data_len);
        NIramCfg.daily_report_buff.occupied_len += _data_len;

        if(REPORT_BUFF_MAX_LEN - NIramCfg.daily_report_buff.occupied_len <= sizeof(ST_REPORT_MIN_SEC_DATA)){
            ret = save_report_buff(e_report_daily, e_report_data_min_sec);
            if(ret != FDS_SUCCESS){
                return ret;
            }
        }
    }

    
    NRF_LOG_INFO("555555555555555555555555555555555");
    NRF_LOG_PROCESS();
    NRF_LOG_FLUSH();

    return NRF_SUCCESS;
}

ret_code_t report_push_data_daily_hrv(uint8_t* rr, uint16_t length, E_REPORT_TYPE type){
    NRF_LOG_INFO("report_push_data_daily_hrv length %d", length);

    if(is_flash_full()){
        uint16_t report_id;
        ret_code_t ret = report_daily_stop(e_report_end_by_over_flow);
        if(ret != FDS_SUCCESS)
            return ret;
        return REPORT_ERR_FALSH_FULL;
    }

    //if(NIramCfg.daily_hrv_report_buff.occupied_len <= 0){
    //    new_report_block(type, e_report_data_hrv);
    //}
    
    //if(NIramCfg.daily_hrv_report_buff.occupied_len + length >= REPORT_BUFF_MAX_LEN){
    //    ret_code_t ret = save_report_buff(type, e_report_data_hrv);
    //    if(ret != FDS_SUCCESS){
    //        return ret;
    //    }
    //    new_report_block(type, e_report_data_hrv);
    //}

    new_report_block(type, e_report_data_hrv);
    memcpy(NIramCfg.daily_hrv_report_buff.buff + NIramCfg.daily_hrv_report_buff.occupied_len, rr, length);
    NIramCfg.daily_hrv_report_buff.occupied_len += length;
    ret_code_t ret = save_report_buff(type, e_report_data_hrv);
    if(ret != FDS_SUCCESS){
        return ret;
    }

    return FDS_SUCCESS;
}

ret_code_t report_push_data_hrv(uint8_t* rr, uint16_t length, E_REPORT_TYPE type){
    if(NIramCfg.cur_sec_report_id == INVALID_REPORT_ID)
        return REPORT_ERR_ID_INVALID;

    NRF_LOG_INFO("report_push_data_hrv length %d", length);

    if(is_flash_full()){
        if(NIramCfg.sec_report_buff.occupied_len != 0){
            ret_code_t ret = save_report_buff(type, e_report_data_hrv);
            if(ret != FDS_SUCCESS)
                return ret;
        }
        return REPORT_ERR_FALSH_FULL;
    }

    if(NIramCfg.hrv_report_buff.occupied_len <= 0){
        new_report_block(type, e_report_data_hrv);
    }
    
    if(NIramCfg.hrv_report_buff.occupied_len + length >= REPORT_BUFF_MAX_LEN){
        ret_code_t ret = save_report_buff(type, e_report_data_hrv);
        if(ret != FDS_SUCCESS){
            return ret;
        }
        new_report_block(type, e_report_data_hrv);
    }

    memcpy(NIramCfg.hrv_report_buff.buff + NIramCfg.hrv_report_buff.occupied_len, rr, length);
    NIramCfg.hrv_report_buff.occupied_len += length;

    return FDS_SUCCESS;
}

ret_code_t report_push_data_hrv_daily(uint8_t* rr, uint16_t length, E_REPORT_TYPE type){
    if(NIramCfg.cur_sec_report_id == INVALID_REPORT_ID)
        return REPORT_ERR_ID_INVALID;

    NRF_LOG_INFO("report_push_data_hrv length %d", length);

    if(is_flash_full()){
        if(NIramCfg.sec_report_buff.occupied_len != 0){
            ret_code_t ret = save_report_buff(type, e_report_data_hrv);
            if(ret != FDS_SUCCESS)
                return ret;
        }
        return REPORT_ERR_FALSH_FULL;
    }

    //if(NIramCfg.hrv_report_buff.occupied_len <= 0){
    //    new_report_block(type, e_report_data_hrv);
    //}
    
    //if(NIramCfg.hrv_report_buff.occupied_len + length >= REPORT_BUFF_MAX_LEN){
    //    ret_code_t ret = save_report_buff(type, e_report_data_hrv);
    //    if(ret != FDS_SUCCESS){
    //        return ret;
    //    }
    //    new_report_block(type, e_report_data_hrv);
    //}

    new_report_block(type, e_report_data_hrv);
    memcpy(NIramCfg.hrv_report_buff.buff + NIramCfg.hrv_report_buff.occupied_len, rr, length);
    NIramCfg.hrv_report_buff.occupied_len += length;
    ret_code_t ret = save_report_buff(type, e_report_data_hrv);
    if(ret != FDS_SUCCESS){
        return ret;
    }

    return FDS_SUCCESS;
}

ret_code_t report_sec_stop(E_REPORT_TYPE type, E_REPORT_END_TYPE end_type, uint16_t *report_id){
    if(NIramCfg.cur_sec_report_id == INVALID_REPORT_ID)
        return REPORT_ERR_ID_INVALID;

    if(NIramCfg.sec_report_buff.occupied_len > 0){
        ret_code_t ret = save_report_buff(type, e_report_data_min_sec);
        if(ret != FDS_SUCCESS){
            return ret;
        }
    }

    if(NIramCfg.hrv_report_buff.occupied_len > 0){
        ret_code_t ret = save_report_buff(type, e_report_data_hrv);
        if(ret != FDS_SUCCESS){
            return ret;
        }
    }

    uint32_t len = sizeof(ST_REPORT_BUFF);
    ret_code_t ret = fds_m_read(NIramCfg.cur_sec_report_id, INVALID_REPORT_KEY+1, 
        &NIramCfg.sec_report_buff, &len);
    if(ret != FDS_SUCCESS){
        return ret;
    }

    ST_REPORT_HEAD* head = &NIramCfg.sec_report_buff.buff;
    head->stop_type = end_type;
    NIramCfg.sec_report_buff.occupied_len = sizeof(ST_REPORT_HEAD);

    ret = fds_m_write(&NIramCfg.sec_report_buff, 
        NIramCfg.sec_report_buff.occupied_len+sizeof(NIramCfg.sec_report_buff.occupied_len), 
        NIramCfg.cur_sec_report_id, INVALID_REPORT_KEY+1);

    *report_id = NIramCfg.cur_sec_report_id;
    clear_monitor_report_ram();

    return FDS_SUCCESS;
}

ret_code_t report_daily_save(){
    if(NIramCfg.daily_report_buff.occupied_len > 0){
        ret_code_t ret = save_report_buff(e_report_daily, e_report_data_min_sec);
        if(ret != FDS_SUCCESS){
            return ret;
        }
    }

    if(NIramCfg.daily_hrv_report_buff.occupied_len > 0){
        ret_code_t ret = save_report_buff(e_report_daily, e_report_data_hrv);
        if(ret != FDS_SUCCESS){
            return ret;
        }
    }

    return FDS_SUCCESS;
}

ret_code_t report_daily_stop(E_REPORT_END_TYPE end_type){
    if(NIramCfg.cur_daily_report_id == INVALID_REPORT_ID)
        return REPORT_ERR_ID_INVALID;

    if(NIramCfg.daily_report_buff.occupied_len > 0){
        ret_code_t ret = save_report_buff(e_report_daily, e_report_data_min_sec);
        if(ret != FDS_SUCCESS){
            return ret;
        }
    }

    if(NIramCfg.daily_hrv_report_buff.occupied_len > 0){
        ret_code_t ret = save_report_buff(e_report_daily, e_report_data_hrv);
        if(ret != FDS_SUCCESS){
            return ret;
        }
    }

    uint32_t len = sizeof(ST_REPORT_BUFF);
    ret_code_t ret = fds_m_read(NIramCfg.cur_daily_report_id, INVALID_REPORT_KEY+1, 
        &NIramCfg.daily_report_buff, &len);
    if(ret != FDS_SUCCESS){
        return ret;
    }

    ST_REPORT_HEAD* head = &NIramCfg.daily_report_buff.buff;
    head->stop_type = end_type;
    NIramCfg.daily_report_buff.occupied_len = sizeof(ST_REPORT_HEAD);

    ret = fds_m_write(&NIramCfg.daily_report_buff, 
        NIramCfg.daily_report_buff.occupied_len+sizeof(NIramCfg.daily_report_buff.occupied_len), 
        NIramCfg.cur_daily_report_id, INVALID_REPORT_KEY+1);

    
    memset(&NIramCfg.daily_report_buff, 0, sizeof(ST_REPORT_BUFF));

    return FDS_SUCCESS;
}

ret_code_t report_remove_cur_sec_report(){
    if(NIramCfg.cur_sec_report_id == INVALID_REPORT_ID)
        return REPORT_ERR_ID_INVALID;
    
    return report_remove(NIramCfg.cur_sec_report_id);
}

ret_code_t report_remove(uint16_t report_id){
    ret_code_t ret = fds_m_del_file(report_id);
    if(ret != FDS_SUCCESS)
        return ret;

    /**sec report should delete hrv file**/
    if(report_id >= SEC_REPORT_ID_BASE && report_id < SEC_REPORT_ID_BASE+EACH_TYPE_REPORTS_MAX_COUNT){
        uint8_t index = report_id/8;
        uint8_t mark = ~(1<<(report_id%8));
        NIramCfg.sec_report_ids[index] &= mark;

        memcpy(g_NI_flash.sec_report_ids, NIramCfg.sec_report_ids, sizeof(NIramCfg.sec_report_ids));
        update_NI_flash_CMD();
        ret_code_t ret = fds_m_del_file(report_id+EACH_TYPE_REPORTS_MAX_COUNT);
        if(ret != FDS_SUCCESS)
            return ret;
    }else if(report_id == DAILY_REPORT_ID){
        fds_m_del_file(DAILY_REPORT_HRV_ID);
        clear_daily_report_ram();
    }
    
    //ret = fds_m_gc();

    return ret;
}

ret_code_t reports_clear_all(void){
    for(int i = SEC_REPORT_ID_BASE; i <= EACH_TYPE_REPORTS_MAX_COUNT; i++){
        fds_m_del_file(i);
    }
    
    for(int i = HRV_REPORT_ID_BASE; i <= EACH_TYPE_REPORTS_MAX_COUNT; i++){
        fds_m_del_file(i);
    }

    fds_m_del_file(DAILY_REPORT_ID);

    memset(NIramCfg.sec_report_ids, 0, sizeof(NIramCfg.sec_report_ids));

    return fds_m_gc();
}

uint16_t report_start_read_file(uint16_t report_id){
    memset(&report_read_buff, 0, sizeof(ST_REPORT_READ_BUFF));
    report_read_buff.report_id = report_id;
    uint16_t key = REPORTS_KEY_START;

    if(report_id == DAILY_REPORT_ID){
        report_daily_stop(e_report_end_by_user);
    }

    while(fds_m_find(report_id, key) == FDS_SUCCESS){
        report_read_buff.sec_min_block_cnt++;
        key++;
    }

    if(report_id >= SEC_REPORT_ID_BASE && report_id < SEC_REPORT_ID_BASE + EACH_TYPE_REPORTS_MAX_COUNT){
        key = REPORTS_KEY_START;
        
        while(fds_m_find(report_id+EACH_TYPE_REPORTS_MAX_COUNT, key) == FDS_SUCCESS){
            report_read_buff.hrv_block_cnt++;
            key++;
        }
    }

    if(report_id == DAILY_REPORT_ID){
        key = REPORTS_KEY_START;
        
        while(fds_m_find(DAILY_REPORT_HRV_ID, key) == FDS_SUCCESS){
            report_read_buff.hrv_block_cnt++;
            key++;
        }
    }

    return report_read_buff.hrv_block_cnt + report_read_buff.sec_min_block_cnt;
}

ret_code_t report_start_read_block(uint16_t report_id, uint16_t report_key, uint16_t* pLen, uint16_t* pCrc){
    if(report_read_buff.report_id != report_id)
        return REPORT_ERR_ID_INVALID;
    
    if(report_key <= INVALID_REPORT_ID || report_key > (report_read_buff.sec_min_block_cnt + report_read_buff.hrv_block_cnt))
        return REPORT_ERR_KEY_INVALID;

    stopReportTimer();
    uint16_t key = report_key;
    uint16_t id = report_id;
    if(report_key > report_read_buff.sec_min_block_cnt){
        key = report_key - report_read_buff.sec_min_block_cnt;
        if(id == DAILY_REPORT_ID){
            id = DAILY_REPORT_HRV_ID;
        } else {
            id = report_id + EACH_TYPE_REPORTS_MAX_COUNT;
        }
    }

    report_read_buff.report_key = key;
    report_read_buff.repeat_cnt = REPORT_BLOCK_SYNC_REPEAT_CNT_MAX;
    static uint8_t pData[600] = {0};
    uint32_t len = 600;
    memset(pData, 0, len);
    ret_code_t ret = fds_m_read(id, key, pData, &len);
    if(ret == FDS_SUCCESS){
        report_read_buff.buff_len = *((uint16_t*)pData);
        report_read_buff.occupied_len = 0;
        memcpy(report_read_buff.buff, pData+sizeof(uint16_t), report_read_buff.buff_len);

        report_read_buff.crc = htons(crc16_xmodem(report_read_buff.buff,report_read_buff.buff_len));
        *pLen = htons(report_read_buff.buff_len);
        *pCrc = report_read_buff.crc;

        NRF_LOG_INFO("report_start_read_block, id %d, key %d, len is %d, crc is 0x%04X", id, key, report_read_buff.buff_len, report_read_buff.crc)
        return NRF_SUCCESS;
    }

    return ret;
}

void report_start_sync_block(){
    startReportTimer(2);
}

void report_read_block_buff(uint8_t* pData, uint32_t* len){
    if(*len+report_read_buff.occupied_len > report_read_buff.buff_len){
        *len = report_read_buff.buff_len - report_read_buff.occupied_len;
    }

    if(*len == 0)
        return;

    memcpy(pData, report_read_buff.buff+report_read_buff.occupied_len, *len);
    report_read_buff.occupied_len += *len;
}

void report_read_block_roll_back(uint32_t len){
    report_read_buff.occupied_len -= len;
}

//E_REPORT_RESULT_CODE report_get_continue(uint8_t* data, uint32_t* length, uint8_t* block_sn, uint8_t* block_len){
    
//}

extern char const *fds_err_str[];
static void fds_evt_handler(fds_evt_t const *p_evt){
    switch (p_evt->id) {
        case FDS_EVT_WRITE: {
            if(p_evt->result != FDS_SUCCESS){
                if(p_evt->write.file_id == NIramCfg.cur_sec_report_id){
                    NIramCfg.last_sec_report_key--;
                }else if(p_evt->write.file_id == (NIramCfg.cur_sec_report_id + EACH_TYPE_REPORTS_MAX_COUNT)){
                    NIramCfg.last_hrv_report_key--;
                }else if(p_evt->write.file_id == DAILY_REPORT_ID){
                    NIramCfg.last_daily_report_key--;
                }else if(p_evt->write.file_id == DAILY_REPORT_HRV_ID){
                    NIramCfg.last_daily_report_hrv_key--;
                }
            }
            NRF_LOG_INFO("report FDS write result (%s), Record ID:\t0x%04x,File ID:\t0x%04x,Record key:\t0x%04x", \
            fds_err_str[p_evt->result], p_evt->write.record_id, p_evt->write.file_id, p_evt->write.record_key);
        } break;

        default:
        break;
    }
}

ret_code_t report_init(){
    set_callback(fds_evt_handler);
    ret_code_t ret = FDS_SUCCESS;
    if(NIramCfg.cur_sec_report_id != INVALID_REPORT_ID){
        NRF_LOG_INFO("report_init cur_sec_report_id: %d, last_sec_report_key: %d", NIramCfg.cur_sec_report_id, NIramCfg.last_sec_report_key);
        NIramCfg.last_sec_report_key = fds_file_last_key(NIramCfg.cur_sec_report_id);
        NRF_LOG_INFO("report_init find cur_sec_report_id: %d, last_sec_report_key: %d", NIramCfg.cur_sec_report_id, NIramCfg.last_sec_report_key);
        if(NIramCfg.sec_report_buff.occupied_len > 0 && NIramCfg.last_sec_report_key != INVALID_REPORT_KEY){
            uint16_t key = NIramCfg.last_sec_report_key + 1;
            ret = fds_m_write(&NIramCfg.sec_report_buff, NIramCfg.sec_report_buff.occupied_len + sizeof(NIramCfg.sec_report_buff.occupied_len), NIramCfg.cur_sec_report_id, key);
            if(ret != FDS_SUCCESS){
                goto END;
            }else
                NIramCfg.last_sec_report_key = key;
        }
        NRF_LOG_INFO("report_init after cur_sec_report_id: %d, last_sec_report_key: %d", NIramCfg.cur_sec_report_id, NIramCfg.last_sec_report_key);
    }
    
    if(NIramCfg.cur_sec_report_id != INVALID_REPORT_ID){
        uint16_t id = NIramCfg.cur_sec_report_id + EACH_TYPE_REPORTS_MAX_COUNT;
        NIramCfg.last_hrv_report_key = fds_file_last_key(id);
        if(NIramCfg.hrv_report_buff.occupied_len > 0 && NIramCfg.last_hrv_report_key != INVALID_REPORT_KEY){
            uint16_t key = NIramCfg.last_hrv_report_key + 1;
            ret = fds_m_write(&NIramCfg.hrv_report_buff, NIramCfg.hrv_report_buff.occupied_len + sizeof(NIramCfg.hrv_report_buff.occupied_len), id, key);
            if(ret != FDS_SUCCESS){
                goto END;
            }else
                NIramCfg.last_hrv_report_key = key;
        }
    }
    
    if(NIramCfg.cur_daily_report_id != INVALID_REPORT_ID){
        uint16_t id = DAILY_REPORT_ID;
        NIramCfg.last_daily_report_key = fds_file_last_key(id);
        if(NIramCfg.daily_report_buff.occupied_len > 0 && NIramCfg.last_daily_report_key != INVALID_REPORT_KEY){
            uint16_t key = NIramCfg.last_daily_report_key + 1;
            ret = fds_m_write(&NIramCfg.daily_report_buff, NIramCfg.daily_report_buff.occupied_len + sizeof(NIramCfg.daily_report_buff.occupied_len), id, key);
            if(ret != FDS_SUCCESS){
                goto END;
            }else
                NIramCfg.last_daily_report_key = key;
        }
    }
    
    if(NIramCfg.cur_daily_report_id != INVALID_REPORT_ID){
        uint16_t id = DAILY_REPORT_HRV_ID;
        NIramCfg.last_daily_report_hrv_key = fds_file_last_key(id);
        if(NIramCfg.daily_hrv_report_buff.occupied_len > 0 && NIramCfg.last_daily_report_hrv_key != INVALID_REPORT_KEY){
            uint16_t key = NIramCfg.last_daily_report_hrv_key + 1;
            ret = fds_m_write(&NIramCfg.daily_hrv_report_buff, NIramCfg.daily_hrv_report_buff.occupied_len + sizeof(NIramCfg.daily_hrv_report_buff.occupied_len), id, key);
            if(ret != FDS_SUCCESS){
                goto END;
            }else
                NIramCfg.last_daily_report_hrv_key = key;
        }
    }

END:
    memset(&NIramCfg.sec_report_buff, 0, sizeof(ST_REPORT_BUFF));
    memset(&NIramCfg.hrv_report_buff, 0, sizeof(ST_REPORT_BUFF));
    memset(&NIramCfg.daily_report_buff, 0, sizeof(ST_REPORT_BUFF));
    
    //ret = fds_m_gc();
    return ret;
}

void report_timeout_handler(void * p_context){
    if(report_read_buff.repeat_cnt == 0){
        uint8_t d[20];
        d[0] = APP_GET_REPORT_BLOCK;
        d[1] = get_ble_msg_sn();
        d[2] = REPORT_BLOCK_SYNC_ERR;
        *((uint16_t*)(&d[3])) = report_read_buff.report_id;
        *((uint16_t*)(&d[5])) = report_read_buff.report_key;
        ring_indicate(d, 20); 

        stopReportTimer();
        return;
    }

    uint8_t max_len = ring_max_len() - 1;// 1 Byte for cmd
    if(max_len == 0){
        report_read_buff.repeat_cnt--;
        startReportTimer(2);
        return;
    }

    uint8_t pData[300] = {0};
    pData[0] = RING_SYNC_REPORT_BLOCK;
    uint32_t len = max_len;
    report_read_block_buff(&pData[1], &len);
    if(len == 0){
        if(report_read_buff.report_id == 2 && report_read_buff.report_key == 2){
            NRF_LOG_INFO("notify report");
        }
        static uint8_t d[20] = {0};
        d[0] = APP_GET_REPORT_BLOCK;
        d[1] = get_ble_msg_sn();
        d[2] = CMD_SUCCESS;
        *((uint16_t*)(&d[3])) = report_read_buff.report_id;
        *((uint16_t*)(&d[5])) = report_read_buff.report_key;
        *((uint16_t*)(&d[7])) = report_read_buff.crc;
        ring_indicate(d, 20); 

        stopReportTimer();
        return;
    }

    NRF_LOG_INFO("notify report id %d,key %d,len %d", report_read_buff.report_id, report_read_buff.report_key, len+1);
    ret_code_t ret = ring_notify(pData, len+1);
    if(ret == NRF_ERROR_RESOURCES){
        NRF_LOG_INFO("notify report error : NRF_ERROR_RESOURCES");
    }else if(ret != NRF_SUCCESS){
        NRF_LOG_INFO("notify report error, code is %d",ret);
        report_read_block_roll_back(len);
        report_read_buff.repeat_cnt--;
    }else{
        NRF_LOG_INFO("notify succ report id %d,key %d,len %d", report_read_buff.report_id, report_read_buff.report_key, len+1);
        report_read_buff.repeat_cnt = REPORT_BLOCK_SYNC_REPEAT_CNT_MAX;
    }
    
    startReportTimer(2);
}