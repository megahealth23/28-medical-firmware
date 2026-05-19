#include "state_module.h"
#include "mTask.h"
#include "ring_utc.h"
#include "inner_flash_ram.h"
#include "DataProcSPO2.h"
#include "reports.h"
#include "ble_ring.h"
#include "peripheral/peripherals.h"
#include "AFE4900/afe4900_defs.h"
#include "nrf_delay.h"
#include "DataProcHealth.h"

extern NIRAM_t NIramCfg;
uint16_t daily_mark_step = INVALID_STEP;
static bool rt_en = false; //true: send real time data, false: not send real time data
uint32_t s_sys_module_evt;
uint32_t s_sys_report_evt;
uint32_t s_sys_connect_evt;
uint32_t s_sys_ni_flash_evt;
static uint32_t cur_auto_sleep_start_time;
static uint32_t cur_auto_sleep_end_time;

static bool hw_check(){
    bool bRet = true;
    //check afe4900
    if (afe_start(RING_AFE_SPO2_MODE)) {
        struct afe_dev_info_s afe_dev_info = show_afe_dev_info();
        ring_set_sensor_status(AFE_DRV_OK, (1 == afe_dev_info.i2c_line) ? true : false);
        if(1 != afe_dev_info.i2c_line)
            bRet = false;
        afe_stop();
    }else
        bRet = false;

    //check g-sensor
    if (acc_start(IMU_STANDBY)) {
        struct acc_dev_info_s acc_dev_info = show_acc_dev_info();
        ring_set_sensor_status(GSENSOR_DRV_OK, (1 == acc_dev_info.i2c_line) ? true : false);
        if(1 != acc_dev_info.i2c_line)
            bRet = false;
        acc_start(IMU_POWEROFF);
    }else
        bRet = false;


    return bRet;
}

ret_code_t base_module_init(){
    //if(!peripherals_check()){
    //    NRF_LOG_INFO("base_module_init hw_check failed!!");
    //    return STATE_MODULE_HW_CHECK_FAILED;
    //}

    fds_m_gc();
    if(is_flash_full()){
        NRF_LOG_INFO("base_module_init flash is full!!");
        return STATE_MODULE_FLASH_FULL;
    }

    if(NIramCfg.cur_module_start_time < 10000)
        NIramCfg.cur_module_start_time = UTC_getClock();
    return STATE_MODULE_SUCCESS;
}

void base_module_uninit(){
    advertising_start(false);
    ring_topled_off();
    NIramCfg.cur_module_start_time = 0;
}

void base_module_process_sys(void *PMsg){

}

void base_module_process_i2c(void *PMsg){

}

bool base_module_process_ble(uint8_t *pR, uint8_t *out){
    return false;
}

ret_code_t processHRV(E_REPORT_TYPE type) {
    if(!HRV_isready_flg())
        return NRF_ERROR_BUSY;
    // ret_code_t ret = report_push_data_hrv((uint8_t*)(&HRV_vect[0]), MAX_HRV_CNT*2, type);
    ret_code_t ret = report_push_data_hrv((uint8_t*)(&HRV_vect[0]), MAX_HRV_CNT, type);
    // ret = report_push_data_hrv((uint8_t*)(&HRV_vect[MAX_HRV_CNT/2]), MAX_HRV_CNT, type);
    HRV_cnt_clear();
    if(ret != NRF_SUCCESS)
        return ret;

    return NRF_SUCCESS;
}


 #define HRV_HEALTH_BLOCK_CNT 200 //200 *sizeof(uint_16) = 400 bytes 

ret_code_t processHRV_1(E_REPORT_TYPE type) {
    if(hrv_health_process_result_get() != NULL)
    {
        NRF_LOG_INFO("hrv_health_process_1 success!!");
        HRV_Para* hrvHealth = hrv_health_process_result_get();
        if(hrvHealth != NULL)
        {
            for(int i = 0;i < hrvHealth->RR_num;i= i+HRV_HEALTH_BLOCK_CNT)
            {
                if(i+HRV_HEALTH_BLOCK_CNT >= hrvHealth->RR_num)
                {
                    report_push_data_hrv_daily((uint8_t*)(&hrvHealth->green_RR[i]), (hrvHealth->RR_num - i)*2, type);
                    break;
                }
                else{
                    report_push_data_hrv_daily((uint8_t*)(&hrvHealth->green_RR[i]), HRV_HEALTH_BLOCK_CNT*2, type);
                }
                
            }

        }

    }

    return NRF_SUCCESS;
}

uint8_t BASE_type(){
    return 0;
}

uint8_t get_raw_data_word(){
    switch ((RING_AFE_HRV_ON_MODE-1) & (int)afe_get_work_mode()) {
    case RING_AFE_SPO2_MODE:
    case RING_AFE_BP_MODE:
        return SPO2_WORD;
    case RING_AFE_SPORT_MODE:
        return EHR_WORD;
    case RING_AFE_DAILY_MODE:
        return EHR_WORD;
    //case RING_AFE_PULSE_MODE:
    //    return PULSE_WORD;
    default:
        return NONE_WORD;
    }
}

void send_raw_data(struct afe4900_sensor_data* data, uint16_t len){
    if (is_enable_rawdata() && data != NULL && len > 0){
        uint8_t raw_type = get_raw_data_word();
        if(raw_type != NONE_WORD){
            copy_to_raw_data(raw_type, (void *)data, len/2);
            copy_to_raw_data(raw_type, (void *)&data[len/2], len/2);
        }
    }
}

//
void send_acc_raw_data(acc_axis* data, uint8_t len){
    uint8_t d[182] = {0};
    d[0] = ACC_WORD;
    d[1] = ACC_WORD+1;
    d[2] = ACC_WORD+2;
    d[3] = ACC_WORD+3;
    d[4] = len;
    memcpy(d+5, data, len*sizeof(acc_axis));
    uint8_t _len = len > 180 ? 180 : len;
    aux_raw_notify(d, len*sizeof(acc_axis)+5);
    nrf_delay_ms(10);
}

void send_hrv_data(uint8_t* data, uint8_t len){
    uint8_t d[182] = {0};
    d[0] = HRV_WORD;
    d[1] = len;
    memcpy(d+2, data, len*sizeof(uint8_t));
    aux_raw_notify(d, 20);
    nrf_delay_ms(10);
}

void set_rt_en(bool en){
    rt_en = en;
}

bool get_rt_en(){
    return rt_en;
}

void sendMonitorData(uint8_t module_type) {
    NRF_LOG_INFO("sendMonitorData");
    uint32_t rslt;
    uint32_4bytes_t t;

    uint8_t resp[NORMAL_NOTIFY_LEN] = {0};

    resp[0] = APP_GET_RT_DATA_CMD; // In app, MegaBleConfig.CMD_LIVECTRL:
    resp[2] = module_type;

    resp[3] = g_afe.flag;
    resp[5] = g_afe.shr;
    if(module_type == e_EXER || module_type == e_PA)
        resp[5] = g_afe.ehr_hr;

    t.value.bytes = UTC_getClock() - NIramCfg.cur_module_start_time;
    resp[6] = t.value.field.byte3;
    resp[7] = t.value.field.byte2;
    resp[8] = t.value.field.byte1;
    resp[9] = t.value.field.byte0;
    resp[13] = get_batt_vol_percent();

    if(module_type == e_PS || 
        module_type == e_REAL){
        resp[4] = (g_afe.Spo2show >> 2) + 37;
        resp[5] = g_afe.shr;
        resp[10] = g_afe.tiredflg; // a[10]== tiredValue in the BLE SDK.
	
        NRF_LOG_INFO("sendMonitorData hr: %d, spo2: %d!", resp[5], resp[4]);
#define ACCX 14
#define ACCY 16
#define ACCZ 18
        resp[ACCX] = accdata.ACCx >> 8;
        resp[ACCX + 1] = accdata.ACCx & 0xFF;
        resp[ACCY] = accdata.ACCy >> 8;
        resp[ACCY + 1] = accdata.ACCy & 0xFF;
        resp[ACCZ] = accdata.ACCz >> 8;
        resp[ACCZ + 1] = accdata.ACCz & 0xFF;

        resp[19] = acc_flag & 0xFF; 
    }

    if(module_type == e_EXER){
        resp[19] = steps_num();
    }

    rslt = ring_notify(resp, NORMAL_NOTIFY_LEN);
    if (rslt != NRF_SUCCESS)
        NRF_LOG_INFO("sendMonitorData error!\n");

    return;
}

void sendMonitorData_daily() {
    uint32_t rslt;
    uint32_4bytes_t t;

    uint8_t resp[NORMAL_NOTIFY_LEN] = {0};

    resp[0] = APP_GET_RT_DATA_CMD; // In app, MegaBleConfig.CMD_LIVECTRL:
    resp[2] = e_DAILY;

    resp[3] = g_afe.flag;
    resp[5] = g_afe.point_hr;

    t.value.bytes = UTC_getClock() - NIramCfg.cur_module_start_time;
    resp[6] = t.value.field.byte3;
    resp[7] = t.value.field.byte2;
    resp[8] = t.value.field.byte1;
    resp[9] = t.value.field.byte0;


    uint16_t s = steps_num();
    resp[10] = s & 0xFF;
    resp[11] = (s >> 8) & 0xFF;

    resp[13] = get_batt_vol_percent();

    rslt = ring_notify(resp, NORMAL_NOTIFY_LEN);
    if (rslt != NRF_SUCCESS)
        NRF_LOG_INFO("sendMonitorData error!\n");

    return;
}

extern uint8_t sub_exercise_type;
void sendMonitorData_exer(uint16_t step) {
    uint32_t rslt;
    uint32_4bytes_t t;

    uint8_t resp[NORMAL_NOTIFY_LEN] = {0};

    resp[0] = APP_GET_RT_DATA_CMD; // In app, MegaBleConfig.CMD_LIVECTRL:
    resp[2] = e_EXER;

    resp[3] = g_afe.flag;
    resp[4] = NIramCfg.cur_sub_module;
    resp[5] = g_afe.ehr_hr;

    t.value.bytes = UTC_getClock() - NIramCfg.cur_module_start_time;
    resp[6] = t.value.field.byte3;
    resp[7] = t.value.field.byte2;
    resp[8] = t.value.field.byte1;
    resp[9] = t.value.field.byte0;
    resp[13] = get_batt_vol_percent();

    resp[19] = step;

    rslt = ring_notify(resp, NORMAL_NOTIFY_LEN);
    if (rslt != NRF_SUCCESS)
        NRF_LOG_INFO("sendMonitorData error!\n");

    return;
}

void sendMonitorData_auto_exer(uint16_t step) {
    uint32_t rslt;
    uint32_4bytes_t t;

    uint8_t resp[NORMAL_NOTIFY_LEN] = {0};

    resp[0] = APP_GET_RT_DATA_CMD; // In app, MegaBleConfig.CMD_LIVECTRL:
    resp[2] = e_PA;

    resp[3] = g_afe.flag;
    resp[5] = g_afe.ehr_hr;

    t.value.bytes = UTC_getClock() - NIramCfg.cur_module_start_time;
    resp[6] = t.value.field.byte3;
    resp[7] = t.value.field.byte2;
    resp[8] = t.value.field.byte1;
    resp[9] = t.value.field.byte0;
    resp[13] = get_batt_vol_percent();

    resp[19] = step;

    rslt = ring_notify(resp, NORMAL_NOTIFY_LEN);
    if (rslt != NRF_SUCCESS)
        NRF_LOG_INFO("sendMonitorData error!\n");

    return;
}

void sendMonitorData_stress() {
    uint32_t rslt;
    uint32_4bytes_t t;

    uint8_t resp[NORMAL_NOTIFY_LEN] = {0};

    resp[0] = APP_GET_RT_DATA_CMD; // In app, MegaBleConfig.CMD_LIVECTRL:
    resp[2] = e_STRESS;

    resp[3] = g_afe.flag;
    resp[5] = g_afe.ehr_hr;

    t.value.bytes = UTC_getClock() - NIramCfg.cur_module_start_time;
    resp[6] = t.value.field.byte3;
    resp[7] = t.value.field.byte2;
    resp[8] = t.value.field.byte1;
    resp[9] = t.value.field.byte0;

    rslt = ring_notify(resp, NORMAL_NOTIFY_LEN);
    if (rslt != NRF_SUCCESS)
        NRF_LOG_INFO("sendMonitorData error!\n");

    return;
}

void temp_read(uint16_t period){
    if(UTC_getClock() % period == 0){
        i2c_init();
        struct tmp117_sensor_data *tmp_pt;
        read_tmp_value(&tmp_pt, RING_INSIDE_TMP117, true, RING_TMP_ONE_SHOT_MODE);
        NRF_LOG_INFO("temp_val is %d, temprature:%u", tmp_pt->temp_val, gCurrent_body_temperature());
        tmp_start(RING_TMP_ONE_SHOT_MODE, RING_INSIDE_TMP117);
        i2c_uninit();
    }
}

void reset_cur_auto_sleep_time(){
    if(NIramCfg.auto_sleep_mark != 1)
        return;

    uint32_t now = UTC_getClock();
    uint16_t auto_start = ((NIramCfg.auto_sleep_start_hour & 0xFF) << 8) | (NIramCfg.auto_sleep_start_min & 0xFF);
    uint16_t auto_end = ((NIramCfg.auto_sleep_end_hour & 0xFF) << 8) | (NIramCfg.auto_sleep_end_min & 0xFF);

    UTCTimeStruct tm_start = {0};
    UTC_convertUTCTime_1(&tm_start, UTC_getClock(), NIramCfg.auto_sleep_timezone_hour, NIramCfg.auto_sleep_timezone_min);
    tm_start.hour = NIramCfg.auto_sleep_start_hour & 0xFF;
    tm_start.minutes = NIramCfg.auto_sleep_start_min & 0xFF;
    UTCTime t_start = UTC_convertUTCSecs_1(&tm_start, NIramCfg.auto_sleep_timezone_hour, NIramCfg.auto_sleep_timezone_min);

    UTCTimeStruct tm_end = {0};
    UTC_convertUTCTime_1(&tm_end, UTC_getClock(), NIramCfg.auto_sleep_timezone_hour, NIramCfg.auto_sleep_timezone_min);
    tm_end.hour = NIramCfg.auto_sleep_end_hour & 0xFF;
    tm_end.minutes = NIramCfg.auto_sleep_end_min & 0xFF;
    UTCTime t_end = UTC_convertUTCSecs_1(&tm_end, NIramCfg.auto_sleep_timezone_hour, NIramCfg.auto_sleep_timezone_min);

    if(now > t_end){
        t_end += 24*60*60;
        if(auto_end > auto_start){
            t_start += 24*60*60;
            NRF_LOG_INFO("current time is %d, t_start : %d, t_end : %d", now, t_start, t_end);
        }else{
            NRF_LOG_INFO("current time is %d, t_start : %d, t_end : %d", now, t_start, t_end);
        }
    }else{
        if(auto_end > auto_start){
            NRF_LOG_INFO("current time is %d, t_start : %d, t_end : %d", now, t_start, t_end);
        }else{
            t_start -= 24*60*60;
            NRF_LOG_INFO("current time is %d, t_start : %d, t_end : %d", now, t_start, t_end);
        }
    }

    cur_auto_sleep_start_time = t_start;
    cur_auto_sleep_end_time = t_end;
}

bool check_auto_sleep_start_3(){
    if(NIramCfg.auto_sleep_mark != 1)
        return false;

    if(NIramCfg.last_auto_sleep_close_time > cur_auto_sleep_start_time)
            return false;

    uint32_t now = UTC_getClock();
    if(now > cur_auto_sleep_start_time && now < cur_auto_sleep_end_time){
        return true;
    }

    return false;
}

bool check_auto_sleep_end_3(){
    if(NIramCfg.auto_sleep_mark != 1)
        return false;

    uint32_t now = UTC_getClock();
    if(now > cur_auto_sleep_end_time || now < cur_auto_sleep_start_time){
        return true;
    }

    return false;
}

void start_auto_sleep(){
    //if(NIramCfg.high_precision_mark == 0){
        switch_module(e_DS, e_report_end_by_timer);
    //}else{
    //    switch_module(e_LPS, e_report_end_by_timer);
    //}
}

//bool check_auto_sleep_start(){
//    if(NIramCfg.auto_sleep_mark != 1)
//        return false;

//    if(UTC_getClock() % AUTO_SLEEP_CHECK_PERIOD == 0){
//        uint16_t start = ((NIramCfg.auto_sleep_start_hour & 0xFF) << 8) | (NIramCfg.auto_sleep_start_min & 0xFF);
//        uint16_t end = ((NIramCfg.auto_sleep_end_hour & 0xFF) << 8) | (NIramCfg.auto_sleep_end_min & 0xFF);
//        UTCTimeStruct tm_start = {0};
//        UTC_convertUTCTime_1(&tm_start, UTC_getClock(), NIramCfg.auto_sleep_timezone_hour, NIramCfg.auto_sleep_timezone_min);
//        tm_start.hour = NIramCfg.auto_sleep_start_hour & 0xFF;
//        tm_start.minutes = NIramCfg.auto_sleep_start_min & 0xFF;
//        UTCTime t_start = UTC_convertUTCSecs_1(&tm_start, NIramCfg.auto_sleep_timezone_hour, NIramCfg.auto_sleep_timezone_min);
//        if(end > start){
//            NRF_LOG_INFO("current time is %d, t_start : %d, last_stop_time : %d", UTC_getClock(), t_start, NIramCfg.last_auto_sleep_close_time);
//            if(NIramCfg.last_auto_sleep_close_time >= t_start)
//                return false;
//        }else{
//            if(UTC_getClock() < t_start){
//                t_start -= 24*60*60;
//            }
//            NRF_LOG_INFO("current time is %d, t_start : %d, last_stop_time : %d", UTC_getClock(), t_start, NIramCfg.last_auto_sleep_close_time);
//            if(NIramCfg.last_auto_sleep_close_time >= t_start)
//                return false;
//        }

//        UTCTimeStruct tm = {0};
//        UTC_convertUTCTime_1(&tm, UTC_getClock(), NIramCfg.auto_sleep_timezone_hour, NIramCfg.auto_sleep_timezone_min);
//        uint16_t now = ((tm.hour & 0xFF) << 8) | (tm.minutes & 0xFF);
//        NRF_LOG_INFO("current time is 0x%04X, start : 0x%04X, end : 0x%04X", now, start, end);
//        if(end > start){
//            if(now >= start && now < end){
//                if(NIramCfg.high_precision_mark == 0){
//                    switch_module(e_DS, e_report_end_by_timer);
//                }else{
//                    switch_module(e_LPS, e_report_end_by_timer);
//                }
//                return true;
//            }
//        }else{
//            if(now >= start || now < end){
//                if(NIramCfg.high_precision_mark == 0){
//                    switch_module(e_DS, e_report_end_by_timer);
//                }else{
//                    switch_module(e_LPS, e_report_end_by_timer);
//                }

//                return true;
//            }
//        }
//    }

//    return false;
//}

//void check_auto_sleep_end(){
//    if(NIramCfg.auto_sleep_mark != 1)
//        return;

//    if(UTC_getClock() % AUTO_SLEEP_CHECK_PERIOD == 0){
//        uint16_t start = ((NIramCfg.auto_sleep_start_hour & 0xFF) << 8) | (NIramCfg.auto_sleep_start_min & 0xFF);
//        uint16_t end = ((NIramCfg.auto_sleep_end_hour & 0xFF) << 8) | (NIramCfg.auto_sleep_end_min & 0xFF);
//        UTCTimeStruct tm = {0};
//        UTC_convertUTCTime_1(&tm, UTC_getClock(), NIramCfg.auto_sleep_timezone_hour, NIramCfg.auto_sleep_timezone_min);
//        uint16_t now = ((tm.hour & 0xFF) << 8) | (tm.minutes & 0xFF);
//        NRF_LOG_INFO("current time is 0x%04X, start : 0x%04X, end : 0x%04X", now, start, end);
//        if(end > start){
//            if(now < start || now >= end){
//                switch_module(e_DAILY, e_report_end_by_timer);
//            }
//        }else{
//            if(now < start && now >= end){
//                switch_module(e_DAILY, e_report_end_by_timer);
//            }
//        }
//    }
//}

//bool check_auto_sleep_end_1(){
//    //if(NIramCfg.auto_sleep_mark != 1)
//    //    return true;

//    if(UTC_getClock() % AUTO_SLEEP_CHECK_PERIOD == 0){
//        uint16_t start = ((NIramCfg.auto_sleep_start_hour & 0xFF) << 8) | (NIramCfg.auto_sleep_start_min & 0xFF);
//        uint16_t end = ((NIramCfg.auto_sleep_end_hour & 0xFF) << 8) | (NIramCfg.auto_sleep_end_min & 0xFF);
//        UTCTimeStruct tm = {0};
//        UTC_convertUTCTime_1(&tm, UTC_getClock(), NIramCfg.auto_sleep_timezone_hour, NIramCfg.auto_sleep_timezone_min);
//        uint16_t now = ((tm.hour & 0xFF) << 8) | (tm.minutes & 0xFF);
//        NRF_LOG_INFO("current time is 0x%04X, start : 0x%04X, end : 0x%04X", now, start, end);
//        if(end > start){
//            if(now < start || now >= end){
//                //switch_module(e_DAILY, e_report_end_by_timer);
//                return true;
//            }
//        }else{
//            if(now < start && now >= end){
//                //switch_module(e_DAILY, e_report_end_by_timer);
//                return true;
//            }
//        }
//    }

//    return false;
//}