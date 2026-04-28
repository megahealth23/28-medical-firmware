#include "state_module.h"
#include "ring_afe.h"
#include "nrf_log.h"
#include "ble_ring.h"
#include "protocol.h"
#include "ble_service.h"
#include "inner_flash_ram.h"
#include "ring_startup.h"
#include "DataProcHealth.h"

extern NIRAM_t NIramCfg;
#define LPS_TEMP_PERIOD      (30)     //second
#define LPS_MONITOR_PERIOD   (10*60)    //second
#define LPS_MONITOR_DURATION (90)       //second

uint32_t lps_monitor_start_time = 0;
static bool start_auto_end_sleep_check = false;

static void send_raw_data_1(struct afe4900_sensor_data* data, uint16_t len){
    if (is_enable_rawdata() && data != NULL && len > 0){
        uint8_t raw_type = SPO2_1_WORD;
        if(raw_type != NONE_WORD){
            copy_to_raw_data(raw_type, (void *)data, len/5);
            nrf_delay_ms(10);
            copy_to_raw_data(raw_type, (void *)&data[len/5], len/5);
            nrf_delay_ms(10);
            copy_to_raw_data(raw_type, (void *)&data[2*len/5], len/5);
            nrf_delay_ms(10);
            copy_to_raw_data(raw_type, (void *)&data[3*len/5], len/5);
            nrf_delay_ms(10);
            copy_to_raw_data(raw_type, (void *)&data[4*len/5], len/5);
        }
    }
}

ret_code_t LPS_init(){
    NIramCfg.cur_module = e_LPS;
    ret_code_t ret = base_module_init();
    if(ret != STATE_MODULE_SUCCESS)
        return ret;

    PPG_board_power_on();
    bool r;
    r = acc_start(IMU_SPO2);
    if (r == false) {
        NRF_LOG_INFO("ACC START FAILED. \n");
        return STATE_MODULE_ACC_FAILED;
    }
    
    acc_clear_fifo();
    //acc_start_active_check_mode();
    start_auto_end_sleep_check = false;

    r = afe_start(RING_AFE_DAILY_MODE);
    if (r == false){
        NRF_LOG_INFO("AFE START FAILED. \n");
        return STATE_MODULE_AFE_FAILED;
    }

    r = afe_start(RING_AFE_HRV_ON_MODE);
    if (r == false){
        NRF_LOG_INFO("AFE HRV START FAILED. \n");
        return STATE_MODULE_AFE_FAILED;
    }

    //start temperature
    r = tmp_start(RING_TMP_ONE_SHOT_MODE, RING_INSIDE_TMP117);
    if (r == false){
        NRF_LOG_INFO("TEMPERATURE START FAILED. \n");
        return STATE_MODULE_TEMP_FAILED;
    }

    lps_monitor_start_time = UTC_getClock();

    //AFE4900_HWshutdown();

    //if(NIramCfg.cur_module != e_PS)
    //    clear_module_report_ram();
    return STATE_MODULE_SUCCESS;
}

void LPS_uninit(E_REPORT_END_TYPE e_end){
    if(e_end != e_report_end_by_charging && e_end != e_report_end_by_low_power)
        NIramCfg.last_auto_sleep_close_time = UTC_getClock();
    //acc_stop_active_check_mode();
    if(!acc_stop())
        NRF_LOG_INFO("LPS_uninit acc_stop FAILED. \n");
    if(!afe_stop())
        NRF_LOG_INFO("LPS_uninit afe_stop FAILED. \n");
    if(!tmp_stop(RING_INSIDE_TMP117))
        NRF_LOG_INFO("LPS_uninit tmp_stop FAILED. \n");
    uint16_t report_id;
    if(report_sec_stop(e_report_low_power_sleep, e_end, &report_id) != NRF_SUCCESS)
        NRF_LOG_INFO("LPS_uninit report_sec_stop FAILED. \n");
    clear_monitor_report_ram();
    //ACC_Sleep_judge_data_clear();

    base_module_uninit();
}

bool LPS_check_on_time_start(){
    uint32_t _now = UTC_getClock();
    uint32_t _different = _now - NIramCfg.cur_module_start_time;
    if((_different % LPS_MONITOR_PERIOD) == 0)
        return true;

    return false;
}

bool LPS_check_on_time_stop(){
    uint32_t _now = UTC_getClock();
    uint32_t _different = _now - lps_monitor_start_time;
    if(_different >= LPS_MONITOR_DURATION)
        return true;

    return false;
}

void LPS_process_sys(void *pMsg){
    uint32_t evt = *((uint32_t*)pMsg);
    uint8_t base_evt = (evt & 0xFF000000) >> 24;
    uint8_t sub_evt = (evt & 0x00FF0000) >> 16;

    switch(base_evt){
        case SYS_MODULE:
        break;
        case SYS_ONE_SECOND:{
            UTCTime _now = UTC_getClock();
            if((_now - NIramCfg.cur_module_start_time) >= MAX_SLEEP_DURATION){
                switch_module(e_DAILY, e_report_end_by_over_time);
                return;
            }

            if(lps_monitor_start_time <= 0){
                report_push_data_sec(/*INVALID_HR*/0xFF, INVALID_TEMP, accval, INVALID_SPO2, INVALID_STEP, e_report_low_power_sleep);
                
                if(is_acc_in_sleep()){
                    if(UTC_getClock() % 3 == 0){
                        acc_set_hold_data(38, e_acc_SPO2);
                    }
                }
                if(LPS_check_on_time_start() || (connect_crtl.isPaired == true && get_rt_en() == true)){
                    PPG_board_power_on();
                    bool r = afe_start(RING_AFE_DAILY_MODE);
                    if (r == false){
                        NRF_LOG_INFO("LPS_process_sys AFE START FAILED. \n");
                    }

                    r = afe_start(RING_AFE_HRV_ON_MODE);
                    if (r == false){
                        NRF_LOG_INFO("LPS_process_sys AFE HRV START FAILED. \n");
                    }

                    r = tmp_start(RING_TMP_ONE_SHOT_MODE, RING_INSIDE_TMP117);
                    if (r == false){
                        NRF_LOG_INFO("LPS_process_sys TEMPERATURE START FAILED. \n");
                    }
                    start_circle_proc_hrv(HEALTH_NON_PROFESSIONAL_TYPE);
                    acc_clear_fifo();

                    lps_monitor_start_time = UTC_getClock();
                }
            }else{
                temp_read(LPS_TEMP_PERIOD);
                report_push_data_sec(g_afe.hr, gCurrent_body_temperature(), accval, INVALID_SPO2, INVALID_STEP, e_report_low_power_sleep);
                
                if(UTC_getClock() % 3){
                    if(is_acc_in_sleep()){
                        acc_set_hold_data(38, e_acc_SPO2);
                    }
                }

                if(connect_crtl.isPaired == true && get_rt_en() == true){
                    sendMonitorData(e_LPS);
                    lps_monitor_start_time = UTC_getClock();
                }

                if(LPS_check_on_time_stop()){
                    processHRV_1(e_report_low_power_sleep);
                    hrv_health_process_flag_stop();
                    if(!afe_stop_without_free())
                        NRF_LOG_INFO("LPS_process_sys afe_stop FAILED. \n");
                    if(!tmp_stop(RING_INSIDE_TMP117))
                        NRF_LOG_INFO("LPS_process_sys tmp_stop FAILED. \n");
                    //PPG_board_power_off();
                    AFE4900_HWshutdown();
                    lps_monitor_start_time = 0;
                    acc_clear_fifo();
                }
            }

            
            if(start_auto_end_sleep_check == false){
                if(check_auto_sleep_end_3()){
                   // start_sleep_auto_end_check();
                    start_auto_end_sleep_check = true;
                }
            }
        }
        break;
        default:
        break;
    }
}

void LPS_process_i2c(void *pMsg){
    uint16_t evt = *((uint16_t *)pMsg);

    i2c_init();
    /******** I2C ACC ***********/
    if (MAJOR_DEV_ACC == (MAJOR_DEV_MASK & evt)) {
        switch (OP_MODE_MASK & evt) {
        case OP_READ_INT_STATUS:
            if (2 == GET_OP_PARM(evt)) {
                NRF_LOG_INFO("LPS_process_i2c int 2 wait_isr");
                uint8_t ret = read_axis(e_acc_SPO2); // Read ACC data and save them to SpO2, EHR or Step buffer.
                if (0 != ret) {
                    acc_start(IMU_SPO2);
                }

                if(start_auto_end_sleep_check == true){
                    if(acc_get_Sleep_flag() == END_SLEEP_MODE){
                        switch_module(e_DAILY, e_report_end_by_auto);
                    }
                }
            }
        break;
        }
    }

    /******** I2C AFE ***********/
    if (MAJOR_DEV_AFE == (MAJOR_DEV_MASK & evt)) {
        switch (OP_MODE_MASK & evt) {
        case OP_READ_SENSOR_DATA:{
            if(AFE4900_check_FIFO_TOGGLE() == false)
                break;
            struct afe4900_sensor_data *pData;
            uint16_t data_len = 0;
            set_afe_fifordy_status();
            read_afe_value(&pData, 0, &data_len, true, 
                (RING_AFE_HRV_ON_MODE-1) & afe_get_work_mode());
                  
            uint8_t ret = DAILY_bufProc_wavelet_LDS((void *)pData, data_len);
            if (ret == 1) {
                afe_daily_proc_wavelate();
            }

            // send_raw_data_1(pData, data_len);
            }
        break;
        }
    }

    i2c_uninit();

}

bool LPS_process_ble(uint8_t *pR, uint8_t *out){
    uint8_t cmd = *(pR);    // Response paired with the request.
    
    uint8_t rStatus = CMD_SUCCESS;
    switch(cmd){
        case APP_KEEPALIVE_CMD:{
            out[3] = get_batt_power_status();
            out[4] = get_batt_vol_percent();
            out[5] = e_LPS;
            LPS_set_adv_data(&out[6]);
        }
        break;
        default:
        return false;
    }

    
    if(rStatus == CMD_NO_RETURN)
        return true;

    out[2] = rStatus;
    ring_indicate(out, 20);
    return true;
}

uint8_t LPS_type(){
    return e_LPS;
}

void LPS_set_adv_data(uint8_t *data){
    uint32_t duration = UTC_getClock() - NIramCfg.cur_module_start_time;
    data[0] = (duration >> 24) & 0xFF;
    data[1] = (duration >> 16) & 0xFF;
    data[2] = (duration >> 8) & 0xFF;
    data[3] = (duration >> 0) & 0xFF;

    data[4] = g_afe.shr;
    data[5] = g_afe.flag;
    data[6] = (g_afe.offhand_flg == 1 ? 0 : 1);
}