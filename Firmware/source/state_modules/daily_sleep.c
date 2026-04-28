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
#define DS_TEMP_PERIOD (2*60)
static bool start_auto_end_sleep_check = false;

//heart process
extern uint16_t g_rr_mean;
extern int HistHrsavepos;
static void send_raw_data_1(struct afe4900_sensor_data* data, uint16_t len){
    if (is_enable_rawdata() && data != NULL && len > 0){
        uint8_t raw_type = SPO2_1_WORD;
        if(raw_type != NONE_WORD){
            copy_to_raw_data(raw_type, (void *)data, len/10);
            nrf_delay_ms(10);
            copy_to_raw_data(raw_type, (void *)&data[len/10], len/10);
            nrf_delay_ms(10);
            copy_to_raw_data(raw_type, (void *)&data[2*len/10], len/10);
            nrf_delay_ms(10);
            copy_to_raw_data(raw_type, (void *)&data[3*len/10], len/10);
            nrf_delay_ms(10);
            copy_to_raw_data(raw_type, (void *)&data[4*len/10], len/10);
            nrf_delay_ms(10);
            copy_to_raw_data(raw_type, (void *)&data[5*len/10], len/10);
            nrf_delay_ms(10);
            copy_to_raw_data(raw_type, (void *)&data[6*len/10], len/10);
            nrf_delay_ms(10);
            copy_to_raw_data(raw_type, (void *)&data[7*len/10], len/10);
            nrf_delay_ms(10);
            copy_to_raw_data(raw_type, (void *)&data[8*len/10], len/10);
            nrf_delay_ms(10);
            copy_to_raw_data(raw_type, (void *)&data[9*len/10], len/10);
        }
    }
}


extern uint16_t g_HrRRNum[HR_ALG_RR_NUM];
extern uint8_t g_HrRRCnt;
extern int pre_data_len;
ret_code_t DS_init(){
    NIramCfg.cur_module = e_DS;
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

    g_rr_mean = 0;
    HistHrsavepos = 0;
    pre_data_len = 0;
    
    memset(g_HrRRNum,0,HR_ALG_RR_NUM*sizeof(uint16_t));
    g_HrRRCnt = 0;
    //AFE4900_SW_reset();
    //AFE4900_HWshutdown();

    //if(NIramCfg.cur_module != e_PS)
    //    clear_module_report_ram();
    return STATE_MODULE_SUCCESS;
}

void DS_uninit(E_REPORT_END_TYPE e_end){
    if(e_end != e_report_end_by_charging && e_end != e_report_end_by_low_power)
        NIramCfg.last_auto_sleep_close_time = UTC_getClock();
    //acc_stop_active_check_mode();
    if(!acc_stop())
        NRF_LOG_INFO("DS_uninit acc_stop FAILED. \n");
    if(!afe_stop())
        NRF_LOG_INFO("DS_uninit afe_stop FAILED. \n");
    if(!tmp_stop(RING_INSIDE_TMP117))
        NRF_LOG_INFO("DS_uninit tmp_stop FAILED. \n");
    uint16_t report_id;
    if(report_sec_stop(e_report_sleep, e_end, &report_id) != NRF_SUCCESS)
        NRF_LOG_INFO("DS_uninit report_sec_stop FAILED. \n");
    clear_monitor_report_ram();

    base_module_uninit();
}

void DS_process_sys(void *pMsg){
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

            temp_read(DS_TEMP_PERIOD);
            report_push_data_sec(g_afe.hr, gCurrent_body_temperature(), accval, INVALID_SPO2, INVALID_STEP, e_report_sleep);
            processHRV(e_report_sleep);
            if(is_acc_in_sleep()){
                if(UTC_getClock() % 3 == 0){
                    acc_set_hold_data(38, e_acc_SPO2);
                }
            }
            if(_now % 10 == 0){
                if(acc_get_Sleep_flag() == END_SLEEP_MODE){
                    switch_module(e_DAILY, e_report_end_by_auto);
                }
            }


            if(connect_crtl.isPaired == true && get_rt_en() == true)
                sendMonitorData(e_DS);
        }
        break;
        default:
        break;
    }
}

void DS_process_i2c(void *pMsg){
    uint16_t evt = *((uint16_t *)pMsg);

    i2c_init();
    /******** I2C ACC ***********/
    if (MAJOR_DEV_ACC == (MAJOR_DEV_MASK & evt)) {
        switch (OP_MODE_MASK & evt) {
        case OP_READ_INT_STATUS:
            if (2 == GET_OP_PARM(evt)) {
                NRF_LOG_INFO("DS_process_i2c int 2 wait_isr");
                uint8_t ret = read_axis(e_acc_SPO2); // Read ACC data and save them to SpO2, EHR or Step buffer.
                if (0 != ret) {
                    acc_start(IMU_SPO2);
                }

                //if(start_auto_end_sleep_check == true){
                // if(acc_get_Sleep_flag() == END_SLEEP_MODE){
                //     switch_module(e_DAILY, e_report_end_by_auto);
                // }
                //}
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
                  
            uint8_t ret = DAILY_bufProc_wavelet((void *)pData, data_len);
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

bool DS_process_ble(uint8_t *pR, uint8_t *out){
    uint8_t cmd = *(pR);    // Response paired with the request.
    
    uint8_t rStatus = CMD_SUCCESS;
    switch(cmd){
        case APP_KEEPALIVE_CMD:{
            out[3] = get_batt_power_status();
            out[4] = get_batt_vol_percent();
            out[5] = e_DS;
            DS_set_adv_data(&out[6]);
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

uint8_t DS_type(){
    return e_DS;
}

void DS_set_adv_data(uint8_t *data){
    uint32_t duration = UTC_getClock() - NIramCfg.cur_module_start_time;
    data[0] = (duration >> 24) & 0xFF;
    data[1] = (duration >> 16) & 0xFF;
    data[2] = (duration >> 8) & 0xFF;
    data[3] = (duration >> 0) & 0xFF;

    data[4] = g_afe.shr;
    data[5] = g_afe.flag;
    data[6] = (g_afe.offhand_flg == 1 ? 0 : 1);
}