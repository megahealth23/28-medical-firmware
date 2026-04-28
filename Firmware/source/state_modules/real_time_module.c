#include "state_module.h"
#include "ring_afe.h"
#include "nrf_log.h"
#include "ble_ring.h"
#include "protocol.h"
#include "ble_service.h"
#include "inner_flash_ram.h"
#include "ring_startup.h"
#include "steps_statistics.h"
#include "DataProcHealth.h"

#define REAL_TEMP_PERIOD (20)

int16_t realtime_reserved = -1;

//heart process
extern int HistHrsavepos;
extern uint16_t g_rr_mean;
ret_code_t REAL_init(){
    NIramCfg.cur_module = e_REAL;
    ret_code_t ret = base_module_init();
    if(ret != STATE_MODULE_SUCCESS)
        return ret;

    realtime_reserved = -1;
    PPG_board_power_on();
    bool r;
    r = acc_start(IMU_SPO2);
    if (r == false) {
        NRF_LOG_INFO("ACC START FAILED. \n");
        return STATE_MODULE_ACC_FAILED;
    }
    acc_clear_fifo();

    r = afe_start(RING_AFE_SPO2_MODE);
    if (r == false){
        NRF_LOG_INFO("AFE START FAILED. \n");
        return STATE_MODULE_AFE_FAILED;
    }

    r = afe_start(RING_AFE_HRV_ON_MODE);
    if (r == false){
        NRF_LOG_INFO("AFE HRV START FAILED. \n");
        return STATE_MODULE_AFE_FAILED;
    }

    reset_steps_count_by_module();

    //r = tmp_start(RING_TMP_ONE_SHOT_MODE, RING_INSIDE_TMP117);
    //if (r == false){
    //    NRF_LOG_INFO("TEMPERATURE START FAILED. \n");
    //    return STATE_MODULE_TEMP_FAILED;
    //}
    g_rr_mean = 0;
    HistHrsavepos = 0;
    return STATE_MODULE_SUCCESS;
}

void REAL_uninit(E_REPORT_END_TYPE e_end){
    report_push_data_daily(g_afe.shr, INVALID_TEMP, accval, 
                                INVALID_SPO2, get_steps_count_by_module(), get_batt_vol_percent());
    //steps_clear();
    report_daily_save();
    //if(!acc_stop())
    //    NRF_LOG_INFO("acc_stop FAILED. \n");
    if(!afe_stop())
        NRF_LOG_INFO("afe_stop FAILED. \n");
    //if(!tmp_stop(RING_INSIDE_TMP117))
    //    NRF_LOG_INFO("tmp_stop FAILED. \n");

    reset_steps_count_by_module();
    //fds_m_gc();
    base_module_uninit();
}

void REAL_process_sys(void *pMsg){
    uint32_t evt = *((uint32_t*)pMsg);
    uint8_t base_evt = (evt & 0xFF000000) >> 24;
    uint8_t sub_evt = (evt & 0x00FF0000) >> 16;

    switch(base_evt){
        case SYS_MODULE:
            if(sub_evt == SYS_BLE_CONNECT){
                realtime_reserved = -1;
            }else if(sub_evt == SYS_BLE_DISCONNECT){
                    realtime_reserved = REALTIME_RESERVED_TIME;
                    //state_modules_switch(e_DAILY, e_report_end_by_user);
            }
        break;
        case SYS_ONE_SECOND:{
            //temp_read(REAL_TEMP_PERIOD);
            //daily_mark();
            
            if(is_acc_in_sleep()){
                acc_set_hold_data(13, e_acc_SPO2);
            }

            if(realtime_reserved > 0){
                realtime_reserved--;
                if(realtime_reserved <= 0){
                    state_modules_switch(e_DAILY, e_report_end_by_user);
                    break;
                }
            }

            if(UTC_getClock() % DAILY_MARK_PERIOD == 0){
                report_push_data_daily(g_afe.shr, INVALID_TEMP, accval, 
                                INVALID_SPO2, get_steps_count_by_module(), get_batt_vol_percent());
                reset_steps_count_by_module();
                //steps_clear();
            }

            // check_auto_sleep_start();//
            if(connect_crtl.isPaired == true && get_rt_en() == true)
                sendMonitorData(e_REAL);
        }
        break;
        default:
        break;
    }
}

void REAL_process_i2c(void *pMsg){
    uint16_t evt = *((uint16_t *)pMsg);

    i2c_init();
    /******** I2C ACC ***********/
    if (MAJOR_DEV_ACC == (MAJOR_DEV_MASK & evt)) {
        switch (OP_MODE_MASK & evt) {
        case OP_READ_INT_STATUS:
            if (2 == GET_OP_PARM(evt)) {
                NRF_LOG_INFO("REAL_process_i2c int 2 wait_isr");
                uint8_t ret = read_axis(e_acc_SPO2); // Read ACC data and save them to SpO2, EHR or Step buffer.
                if (0 != ret) {
                    acc_start(IMU_SPO2);
                }

                //if(acc_get_Sleep_flag() == START_SLEEP_MODE){
                //    if(NIramCfg.low_power_mark == 0){
                //        switch_module(e_DS, e_report_end_by_timer);
                //    }else{
                //        switch_module(e_LPS, e_report_end_by_timer);
                //    }
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
                  
            uint8_t ret = SPO2_bufProc_wavelet((void *)pData, data_len);
            if (ret == 1) {
                afe_spo2_proc_wavelate();
            }

            //send_raw_data(pData, data_len);
            }
        break;
        }
    }

    i2c_uninit();
}

bool REAL_process_ble(uint8_t *pR, uint8_t *out){
    uint8_t cmd = *(pR);    // Response paired with the request.
    
    uint8_t rStatus = CMD_SUCCESS;
    switch(cmd){
        case APP_KEEPALIVE_CMD:{
            out[3] = get_batt_power_status();
            out[4] = get_batt_vol_percent();
            out[5] = e_REAL;
            REAL_set_adv_data(&out[6]);
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

uint8_t REAL_type(){
    return e_REAL;
}

void REAL_set_adv_data(uint8_t *data){
    uint32_t duration = UTC_getClock() - NIramCfg.cur_module_start_time;
    data[0] = (duration >> 24) & 0xFF;
    data[1] = (duration >> 16) & 0xFF;
    data[2] = (duration >> 8) & 0xFF;
    data[3] = (duration >> 0) & 0xFF;

    data[4] = (g_afe.Spo2show >> 2) + 37;
    data[5] = g_afe.shr;

    uint32_t step_in_day = get_steps_in_day();
    data[6] = (step_in_day >> 24) & 0xFF;
    data[7] = (step_in_day >> 16) & 0xFF;
    data[8] = (step_in_day >> 8) & 0xFF;
    data[9] = (step_in_day >> 0) & 0xFF;

    uint16_t step_in_15_min = get_steps_in_15_min();
    data[10] = (step_in_15_min >> 8) & 0xFF;
    data[11] = (step_in_15_min >> 0) & 0xFF;

    data[12] = g_afe.flag;
    data[13] = (g_afe.offhand_flg == 1 ? 0 : 1);
}