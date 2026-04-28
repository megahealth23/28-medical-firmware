#include "state_module.h"
#include "ring_afe.h"
#include "nrf_log.h"
#include "ble_ring.h"
#include "protocol.h"
#include "ble_service.h"
#include "reports.h"
#include "ring_startup.h"
#include "inner_flash_ram.h"
#include "peripheral/peripherals.h"
#include "steps_statistics.h"
#include "algorithm/DataProcEHR.h"

//static int step = 0; 
static uint32_t total_pa_step = 0;
static uint16_t pa_step_in_min = 0;
#define PA_TEMP_PERIOD    (60)

// extern uint8_t g_pre_peak_pos;
// extern MOVE_HR_Para g_OutPara;

ret_code_t PA_init(){
    NIramCfg.cur_module = e_PA;
    ret_code_t ret = base_module_init();
    if(ret != STATE_MODULE_SUCCESS)
        return ret;

    PPG_board_power_on();
    bool r;
    r = acc_start(IMU_EHR);
    if (r == false) {
        NRF_LOG_INFO("ACC START FAILED. \n");
        return STATE_MODULE_ACC_FAILED;
    }
    acc_clear_fifo();

	tmp_start(RING_TMP_SHUTDOWN_MODE, RING_INSIDE_TMP117);

    //PPG_board_power_off();
    //AFE4900_HWshutdown();

    r = afe_start(RING_AFE_SPORT_MODE);
    if (r == false){
        NRF_LOG_INFO("AFE START FAILED. \n");
        return STATE_MODULE_AFE_FAILED;
    }

    //start temperature
    r = tmp_start(RING_TMP_ONE_SHOT_MODE, RING_INSIDE_TMP117);
    if (r == false){
        NRF_LOG_INFO("TEMPERATURE START FAILED. \n");
        return STATE_MODULE_TEMP_FAILED;
    }

    reset_steps_count_by_module();
    //step = steps_num();

    sport_heart_algorithm_init();

    return STATE_MODULE_SUCCESS;
}

void PA_uninit(E_REPORT_END_TYPE e_end){
    //report_push_data_daily(g_afe.ehr_hr, INVALID_TEMP, accval, 
    //                            INVALID_SPO2, steps_num());
    //steps_clear();
    //report_daily_save();
    if(!acc_stop())
        NRF_LOG_INFO("acc_stop FAILED. \n");
    if(!afe_stop())
        NRF_LOG_INFO("afe_stop FAILED. \n");
    if(!tmp_stop(RING_INSIDE_TMP117))
        NRF_LOG_INFO("tmp_stop FAILED. \n");
    uint16_t report_id;
    if(report_sec_stop(e_report_auto_sport, e_end, &report_id) != NRF_SUCCESS)
        NRF_LOG_INFO("report_sec_stop FAILED. \n");
    clear_monitor_report_ram();
    ACC_Exercise_judge_data_clear();

    reset_steps_count_by_module();
    base_module_uninit();

}

extern int16_t daily_mark_step;
void PA_process_sys(void *pMsg){
    uint32_t evt = *((uint32_t*)pMsg);
    uint8_t base_evt = (evt & 0xFF000000) >> 24;
    uint8_t sub_evt = (evt & 0x00FF0000) >> 16;

    switch(base_evt){
        case SYS_MODULE:
        break;
        case SYS_ONE_SECOND:{
            UTCTime _now = UTC_getClock();
            if((_now - NIramCfg.cur_module_start_time) >= MAX_EXERCISE_DURATION){
                switch_module(e_DAILY, e_report_end_by_over_time);
                return;
            }

            if(_now % 10 == 0){
                uint8_t flag = acc_get_Exercise_flag();
                if(flag == 0){
                    switch_module(e_DAILY, e_report_end_by_auto);
                    return;
                }
            }

            temp_read(PA_TEMP_PERIOD);

            //if(UTC_getClock() % DAILY_MARK_PERIOD == 0){
            //    report_push_data_daily(g_afe.ehr_hr, INVALID_TEMP, accval, 
            //                    INVALID_SPO2, steps_num());
            //    steps_clear();
            //}

            uint16_t s = get_steps_in_sec();
            report_push_data_sec(g_afe.ehr_hr, gCurrent_body_temperature(), accval, INVALID_SPO2, s, e_report_auto_sport);
            
            if(is_acc_in_sleep()){
                acc_set_hold_data(100, e_acc_EHR);

            }
            if(connect_crtl.isPaired == true && get_rt_en() == true)
                sendMonitorData_auto_exer(s);
        }
        break;
        default:
        break;
    }
}

static void send_raw_data_ehr(struct afe4900_sensor_data* data, uint16_t len){
    if (is_enable_rawdata() && data != NULL && len > 0){
        uint8_t raw_type = EHR_WORD;
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

extern accFifo_t ax;
void PA_process_i2c(void *pMsg){
    uint16_t evt = *((uint16_t *)pMsg);

    i2c_init();
    /******** I2C ACC ***********/
    if (MAJOR_DEV_ACC == (MAJOR_DEV_MASK & evt)) {
        switch (OP_MODE_MASK & evt) {
        case OP_READ_INT_STATUS:
            if (2 == GET_OP_PARM(evt)) {
                NRF_LOG_INFO("PA_process_i2c int 2 wait_isr");
                NRF_LOG_INFO("start read");
                uint8_t ret = read_axis(e_acc_EHR); // Read ACC data and save them to SpO2, EHR or Step buffer.
                if (0 != ret) {
                    acc_start(IMU_EHR);
                }
                NRF_LOG_INFO("end read");
                //uint8_t flag = acc_get_Exercise_flag();
                //if(flag == 0){
                //    switch_module(e_DAILY, e_report_end_by_auto);
                //}

                // int8_t acc_len = 0;
                // while(acc_len < ax.n){
                //     uint8_t send_len = acc_len + 10 > ax.n ? ax.n - acc_len : 10;
                //     send_acc_raw_data(&ax.axis[acc_len], send_len);
                //     acc_len = acc_len + send_len;
                // }
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
                  
            uint8_t ret = EHR_data_bufProc((void *)pData, data_len);
            if (ret == 1) {
                afe_ehr_proc();
                hr_algorithm_time_acc_sample_num_claer();
            }

            // send_raw_data_ehr(pData, data_len);
        }
        break;
        }
    }

    i2c_uninit();
}

bool PA_process_ble(uint8_t *pR, uint8_t *out){
    uint8_t cmd = *(pR);    // Response paired with the request.
    
    uint8_t rStatus = CMD_SUCCESS;
    switch(cmd){
        case APP_KEEPALIVE_CMD:{
            out[3] = get_batt_power_status();
            out[4] = get_batt_vol_percent();
            out[5] = e_PA;
            PA_set_adv_data(&out[6]);
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

uint8_t PA_type(){
    return e_PA;
}

void PA_set_adv_data(uint8_t *data){
    uint32_t duration = UTC_getClock() - NIramCfg.cur_module_start_time;
    data[0] = (duration >> 24) & 0xFF;
    data[1] = (duration >> 16) & 0xFF;
    data[2] = (duration >> 8) & 0xFF;
    data[3] = (duration >> 0) & 0xFF;

    data[4] = g_afe.ehr_hr;

    uint32_t total_pa_step = get_steps_count_by_module();
    data[5] = (total_pa_step >> 24) & 0xFF;
    data[6] = (total_pa_step >> 16) & 0xFF;
    data[7] = (total_pa_step >> 8) & 0xFF;
    data[8] = (total_pa_step >> 0) & 0xFF;

    uint16_t pa_step_in_min = get_steps_in_min();
    data[9] = (pa_step_in_min >> 8) & 0xFF;
    data[10] = (pa_step_in_min >> 0) & 0xFF;

    data[12] = g_afe.flag;
    data[13] = (g_afe.offhand_flg == 1 ? 0 : 1);
}