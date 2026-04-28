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
//uint8_t sub_exercise_type = 0;
//static int step = 0; 
static uint64_t total_heart_rate = 0;
static uint32_t total_heart_rate_cnt = 0;
#define EXER_TEMP_PERIOD    (60)
// extern uint8_t g_pre_peak_pos;
// extern MOVE_HR_Para g_OutPara;
// extern uint8_t g_sport_alg_cnt;
ret_code_t EXER_init(){
    NIramCfg.cur_module = e_EXER;
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

    //AFE4900_HWshutdown();
    //PPG_board_power_off();

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

    //reset_steps_count_by_module();
    total_heart_rate = 0;
    total_heart_rate_cnt = 0;
    sport_heart_algorithm_init();

    //step = steps_num();

    return STATE_MODULE_SUCCESS;
}

void EXER_uninit(E_REPORT_END_TYPE e_end){
    //report_push_data_daily(g_afe.ehr_hr, INVALID_TEMP, accval, 
    //                            INVALID_SPO2, steps_num());
    //steps_clear();
    report_daily_save();
    if(!acc_stop())
        NRF_LOG_INFO("acc_stop FAILED. \n");
    if(!afe_stop())
        NRF_LOG_INFO("afe_stop FAILED. \n");
    if(!tmp_stop(RING_INSIDE_TMP117))
        NRF_LOG_INFO("tmp_stop FAILED. \n");
    uint16_t report_id;
    if(report_sec_stop(e_report_sport, e_end, &report_id) != NRF_SUCCESS)
        NRF_LOG_INFO("report_sec_stop FAILED. \n");
    clear_monitor_report_ram();
    ACC_Exercise_judge_data_clear();

    reset_steps_count_by_module();
    base_module_uninit();
}

extern int16_t daily_mark_step;
void EXER_process_sys(void *pMsg){
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
            temp_read(EXER_TEMP_PERIOD);

            //if(UTC_getClock() % DAILY_MARK_PERIOD == 0){
            //    report_push_data_daily(g_afe.ehr_hr, INVALID_TEMP, accval, 
            //                    INVALID_SPO2, steps_num());
            //    steps_clear();
            //}

            if(is_acc_in_sleep()){
                acc_set_hold_data(100, e_acc_EHR);
            }

            uint16_t s = get_steps_in_sec();
            total_heart_rate += g_afe.ehr_hr;
            total_heart_rate_cnt++;
            report_push_data_sec(g_afe.ehr_hr, gCurrent_body_temperature(), accval, INVALID_SPO2, s, e_report_sport);
            if(connect_crtl.isPaired == true && get_rt_en() == true)
                sendMonitorData_exer(s);
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

            // copy_to_raw_data(raw_type, (void *)data, len/5);
            // nrf_delay_ms(10);
            // copy_to_raw_data(raw_type, (void *)&data[len/5], len/5);
            // nrf_delay_ms(10);
            // copy_to_raw_data(raw_type, (void *)&data[2*len/5], len/5);
            // nrf_delay_ms(10);
            // copy_to_raw_data(raw_type, (void *)&data[3*len/5], len/5);
            // nrf_delay_ms(10);
            // copy_to_raw_data(raw_type, (void *)&data[4*len/5], len/5);
        }
    }
}
 
extern accFifo_t ax;
void EXER_process_i2c(void *pMsg){
    uint16_t evt = *((uint16_t *)pMsg);

    i2c_init();
    /******** I2C ACC ***********/
    if (MAJOR_DEV_ACC == (MAJOR_DEV_MASK & evt)) {
        switch (OP_MODE_MASK & evt) {
        case OP_READ_INT_STATUS:
            if (2 == GET_OP_PARM(evt)) {
                NRF_LOG_INFO("EXER_process_i2c int 2 wait_isr");
                NRF_LOG_INFO("start read");
                uint8_t ret = read_axis(e_acc_EHR); // Read ACC data and save them to SpO2, EHR or Step buffer.
                if (0 != ret) {
                    acc_start(IMU_EHR);
                }
                NRF_LOG_INFO("end read");

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
            
            //uint8_t toggle = AFE4900_get_FIFO_TOGGLE();

            //for(int i = 0; i < data_len; i++){
            //    pData[i].phase2 = toggle;
            //    pData[i].phase3 = 0;
            //}

            //pData[0].phase3 = get_result;
            //pData[1].phase3 = fifo_byte_end_idx;
            //pData[2].phase3 = maxsize;

            // send_raw_data_ehr(pData, data_len);
        }
        break;
        }
    }

    i2c_uninit();
}

bool EXER_process_ble(uint8_t *pR, uint8_t *out){
    uint8_t cmd = *(pR);    // Response paired with the request.
    
    uint8_t rStatus = CMD_SUCCESS;
    switch(cmd){
        case APP_KEEPALIVE_CMD:{
            out[3] = get_batt_power_status();
            out[4] = get_batt_vol_percent();
            out[5] = e_EXER;
            EXER_set_adv_data(&out[6]);
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

uint8_t EXER_type(){
    return e_EXER;
}

void EXER_set_adv_data(uint8_t *data){
    uint32_t duration = UTC_getClock() - NIramCfg.cur_module_start_time;
    data[0] = (duration >> 24) & 0xFF;
    data[1] = (duration >> 16) & 0xFF;
    data[2] = (duration >> 8) & 0xFF;
    data[3] = (duration >> 0) & 0xFF;

    data[4] = NIramCfg.cur_sub_module;
    data[5] = g_afe.ehr_hr;
    switch(NIramCfg.cur_sub_module){
        case e_sub_run_indoor:
        case e_sub_run_outdoor:
        case e_sub_walk_outdoor:{
            uint32_t total_exer_step = get_steps_count_by_module();
            data[6] = (total_exer_step >> 24) & 0xFF;
            data[7] = (total_exer_step >> 16) & 0xFF;
            data[8] = (total_exer_step >> 8) & 0xFF;
            data[9] = (total_exer_step >> 0) & 0xFF;

            uint16_t exer_step_in_min = get_steps_in_min();
            data[10] = (exer_step_in_min >> 8) & 0xFF;
            data[11] = (exer_step_in_min >> 0) & 0xFF;
            data[12] = g_afe.ehr_flag;
            data[13] = (g_afe.offhand_flg == 1 ? 0 : 1);
        }
        break;
        case e_sub_bicycle_outdoor:
        case e_sub_swimming:
        case e_sub_strength:{
            data[6] = g_afe.ehr_flag;
            data[7] = (g_afe.offhand_flg == 1 ? 0 : 1);
            if(total_heart_rate_cnt > 0){
                data[8] = total_heart_rate/total_heart_rate_cnt;
            }else{
                data[8] = 0;
            }
        }
        break;
    }
}