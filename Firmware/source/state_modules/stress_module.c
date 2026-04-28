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
#include "algorithm/DataProcHealth.h"
#include "algorithm/DataProcEHR.h"

#define MAX_STRESS_PERIOD   (30*60)
static uint16_t stress_time_spent = 0;
ret_code_t STRESS_init(){
    NIramCfg.cur_module = e_STRESS;
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

    r = afe_start(RING_AFE_SPORT_MODE);
    if (r == false){
        NRF_LOG_INFO("AFE START FAILED. \n");
        return STATE_MODULE_AFE_FAILED;
    }

    ret = start_circle_proc_hrv(HEALTH_PROFESSIONAL_TYPE);
    if(ret != NRF_SUCCESS){
        NRF_LOG_ERROR("start_circle_proc_hrv failed.");
        return STATE_MODULE_STRESS_START_FAILED;
    }

    stress_time_spent = 0;
    return STATE_MODULE_SUCCESS;
}

#define STRESS_END_BY_COMPLETE  0
#define STRESS_END_BY_TIMER     1

void STRESS_uninit(E_REPORT_END_TYPE e_end){
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
    if(e_end == e_report_end_by_user){
        hrv_health_process_flag_stop();
        report_remove_cur_sec_report();
    }else if(e_end == e_report_end_by_timer){
        hrv_health_process_flag_stop();
        report_remove_cur_sec_report();
        uint8_t d[20] = {0};
        d[0] = RING_STOP_STRESS_CMD;
        d[1] = 0;
        d[2] = CMD_SUCCESS;
        d[3] = STRESS_END_BY_TIMER;
        d[4] = 0;
        ring_indicate(d, 20); 
    }else if(e_end == e_report_end_by_auto){
        HRV_Para* hrv = hrv_health_process_result_get();
 #define HRV_BLOCK_CNT 200 //200 *sizeof(uint_16) = 400 bytes 
        if(hrv != NULL){
            uint16_t rr_num = hrv->RR_num;
            for(int i = 1; ; i++){
                NRF_LOG_INFO("report_push_data_hrv count %d. \n", i);
                if(i * HRV_BLOCK_CNT >= hrv->RR_num){
                    report_push_data_hrv((uint8_t*)(&hrv->green_RR[(i-1)*HRV_BLOCK_CNT]), 2*(rr_num- (i-1)*HRV_BLOCK_CNT), e_report_professional_stress);
                    break;
                }else{
                    report_push_data_hrv((uint8_t*)(&hrv->green_RR[(i-1)*HRV_BLOCK_CNT]), HRV_BLOCK_CNT*2, e_report_professional_stress);
                }
            }
            hrv_health_process_flag_stop();
        }
        uint16_t report_id;
        if(report_sec_stop(e_report_professional_stress, e_report_end_by_auto, &report_id) != NRF_SUCCESS)
            NRF_LOG_INFO("report_sec_stop FAILED. \n");
        //note app
        uint8_t d[20] = {0};
        d[0] = RING_STOP_STRESS_CMD;
        d[1] = 0;
        d[2] = CMD_SUCCESS;
        d[3] = STRESS_END_BY_COMPLETE;
        d[4] = report_id;
        ring_indicate(d, 20); 
    }
    clear_monitor_report_ram();

    base_module_uninit();
}

extern int16_t daily_mark_step;
void STRESS_process_sys(void *pMsg){
    uint32_t evt = *((uint32_t*)pMsg);
    uint8_t base_evt = (evt & 0xFF000000) >> 24;
    uint8_t sub_evt = (evt & 0x00FF0000) >> 16;

    switch(base_evt){
        case SYS_MODULE:
        break;
        case SYS_ONE_SECOND:{
            stress_time_spent++;
            if(stress_time_spent >= MAX_STRESS_PERIOD){
                hrv_health_process_flag_stop();
                switch_module(e_DAILY, e_report_end_by_timer);
                return;
            }

            report_push_data_sec(g_afe.ehr_hr, INVALID_TEMP, accval, INVALID_SPO2, INVALID_STEP, e_report_professional_stress);
            
            if(NULL != hrv_health_process_result_get())
                switch_module(e_DAILY, e_report_end_by_auto);
            if(connect_crtl.isPaired == true && get_rt_en() == true)
                sendMonitorData_stress();
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
void STRESS_process_i2c(void *pMsg){
    uint16_t evt = *((uint16_t *)pMsg);

    i2c_init();
    /******** I2C ACC ***********/
    if (MAJOR_DEV_ACC == (MAJOR_DEV_MASK & evt)) {
        switch (OP_MODE_MASK & evt) {
        case OP_READ_INT_STATUS:
            if (2 == GET_OP_PARM(evt)) {
                NRF_LOG_INFO("STRESS_process_i2c int 2 wait_isr");
                NRF_LOG_INFO("start read");
                uint8_t ret = read_axis(e_acc_EHR); // Read ACC data and save them to SpO2, EHR or Step buffer.
                if (0 != ret) {
                    acc_start(IMU_EHR);
                }
                NRF_LOG_INFO("end read");
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

            send_raw_data_ehr(pData, data_len);
        }
        break;
        }
    }

    i2c_uninit();
}

bool STRESS_process_ble(uint8_t *pR, uint8_t *out){
    uint8_t cmd = *(pR);    // Response paired with the request.
    
    uint8_t rStatus = CMD_SUCCESS;
    switch(cmd){
        case APP_KEEPALIVE_CMD:{
            out[3] = get_batt_power_status();
            out[4] = get_batt_vol_percent();
            out[5] = e_STRESS;
            STRESS_set_adv_data(&out[6]);
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

uint8_t STRESS_type(){
    return e_STRESS;
}

void STRESS_set_adv_data(uint8_t *data){

}