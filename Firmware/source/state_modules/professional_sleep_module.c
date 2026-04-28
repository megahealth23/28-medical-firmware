#include "state_module.h"
#include "ring_afe.h"
#include "nrf_log.h"
#include "ble_ring.h"
#include "protocol.h"
#include "ble_service.h"
#include "inner_flash_ram.h"
#include "ring_startup.h"

#define PS_TEMP_PERIOD (60)
extern NIRAM_t NIramCfg;

//heart process
extern uint16_t g_rr_mean;
extern int HistHrsavepos;

extern uint16_t g_HrRRNum[HR_ALG_RR_NUM];
extern uint8_t g_HrRRCnt;
extern int pre_data_len;

ret_code_t PS_init(){
    NIramCfg.cur_module = e_PS;
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
    //if(NIramCfg.cur_module != e_PS)
    //    clear_module_report_ram();

    return STATE_MODULE_SUCCESS;
}

void PS_uninit(E_REPORT_END_TYPE e_end){
    if(!acc_stop())
        NRF_LOG_INFO("acc_stop FAILED. \n");
    if(!afe_stop())
        NRF_LOG_INFO("afe_stop FAILED. \n");
    if(!tmp_stop(RING_INSIDE_TMP117))
        NRF_LOG_INFO("tmp_stop FAILED. \n");
    uint16_t report_id;
    if(report_sec_stop(e_report_professional_sleep, e_end, &report_id) != NRF_SUCCESS)
        NRF_LOG_INFO("report_sec_stop FAILED. \n");
    clear_monitor_report_ram();    

    base_module_uninit();
}

uint8_t test_hr = 0;
uint8_t test_spo2 = 0;
void PS_process_sys(void *pMsg){
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
            temp_read(PS_TEMP_PERIOD);
            //report_push_data_sec(test_hr, INVALID_TEMP, accval, test_spo2, INVALID_STEP, e_report_professional_sleep);
            //test_hr++;
            //test_spo2++;
            //report_push_data_sec(test_hr, INVALID_TEMP, accval, test_spo2, INVALID_STEP, e_report_professional_sleep);
            //test_hr++;
            //test_spo2++;
            //report_push_data_sec(test_hr, INVALID_TEMP, accval, test_spo2, INVALID_STEP, e_report_professional_sleep);
            //test_hr++;
            //test_spo2++;
            //report_push_data_sec(test_hr, INVALID_TEMP, accval, test_spo2, INVALID_STEP, e_report_professional_sleep);
            //test_hr++;
            //test_spo2++;
            //report_push_data_sec(test_hr, INVALID_TEMP, accval, test_spo2, INVALID_STEP, e_report_professional_sleep);
            //test_hr++;
            //test_spo2++;
            //report_push_data_sec(test_hr, INVALID_TEMP, accval, test_spo2, INVALID_STEP, e_report_professional_sleep);
            //test_hr++;
            //test_spo2++;
            //report_push_data_sec(test_hr, INVALID_TEMP, accval, test_spo2, INVALID_STEP, e_report_professional_sleep);
            //test_hr++;
            //test_spo2++;
            //report_push_data_sec(test_hr, INVALID_TEMP, accval, test_spo2, INVALID_STEP, e_report_professional_sleep);
            //test_hr++;
            //test_spo2++;
            //report_push_data_sec(test_hr, INVALID_TEMP, accval, test_spo2, INVALID_STEP, e_report_professional_sleep);
            //test_hr++;
            //test_spo2++;
            //report_push_data_sec(test_hr, INVALID_TEMP, accval, test_spo2, INVALID_STEP, e_report_professional_sleep);
            //test_hr++;
            //test_spo2++;

            report_push_data_sec(g_afe.hr, gCurrent_body_temperature(), accval, g_afe.spo2, INVALID_STEP, e_report_professional_sleep);
            //static uint8_t rr[200] = {0};
            //static uint8_t ii = 1;
            //memset(rr, ii, sizeof(rr));
            //report_push_data_hrv(rr, sizeof(rr), e_report_professional_sleep);
            //ii++;
            processHRV(e_report_professional_sleep);
            
            if(is_acc_in_sleep()){
                if(UTC_getClock() % 3 == 0){
                    acc_set_hold_data(38, e_acc_SPO2);
                }
            }

            if(connect_crtl.isPaired == true && get_rt_en() == true)
                sendMonitorData(e_PS);

            
            //UTCTime _now = UTC_getClock();
            //if(_now % (30) == 0){
            //    ASSERT(1==2);
            //}
        }
        break;
        default:
        break;
    }
}

void PS_process_i2c(void *pMsg){
    uint16_t evt = *((uint16_t *)pMsg);

    i2c_init();
    /******** I2C ACC ***********/
    if (MAJOR_DEV_ACC == (MAJOR_DEV_MASK & evt)) {
        switch (OP_MODE_MASK & evt) {
        case OP_READ_INT_STATUS:
            if (2 == GET_OP_PARM(evt)) {
                NRF_LOG_INFO("PS_process_i2c int 2 wait_isr");
                uint8_t ret = read_axis(e_acc_SPO2); // Read ACC data and save them to SpO2, EHR or Step buffer.
                if (0 != ret) {
                    acc_start(IMU_SPO2);
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
                  
            uint8_t ret = SPO2_bufProc_wavelet((void *)pData, data_len);
            if (ret == 1) {
                afe_spo2_proc_wavelate();
            }

            // send_raw_data(pData, data_len);
            }
        break;
        }
    }

    i2c_uninit();
}

bool PS_process_ble(uint8_t *pR, uint8_t *out){
    uint8_t cmd = *(pR);    // Response paired with the request.
    
    uint8_t rStatus = CMD_SUCCESS;
    switch(cmd){
        case APP_KEEPALIVE_CMD:{
            out[3] = get_batt_power_status();
            out[4] = get_batt_vol_percent();
            out[5] = e_PS;
            PS_set_adv_data(&out[6]);
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

uint8_t PS_type(){
    return e_PS;
}

void PS_set_adv_data(uint8_t *data){
    uint32_t duration = UTC_getClock() - NIramCfg.cur_module_start_time;
    data[0] = (duration >> 24) & 0xFF;
    data[1] = (duration >> 16) & 0xFF;
    data[2] = (duration >> 8) & 0xFF;
    data[3] = (duration >> 0) & 0xFF;

    data[4] = (g_afe.Spo2show >> 2) + 37;
    data[5] = g_afe.shr;
    data[6] = g_afe.flag;
    data[7] = (g_afe.offhand_flg == 1 ? 0 : 1);
}