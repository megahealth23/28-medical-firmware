#include "state_module.h"
#include "ring_power.h"
#include "peripheral/peripherals.h"
#include "ring_utc.h"
#include "DataProcEHR.h"
#include "DataProcHealth.h"
#include "steps_statistics.h"

 #define HRV_HEALTH_BLOCK_CNT 200 //200 *sizeof(uint_16) = 400 bytes 

#define POINT_HR_PROCESS_MAX_CNT (10)
#define DA_TEMP_PERIOD (60)
static uint16_t mark_duration = 0;
static bool mark_stress = false;
static bool turn_back_step = false;
static uint16_t point_hr_process_cnt = 0;
//heart process
extern uint16_t g_rr_mean;
extern int HistHrsavepos;
extern uint8_t g_point_pre_peak_pos;// 24*100/2048*60 = 70.3
ret_code_t DAILY_init(){
    NIramCfg.cur_module = e_DAILY;
    ret_code_t ret = base_module_init();
    if(ret != STATE_MODULE_SUCCESS)
        return ret;

    peripherals_power_off_without_power();
    mark_duration = 0;
    acc_start(IMU_STEPS);
    acc_clear_fifo();
    //acc_start_active_check_mode();
    turn_back_step = false;
    //ACC_Sleep_judge_data_clear();
                //PPG_board_power_on();
                //afe_stop();
    //tmp117_measure_init(true, true, MEASURE_MODE_DAILY);
    //reset_steps_count_by_module();
    //reset_cur_auto_sleep_time();
    //start_sleep_auto_start_check();
    g_rr_mean = 0;
    HistHrsavepos = 0;
    ACC_Exercise_judge_data_clear();
    ACC_Sleep_judge_data_clear();
    g_point_pre_peak_pos = 24;// 24*100/2048*60 = 70.3
    //start temperature
    tmp_start(RING_TMP_ONE_SHOT_MODE, RING_INSIDE_TMP117);

    return STATE_MODULE_SUCCESS;
}

void DAILY_uninit(E_REPORT_END_TYPE e_end){
    off_hand_check_set(OFF_HAND_OPEN);
    hrv_health_process_flag_stop();
    wdt_feed();
    idle_state_handle();
    report_push_data_daily(0xFE, INVALID_TEMP, accval, 
                    INVALID_SPO2, get_steps_count_by_module(), get_batt_vol_percent());
    reset_steps_count_by_module();
    //steps_clear();
    wdt_feed();
    idle_state_handle();
    report_daily_save();

    //acc_stop_active_check_mode();
    acc_start(IMU_POWEROFF);
    acc_stop();
    afe_stop();
   // ACC_Exercise_judge_data_clear();

    wdt_feed();
    idle_state_handle();
    // start_sleep_auto_end_check();
    //ACC_Sleep_judge_data_clear();
    //tmp117_measure_deinit();
    base_module_uninit();
}

//void send_null_acc_to_algrthm(uint8_t fifo_len, uint8_t statu){
//    for (int idx = 0; idx < fifo_len; idx++) {
//        acc_axis acc;

//        acc.x = 1;
//        acc.y = 1;
//        acc.z = 1;
//        stepsnum_update(&acc, statu);
//    }
//}

void DAILY_process_sys(void *pMsg){
    uint32_t evt = *((uint32_t*)pMsg);
    uint8_t base_evt = (evt & 0xFF000000) >> 24;
    uint8_t sub_evt = (evt & 0x00FF0000) >> 16;

    switch(base_evt){
        case SYS_MODULE:
        break;
        case SYS_ONE_SECOND:{
            temp_read(DA_TEMP_PERIOD);
            NRF_LOG_INFO("step now %d, mark_duration: %d", steps_num(), mark_duration);
            if(mark_duration > 0){
                mark_duration--;
                report_push_data_daily(g_afe.point_hr, INVALID_TEMP, accval, 
                INVALID_SPO2, INVALID_STEP, get_batt_vol_percent());
                if(mark_duration <= 0){
                    turn_back_step = true;
                }
            }

            UTCTime _now = UTC_getClock();
            //if(check_auto_sleep_start_3() == true){
                //NRF_LOG_INFO("=== acc_get_Sleep_flag() = %d\n",acc_get_Sleep_flag());
            if(_now % 10 == 0){
                if(acc_get_Sleep_flag() == START_SLEEP_MODE){
                    start_auto_sleep();
                    break;
                }
            }
            //}

            if(is_acc_in_sleep()){
                NRF_LOG_INFO("DAILY_process_sys send_null_acc_to_algrthm!!");
                if(mark_duration > 0){
                    acc_set_hold_data(100, e_acc_EHR);
                    
                    //acc_axis axis[100];
                    //memset(axis, 1, sizeof(axis));
                    //uint8_t max_len = 100;
                    //uint16_t acc_len = 0;
                    //while(acc_len < max_len){
                    //    uint8_t send_len = acc_len + 10 > max_len ? max_len - acc_len : 10;
                    //    send_acc_raw_data(&axis[acc_len], send_len);
                    //    acc_len = acc_len + send_len;
                    //}
                }else{
                    //if((_now & 0x1) == 0){
                        acc_set_hold_data(50, e_acc_STEP);
                    
                        //acc_axis axis[100];
                        //memset(axis, 1, sizeof(axis));
                        //uint8_t max_len = 100;
                        //uint16_t acc_len = 0;
                        //while(acc_len < max_len){
                        //    uint8_t send_len = acc_len + 10 > max_len ? max_len - acc_len : 10;
                        //    send_acc_raw_data(&axis[acc_len], send_len);
                        //    acc_len = acc_len + send_len;
                        //}
                    //}
                }
            }

            if(turn_back_step == true){
                turn_back_step = false;
                mark_duration = 0;
                hrv_health_process_flag_stop();
                report_push_data_daily(g_afe.point_hr, gCurrent_body_temperature(), accval, 
                                INVALID_SPO2, get_steps_count_by_module(), get_batt_vol_percent());
                reset_steps_count_by_module();

                afe_stop();
                acc_start(IMU_POWEROFF);
                acc_stop();
                //AFE4900_HWshutdown();
                //PPG_board_power_off();

                acc_start(IMU_STEPS);
                //acc_start_active_check_mode();
                acc_clear_fifo();
                break;
            }

            if(_now % DAILY_MARK_PERIOD == 0){
                NRF_LOG_INFO("DAILY_process_sys start mark!!");
                //PPG_board_power_on();
                //reset_cur_auto_sleep_time();
                off_hand_check_set(OFF_HAND_CLOSE);
                if(daily_sleep_offhand_flag_get() == 1){
                    off_hand_check_set(OFF_HAND_OPEN);
                }else{
                    if(_now % (60*60) == 0){
                        off_hand_check_set(OFF_HAND_OPEN);
                    }
                }

                bool r;
                //acc_stop_active_check_mode();
                advertising_start(true);
                acc_start(IMU_EHR);
                acc_clear_fifo();
                afe_start(RING_AFE_DAILY_MODE);

                report_push_data_daily(INVALID_HR, gCurrent_body_temperature(), accval, 
                                INVALID_SPO2, get_steps_count_by_module(), get_batt_vol_percent());
                reset_steps_count_by_module();
                //tmp_start(RING_TMP_ONE_SHOT_MODE, RING_INSIDE_TMP117);
                
                //if(_now % DAILY_STRESS_MARK_PERIOD == 0 && NIramCfg.auto_stress_mark == 1){
                NRF_LOG_INFO("DAILY_process_ble DAILY_STRESS START. \n");
                //start stress process
                start_circle_proc_hrv(HEALTH_NON_PROFESSIONAL_TYPE);
                mark_stress = true;
                mark_duration = DAILY_STRESS_MARK_MAX_DURATION;

                //point EHR process
                sport_heart_algorithm_init();
                point_hr_process_cnt = 0;
                fifo32_recover(&EHR_green_fifo,0);
                fifo_acc_recover(&EHR_acc_fifo, 0);
                //} else {
                //    mark_duration = DAILY_MARK_MAX_DURATION;
                //}
            }else if(_now % 60 == 0){
                report_push_data_daily(INVALID_HR, gCurrent_body_temperature(), accval, 
                                INVALID_SPO2, get_steps_count_by_module(), get_batt_vol_percent());
                reset_steps_count_by_module();
            }

            //if(connect_crtl.isPaired == true)
            //    sendMonitorData_daily();
        }
        break;
        default:
        break;
    }
}

static void send_raw_data_2(struct afe4900_sensor_data* data, uint16_t len){
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

uint8_t DAILY_Stress_Heart_get(uint16_t *RRData,uint16_t RR_num)
{
    uint8_t hr = 0;
    uint16_t RR_sum = 0;
    uint8_t vaildRRCnt = 0;
    if(RR_num > 10)
    {
        for(int i = RR_num - 10;i < RR_num;i++)
        {
            if( RRData[i] != 255 &&  RRData[i] != 0)
            {
                RR_sum = RR_sum + RRData[i];
                vaildRRCnt = vaildRRCnt + 1;
            }

        }
        if(vaildRRCnt != 0)
        {
            RR_sum = RR_sum/vaildRRCnt;
        }
        
    }
    else if(RR_num <= 10)
    {
        for(int i = 0;i < RR_num;i++)
        {
            if( RRData[i] != 255 &&  RRData[i] != 0)
            {
                RR_sum = RR_sum + RRData[i];
                vaildRRCnt = vaildRRCnt + 1;
            }
            
        }
        if(vaildRRCnt != 0)
        {
            RR_sum = RR_sum/vaildRRCnt;
        }
        
    }
    if(RR_sum != 0)
    {
        hr = (uint8_t)(60000/(RR_sum*10));
    }
    
    
    return hr;
}

extern accFifo_t ax;
extern uint8_t g_hrv_health_process_flag;

void DAILY_process_i2c(void *pMsg){
    uint16_t evt = *((uint16_t *)pMsg);

    i2c_init();
    /******** I2C ACC ***********/
    if (MAJOR_DEV_ACC == (MAJOR_DEV_MASK & evt)) {
        switch (OP_MODE_MASK & evt) {
        case OP_READ_INT_STATUS:
            if (2 == GET_OP_PARM(evt)) {
                NRF_LOG_INFO("DAILY_process_i2c int 2 wait_isr");
                //if(is_adv_stoped() == true){
                //   uint8_t fifo_len = acc_clear_fifo();
                //   send_null_acc_to_algrthm(fifo_len, e_acc_STEP);
                //   break;
                //}

                
                //acc_test();

                if(mark_duration > 0){
                    uint8_t ret = read_axis(e_acc_EHR); // Read ACC data and save them to SpO2, EHR or Step buffer.
                    NRF_LOG_INFO("DAILY_process_i2c acc_start IMU_SPO2!!");
                    acc_start(IMU_EHR);

                    //if(NIramCfg.high_precision_mark == 0){
                        uint8_t flag = acc_get_Exercise_flag();
                        if(flag == 1){
                            switch_module(e_PA, e_report_end_by_auto);
                        }
                    //}
                }
                else{
                    uint8_t ret = read_axis(e_acc_STEP); // Read ACC data and save them to SpO2, EHR or Step buffer.
                    NRF_LOG_INFO("DAILY_process_i2c acc_start IMU_STEPS!!");
                    acc_start(IMU_STEPS);
                    
                    //if(NIramCfg.high_precision_mark == 0){
                        uint8_t flag = acc_get_Exercise_flag();
                        if(flag == 1){
                            switch_module(e_PA, e_report_end_by_auto);
                        }
                    //}
                }

                 //int8_t acc_len = 0;
                 //while(acc_len < ax.n){
                 //    uint8_t send_len = acc_len + 10 > ax.n ? ax.n - acc_len : 10;
                 //    send_acc_raw_data(&ax.axis[acc_len], send_len);
                 //    acc_len = acc_len + send_len;
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
            if(mark_duration <= 0){
                break;
            }
                  
            uint8_t hr = 0xFF;
            uint8_t point_offhand_flag = 0;
            uint8_t ret = point_data_bufProc((void *)pData, data_len, &point_offhand_flag);
            if (ret == 1) {
                if(true == mark_stress){
                    NRF_LOG_INFO("hrv_health_process ing!!");
                    //check is stress process success
                    if(hrv_health_process_result_get() != NULL && g_hrv_health_process_flag == HRV_HEALTH_PROCESS_FINISH_FLAG)
                    {
                        NRF_LOG_INFO("hrv_health_process success!!");
                        HRV_Para* hrvHealth = hrv_health_process_result_get();
                        if(hrvHealth != NULL)
                        {
                            uint16_t rr_cnt = hrvHealth->RR_num > HRV_HEALTH_BLOCK_CNT ? HRV_HEALTH_BLOCK_CNT : hrvHealth->RR_num;
                            // report_push_data_daily_hrv((uint8_t*)(&hrvHealth->green_RR[0]), rr_cnt, e_report_daily);
                            report_push_data_daily_hrv((uint8_t*)(&hrvHealth->green_RR[0]), rr_cnt*2, e_report_daily);
                            //for(int i = 0;i < hrvHealth->RR_num;i = i + HRV_HEALTH_BLOCK_CNT)
                            //{
                            //    if(i+HRV_HEALTH_BLOCK_CNT >= hrvHealth->RR_num)
                            //    {
                            //        report_push_data_daily_hrv((uint8_t*)(&hrvHealth->green_RR[i]), (hrvHealth->RR_num - i)*2, e_report_daily);
                            //        break;
                            //    }
                            //    else{
                            //        report_push_data_daily_hrv((uint8_t*)(&hrvHealth->green_RR[i]), HRV_HEALTH_BLOCK_CNT*2, e_report_daily);
                            //    }
                                
                            //}

                        }

                        
                        // hr = DAILY_Stress_Heart_get(&hrvHealth->green_RR[0],hrvHealth->RR_num);
                        // // smooth daily hr
                        // if(hr > 130 && hr - g_afe.point_hr > 50 && g_afe.point_hr != 0)
                        // {
                        //     g_afe.point_hr = g_afe.point_hr + 10;
                        // }
                        // else if(hr < 50 && g_afe.point_hr - hr > 15 && g_afe.point_hr != 0)
                        // {
                        //     g_afe.point_hr = g_afe.point_hr - 2;
                        // }
                        // else{
                        //     g_afe.point_hr = hr;
                        // }
                        if(point_hr_process_cnt < POINT_HR_PROCESS_MAX_CNT)
                        {
                            if(afe_point_proc() == 1){
                                NRF_LOG_INFO("afe_point_proc successed!!");
                                // turn_back_step = true;
                                point_hr_process_cnt = point_hr_process_cnt + 1;
                                hr = g_afe.point_hr;
                            }
                        }
                        turn_back_step = true;
                        mark_duration = 0;
                        mark_stress = false;

                    }
                    if(point_hr_process_cnt < POINT_HR_PROCESS_MAX_CNT)
                    {
                        if(afe_point_proc() == 1){
                            NRF_LOG_INFO("afe_point_proc successed!!");
                            // turn_back_step = true;
                            point_hr_process_cnt = point_hr_process_cnt + 1;
                            hr = g_afe.point_hr;
                        }
                    }

                    // if success
                    // hr = g_afe.point_hr;
                    // mark_duration = 0;
                    // mark_stress = false;
                    // report_push_data_daily_hrv max data length is 500  e_report_daily
                } else {
                    if(afe_point_proc() == 1){
                        NRF_LOG_INFO("afe_point_proc successed!!");
                        turn_back_step = true;
                        mark_duration = 0;
                        hr = g_afe.point_hr;
                    }
                }

                hr_algorithm_time_acc_sample_num_claer();
            }

            //send_raw_data_2(pData, data_len);
        }
        break;
        }
    }

    i2c_uninit();
}

bool DAILY_process_ble(uint8_t *pR, uint8_t *out){
    uint8_t cmd = *(pR);    // Response paired with the request.
    
    uint8_t rStatus = CMD_SUCCESS;
    switch(cmd){
        case APP_KEEPALIVE_CMD:{
            out[3] = get_batt_power_status();
            out[4] = get_batt_vol_percent();
            out[5] = e_DAILY;
            DAILY_set_adv_data(&out[6]);
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

uint8_t DAILY_type(){
    return e_DAILY;
}

void DAILY_set_adv_data(uint8_t *data){
    uint32_t duration = UTC_getClock() - NIramCfg.cur_module_start_time;
    data[0] = (duration >> 24) & 0xFF;
    data[1] = (duration >> 16) & 0xFF;
    data[2] = (duration >> 8) & 0xFF;
    data[3] = (duration >> 0) & 0xFF;

    uint32_t step_in_day = get_steps_in_day();
    data[4] = (step_in_day >> 24) & 0xFF;
    data[5] = (step_in_day >> 16) & 0xFF;
    data[6] = (step_in_day >> 8) & 0xFF;
    data[7] = (step_in_day >> 0) & 0xFF;

    uint16_t step_in_15_min = get_steps_in_15_min();
    data[8] = (step_in_15_min >> 8) & 0xFF;
    data[9] = (step_in_15_min >> 0) & 0xFF;

    data[10] = g_afe.flag;
    data[11] = (g_afe.offhand_flg == 1 ? 0 : 1);
}