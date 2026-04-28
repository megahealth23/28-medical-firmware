#include "state_module.h"
#include "battery_def.h"

ret_code_t CHARG_init(){
    peripherals_power_off_without_power();
    acc_start(IMU_SPO2);
    return STATE_MODULE_SUCCESS;
}

void CHARG_uninit(E_REPORT_END_TYPE e_end){
    if(!acc_stop())
        NRF_LOG_INFO("acc_stop FAILED. \n");

    report_push_data_daily(0xFE, INVALID_TEMP, 0, 
                    INVALID_SPO2, 0, get_batt_vol_percent());
    report_daily_save();

    base_module_uninit();
}

void CHARG_process_sys(void *pMsg){
    uint32_t evt = *((uint32_t*)pMsg);
    uint8_t base_evt = (evt & 0xFF000000) >> 24;
    uint8_t sub_evt = (evt & 0x00FF0000) >> 16;

    switch(base_evt){
        case SYS_ONE_SECOND:{
            show_chip_status();
            UTCTime _now = UTC_getClock();
            if(_now % 60 == 0){
                report_push_data_daily(INVALID_HR, 0, 0, 
                                INVALID_SPO2, 0, get_batt_vol_percent());
            }
        }
        break;
        default:
        break;
    }
}

void CHARG_process_i2c(void *PMsg){

}

bool CHARG_process_ble(uint8_t *pR, uint8_t *out){
    uint8_t cmd = *(pR);    // Response paired with the request.
    
    uint8_t rStatus = CMD_SUCCESS;
    switch(cmd){
        case APP_KEEPALIVE_CMD:{
            out[3] = get_batt_power_status();
            if(get_batt_power_status() != DEV_CHGFULL)
                out[3] = DEV_CHGING;
            out[4] = get_batt_vol_percent();
            out[5] = e_CHARG;
            CHARG_set_adv_data(&out[6]);
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

uint8_t CHARG_type(){
    return e_CHARG;
}

void CHARG_set_adv_data(uint8_t *data){
    uint32_t duration = UTC_getClock() - NIramCfg.cur_module_start_time;
    data[0] = (duration >> 24) & 0xFF;
    data[1] = (duration >> 16) & 0xFF;
    data[2] = (duration >> 8) & 0xFF;
    data[3] = (duration >> 0) & 0xFF;
}