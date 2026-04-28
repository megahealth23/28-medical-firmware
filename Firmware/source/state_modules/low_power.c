#include "state_module.h"
#include "ring_afe.h"
#include "DataProcSPO2.h"


ret_code_t LP_init(){
    NIramCfg.cur_module = e_LP;
    ret_code_t ret = base_module_init();
    if(ret != STATE_MODULE_SUCCESS)
        return ret;

    peripherals_power_off_4_lp();
    acc_start(IMU_POWEROFF);
    acc_stop();
    //advertising_stop();
    return ret;
}

void LP_uninit(E_REPORT_END_TYPE e_end){
    if(!acc_stop())
        NRF_LOG_INFO("acc_stop FAILED. \n");

    base_module_uninit();
}

void LP_process_sys(void *pMsg){
    uint32_t evt = *((uint32_t*)pMsg);
    uint8_t base_evt = (evt & 0xFF000000) >> 24;
    uint8_t sub_evt = (evt & 0x00FF0000) >> 16;

    switch(base_evt){
        case SYS_ONE_SECOND:{
            if(5 >= get_batt_vol_percent()){
                ring_disconnect();
                advertising_stop();
                saadc_stop();
            }
        }
        break;
        default:
        break;
    }
}

void LP_process_i2c(void *PMsg){
    
}

bool LP_process_ble(uint8_t *pR, uint8_t *out){
    uint8_t cmd = *(pR);    // Response paired with the request.
    
    uint8_t rStatus = CMD_SUCCESS;
    switch(cmd){
        case APP_KEEPALIVE_CMD:{
            out[3] = get_batt_power_status();
            out[4] = get_batt_vol_percent();
            out[5] = e_LP;
            LP_set_adv_data(&out[6]);
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

uint8_t LP_type(){
    return e_LP;
}

void LP_set_adv_data(uint8_t *data){
    uint32_t duration = UTC_getClock() - NIramCfg.cur_module_start_time;
    data[0] = (duration >> 24) & 0xFF;
    data[1] = (duration >> 16) & 0xFF;
    data[2] = (duration >> 8) & 0xFF;
    data[3] = (duration >> 0) & 0xFF;

    data[4] = (g_afe.offhand_flg == 1 ? 0 : 1);
}