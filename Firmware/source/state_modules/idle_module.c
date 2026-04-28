#include "state_module.h"


ret_code_t IDLE_init(){

    peripherals_power_off_without_power();
    acc_start(IMU_POWEROFF);
    return STATE_MODULE_SUCCESS;
}

void IDLE_uninit(E_REPORT_END_TYPE e_end){

}

void IDLE_process_sys(void *PMsg){

}

void IDLE_process_i2c(void *PMsg){

}

bool IDLE_process_ble(uint8_t *pR, uint8_t *out){
    uint8_t cmd = *(pR);    // Response paired with the request.
    
    uint8_t rStatus = CMD_SUCCESS;
    switch(cmd){
        case APP_KEEPALIVE_CMD:{
            out[3] = get_batt_power_status();
            out[4] = get_batt_vol_percent();
            out[5] = e_IDLE;
            IDLE_set_adv_data(&out[6]);
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

uint8_t IDLE_type(){
    return e_IDLE;
}

void IDLE_set_adv_data(uint8_t *data){

}