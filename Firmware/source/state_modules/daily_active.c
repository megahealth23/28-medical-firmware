#include "state_module.h"
#include "ring_power.h"
#include "peripheral/peripherals.h"
#include "ring_utc.h"
#include "DataProcEHR.h"
#include "DataProcHealth.h"
#include "steps_statistics.h"

ret_code_t DAILY_init(){
    NIramCfg.cur_module = e_DAILY;
    ret_code_t ret = base_module_init();
    if(ret != STATE_MODULE_SUCCESS)
        return ret;
		
	advertising_start(true);
    peripherals_power_off_without_power();
    acc_start(IMU_POWEROFF);

    return STATE_MODULE_SUCCESS;
}

void DAILY_uninit(E_REPORT_END_TYPE e_end){
}

void DAILY_process_sys(void *pMsg){
}

void DAILY_process_i2c(void *pMsg){
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
}