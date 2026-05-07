#include "state_module_manager.h"
#include "inner_flash_ram.h"
#include "state_module.h"
#include "mTask.h"
#include "nrf_dfu.h"
#include "battery_chgchip.h"
#include "fds_manager.h"
#include "nrf_log.h"

typedef    ret_code_t      (*state_module_init)();
typedef    uint8_t      (*state_module_type)();
typedef    void        (*state_module_uninit)(E_REPORT_END_TYPE e_end);
typedef    void        (*state_module_process)(void* pMsg);
typedef    bool        (*state_module_process_ble)(uint8_t *pR, uint8_t *out);
typedef    void        (*state_module_set_adv_data)(uint8_t *data);

typedef struct st_state_module{
    state_module_init init;
    state_module_uninit uninit;
    state_module_type module_type;
    state_module_process process_sys;
    state_module_process process_i2c;
    state_module_process_ble process_ble;
    state_module_set_adv_data set_adv_data;
} ST_STATE_MODULE;

static char const *state_str[] =
{
    "e_IDLE",
    "e_LP",                         //low power
    "e_CHARG",                      //charging
    "e_PA",                         //pysical active
    "e_PS",                         //professional sleep
    "e_EXER",                       //exercise
    "e_DS",                         //daily sleep
    "e_DAILY",                      //daily active
    "e_REAL",                       //real time
    "e_STRESS",                     //test stress
    "e_LPS",                        //low power sleep
    "e_BP",                         //blood pressure
};

static const uint32_t s_system_ship_evt = SYS_SHIP << 24;
static const uint32_t s_system_reset = SYS_RESET << 24;

uint32_t system_start_time = 0;
uint8_t air_plane_mode = 0; //0: close, 1: open

extern NIRAM_t NIramCfg;
static ST_STATE_MODULE s_state_module = {0};
#define SET_STATE_MODULE_DETAIL(_module)                        \
        s_state_module.init = _module##_init;                   \
        s_state_module.uninit = _module##_uninit;               \
        s_state_module.module_type = _module##_type;            \
        s_state_module.process_sys = _module##_process_sys;     \
        s_state_module.process_i2c = _module##_process_i2c;     \
        s_state_module.process_ble = _module##_process_ble;     \
        s_state_module.set_adv_data = _module##_set_adv_data;

#define CASE_STATE_MODULE(_module)              \
    case e_##_module:                           \
        SET_STATE_MODULE_DETAIL(_module)        \
        break;

#define  CLEAR_STATE_MODULE()                       \
        s_state_module.init = NULL;                 \
        s_state_module.uninit = NULL;               \
        s_state_module.process_sys = NULL;          \
        s_state_module.process_i2c = NULL;          \
        s_state_module.process_ble = NULL;          \
        s_state_module.set_adv_data = NULL;
    
ret_code_t set_current_module(E_STATE_MODULE e_module){
    switch(e_module){
        CASE_STATE_MODULE(IDLE);
        CASE_STATE_MODULE(LP);
        CASE_STATE_MODULE(CHARG);
        CASE_STATE_MODULE(PA);
        CASE_STATE_MODULE(PS);
        CASE_STATE_MODULE(DS);
        CASE_STATE_MODULE(EXER);
        CASE_STATE_MODULE(DAILY);
        CASE_STATE_MODULE(REAL);
        CASE_STATE_MODULE(STRESS);
        CASE_STATE_MODULE(LPS);
        CASE_STATE_MODULE(BP);
        default:
            CLEAR_STATE_MODULE();
            return STATE_MODULE_STATE_INVALID;
    }

    return STATE_MODULE_SUCCESS;
}

static bool is_module_start = false;
ret_code_t module_run(E_STATE_MODULE module){
    peripherals_check_init();
    afeFifoRdyinit(); //afe int init

    if(is_module_start == true)
        return STATE_MODULE_STATE_INVALID;

    ret_code_t ret = STATE_MODULE_SUCCESS;
    ret = set_current_module(module);
    if(ret != STATE_MODULE_SUCCESS)
        return ret;

    ret = s_state_module.init();
    if(ret != STATE_MODULE_SUCCESS)
        return ret;

    is_module_start = true;
    return ret;
}

extern user_descriptor* user_desp;
extern RingInfo_t gRIval;
extern int Patch;
extern int Minor;
extern int Major;
ret_code_t state_modules_init(){
    system_start_time = UTC_getClock();
    fds_m_init();
    air_plane_mode = 0;

    gRIval.hwVer = 7 << 4 | 0;                         // 8.0
    gRIval.swVer = Major << 4 | Minor;                         // 5.X represent V8 ring.   4.x: V6.  3.x: V5.
    gRIval.rev = htons(Patch);      

    nrf_dfu_settings_t *p_dfu_settings = (nrf_dfu_settings_t *)BOOTLOADER_SETTINGS_ADDRESS;                     //
    gRIval.btVer = p_dfu_settings->bootloader_version; // V8 boot should be 3.X ; 2<<4 | 0 = 2.0;

    report_init();
    battery_start_check();
    ble_power_update();

    ret_code_t ret = STATE_MODULE_SUCCESS;
    E_STATE_MODULE e_module = e_IDLE;

    if(chgchip_is_oncharger()){
        return module_run(e_CHARG);
    }

    return STATE_MODULE_SUCCESS;
}

ret_code_t state_modules_switch(E_STATE_MODULE e_module, E_REPORT_END_TYPE e_end){
    ret_code_t ret = STATE_MODULE_SUCCESS;
    if(is_module_start == false)
        return STATE_MODULE_STATE_INVALID;

    if(s_state_module.module_type() == e_module){
        return ret;
    }

    if(s_state_module.uninit != NULL){
        s_state_module.uninit(e_end);
    }

    if(!IS_USER_ID_VALID(user_desp->user_id)){
        if(e_module == e_DAILY)
            e_module = e_IDLE;
    }

    if(e_module == e_DAILY && get_rt_en() == true)
        e_module = e_REAL;

    ret = set_current_module(e_module);
    if(ret != STATE_MODULE_SUCCESS)
        return ret;

    ret = s_state_module.init();
    if(ret != STATE_MODULE_SUCCESS)
        return ret;

    return STATE_MODULE_SUCCESS;
}


void state_modules_uninit(){
    if(s_state_module.uninit != NULL){
        s_state_module.uninit(e_report_end_by_reset);
    }
}

void charging_wdt_feed(void){
    if(0 != (app_get_sec()% CHGCHIP_FEEDWDT_PERIOD))
        return;

    charg_chip_wdt_handle();
}

void request_sys_reset(void){
    sEvent evt = { SYS_QID, &s_system_reset};
    taskPush(&evt, false);
}

void request_sys_ship(void){    
    sEvent evt = { SYS_QID, &s_system_ship_evt};
    taskPush(&evt, false);
}

static uint8_t app_set_time(uint8_t *pR, uint8_t *out) {
    uint8_t rStatus = 0;
    uint8_t idx = 0;
    uint32_t abs_time = 0;
    // get rtc sync value
    abs_time = ((uint32_t)pR[3] << 24) + ((uint32_t)pR[4] << 16) + ((uint32_t)pR[5] << 8) + pR[6];
    NRF_LOG_INFO("app_set_time old time %u s, now is %u s, diff is %ds.", UTC_getClock(), abs_time, abs_time - UTC_getClock());

    if (abs_time == 0)
        rStatus = OPERA_DISALLOW;
    else {
        rStatus = CMD_SUCCESS;
        NIramCfg.auto_sleep_timezone_hour = pR[7];
        NIramCfg.auto_sleep_timezone_min = pR[8];

        NRF_LOG_INFO("app_set_time timezone : 0x%02X:0x%02X", NIramCfg.auto_sleep_timezone_hour,NIramCfg.auto_sleep_timezone_min);
        UTC_setClock(abs_time);
    }

    out[idx++] = (abs_time >> 24) & 0xFF;
    out[idx++] = (abs_time >> 16) & 0xFF;
    out[idx++] = (abs_time >> 8) & 0xFF;
    out[idx++] = (abs_time >> 0) & 0xFF;

    updateNIClock();

    return rStatus;
}

static uint8_t app_get_time(uint8_t *pR, uint8_t *out) {
    uint8_t rStatus = 0;
    uint8_t idx = 0;
    uint32_t abs_time = UTC_getClock();
    rStatus = CMD_SUCCESS;

    out[idx++] = (abs_time >> 24) & 0xFF;
    out[idx++] = (abs_time >> 16) & 0xFF;
    out[idx++] = (abs_time >> 8) & 0xFF;
    out[idx++] = (abs_time >> 0) & 0xFF;
    out[idx++] = (system_start_time >> 24) & 0xFF;
    out[idx++] = (system_start_time >> 16) & 0xFF;
    out[idx++] = (system_start_time >> 8) & 0xFF;
    out[idx++] = (system_start_time >> 0) & 0xFF;

    return rStatus;
}

extern void wdt_feed(void);
static void power_monitor_one_second(void) {
    wdt_feed();
    charging_wdt_feed();

    switch (get_batt_power_status()) {
        case DEV_CHGFULL:
            ring_chargefull_indicate();
        break;
        case DEV_NORMAL:
            ring_stop_indicate();
        break;
        case DEV_CHGING:{
            ring_charging_indicate();
            //nrf_drv_gpiote_pin_t handon_led_pin = NRF_GPIO_PIN_MAP(0, 5);
            //nrf_gpio_cfg(handon_led_pin,
            //                NRF_GPIO_PIN_DIR_OUTPUT,
            //                NRF_GPIO_PIN_INPUT_DISCONNECT,
            //                NRF_GPIO_PIN_NOPULL,
            //                NRF_GPIO_PIN_S0S1,
            //                NRF_GPIO_PIN_NOSENSE);
            ////nrf_gpio_pin_set(handon_led_pin);
            //nrf_gpio_pin_clear(handon_led_pin);
        }
        break;
        case DEV_LOWPWR:
        break;
        default:
        break;
    }

    return;
}

void state_modules_SYS_process(void *pMsg){
    uint32_t evt = *((uint32_t*)pMsg);
    uint8_t base_evt = (evt & 0xFF000000) >> 24;
    uint8_t sub_evt = (evt & 0x00FF0000) >> 16;

    switch(base_evt){
        case SYS_MODULE:
            if(sub_evt == SYS_FLASH_FULL){
            }else if(sub_evt == SYS_BLE_DISCONNECT){
                set_rt_en(false);
                if(s_state_module.module_type != NULL){
                    if(s_state_module.module_type() == e_STRESS)
                        state_modules_switch(e_DAILY, e_report_end_by_user);
                }
            }else if(sub_evt == SYS_UPDATE_NI_FLASH){
                update_NI_flash();
            }else if(sub_evt == SYS_UPDATE_NI_FLASH_LOCAL){
                update_NI_flash_local();
            }else if(sub_evt == SYS_SWITCH_MODULE){
                uint8_t module = (evt & 0x0000FF00) >> 8;
                uint8_t end_type = (evt & 0x000000FF);
                state_modules_switch(module, end_type);
            }else if(sub_evt == SYS_DEL_REPORT){
                uint16_t report_id = (evt & 0x0000FFFF);
                report_remove(report_id);
            }
        break;
        case SYS_ONE_SECOND:{
            if(is_module_start == false){
                NRF_LOG_INFO("state_modules_SYS_process SYS_ONE_SECOND, module not start, time is %u", UTC_getClock());
                if(get_batt_vol_percent() > 0){
                    E_STATE_MODULE e_module = NIramCfg.cur_module;
                    if(e_module == e_LP){
                        e_module = e_IDLE;
                    }

                    if(IS_USER_ID_VALID(user_desp->user_id)){
                        if(e_module == e_IDLE)
                            e_module = e_DAILY;
                    }

                    if(is_flash_full()){
                        e_module = e_IDLE;
                    }

                    if(e_module == e_REAL || e_module == e_STRESS)
                        e_module = e_DAILY;

                    if(get_batt_power_status() == DEV_LOWPWR)
                        e_module = e_LP;

                    module_run(e_module);
                }
            }else{
                NRF_LOG_INFO("state_modules_SYS_process SYS_ONE_SECOND, state is %s, time is %u", state_str[s_state_module.module_type()], UTC_getClock());
            }

            if(get_batt_power_status() == DEV_LOWPWR)
                switch_module(e_LP, e_report_end_by_user);

            uint32_t _now = UTC_getClock();
            update_steps_per_sec(_now);
            power_monitor_one_second();
            //notifyPower();
            updateNIClock();
            AdvUpdate();
            if(_now % 10 == 0){
                update_batteryInfo_isr();
                
                i2c_init();
                acc_sync_all_status();
                if(is_acc_in_sleep()){
                    acc_set_bypass();
                    acc_set_sleep_int(false);
                    acc_set_wake_int(true);
                }else{
                    acc_set_continous();
                    acc_set_sleep_int(true);
                    acc_set_wake_int(false);
                }
                i2c_uninit();
            }
        }
        break;
        case SYS_SHIP:
            if(s_state_module.uninit != NULL)
                s_state_module.uninit(e_report_end_by_user);
            nrf_delay_ms(2);
            chgchip_set_mode(RING_CHGCHIP_TRANSPORT);
        break;
        case SYS_RESET:
            if(s_state_module.uninit != NULL)
                s_state_module.uninit(e_report_end_by_reset);
            nrf_delay_ms(2);
            nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_RESET);
        break;
        default:
        break;
    }

    if(s_state_module.process_sys != NULL)
        s_state_module.process_sys(pMsg);
}

void state_modules_I2C_process(void *pMsg){
    uint16_t evt = *((uint16_t *)pMsg);

    i2c_init();
    /******** I2C ACC ***********/
    if (MAJOR_DEV_ACC == (MAJOR_DEV_MASK & evt)) {
        switch (OP_MODE_MASK & evt) {
        case OP_READ_INT_STATUS:
            if (1 == GET_OP_PARM(evt)) {
                NRF_LOG_INFO("S int 11111111111111111111 wait_isr");
                set_acc_int1_status();

                acc_sync_all_status();
                if(is_tap_int())  //Tap -> int2
                {
                    connect_crtl.wait_isr = 1;
                    acc_config_Tap(false);
                }

                if(is_wakeup_detect())
                {
                    acc_set_continous();
                    acc_set_sleep_int(true);
                    acc_set_wake_int(false);
                }

                if(is_acc_sleep_detect()){
                    acc_set_bypass();
                    acc_set_sleep_int(false);
                    acc_set_wake_int(true);
                }
            }

            if (2 == GET_OP_PARM(evt)) {
                NRF_LOG_INFO("S int 22222222222222222222222 wait_isr");
                set_acc_int2_status();
            }
        break;
        }
    }

    /******** I2C TMP ***********/
    if (MAJOR_DEV_TMP == (MAJOR_DEV_MASK & evt)) {
        switch (OP_MODE_MASK & evt) {
        case OP_MODE_START:
            tmp_start((ring_tmp_mode_t)GET_OP_PARM(evt), GET_MINOR_DEV(evt));
        break;
        case OP_READ_SENSOR_DATA:{
            struct tmp117_sensor_data *tmp_pt;
            read_tmp_value(&tmp_pt, GET_MINOR_DEV(evt), true, RING_TMP_ONE_SHOT_MODE);
            NRF_LOG_INFO("tmp117: celcius: " NRF_LOG_FLOAT_MARKER ", temprature:%d", NRF_LOG_FLOAT(tmp_pt->celcius), tmp_pt->temp_val);
            }
        break;
        }
    }

    //******** I2C BQ ***********
    if (MAJOR_DEV_BQ == (MAJOR_DEV_MASK & evt)) {
        switch (OP_MODE_MASK & evt) {
           case OP_READ_INT_STATUS:
                update_batteryInfo_isr();
            break;
            case OP_READ_SENSOR_DATA:
            break;
            default:
            break;
        }
    }

    i2c_uninit();
    if(s_state_module.process_i2c != NULL)
        s_state_module.process_i2c(pMsg);
}

bool switch_module(E_STATE_MODULE module, uint8_t end_type){
    if(is_module_start == false)
        return false;
    s_sys_module_evt = (SYS_MODULE << 24) | (SYS_SWITCH_MODULE<<16) | (module<<8) | end_type;
    sEvent evt = { SYS_QID, &s_sys_module_evt};
    taskPush(&evt, false);
    return true;
}

void try_remove_report(uint16_t report_id){
    s_sys_report_evt = (SYS_MODULE << 24) | (SYS_DEL_REPORT<<16) | report_id;
    sEvent evt = { SYS_QID, &s_sys_report_evt};
    taskPush(&evt, false);
}

extern uint8_t sub_exercise_type;
bool state_modules_BLE_process(uint8_t *pR, uint8_t *out){
    if(s_state_module.process_ble != NULL){
        if(s_state_module.process_ble(pR, out))
            return true;
    }

    uint8_t cmd = *(pR);    // Response paired with the request.
    E_STATE_MODULE e_module;
    uint8_t rStatus = CMD_SUCCESS;
    switch(cmd){
        case APP_SET_PS_CMD: // SpO2 sleep monitor and record.
            if(s_state_module.module_type() == e_LP || s_state_module.module_type() == e_CHARG){
                rStatus = MODULE_NOT_ALLOW;
                break;
            }
            e_module = e_PS;
            //e_module = e_DS;
            if(switch_module(e_module, e_report_end_by_user) == false)
                rStatus = MODULE_NOT_ALLOW;
        break;
        case APP_SET_SPORT_CMD:
            if(s_state_module.module_type() == e_LP || s_state_module.module_type() == e_CHARG){
                rStatus = MODULE_NOT_ALLOW;
                break;
            }
            if(s_state_module.module_type() == e_EXER){
                break;
            }
            e_module = e_EXER;
            NIramCfg.cur_sub_module = pR[3];
            out[3] = NIramCfg.cur_sub_module;
            if(switch_module(e_module, e_report_end_by_user) == false)
                rStatus = MODULE_NOT_ALLOW;
        break;
        case APP_SET_DAILY_CMD:
            if(s_state_module.module_type() == e_LP || s_state_module.module_type() == e_CHARG){
                rStatus = MODULE_NOT_ALLOW;
                break;
            }
            if(get_rt_en() == true){
                e_module = e_REAL;
            }else{
                e_module = e_DAILY;
            }

            out[3] = get_current_sec_report_id();
            if(switch_module(e_module, e_report_end_by_user) == false)
                rStatus = MODULE_NOT_ALLOW;
        break;
        case APP_GET_REPORTS_LIST:{
            if(s_state_module.module_type == NULL){
                rStatus = MODULE_NOT_ALLOW;
                break;
            }

            if(s_state_module.module_type() == e_PS || 
            s_state_module.module_type() == e_PA || 
            s_state_module.module_type() == e_DS || 
            s_state_module.module_type() == e_EXER){
                rStatus = MODULE_NOT_ALLOW;
                break;
            }

            //each group has 16 byte
            out[3] = pR[3];
            uint16_t index = 0;
            if(pR[3] == 0){
                index = 0;
            }else if(pR[3] == 1){
                index = 16;
            }else{
                rStatus = REPORT_LIST_OVER;
                break;
            }

            for(int i = 4; i < 20; i++, index++){
                out[i] = NIramCfg.sec_report_ids[index];
            }
        }
        break;
        case APP_GET_REPORT_BLOCK_CNT:{
            NRF_LOG_INFO("APP_GET_REPORT_BLOCK_CNT receive 0x%02X ox%02X", pR[3], pR[4]);
            *((uint16_t*)(&out[3])) = *((uint16_t*)(&pR[3]));
            uint16_t report_id = *((uint16_t*)(&pR[3]));
            NRF_LOG_INFO("APP_GET_REPORT_BLOCK_CNT report id %d", report_id);
            uint16_t block_cnt = report_start_read_file(report_id);
            if(block_cnt <= 0){
                rStatus = REPORT_IS_NULL;
                break;
            }
            
            *((uint16_t*)(&out[5])) = block_cnt;
        }
        break;
        case APP_GET_REPORT_BLOCK:{
            rStatus = CMD_NO_RETURN;
            *((uint16_t*)(&out[3])) = *((uint16_t*)(&pR[3]));
            *((uint16_t*)(&out[5])) = *((uint16_t*)(&pR[5]));
            uint16_t report_id = *((uint16_t*)(&pR[3]));
            uint16_t block = *((uint16_t*)(&pR[5]));
            uint16_t* pLen = (uint16_t*)(&out[7]);
            uint16_t* pCrc = (uint16_t*)(&out[9]);
            NRF_LOG_INFO("report_start_read_block report id %d, block %d", report_id, block);
            if(report_start_read_block(report_id, block, pLen, pCrc) != NRF_SUCCESS){
                rStatus = REPORT_BLOCK_ERR;
                
                //uint8_t d[20] = {0};
                //d[0] = APP_GET_REPORT_BLOCK;
                //d[1] = get_ble_msg_sn();
                //d[2] = REPORT_BLOCK_ERR;
                //*((uint16_t*)(&d[3])) = report_id;
                //*((uint16_t*)(&d[5])) = block;
                //ring_indicate(d, 20); 
                break;
            }
            report_start_sync_block();
        }
        break;
        case APP_DELETE_REPORT:{
            *((uint16_t*)(&out[3])) = *((uint16_t*)(&pR[3]));
            uint16_t report_id = *((uint16_t*)(&pR[3]));
            uint16_t* pLen = (uint16_t*)(&out[7]);
            uint16_t* pCrc = (uint16_t*)(&out[9]);
            NRF_LOG_INFO("APP_DELETE_REPORT report id %d", report_id);
            try_remove_report(report_id);
            //if(report_remove(report_id) != NRF_SUCCESS){
            //    rStatus = REPORT_BLOCK_ERR;
            //}
        }
        break;
        case APP_SET_TIME_CMD: {// set time
            uint32_t interval = app_get_sec() - NIramCfg.last_charge_time;
            rStatus = app_set_time(pR, &out[3]);
            NIramCfg.last_charge_time = app_get_sec() - interval;
            if(s_state_module.module_type != NULL){
                if(s_state_module.module_type() == e_IDLE)
                    if(switch_module(e_DAILY, e_report_end_by_user) == false)
                        rStatus = MODULE_NOT_ALLOW;
            }
        }
        break;
        case APP_GET_TIME_CMD:
            rStatus = app_get_time(pR, &out[3]);
        break;
        case APP_GET_RT_DATA_CMD:{
            uint8_t v = pR[3];              // 0,1 Normal start or stop;  2,3 DirectControl start or stop.
            if (v == 0 || v == 2) { // start realtime data.
                set_rt_en(true);
                if(s_state_module.module_type != NULL){
                    if(s_state_module.module_type() == e_DAILY || s_state_module.module_type() == e_DS)
                        if(switch_module(e_REAL, e_report_end_by_user)== false)
                            rStatus = MODULE_NOT_ALLOW;
                }
            } else if (v == 1 || v == 3) { // stop
                set_rt_en(false);
                //if(s_state_module.module_type() == e_REAL)
                //    switch_module(e_DAILY, e_report_end_by_user);
            } else {
              rStatus = PARAM_NOT_RESOLVE;
            }
        }
        break;
        //case APP_KEEPALIVE_CMD:
        //    out[3] = get_batt_vol_percent();
        //    //out[3] = get_batt_vol();
        //    out[4] = get_batt_power_status();
        //    out[5] = s_state_module.module_type();
        //    out[6] = get_batt_adc_value() & 0xff;
        //    out[7] = (get_batt_adc_value() >> 8) & 0xff;
        //    out[8] = (uint8_t)g_afe.offhand_flg;
        //break;
        case APP_SET_MAC_SN:
            reset_max_sn(&pR[3]);
        break;
        case APP_SET_AUTO_MONITOR_CMD: 
            set_auto_sleep(&pR[3]);
        break;
        case APP_SET_AUTO_STRESS_CMD:
            set_auto_stress(&pR[3]);
        break;
        case APP_SET_DEVICE_SHIP:{
            sEvent evt = { SYS_QID, &s_system_ship_evt};
            taskPush(&evt, false);
        }
        break;
        case APP_GET_CHIP_STATUS:{ 
            get_chip_status(&out[3],&out[4],&out[5],&out[6]);
        }
        break;
        case APP_SET_STRESS_CMD:{
            if(s_state_module.module_type() == e_LP || s_state_module.module_type() == e_CHARG){
                rStatus = MODULE_NOT_ALLOW;
                break;
            }
            e_module = e_STRESS;
            if(switch_module(e_module, e_report_end_by_user) == false)
                rStatus = MODULE_NOT_ALLOW;
        }
        break;
        case APP_START_BP_CMD:{
            if(s_state_module.module_type() == e_LP || s_state_module.module_type() == e_CHARG){
                rStatus = MODULE_NOT_ALLOW;
                break;
            }
            e_module = e_BP;
            if(switch_module(e_module, e_report_end_by_user) == false)
                rStatus = MODULE_NOT_ALLOW;
        }
        break;
        case RING_STOP_STRESS_CMD:
            rStatus = CMD_NO_RETURN;
        break;
        case APP_SET_LOW_POWER_CMD:{
            set_high_precision_mark(&pR[3]);
        }
        break;
        case APP_SET_AIRPLANE_CMD:{
            air_plane_mode = 1;
            ring_disconnect();
            advertising_stop();
        }
        break;
        case APP_SET_CHARGE_ENABLE:{ 
            set_charge_enable(pR[3]);
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

void set_adv_scan_rsp_data(uint8_t* data){
    if(s_state_module.module_type != NULL){
        data[0] = s_state_module.module_type();
        s_state_module.set_adv_data(&data[1]);
    }else{
        data[0] = e_IDLE;
    }
}


E_STATE_MODULE cur_module_type(){
    if(s_state_module.module_type != NULL){
        return s_state_module.module_type();
    }

    return e_IDLE;
}

