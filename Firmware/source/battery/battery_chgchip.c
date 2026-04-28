#include "battery_chgchip.h"
#include "state_module_def.h"
#include "reports_def.h"
#include "battery_def.h"
#include "inner_flash_ram.h"

bool m_chgchip_timer_create = false;
bool m_chgchip_timer_run = false;
APP_TIMER_DEF(m_chgchip_timer_id);

struct chgchip_signal_s chgchip_signal;
static int charging_times;
static int not_charg_times;

DEF_CHARG_CHIP(bq21080);
static ring_chgchip_mode_t cur_chgchip_mode = RING_CHGCHIP_NONE_MODE;

void charg_delay_ms(uint32_t period) {
  // delay time
  nrf_delay_ms(period + 5);
} // user_delay_ms()

int8_t charg_I2C_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
  int8_t rslt = 0;

  uint8_t writebuf[4];
  if (len > 3) {
    return -1;
  }
  writebuf[0] = reg_addr; // MUST
  memcpy(writebuf + 1, reg_data, len);

  rslt = i2c_write(dev_id, reg_addr, writebuf, (uint8_t)len + 1);
  return rslt;
}

int8_t charg_I2C_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
  ret_code_t rslt;
  rslt = i2c_read(dev_id, reg_addr, reg_data, len);
  return (int8_t)rslt;
}

extern productMacSn pms;
uint8_t get_charge_intensity(){
    switch(pms.sn.SizeType.size){
        case 0: //L
            return 22;
        break;
        case 1: //M
            return 18;
        break;
        case 2: //S
            return 12;
        break;
        case 3: //M+
            return 18;
        break;
        default:
            return 12;
        break;
    }
}

bool chgchip_init(void){
    if(_chip.inited > 0)
        return true;

    charging_times = 0;
    not_charg_times = 0;
    i2c_init();
    NEW_CHARG_CHIP(bq21080);
    int8_t rslt = _chip.init(&_chip);

    NRF_LOG_INFO("_chip_init chip_id = 0x%02X,  rslt=%d", _chip.chip_id, rslt);

    chgchip_set_mode(RING_CHGCHIP_NORMAL);
    rslt = _chip.ok(&_chip);
    i2c_uninit();

    i2c_init();
    rslt = _chip.set_charge_intensity(get_charge_intensity(), &_chip);
    i2c_uninit();

    _chip.inited = 1;
    return rslt;
}

bool charg_chip_ok(){
    return _chip.ok(&_chip);
}

void set_charge_enable(bool enable){
    i2c_init();
    int8_t i = (enable == true) ? 0 : 1;
    int8_t rslt = _chip.set_charge_enable(i, &_chip);

    i2c_uninit();
}

bool chgchip_set_mode(ring_chgchip_mode_t mode){
    int8_t rslt = 0;

    if(_chip.dev_mode == mode)
        return true;

    //_chip.dev_mode = BQ21080_DEV_CHARGE_ADAPTER_MODE;

    i2c_init();
    rslt = _chip.set_mode(mode, &_chip);
    i2c_uninit();
    if  (BATTERY_OK != rslt)
        return false;

    return true;
}

extern uint8_t batt_vol_percent;
void update_batteryInfo_isr(void)
{
    i2c_init();
    uint8_t status = _chip.charg_status;
    if(_chip.get_cur_charg_status(&_chip, &status) != BATTERY_OK){
        i2c_uninit();
        NRF_LOG_ERROR("update_batteryInfo_isr get_cur_charg_status failed");
        return;
    }

    if(charging_times > 0){
        NRF_LOG_ERROR("charging_times is %d, now is %d, interval is %d", charging_times, UTC_getClock(), UTC_getClock() - charging_times);
        if(UTC_getClock() - charging_times > 60 && isBLEconnected() == false)
            request_sys_reset();
    }

    if(not_charg_times > 0){
        if(UTC_getClock() - not_charg_times > 5 && is_acc_in_sleep() == false){
            switch_module(e_DAILY, e_report_end_by_charging);
            not_charg_times = 0;
        }
    }
    
    if(_chip.charg_status != status){
        if(status == e_charging || status == e_charg_done){
            switch_module(e_CHARG, e_report_end_by_charging);
            if(_chip.charg_status == e_no_charg){
                charging_times = UTC_getClock();
                set_NI_ram_charging_start(charging_times, batt_vol_percent);
            }
        }else{
            charging_times = 0;
            if(_chip.charg_status == e_charging || _chip.charg_status == e_charg_done){
                not_charg_times = UTC_getClock();
                set_NI_ram_charging_start(0, 0);
            }
            //switch_module(e_DAILY, e_report_end_by_charging);
        }
    }
    _chip.charg_status = status;
    i2c_uninit();
}

static void chgchip_timer_handler(void *p_context) {

  chgchip_signal.interrupt = (MAJOR_DEV_BQ | OP_READ_INT_STATUS);
  sEvent tx_evt = {I2C_QID, &chgchip_signal.interrupt};
  taskPush(&tx_evt, false);

  m_chgchip_timer_run = false;
}

bool chgchip_timer_init(void) {
  uint32_t err_code;

  if (!m_chgchip_timer_create) {
    err_code = app_timer_create(&m_chgchip_timer_id,  APP_TIMER_MODE_SINGLE_SHOT,  chgchip_timer_handler);

    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("Create accel timer ok\r\n");
    m_chgchip_timer_create = true;
  }

}

bool chgchip_timer_start(void) {
  uint32_t err_code;

  if (!m_chgchip_timer_create) 
    return false;

  if (!m_chgchip_timer_run) {
    NRF_LOG_ERROR("chgchip_timer_start isr\r\n");
    err_code = app_timer_start(m_chgchip_timer_id, APP_TIMER_TICKS(CHGCHIP_DELAY_TIME_MS), NULL);
    if (err_code != NRF_SUCCESS) {
      NRF_LOG_ERROR("start accel timer fail\r\n");
      return false;
    }
    m_chgchip_timer_run = true;
  }

  return true;
}

bool chgchip_is_oncharger() {
    if(_chip.charg_status == e_charging || _chip.charg_status == e_charg_done)
        return true;
    else
        return false;
}

bool chgchip_is_charger_full() {
    if(_chip.charg_status == e_charg_done)
        return true;
    else
        return false;
}

uint8_t chgchip_charg_status(){
    return _chip.charg_status;
}

void charg_chip_wdt_handle(){
    i2c_init();
    _chip.wdt_handle(&_chip);
    i2c_uninit();
}

void show_chip_status(){
    i2c_init();
    _chip.show_status(&_chip);
    i2c_uninit();
}

void get_chip_status(uint8_t *s0, uint8_t *s1, uint8_t *f0, uint8_t *i1){
    i2c_init();
    _chip.get_status(&_chip, s0, s1, f0, i1);
    i2c_uninit();
}

void chg_hard_reset(){
    i2c_init();
    _chip.hardware_reset(&_chip);
    i2c_uninit();
}
