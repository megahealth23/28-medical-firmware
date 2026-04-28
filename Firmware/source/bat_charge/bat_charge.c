#include "bat_charge.h"
#include "BQ21080.h"
#include "app_timer.h"
#include "ring_bsp.h"
#include "ring_startup.h"
#include "ring_swtimer.h"

#include "I2C.h"
#include "mTask.h"

#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "system_clock.h"


bool bat_charge_start(){
  if (!i2c_init()) {
    NRF_LOG_INFO("i2c_init() failed!");
    return false;
  }

  if (!bq21080_init()) {
    NRF_LOG_INFO("bq21080_init() failed!");
    return false;
  }

  i2c_uninit();
  return true; 
}

bool bat_charge_stop(){
  NRF_LOG_INFO("bat_charge_stop! \n");

  return true;
}
