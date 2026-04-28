/*
 * Copyright (c) 2019 MegaHealth. All Rights Reserved.
 * @file
 * @date 2020.07.29
 * @version
 * @brief
 */

#include "ring_battery.h"
#include "app_util_platform.h"
#include "battery_adc.h"
#include "ring_bsp.h"

#include "nrf_log.h"
#include "system_clock.h"

static void oncharger_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {}

static void battChg_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
  nrf_drv_gpiote_pin_t dect_bq21080_int_pin = DETECTION_BQ21080_INT_PIN;

  if (pin == dect_bq21080_int_pin) {
      chgchip_timer_start();
  }
}

bool device_is_oncharger() {
    return chgchip_is_oncharger();
}

bool device_is_charge_full() {
    return chgchip_is_charger_full();
}

uint8_t device_charg_status(){
    return chgchip_charg_status();
}

/*********************************************************************
 * @fn      battChgPin_init
 * @brief   Initialize the battery charge pin.
 *		   callbalk.
 */
static bool battChgPin_init(void) {
  uint32_t err_code;
  nrf_drv_gpiote_pin_t dect_bq21080_int_pin = DETECTION_BQ21080_INT_PIN;

	//nrf_gpio_cfg_default(dect_bq21080_int_pin);
 //   nrf_gpio_cfg(dect_bq21080_int_pin,
 //                   NRF_GPIO_PIN_DIR_OUTPUT,
 //                   NRF_GPIO_PIN_INPUT_DISCONNECT,
 //                   NRF_GPIO_PIN_NOPULL,
 //                   NRF_GPIO_PIN_S0D1,
 //                   NRF_GPIO_PIN_NOSENSE);
 //   nrf_gpio_pin_set(dect_bq21080_int_pin);
 //   return true;

  nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);
  in_config.pull = NRF_GPIO_PIN_PULLUP;
  err_code = nrf_drv_gpiote_in_init(dect_bq21080_int_pin, &in_config, battChg_handler);
  APP_ERROR_CHECK(err_code);

  if (err_code != NRF_SUCCESS) {
    NRF_LOG_INFO("battery charge pin config fail");
    return false;
  }
  NRF_LOG_INFO("battery charge pin config ok");

  nrf_drv_gpiote_in_event_enable(dect_bq21080_int_pin, true);

  NRF_LOG_INFO("battery charge pin config ok\r\n");

  return true;
}

void battery_init(void) {
    chgchip_init();
}

void battery_start_check(void) {
    battChgPin_init();
    saadc_run(true);    // start battery sample ,get battery percent(battery ADC VALUE)

    chgchip_timer_init();
}

void battery_uninit(void) {
  nrf_drv_gpiote_pin_t dect_bq21080_int_pin = DETECTION_BQ21080_INT_PIN;

  saadc_stop();

  //nrf_gpio_cfg_default(BATT_CHG_CURRENT_CTRL_PIN);  //default  pull-up big-current
  nrf_drv_gpiote_in_uninit(dect_bq21080_int_pin);
}

extern uint8_t batt_vol_percent;
uint8_t get_batt_vol_percent(){
    //uint8_t status = chgchip_charg_status();
    //switch(status){
    //case e_charging:{
    //    uint32_t charged_time = UTC_getClock() - get_NI_ram_charging_start_time();
    //    uint32_t charged_percent = charged_time*100/FULL_CHARGE_PERIOD;
    //    printf("now: %d, charging time: %d, duration: %d, charging percent: %d, charged percent: %d\r\n", 
    //        UTC_getClock(), get_NI_ram_charging_start_time(), charged_time, get_NI_ram_charging_start_percent(), charged_percent);
    //    uint8_t percent = (get_NI_ram_charging_start_percent() + charged_percent) > 95 ? 95 : (get_NI_ram_charging_start_percent() + charged_percent);
    //    set_NI_ram_charged_percent(percent);
    //    return percent;
    //}
    //case e_charg_done:
    //    uint8_t percent = 100;
    //    set_NI_ram_charged_percent(percent);
    //    return percent;
    //default:
        return batt_vol_percent;
    //}
}

extern nrf_saadc_value_t batt_adc_value;
uint8_t get_batt_adc_value(){
    return batt_adc_value;
}

extern float batt_vol;
uint8_t get_batt_vol(){
    return (uint8_t)(batt_vol*10);
}

uint8_t get_batt_power_status(){
    uint8_t status = chgchip_charg_status();
    switch(status){
    case e_charging:
        return DEV_CHGING;
    case e_charg_done:
        return DEV_CHGFULL;
    case e_no_charg:
        if(BATTERY_VOL_10_PER > batt_vol)
            return DEV_LOWPWR;
        else
            return DEV_NORMAL;
    default:
        return DEV_FAULTS;
    }
}