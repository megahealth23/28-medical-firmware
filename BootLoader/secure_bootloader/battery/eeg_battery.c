#include "sdk_config.h"

#include "eeg_battery.h"
#include "app_util_platform.h"
#include "battery_adc.h"

#include "nrf_log.h"
#include "app_gpiote.h"

/*********************************************************************
* @fn      battChg_stat_handler
* @brief   gpiote change interupt.
*/
static void battChg_stat_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	return;
}


/*********************************************************************
* @fn      battChg_handler
* @brief   gpiote change interupt.
*/

static void battChg_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
  if (pin == BATT_CHG_PIN) {
    if (!nrf_drv_gpiote_in_is_set(BATT_CHG_PIN)) { // Low, NOCHARGE
      BattInfo.is_oncharger = false;
      NRF_LOG_INFO("Charge off.");
    } else { // Hight, Charge on. We should reset.
      BattInfo.is_oncharger = true;
      NRF_LOG_INFO("Charge on.");
      BattInfo.dev_ps = DEV_CHGING;

      saadc_stop();
      // Should safe backup in NIRAM. And then reset.
      NVIC_SystemReset(); // RESET
    }
    // restart saadc_timer.
    saadc_stop();
    saadc_run(false);
  }
}

/*********************************************************************
* @fn      battChgPin_init
* @brief   Initialize the battery charge pin. 
*		   callbalk.
*/
static bool battChgPin_init(void) {
  uint32_t err_code;

  if (!nrfx_gpiote_is_init()) {
    err_code = nrfx_gpiote_init();
    APP_ERROR_CHECK(err_code);
  }

  // DOUBLE EDGE
  nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
  in_config.pull = NRF_GPIO_PIN_NOPULL;

#if 1
  err_code = nrf_drv_gpiote_in_init(BATT_CHG_PIN, &in_config, battChg_handler);

  APP_ERROR_CHECK(err_code);
  if (err_code != NRF_SUCCESS) {
    NRF_LOG_INFO("Charge pin config fail");
    return false;
  }
  NRF_LOG_INFO("Charge pin config ok");

  nrf_drv_gpiote_in_event_enable(BATT_CHG_PIN, true);
#endif

//  nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
  in_config.pull = NRF_GPIO_PIN_PULLUP;
  err_code = nrf_drv_gpiote_in_init(CHG_STAT_PIN, &in_config, battChg_stat_handler);
	
  APP_ERROR_CHECK(err_code);
  if(err_code != NRF_SUCCESS)
  {
//    DEBUG_ERROR("battery charge pin config fail\r\n");
    return false;
  }
//	DEBUG_LOG("battery charge pin config ok\r\n");
  nrf_drv_gpiote_in_event_enable(CHG_STAT_PIN, true);

	return true;
}

void battery_init(void) {
  //nrf_gpio_cfg_output(BATT_CHG_CURRENT_CTRL_PIN);
  //charger_large_current();

  //memset((void *)&BattInfo, 0, sizeof(BattInfo));
  //BattInfo.dev_ps = DEV_NORMAL;
  //battChgPin_init();

  //saadc_run(true);   //start battery sample ,get battery percent(battery ADC VALUE)
  
//  nrf_delay_ms(100); //wait 500ms, get a adc value
}

bool device_is_oncharger(void) {
  /* HIGH: oncharge, LOW: offcharge */
  bool isOn = nrf_drv_gpiote_in_is_set(BATT_CHG_PIN);
  //nrf_drv_gpiote_in_uninit(CHG_STAT_PIN);
  return isOn;
}

void charger_large_current(void)
{
  nrf_gpio_pin_set(BATT_CHG_CURRENT_CTRL_PIN);
}

void charger_small_current(void)
{
  nrf_gpio_pin_clear(BATT_CHG_CURRENT_CTRL_PIN);
}