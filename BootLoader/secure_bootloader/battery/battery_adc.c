#include "app_timer.h"
#include "app_util_platform.h"
#include "nrf_drv_gpiote.h"

#include "battery_adc.h"
#include "eeg_battery.h"
//#include "eeg_bsp.h"
#include "nrf_log.h"

//#define USE_SAADC_TRACE
#ifdef USE_SAADC_TRACE
#define SAADC_TRACE_LOG NRF_LOG_INFO
#define SAADC_TRACE_ERROR NRF_LOG_ERROR
#else
#define SAADC_TRACE_LOG(...)
#define SAADC_TRACE_ERROR(...)
#endif

/////////////////////////////////////SAADC LOWPWER//////////////////////////////////////////////////////////////////////////////////
#define SAADC_CALIBRATION_INTERVAL 5 //Determines how often the SAADC should be calibrated relative to NRF_DRV_SAADC_EVT_DONE event. E.g. value 5 will make the SAADC calibrate every fifth time the NRF_DRV_SAADC_EVT_DONE is received.
#define SAADC_SAMPLES_IN_BUFFER 4    //Number of SAADC samples in RAM before returning a SAADC event. For low power SAADC set this constant to 1. Otherwise the EasyDMA will be enabled for an extended time which consumes high current.
#define SAADC_BURST_MODE 1           //Set to 1 to enable BURST mode, otherwise set to 0.
static nrf_saadc_value_t m_buffer_pool[2][SAADC_SAMPLES_IN_BUFFER];
static nrf_saadc_value_t m_adc_buffer[SAADC_SAMPLES_IN_BUFFER];
static uint32_t m_adc_evt_counter = 0;
static bool m_saadc_initialized = false;
uint16_t chg_calibrate_diff = 0;
nrf_saadc_value_t adc_value = 0;

struct battery_st BattInfo = {0};

static void saadc_init(void);

#define SAADC_START_SAMPLE_INTERVAL   APP_TIMER_TICKS(100)
#define SAADC_SAMPLE_INTERVAL         APP_TIMER_TICKS(100)

#define SAADC_CHARGE_LONG_WAIT_INTERVAL    APP_TIMER_TICKS(15000)  //lt 3.05V
#define SAADC_UNCHARGE_LONG_WAIT_INTERVAL  APP_TIMER_TICKS(30000)  //lt 3.05V

#define SAADC_CHARGE_SHORT_WAIT_INTERVAL    APP_TIMER_TICKS(1000)  //gt 3.05V
#define SAADC_UNCHARGE_SHORT_WAIT_INTERVAL  APP_TIMER_TICKS(3000)  //gt 3.05V

APP_TIMER_DEF(m_saadc_timer_id); /**< ble pair timer. */

static void saadc_timeout_handler(void *p_context) {
  UNUSED_PARAMETER(p_context);
 // uint32_t err_code;

  if (!m_saadc_initialized) {
    //saadc_init();    //Initialize the SAADC. In the case when SAADC_SAMPLES_IN_BUFFER > 1 then we only need to initialize the SAADC when the the buffer is empty.
  }
  m_saadc_initialized = true; //Set SAADC as initialized
  nrf_drv_saadc_sample();     //Trigger the SAADC SAMPLE task
  
  SAADC_TRACE_LOG("SAADC sampling start...\r"); //Toggle LED1 to indicate SAADC sampling start

 // APP_ERROR_CHECK(err_code);
}

static uint8_t convert_adc_2_percent(double v) {
  double k = 0.0;
  double b = 0.0;

  double max_adc = (double)ADC_MAX_SAMPLE, 
		 burst_adc = (double)ADC_BURST_SAMPLE,
		 lowpower_adc = (double)ADC_LOWPOWER_SAMPLE,  
		 shutdown_adc = (double)ADC_SHUTDOWN_SAMPLE;
  uint8_t ret = 0;

  if (v >= max_adc) {
    ret = 100;
    goto convert_adc_2_percent_end;
  } else if (v > burst_adc) {
    k = 0.147;
    b = -385.23;
  } else if (v > lowpower_adc) {
    k = 0.0272;
    b = -59.1;
  } else if (v > shutdown_adc) {
    k = 0.0416;
    b = -95.7;
  } else {
    k = 0.0416;
    b = -95.7;
    ret = (uint8_t)( ((k * v + b)>=1)?  (k * v + b)  :1);
    goto convert_adc_2_percent_end;
  }
  ret = (uint8_t)(k * v + b);
  // shouldn't show %0
convert_adc_2_percent_end:
  return ret;
}

void saadc_callback(nrf_drv_saadc_evt_t const *p_event) {
  static uint8_t sampletimes = 0;

  int32_t adc_value_mean;
  enum dev_power_state dev_ps;
  uint8_t batt_vol_percent = 0;

  if (p_event->type == NRF_DRV_SAADC_EVT_DONE) //Capture offset calibration complete event
  {
    ret_code_t err_code;

    SAADC_TRACE_LOG("SAADC buffer full\r"); //Toggle LED2 to indicate SAADC buffer full

    if ((m_adc_evt_counter % SAADC_CALIBRATION_INTERVAL) == 0) //Evaluate if offset calibration should be performed. Configure the SAADC_CALIBRATION_INTERVAL constant to change the calibration frequency
    {
      SAADC_TRACE_LOG("SAADC calibration starting...  \r");
      NRF_SAADC->EVENTS_CALIBRATEDONE = 0;                    //Clear the calibration event flag
      nrf_saadc_task_trigger(NRF_SAADC_TASK_CALIBRATEOFFSET); //Trigger calibration task
      while (!NRF_SAADC->EVENTS_CALIBRATEDONE)
        ; //Wait until calibration task is completed. The calibration tasks takes about 1000us with 10us acquisition time. Configuring shorter or longer acquisition time will make the calibration take shorter or longer respectively.
      while (NRF_SAADC->STATUS == (SAADC_STATUS_STATUS_Busy << SAADC_STATUS_STATUS_Pos))
        ; //Additional wait for busy flag to clear. Without this wait, calibration is actually not completed. This may take additional 100us - 300us

      SAADC_TRACE_LOG("SAADC calibration complete ! \r");
    }

    err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAADC_SAMPLES_IN_BUFFER); //Set buffer so the SAADC can write to it again. This is either "buffer 1" or "buffer 2"
    APP_ERROR_CHECK(err_code);

    SAADC_TRACE_LOG("ADC event number: %d\r", (int)m_adc_evt_counter); //SAADC event number.

    m_adc_buffer[sampletimes] = p_event->data.done.p_buffer[0];
    sampletimes++;

    if (sampletimes < SAADC_SAMPLES_IN_BUFFER) {
         err_code = app_timer_start(m_saadc_timer_id, SAADC_SAMPLE_INTERVAL, NULL);
         goto quit_group_sample;
    }

    int32_t adc_value_sum = 0;

    for (int i = 1; i < SAADC_SAMPLES_IN_BUFFER; i++) {
      SAADC_TRACE_LOG("%d\r",m_adc_buffer[i]); //Output SAADC samples.
      adc_value_sum += m_adc_buffer[i];
    }
    adc_value_mean = adc_value_sum / (SAADC_SAMPLES_IN_BUFFER - 1);

    //NRF_LOG_INFO("battery adc value = %d",adc_value_sum);
    //NRF_LOG_FLUSH();

    adc_value = adc_value_mean;

    if (!device_is_oncharger()) // no charging
    {
      BattInfo.is_oncharger = false;
      //if (0 == adc_value)
      //  adc_value = adc_value_mean;
      //else {
      //  adc_value = MIN(adc_value_mean, adc_value);
      //}

      batt_vol_percent = convert_adc_2_percent((double)adc_value);

      if (ADC_LOWPOWER_SAMPLE < adc_value)
        dev_ps = DEV_NORMAL;
      else if (ADC_SHUTDOWN_SAMPLE < adc_value) {
        dev_ps = DEV_LOWPWR;
//        stopMonitoring(STOP_LOWPOWER); // Stop monitoring and recording.
      } else
        dev_ps = DEV_SHUTDOWN;
      chg_calibrate_diff = 0;
    } else // charging
    {
      BattInfo.is_oncharger = true;

      //if (0 == adc_value)
      //  adc_value = adc_value_mean;
      //else
      //  adc_value = MAX((adc_value + adc_value_mean) / 2, adc_value_mean);
      batt_vol_percent = convert_adc_2_percent((double)adc_value);
      if (batt_vol_percent == 100) {
        //ADC_CALIBRATE_MAX min, ????,?????
        if (chg_calibrate_diff < ADC_CALIBRATE_MAX) {
          chg_calibrate_diff++;
          batt_vol_percent = batt_vol_percent - 10 + chg_calibrate_diff * 10 / ADC_CALIBRATE_MAX;
        }
        dev_ps = (batt_vol_percent >= 100) ? DEV_CHGFULL : DEV_CHGING;
      } else {

        batt_vol_percent -= (batt_vol_percent > 12) ? 12 : 0;
        dev_ps = DEV_CHGING;
        chg_calibrate_diff = 0;
      }

      /*charg-Chip pull-down when charging, and pull-up when discharge*/
      if (nrf_drv_gpiote_in_is_set(CHG_STAT_PIN))
	BattInfo.is_full = true;
							
      if (BattInfo.is_full) {
        batt_vol_percent = 100;
        dev_ps = DEV_CHGFULL;
      } else
        dev_ps = DEV_CHGING;
    }
    //dev_ps = DEV_NORMAL;
    BattInfo.adc_val = adc_value;
    BattInfo.dev_ps = dev_ps;
    BattInfo.is_working = true;
    BattInfo.vol_pct = batt_vol_percent;
//    BattInfo.vol = CONVERT_ADC_2_VOL(adc_value);

    SAADC_TRACE_LOG("ADC value = %d, batt_vol_percent=%d, dev_ps=%d",
    adc_value, batt_vol_percent, BattInfo.dev_ps);

     sampletimes = 0; //start
     if (!device_is_oncharger()) {                                       // no charging

          if( ADC_SHUTDOWN_SAMPLE <= BattInfo.adc_val  )
              err_code = app_timer_start(m_saadc_timer_id, SAADC_UNCHARGE_SHORT_WAIT_INTERVAL, NULL); //ADC sample interval
          else
              err_code = app_timer_start(m_saadc_timer_id, SAADC_UNCHARGE_LONG_WAIT_INTERVAL, NULL); //ADC sample interval

     } else {

          if( ADC_SHUTDOWN_SAMPLE <= BattInfo.adc_val  )
              err_code = app_timer_start(m_saadc_timer_id, SAADC_CHARGE_SHORT_WAIT_INTERVAL, NULL);
          else
              err_code = app_timer_start(m_saadc_timer_id, SAADC_CHARGE_LONG_WAIT_INTERVAL, NULL);
     }

quit_group_sample:

    m_adc_evt_counter++;

    nrf_drv_saadc_uninit();                                                     //Unintialize SAADC to disable EasyDMA and save power
    NRF_SAADC->INTENCLR = (SAADC_INTENCLR_END_Clear << SAADC_INTENCLR_END_Pos); //Disable the SAADC interrupt

    sd_nvic_ClearPendingIRQ(SAADC_IRQn);
    m_saadc_initialized = false; //Set SAADC as uninitialized
  }
}

static void saadc_init(void) {
  ret_code_t err_code;

  //Configure SAADC
  nrf_drv_saadc_config_t saadc_config = NRF_DRV_SAADC_DEFAULT_CONFIG;
  saadc_config.oversample = NRF_SAADC_OVERSAMPLE_4X; //Set oversample to 4x. This will make the SAADC output a single averaged value when the SAMPLE task is triggered 4 times.

  //Initialize SAADC
  err_code = nrf_drv_saadc_init(&saadc_config, saadc_callback); //Initialize the SAADC with configuration and callback function. The application must then implement the saadc_callback function, which will be called when SAADC interrupt is triggered
  APP_ERROR_CHECK(err_code);

  //Configure SAADC channel
  nrf_saadc_channel_config_t channel_config =
      NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0); // ring  AIN0

  //Configure SAADC channel
  channel_config.gain = NRF_SAADC_GAIN1_2; //Set input gain to 1/2. The maximum SAADC input voltage is then 0.6V/(1/4)=2.4V. The single ended input range is then 0V-2.4V

  //Initialize SAADC channel
  err_code = nrf_drv_saadc_channel_init(0, &channel_config); //Initialize SAADC channel 0 with the channel configuration
  APP_ERROR_CHECK(err_code);

  if (SAADC_BURST_MODE) {
    NRF_SAADC->CH[0].CONFIG |= 0x01000000; //Configure burst mode for channel 0. Burst is useful together with oversampling. When triggering the SAMPLE task in burst mode, the SAADC will sample "Oversample" number of times as fast as it can and then output a single averaged value to the RAM buffer. If burst mode is not enabled, the SAMPLE task needs to be triggered "Oversample" number of times to output a single averaged value to the RAM buffer.
  }

  err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAADC_SAMPLES_IN_BUFFER/SAADC_SAMPLES_IN_BUFFER); //Set SAADC buffer 1. The SAADC will start to write to this buffer
  APP_ERROR_CHECK(err_code);

  err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1], SAADC_SAMPLES_IN_BUFFER/SAADC_SAMPLES_IN_BUFFER); //Set SAADC buffer 2. The SAADC will write to this buffer when buffer 1 is full. This will give the applicaiton time to process data in buffer 1.
  APP_ERROR_CHECK(err_code);
}

/**
 * @brief Function for adc rtc run.
 */
void saadc_run(bool timer) {
  uint32_t err_code;

  if (timer) {
    err_code = app_timer_create(&m_saadc_timer_id,
        APP_TIMER_MODE_SINGLE_SHOT,
        saadc_timeout_handler);
    if (err_code != NRF_SUCCESS) {
      SAADC_TRACE_ERROR("battery create saadc softtimer fail\r");
    }
    SAADC_TRACE_LOG("battery create saadc softtimer ok\r");
  }
  err_code = app_timer_start(m_saadc_timer_id, SAADC_START_SAMPLE_INTERVAL, NULL);

  if (err_code != NRF_SUCCESS) {
    SAADC_TRACE_ERROR("start saadc softtimer fail\r");
    return;
  }

  SAADC_TRACE_LOG("start saadc softtimer ok\r");
  //adc_value = 0;
  m_saadc_initialized = false;
}

void saadc_stop(void) {
  app_timer_stop(m_saadc_timer_id);

  if (m_saadc_initialized == true) {
    nrf_drv_saadc_abort();
    nrf_drv_saadc_uninit();
  }

  m_saadc_initialized = false; //Set SAADC as uninitialized
  adc_value = 0;

  NRF_SAADC->INTENCLR = (SAADC_INTENCLR_END_Clear << SAADC_INTENCLR_END_Pos); //Disable the SAADC interrupt
  sd_nvic_ClearPendingIRQ(SAADC_IRQn);
}