/*
* Copyright (c) 2019 MegaHealth. All Rights Reserved.
* @file
* @date 2020.07.29
* @version
* @brief
*/

#include "app_timer.h"
#include "app_util_platform.h"
#include "nrf_drv_gpiote.h"

#include "battery_adc.h"
#include "nrf_log.h"
#include "ring_battery.h"
#include "ring_bsp.h"
#include "system_clock.h"
#include "state_module_def.h"
#include "reports_def.h"

//#define  USE_SAADC_TRACE
#ifdef USE_SAADC_TRACE
#define SAADC_TRACE_LOG NRF_LOG_INFO
#define SAADC_TRACE_ERROR NRF_LOG_ERROR
#else
#define SAADC_TRACE_LOG(...)
#define SAADC_TRACE_ERROR(...)
#endif

/////////////////////////////////////SAADC LOWPWER//////////////////////////////////////////////////////////////////////////////////
#define SAADC_CALIBRATION_INTERVAL 5 //Determines how often the SAADC should be calibrated relative to NRF_DRV_SAADC_EVT_DONE event. E.g. value 5 will make the SAADC calibrate every fifth time the NRF_DRV_SAADC_EVT_DONE is received.
#define SAADC_SAMPLES_IN_BUFFER 8    //Number of SAADC samples in RAM before returning a SAADC event. For low power SAADC set this constant to 1. Otherwise the EasyDMA will be enabled for an extended time which consumes high current.
#define SAADC_BURST_MODE 1           //Set to 1 to enable BURST mode, otherwise set to 0.
#define BATT_VOL_MAX    4.5
static nrf_saadc_value_t m_buffer_pool[2][SAADC_SAMPLES_IN_BUFFER];
static nrf_saadc_value_t m_adc_buffer[SAADC_SAMPLES_IN_BUFFER];
static uint32_t m_adc_evt_counter = 0;
static bool m_saadc_initialized = false;
static bool m_saadc_pwr = false;  //   init && uninit status
uint8_t batt_full_watermark = 0;

//uint16_t chg_calibrate_diff = 0;
nrf_saadc_value_t batt_adc_value = 0;
uint8_t batt_vol_percent = 0;
float batt_vol = BATT_VOL_MAX; //max voltage set is 4.5V
static uint8_t sampletimes = 0;

static void saadc_init(void);

#define SAADC_SAMPLE_INTERVAL APP_TIMER_TICKS(10)
#define SAADC_START_INTERVAL APP_TIMER_TICKS(1000)
#define SAADC_CHARGE_WAIT_INTERVAL APP_TIMER_TICKS(5000)
#define SAADC_UNCHARGE_WAIT_INTERVAL APP_TIMER_TICKS(60000)

#define SAADC_WAIT_HANDON_LEDOFF_INTERVAL APP_TIMER_TICKS(3000, APP_TIMER_PRESCALER)

APP_TIMER_DEF(m_saadc_timer_id); /**< ble pair timer. */

static void ble_saadc_timeout_handler(void *p_context) {
    UNUSED_PARAMETER(p_context);
    uint32_t err_code;

    SAADC_TRACE_LOG("battery saadc time out...\r\n");
    if (!m_saadc_initialized) {
        saadc_init();    //Initialize the SAADC. In the case when SAADC_SAMPLES_IN_BUFFER > 1 then we only need to initialize the SAADC when the the buffer is empty.
    }

    m_saadc_initialized = true; //Set SAADC as initialized

  
    //if (!device_is_oncharger()) {
    //    if(BATTERY_VOL_5_PER > batt_vol){
    //        err_code = app_timer_start(m_saadc_timer_id, SAADC_UNCHARGE_WAIT_INTERVAL, NULL); //ADC sample interval
    //        return;
    //    }
    //}

    nrf_drv_saadc_sample();     //Trigger the SAADC SAMPLE task

    SAADC_TRACE_LOG("SAADC sampling start...\r"); //Toggle LED1 to indicate SAADC sampling start
}

/*****************************************************************************
3300 100%
2900 10%
2540 5%		
2420 2%
2340 1%
(2900 3300] y = k*x + b (y = 0.225 * x - 642.5)
(2540 2900] y = a + b*x + c * x * x (y = -1.091e-5 * x * x + 0.07325 * x - 110.7)
(2540 2900] y = a + b*x + c * x * x (y = -1.091e-5 * x * x + 0.07325 * x - 110.7)
[2340 2540] y = a + b*x + c * x * x (y = 6.25e-5 * x * x - 0.285 * x + 325.7)
*****************************************************************************/
static uint8_t convert_adc_2_percent(double v) {
  double k, b;
  if (v >= (double)ADC_MAX_SAMPLE) {
    return 100;
  } else if (v >= (double)ADC_BURST_1_SAMPLE) {
    k = 0.109;
    b = -259.62;
  } else if (v >= (double)ADC_BURST_2_SAMPLE) {
    k = 0.238;
    b = -641.5;
  } else if (v >= (double)ADC_LOWPOWER_SAMPLE) {
    k = 0.0375;
    b = -85.2;
  } else if (v >= (double)ADC_MIN_SAMPLE) {
    k = 0.0416;
    b = -95.67;
  } else {
    return 0;
  }
  // shouldn't show %0
convert_adc_2_percent_end:
  return (uint8_t)(k * v + b);
}

static uint8_t convert_vol_2_percent(float v) {
    if (v >= BATTERY_VOL_MAX) {
        return 100;
    }else if(v >= 4.1){
        float per_f = ((v - 4.1)/(BATTERY_VOL_MAX - 4.1))*0.1 + 0.9;
        return (uint8_t)(per_f * 100);
    }else if(v >= 4.05){
        float per_f = ((v - 4.05)/(4.1 - 4.05))*0.1 + 0.8;
        return (uint8_t)(per_f * 100);
    }else if(v >= 3.95){
        float per_f = ((v - 3.95)/(4.05 - 3.95))*0.1 + 0.7;
        return (uint8_t)(per_f * 100);
    }else if(v >= 3.9){
        float per_f = ((v - 3.9)/(3.95 - 3.9))*0.1 + 0.6;
        return (uint8_t)(per_f * 100);
    }else if(v >= 3.85){
        float per_f = ((v - 3.85)/(3.9 - 3.85))*0.1 + 0.5;
        return (uint8_t)(per_f * 100);
    }else if(v >= 3.8){
        float per_f = ((v - 3.8)/(3.85 - 3.8))*0.1 + 0.4;
        return (uint8_t)(per_f * 100);
    }else if(v >= 3.75){
        float per_f = ((v - 3.75)/(3.8 - 3.75))*0.1 + 0.3;
        return (uint8_t)(per_f * 100);
    }else if(v >= 3.7){
        float per_f = ((v - 3.7)/(3.75 - 3.7))*0.1 + 0.2;
        return (uint8_t)(per_f * 100);
    }else if(v >= BATTERY_VOL_10_PER){
        float per_f = ((v - BATTERY_VOL_10_PER)/(3.7 - BATTERY_VOL_10_PER))*0.1 + 0.1;
        return (uint8_t)(per_f * 100);
    }else if(v >= BATTERY_VOL_5_PER){
        float per_f = ((v - BATTERY_VOL_5_PER)/(BATTERY_VOL_10_PER - BATTERY_VOL_5_PER))*0.05 + 0.05;
        return (uint8_t)(per_f * 100);
    }else {
        return 5;
    }
  //} else if (v >= BATTERY_VOL_10_PER) {
  //  float per_f = (v - BATTERY_VOL_10_PER)/(BATTERY_VOL_MAX - BATTERY_VOL_10_PER)*0.9 + 0.1;
  //  return (uint8_t)(per_f * 100);
  //} else if (v >= BATTERY_VOL_5_PER) {
  //  return 5;
  //} else {
  //  return 1;
  //}
}


#define FULL_CHARGE_PERIOD  (90*60) //90 minute
#define ADC_BUFF_CNT    3
uint32_t adc_buff[ADC_BUFF_CNT];
uint8_t adc_buff_saved_cnt = 0;
void saadc_callback(nrf_drv_saadc_evt_t const *p_event) {
  int32_t adc_value_sum = 0;
  int32_t adc_value_mean = 0;
  int32_t adc_value_max = 0;

  if (p_event->type == NRF_DRV_SAADC_EVT_DONE) //Capture offset calibration complete event
  {
    ret_code_t err_code;

    SAADC_TRACE_LOG("SAADC buffer full\r\n"); //Toggle LED2 to indicate SAADC buffer full

    if ((m_adc_evt_counter % SAADC_CALIBRATION_INTERVAL) == 0) //Evaluate if offset calibration should be performed. Configure the SAADC_CALIBRATION_INTERVAL constant to change the calibration frequency
    {
      SAADC_TRACE_LOG("SAADC calibration starting...  \r\n"); //Print on UART
      NRF_SAADC->EVENTS_CALIBRATEDONE = 0;                    //Clear the calibration event flag
      nrf_saadc_task_trigger(NRF_SAADC_TASK_CALIBRATEOFFSET); //Trigger calibration task
      while (!NRF_SAADC->EVENTS_CALIBRATEDONE)
        ; //Wait until calibration task is completed. The calibration tasks takes about 1000us with 10us acquisition time. Configuring shorter or longer acquisition time will make the calibration take shorter or longer respectively.
      while (NRF_SAADC->STATUS == (SAADC_STATUS_STATUS_Busy << SAADC_STATUS_STATUS_Pos)) {
      }
      //Additional wait for busy flag to clear. Without this wait, calibration is actually not completed. This may take additional 100us - 300us

      SAADC_TRACE_LOG("SAADC calibration complete ! \r\n"); //Print on UART
    }

    err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAADC_SAMPLES_IN_BUFFER);
    //Set buffer so the SAADC can write to it again. This is either "buffer 1" or "buffer 2"
    APP_ERROR_CHECK(err_code);

    SAADC_TRACE_LOG("ADC event number: %d\r\n", (int)m_adc_evt_counter); //Print the event number on UART


    m_adc_buffer[sampletimes] = p_event->data.done.p_buffer[0];
    sampletimes++;

    if (sampletimes < SAADC_SAMPLES_IN_BUFFER) {
         err_code = app_timer_start(m_saadc_timer_id, SAADC_SAMPLE_INTERVAL, NULL);
         goto quit_group_sample;
    }

    int32_t adc_value_sum = 0;

    for (int i = 0; i < SAADC_SAMPLES_IN_BUFFER; i++) {
      SAADC_TRACE_LOG("%d\r",m_adc_buffer[i]); //Output SAADC samples.
      //adc_value_sum += m_adc_buffer[i];
      adc_value_max = adc_value_max > m_adc_buffer[i] ? adc_value_max : m_adc_buffer[i];
    }
    //NRF_LOG_INFO("battery adc value = %d",adc_value_sum);
    //NRF_LOG_FLUSH();
    //adc_value_mean = adc_value_sum / SAADC_SAMPLES_IN_BUFFER;

    //            ************************************************
    if (!device_is_oncharger()) // no charging
    {
        //if (0 == batt_adc_value){
        //    batt_adc_value = 3400;
        //    adc_buff_saved_cnt = 0;
        //    adc_buff[adc_buff_saved_cnt] = adc_value_max;
        //    adc_buff_saved_cnt++;
        //} else {
        //    adc_buff[adc_buff_saved_cnt] = adc_value_max;
        //    adc_buff_saved_cnt++;
        //    if(adc_buff_saved_cnt == ADC_BUFF_CNT){
        //        for(int i = 0; i++; i < ADC_BUFF_CNT){
        //            adc_value_max = adc_value_max > adc_buff[i] ? adc_value_max : adc_buff[i];
        //        }
        //        adc_buff_saved_cnt = 0;
        //        batt_adc_value = MIN(adc_value_max, batt_adc_value);
        //    }
        //}

        //batt_vol = CONVERT_ADC_2_VOL(batt_adc_value);
        //batt_vol_percent = convert_vol_2_percent(batt_vol);

        float temp_batt_vol = CONVERT_ADC_2_VOL(adc_value_max);
        if(batt_vol < BATTERY_VOL_10_PER){
            if(temp_batt_vol < batt_vol)
                batt_vol = temp_batt_vol;
        }else{
            batt_vol = temp_batt_vol;
        }

        //batt_vol = BATTERY_VOL_5_PER > batt_vol ? batt_vol : temp_batt_vol;
        batt_vol_percent = convert_vol_2_percent(batt_vol);
        batt_vol_percent = batt_vol_percent > get_NI_ram_charged_percent() ? get_NI_ram_charged_percent() : batt_vol_percent;
        //if(BATTERY_VOL_5_PER >= batt_vol){
        //    request_sys_ship();
        //}

        if(BATTERY_VOL_10_PER > batt_vol){
            switch_module(e_LP, e_report_end_by_low_power);
            //goto quit_group_sample;
        }else if(BATTERY_VOL_5_PER > batt_vol){
            switch_module(e_LP, e_report_end_by_low_power);
            goto quit_group_sample;
        }
			
//skip_normal:      
//			chg_calibrate_diff = 0;
    } else // charging
    {
        //batt_adc_value = 3400;
        //batt_vol = BATT_VOL_MAX - 0.4;
        //batt_vol_percent = 50;
        adc_buff_saved_cnt = 0;

        if(device_is_charge_full() == true){
            batt_adc_value = 3400;
            batt_vol = BATT_VOL_MAX;
            batt_vol_percent = 100;
            set_NI_ram_charged_percent(batt_vol_percent);
        }else{
            uint32_t charged_time = UTC_getClock() - get_NI_ram_charging_start_time();
            uint32_t charged_percent = charged_time*100/FULL_CHARGE_PERIOD;
            printf("now: %d, charging time: %d, duration: %d, charging percent: %d, charged percent: %d\r\n", 
                UTC_getClock(), get_NI_ram_charging_start_time(), charged_time, get_NI_ram_charging_start_percent(), charged_percent);
            batt_vol_percent = (get_NI_ram_charging_start_percent() + charged_percent) > 95 ? 95 : (get_NI_ram_charging_start_percent() + charged_percent);
            set_NI_ram_charged_percent(batt_vol_percent);
        }
      //if (0 == batt_adc_value)
      //  batt_adc_value = adc_value_max;
      //else
      //  batt_adc_value = MAX((batt_adc_value + adc_value_max) / 2, adc_value_max);
      //batt_vol_percent = convert_adc_2_percent((double)batt_adc_value);
      //if (batt_vol_percent == 100) {
      //  //ADC_CALIBRATE_MAX min, ????,?????
      //  if (chg_calibrate_diff < ADC_CALIBRATE_MAX) {
      //    chg_calibrate_diff++;
      //    batt_vol_percent = batt_vol_percent - 10 + chg_calibrate_diff * 10 / ADC_CALIBRATE_MAX;
      //  }
      //} else {
      //  batt_vol_percent -= (batt_vol_percent > 12) ? 12 : 0;
      //  chg_calibrate_diff = 0;
      //}
      
      //  batt_vol = CONVERT_ADC_2_VOL(batt_adc_value);
    }

     sampletimes = 0; //start
     if (!device_is_oncharger()) {                                       // no charging
              err_code = app_timer_start(m_saadc_timer_id, SAADC_UNCHARGE_WAIT_INTERVAL, NULL); //ADC sample interval
     } else {
              err_code = app_timer_start(m_saadc_timer_id, SAADC_CHARGE_WAIT_INTERVAL, NULL);
     }

quit_group_sample:
    NRF_LOG_ERROR("adc back: voltage: " NRF_LOG_FLOAT_MARKER ", adc:%d, percent:%d", NRF_LOG_FLOAT(batt_vol), batt_adc_value, batt_vol_percent);
    m_adc_evt_counter++;

    nrf_drv_saadc_uninit(); //Unintialize SAADC to disable EasyDMA and save power
    m_saadc_pwr = false;
    NRF_SAADC->INTENCLR = (SAADC_INTENCLR_END_Clear << SAADC_INTENCLR_END_Pos); //Disable the SAADC interrupt

#ifdef SOFTDEVICE_PRESENT
    sd_nvic_ClearPendingIRQ(SAADC_IRQn);
#else
    NVIC_ClearPendingIRQ(SAADC_IRQn); //Clear the SAADC interrupt if set
#endif

    m_saadc_initialized = false; //Set SAADC as uninitialized
  }
}


static void saadc_init(void) {
  ret_code_t err_code;

  if(m_saadc_pwr)
   return;

  nrf_saadc_input_t adc_bat_vol_pin = ADC_BAT_VOL_PIN;

  //Configure SAADC
  nrf_drv_saadc_config_t saadc_config = NRF_DRV_SAADC_DEFAULT_CONFIG;
  saadc_config.oversample = NRF_SAADC_OVERSAMPLE_4X; //Set oversample to 4x. This will make the SAADC output a single averaged value when the SAMPLE task is triggered 4 times.

//Initialize SAADC
  err_code = nrf_drv_saadc_init(&saadc_config, saadc_callback); //Initialize the SAADC with configuration and callback function. The application must then implement the saadc_callback function, which will be called when SAADC interrupt is triggered
  APP_ERROR_CHECK(err_code);

  //Configure SAADC channel
  nrf_saadc_channel_config_t channel_config =
      NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(adc_bat_vol_pin); //p0.02
                                                                //Configure SAADC channel
                                                                //	channel_config.gain = NRF_SAADC_GAIN1_4; //Set input gain to 1/4. The maximum SAADC input voltage is then 0.6V/(1/4)=2.4V. The single ended input range is then 0V-2.4V
  channel_config.gain = NRF_SAADC_GAIN1_2;                      //Set input gain to 1/4. The maximum SAADC input voltage is then 0.6V/(1/4)=2.4V. The single ended input range is then 0V-2.4V

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

  m_saadc_pwr=true;
}

/**
 * @brief Function for adc rtc run.
 */
void saadc_run(bool create_timer) {
  uint32_t err_code;
  if (create_timer) {
    //rtc_config();	//Configure RTC. The RTC will generate periodic interrupts. Requires 32kHz clock to operate.
    err_code = app_timer_create(&m_saadc_timer_id,
        APP_TIMER_MODE_SINGLE_SHOT,
        ble_saadc_timeout_handler);
    if (err_code != NRF_SUCCESS) {
      SAADC_TRACE_ERROR("battery create saadc softtimer fail\r\n");
    }
    SAADC_TRACE_LOG("battery create saadc softtimer ok\r\n");
  }

  err_code = app_timer_start(m_saadc_timer_id, SAADC_SAMPLE_INTERVAL, NULL);

  if (err_code != NRF_SUCCESS) {
    SAADC_TRACE_ERROR("start saadc softtimer fail\r\n");
  }
  SAADC_TRACE_LOG("start saadc softtimer ok\r\n");
  m_saadc_initialized = false; //Set SAADC as uninitialized
  batt_adc_value = 0;
  batt_full_watermark = 0;
}

void saadc_stop(void) {
  m_saadc_initialized = false; //Set SAADC as uninitialized
  batt_adc_value = 0;

  app_timer_stop(m_saadc_timer_id);

  if(m_saadc_pwr)
  {
  nrf_drv_saadc_abort();

  nrf_drv_saadc_uninit();

  NRF_SAADC->INTENCLR = (SAADC_INTENCLR_END_Clear << SAADC_INTENCLR_END_Pos); //Disable the SAADC interrupt

#ifdef SOFTDEVICE_PRESENT
  sd_nvic_ClearPendingIRQ(SAADC_IRQn);
#else
  NVIC_ClearPendingIRQ(SAADC_IRQn); //Clear the SAADC interrupt if set
#endif
  m_saadc_pwr = false;
  }
}