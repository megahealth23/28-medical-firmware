/*
 * Copyright (c) 2019 MegaHealth. All Rights Reserved.
 * @file ring_startup.c
 * @author:li tianzheng
 * @modifing:
 * @date 2020.10.15
 * @version
 * @brief
 */
#include "ring_startup.h"
#include "fds.h"
#include "mTask.h"
#include "ring_handonled.h"
#include "state_module_manager.h"

//sys_state_t sState = {0};

emMultiState msState = MSOFF;

extern struct acc_signal_s acc_signal;
extern struct afe_signal_s afe_signal;
extern struct tmp_signal_s tmp0_signal;
extern struct tmp_signal_s tmp1_signal;

/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
void idle_state_handle(void) {
  if (NRF_LOG_PROCESS() == false) {
    nrf_pwr_mgmt_run();
  }
}

//static void procIMU() {
//  switch (sState.mode) {
//  case IDLE:
//    nrf_delay_us((uint32_t)500);
//    acc_start(IMU_STEPS);
//    break;

//  case SPO2:
//  case RTSHOW:
//    acc_start(IMU_SPO2);
//    break;

//  case EHR:
//    acc_start(IMU_EHR);
//    break;

//  default:
//    NRF_LOG_INFO("sState.mode=%d", sState.mode);
//    break;
//  }
//  return;
//}

static void i2cTaskProc(void *pMsg) {
    uint16_t evt = *((uint16_t *)pMsg);
    uint8_t ret;
    int16_t frame_length = 0;
    struct afe4900_sensor_data *afe_bpt;
    struct tmp117_sensor_data *tmp_pt;
    enum raw_data_start_word raw_type;

    i2c_init();
    if (evt == MR_OPEN_PAIR_ISR) {
        // forbit wake isr, enable click isr.
        i2c_init();
        acc_config_Tap(true);
        i2c_uninit();
    } else if (evt == MR_PAIR_ISR_TIMEOUT) {
        i2c_init();
        acc_config_Tap(false);
        i2c_uninit();
        NRF_LOG_INFO("acc pair isr timeout\r\n");
    }
    i2c_uninit();

    state_modules_I2C_process(pMsg);
    return;
}

//extern void wdt_feed(void);
//static void PowerManager(void) {
//  wdt_feed();

//  chgchip_poll();

//  // ring_topled_on(); // Fast consuming power.

//  switch (battery_manager.dev_ps) {
//  case DEV_CHGFULL:
//    ring_chargefull_indicate();
//    // Indicate FULL
//    break;
//  case DEV_NORMAL:
//    ring_stop_indicate();
//    // mring_safeSystemAction(SAFE_RESUME);
//    // state_handle(IDLE_RUN);

//    break;

//  case DEV_CHGING:
//    ring_charging_indicate();
//    break;

//  case DEV_LOWPWR:
//    //if ( (sState.mode == SPO2) || (sState.mode == EHR))
//    //    stopMonitoring(STOP_LOWPOWER); 
		
//    //storeDailyRecord();

//    //ring_systemlowpower_entry();
//    break;

//  case DEV_SHUTDOWN:
//    ring_systemoff_entry();
//    break;

//  default:
//    break;
//  }
//  return;
//}

bool bShouldReset = false;

static uint32_t dailyCnt = 0;
static uint32_t spo2Cnt = 0;
static uint32_t spo2_clcnum = 0;
//static void sysTaskProc(void *pMsg) {
//  //UNUSED_PARAMETER(pMsg);
//  uint16_t evt = *((uint16_t*)pMsg);
//  uint8_t base_evt = (evt | 0xFF00) >> 8;
//  uint8_t sub_evt = (evt | 0x00FF);

//  state_modules_SYS_process(pMsg);
//}

#define HW_ERR (0x0f != ring_get_sensor_status())

void app_startup(void) {
    /*
    *    Every task process its queue.  Every code snippets could sent msg to tasks.
    *    Register queue of the task.
    */
    bShouldReset = false;

    taskInit();
    taskRegister(SYS_QID, state_modules_SYS_process);
    taskRegister(I2C_QID, i2cTaskProc);
    taskRegister(BLECmd_QID, proTaskProc);

    //	rawdata_notify_timer_start(false, true);
    ring_handonled_init(true, HANDONLED_DUTY_NONE);

    //ring_startup_idle();

    //LastMonitoring();

    state_modules_init();

    // Enter main loop.
    for (;;) {
        if (bShouldReset) {
            ring_systemreset_entry();
            NVIC_SystemReset();
        }
        taskProcess();

        idle_state_handle();
    }
}