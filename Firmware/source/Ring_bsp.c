/*
 * Copyright (c) 2019 MegaHealth. All Rights Reserved.
 * @file
 * @date 2020.07.29
 * @version
 * @brief
 */
#include "boards.h"
#if defined(BOARDS_WITH_USB_DFU_TRIGGER) && defined(BOARD_PCA10059)
#include "nrf_dfu_trigger_usb.h"
#endif

#include "app_error.h"
#include <nrf_drv_gpiote.h>
#include <nrfx_gpiote.h>

#include "Ring_bsp.h"
#include "mTask.h"
#include "system_clock.h"

#include "nrf_log.h"

#include "app_timer.h"
#include "ring_acc.h"
#include "ring_afe.h"

extern struct acc_signal_s acc_signal;
extern struct afe_signal_s afe_signal;

static bool acc_int1_status = false;
static bool acc_int2_status = false;

static bool bsp_timer_start(void);
#define BSP_TIME_MS 2
APP_TIMER_DEF(m_bsp_timer_id);

static bool m_bsp_timer_create = false;
static bool m_bsp_timer_run = false;

/*********************************************************************
 * @fn      accInt1Handler
 *
 * @brief   gpio change interupt.
 *
 * @param   None.
 *
 * @return  None.
 */
static void accInt1Handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
    nrf_drv_gpiote_pin_t accint1_pin = ACC_INT1_PIN;

    //>>>>>>>>>>>>>>>>>>>>   M1    >>>>>>>>>>>>>>>>>>>>>
    //	static uint16_t acc_int1_read = ACC_INT1SRC_READ;

    //>>>>>>>>>>>>>>>>>>>>   M2    >>>>>>>>>>>>>>>>>>>>>
    acc_signal.int1_flag = (MAJOR_DEV_ACC | OP_READ_INT_STATUS | SET_OP_PARM(1));
    if (pin == accint1_pin) {
        NRF_LOG_INFO("accInt1Handler ISR");
        //>>>>>>>>>>>>>>>>>>>>   M1    >>>>>>>>>>>>>>>>>>>>>
        //		sEvent evt = { I2C_QID, &acc_int1_read };
        //		taskPush(&evt, false);

        //>>>>>>>>>>>>>>>>>>>>   M2    >>>>>>>>>>>>>>>>>>>>>
        sEvent tx_evt = {I2C_QID, &acc_signal.int1_flag};
        taskPush(&tx_evt, false);
    }

    return;
}

/*********************************************************************
 * @fn      acc_int2_handler
 *
 * @brief   gpio change interupt.
 *
 * @param   None.
 *
 * @return  None.
 */
static void accInt2Handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
    nrf_drv_gpiote_pin_t accint2_pin = ACC_INT2_PIN;

    //>>>>>>>>>>>>>>>>>>>>   M1    >>>>>>>>>>>>>>>>>>>>>
    //	static uint16_t acc_int2_read = ACC_INT2SRC_READ;

    //>>>>>>>>>>>>>>>>>>>>   M2    >>>>>>>>>>>>>>>>>>>>>
    acc_signal.int2_flag = (MAJOR_DEV_ACC | OP_READ_INT_STATUS | SET_OP_PARM(2));
    if (pin == accint2_pin) {
        NRF_LOG_INFO("%u s, ACCINT2 ISR", app_get_sec());

        //>>>>>>>>>>>>>>>>>>>>   M1    >>>>>>>>>>>>>>>>>>>>>
        //		sEvent evt = { I2C_QID, &acc_int2_read };
        //		taskPush(&evt, false);

        //>>>>>>>>>>>>>>>>>>>>   M2    >>>>>>>>>>>>>>>>>>>>>
        sEvent tx_evt = {I2C_QID, &acc_signal.int2_flag};
        taskPush(&tx_evt, false);
    }

    return;
}

/*********************************************************************
 * @fn      acc_int2_handler
 *
 * @brief   gpio change interupt.
 *
 * @param   None.
 *
 * @return  None.
 */
#include "ring_afe.h"
//#include "ring_swtimer.h"
static void afeFifoRdyHandler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
  static uint32_t sec_bas = 0;

  NRF_LOG_INFO("aaaaaaaaaaaaaaaaaaaaaaa");
  nrf_drv_gpiote_pin_t adcready_pin = AFE_ADCREADY_PIN;

  if (sec_bas != app_get_sec()) {
    sec_bas = app_get_sec();
  }
  //>>>>>>>>>>>>>>>>>>>>   M1    >>>>>>>>>>>>>>>>>>>>>
  //	static uint16_t afe_fifordy_read = AFE_FIFORDY_READ;

  if (pin == adcready_pin) {
    //				NRF_LOG_INFO("%ds, AFE_FIFORDY ISR", app_get_sec());

    //>>>>>>>>>>>>>>>>>>>>   M1    >>>>>>>>>>>>>>>>>>>>>
    //				sEvent evt = { I2C_QID, &afe_fifordy_read };
    //				taskPush(&evt, false);

    //>>>>>>>>>>>>>>>>>>>>   M2    >>>>>>>>>>>>>>>>>>>>>
    afe_signal.read = (MAJOR_DEV_AFE | OP_READ_SENSOR_DATA);
    sEvent tx_evt = {I2C_QID, &afe_signal.read};
    taskPush(&tx_evt, false);

    //>>>>>>>>>>>>>>>>>>>>   M2    >>>>>>>>>>>>>>>>>>>>>
    //					bsp_timer_start();
  }

  return;
}

uint32_t boardInit(void) {
  uint32_t err_code = NRF_SUCCESS;

#if defined(BOARDS_WITH_USB_DFU_TRIGGER) && defined(BOARD_PCA10059)
  (void)nrf_dfu_trigger_usb_init();
#endif

  if (!nrfx_gpiote_is_init()) {
    err_code = nrfx_gpiote_init();
    APP_ERROR_CHECK(err_code);
  }

  return err_code;
}

uint32_t accInt1init(void) {
  uint32_t err_code;

  if (true == acc_int1_status)
    return true;

  nrf_drv_gpiote_pin_t accint1_pin = ACC_INT1_PIN;

  nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
  // nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
  // nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
  in_config.pull = NRF_GPIO_PIN_NOPULL;

  err_code = nrf_drv_gpiote_in_init(accint1_pin, &in_config, accInt1Handler);

  APP_ERROR_CHECK(err_code);
  if (err_code != NRF_SUCCESS) {
    NRF_LOG_ERROR("acc int1 pin config fail\r\n");
    return err_code;
  }
  NRF_LOG_INFO("acc int1 charge pin config ok\r\n");

  nrf_drv_gpiote_in_event_enable(accint1_pin, true);

  acc_int1_status = true;

  return NRF_SUCCESS;
}

bool accInt1deinit(void) {
  nrf_drv_gpiote_pin_t accint1_pin = ACC_INT1_PIN;

  if (false == acc_int1_status)
    return true;

  nrf_drv_gpiote_in_uninit(accint1_pin);

  acc_int1_status = false;
  return true;
}

uint32_t accInt2init(void) {
  uint32_t err_code;

  if (true == acc_int2_status)
    return true;

  nrf_drv_gpiote_pin_t accint2_pin = ACC_INT2_PIN;

  // DOUBLE EDGE
  nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
  in_config.pull = NRF_GPIO_PIN_NOPULL;

  err_code = nrf_drv_gpiote_in_init(accint2_pin, &in_config, accInt2Handler);

  APP_ERROR_CHECK(err_code);
  if (err_code != NRF_SUCCESS) {
    NRF_LOG_ERROR("acc int2 pin config fail\r\n");
    return err_code;
  }
  NRF_LOG_INFO("acc int2 charge pin config ok\r\n");

  nrf_drv_gpiote_in_event_enable(accint2_pin, true);

  acc_int2_status = true;

  return NRF_SUCCESS;
}

bool accInt2deinit(void) {
  if (false == acc_int2_status)
    return true;

  nrf_drv_gpiote_pin_t accint2_pin = ACC_INT2_PIN;

  nrf_drv_gpiote_in_uninit(accint2_pin);

  acc_int2_status = false;
  return true;
}

uint32_t afeFifoRdyinit(void) {
  uint32_t err_code;

  nrf_drv_gpiote_pin_t adcready_pin = AFE_ADCREADY_PIN;

  // DOUBLE EDGE
  nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
  in_config.pull = NRF_GPIO_PIN_NOPULL;

  err_code = nrf_drv_gpiote_in_init(adcready_pin, &in_config, afeFifoRdyHandler);

  APP_ERROR_CHECK(err_code);
  if (err_code != NRF_SUCCESS) {
    NRF_LOG_ERROR("afe ready pin config fail\r\n");
    return err_code;
  }
  NRF_LOG_INFO("afe ready pin config ok\r\n");

  nrf_drv_gpiote_in_event_enable(adcready_pin, true);

  return NRF_SUCCESS;
}

static void bsp_timer_handler(void *p_context) {
  afe_signal.read = (MAJOR_DEV_AFE | OP_READ_SENSOR_DATA);
  sEvent tx_evt = {I2C_QID, &afe_signal.read};
  taskPush(&tx_evt, false);
  m_bsp_timer_run = false;
}

static bool bsp_timer_start(void) {
  uint32_t err_code;

  if (!m_bsp_timer_create) {
    err_code = app_timer_create(&m_bsp_timer_id,
        APP_TIMER_MODE_SINGLE_SHOT,
        bsp_timer_handler);

    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("Create accel timer ok\r\n");
    m_bsp_timer_create = true;
  }

  if (!m_bsp_timer_run) {
    err_code = app_timer_start(m_bsp_timer_id, APP_TIMER_TICKS(BSP_TIME_MS), NULL);
    if (err_code != NRF_SUCCESS) {
      NRF_LOG_ERROR("start accel timer fail\r\n");
      return false;
    }
    m_bsp_timer_run = true;
  }

  return true;
}
