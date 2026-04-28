/*
 * Copyright (c) 2019 MegaHealth. All Rights Reserved.
 * @file ring_acc.c
 * @author:
 * @modifing:li tianzheng
 * @date 2020.07.29
 * @version
 * @brief
 */

#include "ring_acc.h"
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
#include "inner_flash_ram.h"

struct acc_signal_s acc_signal;

static struct ring_acc_s module_acc =
    {
        .dev_info =
            {
                .smp_data_used = ACC_SAMPLE_NONEDATA,
                .i2c_line = ACC_DEV_NONE,
                .int1_line = ACC_DEV_NONE,
                .int2_line = ACC_DEV_NONE,
                .smp_period = ACC_DOWNSAMPLE_NONE,
                .smp_times = 0,
            },
};

static bool AccTapIsrEnable = false;
static bool AccWakeIsrEnable = false;

static bool m_acc_pwr = false;
static bool m_accel_timer_create = false;
static bool m_accel_timer_run = false;

struct lis2dw12_sensor_data accData[ACC_FIFOLEN]; // 100Hz accl.

/* Declare memory to store the raw FIFO buffer information */
#define FIFO_LEN_BYTES ACC_FIFOLEN * 7
uint8_t fifo_buff[FIFO_LEN_BYTES];

stmdev_ctx_t dev_ctx;

lis2dw12_sensor_t sensor = {
   .enable = PROPERTY_ENABLE,
   .odr = LIS2DW12_XL_ODR_100Hz,		
   .mode = LIS2DW12_CONT_LOW_PWR_4,	
   .fs = LIS2DW12_4g,					
};

static ring_acc_mode_t curMode = IMU_POWEROFF;

bool acc_start_active_check_mode(uint8_t odr);

static bool acc_pin_deconfig(void) // AnyMotionIntCfg
{
  lis2dw12_reg_t int_route;

  lis2dw12_pin_int1_route_get(&dev_ctx, &int_route.ctrl4_int1_pad_ctrl);
  int_route.ctrl4_int1_pad_ctrl.int1_wu = PROPERTY_DISABLE;
  int_route.ctrl4_int1_pad_ctrl.int1_single_tap = PROPERTY_DISABLE;
  lis2dw12_pin_int1_route_set(&dev_ctx, &int_route.ctrl4_int1_pad_ctrl);

  return true;
}

int8_t lis2dw12I2Cwrite(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
    int8_t rslt = 0;
    uint8_t writebuf[4];

    UNUSED_PARAMETER(handle);

    if (len > 3) {
        return -1;
    }

    writebuf[0] = reg;		//重要
    memcpy(writebuf+1, bufp, len);

    rslt = i2c_write(LIS2DW12_I2C_ADDR, reg, writebuf, (uint8_t)len+1);
    return rslt;
}

int8_t lis2dw12I2Cread(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
    ret_code_t rslt;

    UNUSED_PARAMETER(handle);

    rslt = i2c_read(LIS2DW12_I2C_ADDR, reg, bufp, len);
    return (int8_t)rslt;
}

bool lis2dw12_init() {
    uint8_t whoami;
    uint32_t reset_times;

    dev_ctx.write_reg = lis2dw12I2Cwrite;
    dev_ctx.read_reg = lis2dw12I2Cread;
    dev_ctx.handle = NULL;

    int8_t rslt = 0;

    lis2dw12_device_id_get(&dev_ctx, &whoami);
    lis2dw12_reset_set(&dev_ctx, PROPERTY_ENABLE);
    nrf_delay_ms(5);
    lis2dw12_reset_get(&dev_ctx, &rslt);

    if ((0 == rslt) && (LIS2DW12_ID == whoami))
        module_acc.dev_info.i2c_line = ACC_DEV_OK;
    // else
    //   return false;

    NRF_LOG_INFO("lis2dw12_init = %d,  rslt=%d", whoami, rslt);
    return true;
}

void lis2dw12_unInit() {
    dev_ctx.write_reg = NULL;
    dev_ctx.read_reg = NULL;
    dev_ctx.handle = NULL;
}

static int8_t lis2dw12_set_sens_conf(void)
{
    int8_t rslt = 0;

    lis2dw12_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
    lis2dw12_fifo_mode_set(&dev_ctx, LIS2DW12_BYPASS_MODE);
	
    if(-1 == lis2dw12_power_mode_set(&dev_ctx, sensor.mode))
      return -1;
    if(-1 == lis2dw12_full_scale_set(&dev_ctx, sensor.fs)) 
     return -1;
    if(-1 == lis2dw12_data_rate_set(&dev_ctx, sensor.odr)) 
      return -1;
    if(-1 == lis2dw12_all_on_int1_set(&dev_ctx, PROPERTY_ENABLE))
      return -1;
    if(-1 == lis2dw12_reference_mode_set(&dev_ctx, PROPERTY_DISABLE))
      return -1;
    if(-1 == lis2dw12_filter_path_set(&dev_ctx, LIS2DW12_LPF_ON_OUT))
      return -1;
    //if(-1 == lis2dw12_filter_bandwidth_set(&dev_ctx, LIS2DW12_ODR_DIV_20))
    if(-1 == lis2dw12_filter_bandwidth_set(&dev_ctx, LIS2DW12_ODR_DIV_10))
      return -1;
    //if(-1 == lis2dw12_pin_mode_set(&dev_ctx, LIS2DW12_OPEN_DRAIN))
    //  return -1;
    //if(-1 == lis2dw12_pin_polarity_set(&dev_ctx, LIS2DW12_ACTIVE_LOW))
    //  return -1;

    // lis2dw12_filter_bandwidth_set(&dev_ctx, LIS2DW12_ODR_DIV_4);

    acc_start_active_check_mode(sensor.odr);

    return 0;
}

static bool acc_set_mode(ring_acc_mode_t mode) {
    int8_t rslt = 0;
    if(curMode == mode)
        return true;

    NRF_LOG_INFO("acc_set_mode=%d", mode);
    i2c_init();
    acc_reset();
    nrf_delay_ms(1);

    switch (mode) {
    case IMU_POWEROFF:
        sensor.enable = PROPERTY_DISABLE;
        sensor.mode = LIS2DW12_CONT_LOW_PWR_12bit;
        sensor.odr = LIS2DW12_XL_ODR_OFF;
        acc_pin_deconfig(); // AnyMotionIntCfg
        NRF_LOG_INFO("SET IMUPOWEROFF\r");
    break;
    case IMU_STEPS:
        NRF_LOG_INFO("SET IMU_STEPS\r");
        sensor.enable = PROPERTY_ENABLE;
        sensor.mode = LIS2DW12_CONT_LOW_PWR_LOW_NOISE_3;
        sensor.odr = LIS2DW12_XL_ODR_50Hz;
    break;
    case IMU_SPO2:
        sensor.enable = PROPERTY_ENABLE;
        sensor.odr = LIS2DW12_XL_ODR_12Hz5;
        sensor.mode = LIS2DW12_CONT_LOW_PWR_4;
    break;
    case IMU_EHR:
        sensor.enable = PROPERTY_ENABLE;
        sensor.odr = LIS2DW12_XL_ODR_100Hz;
        sensor.mode = LIS2DW12_CONT_LOW_PWR_4;
    break;
    case IMU_STANDBY:
        NRF_LOG_INFO("SET IMU_STANDBY ! \r");
        sensor.enable = PROPERTY_ENABLE;
        sensor.odr = LIS2DW12_XL_ODR_12Hz5;
        sensor.mode = LIS2DW12_CONT_LOW_PWR_LOW_NOISE_12bit;
    break;
    default:
        return false; // Not valid mode.
    }

    rslt = lis2dw12_set_sens_conf();
    if (rslt != 0) {
        //  	NVIC_SystemReset();  For test.
        NRF_LOG_INFO("lis2dw12_set_sens_conf=%d", rslt);
        return false;
    }

#if 0
	struct bmi160_pmu_status pmuStatus;
	bmi160_get_power_mode(&pmuStatus, &sensor);
	NRF_LOG_INFO("accel power:%d, gyro power:%d\r\n", pmuStatus.accel_pmu_status, pmuStatus.gyro_pmu_status);
	NRF_LOG_INFO("SensorFIFO header=%02X \t data=%02X \t accelStart=%02X \t skippedframe=%02X",
		sensor.fifo->fifo_header_enable, sensor.fifo->fifo_data_enable,
		sensor.fifo->accel_byte_start_idx, sensor.fifo->skipped_frame_count);
#endif
  if (rslt == 0) // success
  {
    if (mode >= IMU_STANDBY) {
      if (AccTapIsrEnable) {
        NRF_LOG_INFO("acc_config_Tap !!!\n");
        acc_config_Tap(true);
      }
    }
	curMode = mode;
  }

    //if(mode == IMU_STEPS){
    //    lis2dw12_act_mode_set(&dev_ctx, LIS2DW12_DETECT_ACT_INACT);
    //    lis2dw12_wkup_dur_set(&dev_ctx, 3);
    //    lis2dw12_wkup_threshold_set(&dev_ctx, 3);
    //    lis2dw12_act_sleep_dur_set(&dev_ctx, 2);
    //}else{
    //    lis2dw12_act_mode_set(&dev_ctx, LIS2DW12_NO_DETECTION);
    //}

    i2c_uninit();
    i2c_init();
    rslt = lis2dw12_fifo_watermark_set(&dev_ctx, 31);
    NRF_LOG_INFO("lis2dw12_fifo_watermark_set=%d", rslt);
    lis2dw12_ctrl5_int2_pad_ctrl_t int2_pad_t;
    lis2dw12_pin_int2_route_get(&dev_ctx, &int2_pad_t);
    //NRF_LOG_INFO("lis2dw12_pin_int2_route_get is int2_drdy:%d", int2_pad_t.int2_drdy);
    //NRF_LOG_INFO("lis2dw12_pin_int2_route_get is int2_fth:%d", int2_pad_t.int2_fth);
    //NRF_LOG_INFO("lis2dw12_pin_int2_route_get is int2_diff5:%d", int2_pad_t.int2_diff5);
    //NRF_LOG_INFO("lis2dw12_pin_int2_route_get is int2_ovr:%d", int2_pad_t.int2_ovr);
    //NRF_LOG_INFO("lis2dw12_pin_int2_route_get is int2_drdy_t:%d", int2_pad_t.int2_drdy_t);
    //NRF_LOG_INFO("lis2dw12_pin_int2_route_get is int2_boot:%d", int2_pad_t.int2_boot);
    //NRF_LOG_INFO("lis2dw12_pin_int2_route_get is int2_sleep_chg:%d", int2_pad_t.int2_sleep_chg);
    //NRF_LOG_INFO("lis2dw12_pin_int2_route_get is int2_sleep_state:%d", int2_pad_t.int2_sleep_state);
    int2_pad_t.int2_fth = 1;
    lis2dw12_pin_int2_route_set(&dev_ctx,&int2_pad_t);
    //lis2dw12_pin_int2_route_get(&dev_ctx, &int2_pad_t);
    //NRF_LOG_INFO("lis2dw12_pin_int2_route_get is int2_drdy:%d", int2_pad_t.int2_drdy);
    //NRF_LOG_INFO("lis2dw12_pin_int2_route_get is int2_fth:%d", int2_pad_t.int2_fth);
    //NRF_LOG_INFO("lis2dw12_pin_int2_route_get is int2_diff5:%d", int2_pad_t.int2_diff5);
    //NRF_LOG_INFO("lis2dw12_pin_int2_route_get is int2_ovr:%d", int2_pad_t.int2_ovr);
    //NRF_LOG_INFO("lis2dw12_pin_int2_route_get is int2_drdy_t:%d", int2_pad_t.int2_drdy_t);
    //NRF_LOG_INFO("lis2dw12_pin_int2_route_get is int2_boot:%d", int2_pad_t.int2_boot);
    //NRF_LOG_INFO("lis2dw12_pin_int2_route_get is int2_sleep_chg:%d", int2_pad_t.int2_sleep_chg);
    //NRF_LOG_INFO("lis2dw12_pin_int2_route_get is int2_sleep_state:%d", int2_pad_t.int2_sleep_state);
    //lis2dw12_ctrl4_int1_pad_ctrl_t int1_pad_t;
    //int1_pad_t.int1_fth = 1;
    //lis2dw12_pin_int1_route_set(&dev_ctx,&int1_pad_t);
    uint8_t rr = lis2dw12_fifo_mode_set(&dev_ctx, LIS2DW12_STREAM_MODE);
    NRF_LOG_INFO("lis2dw12_fifo_mode_set=%d", rr);
    i2c_uninit();

    rslt |= rr;
  if (rslt != 0) {
    NRF_LOG_INFO("lis2dw12_fifo_mode_set=%d", rslt);
    return false;
  }

  if (IMU_STANDBY == mode)
    NRF_LOG_INFO("IMU_STANDBY AGAIN !!!\n");
  // NRF_LOG_INFO("SET mode=%d \n",mode);

  return true;
}

//static void acc_timer_stop(void) {
//  uint32_t err_code;

//  if (m_accel_timer_run) {
//    err_code = app_timer_stop(m_accel_timer_id);
//    if (err_code != NRF_SUCCESS) {
//      NRF_LOG_ERROR("stop accel timer fail\r\n");
//      return;
//    }
//    m_accel_timer_run = false;
//  }
//}

ring_acc_mode_t acc_get_work_mode(void) {
  return curMode;
}

bool acc_reset(void) {
  int8_t rslt = 0;
  rslt = lis2dw12_reset_set(&dev_ctx, PROPERTY_ENABLE);
  if (rslt != 0)
    NRF_LOG_INFO("acc_reset rslt=%d", rslt);
  return rslt;
}

bool acc_config_Tap(bool enable) // AnyMotionIntCfg
{
  bool enableMotion = enable;
  lis2dw12_reg_t int_route;

  int8_t rslt = 0;

  i2c_init();

  if (!enableMotion) {
    //accInt1deinit();
    //accInt2deinit();
  }

  lis2dw12_data_rate_set(&dev_ctx, LIS2DW12_XL_ODR_200Hz);
  nrf_delay_ms(4);
  lis2dw12_data_rate_set(&dev_ctx, enable? LIS2DW12_XL_ODR_12Hz5: sensor.odr);

  /* Enable Tap detection on X, Y, Z */
  lis2dw12_tap_detection_on_z_set(&dev_ctx, enable? PROPERTY_ENABLE: PROPERTY_DISABLE);
  lis2dw12_tap_detection_on_y_set(&dev_ctx, enable? PROPERTY_ENABLE: PROPERTY_DISABLE);
  lis2dw12_tap_detection_on_x_set(&dev_ctx, enable? PROPERTY_ENABLE: PROPERTY_DISABLE);
  /* Set Tap threshold on all axis */
  lis2dw12_tap_threshold_x_set(&dev_ctx, 4);
  lis2dw12_tap_threshold_y_set(&dev_ctx, 4);
  lis2dw12_tap_threshold_z_set(&dev_ctx, 4);
  /* Configure Single Tap parameter */
  lis2dw12_tap_quiet_set(&dev_ctx, 1);
  lis2dw12_tap_shock_set(&dev_ctx, 2);
  /* Enable Single Tap detection only */
  lis2dw12_tap_mode_set(&dev_ctx, LIS2DW12_ONLY_SINGLE);
  /* Enable single tap detection interrupt */
  lis2dw12_pin_int1_route_get(&dev_ctx, &int_route.ctrl4_int1_pad_ctrl);
  int_route.ctrl4_int1_pad_ctrl.int1_single_tap = enable? PROPERTY_ENABLE: PROPERTY_DISABLE;
  lis2dw12_pin_int1_route_set(&dev_ctx, &int_route.ctrl4_int1_pad_ctrl);

  NRF_LOG_INFO("lis2dw12_set_Tap_config=%d", rslt);

  i2c_uninit();

  if (enableMotion) {
    accInt1init();
    accInt2init();
  }

  AccTapIsrEnable = enable;
  return true;
}

bool acc_start_active_check_mode(uint8_t odr){
    NRF_LOG_INFO("acc_start_active_check_mode!!");

    i2c_init();
    lis2dw12_reg_t int_route;
    accInt1init();
    accInt2init();

    uint8_t wkup_dur = 3;
    uint8_t wkup_threshold = 1;
    uint8_t sleep_dur = 1;


    switch(odr){
        case LIS2DW12_XL_ODR_100Hz:
            sleep_dur = 2;  //10 s
        break;
        case LIS2DW12_XL_ODR_50Hz:
            sleep_dur = 1;  //10 s
        break;
        case LIS2DW12_XL_ODR_12Hz5:
            sleep_dur = 1;  //40 s
        break;
        default:
        break;
    }

    //nrf_delay_ms(1000);
    lis2dw12_act_mode_set(&dev_ctx, LIS2DW12_DETECT_ACT_INACT);
    lis2dw12_wkup_dur_set(&dev_ctx, wkup_dur);
    lis2dw12_wkup_threshold_set(&dev_ctx, wkup_threshold);
    lis2dw12_act_sleep_dur_set(&dev_ctx, sleep_dur);

    
    lis2dw12_pin_int2_route_get(&dev_ctx, &int_route.ctrl5_int2_pad_ctrl);
    int_route.ctrl5_int2_pad_ctrl.int2_sleep_chg = PROPERTY_ENABLE;
    //int_route.ctrl5_int2_pad_ctrl.int2_sleep_state = PROPERTY_ENABLE;
    lis2dw12_pin_int2_route_set(&dev_ctx, &int_route.ctrl5_int2_pad_ctrl);

    //lis2dw12_pin_int1_route_get(&dev_ctx, &int_route.ctrl4_int1_pad_ctrl);
    //int_route.ctrl4_int1_pad_ctrl.int1_wu = PROPERTY_ENABLE;
    ////int_route.ctrl5_int2_pad_ctrl.int2_sleep_state = PROPERTY_ENABLE;
    //lis2dw12_pin_int1_route_set(&dev_ctx, &int_route.ctrl4_int1_pad_ctrl);

    i2c_uninit();
    return true;
}

void acc_set_sleep_int(bool is_open){
    lis2dw12_reg_t int_route;
    lis2dw12_pin_int2_route_get(&dev_ctx, &int_route.ctrl5_int2_pad_ctrl);
    int_route.ctrl5_int2_pad_ctrl.int2_sleep_chg = (is_open == true ? PROPERTY_ENABLE : PROPERTY_DISABLE);
    //int_route.ctrl5_int2_pad_ctrl.int2_sleep_state = PROPERTY_ENABLE;
    lis2dw12_pin_int2_route_set(&dev_ctx, &int_route.ctrl5_int2_pad_ctrl);
}

void acc_set_wake_int(bool is_open){
    lis2dw12_reg_t int_route;
    lis2dw12_pin_int1_route_get(&dev_ctx, &int_route.ctrl4_int1_pad_ctrl);
    int_route.ctrl4_int1_pad_ctrl.int1_wu = (is_open == true ? PROPERTY_ENABLE : PROPERTY_DISABLE);
    //int_route.ctrl5_int2_pad_ctrl.int2_sleep_state = PROPERTY_ENABLE;
    lis2dw12_pin_int1_route_set(&dev_ctx, &int_route.ctrl4_int1_pad_ctrl);
}

bool acc_stop_active_check_mode(){
    NRF_LOG_INFO("acc_stop_active_check_mode!!");

    i2c_init();
    lis2dw12_reg_t int_route;
    lis2dw12_act_mode_set(&dev_ctx, LIS2DW12_NO_DETECTION);
    /* Disable interrupt generation on Active INT1 pin */
    lis2dw12_pin_int2_route_get(&dev_ctx, &int_route.ctrl5_int2_pad_ctrl);
    int_route.ctrl5_int2_pad_ctrl.int2_sleep_chg = PROPERTY_DISABLE;
    //int_route.ctrl5_int2_pad_ctrl.int2_sleep_state = PROPERTY_DISABLE;
    lis2dw12_pin_int2_route_set(&dev_ctx, &int_route.ctrl5_int2_pad_ctrl);
    i2c_uninit();
    return true;
}

bool acc_config_wake_int(bool enable, uint8_t value) {

  bool enableMotion = enable;
  lis2dw12_reg_t int_route;

  int8_t rslt = 0;

  i2c_init();

  if (!enableMotion) {
    //accInt1deinit();
    //accInt2deinit();
  }

  /* Set Output Data Rate */
  lis2dw12_data_rate_set(&dev_ctx, LIS2DW12_XL_ODR_50Hz);
  nrf_delay_ms(4);
  lis2dw12_data_rate_set(&dev_ctx, LIS2DW12_XL_ODR_12Hz5);
  
  /* Apply high-pass digital filter on Wake-Up function */
  lis2dw12_filter_path_set(&dev_ctx, LIS2DW12_HIGH_PASS_ON_OUT);
  /* Apply high-pass digital filter on Wake-Up function
   * Duration time is set to zero so Wake-Up interrupt signal
   * is generated for each X,Y,Z filtered data exceeding the
   * configured threshold
   */
  lis2dw12_wkup_dur_set(&dev_ctx, 0);
  /* Set wake-up threshold
   * Set Wake-Up threshold: 1 LSb corresponds to FS_XL/2^6
   */
  lis2dw12_wkup_threshold_set(&dev_ctx, (uint8_t)(value/3));
  /* Enable interrupt generation on Wake-Up INT1 pin */
  lis2dw12_pin_int1_route_get(&dev_ctx, &int_route.ctrl4_int1_pad_ctrl);
  int_route.ctrl4_int1_pad_ctrl.int1_wu =  enable? PROPERTY_ENABLE: PROPERTY_DISABLE;
  lis2dw12_pin_int1_route_set(&dev_ctx, &int_route.ctrl4_int1_pad_ctrl);

  NRF_LOG_INFO("acc_config_wake_int=%d", rslt);

  i2c_uninit();

  if (enableMotion) {
    accInt1init();
    accInt2init();
  } 

  AccWakeIsrEnable = enable;
  return true;
}

bool acc_wake_int_status(void) {
  return AccWakeIsrEnable;
}

bool acc_power_down(void) {
  return acc_set_mode(IMU_POWEROFF);
}

bool acc_enable(void) {
  return acc_set_mode(IMU_STANDBY);
}

bool acc_start(ring_acc_mode_t mode) {
    bool ret;

    if(m_acc_pwr && (mode == curMode) )
        return true;

    m_acc_pwr = false;
    nrf_delay_us(100);

    i2c_init();

    if (!lis2dw12_init()) {
        i2c_uninit();
        NRF_LOG_INFO("lis2dw12_init() failed!");
        return false;
    }

    if (!acc_set_mode(mode)) {
        i2c_uninit();
        NRF_LOG_INFO("acc_set_mode() mode=%d failed!", mode);
        return false;
    }

    accInt1init();
    accInt2init();

    //lis2dw12_auto_increment_set(&dev_ctx, 1);

    if (mode > IMU_STANDBY) {
    }

    i2c_uninit();

    m_acc_pwr = true;
    return true;
}

bool acc_stop(void) {
    bool rslt;

    if(!m_acc_pwr && ((IMU_STANDBY == curMode)||(IMU_POWEROFF == curMode)))
        return true;

    if (!i2c_init()) {
        i2c_uninit();
        return false;
    }

    accInt1deinit();
    accInt2deinit();
    acc_reset();
    nrf_delay_ms(1);
    if (!lis2dw12_init()){
        i2c_uninit();
        return false;
    }

    rslt = acc_set_mode(IMU_STANDBY);
    if (!rslt) {
        i2c_uninit();
        return false;
    }

    i2c_uninit();

    NRF_LOG_INFO("acc acc_stop! \n");
    m_acc_pwr = false;

    return true;
}

extern void SpO2_Acc_Proc(acc_axis *);
extern void EHR_Acc_Proc(acc_axis *);

static void AccProc(acc_axis *pAx, E_ACC_PROC statu) {
  if (statu == e_acc_SPO2)
    SpO2_Acc_Proc(pAx);
  else {
    if (statu == e_acc_EHR) {
      EHR_Acc_Proc(pAx);
    }
    stepsnum_update(pAx, statu);
  }

  // NRF_LOG_INFO("AccProc x=%d, y=%d, z=%d\n ", pAx->x,pAx->y,pAx->z);

  return;
}
uint32_t g_ACC_num = 0;
void acc_set_hold_data(uint8_t count, E_ACC_PROC statu){
    for (int idx = 0; idx < count; idx++) {
        acc_axis acc;

        acc.x = 980;
        acc.y = 0;
        acc.z = 0;
        AccProc(&acc, statu);
    }
    g_ACC_num = g_ACC_num + count;
}

static uint8_t accFifoErrCnt = 0;
static uint8_t accLenErrCnt = 0;
#define ACC_ERR_MAX 100

accFifo_t ax = {0};
/*******************************************************************************
 * Function Name  : read_axis
 * Description    : Read ALL samples stored in FIFO
 * Input          : Byte to empty by FIFO unread sample value
 * Output         : None
 * Return         : Status [value of FSS]
 *******************************************************************************/
E_ACC_PROC g_ACC_statu = e_acc_EHR;
int8_t read_axis(E_ACC_PROC statu) {
    int8_t rslt = 0;
    uint8_t accLen = ACC_FIFOLEN;
    int idx = 0;

    g_ACC_statu = statu;
    uint8_t wtm, fifo_len = 0;
    lis2dw12_ctrl5_int2_pad_ctrl_t val = {0};

    accLen = 0;
    /* Read output only if new value is available */
    //lis2dw12_pin_int2_route_get(&dev_ctx, &val);
    //  NRF_LOG_INFO("lis2dw12_pin_int2_route_get is int2_drdy:%d", val.int2_drdy);
    //  NRF_LOG_INFO("lis2dw12_pin_int2_route_get is int2_fth:%d", val.int2_fth);
    //  NRF_LOG_INFO("lis2dw12_pin_int2_route_get is int2_diff5:%d", val.int2_diff5);
    //  NRF_LOG_INFO("lis2dw12_pin_int2_route_get is int2_ovr:%d", val.int2_ovr);
    //  NRF_LOG_INFO("lis2dw12_pin_int2_route_get is int2_drdy_t:%d", val.int2_drdy_t);
    //  NRF_LOG_INFO("lis2dw12_pin_int2_route_get is int2_boot:%d", val.int2_boot);
    //  NRF_LOG_INFO("lis2dw12_pin_int2_route_get is int2_sleep_chg:%d", val.int2_sleep_chg);
    //  NRF_LOG_INFO("lis2dw12_pin_int2_route_get is int2_sleep_state:%d", val.int2_sleep_state);
    lis2dw12_fifo_wtm_flag_get(&dev_ctx, &wtm);
    NRF_LOG_INFO("lis2dw12_fifo_wtm_flag_get is %d", wtm);
    lis2dw12_auto_increment_set(&dev_ctx, 1);
    lis2dw12_fifo_data_level_get(&dev_ctx, &fifo_len);
    if(fifo_len > 32){
        fifo_len = 32;
    }
    //fifo_len = 32;

    if(fifo_len > 0){
        static int8_t data_raw_acceleration[6*32];
        uint8_t ret = lis2dw12_acceleration_raw_get_1(&dev_ctx, data_raw_acceleration, 6*fifo_len);
        NRF_LOG_INFO("ret is %d", ret);
        for (uint8_t ite = 0; ite < fifo_len; ite++) {
            /* Read acceleration data */
            uint16_t index = ite*6;
            accData[ite].x = ((data_raw_acceleration[index+1]<<8)&0xFF00)|((uint16_t)data_raw_acceleration[index]&0xFF);
            accData[ite].y = ((data_raw_acceleration[index+3]<<8)&0xFF00)|((uint16_t)data_raw_acceleration[index+2]&0xFF);
            accData[ite].z = ((data_raw_acceleration[index+5]<<8)&0xFF00)|((uint16_t)data_raw_acceleration[index+4]&0xFF);
            //NRF_LOG_INFO("x:%d, y:%d, z:%d", accData[ite].x, accData[ite].y, accData[ite].z);

            accLen++;
        }
    }
    
    int16_t t = 4010;
    //NRF_LOG_INFO("t0: %02X, t1: %02X", ((uint8_t *)(&t))[0], ((uint8_t *)(&t))[1]);
    NRF_LOG_INFO("fifo_len: %d", fifo_len);

    g_ACC_num = g_ACC_num + accLen;
    memset(&ax, 0, sizeof(ax));
    #if 1
    if ( 0 < accLen) {
        if (module_acc.dev_info.max_fifo_len < fifo_len)
            module_acc.dev_info.max_fifo_len = fifo_len;

        if (rslt == 0 && accLen > 0 && accLen <= ACC_FIFOLEN) {
            int16_t ACC_x, ACC_y, ACC_z, ACC_g;
            for (idx = 0; idx < accLen; idx++) {
                ACC_x = (int16_t)FS4G_TO_MG(accData[idx].x);
                ACC_y = (int16_t)FS4G_TO_MG(accData[idx].y);
                ACC_z = (int16_t)FS4G_TO_MG(accData[idx].z);

                ax.axis[idx].x = ACC_x;
                ax.axis[idx].y = ACC_y;
                ax.axis[idx].z = ACC_z;
                AccProc(&ax.axis[idx], statu);
                //NRF_LOG_INFO("ACC: %d, %d, %d", ACC_x, ACC_y, ACC_z);
            }
            NRF_LOG_INFO("ACC: %d, %d, %d", ACC_x, ACC_y, ACC_z);

            //ACC_g = (uint16_t)pow(ACC_x * ACC_x + ACC_y * ACC_y + ACC_z * ACC_z, 0.5);
            ax.n = accLen;
            //NRF_LOG_INFO("ACC_g: %d", ACC_g);
            // if( accLen < 20)
            //NRF_LOG_INFO("ACC: %d, %d, %d;  accLen = %d\r\n ", ACC_x, ACC_y, ACC_z, accLen);
            int xyz;
            if ((ACC_DOWNSAMPLE_NONE != module_acc.dev_info.smp_period) && (0 == (module_acc.dev_info.smp_times++) % module_acc.dev_info.smp_period)) {
                module_acc.dev_info.sample_data = accData[0];
                ACC_x = (int16_t)FS4G_TO_MG(module_acc.dev_info.sample_data.x);
                ACC_y = (int16_t)FS4G_TO_MG(module_acc.dev_info.sample_data.y);
                ACC_z = (int16_t)FS4G_TO_MG(module_acc.dev_info.sample_data.z);

                xyz = (int)ACC_x*ACC_x + (int)ACC_y*ACC_y + (int)ACC_z*ACC_z;
		        module_acc.dev_info.sample_acc_g = (uint16_t)(sqrt_bv(xyz*100)/10);
                //module_acc.dev_info.sample_acc_g = (uint16_t)pow(ACC_x * ACC_x + ACC_y * ACC_y + ACC_z * ACC_z, 0.5);
                module_acc.dev_info.smp_data_used = ACC_SAMPLE_NEWDATA;
            }
        }
    }
    #endif
    return rslt;
}

uint8_t acc_clear_fifo(){
    i2c_init();
    uint8_t fifo_len = 0;
    lis2dw12_auto_increment_set(&dev_ctx, 1);
    lis2dw12_fifo_data_level_get(&dev_ctx, &fifo_len);
    if(fifo_len > 32){
        fifo_len = 32;
    }

    if(fifo_len > 0){
        static int8_t data_raw_acceleration[6*32];
        uint8_t ret = lis2dw12_acceleration_raw_get_1(&dev_ctx, data_raw_acceleration, 6*fifo_len);
    }

    i2c_uninit();

    NRF_LOG_INFO("acc_clear_fifo fifo_len: %d", fifo_len);
    return fifo_len;
}

void set_acc_int1_status(void) {
  module_acc.dev_info.int1_line = ACC_DEV_OK;
}

void set_acc_int2_status(void) {
  module_acc.dev_info.int2_line = ACC_DEV_OK;
}

struct acc_dev_info_s show_acc_dev_info(void) {
#if 1
  NRF_LOG_INFO("< i2c:%d, int1:%d, int2:%d, <%d, %d>",
      module_acc.dev_info.i2c_line,
      module_acc.dev_info.int1_line,
      module_acc.dev_info.int2_line,
      module_acc.dev_info.sample_data.z,
      module_acc.dev_info.sample_acc_g);
#endif
  module_acc.dev_info.smp_data_used = ACC_SAMPLE_USEDDATA;
  return module_acc.dev_info;
}

bool acc_is_new_data(void) {
  if (ACC_SAMPLE_NEWDATA == module_acc.dev_info.smp_data_used)
    return true;
  else
    return false;
}

void set_acc_downsample_rate(enum acc_downsample_rate_e downsample_rate) {
  if (ACC_DOWNSAMPLE_INVALID <= downsample_rate)
    module_acc.dev_info.smp_period = ACC_DOWNSAMPLE_NONE;
  else
    module_acc.dev_info.smp_period = downsample_rate;
}

static lis2dw12_all_sources_t s_all_source = {0};
void acc_sync_all_status(){
    lis2dw12_all_sources_get(&dev_ctx, &s_all_source);
    printf("acc_sync_all_status %d, %d, %d\r\n", s_all_source.all_int_src.sleep_change_ia, s_all_source.status_dup.sleep_state_ia,s_all_source.wake_up_src.wu_ia);
}

bool is_tap_int(void){
    //lis2dw12_all_sources_t all_source;
    //lis2dw12_all_sources_get(&dev_ctx, &all_source);
    NRF_LOG_INFO("all_source, single_tap, %X", (uint8_t)s_all_source.tap_src.single_tap);
    if (s_all_source.tap_src.single_tap) 
      return true;
    else
      return false;

}

bool is_wakeup_detect(void){
    //lis2dw12_all_sources_t all_source;
    //lis2dw12_all_sources_get(&dev_ctx, &all_source);
    NRF_LOG_INFO("all_source, wake_up_src, %X", (uint8_t)s_all_source.wake_up_src.wu_ia);
    if (s_all_source.wake_up_src.wu_ia) 		
      return true;
    else
      return false;

}

void acc_set_bypass(void){
    int ret = lis2dw12_fifo_mode_set(&dev_ctx, LIS2DW12_BYPASS_MODE);
    NRF_LOG_INFO("acc_set_bypass : %d", ret);
}

void acc_set_continous(void){
    int ret  = lis2dw12_fifo_mode_set(&dev_ctx, LIS2DW12_STREAM_MODE);
    NRF_LOG_INFO("acc_set_continous : %d", ret);
}

bool is_acc_in_sleep(void){
    if(s_all_source.status_dup.sleep_state_ia == true){
        //printf("sleep_state_ia true\r\n");
        return true;
        //advertising_stop();
    }else{
        //printf("sleep_state_ia false\r\n");
        //advertising_start(false);
    }   

    return false;
}

bool is_acc_sleep_detect(void){
    if(s_all_source.all_int_src.sleep_change_ia == true && s_all_source.status_dup.sleep_state_ia == true){
        return true;
    }

    return false;
}

//void acc_test(void){
//    uint8_t odr = 0;
//    lis2dw12_data_rate_get(&dev_ctx, &odr);

//    printf("acc_test lis2dw12_data_rate_get odr=%d\n", odr);

//    acc_sync_all_status();
//    is_acc_sleep_detect();
//}

bool is_fth_int(void){
    uint8_t wtm = 0;
    lis2dw12_fifo_wtm_flag_get(&dev_ctx, &wtm);
    NRF_LOG_INFO("is_fth_int lis2dw12_fifo_wtm_flag_get %d", wtm);
    return wtm;
}
