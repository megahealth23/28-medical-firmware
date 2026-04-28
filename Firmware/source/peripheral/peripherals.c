#include "peripherals.h"
#include "nrf_drv_gpiote.h"
#include "nrf_gpio.h"
#include "Ring_bsp.h"
#include "ring_acc.h"
#include "ring_afe.h"
#include "ring_tmp.h"
#include "ring_battery.h"

void PPG_board_power_on(){
    //advertising_stop();
    nrf_delay_ms(2);
    nrf_drv_gpiote_pin_t afereset_pin = AFE_RESET_PIN;
    nrf_gpio_cfg(afereset_pin,
                    NRF_GPIO_PIN_DIR_OUTPUT,
                    NRF_GPIO_PIN_INPUT_DISCONNECT,
                    NRF_GPIO_PIN_PULLUP,
                    NRF_GPIO_PIN_S0S1,
                    NRF_GPIO_PIN_NOSENSE);
    nrf_gpio_pin_set(afereset_pin);
    nrf_delay_ms(20);
    //advertising_start(false); 
    return;
}

/***************************/
/* NOTICE: the time period from power off cmd to PPG real power off is 35 seconds.
/****************************/
void PPG_board_power_off(){
    nrf_drv_gpiote_pin_t afereset_pin = AFE_RESET_PIN;
    nrf_gpio_cfg(afereset_pin,
                    NRF_GPIO_PIN_DIR_OUTPUT,
                    NRF_GPIO_PIN_INPUT_DISCONNECT,
                    NRF_GPIO_PIN_NOPULL,
                    NRF_GPIO_PIN_S0S1,
                    NRF_GPIO_PIN_NOSENSE);
    nrf_gpio_pin_clear(afereset_pin);
    nrf_delay_ms(500);
    //nrf_gpio_cfg_default(afereset_pin);
    //nrf_delay_us(500); 
    return;
}

bool peripherals_check_init(void) {
    struct acc_dev_info_s acc_dev_info = {0};
    struct afe_dev_info_s afe_dev_info = {0};
    struct tmp_dev_info_s tmp_dev_info1 = {0};
    struct tmp_dev_info_s tmp_dev_info2 = {0};

    //PPG_board_power_off();
    //nrf_delay_ms(10*1000);

    if(chgchip_init())   {
        ring_set_sensor_status(CHGCHIP_DRV_OK, charg_chip_ok());
    }

    PPG_board_power_on();
    ring_topled_on();
    tmp_dev_info1.i2c_line = 1;
    if (tmp_start(RING_TMP_ONE_SHOT_MODE, RING_INSIDE_TMP117)) {
        tmp_dev_info2 = show_tmp_dev_info(RING_INSIDE_TMP117);
        tmp_start(RING_TMP_SHUTDOWN_MODE, RING_INSIDE_TMP117);
    }
    ring_set_sensor_status(TMP117_DRV_OK,
      (1 == tmp_dev_info1.i2c_line && 1 == tmp_dev_info2.i2c_line) ? true : false);

    if (acc_start(IMU_STANDBY)) {
        acc_dev_info = show_acc_dev_info();
        ring_set_sensor_status(GSENSOR_DRV_OK, (1 == acc_dev_info.i2c_line) ? true : false);
    }
    acc_start(IMU_POWEROFF);
    acc_stop();

    if (afe_start(RING_AFE_SPO2_MODE)) {
        afe_dev_info = show_afe_dev_info();
        ring_set_sensor_status(AFE_DRV_OK, (1 == afe_dev_info.i2c_line) ? true : false);
    }
    afe_stop();

    ring_topled_off();

    if(tmp_dev_info2.i2c_line == 1 && 
    true == charg_chip_ok() &&
    1 == acc_dev_info.i2c_line && 
    1 == afe_dev_info.i2c_line)
        return true;

    return false;
}

bool peripherals_check(void) {
    struct acc_dev_info_s acc_dev_info = {0};
    struct afe_dev_info_s afe_dev_info = {0};
    struct tmp_dev_info_s tmp_dev_info1 = {0};
    struct tmp_dev_info_s tmp_dev_info2 = {0};

    if(chgchip_init())   {
        ring_set_sensor_status(CHGCHIP_DRV_OK, charg_chip_ok());
    }

    PPG_board_power_on();

    tmp_dev_info1.i2c_line = 1;
    if (tmp_start(RING_TMP_ONE_SHOT_MODE, RING_INSIDE_TMP117)) {
        tmp_dev_info2 = show_tmp_dev_info(RING_INSIDE_TMP117);
        tmp_start(RING_TMP_SHUTDOWN_MODE, RING_INSIDE_TMP117);
    }
    ring_set_sensor_status(TMP117_DRV_OK,
      (1 == tmp_dev_info1.i2c_line && 1 == tmp_dev_info2.i2c_line) ? true : false);

    if (acc_start(IMU_STANDBY)) {
        acc_dev_info = show_acc_dev_info();
        ring_set_sensor_status(GSENSOR_DRV_OK, (1 == acc_dev_info.i2c_line) ? true : false);
        acc_start(IMU_POWEROFF);
        acc_stop();
    }

    //if (afe_start(RING_AFE_SPO2_MODE)) {
    //    afe_dev_info = show_afe_dev_info();
    //    ring_set_sensor_status(AFE_DRV_OK, (1 == afe_dev_info.i2c_line) ? true : false);
    //    afe_start(RING_AFE_POWEROFF);
    //}

    ring_topled_off();

    if(tmp_dev_info2.i2c_line == 1 && 
    true == charg_chip_ok() &&
    1 == acc_dev_info.i2c_line && 
    1 == afe_dev_info.i2c_line)
        return true;

    return true;
}

void peripherals_power_off(){
    //PPG_board_power_on();
    ring_handonled_off();

	i2c_init();
	acc_start(IMU_STANDBY);     //timer 
	//acc_config_wake_int(true, WAKE_LOW);
	afe_stop();
	tmp117_measure_deinit();
	tmp_start(RING_TMP_SHUTDOWN_MODE, RING_INSIDE_TMP117);
    saadc_stop();
    i2c_uninit();

    //AFE4900_HWshutdown();
    PPG_board_power_off();
}

void peripherals_power_off_4_lp(){
    //PPG_board_power_on();
    ring_handonled_off();

	i2c_init();
	acc_start(IMU_STANDBY);     //timer 
	//acc_config_wake_int(true, WAKE_LOW);
	afe_stop();
	tmp117_measure_deinit();
	tmp_start(RING_TMP_SHUTDOWN_MODE, RING_INSIDE_TMP117);
    //saadc_stop();
    i2c_uninit();

    //AFE4900_HWshutdown();
    PPG_board_power_off();
}

void peripherals_power_off_without_power(){
    //PPG_board_power_on();
    ring_handonled_off();

	i2c_init();
	acc_start(IMU_STANDBY);     //timer 
	//acc_config_wake_int(true, WAKE_LOW);
	afe_stop();
	tmp117_measure_deinit();
	tmp_start(RING_TMP_SHUTDOWN_MODE, RING_INSIDE_TMP117);
    i2c_uninit();

    //AFE4900_HWshutdown();
    //PPG_board_power_off();
}