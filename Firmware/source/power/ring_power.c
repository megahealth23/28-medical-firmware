#include "ring_power.h"


/****************    systemoff     **********************/

static void systemoff_offchip_peripheral(void)
{
	ring_handonled_off();
	i2c_init();
	acc_start(IMU_POWEROFF);     //timer 
	afe_stop();
	
	tmp117_measure_deinit();

	tmp_start(RING_TMP_SHUTDOWN_MODE, RING_INSIDE_TMP117);

	//chgchip_start(RING_CHGCHIP_POWEROFF);
}


void systemoff_onchip_peripheral(void)
{
	nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
	
	nrf_drv_gpiote_pin_t accint1_pin = ACC_INT1_PIN;
	nrf_drv_gpiote_pin_t accint2_pin = ACC_INT2_PIN;
	//nrf_drv_gpiote_pin_t adcready_pin = AFE_ADCREADY_PIN;
//	nrf_drv_gpiote_pin_t dect_charge_status_pin = BATT_IS_CHARGE_PG25120A;
	//nrf_drv_gpiote_pin_t afereset_pin = AFE_RESET_PIN;

/*********   Battery domain  ***********	
* relative pin: 1)DETECTION_5V_IN_PIN, 2)BATT_IS_CHARGE_PG25120A(PULLUP), 3)ADC_BAT_VOL_PIN/<Tip:without configure>/
* other source:
******************************************/	
	saadc_stop();
	
/*********** I2C common domain ************
* relative pin: 1)RING_SCL_PIN(PASS), 2)RING_SDA_PIN(PASS)
******************************************/
	i2c_uninit();
	
/***********  ACC domain   ***************
* relative pin: 1)ACC_INT1_PIN(PULLUP), 2)ACC_INT2_PIN(PULLUP)
* other source:
******************************************/

/***********  AFE domain  ***************
* relative pin: 1)AFE_ADCREADY_PIN(PULLUP), 2)AFE_RESET_PIN(PULLDOWN), 3)AFE_PROG_PIN(PULLDOWN), 4)AFE_SEN_PIN(NOUSE)
* other source:
******************************************/
	
/***********  TMP domain  ***************
* relative pin: 1)TMP_INT1_PIN(PULLUP), 2)TMP_INT2_PIN(PULLUP)
* other source:
******************************************/

/***********  HANDON LED domain  ***************
* relative pin: 1)GREEN_LED_2MA, 2)GREEN_LED_3MA(NOUSE), 3)GREEN_LED_4MA(NOUSE)(NOUSE), 3)GREEN_LED_7MA(NOUSE)
* other source:
******************************************/	
	in_config.pull = NRF_GPIO_PIN_PULLUP;
	nrf_drv_gpiote_in_init(accint1_pin, &in_config, NULL);
	nrf_drv_gpiote_in_init(accint2_pin, &in_config, NULL);
	//nrf_drv_gpiote_in_init(adcready_pin, &in_config, NULL);
//	nrf_drv_gpiote_in_init(dect_charge_status_pin, &in_config, NULL);	

	in_config.pull = NRF_GPIO_PIN_PULLDOWN;
	//nrf_drv_gpiote_in_init(AFE_PROG_PIN, &in_config, NULL);
	//nrf_drv_gpiote_in_init(afereset_pin, &in_config, NULL);
	
	nrf_drv_gpiote_in_event_enable(accint1_pin, true);
	nrf_drv_gpiote_in_event_enable(accint2_pin, true);
	//nrf_drv_gpiote_in_event_enable(dect_charge_status_pin, true);
	//nrf_drv_gpiote_in_event_enable(AFE_PROG_PIN, true);
	//nrf_drv_gpiote_in_event_enable(afereset_pin, true);
}

// DETECTION_5V_IN_PIN
static void systemoff_wakeuppin_cfg(void)
{
//	nrf_drv_gpiote_pin_t dect_5v_in_pin = DETECTION_5V_IN_PIN;

	nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(false);
	in_config.pull = 	NRF_GPIO_PIN_NOPULL;	
	//nrf_drv_gpiote_in_init(dect_5v_in_pin, &in_config, NULL);
	//nrf_drv_gpiote_in_event_enable(dect_5v_in_pin, true);
}

static void systemoff_core(void)
{
//	uint32_t err_code;
	nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
}

void ring_systemoff_entry(void)
{
	systemoff_offchip_peripheral();
	systemoff_onchip_peripheral();
	systemoff_wakeuppin_cfg();
	systemoff_core();
}

/*************************    ****************************/
//static void systemidle_offchip_peripheral(void)
//{
//	ring_handonled_off();
//	i2c_init();

//	acc_start(IMU_STANDBY);     //timer 
//	acc_config_wake_int(true, WAKE_LOW);

//	afe_start(RING_AFE_POWEROFF);
	
//	tmp117_measure_deinit();

//	tmp_start(RING_TMP_SHUTDOWN_MODE, RING_INSIDE_TMP117);
//}

//void ring_startup_idle(void)
//{
////	ble_gap_conn_params_t gpt = NORMAL_BLE_RXTX_PARAM;//ANDROID_BLE_RXTX_SLOW_PARAM;
////	ring_connparam_update(&gpt);//max connection time interval
//	systemidle_offchip_peripheral();
//    i2c_uninit();
//}


/************    system reset    ****************/
void systemreset_offchip_peripheral(void)
{
	ring_handonled_off();
	i2c_init();

	acc_start(IMU_POWEROFF);     //timer 
	afe_stop();
	
	tmp117_measure_deinit();

	tmp_start(RING_TMP_SHUTDOWN_MODE, RING_INSIDE_TMP117);
}


static void systemreset_onchip_peripheral(void)
{
	NVIC_SystemReset();
}

void ring_systemreset_entry(void)
{
	systemreset_offchip_peripheral();
	systemreset_onchip_peripheral();
}

/*********  LowPower *************/
static void systemlowpower_offchip_peripheral(void)
{
	ring_handonled_off();
	i2c_init();

	acc_stop();     //timer 
//	acc_config_wake_int(false, 0);
	
	afe_stop();
	
	tmp117_measure_deinit();

	tmp_stop(RING_INSIDE_TMP117);
}

void ring_systemlowpower_entry(void)
{
	systemlowpower_offchip_peripheral();
}

/********* Ultra-LowPower *************/
//static void ultra_lowpower_offchip_peripheral(void)
//{
//	nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
	
//	//nrf_drv_gpiote_pin_t afereset_pin = AFE_RESET_PIN;

//		/****  TOP_LED  &  Timer  &  PWN   ****/
//	ring_stop_indicate();// Timer:handonled_timer_id  & TOP_LED

//	i2c_init();
	
//	/****  ACC  &&  Timer  && ACC_INT1 && ACC_INT2  ****/
//	acc_config_wake_int(false, 0); // ACC_INT1   
//	acc_config_Tap(false);  // ACC_INT2

////CRITICAL_REGION_ENTER();
//	acc_start(IMU_POWEROFF); // Timer:m_accel_timer_id  & ACC 

//	/****  Timer && TMP ****/
//	tmp117_measure_deinit();
//#ifndef ZG28_V5_ENABLE
//	tmp_stop(RING_OUTSIDE_TMP117);
//#else
//	if(BSP_ZG28_MAIN_V4 == get_bsp_version()){
//		tmp_stop(RING_OUTSIDE_TMP117);
//	}
//#endif	
//	tmp_stop(RING_INSIDE_TMP117);
	
//	/***  AFE4900  &&  RESET  &&  ADCREADY  ***/
//	afe_start(RING_AFE_POWEROFF);
//	//nrf_gpio_cfg_default(afereset_pin);  //  ADCREADY   in ring_afe.c  nrf_gpio_cfg(NOPULL)
//	in_config.pull = NRF_GPIO_PIN_PULLDOWN;
//	//nrf_drv_gpiote_in_init(afereset_pin, &in_config, NULL);   // PWDN mode, Keeping low) 
	
////CRITICAL_REGION_EXIT();

//}

//static void ultra_lowpower_onchip_peripheral(void)
//{
//	/****  I2C-Bus  ****/
//	i2c_uninit();
	
//	/****  Timer && ADC  &&   DETECTION_5V  && BATT_IS_CHARGE && BAT_VOL_PIN****/
//	// SKIP m_read_tmp117_timer
//	ring_rawdata_stop();  //m_notify_timer
//	// SKIP handonled_timer_id
//	battery_uninit();  // m_saadc_timer_id  &  ADC  &   DETECTION_5V  &  BATT_IS_CHARGE   &    BAT_VOL_PIN
//	// SKIP m_accel_timer_id
	
//	/****   Radio   ****/
//	ble_gap_conn_params_t gpt = STANDBY_BLE_RXTX_PARAM;//ANDROID_BLE_RXTX_SLOW_PARAM;
//	ring_connparam_update(&gpt);//max connection time interval
//}

//void ring_ultra_lowpower_entry(void)
//{
//	ultra_lowpower_offchip_peripheral();
//	ultra_lowpower_onchip_peripheral();
//}
