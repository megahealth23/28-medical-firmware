/*
* Copyright (c) 2019 MegaHealth. All Rights Reserved.
* @file ring_handonled.c
* @author:li tianzheng
* @modifing:
* @date 2020.0.0
* @version
* @brief
*/

#include "ring_handonled.h"
#include "ring_bsp.h"

/*** Create the instance "PWM1" using TIMER1 ***/
APP_PWM_INSTANCE(PWM1,1);   
APP_TIMER_DEF(handonled_timer_id);											/**< ble pair timer. */

static struct ring_handonled_s module_handonled = 
{			
	.visual_action   = HANDON_LED_ACTION_NONE,
	.using_timer     = false,
	.using_pwn			 = false,
	.win_10sec			 = false,
	.auto_flashing   = false,
	.flashing_period = HANDONLED_PERIOD_NONE,
	.flashing_times  = 0,
	.run_tick_10sec    = 0,
	.pwm_duty = HANDONLED_DUTY_NONE,
};

static void pwm_ready_callback(uint32_t pwm_id)  {}  // PWM callback function

static void hw_handonled_pwn_init(void)
{
	uint32_t err_code;

	nrf_drv_gpiote_pin_t handon_led_pin = HANDON_LED;

	app_pwm_config_t pwm1_cfg = APP_PWM_DEFAULT_CONFIG_1CH(30000L,  handon_led_pin/*PIN1.8*/);
/* V8's setting parameters is different from V6, because the LED is driven by low level. */
	
	pwm1_cfg.pin_polarity[0] = APP_PWM_POLARITY_ACTIVE_HIGH;


	if( !module_handonled.using_pwn)
	{
		/* Initialize and enable PWM. */
		err_code = app_pwm_init(&PWM1,&pwm1_cfg,pwm_ready_callback);
		APP_ERROR_CHECK(err_code);		
		module_handonled.using_pwn = true;
	}
}

static void hw_handonled_pwn_uninit(void)
{
	if(module_handonled.using_pwn)
	{
		app_pwm_disable(&PWM1);
		app_pwm_uninit(&PWM1);
        
        //nrf_drv_gpiote_pin_t handon_led_pin = HANDON_LED;
        //nrf_gpio_cfg(handon_led_pin,
        //                NRF_GPIO_PIN_DIR_INPUT,
        //                NRF_GPIO_PIN_INPUT_CONNECT,
        //                NRF_GPIO_PIN_NOPULL,
        //                NRF_GPIO_PIN_S0D1,
        //                NRF_GPIO_PIN_NOSENSE);
		module_handonled.using_pwn = false;
	}
}

/*static void hw_handonled_on(void)
	Tip:
	1) We have to obey the official rules of the configuration for PWM.
		Thus, Enable. then Setting Duty,
	
	Warning:
	1)the following setting is important, because of the first Waveform should be keep to low.
	>			pwm1_cfg.pin_polarity[0] = APP_PWM_POLARITY_ACTIVE_HIGH;
*/

static void hw_handonled_on(void)
{
	app_pwm_enable(&PWM1);
	while (app_pwm_channel_duty_set(&PWM1, 0, module_handonled.pwm_duty) == NRF_ERROR_BUSY);
}

static void hw_handonled_off(void)
{
	nrf_drv_gpiote_pin_t handon_led_pin = HANDON_LED;
	
	hw_handonled_pwn_uninit();
	nrf_gpio_cfg_default(handon_led_pin);

    
    //int pin = handon_led_pin;
    //nrf_gpio_cfg(pin,
    //                NRF_GPIO_PIN_DIR_OUTPUT,
    //                NRF_GPIO_PIN_INPUT_DISCONNECT,
    //                NRF_GPIO_PIN_NOPULL,
    //                NRF_GPIO_PIN_S0D1,
    //                NRF_GPIO_PIN_NOSENSE);
    //nrf_gpio_pin_set(pin);
}

static void handonled_timeout_handler(void * p_context)
{
	UNUSED_PARAMETER(p_context);
	uint32_t err_code = NRF_SUCCESS;
	
	if( 0x00001 & (++module_handonled.flashing_times) )
	{
		module_handonled.visual_action = HANDON_LED_ACTION_ON;
			
		if( module_handonled.win_10sec && 0 == (module_handonled.run_tick_10sec--) )
		{
			ring_handonled_off();
			return;
		}
	}
	else
		module_handonled.visual_action = HANDON_LED_ACTION_OFF;
		
	if(HANDON_LED_ACTION_ON == module_handonled.visual_action )
	{
		hw_handonled_pwn_init();
		hw_handonled_on();
	}
		
	if(HANDON_LED_ACTION_OFF == module_handonled.visual_action )
	{
		hw_handonled_off();
	}
	APP_ERROR_CHECK(err_code);
}

bool ring_handonled_flashing_start(bool win_10s, enum handonled_flashing_period_e flashing_period)
{
	uint32_t err_code;
	module_handonled.using_timer = true;
	
	module_handonled.flashing_times = 0;
	module_handonled.win_10sec = win_10s;
	if(win_10s)
		module_handonled.run_tick_10sec = 5000/((uint16_t)flashing_period);
	
	if( (HANDONLED_PERIOD_NONE != module_handonled.flashing_period)
					&& (flashing_period != module_handonled.flashing_period) )
			app_timer_stop(handonled_timer_id);
	module_handonled.flashing_period = flashing_period;
	
	err_code = app_timer_start(handonled_timer_id, APP_TIMER_TICKS((uint16_t)flashing_period), NULL);			
	APP_ERROR_CHECK(err_code);
	
	return true;
}

void ring_handonled_flashing_stop(void)
{
	module_handonled.win_10sec = false;
	module_handonled.flashing_period = HANDONLED_PERIOD_NONE;
	ring_handonled_off();
}

bool ring_handonled_on(void)
{
	if(module_handonled.using_timer)
	{
		app_timer_stop(handonled_timer_id);
		module_handonled.using_timer = false;
	}
	
	hw_handonled_on();
	return true;
}

void ring_handonled_off(void)
{
	if(module_handonled.using_timer)
	{
		app_timer_stop(handonled_timer_id);
		module_handonled.using_timer = false;
	}

	hw_handonled_off();
}

bool ring_handonled_init(bool create_timer, enum handonled_duty_e duty)
{
	uint32_t err_code;

	nrf_drv_gpiote_pin_t handon_led_pin = HANDON_LED;

	nrf_gpio_cfg_default(handon_led_pin);
    
    //int pin = handon_led_pin;
    //nrf_gpio_cfg(pin,
    //                NRF_GPIO_PIN_DIR_OUTPUT,
    //                NRF_GPIO_PIN_INPUT_DISCONNECT,
    //                NRF_GPIO_PIN_NOPULL,
    //                NRF_GPIO_PIN_S0D1,
    //                NRF_GPIO_PIN_NOSENSE);
    //nrf_gpio_pin_set(pin);

	if(create_timer)
	{
		err_code = app_timer_create(&handonled_timer_id,
											APP_TIMER_MODE_REPEATED,
													handonled_timeout_handler);
		APP_ERROR_CHECK(err_code);
		return true;
	}
	
	module_handonled.win_10sec = false;
	if( duty != module_handonled.pwm_duty)
		module_handonled.pwm_duty = duty;
	
	hw_handonled_pwn_init();
	return true;
}

