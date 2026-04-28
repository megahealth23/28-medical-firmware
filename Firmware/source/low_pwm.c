/*
*author:add by tianwanjie
*brief:low pwm function
*
*
*/
#include "low_pwm.h"


/*Ticks before change duty cycle of each LED*/
#define TICKS_BEFORE_CHANGE_0   100

static low_power_pwm_t low_power_pwm_0;


/**
 * @brief Function to be called in timer interrupt.
 *
 * @param[in] p_context     General purpose pointer (unused).
 */
static void pwm_handler(void* p_context)
{
	uint8_t new_duty_cycle;
	static uint16_t led_0;
	uint32_t err_code;
	UNUSED_PARAMETER(p_context);
	low_power_pwm_t* pwm_instance = (low_power_pwm_t*)p_context;
	led_0++;
	if (led_0 > TICKS_BEFORE_CHANGE_0)
	{
		new_duty_cycle = pwm_instance->period - pwm_instance->duty_cycle;
		err_code = low_power_pwm_duty_set(pwm_instance, new_duty_cycle);
		led_0 = 0;
		APP_ERROR_CHECK(err_code);
	}
}


/**
 * @brief Function to initalize low_power_pwm instances.
 *
 */

void pwm_init(uint32_t select_output_pwm_pin)
{
#if (select_output_pwm_pin < 0 ||  select_output_pwm_pin > 32)
#error "please input pin 0-31"
#endif
	uint32_t err_code;
	low_power_pwm_config_t low_power_pwm_config;

	APP_TIMER_DEF(lpp_timer_0);
	low_power_pwm_config.active_high = false;
	low_power_pwm_config.period = 220;
	low_power_pwm_config.bit_mask = 1 << (select_output_pwm_pin & (~32));
	low_power_pwm_config.p_timer_id = &lpp_timer_0;
	low_power_pwm_config.p_port = NRF_GPIO;

	err_code = low_power_pwm_init((&low_power_pwm_0), &low_power_pwm_config, pwm_handler);
	APP_ERROR_CHECK(err_code);
	err_code = low_power_pwm_duty_set(&low_power_pwm_0, 0);
	APP_ERROR_CHECK(err_code);
}


void pwm_start(void)
{
	uint32_t err_code;
	err_code = low_power_pwm_start((&low_power_pwm_0), low_power_pwm_0.bit_mask);
	APP_ERROR_CHECK(err_code);
}

void pwm_stop(void)
{
	uint32_t err_code;
	err_code = low_power_pwm_stop(&low_power_pwm_0);
	APP_ERROR_CHECK(err_code);
}

void pwm_set_duty(uint8_t new_duty_cycle)
{
	uint32_t err_code;
	err_code = low_power_pwm_duty_set(&low_power_pwm_0, new_duty_cycle);
	APP_ERROR_CHECK(err_code);
}


//transplant code test function
void pwm_test(void)
{
	pwm_init(14);
	pwm_start();
}



