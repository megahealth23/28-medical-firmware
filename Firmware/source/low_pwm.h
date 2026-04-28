
#ifndef _LOW_PWM_H  
#define _LOW_PWM_H 

#include "low_power_pwm.h"
#include "app_timer.h"

#include "ring_bsp.h"

void pwm_init(uint32_t select_output_pwm_pin);
void pwm_start(void);
void pwm_stop(void);
void pwm_set_duty(uint8_t new_duty_cycle);

//test function
void pwm_test(void);

#endif
