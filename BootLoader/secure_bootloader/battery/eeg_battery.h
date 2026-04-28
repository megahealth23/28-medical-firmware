#ifndef _EEG_BATTERY_H_
#define _EEG_BATTERY_H_

#include <stdbool.h>
#include <stdint.h>
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_saadc.h"

#ifdef __cplusplus
extern "C" {
#endif

//extern struct battery_st BattInfo;

// Charging status
#define BATT_CHG_PIN NRF_GPIO_PIN_MAP(0, 29)  //5V-IN-DEC. Trigger pin,charge voltage detect sampling
#define CHG_STAT_PIN NRF_GPIO_PIN_MAP(0, 30) //Charging status.Charging,Low. Other,High.
#define BAT_ADC_PIN NRF_SAADC_INPUT_AIN0  //ADC pin,BAT voltage detect sampling
#define BATT_CHG_CURRENT_CTRL_PIN NRF_GPIO_PIN_MAP(0, 6) //Large current charging control;CD-25120A must be set HIGH initially;When charging,MCU£¨BOOT£© can detect the BAT voltage3.0-3.1V,CD-25120A must be set LOW;When MCU works normally,CD-25120A must be set LOW.

#define ADC_LOWPOWER_VOL (3.15f)
#define ADC_SHUTDOWN_VOL (3.0f)

void battery_init(void);
void charger_large_current(void);
void charger_small_current(void);
bool device_is_oncharger(void);

#ifdef __cplusplus
}
#endif
#endif /* _EEG_BATTERY_H_ */