#ifndef BATTERY_ADC_H
#define BATTERY_ADC_H
#include "nrf_drv_saadc.h"
#include "nrf_log_ctrl.h"
#include "nrf_saadc.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ADC_MAX_SAMPLE 3300
#define ADC_MIN_SAMPLE 2340 // R1=2.2Mom, R2=680Kom, 2.9V
//#define ADC_MAX_SAMPLE				(3300)		// R1=2.2Mom, R2=680Kom, 4.101V
#define ADC_BURST_SAMPLE (2725)    // R1=2.2Mom, R2=680Kom, 3.6V
#define ADC_LOWPOWER_SAMPLE (2540) // R1=2.2Mom, R2=680Kom, 3.15V
#define ADC_SHUTDOWN_SAMPLE (2420) // R1=2.2Mom, R2=680Kom, 3.0V

#define ADC_SAFE_START  2700

#define ADC_CALIBRATE_MAX 			480

enum dev_power_state {
  DEV_NORMAL,   //device normal work
  DEV_CHGING,   //device charging
  DEV_CHGFULL,  //device charge full
  DEV_LOWPWR,   //device low power
  DEV_FAULTS,   //device faults
  DEV_SHUTDOWN, //device shutdown
};

struct battery_st {
  bool is_working;
  bool is_oncharger;
  bool is_full;
  uint8_t vol_pct; // convert batter volatage percent
  int16_t adc_val;
  int16_t paddings;

  volatile enum dev_power_state dev_ps;
};

void saadc_run(bool timer);
void saadc_stop(void);

extern struct battery_st BattInfo;

#ifdef __cplusplus
}
#endif
#endif /* BATTERY_ADC_H */