/*********************************************************************************
 * Copyright (c) 2019 MegaHealth. All Rights Reserved.
 * battery_adc.h
 * AUTHOR:zhao mengshou
 * DATE:2019/9/21 17:16:38
 * http://www.megahealth.cn
 *
 */

#ifndef BATTERY_ADC_H  
#define BATTERY_ADC_H  
#include "nrf_drv_saadc.h"
#include "nrf_log_ctrl.h"

#ifdef __cplusplus  
extern "C"
{
#endif  


//#define USE_SAADC_TRACE

//#define ADC_MAX_SAMPLE			3800
//#define ADC_MIN_SAMPLE			2800


/*****************************************************************************
3300 100%
2900 10%
2540 5%		
2420 2%
2340 1%
(2900 3300] y = k*x + b (y = 0.225 * x - 642.5)
(2540 2900] y = a + b*x + c * x * x (y = -1.091e-5 * x * x + 0.07325 * x - 110.7)
[2340 2540] y = a + b*x + c * x * x (y = 6.25e-5 * x * x - 0.285 * x + 325.7)
*****************************************************************************/
#define ADC_CHGFULL_WM                          (3355)          //4.16V,   other condition for chgfull Watermark 
#define ADC_MAX_SAMPLE				(3300)		// R1=2.2Mom, R2=680Kom, 4.101V

#define ADC_BURST_SAMPLE			(2900)		// R1=2.2Mom, R2=680Kom, 3.6V

#define ADC_BURST_1_SAMPLE			(2960)		// R1=2.2Mom, R2=680Kom, 3.6V
#define ADC_BURST_2_SAMPLE			(2780)		// R1=2.2Mom, R2=680Kom, 3.6V

#define ADC_LOWPOWER_SAMPLE			(2600)		// R1=2.2Mom, R2=680Kom, 3.15V

#define ADC_SHUTDOWN_SAMPLE			(2420)		// R1=2.2Mom, R2=680Kom, 3.0V

#define ADC_MIN_SAMPLE				(2340)		// R1=2.2Mom, R2=680Kom, 2.9V

#define ADC_LOWPOWER_VOL			(3.15f)
#define ADC_SHUTDOWN_VOL			(3.0f)

#define ADC_CALIBRATE_MAX 			480

//0.6*4*2*ADC/4096 = battery vol
#define CONVERT_ADC_2_VOL(ADC)		(1.2*(2200.0+680.0)*ADC/680.0/4096.0)

#define DEFECTIVE_CHGING_IC_SN_YYMM		(21 << 8 | 10 << 0)
#define ADC_3V3_SAMPLE				(2660)		// R1=2.2Mom, R2=680Kom, 3.3V
#define ADC_3V2_SAMPLE				(2600)		// R1=2.2Mom, R2=680Kom, 3.3V


/*       The Domain of Pin Definitions         */
#define DETECTION_BQ21080_INT_PIN	   NRF_GPIO_PIN_MAP(0, 30) //low=Charging,  otherwise High. 25122A        
#define ADC_BAT_VOL_PIN				NRF_SAADC_INPUT_AIN3 // AIN2 ADC. BATVol.    see NRF_SAADC_INPUT_AIN2

//#define BATT_CTRL_CURRENT_CD25120A	NRF_GPIO_PIN_MAP(0, 6) // X ? HSIv8
//#define BATT_IS_CHARGE_PG25120A		NRF_GPIO_PIN_MAP(1, 9) //low=Charging,  otherwise High. 25122A          
//#define BATT_INTERRUPT_INT_25120A	NRF_GPIO_PIN_MAP(0, 8) // INT25120A LDO1 of CPC4051 vol out control
//#define BATT_SAMPLE_LSCTRL                     4
	
//#define ADC_BAT_VOL_PIN	NRF_GPIO_PIN_MAP(0, 4) // AIN2 ADC. BATVol.
//#define ADC_CHG_VOL_PIN	NRF_GPIO_PIN_MAP(0, 5) // AIN3 ADC. CHGVol.

	void saadc_run(bool create_timer);
	void saadc_stop(void);

#ifdef __cplusplus  
}
#endif  
#endif /* BATTERY_ADC_H */  


