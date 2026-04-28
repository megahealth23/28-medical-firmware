/*********************************************************************************
 * Copyright (c) 2019 MegaHealth. All Rights Reserved.
 * Ring_bsp.h
 * AUTHOR:
 * DATE:
 *  From. ZG28-HSI20200715.xlsx
 */

#ifndef _V8_BSP_H
#define _V8_BSP_H

#ifdef __cplusplus  
extern "C"
{
#endif 

#include "app_gpiote.h"

		// ring board mappings
#define RING_SCL_PIN	NRF_GPIO_PIN_MAP(0, 24)	// SCL I2C_CLK
#define RING_SDA_PIN	NRF_GPIO_PIN_MAP(0, 23)	// SDA I2C_DAT

#define RING_SDA_PIN_1	NRF_GPIO_PIN_MAP(0, 20)	// SDA I2C_DAT

//#define RING_LED_TX	NRF_GPIO_PIN_MAP(0, 13)	// SCL I2C_CLK
//#define RING_LED_RX	NRF_GPIO_PIN_MAP(0, 14)	// SDA I2C_DAT


/** Pin define for HW_V2 **/
//#define GREEN_LED_2MA NRF_GPIO_PIN_MAP(0,12) // LED-2mA
//#define GREEN_LED_3MA NRF_GPIO_PIN_MAP(0, 7) // LED-3mA
//#define GREEN_LED_4MA NRF_GPIO_PIN_MAP(0,22) // LED-4mA
//#define GREEN_LED_7MA NRF_GPIO_PIN_MAP(1, 8) // LED-7mA

//#define AFE_PROG_PIN		NRF_GPIO_PIN_MAP(0,11)		// PROG pin. AFE4900
//#define AFE_SEN_PIN			NRF_GPIO_PIN_MAP(1, 4)		// I2C address selection of AFE4900

#define AFE_ADCREADY_PIN	NRF_GPIO_PIN_MAP(1, 0)	// ADCREADY pin. AFE4900
#define AFE_RESET_PIN		NRF_GPIO_PIN_MAP(1, 13)		// RESET pin. AFE4900

#define ACC_INT1_PIN	NRF_GPIO_PIN_MAP(0, 13)	// INT pin1 of BMI160
#define ACC_INT2_PIN	NRF_GPIO_PIN_MAP(0, 14)	// INT pin2 of BMI160

//#define TMP_INT1_PIN	NRF_GPIO_PIN_MAP(1, 3)	// Over temperature alert or data-ready sinal of TMP117-1
////NO external pull-up;software must make sure it is internally pulled-up
//#define TMP_INT2_PIN	NRF_GPIO_PIN_MAP(0, 30) // Over temperature alert or data-ready sinal of TMP117-2

	uint32_t boardInit(void);
	uint32_t accInt1init(void);
	uint32_t accInt2init(void);
	
	bool accInt1deinit(void);
	bool accInt2deinit(void);

	//  V8
	uint32_t afeFifoRdyinit(void);
	

#define RTC2_CONFIG_FREQUENCY    32		
#define RTC2_CONFIG_IRQ_PRIORITY APP_IRQ_PRIORITY_HIGH//APP_IRQ_PRIORITY_LOW
#define RTC2_CONFIG_RELIABLE     false
	
#define RTC2_INSTANCE_INDEX      (RTC0_ENABLED+RTC1_ENABLED)
	
	
//#define RTC_COUNT                (RTC0_ENABLED+RTC1_ENABLED+RTC2_ENABLED)
	
#define NRF_MAXIMUM_LATENCY_US 2000

#ifdef ZG28_V5_ENABLE

typedef enum bsp_version_e{
	BSP_ZG28_MAIN_V4 = 0/* include 2 TMP117 */,  
	BSP_ZG28_HSI_V1 = 1/* include 1 TMP117 */,
	MAX_BSP_VERSION_CNT,
}bsp_version_t;

#define  RING_BSP_VERSION		BSP_ZG28_HSI_V1   // used in ftmode

typedef struct sn_bsp_map_s{
/*
(21<<8|7<<0),BSP_ZG28_MAIN_V4;   21-07
(21<<8|8<<0),BSP_ZG28_HSI_V1;    21-08
(21<<8|9<<0),BSP_ZG28_HSI_V1; 	 21-09
*/
	uint16_t sn_yymm;
	bsp_version_t bsp_ver;
}sn_bsp_map_t;

void set_bsp_version(bsp_version_t bsp_ver);
bsp_version_t get_bsp_version(void);
#endif		

#ifdef __cplusplus  
}
#endif  
#endif /* _V8_BSP_H */  

