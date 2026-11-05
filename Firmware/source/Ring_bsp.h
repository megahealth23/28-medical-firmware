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

/*
FLASH电源EN
VFLASH-SW	P0.04	OUTPUT	软件1使能开启电压
*/
#define VFLASH_PIN NRF_GPIO_PIN_MAP(0, 4) // 1 Enable
#define FLASH_H() nrf_gpio_pin_set(VFLASH_PIN);   // Enable
#define FLASH_L() nrf_gpio_pin_clear(VFLASH_PIN); // Disable

/* SPI communication with extern flash
PIN			IO PROT	Direction	STATUS
QSPI_CS		P1.08	OUTPUT		硬件默认上拉1
QSPI_CLK	P0.19	OUTPUT
QSPI_DIO0	P0.20	IO
QSPI_DIO1	P0.21	IO

QSPI_DIO2	P0.26	IO			硬件默认上拉1
QSPI_DIO3	P0.12	IO			硬件默认上拉1
*/
// QSPI_CS low active

#define QSPI_CS NRF_GPIO_PIN_MAP(1, 8)   // Default 1 deactive.
#define QSPI_CLK NRF_GPIO_PIN_MAP(0, 19) // NO external pull-up or pull-down
#define QSPI_DIO0 NRF_GPIO_PIN_MAP(0, 20)
#define QSPI_DIO1 NRF_GPIO_PIN_MAP(0, 21)
#define QSPI_DIO2 NRF_GPIO_PIN_MAP(0, 26) // External pull-up
#define QSPI_DIO3 NRF_GPIO_PIN_MAP(0, 12) // External pull-up

#define EXT_FLASH_SPI_CS QSPI_CS
#define EXT_FLASH_SPI_SCK QSPI_CLK
#define EXT_FLASH_SPI_MOSI QSPI_DIO0
#define EXT_FLASH_SPI_MISO QSPI_DIO1

#define EXT_FLASH_SPI_HOLD QSPI_DIO3
#define EXT_FLASH_SPI_WP QSPI_DIO2

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

