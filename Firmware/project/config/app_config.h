/*
* Copyright (c) 2019 MegaHealth. All Rights Reserved.
* @file
* @date 2020.07.29
* @version
* @brief 
*/
#ifndef APP_CONFIG_H  
#define APP_CONFIG_H  
// <<< Use Configuration Wizard in Context Menu >>>\n

// <h> RTT moudle 

//==========================================================
// <o> RTT_SAMPLE_HZ  - ADS8867 adc sample - Control for ADS8867

#ifndef RTT_SAMPLE_HZ
#define RTT_SAMPLE_HZ 200
#endif
//==========================================================

// </h>

// <h> Fatfs module

//==========================================================
// <q> NRF_BLOCK_DEV_QSPI_ENABLED  - nrf_block_dev_qspi - QSPI block device
 

#ifndef NRF_BLOCK_DEV_QSPI_ENABLED
#define NRF_BLOCK_DEV_QSPI_ENABLED 0
#endif

// <q> NRF_BLOCK_DEV_RAM_ENABLED  - nrf_block_dev_ram - RAM block device

#ifndef NRF_BLOCK_DEV_RAM_ENABLED
#define NRF_BLOCK_DEV_RAM_ENABLED 0
#endif

// <q> NAND_BLOCK_DEV_SPI_ENABLED  - nand_block_dev_spi - SPI NAND block device

#ifndef NAND_BLOCK_DEV_SPI_ENABLED
#define NAND_BLOCK_DEV_SPI_ENABLED 1
#endif
//==========================================================

// <e> NAND_BLOCK_DEV_SPI_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef NAND_BLOCK_DEV_SPI_CONFIG_LOG_ENABLED
#define NAND_BLOCK_DEV_SPI_CONFIG_LOG_ENABLED 1
#endif
// <o> NAND_BLOCK_DEV_SPI_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NAND_BLOCK_DEV_SPI_CONFIG_LOG_LEVEL
#define NAND_BLOCK_DEV_SPI_CONFIG_LOG_LEVEL 3
#endif

// <o> NAND_BLOCK_DEV_SPI_CONFIG_LOG_INIT_FILTER_LEVEL  - Initial severity level if dynamic filtering is enabled
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef NAND_BLOCK_DEV_SPI_CONFIG_LOG_INIT_FILTER_LEVEL
#define NAND_BLOCK_DEV_SPI_CONFIG_LOG_INIT_FILTER_LEVEL 3
#endif

// <o> NAND_BLOCK_DEV_SPI_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NAND_BLOCK_DEV_SPI_CONFIG_INFO_COLOR
#define NAND_BLOCK_DEV_SPI_CONFIG_INFO_COLOR 0
#endif

// <o> NAND_BLOCK_DEV_SPI_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef NAND_BLOCK_DEV_SPI_CONFIG_DEBUG_COLOR
#define NAND_BLOCK_DEV_SPI_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <q> INNER_FLASH_BLOCK_DEV_ENABLED  - inner_flash_block_dev - INNER flash block device

#ifndef INNER_FLASH_BLOCK_DEV_ENABLED
#define INNER_FLASH_BLOCK_DEV_ENABLED 1
#endif
//==========================================================

// <e> INNER_FLASH_BLOCK_DEV_CONFIG_LOG_ENABLED - Enables logging in the module.
//==========================================================
#ifndef INNER_FLASH_BLOCK_DEV_CONFIG_LOG_ENABLED
#define INNER_FLASH_BLOCK_DEV_CONFIG_LOG_ENABLED 1
#endif
// <o> INNER_FLASH_BLOCK_DEV_CONFIG_LOG_LEVEL  - Default Severity level
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef INNER_FLASH_BLOCK_DEV_CONFIG_LOG_LEVEL
#define INNER_FLASH_BLOCK_DEV_CONFIG_LOG_LEVEL 3
#endif

// <o> INNER_FLASH_BLOCK_DEV_CONFIG_LOG_INIT_FILTER_LEVEL  - Initial severity level if dynamic filtering is enabled
 
// <0=> Off 
// <1=> Error 
// <2=> Warning 
// <3=> Info 
// <4=> Debug 

#ifndef INNER_FLASH_BLOCK_DEV_CONFIG_LOG_INIT_FILTER_LEVEL
#define INNER_FLASH_BLOCK_DEV_CONFIG_LOG_INIT_FILTER_LEVEL 3
#endif

// <o> INNER_FLASH_BLOCK_DEV_CONFIG_INFO_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef INNER_FLASH_BLOCK_DEV_CONFIG_INFO_COLOR
#define INNER_FLASH_BLOCK_DEV_CONFIG_INFO_COLOR 0
#endif

// <o> INNER_FLASH_BLOCK_DEV_CONFIG_DEBUG_COLOR  - ANSI escape code prefix.
 
// <0=> Default 
// <1=> Black 
// <2=> Red 
// <3=> Green 
// <4=> Yellow 
// <5=> Blue 
// <6=> Magenta 
// <7=> Cyan 
// <8=> White 

#ifndef INNER_FLASH_BLOCK_DEV_CONFIG_DEBUG_COLOR
#define INNER_FLASH_BLOCK_DEV_CONFIG_DEBUG_COLOR 0
#endif

// </e>

// <q> TEST_INNER_RAM_ENABLED  - test ram

#ifndef TEST_INNER_RAM_ENABLED
#define TEST_INNER_RAM_ENABLED 1
#endif

// </h>

// <<< end of configuration section >>>
#endif /* APP_CONFIG_H */  

