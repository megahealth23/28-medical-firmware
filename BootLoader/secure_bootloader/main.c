/**
 * Copyright (c) 2016 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup bootloader_secure_ble main.c
 * @{
 * @ingroup dfu_bootloader_api
 * @brief Bootloader project main file for secure DFU.
 *
 */

#include <stdint.h>
#include "boards.h"
#include "nrf_mbr.h"
#include "nrf_bootloader.h"
#include "nrf_bootloader_app_start.h"
#include "nrf_bootloader_dfu_timers.h"
#include "nrf_dfu.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "app_error.h"
#include "app_error_weak.h"
#include "nrf_bootloader_info.h"
#include "nrf_delay.h"

#include "nrf_gpio.h"
#include "nrf_drv_gpiote.h"

#include "nrf_bootloader_info.h"
#include "eeg_battery.h"
#include "app_timer.h"

static void on_error(void)
{
    NRF_LOG_FINAL_FLUSH();

#if NRF_MODULE_ENABLED(NRF_LOG_BACKEND_RTT)
    // To allow the buffer to be flushed by the host.
    nrf_delay_ms(100); 
#endif
	
#ifdef NRF_DFU_DEBUG_VERSION
    NRF_BREAKPOINT_COND;
#endif
    NVIC_SystemReset();
}


void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    NRF_LOG_ERROR("%s:%d", p_file_name, line_num);
    on_error();
}


void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    NRF_LOG_ERROR("Received a fault! id: 0x%08x, pc: 0x%08x, info: 0x%08x", id, pc, info);
    on_error();
}


void app_error_handler_bare(uint32_t error_code)
{
    NRF_LOG_ERROR("Received an error: 0x%08x!", error_code);
    on_error();
}

/**
 * @brief Function notifies certain events in DFU process.
 */
static void dfu_observer(nrf_dfu_evt_type_t evt_type)
{
    switch (evt_type)
    {
        case NRF_DFU_EVT_DFU_FAILED:
        case NRF_DFU_EVT_DFU_ABORTED:
        case NRF_DFU_EVT_DFU_INITIALIZED:
            //bsp_board_init(BSP_INIT_LEDS);

            //bsp_board_led_on(BSP_BOARD_LED_0);
            //bsp_board_led_on(BSP_BOARD_LED_1);
            //bsp_board_led_off(BSP_BOARD_LED_2);
            
            //bsp_board_led_off(BSP_BOARD_LED_1);

            //bsp_board_led_on(BSP_BOARD_LED_2);

            //bsp_board_led_off(BSP_BOARD_LED_2);

            break;
        case NRF_DFU_EVT_TRANSPORT_ACTIVATED:
            //bsp_board_led_on(BSP_BOARD_LED_2);
            //bsp_board_led_off(BSP_BOARD_LED_2);
            break;

        case NRF_DFU_EVT_DFU_STARTED:
            break;
        default:
            break;
    }
}

#if 0
#define DETECTION_5V_IN_PIN			NRF_GPIO_PIN_MAP(0, 5) //low=Charging,  otherwise High. 25122A        
static void battChg_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	if (pin == DETECTION_5V_IN_PIN) {
		if ( nrf_drv_gpiote_in_is_set(DETECTION_5V_IN_PIN)) {// low, NOCHARGE
			NVIC_SystemReset();
		}
	}
}
/*********************************************************************
* @fn      battChgPin_init
* @brief   Initialize the battery charge pin.
*		   callbalk.
*/
static bool battChgPin_init(void)
{
	uint32_t err_code;

	if (!nrfx_gpiote_is_init())
	{
		err_code = nrfx_gpiote_init();
		NRF_LOG_INFO("nrfx_gpiote_init=%d", err_code);
		APP_ERROR_CHECK(err_code);
	}
	
	nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);
	in_config.pull = 	NRF_GPIO_PIN_NOPULL;	
	err_code = nrf_drv_gpiote_in_init(DETECTION_5V_IN_PIN, &in_config, battChg_handler);

	//APP_ERROR_CHECK(err_code);
	if (err_code != NRF_SUCCESS)
	{
		NRF_LOG_INFO("battery charge pin config fail, err_code=%d", err_code);
		return false;
	}
	NRF_LOG_INFO("battery charge pin config ok");

	nrf_drv_gpiote_in_event_enable(DETECTION_5V_IN_PIN, true);

	return true;
}
#endif

#define APP_DFU_DATA_SETTING0	0xFD000
#define	BOOTLOADER_MAJOR_VER	0x20
#define	BOOTLOADER_SUB_VER		0x30

#define BOOT_VER	{'A', 'G', 'E', 'M', BOOTLOADER_MAJOR_VER|BOOTLOADER_SUB_VER};  
//const volatile char DFU_TAG[8] __attribute__((section(".ARM.__at_0xFD000"))) = BOOT_VER;

#include "nrf_drv_clock.h"
static void lfclk_init(void)
{
    uint32_t err_code;
    err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_clock_lfclk_request(NULL);
}

static void gpio_output_voltage_setup_3v3(void)
{
    if ((NRF_UICR->REGOUT0 & UICR_REGOUT0_VOUT_Msk) !=
        (UICR_REGOUT0_VOUT_3V3 << UICR_REGOUT0_VOUT_Pos))
    {
        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}

        NRF_UICR->REGOUT0 = (NRF_UICR->REGOUT0 & ~((uint32_t)UICR_REGOUT0_VOUT_Msk)) |
                            (UICR_REGOUT0_VOUT_3V3 << UICR_REGOUT0_VOUT_Pos);

        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
     
        NVIC_SystemReset();
    }
}

static void gpio_output_voltage_setup_2v1(void)
{
    if ((NRF_UICR->REGOUT0 & UICR_REGOUT0_VOUT_Msk) !=
        (UICR_REGOUT0_VOUT_2V1 << UICR_REGOUT0_VOUT_Pos))
    {
        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}

        NRF_UICR->REGOUT0 = (NRF_UICR->REGOUT0 & ~((uint32_t)UICR_REGOUT0_VOUT_Msk)) |
                            (UICR_REGOUT0_VOUT_2V1 << UICR_REGOUT0_VOUT_Pos);

        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
     
        NVIC_SystemReset();
    }
}

static void gpio_output_voltage_setup_2v4(void)
{
    if ((NRF_UICR->REGOUT0 & UICR_REGOUT0_VOUT_Msk) !=
        (UICR_REGOUT0_VOUT_2V4 << UICR_REGOUT0_VOUT_Pos))
    {
        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}

        NRF_UICR->REGOUT0 = (NRF_UICR->REGOUT0 & ~((uint32_t)UICR_REGOUT0_VOUT_Msk)) |
                            (UICR_REGOUT0_VOUT_2V4 << UICR_REGOUT0_VOUT_Pos);

        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
     
        NVIC_SystemReset();
    }
}

static void gpio_output_voltage_setup_2v7(void)
{
    if ((NRF_UICR->REGOUT0 & UICR_REGOUT0_VOUT_Msk) !=
        (UICR_REGOUT0_VOUT_2V7 << UICR_REGOUT0_VOUT_Pos))
    {
        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}

        NRF_UICR->REGOUT0 = (NRF_UICR->REGOUT0 & ~((uint32_t)UICR_REGOUT0_VOUT_Msk)) |
                            (UICR_REGOUT0_VOUT_2V7 << UICR_REGOUT0_VOUT_Pos);

        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
     
        NVIC_SystemReset();
    }
}


static void gpio_output_voltage_setup(uint8_t voltage)
{
    printf("1111 0x%x\r\n", NRF_UICR->REGOUT0 & UICR_REGOUT0_VOUT_Msk);
    if ((NRF_UICR->REGOUT0 & UICR_REGOUT0_VOUT_Msk) !=
        (voltage << UICR_REGOUT0_VOUT_Pos))
    {
        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Een;
        printf("2222 0x%x\r\n", NRF_NVMC->CONFIG);
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}

        NRF_NVMC->ERASEUICR = NVMC_ERASEUICR_ERASEUICR_Erase;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
        
        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
        printf("333 0x%x\r\n", NRF_UICR->REGOUT0 & UICR_REGOUT0_VOUT_Msk);

        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
        printf("4444 0x%x\r\n", NRF_NVMC->CONFIG);
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}

        NRF_UICR->REGOUT0 = (NRF_UICR->REGOUT0 & ~((uint32_t)UICR_REGOUT0_VOUT_Msk)) |
                            (voltage << UICR_REGOUT0_VOUT_Pos);

        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
        while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}

        printf("1111 0x%x\r\n", NRF_UICR->REGOUT0 & UICR_REGOUT0_VOUT_Msk);
     
        NVIC_SystemReset();
    }
}

/**@brief Function for application main entry. */
int main(void)
{
    uint32_t ret_val;

    // Direct to system off. Test.  Linwei  20121202
    //nrf_power_system_off();

  //bsp_board_init(0);
  lfclk_init();
  //gpio_output_voltage_setup_3v3();
  //gpio_output_voltage_setup_2v1();
        gpio_output_voltage_setup(UICR_REGOUT0_VOUT_2V1);
  //gpio_output_voltage_setup_2v7();

  /* By default, the LDO regulators are enabled and the DC/DC regulators are disabled. */
  /* When a DC/DC converter is enabled, the LDO for the corresponding regulator stage will be disabled.
  External LC filters must be connected for each of the DC/DC regulators being used. The advantage of
  using a DC/DC regulator is that the overall power consumption is normally reduced as the efficiency of
  such a regulator is higher than that of a LDO. */

  sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);

    // Protect MBR and bootloader code from being overwritten.
    ret_val = nrf_bootloader_flash_protect(0, MBR_SIZE, false);
    APP_ERROR_CHECK(ret_val);
    ret_val = nrf_bootloader_flash_protect(BOOTLOADER_START_ADDR, BOOTLOADER_SIZE, false);
    APP_ERROR_CHECK(ret_val);

    (void) NRF_LOG_INIT(nrf_bootloader_dfu_timer_counter_get);
    NRF_LOG_DEFAULT_BACKENDS_INIT();

	//battChgPin_init();
	
    NRF_LOG_INFO("Inside main");

    ret_code_t err_code;
  err_code = app_timer_init();
  APP_ERROR_CHECK(err_code);
  //battery_init();

    ret_val = nrf_bootloader_init(dfu_observer);
    APP_ERROR_CHECK(ret_val);

    // Either there was no DFU functionality enabled in this project or the DFU module detected
    // no ongoing DFU operation and found a valid main application.
    // Boot the main application.
    nrf_bootloader_app_start();

    // Should never be reached.
    NRF_LOG_INFO("After main");
}

/**
 * @}
 */
