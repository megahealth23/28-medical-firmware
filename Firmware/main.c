/*********************************************************************************
 * Copyright (c) 2019 MegaHealth. All Rights Reserved.
 * main.c
 * AUTHOR:
 * DATE:
 *
 */
#include "app_error.h"
#include "app_timer.h"
#include "nrf_ble_lesc.h"
#include "nrf_drv_rng.h"
#include "nrf_drv_wdt.h"
#include "nrf_pwr_mgmt.h"

#include "hardfault.h"

#include "ble_ring.h"
#include "ble_service.h"
#include "ring_bsp.h"

#include "mTask.h"
#include "ring_bsp.h"
#include "system_clock.h"

#include "acc\ring_acc.h"
#include "battery\ring_battery.h"
#include "nrf_drv_clock.h"
#include "protocol.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "ble_rawdata.h"
#include "low_pwm.h"
#include "ring_swtimer.h"

#include "ring_afe.h"
#include "ring_tmp.h"
#include "ring_utc.h"
#include "nrf_nvmc.h"
#include "nrf_dfu.h"

#include "I2C.h"
#include "ring_startup.h"
#include "tools.h"

#include "ble_ring.h"
#include "mem_manager.h"
#include "inner_flash_ram.h"
#include "peripheral/peripherals.h"
// APP_TIMER_DEF(m_notify_timer);

#define NRF_CPU_ID(ID)                         \
  do {                                         \
    memcpy(ID, (void *)NRF_FICR->DEVICEID, 8); \
  } while (0);

static sn_bsp_map_t sn_bsp_map[] = {
    {.sn_yymm = (21 << 8 | 7 << 0), .bsp_ver = BSP_ZG28_MAIN_V4}, // 2021-07: BSP_ZG28_MAIN_V4
    {.sn_yymm = (21 << 8 | 8 << 0), .bsp_ver = BSP_ZG28_HSI_V1},  // 2021-08: BSP_ZG28_HSI_V1
    {.sn_yymm = (21 << 8 | 9 << 0), .bsp_ver = BSP_ZG28_HSI_V1},  // 2021-09: BSP_ZG28_HSI_V1
                                                                  /***************
                                                                          TODO:
                                                                          { .sn_yymm = (yy<<8|mm<<0), .bsp_ver = BSP_ZG28_HSI_V2}, // 20yy-mm: BSP_ZG28_HSI_V2
                                                                          { .sn_yymm = (yy<<8|mm<<0), .bsp_ver = BSP_ZG28_HSI_V3}, // 20yy-mm: BSP_ZG28_HSI_V3
                                                                          { .sn_yymm = (yy<<8|mm<<0), .bsp_ver = BSP_ZG28_HSI_V4}, // 20yy-mm: BSP_ZG28_HSI_V4
                                                                  ****************/
};

/**@brief BLE buttonless handler.
 *
 */
/*
static void ble_dfu_evt_handler(ble_dfu_buttonless_evt_type_t event)
{
        switch (event)
        {
        case BLE_DFU_EVT_BOOTLOADER_ENTER_PREPARE:
                NRF_LOG_INFO("Device is preparing to enter bootloader mode\r\n");
                break;

        case BLE_DFU_EVT_BOOTLOADER_ENTER:
                NRF_LOG_INFO("Device will enter bootloader mode\r\n");
                break;

        case BLE_DFU_EVT_BOOTLOADER_ENTER_FAILED:
                NRF_LOG_ERROR("Device will enter bootloader mode\r\n");
                break;
        default:
                NRF_LOG_INFO("Unknown event from ble_dfu.\r\n");
                break;
        }
}
*/

nrf_drv_wdt_channel_id m_channel_id;

void wdt_feed(void) {
  // feed watch dog
  nrf_drv_wdt_channel_feed(m_channel_id);
}

/**@brief Handler for shutdown preparation.
 *
 * @details During shutdown procedures, this function will be called at a 1 second interval
 *          untill the function returns true. When the function returns true, it means that the
 *          app is ready to reset to DFU mode.
 *
 * @param[in]   event   Power manager event.
 *
 * @retval  True if shutdown is allowed by this power manager handler, otherwise false.
 */
static bool app_shutdown_handler(nrf_pwr_mgmt_evt_t event) {
  switch (event) {
  case NRF_PWR_MGMT_EVT_PREPARE_DFU:{
    NRF_LOG_INFO("Power management wants to reset to DFU mode......");
    sd_softdevice_disable();
    //nrf_nvmc_page_erase(0x000EE000-FDS_VIRTUAL_PAGES*0x1000);
    //for(int i = 1; i <= FDS_VIRTUAL_PAGES; i++){
    //    nrf_nvmc_page_erase(0x000EE000-i*0x1000);
    //    wdt_feed();
    //}

    update_NI_ram_4_dfu();
    update_NI_flash_nvmc_4_dfu();

    NRF_LOG_INFO("Power management wants to reset to DFU mode.");
    NRF_LOG_PROCESS();
    }
    break;

  default:
    return true;
  }

  NRF_LOG_INFO("Power management allowed to reset to DFU mode.");
  return true;
}

NRF_PWR_MGMT_HANDLER_REGISTER(app_shutdown_handler, 0);

static void buttonless_dfu_sdh_state_observer(nrf_sdh_state_evt_t state, void *p_context) {
  if (state == NRF_SDH_EVT_STATE_DISABLED) {
    // Softdevice was disabled before going into reset. Inform bootloader to skip CRC on next boot.
    nrf_power_gpregret2_set(BOOTLOADER_DFU_SKIP_CRC);

    // Go to system off.
    nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
  }
}

/* nrf_sdh state observer. */
NRF_SDH_STATE_OBSERVER(m_buttonless_dfu_state_obs, 0) =
    {
        .handler = buttonless_dfu_sdh_state_observer,
};

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void) {
  ret_code_t err_code;

  // Initialize timer module.
  err_code = app_timer_init();
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the nrf log module.
 */
static inline uint32_t get_rtc_counter(void) {
  return NRF_RTC1->COUNTER;
}

static void log_init(void) {
  ret_code_t err_code = NRF_LOG_INIT(get_rtc_counter); // Add timestamp in RTT log.
  APP_ERROR_CHECK(err_code);

  NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing power management.
 */
static void power_management_init(void) {
  ret_code_t err_code;
  err_code = nrf_pwr_mgmt_init();
  APP_ERROR_CHECK(err_code);
}

/**
 * @brief WDT events handler.
 */
struct exception_frame_type {
  uint32_t r0;
  uint32_t r1;
  uint32_t r2;
  uint32_t r3;
  uint32_t r12;
  uint32_t lr;
  uint32_t pc;
  uint32_t xpsr;
};

#if defined(__CC_ARM)
__asm int return_sp() {
  MOV r0, sp
              BX LR
}
#endif
static void wdt_event_handler(void) {
  static uint8_t *pc;
  static uint8_t *isp;
  static uint32_t sp;

#if defined(__SES_ARM)
  __asm volatile("mov %0, sp"
                 : "=r"(isp)
                 :
                 :);
#else // KEIL
  sp = return_sp();
  isp = (uint8_t *)sp;
#endif

  pc = isp + 0x40;
#if 1
  UTCTime t = UTC_getClock();
  __ALIGN(sizeof(uint32_t))
  struct st_dump dp = {WATCHDOG_RESET, t, 0, *(uint32_t *)pc}; // PC
  SaveCrash(&dp);

  ring_systemreset_entry();

  NVIC_SystemReset();
#else
  safeStop(STOP_WATCHDOG);
  NVIC_SystemReset();
  // NOTE: The max amount of time we can spend in WDT interrupt is two cycles of 32768[Hz] clock - after that, reset occurs
#endif
}

void wdt_init(void) {
  // Configure WDT.
  ret_code_t err_code;
  nrf_drv_wdt_config_t config = NRF_DRV_WDT_DEAFULT_CONFIG;
  err_code = nrf_drv_wdt_init(&config, wdt_event_handler);
  APP_ERROR_CHECK(err_code);
  err_code = nrf_drv_wdt_channel_alloc(&m_channel_id);
  APP_ERROR_CHECK(err_code);
  nrf_drv_wdt_enable();
}

productMacSn pms;
void getPMS() {
  extern productMacSn g_mac_sn;

  pms.ftFlag = htons(g_mac_sn.ftFlag);
  NRF_LOG_INFO("ftFlag = %d", pms.ftFlag);

  memcpy(&pms.mac[0], g_mac_sn.mac, 6);

  memcpy(&pms.sn, &g_mac_sn.sn, 6);

  NRF_LOG_INFO("Mac: %2X:%2X:%2X:%2X:%2X:%2X", pms.mac[0], pms.mac[1], pms.mac[2],
      pms.mac[3], pms.mac[4], pms.mac[5]);

  NRF_LOG_INFO("SN: version %x, year_5 %x, year_1 %x, month %x, reserved %x", pms.sn.VYM.snVer, pms.sn.VYM.year_5,
    pms.sn.VYM.year_1, pms.sn.VYM.month, pms.sn.VYM.reserved);

  NRF_LOG_INFO("SN: type %x, size %x, mark %x", pms.sn.SizeType.type, pms.sn.SizeType.size, 
    pms.sn.SizeType.mark);

  return;
}

/**@brief Function for application main entry.
 */
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

        printf("1111 0x%x\r\n", NRF_UICR->REGOUT0 & UICR_REGOUT0_VOUT_Msk);
     
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

static void dcdc_enable(void){
  NRF_POWER->DCDCEN = POWER_DCDCEN_DCDCEN_Enabled;
  NRF_POWER->DCDCEN0 = POWER_DCDCEN0_DCDCEN_Enabled;

  nrf_delay_ms(2);
}

//static void open_led(void){
//  int pin = NRF_GPIO_PIN_MAP(0, 5);
//  nrf_gpio_cfg(pin,
//					NRF_GPIO_PIN_DIR_OUTPUT,
//					NRF_GPIO_PIN_INPUT_DISCONNECT,
//					NRF_GPIO_PIN_PULLUP,
//					NRF_GPIO_PIN_S0D1,
//					NRF_GPIO_PIN_NOSENSE);

//	nrf_gpio_pin_clear(pin);
//}

//static void close_led(void){
//  int pin = NRF_GPIO_PIN_MAP(0, 5);
//  nrf_gpio_cfg(pin,
//					NRF_GPIO_PIN_DIR_OUTPUT,
//					NRF_GPIO_PIN_INPUT_DISCONNECT,
//					NRF_GPIO_PIN_PULLUP,
//					NRF_GPIO_PIN_S0D1,
//					NRF_GPIO_PIN_NOSENSE);

//	nrf_gpio_pin_set(pin);
//}

/**@brief Function for application main entry.
 */
int main(void) {
    bool erase_bonds; 
    uint32_t err_code;

#if CONFIG_JLINK_MONITOR_ENABLED
    NVIC_SetPriority(DebugMonitor_IRQn, _PRIO_SD_LOW);
#endif

    nrf_mem_init(); 

    //close_led();
    log_init();
    err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
    nrf_dfu_settings_t *p_dfu_settings = (nrf_dfu_settings_t *)BOOTLOADER_SETTINGS_ADDRESS;                     //
    //gRIval.btVer = p_dfu_settings->bootloader_version; // V8 boot should be 3.X ; 2<<4 | 0 = 2.0;
    printf("booload version is 0x%x\r\n", p_dfu_settings->bootloader_version);
    //if((p_dfu_settings->bootloader_version & 0x40) == 0x40){
    //    printf("aaaaaaaaaaaaaaaaaaaaaaaaaaa\r\n");
    //    gpio_output_voltage_setup(UICR_REGOUT0_VOUT_2V4);
    //}else{
        printf("cccccccccccccccccccccccccccccc\r\n");
        gpio_output_voltage_setup(UICR_REGOUT0_VOUT_2V7);
    //}
    //gpio_output_voltage_setup_3v3();
    //gpio_output_voltage_setup_2v1(); 
    //gpio_output_voltage_setup_2v4();
    //gpio_output_voltage_setup_2v7();
    timers_init();

    //int n = 1;
    //while(1){
    //NRF_LOG_INFO("test test test %d!!!", n++);
    //nrf_delay_ms(1000);

    //NRF_LOG_PROCESS();
    //}

    ring_swtimer_init();  //start software timer
    boardInit();    //start GPIO

    /* By default, the LDO regulators are enabled and the DC/DC regulators are disabled. */
    /* When a DC/DC converter is enabled, the LDO for the corresponding regulator stage will be disabled.
    External LC filters must be connected for each of the DC/DC regulators being used. The advantage of
    using a DC/DC regulator is that the overall power consumption is normally reduced as the efficiency of
    such a regulator is higher than that of a LDO. */
    //err_code = sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
    //NRF_ERROR_INVALID_PARAM;
    //err_code = sd_power_dcdc0_mode_set(NRF_POWER_DCDC_ENABLE);
    if((p_dfu_settings->bootloader_version & 0x40) != 0x40){
        dcdc_enable();
    }
    //dcdc_enable();
    nrf_delay_ms(2000);

    rtc_start();
    //UTC_init();

    init_NI_flash();
    getPMS();   //get mac and sn
    initNIRAM();

    PPG_board_power_off();
    nrf_delay_ms(5000);

    power_management_init(); //init 52840 power manager

    ble_service_init(); //start ble and advertising
    nrf_delay_ms(5000);

    NRF_LOG_PROCESS();
    
    //nrf_delay_ms(1000);
    //int i = 500;
    //while(i--){
    //    nrf_delay_ms(10);
    //    if (NRF_LOG_PROCESS() == false) {
    //        nrf_pwr_mgmt_run();
    //    }
    //}
    battery_init(); //init 28010 and set register

    //    chg_hard_reset();
    if(need_hard_restart()){
        //update_NI_flash_nvmc_4_initiactive_hard_reset();
        chg_hard_reset();
    }

    NRF_LOG_FLUSH();
    NRF_LOG_PROCESS();

    err_code = nrf_drv_rng_init(NULL);
    APP_ERROR_CHECK(err_code);

    ring_rawdata_init();

    uint32_t id[2];
    NRF_CPU_ID(id);
    NRF_LOG_INFO("cpu id is %08X%08X", id[0], id[1]);

    NRF_LOG_INFO("Application started.");

    NRF_LOG_INFO("SystemSec=%u, UTCsec=%u", app_get_sec(), UTC_getClock());

    NRF_LOG_PROCESS();


    wdt_init();
    PPG_board_power_on();

    app_startup();
}