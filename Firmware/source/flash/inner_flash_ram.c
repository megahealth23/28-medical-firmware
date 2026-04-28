#include "hardfault.h"
#include "nrf_log.h"
#include "inner_flash_ram.h"
#include "ring_utc.h"
#include "system_clock.h"
#include "ring_power.h"
#include "nrf_fstorage.h"
#include "nrf_fstorage_sd.h"
#include "nrf_nvmc.h"
#include "crc16.h"

#ifndef __GNUC__
#warning "USING ARMCC"
	NIRAM_t NIramCfg  __attribute__((at(NIRAM_BASE)));
#else
#pragma message("USING GNUC")
	NIRAM_t NIramCfg __attribute__((section(".retained_section")));
#endif

__ALIGN(sizeof(uint32_t))
struct st_dump dump = {0};

#define G_NI_FLASH_MARK (23456)

productMacSn g_mac_sn = {0};
ST_NI_FLASH_DATA g_NI_flash = {0};

user_descriptor* user_desp = &g_NI_flash.user; // UsrInfo

#define NI_FLASH_DATA_START_ADDR    (NI_FLASH_START_ADDR + 0x1000)
#define NI_FLASH_END_ADDR       (NI_FLASH_START_ADDR + USER_FLASH_SIZE -1)

static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt);
NRF_FSTORAGE_DEF(nrf_fstorage_t fstorage) =
{
    .evt_handler    = fstorage_evt_handler,
    .start_addr     = NI_FLASH_START_ADDR,
    .end_addr       = NI_FLASH_END_ADDR,
};

/**@brief   Sleep until an event is received. */
static void power_manage(void)
{
#ifdef SOFTDEVICE_PRESENT
    (void) sd_app_evt_wait();
#else
    __WFE();
#endif
}

static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt)
{
    if (p_evt->result != NRF_SUCCESS)
    {
        NRF_LOG_INFO("--> Event received: ERROR while executing an fstorage operation.");
        return;
    }

    switch (p_evt->id)
    {
        case NRF_FSTORAGE_EVT_WRITE_RESULT:
        {
            NRF_LOG_INFO("--> Event received: wrote %d bytes at address 0x%x.",
                         p_evt->len, p_evt->addr);
        } break;

        case NRF_FSTORAGE_EVT_ERASE_RESULT:
        {
            NRF_LOG_INFO("--> Event received: erased %d page from address 0x%x.",
                         p_evt->len, p_evt->addr);
        } break;

        default:
            break;
    }
}

void wait_for_flash_ready(nrf_fstorage_t const * p_fstorage)
{
    /* While fstorage is busy, sleep and wait for an event. */
    while (nrf_fstorage_is_busy(p_fstorage))
    {
        power_manage();
    }
}


static void print_flash_info(nrf_fstorage_t * p_fstorage)
{
    NRF_LOG_INFO("========| flash info |========");
    NRF_LOG_INFO("erase unit: \t%d bytes",      p_fstorage->p_flash_info->erase_unit);
    NRF_LOG_INFO("program unit: \t%d bytes",    p_fstorage->p_flash_info->program_unit);
    NRF_LOG_INFO("==============================");
}

ret_code_t update_NI_flash_by_nvmc(){
    g_NI_flash.mark = G_NI_FLASH_MARK;
    NRF_LOG_INFO("crc start is %04X", g_NI_flash.crc);
    g_NI_flash.crc = crc16_xmodem(&g_NI_flash, sizeof(ST_NI_FLASH_DATA) - sizeof(g_NI_flash.crc));
    NRF_LOG_INFO("crc end is %04X", g_NI_flash.crc);
    nrf_nvmc_page_erase(NI_FLASH_DATA_START_ADDR);
    nrf_nvmc_write_bytes(NI_FLASH_DATA_START_ADDR, &g_NI_flash, sizeof(ST_NI_FLASH_DATA));

    return NRF_SUCCESS;
}

ret_code_t update_NI_flash(){
    g_NI_flash.mark = G_NI_FLASH_MARK;
    NRF_LOG_INFO("crc start is %04X", g_NI_flash.crc);
    g_NI_flash.crc = crc16_xmodem(&g_NI_flash, sizeof(ST_NI_FLASH_DATA) - sizeof(g_NI_flash.crc));
    NRF_LOG_INFO("crc end is %04X", g_NI_flash.crc);

    wait_for_flash_ready(&fstorage);
    ret_code_t rc = nrf_fstorage_erase(&fstorage, NI_FLASH_DATA_START_ADDR, 1, NULL);
    if (rc != NRF_SUCCESS)
    {
        NRF_LOG_INFO("nrf_fstorage_erase() returned: %s\n",
                        nrf_strerror_get(rc));
        return rc;
    }

    wait_for_flash_ready(&fstorage);

     rc = nrf_fstorage_write(&fstorage, NI_FLASH_DATA_START_ADDR, &g_NI_flash, sizeof(ST_NI_FLASH_DATA), NULL);
    if (rc != NRF_SUCCESS)
    {
        NRF_LOG_INFO("nrf_fstorage_write() returned: %s\n",
                        nrf_strerror_get(rc));
    }
    wait_for_flash_ready(&fstorage);

    return rc;
}

ret_code_t update_NI_flash_local(){
    wait_for_flash_ready(&fstorage);
    ret_code_t rc = nrf_fstorage_erase(&fstorage, NI_FLASH_START_ADDR, 1, NULL);
    if (rc != NRF_SUCCESS)
    {
        NRF_LOG_INFO("nrf_fstorage_erase() returned: %s\n",
                        nrf_strerror_get(rc));
        return rc;
    }

    wait_for_flash_ready(&fstorage);

     rc = nrf_fstorage_write(&fstorage, NI_FLASH_START_ADDR, &g_mac_sn, sizeof(g_mac_sn), NULL);
    if (rc != NRF_SUCCESS)
    {
        NRF_LOG_INFO("nrf_fstorage_write() returned: %s\n",
                        nrf_strerror_get(rc));
    }
    wait_for_flash_ready(&fstorage);

    return rc;
}

void update_NI_flash_nvmc_4_dfu(){
    g_NI_flash.is_dfu = 1;
    NRF_LOG_INFO("update_NI_flash_nvmc_4_dfu crc start is %04X", g_NI_flash.crc);
    g_NI_flash.crc = crc16_xmodem(&g_NI_flash, sizeof(ST_NI_FLASH_DATA) - sizeof(g_NI_flash.crc));
    NRF_LOG_INFO("update_NI_flash_nvmc_4_dfu crc end is %04X", g_NI_flash.crc);
    nrf_nvmc_page_erase(NI_FLASH_DATA_START_ADDR);
    nrf_nvmc_write_bytes(NI_FLASH_DATA_START_ADDR, &g_NI_flash, sizeof(ST_NI_FLASH_DATA));
}

void update_NI_flash_nvmc_4_initiactive_hard_reset(){
    g_NI_flash.initative_hard_reset = INITIATIVE_HARD_RESET;
    NRF_LOG_INFO("update_NI_flash_nvmc_4_initiactive_hard_reset crc start is %04X", g_NI_flash.crc);
    g_NI_flash.crc = crc16_xmodem(&g_NI_flash, sizeof(ST_NI_FLASH_DATA) - sizeof(g_NI_flash.crc));
    NRF_LOG_INFO("update_NI_flash_nvmc_4_initiactive_hard_reset crc end is %04X", g_NI_flash.crc);

    wait_for_flash_ready(&fstorage);
    ret_code_t rc = nrf_fstorage_erase(&fstorage, NI_FLASH_DATA_START_ADDR, 1, NULL);
    if (rc != NRF_SUCCESS)
    {
        NRF_LOG_INFO("nrf_fstorage_erase() returned: %s\n",
                        nrf_strerror_get(rc));
        return rc;
    }

    wait_for_flash_ready(&fstorage);

     rc = nrf_fstorage_write(&fstorage, NI_FLASH_DATA_START_ADDR, &g_NI_flash, sizeof(ST_NI_FLASH_DATA), NULL);
    if (rc != NRF_SUCCESS)
    {
        NRF_LOG_INFO("nrf_fstorage_write() returned: %s\n",
                        nrf_strerror_get(rc));
    }
    wait_for_flash_ready(&fstorage);
}

void update_NI_ram_4_dfu(){
    memset(NIramCfg.sec_report_ids, 0, sizeof(NIramCfg.sec_report_ids));
    NIramCfg.cur_daily_report_id = INVALID_REPORT_ID;
    NIramCfg.last_daily_report_key = INVALID_REPORT_KEY;
    NIramCfg.cur_sec_report_id = INVALID_REPORT_ID;
    NIramCfg.last_sec_report_key = INVALID_REPORT_KEY;
    NIramCfg.last_hrv_report_key = INVALID_REPORT_KEY;
    NIramCfg.last_daily_report_hrv_key = INVALID_REPORT_KEY;
}

void set_NI_ram_charging_start(uint32_t start_time, uint8_t start_percent){
    NIramCfg.last_charge_time = start_time;
    NIramCfg.last_charge_percent = start_percent;
}

uint32_t get_NI_ram_charging_start_time(){
    return NIramCfg.last_charge_time;
}

uint32_t get_NI_ram_charging_start_percent(){
    return NIramCfg.last_charge_percent;
}

void set_NI_ram_charged_percent(uint8_t percent){
    NIramCfg.charged_percent = percent;
}

uint8_t get_NI_ram_charged_percent(){
    return NIramCfg.charged_percent;
}

ret_code_t restore_flash_from_dfu(){
    if(g_NI_flash.is_dfu == 1 || g_NI_flash.mark != G_NI_FLASH_MARK){
        printf("restore_flash_from_dfu() start\r\n");
        for(int i = 1; i <= FDS_VIRTUAL_PAGES; i++){
            nrf_nvmc_page_erase(0x000EE000-i*0x1000);
        }

        g_NI_flash.is_dfu = 0;
        g_NI_flash.cur_daily_report_id = INVALID_REPORT_ID;
        g_NI_flash.cur_sec_report_id = INVALID_REPORT_ID;
        memset(g_NI_flash.sec_report_ids, 0, sizeof(g_NI_flash.sec_report_ids));
        update_NI_flash_by_nvmc();

        NRF_LOG_INFO("restore_flash_from_dfu() end");
    }

    return NRF_SUCCESS;
}

static bool need_refresh_flash = false;
void init_NI_flash(){
    /* Initialize an fstorage instance using the nrf_fstorage_sd backend.
     * nrf_fstorage_sd uses the SoftDevice to write to flash. This implementation can safely be
     * used whenever there is a SoftDevice, regardless of its status (enabled/disabled). */
    nrf_fstorage_api_t * p_fs_api;
    p_fs_api = &nrf_fstorage_sd;

    ret_code_t rc = nrf_fstorage_init(&fstorage, p_fs_api, NULL);
    APP_ERROR_CHECK(rc);

    print_flash_info(&fstorage);

    rc = nrf_fstorage_read(&fstorage, NI_FLASH_START_ADDR, &g_mac_sn, sizeof(g_mac_sn));
    if (rc != NRF_SUCCESS)
    {
        NRF_LOG_INFO("nrf_fstorage_read() returned: %s\n",
                        nrf_strerror_get(rc));
        return;
    }

    rc = nrf_fstorage_read(&fstorage, NI_FLASH_DATA_START_ADDR, &g_NI_flash, sizeof(ST_NI_FLASH_DATA));
    if (rc != NRF_SUCCESS)
    {
        NRF_LOG_INFO("nrf_fstorage_read() returned: %s\n",
                        nrf_strerror_get(rc));
        return;
    }

    uint16_t crc = 0;
    printf("crc check start is %04X\r\n", g_NI_flash.crc);
    crc = crc16_xmodem(&g_NI_flash, sizeof(ST_NI_FLASH_DATA)-sizeof(g_NI_flash.crc));
    printf("crc check end is %04X\r\n", crc);
    NRF_LOG_FLUSH();
    NRF_LOG_PROCESS();
    printf("g_NI_flash.initative_hard_reset: %d\r\n", g_NI_flash.initative_hard_reset);
    if(crc != g_NI_flash.crc /*|| g_NI_flash.initative_hard_reset == INITIATIVE_HARD_RESET*/){
        printf("nrf_nvmc_page_erase() \r\n");
        for(int i = 1; i <= FDS_VIRTUAL_PAGES; i++){
            nrf_nvmc_page_erase(0x000EE000-i*0x1000);
        }
        memset(&g_NI_flash, 0, sizeof(ST_NI_FLASH_DATA));
        update_NI_flash_by_nvmc();
    }else{
        rc = restore_flash_from_dfu();
    }
}

void reset_max_sn(uint8_t* data){
    memcpy(g_mac_sn.mac, data, 6);
    memcpy(&g_mac_sn.sn, data+6, 6);
    update_NI_flash_local_CMD();
}

void set_auto_sleep(uint8_t* data){
    NIramCfg.auto_sleep_mark = data[0];
    NIramCfg.auto_sleep_timezone_hour = data[1];
    NIramCfg.auto_sleep_timezone_min = data[2];
    NIramCfg.auto_sleep_start_hour = data[3];
    NIramCfg.auto_sleep_start_min = data[4];
    NIramCfg.auto_sleep_end_hour = data[5];
    NIramCfg.auto_sleep_end_min = data[6];

    NRF_LOG_INFO("auto sleep mark : 0x%02X", 
        NIramCfg.auto_sleep_mark);

    NRF_LOG_INFO("auto sleep timezone : 0x%02X:0x%02X, start : %d:%d, end : %d:%d", 
        NIramCfg.auto_sleep_timezone_hour,
        NIramCfg.auto_sleep_timezone_min,
        NIramCfg.auto_sleep_start_hour,
        NIramCfg.auto_sleep_start_min,
        NIramCfg.auto_sleep_end_hour,
        NIramCfg.auto_sleep_end_min);
}

void set_auto_stress(uint8_t* data){
    NIramCfg.auto_stress_mark = data[0];
}

void set_high_precision_mark(uint8_t* data){
    NIramCfg.high_precision_mark = data[0];
    //if(NIramCfg.low_power_mark == 0){
    //    NIramCfg.auto_stress_mark = 1;
    //}else{
    //    NIramCfg.auto_stress_mark = 0;
    //}
}

extern uint32_t s_sys_ni_flash_evt;
void update_NI_flash_CMD(){
    s_sys_ni_flash_evt = (SYS_MODULE << 24) | (SYS_UPDATE_NI_FLASH<<16);
    sEvent evt = { SYS_QID, &s_sys_ni_flash_evt};
    taskPush(&evt, false);
}

void update_NI_flash_local_CMD(){
    static sys_module_evt = (SYS_MODULE << 24) | (SYS_UPDATE_NI_FLASH_LOCAL<<16);
    sEvent evt = { SYS_QID, &sys_module_evt};
    taskPush(&evt, false);
}

void updateNIClock() // save clock in nvram.
{
    NIramCfg.systme_sec = app_get_sec();
    NIramCfg.utc_sec = UTC_getClock();
    return;
}

bool need_hard_restart(){
    int time_interval = NIramCfg.utc_sec - NIramCfg.last_start_time;
    printf("\r\nutc_sec=%u, last_start_time: %u, time_interval is %d\r\n", NIramCfg.utc_sec, NIramCfg.last_start_time, time_interval);
    NIramCfg.last_start_time = NIramCfg.utc_sec;

    nrf_delay_ms(500);
    if(NIramCfg.utc_sec <= 100 && NIramCfg.last_start_time <= 100)
        return false;

    if(time_interval < 50 && time_interval > -50)
        return true;

    return false;
}

void initNIRAM() {
    uint8_t *pFlag = "NIRA";
    uint8_t *p = NIramCfg.NIflag;
    uint32_t uSec = NIramCfg.utc_sec;
    uint32_t last = NIramCfg.last_start_time;

    uint32_t test = sizeof(NIRAM_t);
    //printf("uSec=%u, last: %d, Flag: %s, size is %d\r\n", uSec, last, p, sizeof(NIramCfg));

    printf("last charge time is %d, now is %d, batt is %d\r\n", NIramCfg.last_charge_time, NIramCfg.utc_sec, NIramCfg.last_charge_percent);
    // 946656000 2000-01-01 00:00:00 UTC+8
    // 2003500800 2033-06-28 00:00:00 UTC+8
#if 1
    if (!memcmp(p, pFlag, 4) && ((uSec >= 946656000 && uSec <= 2003500800))){
        app_set_sec(uSec);
        return;
    }
#endif

    NRF_LOG_INFO("Need init NIramCfg, uSec=%u", uSec);

    memset(&NIramCfg, 0, sizeof(NIRAM_t));
    memcpy(&NIramCfg.NIflag[0], pFlag, 4);
    NIramCfg.utc_sec = 946656000;
    NIramCfg.last_start_time = 0;
    NIramCfg.last_charge_time = NIramCfg.utc_sec;
    NIramCfg.last_charge_percent = 20;
    NIramCfg.charged_percent = 70;
    app_set_sec(NIramCfg.utc_sec);

    memcpy(NIramCfg.sec_report_ids, g_NI_flash.sec_report_ids, sizeof(g_NI_flash.sec_report_ids));
    //NIramCfg.cur_daily_report_id = g_NI_flash.cur_daily_report_id;
    NIramCfg.cur_daily_report_id = g_NI_flash.cur_daily_report_id;
    NIramCfg.cur_sec_report_id = INVALID_REPORT_ID;

}

static void saveNIRAM() // NIRAM_BASE , len 0x800
{
    NIramCfg.systme_sec = app_get_sec();
    NIramCfg.utc_sec = UTC_getClock();

    memcpy(&NIramCfg.dump, &dump, sizeof(struct st_dump));
    return;
}

static void restoreNIRAM() {
    uint32_t uSec = ++NIramCfg.utc_sec;

    UTC_setClock(uSec);

    uint32_t u = UTC_getClock();
    NRF_LOG_WARNING("restoreNIRAM utc=%u", u);
    memcpy(&dump, &NIramCfg.dump, sizeof(struct st_dump));
    // dailyRecord_t *pDR = &(NIramCfg.dailyR); No need save or restore.
}

void SaveCrash(struct st_dump *dp) {
    if (dp != NULL)
        memcpy(&dump, (const uint32_t *)dp, sizeof(dump_t));

    saveNIRAM();
}

void safeStop(uint8_t stopType) {
    SaveCrash(NULL);
    saveNIRAM();
}

void safeRestore() {
    restoreNIRAM();
}

/**
 * @brief  This function handles Hard Fault exception.
 * @param  hardfault_args
 * @retval None
 */
void HardFault_process(HardFault_stack_t *p_stack) {
    UTCTime t = UTC_getClock();
    __ALIGN(sizeof(uint32_t))
    struct st_dump dp = {HARDDEFAULT_ERROR, t, p_stack->lr, p_stack->pc};
    SaveCrash(&dp);

    ring_systemreset_entry();
    NVIC_SystemReset();
}

#ifndef DEBUG
void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info) {
    // On assert, the system can only recover with a reset.
    error_info_t *error_info = (error_info_t *)info;
    dump.error_type = NRF_ERROR_RESET;
    dump.time = UTC_getClock();
    dump.lr = id;
    dump.pc = pc;
    SaveCrash(NULL);

    ring_systemreset_entry();

    NVIC_SystemReset();
}
#endif // DEBUG