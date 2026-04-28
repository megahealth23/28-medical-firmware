#ifndef __REPORTS_H__
#define  __REPORTS_H__

#include <sdk_common.h>
#include "core_cm4.h"
#include "fds.h"
#include "reports_def.h"


/**@brief Function for initialize reports module.
 *
 * The function initialize FDS, gc FDS files
 *
 * @retval     E_REPORT_RESULT_CODE         
 */
ret_code_t report_init();

/**@brief Function for start get a report
 *
 * The function get report data from report which id is report_id, data length is length,
 * if report is exist.
 * if report data length is less than length, only copy data full size.
 * reports module record this report and current read position, use report_get_continue to get remaind data.
 *
 * @param[in]       report_id           report id for get.
 * @param[out]      data                memory for write data.
 * @param[in-out]   length              in . data memory length.
 *                                      out . write data length.
 *
 * @retval     E_REPORT_RESULT_CODE         
 */

uint16_t report_start_read_file(uint16_t report_id);
ret_code_t report_start_read_block(uint16_t report_id, uint16_t report_key, uint16_t* pLen, uint16_t* pCrc);
//void report_read_block_buff(uint8_t* pData, uint32_t* len);
ret_code_t reports_clear_all(void);
ret_code_t report_remove(uint16_t report_id);
ret_code_t report_sec_stop(E_REPORT_TYPE type, E_REPORT_END_TYPE end_type, uint16_t *report_id);
ret_code_t report_push_data_hrv(uint8_t* rr, uint16_t length, E_REPORT_TYPE type);
ret_code_t report_push_data_sec(uint8_t hr, int8_t temp, uint8_t acc, uint8_t spo2, uint16_t step, E_REPORT_TYPE type);
void clear_monitor_report_ram();
void clear_daily_report_ram();
void report_start_sync_block();
bool is_flash_full();
ret_code_t report_push_data_daily(uint8_t hr, uint8_t temp, uint8_t acc, uint8_t spo2, uint16_t step, uint8_t eletric);
ret_code_t report_push_data_daily_hrv(uint8_t* rr, uint16_t length, E_REPORT_TYPE type);
ret_code_t report_daily_stop(E_REPORT_END_TYPE end_type);
ret_code_t report_daily_save();
uint16_t get_current_sec_report_id();
#endif
