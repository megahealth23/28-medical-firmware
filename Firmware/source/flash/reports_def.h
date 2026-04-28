#ifndef __REPORTS_DEF_H__
#define __REPORTS_DEF_H__
#include "fds.h"

#pragma pack(1)
typedef enum e_report_type{
    e_report_sleep,
    e_report_daily,
    e_report_sport,
    e_report_professional_sleep,
    e_report_professional_stress,
    e_report_auto_sport,
    e_report_low_power_sleep,
    e_report_unknown,
} E_REPORT_TYPE;

typedef enum e_report_data_type{
    e_report_data_min_sec,
    e_report_data_hrv,
} E_REPORT_DATA_TYPE;

typedef enum e_report_end_type{
    e_report_end_by_timer,
    e_report_end_by_auto,
    e_report_end_by_user,
    e_report_end_by_over_time,
    e_report_end_by_over_flow,
    e_report_end_by_reset,
    e_report_end_by_charging,
    e_report_end_by_low_power,
    e_report_end_by_fault,
} E_REPORT_END_TYPE;

enum
{
    REPORT_ERR_ID_FULL = FDS_ERR_INTERNAL+1, 
    REPORT_ERR_FALSH_FULL, 
    REPORT_ERR_ID_INVALID,
    REPORT_ERR_KEY_INVALID,
    REPORT_ERR_DATA_NULL,
    REPORT_ERR_IN_READ_PROC,
};
#pragma pack()

#define INVALID_REPORT_ID           0
#define INVALID_REPORT_KEY          0
#define REPORT_BUFF_MAX_LEN         512

#define INVALID_HR                  0
#define INVALID_SPO2                0
#define INVALID_TEMP                0
#define INVALID_ACC                 0
#define INVALID_STEP                0

#define REPORT_PROTOCOL_ID          3

#pragma  pack(1)
typedef struct {
    uint16_t occupied_len;
    uint8_t buff[REPORT_BUFF_MAX_LEN];
}ST_REPORT_BUFF;
#pragma pack() 

#endif