#ifndef  __STATE_MODULE_DEF_H__
#define  __STATE_MODULE_DEF_H__
#include "sdk_errors.h"

#define STATE_MODULE_SUCCESS                (NRF_SUCCESS)
#define STATE_MODULE_STATE_INVALID          (NRF_SUCCESS+1)
#define STATE_MODULE_ACC_FAILED             (NRF_SUCCESS+2)
#define STATE_MODULE_AFE_FAILED             (NRF_SUCCESS+3)
#define STATE_MODULE_HW_CHECK_FAILED        (NRF_SUCCESS+4)
#define STATE_MODULE_FLASH_FULL             (NRF_SUCCESS+5)
#define STATE_MODULE_TEMP_FAILED            (NRF_SUCCESS+6)
#define STATE_MODULE_STRESS_START_FAILED    (NRF_SUCCESS+7)

typedef enum {
    e_IDLE = 0,
    e_LP,                       //low power
    e_CHARG,                    //charging
    e_PA,                       //pysical active
    e_PS,                       //professional sleep
    e_EXER,                     //exercise
    e_DS,                       //daily sleep
    e_DAILY,                    //daily active
    e_REAL,                     //real time
    e_STRESS,                   //stress test
    e_LPS,                      //low power sleep
    e_BP,                       //blood pressure
} E_STATE_MODULE;

typedef enum {
    e_sub_run_indoor = 0,
    e_sub_run_outdoor,
    e_sub_bicycle_outdoor,
    e_sub_swimming,
    e_sub_strength,
    e_sub_adl,
    e_sub_6mwt,
    e_sub_walk_outdoor,

} E_SUB_STATE_EXERCISE;

#define NORMAL_NOTIFY_LEN   20

#define DAILY_MARK_PERIOD  (10*60)// (10*60) //seconds


#define DAILY_STRESS_MARK_PERIOD   (60*60)//(2*60*60)//(30*60) //seconds
#define AUTO_SLEEP_CHECK_PERIOD     (2*60+30)
#define DAILY_MARK_MAX_DURATION     (60)
#define DAILY_STRESS_MARK_MAX_DURATION     (35) //35 second
#define MAX_SLEEP_DURATION          (12*60*1000)
#define MAX_EXERCISE_DURATION       (10*60*1000)
#define MAX_BP_DURATION             (10*1000)

#define REALTIME_RESERVED_TIME  20 //senconds

#endif
