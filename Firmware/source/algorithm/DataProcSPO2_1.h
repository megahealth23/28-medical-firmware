#ifndef DATAPROCSPO2_H
#define DATAPROCSPO2_H

#include "../gesture/mring_gesture.h"
#include "HRI.h"
#include "afe\AFE4900\afe4900_defs.h"
#include "arm_math.h"
#include "fifo.h"
#include "math.h"
#include "mem_manager.h"
#include <stdbool.h>
#include <stdint.h>

#define zero_datalength 100 // datalength of passzero;
#define acdc_datalength 80  // datalength of acdc calculate;
#define hrr_datalength 40   // datalength of hr;
#define MS_datalength 10    // datalength of MotionState
#define WAVE_PROC_LEN 1000
#define WAVELET_LEVEL 8
#define MeanDataNUM 30
#define MARLEN 0
#define rangofk 2100
#define SPO2_HR_VALID 0
#define SPO2_HR_READYING 1
#define SPO2_HR_INVALID 2
#define OFFLINE_SN 0x1304 // 19->13,04->04
#define OFFLINE_THRE 5000
#define MIN_ADRATE 0.48
#define MAX_ADRATE 2.0
#define MAX_HIST_LEN 600
#define ACC_104_SAMPLERATE 104
#define ACC_13_SAMPLERATE 13
#define ACC_Z_LENGTH (ACC_13_SAMPLERATE * 3)

#define CHRGING_INDICATE_CURRENT 0x00000C

#define AFE_LED1ENDC_VAL (1499)
#define AFE_CLKDIV_PRF (0)

#define LEDRED_ONHAND_MAXV 10000
#define OFFLINE_STATICACC 15
#define OFFLINE_LED_ON 1
#define OFFLINE_LED_OFF 0

#define test_data_num 200
#define length_wavelet 1000
#define length_rawdata 1050
#define length_recover (length_rawdata - test_data_num)
#define length_accbuf 104

#define MAX_HRV_CNT 250 // 256  // 5 * 60(1 min)
#define MAXNUM_HRVvect 260
#define MAX_RR_Cnt 8

#define MAX_BREATH_CNT 60 // 60min

//#define AFE4405_FIFO_NUM		(2)
#define AFE4900_ARRAY_SIZE PPG_FIFO_PERIOD
typedef struct _afe4900_fifo_data {
  struct {
    int ledGreen;
    int ledRed;
    int ledInfra;
    int ledambient1;
  } recipes[AFE4900_ARRAY_SIZE];
} afe4900_fifo_data;

enum SLEEP_STAGES {
  INVALID_STAGE,
  WAKE_STAGE,
  SLEEP_STAGE
};

typedef struct {
  uint8_t spo2;     // spo2 data for display
  uint8_t hr;       //
  uint8_t flag;     // flag:BIT0- adc ready; BIT1-can go to LPM;BIT2-start sleep detection
  uint8_t Spo2show; // spo2 data from different parameters
  uint8_t shr;
  uint8_t ehr_hr;
  uint8_t ehr_hrorig;
  uint8_t ehr_flag;
  uint32_t time;
  uint8_t spo2_error;
  uint8_t tiredflg;
  uint8_t offhand_flg;
  uint8_t rr_var;
  uint8_t breathrate;
  enum SLEEP_STAGES sleep_status;
} T_AFE_DATA;
extern T_AFE_DATA g_afe;

typedef struct {
  uint8_t is_offline;
  uint8_t is_online;
  int offline_num;
  int min_ambval;
} DETECT_OFFLINE;

typedef struct {
  int ACCdatanum;
  int accstfp;
  int ACC_g;
  int pre_ACC_g;
  uint8_t movement;
  uint8_t movetime;
  int acc_znum;
  int16_t ACCx;
  int16_t ACCy;
  int16_t ACCz;
} ACCData;
extern ACCData accdata;

typedef float AlgData_t;
typedef struct
{
  int Proccnt;
  AlgData_t Spo2;
  AlgData_t SSpo2;
  uint8_t heartratehist;
  uint8_t heartrate;
  uint8_t Sheartrate;
  uint8_t Acc;
  uint8_t tiredflg;
  uint8_t HrCnt;
  uint8_t HrVect[MAX_RR_Cnt];
} AlgProc_Para;

typedef struct
{
  uint8_t red_reg;
  uint8_t infra_reg;
} LED_Current_Level;
extern LED_Current_Level led_reg;

enum afe4405_pattern_t {
  AFE4405_SPO2_PATTERN,
  AFE4405_EHR_PATTERN,
  AFE4405_CHARGING_PATTERN,
  AFE4405_BP_PATTERN,
  AFE4405_PULSE_IMP_PATTERN
};

extern int test_accval;
extern int test_afe_proc;
extern int length_accval;
extern uint8_t accval;
extern bool LED1readyfor_calc;
extern bool LED2readyfor_calc;
extern uint8_t redvalflag;
extern int test_greenval;
extern uint8_t HRV_vect[MAXNUM_HRVvect];
extern uint32_t HRV_timestamp;
extern uint8_t acc_flag;
extern uint8_t BR_vect[MAX_BREATH_CNT];
extern uint32_t BR_timestamp;

extern bool afe_spo2_init(void);
extern bool afe_pulse_init(void);
extern int afe_spo2_malloc(void);
extern void afe_spo2_free(void);
extern void Accproc_init(void);
extern void afe_charging_led_indicate(int ledreg);

extern uint8_t afe_spo2_proc_wavelate(void);
extern void afe_wave_free(void);
extern void offwrist_init(void);
extern void offwrist_Detection(int ambient, int diffval);
extern void offwrist_NigtDetection(int redreg, int redval);
extern uint8_t data_compress16(int src_data, uint8_t flag, uint8_t flag_spo2);
extern uint8_t SPO2_bufProc_wavelet(void *afe_in, uint8_t frame_length);
extern uint8_t PULSE_bufProc_wavelet(void *afe_in, uint8_t frame_length);
extern void ble_send_afe_raw_data(int reddata[], int greendata[], int num);
extern int get_var(int *input, int inlen);
extern int Detect_ohhand(int LED1val);
extern void light_green_led(int value);
extern int get_int_mean(int *input, int inlen);
extern uint8_t HRV_isready_flg(void);
extern void HRV_cnt_clear(void);
extern int offline_proc(int *ledamb, int len);
extern uint8_t pulse_offline_flag(void);
extern uint8_t get_spo2_procnum(void);
extern void init_spo2_procnum(void);
extern uint32_t get_spo2_calculnum(void);
extern uint8_t BR_isready_flg(void);
extern void BR_cnt_clear(void);
extern uint8_t BR_stopsaving(void);

#endif /* AFE_DRV_H */