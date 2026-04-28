#ifndef _DATA_PROC_EHR_H_
#define _DATA_PROC_EHR_H_


#include "math.h"
#include "arm_math.h"
//#include "mring_queue.h"
#include "fifo.h"
#include "DataProcSPO2.h"
#include <stdbool.h>

#define AFE_SAMPLERATE  100
#define MAXFRAMENUM		2100
#define MAX_FFT_SIZE	2048
#define FFT_PROC_LEN    2048 
#define Ch_FFT_SIZE     2100
#define Ch_ACC_SIZE     2200
#define FFT_MAX_OUT_LEN 110
#define ehr_data_num    100
#define ehr_recover     (MAXFRAMENUM-ehr_data_num)
#define ehr_reg22       0x000003
#define off_dc_time     200
typedef short int16_t;


#define PPG_SAMPLE (100)
#define HR_FFT_SEARCH_LEN (3)
#define HR_FFT_SEARCH_LEN_TWO (6)
#define HR_FFT_SEARCH_LEN_THREE (3)
#define FFT_SEARCH_LEN (80)
#define FFT_LENGTH (2048)
#define SPORT_HR_SPAN (5)//(8)//(15)//(5)
#define SPORT_REST_HR_SPAN (15)//(8)//(15)//(5)
#define SPORT_HR_DATA_LEN (2048)
#define SPORT_DATA_TIME (20)
#define REST_FLAG_DATA_LEN (500)//(300)
#define SECOND_DATA_LEN (100)
#define SPORT_MIN_HEART (40)
#define SPORT_MAX_HEART (220)
#define REST_FLAG_DATA_MAX_PEAKS (REST_FLAG_DATA_LEN/2)
#define REST_FLAG_DATA_MOVE_LEN (100)
#define REST_FLAG (1)
#define REST_MAX_RR_NUM (20)//(10)
#define ACC_MOVE_THRESHOLD (10000)

#define SPORT_MIN_RR (27)
#define SPORT_MAX_RR (150)
#define SPORT_ALG_DATA_PERIOD (100)//1s
// #define SPORT_MIN_HEART (40)
// #define SPORT_MAX_HEART (220)

#define OFF_HAND_OPEN (0)
#define OFF_HAND_CLOSE (1)


#define POINT_DAILY_HEART_PROCESS_TIME (20)


typedef struct
{
	uint16_t green_RR[REST_MAX_RR_NUM];//RR intervel data
	uint16_t RR_num;//RR intervel num

}RR_DATA_RECORD;



typedef struct
{
    uint8_t offline_start_flg;
    uint8_t error_data_num;
    uint8_t is_offline;
    uint8_t is_online;
}EHR_OFFLINE;

typedef struct
{
	int MvPos;
	float Accpwr;
	float showHr;
	int cnt;
	int Hrpos[4];
	float Hrvld[4];
}MOVE_HR_Para;

#define MAX_SAVED_HIST_LEN 10
typedef struct
{
	float MinHrpeace;
	float MaxHrmove;
	float VldScale;
	int SavePos;
	uint8_t HistMvPos;
	uint8_t HistHr;
	uint8_t MvLastTime;
	uint8_t KeepCnt;
	MOVE_HR_Para HistParaVect[MAX_SAVED_HIST_LEN];
}MOVE_HR_HISTPara;

typedef struct
{
	int samplerate;
	arm_rfft_fast_instance_f32 fftPara;
}ProcPara;

typedef struct
{
    int max_led1_val;
//    int max_amb1_val;
}Off_Dac_Para;
//typedef struct
//{
//	float a;
//	float b;
//	float c;
//	float d;
//	float e;
//}fftdata;

//typedef struct
//{
//	int movecnt;
//	int staticcnt;
//	int constaticlen;
//	int conmovelen;
//}EHR_STATE;


extern ProcPara DataProcPara;
extern FIFO32 EHR_green_fifo;
//extern FIFO_acc EHR_acc_fifo;
//extern Queue Red_Queue;
//extern Queue Green_Queue;
//extern Queue2 Acc_Queue;
//extern MOVE_HR_HISTPara GHistPara;
extern int datano;
extern uint8_t test_moveflag;
extern uint8_t test_ehr_flg;
extern MOVE_HR_Para GPara;
extern int test_datano;
extern uint8_t green_sp;
extern uint8_t acc_sp;


extern void off_hand_check_set(uint8_t flag);
extern uint8_t off_hand_check_get(void);

extern void hr_algorithm_time_acc_sample_num_claer(void);
extern uint32_t hr_algorithm_time_acc_sample_num_get(void);

extern bool afe_ehr_init(void);

extern void afe_ehr_proc(void);
//extern void EHR_Proc_shedule(Queue2 *ch_acc, Queue *ch_green, int samplerate, unsigned char *heartrate);

extern uint8_t EHR_data_bufProc(void *afe_bpt, uint8_t frame_length);

extern int EHR_malloc(void);
extern int DAILY_point_EHR_malloc(void);

extern void EHR_free(void);

extern int get_EHR_step(void);

extern uint8_t point_data_bufProc(void *afe_in, uint8_t frame_length, uint8_t *offhand_flag);

extern uint8_t afe_point_proc(void);
extern void sport_heart_algorithm_init(void);

extern void get_MoveHr_sport_daily_point(int* chg, int16_t* acc, int inlen, MOVE_HR_HISTPara* HPara, float* ShowHr);
extern void get_MoveHr_sport_daily_point_new(int* chg, int16_t* acc, int inlen, MOVE_HR_HISTPara* HPara, float* ShowHr);
#endif
