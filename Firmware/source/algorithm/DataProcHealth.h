#pragma once
#include"stdio.h"
#include"stdlib.h"
#include"string.h"
#include "math.h"
#include "mem_manager.h"
#include "fifo.h"
//using namespace std;

#define RESULT_SAVED_MINUTES (60*24)
//#define MAX_HIST_LEN 600
#define VECT_OUT_LEN 200

#define HRV_PROCSECLEN 300


#define HRV_PROCESS_DATA_HEART_PERIOD (100)

#define HRV_PROCESS_DATA_PERIOD_NEW (1000)//(200)
#define HRV_PROCESS_DATA_PERIOD HRV_PROCESS_DATA_PERIOD_NEW//(1000)
#define HRV_PROCESS_DATA_PERIOD_NOW_ALGORITHM (200)
#define HRV_PROCESS_DATA_PERIOD_VALID_THRESHOLD (350)//(300)
#define HRV_PROCESS_DATA_VALID_PEAK_THRESHOLD (4500)
#define HRV_PROCESS_DATA_VALID_PEAK_THRESHOLD_TWO (15000)
#define HRV_PROCESS_DATA_VALID_PEAK_THRESHOLD_NEW (2000)

#define VALID_PERIOD (1)
#define INVALID_PERIOD (0)
#define HRV_SMOOTH_SPAN (10)

#define PPG_DATA_WAVELET_LEVEL_NEW (9)//(6)
#define PPG_DATA_WAVELET_LEVEL PPG_DATA_WAVELET_LEVEL_NEW//(9)

#define HRV_TIME_PARA_SMOOTH_SPAN (5)

#define PPG_DATA_WAVELET_START_LEVEL (PPG_DATA_WAVELET_LEVEL - 5)//(4)
#define PPG_DATA_WAVELET_END_LEVEL (PPG_DATA_WAVELET_LEVEL - 1)//(8)

#define VAILD_PEAK_HIGH_THRESHOLD (700)

#define LENGTH_RAWDATA (1000)

#define PPG_SAMPLE (100)
#define STRESS_ACC_NUM (2*HRV_PROCESS_DATA_PERIOD/PPG_SAMPLE)
#define STRESS_VAILD_THRESHOLD (7)//ACC:0-63

#define EXERCISE_ACC_NUM (15*5)//(15*5)//(15*3)//(20*HRV_PROCESS_DATA_PERIOD/PPG_SAMPLE)
#define START_EXERCISE_MODE (1)
#define END_EXERCISE_MODE (0)
#define EXERCISE_VAILD_ACC_THRESHOLD (70)//(100)//



#define SLEEP_OFFHAND_NUM (30)//(60*5)//s
#define SLEEP_ACC_NUM (60*7)//(60*5)//(60*2)//(60*10)//s

#define START_SLEEP_MODE (1)
#define END_SLEEP_MODE (0)
#define NO_CHECK_SLEEP_MODE (-1)
#define SLEEP_VAILD_ACC_THRESHOLD (1)//(5)
#define SLEEP_END_VAILD_ACC_THRESHOLD (10)//(20)
#define AUTO_DAILY_SLEEP_CHECK_FLAG (1)


#define WAKE_INDIRECT_MOVE_THRESHOLD (35)//(40)
#define WAKE_INDIRECT_MOVE_CNT (12)//(8)


#define SLEEP_KEEP_TIME_ONE (30*60)//0.5h
#define SLEEP_KEEP_TIME_TWO (60*60)//T=1h
#define SLEEP_KEEP_TIME_THREE (2*60*60)//1h< T <= 2h
#define SLEEP_KEEP_TIME_FOUR (3*60*60) // 2h < T <= 3h

#define SLEEP_KEEP_TIME_FIVE (6*60*60) // 2h < T <= 3h

#define SLEEP_END_JUDGE_TIME_ONE (60) //S
#define SLEEP_END_JUDGE_TIME_TWO (2*60)//S
#define SLEEP_END_JUDGE_THREE_THREE (4*60)//S
#define SLEEP_END_JUDGE_THREE_FOUR (5*60)//S
#define SLEEP_END_JUDGE_THREE_FIVE (7*60)//S

#define SLEEP_END_ACC_NUM_ONE (15)//s
#define SLEEP_END_ACC_NUM_TWO (30)//s
#define SLEEP_END_ACC_NUM_THREE (60)//s
#define SLEEP_END_ACC_NUM_FOUR (90)//s
#define SLEEP_END_ACC_NUM_FIVE (120)//(150)//s


#define HEALTH_NON_PROFESSIONAL_TYPE (0)
#define HEALTH_PROFESSIONAL_TYPE (1)
#define NON_PROFESSIONAL_SPORT_HEALTH_PROCESS_TIME (30*PPG_SAMPLE)//30 second//1minutes//(3*60*PPG_SAMPLE)//3minutes
#define PROFESSIONAL_SPORT_HEALTH_PROCESS_TIME (5*60*PPG_SAMPLE)//5minutes

#define HRV_HEALTH_PROCESS_STOP_FLAG (0)
#define HRV_HEALTH_PROCESS_START_FLAG (1)
#define HRV_HEALTH_PROCESS_FINISH_FLAG (2)
#define HRV_MAX_DATA_LEN (1500)//5minutes RR MAX num

#define HRV_MAX_PEAKS_DATA_LEN (HRV_PROCESS_DATA_PERIOD)//5minutes RR MAX num

/*溢出标志：0-正常，-1-溢出*/
#define FLAGS_OVERRUN		0x0001  

#ifndef MIN
#define MIN(x1,x2) ((x1) < (x2) ? (x1):(x2))
#define MAX(x1,x2) ((x1) > (x2) ? (x1):(x2))
#endif

typedef float AlgData_t;

#if 1
typedef struct
{
	int cnt;//分析的心搏数
	AlgData_t meanbpm;//平均心率
	AlgData_t RRmean;
	AlgData_t SDNN;//
	AlgData_t SDANN;//
	AlgData_t RMSSD;//
	int NN50;//
	AlgData_t pNN50;//
	AlgData_t TrangleIdx;//三角指数
	int MaxRR;//最大RR间隔
	int MaxRRTimeStamp;//最大RR间隔发生时间
	int MinBpm;//最慢心率
	int MinBpmTimeStamp;//最慢心率发生时间
	int MaxBpm;//最快心率
	int MaxBpmTimeStamp;//最快心率发生时间
	int FastBpmCnt;//心动过速博数
	AlgData_t FastBpmRate;//心动过速占比
	int SlowBpmCnt;//心动过缓博数
	AlgData_t SlowBpmRate;//心动过缓占比
}Time_Para;


typedef struct
{
	float VLFP;//
	float LFP;//
	float HFP;//
	float LHFR;//
}Freq_Para;

typedef struct
{
	float SD1; //µ¥Î»ms^2
	float SD2; //µ¥Î»ms^2
	float SDRate;
}Geom_Para;

typedef struct{
	int acc_len;
	uint8_t acc_value[STRESS_ACC_NUM];

}Stress_Acc_Para;
typedef struct{
	int acc_len;
	int acc_value[EXERCISE_ACC_NUM];

}Exercise_Acc_Para;


typedef struct{
	int acc_len;
	uint8_t acc_value[SLEEP_ACC_NUM];

}Sleep_Acc_Para;






#pragma aram pack(1)
typedef struct
{
	int starttime;//测试起始时间
	int lasttime;//测试持续时间
	Time_Para tTimePara;//时域参数
	Freq_Para tFreqPara;
	Geom_Para tGeomPara;//¼¸ºÎ²ÎÊý
	int SDNNCnt;//SDNN数目
	float* SDNNVect;//SDNN数组
	int HRcnt;//心率数目
	float* HRVect;//心率数组
	int HistVCnt; //直方图数组长度
	int* HistVect;//直方图数组
	int HistVect_avg;//直方图数组平均数
	int FreqVCnt; //频率显示数组长度
	float* FreqVect;//频率显示数组
	float fatigueIndex;//疲劳指数
	float stressIndex;//精神压力指数
	uint16_t green_RR[HRV_MAX_DATA_LEN];//RR间隔的数据
	uint16_t RR_num;//RR间隔的数目
}HRV_Para;
#pragma pack()


//typedef struct {
//	int* buf;
//	int putP;// 下一个数据写入位置
//	int size;// 大小
//	int flags;
//} FIFO32;

#endif

void stress_modules_heart_get(int* greenData, int dataLen, uint8_t* heartRate);

HRV_Para* hrv_health_process_result_get(void);
void hrv_health_process_flag_stop(void);
void hrv_health_process_flag_set(uint8_t flag);
uint8_t hrv_health_process_flag_get(void);
void smooth_data_int_out_int(int* frame_data, int smooth_span, int data_len, int* frame_data_smooth);
void smooth_data(int* frame_data, int smooth_span, int data_len, float* frame_data_smooth);
void smooth_data_16uint(uint16_t* frame_data, int smooth_span, int data_len, float* frame_data_smooth);
void smooth_data_f(float* frame_data, int smooth_span, int data_len, float* frame_data_smooth);
void PPG_data_wavelet(float* PPGData, int inLen, float* PPGDataWavelet);
void Init_ProcPara();
void PPG_data_RR_peaks_get(float* PPGData, int DataLen, uint16_t* RRData, int* RRLen);
void PPG_data_RR_get(uint16_t* RRData, int RRLen, HRV_Para* hrv_health_para_out);
void get_HrvTimePara(float* rr, int inlen, Time_Para* TimePara);
void get_HrvGeomPara(AlgData_t* input, int inlen, Geom_Para* GeomPara);
void HRV_health_data_get(void);
void HRV_health_data_get_daily_algorithm_rr(void);
// void HRV_health_data_get_new(void);
uint8_t PPG_data_RR_peaks_get_new(float* PPGData, int DataLen, uint16_t* RRData, int* RRLen);
void PPG_data_RR_get_new(uint16_t* RRData, int RRLen, HRV_Para* hrv_health_para_out);

void stress_acc_get(uint8_t acc);
uint8_t start_circle_proc_hrv(uint8_t type);

void Exercise_judge_acc_get(int acc);
uint8_t acc_get_Exercise_flag(void);
void ACC_Exercise_judge_data_clear(void);

void Sleep_end_judge_acc_get(uint8_t acc);
void Sleep_start_judge_acc_get(uint8_t acc);

uint8_t acc_get_Sleep_flag(void);
void ACC_Sleep_judge_data_clear(void);
// void HRV_health_data_get_low_power(void);

void daily_sleep_offhand_flag_set(uint8_t flag);
uint8_t daily_sleep_offhand_flag_get(void);


#if 0
int fifo32_recover(FIFO32* fifo, int num);
int fifo32_putPut(FIFO32* fifo, int data);
int fifo32_status(FIFO32* fifo);

#endif


