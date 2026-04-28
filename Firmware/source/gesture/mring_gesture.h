/* Copyright (c) 2016 MegaHealth. All Rights Reserved.
 * "mring_gesture.h"
 * AUTHOR:zhao mengshou
 * DATE:2018/8/15 14:19:14
 * http://www.megahealth.cn
 *
 */
#ifndef _MEING_GESTURE_H  
#define _MEING_GESTURE_H  
  
#ifdef __cplusplus  
extern "C"  
{  
#endif 

//#include "mring_queue.h"
#include <stdint.h>
#include "ring_acc.h"


#define ACC_SAMPLERATE		(50)
#define STEP_COUNT_PERIOD  (200)
#define SMOOTH_SPAN  (10)
#define PERIOD_NUM (3)
#define PERIOD_NUM_NEW (4)
#define KEEP_VAILD_STEP_NUM (12)//(6)//(10)//step and EHR is compatiblity by "KEEP_VAILD_STEP_NUM = 6"
#define KEEP_EHR_VAILD_STEP_NUM (4)//(3) // min stride frequency is 1 step/s

#define KEEP_PERIOD_DAILY_STEP_NUM (3)
#define KEEP_PERIOD_SPORT_STEP_NUM (2)


#define KEEP_PERIOD_DAILY_STEP_NUM_MAX (12)
#define KEEP_PERIOD_SPORT_STEP_NUM_MAX (8)

#define MAX_STEP_PER_SECOND (10)//(5)

#define STEP_PERIOD_VAILD_FLAG (1)
#define STEP_PERIOD_NO_VAILD_FLAG (0)

#define STEP_ACC_CHANGE_THRESHOLD (100)
#define STEP_ACC_CHANGE_THRESHOLD_TWO (70)
#define STEP_ACC_PEAK_MIN_THRESHOLD (1090)//(1100)//(1200)



#define STEP_FLAGS_OVERRUN		0x0001  
#define step_gbuf_length        (STEP_COUNT_PERIOD) //600
#define step_clc_datanum        300
#define gesture_clc_num         25
#define gesture_accbuf_length   200
#define gesture_angbuf_length   200
#define HIGH_FREQ               4
#define LOW_FREQ                0.6
#define accbuf_length           100
#define acc_error_data1         0x7fff
#define acc_error_data2         0
#define acc_error_data3         15000
#define defined_accg            980
#define gesture_start_datanum   300

#define AAC_RECORD_MAX_NUM (150)

typedef struct {
	int maxValue[STEP_COUNT_PERIOD];
	int maxValuePos[STEP_COUNT_PERIOD];
	int maxValueCnt;
}MAX_PARA;
typedef struct {
	int minValue[STEP_COUNT_PERIOD];
	int minValuePos[STEP_COUNT_PERIOD];
	int minValueCnt;
}MIN_PARA;

enum GESTURE_STATUS
{
	NONE,
	TAP_TWICE,
    TAP_THREETIMES,
    ROTATE_OUTANDIN,
    WAVE_UP_DOWN,
    WAVE_AROUND,
    DRAW_CIRCLE
};

typedef struct{
	uint16_t firstPos;
	uint16_t lastPos;
}PERIOD_PEAK_POS;
//typedef struct{  
//	uint16_t *buf;  
//	int putP;
//	int getP;
//	int size;
//	int free;
//	int flags;    
//} ACC_G_FIFO;

typedef struct{  
	int16_t *accx; 
    int16_t *accy;
    int16_t *accz;
    int16_t *gyrox; 
    int16_t *gyroy;
    int16_t *gyroz;
	int putP;
	int getP;
	int size;
	int free;
	int flags;    
} ACC_GYRO_FIFO;

typedef struct
{
	uint32_t stepNum;
	int intensity;
    uint32_t step_cnt;
}step_para;
extern step_para Stepara;

typedef struct
{
	uint8_t accnum;
	int stdaccval;
}ACC_PARA;
extern ACC_PARA  acc_para;
//extern int acc_buf[25];
//extern int acc_num;
typedef struct{

    short *buf;
	int putP;
	int size;
	int flags;
}FIFO_acc;
extern FIFO_acc EHR_acc_fifo;

typedef struct _gesture_warning
{
	int conti_time;		// current gesture status continue time from start, unit:sec
	int timeout;		// current gesture status continue max time, unit:sec
	uint32_t base_time;		// when gesture status update, record the system current time 
	enum GESTURE_STATUS curr_gesture;		// current gesture status
}gesture_warning_t;

#define DEFAULT_GESTURE_TIMEOUT		4

// BMI160  Xg=(9.81*raw)/sensitivity=(Grange*9.81*raw)/2^15
// So 4G range.  Xmg= 1000*4*9.81*raw/2^15= 9.81*raw/8192 = raw*1000/835= raw*0.122
//#define FS4G_TO_MG(raw) ((float_t)(raw) /835.0f)
//#define FS4G_TO_MG(x) ((float_t)(x) * 122.0f / 1000.0f)

#define FS4G_TO_MG(x) ((int32_t)(x) * 122 / 1000)

#define FS2G_TO_MG(x) ((float_t)(x) * 61.0f / 1000.0f)

#define FS2000DPS_TO_MDPS(x) ((float_t)(x) * 70.0f)

#define STD_ACC  980
extern int test_draw;
extern uint8_t test_dur;
extern int max_angz;
extern int min_angz;
extern int test_wavearound;
extern uint8_t testwave_acc;
extern uint8_t testrotate_acc;
extern uint8_t testknock_acc;
extern uint8_t knock_peaknum;
extern uint8_t knock_z_peaknum;
extern enum GESTURE_STATUS gest_status;
extern uint8_t daily_spo2_flg;
extern uint8_t test_into_spo2;
extern int testmaxy;
extern int test_angx;
extern int testflg;
extern int gesture_on;

extern void 	steps_init(void);
//extern void     steps_free(void);
extern void     stepsnum_update(acc_axis *pt, E_ACC_PROC statu);
extern int 		gesture_intensity(void);
extern uint32_t 		steps_num(void);
extern void 	intensity_set(int intensity);
extern void		steps_clear(void);

//extern void 	acc_rawdata_send(int *x, int *y, int num);
//extern int		gesture_status_update(enum GESTURE_STATUS status);
//extern void		gesture_status_update_explicit(enum GESTURE_STATUS status, int timeout);
//extern enum     GESTURE_STATUS get_gesture_status(void);
extern void     gesture_init(void);
//extern int      acc_fifo_recover_wavelet(ACC_G_FIFO *fifo, int num, int clc_num);
extern int 		gesture_support(void);


extern void Count_Move_get_new_walk_new(uint16_t* input, int inlen, int* period_step_count);
extern void Count_Move_get_new_walk(uint16_t* input, int inlen, int* period_step_count);
extern void Count_Move_get_new(uint16_t* input, int inlen, int* period_step_count);
extern void smooth_ACC_period_data(uint16_t* period_data, int smooth_span, int* period_data_smooth);
extern void smooth_short_ACC_period_data(short* period_data, int smooth_span, int* period_data_smooth);
extern void mean_get(float* data, int dataLen, float* meanValue);
extern void mean_get_int(int *data,int dataLen,int* meanValue);
extern void cross_mean_cnt_get(int* data, int dataLen, int meanValue, int* cross_mean_cnt);
extern float var_get(float* data, int dataLen);
extern int var_get_int(int* data, int dataLen);

extern void max_get_int(int* data,int dataLen,int* maxValue,int* maxValuePos);

extern void max_get(float* data,int dataLen,float* maxValue,int* maxValuePos);

extern void step_no_sport_delete(int* period_step);
extern void step_no_sport_delete_new(int *period_step);
extern int64_t sqrt_bv(int64_t n);

#ifdef __cplusplus  
}  
#endif  
#endif /* _MEING_GESTURE_H */

