/*
* Copyright (c) 2019 MegaHealth. All Rights Reserved.
* @file ring_tmp.h
* @author:li tianzheng
* @modifing:
* @date 2020.10.15
* @version
* @brief
*/

#ifndef _RING_TMP_H  
#define _RING_TMP_H  

#ifdef __cplusplus  
extern "C"
{
#endif


#include <stdbool.h>
#include <stdint.h>
#include "app_error.h"
#include "TMP117\TMP117_defs.h"
#include "TMP117\TMP117.h"
#include "mTask.h"

#define RING_TMP_MARKER "%s%d.%02d"
 
/**
 * @brief Macro for dissecting a float number into two numbers (integer and residuum).
 */
#define TMP_SHOW(val) (uint32_t)(((val) < 0 && (val) > -1.0) ? "-" : ""),   \
                           (int32_t)(val),                                       \
                           (int32_t)((((val) > 0) ? (val) - (int32_t)(val)       \
                                                : (int32_t)(val) - (val))*100)

enum tmp_downsample_rate_e
{
/*  
*   In our project, the measurement period of temperature is setting to 1Hz
*	TMP_SAMPLE_TIME_MS  =  1s  
*   1s = 1000ms  
*   Thus, max(TMP_FREQUENCY) is about 1Hz.	
*/   
	TMP_DOWNSAMPLE_NONE = 0,
	TMP_DOWNSAMPLE_1_60_HZ = 60,
	TMP_DOWNSAMPLE_1_30_HZ = 30,
	TMP_DOWNSAMPLE_1_10_HZ = 10,
	TMP_DOWNSAMPLE_1_5_HZ = 5,
	TMP_DOWNSAMPLE_1_2_HZ = 2,
	TMP_DOWNSAMPLE_1HZ = 1,
	TMP_DOWNSAMPLE_INVALID = 61
};


typedef enum
{
	RING_TMP_NONE_MODE = 0x00,
	RING_TMP_SHUTDOWN_MODE,
	RING_TMP_ONE_SHOT_MODE,
	RING_TMP_MAX_MODE
} ring_tmp_mode_t;

typedef enum{
	MEASURE_MODE_NONE = 0x00,
	MEASURE_MODE_DAILY,
	MEASURE_MODE_MONITOR,
	MEASURE_MODE_PTT_MONITOR,
}measure_mode_t;

//
struct tmp_dev_info_s
{
#define      TMP_DEV_NONE     0
#define      TMP_DEV_OK       1
	int8_t    i2c_line;

/******     sensor data  by downsampling per second	   ************/
#define 	TMP_SAMPLE_NONEDATA		0
#define 	TMP_SAMPLE_NEWDATA		1
#define 	TMP_SAMPLE_USEDDATA		2
	uint8_t smp_data_used;
	struct tmp117_sensor_data  sample_data;
	uint32_t app_sec;		/* uint32_t app_get_sec(void) */
	
/**************      time  domain          ************/
	int32_t    smp_period;
	int32_t    smp_times;
};

struct ring_tmp_s
{
#define  		TMP117_I2C_ADDR_GRP			{(0x48), (0x49)}	
#define     RING_OUTSIDE_TMP117				(0)
#define     RING_INSIDE_TMP117				(1)
#define  		DEVICE_NUM_MAX 						(2)

/************         control domain          ************/
  ring_tmp_mode_t curMode[DEVICE_NUM_MAX];
  bool m_pwr[DEVICE_NUM_MAX];
	bool timer_running;
//	uint16_t read_cmd[DEVICE_NUM_MAX];
#define 	TMP117_MEASURE_NONE 													0x00
#define 	TMP117_MEASURING_REPEAT_START 								0x01
#define 	TMP117_MEASURING_REPEAT_GETTING_RESULTS 			0x02
	uint8_t current_status;
	uint32_t interval_ms;
/*************        device  domain         *************/
	uint8_t    addr[DEVICE_NUM_MAX]; 
	struct  tmp117_dev 		       sensor[DEVICE_NUM_MAX];

/**************      data domain            ************/
	struct tmp117_sensor_data    data[DEVICE_NUM_MAX];

/*************       factor test domain      **************/
	struct tmp_dev_info_s        dev_info[DEVICE_NUM_MAX];

};

struct tmp_signal_s
{
	uint16_t start;
	uint16_t read;
	uint16_t stop;
};

extern struct tmp_signal_s tmp0_signal;
extern struct tmp_signal_s tmp1_signal;

/**************************************/
ring_tmp_mode_t tmp_get_work_mode(int8_t dev_id);
bool tmp_start(ring_tmp_mode_t mode, int8_t dev_id);
bool tmp_stop(int8_t dev_id);

/*
*  usage: 
*
*  int main()
*  {
*     	struct tmp117_sensor_data *test_pt;
*	      read_tmp_value(&test_pt, x, bool, RING_TMP_ONE_SHOT_MODE);
*				test_pt->temp_val;
*				test_pt->celcius;
*  }
*/
int8_t read_tmp_value(struct tmp117_sensor_data **this_data,  int8_t dev_id, bool local_convert, ring_tmp_mode_t curMode);
struct tmp_dev_info_s  show_tmp_dev_info( int8_t dev_id);
bool tmp_is_new_data(int8_t dev_id);
void set_tmp_downsample_rate(enum tmp_downsample_rate_e downsample_rate, int8_t dev_id);

uint8_t gCurrent_body_temperature();
bool tmp117_measure_init(bool create_timer, bool start_timer, measure_mode_t mode);
bool tmp117_measure_deinit(void);

#ifdef __cplusplus  
}
#endif  
#endif /* _RING_TMP_H */
